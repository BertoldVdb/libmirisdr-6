/*
 * Copyright (C) 2026 by Bertold Van den Bergh <vandenbergh@bertold.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Scan lists. A copy of the device runs the normal tune path with its register
 * writes kept instead of sent, so a hop is exactly what a tune would send.
 */

/* a bank, less the wait a chunk leads with when it starts the list */
#define MIRISDR_SCAN_CHUNK      (MIRISDR_LIST_MAX - 1)

/* the tuner's searches settle within a few ms: read until two reads agree */
#define MIRISDR_LEARN_US        2000
#define MIRISDR_LEARN_READS     20

struct mirisdr_scan
{
    mirisdr_list_entry_t *e;            /* every chunk, back to back */
    uint32_t            *chunk_at;      /* first entry of each chunk, then the end */
    uint32_t            *chunk_hop;     /* first hop of each chunk, then n */
    uint32_t            chunks;
    mirisdr_scan_hop_t  *hops;
    uint32_t            *tag;           /* each hop's change_tag */
    uint32_t            n;
    uint16_t            dwell;
};

int mirisdr_tune_learn (mirisdr_dev_t *p, int tuner, mirisdr_tune_config_t *cfg)
{
    mirisdr_tune_config_t c;
    mirisdr_tuner_status_t a, b;
    mirisdr_tuner_override_t *ov;
    const mirisdr_tuner_ctx_t *t;
    int i;

    if (!p || !cfg || !p->dh || p->external_tuner || (tuner < 0) || (tuner >= mirisdr_tuners_of(p))) return -1;

    c = *cfg;
    memset(&c.override, 0, sizeof c.override);
    if (mirisdr_tune(p, tuner, &c, NULL) < 0) return -1;

    memset(&a, 0, sizeof a);
    for (i = 0; i < MIRISDR_LEARN_READS; i++)
    {
        usleep(MIRISDR_LEARN_US);
        if (mirisdr_get_tuner_status(p, tuner, &b) < 0) return -1;
        if (i && (b.raw == a.raw)) break;
        a = b;
    }

    if ((i == MIRISDR_LEARN_READS) || (b.coarse < 0)) return -1;

    /* this tuner's state, which the tune left in its context: the device's fields are the
       primary tuner's again */
    t = &p->tctx[tuner];
    ov = &cfg->override;
    memset(ov, 0, sizeof *ov);
    ov->hold_vco = 1;     ov->coarse = (uint8_t) b.coarse; ov->fine = b.fine;
    ov->hold_unknown = 1; ov->unknown = b.unknown;
    ov->hold_upconv = 1;  ov->upconv = b.upconv;
    ov->hold_lna = 1;     ov->lna_cal = b.lna_cal;

    /* the status reads the code calibrated, not the one in use: the widest bandwidth
       holds 0 and the gap rows hold the one from the real crystal */
    ov->hold_filter = 1;
    if (t->bandwidth == MIRISDR_BW_MAX) ov->filter = 0;
    else if (t->tuner_gap && (t->filter_cal >= 0)) ov->filter = (uint8_t) t->filter_cal;
    else ov->filter = b.filter;

    return 0;
}

void mirisdr_scan_free (mirisdr_scan_t *s)
{
    if (!s) return;

    free(s->e);
    free(s->chunk_at);
    free(s->chunk_hop);
    free(s->hops);
    free(s->tag);
    free(s);
}

uint32_t mirisdr_scan_chunks (const mirisdr_scan_t *s)
{
    return s ? s->chunks : 0;
}

int mirisdr_scan_chunk (const mirisdr_scan_t *s, uint32_t chunk, const mirisdr_list_entry_t **e)
{
    if (!s || (chunk >= s->chunks)) return -1;

    if (e) *e = s->e + s->chunk_at[chunk];

    return (int) (s->chunk_at[chunk + 1] - s->chunk_at[chunk]);
}

int mirisdr_scan_hop (const mirisdr_scan_t *s, uint32_t hop, mirisdr_scan_hop_t *info)
{
    if (!s || !info || (hop >= s->n)) return -1;

    *info = s->hops[hop];

    return 0;
}

/* Compile one hop on the shadow; its entries are rec[from, rec_n) */

static int mirisdr_scan_hop_words (mirisdr_dev_t *sh, mirisdr_dev_t *p, const mirisdr_tune_config_t *c,
                                   uint32_t k, uint16_t dwell, mirisdr_scan_hop_t *h)
{
    mirisdr_tune_plan_t pl;
    mirisdr_stream_plan_t spl;
    uint32_t from = sh->scan.rec_n, i;
    int r2 = 0;

    if (mirisdr_tune_plan(sh, c, 0, &pl) < 0) goto refused;

    if (pl.ds)
    {
        fprintf(stderr, "hop %u: direct sampling is not for scans\n", k);
        return -1;
    }

    if ((pl.w.band == MIRISDR_BAND_L) && !c->override.hold_lna)
    {
        fprintf(stderr, "hop %u: an L band hop needs its LNA code held, see mirisdr_tune_learn()\n", k);
        return -1;
    }

    /* a list cannot restart the stream */
    if (p->stream.follow_tune || p->stream.baseband)
    {
        if (mirisdr_stream_plan(p, &p->stream, 0, pl.siq, pl.sif, pl.ds_edge, &spl) < 0) goto refused;
        if ((spl.format != (int) p->format) || (spl.swap != p->swap_iq) || (spl.rate != p->rate) ||
            (spl.bb_path != p->bb_path) || (spl.bb_stages != p->bb_stages) || (spl.bb_rate != p->bb_rate) ||
            (spl.bb_if != p->bb_if) || (spl.bb_if2 != p->bb_if2) || (spl.bb_ds != p->bb_ds))
        {
            fprintf(stderr, "hop %u: the stream following it would change format\n", k);
            return -1;
        }
    }

    /* everything at the start of a pass, so the list wraps; register 2 always, so
       every hop toggles the tune event */
    if (!k)
    {
        sh->tuner_valid = 0;
        sh->reg8_valid = 0;
        sh->exp_valid = 3;      /* set up when the device opened: only their two bytes */
        sh->exp_stale = 3;
    }
    else sh->tuner_valid &= (uint16_t) ~(1u << 2);

    if (mirisdr_tune_apply(sh, c, 0, &h->res) < 0) goto refused;

    /* the only wait a hop takes is the front end's settling after its word changed: a
       calibration's would hold the list far longer */
    for (i = from; i < sh->scan.rec_n; i++)
    {
        if ((sh->scan.rec[i].reg == MIRISDR_LIST_WAIT_US) && (sh->scan.rec[i].val != MIRISDR_EXP_SETTLE_US))
        {
            fprintf(stderr, "hop %u: would calibrate in the list\n", k);
            return -1;
        }

        if ((sh->scan.rec[i].reg != 0x09) || ((sh->scan.rec[i].val & 15) != 2)) continue;

        if (r2++) continue;
        h->pre_entries = (uint16_t) (i - from);
    }

    /* the entries before register 2 and its own: the lead from the first, as the list runs */
    for (i = from, h->pre_words = 0, h->pre_us = 0, h->lead_us = 0; i <= from + h->pre_entries; i++)
    {
        if (sh->scan.rec[i].reg == 0x09) h->pre_words += i < from + h->pre_entries;
        if ((sh->scan.rec[i].reg == MIRISDR_LIST_WAIT_US) && (i < from + h->pre_entries))
            h->pre_us = (uint16_t) (h->pre_us + sh->scan.rec[i].val);
        h->lead_us = (uint16_t) (h->lead_us + mirisdr_list_entry_us(sh->scan.rec[i].reg, sh->scan.rec[i].val));
    }

    /* settling from register 2, but the synthesizer's (set once the hop before is known):
       a gain word after it, front-end words before it less the time since */
    h->settle_us = 0;
    for (i = from; i < sh->scan.rec_n; i++)
    {
        int before = i < from + h->pre_entries;

        if ((sh->scan.rec[i].reg == 0x09) && ((sh->scan.rec[i].val & 15) == 1) && (h->settle_us < MIRISDR_SETTLE_GAIN_US))
            h->settle_us = MIRISDR_SETTLE_GAIN_US;
        if (before && (sh->scan.rec[i].reg != 0x09) && (sh->scan.rec[i].reg != MIRISDR_LIST_WAIT_US) &&
            (MIRISDR_SETTLE_FRONTEND_US > h->pre_us + h->settle_us))
            h->settle_us = MIRISDR_SETTLE_FRONTEND_US - h->pre_us;
    }

    if (r2 != 1)
    {
        fprintf(stderr, "hop %u: %d synthesizer words\n", k, r2);
        return -1;
    }

    if (mirisdr_write_reg(sh, MIRISDR_LIST_WAIT_IRQ, dwell) < 0) return -1;

    h->entries = (uint16_t) (sh->scan.rec_n - from);
    if (h->entries > MIRISDR_SCAN_CHUNK - 1)
    {
        fprintf(stderr, "hop %u: %u entries do not fit a chunk\n", k, h->entries);
        return -1;
    }

    return 0;

refused:
    fprintf(stderr, "hop %u refused\n", k);
    return -1;
}

int mirisdr_scan_compile (mirisdr_dev_t *p, int tuner, const mirisdr_tune_config_t *hops, uint32_t n,
                          uint16_t dwell, mirisdr_scan_t **out)
{
    mirisdr_dev_t *sh = NULL;
    mirisdr_scan_t *s = NULL;
    uint32_t k, c, at, size;

    if (!p || !hops || !n || !dwell || !out || (tuner < 0) || (tuner >= mirisdr_tuners_of(p))) return -1;
    if (!p->fw_ours || p->external_tuner) return -1;

    /* scans run on tuner 0 alone for now */
    if (tuner || (mirisdr_stream_tuners(p) != 1))
    {
        fprintf(stderr, "scans run on tuner 0 streamed alone\n");
        return -1;
    }

    if (!(s = calloc(1, sizeof *s))) return -1;
    s->n = n;
    s->dwell = dwell;
    if (!(s->hops = calloc(n, sizeof *s->hops))) goto failed;
    if (!(s->tag = calloc(n, sizeof *s->tag))) goto failed;

    /* the device as it is, writing nowhere */
    if (!(sh = malloc(sizeof *sh))) goto failed;
    memcpy(sh, p, sizeof *sh);
    sh->dh = NULL;
    sh->fake = 1;
    sh->fw_list_at = 1;             /* SPI writes wait in the list */
    sh->stream.follow_tune = 0;
    sh->bb = NULL;              /* the device's filters are not the shadow's */
    sh->bb2 = NULL;
    sh->bb_split = NULL;
    sh->bb_split_cap = 0;
    sh->batch_depth = 0;
    sh->batch_n = 0;
    sh->batch_running = 0;
    sh->scan.rec_n = 0;
    sh->scan.rec_max = 256;
    if (!(sh->scan.rec = malloc(sh->scan.rec_max * sizeof *sh->scan.rec))) goto failed;

    for (k = 0; k < n; k++)
        if (mirisdr_scan_hop_words(sh, p, &hops[k], k, dwell, &s->hops[k]) < 0) goto failed;

    /* the synthesizer's settling: a jump from the hop before (the first follows the last),
       held codes, or a search */
    for (k = 0; k < n; k++)
    {
        mirisdr_scan_hop_t *h = &s->hops[k];
        uint32_t lo = s->hops[k ? k - 1 : n - 1].res.lo, synth;

        synth = (llabs((long long) h->res.lo - (long long) lo) > MIRISDR_SETTLE_JUMP_HZ) ? MIRISDR_SETTLE_JUMP_US :
                hops[k].override.hold_vco ? MIRISDR_SETTLE_HELD_US : MIRISDR_SETTLE_SYNTH_US;
        if (synth > h->settle_us) h->settle_us = synth;
        s->tag[k] = hops[k].change_tag;
    }

    /* chunks on hop boundaries, each led by the marker */
    for (k = 0, c = 0, size = MIRISDR_SCAN_CHUNK; k < n; k++)
    {
        if (size + s->hops[k].entries > MIRISDR_SCAN_CHUNK)
        {
            c++;
            size = 1;
        }
        size += s->hops[k].entries;
        s->hops[k].chunk = c - 1;
    }

    s->chunks = c;
    s->chunk_at = calloc(c + 1, sizeof *s->chunk_at);
    s->chunk_hop = calloc(c + 1, sizeof *s->chunk_hop);
    s->e = calloc(sh->scan.rec_n + c, sizeof *s->e);
    if (!s->chunk_at || !s->chunk_hop || !s->e) goto failed;

    for (k = 0, at = 0, size = 0; k < n; k++)
    {
        const mirisdr_scan_hop_t *h = &s->hops[k];

        if (!k || (h->chunk != s->hops[k - 1].chunk))
        {
            s->chunk_at[h->chunk] = at;
            s->chunk_hop[h->chunk] = k;
            s->e[at].reg = MIRISDR_MARK_REG;
            s->e[at++].val = 0;     /* the level is set as it loads */
        }

        memcpy(s->e + at, sh->scan.rec + size, h->entries * sizeof *s->e);
        at += h->entries;
        size += h->entries;
    }

    s->chunk_at[c] = at;
    s->chunk_hop[c] = n;

    mirisdr_bb_free(sh);
    free(sh->scan.rec);
    free(sh);
    *out = s;

    return 0;

failed:
    if (sh) {
        mirisdr_bb_free(sh);
        free(sh->scan.rec);
    }
    free(sh);
    mirisdr_scan_free(s);

    return -1;
}

/* ------------------------------------------------------------------ */
/* running it                                                          */
/* ------------------------------------------------------------------ */

/* How long a chunk that starts the list waits first: its hop then starts on an
   interrupt as the others do, and after the next chunk's load (without it the first
   hop came ~0.4 ms late) */
#define MIRISDR_SCAN_LEAD_US    1000

static uint32_t mirisdr_scan_lead (mirisdr_dev_t *p)
{
    uint64_t irq_ns = p->rate ? (uint64_t) p->scan.burst * p->addr_step * 1000000000ULL / p->rate : 0;
    uint32_t n = irq_ns ? (uint32_t) ((MIRISDR_SCAN_LEAD_US * 1000ULL + irq_ns - 1) / irq_ns) : 1;

    return n ? n : 1;
}

/* Chunk g of the run into its bank, led by a wait when it starts the list */
static int mirisdr_scan_load (mirisdr_dev_t *p, uint64_t g, int queue)
{
    const mirisdr_scan_t *s = p->scan.run;
    mirisdr_list_entry_t buf[MIRISDR_LIST_MAX];
    const mirisdr_list_entry_t *e = NULL;
    int n = mirisdr_scan_chunk(s, (uint32_t) (g % s->chunks), &e), at = 0;

    if (n < 0) return -1;

    if (!queue)
    {
        buf[at].reg = MIRISDR_LIST_WAIT_IRQ;
        buf[at++].val = mirisdr_scan_lead(p);
    }

    memcpy(buf + at, e, n * sizeof *e);

    /* the marker flips on every chunk: it was cleared before the first */
    buf[at].val = (g & 1) ? 0 : MIRISDR_MARK_BIT;

    if (mirisdr_load_list(p, (int) (g & 1), queue ? MIRISDR_LIST_QUEUE : 0, buf, at + n) < 0) return -1;

    p->scan.loaded = g + 1;

    return 0;
}

static int mirisdr_scan_more (mirisdr_dev_t *p)
{
    return !p->scan.total || (p->scan.loaded < p->scan.total);
}

int mirisdr_scan_start (mirisdr_dev_t *p, const mirisdr_scan_t *s, uint32_t passes,
                        mirisdr_scan_cb_t cb, void *ctx)
{
    if (!p || !s || !p->dh || !p->fw_ours || !p->fw_list_at || p->scan.run) return -1;

    if (mirisdr_mark_write(p, 0) < 0) return -1;

    p->scan.total = (uint64_t) passes * s->chunks;
    p->scan.loaded = 0;
    p->scan.restarts = 0;
    p->scan.restarts_seen = 0;
    p->scan.burst = mirisdr_burst(p);
    p->scan.g = UINT64_MAX;
    p->scan.unsure = 0;
    p->scan.pos = 0;
    p->scan.in = 0;
    p->scan.flags = 0;
    p->scan.cb = cb;
    p->scan.ctx = ctx;
    p->scan.run = s;
    p->scan.on = 1;

    if (mirisdr_scan_load(p, 0, 0) < 0) goto failed;
    if (mirisdr_scan_more(p) && (mirisdr_scan_load(p, 1, 1) < 0)) goto failed;

    return 0;

failed:
    mirisdr_scan_stop(p);
    return -1;
}

int mirisdr_scan_feed (mirisdr_dev_t *p)
{
    mirisdr_list_status_t st;

    if (!p || !p->scan.run) return -1;

    if (mirisdr_get_list_status(p, &st) < 0) return -1;
    if (st.spi_timeout) return -1;

    if (st.running)
    {
        /* the chunk queued last has started: its other bank is free */
        if ((st.bank == ((p->scan.loaded - 1) & 1)) && mirisdr_scan_more(p))
            return (mirisdr_scan_load(p, p->scan.loaded, 1) < 0) ? -1 : 1;

        return 1;
    }

    if (!mirisdr_scan_more(p)) return 0;

    /* ran dry: start it again, one late chunk is a longer hop */
    p->scan.restarts++;
    if (mirisdr_scan_load(p, p->scan.loaded, 0) < 0) return -1;
    if (mirisdr_scan_more(p) && (mirisdr_scan_load(p, p->scan.loaded, 1) < 0)) return -1;

    return 1;
}

int mirisdr_get_scan_status (mirisdr_dev_t *p, mirisdr_scan_status_t *st)
{
    mirisdr_list_status_t ls;

    if (!p || !st) return -1;

    memset(st, 0, sizeof *st);
    if (!p->scan.run) return 0;

    st->running = (mirisdr_get_list_status(p, &ls) == 0) && ls.running;
    st->loaded = p->scan.loaded;
    st->pass = p->scan.loaded ? (uint32_t) ((p->scan.loaded - 1) / p->scan.run->chunks) : 0;
    st->restarts = p->scan.restarts;

    return 0;
}

int mirisdr_scan_stop (mirisdr_dev_t *p)
{
    int r;

    if (!p || !p->scan.run) return -1;

    /* under the events' lock, which the stream side holds while it reads the scan: once
       this returns it is through with it, and the caller may free it */
    mirisdr_ev_lock(p);
    p->scan.on = 0;
    p->scan.run = NULL;
    mirisdr_ev_unlock(p);
    r = mirisdr_load_list(p, 0, 0, NULL, 0);

    /* the marker as it was before the scan cleared it */
    if (mirisdr_mark_write(p, 0) < 0) r = -1;

    /* the list wrote behind the caches' back */
    p->tuner_valid = 0;
    p->reg8_valid = 0;
    p->exp_stale = 3;
    p->lna_cal_mhz = 0;

    if (mirisdr_set_soft(p) < 0) r = -1;

    return r;
}

/* The stream side, once per packet with the header bits that changed */
/* a hop's change report (under ev_lock) */
static void mirisdr_scan_change (mirisdr_dev_t *p, const mirisdr_scan_t *s, uint32_t hop, uint32_t kind,
                                 uint64_t index, uint32_t flags)
{
    mirisdr_change_t c;

    memset(&c, 0, sizeof c);
    c.size = sizeof c;
    c.kind = kind;
    c.flags = flags;
    c.tag = s->tag[hop];
    c.index = index;
    if (kind == MIRISDR_CHANGE_CLEAN) {
        c.settle_us = s->hops[hop].settle_us;
        c.result = &s->hops[hop].res;
    }
    p->chg.cb(&c, p->chg.ctx);
}

static void mirisdr_scan_latch (mirisdr_dev_t *p, uint16_t w, uint16_t x, uint64_t sample, uint64_t index,
                                uint32_t step)
{
    const mirisdr_scan_t *s = p->scan.run;
    mirisdr_scan_report_t r;
    uint32_t c;

    uint64_t before = p->scan.pos;

    if (!s || !p->scan.on) return;

    p->scan.pos = index + step;
    if (p->ev_missed) p->scan.flags |= MIRISDR_SCAN_MISSED;

    /* packets lost: the hops until the next chunk cannot be told apart */
    if (p->ev_missed && p->chg.cb && !p->scan.unsure && (p->scan.g != UINT64_MAX)) {
        uint32_t c0 = (uint32_t) (p->scan.g % s->chunks), next = s->chunk_hop[c0] + p->scan.in;

        if (next >= s->n) next = 0;
        p->scan.unsure = 1;
        mirisdr_scan_change(p, s, next, MIRISDR_CHANGE_DIRTY, (before >= step) ? before - step : 0,
                            MIRISDR_CHANGE_UNCERTAIN);
    }

    /* clearing the marker before the first chunk is not a chunk */
    if ((x & MIRISDR_HDR_MARK) && (p->scan.g == UINT64_MAX) && !(w & MIRISDR_HDR_MARK)) x &= (uint16_t) ~MIRISDR_HDR_MARK;

    if (x & MIRISDR_HDR_MARK)
    {
        c = p->scan.g == UINT64_MAX ? 0 : (uint32_t) (p->scan.g % s->chunks);

        /* tune events went missing in the chunk before */
        if ((p->scan.g != UINT64_MAX) && (p->scan.in != s->chunk_hop[c + 1] - s->chunk_hop[c]))
            p->scan.flags |= MIRISDR_SCAN_LOST;

        p->scan.g++;
        p->scan.in = 0;
        p->scan.unsure = 0;

        if (p->scan.restarts != p->scan.restarts_seen)
        {
            p->scan.restarts_seen = p->scan.restarts;
            p->scan.flags |= MIRISDR_SCAN_RESTART;
        }
    }

    if (!(x & MIRISDR_HDR_TUNE) || (p->scan.g == UINT64_MAX)) return;

    c = (uint32_t) (p->scan.g % s->chunks);
    if (p->scan.in >= s->chunk_hop[c + 1] - s->chunk_hop[c])
    {
        p->scan.flags |= MIRISDR_SCAN_LOST;
        /* more toggles than hops: not reported until the next chunk */
        if (p->chg.cb && !p->scan.unsure) {
            p->scan.unsure = 1;
            mirisdr_scan_change(p, s, s->chunk_hop[c + 1] % s->n, MIRISDR_CHANGE_DIRTY,
                                (index >= step) ? index - step : 0, MIRISDR_CHANGE_UNCERTAIN);
        }
        return;
    }

    memset(&r, 0, sizeof r);
    r.hop = s->chunk_hop[c] + p->scan.in++;
    r.pass = (uint32_t) (p->scan.g / s->chunks);
    r.sample = sample >= step ? sample - step : 0;
    r.index = index >= step ? index - step : 0;
    r.length = s->dwell * p->scan.burst * step;
    r.flags = p->scan.flags;
    p->scan.flags = 0;

    /* the change: from its first entry (its lead before register 2) until settled after it */
    if (p->chg.cb && !p->scan.unsure) {
        const mirisdr_scan_hop_t *h = &s->hops[r.hop];
        uint64_t lead = mirisdr_chg_samples(p, h->lead_us);

        mirisdr_scan_change(p, s, r.hop, MIRISDR_CHANGE_DIRTY, (r.index >= lead) ? r.index - lead : 0, 0);
        mirisdr_scan_change(p, s, r.hop, MIRISDR_CHANGE_CLEAN, index + mirisdr_chg_samples(p, h->settle_us), 0);
    }

    if (p->scan.cb) p->scan.cb(&r, p->scan.ctx);
}
