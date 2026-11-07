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
 * The ADC watches the words written to the tuner port and keeps:
 *
 *    9:0   the gain fields of the last register 1 word (BBGAIN, MIXBU, MIXL, LNAGR)
 *    11    MMIO register 0x10 bit 21, which does nothing else, so a free marker
 *    12    MMIO register 7 bit 2, the output saturating instead of wrapping
 *    13    toggles on every tuner register 1 (gain) word
 *    14    toggles on every tuner register 2 (synthesizer) word
 *    15    toggles on every write to MMIO register 3, the sample rate PLL
 *
 * Bit 10 has never been seen set. The header is fixed when the packet starts, so a
 * change shows in the packet after the one it happened in.
 */

#define MIRISDR_HDR_MARK        (1 << 11)
#define MIRISDR_HDR_SATURATE    (1 << 12)
#define MIRISDR_HDR_GAIN        (1 << 13)
#define MIRISDR_HDR_TUNE        (1 << 14)
#define MIRISDR_HDR_RATE        (1 << 15)

/* MIRISDR_MARK_REG, MIRISDR_MARK_BIT: constants.h */

static void mirisdr_scan_latch (mirisdr_dev_t *p, uint16_t w, uint16_t x, uint64_t sample, uint64_t index,
                                uint32_t step);

/*
 * Change reports. Each marker flip written while a change callback is set is queued with
 * the level it leaves; the stream's flips are matched to the queue in order. If they cannot
 * be matched (lost packets, wrong level, a flip not seen in time, a failed write), the
 * outstanding changes are reported UNCERTAIN: dirty from where they could have started,
 * clean once the marker has been quiet long enough for every flip to have shown.
 */
#define MIRISDR_CHG_LATE_MS     500
#define MIRISDR_CHG_QUIET_MS    250

static mirisdr_chg_t *mirisdr_chg_nth (mirisdr_dev_t *p, uint32_t k)
{
    return &p->chg.ring[(p->chg.head + k) % MIRISDR_CHG_MAX];
}

/* flips can no longer be matched: the changes may have started at from */
static void mirisdr_chg_unsure (mirisdr_dev_t *p, uint64_t from)
{
#if MIRISDR_DEBUG >= 1
    if (!p->chg.unsure) fprintf(stderr, "libmirisdr: change reports unsure from %llu\n", (unsigned long long) from);
#endif
    if (!p->chg.unsure || (from < p->chg.unsure_from)) p->chg.unsure_from = from;
    p->chg.unsure = 1;
}

/* queue a marker flip being written; returns its slot, -1 if not queued */
static int mirisdr_chg_push (mirisdr_dev_t *p, int level)
{
    mirisdr_chg_t *c;
    int slot;

    p->chg.last_ns = mirisdr_now_ns();
    if (!p->chg.cb) return -1;

    if (p->chg.n == MIRISDR_CHG_MAX) {
        /* ring full */
        mirisdr_chg_unsure(p, p->chg.pos);
        return -1;
    }

    slot = (int) ((p->chg.head + p->chg.n++) % MIRISDR_CHG_MAX);
    c = &p->chg.ring[slot];
    memset(c, 0, sizeof *c);
    c->level = (uint8_t) level;
    c->at_ns = p->chg.last_ns;
    c->pos = p->chg.pos;
    c->tuner = p->tuner_cur;
    if (p->chg.req) {
        c->report = 1;
        c->tag = p->chg.tag;
        p->chg.req = 0;
        p->chg.slot = slot;
    } else {
        c->known = 1;       /* not a tune's flip */
    }

    return slot;
}

/* the write of the flip in slot may have failed */
static void mirisdr_chg_fail (mirisdr_dev_t *p, int slot)
{
    mirisdr_ev_lock(p);
    mirisdr_chg_unsure(p, (slot >= 0) ? p->chg.ring[slot].pos : p->chg.pos);
    mirisdr_ev_unlock(p);
}

/* the running tune returned: store its result and settling */
static int mirisdr_chg_done (mirisdr_dev_t *p, int ok, const mirisdr_tune_result_t *res, uint32_t settle_us)
{
    int slot, reported = 0;

    mirisdr_ev_lock(p);
    slot = p->chg.slot;
    p->chg.slot = -1;
    p->chg.req = 0;
    if ((slot >= 0) && p->chg.n) {
        mirisdr_chg_t *c = &p->chg.ring[slot];

        c->known = 1;
        c->failed = !ok || !res;
        if (!c->failed) {
            c->res = *res;
            c->settle_us = settle_us;
        } else {
            /* unknown what was sent */
            mirisdr_chg_unsure(p, c->pos);
        }
        reported = 1;
    }
    mirisdr_ev_unlock(p);

    return reported;
}

/* stream restart (index from 0): unfinished changes are reported from index 0 */
static void mirisdr_chg_restart (mirisdr_dev_t *p)
{
    uint32_t k, n = 0;
    mirisdr_chg_t keep[MIRISDR_CHG_MAX];

    mirisdr_ev_lock(p);
    for (k = 0; k < p->chg.n; k++) {
        mirisdr_chg_t *c = mirisdr_chg_nth(p, k);

        if (!c->report) continue;
        keep[n] = *c;
        keep[n].seen = 1;
        keep[n].dirty = keep[n].from = keep[n].pos = 0;
        keep[n].dirty_sent = 0;
        keep[n].flags |= MIRISDR_CHANGE_RESTARTED;
        n++;
    }
    /* keep chg_slot pointing at the running tune's entry */
    if (p->chg.slot >= 0) {
        uint32_t j = 0;

        for (k = 0; k < p->chg.n; k++) {
            mirisdr_chg_t *c = mirisdr_chg_nth(p, k);

            if (!c->report) continue;
            if (c == &p->chg.ring[p->chg.slot]) break;
            j++;
        }
        p->chg.slot = (k < p->chg.n) ? (int) j : -1;
    }
    for (k = 0; k < n; k++) p->chg.ring[k] = keep[k];
    p->chg.head = 0;
    p->chg.n = n;
    p->chg.pos = 0;
    p->chg.unsure = 0;
    mirisdr_ev_unlock(p);
}

static void mirisdr_chg_emit (mirisdr_dev_t *p, const mirisdr_chg_t *c, uint32_t kind, uint64_t index, uint32_t flags)
{
    mirisdr_change_t e;

    memset(&e, 0, sizeof e);
    e.size = sizeof e;
    e.kind = kind;
    e.flags = c->flags | flags;
    e.tag = c->tag;
    e.tuner = c->tuner;
    e.index = index;
    if (kind == MIRISDR_CHANGE_CLEAN) {
        e.settle_us = c->settle_us;
        if (c->failed) e.flags |= MIRISDR_CHANGE_FAILED;
        else e.result = &c->res;
    }
    if (p->chg.cb) p->chg.cb(&e, p->chg.ctx);
}

/* microseconds to stream samples, rounded up */
static uint64_t mirisdr_chg_samples (mirisdr_dev_t *p, uint32_t us)
{
    return ((uint64_t) us * p->rate + 999999) / 1000000;
}

static void mirisdr_chg_pop (mirisdr_dev_t *p)
{
    p->chg.head = (p->chg.head + 1) % MIRISDR_CHG_MAX;
    p->chg.n--;
}

/* per packet, before its samples go out: match marker flips and emit the reports */
static void mirisdr_chg_latch (mirisdr_dev_t *p, int toggled, int level, uint64_t index, uint32_t step, int gap)
{
    uint64_t before = p->chg.pos, now;
    uint32_t k;

    p->chg.pos = index + step;
    if (!p->chg.n && !p->chg.unsure) return;

    now = mirisdr_now_ns();

    if (!p->chg.unsure) {
        mirisdr_chg_t *c = NULL;

        for (k = 0; k < p->chg.n; k++) {
            if (!mirisdr_chg_nth(p, k)->seen) {
                c = mirisdr_chg_nth(p, k);
                break;
            }
        }

        if (c && gap) {
            /* packets lost: a flip may have been in them */
            mirisdr_chg_unsure(p, (before >= step) ? before - step : 0);
        } else if (c && toggled && (c->level != level)) {
            mirisdr_chg_unsure(p, ((index >= step) && (index - step < c->pos)) ? index - step : c->pos);
        } else if (c && toggled) {
            c->seen = 1;
            c->dirty = (index >= step) ? index - step : 0;
            c->from = index;
        } else if (c && (now - c->at_ns > (uint64_t) MIRISDR_CHG_LATE_MS * 1000000)) {
            mirisdr_chg_unsure(p, c->pos);
        }
        /* (a flip nobody queued is ignored) */
    }

    if (p->chg.unsure) {
        uint64_t to = p->chg.pos;
        int all = 1;

        /* DIRTY right away, from where they could have started */
        for (k = 0; k < p->chg.n; k++) {
            mirisdr_chg_t *c = mirisdr_chg_nth(p, k);

            if (c->report && (!c->dirty_sent || (c->dirty > p->chg.unsure_from))) {
                c->dirty = p->chg.unsure_from;
                c->dirty_sent = 1;
                mirisdr_chg_emit(p, c, MIRISDR_CHANGE_DIRTY, c->dirty, MIRISDR_CHANGE_UNCERTAIN);
            }
            if (!c->known) all = 0;
        }

        /* CLEAN once quiet and all tunes returned: every flip has shown by then */
        if (!all || (now - p->chg.last_ns < (uint64_t) MIRISDR_CHG_QUIET_MS * 1000000)) return;

        while (p->chg.n) {
            mirisdr_chg_t *c = mirisdr_chg_nth(p, 0);

            if (c->report)
                mirisdr_chg_emit(p, c, MIRISDR_CHANGE_CLEAN, to + mirisdr_chg_samples(p, c->settle_us),
                                 MIRISDR_CHANGE_UNCERTAIN);
            mirisdr_chg_pop(p);
        }
        p->chg.unsure = 0;
        /* resync the level from the stream */
        p->mark_level = level;
        return;
    }

    /* DIRTY when seen; CLEAN when seen and its tune returned */
    for (k = 0; k < p->chg.n; k++) {
        mirisdr_chg_t *c = mirisdr_chg_nth(p, k);

        if (!c->seen) break;
        if (c->report && !c->dirty_sent) {
            c->dirty_sent = 1;
            mirisdr_chg_emit(p, c, MIRISDR_CHANGE_DIRTY, c->dirty, 0);
        }
    }
    while (p->chg.n) {
        mirisdr_chg_t *c = mirisdr_chg_nth(p, 0);

        if (!c->seen || !c->known) break;
        if (c->report) mirisdr_chg_emit(p, c, MIRISDR_CHANGE_CLEAN, c->from + mirisdr_chg_samples(p, c->settle_us), 0);
        mirisdr_chg_pop(p);
    }
}

/* once per packet, before its samples are counted */
static void mirisdr_events_latch (mirisdr_dev_t *p, const uint8_t *hdr, uint64_t sample, uint64_t index,
                                  uint32_t step) {
    mirisdr_stream_event_t e;
    uint16_t w = hdr[4] | hdr[5] << 8, x;

    if (!p->ev_cb && !p->scan.on && !p->chg.cb) return;

    if (!p->ev_valid) {
        p->ev_valid = 1;
        p->ev_missed = 0;
        /* first header is the baseline, except the marker, whose level is known (so a flip
           in this packet shows). Scans keep the header as baseline */
        if (p->scan.on) {
            p->ev_last = w;
            return;
        }
        p->ev_last = (uint16_t) ((w & ~MIRISDR_HDR_MARK) | (p->ev_mark_base ? MIRISDR_HDR_MARK : 0));
    }

    x = w ^ p->ev_last;
    p->ev_last = w;

    /* change reports (not during scans) */
    if (p->chg.cb && !p->scan.on) {
        mirisdr_ev_lock(p);
        if (p->chg.cb)
            mirisdr_chg_latch(p, (x & MIRISDR_HDR_MARK) != 0, (w & MIRISDR_HDR_MARK) != 0, index, step, p->ev_missed);
        mirisdr_ev_unlock(p);
    }

    /* under the lock mirisdr_scan_stop() takes, so a stopped scan is not read after */
    if (p->scan.on) {
        mirisdr_ev_lock(p);
        mirisdr_scan_latch(p, w, x, sample, index, step);
        mirisdr_ev_unlock(p);
    }
    if (!p->ev_cb) {
        p->ev_missed = 0;
        return;
    }

    memset(&e, 0, sizeof e);
    if (x & MIRISDR_HDR_GAIN) e.events |= MIRISDR_EVENT_GAIN;
    if (x & MIRISDR_HDR_TUNE) e.events |= MIRISDR_EVENT_TUNE;
    if (x & MIRISDR_HDR_RATE) e.events |= MIRISDR_EVENT_RATE;
    if (x & MIRISDR_HDR_MARK) e.events |= MIRISDR_EVENT_MARK;
    if (p->ev_missed) e.events |= MIRISDR_EVENT_MISSED;
    p->ev_missed = 0;

    if (!e.events) return;

    e.sample   = sample;
    e.index    = index;
    e.bb_gr    = w & 0x3f;
    e.mixbu    = (w >> 6) & 3;
    e.mixl     = (w >> 8) & 1;
    e.lna      = (w >> 9) & 1;
    e.mark     = !!(w & MIRISDR_HDR_MARK);
    e.saturate = !!(w & MIRISDR_HDR_SATURATE);
    e.raw      = w;

    /* the pair as one, and once the setter returns the old one no longer runs */
    mirisdr_ev_lock(p);
    if (p->ev_cb) p->ev_cb(&e, p->ev_ctx);
    mirisdr_ev_unlock(p);
}

int mirisdr_set_stream_events (mirisdr_dev_t *p, mirisdr_stream_event_cb_t cb, void *ctx) {
    if (!p) goto failed;

    mirisdr_ev_lock(p);
    p->ev_cb = cb;
    p->ev_ctx = ctx;
    p->ev_valid = 0;
    p->ev_mark_base = p->mark_level;
    mirisdr_ev_unlock(p);

    return 0;

failed:
    return -1;
}

int mirisdr_set_change_callback (mirisdr_dev_t *p, mirisdr_change_cb_t cb, void *ctx) {
    if (!p) goto failed;

    /* drop the old callback's queue */
    mirisdr_ev_lock(p);
    p->chg.cb = cb;
    p->chg.ctx = ctx;
    p->chg.head = p->chg.n = 0;
    p->chg.slot = -1;
    p->chg.unsure = 0;
    mirisdr_ev_unlock(p);

    return 0;

failed:
    return -1;
}

int mirisdr_set_stream_mark (mirisdr_dev_t *p, int on) {
    if (!p) goto failed;

    return mirisdr_mark_write(p, on ? 1 : 0);

failed:
    return -1;
}
