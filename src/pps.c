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
 * An edge on GPIO_0 or, with mirisdr_set_pps_source, a USB start of frame,
 * is latched as a packet count and a position inside that packet, the firmware's
 * poll loop counting six machine cycles per tick. The anchor ties packet counts
 * to the streams sample counter.
 * Taking the anchor requires remapping the DSP buffer which corrupts some ADC samples,
 * so it is only done once per gap (in practice: once per stream start).
 */

static uint32_t mirisdr_burst (mirisdr_dev_t *p);
static void mirisdr_pps_checks_reset (mirisdr_dev_t *p, double expect);

#define CMD_PPS_ENABLE          0x52

/* The counters and the anchor are blocks the firmware keeps, read with request
   0x42 at the addresses its firmware block gives */

#define PPS_TIME_LEN            24
#define PPS_ANCHOR_LEN          17      /* 4 counters, then valid */

/* 200ns loop duration, accept 1% higher (interruptions can't speed up the loop) and
 * 10% lower (inrerruptions only slow down) */
#define MIRISDR_PPS_TURN_NS     200.0
#define MIRISDR_PPS_SCALE_HI    1.01
#define MIRISDR_PPS_SCALE_LO    0.90

/* Streaming interrupt duration, in loop turns */
#define MIRISDR_PPS_STREAM_TURNS  14.0

/* A microframe, in turns: 125 us at 200 ns. */
#define MIRISDR_PPS_UFRAME_TURNS 625.0

/* How far into an interval a first SOF has to be before its tick is sound */
#define MIRISDR_PPS_TAIL_TURNS     3.0

/* Which of the four capture buffers the anchor interrupt was announcing. */
#define MIRISDR_PPS_ANCHOR_BACK 0

/* Blocks either side of the newest capture buffer the anchor saw that it may have damaged.
   The remap hits whatever the USB controller is sending, which can be anywhere in the
   device's queue of twelve 512 byte packets: up to 6 blocks behind the newest, 4 and 5
   seen. Nothing newer can be, the blocks ahead only keep the window a little longer */
#define MIRISDR_ANCHOR_SPREAD   8

static uint32_t mirisdr_pps_le32 (const uint8_t *b)
{
    return (uint32_t) b[0] | ((uint32_t) b[1] << 8)
         | ((uint32_t) b[2] << 16) | ((uint32_t) b[3] << 24);
}

static uint16_t mirisdr_pps_wvalue (mirisdr_dev_t *p, int run)
{
    uint16_t v = run ? 3 : 0;               /* bit 0 run, bit 1 take an anchor */

    if (p->pps.src_sof) v |= 8;             /* edges from the SOF interrupt */

    return v;
}

int mirisdr_set_pps_source (mirisdr_dev_t *p, int sof)
{
    if (!p) return -1;

    p->pps.src_sof = sof ? 1 : 0;

    return 0;
}

/* The stream's count of one block header, whole: the stream thread updates it meanwhile */
static void mirisdr_pps_count (mirisdr_dev_t *p, uint64_t *index, uint32_t *addr, uint64_t *gaps)
{
    int s, tries;

    for (tries = 0; tries < 1000; tries++)
    {
        s = mirisdr_atomic_add(&p->count_seq, 0);
        if (s & 1) continue;
        *index = p->count_index;
        *addr = p->count_addr;
        *gaps = p->count_gaps;
        if (mirisdr_atomic_add(&p->count_seq, 0) == s) return;
    }
}

/* Taking the anchor cuts the DSP buffer off the USB controller for a moment, and the
   bytes sent meanwhile are garbage: samples, and a header when one falls there. From
   the request until the stream is past the anchor the stream thread holds to its own
   count over the headers, and leaves their events and IR byte alone */
static void mirisdr_anchor_open (mirisdr_dev_t *p)
{
    mirisdr_atomic_add(&p->anc.open, 1);
}

/* The blocks from..to may be damaged, the rest of the window is not */
static void mirisdr_anchor_place (mirisdr_dev_t *p, uint32_t from, uint32_t to)
{
    p->anc.from = from;
    p->anc.to = to;
    mirisdr_atomic_xchg(&p->anc.known, mirisdr_atomic_add(&p->anc.open, 0));
}

/* No anchor to place the window by: it ends at the next block */
static void mirisdr_anchor_none (mirisdr_dev_t *p)
{
    uint64_t index = 0, gaps = 0;
    uint32_t addr = 0;

    mirisdr_pps_count(p, &index, &addr, &gaps);
    mirisdr_anchor_place(p, addr, addr - 1);
}

/* The anchor is taken by the next streaming interrupt, so it is read here while
   the sample counter it carries is still within a wrap of the stream: from then
   on everything is 64 bit and counted from it. */
static int mirisdr_pps_anchor (mirisdr_dev_t *p)
{
    uint8_t b[PPS_TIME_LEN];
    uint32_t c[4], addr = 0;
    uint64_t index = 0, gaps = 0;
    int i, newest = 0, tries;

    for (tries = 0; tries < 100; tries++)
    {
        if (!p->fw_anchor_at || mirisdr_read_mem(p, p->fw_anchor_at, b, PPS_ANCHOR_LEN, MIRISDR_MEM_XDATA) < 0)
        {
            mirisdr_anchor_none(p);
            return -1;
        }

        if (b[16]) break;

        usleep(2000);
    }

    if (!b[16])
    {
        mirisdr_anchor_none(p);
        return -1;
    }

    for (i = 0; i < 4; i++) c[i] = mirisdr_pps_le32(b + i * 4);

    /* consecutive packets in an unknown rotation, so the newest is found rather
       than indexed */
    for (i = 1; i < 4; i++)
        if ((int32_t) (c[i] - c[newest]) > 0) newest = i;

    mirisdr_anchor_place(p, c[newest] - MIRISDR_ANCHOR_SPREAD * p->addr_step,
                         c[newest] + MIRISDR_ANCHOR_SPREAD * p->addr_step);

    mirisdr_pps_count(p, &index, &addr, &gaps);
    p->pps.base = (int64_t) index + (int32_t) (c[newest] - MIRISDR_PPS_ANCHOR_BACK * p->addr_step - addr);

    p->pps.gaps0 = gaps;
    p->pps.anchor_valid = 1;
    p->pps.seen = 0;
    p->pps.k_min = 0;
    p->pps.irq_last = 0;
    p->pps.irq_high = 0;

    /* whatever is latched now was captured against the count this anchor has
       just reset, so it is not placeable until the next pulse */
    if (!p->fw_pps_at || mirisdr_read_mem(p, p->fw_pps_at, b, PPS_TIME_LEN, MIRISDR_MEM_IRAM) < 0) return -1;

    p->pps.edge0 = b[14];
    p->pps.stale = 1;

    return 0;
}

/* A gap unties the packet count from real time and nothing here can put that
   back, so the anchor is taken again. */
static int mirisdr_pps_reanchor (mirisdr_dev_t *p)
{
    mirisdr_anchor_open(p);
    if (libusb_control_transfer(p->dh, 0x40, CMD_PPS_ENABLE,
                                mirisdr_pps_wvalue(p, 1), 0, NULL, 0,
                                CTRL_TIMEOUT) < 0)
    {
        mirisdr_anchor_none(p);
        return -1;
    }

    return mirisdr_pps_anchor(p);
}

int mirisdr_enable_pps (mirisdr_dev_t *p, int run)
{
    /* the null device takes no anchor: its window is placed at the stream's count, for the
       offline tests */
    if (p && p->fake && !p->dh)
    {
        uint64_t index = 0, gaps = 0;
        uint32_t addr = 0;

        if (!run) return 0;
        mirisdr_anchor_open(p);
        mirisdr_pps_count(p, &index, &addr, &gaps);
        mirisdr_anchor_place(p, addr - MIRISDR_ANCHOR_SPREAD * p->addr_step,
                             addr + MIRISDR_ANCHOR_SPREAD * p->addr_step);
        return 0;
    }

    if (!p || !p->dh || !p->fw_ours) return -1;

    if (run && !p->pps.src_sof) mirisdr_set_gpio_direction(p, 0, 0);

    p->pps.anchor_valid = 0;
    mirisdr_pps_checks_reset(p, 0);

    /* bit 1 asks for the anchor, which the next streaming interrupt takes */
    if (run) mirisdr_anchor_open(p);
    if (libusb_control_transfer(p->dh, 0x40, CMD_PPS_ENABLE,
                                mirisdr_pps_wvalue(p, run), 0,
                                NULL, 0, CTRL_TIMEOUT) < 0)
    {
        if (run) mirisdr_anchor_none(p);
        return -1;
    }

    if (!run) return 0;
    if (!p->rate || !p->addr_step || !p->addr_valid)
    {
        mirisdr_anchor_none(p);
        return -1;
    }

    return mirisdr_pps_anchor(p);
}

/* The PPS_TIME reply. */
typedef struct {
    uint32_t full;          /* turns in the last full packet interval        */
    uint32_t packets;       /* free running packet count                     */
    uint32_t edge_tick;     /* the edge own tick in its interval             */
    uint32_t edge_packet;   /* packet count when the edge was latched        */
    uint8_t  run;           /* 0 once the host stopped, reset or rebooted    */
    uint8_t  guard;         /* the firmware's verdict on the capture, 0 = clean */
    uint8_t  edges;         /* edge count, wraps at 256                      */
    uint8_t  carry;         /* PPS_CARRY_* bits                              */
    uint16_t frame;         /* USB frame the edge fell in                    */
    uint8_t  sofs;          /* SOF interrupts in the last full interval      */
    uint8_t  edge_sof;      /* SOFs before the edge in its interval          */
    uint32_t first_tick;    /* the interval's first SOF's tick               */
    uint8_t  microframe;    /* the edge's microframe in its frame, on SOF    */
} mirisdr_pps_time_t;

#define PPS_CARRY_EDGE   1  /* the edge's tick was on the counter's wrap     */
#define PPS_CARRY_FIRST  2  /* the interval's first SOF's tick was           */
#define PPS_CARRY_USB    4  /* a USB interrupt ran between the streaming
                               interrupt and the first SOF                  */

static uint32_t mirisdr_pps_ticks (uint8_t lo, uint8_t hi)
{
    return (uint32_t) hi * 256 + ((256 - lo) & 0xFF);
}

static void mirisdr_pps_unpack (const uint8_t *b, mirisdr_pps_time_t *t)
{
    t->full        = mirisdr_pps_ticks(b[0], b[1]);
    t->packets     = mirisdr_pps_le32(b + 2);
    t->edge_tick   = mirisdr_pps_ticks(b[6], b[7]);
    t->edge_packet = mirisdr_pps_le32(b + 8);
    t->run         = b[12];
    t->guard       = b[13];
    t->edges       = b[14];
    t->carry       = b[15];
    t->frame       = (uint16_t) b[16] | ((uint16_t) b[17] << 8);
    t->sofs        = b[20];
    t->edge_sof    = b[21];
    t->first_tick  = mirisdr_pps_ticks(b[22], b[23]);
    t->microframe  = b[18] & 7;
}

/*
 * A SOF source edge is a SOF, and says which: its frame and microframe. The SOFs are
 * 125 us apart on the host's clock, so an edge sits on the line through the edges
 * before it, as many microframes on as its label says, and one that does not is one
 * the firmware could not see was wrong. The device misses a SOF now and then, which
 * leaves the microframe count one short, and it at times loses a few us with no
 * interrupt running, which moves the edge as much. Good edges sit within half a us.
 *
 * The line runs through two edges on it, the last and one up to twice the span
 * before, so its slope is good to well under a us over the span. What it may be off by
 * grows with how far ahead it is asked: the two ends' jitter over its length, and a
 * drift of the clocks against each other.
 */
#define MIRISDR_PPS_GRID_US     2.0     /* off the line by more than this flags the edge */
#define MIRISDR_PPS_GRID_SPAN_S 10.0    /* the line's length, between this and twice it */
#define MIRISDR_PPS_GRID_PPM    100.0   /* the clocks' difference before the line has a slope */
#define MIRISDR_PPS_LABELS      16384   /* microframes before the frame number wraps */

static void mirisdr_pps_checks_reset (mirisdr_dev_t *p, double expect)
{
    p->pps.chk_expect = expect;
    p->pps.chk_valid = 0;
    p->pps.grid_have = 0;
    p->pps.grid_off = 0;
}

static void mirisdr_pps_grid_start (mirisdr_dev_t *p, int64_t from, int64_t at, int label, int64_t microframes)
{
    p->pps.grid_a = p->pps.grid_m = from;
    p->pps.grid_ka = p->pps.grid_km = 0;
    p->pps.grid_b = at;
    p->pps.grid_kb = microframes;
    p->pps.grid_lb = label;
    p->pps.grid_have = 1;
    p->pps.grid_off = 0;
}

/* d is n microframes, n at least one and what the labels dl apart say, within tol and
   grow more a microframe */
static int mirisdr_pps_grid_fits (double d, double per, double tol, double grow, int dl, double *n)
{
    *n = floor(d / per + 0.5);

    return (*n >= 1) && ((((int64_t) *n - dl) % MIRISDR_PPS_LABELS) == 0)
        && (fabs(d - *n * per) <= tol + *n * grow);
}

/* 1 when the edge at, with its frame and microframe as label, is off the line */
static int mirisdr_pps_grid (mirisdr_dev_t *p, int64_t at, int label)
{
    double rate = p->rate, nominal = 0.000125 * rate;
    double tol = MIRISDR_PPS_GRID_US * 1e-6 * rate, span = MIRISDR_PPS_GRID_SPAN_S * rate, n;

    if (p->pps.grid_have)
    {
        int64_t k = p->pps.grid_kb - p->pps.grid_ka;
        double per = k ? (double) (p->pps.grid_b - p->pps.grid_a) / (double) k : nominal;
        double grow = (k ? 1e-6 * rate / (double) k : MIRISDR_PPS_GRID_PPM * 1e-6 * nominal) + 2e-7 * per;

        if (mirisdr_pps_grid_fits((double) (at - p->pps.grid_b), per, tol, grow,
                                  (label - p->pps.grid_lb) & (MIRISDR_PPS_LABELS - 1), &n))
        {
            p->pps.grid_b = at;
            p->pps.grid_lb = label;
            p->pps.grid_kb += (int64_t) n;
            p->pps.grid_off = 0;

            /* the line's far end moves up a half span at a time */
            if ((double) (p->pps.grid_b - p->pps.grid_m) >= span / 2)
            {
                if ((double) (p->pps.grid_b - p->pps.grid_a) >= span)
                {
                    p->pps.grid_a = p->pps.grid_m;
                    p->pps.grid_ka = p->pps.grid_km;
                }
                p->pps.grid_m = p->pps.grid_b;
                p->pps.grid_km = p->pps.grid_kb;
            }
            return 0;
        }

        /* edges off it that keep agreeing with each other start it again */
        if (p->pps.grid_off && mirisdr_pps_grid_fits((double) (at - p->pps.grid_cand), per, tol, grow,
                                                     (label - p->pps.grid_lc) & (MIRISDR_PPS_LABELS - 1), &n))
            p->pps.grid_off++;
        else
            p->pps.grid_off = 1;
        p->pps.grid_cand = at;
        p->pps.grid_lc = label;

        if (p->pps.grid_off >= 3) mirisdr_pps_grid_start(p, at, at, label, 0);
        return 1;
    }

    /* no line yet: an edge that agrees with the one before starts it */
    if (p->pps.grid_off && mirisdr_pps_grid_fits((double) (at - p->pps.grid_cand), nominal, tol,
                                                 MIRISDR_PPS_GRID_PPM * 1e-6 * nominal,
                                                 (label - p->pps.grid_lc) & (MIRISDR_PPS_LABELS - 1), &n))
    {
        mirisdr_pps_grid_start(p, p->pps.grid_cand, at, label, (int64_t) n);
        return 0;
    }

    p->pps.grid_cand = at;
    p->pps.grid_lc = label;
    p->pps.grid_off = 1;
    return 1;
}

int mirisdr_get_pps (mirisdr_dev_t *p, mirisdr_pps_t *out)
{
    uint8_t b[PPS_TIME_LEN];
    mirisdr_pps_time_t t;
    uint64_t now, edge, group;
    int64_t at;
    double first, expect, fullc, turns;
    uint8_t wrap;

    if (!p || !p->dh || !out || !p->fw_ours || !p->pps.anchor_valid) return -1;

    /* the block the firmware keeps, read as one snapshot */
    if (!p->fw_pps_at || mirisdr_read_mem(p, p->fw_pps_at, b, PPS_TIME_LEN, MIRISDR_MEM_IRAM) < 0) return -1;

    mirisdr_pps_unpack(b, &t);

    if (!t.run) return -1;

    /* The firmware's guard is a count, so we clamp to defined value */
    if (t.guard == 1 || t.guard == 2) t.guard = MIRISDR_PPS_GUARD_USB;

    /* The packet count wraps after about thirty hours at the fastest the chip
       streams: track it from the free running half, which is read every time. */
    if (t.packets < p->pps.irq_last) p->pps.irq_high++;
    p->pps.irq_last = t.packets;

    now  = t.packets     | ((uint64_t) p->pps.irq_high << 32);
    edge = t.edge_packet | ((uint64_t) p->pps.irq_high << 32);
    if (edge > now) edge-= 0x100000000ULL;      /* latched before the wrap */

    /* Where the edge sits in its interval: on SOF the interval's first SOF
       plus whole microframes, on GPIO_0 the latch itself, which the poll loop
       took, so no SOF is involved. */
    first = t.first_tick;

    if (!p->pps.src_sof) {
        first      = t.edge_tick;
        t.edge_sof = 0;
        t.carry    = (t.edge_tick & 0xFF) ? 0 : PPS_CARRY_EDGE | PPS_CARRY_FIRST;
    }

    group  = (uint64_t) p->addr_step * mirisdr_burst(p);
    expect = p->rate ? (double) group * 1e9 / p->rate / MIRISDR_PPS_TURN_NS : 0;
    fullc  = expect - MIRISDR_PPS_STREAM_TURNS;

    /* the edge's interval against the one a packet group takes at the rate, in turns of
       the firmware's loop */
    if (expect > 0
        && t.full > expect * MIRISDR_PPS_SCALE_LO
        && t.full < expect * MIRISDR_PPS_SCALE_HI)
    {
        /* What one SOF handler costs the loop.  Nothing is placed with it
           any more - it only says how much a handler is stalling. */
        double ke = t.sofs ? (fullc - t.full) / t.sofs : 0;

        p->pps.seen = 1;

        if (ke > 4.0 && ke < 60.0 && (p->pps.k_min <= 0 || ke < p->pps.k_min))
            p->pps.k_min = ke;
    }
    if (!p->pps.seen || fullc <= 0) return -1;   /* no interval seen yet */

    wrap = t.edge_sof == 0 ? PPS_CARRY_EDGE : PPS_CARRY_EDGE | PPS_CARRY_FIRST;

    if (!t.guard && (t.carry & wrap))
        t.guard = MIRISDR_PPS_GUARD_CARRY;

    if (!t.guard && p->pps.src_sof && first >= MIRISDR_PPS_UFRAME_TURNS)
        t.guard = MIRISDR_PPS_GUARD_CARRY;

    if (!t.guard && (t.carry & PPS_CARRY_USB))
        t.guard = MIRISDR_PPS_GUARD_USB;

    /* The edge landed on the streaming interrupt's tail, or ahead of it. */
    if (!t.guard && first < MIRISDR_PPS_TAIL_TURNS)
        t.guard = MIRISDR_PPS_GUARD_TAIL;

    turns = first + MIRISDR_PPS_UFRAME_TURNS * t.edge_sof + MIRISDR_PPS_STREAM_TURNS;

    at = p->pps.base + (int64_t) (edge * group)
       + (int64_t) ((turns * (double) group) / expect);

    /* once an edge, which repeated reads of it then share */
    if (p->pps.src_sof)
    {
        if (!p->pps.chk_valid || (t.edges != p->pps.chk_edges) || (t.edge_packet != p->pps.chk_packet))
        {
            if (expect != p->pps.chk_expect) mirisdr_pps_checks_reset(p, expect);

            p->pps.chk_guard = (!t.guard && mirisdr_pps_grid(p, at, (((int) t.frame << 3) | t.microframe)
                                                                        & (MIRISDR_PPS_LABELS - 1)))
                             ? MIRISDR_PPS_GUARD_GRID : 0;
            p->pps.chk_edges = t.edges;
            p->pps.chk_packet = t.edge_packet;
            p->pps.chk_valid = 1;
        }
        if (!t.guard) t.guard = p->pps.chk_guard;
    }

    memset(out, 0, sizeof *out);
    out->sample  = at < 0 ? 0 : (uint64_t) at;
    out->edges   = t.edges;
    out->trusted = !t.guard;
    out->guard   = t.guard;
    out->frame   = t.frame;
    out->microframe = p->pps.src_sof ? t.microframe : 0;

    /* any gap, one gap fill covered too: the device's count went on without the stream */
    {
        uint64_t index = 0, gaps = p->pps.gaps0;
        uint32_t addr = 0;

        mirisdr_pps_count(p, &index, &addr, &gaps);
        if ((gaps != p->pps.gaps0) && (mirisdr_pps_reanchor(p) < 0)) return -1;
    }

    if (p->pps.stale && (out->edges != p->pps.edge0)) p->pps.stale = 0;

    out->gapless = !p->pps.stale;

    /* Edge before the anchor? */
    if (at < 0) return -1;

    return 0;
}
