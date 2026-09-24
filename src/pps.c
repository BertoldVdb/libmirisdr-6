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

#define CMD_PPS_TIME            0x51
#define CMD_PPS_ENABLE          0x52
#define CMD_PPS_ANCHOR          0x53

#define PPS_TIME_LEN            24
#define PPS_ANCHOR_LEN          18

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

static uint32_t mirisdr_pps_le32 (const uint8_t *b)
{
    return (uint32_t) b[0] | ((uint32_t) b[1] << 8)
         | ((uint32_t) b[2] << 16) | ((uint32_t) b[3] << 24);
}

static uint16_t mirisdr_pps_wvalue (mirisdr_dev_t *p, int run)
{
    uint16_t v = run ? 3 : 0;               /* bit 0 run, bit 1 take an anchor */

    if (p->pps_src_sof) v |= 8;             /* edges from the SOF interrupt */
    v |= (uint16_t)(p->pps_div ? p->pps_div : 1) << 8;

    return v;
}

int mirisdr_set_pps_source (mirisdr_dev_t *p, int sof, int divider)
{
    if (!p || divider < 1 || divider > 255) return -1;

    p->pps_src_sof = sof ? 1 : 0;
    p->pps_div = divider;

    return 0;
}

/* The anchor is taken by the next streaming interrupt, so it is read here while
   the sample counter it carries is still within a wrap of the stream: from then
   on everything is 64 bit and counted from it. */
static int mirisdr_pps_anchor (mirisdr_dev_t *p)
{
    uint8_t b[PPS_TIME_LEN];
    uint32_t c[4];
    int i, newest = 0, tries;

    for (tries = 0; tries < 100; tries++)
    {
        if (libusb_control_transfer(p->dh, 0xC0, CMD_PPS_ANCHOR, 0, 0, b, PPS_ANCHOR_LEN,
                                    CTRL_TIMEOUT) != PPS_ANCHOR_LEN) return -1;

        if (b[16]) break;

        usleep(2000);
    }

    if (!b[16]) return -1;

    for (i = 0; i < 4; i++) c[i] = mirisdr_pps_le32(b + i * 4);

    /* consecutive packets in an unknown rotation, so the newest is found rather
       than indexed */
    for (i = 1; i < 4; i++)
        if ((int32_t) (c[i] - c[newest]) > 0) newest = i;

    p->pps_base = (int64_t) (p->stats.samples + p->stats.lost)
                + (int32_t) (c[newest] - MIRISDR_PPS_ANCHOR_BACK * p->addr_step - p->addr);

    p->pps_lost0 = p->stats.lost;
    p->pps_anchor_valid = 1;
    p->pps_seen = 0;
    p->pps_k_min = 0;
    p->pps_irq_last = 0;
    p->pps_irq_high = 0;

    /* whatever is latched now was captured against the count this anchor has
       just reset, so it is not placeable until the next pulse */
    if (libusb_control_transfer(p->dh, 0xC0, CMD_PPS_TIME, 0, 0, b, PPS_TIME_LEN,
                                CTRL_TIMEOUT) != PPS_TIME_LEN) return -1;

    p->pps_edge0 = b[14];
    p->pps_stale = 1;

    return 0;
}

/* A gap unties the packet count from real time and nothing here can put that
   back, so the anchor is taken again. */
static int mirisdr_pps_reanchor (mirisdr_dev_t *p)
{
    if (libusb_control_transfer(p->dh, 0x40, CMD_PPS_ENABLE,
                                mirisdr_pps_wvalue(p, 1), 0, NULL, 0,
                                CTRL_TIMEOUT) < 0) return -1;

    return mirisdr_pps_anchor(p);
}

int mirisdr_enable_pps (mirisdr_dev_t *p, int run)
{
    if (!p || !p->dh || !p->fw_ours) return -1;

    if (run && !p->pps_src_sof) mirisdr_set_gpio_direction(p, 0, 0);

    p->pps_anchor_valid = 0;

    /* bit 1 asks for the anchor, which the next streaming interrupt takes */
    if (libusb_control_transfer(p->dh, 0x40, CMD_PPS_ENABLE,
                                mirisdr_pps_wvalue(p, run), 0,
                                NULL, 0, CTRL_TIMEOUT) < 0) return -1;

    if (!run) return 0;
    if (!p->rate || !p->addr_step || !p->addr_valid) return -1;

    return mirisdr_pps_anchor(p);
}

int mirisdr_get_pps (mirisdr_dev_t *p, mirisdr_pps_t *out)
{
    uint8_t b[PPS_TIME_LEN];
    uint64_t ticks, first, now, edge, group;
    int64_t at;
    double expect, fullc;

    if (!p || !p->dh || !out || !p->fw_ours || !p->pps_anchor_valid) return -1;

    if (libusb_control_transfer(p->dh, 0xC0, CMD_PPS_TIME, 0, 0, b, PPS_TIME_LEN,
                                CTRL_TIMEOUT) != PPS_TIME_LEN) return -1;

    /* Run flag */
    if (!b[12]) return -1;

    /* The packet count wraps after about thirty hours at the fastest the chip
       streams: track it from the free running half, which is read every time. */
    now = mirisdr_pps_le32(b + 2);

    if (now < p->pps_irq_last) p->pps_irq_high++;

    p->pps_irq_last = (uint32_t) now;
    now|= (uint64_t) p->pps_irq_high << 32;

    edge = mirisdr_pps_le32(b + 8) | ((uint64_t) p->pps_irq_high << 32);

    if (edge > now) edge-= 0x100000000ULL;      /* latched before the wrap */

    ticks = (uint64_t) b[7] * 256 + ((256 - b[6]) & 0xFF);
    first = (uint64_t) b[23] * 256 + ((256 - b[22]) & 0xFF);

    /* On GPIO_0 the poll loop latches the edge itself */
    if (!p->pps_src_sof) {
        first = ticks;
        b[21] = 0;
        b[15] = b[6] ? 0 : 3;      /* one tick, so both carry bits track it */
    }

    group = (uint64_t) p->addr_step * mirisdr_burst(p);

    expect = p->rate ? (double) group * 1e9 / p->rate / MIRISDR_PPS_TURN_NS : 0;
    fullc = expect - MIRISDR_PPS_STREAM_TURNS;

    {
        uint32_t full = (uint32_t) b[1] * 256 + ((256 - b[0]) & 0xFF);

        /* Validate against 30MHz clock */
        if (expect > 0
            && full > expect * MIRISDR_PPS_SCALE_LO
            && full < expect * MIRISDR_PPS_SCALE_HI)
        {
            /* What one SOF handler costs the loop.  Nothing is placed with it
               any more - it only says how much a handler is stalling. */
            double ke = b[20] ? (fullc - full) / b[20] : 0;

            p->pps_seen = 1;

            if (ke > 4.0 && ke < 60.0 && (p->pps_k_min <= 0 || ke < p->pps_k_min))
                p->pps_k_min = ke;
        }
    }
    if (!p->pps_seen || fullc <= 0) return -1;   /* no interval seen yet */


    /* A tick whose low byte has just wrapped is 256 turns, 51.2 us, either
       way: the carry may or may not have run yet, mark as bad */
    if ((b[15] & (b[21] == 0 ? 1 : 3)) && !b[13]) b[13] = MIRISDR_PPS_GUARD_CARRY;

    if (p->pps_src_sof && (double) first >= MIRISDR_PPS_UFRAME_TURNS && !b[13])
        b[13] = MIRISDR_PPS_GUARD_CARRY;

    /* A USB interrupt between the streaming interrupt and the first SOF */
    if ((b[15] & 4) && !b[13]) b[13] = 2;

    /* The edge landed on the streaming interrupt's tail, or ahead of it.  On
       SOF that is the first SOF, on GPIO_0 the latch itself. */
    if ((double) first < MIRISDR_PPS_TAIL_TURNS && !b[13])
        b[13] = MIRISDR_PPS_GUARD_TAIL;

    {
        double turns = (double) first + MIRISDR_PPS_UFRAME_TURNS * b[21]
                     + MIRISDR_PPS_STREAM_TURNS;

        at = p->pps_base + (int64_t) (edge * group)
           + (int64_t) ((turns * (double) group) / expect);

        out->sample = at < 0 ? 0 : (uint64_t) at;
    }

    out->edges = b[14];
    out->trusted = !b[13];
    out->guard = b[13];
    out->frame = (uint16_t)b[16] | ((uint16_t)b[17] << 8);

    if (p->stats.lost != p->pps_lost0)
    {
        if (mirisdr_pps_reanchor(p) < 0) return -1;
    }

    if (p->pps_stale && (out->edges != p->pps_edge0)) p->pps_stale = 0;

    out->gapless = !p->pps_stale;

    /* Edge before the anchor? */
    if (at < 0) return -1;

    return 0;
}
