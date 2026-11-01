/*
 * At some very high sample rates the index counter is not exact. I assume this
 * is because the DSP block takes a snapshot a free running counter. These are
 * not lost samples, as we have tested that the stream always makes up for them.
 * (eg sometimes -1, next packet 0, then +1, ...). It happens, very rarely, at lower
 * rates as well.
 */
#define MIRISDR_ADDR_JITTER 4

/* After a start the device can still hold blocks of the stream before, at times
   with a header cleared or overwritten: up to 4 kB in the sample engine and 2x
   4 kB in the USB FIFO. Those 12 blocks are dropped and the next starts the count */
#define MIRISDR_ADDR_DROP 12

/* Without our firmware's stamp the most a gap is taken to be, about a second at the
   highest rate: more is a misread header */
#define MIRISDR_ADDR_GAP_MAX (1u << 25)

/* Output bytes a sample, I/Q pair or real value, takes */
static uint32_t mirisdr_unit_bytes (mirisdr_dev_t *p) {
    switch (p->format) {
    case MIRISDR_FORMAT_504_S8:
    case MIRISDR_FORMAT_504_REAL_S16:
    case MIRISDR_FORMAT_672_REAL_S16:
    case MIRISDR_FORMAT_768_REAL_S16:
        return 2;
    default:
        return 4;
    }
}

/* A gap of 'missing' samples before the block being converted: queue it for the
   callback that delivers it, and the zeros for gap fill. Returns the samples filled. */
static uint64_t mirisdr_gap_note (mirisdr_dev_t *p, uint64_t missing) {
    uint64_t f = 0, ub = mirisdr_unit_bytes(p), at;
    int i;

    if (!p->gap_track) return 0;

    /* up to one bulk transfer's worth. Not with the queue full: the gap folds into the
       last one, and fill there would not follow on from that one's */
    if (p->gap_fill && p->fills_n < (int) (sizeof p->fills / sizeof p->fills[0]) &&
        p->gapq_n < (int) (sizeof p->gapq / sizeof p->gapq[0])) {
        f = missing;
        if (f > 16 * (uint64_t) p->addr_step) f = 16 * (uint64_t) p->addr_step;
        p->fills[p->fills_n].off = (uint32_t) ((p->stats.samples - p->conv_samples - p->conv_filled) * ub);
        p->fills[p->fills_n].n = (uint32_t) f;
        p->fills_n++;
        p->conv_filled += f;
    }

    at = p->fed_bytes + (p->stats.samples - p->conv_samples) * ub;
    i = p->gapq_n;
    if (i == (int) (sizeof p->gapq / sizeof p->gapq[0])) {
        /* full: fold into the last, unfilled, so the timeline still adds up */
        i--;
        p->gapq[i].samples += missing;
    } else {
        p->gapq[i].at = at;
        p->gapq[i].samples = missing;
        p->gapq[i].filled = (uint32_t) f;
        p->gapq_n++;
    }

    return f;
}

/* 1 while this block may be one an anchor damaged, see mirisdr_anchor_open() */
static int mirisdr_anchor_suspect (mirisdr_dev_t *p) {
    int open = mirisdr_atomic_add(&p->anc_open, 0);

    if (!p->addr_valid || (open == p->anc_shut)) return 0;
    /* taken, not yet read back: any block may be the one */
    if (mirisdr_atomic_add(&p->anc_known, 0) != open) return 1;
    if ((int32_t) (p->addr - p->anc_to) > 0) {
        p->anc_shut = open;
        return 0;
    }
    return (int32_t) (p->addr - p->anc_from) >= 0;
}

/* Called once per 1024 byte block with its header, which carries the sample
   counter and the IR block last completed run (if enabled). */
/* returns 1 for a block to drop */
static int mirisdr_addr_next (mirisdr_dev_t *p, const uint8_t *hdr, uint32_t step) {
    uint32_t addr = (uint32_t) hdr[3] << 24 | (uint32_t) hdr[2] << 16 | (uint32_t) hdr[1] << 8 | hdr[0];
    const uint32_t seen = addr;
    int32_t d;
    int suspect;

    if (p->addr_restart) {
        if (p->addr_dropped++ < MIRISDR_ADDR_DROP) return 1;

        /* the device counts again from its start */
        p->addr_restart = 0;
        /* the events go on from the last packet before: what a setting changed while
           stopped shows in the first one after, with the packets between missed */
        if (p->addr_valid) {
            mirisdr_ir_resync(p);
            p->ev_missed = 1;
        }
        p->addr_valid = 0;

        /* a window placed in the stream before ends with it */
        {
            int open = mirisdr_atomic_add(&p->anc_open, 0);

            if (mirisdr_atomic_add(&p->anc_known, 0) == open) p->anc_shut = open;
        }
    }

    d = (int32_t) (addr - p->addr);
    suspect = mirisdr_anchor_suspect(p);

    if (!p->addr_valid) {
        p->addr_valid = 1;
    } else if (((d > MIRISDR_ADDR_JITTER) || (d < -MIRISDR_ADDR_JITTER)) && suspect &&
               (addr != p->addr_seen + step)) {
        /* where the anchor may have damaged it: garbage, unless the next header agrees,
           when it is taken below as any other */
        fprintf(stderr, "libmirisdr: a block header off the count as the anchor was taken, %08x:%08x\n",
                p->addr, addr);
        addr = p->addr;
    } else if ((d > MIRISDR_ADDR_JITTER) || (d < -MIRISDR_ADDR_JITTER)) {
        /* Blocks go missing whole. Without our firmware's stamp a misaligned stream reads
           sample data as a header: a counter that went backwards, or on by part of a block,
           is that, not samples that went missing. The count goes on from the last true one */
        uint32_t part = (d > 0) ? (uint32_t) d % step : 0;
        int misread = (d < 0) || (!p->fw_ours && (((uint32_t) d > MIRISDR_ADDR_GAP_MAX) ||
                                                  ((part > MIRISDR_ADDR_JITTER) && (step - part > MIRISDR_ADDR_JITTER))));

        p->stats.gaps++;
        p->sync_run++;
        mirisdr_ir_resync(p);
        p->ev_missed = 1;

        /* two headers in a row that agree with each other: the count is theirs from here,
           what came between unknown */
        if (misread && (addr == p->addr_seen + step)) {
            fprintf(stderr, "libmirisdr: the block count taken again, %08x:%08x\n", p->addr, addr);
        } else if (misread) {
            fprintf(stderr, "libmirisdr: a block header off the count, %08x:%08x\n", p->addr, addr);
            addr = p->addr;
        } else {
            uint64_t f = mirisdr_gap_note(p, (uint32_t) d);

            fprintf(stderr, "%d samples lost, %08x:%08x\n", d, p->addr, addr);

            /* filled samples are delivered, at their own index */
            p->stats.lost += (uint32_t) d - f;
            p->stats.samples += f;
            p->stats.filled += f;
        }
    } else {
        if (d) p->stats.jitter++;
        p->sync_run = 0;
    }

    if (p->stats_head) {
        p->stats_head = 0;
        p->stats.index = p->stats.samples + p->stats.lost;
    }

    /* a damaged header's events would be garbage, the next one's tell what changed */
    if (!suspect) mirisdr_events_latch(p, hdr, p->stats.samples, p->stats.samples + p->stats.lost, step);

    p->stats.samples+= step;
    p->addr_seen = seen;
    p->addr = addr + step;

    mirisdr_atomic_add(&p->count_seq, 1);
    p->count_index = p->stats.samples + p->stats.lost;
    p->count_addr = p->addr;
    p->count_gaps = p->stats.gaps;
    mirisdr_atomic_add(&p->count_seq, 1);

    if (suspect) mirisdr_ir_resync(p);
    else mirisdr_ir_latch(p, hdr[6], addr);

    return 0;
}

#include "252_s16.c"
#include "336_s16.c"
#include "384_s16.c"
#include "504_s16.c"
#include "504_s8.c"

