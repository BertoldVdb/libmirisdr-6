/*
 * At some very high sample rates the index counter is not exact. I assume this
 * is because the DSP block takes a snapshot a free running counter. These are
 * not lost samples, as we have tested that the stream always makes up for them.
 * (eg sometimes -1, next packet 0, then +1, ...). It happens, very rarely, at lower
 * rates as well.
 */
#define MIRISDR_ADDR_JITTER 4

static void mirisdr_addr_next (mirisdr_dev_t *p, uint32_t addr, uint32_t step) {
    int32_t d = (int32_t) (addr - p->addr);

    if (!p->addr_valid) {
        p->addr_valid = 1;
    } else if ((d > MIRISDR_ADDR_JITTER) || (d < -MIRISDR_ADDR_JITTER)) {
        fprintf(stderr, "%d samples lost, %08x:%08x\n", d, p->addr, addr);
        p->stats.gaps++;
        p->sync_run++;

        /* a counter that went backwards is a misaligned stream reading sample
           data as a header, not samples that went missing */
        if (d > 0) p->stats.lost+= (uint32_t) d;
    } else {
        if (d) p->stats.jitter++;
        p->sync_run = 0;
    }

    if (p->stats_head) {
        p->stats_head = 0;
        p->stats.index = p->stats.samples + p->stats.lost;
    }

    p->stats.samples+= step;
    p->addr = addr + step;
}

#include "252_s16.c"
#include "336_s16.c"
#include "384_s16.c"
#include "504_s16.c"
#include "504_s8.c"

