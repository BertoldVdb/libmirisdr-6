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
 * An edge on GPIO_0 is latched by the firmware's poll loop as a packet count
 * and a position inside that packet, six machine cycles per tick. The anchor
 * ties packet counts to the streams sample counter.
 * Taking the anchor requires remapping the DSP buffer which corrupts some ADC samples,
 * so it is only done once per gap (in practice: once per stream start).
 */

static uint32_t mirisdr_burst (mirisdr_dev_t *p);

#define CMD_PPS_TIME            0x51
#define CMD_PPS_ENABLE          0x52
#define CMD_PPS_ANCHOR          0x53

#define MIRISDR_PPS_TICK_PS     200500

/* Which of the four capture buffers the anchor interrupt was announcing. */
#define MIRISDR_PPS_ANCHOR_BACK 0

static uint32_t mirisdr_pps_le32 (const uint8_t *b)
{
    return (uint32_t) b[0] | ((uint32_t) b[1] << 8)
         | ((uint32_t) b[2] << 16) | ((uint32_t) b[3] << 24);
}

/* The anchor is taken by the next streaming interrupt, so it is read here while
   the sample counter it carries is still within a wrap of the stream: from then
   on everything is 64 bit and counted from it. */
static int mirisdr_pps_anchor (mirisdr_dev_t *p)
{
    uint8_t b[18];
    uint32_t c[4];
    int i, newest = 0, tries;

    for (tries = 0; tries < 100; tries++)
    {
        if (libusb_control_transfer(p->dh, 0xC0, CMD_PPS_ANCHOR, 0, 0, b, sizeof(b),
                                    CTRL_TIMEOUT) != (int) sizeof(b)) return -1;

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
    p->pps_irq_last = 0;
    p->pps_irq_high = 0;

    /* whatever is latched now was captured against the count this anchor has
       just reset, so it is not placeable until the next pulse */
    if (libusb_control_transfer(p->dh, 0xC0, CMD_PPS_TIME, 0, 0, b, 16,
                                CTRL_TIMEOUT) != 16) return -1;

    p->pps_edge0 = b[14];
    p->pps_stale = 1;

    return 0;
}

/* A gap unties the packet count from real time and nothing here can put that
   back, so the anchor is taken again. */
static int mirisdr_pps_reanchor (mirisdr_dev_t *p)
{
    if (libusb_control_transfer(p->dh, 0x40, CMD_PPS_ENABLE, 3, 0, NULL, 0,
                                CTRL_TIMEOUT) < 0) return -1;

    return mirisdr_pps_anchor(p);
}

int mirisdr_enable_pps (mirisdr_dev_t *p, int run)
{
    if (!p || !p->dh || !p->fw_ours) return -1;

    p->pps_anchor_valid = 0;

    /* bit 1 asks for the anchor, which the next streaming interrupt takes */
    if (libusb_control_transfer(p->dh, 0x40, CMD_PPS_ENABLE, run ? 3 : 0, 0,
                                NULL, 0, CTRL_TIMEOUT) < 0) return -1;

    if (!run) return 0;
    if (!p->rate || !p->addr_step || !p->addr_valid) return -1;

    return mirisdr_pps_anchor(p);
}

int mirisdr_get_pps (mirisdr_dev_t *p, mirisdr_pps_t *out)
{
    uint8_t b[16];
    uint64_t ticks, now, edge, group;

    if (!p || !p->dh || !out || !p->fw_ours || !p->pps_anchor_valid) return -1;

    if (libusb_control_transfer(p->dh, 0xC0, CMD_PPS_TIME, 0, 0, b, sizeof(b),
                                CTRL_TIMEOUT) != (int) sizeof(b)) return -1;

    /* The packet count wraps after about thirty hours at the fastest the chip
       streams: track it from the free running half, which is read every time. */
    now = mirisdr_pps_le32(b + 2);

    if (now < p->pps_irq_last) p->pps_irq_high++;

    p->pps_irq_last = (uint32_t) now;
    now|= (uint64_t) p->pps_irq_high << 32;

    edge = mirisdr_pps_le32(b + 8) | ((uint64_t) p->pps_irq_high << 32);

    if (edge > now) edge-= 0x100000000ULL;      /* latched before the wrap */

    ticks = (uint64_t) b[7] * 256 + ((256 - b[6]) & 0xFF);
    group = (uint64_t) p->addr_step * mirisdr_burst(p);

    out->sample = (uint64_t) (p->pps_base + (int64_t) (edge * group)
                + (int64_t) ((ticks * MIRISDR_PPS_TICK_PS * p->rate) / 1000000000000ULL));
    out->edges = b[14];
    out->trusted = !b[13];

    if (p->stats.lost != p->pps_lost0)
    {
        if (mirisdr_pps_reanchor(p) < 0) return -1;
    }

    if (p->pps_stale && (out->edges != p->pps_edge0)) p->pps_stale = 0;

    out->gapless = !p->pps_stale;

    return 0;
}
