/*
 * Copyright (C) 2013 by Miroslav Slugen <thunder.m@email.cz
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

#define CMD_REG_LIST            0x5A
#define MIRISDR_LIST_MAX        63      /* a bank is one page */

/* Up to 64 bytes in one request with our firmware, copied with interrupts off so
   they are one snapshot: xdata, or internal RAM.  The ROM gives 4 at a time. */
static int mirisdr_read_block (mirisdr_dev_t *p, uint16_t addr, int iram, int remap, uint8_t *buf, int len)
{
    return (libusb_control_transfer(p->dh, 0xC0, 0x42, (uint16_t) ((remap ? 1 : 0) | (iram ? 2 : 0)),
                                    (uint16_t) (iram ? addr : addr - 0xC000), buf, (uint16_t) len,
                                    CTRL_TIMEOUT) == len) ? 0 : -1;
}

/* The firmware's list block: run, waiting, PPS paused, SPI timeout, queued, bank,
   entry offset, passes */
#define MIRISDR_LIST_BLOCK      9

/* The firmware refuses register writes while a list runs. Do short wait for the list to be done */
static void mirisdr_batch_wait (mirisdr_dev_t *p)
{
    uint8_t run;
    int i;

    for (i = 0; p->fw_list_at && i < 100; i++)
        if (mirisdr_read_block(p, p->fw_list_at, 1, 0, &run, 1) < 0 || !run) break;

    p->batch_running = 0;
}

static int mirisdr_batch_flush (mirisdr_dev_t *p)
{
    int n = p->batch_n;

    if (!n) return 0;
    p->batch_n = 0;

    /* a list replaces a running one: let the last tune's finish first */
    if (p->batch_running) mirisdr_batch_wait(p);

    if (libusb_control_transfer(p->dh, 0x42, CMD_REG_LIST, 0, 0, p->batch, (uint16_t) (n * 4),
                                CTRL_TIMEOUT) == n * 4)
    {
        p->batch_running = 1;
        return 0;
    }

    /* what went out is not known */
    p->tuner_valid = 0;
    p->reg8_valid = 0;

    return -1;
}

/* Hold register writes for one list request, nested. If firmware doesn't support it, does nothing. */
static void mirisdr_batch_begin (mirisdr_dev_t *p)
{
    p->batch_depth++;
}

static int mirisdr_batch_end (mirisdr_dev_t *p)
{
    if (p->batch_depth && --p->batch_depth) return 0;

    return mirisdr_batch_flush(p);
}

int mirisdr_load_list (mirisdr_dev_t *p, int bank, int flags, const mirisdr_list_entry_t *e, int n)
{
    uint8_t buf[4 * MIRISDR_LIST_MAX];
    int i;

    if (!p || !p->dh || !p->fw_ours) return -1;
    if ((bank & ~1) || (n < 0) || (n > MIRISDR_LIST_MAX) || (n && !e)) return -1;

    for (i = 0; i < n; i++)
    {
        buf[4 * i]     = e[i].reg;
        buf[4 * i + 1] = (uint8_t) e[i].val;
        buf[4 * i + 2] = (uint8_t) (e[i].val >> 8);
        buf[4 * i + 3] = (uint8_t) (e[i].val >> 16);
    }

    if (p->batch_running) mirisdr_batch_wait(p);

    /* the list writes behind the caches' back */
    p->tuner_valid = 0;
    p->reg8_valid = 0;

    return (libusb_control_transfer(p->dh, 0x42, CMD_REG_LIST,
                                    (uint16_t) (bank | ((flags & MIRISDR_LIST_QUEUE) ? 2 : 0)),
                                    (uint16_t) ((flags >> 8) & 0xff), n ? buf : NULL, (uint16_t) (4 * n),
                                    CTRL_TIMEOUT) == 4 * n) ? 0 : -1;
}

int mirisdr_get_list_status (mirisdr_dev_t *p, mirisdr_list_status_t *st)
{
    uint8_t b[MIRISDR_LIST_BLOCK];

    if (!p || !p->dh || !p->fw_ours || !p->fw_list_at || !st) return -1;

    if (mirisdr_read_block(p, p->fw_list_at, 1, 0, b, sizeof b) < 0) return -1;

    st->running     = b[0];
    st->waiting     = b[1];
    st->pps_paused  = b[2];
    st->spi_timeout = b[3];
    st->queued      = b[4];
    st->bank        = b[5];
    st->entry       = b[6] >> 2;
    st->passes      = (uint16_t) (b[7] | b[8] << 8);

    return 0;
}

/* Every word to the tuner passes here, raw writes included, so the cache stays true */
static void mirisdr_tuner_track (mirisdr_dev_t *p, uint32_t w, int ok)
{
    unsigned a = w & 15;

    /* clocked the readback out, the line was the other way */
    if (p->tuner_turned)
    {
        p->tuner_turned = 0;
        return;
    }

    /* register 12 is a one-shot, never cached */
    if (a == 12)
    {
        p->tuner_turned = ok && (w & 0x10);
        return;
    }

    if (ok)
    {
        p->tuner_reg[a] = w;
        p->tuner_valid |= (uint16_t) (1u << a);
    }
    else p->tuner_valid &= (uint16_t) ~(1u << a);
}

int mirisdr_write_reg (mirisdr_dev_t *p, uint8_t reg, uint32_t val) {
    uint16_t value = (val & 0xff) << 8 | reg;
    uint16_t index = (val >> 8) & 0xffff;
    int r;

    if (!p) goto failed;
    if (!p->dh) goto failed;

#if MIRISDR_DEBUG >= 2
    fprintf( stderr, "write reg: 0x%02x, val 0x%08x\n", reg, val);
#endif

    if (p->batch_depth && p->fw_ours)
    {
        uint8_t *e = p->batch + 4 * p->batch_n;

        e[0] = reg;
        e[1] = (uint8_t) val;
        e[2] = (uint8_t) (val >> 8);
        e[3] = (uint8_t) (val >> 16);
        r = (++p->batch_n == MIRISDR_LIST_MAX) ? mirisdr_batch_flush(p) : 0;
    }
    else
    {
        if (p->batch_running) mirisdr_batch_wait(p);
        r = libusb_control_transfer(p->dh, 0x42, 0x41, value, index, NULL, 0, CTRL_TIMEOUT);
    }

    if (reg == 0x09) mirisdr_tuner_track(p, val, r >= 0);

    if (reg == 0x08)
    {
        p->reg8_sent = val;
        p->reg8_valid = r >= 0;
    }

    /* the SPI master shares the tuner's lines on boards without a gate */
    if ((reg >= 0x0B) && (reg <= 0x0D)) p->tuner_valid = 0;

    return r;

failed:
    return -1;
}

/* Send a tuner register unless the tuner already holds it: 1 sent, 0 skipped, -1 failed */
static int mirisdr_tuner_write (mirisdr_dev_t *p, uint8_t reg, uint32_t data, int force)
{
    uint32_t w = (data << 4) | (reg & 15);

    if (!force && ((p->tuner_valid >> (reg & 15)) & 1) && (p->tuner_reg[reg & 15] == w)) return 0;

    return (mirisdr_write_reg(p, 0x09, w) < 0) ? -1 : 1;
}

#define CMD_RESET              0x40
#define CMD_WREG               0x41
#define CMD_RREG               0x42
#define CMD_START_STREAMING    0x43
#define CMD_DOWNLOAD           0x44
#define CMD_STOP_STREAMING     0x45
//WValue = Addr?
#define CMD_REEPROM            0x46
//WValue = Addr?
#define CMD_WEEPROM            0x47
#define CMD_READ_UNKNOWN       0x48
//wValue = gpio << 8 | val
#define CMD_WGPIO              0x49
#define CMD_EXT_WGPIO_BASE     0x4b
/*
RSP1
GPIO(0x13) & 0x01 = DSB_NOTCH
GPIO(0x13) & 0x04 = BROADCAST_NOTCH




*/

/*
 * The 24 bit registers written above are write only.  The read window is a
 * separate decode, four bytes per index, and holds status rather than a mirror
 * of what was written - index 6 is the GPIO input nibble.
 */
int mirisdr_read_reg (mirisdr_dev_t *p, uint8_t index, uint8_t *buf, int len) {
    if (!p) goto failed;
    if (!p->dh) goto failed;

    return libusb_control_transfer(p->dh, 0xC0, CMD_RREG, 0, index * 4, buf, len, CTRL_TIMEOUT);

failed:
    return -1;
}