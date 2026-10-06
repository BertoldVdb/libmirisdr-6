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

    r = libusb_control_transfer(p->dh, 0x42, 0x41, value, index, NULL, 0, CTRL_TIMEOUT);

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