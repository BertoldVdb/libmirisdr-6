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

/* The 32 bits as read:
 *
 *   31:28  0
 *   27:26  always 11 so far
 *   25:23  VCO range, one-hot: 001 low, 010 middle, 100 high, 000 synthesizer off
 *   22:18  VCO coarse code
 *   17:14  up-converter LO calibration (updated while the up-converter is on)
 *   13:9   VCO fine code
 *    8:5   L-band LNA calibration (updated by a register 2 write with bit 22)
 *    4:0   a count set by XTALSEL, +-1 between calibrations
 */

#define MIRISDR_TUNER_READBACK  0x00001C    /* register 12, data bit 0 */

int mirisdr_get_tuner_status (mirisdr_dev_t *p, mirisdr_tuner_status_t *st)
{
    uint8_t b[4];
    uint32_t v, range;

    if (!p || !st) goto failed;

    /* the tuner port is wired somewhere else, or nowhere */
    if (p->external_tuner) goto failed;

    /* clock with register 0 as a safety, so nothing before the first tune */
    if (!p->tuner_reg0) goto failed;

    if (mirisdr_write_reg(p, 0x09, MIRISDR_TUNER_READBACK) < 0) goto failed;
    if (mirisdr_write_reg(p, 0x09, p->tuner_reg0) < 0) goto failed;
    if (mirisdr_read_reg(p, 4, b, 4) != 4) goto failed;

    v = b[0] | b[1] << 8 | (uint32_t) b[2] << 16 | (uint32_t) b[3] << 24;

    /* the line left idle rather than driven by the tuner */
    if ((v & 0x3FFFFFFF) == 0 || (v & 0x3FFFFFFF) == 0x3FFFFFFF) goto failed;

    memset(st, 0, sizeof *st);
    st->raw     = v;
    st->top     = (v >> 26) & 3;
    st->coarse  = (v >> 18) & 31;
    st->fine    = (v >> 9) & 31;
    st->upconv  = (v >> 14) & 15;
    st->lna_cal = (v >> 5) & 15;
    st->xtal    = v & 31;

    range = (v >> 23) & 7;
    st->vco_range = range == 1 ? 0 : range == 2 ? 1 : range == 4 ? 2 : -1;

    if (!range) st->flags |= MIRISDR_TUNER_SYNTH_OFF;
    if (range == 1 && st->coarse == 22 && st->fine == 31) st->flags |= MIRISDR_TUNER_AT_LOW_LIMIT;
    if (range == 4 && st->fine == 0) st->flags |= MIRISDR_TUNER_AT_HIGH_LIMIT;

    return 0;

failed:
    return -1;
}
