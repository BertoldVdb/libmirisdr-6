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
 *   25:23  VCO coarse, the range, one-hot: 001 low, 010 middle, 100 high, 000 off
 *   22:18  unknown, a finer VCO frequency control set by a second search after
 *          the band, higher is lower; the search register, not an overridden value
 *   17:14  up-converter LO calibration (updated while the up-converter is on)
 *   13:9   VCO fine, the tank capacitor band within the range
 *    8:5   L-band LNA calibration (updated by a register 2 write with bit 22)
 *    4:0   IF filter calibration code, timed against the crystal XTALSEL names,
 *          +-1 between calibrations; the code in force unless overridden
 */

#define MIRISDR_TUNER_READBACK  0x00001C    /* register 12, data bit 0 */

int mirisdr_get_tuner_status (mirisdr_dev_t *p, mirisdr_tuner_status_t *st)
{
    uint8_t b[4];
    uint32_t v, range;

    if (!p || !st) goto failed;

    /* the tuner port is wired somewhere else, or nowhere */
    if (p->external_tuner) goto failed;

    /* clock with the register 0 word the tuner already has, so nothing before the first tune */
    if (!(p->tuner_valid & 1)) goto failed;

    if (mirisdr_write_reg(p, 0x09, MIRISDR_TUNER_READBACK) < 0) goto failed;
    if (mirisdr_write_reg(p, 0x09, p->tuner_reg[0]) < 0) goto failed;
    if (mirisdr_read_reg(p, 4, b, 4) != 4) goto failed;

    v = b[0] | b[1] << 8 | (uint32_t) b[2] << 16 | (uint32_t) b[3] << 24;

    /* the line left idle rather than driven by the tuner */
    if ((v & 0x3FFFFFFF) == 0 || (v & 0x3FFFFFFF) == 0x3FFFFFFF) goto failed;

    memset(st, 0, sizeof *st);
    st->raw     = v;
    st->top     = (v >> 26) & 3;
    st->unknown = (v >> 18) & 31;
    st->fine    = (v >> 9) & 31;
    st->upconv  = (v >> 14) & 15;
    st->lna_cal = (v >> 5) & 15;
    st->filter  = v & 31;

    range = (v >> 23) & 7;
    st->coarse = range == 1 ? 0 : range == 2 ? 1 : range == 4 ? 2 : -1;

    if (!range) st->flags |= MIRISDR_TUNER_SYNTH_OFF;
    if (range == 1 && st->fine == 31 && st->unknown == 22) st->flags |= MIRISDR_TUNER_AT_LOW_LIMIT;
    if (range == 4 && st->fine == 0) st->flags |= MIRISDR_TUNER_AT_HIGH_LIMIT;

    return 0;

failed:
    return -1;
}

/*
 * Overrides, data bits:
 *
 *   register 14   0      hold the VCO
 *                 5:1    fine
 *                 6      hold the up-converter code
 *                 10:7   up-converter code
 *                 11     hold unknown
 *                 16:12  unknown
 *                 18:17  range: 01 low, 10 middle, 11 high
 *   register 13   0      hold the IF filter code (the library's BW_MAX holds 0)
 *                 5:1    IF filter code, 0 the widest
 *                 6      hold the L-band LNA code
 *                 10:7   L-band LNA code
 */

int mirisdr_set_tuner_override (mirisdr_dev_t *p, const mirisdr_tuner_override_t *ov)
{
    uint32_t r13 = 0, r14 = 0;

    if (!p) goto failed;
    if (p->external_tuner) goto failed;

    if (ov)
    {
        if (ov->coarse > 2 || ov->fine > 31 || ov->upconv > 15 || ov->lna_cal > 15 || ov->filter > 31 ||
            ov->unknown > 31) goto failed;

        if (ov->hold_vco)    r14 |= 1 | (uint32_t) ov->fine << 1 | (uint32_t) (ov->coarse + 1) << 17;
        if (ov->hold_upconv) r14 |= 1 << 6 | (uint32_t) ov->upconv << 7;
        if (ov->hold_unknown) r14 |= 1 << 11 | (uint32_t) ov->unknown << 12;
        if (ov->hold_lna)    r13 |= 1 << 6 | (uint32_t) ov->lna_cal << 7;
        /* ORed with the BW_MAX bit, which is the same hold: a held code wins */
        if (ov->hold_filter) r13 |= 1 | (uint32_t) ov->filter << 1;
    }

    p->tuner_ovr13 = r13;
    p->tuner_ovr14 = r14;

    /* before the first tune they go out with it */
    if (!(p->tuner_valid & 1)) return 0;

    if (mirisdr_tuner_write(p, 14, p->tuner_ovr14, 0) < 0) goto failed;
    if (mirisdr_tuner_write(p, 13, p->tuner_regd | p->tuner_ovr13, 0) < 0) goto failed;

    return 0;

failed:
    return -1;
}
