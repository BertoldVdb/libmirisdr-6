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

#include "soft.h"
#include "sdrplay.c"

//float band_limits[] = {
//        0.,     12.,    30.,    50.,    108.,   250.,   390.,   960.,   2400,   -1.
//};
//
//uint32_t band_select[] = {
//        0xf780, 0xff80, 0xf280, 0xf380, 0xfa80, 0xf680, 0xf380, 0xfa80, 0x0000, 0x0000
//};
//GPIO0 - DAB notch
//GPIO2 - Broadcast FM notch

hw_switch_freq_plan_t hw_switch_freq_plan_default[] = {
        {0,    MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf780, 0, 0},
        {12,   MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xff80, 0, 0},
        {30,   MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf280, 0, 0},
        {55,   MIRISDR_MODE_VHF, 0, 0, 32, 0xf380, 0, 0},
        {108,  MIRISDR_MODE_B3,  0, 0, 16, 0xfa80, 0, 0},
        {250,  MIRISDR_MODE_B3,  0, 0, 16, 0xf680, 0, 0},
        {255,  MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf680, 6, 0},
        {280,  MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf680, 7, 0},
        {420,  MIRISDR_MODE_B45, 0, 0, 4,  0xf380, 0, 0},
        {960,  MIRISDR_MODE_BL,  0, 0, 2,  0xfa80, 0, 0},
        {2400, -1, 0, 0, 0, 0x0000, 0, 0},
};

hw_switch_freq_plan_t hw_switch_freq_plan_sdrplay[] = {
        {0,    MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf580, 0, 0},
        {12,   MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf580, 0, 0},
        {30,   MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf580, 0, 0},
        {50,   MIRISDR_MODE_VHF, 0, 0, 32, 0xf180, 0, 0},
        {112,  MIRISDR_MODE_B3,  0, 0, 16, 0xf580, 0, 0},
        {250,  MIRISDR_MODE_B3,  0, 0, 16, 0xf480, 0, 0},
        {261,  6              ,  0, 0, 8,  0xf480, 0, 0},
        {404,  MIRISDR_MODE_B45, 0, 0, 4,  0xf580, 0, 0},
        {1000, MIRISDR_MODE_BL,  0, 0, 2,  0xf580, 0, 0},
        {2400, -1, 0, 0, 0, 0x0000, 0, 0},
};

/* RSP1B: the default plan's tuner modes, register 8 holding GPIO_1 and GPIO_3 high with
   GPIO_0 (MISO) an input, and the front end on the expander. A5 low costs 35-40 dB below
   3 MHz and gains 13-16 dB at 40 MHz, so HF keeps it released up to 30 MHz. */
#define RSP1B_VHF_B45   0xBFDF  /* B6 A5 low: the VHF and B45 inputs' path */
#define RSP1B_B3_L      0x7FDF  /* B7 A5 low: the B3 and L inputs' path */
#define RSP1B_LPF2      0x3FFF  /* B7 B6 low: the 2 MHz low pass */
#define RSP1B_2_12      0x3CFF  /* and B1 B0 low */
#define RSP1B_12_30     0x3EFF  /* and B0 low */
#define RSP1B_30_60     0x3D5F  /* B7 B6 A7 B1 low, A5 low */
#define RSP1B_250_300   0x3F5F  /* B7 B6 A7 low, A5 low */
#define RSP1B_300_380   0x3C5F  /* and B1 B0 low */
#define RSP1B_380_420   0x3E5F  /* and B0 low */

hw_switch_freq_plan_t hw_switch_freq_plan_rsp1b[] = {
        {0,    MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 0, RSP1B_LPF2},
        {2,    MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 0, RSP1B_2_12},
        {12,   MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 0, RSP1B_12_30},
        {30,   MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 0, RSP1B_30_60},
        {55,   MIRISDR_MODE_VHF, 0, 0, 32, 0xea80, 0, RSP1B_VHF_B45},
        {108,  MIRISDR_MODE_B3,  0, 0, 16, 0xea80, 0, RSP1B_B3_L},
        {250,  MIRISDR_MODE_B3,  0, 0, 16, 0xea80, 0, RSP1B_B3_L},
        {255,  MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 6, RSP1B_250_300},
        {280,  MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 7, RSP1B_250_300},
        {300,  MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 7, RSP1B_300_380},
        {380,  MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 7, RSP1B_380_420},
        {420,  MIRISDR_MODE_B45, 0, 0, 4,  0xea80, 0, RSP1B_VHF_B45},
        {960,  MIRISDR_MODE_BL,  0, 0, 2,  0xea80, 0, RSP1B_B3_L},
        {2400, -1, 0, 0, 0, 0x0000, 0, 0},
};

hw_switch_freq_plan_t *hw_switch_freq_plan[3] = {
        hw_switch_freq_plan_default,
        hw_switch_freq_plan_sdrplay,
        hw_switch_freq_plan_rsp1b
};

/* The row a frequency falls in */
static const hw_switch_freq_plan_t *mirisdr_plan_row_at (mirisdr_dev_t *p, uint32_t freq)
{
    const hw_switch_freq_plan_t *plan = hw_switch_freq_plan[(int) p->hw_flavour];
    int i = 0;

    while ((uint64_t) freq >= 1000000ULL * plan[i].low_cut && plan[i].mode >= 0) i++;

    return &plan[i - 1];
}

static const hw_switch_freq_plan_t *mirisdr_plan_row (mirisdr_dev_t *p)
{
    return mirisdr_plan_row_at(p, p->freq);
}

/* Where the plan ends: tuning past it is refused by mirisdr_tune() */
static uint32_t mirisdr_plan_end (mirisdr_dev_t *p)
{
    const hw_switch_freq_plan_t *plan = hw_switch_freq_plan[(int) p->hw_flavour];
    int i = 0;

    while (plan[i].mode >= 0) i++;

    return plan[i].low_cut * 1000000U;
}

/* The tuner's row follows the LO. The front end's filter follows the received
   frequency, which a low IF or an offset moves a little from it, as long as both rows
   use the same tuner input: otherwise the path to that input wins. */
static uint16_t mirisdr_frontend_word (mirisdr_dev_t *p, const hw_switch_freq_plan_t *row, uint32_t rx)
{
    const hw_switch_freq_plan_t *r = mirisdr_plan_row_at(p, rx);

    if ((r->mode == row->mode) && (r->am_port == row->am_port) &&
        (r->upconvert_mixer_on == row->upconvert_mixer_on)) return r->expander;

    return row->expander;
}

/* What one tune writes, worked out without touching the device */
typedef struct mirisdr_tune_words
{
    uint32_t reg0, reg2, reg3, reg5, regd;
    uint32_t lo_real;           /* Hz the LO reaches, as the received frequency */
    mirisdr_band_t band;
    uint8_t dc_n, gap;
    uint16_t expander;
    const hw_switch_freq_plan_t *row;
} mirisdr_tune_words_t;

static void mirisdr_tune_words (mirisdr_dev_t *p, uint32_t lo, uint32_t rx, int ifm, int bw, int iq,
                                int cal, uint32_t fixed, mirisdr_tune_words_t *w)
{
    uint32_t reg0 = 0, reg2 = 0, reg5 = 0, reg3 = 0, regd = 0, xsel;
    uint64_t n, thresh, frac, lo_div = 0, fvco = 0, rfvco = 0, offset = 0, afc = 0, a, b, c, flo, synth;
    const hw_switch_freq_plan_t *row = mirisdr_plan_row_at(p, lo);

    w->row = row;
    w->expander = mirisdr_frontend_word(p, row, rx);

    if (row->mode == MIRISDR_MODE_AM)
    {
        reg0 |= MIRISDR_MODE_AM;
        reg0 |= row->upconvert_mixer_on << 5;
        reg0 |= row->am_port << 7;

        /* 255-420 MHz goes low side instead (below), high side puts the VCO far out of range */
        if (row->upconvert_mixer_on && !row->if1_low) offset += 120000000UL;

        lo_div = 16;
        w->band = (row->am_port == 0) ? MIRISDR_BAND_AM1 : MIRISDR_BAND_AM2;
    }
    else
    {
        reg0 |= row->mode;
        lo_div = row->lo_div;

        if (row->mode == MIRISDR_MODE_VHF) w->band = MIRISDR_BAND_VHF;
        else if (row->mode == MIRISDR_MODE_B3) w->band = MIRISDR_BAND_3;
        else if (row->mode == MIRISDR_MODE_B45) w->band = MIRISDR_BAND_45;
        else w->band = MIRISDR_BAND_L;
    }

    /* the RF synthesizer is always on */
    reg0 |= MIRISDR_RF_SYNTHESIZER_ON << 6;

    switch (ifm)
    {
    case MIRISDR_IF_ZERO:    reg0 |= MIRISDR_IF_MODE_ZERO << 8; break;
    case MIRISDR_IF_450KHZ:  reg0 |= MIRISDR_IF_MODE_450KHZ << 8; break;
    case MIRISDR_IF_1620KHZ: reg0 |= MIRISDR_IF_MODE_1620KHZ << 8; break;
    case MIRISDR_IF_2048KHZ: reg0 |= MIRISDR_IF_MODE_2048KHZ << 8; break;
    }

    /* the IF filter: the widest code with its filter bypassed (register 13) is BW_MAX */
    reg0 |= (uint32_t) ((bw == MIRISDR_BW_MAX) ? 7 : bw) << 10;
    if (bw == MIRISDR_BW_MAX) regd |= 1;

    /* the gap rows' first IF follows XTALSEL instead: 001 x6, 000 x7 */
    xsel = row->if1_low ? (uint32_t) (7 - row->if1_low) : mirisdr_xtalsel(p);
    reg0 |= xsel << 13;
    w->dc_n = mirisdr_xtalsel_n[xsel];

    /* which recalibrates the IF filter for that crystal, see mirisdr_reg13 */
    w->gap = row->if1_low != 0;

    /* the baseband amplifiers of the output not wanted can be switched off */
    reg0 |= (uint32_t) ((iq == MIRISDR_IQ_ONLY_I) ? MIRISDR_IF_LPMODE_ONLY_I :
                        (iq == MIRISDR_IQ_ONLY_Q) ? MIRISDR_IF_LPMODE_ONLY_Q : MIRISDR_IF_LPMODE_NORMAL) << 16;
    reg0 |= MIRISDR_VCO_LPMODE_NORMAL << 19;

    flo = lo + offset - (uint64_t) row->if1_low * 24000000UL;
    fvco = flo * lo_div;

    /* N, then the fraction over a threshold reduced to fit 12 bits */
    n = fvco / 96000000UL;

    if (fixed)
    {
        /* the nearest point of the grid, no AFC: only register 2 differs between them */
        thresh = fixed;
        frac = ((fvco % 96000000UL) * thresh + 48000000UL) / 96000000UL;
        if (frac == thresh)
        {
            n++;
            frac = 0;
        }
    }
    else
    {
        thresh = 96000000UL / lo_div;
        frac = (fvco % 96000000UL) / lo_div;

        for (a = thresh, b = frac; a != 0;)
        {
            c = a;
            a = b % a;
            b = c;
        }

        thresh /= b;
        frac /= b;

        a = (thresh + 4094) / 4095;
        thresh = (thresh + (a / 2)) / a;
        frac = (frac + (a / 2)) / a;

        rfvco = (96000000UL * (n * thresh * 4096UL + (frac * 4096UL))) / (thresh * 4096UL * lo_div);
        if (flo < rfvco) frac--;
        rfvco = (96000000UL * (n * thresh * 4096UL + (frac * 4096UL + afc))) / (thresh * 4096UL * lo_div);
        afc = ((flo - rfvco) * thresh * 4096UL * lo_div) / 96000000UL;
    }

    reg3 |= (afc & 4095);
    reg5 |= (0xFFF & thresh);
    reg5 |= MIRISDR_RF_SYNTHESIZER_RESERVED_PROGRAMMING << 12;   /* reserved, must be 0x28 */

    reg2 |= (0xFFF & frac);
    reg2 |= (0x3F & n) << 12;
    reg2 |= (uint32_t) (cal ? MIRISDR_LBAND_LNA_CALIBRATION_ON : MIRISDR_LBAND_LNA_CALIBRATION_OFF) << 18;

    /* the synthesizer as programmed, back in terms of the received frequency */
    synth = (96000000ULL * (n * thresh * 4096ULL + frac * 4096ULL + (afc & 4095)) +
             thresh * 2048ULL * lo_div) / (thresh * 4096ULL * lo_div);
    w->lo_real = (uint32_t) (synth - offset + (uint64_t) row->if1_low * 24000000UL);

    w->reg0 = reg0;
    w->reg2 = reg2;
    w->reg3 = reg3;
    w->reg5 = reg5;
    w->regd = regd;

#if MIRISDR_DEBUG >= 1
    fprintf(stderr, "mirisdr tune: lo %u rx %u sel %x, n %lu, fraction %lu/%lu, LO %u\n", lo, rx,
            row->band_select_word, (long unsigned) n, (long unsigned) frac, (long unsigned) thresh, w->lo_real);
#endif
}

static int mirisdr_tune_send (mirisdr_dev_t *p, const mirisdr_tune_words_t *w)
{
    if (p->hw_flavour == MIRISDR_HW_RSP1B) mirisdr_rsp1b_frontend(p, w->expander);

    p->reg8 = w->row->band_select_word;
    update_reg_8(p);

    p->band = w->band;
    p->dc_n = w->dc_n;
    p->tuner_gap = w->gap;

    /* register 9 is the MSi001's serial port: a board with another tuner on it
       has this port wired somewhere else, or nowhere */
    if (!p->external_tuner)
    {
        int synth = 0;

        /* only what changed; register 2 starts the calibration, so it follows any of 0, 3 and 5 */
        mirisdr_tuner_write(p, 14, p->tuner_ovr14, 0);
        mirisdr_tuner_write(p, 6, mirisdr_dc_word(p), 0);
        synth |= mirisdr_tuner_write(p, 3, w->reg3, 0) != 0;
        synth |= mirisdr_tuner_write(p, 0, w->reg0, 0) != 0;
        synth |= mirisdr_tuner_write(p, 5, w->reg5, 0) != 0;
        mirisdr_tuner_write(p, 2, w->reg2, synth);
        p->tuner_regd = w->regd;
        mirisdr_tuner_write(p, 13, mirisdr_reg13(p), 0);
    }

    return 0;
}

/* The gap rows name another crystal for their first IF, and the IF filter
   calibrates against the named one.  Read the code the real setting gave once,
   on the way in from a normal band (open tunes one), to hold it there. */
static void mirisdr_filter_learn (mirisdr_dev_t *p, const hw_switch_freq_plan_t *row)
{
    mirisdr_tuner_status_t st;

    if (p->filter_cal >= 0 || p->external_tuner || !row->if1_low || !(p->tuner_valid & 1)) return;
    if ((p->tuner_reg[0] & (7UL << 17)) != mirisdr_xtalsel(p) << 17) return;

    if (mirisdr_get_tuner_status(p, 0, &st) == 0) p->filter_cal = st.filter;
}

/* Datasheet table 19: the synthesizer frequency that calibrates the L-band LNA to centre */
#define MIRISDR_LNA_CAL_US      1000
#define MIRISDR_LNA_CAL_MIN     1000    /* MHz: lower lands on code 15 as well, outside L band */

static const uint16_t mirisdr_lna_cal_vco[] = { 2380, 2400, 2420, 2440, 2480, 2500, 2520, 2540, 2560, 2580 };
static const uint16_t mirisdr_lna_cal_centre[] = { 1440, 1460, 1500, 1530, 1570, 1600, 1640, 1670, 1700, 1750 };

/* The tune that calibrates the LNA for this frequency, MHz; beyond the table the end
   segments carry on, which still lands on the end codes (15 below ~1350, 0 above ~1800) */
static int mirisdr_lna_cal_tune (int mhz)
{
    int n = sizeof mirisdr_lna_cal_vco / sizeof mirisdr_lna_cal_vco[0], i = 1, f;

    while (i < n - 1 && mhz > mirisdr_lna_cal_centre[i]) i++;

    f = (mirisdr_lna_cal_vco[i - 1] + (mhz - mirisdr_lna_cal_centre[i - 1]) *
         (mirisdr_lna_cal_vco[i] - mirisdr_lna_cal_vco[i - 1]) /
         (mirisdr_lna_cal_centre[i] - mirisdr_lna_cal_centre[i - 1])) / 2;

    return (f < MIRISDR_LNA_CAL_MIN) ? MIRISDR_LNA_CAL_MIN : f;
}

/* centred on the frequency received */
static int mirisdr_lna_calibrate (mirisdr_dev_t *p, const hw_switch_freq_plan_t *row, uint32_t rx,
                                  int ifm, int bw, int iq)
{
    int mhz = (int) (rx / 1000000), r;
    uint32_t cal;
    mirisdr_tune_words_t w;

    if (p->external_tuner || row->mode != MIRISDR_MODE_BL || (p->tuner_ovr13 & 0x40)) return 0;
    if ((p->tuner_valid & 1) && p->lna_cal_mhz && abs(mhz - p->lna_cal_mhz) <= 50) return 0;

    cal = (uint32_t) mirisdr_lna_cal_tune(mhz) * 1000000;
    mirisdr_tune_words(p, cal, cal, ifm, bw, iq, 1, 0, &w);
    r = mirisdr_tune_send(p, &w);

    /* it needs the synthesizer settled, which can take ~400 us; 20 us sufficed in tests */
    if (p->fw_ours && p->fw_list_at) r |= mirisdr_write_reg(p, MIRISDR_LIST_WAIT_US, MIRISDR_LNA_CAL_US);
    else { r |= mirisdr_batch_flush(p); usleep(MIRISDR_LNA_CAL_US); }

    if (r >= 0) p->lna_cal_mhz = mhz;

    return r < 0 ? r : 0;
}

/* ------------------------------------------------------------------ */
/* the tune config                                                     */
/* ------------------------------------------------------------------ */

#define MIRISDR_TUNE_ADJUST     1       /* the single setters: no combination checks */

static const uint32_t mirisdr_bw_hz[] = { 200000, 300000, 600000, 1536000, 5000000, 6000000,
                                          7000000, 8000000, 14000000 };

static int mirisdr_if_mode (uint32_t hz)
{
    switch (hz)
    {
    case 0:       return MIRISDR_IF_ZERO;
    case 450000:  return MIRISDR_IF_450KHZ;
    case 1620000: return MIRISDR_IF_1620KHZ;
    case 2048000: return MIRISDR_IF_2048KHZ;
    }

    return -1;
}

static const uint32_t mirisdr_if_hz[] = { 0, 450000, 1620000, 2048000 };

/* which filters each IF allows, by bit MIRISDR_BW_*: table 12 notes 3 and table 13 */
static const uint16_t mirisdr_if_bws[] = {
    (1 << MIRISDR_BW_1536KHZ) | (1 << MIRISDR_BW_5MHZ) | (1 << MIRISDR_BW_6MHZ) |
    (1 << MIRISDR_BW_7MHZ) | (1 << MIRISDR_BW_8MHZ) | (1 << MIRISDR_BW_MAX),
    (1 << MIRISDR_BW_200KHZ) | (1 << MIRISDR_BW_300KHZ) | (1 << MIRISDR_BW_600KHZ),
    (1 << MIRISDR_BW_600KHZ) | (1 << MIRISDR_BW_1536KHZ),
    (1 << MIRISDR_BW_600KHZ) | (1 << MIRISDR_BW_1536KHZ),
};
static const uint8_t mirisdr_if_bw_widest[] = { MIRISDR_BW_8MHZ, MIRISDR_BW_600KHZ,
                                                MIRISDR_BW_1536KHZ, MIRISDR_BW_1536KHZ };

typedef struct mirisdr_tune_plan
{
    mirisdr_tune_config_t cfg;  /* as asked, the bandwidth filled in */
    int ifm, bw, iq;
    uint32_t lo, ovr13, ovr14;
    mirisdr_tune_words_t w;
    mirisdr_gr_t gr;            /* the gain reductions it ends up with */
} mirisdr_tune_plan_t;

static int mirisdr_tune_plan (mirisdr_dev_t *p, const mirisdr_tune_config_t *c, int flags,
                              mirisdr_tune_plan_t *pl)
{
    int adjust = flags & MIRISDR_TUNE_ADJUST, i;
    int64_t lo;

    pl->cfg = *c;

    if ((pl->ifm = mirisdr_if_mode(c->if_freq)) < 0)
    {
        fprintf(stderr, "unsupported if frequency: %u Hz\n", c->if_freq);
        return -1;
    }

    if (!c->bandwidth) pl->bw = mirisdr_if_bw_widest[pl->ifm];
    else
    {
        for (i = 0; (i < (int) (sizeof mirisdr_bw_hz / sizeof mirisdr_bw_hz[0])) &&
                    (mirisdr_bw_hz[i] != c->bandwidth); i++);

        if (i == (int) (sizeof mirisdr_bw_hz / sizeof mirisdr_bw_hz[0]))
        {
            fprintf(stderr, "unsupported bandwidth: %u Hz\n", c->bandwidth);
            return -1;
        }

        pl->bw = i;
    }

    pl->cfg.bandwidth = mirisdr_bw_hz[pl->bw];

    if (!adjust && !(mirisdr_if_bws[pl->ifm] & (1 << pl->bw)))
    {
        fprintf(stderr, "a %u Hz bandwidth is not available with a %u Hz IF\n", pl->cfg.bandwidth, c->if_freq);
        return -1;
    }

    if ((c->iq < MIRISDR_IQ_BOTH) || (c->iq > MIRISDR_IQ_ONLY_Q))
    {
        fprintf(stderr, "unsupported iq selection: %d\n", c->iq);
        return -1;
    }

    pl->iq = c->iq;

    /* one output alone cannot tell the two sides of the LO apart */
    if (pl->iq != MIRISDR_IQ_BOTH && pl->ifm == MIRISDR_IF_ZERO)
    {
        if (!adjust)
        {
            fprintf(stderr, "a single tuner output needs a low IF\n");
            return -1;
        }

        pl->iq = MIRISDR_IQ_BOTH;
    }

    /* the stream capturing the output switched off would go dead, unless it follows */
    if (!adjust && !p->stream.follow_tune && (pl->iq != MIRISDR_IQ_BOTH) &&
        (mirisdr_stream_adc(p) != MIRISDR_IQ_BOTH) && (mirisdr_stream_adc(p) != pl->iq))
    {
        fprintf(stderr, "the stream captures the tuner output this tune switches off\n");
        return -1;
    }

    if (c->synth_thresh > 4095)
    {
        fprintf(stderr, "unsupported synthesizer threshold: %u\n", c->synth_thresh);
        return -1;
    }

    if (mirisdr_override_words(&c->override, &pl->ovr13, &pl->ovr14) < 0)
    {
        fprintf(stderr, "tuner override code out of range\n");
        return -1;
    }

    if ((c->gain.mode < MIRISDR_GAIN_KEEP) || (c->gain.mode > MIRISDR_GAIN_STAGES) ||
        ((c->gain.mode == MIRISDR_GAIN_TOTAL) && (c->gain.total < 0)))
    {
        fprintf(stderr, "unsupported gain: mode %d, %d dB\n", c->gain.mode, c->gain.total);
        return -1;
    }

    lo = (int64_t) c->frequency + c->lo_offset;
    if (c->low_if_auto) lo += mirisdr_if_hz[pl->ifm];

    if ((lo < 0) || (lo > (int64_t) UINT32_MAX) || (!adjust && (lo >= mirisdr_plan_end(p))))
    {
        fprintf(stderr, "cannot tune the LO to %lld Hz\n", (long long) lo);
        return -1;
    }

    pl->lo = (uint32_t) lo;

    mirisdr_tune_words(p, pl->lo, c->frequency, pl->ifm, pl->bw, pl->iq, 0, c->synth_thresh, &pl->w);

    /* the gain in that band */
    if (c->gain.mode == MIRISDR_GAIN_STAGES)
    {
        if (mirisdr_gr_of_stages(pl->w.band, &c->gain, &pl->gr) < 0)
        {
            fprintf(stderr, "unsupported gain stages for the band: lna %d mixer %d mixbuffer %d baseband %d\n",
                    c->gain.lna, c->gain.mixer, c->gain.mixbuffer, c->gain.baseband);
            return -1;
        }
    }
    else if (c->gain.mode == MIRISDR_GAIN_TOTAL) mirisdr_split_of(pl->w.band, c->gain.total, &pl->gr);
    else if (p->gain_stages_set) mirisdr_gr_get(p, &pl->gr);
    else mirisdr_split_of(pl->w.band, p->gain, &pl->gr);

    return 0;
}

/* Where the received frequency lands, given what the stream captures */
static void mirisdr_tune_result_of (int adc, uint32_t rx, uint32_t lo, int iq, uint32_t bw,
                                    mirisdr_band_t band, const mirisdr_gr_t *gr, mirisdr_tune_result_t *res)
{
    int64_t off = (int64_t) rx - lo;
    int real = (iq != MIRISDR_IQ_BOTH) || (adc != MIRISDR_IQ_BOTH);

    res->lo = lo;
    res->iq = iq;
    res->bandwidth = bw;
    res->band = band;
    mirisdr_stages_of(band, gr, &res->gain);

    /* a real signal shows only the size of the offset, mirrored when below the LO */
    res->offset = (int32_t) (real && off < 0 ? -off : off);
    res->inverted = real && off < 0;
}

static int mirisdr_tune_adc (mirisdr_dev_t *p, int iq)
{
    return p->stream.follow_tune ? iq : mirisdr_stream_adc(p);
}

static int mirisdr_tune_apply (mirisdr_dev_t *p, const mirisdr_tune_config_t *c, int flags,
                               mirisdr_tune_result_t *res)
{
    mirisdr_tune_plan_t pl;
    mirisdr_stream_plan_t spl;
    int r, restream = 0, streaming = 0;

    if (!p || !c) return -1;
    if (mirisdr_tune_plan(p, c, flags, &pl) < 0) return -1;

    /* a stream following the tune changes kind or converter with it, in one restart */
    if (p->stream.follow_tune)
    {
        if (mirisdr_stream_plan(p, &p->stream, flags & MIRISDR_TUNE_ADJUST ? MIRISDR_STREAM_ADJUST : 0,
                                pl.iq, &spl) < 0) return -1;

        restream = (spl.format != (int) p->format) || (spl.swap != p->swap_iq);
        if (restream && ((streaming = mirisdr_stream_pause(p)) < 0)) return -1;
    }

    mirisdr_filter_learn(p, pl.w.row);
    mirisdr_batch_begin(p);

    /* the holds go out in the same list, and a held LNA code skips its calibration */
    p->tuner_ovr13 = pl.ovr13;
    p->tuner_ovr14 = pl.ovr14;

    r = mirisdr_lna_calibrate(p, pl.w.row, c->frequency, pl.ifm, pl.bw, pl.iq);
    r += mirisdr_tune_send(p, &pl.w);

    p->freq = pl.lo;
    p->if_freq = pl.ifm;
    p->bandwidth = pl.bw;
    p->tune = pl.cfg;
    p->tune.iq = pl.iq;
    p->tune.gain.mode = MIRISDR_GAIN_KEEP;  /* the gain lives on in the stages, which a tune replaces */
    p->tune_lo = pl.w.lo_real;
    p->tune_iq = pl.iq;

    if (c->gain.mode == MIRISDR_GAIN_TOTAL)
    {
        p->gain = c->gain.total;
        p->gain_stages_set = 0;
    }
    else if (c->gain.mode == MIRISDR_GAIN_STAGES)
    {
        mirisdr_gr_set(p, &pl.gr);
        if (p->gain < 0) p->gain = DEFAULT_GAIN;
        p->gain_stages_set = 1;
    }

    mirisdr_gain_retune(p);
    r += mirisdr_set_gain(p);
    r += mirisdr_batch_end(p);

    if (restream)
    {
        mirisdr_stream_regs(p, &spl);
        if (streaming && (mirisdr_start_async(p) < 0)) r = -1;
    }

    if (res)
    {
        mirisdr_gr_t gr;

        mirisdr_gr_get(p, &gr);
        mirisdr_tune_result_of(mirisdr_stream_adc(p), c->frequency, p->tune_lo, p->tune_iq,
                               pl.cfg.bandwidth, p->band, &gr, res);
    }

    return r;
}

/* The tuner side of a fresh device, the null one included */
static void mirisdr_tuner_defaults (mirisdr_dev_t *p)
{
    mirisdr_tune_config_default(&p->tune);
    p->freq = DEFAULT_FREQ;
    p->gain = DEFAULT_GAIN;
    p->band = MIRISDR_BAND_VHF; // matches always the default frequency of 90 MHz

    p->gain_reduction_lna = 0;
    p->gain_reduction_mixer = 0;
    p->gain_reduction_baseband = 43;
    p->if_freq = MIRISDR_IF_ZERO;
    p->bandwidth = MIRISDR_BW_8MHZ;
    p->xtal = MIRISDR_XTAL_24M;
    p->bias = 0;
    p->dc_mode = MIRISDR_DC_PERIODIC2;
    p->dc_speedup = 0;
    p->dc_track = 0x1f;
    p->dc_period = 0x800;
    p->filter_cal = -1;
}

/* the stored tune again, for what changes the front end or the stream's view of it */
int mirisdr_set_soft(mirisdr_dev_t *p)
{
    return mirisdr_tune_apply(p, &p->tune, MIRISDR_TUNE_ADJUST, NULL);
}

void mirisdr_tune_config_default (mirisdr_tune_config_t *cfg)
{
    if (!cfg) return;

    memset(cfg, 0, sizeof *cfg);
    cfg->frequency = DEFAULT_FREQ;
    cfg->iq = MIRISDR_IQ_BOTH;
    cfg->gain.mode = MIRISDR_GAIN_KEEP;
}

int mirisdr_tune (mirisdr_dev_t *p, int tuner, const mirisdr_tune_config_t *cfg, mirisdr_tune_result_t *res)
{
    if ((tuner < 0) || (tuner >= MIRISDR_TUNERS)) return -1;

    return mirisdr_tune_apply(p, cfg, 0, res);
}

int mirisdr_tune_check (mirisdr_dev_t *p, int tuner, const mirisdr_tune_config_t *cfg, mirisdr_tune_result_t *res)
{
    mirisdr_tune_plan_t pl;

    if (!p || !cfg || (tuner < 0) || (tuner >= MIRISDR_TUNERS)) return -1;
    if (mirisdr_tune_plan(p, cfg, 0, &pl) < 0) return -1;

    if (res) mirisdr_tune_result_of(mirisdr_tune_adc(p, pl.iq), cfg->frequency, pl.w.lo_real, pl.iq,
                                    pl.cfg.bandwidth, pl.w.band, &pl.gr, res);

    return 0;
}

int mirisdr_get_tune (mirisdr_dev_t *p, int tuner, mirisdr_tune_config_t *cfg, mirisdr_tune_result_t *res)
{
    mirisdr_gr_t gr;

    if (!p || (tuner < 0) || (tuner >= MIRISDR_TUNERS)) return -1;

    mirisdr_gr_get(p, &gr);
    if (cfg) *cfg = p->tune;
    if (res) mirisdr_tune_result_of(mirisdr_stream_adc(p), p->tune.frequency, p->tune_lo, p->tune_iq,
                                    p->tune.bandwidth, p->band, &gr, res);

    return 0;
}

/* ------------------------------------------------------------------ */
/* the single setters, on top of it                                    */
/* ------------------------------------------------------------------ */

int mirisdr_set_tuner_override (mirisdr_dev_t *p, int tuner, const mirisdr_tuner_override_t *ov)
{
    mirisdr_tune_config_t c;

    if (!p || (tuner < 0) || (tuner >= MIRISDR_TUNERS) || p->external_tuner) return -1;

    c = p->tune;
    if (ov) c.override = *ov;
    else memset(&c.override, 0, sizeof c.override);

    return mirisdr_tune_apply(p, &c, MIRISDR_TUNE_ADJUST, NULL);
}

int mirisdr_set_center_freq(mirisdr_dev_t *p, uint32_t freq)
{
    mirisdr_tune_config_t c;

    if (!p) return -1;

    /* always the LO, whatever a tune set before */
    c = p->tune;
    c.frequency = freq;
    c.low_if_auto = 0;
    c.lo_offset = 0;

    return mirisdr_tune_apply(p, &c, MIRISDR_TUNE_ADJUST, NULL);
}

uint32_t mirisdr_get_center_freq(mirisdr_dev_t *p)
{
    return p->freq;
}

int mirisdr_set_if_freq(mirisdr_dev_t *p, uint32_t freq)
{
    mirisdr_tune_config_t c;

#if MIRISDR_DEBUG >= 1
    fprintf(stderr, "mirisdr_set_if_freq: %u Hz\n", freq);
#endif
    if (!p) return -1;

    c = p->tune;
    c.if_freq = freq;

    return mirisdr_tune_apply(p, &c, MIRISDR_TUNE_ADJUST, NULL);
}

uint32_t mirisdr_get_if_freq(mirisdr_dev_t *p)
{
    if (!p) return -1;

    return mirisdr_if_hz[p->if_freq];
}

/* not supported yet */
int mirisdr_set_xtal_freq(mirisdr_dev_t *p, uint32_t freq)
{
    (void) p;
    (void) freq;
    return -1;
}

uint32_t mirisdr_get_xtal_freq(mirisdr_dev_t *p)
{
    if (!p)
        goto failed;

    switch (p->xtal)
    {
    case MIRISDR_XTAL_19_2M:
        return 19200000;
    case MIRISDR_XTAL_22M:
        return 22000000;
    case MIRISDR_XTAL_24M:
    case MIRISDR_XTAL_24_576M:
        /* realně 24 MHz ??? */
        return 24000000;
    case MIRISDR_XTAL_26M:
        return 26000000;
    case MIRISDR_XTAL_38_4M:
        return 38400000;
    }

    failed: return -1;
}

int mirisdr_set_freq_correction(mirisdr_dev_t *p, int ppm)
{
    (void) p;
    (void) ppm;
    fprintf(stderr, "frequency correction not implemented yet\n");
    return -1;
}

int mirisdr_set_direct_sampling(mirisdr_dev_t *p, int on)
{
    (void) p;
    (void) on;
    fprintf(stderr, "direct sampling not implemented yet\n");
    return -1;
}

/* As before: the nearest filter at or above the request, and a narrow one moves the IF */
int mirisdr_set_bandwidth(mirisdr_dev_t *p, uint32_t bw)
{
    mirisdr_tune_config_t c;
    int b = MIRISDR_BW_MAX;

#if MIRISDR_DEBUG >= 1
    fprintf(stderr, "mirisdr_set_bandwidth: %u Hz\n", bw);
#endif
    if (!p) return -1;

    c = p->tune;

    if (bw <= 8000000) b = MIRISDR_BW_8MHZ;
    if (bw <= 7000000) b = MIRISDR_BW_7MHZ;
    if (bw <= 6000000) b = MIRISDR_BW_6MHZ;
    if (bw <= 5000000) b = MIRISDR_BW_5MHZ;
    if (bw <= 1536000)
    {
        b = MIRISDR_BW_1536KHZ;
        if (p->rate >= 5000000) c.if_freq = 1620000;
    }
    if (bw <= 600000) { b = MIRISDR_BW_600KHZ; c.if_freq = 450000; }
    if (bw <= 300000) b = MIRISDR_BW_300KHZ;
    if (bw <= 200000) b = MIRISDR_BW_200KHZ;

    c.bandwidth = mirisdr_bw_hz[b];

    return mirisdr_tune_apply(p, &c, MIRISDR_TUNE_ADJUST, NULL);
}

uint32_t mirisdr_get_bandwidth(mirisdr_dev_t *p)
{
    if (!p) return -1;

    return mirisdr_bw_hz[p->bandwidth];
}

int mirisdr_set_offset_tuning(mirisdr_dev_t *p, int on)
{
    mirisdr_tune_config_t c;

#if MIRISDR_DEBUG >= 1
    fprintf(stderr, "mirisdr_set_offset_tuning: %d\n", on);
#endif
    if (!p) return -1;

    c = p->tune;
    c.if_freq = on ? 450000 : 0;

    return mirisdr_tune_apply(p, &c, MIRISDR_TUNE_ADJUST, NULL);
}

int mirisdr_set_transfer(mirisdr_dev_t *p, const char *v)
{
    mirisdr_stream_config_t c;

    if (!p || !v) return -1;

    c = p->stream;
    c.transfer = v;

    /* the endpoint burst size follows it */
    return mirisdr_stream_apply(p, &c, MIRISDR_STREAM_ADJUST | MIRISDR_STREAM_FORCE, NULL);
}

const char *mirisdr_get_transfer(mirisdr_dev_t *p)
{
    switch (p->transfer)
    {
    case MIRISDR_TRANSFER_BULK:
        return "BULK";
    case MIRISDR_TRANSFER_ISOC:
        switch (p->alt_setting)
        {
        case 2:  return "ISOC1";
        case 4:  return "ISOC2";
        default: return "ISOC";
        }
    }

    return "";
}

mirisdr_band_t mirisdr_get_band (mirisdr_dev_t *p)
{
    return p->band;
}

int mirisdr_set_bias (mirisdr_dev_t *p, int bias)
{
	p->bias=bias;

	/* the RSP1B's is on its expander, which the band plan writes */
	if (p->hw_flavour == MIRISDR_HW_RSP1B) return mirisdr_set_soft(p);

	update_reg_8(p);
	return 0;
}

int mirisdr_set_notch (mirisdr_dev_t *p, int notches)
{
	if (!p || (p->hw_flavour != MIRISDR_HW_RSP1B)) return -1;
	if (notches & ~(MIRISDR_NOTCH_FM | MIRISDR_NOTCH_DAB)) return -1;

	p->notch = notches;
	return mirisdr_set_soft(p);
}

int mirisdr_get_notch (mirisdr_dev_t *p)
{
	return p ? p->notch : -1;
}

int mirisdr_get_bias (mirisdr_dev_t *p)
{
	return p->bias;
}
