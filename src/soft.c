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
        {259,  MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xf680, 6, 0},
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
#define RSP1B_DIRECT    0xFFDF  /* A5 low */
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
        {55,   MIRISDR_MODE_VHF, 0, 0, 32, 0xea80, 0, RSP1B_DIRECT},
        {108,  MIRISDR_MODE_B3,  0, 0, 16, 0xea80, 0, RSP1B_DIRECT},
        {250,  MIRISDR_MODE_B3,  0, 0, 16, 0xea80, 0, RSP1B_DIRECT},
        {259,  MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 6, RSP1B_250_300},
        {280,  MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 7, RSP1B_250_300},
        {300,  MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 7, RSP1B_300_380},
        {380,  MIRISDR_MODE_AM,  MIRISDR_UPCONVERT_MIXER_ON, MIRISDR_AM_PORT2, 16, 0xea80, 7, RSP1B_380_420},
        {420,  MIRISDR_MODE_B45, 0, 0, 4,  0xea80, 0, RSP1B_DIRECT},
        {960,  MIRISDR_MODE_BL,  0, 0, 2,  0xea80, 0, RSP1B_DIRECT},
        {2400, -1, 0, 0, 0, 0x0000, 0, 0},
};

hw_switch_freq_plan_t *hw_switch_freq_plan[3] = {
        hw_switch_freq_plan_default,
        hw_switch_freq_plan_sdrplay,
        hw_switch_freq_plan_rsp1b
};

static int mirisdr_set_soft_words(mirisdr_dev_t *p);

static const hw_switch_freq_plan_t *mirisdr_plan_row (mirisdr_dev_t *p)
{
    const hw_switch_freq_plan_t *plan = hw_switch_freq_plan[(int) p->hw_flavour];
    int i = 0;

    while (p->freq >= 1000000 * plan[i].low_cut && plan[i].mode >= 0) i++;

    return &plan[i - 1];
}

/* The gap rows name another crystal for their first IF, and the IF filter
   calibrates against the named one.  Read the code the real setting gave once,
   on the way in from a normal band (open tunes one), to hold it there. */
static void mirisdr_filter_learn (mirisdr_dev_t *p)
{
    mirisdr_tuner_status_t st;

    if (p->filter_cal >= 0 || p->external_tuner || !mirisdr_plan_row(p)->if1_low || !(p->tuner_valid & 1)) return;
    if ((p->tuner_reg[0] & (7UL << 17)) != mirisdr_xtalsel(p) << 17) return;

    if (mirisdr_get_tuner_status(p, &st) == 0) p->filter_cal = st.filter;
}

/* Datasheet table 19: the synthesizer frequency that calibrates the L-band LNA to centre */
#define MIRISDR_LNA_CAL_US      1000

static const uint16_t mirisdr_lna_cal_vco[] = { 2380, 2400, 2420, 2440, 2480, 2500, 2520, 2540, 2560, 2580 };
static const uint16_t mirisdr_lna_cal_centre[] = { 1440, 1460, 1500, 1530, 1570, 1600, 1640, 1670, 1700, 1750 };

/* The tune that calibrates the LNA for this frequency, MHz; beyond the table the end
   segments carry on, which still lands on the end codes (15 below ~1350, 0 above ~1800) */
static int mirisdr_lna_cal_tune (int mhz)
{
    int n = sizeof mirisdr_lna_cal_vco / sizeof mirisdr_lna_cal_vco[0], i = 1;

    while (i < n - 1 && mhz > mirisdr_lna_cal_centre[i]) i++;

    return (mirisdr_lna_cal_vco[i - 1] + (mhz - mirisdr_lna_cal_centre[i - 1]) *
            (mirisdr_lna_cal_vco[i] - mirisdr_lna_cal_vco[i - 1]) /
            (mirisdr_lna_cal_centre[i] - mirisdr_lna_cal_centre[i - 1])) / 2;
}

static int mirisdr_lna_calibrate (mirisdr_dev_t *p)
{
    int mhz = (int) (p->freq / 1000000), r;
    uint32_t freq = p->freq;

    if (p->external_tuner || mirisdr_plan_row(p)->mode != MIRISDR_MODE_BL || (p->tuner_ovr13 & 0x40)) return 0;
    if ((p->tuner_valid & 1) && p->lna_cal_mhz && abs(mhz - p->lna_cal_mhz) <= 50) return 0;

    p->freq = (uint32_t) mirisdr_lna_cal_tune(mhz) * 1000000;
    p->lna_cal_run = 1;
    r = mirisdr_set_soft_words(p);
    p->lna_cal_run = 0;
    p->freq = freq;

    /* it needs the synthesizer settled, which can take ~400 us; 20 us sufficed in tests */
    if (p->fw_ours && p->fw_list_at) r |= mirisdr_write_reg(p, MIRISDR_LIST_WAIT_US, MIRISDR_LNA_CAL_US);
    else { r |= mirisdr_batch_flush(p); usleep(MIRISDR_LNA_CAL_US); }

    if (r >= 0) p->lna_cal_mhz = mhz;

    return r < 0 ? r : 0;
}

/* one list request for the whole tune */
int mirisdr_set_soft(mirisdr_dev_t *p)
{
    int r;

    mirisdr_filter_learn(p);
    mirisdr_batch_begin(p);
    r = mirisdr_lna_calibrate(p);
    r += mirisdr_set_soft_words(p);

    return r + mirisdr_batch_end(p);
}

static int mirisdr_set_soft_words(mirisdr_dev_t *p)
{
    uint32_t reg0 = 0, reg2 = 0, reg5 = 0, reg3 = 0, regd = 0;
    uint64_t n, thresh, frac, lo_div = 0, fvco = 0, rfvco = 0, offset = 0, afc = 0, a, b, c, flo;
    uint32_t xsel;

    /*** registr0 - parametry pásma ***/
    /*** registr0 - parameters zone ***/

    /* pásmo */
    /* zone */

    hw_switch_freq_plan_t switch_plan = *mirisdr_plan_row(p);

#if MIRISDR_DEBUG >= 1
    fprintf(stderr, "mirisdr_set_soft: flavour:%d flow:%u mode:%d up:%d port:%d lo:%d\n",
            (int) p->hw_flavour,
            switch_plan.low_cut,
            switch_plan.mode,
            switch_plan.upconvert_mixer_on,
            switch_plan.am_port,
            switch_plan.lo_div);
#endif

    if (switch_plan.mode == MIRISDR_MODE_AM)
    {
        reg0 |= MIRISDR_MODE_AM;
        reg0 |= switch_plan.upconvert_mixer_on << 5;
        reg0 |= switch_plan.am_port << 7;

        /* 259-420 MHz goes low side instead (below), high side puts the VCO far out of range */
        if (switch_plan.upconvert_mixer_on && !switch_plan.if1_low)
        {
            offset += 120000000UL;
        }

        lo_div = 16;

        if (switch_plan.am_port == 0) {
            p->band = MIRISDR_BAND_AM1;
        } else {
            p->band = MIRISDR_BAND_AM2;
        }
    }
    else
    {
        reg0 |= switch_plan.mode;
        lo_div = switch_plan.lo_div;

        if (switch_plan.mode == MIRISDR_MODE_VHF) {
            p->band = MIRISDR_BAND_VHF;
        } else if (switch_plan.mode == MIRISDR_MODE_B3) {
            p->band = MIRISDR_BAND_3;
        } else if (switch_plan.mode == MIRISDR_MODE_B45) {
            p->band = MIRISDR_BAND_45;
        } else if (switch_plan.mode == MIRISDR_MODE_BL) {
            p->band = MIRISDR_BAND_L;
        }
    }

//    if (p->freq < 50000000)
//    {
//        /* AM režim2, antena 2, AM režim1 - 0x61 */
//        /* AM mode2, Antena 2, AM mode1 - 0x61   */
//        reg0 |= MIRISDR_MODE_AM << 4;
//        reg0 |= MIRISDR_UPCONVERT_MIXER_ON << 9;
//        reg0 |= MIRISDR_AM_PORT2 << 11;
//        /* AM režim je posunutý o 5 * referenční frekvence, tj. o 120 MHz */
//        /* AM mode is shifted about 5 * reference frequency, i.e. 120 MHz */
//        lo_div = 16;
//        offset += 120000000UL;
//    }
//    else if (p->freq < 108000000)
//    {
//        /* VHF */
//        reg0 |= MIRISDR_MODE_VHF << 4;
//        lo_div = 32;
//    }
//    else if (p->freq < 330000000)
//    {
//        /* B3 */
//        reg0 |= MIRISDR_MODE_B3 << 4;
//        lo_div = 16;
//    }
//    else if (p->freq < 960000000)
//    {
//        /* B45 */
//        reg0 |= MIRISDR_MODE_B45 << 4;
//        lo_div = 4;
//    }
//    else
//    {
//        /* BL */
//        reg0 |= MIRISDR_MODE_BL << 4;
//        lo_div = 2;
//    }

    /* RF syntetizer je vždy aktivní */
    /* RF synthesizer is always active */
    reg0 |= MIRISDR_RF_SYNTHESIZER_ON << 6;

    /* režim IF filtru - zatím nefunguje? */
    /* IF filter mode - has not worked? */
    switch (p->if_freq)
    {
    case MIRISDR_IF_ZERO:
        reg0 |= MIRISDR_IF_MODE_ZERO << 8;
        break;
    case MIRISDR_IF_450KHZ:
        reg0 |= MIRISDR_IF_MODE_450KHZ << 8;
        break;
    case MIRISDR_IF_1620KHZ:
        reg0 |= MIRISDR_IF_MODE_1620KHZ << 8;
        break;
    case MIRISDR_IF_2048KHZ:
        reg0 |= MIRISDR_IF_MODE_2048KHZ << 8;
        break;
    }

    /* šířka pásma - 8 MHz, nejvyšší možná */
    /* Bandwidth - 8 MHz, the highest possible */
    switch (p->bandwidth)
    {
    case MIRISDR_BW_200KHZ:
        reg0 |= 0x00 << 10;
        break;
    case MIRISDR_BW_300KHZ:
        reg0 |= 0x01 << 10;
        break;
    case MIRISDR_BW_600KHZ:
        reg0 |= 0x02 << 10;
        break;
    case MIRISDR_BW_1536KHZ:
        reg0 |= 0x03 << 10;
        break;
    case MIRISDR_BW_5MHZ:
        reg0 |= 0x04 << 10;
        break;
    case MIRISDR_BW_6MHZ:
        reg0 |= 0x05 << 10;
        break;
    case MIRISDR_BW_7MHZ:
        reg0 |= 0x06 << 10;
        break;
    case MIRISDR_BW_8MHZ:
        reg0 |= 0x07 << 10;
        break;
    case MIRISDR_BW_MAX:
        reg0 |= 0x07 << 10;
        regd |= 1;
        break;
    }

    /* xtal frekvence - nepodporujeme změnu */
    /* xtal frequency - we do not support change */
    /* the gap rows' first IF follows XTALSEL instead: 001 x6, 000 x7 */
    xsel = switch_plan.if1_low ? (uint32_t) (7 - switch_plan.if1_low) : mirisdr_xtalsel(p);
    reg0 |= xsel << 13;
    p->dc_n = mirisdr_xtalsel_n[xsel];

    /* which recalibrates the IF filter for that crystal, see mirisdr_reg13 */
    p->tuner_gap = switch_plan.if1_low != 0;

    /* 4 bity pro režimy snížené spotřeby */
    /* 4 bits for power saving modes */
    reg0 |= MIRISDR_IF_LPMODE_NORMAL << 16;
    reg0 |= MIRISDR_VCO_LPMODE_NORMAL << 19;

    /* vco frekvence, je lepší použít 64bitový rozsah */
    /* VCO frequency is better to use a 64-bit range */
    flo = p->freq + offset - (uint64_t) switch_plan.if1_low * 24000000UL;
    fvco = flo * lo_div;

    /* posun po hlavní frekvenci */
    /* shift the main frequency */
    n = fvco / 96000000UL;

    /* hlavní registr, hrubé ladění */
    /* major registry, coarse tuning */
    thresh = 96000000UL / lo_div;

    /* vedlejší registr, jemné ladění */
    /* side register, fine tuning */
    frac = (fvco % 96000000UL) / lo_div;

    /* najdeme největší společný dělitel pro thresh a frac */
    /* We find the greatest common divisor for thresh and frac */
    for (a = thresh, b = frac; a != 0;)
    {
        c = a;
        a = b % a;
        b = c;
    }

    /* dělíme */
    /* divided */
    thresh /= b;
    frac /= b;

    /* v této části musíme rozlišení snížit na maximální rozsah registru */
    /* In this section we reduce the resolution to the maximum extent registry */
    a = (thresh + 4094) / 4095;
    thresh = (thresh + (a / 2)) / a;
    frac = (frac + (a / 2)) / a;

    rfvco=(96000000UL * (n * thresh * 4096UL + (frac * 4096UL))) / (thresh * 4096UL * lo_div);
    if(flo < rfvco)
        frac --;
    rfvco=(96000000UL * (n * thresh * 4096UL + (frac * 4096UL + afc))) / (thresh * 4096UL * lo_div);
    afc = ((flo - rfvco) * thresh * 4096UL * lo_div) /96000000UL;

    reg3 |= (afc & 4095);
    reg5 |= (0xFFF & thresh);
    /* rezervováno, musí být 0x28 */
    /* Reserved, must be 0x28 */
    reg5 |= MIRISDR_RF_SYNTHESIZER_RESERVED_PROGRAMMING << 12;

    reg2 |= (0xFFF & frac);
    reg2 |= (0x3F & n) << 12;
    reg2 |= (uint32_t) (p->lna_cal_run ? MIRISDR_LBAND_LNA_CALIBRATION_ON : MIRISDR_LBAND_LNA_CALIBRATION_OFF) << 18;

    /* kernel driver nastavuje až při změně frekvence */
    /* kernel driver adjusts to changing frequencies  */

//    i = 0;
//
//    while (p->freq >= 1000000 * band_limits[i + 1]) {
//        i++;
//    }
//
//    if (band_select[i] != 0)
//    {
//        mirisdr_write_reg(p, 0x08, 0xf380);
//        mirisdr_write_reg(p, 0x08, 0x6280);
//        mirisdr_write_reg(p, 0x08, band_select[i]);
//    }

    //mirisdr_write_reg(p, 0x08, switch_plan.band_select_word);
    if (p->hw_flavour == MIRISDR_HW_RSP1B) mirisdr_rsp1b_frontend(p, switch_plan.expander);

    p->reg8=switch_plan.band_select_word;
    update_reg_8(p);

    /* register 9 is the MSi001's serial port: a board with another tuner on it
       has this port wired somewhere else, or nowhere */
    if (!p->external_tuner)
    {
        int synth = 0;

        /* only what changed; register 2 starts the calibration, so it follows any of 0, 3 and 5 */
        mirisdr_tuner_write(p, 14, p->tuner_ovr14, 0);
        mirisdr_tuner_write(p, 6, mirisdr_dc_word(p), 0);
        synth |= mirisdr_tuner_write(p, 3, reg3, 0) != 0;
        synth |= mirisdr_tuner_write(p, 0, reg0, 0) != 0;
        synth |= mirisdr_tuner_write(p, 5, reg5, 0) != 0;
        mirisdr_tuner_write(p, 2, reg2, synth);
        p->tuner_regd = regd;
        mirisdr_tuner_write(p, 13, mirisdr_reg13(p), 0);
    }

//    if (band_select[i] != 0)
//    {
//        mirisdr_write_reg(p, 0x08, 0xf380);
//        mirisdr_write_reg(p, 0x08, 0x6280);
//        mirisdr_write_reg(p, 0x08, band_select[i]);
//        mirisdr_write_reg(p, 0x09, 0x0e);
//        mirisdr_write_reg(p, 0x09, 0x03);
//
//        mirisdr_write_reg(p, 0x09, reg0);
//        mirisdr_write_reg(p, 0x09, reg5);
//        mirisdr_write_reg(p, 0x09, reg2);
//    }

#if MIRISDR_DEBUG >= 1
    fprintf( stderr,"mirisdr sel:%x ",switch_plan.band_select_word);
    fprintf( stderr,"freq: %.3f MHz (offset: %.3f MHz), n: %lu, fraction: %lu/%lu\n",
            ((double) n + (double) frac / (double) thresh) * 96.0 / (double) lo_div,
            (double) offset / 1.0e6, (long unsigned)n,
            (long unsigned)frac, (long unsigned)thresh);
    char* if_str = "undefined";
    switch (p->if_freq)
    {
    case MIRISDR_IF_ZERO:
        if_str = "0 Hz";
        break;
    case MIRISDR_IF_450KHZ:
        if_str = "450 kHz";
        break;
    case MIRISDR_IF_1620KHZ:
        if_str = "1620 kHz";
        break;
    case MIRISDR_IF_2048KHZ:
        if_str = "2048 kHz";
        break;
    }
    char* bw_str = "undefined";
    switch (p->bandwidth)
    {
    case MIRISDR_BW_200KHZ:
        bw_str = "200 kHz";
        break;
    case MIRISDR_BW_300KHZ:
        bw_str = "300 kHz";
        break;
    case MIRISDR_BW_600KHZ:
        bw_str = "600 kHz";
        break;
    case MIRISDR_BW_1536KHZ:
        bw_str = "1536 kHz";
        break;
    case MIRISDR_BW_5MHZ:
        bw_str = "5 MHz";
        break;
    case MIRISDR_BW_6MHZ:
        bw_str = "6 MHz";
        break;
    case MIRISDR_BW_7MHZ:
        bw_str = "7 MHz";
        break;
    case MIRISDR_BW_8MHZ:
        bw_str = "8 MHz";
        break;
    case MIRISDR_BW_MAX:
        bw_str = "maximal";
        break;
    }
    fprintf(stderr, "mirisdr IF: %s, BW: %s\n", if_str, bw_str);
#endif

    return 0;
}

int mirisdr_set_center_freq(mirisdr_dev_t *p, uint32_t freq)
{
    p->freq = freq;
    mirisdr_batch_begin(p);
    int r = mirisdr_set_soft(p);
    mirisdr_gain_retune(p);
    r += mirisdr_set_gain(p); // restore gain
    r += mirisdr_batch_end(p);
    return r;
}

uint32_t mirisdr_get_center_freq(mirisdr_dev_t *p)
{
    return p->freq;
}

int mirisdr_set_if_freq(mirisdr_dev_t *p, uint32_t freq)
{
#if MIRISDR_DEBUG >= 1
    fprintf(stderr, "mirisdr_set_if_freq: %u Hz\n", freq);
#endif
    if (!p)
        goto failed;

    switch (freq)
    {
    case 0:
        p->if_freq = MIRISDR_IF_ZERO;
        break;
    case 450000:
        p->if_freq = MIRISDR_IF_450KHZ;
        break;
    case 1620000:
        p->if_freq = MIRISDR_IF_1620KHZ;
        break;
    case 2048000:
        p->if_freq = MIRISDR_IF_2048KHZ;
        break;
    default:
        fprintf(stderr, "unsupported if frequency: %u Hz\n", freq);
        goto failed;
    }

    mirisdr_batch_begin(p);
    int r = mirisdr_set_soft(p);
    r += mirisdr_set_gain(p); // restore gain
    r += mirisdr_batch_end(p);
    return r;

    failed: return -1;
}

uint32_t mirisdr_get_if_freq(mirisdr_dev_t *p)
{
    if (!p)
        goto failed;

    switch (p->if_freq)
    {
    case MIRISDR_IF_ZERO:
        return 0;
    case MIRISDR_IF_450KHZ:
        return 450000;
    case MIRISDR_IF_1620KHZ:
        return 1620000;
    case MIRISDR_IF_2048KHZ:
        return 2048000;
    }

    failed: return -1;
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

int mirisdr_set_bandwidth(mirisdr_dev_t *p, uint32_t bw)
{
#if MIRISDR_DEBUG >= 1
    fprintf(stderr, "mirisdr_set_bandwidth: %u Hz\n", bw);
#endif
    if (!p)
        return -1;

    p->bandwidth = MIRISDR_BW_MAX;

    if(bw <= 8000000)
        p->bandwidth = MIRISDR_BW_8MHZ;
    if(bw <= 7000000)
        p->bandwidth = MIRISDR_BW_7MHZ;
    if(bw <= 6000000)
        p->bandwidth = MIRISDR_BW_6MHZ;
    if(bw <= 5000000)
        p->bandwidth = MIRISDR_BW_5MHZ;
    if(bw <= 1536000)
    {
        p->bandwidth = MIRISDR_BW_1536KHZ;
        if(p->rate >= 5000000)
            p->if_freq = MIRISDR_IF_1620KHZ;
    }
    if(bw <= 600000)
    {
        p->bandwidth = MIRISDR_BW_600KHZ;
        p->if_freq = MIRISDR_IF_450KHZ;
    }
    if(bw <= 300000)
    {
        p->bandwidth = MIRISDR_BW_300KHZ;
        p->if_freq = MIRISDR_IF_450KHZ;
    }
    if(bw <= 200000)
    {
        p->bandwidth = MIRISDR_BW_200KHZ;
        p->if_freq = MIRISDR_IF_450KHZ;
    }
    mirisdr_batch_begin(p);
    int r = mirisdr_set_soft(p);
    r += mirisdr_set_gain(p); // restore gain
    r += mirisdr_batch_end(p);
    return r;
}

uint32_t mirisdr_get_bandwidth(mirisdr_dev_t *p)
{
    if (!p)
        goto failed;

    switch (p->bandwidth)
    {
    case MIRISDR_BW_200KHZ:
        return 200000;
    case MIRISDR_BW_300KHZ:
        return 300000;
    case MIRISDR_BW_600KHZ:
        return 600000;
    case MIRISDR_BW_1536KHZ:
        return 1536000;
    case MIRISDR_BW_5MHZ:
        return 5000000;
    case MIRISDR_BW_6MHZ:
        return 6000000;
    case MIRISDR_BW_7MHZ:
        return 7000000;
    case MIRISDR_BW_8MHZ:
        return 8000000;
    case MIRISDR_BW_MAX:
        return 14000000;
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

int mirisdr_set_offset_tuning(mirisdr_dev_t *p, int on)
{
#if MIRISDR_DEBUG >= 1
    fprintf(stderr, "mirisdr_set_offset_tuning: %d\n", on);
#endif
    if (!p)
        goto failed;

    if (on)
    {
        p->if_freq = MIRISDR_IF_450KHZ;
    }
    else
    {
        p->if_freq = MIRISDR_IF_ZERO;
    }

    return mirisdr_set_soft(p);

    failed: return -1;
}

int mirisdr_set_transfer(mirisdr_dev_t *p, const char *v)
{
    if (!p)
        goto failed;

    /* The isochronous settings differ in how many 1kB slots they reserve in the microframe:
     * ISOC=3, ISOC1=1, ISOC2=2 (2 requires custom fw) */
    if (!strcmp(v, "BULK"))
    {
        p->transfer = MIRISDR_TRANSFER_BULK;
        p->alt_setting = 3;
    }
    else if (!strcmp(v, "ISOC"))
    {
        p->transfer = MIRISDR_TRANSFER_ISOC;
        p->alt_setting = 1;
    }
    else if (!strcmp(v, "ISOC2"))
    {
        p->transfer = MIRISDR_TRANSFER_ISOC;
        p->alt_setting = 4;
    }
    else if (!strcmp(v, "ISOC1"))
    {
        p->transfer = MIRISDR_TRANSFER_ISOC;
        p->alt_setting = 2;
    }
    else
    {
        fprintf(stderr, "unsupported transfer type: %s\n", v);
        goto failed;
    }

    /* Configure the endpoint burst size */
    return mirisdr_set_hard(p);

    failed: return -1;
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
