
/*** Register 0: IC Mode / Power Control ***/

/* reg0: 4:8 (AM_MODE, VHF_MODE, B3_MODE, B45_MODE, BL_MODE) */
#define MIRISDR_MODE_NONE                               0x00    /* a plan row without a tuner input: refused */
#define MIRISDR_MODE_AM                                 0x01
#define MIRISDR_MODE_VHF                                0x02
#define MIRISDR_MODE_B3                                 0x04
#define MIRISDR_MODE_B45                                0x08
#define MIRISDR_MODE_BL                                 0x10
/* not documented: VHF and band III together run their mixer on a VCO / 8 LO, which no band
   has alone. The original libmirisdr used it from 259 to 330 MHz. With band IV/V as well the
   /8 LO works over another VCO range, with better image rejection */
#define MIRISDR_MODE_VHF_B3                             (MIRISDR_MODE_VHF | MIRISDR_MODE_B3)
#define MIRISDR_MODE_VHF_B3_B45                         (MIRISDR_MODE_VHF | MIRISDR_MODE_B3 | MIRISDR_MODE_B45)

/* plan row edge, with the mode it hands over to: the band's VCO search pinned at that end of
   its range hands the tune to the plan's first row in that mode. The search pinned 8 to 12
   MHz before the tone was lost at both ends on two parts (2026-10-22) */
#define MIRISDR_EDGE_HIGH                               0x100
#define MIRISDR_EDGE_LOW                                0x200
#define MIRISDR_EDGE_MODE(e)                            ((e) & 0xff)

/* reg0: 9 (AM_MODE2) */
#define MIRISDR_UPCONVERT_MIXER_OFF                     0
#define MIRISDR_UPCONVERT_MIXER_ON                      1

/* reg0: 10 (RF_SYNTH) */
#define MIRISDR_RF_SYNTHESIZER_OFF                      0
#define MIRISDR_RF_SYNTHESIZER_ON                       1

/* reg0: 11 (AM_PORT_SEL) */
#define MIRISDR_AM_PORT1                                0
#define MIRISDR_AM_PORT2                                1

/* reg0: 12:13 (FIL_MODE_SEL0, FIL_MODE_SEL1) */
#define MIRISDR_IF_MODE_2048KHZ                         0
#define MIRISDR_IF_MODE_1620KHZ                         1
#define MIRISDR_IF_MODE_450KHZ                          2
#define MIRISDR_IF_MODE_ZERO                            3

/* reg0: 14:16 (FIL_BW_SEL0 - FIL_BW_SEL2) */

/* reg0: 17:19 (XTAL_SEL0 - XTAL_SEL2) */

/* reg0: 20:22 (IF_LPMODE0 - IF_LPMODE2) */
#define MIRISDR_IF_LPMODE_NORMAL                        0
#define MIRISDR_IF_LPMODE_ONLY_Q                        1
#define MIRISDR_IF_LPMODE_ONLY_I                        2
#define MIRISDR_IF_LPMODE_LOW_POWER                     4

/* reg0: 23 (VCO_LPMODE) */
#define MIRISDR_VCO_LPMODE_NORMAL                       0
#define MIRISDR_VCO_LPMODE_LOW_POWER                    1

/*** Register 2: Synthesizer Programming ***/

/* reg2: 4:15 (FRAC0 - FRAC11) */

/* reg2: 16:21 (INT0 - INT5) */

/* reg2: 22 (LNACAL_EN) */
#define MIRISDR_LBAND_LNA_CALIBRATION_OFF               0
#define MIRISDR_LBAND_LNA_CALIBRATION_ON                1

/*** Register 5: RF Synthesizer Configuration ***/

/* reg5: 4:15 (THRESH0 - THRESH11) */

/* reg5: 16:21 (reserved) */
#define MIRISDR_RF_SYNTHESIZER_RESERVED_PROGRAMMING     0x28

typedef struct
{
    uint32_t low_khz;   /* the row from this frequency, kHz */
    int mode;
    int upconvert_mixer_on;
    int am_port;
    int lo_div;
    uint32_t band_select_word;
    int if1_low;        /* AM low side: first IF this many x 24 MHz (XTALSEL 7 - n), 0 = high side 120 MHz */
    uint16_t expander;  /* SDRplay: expander IODIR word, 0 bits pulled low (sdrplay.c), 0 = none */
    int amp_db;         /* the board's amplifier on this path (RSP1B, RSPdx-R2, RSPduo), dB, 0 = none */
    int edge;           /* MIRISDR_EDGE_* | mode: tried first, that mode's row where it pins, 0 = none */
} hw_switch_freq_plan_t;
