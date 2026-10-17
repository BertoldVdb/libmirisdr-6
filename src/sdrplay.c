/*
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
 * SDRplay front ends: the filters, notches and bias-T sit on SPI I/O expanders on the
 * MSi2500's SPI master (0x0B-0x0D), and some models carry two. The RSP1B and RSPdx-R2 are
 * mapped, and share the expander's wiring and most of its pins.
 *
 * One MCP23S18, chip select GPIO_3, MISO GPIO_0. GPIO_1 is the EEPROM's chip select.
 * The tuner takes words only with GPIO_1 and GPIO_3 high. The master pulls GPIO_1 low
 * itself, which gates the tuner of, and GPIO_3 low gates the EEPROM off, so the
 * expander can be written.
 *
 * The outputs are open drain: a pin is pulled low by clearing its IODIR bit, with
 * OLAT left at 0. IODIR bits, 0 = pulled low. RSP1B measured 2026-10-07 with a signal
 * generator and an FM antenna, the notches and HF 2026-10-08 with tone sweeps:
 *
 *   B7 B6 A7   path: 010 (B6 low) the VHF and B45 inputs, 100 (B7 low) the B3 and L
 *              inputs, each blocking the other's bands by 16-45 dB; 000 passes both
 *              2-13 dB down. 110 the HF bank, 111 the 250-420 MHz bank
 *   B1 B0      filter in the bank. HF: 2 MHz low pass both released, 2-12 both low,
 *              12-30 B0 low. 250-420 bank: 250-300 both released, 300-380 both low,
 *              380-420 B0 low, 30-60 B1 low
 *   A4         broadcast notch: MW up to 44 dB at 1 MHz, FM 37-42 dB at 80-115 MHz
 *   A6         DAB notch: 13-30 dB at 165-235 MHz
 *   A5         gain with a high pass: +13-16 dB at 40 MHz and above, -35..-40 below
 *              3 MHz
 *   A1         bias-T
 *   B2-B4      step attenuator, 6, 12.5 and 18.5 dB
 *   A0, A2, A3, B5 are not connected on the board.
 *
 * RSPdx-R2, measured 2026-10-15 with tone sweeps (a MAX2870 above 24 MHz, an Si5351
 * below), the same as the RSP1B except:
 *
 *   B1 B0: HF bank (A7 released), 500 kHz low pass both released, 2 MHz low pass
 *     B1 low, 2-12 both low, 12-30 B0 low. A5 low with 12-30 takes 11-20 dB
 *     more off below 12 MHz and costs nothing from 14 MHz
 *   A3 A2: antenna, A2 low A, A3 low B, both low C. Both released connects A and B
 *     together. The other ports are 26-45 dB down
 *   A1: bias-T, on port B
 *   B2-B5: step attenuator, about 3, 6, 12 and 15 dB
 *   A4: FM notch 51-53 dB at 85-90 MHz, MW notch 19-28 dB at 0.55-1.6 MHz
 *   A6: DAB notch 50-72 dB at 175-210 MHz
 *   A0: direct sampling amplifier, on when released, +37.5 dB flat from 0.1 to 2
 *     MHz, the I converter's floor 4-5 dB up. No effect with GPIO_2 low
 *
 * RSPdx-R2 direct sampling: GPIO_2 high (register 8, the MSi2500's own pin) switches the
 * I converter from the tuner's I output to the antenna through the HF bank, the tuner
 * bypassed (its gain changes nothing there). Q stays on the tuner. It follows the bank
 * code: the 2 MHz low pass flat from 30 kHz to 1.5 MHz, -15 dB at 2.5 MHz, the 500 kHz
 * low pass flat to 500 kHz and ~60 dB down from 700 kHz. Below 30 kHz it rolls off
 * (-8 dB at 10 kHz, -21 at 5 kHz). It follows the antenna bits.
 *
 * RSPduo, 2026-10-17: two MCP23S18, the chip selects decoded from GPIO_3 and GPIO_2 with
 * the master's own: GPIO_3 low the first, GPIO_2 high (GPIO_3 high) the second, both with
 * GPIO_3 low and GPIO_2 high, and GPIO_3 high and GPIO_2 low the EEPROM. Every SPI read needs
 * GPIO_2 low except the second chip's. Of the first chip's pins:
 *
 *   A7 B0: which tuner the converters get. Both released I and Q from tuner 2, both
 *     low I and Q from tuner 1, one low a tuner each (low IF from both)
 *   B7 B6: which tuner takes the tuner words. Both released tuner 1, B6 low both,
 *     B7 low tuner 2, both low neither. Tuner 1 is the one the readback reads
 *   A6: held low on the board
 *   B1-B5: tuner 1's filter, a decoder. B4 low with B3 released enables the first
 *     bank, B1 B4 110-240 MHz and L band, B1 B4 B5 60-100 and 420-900 MHz (the
 *     tuner's band picks one), B2 18 dB down. B3 low the second: B3 B4 B5 30-60,
 *     B1 B3 B4 250-300, B1 B3 B5 300-380, B1 B3 380-420 MHz (16 dB weaker)
 *   A0 A1: with B1 B4 in L band, +5-17 dB from 1 to 1.5 GHz, worse from 1.7 GHz
 *
 * Of the second's, on tuner 1 (measured with its filter on, 2026-10-17):
 *
 *   A2: its 50 ohm input to the filters, +30-47 dB (released: the other input,
 *     probably Hi-Z, not checked)
 *   A7: FM notch, 40 dB at 80 MHz and 61 dB at 95 MHz
 *   B1: DAB notch, 28-30 dB at 180-200 MHz
 *   B5 B6 B7: attenuator, about 6, 12 and 18 dB
 *   A4: 2-15 dB, rising with frequency up to 600 MHz, purpose unknown
 *
 * Not mapped yet: tuner 2's front end, the HF bank below 30 MHz, the Hi-Z input and the
 * bias-T.
 */

#define EXP_WRITE               0x40    /* MCP23S18 opcode, no address pins */
#define EXP_IODIRA              0x00
#define EXP_IODIRB              0x01
#define EXP_IOCON_BANK1         0x05    /* IOCON's address if BANK was set */
#define EXP_OLATA               0x14
#define EXP_OLATB               0x15

/* register 8: GPIO_2 and GPIO_3 as outputs, and their levels */
#define SDRPLAY_G2_OUT          (1u << 14)
#define SDRPLAY_G3_OUT          (1u << 15)
#define SDRPLAY_G2              (1u << 10)
#define SDRPLAY_G3              (1u << 11)

#define SDRPLAY_A6              (1u << 6)
#define SDRPLAY_A4              (1u << 4)
#define SDRPLAY_A3              (1u << 3)
#define SDRPLAY_A2              (1u << 2)
#define SDRPLAY_A1              (1u << 1)
#define SDRPLAY_A0              (1u << 0)

/* The plan's expander word with the notches, bias-T, antenna and direct sampling asked
   for: the pins they pull low */
static uint16_t mirisdr_sdrplay_word (int flavour, uint16_t word, int notch, int bias, int antenna,
                                      int ds, int ds_lna)
{
    uint16_t w = word;

    if (!w || (flavour == MIRISDR_HW_RSPDUO)) return w;

    if (notch & (MIRISDR_NOTCH_FM | MIRISDR_NOTCH_MW)) w &= (uint16_t) ~SDRPLAY_A4;
    if (notch & MIRISDR_NOTCH_DAB) w &= (uint16_t) ~SDRPLAY_A6;

    if (bias) w &= (uint16_t) ~SDRPLAY_A1;

    if (flavour == MIRISDR_HW_RSPDXR2)
    {
        if (antenna != MIRISDR_ANTENNA_B) w &= (uint16_t) ~SDRPLAY_A2;
        if (antenna != MIRISDR_ANTENNA_A) w &= (uint16_t) ~SDRPLAY_A3;
        if (ds && !ds_lna) w &= (uint16_t) ~SDRPLAY_A0;
    }

    return w;
}

/* The RSPduo's second expander: tuner 1's 50 ohm input (A2 low) and its notches. The FM
   and MW notches share a switch, as mirisdr_notch_in_force() has it */
#define DUO2_INPUT_50   (1u << 2)       /* A2 */
#define DUO2_NOTCH_FM   (1u << 7)       /* A7: 61 dB at 95 MHz */
#define DUO2_NOTCH_DAB  (1u << 9)       /* B1: 30 dB at 180-200 MHz */

static uint16_t mirisdr_sdrplay_word2 (int notch)
{
    uint16_t w = (uint16_t) ~DUO2_INPUT_50;

    if (notch & (MIRISDR_NOTCH_FM | MIRISDR_NOTCH_MW)) w &= (uint16_t) ~DUO2_NOTCH_FM;
    if (notch & MIRISDR_NOTCH_DAB) w &= (uint16_t) ~DUO2_NOTCH_DAB;

    return w;
}

/* One write to expander chip (0 the first), register 8 selecting it for the frame and
   then back to idle, which keeps GPIO_3 high. The RSPduo decodes the selects: GPIO_3 low
   is the first, GPIO_2 high the second, so the first needs GPIO_2 low with it */
static int mirisdr_sdrplay_xfer (mirisdr_dev_t *p, uint32_t idle, int chip, uint8_t reg, uint8_t val)
{
    uint32_t r8 = idle;

    if (chip) r8 |= SDRPLAY_G2_OUT | SDRPLAY_G2;
    else
    {
        r8 &= ~SDRPLAY_G3;
        if (p->hw_flavour == MIRISDR_HW_RSPDUO) r8 = (r8 | SDRPLAY_G2_OUT) & ~SDRPLAY_G2;
    }

    if (mirisdr_write_reg(p, 0x08, r8) < 0) return -1;
    if (mirisdr_spi(p, 3, EXP_WRITE, reg, val, 0, NULL) < 0) return -1;
    if (mirisdr_write_reg(p, 0x0B, 0) < 0) return -1;

    return mirisdr_write_reg(p, 0x08, idle);
}

/* Bring expander chip to word w: set up the first time, after that only the bytes that
   changed */
static int mirisdr_sdrplay_send (mirisdr_dev_t *p, int chip, uint16_t w)
{
    uint32_t idle = (p->reg8_valid ? p->reg8_sent : 0xEA80) | SDRPLAY_G3_OUT | SDRPLAY_G3;
    int bit = 1 << chip, first = !(p->exp_valid & bit), stale = p->exp_stale & bit, r = 0;

    if (!first && !stale && (p->exp_iodir[chip] == w)) return 0;

    if (first)
    {
        uint8_t a;

        r |= mirisdr_sdrplay_xfer(p, idle, chip, EXP_IOCON_BANK1, 0);
        for (a = 0x02; a <= 0x0D; a++) r |= mirisdr_sdrplay_xfer(p, idle, chip, a, 0);
        r |= mirisdr_sdrplay_xfer(p, idle, chip, EXP_OLATA, 0);
        r |= mirisdr_sdrplay_xfer(p, idle, chip, EXP_OLATB, 0);
    }

    if (first || stale || ((p->exp_iodir[chip] ^ w) & 0x00FF))
        r |= mirisdr_sdrplay_xfer(p, idle, chip, EXP_IODIRA, (uint8_t) w);
    if (first || stale || ((p->exp_iodir[chip] ^ w) & 0xFF00))
        r |= mirisdr_sdrplay_xfer(p, idle, chip, EXP_IODIRB, (uint8_t) (w >> 8));

    p->exp_iodir[chip] = w;
    if (r) p->exp_valid &= ~bit;
    else p->exp_valid |= bit;
    p->exp_stale &= ~bit;

    return r;
}
