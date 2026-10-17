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
 */

#define EXP_WRITE               0x40    /* MCP23S18 opcode, no address pins */
#define EXP_IODIRA              0x00
#define EXP_IODIRB              0x01
#define EXP_IOCON_BANK1         0x05    /* IOCON's address if BANK was set */
#define EXP_OLATA               0x14
#define EXP_OLATB               0x15
#define SDRPLAY_CS              (1 << 11)   /* register 8: GPIO_3's level */

#define SDRPLAY_A6              (1u << 6)
#define SDRPLAY_A4              (1u << 4)
#define SDRPLAY_A3              (1u << 3)
#define SDRPLAY_A2              (1u << 2)
#define SDRPLAY_A1              (1u << 1)
#define SDRPLAY_A0              (1u << 0)

static int mirisdr_sdrplay_xfer (mirisdr_dev_t *p, uint32_t r8, uint32_t cs, uint8_t reg, uint8_t val)
{
    if (mirisdr_write_reg(p, 0x08, r8 & ~cs) < 0) return -1;
    if (mirisdr_spi(p, 3, EXP_WRITE, reg, val, 0, NULL) < 0) return -1;
    if (mirisdr_write_reg(p, 0x0B, 0) < 0) return -1;

    return mirisdr_write_reg(p, 0x08, r8);
}

/* Bring the expander to a band plan word plus the notches, bias-T and antenna asked
   for; only bytes that changed are sent */
static int mirisdr_sdrplay_frontend (mirisdr_dev_t *p, uint16_t word)
{
    uint16_t w = word;
    uint32_t r8 = (p->reg8_valid ? p->reg8_sent : 0xEA80) | (1 << 15) | SDRPLAY_CS;
    int first = !p->exp_valid, r = 0;

    if (!w) return 0;

    if (p->notch & (MIRISDR_NOTCH_FM | MIRISDR_NOTCH_MW)) w &= (uint16_t) ~SDRPLAY_A4;
    if (p->notch & MIRISDR_NOTCH_DAB) w &= (uint16_t) ~SDRPLAY_A6;

    if (p->bias) w &= (uint16_t) ~SDRPLAY_A1;

    if (p->hw_flavour == MIRISDR_HW_RSPDXR2)
    {
        if (p->antenna != MIRISDR_ANTENNA_B) w &= (uint16_t) ~SDRPLAY_A2;
        if (p->antenna != MIRISDR_ANTENNA_A) w &= (uint16_t) ~SDRPLAY_A3;
        if (p->ds && !p->ds_lna) w &= (uint16_t) ~SDRPLAY_A0;
    }

    if (!first && !p->exp_stale && (p->exp_iodir == w)) return 0;

    if (first)
    {
        uint8_t a;

        r |= mirisdr_sdrplay_xfer(p, r8, SDRPLAY_CS, EXP_IOCON_BANK1, 0);
        for (a = 0x02; a <= 0x0D; a++) r |= mirisdr_sdrplay_xfer(p, r8, SDRPLAY_CS, a, 0);
        r |= mirisdr_sdrplay_xfer(p, r8, SDRPLAY_CS, EXP_OLATA, 0);
        r |= mirisdr_sdrplay_xfer(p, r8, SDRPLAY_CS, EXP_OLATB, 0);
    }

    if (first || p->exp_stale || ((p->exp_iodir ^ w) & 0x00FF))
        r |= mirisdr_sdrplay_xfer(p, r8, SDRPLAY_CS, EXP_IODIRA, (uint8_t) w);
    if (first || p->exp_stale || ((p->exp_iodir ^ w) & 0xFF00))
        r |= mirisdr_sdrplay_xfer(p, r8, SDRPLAY_CS, EXP_IODIRB, (uint8_t) (w >> 8));

    p->exp_iodir = w;
    p->exp_valid = !r;
    p->exp_stale = 0;

    return r;
}
