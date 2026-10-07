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
 * MSi2500's SPI master (0x0B-0x0D); some models carry two. Only the RSP1B is mapped.
 *
 * RSP1B: one MCP23S18, chip select GPIO_3, MISO GPIO_0; GPIO_1 is the EEPROM's chip
 * select. The tuner takes words only with GPIO_1 and GPIO_3 high. The master pulls
 * GPIO_1 low itself, which gates the tuner of, and GPIO_3 low gates the EEPROM off, so
 * the expander can be written.
 *
 * The outputs are open drain: a pin is pulled low by clearing its IODIR bit, with
 * OLAT left at 0. IODIR bits, 0 = pulled low (measured 2026-10-07 with a signal generator
 * and an FM antenna):
 *
 *   B7 B6 A7   path: 000 direct (the VHF, B3, B45 and L inputs), 100 VHF through the
 *              FM notch, 110 the HF bank, 111 the 250-420 MHz bank
 *   B1 B0      filter in the bank: 250-300 both released, 300-380 both low, 380-420
 *              B0 low.  HF undetermined: B0 low received 7 and 14 MHz best, the four
 *              HF filters are not told apart yet
 *   A6         DAB notch
 *   A5         +9-15 dB everywhere, kept low
 *   A1         bias-T
 *   A4, B2-B4  not placed (the HF filters, the MW notch)
 *   A0, A2, A3, B5 are not connected on the board.
 */

#define EXP_WRITE               0x40    /* MCP23S18 opcode, no address pins */
#define EXP_IODIRA              0x00
#define EXP_IODIRB              0x01
#define EXP_IOCON_BANK1         0x05    /* IOCON's address if BANK was set */
#define EXP_OLATA               0x14
#define EXP_OLATB               0x15
#define RSP1B_CS                (1 << 11)   /* register 8: GPIO_3's level */

#define RSP1B_B7                (1u << 15)
#define RSP1B_B6                (1u << 14)
#define RSP1B_A7                (1u << 7)
#define RSP1B_A6                (1u << 6)
#define RSP1B_A1                (1u << 1)

static int mirisdr_sdrplay_xfer (mirisdr_dev_t *p, uint32_t r8, uint32_t cs, uint8_t reg, uint8_t val)
{
    uint8_t b[4];
    int i;

    if (mirisdr_write_reg(p, 0x08, r8 & ~cs) < 0) return -1;
    if (mirisdr_write_reg(p, 0x0B, 0x04 | 2) < 0) return -1;
    if (mirisdr_write_reg(p, 0x0C, (uint32_t) val << 8) < 0) return -1;
    if (mirisdr_write_reg(p, 0x0D, (uint32_t) EXP_WRITE << 8 | reg) < 0) return -1;

    /* in a list the firmware waits for the master, otherwise poll it */
    if (p->batch_depth && p->fw_ours)
    {
        if (mirisdr_write_reg(p, MIRISDR_LIST_WAIT_SPI, 0) < 0) return -1;
    }
    else
    {
        for (i = 0; i < 200; i++)
        {
            if (mirisdr_read_reg(p, 5, b, sizeof(b)) != (int) sizeof(b)) return -1;
            if ((b[0] & 0x30) == 0x30) break;
        }

        if (i == 200) return -1;
    }

    if (mirisdr_write_reg(p, 0x0B, 0) < 0) return -1;

    return mirisdr_write_reg(p, 0x08, r8);
}

/* Bring the RSP1B's expander to a band plan word plus the notches and bias-T asked
   for; only bytes that changed are sent */
static int mirisdr_rsp1b_frontend (mirisdr_dev_t *p, uint16_t word)
{
    uint16_t w = word;
    uint32_t r8 = (p->reg8_valid ? p->reg8_sent : 0xEA80) | (1 << 15) | RSP1B_CS;
    int first = !p->exp_valid, r = 0;

    if (!w) return 0;

    if ((p->notch & MIRISDR_NOTCH_FM) &&
        ((w & (RSP1B_B7 | RSP1B_B6 | RSP1B_A7)) == (RSP1B_B7 | RSP1B_B6 | RSP1B_A7))) w &= (uint16_t) ~RSP1B_B7;
    if (p->notch & MIRISDR_NOTCH_DAB) w &= (uint16_t) ~RSP1B_A6;

    if (p->bias) w &= (uint16_t) ~RSP1B_A1;

    if (!first && (p->exp_iodir == w)) return 0;

    if (first)
    {
        uint8_t a;

        r |= mirisdr_sdrplay_xfer(p, r8, RSP1B_CS, EXP_IOCON_BANK1, 0);
        for (a = 0x02; a <= 0x0D; a++) r |= mirisdr_sdrplay_xfer(p, r8, RSP1B_CS, a, 0);
        r |= mirisdr_sdrplay_xfer(p, r8, RSP1B_CS, EXP_OLATA, 0);
        r |= mirisdr_sdrplay_xfer(p, r8, RSP1B_CS, EXP_OLATB, 0);
    }

    if (first || ((p->exp_iodir ^ w) & 0x00FF)) r |= mirisdr_sdrplay_xfer(p, r8, RSP1B_CS, EXP_IODIRA, (uint8_t) w);
    if (first || ((p->exp_iodir ^ w) & 0xFF00)) r |= mirisdr_sdrplay_xfer(p, r8, RSP1B_CS, EXP_IODIRB, (uint8_t) (w >> 8));

    p->exp_iodir = w;
    p->exp_valid = !r;

    return r;
}
