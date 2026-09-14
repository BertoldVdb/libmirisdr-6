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

#define MIRISDR_EE_READ         0x03
#define MIRISDR_EE_WRITE        0x02
#define MIRISDR_EE_WREN         0x06
#define MIRISDR_EE_WRDI         0x04
#define MIRISDR_EE_RDSR         0x05

#define MIRISDR_EE_WEL          0x02
#define MIRISDR_EE_SMALL        0x200
#define MIRISDR_EE_LARGE        0x10000

#define MIRISDR_EE_WIP          0x01
#define MIRISDR_EE_READY        0x30

#define MIRISDR_EE_POLL         200
#define MIRISDR_EE_WRITE_MS     500

static int mirisdr_spi (mirisdr_dev_t *p, int n, uint8_t t1, uint8_t t2, uint8_t t3,
                        uint8_t t4, uint8_t *rx)
{
    uint8_t buf[4];
    int i;

    if (mirisdr_write_reg(p, 0x0B, (uint32_t) (0x04 | (n - 1))) < 0) return -1;
    if (mirisdr_write_reg(p, 0x0C, (uint32_t) t4 | ((uint32_t) t3 << 8)) < 0) return -1;
    if (mirisdr_write_reg(p, 0x0D, (uint32_t) t2 | ((uint32_t) t1 << 8)) < 0) return -1;

    for (i = 0; i < MIRISDR_EE_POLL; i++)
    {
        if (mirisdr_read_reg(p, 5, buf, sizeof(buf)) != (int) sizeof(buf)) return -1;
        if ((buf[0] & MIRISDR_EE_READY) == MIRISDR_EE_READY) break;
    }

    if (i == MIRISDR_EE_POLL) return -1;

    if (!rx) return 0;

    if (mirisdr_read_reg(p, 7, buf, sizeof(buf)) != (int) sizeof(buf)) return -1;

    *rx = buf[0];

    return 0;
}

static int mirisdr_ee_get (mirisdr_dev_t *p, uint16_t addr, uint8_t *v)
{
    if (p->ee_size == MIRISDR_EE_SMALL)
        return mirisdr_spi(p, 3, (uint8_t) (MIRISDR_EE_READ | ((addr >> 5) & 0x08)),
                           (uint8_t) addr, 0x00, 0, v);

    return mirisdr_spi(p, 4, MIRISDR_EE_READ, (uint8_t) (addr >> 8), (uint8_t) addr, 0x00, v);
}

static int mirisdr_ee_put (mirisdr_dev_t *p, uint16_t addr, uint8_t v)
{
    int i;

    if (mirisdr_spi(p, 1, MIRISDR_EE_WREN, 0, 0, 0, NULL) < 0) return -1;

    if (p->ee_size == MIRISDR_EE_SMALL)
    {
        if (mirisdr_spi(p, 3, (uint8_t) (MIRISDR_EE_WRITE | ((addr >> 5) & 0x08)),
                        (uint8_t) addr, v, 0, NULL) < 0) return -1;
    }
    else if (mirisdr_spi(p, 4, MIRISDR_EE_WRITE, (uint8_t) (addr >> 8),
                         (uint8_t) addr, v, NULL) < 0) return -1;

    for (i = 0; i < MIRISDR_EE_WRITE_MS; i++)
    {
        uint8_t sr;

        if (mirisdr_spi(p, 2, MIRISDR_EE_RDSR, 0x00, 0, 0, &sr) < 0) return -1;
        if (!(sr & MIRISDR_EE_WIP)) return 0;

        usleep(1000);
    }

    return -1;
}

static int mirisdr_ee_probe (mirisdr_dev_t *p)
{
    uint8_t wren = 0, wrdi = 0;
    int g;

    p->ee_size = 0;

    if (mirisdr_spi(p, 1, MIRISDR_EE_WREN, 0, 0, 0, NULL) < 0) return -1;
    if (mirisdr_spi(p, 2, MIRISDR_EE_RDSR, 0x00, 0, 0, &wren) < 0) return -1;
    if (mirisdr_spi(p, 1, MIRISDR_EE_WRDI, 0, 0, 0, NULL) < 0) return -1;
    if (mirisdr_spi(p, 2, MIRISDR_EE_RDSR, 0x00, 0, 0, &wrdi) < 0) return -1;

    if (!(wren & MIRISDR_EE_WEL) || (wrdi & MIRISDR_EE_WEL)) return 0;

    if ((g = mirisdr_get_gpio_inputs(p)) < 0) return -1;

    p->ee_size = (g & 0x04) ? MIRISDR_EE_LARGE : MIRISDR_EE_SMALL;

    return p->ee_size;
}

static int mirisdr_ee_begin (mirisdr_dev_t *p)
{
    if (!p || !p->dh) return -1;

    /* the tuner is programmed over this same port */
    if (p->async_status != MIRISDR_ASYNC_INACTIVE) return -1;

    /* set GPIO for EEPROM access */
    if (mirisdr_write_reg(p, 0x08, 0x002280) < 0) return -1;

    if (p->ee_size < 0) return mirisdr_ee_probe(p);

    return p->ee_size;
}

static void mirisdr_ee_end (mirisdr_dev_t *p)
{
    mirisdr_write_reg(p, 0x0B, 0);

    update_reg_8(p);
}

int mirisdr_read_eeprom (mirisdr_dev_t *p, uint16_t addr, uint8_t *buf, int len)
{
    int i, r = -1;

    if (!p || !buf || (len < 0)) return -1;
    if (mirisdr_ee_begin(p) <= 0) goto out;
    if ((int) addr + len > p->ee_size) goto out;

    for (i = 0; i < len; i++)
        if (mirisdr_ee_get(p, (uint16_t) (addr + i), buf + i) < 0) goto out;

    r = 0;

out:
    mirisdr_ee_end(p);

    return r;
}

int mirisdr_write_eeprom (mirisdr_dev_t *p, uint16_t addr, const uint8_t *buf, int len)
{
    int i, r = -1;

    if (!p || !buf || (len < 0)) return -1;
    if (mirisdr_ee_begin(p) <= 0) goto out;
    if ((int) addr + len > p->ee_size) goto out;

    for (i = 0; i < len; i++)
    {
        uint16_t at = (uint16_t) (addr + i);
        uint8_t now;

        /* dont write identical bytes */
        if (mirisdr_ee_get(p, at, &now) < 0) goto out;
        if (now == buf[i]) continue;

        if (mirisdr_ee_put(p, at, buf[i]) < 0) goto out;
    }

    r = 0;

out:
    mirisdr_ee_end(p);

    return r;
}

int mirisdr_eeprom_size (mirisdr_dev_t *p)
{
    int r;

    if (!p) return -1;
    if (mirisdr_ee_begin(p) < 0) return -1;

    r = p->ee_size;

    mirisdr_ee_end(p);

    return r;
}
