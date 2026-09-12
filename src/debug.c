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

#define MIRISDR_MEM_READ_UNIT   4
#define MIRISDR_MEM_WRITE_CHUNK 64

int mirisdr_read_mem (mirisdr_dev_t *p, uint16_t addr, uint8_t *buf, int len, int remap)
{
    uint8_t four[MIRISDR_MEM_READ_UNIT];
    int done = 0;

    if (!p) goto failed;
    if (!p->dh || !buf || (len < 0)) goto failed;

    while (done < len)
    {
        int n = len - done;
        uint16_t at = (uint16_t) (addr + done);

        if (libusb_control_transfer(p->dh, 0xC0, CMD_RREG, remap ? 1 : 0,
                                    (uint16_t) (at - 0xC000), four, sizeof(four),
                                    CTRL_TIMEOUT) != (int) sizeof(four)) goto failed;

        if (n > MIRISDR_MEM_READ_UNIT) n = MIRISDR_MEM_READ_UNIT;
        memcpy(buf + done, four, n);
        done+= n;
    }

    return 0;

failed:
    return -1;
}

int mirisdr_write_mem (mirisdr_dev_t *p, uint16_t addr, const uint8_t *buf, int len, int remap)
{
    int done = 0;

    if (!p) goto failed;
    if (!p->dh || !buf || (len < 0)) goto failed;

    while (done < len)
    {
        int n = len - done;

        if (n > MIRISDR_MEM_WRITE_CHUNK) n = MIRISDR_MEM_WRITE_CHUNK;

        if (libusb_control_transfer(p->dh, 0x40, CMD_DOWNLOAD,
                                    (uint16_t) (addr + done), remap ? 1 : 0,
                                    (unsigned char *) buf + done, n,
                                    CTRL_TIMEOUT) != n) goto failed;

        done+= n;
    }

    return 0;

failed:
    return -1;
}

/*
 * Reboot the device. Setting from_ram will boot a firmware that is loaded into
 * RAM. Returns MIRISDR_REOPEN on success, since the device leaves the bus and
 * the handle has to be closed and reopened, or -1 if the request failed.
 */
int mirisdr_reboot (mirisdr_dev_t *p, int from_ram)
{
    if (!p) goto failed;
    if (!p->dh) goto failed;

    if (libusb_control_transfer(p->dh, 0x40, CMD_RESET, from_ram ? 1 : 0, 0,
                                NULL, 0, CTRL_TIMEOUT) < 0) goto failed;

    return MIRISDR_REOPEN;

failed:
    return -1;
}
