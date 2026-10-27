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

#ifdef _WIN32
#define _CRT_RAND_S
#endif

#include <mirisdr.h>
#include <libusb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef _WIN32
#include <unistd.h>
#endif

#define EE_IDS              0xB4
#define EE_IMAGE            0xD2
#define EE_IDLE             0xFF
#define EE_RAM              0x1800      /* the ROM loads code below this */
#define EE_MIN              256         /* the smallest part */
#define EE_CHUNK            256
#define EE_SERIAL_AT        7           /* SDRplay keep the serial right behind the ids */

static mirisdr_dev_t *dev;
static int ee_size;
static int ee_fit;                      /* the part as measured, 0 until then */
static int force;
static char backup_name[160];

static void usage (void)
{
    fprintf(stderr,
        "usage: miri_eeprom [-d index] <command> [options]\n\n"
        "  saferead <file> [bytes] [-u vid:pid] [-g reg8]\n"
        "                                safest way to make an EEPROM dump; counts -d among\n"
        "                                devices with VID 1df7, or -u's\n"
        "  info                          the addressing and what byte 0 says\n"
        "  probe                         measure the real size, restoring what it touches\n"
        "  read <file> [bytes]           dump to a file\n"
        "  write <file> [address]        write a file back. At address 0, the blocks of a\n"
        "                                boot image (0xD2) must load below 0x1800\n"
        "  writeids <vid> <pid> [bcd]    boot the ROM with these ids (hex, 0x optional)\n"
        "  writefw <image|default> [-v vid] [-p pid] [-s serial|random]\n"
        "                                boot this image instead of the ROM (ids in hex).\n"
        "                                With none of -v, -p and -s it keeps the ids and\n"
        "                                serial the device has now, else the image's for\n"
        "                                those not given (-s \"\" for none). It covers\n"
        "                                bytes 0x40-0x4F, where SDRplay boards keep a\n"
        "                                factory string, so only the backup keeps that\n"
        "  disable                       byte 0 back to 0xFF\n\n"
        "  -d may come anywhere; numbers are decimal unless they start with 0x, except\n"
        "  the USB ids, which are hex as lsusb prints them\n\n"
        "  Every command that writes first saves what the whole EEPROM holds to\n"
        "  miri_eeprom-backup-<serial or ids>-<time>.bin in this directory, and does not\n"
        "  write if that fails. \"miri_eeprom write <that file>\" puts it back.\n"
        "  --force writes ids this library does not find again, an image it does not\n"
        "  know, or boot blocks it would refuse\n");

    exit(1);
}

static uint8_t *slurp (const char *path, long *len)
{
    uint8_t *buf = NULL;
    FILE *f = fopen(path, "rb");

    if (!f) { perror(path); return NULL; }

    if (fseek(f, 0, SEEK_END) || ((*len = ftell(f)) <= 0) || fseek(f, 0, SEEK_SET)
        || !(buf = malloc((size_t) *len))
        || (fread(buf, 1, (size_t) *len, f) != (size_t) *len))
    {
        fprintf(stderr, "%s: cannot read\n", path);
        free(buf);
        buf = NULL;
    }

    fclose(f);

    return buf;
}

static int verify (uint16_t addr, const uint8_t *want, int len)
{
    uint8_t *got;
    int bad;

    if (!(got = malloc((size_t) len))) return -1;

    bad = mirisdr_read_eeprom(dev, addr, got, len) || memcmp(got, want, (size_t) len);
    free(got);

    return bad ? -1 : 0;
}

/* After a failed write: where the contents from before are */
static void lost (void)
{
    static int told;

    if (told || !backup_name[0]) return;

    told = 1;
    fprintf(stderr, "what the EEPROM held before is in %s\n"
                    "\"miri_eeprom write %s\" puts it back\n", backup_name, backup_name);
}

static int store (uint16_t addr, const uint8_t *buf, int len)
{
    if (mirisdr_write_eeprom(dev, addr, buf, len))
    {
        fprintf(stderr, "write failed at 0x%04X\n", addr);
        lost();

        return -1;
    }

    if (verify(addr, buf, len))
    {
        fprintf(stderr, "verify failed at 0x%04X\n", addr);
        lost();

        return -1;
    }

    return 0;
}

static int commit (const uint8_t *buf, int len)
{
    uint8_t idle = EE_IDLE;

    if ((len < 2) || (len > ee_fit))
    {
        fprintf(stderr, "%d bytes does not fit a %d byte part\n", len, ee_fit);

        return -1;
    }

    /* Deactivate the eeprom configuration before overwriting */
    if (store(0, &idle, 1)) return -1;

    if (store(1, buf + 1, len - 1)) return -1;

    /* Write the first byte last as this controls the boot settings */
    return store(0, buf, 1);
}

static int boot (void)
{
    if (mirisdr_reboot(dev, MIRISDR_BOOT_ROM) != MIRISDR_REOPEN)
    {
        fprintf(stderr, "  written, but the reset failed: power cycle to apply it\n");

        return -1;
    }

    printf("  reset, the ROM has re-read it\n");

    return 0;
}

static int entropy (uint8_t *buf, size_t len)
{
#ifdef _WIN32
    unsigned int v;
    size_t i;

    for (i = 0; i < len; i++)
    {
        if (rand_s(&v)) return -1;

        buf[i] = (uint8_t) v;
    }

    return 0;
#else
    return getentropy(buf, len);
#endif
}

static const char *make_serial (void)
{
    static char out[MIRISDR_FW_SERIAL_MAX + 1];
    uint8_t r[MIRISDR_FW_SERIAL_MAX / 2];
    unsigned i;

    if (entropy(r, sizeof(r)))
    {
        fprintf(stderr, "no entropy available\n");

        return NULL;
    }

    for (i = 0; i < sizeof(r); i++) sprintf(out + 2 * i, "%02X", r[i]);

    return out;
}

/* The receivers the library finds, its table in devices.c. A board given other ids is
   lost to the library and to this tool, only saferead -u still reaches it */
#define OWN_VID             0x16d0
#define OWN_PID             0x158c

static int ids_ok (long vid, long pid)
{
    if ((vid >= 0) && (vid <= 0xFFFF) && (pid >= 0) && (pid <= 0xFFFF) &&
        mirisdr_device_known((uint16_t) vid, (uint16_t) pid)) return 0;

    if (force)
    {
        fprintf(stderr, "  %04lx:%04lx is not a receiver this library finds again, written anyway (--force)\n",
                vid, pid);

        return 0;
    }

    fprintf(stderr, "%04lx:%04lx is not a receiver this library finds again: neither it nor this tool\n"
                    "would reach the board afterwards (--force writes it anyway)\n", vid, pid);

    return -1;
}

/* Byte 0 0xD2: blocks of a big endian length and load address, then the bytes, up to a
   length of 0. The ROM loads each into RAM, where code may only go below 0x1800 */
static int chain_ok (const uint8_t *buf, long len)
{
    long at = 1, n, to;

    while (at + 2 <= len)
    {
        if (!(n = (buf[at] << 8) | buf[at + 1])) return 0;
        if (at + 4 > len) break;

        to = (buf[at + 2] << 8) | buf[at + 3];

        if (to + n > EE_RAM)
        {
            fprintf(stderr, "the block at 0x%04lX loads %ld bytes at 0x%04lX, past 0x%04X\n", at, n, to, EE_RAM);

            return -1;
        }

        at+= 4 + n;
    }

    fprintf(stderr, "the boot blocks run past the end of the file, no length of 0 ends them\n");

    return -1;
}

/* What the ROM would boot from a file written at address 0 */
static int boot_ok (const uint8_t *buf, long len)
{
    if (buf[0] == EE_IMAGE)
    {
        if (!chain_ok(buf, len)) return 0;
        if (!force) { fprintf(stderr, "(--force writes it anyway)\n"); return -1; }

        fprintf(stderr, "  written anyway (--force)\n");

        return 0;
    }

    if (buf[0] == EE_IDS)
    {
        if (len < 7)
        {
            fprintf(stderr, "byte 0 is 0xB4 but the ids after it are missing\n");

            return -1;
        }

        return ids_ok(buf[1] | (buf[2] << 8), buf[3] | (buf[4] << 8));
    }

    return 0;
}

/* Our firmware as the library knows it: its information block with a device descriptor,
   and a jump at the reset vector into the image */
static int image_known (const uint8_t *img, long len)
{
    mirisdr_fw_patch_t p;
    long to = (len >= 3) ? ((img[1] << 8) | img[2]) : len;

    if ((img[0] != 0x02) || (to >= len))
    {
        fprintf(stderr, "the image does not start with a jump into itself\n");

        return -1;
    }

    if (mirisdr_fw_get(img, (uint32_t) len, &p) || !(p.fields & MIRISDR_FW_PATCH_IDS))
    {
        fprintf(stderr, "the image has no information block this library knows\n");

        return -1;
    }

    return 0;
}

/* The board an SDRplay is under our own ids, by its serial's first two letters */
static const char *own_prefix (uint16_t vid, uint16_t pid)
{
    if (vid != 0x1df7) return NULL;
    if (pid == 0x3050) return "B-";
    if (pid == 0x3060) return "C-";
    if (pid == 0x3020) return "D-";
    if (pid == 0x3010) return "E-";
    if (pid == 0x3040) return "F-";

    return NULL;
}

/* The serial an SDRplay keeps behind its ids, printable up to the first byte that is not */
static int ee_serial (const uint8_t *ee, int ee_len, char *out, int len)
{
    int n;

    if ((ee_len < EE_SERIAL_AT + MIRISDR_FW_SERIAL_MAX) || (ee[0] != EE_IDS)) return 0;

    for (n = 0; (n < MIRISDR_FW_SERIAL_MAX) && (ee[EE_SERIAL_AT + n] >= 0x20) && (ee[EE_SERIAL_AT + n] < 0x7F); n++);
    if (!n || (n >= len)) return 0;

    memcpy(out, ee + EE_SERIAL_AT, (size_t) n);
    out[n] = 0;

    return n;
}

/* The ids and serial the image gets. With none of -v, -p and -s, what the device has now,
   as the library does when it loads its firmware (MIRISDR_FW_IDS_DEVICE), the serial
   from the EEPROM of an SDRplay running the ROM if it declares none. Else the image's,
   with what is given in place */
static int patch (uint8_t *img, long len, long vid, long pid, const char *serial,
                  const uint8_t *ee, int ee_len)
{
    mirisdr_fw_patch_t p, has, board;
    uint16_t dvid, dpid;
    char now[256];
    const char *pre;
    int keep;

    if (mirisdr_fw_get(img, (uint32_t) len, &has) || !(has.fields & MIRISDR_FW_PATCH_IDS))
    {
        if ((vid >= 0) || (pid >= 0) || serial)
        {
            fprintf(stderr, "the image has no information block, nothing can be patched\n");

            return -1;
        }

        printf("  the image has no information block: it keeps the ids it has, which are not checked\n");

        return 0;
    }

    keep = (vid < 0) && (pid < 0) && !serial;

    if (mirisdr_get_usb_ids(dev, &dvid, &dpid))
    {
        if (keep)
        {
            fprintf(stderr, "cannot read the device's USB ids\n");

            return -1;
        }

        dvid = dpid = 0;
    }

    memset(&p, 0, sizeof(p));
    p.fields = MIRISDR_FW_PATCH_IDS;
    p.vid = (vid >= 0) ? (uint16_t) vid : (keep ? dvid : has.vid);
    p.pid = (pid >= 0) ? (uint16_t) pid : (keep ? dpid : has.pid);

    if (keep)
    {
        if ((mirisdr_get_serial(dev, now, sizeof(now)) > 0) || ee_serial(ee, ee_len, now, sizeof(now)))
            serial = now;
    }
    else if (serial && !(has.fields & MIRISDR_FW_PATCH_SERIAL))
    {
        fprintf(stderr, "the image has no serial to set\n");

        return -1;
    }

    if (serial && (has.fields & MIRISDR_FW_PATCH_SERIAL))
    {
        if ((int) strlen(serial) > MIRISDR_FW_SERIAL_MAX)
        {
            fprintf(stderr, "%s: a serial is at most %d characters%s\n", serial, MIRISDR_FW_SERIAL_MAX,
                    (serial == now) ? ", give one with -s" : "");

            return -1;
        }

        strcpy(p.serial, serial);
        p.fields|= MIRISDR_FW_PATCH_SERIAL;
    }

    /* the board's power, whatever the ids, as the library gives it when it loads the image */
    if (!mirisdr_fw_board(dev, &board) && (board.fields & MIRISDR_FW_PATCH_POWER))
    {
        if (mirisdr_fw_get(img, (uint32_t) len, &has) || !(has.fields & MIRISDR_FW_PATCH_POWER))
            fprintf(stderr, "  note: the image has no place for the board's power, so it keeps its own\n");
        else
        {
            p.fields|= MIRISDR_FW_PATCH_POWER;
            p.self_powered = board.self_powered;
            p.max_ma = board.max_ma;
        }
    }

    if (mirisdr_fw_patch(img, (uint32_t) len, &p) || mirisdr_fw_get(img, (uint32_t) len, &has))
    {
        fprintf(stderr, "the image could not be patched\n");

        return -1;
    }

    printf("  the image will enumerate as %04x:%04x, serial \"%s\"", has.vid, has.pid, has.serial);
    if (has.fields & MIRISDR_FW_PATCH_POWER)
        printf(", %s powered, %d mA from VBUS", has.self_powered ? "self" : "bus", has.max_ma);
    printf("\n");

    /* under our own ids the library tells an SDRplay board by its serial's prefix */
    if ((has.vid == OWN_VID) && (has.pid == OWN_PID) && (pre = own_prefix(dvid, dpid))
        && strncmp(has.serial, pre, 2))
        fprintf(stderr, "  note: under %04x:%04x the library knows this board only by a serial "
                        "starting \"%s\"\n", OWN_VID, OWN_PID, pre);

    return ids_ok(has.vid, has.pid);
}

#define SD_VID              0x1df7
#define SD_TIMEOUT          1000
#define SD_POLL             200
#define SD_MIN              256
#define SD_STRAP_GPIO       0x002280    /* GPIO_1 an output, high, the rest inputs */
#define SD_GPIO             0x006280    /* as the ROM reads it: GPIO_1 high, GPIO_2 low */

static libusb_device_handle *sd_dh;

static int sd_wreg (uint8_t reg, uint32_t val)
{
    return (libusb_control_transfer(sd_dh, 0x42, 0x41, (uint16_t) ((val & 0xff) << 8 | reg),
                                    (uint16_t) (val >> 8), NULL, 0, SD_TIMEOUT) < 0) ? -1 : 0;
}

static int sd_rreg (int index, uint8_t *v)
{
    uint8_t b[4];

    if (libusb_control_transfer(sd_dh, 0xC0, 0x42, 0, (uint16_t) (index * 4), b, 4, SD_TIMEOUT) != 4) return -1;

    *v = b[0];

    return 0;
}

/* one transfer of n bytes from t1 on, rx the last byte clocked in */
static int sd_spi (int n, uint8_t t1, uint8_t t2, uint8_t t3, uint8_t t4, uint8_t *rx)
{
    uint8_t st;
    int i;

    if (sd_wreg(0x0B, (uint32_t) (0x04 | (n - 1))) || sd_wreg(0x0C, (uint32_t) (t4 | t3 << 8))
        || sd_wreg(0x0D, (uint32_t) (t2 | t1 << 8))) return -1;

    for (i = 0; i < SD_POLL; i++)
    {
        if (sd_rreg(5, &st)) return -1;
        if ((st & 0x30) == 0x30) break;
    }

    if (i == SD_POLL) return -1;

    return sd_rreg(7, rx);
}

/* READ only: 16 bit addresses, or 9 on a small part with the top bit in the command */
static int sd_read (int large, int addr, uint8_t *v)
{
    if (!large) return sd_spi(3, (uint8_t) (0x03 | ((addr >> 5) & 0x08)), (uint8_t) addr, 0, 0, v);

    return sd_spi(4, 0x03, (uint8_t) (addr >> 8), (uint8_t) addr, 0, v);
}

static int sd_open (long vid, long pid, uint32_t index)
{
    libusb_device **list;
    libusb_device_handle *dh = NULL;
    struct libusb_device_descriptor dd;
    unsigned char product[64] = "";
    ssize_t n, i;

    if (libusb_init(NULL) < 0 || (n = libusb_get_device_list(NULL, &list)) < 0) return -1;

    for (i = 0; i < n; i++)
    {
        if (libusb_get_device_descriptor(list[i], &dd) < 0) continue;
        if (dd.idVendor != ((vid < 0) ? SD_VID : vid) || ((pid >= 0) && (dd.idProduct != pid))) continue;
        if (index--) continue;

        if (libusb_open(list[i], &dh) < 0) { fprintf(stderr, "  cannot open %04x:%04x\n", dd.idVendor, dd.idProduct); dh = NULL; }
        break;
    }

    libusb_free_device_list(list, 1);

    if (!dh) return -1;

    libusb_set_auto_detach_kernel_driver(dh, 1);

    if (libusb_claim_interface(dh, 0) < 0)
    {
        fprintf(stderr, "  interface 0 is in use\n");
        libusb_close(dh);

        return -1;
    }

    if (dd.iProduct) libusb_get_string_descriptor_ascii(dh, dd.iProduct, product, sizeof(product));

    printf("  %04x:%04x bcdDevice %04x, product \"%s\"\n", dd.idVendor, dd.idProduct, dd.bcdDevice, product);

    sd_dh = dh;

    return 0;
}

static int cmd_saferead (const char *path, int len, long vid, long pid, uint32_t index, long reg8)
{
    uint8_t *buf, strap;
    int large, max, n, a, r = -1;
    FILE *f;

    if (sd_open(vid, pid, index))
    {
        fprintf(stderr, "  no device\n");

        return -1;
    }

    /* The strap is the GPIO_2 pad, so it is read with the pin an input. The ROM then
       drives GPIO_2 low before it reads, which the RSPduo needs to enable its EEPROM */
    if (sd_wreg(0x08, SD_STRAP_GPIO)) goto out;
    if (sd_rreg(6, &strap)) goto out;

    if (sd_wreg(0x08, (uint32_t) ((reg8 >= 0) ? reg8 : SD_GPIO))) goto out;

    large = (strap & 0x04) != 0;
    max = large ? 0x10000 : 0x200;

    printf("  %s addressing (strap %s)\n", large ? "16 bit" : "9 bit", large ? "high" : "low");

    if ((len < 0) || (len > max)) len = max;
    if (!(buf = malloc((size_t) max))) goto out;

    /* doubling blocks, find the size by looking for repetition (doesn't work for blank part/header) */
    for (n = 0; n < (len ? len : max); n++)
    {
        if (sd_read(large, n, &buf[n])) { fprintf(stderr, "  read failed at 0x%04x\n", n); free(buf); goto out; }

        if (!len && (n + 1 >= 2 * SD_MIN) && !((n + 1) & n) && !memcmp(buf, buf + (n + 1) / 2, (size_t) (n + 1) / 2))
        {
            n = (n + 1) / 2;
            printf("  the contents repeat every %d bytes\n", n);
            break;
        }
    }

    for (a = 1; a < n && buf[a] == buf[0]; a++);
    if (a == n) printf("  every byte is 0x%02X: a blank part, or no EEPROM\n", buf[0]);
    else printf("  byte 0 is 0x%02X (0xB4 ids, 0xD2 an image)\n", buf[0]);

    if (!(f = fopen(path, "wb"))) { perror(path); free(buf); goto out; }

    r = (fwrite(buf, 1, (size_t) n, f) == (size_t) n) ? 0 : -1;
    fclose(f);
    free(buf);

    if (!r) printf("  %d bytes to %s\n", n, path);

out:
    sd_wreg(0x0B, 0);
    libusb_release_interface(sd_dh, 0);
    libusb_close(sd_dh);
    libusb_exit(NULL);

    return r;
}

static int cmd_info (void)
{
    uint8_t head[8];

    printf("  %s address: up to %d bytes (probe measures the part)\n", (ee_size == 512) ? "9 bit" : "16 bit", ee_size);

    if (mirisdr_read_eeprom(dev, 0, head, sizeof(head))) return -1;

    switch (head[0])
    {
    case EE_IDS:
        printf("  byte 0 is 0x%02X: the ROM boots itself as %04X:%04X, bcdDevice %04X\n",
               head[0], head[1] | (head[2] << 8), head[3] | (head[4] << 8),
               head[5] | (head[6] << 8));
        break;

    case EE_IMAGE:
        printf("  byte 0 is 0x%02X: %d bytes of code load at 0x%04X and run\n", head[0],
               (head[1] << 8) | head[2], (head[3] << 8) | head[4]);
        break;

    default:
        printf("  byte 0 is 0x%02X: the ROM boots itself\n", head[0]);
        break;
    }

    return 0;
}

/* A part that ignores an address bit reads one cell through two addresses. Two
   different markers tell that apart from two cells that happen to agree. The markers
   go at 0x20 and 0x20 plus a power of two from 0x100 on, which a part of 256 bytes or
   more never folds onto bytes 0 to 6 */
static int cmd_probe (int *found)
{
    uint16_t base = 0x0020;
    uint8_t head, idle = EE_IDLE;
    int b, r = 0, size = ee_size, gone = 0, clear;

    /* the markers land inside a boot image: the ROM boots itself while they are there, and
       byte 0 comes back last, only once both are back. The ROM reads no more than bytes
       0 to 6 for its ids (0xB4), so that byte 0 stays */
    if (mirisdr_read_eeprom(dev, 0, &head, 1)) return -1;

    clear = (head != EE_IDLE) && (head != EE_IDS);

    if (clear && store(0, &idle, 1)) return -1;

    for (b = 8; (1 << b) < ee_size; b++)
    {
        uint16_t other = (uint16_t) (base | (1 << b));
        uint8_t was[2], mark[2] = { 0xA5, 0x5A }, got[2];
        int bad;

        if (mirisdr_read_eeprom(dev, base, was, 1)
            || mirisdr_read_eeprom(dev, other, was + 1, 1)) { r = -1; break; }

        bad = mirisdr_write_eeprom(dev, base, mark, 1)
              || mirisdr_write_eeprom(dev, other, mark + 1, 1)
              || mirisdr_read_eeprom(dev, base, got, 1)
              || mirisdr_read_eeprom(dev, other, got + 1, 1);

        /* put both back first, whatever the markers did */
        if (mirisdr_write_eeprom(dev, base, was, 1)
            || mirisdr_write_eeprom(dev, other, was + 1, 1)
            || verify(base, was, 1) || verify(other, was + 1, 1))
        {
            fprintf(stderr, "could not put 0x%04X and 0x%04X back\n", base, other);
            gone = 1;
            r = -1;
            break;
        }

        if (bad) { r = -1; break; }

        /* the second marker not taken: the part does not write (protected?) */
        if (got[1] != mark[1])
        {
            fprintf(stderr, "  the part does not take writes at 0x%04X: write protected?\n", other);
            r = -1;
            break;
        }

        if (got[0] != mark[0])
        {
            size = 1 << b;
            break;
        }
    }

    if (gone)
    {
        if (clear) fprintf(stderr, "byte 0 stays 0xFF, so the ROM boots itself, not the image with a marker in it\n");
        lost();

        return -1;
    }

    if (clear && store(0, &head, 1))
    {
        fprintf(stderr, "could not put byte 0 (0x%02X) back\n", head);
        return -1;
    }

    if (r) return r;

    if (size < ee_size) printf("  A%d is ignored, so the part holds %d bytes\n", b, size);
    else printf("  no address bit is ignored, so the part holds %d bytes\n", ee_size);

    if (found) *found = size;

    return 0;
}

/* Read the part from 0 in doubling blocks until the contents repeat: a part smaller than
   its addressing wraps around, so that is its size. A first half that is one byte
   throughout says nothing, so the reading goes on. Sets ee_fit, 0 when every byte is the
   same, and returns the contents */
static uint8_t *snapshot (int *len)
{
    uint8_t *buf;
    time_t t0 = time(NULL);
    int n = 0, want, half, i;

    if (!(buf = malloc((size_t) ee_size))) return NULL;

    printf("  reading the EEPROM for the backup\n");

    ee_fit = 0;

    for (want = (ee_size < 2 * EE_MIN) ? ee_size : 2 * EE_MIN; ; want*= 2)
    {
        for (; n < want; n+= EE_CHUNK)
        {
            int c = (want - n < EE_CHUNK) ? want - n : EE_CHUNK;

            if (mirisdr_read_eeprom(dev, (uint16_t) n, buf + n, c))
            {
                fprintf(stderr, "read failed at 0x%04X\n", n);
                free(buf);

                return NULL;
            }
        }

        half = want / 2;

        for (i = 1; (i < half) && (buf[i] == buf[0]); i++);

        if ((i < half) && !memcmp(buf, buf + half, (size_t) half))
        {
            ee_fit = *len = half;
            printf("  %d bytes read in %ld s, the contents repeat every %d\n", want, (long) (time(NULL) - t0), half);
            break;
        }

        if (want >= ee_size)
        {
            for (i = 1; (i < want) && (buf[i] == buf[0]); i++);

            *len = (i < want) ? want : EE_MIN;
            if (i < want) ee_fit = want;
            printf("  %d bytes read in %ld s, %s\n", want, (long) (time(NULL) - t0),
                   (i < want) ? "no repeat" : "every byte the same");
            break;
        }

        printf("  %d bytes read in %ld s\n", want, (long) (time(NULL) - t0));
        fflush(stdout);
    }

    return buf;
}

/* A file name from the serial, or the ids */
static void backup_id (char *id, int len, const uint8_t *ee, int ee_len)
{
    char s[256];
    uint16_t vid = 0, pid = 0;
    int i;

    if ((mirisdr_get_serial(dev, s, sizeof(s)) <= 0) && !ee_serial(ee, ee_len, s, sizeof(s)))
    {
        mirisdr_get_usb_ids(dev, &vid, &pid);
        snprintf(s, sizeof(s), "%04x-%04x", vid, pid);
    }

    for (i = 0; s[i] && (i < len - 1); i++)
    {
        char c = s[i];

        id[i] = (((c >= '0') && (c <= '9')) || ((c >= 'A') && (c <= 'Z')) || ((c >= 'a') && (c <= 'z'))
                 || (c == '-') || (c == '_')) ? c : '_';
    }

    id[i] = 0;
}

/* The whole EEPROM to a new file here, before anything is written. Measure: when the
   contents give no size, the markers find it */
static int backup (int measure)
{
    uint8_t *ee;
    char id[40], stamp[32];
    time_t t = time(NULL);
    struct tm *tm = localtime(&t);
    FILE *f;
    uint8_t first;
    int len, k, bad;

    if (!(ee = snapshot(&len))) return -1;

    backup_id(id, sizeof(id), ee, len);

    if (!tm || !strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", tm)) snprintf(stamp, sizeof(stamp), "%ld", (long) t);

    /* never over an earlier one */
    for (k = 0; k < 100; k++)
    {
        if (k) snprintf(backup_name, sizeof(backup_name), "miri_eeprom-backup-%s-%s-%d.bin", id, stamp, k);
        else snprintf(backup_name, sizeof(backup_name), "miri_eeprom-backup-%s-%s.bin", id, stamp);

        if (!(f = fopen(backup_name, "rb"))) break;
        fclose(f);
    }

    if ((k == 100) || !(f = fopen(backup_name, "wb")))
    {
        perror(backup_name);
        fprintf(stderr, "no backup, so nothing is written\n");
        backup_name[0] = 0;
        free(ee);

        return -1;
    }

    bad = fwrite(ee, 1, (size_t) len, f) != (size_t) len;
    bad|= fclose(f) != 0;
    first = ee[0];
    free(ee);

    if (bad)
    {
        fprintf(stderr, "%s: cannot write the backup, so nothing is written\n", backup_name);
        backup_name[0] = 0;

        return -1;
    }

    printf("  backup of the %d bytes the EEPROM held: %s\n", len, backup_name);

    /* the contents gave no size: the markers measure it, with the backup made */
    if (!ee_fit && measure)
    {
        printf("  every byte is 0x%02X, so the markers measure the size\n", first);

        if (cmd_probe(&ee_fit))
        {
            fprintf(stderr, "the size of the part is unknown, so nothing is written\n");
            ee_fit = 0;

            return -1;
        }
    }

    return 0;
}

static int cmd_read (const char *path, int len)
{
    uint8_t *buf;
    FILE *f;
    int bad;

    if ((len <= 0) || (len > ee_size)) len = ee_size;
    if (!(buf = malloc((size_t) len))) return -1;

    if (mirisdr_read_eeprom(dev, 0, buf, len))
    {
        free(buf);

        return -1;
    }

    if (!(f = fopen(path, "wb"))) { perror(path); free(buf); return -1; }

    bad = fwrite(buf, 1, (size_t) len, f) != (size_t) len;
    fclose(f);
    free(buf);

    if (!bad) printf("  %d bytes to %s\n", len, path);

    return bad ? -1 : 0;
}

static int cmd_write (const char *path, const uint8_t *buf, long len, int addr)
{
    int r;

    if (addr + len > ee_fit)
    {
        fprintf(stderr, "%ld bytes at 0x%04X does not fit a %d byte part\n", len, addr, ee_fit);

        return -1;
    }

    r = addr ? store((uint16_t) addr, buf, (int) len) : commit(buf, (int) len);

    if (!r) printf("  %ld bytes from %s at 0x%04X\n", len, path, addr);

    return r;
}

static int cmd_ids (long vid, long pid, long bcd)
{
    uint8_t blob[7];

    blob[0] = EE_IDS;
    blob[1] = (uint8_t) vid;
    blob[2] = (uint8_t) (vid >> 8);
    blob[3] = (uint8_t) pid;
    blob[4] = (uint8_t) (pid >> 8);
    blob[5] = (uint8_t) bcd;
    blob[6] = (uint8_t) (bcd >> 8);

    if (commit(blob, sizeof(blob))) return -1;

    printf("  the device will enumerate as %04lX:%04lX, bcdDevice %04lX\n", vid, pid, bcd);

    return boot();
}

/* The image, checked before the device is opened */
static uint8_t *fw_load (const char *path, long *len)
{
    uint8_t *img;

    if (!strcmp(path, "default"))
    {
        uint32_t n;
        const uint8_t *built = mirisdr_default_firmware(&n);

        *len = (long) n;

        if (!(img = malloc((size_t) *len))) return NULL;

        memcpy(img, built, (size_t) *len);
    }
    else if (!(img = slurp(path, len))) return NULL;

    /* the ROM loads the image into RAM below 0x1800, and an EEPROM dump starts with a boot byte */
    if (*len > EE_RAM)
    {
        fprintf(stderr, "%s: %ld bytes, more than the 6144 an image may have\n", path, *len);
        free(img);

        return NULL;
    }
    if ((img[0] == EE_IMAGE) || (img[0] == EE_IDS))
    {
        fprintf(stderr, "%s starts with 0x%02X: an EEPROM dump, not a firmware image (use write)\n", path, img[0]);
        free(img);

        return NULL;
    }

    if (image_known(img, *len))
    {
        if (!force)
        {
            fprintf(stderr, "%s: not an image this library knows (--force writes it anyway)\n", path);
            free(img);

            return NULL;
        }

        fprintf(stderr, "  written anyway (--force)\n");
    }

    return img;
}

/* The ids and serial go in first, so the backup (made then) is the last step before writing */
static int cmd_fw (uint8_t *img, long len, long vid, long pid, const char *serial)
{
    uint8_t head[EE_SERIAL_AT + MIRISDR_FW_SERIAL_MAX], *blob;
    int n, r, have = sizeof(head);

    if (mirisdr_read_eeprom(dev, 0, head, have)) have = 0;

    if (serial) printf("  serial %s\n", serial);

    if (patch(img, len, vid, pid, serial, head, have)) return -1;

    n = (int) len + 7;

    if (!(blob = malloc((size_t) n))) return -1;

    blob[0] = EE_IMAGE;
    blob[1] = (uint8_t) (len >> 8);
    blob[2] = (uint8_t) len;
    blob[3] = 0;
    blob[4] = 0;
    memcpy(blob + 5, img, (size_t) len);
    blob[n - 2] = 0;
    blob[n - 1] = 0;

    r = backup(1) || commit(blob, n);

    free(blob);

    if (r) return -1;

    printf("  %ld bytes of code, %d in the EEPROM\n", len, n);

    return boot();
}

static int cmd_disable (void)
{
    uint8_t idle = EE_IDLE;

    if (store(0, &idle, 1)) return -1;

    printf("  byte 0 is 0xFF, the ROM boots itself again\n");

    return boot();
}

/* A number: the whole string, in range. Hex (0x optional) when hex is set, else decimal or 0x */
static int number (const char *s, int hex, long lo, long hi, long *out)
{
    char *e;
    long v;

    if (!s || !*s) return -1;
    v = strtol(s, &e, hex ? 16 : 0);
    if (*e || (v < lo) || (v > hi)) return -1;
    *out = v;

    return 0;
}

static void bad_arg (const char *what, const char *s)
{
    fprintf(stderr, "miri_eeprom: %s '%s' is not valid\n\n", what, s ? s : "");
    usage();
}

int main (int argc, char **argv)
{
    const char *serial = NULL, *cmd = NULL, *pos[4] = { NULL, NULL, NULL, NULL };
    long vid = -1, pid = -1, bcd = 0x0200, uvid = -1, upid = -1, reg8 = -1, v, len = 0, addr = 0;
    uint8_t *data = NULL;
    uint32_t index = 0;
    int i, r = 1, npos = 0, need, most, writes;

    /* the command, its arguments and the options, in any order */
    for (i = 1; i < argc; i++)
    {
        const char *a = argv[i], *val = (i + 1 < argc) ? argv[i + 1] : NULL;

        if ((a[0] == '-') && a[1] && !a[2] && strchr("dvpsgu", a[1]))
        {
            if (!val) bad_arg("a value for", a);
            i++;
            switch (a[1])
            {
            case 'd': if (number(val, 0, 0, 255, &v)) bad_arg("device index", val); index = (uint32_t) v; break;
            case 'v': if (number(val, 1, 0, 0xFFFF, &vid)) bad_arg("vendor id", val); break;
            case 'p': if (number(val, 1, 0, 0xFFFF, &pid)) bad_arg("product id", val); break;
            case 's': serial = val; break;
            case 'g': if (number(val, 0, 0, 0xFFFFFF, &reg8)) bad_arg("register 8 value", val); break;
            case 'u':
            {
                char ids[16], *c;

                snprintf(ids, sizeof ids, "%s", val);
                if ((c = strchr(ids, ':'))) *c++ = 0;
                if (number(ids, 1, 0, 0xFFFF, &uvid) || (c && number(c, 1, 0, 0xFFFF, &upid))) bad_arg("vid:pid", val);
                break;
            }
            }
        }
        else if (!strcmp(a, "--force")) force = 1;
        else if ((a[0] == '-') && a[1] && !((a[1] >= '0') && (a[1] <= '9'))) bad_arg("option", a);
        else if (!cmd) cmd = a;
        else if (npos < 4) pos[npos++] = a;
        else bad_arg("argument", a);
    }

    if (!cmd) usage();

    /* the command and how many arguments it takes, checked before any open */
    if (!strcmp(cmd, "saferead") || !strcmp(cmd, "read")) { need = 1; most = 2; }
    else if (!strcmp(cmd, "write")) { need = 1; most = 2; }
    else if (!strcmp(cmd, "writeids")) { need = 2; most = 3; }
    else if (!strcmp(cmd, "writefw")) { need = 1; most = 1; }
    else if (!strcmp(cmd, "info") || !strcmp(cmd, "probe") || !strcmp(cmd, "disable")) { need = 0; most = 0; }
    else { fprintf(stderr, "miri_eeprom: no command '%s'\n\n", cmd); usage(); }
    if ((npos < need) || (npos > most)) usage();

    writes = strcmp(cmd, "saferead") && strcmp(cmd, "read") && strcmp(cmd, "info");

    if (!strcmp(cmd, "writeids"))
    {
        if (number(pos[0], 1, 0, 0xFFFF, &vid)) bad_arg("vendor id", pos[0]);
        if (number(pos[1], 1, 0, 0xFFFF, &pid)) bad_arg("product id", pos[1]);
        if ((npos > 2) && number(pos[2], 1, 0, 0xFFFF, &bcd)) bad_arg("bcdDevice", pos[2]);
    }
    if ((!strcmp(cmd, "read") || !strcmp(cmd, "saferead") || !strcmp(cmd, "write")) && (npos > 1) &&
        number(pos[1], 0, 0, 0xFFFF, &v)) bad_arg(strcmp(cmd, "write") ? "byte count" : "address", pos[1]);

    /* before any open: the library would upload its firmware */
    if (!strcmp(cmd, "saferead"))
    {
        printf("\n");
        r = cmd_saferead(pos[0], (npos > 1) ? (int) strtol(pos[1], NULL, 0) : 0, uvid, upid, index, reg8);
        printf("\n");

        return r ? 1 : 0;
    }

    /* what would be written, checked before any open too */
    if (!strcmp(cmd, "writeids") && ids_ok(vid, pid)) return 1;

    if (!strcmp(cmd, "write"))
    {
        addr = (npos > 1) ? strtol(pos[1], NULL, 0) : 0;

        if (!(data = slurp(pos[0], &len))) return 1;

        if (addr + len > 0x10000) fprintf(stderr, "%ld bytes at 0x%04lX is past the end of any part\n", len, addr);
        else if (!addr && (len < 2)) fprintf(stderr, "%s: at address 0 a file needs 2 bytes or more\n", pos[0]);

        if ((addr + len > 0x10000) || (!addr && ((len < 2) || boot_ok(data, len))))
        {
            free(data);

            return 1;
        }
    }

    if (!strcmp(cmd, "writefw"))
    {
        if (serial && !strcmp(serial, "random") && !(serial = make_serial())) return 1;
        if (!(data = fw_load(pos[0], &len))) return 1;
    }

    if (mirisdr_open(&dev, index) < 0)
    {
        fprintf(stderr, "no device\n");
        free(data);

        return 1;
    }

    printf("\n");

    if ((ee_size = mirisdr_eeprom_size(dev)) <= 0)
    {
        fprintf(stderr, "  no EEPROM found on this device\n"
                        "  depending on the board design, a power cycle may be needed\n\n");
        mirisdr_close(dev);
        free(data);

        return 1;
    }

    /* writefw makes its backup itself, once the image is ready */
    if (writes && strcmp(cmd, "writefw") && backup(strcmp(cmd, "probe") != 0)) r = -1;
    else if (!strcmp(cmd, "info")) r = cmd_info();
    else if (!strcmp(cmd, "probe")) r = cmd_probe(NULL);
    else if (!strcmp(cmd, "disable")) r = cmd_disable();
    else if (!strcmp(cmd, "read")) r = cmd_read(pos[0], (npos > 1) ? (int) strtol(pos[1], NULL, 0) : 0);
    else if (!strcmp(cmd, "write")) r = cmd_write(pos[0], data, len, (int) addr);
    else if (!strcmp(cmd, "writeids")) r = cmd_ids(vid, pid, bcd);
    else r = cmd_fw(data, len, vid, pid, serial);

    printf("\n");
    mirisdr_close(dev);
    free(data);

    return r ? 1 : 0;
}
