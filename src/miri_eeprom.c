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

#ifndef _WIN32
#include <unistd.h>
#endif

#define EE_IDS              0xB4
#define EE_IMAGE            0xD2
#define EE_IDLE             0xFF

static mirisdr_dev_t *dev;
static int ee_size;

static void usage (void)
{
    fprintf(stderr,
        "usage: miri_eeprom [-d index] <command>\n\n"
        "  saferead <file> [bytes] [-u vid:pid] [-g reg8]\n"
        "                                safest way to make an EEPROM dump\n"
        "  info                          what is fitted and what byte 0 says\n"
        "  probe                         measure the real size, restoring what it touches\n"
        "  read <file> [bytes]           dump to a file\n"
        "  write <file> [address]        write a file back\n"
        "  writeids <vid> <pid> [bcd]    boot the ROM with these ids\n"
        "  writefw <image|default> [-v vid] [-p pid] [-s serial|random]\n"
        "                                boot this image instead of the ROM\n"
        "  disable                       byte 0 back to 0xFF\n");

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

static int store (uint16_t addr, const uint8_t *buf, int len)
{
    if (mirisdr_write_eeprom(dev, addr, buf, len))
    {
        fprintf(stderr, "write failed at 0x%04X\n", addr);

        return -1;
    }

    if (verify(addr, buf, len))
    {
        fprintf(stderr, "verify failed at 0x%04X\n", addr);

        return -1;
    }

    return 0;
}

static int commit (const uint8_t *buf, int len)
{
    uint8_t idle = EE_IDLE;

    if ((len < 2) || (len > ee_size))
    {
        fprintf(stderr, "%d bytes does not fit a %d byte part\n", len, ee_size);

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

static int patch (uint8_t *img, long len, long vid, long pid, const char *serial)
{
    mirisdr_fw_patch_t p;

    if ((vid < 0) && (pid < 0) && !serial) return 0;

    if (mirisdr_fw_get(img, (uint32_t) len, &p))
    {
        fprintf(stderr, "the image has no information block, nothing can be patched\n");

        return -1;
    }

    p.fields = 0;

    if ((vid >= 0) || (pid >= 0))
    {
        if (vid >= 0) p.vid = (uint16_t) vid;
        if (pid >= 0) p.pid = (uint16_t) pid;

        p.fields|= MIRISDR_FW_PATCH_IDS;
    }

    if (serial)
    {
        if ((int) strlen(serial) > MIRISDR_FW_SERIAL_MAX)
        {
            fprintf(stderr, "a serial is at most %d characters\n", MIRISDR_FW_SERIAL_MAX);

            return -1;
        }

        strcpy(p.serial, serial);
        p.fields|= MIRISDR_FW_PATCH_SERIAL;
    }

    return mirisdr_fw_patch(img, (uint32_t) len, &p);
}

#define SD_VID              0x1df7
#define SD_TIMEOUT          1000
#define SD_POLL             200
#define SD_MIN              256
#define SD_GPIO             0x002280    /* GPIO_1 an output, high, the rest inputs */

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

    if (sd_wreg(0x08, (uint32_t) ((reg8 >= 0) ? reg8 : SD_GPIO))) goto out;

    if (sd_rreg(6, &strap)) goto out;

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

    printf("  %d bytes, %s address\n", ee_size, (ee_size == 512) ? "9 bit" : "16 bit");

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

/* A part that ignores an address bit reads one cell through two addresses.  Two
   different markers tell that apart from two cells that happen to agree. */
static int cmd_probe (void)
{
    uint16_t base = 0x0020;
    int b;

    for (b = 8; (1 << b) < ee_size; b++)
    {
        uint16_t other = (uint16_t) (base | (1 << b));
        uint8_t was[2], mark[2] = { 0xA5, 0x5A }, got;
        int aliased;

        if (mirisdr_read_eeprom(dev, base, was, 1)
            || mirisdr_read_eeprom(dev, other, was + 1, 1)) return -1;

        if (mirisdr_write_eeprom(dev, base, mark, 1)
            || mirisdr_write_eeprom(dev, other, mark + 1, 1)
            || mirisdr_read_eeprom(dev, base, &got, 1)) return -1;

        aliased = got != mark[0];

        if (mirisdr_write_eeprom(dev, base, was, 1)
            || mirisdr_write_eeprom(dev, other, was + 1, 1)
            || verify(base, was, 1))
        {
            fprintf(stderr, "could not put 0x%04X and 0x%04X back\n", base, other);

            return -1;
        }

        if (aliased)
        {
            printf("  A%d is ignored, so the part holds %d bytes\n", b, 1 << b);

            return 0;
        }
    }

    printf("  no address bit is ignored, so the part holds %d bytes\n", ee_size);

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

static int cmd_write (const char *path, int addr)
{
    uint8_t *buf;
    long len;
    int r;

    if (!(buf = slurp(path, &len))) return -1;

    if (addr + len > ee_size)
    {
        fprintf(stderr, "%ld bytes at 0x%04X does not fit a %d byte part\n", len, addr, ee_size);
        free(buf);

        return -1;
    }

    r = addr ? store((uint16_t) addr, buf, (int) len) : commit(buf, (int) len);
    free(buf);

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

static int cmd_fw (const char *path, long vid, long pid, const char *serial)
{
    uint8_t *img, *blob;
    long len;
    int n, r;

    if (!strcmp(path, "default"))
    {
        uint32_t n;
        const uint8_t *built = mirisdr_default_firmware(&n);

        len = (long) n;

        if (!(img = malloc((size_t) len))) return -1;

        memcpy(img, built, (size_t) len);
    }
    else if (!(img = slurp(path, &len))) return -1;

    if (serial && !strcmp(serial, "random") && !(serial = make_serial()))
    {
        free(img);

        return -1;
    }

    if (serial) printf("  serial %s\n", serial);

    if (patch(img, len, vid, pid, serial))
    {
        free(img);

        return -1;
    }

    n = (int) len + 7;

    if (!(blob = malloc((size_t) n))) { free(img); return -1; }

    blob[0] = EE_IMAGE;
    blob[1] = (uint8_t) (len >> 8);
    blob[2] = (uint8_t) len;
    blob[3] = 0;
    blob[4] = 0;
    memcpy(blob + 5, img, (size_t) len);
    blob[n - 2] = 0;
    blob[n - 1] = 0;

    r = commit(blob, n);

    free(blob);
    free(img);

    if (r) return r;

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

int main (int argc, char **argv)
{
    const char *serial = NULL, *cmd;
    long vid = -1, pid = -1, bcd = 0x0200, uvid = -1, upid = -1, reg8 = -1;
    uint32_t index = 0;
    int i, r = 1;

    while ((argc > 2) && !strcmp(argv[1], "-d"))
    {
        index = (uint32_t) strtoul(argv[2], NULL, 0);
        argv+= 2;
        argc-= 2;
    }

    if (argc < 2) usage();

    cmd = argv[1];

    for (i = 2; i < argc - 1; i++)
    {
        if (!strcmp(argv[i], "-v")) vid = strtol(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "-p")) pid = strtol(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "-s")) serial = argv[++i];
        else if (!strcmp(argv[i], "-g")) reg8 = strtol(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "-u"))
        {
            char *e;

            uvid = strtol(argv[++i], &e, 16);
            if (*e == ':') upid = strtol(e + 1, NULL, 16);
        }
    }

    /* before any open: the library would upload its firmware */
    if (!strcmp(cmd, "saferead") && (argc > 2))
    {
        printf("\n");
        r = cmd_saferead(argv[2], ((argc > 3) && (argv[3][0] != '-')) ? (int) strtol(argv[3], NULL, 0) : 0,
                     uvid, upid, index, reg8);
        printf("\n");

        return r ? 1 : 0;
    }


    if (mirisdr_open(&dev, index) < 0)
    {
        fprintf(stderr, "no device\n");

        return 1;
    }

    printf("\n");

    if ((ee_size = mirisdr_eeprom_size(dev)) <= 0)
    {
        fprintf(stderr, "  no EEPROM found on this device\n"
                        "  depending on the board design, a power cycle may be needed\n\n");
        mirisdr_close(dev);

        return 1;
    }

    if (!strcmp(cmd, "info")) r = cmd_info();
    else if (!strcmp(cmd, "probe")) r = cmd_probe();
    else if (!strcmp(cmd, "disable")) r = cmd_disable();
    else if (!strcmp(cmd, "read") && (argc > 2))
        r = cmd_read(argv[2], (argc > 3) ? (int) strtol(argv[3], NULL, 0) : 0);
    else if (!strcmp(cmd, "write") && (argc > 2))
        r = cmd_write(argv[2], (argc > 3) ? (int) strtol(argv[3], NULL, 0) : 0);
    else if (!strcmp(cmd, "writeids") && (argc > 3))
        r = cmd_ids(strtol(argv[2], NULL, 0), strtol(argv[3], NULL, 0),
                    (argc > 4) ? strtol(argv[4], NULL, 0) : bcd);
    else if (!strcmp(cmd, "writefw") && (argc > 2)) r = cmd_fw(argv[2], vid, pid, serial);
    else { mirisdr_close(dev); printf("\n"); usage(); }

    printf("\n");
    mirisdr_close(dev);

    return r ? 1 : 0;
}
