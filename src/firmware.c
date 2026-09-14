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

static int mirisdr_open_raw (mirisdr_dev_t **p, uint32_t index);
static int mirisdr_open_fd_raw (mirisdr_dev_t **p, int fd);

void mirisdr_open_config_default (mirisdr_open_config_t *cfg)
{
    if (!cfg) return;

    memset(cfg, 0, sizeof(*cfg));

    cfg->fd = -1;
    cfg->firmware_ids = MIRISDR_FW_IDS_DEVICE;
}

#define MIRISDR_FW_GONE_MS      2000
#define MIRISDR_FW_WAIT_MS      6000

static int mirisdr_fw_block (mirisdr_dev_t *p, uint8_t *block)
{
    if (!p) return -1;
    if (mirisdr_read_mem(p, MIRISDR_FW_BLOCK, block, 16, 0) < 0) return -1;

    if ((block[0] != 'B') || (block[1] != 'V')
        || (block[2] != 'D') || (block[3] != 'B')) return -1;

    return 0;
}

int mirisdr_get_fw_id (mirisdr_dev_t *p, uint8_t *buf, int len)
{
    uint8_t block[16];

    if (!buf || (len <= 0)) return -1;
    if (mirisdr_fw_block(p, block) < 0) return -1;

    if (len > MIRISDR_FW_ID_LEN) len = MIRISDR_FW_ID_LEN;
    memcpy(buf, block + 4, (size_t) len);

    return len;
}

int mirisdr_running_from_rom (mirisdr_dev_t *p)
{
    uint8_t low[4], mirror[4];
    uint16_t at;

    for (at = 0; at < 0x1000; at+= sizeof(low))
    {
        if (mirisdr_read_mem(p, at, low, sizeof(low), 0) < 0) return -1;
        if (mirisdr_read_mem(p, (uint16_t) (0x2000 + at), mirror, sizeof(mirror), 0) < 0) return -1;

        if (memcmp(low, mirror, sizeof(low))) return 0;
    }

    return 1;
}

typedef struct
{
    uint8_t bus;
    uint8_t ports[8];
    int     nports;
    int     valid;
} mirisdr_fw_path_t;

static void mirisdr_fw_path_of (mirisdr_dev_t *p, mirisdr_fw_path_t *path)
{
    libusb_device *d;

    path->valid = 0;

    if (!p || !p->dh) return;
    if (!(d = libusb_get_device(p->dh))) return;

    path->bus = libusb_get_bus_number(d);
    path->nports = libusb_get_port_numbers(d, path->ports, sizeof(path->ports));
    path->valid = path->nports > 0;
}

static int mirisdr_fw_find (const mirisdr_fw_path_t *path, uint32_t fallback)
{
    libusb_context *ctx = NULL;
    libusb_device **list;
    ssize_t i, i_max;
    int found = -1, count = 0;

    if (libusb_init(&ctx) < 0) return -1;

    i_max = libusb_get_device_list(ctx, &list);

    for (i = 0; i < i_max; i++)
    {
        struct libusb_device_descriptor dd;

        libusb_get_device_descriptor(list[i], &dd);
        if (!mirisdr_device_get(dd.idVendor, dd.idProduct)) continue;

        if (path->valid)
        {
            uint8_t ports[8];
            int n = libusb_get_port_numbers(list[i], ports, sizeof(ports));

            if ((libusb_get_bus_number(list[i]) == path->bus) && (n == path->nports)
                && !memcmp(ports, path->ports, (size_t) n))
            {
                found = count;
                break;
            }
        }
        else if ((uint32_t) count == fallback)
        {
            found = count;
            break;
        }

        count++;
    }

    if (i_max >= 0) libusb_free_device_list(list, 1);
    libusb_exit(ctx);

    return found;
}

static int mirisdr_fw_wait (const mirisdr_fw_path_t *path, uint32_t fallback)
{
    int waited, index;

    for (waited = 0; waited < MIRISDR_FW_GONE_MS; waited+= 100)
    {
        if (mirisdr_fw_find(path, fallback) < 0) break;
        usleep(100000);
    }

    for (waited = 0; waited < MIRISDR_FW_WAIT_MS; waited+= 100)
    {
        if ((index = mirisdr_fw_find(path, fallback)) >= 0)
        {
            usleep(300000);

            return index;
        }

        usleep(100000);
    }

    return -1;
}

/* Find address of USB descriptor in the firmware to patch it */
static int mirisdr_fw_descriptor (const uint8_t *image, uint32_t size)
{
    int found = -1;
    uint32_t i;

    for (i = 8; (uint64_t) i + 4 <= size; i++)
    {
        if ((image[i] != 0xF7) || (image[i + 1] != 0x1D)) continue;         /* 1DF7 */
        if ((image[i + 2] != 0x00) || (image[i + 3] != 0x25)) continue;     /* 2500 */
        if ((image[i - 8] != 18) || (image[i - 7] != 1)) continue;          /* a descriptor */

        if (found >= 0) return -1;

        found = (int) i - 8;
    }

    return found;
}

static int mirisdr_fw_running (mirisdr_dev_t *p, const uint8_t *image, uint32_t size)
{
    uint8_t block[16];

    if (size < MIRISDR_FW_BLOCK + sizeof(block)) return 0;
    if (mirisdr_read_mem(p, MIRISDR_FW_BLOCK, block, sizeof(block), 0) < 0) return 0;

    return !memcmp(block, image + MIRISDR_FW_BLOCK, sizeof(block));
}

static int mirisdr_fw_set_ids (uint8_t *image, uint32_t size, uint16_t vid, uint16_t pid)
{
    int at = mirisdr_fw_descriptor(image, size);

    if (at < 0)
    {
        fprintf(stderr, "no 1DF7:2500 device descriptor in the firmware image\n");

        return -1;
    }

    image[at + 8] = (uint8_t) vid;
    image[at + 9] = (uint8_t) (vid >> 8);
    image[at + 10] = (uint8_t) pid;
    image[at + 11] = (uint8_t) (pid >> 8);

    return 0;
}

static uint8_t *mirisdr_fw_read (const char *path, uint32_t *size)
{
    uint8_t *image = NULL;
    long len;
    FILE *f = fopen(path, "rb");

    if (!f) goto failed;
    if (fseek(f, 0, SEEK_END)) goto failed;
    if ((len = ftell(f)) <= 0) goto failed;
    if (fseek(f, 0, SEEK_SET)) goto failed;

    if (len > 0x1800) goto failed;

    if (!(image = malloc((size_t) len))) goto failed;
    if (fread(image, 1, (size_t) len, f) != (size_t) len) goto failed;

    fclose(f);
    *size = (uint32_t) len;

    return image;

failed:
    if (image) free(image);
    if (f) fclose(f);
    fprintf(stderr, "cannot read firmware %s\n", path);

    return NULL;
}

static int mirisdr_fw_ids_ok (mirisdr_dev_t *p, const mirisdr_open_config_t *cfg)
{
    struct libusb_device_descriptor dd;
    libusb_device *d;

    if (cfg->firmware_ids != MIRISDR_FW_IDS_SET) return 1;

    if (!(d = libusb_get_device(p->dh))) return 0;
    if (libusb_get_device_descriptor(d, &dd) < 0) return 0;

    return (dd.idVendor == cfg->firmware_vid) && (dd.idProduct == cfg->firmware_pid);
}

static int mirisdr_fw_open (mirisdr_dev_t **dev, const mirisdr_open_config_t *cfg, uint32_t at)
{
    if (cfg->fd >= 0) return mirisdr_open_fd_raw(dev, cfg->fd);

    return mirisdr_open_raw(dev, at);
}

static int mirisdr_fw_recycle (mirisdr_dev_t **dev, const mirisdr_open_config_t *cfg,
                               mirisdr_fw_path_t *path, uint32_t *at, uint32_t index)
{
    int again;

    mirisdr_close(*dev);
    *dev = NULL;

    if (cfg->fd >= 0) return MIRISDR_REOPEN;

    if ((again = mirisdr_fw_wait(path, index)) < 0) return -1;
    *at = (uint32_t) again;

    return mirisdr_fw_open(dev, cfg, *at);
}

int mirisdr_open_ex (mirisdr_dev_t **out, const mirisdr_open_config_t *cfg)
{
    const uint8_t *image;
    uint8_t *owned = NULL, *work = NULL;
    mirisdr_dev_t *dev = NULL;
    mirisdr_fw_path_t path;
    uint32_t index, size, at;
    int r = -1;

    if (!out || !cfg) return -1;

    index = at = cfg->index;
    path.valid = 0;

    image = cfg->firmware;
    size = cfg->firmware_size;

    if (!image && cfg->firmware_path && !cfg->keep_running)
    {
        if (!(owned = mirisdr_fw_read(cfg->firmware_path, &size))) return -1;
        image = owned;
    }

    if (image && size)
    {
        if (!(work = malloc(size))) goto out;
        memcpy(work, image, size);

        if ((cfg->firmware_ids == MIRISDR_FW_IDS_SET)
            && (mirisdr_fw_set_ids(work, size, cfg->firmware_vid, cfg->firmware_pid) < 0))
            goto out;
    }

    if (mirisdr_fw_open(&dev, cfg, at) < 0) goto out;

    if (cfg->fd < 0) mirisdr_fw_path_of(dev, &path);

    if (cfg->keep_running || !image || !size
        || (mirisdr_fw_running(dev, work, size) && mirisdr_fw_ids_ok(dev, cfg)))
    {
        *out = dev;
        dev = NULL;
        r = 0;

        goto out;
    }

    if (mirisdr_running_from_rom(dev) == 0)
    {
        if (mirisdr_reboot(dev, 0) != MIRISDR_REOPEN) goto out;

        if ((r = mirisdr_fw_recycle(&dev, cfg, &path, &at, index))) goto out;

        /* Booting to the ROM landed on firmware, so the ROM handed off to an
           image in the SPI flash.  RAM cannot be written under it, but what
           booted is usable, so hand that back instead of failing. */
        if (mirisdr_running_from_rom(dev) != 1)
        {
            fprintf(stderr, "not loading: the device boots firmware from its SPI flash\n");

            *out = dev;
            dev = NULL;
            r = 0;

            goto out;
        }
    }

    if (cfg->firmware_ids == MIRISDR_FW_IDS_DEVICE)
    {
        struct libusb_device_descriptor dd;
        libusb_device *d = libusb_get_device(dev->dh);

        if (!d || (libusb_get_device_descriptor(d, &dd) < 0)
            || (mirisdr_fw_set_ids(work, size, dd.idVendor, dd.idProduct) < 0))
        {
            r = -1;

            goto out;
        }
    }

    if (mirisdr_write_mem(dev, 0x0000, work, (int) size, 0) < 0)
    {
        r = -1;

        goto out;
    }

    if (mirisdr_reboot(dev, 1) != MIRISDR_REOPEN) goto out;

    if ((r = mirisdr_fw_recycle(&dev, cfg, &path, &at, index))) goto out;

    if (mirisdr_fw_running(dev, work, size) && mirisdr_fw_ids_ok(dev, cfg))
    {
        *out = dev;
        dev = NULL;
        r = 0;

        goto out;
    }

    fprintf(stderr, "the firmware did not come up\n");
    r = -1;

out:
    if (dev) mirisdr_close(dev);
    if (work) free(work);
    if (owned) free(owned);

    return r;
}
