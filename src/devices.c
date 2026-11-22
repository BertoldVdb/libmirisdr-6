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

static mirisdr_device_t mirisdr_devices[] = {
    { 0x1df7, 0x2500, "Mirics MSi2500 default (e.g. VTX3D card, SDRplay RSP1)", "Mirics", "MSi2500", MIRISDR_HW_DEFAULT, 0, 0, 0},
    { 0x1df7, 0x3000, "SDRplay RSP1A", "SDRPlay", "RSP1A", MIRISDR_HW_RSP1A, 0, 0, 0},
    { 0x1df7, 0x3010, "SDRplay RSP2/RSP2pro", "SDRPlay", "RSP2", MIRISDR_HW_RSP2, 0, 0, 0},
    { 0x1df7, 0x3050, "SDRplay RSP1B", "SDRPlay", "RSP1B", MIRISDR_HW_RSP1B, 0, 0, 0},
    { 0x1df7, 0x3060, "SDRplay RSPdx-R2", "SDRPlay", "RSPdxR2", MIRISDR_HW_RSPDXR2, 0, 0, 0},
    { 0x1df7, 0x3030, "SDRplay RSPdx", "SDRPlay", "RSPdx", MIRISDR_HW_RSPDXR2, 0, 0, 32},
    { 0x1df7, 0x3040, "SDRplay nRSP-ST", "SDRPlay", "nRSP-ST", MIRISDR_HW_RSPDXR2, 1, 10, 0},
    { 0x1df7, 0x3020, "SDRplay RSPduo", "SDRPlay", "RSPduo", MIRISDR_HW_RSPDUO, 0, 0, 0},
    { 0x16d0, 0x158c, "BertoldVdb MSi2500 Acquisition Board", "BertoldVdb", "MSI2500-PPS", MIRISDR_HW_DEFAULT, 0, 0, 0},
    { 0x2040, 0xd300, "Hauppauge WinTV 133559 LF", "Hauppauge", "WinTV 133559 LF", MIRISDR_HW_MIRICSTV, 0, 0, 0},
    { 0x07ca, 0x8591, "AverMedia A859 Pure DVBT", "AverTV", "A859 Pure DVBT", MIRISDR_HW_MIRICSTV, 0, 0, 0},
    { 0x04bb, 0x0537, "IO-DATA GV-TV100 stick", "IO-DATA", "GV-TV100", MIRISDR_HW_MIRICSTV, 0, 0, 0},
    { 0x0511, 0x0037, "Logitec LDT-1S310U/J", "Logitec", "LDT-1S310U/J", MIRISDR_HW_MIRICSTV, 0, 0, 0}
};

static mirisdr_device_t *mirisdr_device_get (uint16_t vid, uint16_t pid) {
    size_t i;

    for (i = 0; i < sizeof(mirisdr_devices) / sizeof(mirisdr_device_t); i++) {
        if ((mirisdr_devices[i].vid == vid) && (mirisdr_devices[i].pid == pid)) return &mirisdr_devices[i];
    }

    return NULL;
}

int mirisdr_device_known (uint16_t vid, uint16_t pid) {
    return mirisdr_device_get(vid, pid) != NULL;
}

#define MIRISDR_OWN_VID         0x16d0
#define MIRISDR_OWN_PID         0x158c
/* our ids hide the board, so on an SDRplay the serial starts with its letter: the board's
   own ids by that letter */
static const struct { char prefix[3]; uint16_t pid; } mirisdr_own_boards[] = {
    { "B-", 0x3050 },   /* RSP1B */
    { "C-", 0x3060 },   /* RSPdx-R2 */
    { "D-", 0x3020 },   /* RSPduo */
    { "E-", 0x3010 },   /* RSP2 */
    { "F-", 0x3040 },   /* nRSP-ST */
    { "G-", 0x3030 },   /* RSPdx */
    { "H-", 0x3000 },   /* RSP1A */
};

/* the table entry of the board behind these ids, under our own ids by the serial */
static mirisdr_device_t *mirisdr_board_get (uint16_t vid, uint16_t pid, const char *serial)
{
    size_t i;

    if ((vid == MIRISDR_OWN_VID) && (pid == MIRISDR_OWN_PID) && serial)
    {
        for (i = 0; i < sizeof(mirisdr_own_boards) / sizeof(mirisdr_own_boards[0]); i++)
        {
            if (!strncmp(serial, mirisdr_own_boards[i].prefix, 2))
                return mirisdr_device_get(0x1df7, mirisdr_own_boards[i].pid);
        }
    }

    return mirisdr_device_get(vid, pid);
}

/* the board behind an open handle */
static mirisdr_device_t *mirisdr_board_of (libusb_device_handle *dh)
{
    struct libusb_device_descriptor dd;
    unsigned char serial[MIRISDR_FW_SERIAL_MAX + 1];

    if (libusb_get_device_descriptor(libusb_get_device(dh), &dd) < 0) return NULL;

    /* the serial only matters under our own ids */
    serial[0] = 0;
    if ((dd.idVendor == MIRISDR_OWN_VID) && (dd.idProduct == MIRISDR_OWN_PID) && dd.iSerialNumber
        && (libusb_get_string_descriptor_ascii(dh, dd.iSerialNumber, serial, sizeof serial) < 2))
        serial[0] = 0;

    return mirisdr_board_get(dd.idVendor, dd.idProduct, (const char *) serial);
}

static int mirisdr_flavour_of (libusb_device_handle *dh)
{
    mirisdr_device_t *d = mirisdr_board_of(dh);

    return d ? d->flavour : MIRISDR_HW_DEFAULT;
}

/* what sets the board apart within its flavour: the RSPdx's direct sampling amplifier */
static void mirisdr_board_parts (mirisdr_dev_t *p)
{
    mirisdr_device_t *d = mirisdr_board_of(p->dh);

    p->ds_lna_db = d ? d->ds_lna_db : 0;
}

const char *mirisdr_get_board_name (mirisdr_dev_t *p)
{
    mirisdr_device_t *d;

    if (!p || !p->dh) return NULL;
    d = mirisdr_board_of(p->dh);

    return d ? d->product : NULL;
}

/* the direct sampling amplifier's gain, dB */
static int mirisdr_ds_lna_db (const mirisdr_dev_t *p)
{
    return p->ds_lna_db ? p->ds_lna_db : MIRISDR_DS_LNA_DB;
}

/* Interface 0 tells the programs on a receiver apart: whoever holds it has the receiver, and
   nothing is sent to a receiver before it is held. The system refuses a second holder, though
   not what a program sends without holding it, so the rule is what makes it a lock. Another
   program holds it for a moment while it reads a serial from the EEPROM (mirisdr_probe_serial),
   so a busy one is waited for a little while: an open far longer than such a read takes, a
   listing only long enough for another read to end. Asked again every few ms, well within the
   5 to 10 ms a read takes: with several programs reading in turn, one asking less often could
   miss every gap between them */
#define MIRISDR_CLAIM_WAIT_MS   500
#define MIRISDR_PROBE_WAIT_MS   300
#define MIRISDR_CLAIM_STEP_MS   2

/* held by another program. Windows gives the device to one program at a time and refuses the
   others access, at the open already */
static int mirisdr_usb_busy (int r)
{
#ifdef _WIN32
    if (r == LIBUSB_ERROR_ACCESS) return 1;
#endif
    return r == LIBUSB_ERROR_BUSY;
}

static int mirisdr_open_wait (libusb_device *d, libusb_device_handle **dh, int wait_ms)
{
    int r, waited;

    for (waited = 0; ; waited += MIRISDR_CLAIM_STEP_MS)
    {
        r = libusb_open(d, dh);
        if (!mirisdr_usb_busy(r) || (waited >= wait_ms)) return r;
        usleep(MIRISDR_CLAIM_STEP_MS * 1000);
    }
}

static int mirisdr_claim_wait (libusb_device_handle *dh, int wait_ms)
{
    int r, waited;

    for (waited = 0; ; waited += MIRISDR_CLAIM_STEP_MS)
    {
        r = libusb_claim_interface(dh, 0);
        if (!mirisdr_usb_busy(r) || (waited >= wait_ms)) return r;
        usleep(MIRISDR_CLAIM_STEP_MS * 1000);
    }
}

/* the USB devices, or -1 with nothing to free when libusb cannot start or list them */
static ssize_t mirisdr_list_devices (libusb_context **ctx, libusb_device ***list)
{
    ssize_t n;

#ifdef __ANDROID__
    /* LibUSB does not support device discovery on android */
    libusb_set_option(NULL, LIBUSB_OPTION_NO_DEVICE_DISCOVERY, NULL);
#endif

    if (libusb_init(ctx) < 0) return -1;

    if ((n = libusb_get_device_list(*ctx, list)) < 0)
    {
        libusb_exit(*ctx);
        return -1;
    }

    return n;
}

/* the number of devices available */
uint32_t mirisdr_get_device_count (void) {
    ssize_t i, i_max;
    uint32_t ret = 0;
    libusb_context *ctx;
    libusb_device **list;
    struct libusb_device_descriptor dd;

    if ((i_max = mirisdr_list_devices(&ctx, &list)) < 0) return 0;

    for (i = 0; i < i_max; i++) {
        if (libusb_get_device_descriptor(list[i], &dd) < 0) continue;

        if (mirisdr_device_get(dd.idVendor, dd.idProduct)) ret++;
    }

    libusb_free_device_list(list, 1);

    libusb_exit(ctx);

    return ret;
}

/* the device name */
const char *mirisdr_get_device_name (uint32_t index) {
    ssize_t i, i_max;
    size_t j = 0;
    libusb_context *ctx;
    libusb_device **list;
    struct libusb_device_descriptor dd;
    mirisdr_device_t *device = NULL;

    if ((i_max = mirisdr_list_devices(&ctx, &list)) < 0) return "";

    for (i = 0; i < i_max; i++) {
        if (libusb_get_device_descriptor(list[i], &dd) < 0) continue;

        if ((device = mirisdr_device_get(dd.idVendor, dd.idProduct)) &&
            (j++ == index)) {
            libusb_free_device_list(list, 1);
            libusb_exit(ctx);
            return device->name;
        }
    }

    libusb_free_device_list(list, 1);
    libusb_exit(ctx);

    return "";
}

static int mirisdr_usb_string (libusb_device_handle *dh, uint8_t at, char *out, int len)
{
    if (!at || !dh) return 0;

    return libusb_get_string_descriptor_ascii(dh, at, (unsigned char *) out, len) > 0;
}

static int mirisdr_device_serial_cached (libusb_device *d, char *out, int len)
{
#if LIBUSBX_API_VERSION >= 0x0100010C
    if (libusb_get_device_string(d, LIBUSB_DEVICE_STRING_SERIAL_NUMBER, out, len) > 1) return 1;
#endif

#if defined(__linux__) && (LIBUSBX_API_VERSION >= 0x01000102)
    {
        uint8_t ports[7];
        char path[96];
        int n, i, at;
        size_t got;
        FILE *f;

        if ((n = libusb_get_port_numbers(d, ports, sizeof(ports))) <= 0) return 0;

        at = snprintf(path, sizeof path, "/sys/bus/usb/devices/%u", libusb_get_bus_number(d));
        for (i = 0; i < n; i++)
            at+= snprintf(path + at, sizeof path - at, i ? ".%u" : "-%u", ports[i]);
        snprintf(path + at, sizeof path - at, "/serial");

        if (!(f = fopen(path, "r"))) return 0;
        got = fgets(out, len, f) ? strcspn(out, "\n") : 0;
        fclose(f);

        out[got] = 0;
        return got > 0;
    }
#else
    (void) d; (void) out; (void) len;
    return 0;
#endif
}

static int mirisdr_device_serial (libusb_device *d, uint8_t at, char *out, int len)
{
    libusb_device_handle *dh;
    int got;

    if (!at) return 0;
    if (mirisdr_device_serial_cached(d, out, len)) return 1;
    if (libusb_open(d, &dh)) return 0;

    got = mirisdr_usb_string(dh, at, out, len);
    libusb_close(dh);

    return got;
}

// because all known SDRplay clones do not have an EEPROM chip with the serial number,
// use the physical location of the USB port
static void mirisdr_port_name (libusb_device *d, char *serial)
{
    char *cursor = serial;

    cursor+= sprintf(cursor, "%d:", libusb_get_bus_number(d));

#if LIBUSBX_API_VERSION >= 0x01000102
    {
        uint8_t usb_path[16];
        int path_len = libusb_get_port_numbers(d, usb_path, sizeof(usb_path));
        int u;

        if (path_len == LIBUSB_ERROR_OVERFLOW) path_len = sizeof(usb_path); // array too small

        for (u = 0; u < path_len; u++) cursor+= sprintf(cursor, "%d.", usb_path[u]);
    }
#endif

    *(cursor - 1) = '\0'; // remove last dot or :
}

/* our own implementation */
int mirisdr_get_device_usb_strings (uint32_t index, char *manufact, char *product, char *serial) {
    ssize_t i, i_max;
    size_t j = 0;
    libusb_context *ctx;
    libusb_device **list;
    struct libusb_device_descriptor dd;
    mirisdr_device_t *device = NULL;

    if (!manufact || !product) return -1;

    if ((i_max = mirisdr_list_devices(&ctx, &list)) < 0)
    {
        memset(manufact, 0, 256);
        memset(product, 0, 256);
        if (serial) memset(serial, 0, 256);

        return -1;
    }

    for (i = 0; i < i_max; i++) {
        if (libusb_get_device_descriptor(list[i], &dd) < 0) continue;

        if ((device = mirisdr_device_get(dd.idVendor, dd.idProduct)) &&
            (j++ == index)) {
            strcpy(manufact, device->manufacturer);
            strcpy(product, device->product);

            /* the serial is the only one that opens the device */
            if (serial && !mirisdr_device_serial(list[i], dd.iSerialNumber, serial, 256))
                mirisdr_port_name(list[i], serial);

            libusb_free_device_list(list, 1);
            libusb_exit(ctx);
            return 0;
        }
    }

    memset(manufact, 0, 256);
    memset(product, 0, 256);
    if (serial) memset(serial, 0, 256);

    libusb_free_device_list(list, 1);
    libusb_exit(ctx);

    return -1;
}

int mirisdr_get_device_port (uint32_t index, char *port, int port_len, char *serial, int serial_len)
{
    ssize_t i, i_max;
    size_t j = 0;
    libusb_context *ctx;
    libusb_device **list;
    struct libusb_device_descriptor dd;
    char name[256];
    int ret = -1;

    if (!port || (port_len <= 0) || (serial && (serial_len <= 0))) return -1;

    *port = 0;
    if (serial) *serial = 0;

    if ((i_max = mirisdr_list_devices(&ctx, &list)) < 0) return -1;

    for (i = 0; i < i_max; i++) {
        if (libusb_get_device_descriptor(list[i], &dd) < 0) continue;

        if (mirisdr_device_get(dd.idVendor, dd.idProduct) && (j++ == index)) {
            mirisdr_port_name(list[i], name);
            if ((int) strlen(name) < port_len) {
                strcpy(port, name);
                ret = 0;
            }
            if (serial && !mirisdr_device_serial(list[i], dd.iSerialNumber, serial, serial_len))
                *serial = 0;
            break;
        }
    }

    libusb_free_device_list(list, 1);
    libusb_exit(ctx);

    return ret;
}

static int mirisdr_pick_is (libusb_device *d, const struct libusb_device_descriptor *dd,
                            int own, const char *want)
{
    char have[256];

    if (!own) mirisdr_port_name(d, have);
    else if (!mirisdr_device_serial(d, dd->iSerialNumber, have, sizeof have)) *have = 0;

    return *have && !strcmp(have, want);
}

/* The serial in the EEPROM of a receiver running from ROM, see firmware.c */
static int mirisdr_probe_serial (libusb_device *d, const struct libusb_device_descriptor *dd, int wait_ms,
                                 char *out, int len);

/* The receiver an open or a lookup takes from one device list. In turn: the one in the port
   given as serial, the one declaring the serial, with probe the one running from ROM whose
   EEPROM holds the serial (the one in usb_port first: mostly it is that one, and the others
   are then not opened), and last the one in usb_port. probe is how long, in ms, a receiver
   held elsewhere is waited for, 0 for no reading of EEPROMs. probed (may be NULL,
   MIRISDR_FW_SERIAL_MAX + 1 bytes) gets the serial when the EEPROM gave it, else it is left
   empty */
static libusb_device *mirisdr_pick (libusb_device **list, ssize_t n, uint32_t index,
                                    const char *serial, const char *usb_port, int probe, char *probed, int *at)
{
    struct libusb_device_descriptor dd;
    char own[256];
    ssize_t i;
    int j, pass, hit, here, other = 0;

    if (probed) *probed = 0;

    /* a serial longer than the EEPROM read gives is in none */
    if (serial && (strlen(serial) > MIRISDR_FW_SERIAL_MAX)) probe = 0;

    for (pass = 0; pass < 5; pass++) {
        if ((pass < 4) ? !serial : !usb_port) continue;
        if (((pass == 2) || (pass == 3)) && !probe) continue;
        if ((pass == 2) && !usb_port) continue;

        for (i = 0, j = 0; i < n; i++) {
            if (libusb_get_device_descriptor(list[i], &dd) < 0) continue;
            if (!mirisdr_device_get(dd.idVendor, dd.idProduct)) continue;

            if (pass < 2) hit = mirisdr_pick_is(list[i], &dd, pass, serial);
            else if (pass < 4) {
                /* pass 2 the receiver in usb_port, pass 3 the others */
                here = usb_port && mirisdr_pick_is(list[i], &dd, 0, usb_port);
                hit = 0;
                if (((pass == 2) ? here : !here) && mirisdr_probe_serial(list[i], &dd, probe, own, sizeof own)) {
                    hit = !strcmp(own, serial);
                    /* the one in usb_port is another receiver: not taken for its port below */
                    if (!hit && here) other = 1;
                    if (hit && probed) strcpy(probed, own);
                }
            }
            else hit = !other && mirisdr_pick_is(list[i], &dd, 0, usb_port)
                       && (!serial || !mirisdr_device_serial(list[i], dd.iSerialNumber, own, sizeof own));

            if (hit) {
                if (at) *at = j;
                return list[i];
            }

            j++;
        }
    }

    if (serial || usb_port) return NULL;

    for (i = 0, j = 0; i < n; i++) {
        if (libusb_get_device_descriptor(list[i], &dd) < 0) continue;
        if (!mirisdr_device_get(dd.idVendor, dd.idProduct)) continue;

        if ((uint32_t) j == index) {
            if (at) *at = j;
            return list[i];
        }

        j++;
    }

    return NULL;
}

int mirisdr_get_index_by_serial_port (const char *serial, const char *usb_port)
{
    ssize_t i_max;
    int found = -1;
    libusb_context *ctx;
    libusb_device **list;

    if (!serial && !usb_port) return -1;

    if ((i_max = mirisdr_list_devices(&ctx, &list)) < 0) return -1;

    if (!mirisdr_pick(list, i_max, 0, serial, usb_port, 0, NULL, &found)) found = -1;

    libusb_free_device_list(list, 1);
    libusb_exit(ctx);

    return found;
}

/* one receiver's entry. probe: a serial it does not declare is read from its EEPROM, or
   taken from probed when the pick has read it already */
static void mirisdr_receiver_of (libusb_device *d, const struct libusb_device_descriptor *dd, int probe,
                                 const char *probed, mirisdr_receiver_t *r)
{
    mirisdr_device_t *board;
    char port[256], own[MIRISDR_FW_SERIAL_MAX + 1];

    memset(r, 0, sizeof *r);
    r->vid = dd->idVendor;
    r->pid = dd->idProduct;
    if (!mirisdr_device_serial(d, dd->iSerialNumber, r->serial, sizeof r->serial))
    {
        *r->serial = 0;
        *own = 0;
        if (probed && *probed) snprintf(own, sizeof own, "%s", probed);
        else if (probe && !mirisdr_probe_serial(d, dd, probe, own, sizeof own)) *own = 0;
        if (*own)
        {
            strcpy(r->serial, own);
            r->serial_from_eeprom = 1;
        }
    }
    mirisdr_port_name(d, port);
    if (strlen(port) < sizeof r->usb_port) strcpy(r->usb_port, port);
    if ((board = mirisdr_board_get(dd->idVendor, dd->idProduct, r->serial)))
    {
        r->manufacturer = board->manufacturer;
        r->product = board->product;
        r->name = board->name;
        r->hw_flavour = board->flavour;
    }
}

void mirisdr_list_config_default (mirisdr_list_config_t *cfg)
{
    if (cfg) memset(cfg, 0, sizeof *cfg);
}

int mirisdr_list_receivers_ex (const mirisdr_list_config_t *cfg, mirisdr_receiver_t *out, int max)
{
    struct libusb_device_descriptor dd;
    libusb_context *ctx;
    libusb_device **list, *one;
    char probed[MIRISDR_FW_SERIAL_MAX + 1];
    ssize_t i, i_max;
    int n = 0, probe;

    if (!cfg || (max < 0) || (max && !out)) return -1;
    if (MIRISDR_RESERVED_SET(*cfg))
    {
        fprintf(stderr, "libmirisdr: " MIRISDR_RESERVED_MSG, "the list config");
        return -1;
    }
    if (cfg->flags & ~MIRISDR_LIST_FLAGS)
    {
        fprintf(stderr, "libmirisdr: " MIRISDR_FLAGS_MSG, "the list config", cfg->flags & ~MIRISDR_LIST_FLAGS);
        return -1;
    }
    probe = (cfg->flags & MIRISDR_LIST_PROBE_SERIAL) ? MIRISDR_PROBE_WAIT_MS : 0;

    if ((i_max = mirisdr_list_devices(&ctx, &list)) < 0) return -1;

    /* asked for: the one the open takes, by its rules */
    if (cfg->serial || cfg->usb_port)
    {
        if ((one = mirisdr_pick(list, i_max, 0, cfg->serial, cfg->usb_port, probe, probed, NULL))
            && (libusb_get_device_descriptor(one, &dd) == 0))
        {
            if (max) mirisdr_receiver_of(one, &dd, probe, probed, out);
            n = 1;
        }
    }
    else for (i = 0; i < i_max; i++)
    {
        if (libusb_get_device_descriptor(list[i], &dd) < 0) continue;
        if (!mirisdr_device_get(dd.idVendor, dd.idProduct)) continue;
        if (n < max) mirisdr_receiver_of(list[i], &dd, probe, NULL, &out[n]);
        n++;
    }

    libusb_free_device_list(list, 1);
    libusb_exit(ctx);

    return n;
}

int mirisdr_list_receivers (const char *serial, const char *usb_port, mirisdr_receiver_t *out, int max)
{
    mirisdr_list_config_t cfg;

    mirisdr_list_config_default(&cfg);
    cfg.serial = serial;
    cfg.usb_port = usb_port;

    return mirisdr_list_receivers_ex(&cfg, out, max);
}

int mirisdr_get_index_by_serial (const char *serial)
{
    return serial ? mirisdr_get_index_by_serial_port(serial, NULL) : -1;
}
