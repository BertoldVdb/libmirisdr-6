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
    { 0x1df7, 0x2500, "Mirics MSi2500 default (e.g. VTX3D card)", "Mirics", "MSi2500"},
    { 0x1df7, 0x3000, "SDRplay RSP1A", "SDRPlay", "RSP1A"},
    { 0x1df7, 0x3010, "SDRplay RSP1A", "SDRPlay", "RSP2"},
    { 0x16d0, 0x158c, "BertoldVdb MSi2500 Acquisition Board", "BertoldVdb", "MSI2500-PPS"},
    { 0x2040, 0xd300, "Hauppauge WinTV 133559 LF", "Hauppauge", "WinTV 133559 LF"},
    { 0x07ca, 0x8591, "AverMedia A859 Pure DVBT", "AverTV", "A859 Pure DVBT"},
    { 0x04bb, 0x0537, "IO-DATA GV-TV100 stick", "IO-DATA", "GV-TV100"},
    { 0x0511, 0x0037, "Logitec LDT-1S310U/J", "Logitec", "LDT-1S310U/J"}
};

static mirisdr_device_t *mirisdr_device_get (uint16_t vid, uint16_t pid) {
    size_t i;

    for (i = 0; i < sizeof(mirisdr_devices) / sizeof(mirisdr_device_t); i++) {
        if ((mirisdr_devices[i].vid == vid) && (mirisdr_devices[i].pid == pid)) return &mirisdr_devices[i];
    }

    return NULL;
}

/* počet dostupných zařízení */
uint32_t mirisdr_get_device_count (void) {
    ssize_t i, i_max;
    uint32_t ret = 0;
    libusb_context *ctx;
    libusb_device **list;
    struct libusb_device_descriptor dd;

#ifdef __ANDROID__
    /* LibUSB does not support device discovery on android */
    libusb_set_option(NULL, LIBUSB_OPTION_NO_DEVICE_DISCOVERY, NULL);
#endif

    libusb_init(&ctx);

    i_max = libusb_get_device_list(ctx, &list);

    for (i = 0; i < i_max; i++) {
        libusb_get_device_descriptor(list[i], &dd);

        if (mirisdr_device_get(dd.idVendor, dd.idProduct)) ret++;
    }

    libusb_free_device_list(list, 1);

    libusb_exit(ctx);

    return ret;
}

/* název zařízení */
const char *mirisdr_get_device_name (uint32_t index) {
    ssize_t i, i_max;
    size_t j = 0;
    libusb_context *ctx;
    libusb_device **list;
    struct libusb_device_descriptor dd;
    mirisdr_device_t *device = NULL;

#ifdef __ANDROID__
    /* LibUSB does not support device discovery on android */
    libusb_set_option(NULL, LIBUSB_OPTION_NO_DEVICE_DISCOVERY, NULL);
#endif

    libusb_init(&ctx);
    i_max = libusb_get_device_list(ctx, &list);

    for (i = 0; i < i_max; i++) {
        libusb_get_device_descriptor(list[i], &dd);

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

static int mirisdr_device_string (libusb_device *d, uint8_t at, char *out, int len)
{
    libusb_device_handle *dh;
    int got;

    if (!at || libusb_open(d, &dh)) return 0;

    got = mirisdr_usb_string(dh, at, out, len);
    libusb_close(dh);

    return got;
}

/* Without one, the port it is plugged into is the only stable name it has. */
static void mirisdr_port_name (libusb_device *d, char *serial)
{
    char *cursor = serial;

    cursor+= sprintf(cursor, "%d:", libusb_get_bus_number(d));

#if LIBUSBX_API_VERSION >= 0x01000102
    {
        uint8_t usb_path[16];
        int path_len = libusb_get_port_numbers(d, usb_path, sizeof(usb_path));
        int u;

        if (path_len == LIBUSB_ERROR_OVERFLOW) path_len = sizeof(usb_path);

        for (u = 0; u < path_len; u++) cursor+= sprintf(cursor, "%d.", usb_path[u]);
    }
#endif

    *(cursor - 1) = '\0';
}

/* vlastní implementace */
int mirisdr_get_device_usb_strings (uint32_t index, char *manufact, char *product, char *serial) {
    ssize_t i, i_max;
    size_t j = 0;
    libusb_context *ctx;
    libusb_device **list;
    struct libusb_device_descriptor dd;
    mirisdr_device_t *device = NULL;

#ifdef __ANDROID__
    /* LibUSB does not support device discovery on android */
    libusb_set_option(NULL, LIBUSB_OPTION_NO_DEVICE_DISCOVERY, NULL);
#endif

    libusb_init(&ctx);
    i_max = libusb_get_device_list(ctx, &list);

    for (i = 0; i < i_max; i++) {
        libusb_get_device_descriptor(list[i], &dd);

        if ((device = mirisdr_device_get(dd.idVendor, dd.idProduct)) &&
            (j++ == index)) {
            strcpy(manufact, device->manufacturer);
            strcpy(product, device->product);

            if (!mirisdr_device_string(list[i], dd.iSerialNumber, serial, 256))
                mirisdr_port_name(list[i], serial);

            libusb_free_device_list(list, 1);
            libusb_exit(ctx);
            return 0;
        }
    }

    memset(manufact, 0, 256);
    memset(product, 0, 256);
    memset(serial, 0, 256);

    libusb_free_device_list(list, 1);
    libusb_exit(ctx);

    return -1;
}

int mirisdr_get_index_by_serial (const char *serial)
{
    char manufact[256], product[256], have[256];
    uint32_t i, i_max;

    if (!serial) return -1;

    i_max = mirisdr_get_device_count();

    for (i = 0; i < i_max; i++)
    {
        if (mirisdr_get_device_usb_strings(i, manufact, product, have)) continue;
        if (!strcmp(have, serial)) return (int) i;
    }

    return -2;
}
