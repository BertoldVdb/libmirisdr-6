/*
 * miri_test - exercise the library against real hardware.
 *
 * gcc -O2 -o miri_test miri_test.c -lmirisdr -lpthread -lm
 */

#include <mirisdr.h>

#include <math.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* ------------------------------------------------------------------ */
/* harness                                                             */
/* ------------------------------------------------------------------ */

typedef enum { T_PASS, T_FAIL, T_SKIP } tres_t;

static mirisdr_dev_t   *dev;
static int              fw_ours;
static int              from_rom;
static char             detail[512];

static int  opt_io;
static int  opt_eeprom;
static int  opt_eeprom_write;
static int  opt_pps;
static int  opt_chars;
static double opt_step = 1.0;
static int  opt_verbose;
static const char *opt_only;
static const char *opt_fw;
static int         opt_rom;

static int  n_pass, n_fail, n_skip;

static void say (const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(detail, sizeof detail, fmt, ap);
    va_end(ap);
}

static void note (const char *fmt, ...)
{
    va_list ap;

    if (!opt_verbose) return;

    printf("        ");
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
}

static double now (void)
{
    struct timespec t;

    clock_gettime(CLOCK_MONOTONIC, &t);

    return t.tv_sec + t.tv_nsec / 1e9;
}

/* ------------------------------------------------------------------ */
/* streaming helpers                                                   */
/* ------------------------------------------------------------------ */

static pthread_t        pump_thread;
static int              pump_running;
static volatile int     pump_bytes;
static volatile int     pump_result;   /* what read_async returned: -1 means it
                                          gave up on its transfers */

/* the last buffer's converters, and buffers whose position went back */
static volatile int     pump_adc = -1, pump_backwards, pump_infos;
static uint64_t         pump_sample, pump_index;
static volatile int64_t pump_ahead;    /* stats.samples past the end of the last buffer */

static void stream_cb (unsigned char *buf, uint32_t len, void *ctx)
{
    mirisdr_buffer_info_t in;

    (void) buf; (void) ctx;
    pump_bytes += (int) len;

    if (mirisdr_get_buffer_info(dev, &in) < 0) return;

    if (pump_infos && ((in.sample <= pump_sample) || (in.index < pump_index))) pump_backwards++;
    {
        mirisdr_stream_stats_t st;
        uint32_t ub = (in.type == MIRISDR_SAMPLE_S8 ? 1 : 2) * (in.adc == MIRISDR_IQ_BOTH ? 2 : 1);

        if (mirisdr_get_stream_stats(dev, &st) == 0) pump_ahead = (int64_t) (st.samples - (in.sample + len / ub));
    }
    pump_sample = in.sample;
    pump_index = in.index;
    pump_adc = in.adc;
    pump_infos++;
}

static void *pump_main (void *arg)
{
    (void) arg;
    pump_result = mirisdr_read_async(dev, stream_cb, NULL, 8, 65536);
    return NULL;
}

static int pump_start (void)
{
    if (pump_running) return 0;

    pump_bytes = 0;
    pump_result = 0;
    pump_adc = -1;
    pump_backwards = 0;
    pump_infos = 0;

    if (mirisdr_reset_buffer(dev) < 0) return -1;
    if (pthread_create(&pump_thread, NULL, pump_main, NULL)) return -1;

    pump_running = 1;
    usleep(400000);

    return 0;
}

static void pump_stop (void)
{
    if (!pump_running) return;

    mirisdr_cancel_async(dev);
    pthread_join(pump_thread, NULL);
    pump_running = 0;
}

/* samples per second over a window, plus what the stream reported */
static double stream_rate (double seconds, mirisdr_stream_stats_t *delta)
{
    mirisdr_stream_stats_t a, b;
    double t0, t1;

    mirisdr_get_stream_stats(dev, &a);
    t0 = now();
    usleep((useconds_t) (seconds * 1e6));
    mirisdr_get_stream_stats(dev, &b);
    t1 = now();

    if (delta) {
        delta->samples = b.samples - a.samples;
        delta->lost    = b.lost    - a.lost;
        delta->gaps    = b.gaps    - a.gaps;
        delta->jitter  = b.jitter  - a.jitter;
        delta->resyncs = b.resyncs - a.resyncs;
        delta->index   = b.index   - a.index;
    }

    return (double) (b.index - a.index) / (t1 - t0);
}

/* Discard one window before measuring: index counts lost samples too, so a
   restart still settling reads as a wildly high rate. */
static double stream_rate_settled (double seconds, mirisdr_stream_stats_t *delta)
{
    stream_rate(0.4, NULL);

    return stream_rate(seconds, delta);
}

static int within (double got, double want, double frac)
{
    return (got >= want * (1.0 - frac)) && (got <= want * (1.0 + frac));
}

/* A test that upsets the device must not make every later one fail, so the
   handle is checked between tests and reopened if it has gone. */
static const char *open_serial;

static int device_healthy (void)
{
    uint8_t rd[4];

    return mirisdr_read_reg(dev, 0, rd, sizeof rd) == (int) sizeof rd;
}

static int device_open (void)
{
    mirisdr_open_config_t cfg;

    mirisdr_open_config_default(&cfg);
    cfg.serial = open_serial;

    if (opt_rom) {
        cfg.firmware = NULL;
        cfg.firmware_size = 0;
        cfg.firmware_path = NULL;
        cfg.keep_running = 1;
    } else if (opt_fw) {
        cfg.firmware_path = opt_fw;
    }

    return mirisdr_open_ex(&dev, &cfg);
}

static int device_reopen (void)
{
    int tries;

    pump_stop();

    if (dev) mirisdr_close(dev);
    dev = NULL;

    /* udev can take 10 s to open a re-enumerated device to us */
    for (tries = 0; tries < 40; tries++) {
        usleep(500000);

        if (device_open() == 0) return 0;
    }

    return -1;
}

/* ------------------------------------------------------------------ */
/* enumeration and identity                                            */
/* ------------------------------------------------------------------ */

static tres_t t_enumerate (void)
{
    uint32_t n = mirisdr_get_device_count();
    const char *name;

    if (!n) { say("no device found"); return T_FAIL; }

    name = mirisdr_get_device_name(0);
    say("%u device%s, first is \"%s\"", n, n == 1 ? "" : "s", name ? name : "(null)");

    return (name && *name) ? T_PASS : T_FAIL;
}

static tres_t t_usb_strings (void)
{
    char manuf[256] = "", prod[256] = "", serial[256] = "";

    if (mirisdr_get_device_usb_strings(0, manuf, prod, serial) < 0) {
        say("could not read the descriptors");
        return T_FAIL;
    }

    say("\"%s\" / \"%s\" / serial \"%s\"", manuf, prod, serial);

    return (*manuf || *prod) ? T_PASS : T_FAIL;
}

static tres_t t_open_by_serial (void)
{
    char serial[256] = "";
    int index;

    if (mirisdr_get_serial(dev, serial, sizeof serial) < 0 || !*serial) {
        say("this device reports no serial");
        return T_SKIP;
    }

    index = mirisdr_get_index_by_serial(serial);

    if (index < 0) { say("serial \"%s\" did not resolve to an index", serial); return T_FAIL; }

    say("serial \"%s\" resolves to index %d", serial, index);

    return T_PASS;
}

static tres_t t_usb_position (void)
{
    uint8_t busnum = 0, devnum = 0;
    char port[32] = "", small[2];

    if (mirisdr_get_usb_position(dev, &busnum, &devnum, port, sizeof port) < 0) {
        say("could not read the position");
        return T_FAIL;
    }
    say("bus %u device %u, port %s", busnum, devnum, port);

    /* a buffer too small for the path is an error, not a truncated path */
    if (mirisdr_get_usb_position(dev, NULL, NULL, small, sizeof small) == 0) {
        say("a 2-byte buffer took the path");
        return T_FAIL;
    }

#ifdef __linux__
    {
        /* sysfs names the device by the same path: its devnum must agree */
        char path[96];
        unsigned sys_devnum = 0;
        FILE *f;

        snprintf(path, sizeof path, "/sys/bus/usb/devices/%s/devnum", port);
        if (!(f = fopen(path, "r"))) { say("no %s", path); return T_FAIL; }
        if (fscanf(f, "%u", &sys_devnum) != 1) sys_devnum = 0;
        fclose(f);
        if (sys_devnum != devnum) { say("sysfs says device %u", sys_devnum); return T_FAIL; }
    }
#endif

    return T_PASS;
}

static tres_t t_identity (void)
{
    uint8_t id[MIRISDR_FW_ID_LEN];
    uint16_t vid = 0, pid = 0;
    char hex[2 * MIRISDR_FW_ID_LEN + 1] = "";
    int n, i;

    mirisdr_get_usb_ids(dev, &vid, &pid);

    n = mirisdr_get_fw_id(dev, id, sizeof id);

    for (i = 0; (i < n) && (i < MIRISDR_FW_ID_LEN); i++)
        sprintf(hex + 2 * i, "%02x", id[i]);

    say("%04X:%04X, %s, build %s", vid, pid,
        from_rom ? "running from ROM" : "running from RAM",
        *hex ? hex : "(none - not our firmware)");

    return vid ? T_PASS : T_FAIL;
}

static tres_t t_firmware_precedence (void)
{
    mirisdr_open_config_t cfg;
    const uint8_t *image;
    uint32_t size = 0;

    image = mirisdr_default_firmware(&size);

    if (!image || !size) { say("no firmware is built in"); return T_FAIL; }

    mirisdr_open_config_default(&cfg);

    if (cfg.firmware != image) {
        say("the default config does not carry the built-in image");
        return T_FAIL;
    }

    if (cfg.firmware_size != size) {
        say("the default config's size %u does not match the image's %u",
            cfg.firmware_size, size);
        return T_FAIL;
    }

    say("%u bytes built in, and the default config uses it", size);

    return T_PASS;
}

static tres_t t_fw_patch_roundtrip (void)
{
    mirisdr_fw_patch_t got, set;
    const uint8_t *image;
    uint32_t size = 0;
    uint8_t *copy;
    int r;

    image = mirisdr_default_firmware(&size);
    if (!image || !size) { say("no firmware is built in"); return T_SKIP; }

    if (!(copy = malloc(size))) { say("out of memory"); return T_FAIL; }
    memcpy(copy, image, size);

    memset(&got, 0, sizeof got);

    if (mirisdr_fw_get(copy, size, &got) < 0) {
        say("the image carries no information block");
        free(copy);
        return T_FAIL;
    }

    note("image carries %04X:%04X serial \"%s\"", got.vid, got.pid, got.serial);

    memset(&set, 0, sizeof set);
    set.fields = MIRISDR_FW_PATCH_IDS;
    set.vid = 0x1234;
    set.pid = 0x5678;

    r = mirisdr_fw_patch(copy, size, &set);

    if (r < 0) { say("patching the ids failed"); free(copy); return T_FAIL; }

    memset(&got, 0, sizeof got);
    mirisdr_fw_get(copy, size, &got);

    free(copy);

    if ((got.vid != 0x1234) || (got.pid != 0x5678)) {
        say("read back %04X:%04X after patching 1234:5678", got.vid, got.pid);
        return T_FAIL;
    }

    say("ids patch and read back correctly, the original image untouched");

    return T_PASS;
}

/* ------------------------------------------------------------------ */
/* streaming                                                           */
/* ------------------------------------------------------------------ */

static int stream_setup (const char *transport, const char *format, uint32_t rate)
{
    pump_stop();

    if (mirisdr_set_transfer(dev, transport) < 0) { say("transport %s refused", transport); return -1; }
    if (mirisdr_set_sample_format(dev, format) < 0) { say("format %s refused", format); return -1; }
    if (mirisdr_set_center_freq(dev, 144000000) < 0) { say("could not tune"); return -1; }
    if (mirisdr_set_sample_rate(dev, rate) < 0) { say("rate %u refused", rate); return -1; }

    if (pump_start() < 0) { say("the stream would not start"); return -1; }

    /* confirm data is really flowing, and give it one reopen if not */
    if (stream_rate(0.3, NULL) > 100000) return 0;

    if (device_reopen() < 0) { say("the device did not come back"); return -1; }

    mirisdr_set_transfer(dev, transport);
    mirisdr_set_sample_format(dev, format);
    mirisdr_set_center_freq(dev, 144000000);
    mirisdr_set_sample_rate(dev, rate);

    if (pump_start() < 0) { say("the stream would not start after a reopen"); return -1; }

    if (stream_rate(0.3, NULL) < 100000) { say("no data even after a reopen"); return -1; }

    return 0;
}

static tres_t stream_one (const char *transport, const char *format, uint32_t rate)
{
    mirisdr_stream_stats_t d;
    double sps;

    if (stream_setup(transport, format, rate) < 0) return T_FAIL;

    sps = stream_rate(0.6, &d);

    say("%s %s at %u: %.0f sps (%+.2f%%), %llu gaps, %llu lost, %llu resyncs",
        transport, format, rate, sps, 100.0 * (sps - rate) / rate,
        (unsigned long long) d.gaps, (unsigned long long) d.lost,
        (unsigned long long) d.resyncs);

    if (!within(sps, rate, 0.03)) return T_FAIL;

    return T_PASS;
}

static tres_t t_stream_bulk  (void) { return stream_one("BULK",  "504_S8", 2000000); }
static tres_t t_stream_isoc  (void) { return stream_one("ISOC",  "504_S8", 2000000); }
static tres_t t_stream_isoc1 (void) { return stream_one("ISOC1", "504_S8", 2000000); }

static tres_t t_stream_isoc2 (void)
{
    if (!fw_ours) { say("alt 4 needs our firmware"); return T_SKIP; }

    return stream_one("ISOC2", "504_S8", 2000000);
}

static tres_t t_transport_capacity (void)
{
    static const struct { const char *name; uint32_t rate; } want[] = {
        { "ISOC1", 3500000  },      /* 1 x 1024 -> 8.19 MB/s  */
        { "ISOC2", 7000000  },      /* 2 x 1024 -> 16.4 MB/s  */
        { "ISOC",  11000000 },      /* 3 x 1024 -> 24.6 MB/s  */
        { "BULK",  11000000 },
    };
    unsigned i;
    int bad = 0, ran = 0;

    for (i = 0; i < sizeof want / sizeof want[0]; i++)
    {
        mirisdr_stream_stats_t d;
        double sps;

        if (!fw_ours && !strcmp(want[i].name, "ISOC2")) continue;

        if (stream_setup(want[i].name, "504_S8", want[i].rate) < 0) return T_FAIL;

        usleep(150000);
        sps = stream_rate(0.6, &d);
        ran++;

        note("%-6s %8u -> %10.0f sps  %+6.2f%%  lost %llu", want[i].name,
             want[i].rate, sps, 100.0 * (sps - want[i].rate) / want[i].rate,
             (unsigned long long) d.lost);

        if (!within(sps, want[i].rate, 0.03)) bad++;
    }

    if (bad) { say("%d of %d transports could not hold their rate", bad, ran); return T_FAIL; }

    say("%d transports each sustain their own ceiling", ran);

    return T_PASS;
}

static tres_t t_formats (void)
{
    static const char *fmts[] = {
        "252_S16", "336_S16", "384_S16", "504_S16", "504_S8",
        "504_REAL_S16", "672_REAL_S16", "768_REAL_S16"
    };
    unsigned i;
    int bad = 0;

    for (i = 0; i < sizeof fmts / sizeof fmts[0]; i++)
    {
        mirisdr_stream_stats_t d;
        double sps;

        if (stream_setup("BULK", fmts[i], 2000000) < 0) return T_FAIL;

        sps = stream_rate(0.4, &d);

        note("%-14s %10.0f sps  %+6.2f%%  gaps %llu", fmts[i], sps,
             100.0 * (sps - 2000000) / 2000000, (unsigned long long) d.gaps);

        if (!within(sps, 2000000, 0.03)) bad++;
    }

    if (bad) { say("%d of %u formats were off rate", bad, (unsigned)(sizeof fmts / sizeof fmts[0])); return T_FAIL; }

    say("all %u packing formats deliver the expected rate",
        (unsigned)(sizeof fmts / sizeof fmts[0]));

    return T_PASS;
}

static tres_t t_rate_range (void)
{
    static const uint32_t rates[] = {
        1300000, 2000000, 3000000, 4000000, 6000000, 8000000, 10000000, 12000000
    };
    unsigned i;
    int bad = 0;

    if (stream_setup("BULK", "504_S8", 2000000) < 0) return T_FAIL;

    for (i = 0; i < sizeof rates / sizeof rates[0]; i++)
    {
        mirisdr_stream_stats_t d;
        double sps;

        if (mirisdr_set_sample_rate(dev, rates[i]) < 0) { say("rate %u refused", rates[i]); return T_FAIL; }

        usleep(250000);
        sps = stream_rate(0.5, &d);

        note("%8u -> %10.0f sps  %+6.2f%%", rates[i], sps,
             100.0 * (sps - rates[i]) / rates[i]);

        if (!within(sps, rates[i], 0.03)) bad++;
    }

    if (bad) { say("%d of %u rates were off", bad, (unsigned)(sizeof rates / sizeof rates[0])); return T_FAIL; }

    say("1.3 to 12 Msps all deliver within 3%%");

    return T_PASS;
}

static tres_t t_rate_clamping (void)
{
    uint32_t got_low, got_high;

    pump_stop();

    mirisdr_set_sample_rate(dev, 1);
    got_low = mirisdr_get_sample_rate(dev);

    mirisdr_set_sample_rate(dev, 100000000);
    got_high = mirisdr_get_sample_rate(dev);

    say("1 sps clamps to %u, 100 Msps clamps to %u", got_low, got_high);

    /* the ceiling doubles when the decimator is bypassed, so 30 Msps is right */
    if ((got_low < 1000000) || (got_low > 2000000)) return T_FAIL;
    if ((got_high < 10000000) || (got_high > 32000000)) return T_FAIL;

    return T_PASS;
}

/* Does the stream come back after each rate change? */
static tres_t t_rate_changes (void)
{
    static const uint32_t rates[] = {
        1300000, 2000000, 3000000, 4000000, 6000000, 8000000, 10000000, 12000000
    };
    mirisdr_stream_stats_t total;
    unsigned pass, i;
    int changes = 0, bad = 0, stalled = 0, misread = 0;

    if (stream_setup("BULK", "504_S8", 2000000) < 0) return T_FAIL;

    memset(&total, 0, sizeof total);

    for (pass = 0; pass < 3; pass++)
        for (i = 0; i < sizeof rates / sizeof rates[0]; i++)
        {
            mirisdr_stream_stats_t a, b, d;
            double sps;

            mirisdr_get_stream_stats(dev, &a);
            if (mirisdr_set_sample_rate(dev, rates[i]) < 0) { say("rate %u refused", rates[i]); return T_FAIL; }

            usleep(200000);
            sps = stream_rate(0.35, &d);
            mirisdr_get_stream_stats(dev, &b);
            changes++;

            /* from just before the change to the end of the window */
            total.gaps    += b.gaps - a.gaps;
            total.lost    += b.lost - a.lost;
            total.resyncs += b.resyncs - a.resyncs;

            if (!d.samples) {
                stalled++;
                note("FAILED at %u: no samples", rates[i]);
            } else if (!within(sps, rates[i], 0.03)) {
                bad++;
                note("FAILED at %u: %.0f sps", rates[i], sps);
            }
            if (b.resyncs > a.resyncs)
                note("resync at %u -> %u: %llu, %llu gaps", i ? rates[i - 1] : rates[sizeof rates / sizeof rates[0] - 1],
                     rates[i], (unsigned long long) (b.resyncs - a.resyncs), (unsigned long long) (b.gaps - a.gaps));
            if ((b.gaps > a.gaps) && (b.lost == a.lost)) {
                misread++;
                note("FAILED at %u: the counter went backwards", rates[i]);
            }
        }

    say("%d rate changes, %d stalled, %d off rate, %d restarts misread, %llu resyncs; "
        "%llu gaps, %llu lost (host load, not counted)",
        changes, stalled, bad, misread, (unsigned long long) total.resyncs,
        (unsigned long long) total.gaps, (unsigned long long) total.lost);

    return (stalled || bad || misread || total.resyncs) ? T_FAIL : T_PASS;
}

static tres_t t_stop_start (void)
{
    mirisdr_stream_stats_t d;
    double sps;
    int i;

    if (stream_setup("BULK", "504_S8", 4000000) < 0) return T_FAIL;

    for (i = 0; i < 12; i++)
    {
        if (mirisdr_stop_async(dev) < 0)  { say("stop %d failed", i + 1); return T_FAIL; }
        usleep(20000);
        if (mirisdr_start_async(dev) < 0) { say("start %d failed", i + 1); return T_FAIL; }
        usleep(60000);
    }

    usleep(200000);
    sps = stream_rate(0.5, &d);

    say("12 stop/start cycles, then %.0f sps (%+.2f%%)", sps, 100.0 * (sps - 4000000) / 4000000);

    return within(sps, 4000000, 0.03) ? T_PASS : T_FAIL;
}

static tres_t t_sync_read (void)
{
    unsigned char *buf;
    int n = 0, r, got = 0, tries;

    pump_stop();

    if (mirisdr_set_transfer(dev, "BULK") < 0) { say("bulk refused"); return T_FAIL; }
    if (mirisdr_set_sample_format(dev, "504_S8") < 0) { say("format refused"); return T_FAIL; }
    if (mirisdr_set_sample_rate(dev, 2000000) < 0) { say("rate refused"); return T_FAIL; }

    if (!(buf = malloc(65536))) { say("out of memory"); return T_FAIL; }

    for (tries = 0; (tries < 40) && (got < 262144); tries++) {
        r = mirisdr_read_sync(dev, buf, 65536, &n);
        if (r < 0) break;
        got += n;
    }

    mirisdr_streaming_stop(dev);
    free(buf);

    say("read %d bytes synchronously", got);

    return (got >= 262144) ? T_PASS : T_FAIL;
}

/* ------------------------------------------------------------------ */
/* the faults this driver has had to fix                               */
/* ------------------------------------------------------------------ */

/* The stats count what was delivered: a restart drops the buffer being filled, which
   they must not go on counting. Ahead by a transfer and a buffer being filled at most */
static tres_t t_restart_counts (void)
{
    int i;
    int64_t ahead, most = 65536 / 4 + 24 * 252;

    if (stream_setup("BULK", "252_S16", 2000000) < 0) return T_FAIL;

    for (i = 0; i < 10; i++) {
        if (mirisdr_set_sample_rate(dev, (i & 1) ? 2000000 : 2400000) < 0) { say("rate refused"); return T_FAIL; }
        usleep(150000);
    }
    usleep(300000);
    ahead = pump_ahead;

    say("after 10 restarts the stats are %lld samples past the buffers, at most %lld", (long long) ahead, (long long) most);
    return (ahead >= 0 && ahead <= most) ? T_PASS : T_FAIL;
}

/* The transfer changed while streaming: other transfers, on another alternate setting */
static tres_t t_transfer_live (void)
{
    /* from one 1024 byte slot to three: kept on the old alternate setting, it stalls */
    static const char *const kinds[] = { "ISOC", "BULK", "ISOC1", "BULK", "ISOC" };
    unsigned i;
    int bad = 0;
    char line[160] = "";

    if (stream_setup("ISOC1", "504_S8", 2000000) < 0) return T_FAIL;

    for (i = 0; i < sizeof kinds / sizeof kinds[0]; i++) {
        double sps;

        if (mirisdr_set_transfer(dev, kinds[i]) < 0) { say("%s refused", kinds[i]); return T_FAIL; }
        sps = stream_rate_settled(0.5, NULL);
        snprintf(line + strlen(line), sizeof line - strlen(line), "%s%s %.2f", i ? ", " : "", kinds[i], sps / 1e6);
        if (!within(sps, 2000000, 0.03) || strcmp(mirisdr_get_transfer(dev), kinds[i]) || pump_result < 0) bad++;
    }

    say("while streaming, Msps: %s", line);
    return bad ? T_FAIL : T_PASS;
}

/* Sync reads take bulk only, and a rate change between them restarts the stream */
static tres_t t_sync_restart (void)
{
    unsigned char *buf;
    mirisdr_stream_stats_t st;
    int n = 0, r, got = 0, i;

    pump_stop();

    if (mirisdr_set_transfer(dev, "ISOC") < 0) { say("isoc refused"); return T_FAIL; }
    if (!(buf = malloc(65536))) { say("out of memory"); return T_FAIL; }
    r = mirisdr_read_sync(dev, buf, 65536, &n);
    if (r != -1) { say("a sync read with isochronous transfers: %d", r); free(buf); return T_FAIL; }

    mirisdr_set_transfer(dev, "BULK");
    mirisdr_set_sample_format(dev, "504_S8");
    mirisdr_set_sample_rate(dev, 2000000);

    for (i = 0; i < 2; i++) {
        int k;

        for (k = 0; k < 8; k++) {
            if (mirisdr_read_sync(dev, buf, 65536, &n) < 0) break;
            got += n;
        }
        if (!i) mirisdr_set_sample_rate(dev, 3000000);
    }
    mirisdr_get_stream_stats(dev, &st);
    mirisdr_streaming_stop(dev);
    free(buf);

    say("isochronous refused; %d bytes over a rate change, %llu lost after it", got, (unsigned long long) st.lost);
    return (got >= 16 * 65536 && st.lost < 100000) ? T_PASS : T_FAIL;
}

static tres_t t_stall_recovery (void)
{
    mirisdr_stream_stats_t d;
    double sps;
    int i, ok = 0;

    if (!fw_ours) { say("needs our firmware, which implements the halt"); return T_SKIP; }

    for (i = 0; i < 5; i++)
    {
        if (stream_setup("BULK", "504_S8", 4000000) < 0) return T_FAIL;

        if (mirisdr_set_endpoint_halt(dev, 1) < 0) {
            say("the device refused to stall its endpoint");
            return T_SKIP;
        }

        /* The transfers in flight come back stalled and the async engine gives
           up on them, so the handle is rebuilt; what is under test is whether
           the part is still usable, which before the firmware fix it was not. */
        usleep(120000);
        pump_stop();

        if (mirisdr_set_endpoint_halt(dev, 0) < 0) { say("the stall would not clear"); return T_FAIL; }

        if (device_reopen() < 0) { say("the device did not come back at all"); return T_FAIL; }

        if (stream_setup("BULK", "504_S8", 4000000) < 0) {
            say("the stream would not come back after attempt %d", i + 1);
            return T_FAIL;
        }

        sps = stream_rate(0.4, &d);

        if (within(sps, 4000000, 0.05)) ok++;
        else note("attempt %d did not come back: %.0f sps", i + 1, sps);
    }

    say("%d of 5 stall/clear cycles recovered", ok);

    return (ok == 5) ? T_PASS : T_FAIL;
}

static tres_t t_stall_clear_then_cancel (void)
{
    mirisdr_stream_stats_t d;
    double sps, t0, dt;
    int i, clean = 0;

    if (!fw_ours) { say("needs our firmware, which implements the halt"); return T_SKIP; }

    for (i = 0; i < 5; i++)
    {
        if (stream_setup("BULK", "504_S8", 4000000) < 0) return T_FAIL;

        if (mirisdr_set_endpoint_halt(dev, 1) < 0) {
            say("the device refused to stall its endpoint");
            return T_SKIP;
        }

        usleep(120000);

        if (mirisdr_set_endpoint_halt(dev, 0) < 0) { say("the stall would not clear"); return T_FAIL; }

        t0 = now();
        pump_stop();
        dt = now() - t0;

        if (dt < 3.5) clean++;
        else note("attempt %d took %.1f s to cancel", i + 1, dt);
    }

    /* and it has to still stream without being reopened */
    if (stream_setup("BULK", "504_S8", 4000000) < 0) {
        say("no stream after %d unstalled cancels", 5);
        return T_FAIL;
    }

    sps = stream_rate(0.4, &d);

    say("%d of 5 cancelled without giving up, then %.0f sps", clean, sps);

    return (clean == 5 && within(sps, 4000000, 0.05)) ? T_PASS : T_FAIL;
}

/* Baseband on the receiver: each path at its rates, float buffers in order, then a
   tune to another IF while streaming moves the rate */
static struct { uint64_t pairs, want; int bad, calls; uint32_t rate; } bbd;

static void bbd_cb (unsigned char *buf, uint32_t len, void *ctx)
{
    mirisdr_buffer_info_t in;

    (void) buf; (void) ctx;
    if (mirisdr_get_buffer_info(dev, &in) < 0 || in.type != MIRISDR_SAMPLE_F32 || in.adc != MIRISDR_IQ_BOTH)
        bbd.bad++;
    bbd.rate = in.rate;
    bbd.pairs += len / 8;
    bbd.calls++;
}

static void *bbd_main (void *arg)
{
    (void) arg;
    pump_result = mirisdr_read_async(dev, bbd_cb, NULL, 0, 0);
    return NULL;
}

static tres_t t_bb_device (void)
{
    static const struct { uint32_t ifhz; int iq; uint32_t rate; } cases[] = {
        { 0, MIRISDR_IQ_BOTH, 2000000 }, { 450000, MIRISDR_IQ_BOTH, 900000 }, { 450000, MIRISDR_IQ_ONLY_Q, 225000 },
        { 1620000, MIRISDR_IQ_ONLY_I, 1620000 }, { 2048000, MIRISDR_IQ_BOTH, 2048000 },
        { 2048000, MIRISDR_IQ_ONLY_I, 4096000 }, { 2048000, MIRISDR_IQ_BOTH, 256000 } };
    mirisdr_tune_config_t tc;
    mirisdr_stream_config_t sc;
    mirisdr_stream_result_t sr;
    unsigned i;
    int bad = 0;
    double t0, sps;

    pump_stop();
    for (i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        mirisdr_get_tune(dev, 0, &tc, NULL);
        tc.frequency = 144000000; tc.if_freq = cases[i].ifhz; tc.iq = cases[i].iq; tc.bandwidth = 0;
        tc.low_if_auto = 0; tc.gain.mode = MIRISDR_GAIN_KEEP;
        if (mirisdr_tune(dev, 0, &tc, NULL) < 0) { say("IF %u refused", cases[i].ifhz); bad++; continue; }
        mirisdr_get_stream(dev, &sc, NULL);
        sc.baseband = 1; sc.rate = cases[i].rate; sc.format = NULL; sc.format_single = NULL;
        if (mirisdr_set_stream(dev, &sc, &sr) < 0) { say("IF %u at %u refused", cases[i].ifhz, cases[i].rate); bad++; continue; }

        memset(&bbd, 0, sizeof bbd);
        if (mirisdr_reset_buffer(dev) < 0 || pthread_create(&pump_thread, NULL, bbd_main, NULL)) return T_FAIL;
        pump_running = 1;
        usleep(300000);
        bbd.pairs = 0;
        t0 = now();
        usleep(700000);
        sps = bbd.pairs / (now() - t0);

        /* a tune to the 450 kHz IF while it runs: the rate moves to the nearest that has */
        if (i == sizeof cases / sizeof cases[0] - 1) {
            tc.if_freq = 450000;
            if (mirisdr_tune(dev, 0, &tc, NULL) < 0) { say("the tune to 450 kHz refused"); bad++; }
            usleep(300000);
            bbd.pairs = 0;
            t0 = now();
            usleep(500000);
            mirisdr_get_stream(dev, &sc, &sr);
            if (sr.rate != 225000 || !within(bbd.pairs / (now() - t0), 225000, 0.03) || bbd.rate != 225000)
            { say("after the tune to 450 kHz: %u planned, %u in the infos, %.0f sps", sr.rate, bbd.rate,
                  bbd.pairs / (now() - t0)); bad++; }
        }
        pump_stop();

        note("IF %u, %s, %u sps: %.0f sps out of %u buffers, adc %u, %s", cases[i].ifhz,
             cases[i].iq == MIRISDR_IQ_BOTH ? "both" : "one", cases[i].rate, sps, bbd.calls, sr.adc_rate, sr.format);
        if (bbd.bad || !within(sps, cases[i].rate, 0.03))
        { say("IF %u at %u: %.0f sps, %d bad infos", cases[i].ifhz, cases[i].rate, sps, bbd.bad); bad++; }
    }

    /* back to how the other tests expect it */
    mirisdr_get_tune(dev, 0, &tc, NULL);
    tc.if_freq = 0; tc.iq = MIRISDR_IQ_BOTH; tc.bandwidth = 0;
    mirisdr_tune(dev, 0, &tc, NULL);
    mirisdr_get_stream(dev, &sc, NULL);
    sc.baseband = 0; sc.rate = 2000000;
    mirisdr_set_stream(dev, &sc, NULL);

    if (!bad) say("7 paths at their rates, float buffers, a tune to another IF moving the rate while streaming");
    return bad ? T_FAIL : T_PASS;
}

/* A cancel while read_async starts up is kept: before, one that came before the
 * transfers ran was dropped and the stream ran on */
static volatile int start_entering, start_done;

static int pump_infos_since (void)
{
    return pump_infos;
}

static void *start_pump (void *arg)
{
    (void) arg;
    pump_infos = 0;
    start_entering = 1;
    pump_result = mirisdr_read_async(dev, stream_cb, NULL, 8, 65536);
    start_done = 1;
    return NULL;
}

static tres_t t_cancel_at_start (void)
{
    mirisdr_stream_stats_t d;
    pthread_t th;
    double t0, sps;
    int i, r, e, kept = 0, before = 0, lost = 0;

    if (stream_setup("BULK", "252_S16", 2000000) < 0) return T_FAIL;
    pump_stop();

    /* cancel 0 to 4 ms after starting, across the start up */
    for (i = 0; i < 40; i++)
    {
        start_done = start_entering = 0;
        if (mirisdr_reset_buffer(dev) < 0) { say("reset failed"); return T_FAIL; }
        if (pthread_create(&th, NULL, start_pump, NULL)) return T_FAIL;
        usleep(100 * i);
        e = start_entering;
        r = mirisdr_cancel_async(dev);

        t0 = now();
        while (!start_done && now() - t0 < 2) usleep(1000);

        /* one before read_async is entered is not kept */
        if (start_done) kept++;
        else if (!e) before++;
        else { lost++; note("cancel %d at %d us was lost (returned %d)", i, 100 * i, r); }

        if (!start_done) mirisdr_cancel_async(dev);
        pthread_join(th, NULL);
    }

    if (stream_setup("BULK", "252_S16", 2000000) < 0) { say("no stream after the cancels"); return T_FAIL; }
    sps = stream_rate(0.4, &d);

    say("%d cancels kept, %d lost, %d before the stream was entered; then %.0f sps", kept, lost, before, sps);

    return (!lost && kept > 0 && within(sps, 2000000, 0.05)) ? T_PASS : T_FAIL;
}

/* A cancel while another thread stops or restarts the stream: the stream thread must
   not free the transfers that one is still using */
static volatile int churn_quit, churn_cycles;

static void *churn_main (void *arg)
{
    (void) arg;
    while (!churn_quit) {
        if (mirisdr_stop_async(dev) == 0 && mirisdr_start_async(dev) == 0) churn_cycles++;
        else usleep(100);
    }
    return NULL;
}

static tres_t t_cancel_while_restarting (void)
{
    mirisdr_stream_stats_t d;
    pthread_t pump, churn;
    double t0, sps;
    int i, hung = 0, cycles = 0;

    if (stream_setup("BULK", "252_S16", 2000000) < 0) return T_FAIL;
    pump_stop();

    for (i = 0; i < 200; i++)
    {
        start_done = start_entering = 0;
        if (mirisdr_reset_buffer(dev) < 0) { say("reset failed"); return T_FAIL; }
        if (pthread_create(&pump, NULL, start_pump, NULL)) return T_FAIL;

        /* running, with another thread stopping and restarting it */
        t0 = now();
        while (!pump_infos_since() && now() - t0 < 1) usleep(1000);
        churn_quit = 0;
        churn_cycles = 0;
        if (pthread_create(&churn, NULL, churn_main, NULL)) return T_FAIL;

        /* the cancel anywhere in a cycle */
        usleep(500 + (useconds_t) (rand() % 8000));
        mirisdr_cancel_async(dev);

        t0 = now();
        while (!start_done && now() - t0 < 3) usleep(1000);
        if (!start_done) { hung++; mirisdr_cancel_async(dev); }
        pthread_join(pump, NULL);
        churn_quit = 1;
        pthread_join(churn, NULL);
        cycles += churn_cycles;
    }

    if (stream_setup("BULK", "252_S16", 2000000) < 0) { say("no stream after the cancels"); return T_FAIL; }
    sps = stream_rate(0.4, &d);

    say("200 cancels during %d stop and restart cycles, %d hung; then %.0f sps", cycles, hung, sps);
    return (!hung && cycles > 0 && within(sps, 2000000, 0.05)) ? T_PASS : T_FAIL;
}

/* Header bytes 8-11 are marked at startup; read them back through the remap */
static tres_t t_header_stamp (void)
{
    static const uint8_t want[4] = { 'B', 'V', 'D', 'B' };
    int i;

    if (!fw_ours) { say("needs our firmware, which writes the stamp"); return T_SKIP; }

    pump_stop();

    for (i = 0; i < 4; i++)
    {
        uint8_t got[5];
        uint16_t addr = (uint16_t) (0xE000 + i * 0x400 + 8);

        if (mirisdr_read_mem(dev, addr, got, sizeof got, MIRISDR_MEM_REMAP) < 0) {
            say("could not read buffer %d through the remap", i);
            return T_FAIL;
        }

        if (memcmp(got, want, 4)) {
            say("buffer %d at %04X carries %02X%02X%02X%02X, not BVDB",
                i, addr, got[0], got[1], got[2], got[3]);
            return T_FAIL;
        }

        if (got[4] != i) {
            say("buffer %d carries ring index %u", i, got[4]);
            return T_FAIL;
        }
    }

    say("all four buffers stamped BVDB and indexed 0-3");

    return T_PASS;
}

/* The resync used to fire on stale blocks and misalign a good stream */
static tres_t t_no_false_resync (void)
{
    mirisdr_stream_stats_t d;
    double sps;

    if (stream_setup("BULK", "504_S8", 8000000) < 0) return T_FAIL;

    sps = stream_rate(3.0, &d);

    say("3 s at 8 Msps: %.0f sps, %llu resyncs, %llu gaps, %llu lost",
        sps, (unsigned long long) d.resyncs, (unsigned long long) d.gaps,
        (unsigned long long) d.lost);

    if (!within(sps, 8000000, 0.03)) return T_FAIL;

    return d.resyncs ? T_FAIL : T_PASS;
}

/* The library resets inside every open, so it has to leave a usable device */
static tres_t t_usb_reset (void)
{
    mirisdr_stream_stats_t d;
    double sps;

    pump_stop();

    if (mirisdr_reset(dev) < 0) { say("the reset itself failed"); return T_FAIL; }

    usleep(300000);

    if (mirisdr_adc_init(dev) < 0) { say("the converter would not re-initialise"); return T_FAIL; }

    if (stream_setup("BULK", "504_S8", 2000000) < 0) return T_FAIL;

    sps = stream_rate(0.5, &d);

    say("streaming again after a port reset: %.0f sps", sps);

    return within(sps, 2000000, 0.03) ? T_PASS : T_FAIL;
}

static int pll_candidates (uint32_t rate)
{
    int i, n = 0;

    for (i = 4; i <= 16; i += 2) {
        uint64_t vco = (uint64_t) rate * i * 12;

        if ((vco >= 202000000ULL) && (vco <= 767999999ULL)) n++;
    }

    return n;
}

/* The band code should stay off the ends of the bank */
static tres_t t_pll_band (void)
{
    static const uint32_t rates[] = { 1300000, 2000000, 3000000, 4000000,
                                      6000000, 8000000, 12000000 };
    unsigned i;
    int edge = 0, forced = 0;

    pump_stop();

    for (i = 0; i < sizeof rates / sizeof rates[0]; i++)
    {
        uint8_t rd[4];
        int band;

        if (mirisdr_set_sample_rate(dev, rates[i]) < 0) { say("rate %u refused", rates[i]); return T_FAIL; }

        if (mirisdr_read_reg(dev, 0, rd, sizeof rd) != (int) sizeof rd) {
            say("could not read the band code");
            return T_FAIL;
        }

        band = rd[0] & 0x0f;
        note("%8u -> band %X, %d divider%s available", rates[i], band,
             pll_candidates(rates[i]), pll_candidates(rates[i]) == 1 ? "" : "s");

        if ((band != 0) && (band != 0x0f)) continue;

        /* With one divider in range there is nowhere else to go, so railing
           there is the part's limit, not a bad choice. */
        if (pll_candidates(rates[i]) > 1) edge++;
        else forced++;
    }

    if (edge) {
        say("%d rate%s railed with another divider available", edge, edge == 1 ? "" : "s");
        return T_FAIL;
    }

    say("no rate railed except %d with only one divider in range", forced);

    return T_PASS;
}

/* Asking the synthesiser for a word it cannot lock leaves the VCO railed near
 * 741 MHz with only the divider still responding, and nothing short of a core
 * reboot brings it back. */
static tres_t t_pll_unlockable (void)
{
    mirisdr_stream_stats_t d;
    uint8_t rd[4];
    double sps;
    int band, parked;

    if (stream_setup("BULK", "504_S8", 2000000) < 0) return T_FAIL;

    pump_stop();

    /* n = 1 asks for a VCO near 48 MHz, far below the valid range */
    if (mirisdr_write_reg(dev, 0x04, 0) < 0)       { say("reg 4 refused"); return T_FAIL; }
    if (mirisdr_write_reg(dev, 0x03, 0x1011F) < 0) { say("reg 3 refused"); return T_FAIL; }

    usleep(50000);

    band = (mirisdr_read_reg(dev, 0, rd, sizeof rd) == (int) sizeof rd) ? (rd[0] & 0x0f) : -1;
    note("band reads %X while the word is unlockable", band);

    if (mirisdr_set_sample_rate(dev, 2000000) < 0) { say("the rate would not go back"); return T_FAIL; }
    if (stream_setup("BULK", "504_S8", 2000000) < 0) { say("the stream would not restart"); return T_FAIL; }

    sps = stream_rate_settled(0.5, &d);
    parked = !within(sps, 2000000, 0.03);

    if (parked) {
        pump_stop();
        mirisdr_reboot(dev, MIRISDR_BOOT_RAM);
        mirisdr_close(dev);
        dev = NULL;

        if (device_reopen() < 0) { say("parked, and it did not come back"); return T_FAIL; }
        if (stream_setup("BULK", "504_S8", 2000000) < 0) { say("parked, no stream after the reboot"); return T_FAIL; }

        sps = stream_rate_settled(0.5, &d);

        if (!within(sps, 2000000, 0.03)) { say("parked, and a core reboot did not clear it"); return T_FAIL; }

        say("parked the vco at band %X as expected, a core reboot cleared it", band);

        return T_PASS;
    }

    say("band %X unlocked, recovered on its own at %.0f sps", band, sps);

    return T_PASS;
}

/* The same hazard through the public interface: ask for far more than the part
   or the bus can carry, then come back down. */
static tres_t t_rate_over_ceiling (void)
{
    mirisdr_stream_stats_t d;
    double sps;
    uint32_t got;

    if (stream_setup("BULK", "504_S8", 2000000) < 0) return T_FAIL;

    mirisdr_set_sample_rate(dev, 100000000);
    got = mirisdr_get_sample_rate(dev);
    usleep(300000);

    note("100 Msps clamped to %u", got);

    if (mirisdr_set_sample_rate(dev, 2000000) < 0) { say("the rate would not go back"); return T_FAIL; }

    usleep(250000);
    sps = stream_rate_settled(0.5, &d);

    if (!within(sps, 2000000, 0.03)) {
        pump_stop();
        if (device_reopen() < 0) { say("stuck at %u, and the device did not come back", got); return T_FAIL; }
        say("stuck after %u, only a reopen recovered it", got);
        return T_FAIL;
    }

    say("clamped to %u, then back to 2 Msps at %.0f sps", got, sps);

    return T_PASS;
}

/* ------------------------------------------------------------------ */
/* registers, memory, call gate                                        */
/* ------------------------------------------------------------------ */

static tres_t t_read_regs (void)
{
    uint8_t rd[4];
    int i, nonzero = 0;

    pump_stop();

    for (i = 0; i < 8; i++)
    {
        if (mirisdr_read_reg(dev, i, rd, sizeof rd) != (int) sizeof rd) {
            say("read index %d failed", i);
            return T_FAIL;
        }

        note("index %d: %02x%02x%02x%02x", i, rd[3], rd[2], rd[1], rd[0]);

        if (rd[0] | rd[1] | rd[2] | rd[3]) nonzero++;
    }

    say("all eight read indices respond, %d carry data", nonzero);

    return nonzero ? T_PASS : T_FAIL;
}

/* Register 8 holds the GPIO state, so it can be written and read back without
   disturbing anything the driver needs */
static tres_t t_write_reg (void)
{
    int before = mirisdr_get_gpio_inputs(dev);

    pump_stop();

    if (mirisdr_write_reg(dev, 0x08, 0x00f380) < 0) { say("register 8 refused a write"); return T_FAIL; }

    say("register write accepted, gpio inputs read %02X", before & 0xff);

    return T_PASS;
}

static tres_t t_read_mem (void)
{
    uint8_t a[20], b[20], iram[256];

    pump_stop();

    if (mirisdr_read_mem(dev, 0x0040, a, sizeof a, MIRISDR_MEM_XDATA) < 0) { say("could not read 0x0040"); return T_FAIL; }
    if (mirisdr_read_mem(dev, 0x0040, b, sizeof b, MIRISDR_MEM_XDATA) < 0) { say("second read failed"); return T_FAIL; }

    if (memcmp(a, b, sizeof a)) { say("two reads of the same address disagreed"); return T_FAIL; }

    if (fw_ours && memcmp(a, "BVDB", 4)) {
        say("our firmware is running but 0x0040 does not carry BVDB");
        return T_FAIL;
    }

    say("%02X%02X%02X%02X at 0x0040%s", a[0], a[1], a[2], a[3],
        fw_ours ? " - the information block" : "");

    if (!fw_ours) {
        if (mirisdr_read_mem(dev, 0x00, iram, 1, MIRISDR_MEM_IRAM) == 0) { say("internal RAM read accepted on the ROM"); return T_FAIL; }
        return T_PASS;
    }

    /* all of internal RAM; the list block (byte 19) says no list runs */
    if (mirisdr_read_mem(dev, 0x00, iram, sizeof iram, MIRISDR_MEM_IRAM) < 0) { say("could not read internal RAM"); return T_FAIL; }
    if (a[19] < 0x08 || iram[a[19]]) { say("list block at 0x%02X reads %d", a[19], iram[a[19]]); return T_FAIL; }
    if (mirisdr_read_mem(dev, 0xF0, iram, 32, MIRISDR_MEM_IRAM) == 0) { say("read past 0xFF accepted"); return T_FAIL; }
    say("internal RAM read, list block at 0x%02X idle", a[19]);

    return T_PASS;
}

static tres_t t_write_mem (void)
{
    uint8_t save[16], probe[16], back[16];
    unsigned i;

    if (!fw_ours) { say("needs our firmware, the ROM has nowhere safe to write"); return T_SKIP; }

    pump_stop();

    /* scratch above the image, below xdata */
    if (mirisdr_read_mem(dev, 0x1900, save, sizeof save, MIRISDR_MEM_XDATA) < 0) { say("could not read the scratch area"); return T_FAIL; }

    for (i = 0; i < sizeof probe; i++) probe[i] = (uint8_t) (0xA5 ^ i);

    if (mirisdr_write_mem(dev, 0x1900, probe, sizeof probe, 0) < 0) { say("write failed"); return T_FAIL; }
    if (mirisdr_read_mem(dev, 0x1900, back, sizeof back, MIRISDR_MEM_XDATA) < 0) { say("read back failed"); return T_FAIL; }

    mirisdr_write_mem(dev, 0x1900, save, sizeof save, 0);

    if (memcmp(probe, back, sizeof probe)) { say("what came back differs from what went in"); return T_FAIL; }

    say("16 bytes written and read back, original restored");

    return T_PASS;
}

/* The call gate runs 8051 code on the device. The stub sums r0 and r1 and
   leaves known values everywhere else */
static tres_t t_call_gate (void)
{
    static const uint8_t stub[] = {
        0xE8,                   /* mov  a,r0     */
        0x29,                   /* add  a,r1     */
        0x75, 0xF0, 0x5A,       /* mov  b,#0x5A  */
        0x75, 0x82, 0x34,       /* mov  dpl,#0x34*/
        0x75, 0x83, 0x12,       /* mov  dph,#0x12*/
        0x78, 0xAA,             /* mov  r0,#0xAA */
        0x79, 0xBB,             /* mov  r1,#0xBB */
        0x22                    /* ret           */
    };
    mirisdr_call_regs_t regs;

    if (!fw_ours) { say("needs our firmware, the ROM has no call gate"); return T_SKIP; }

    pump_stop();

    /* 0x1A00-0x1BFF is kept free between the firmware's code and its xdata */
    if (mirisdr_write_mem(dev, 0x1A00, stub, sizeof stub, 0) < 0) { say("could not place the stub"); return T_FAIL; }

    memset(&regs, 0, sizeof regs);
    regs.r0 = 0x40;
    regs.r1 = 0x0F;

    if (mirisdr_call(dev, 0x1A00, &regs) < 0) { say("the call failed"); return T_FAIL; }

    if (regs.a != 0x4F)   { say("a came back %02X, wanted 4F", regs.a); return T_FAIL; }
    if (regs.b != 0x5A)   { say("b came back %02X, wanted 5A", regs.b); return T_FAIL; }
    if (regs.dptr != 0x1234) { say("dptr came back %04X, wanted 1234", regs.dptr); return T_FAIL; }
    if (regs.r0 != 0xAA)  { say("r0 came back %02X, wanted AA", regs.r0); return T_FAIL; }
    if (regs.r1 != 0xBB)  { say("r1 came back %02X, wanted BB", regs.r1); return T_FAIL; }

    say("code ran on the device, every register came back as written");

    return T_PASS;
}

/* ------------------------------------------------------------------ */
/* tuner, gain, gpio                                                   */
/* ------------------------------------------------------------------ */

static tres_t t_tuning (void)
{
    static const uint32_t freqs[] = { 1000000, 40000000, 144000000, 450000000, 900000000 };
    unsigned i;

    pump_stop();

    for (i = 0; i < sizeof freqs / sizeof freqs[0]; i++)
    {
        uint32_t got;

        if (mirisdr_set_center_freq(dev, freqs[i]) < 0) { say("%u Hz refused", freqs[i]); return T_FAIL; }

        got = mirisdr_get_center_freq(dev);

        note("%10u -> %10u, band %d", freqs[i], got, (int) mirisdr_get_band(dev));

        if (got != freqs[i]) { say("set %u, read back %u", freqs[i], got); return T_FAIL; }
    }

    mirisdr_set_center_freq(dev, 144000000);

    say("five frequencies from 1 MHz to 900 MHz set and read back");

    return T_PASS;
}

static tres_t t_tuner_status (void)
{
    static const uint32_t freqs[] = { 100000000, 200000000, 350000000, 500000000, 1500000000 };
    mirisdr_tuner_status_t st[3];
    uint32_t first = 0;
    unsigned i, k, moved = 0;

    pump_stop();

    for (i = 0; i < sizeof freqs / sizeof freqs[0]; i++)
    {
        if (mirisdr_set_center_freq(dev, freqs[i]) < 0) { say("%u Hz refused", freqs[i]); return T_FAIL; }
        usleep(20000);

        for (k = 0; k < 3; k++)
            if (mirisdr_get_tuner_status(dev, 0, &st[k]) < 0) { say("no readback at %u Hz", freqs[i]); return T_FAIL; }

        note("%10u Hz: %08x  coarse %d fine %2u unknown %2u  upconv %2u lna %2u filter %2u%s%s",
             freqs[i], st[0].raw, st[0].coarse, st[0].fine, st[0].unknown,
             st[0].upconv, st[0].lna_cal, st[0].filter,
             st[0].flags & MIRISDR_TUNER_AT_LOW_LIMIT ? "  at low limit" : "",
             st[0].flags & MIRISDR_TUNER_AT_HIGH_LIMIT ? "  at high limit" : "");

        if (st[0].coarse < 0) { say("%u Hz: no VCO range in %08x", freqs[i], st[0].raw); return T_FAIL; }

        /* without a retune the calibration result does not move */
        if (st[1].raw != st[0].raw || st[2].raw != st[0].raw)
        { say("%u Hz: reads differ, %08x %08x %08x", freqs[i], st[0].raw, st[1].raw, st[2].raw); return T_FAIL; }

        /* the VCO fields, bits 25:18 and 13:9 */
        if (!i) first = st[0].raw & 0x03FC3E00;
        else if ((st[0].raw & 0x03FC3E00) != first) moved++;
    }

    mirisdr_set_center_freq(dev, 144000000);

    /* a tuner that stopped taking words, as after EEPROM traffic on a board without
       the clock gate, reads the same calibration whatever the frequency */
    if (!moved) { say("the VCO codes never change with the frequency: the tuner is not responding"); return T_FAIL; }

    say("five frequencies read back, one VCO range each, stable, codes follow the tuning");

    return T_PASS;
}

static tres_t t_tuner_override (void)
{
    mirisdr_tuner_override_t ov;
    mirisdr_tuner_status_t a, b, c;

    pump_stop();

    if (mirisdr_set_center_freq(dev, 159000000) < 0) { say("159 MHz refused"); return T_FAIL; }
    usleep(20000);
    if (mirisdr_get_tuner_status(dev, 0, &a) < 0 || a.coarse < 0) { say("no calibrated readback"); return T_FAIL; }

    /* one fine code off stays inside the PLL's pull-in */
    memset(&ov, 0, sizeof ov);
    ov.hold_vco = 1;    ov.coarse = a.coarse;  ov.fine = a.fine < 31 ? a.fine + 1 : 30;
    ov.hold_upconv = 1; ov.upconv = 5;
    ov.hold_lna = 1;    ov.lna_cal = 7;
    if (mirisdr_set_tuner_override(dev, 0, &ov) < 0) { say("override refused"); return T_FAIL; }

    /* kept across a retune */
    mirisdr_set_center_freq(dev, 159100000);
    usleep(20000);
    if (mirisdr_get_tuner_status(dev, 0, &b) < 0) { say("no readback with the override"); mirisdr_set_tuner_override(dev, 0, NULL); return T_FAIL; }
    note("calibrated coarse %d fine %2u, held coarse %d fine %2u upconv %2u lna %2u",
         a.coarse, a.fine, b.coarse, b.fine, b.upconv, b.lna_cal);

    mirisdr_set_tuner_override(dev, 0, NULL);
    mirisdr_set_center_freq(dev, 159000000);
    usleep(20000);
    if (mirisdr_get_tuner_status(dev, 0, &c) < 0) { say("no readback after clearing"); return T_FAIL; }
    note("cleared coarse %d fine %2u", c.coarse, c.fine);

    if (b.coarse != ov.coarse || b.fine != ov.fine || b.upconv != ov.upconv || b.lna_cal != ov.lna_cal)
    { say("the held values do not read back"); return T_FAIL; }

    if (c.coarse != a.coarse || c.fine + 1 < a.fine || c.fine > a.fine + 1)
    { say("clearing did not bring the calibration back"); return T_FAIL; }

    say("held VCO, up-converter and LNA codes read back across a retune, cleared to the calibration");

    return T_PASS;
}

static mirisdr_stream_event_t ev_log[64];
static volatile int ev_n;

static void ev_cb (const mirisdr_stream_event_t *ev, void *ctx)
{
    (void) ctx;
    if (ev_n < (int) (sizeof ev_log / sizeof ev_log[0])) ev_log[ev_n++] = *ev;
}

/* the first logged event from 'from' on with 'kind' set, or -1 */
static int ev_find (int from, uint8_t kind)
{
    int i;

    for (i = from; i < ev_n; i++) if (ev_log[i].events & kind) return i;

    return -1;
}

static tres_t t_stream_events (void)
{
    int before, after, i, g, m1, m0, t;
    uint64_t last = 0;

    pump_stop();

    mirisdr_set_center_freq(dev, 144000000);
    mirisdr_set_tuner_gain_mode(dev, 1);
    mirisdr_set_tuner_gain(dev, 20);

    ev_n = 0;
    mirisdr_set_stream_events(dev, ev_cb, NULL);

    if (pump_start() < 0) { mirisdr_set_stream_events(dev, NULL, NULL); say("stream did not start"); return T_FAIL; }

    before = ev_n;
    mirisdr_set_tuner_gain(dev, 60);
    usleep(200000);
    mirisdr_set_center_freq(dev, 145000000);
    usleep(200000);
    mirisdr_set_stream_mark(dev, 1);
    usleep(200000);
    mirisdr_set_stream_mark(dev, 0);
    usleep(200000);
    after = ev_n;

    pump_stop();
    mirisdr_set_stream_events(dev, NULL, NULL);
    mirisdr_set_center_freq(dev, 144000000);

    for (i = 0; i < ev_n; i++)
    {
        note("sample %10llu index %10llu  events %02x  bb %2u mixbu %u mixl %u lna %u mark %u sat %u  raw %04x",
             (unsigned long long) ev_log[i].sample, (unsigned long long) ev_log[i].index, ev_log[i].events,
             ev_log[i].bb_gr, ev_log[i].mixbu, ev_log[i].mixl, ev_log[i].lna, ev_log[i].mark, ev_log[i].saturate, ev_log[i].raw);

        if (ev_log[i].sample < last) { say("event positions go backwards at %d", i); return T_FAIL; }
        last = ev_log[i].sample;
    }

    g  = ev_find(before, MIRISDR_EVENT_GAIN);
    t  = ev_find(g < 0 ? before : g, MIRISDR_EVENT_TUNE);
    m1 = ev_find(t < 0 ? before : t, MIRISDR_EVENT_MARK);
    m0 = ev_find(m1 < 0 ? before : m1 + 1, MIRISDR_EVENT_MARK);

    if (g < 0)  { say("no gain event (%d events)", after - before); return T_FAIL; }
    if (t < 0)  { say("no tune event after the gain event"); return T_FAIL; }
    if (m1 < 0 || !ev_log[m1].mark) { say("no marker set event"); return T_FAIL; }
    if (m0 < 0 || ev_log[m0].mark)  { say("no marker clear event"); return T_FAIL; }

    say("gain, tune, marker set and clear seen in order (%d events)", after - before);

    return T_PASS;
}

static tres_t t_gain (void)
{
    int gains[256], n, i, before;

    pump_stop();

    before = mirisdr_get_tuner_gain(dev);

    n = mirisdr_get_tuner_gains(dev, NULL);

    if (n <= 0) { say("the tuner reports no gain steps"); return T_FAIL; }
    if (n > (int)(sizeof gains / sizeof gains[0]))
    { say("%d gain steps, more than this test holds", n); return T_FAIL; }

    n = mirisdr_get_tuner_gains(dev, gains);

    for (i = 0; i < n; i += (n > 8 ? n / 8 : 1))
    {
        if (mirisdr_set_tuner_gain(dev, gains[i]) < 0) { say("gain %d refused", gains[i]); return T_FAIL; }
        note("gain %d -> %d", gains[i], mirisdr_get_tuner_gain(dev));
    }

    mirisdr_set_tuner_gain(dev, before);

    say("%d gain steps, a spread of them set and read back", n);

    return T_PASS;
}

static tres_t t_bandwidth (void)
{
    static const uint32_t bws[] = { 200000, 300000, 600000, 1536000, 5000000, 6000000, 7000000, 8000000 };
    unsigned i;

    pump_stop();

    for (i = 0; i < sizeof bws / sizeof bws[0]; i++)
    {
        if (mirisdr_set_bandwidth(dev, bws[i]) < 0) { say("%u Hz refused", bws[i]); return T_FAIL; }
        note("%8u -> %8u", bws[i], mirisdr_get_bandwidth(dev));
    }

    say("%u filter settings accepted", (unsigned)(sizeof bws / sizeof bws[0]));

    return T_PASS;
}

static tres_t t_gpio_read (void)
{
    int mask, i;

    pump_stop();

    mask = mirisdr_get_gpio_inputs(dev);

    if (mask < 0) { say("the gpio inputs would not read"); return T_FAIL; }

    for (i = 0; i < 4; i++)
        note("pin %d: direction %d, input %d", i,
             mirisdr_get_gpio_direction(dev, i), mirisdr_get_gpio_input(dev, i));

    say("gpio inputs read %02X", mask & 0xff);

    return T_PASS;
}

/* ------------------------------------------------------------------ */
/* eeprom, uart, i2c, pps                                              */
/* ------------------------------------------------------------------ */

static tres_t t_eeprom_probe (void)
{
    int size;

    if (!opt_eeprom) { say("needs --eeprom"); return T_SKIP; }
    if (!fw_ours) { say("needs our firmware"); return T_SKIP; }

    pump_stop();

    size = mirisdr_eeprom_size(dev);

    if (size < 0)  { say("the probe failed"); return T_FAIL; }
    if (size == 0) { say("no eeprom on this board"); return T_SKIP; }

    say("%d bytes, %s addressing", size, size > 512 ? "16 bit" : "9 bit");

    return T_PASS;
}

static tres_t t_eeprom_read (void)
{
    uint8_t a[32], b[32];

    if (!opt_eeprom) { say("needs --eeprom"); return T_SKIP; }
    if (!fw_ours) { say("needs our firmware"); return T_SKIP; }
    if (mirisdr_eeprom_size(dev) <= 0) { say("no eeprom on this board"); return T_SKIP; }

    pump_stop();

    if (mirisdr_read_eeprom(dev, 0, a, sizeof a) < 0) { say("read failed"); return T_FAIL; }
    if (mirisdr_read_eeprom(dev, 0, b, sizeof b) < 0) { say("second read failed"); return T_FAIL; }

    if (memcmp(a, b, sizeof a)) { say("two reads of the same bytes disagreed"); return T_FAIL; }

    say("byte 0 is %02X (%s), 32 bytes read twice identically", a[0],
        a[0] == 0xB4 ? "id override" : a[0] == 0xD2 ? "image container" : "unprogrammed");

    return T_PASS;
}

/* Writes the same bytes back, so the content never changes. Do not cut power in the middle though */
static tres_t t_eeprom_write (void)
{
    uint8_t save[16], back[16];

    if (!opt_eeprom_write) { say("needs --eeprom-write"); return T_SKIP; }
    if (!fw_ours) { say("needs our firmware"); return T_SKIP; }
    if (mirisdr_eeprom_size(dev) <= 0) { say("no eeprom on this board"); return T_SKIP; }

    pump_stop();

    /* well away from byte 0, which the ROM dispatches on */
    if (mirisdr_read_eeprom(dev, 0x100, save, sizeof save) < 0) { say("read failed"); return T_FAIL; }
    if (mirisdr_write_eeprom(dev, 0x100, save, sizeof save) < 0) { say("write failed"); return T_FAIL; }
    if (mirisdr_read_eeprom(dev, 0x100, back, sizeof back) < 0) { say("read back failed"); return T_FAIL; }

    if (memcmp(save, back, sizeof save)) { say("the bytes changed across a write back"); return T_FAIL; }

    say("16 bytes at 0x100 written back unchanged and verified");

    return T_PASS;
}

static tres_t t_uart (void)
{
    static const uint8_t msg[] = "miri_test\r\n";

    if (!fw_ours) { say("needs our firmware"); return T_SKIP; }
    if (!opt_io)  { say("drives a pin, needs --io"); return T_SKIP; }

    pump_stop();

    if (mirisdr_uart_write(dev, 115200, msg, sizeof msg - 1) < 0) { say("115200 baud rejected"); return T_FAIL; }
    if (mirisdr_uart_write(dev, 9600, msg, sizeof msg - 1) < 0)   { say("9600 baud rejected"); return T_FAIL; }

    say("sent at 9600 and 115200 - the wire is not checked here");

    return T_PASS;
}

static tres_t t_i2c (void)
{
    uint8_t byte = 0;
    int acked = 0, addr, r;

    if (!fw_ours) { say("needs our firmware"); return T_SKIP; }
    if (!opt_io)  { say("drives the bus, needs --io"); return T_SKIP; }

    pump_stop();

    if (mirisdr_set_i2c_rate(dev, 100000) < 0) { say("the rate was refused"); return T_FAIL; }
    if (mirisdr_i2c_recover(dev) < 0) { say("bus recovery failed"); return T_FAIL; }

    for (addr = 0x08; addr < 0x78; addr++)
    {
        r = mirisdr_i2c_read(dev, (uint8_t) addr, 0, &byte, 1);

        if (r >= 0) { acked++; note("device at %02X", addr); }
    }

    say("bus scanned, %d device%s answered", acked, acked == 1 ? "" : "s");

    return T_PASS;
}

static int cmp_i64 (const void *a, const void *b)
{
    int64_t x = *(const int64_t *) a, y = *(const int64_t *) b;

    return x < y ? -1 : x > y;
}

static tres_t t_pps (void)
{
    mirisdr_pps_t q;
    int64_t d[16];
    uint64_t last = 0;
    uint8_t edges = 0;
    int seen = 0, have = 0, n = 0, nd = 0, i;
    double t0, worst = 0, ppm;
    int64_t med;

    if (!fw_ours) { say("needs our firmware"); return T_SKIP; }
    if (!opt_pps) { say("needs a 1PPS on GPIO_0 and --pps"); return T_SKIP; }

    if (stream_setup("BULK", "504_S8", 2000000) < 0) return T_FAIL;

    if (mirisdr_enable_pps(dev, 1) < 0) { say("could not enable"); return T_FAIL; }

    /* ten pulses, or twelve seconds: the deltas between them are the rate */
    t0 = now();

    while (now() - t0 < 12.0 && n < 10)
    {
        usleep(20000);

        if (mirisdr_get_pps(dev, &q) < 0) continue;
        if (seen && q.edges == edges) continue;

        n++;
        note("edge %u at sample %llu, trusted %d, gapless %d, guard %u", q.edges,
             (unsigned long long) q.sample, q.trusted, q.gapless, q.guard);

        if (q.trusted && q.gapless) {
            if (have && (uint8_t) (q.edges - edges) == 1 && nd < 16) d[nd++] = (int64_t) (q.sample - last);
            last = q.sample;
            have = 1;
        }
        else have = 0;

        edges = q.edges;
        seen = 1;
    }

    mirisdr_enable_pps(dev, 0);

    if (!n) { say("no edge arrived in 12 s"); return T_FAIL; }
    if (nd < 2) { say("%d edges, but fewer than three trusted in a row", n); return T_FAIL; }

    qsort(d, nd, sizeof d[0], cmp_i64);
    med = d[nd / 2];

    for (i = 0; i < nd; i++) {
        double dev_us = (double) (d[i] - med) / 2.0;    /* 2 Msps: a sample is 0.5 us */

        if (fabs(dev_us) > worst) worst = fabs(dev_us);
    }

    ppm = ((double) med - 2000000.0) / 2000000.0 * 1e6;

    say("%d edges, %d deltas: median %lld samples a pulse (%+.1f ppm of 1 Hz at 2 Msps), worst %.1f us off",
        n, nd, (long long) med, ppm, worst);

    /* the device clock against the pulse within 0.1 %, the pulses within 5 us of each other */
    return (fabs(ppm) < 1000.0 && worst <= 5.0) ? T_PASS : T_FAIL;
}

static tres_t t_sof (void)
{
    static int64_t d[400];
    mirisdr_pps_t q;
    uint64_t last = 0;
    uint8_t edges = 0;
    int seen = 0, have = 0, n = 0, trusted = 0, nd = 0, i;
    double t0, worst = 0, ppm;
    int64_t med;

    if (!fw_ours) { say("needs our firmware"); return T_SKIP; }

    if (stream_setup("BULK", "504_S8", 2000000) < 0) return T_FAIL;

    if (mirisdr_set_pps_source(dev, 1, 25) < 0) { say("could not select the frame source"); return T_FAIL; }

    if (mirisdr_enable_pps(dev, 1) < 0) {
        mirisdr_set_pps_source(dev, 0, 1);
        say("could not enable");
        return T_FAIL;
    }

    t0 = now();

    while (now() - t0 < 3.0)
    {
        usleep(5000);

        if (mirisdr_get_pps(dev, &q) < 0) continue;
        if (seen && q.edges == edges) continue;

        n++;

        if (q.trusted && q.gapless) {
            trusted++;
            if (have && (uint8_t) (q.edges - edges) == 1 && nd < 400) d[nd++] = (int64_t) (q.sample - last);
            last = q.sample;
            have = 1;
        }
        else have = 0;

        edges = q.edges;
        seen = 1;
    }

    mirisdr_enable_pps(dev, 0);
    mirisdr_set_pps_source(dev, 0, 1);

    if (n < 40) { say("%d captures in 3 s, expected about 60", n); return T_FAIL; }
    if (nd < 10) { say("%d captures, only %d trusted", n, trusted); return T_FAIL; }

    qsort(d, nd, sizeof d[0], cmp_i64);
    med = d[nd / 2];

    for (i = 0; i < nd; i++) {
        double dev_us = (double) (d[i] - med) / 2.0;    /* 2 Msps: a sample is 0.5 us */

        if (fabs(dev_us) > worst) worst = fabs(dev_us);
    }

    /* 25 odd frames are 50 ms of the host's frame clock */
    ppm = ((double) med - 100000.0) / 100000.0 * 1e6;

    say("%d captures, %d trusted, %d deltas: median %lld samples per 50 ms (%+.1f ppm of the frame clock), worst %.1f us off",
        n, trusted, nd, (long long) med, ppm, worst);

    /* a wrong tick wrap would be 51 us, a packet interrupt taken late 2 us */
    return (fabs(ppm) < 1000.0 && worst <= 5.0) ? T_PASS : T_FAIL;
}

/* ------------------------------------------------------------------ */
/* odds and ends                                                       */
/* ------------------------------------------------------------------ */

static tres_t t_settings_roundtrip (void)
{
    int swap;

    pump_stop();

    if (mirisdr_set_swap_iq(dev, 1) < 0) { say("swap refused"); return T_FAIL; }
    swap = mirisdr_get_swap_iq(dev);
    mirisdr_set_swap_iq(dev, 0);

    if (swap != 1) { say("swap_iq read back %d", swap); return T_FAIL; }

    if (mirisdr_set_decimation_bypass(dev, "ON") < 0)   { say("decimation ON refused"); return T_FAIL; }
    if (strcmp(mirisdr_get_decimation_bypass(dev), "ON")) { say("decimation read back wrong"); return T_FAIL; }
    mirisdr_set_decimation_bypass(dev, "AUTO");

    if (mirisdr_set_transfer(dev, "ISOC") < 0) { say("ISOC refused"); return T_FAIL; }
    if (strcmp(mirisdr_get_transfer(dev), "ISOC")) { say("transfer read back wrong"); return T_FAIL; }
    mirisdr_set_transfer(dev, "BULK");

    say("swap, decimation and transfer all round trip");

    return T_PASS;
}

static tres_t t_format_auto (void)
{
    pump_stop();

    if (mirisdr_set_sample_format(dev, "AUTO") < 0) { say("AUTO refused"); return T_FAIL; }
    if (mirisdr_set_sample_rate(dev, 2000000) < 0)  { say("rate refused"); return T_FAIL; }

    note("2 Msps auto picks %s", mirisdr_get_sample_format_selected(dev));

    if (mirisdr_set_sample_rate(dev, 12000000) < 0) { say("rate refused"); return T_FAIL; }

    say("auto picks %s at 12 Msps", mirisdr_get_sample_format_selected(dev));

    mirisdr_set_sample_format(dev, "504_S8");

    return T_PASS;
}

static void chars_word (unsigned n, double frac)
{
    uint32_t fr = (uint32_t) (frac * 2097152.0);

    mirisdr_write_reg(dev, 0x04, fr & 0xFFFFF);
    mirisdr_write_reg(dev, 0x03, 0x1D01F | ((uint32_t) n << 8) | ((fr & 0x100000) ? 0x80 : 0));
}

static double chars_vco (void)
{
    mirisdr_stream_stats_t d;

    stream_rate(0.35, &d);

    return (double) d.samples / 0.35 * 192.0 / 1e6;
}

static void chars_flags (int *band, int *pct0, int *pct1)
{
    uint8_t rd[4];
    int r, n0 = 0, n1 = 0, got = 0;

    *band = -1;

    for (r = 0; r < 24; r++)
    {
        if (mirisdr_read_reg(dev, 0, rd, sizeof rd) != (int) sizeof rd) continue;
        *band = rd[0] & 0x0f;
        if (rd[2] & 0x01) n0++;
        if (rd[2] & 0x02) n1++;
        got++;
    }

    *pct0 = got ? n0 * 100 / got : -1;
    *pct1 = got ? n1 * 100 / got : -1;
}

struct chars_pt { double want, got; int band, p0, p1, locked; };

/* light skips the rate measurement, which only the lock predicate needs */
static void chars_probe_ex (double n, struct chars_pt *o, int light)
{
    chars_word((unsigned) n, n - (unsigned) n);
    usleep(light ? 150000 : 250000);

    o->want = n * 48.0;

    if (light) {
        o->got = o->want;
        o->locked = 1;
    } else {
        o->got = chars_vco();
        o->locked = (fabs(o->got - o->want) < o->want * 0.02);
    }

    chars_flags(&o->band, &o->p0, &o->p1);
}

static void chars_probe (double n, struct chars_pt *o) { chars_probe_ex(n, o, 0); }

/* Register 0 bits 0-5 force the capacitance the band search would otherwise
   pick: bit 0 enables, 1-4 choose, bit 5 opens the loop and fixes tune voltage
   (close?) to the reference used to determine when the next band should be used. */
#define CHARS_FORCE(k)   (0x000200UL | 0x01UL | ((uint32_t)(k) << 1))
#define CHARS_OPEN(k)    (CHARS_FORCE(k) | 0x20UL)

static double chars_at_mhz (double mhz)
{
    double x = mhz / 48.0;

    chars_word((unsigned) x, x - (unsigned) x);
    usleep(220000);

    return chars_vco();
}

/* walk from a frequency inside the band until it stops tracking */
static double chars_edge_walk (double from, double step)
{
    double last = from, f = from;
    int i;

    for (i = 0; i < 24; i++)
    {
        f += step;
        if ((f < 150.0) || (f > 800.0)) break;
        if (fabs(chars_at_mhz(f) - f) > 12.0) break;
        last = f;
    }

    return last;
}

static tres_t t_pll_characterise (void)
{
    double lo_seen[16], hi_seen[16];
    double lock_lo = 0, lock_hi = 0;
    double hi_first = 0, hi_last = 0, lo_last = 0, rail_mhz = 0;
    unsigned step;
    int i, parked, seen_any = 0;

    if (!opt_chars) { say("needs --characterise"); return T_SKIP; }
    if (stream_setup("BULK", "504_S8", 2500000) < 0) return T_FAIL;

    for (i = 0; i < 16; i++) { lo_seen[i] = 0; hi_seen[i] = 0; }

    {
        double pts = 13.5 * 48.0 / opt_step;

        printf("\n     96 to 744 MHz in %.2f MHz steps, %.0f points, about %.0f s\n",
               opt_step, pts, pts * 0.18 + pts * opt_step / 12.0 * 0.35);
        printf("     asked      delivered   error    band   low  high\n");
    }

    {
        double dn = opt_step / 48.0;
        unsigned nsteps = (unsigned) (13.5 / dn), every = (unsigned) (12.0 / opt_step);

        if (every < 1) every = 1;

        for (step = 0; step <= nsteps; step++)
        {
           struct chars_pt p;
            int heavy = ((step % every) == 0);

            chars_probe_ex(2.0 + step * dn, &p, !heavy);

            if ((p.band >= 0) && (p.band < 16)) {
                if (!lo_seen[p.band]) lo_seen[p.band] = p.want;
                hi_seen[p.band] = p.want;
            }

            if (heavy) {
                printf("     %6.1f MHz  %6.1f MHz  %+6.2f%%    %X   %3d%% %3d%%\n",
                       p.want, p.got, (p.got - p.want) / p.want * 100.0,
                       p.band, p.p0, p.p1);

                if (p.locked) { if (!lock_lo) lock_lo = p.want; lock_hi = p.want; }
            }

            if (p.p1 > 50) { if (!hi_first) hi_first = p.want; hi_last = p.want; }
            if (p.p0 < 90) lo_last = p.want;
        }
    }

    printf("\n     each band code, over the span it was seen.  Codes overlap: the\n");
    printf("     search makes a marginal call near an edge, so a frequency in the\n");
    printf("     overlap can land either side.  F also covers everything below the\n");
    printf("     floor, where the search saturates.\n\n");

    for (i = 15; i >= 0; i--)
    {
        if (!lo_seen[i]) continue;
        printf("     band %X  %6.1f to %6.1f MHz  (%.1f wide)\n",
               i, lo_seen[i], hi_seen[i], hi_seen[i] - lo_seen[i]);
        seen_any++;
    }

    /* force each band and see its full range */
    printf("\n     each band forced, against the span the search gives it\n");
    printf("     (the search column is only as good as --step; use 1 to compare widths)\n\n");
    printf("     band   search assigns      forced reaches      open loop\n");

    for (i = 15; i >= 0; i--)
    {
        struct chars_pt p;
        double f0, blo, bhi;

        if (!lo_seen[i]) continue;

        /* open loop first: one frequency, and it is inside the band */
        mirisdr_write_reg(dev, 0x00, CHARS_OPEN(i));
        usleep(250000);
        chars_probe_ex(10.0, &p, 0);
        f0 = p.got;

        if ((f0 < 150.0) || (f0 > 800.0)) {
            printf("     %X      %6.1f to %6.1f    open loop unusable\n",
                   i, lo_seen[i], hi_seen[i]);
            continue;
        }

        /* closed loop with the band still forced: walk out both ways */
        mirisdr_write_reg(dev, 0x00, CHARS_FORCE(i));
        usleep(250000);

        blo = chars_edge_walk(f0, -12.0);
        bhi = chars_edge_walk(f0, +12.0);

        printf("     %X      %6.1f to %6.1f    %6.1f to %6.1f    %6.1f  (%.0f vs %.0f wide)\n",
               i, lo_seen[i], hi_seen[i], blo, bhi, f0,
               bhi - blo, hi_seen[i] - lo_seen[i]);
    }

    mirisdr_write_reg(dev, 0x00, 0x000200);
    usleep(200000);

    printf("\n     locks from %.1f MHz up to at least %.1f, the top of this sweep\n",
           lock_lo, lock_hi);
    printf("     but only 720.0 MHz and below is clean.\n");
    if (hi_first) printf("     0xC002 bit 1 first set at %.1f MHz\n", hi_first);
    else          printf("     0xC002 bit 1 never set\n");
    if (lo_last)  printf("     0xC002 bit 0 dithers up to %.1f MHz asked\n", lo_last);
    else          printf("     0xC002 bit 0 steady throughout\n");
    printf("\n");

    /* Set VCO to absolute highest frequency possible to measure it */
    {
        double railed;

        mirisdr_write_reg(dev, 0x04, 0xFFFFF);
        mirisdr_write_reg(dev, 0x03, 0x1DF9F);      /* n=15, fraction nearly full */
        usleep(1200000);

        mirisdr_set_sample_rate(dev, 2000000);      /* divider 16, so vco/192 */
        usleep(300000);
        railed = stream_rate_settled(0.5, NULL) * 192.0 / 1e6;

        if (railed > 100.0) {
            printf("     free runs at %.1f MHz railed, %.1f MHz above the 720 ceiling\n",
                   railed, railed - 720.0);
            rail_mhz = railed;
        } else {
            printf("     could not measure the railed maximum\n");
        }
    }

    printf("\n");

    parked = !within(stream_rate_settled(0.5, NULL), 2000000, 0.05);

    if (parked) {
        pump_stop();
        mirisdr_reboot(dev, MIRISDR_BOOT_RAM);
        mirisdr_close(dev);
        dev = NULL;
        if (device_reopen() < 0) { say("parked it and it did not come back"); return T_FAIL; }
        if (stream_setup("BULK", "504_S8", 2000000) < 0) { say("parked, no stream after the reboot"); return T_FAIL; }
        if (!within(stream_rate_settled(0.5, NULL), 2000000, 0.05)) {
            say("parked, and a core reboot did not clear it");
            return T_FAIL;
        }
    }

    (void) hi_last;
    say("%d bands, locks from %.1f MHz, rails at %.1f, high flag from %.1f",
        seen_any, lock_lo, rail_mhz, hi_first);

    return T_PASS;
}

#define VCO_THRESH      4000
#define VCO_MARGIN      2       /* bands kept from either pin for "safe" */
#define VCO_START       2800.0
#define VCO_FLOOR       1400.0
#define VCO_CEIL        4600.0

static int vco_set (double mhz, mirisdr_tuner_status_t *st)
{
    unsigned n = (unsigned) (mhz / 96.0);
    unsigned frac = (unsigned) ((mhz / 96.0 - n) * VCO_THRESH);

    if (mirisdr_write_reg(dev, 0x09, ((uint32_t) VCO_THRESH | 0x28UL << 12) << 4 | 5) < 0) return -1;
    if (mirisdr_write_reg(dev, 0x09, ((uint32_t) frac | (uint32_t) n << 12) << 4 | 2) < 0) return -1;
    usleep(1000);

    return mirisdr_get_tuner_status(dev, 0, st);
}

static tres_t t_vco_limits (void)
{
    /* how each band reaches the VCO: f = vco / div - off, the AM rows' first IF in off */
    static const struct { const char *name; double div, off; } bands[] = {
        { "AM up-converted",  16, 120 }, { "VHF", 32, 0 }, { "B3", 16, 0 },
        { "gap x6 (low side)", 16, -144 }, { "gap x7 (low side)", 16, -168 },
        { "B45", 4, 0 }, { "L", 2, 0 },
    };
    double v, lo_pin = 0, hi_pin = 0, lo_safe = 0, hi_safe = 0, span[3][2], code[3][32][2];
    uint8_t ulo[3][32], uhi[3][32];
    double step = opt_step;
    mirisdr_tuner_status_t st;
    uint32_t was = mirisdr_get_center_freq(dev);
    unsigned i;
    int none = 0, dir;

    if (!opt_chars) { say("needs --characterise"); return T_SKIP; }

    pump_stop();

    for (i = 0; i < 3; i++) span[i][0] = span[i][1] = 0;
    memset(code, 0, sizeof code);

    /* any mode will do: L band, so nothing else is in the way */
    if (mirisdr_set_center_freq(dev, 1500000000) < 0) { say("could not tune"); return T_FAIL; }

    printf("\n     VCO from %.0f MHz out to the first pin each way, %.2f MHz steps\n", VCO_START, step);

    /* down to the low pin, then up to the high one, never past either */
    for (dir = -1; dir <= 1; dir += 2)
    {
        for (v = (dir < 0) ? VCO_START : VCO_START + step; (v >= VCO_FLOOR) && (v <= VCO_CEIL); v += dir * step)
        {
            if (vco_set(v, &st) < 0) { mirisdr_set_center_freq(dev, was); say("no readback at %.1f MHz", v); return T_FAIL; }

            if (st.flags & MIRISDR_TUNER_SYNTH_OFF) { none++; continue; }
            if (st.flags & MIRISDR_TUNER_AT_LOW_LIMIT) { lo_pin = v; break; }
            if (st.flags & MIRISDR_TUNER_AT_HIGH_LIMIT) { hi_pin = v; break; }

            if ((st.coarse >= 0) && (st.coarse < 3))
            {
                double *c = code[st.coarse][st.fine & 31];

                if (!span[st.coarse][0] || v < span[st.coarse][0]) span[st.coarse][0] = v;
                if (v > span[st.coarse][1]) span[st.coarse][1] = v;

                /* each code's span and the fine control's values there, for -v */
                if (!c[0]) { ulo[st.coarse][st.fine & 31] = uhi[st.coarse][st.fine & 31] = st.unknown; }
                if (!c[0] || v < c[0]) c[0] = v;
                if (v > c[1]) c[1] = v;
                if (st.unknown < ulo[st.coarse][st.fine & 31]) ulo[st.coarse][st.fine & 31] = st.unknown;
                if (st.unknown > uhi[st.coarse][st.fine & 31]) uhi[st.coarse][st.fine & 31] = st.unknown;
            }

            /* safe: at least VCO_MARGIN bands from either end */
            if ((st.coarse > 0 || st.fine <= 31 - VCO_MARGIN) && (st.coarse < 2 || st.fine >= VCO_MARGIN))
            {
                if (!lo_safe || v < lo_safe) lo_safe = v;
                if (v > hi_safe) hi_safe = v;
            }
        }
    }

    mirisdr_set_center_freq(dev, was);

    if (!lo_pin || !hi_pin) { say("no pin found: low %.1f, high %.1f MHz", lo_pin, hi_pin); return T_FAIL; }

    printf("     pinned low up to %.1f MHz, pinned high from %.1f MHz\n", lo_pin, hi_pin);
    for (i = 0; i < 3; i++)
        printf("     range %u  %6.1f to %6.1f MHz\n", i, span[i][0], span[i][1]);
    if (none) printf("     %d points with no range selected\n", none);

    if (opt_verbose)
    {
        int r, f;

        printf("\n     each code, over the VCO span the search chose it (the pinned ends excluded)\n");
        printf("     range band      from       to   width   unknown\n");
        for (r = 0; r < 3; r++)
            for (f = 31; f >= 0; f--)
            {
                double *c = code[r][f];

                if (!c[0]) continue;
                printf("     %u     %2d    %7.1f  %7.1f  %5.1f   %2u-%2u\n", r, f, c[0], c[1],
                       c[1] - c[0] + step, ulo[r][f], uhi[r][f]);
            }
    }

    printf("\n     with %d bands kept from either end, %.1f to %.1f MHz, so for each band\n", VCO_MARGIN, lo_safe, hi_safe);
    printf("     band                  divider   unpinned (MHz)       safe (MHz)\n");
    for (i = 0; i < sizeof bands / sizeof bands[0]; i++)
        printf("     %-20s  /%-3.0f  %8.1f to %7.1f  %8.1f to %7.1f\n", bands[i].name, bands[i].div,
               (lo_pin + step) / bands[i].div - bands[i].off, (hi_pin - step) / bands[i].div - bands[i].off,
               lo_safe / bands[i].div - bands[i].off, hi_safe / bands[i].div - bands[i].off);
    printf("\n");

    say("VCO unpinned %.1f to %.1f MHz, %.1f to %.1f with %d bands to spare",
        lo_pin + step, hi_pin - step, lo_safe, hi_safe, VCO_MARGIN);

    return T_PASS;
}

/* The capture engine seems to have a hard bandwidth ceiling that is independent of the
 * chip or USB host, check for corruption in the internal buffer */
static int chars_stamps (void)
{
    static const uint8_t want[4] = { 'B', 'V', 'D', 'B' };
    int i, ok = 0;

    for (i = 0; i < 4; i++)
    {
        uint8_t got[4];

        if (mirisdr_read_mem(dev, (uint16_t) (0xE000 + i * 0x400 + 8), got, sizeof got, MIRISDR_MEM_REMAP) < 0)
            return -1;

        if (!memcmp(got, want, sizeof want)) ok++;
    }

    return ok;
}

/* 252 complex samples of 16 bits in each 1 kB block, so the block byte rate is
   rate x 1024 / 252.  The edge lands near 13.85 Msps, inside the 15 Msps clamp
   and below the decimation threshold. */
#define CEIL_SPP 252

static int chars_restamp (uint32_t rate)
{
    pump_stop();
    mirisdr_reboot(dev, MIRISDR_BOOT_RAM);
    mirisdr_close(dev);
    dev = NULL;

    if (device_reopen() < 0) return -1;
    if (stream_setup("BULK", "252_S16", rate) < 0) return -1;

    usleep(300000);

    return (chars_stamps() == 4) ? 0 : -1;
}

/* The library clamps the rate to the engine's byte ceiling, which is what
   this test is trying to measure, so we have to program the PLL here */
static int chars_stamped_at (uint32_t rate)
{
    double vco = (double) rate * 48.0;
    uint32_t n = (uint32_t) (vco / 48000000.0);
    uint32_t fr = (uint32_t) ((vco / 48000000.0 - n) * 2097152.0);
    int got;

    if (mirisdr_write_reg(dev, 0x04, fr & 0xFFFFF) < 0) return -1;
    if (mirisdr_write_reg(dev, 0x03, 0x1D007 | (n << 8) | ((fr & 0x100000) ? 0x80 : 0)) < 0) return -1;

    usleep(350000);
    got = chars_stamps();

    return (got < 0) ? -1 : (got == 4);
}

/* walk up from lo until the stamps break, in `stepsize` increments */
static uint32_t chars_walk (uint32_t lo, uint32_t hi, uint32_t stepsize, uint32_t *first_bad)
{
    uint32_t r, last_good = 0;

    for (r = lo; r <= hi; r += stepsize)
    {
        int ok = chars_stamped_at(r);

        /* a read that failed is not the same as a rate that broke the marks */
        if (ok < 0) { ok = chars_stamped_at(r); if (ok < 0) { *first_bad = 0; return 0; } }

        if (!ok) { *first_bad = r; return last_good; }

        last_good = r;
    }

    *first_bad = 0;

    return last_good;
}

static tres_t t_engine_ceiling (void)
{
    uint32_t coarse_good, coarse_bad = 0, fine_good, fine_bad = 0;
    double blockrate;

    if (!opt_chars) { say("needs --characterise"); return T_SKIP; }
    if (!fw_ours)   { say("needs our firmware, which writes the stamp"); return T_SKIP; }

    if ((chars_restamp(12000000) < 0) && (chars_restamp(12000000) < 0)) {
        say("could not get a clean start, the marks did not come back");
        return T_FAIL;
    }

    coarse_good = chars_walk(12000000, 15000000, 100000, &coarse_bad);
    if (!coarse_good)  { say("no clean rate found at all"); return T_FAIL; }
    if (!coarse_bad)   { say("still clean at 15 Msps, the edge is above the clamp"); return T_FAIL; }

    if ((chars_restamp(coarse_good) < 0) && (chars_restamp(coarse_good) < 0)) {
        say("could not restamp for the fine pass");
        return T_FAIL;
    }

    fine_good = chars_walk(coarse_good, coarse_bad, 10000, &fine_bad);
    if (!fine_good) { say("the fine pass found nothing clean"); return T_FAIL; }
    if (!fine_bad) fine_bad = coarse_bad;

    blockrate = (double) fine_good * 1024.0 / CEIL_SPP;

    printf("\n     last clean %u sps, first disturbed %u sps\n", fine_good, fine_bad);
    printf("     engine ceiling %.3f MB/s of blocks, %.3f MB/s of payload\n",
           blockrate / 1e6, (double) fine_good * 4.0 / 1e6);
    printf("\n");

    mirisdr_set_sample_rate(dev, 2000000);

    say("%.3f MB/s of blocks, edge between %u and %u sps", blockrate / 1e6, fine_good, fine_bad);

    return T_PASS;
}


/* ------------------------------------------------------------------ */
/* decode: the bulk parser and gap handling, offline                   */
/* ------------------------------------------------------------------ */

/* Synthetic 504_S8 streams: stamped blocks whose header counter is the first
   sample's index, and every sample carrying its own index (mod 65536) as its
   I/Q bytes, so the callback can check each one against the buffer info. */
#define DEC_STEP 504

typedef struct {
    int      blocks;        /* to generate */
    int      skip_at[8];    /* block numbers whose blocks go missing, from each on... */
    int      skip_n[8];     /* ...this many */
    int      junk_at[8];    /* half a block of junk after this block */
    int      cut;           /* this block keeps only its first half, -1 none */
    int      nskip, njunk;
    int      fake;          /* the junk carries a stamp where a header's would be */
} dec_plan_t;

static struct {
    uint64_t delivered;     /* samples */
    uint64_t gaps, missing, filled;
    int      bad;           /* samples not where the info says */
    int      calls;
} dec;

static uint8_t *dec_gen (const dec_plan_t *pl, uint32_t *len)
{
    uint8_t *b = malloc((size_t) (pl->blocks + 2 * pl->njunk) * 1024 + 1024), *h;
    uint64_t idx = 0;
    uint32_t n = 0;
    int k, j, i;

    for (k = 0; k < pl->blocks; k++) {
        for (j = 0; j < pl->nskip; j++)
            if (pl->skip_at[j] == k) idx += (uint64_t) pl->skip_n[j] * DEC_STEP;

        h = b + n;
        memset(h, 0, 16);
        h[0] = (uint8_t) idx; h[1] = (uint8_t) (idx >> 8); h[2] = (uint8_t) (idx >> 16); h[3] = (uint8_t) (idx >> 24);
        memcpy(h + 8, "BVDB", 4);
        for (i = 0; i < DEC_STEP; i++) {
            uint64_t v = idx + (uint64_t) i;
            h[16 + 2 * i] = (uint8_t) v;
            h[17 + 2 * i] = (uint8_t) (v >> 8);
        }
        idx += DEC_STEP;
        n += (k == pl->cut) ? 512 : 1024;

        for (j = 0; j < pl->njunk; j++)
            if (pl->junk_at[j] == k) {
                for (i = 0; i < 512; i++) b[n + i] = (uint8_t) rand();
                if (pl->fake) memcpy(b + n + 8, "BVDB", 4);
                n += 512;
            }
    }

    *len = n;
    return b;
}

static void dec_cb (unsigned char *buf, uint32_t len, void *ctx)
{
    mirisdr_dev_t *d = (mirisdr_dev_t *) ctx;
    mirisdr_buffer_info_t in;
    uint32_t n = len / 2, p, g = 0;
    uint64_t lost = 0;

    dec.calls++;
    if (mirisdr_get_buffer_info(d, &in) < 0) { dec.bad++; return; }
    if (in.sample != dec.delivered) dec.bad++;

    for (p = 0; p < n; p++) {
        uint16_t v = (uint16_t) (buf[2 * p] | buf[2 * p + 1] << 8);
        int zero = 0;

        /* gaps up to this sample: their unfilled part shifts the index, their
           filled part sits right here */
        while (g < in.gaps_len && in.gaps[g].offset <= p) {
            if (in.gaps[g].offset > 0 || p == 0) {
                if (in.gaps[g].offset > 0) lost += in.gaps[g].samples - in.gaps[g].filled;
                dec.gaps++;
                dec.missing += in.gaps[g].samples;
                dec.filled += in.gaps[g].filled;
            }
            g++;
        }
        for (uint32_t k = 0; k < in.gaps_len; k++)
            if (p >= in.gaps[k].offset && p < in.gaps[k].offset + in.gaps[k].filled) zero = 1;

        if (zero ? v != 0 : v != (uint16_t) (in.index + p + lost)) {
            if (!dec.bad && getenv("DEC_DEBUG")) {
                fprintf(stderr, "misplaced: call %d sample %llu p %u v %u want %u zero %d lost %llu; info sample %llu index %llu gaps %u:",
                        dec.calls, (unsigned long long) dec.delivered, p, v, zero ? 0 : (uint16_t) (in.index + p + lost), zero,
                        (unsigned long long) lost, (unsigned long long) in.sample, (unsigned long long) in.index, in.gaps_len);
                for (uint32_t k = 0; k < in.gaps_len; k++) fprintf(stderr, " [%u %u %llu]", in.gaps[k].offset, in.gaps[k].filled, (unsigned long long) in.gaps[k].samples);
                fprintf(stderr, " len %u\n", n);
            }
            dec.bad++;
        }
    }
    dec.delivered += n;
}

/* Feed a plan in random transfer sizes, or 'dec_fixed' ones, 'buf' 0 for buffers as converted */
static uint32_t dec_fixed;

static int dec_run (const dec_plan_t *pl, int fill, uint32_t buf, mirisdr_stream_stats_t *st)
{
    mirisdr_dev_t *d;
    uint32_t len, at = 0, t;
    uint8_t *b = dec_gen(pl, &len);

    memset(&dec, 0, sizeof dec);
    if (mirisdr_open_null(&d, "504_S8") < 0) { free(b); return -1; }
    mirisdr_set_gap_fill(d, fill);

    while (at < len) {
        t = dec_fixed ? dec_fixed : 1 + (uint32_t) rand() % (16 * 1024);
        if (t > len - at) t = len - at;
        mirisdr_feed_bulk(d, dec_cb, d, buf, b + at, t);
        at += t;
    }
    mirisdr_get_stream_stats(d, st);
    mirisdr_close(d);
    free(b);

    return 0;
}

/* Every sample where the info says; the counts as expected */
/* missing may exceed want_missing by up to 'slack' blocks: a block right after a slip
   that ends a transfer cannot be confirmed, and is dropped */
static tres_t dec_check_slack (const char *what, const dec_plan_t *pl, int fill, uint64_t want_missing,
                               uint64_t want_filled, uint64_t want_resyncs, int slack)
{
    static const uint32_t bufs[2] = { 0, 4096 };
    mirisdr_stream_stats_t st;
    int seed, b;

    for (seed = 1; seed <= 50; seed++)
        for (b = 0; b < 2; b++) {
            srand((unsigned) seed);
            if (dec_run(pl, fill, bufs[b], &st) < 0) { say("%s: no null device", what); return T_FAIL; }
            if (dec.bad || dec.missing < want_missing || dec.missing > want_missing + (uint64_t) slack * DEC_STEP ||
                dec.missing % DEC_STEP || st.resyncs != want_resyncs ||
                dec.filled != want_filled + (fill ? dec.missing - want_missing : 0) ||
                st.filled != dec.filled || st.lost != dec.missing - dec.filled) {
                say("%s, seed %d, buffers %u: %d misplaced, %llu missing %llu filled (stats %llu lost %llu filled %llu resyncs)",
                    what, seed, bufs[b], dec.bad, (unsigned long long) dec.missing, (unsigned long long) dec.filled,
                    (unsigned long long) st.lost, (unsigned long long) st.filled, (unsigned long long) st.resyncs);
                return T_FAIL;
            }
        }
    return T_PASS;
}

static tres_t dec_check (const char *what, const dec_plan_t *pl, int fill, uint64_t want_missing,
                         uint64_t want_filled, uint64_t want_resyncs)
{
    return dec_check_slack(what, pl, fill, want_missing, want_filled, want_resyncs, 0);
}

static tres_t t_dec_clean (void)
{
    dec_plan_t pl = { .blocks = 600, .cut = -1 };

    if (dec_check("clean", &pl, 0, 0, 0, 0) != T_PASS) return T_FAIL;
    dec_fixed = 16 * 1024;              /* whole transfers on the grid, as live */
    if (dec_check("clean, 16 kB transfers", &pl, 0, 0, 0, 0) != T_PASS) { dec_fixed = 0; return T_FAIL; }
    dec_fixed = 0;
    say("600 blocks, 50 random splittings and 16 kB transfers, x 2 buffer sizes, every sample in place");
    return T_PASS;
}

static tres_t t_dec_gaps (void)
{
    dec_plan_t pl = { .blocks = 600, .cut = -1, .nskip = 3,
                      .skip_at = { 100, 101, 400 }, .skip_n = { 1, 2, 20 } };

    if (dec_check("gaps", &pl, 0, 23 * DEC_STEP, 0, 0) != T_PASS) return T_FAIL;
    /* filled up to 16 blocks a gap: 1 + 2 + 16 */
    if (dec_check("gaps filled", &pl, 1, 23 * DEC_STEP, 19 * DEC_STEP, 0) != T_PASS) return T_FAIL;
    dec_fixed = 16 * 1024;
    if (dec_check("gaps filled, 16 kB transfers", &pl, 1, 23 * DEC_STEP, 19 * DEC_STEP, 0) != T_PASS) { dec_fixed = 0; return T_FAIL; }
    dec_fixed = 0;
    say("missing blocks placed to the sample, with and without gap fill (20 blocks fill 16)");
    return T_PASS;
}

static tres_t t_dec_slips (void)
{
    dec_plan_t junk = { .blocks = 600, .cut = -1, .njunk = 3, .junk_at = { 100, 300, 500 } };
    dec_plan_t fake = { .blocks = 600, .cut = -1, .njunk = 3, .junk_at = { 100, 300, 500 }, .fake = 1 };
    dec_plan_t pair = { .blocks = 600, .cut = -1, .njunk = 2, .junk_at = { 100, 101 } };
    dec_plan_t cut  = { .blocks = 600, .cut = 300 };

    /* a slip costs the block before it: its successor's stamp is junk */
    if (dec_check("half blocks of junk", &junk, 0, 3 * DEC_STEP, 0, 3) != T_PASS) return T_FAIL;
    /* a false stamp where the next header's would be lets the block before through, whole */
    if (dec_check("junk with a false stamp", &fake, 0, 0, 0, 3) != T_PASS) return T_FAIL;
    if (dec_check("half blocks of junk, filled", &junk, 1, 3 * DEC_STEP, 3 * DEC_STEP, 3) != T_PASS) return T_FAIL;
    /* blocks 100 and 101 both have junk behind them; off the grid until 102, one resync */
    if (dec_check("two slips a block apart", &pair, 0, 2 * DEC_STEP, 0, 1) != T_PASS) return T_FAIL;
    if (dec_check("a block cut short", &cut, 0, DEC_STEP, 0, 1) != T_PASS) return T_FAIL;
    if (dec_check("a block cut short, filled", &cut, 1, DEC_STEP, DEC_STEP, 1) != T_PASS) return T_FAIL;
    dec_fixed = 16 * 1024;
    if (dec_check("half blocks of junk, 16 kB transfers", &junk, 0, 3 * DEC_STEP, 0, 3) != T_PASS) { dec_fixed = 0; return T_FAIL; }
    if (dec_check("a block cut short, 16 kB transfers", &cut, 0, DEC_STEP, 0, 1) != T_PASS) { dec_fixed = 0; return T_FAIL; }
    dec_fixed = 0;
    say("junk, false stamps and a cut block: the block before each slip, one resync each, every sample in place");
    return T_PASS;
}


/* Holding unknown, the finer VCO control, high lowers the frequency: with the VCO
   free the band search makes up for it with a lower fine code */
static tres_t t_tuner_unknown (void)
{
    mirisdr_tuner_status_t a, b, c;
    mirisdr_tuner_override_t ov;

    if (mirisdr_set_center_freq(dev, 159000000) < 0) { say("could not tune"); return T_FAIL; }
    usleep(5000);
    if (mirisdr_get_tuner_status(dev, 0, &a) < 0) { say("no readback"); return T_SKIP; }

    memset(&ov, 0, sizeof ov);
    ov.hold_unknown = 1;
    ov.unknown = 31;
    if (mirisdr_set_tuner_override(dev, 0, &ov) < 0) { say("override refused"); return T_FAIL; }
    mirisdr_set_center_freq(dev, 159100000);
    mirisdr_set_center_freq(dev, 159000000);
    usleep(5000);
    mirisdr_get_tuner_status(dev, 0, &b);

    mirisdr_set_tuner_override(dev, 0, NULL);
    mirisdr_set_center_freq(dev, 159100000);
    mirisdr_set_center_freq(dev, 159000000);
    usleep(5000);
    mirisdr_get_tuner_status(dev, 0, &c);

    note("calibrated fine %u unknown %u, unknown held at 31 fine %u, cleared fine %u unknown %u",
         a.fine, a.unknown, b.fine, c.fine, c.unknown);

    if (b.fine >= a.fine) { say("held high, the band search did not move down (fine %u, was %u)", b.fine, a.fine); return T_FAIL; }
    if (c.fine + 1 < a.fine || c.fine > a.fine + 1) { say("cleared, fine %u, was %u", c.fine, a.fine); return T_FAIL; }

    say("held at 31 the band search went from fine %u to %u, and back when cleared", a.fine, b.fine);
    return T_PASS;
}

/* ------------------------------------------------------------------ */
/* plan: the tune and stream configs, worked out without a device     */
/* ------------------------------------------------------------------ */

static int plan_bad;

/* one tune check against what it should give; want 0 is a refusal */
static void plan_tune (mirisdr_dev_t *n, const char *what, mirisdr_tune_config_t *c, int want,
                       mirisdr_tune_result_t *r)
{
    int got = mirisdr_tune_check(n, 0, c, r) == 0;

    if (got != want) { plan_bad++; say("%s: %s", what, got ? "accepted" : "refused"); }
    note("%-36s %s", what, got ? "accepted" : "refused");
}

static int plan_near (double got, double want, double tol, const char *what)
{
    if (fabs(got - want) <= tol) return 1;

    plan_bad++;
    say("%s: %.0f, not %.0f", what, got, want);

    return 0;
}

static tres_t t_plan_tune (void)
{
    static const uint32_t lo_at[] = { 500000, 10000000, 45000000, 100000000, 200000000, 257000000,
                                      300000000, 600000000, 1500000000, 2050000000, 433920123,
                                      1575420000, 868300000, 12345678 };
    mirisdr_dev_t *n;
    mirisdr_tune_config_t c, got;
    mirisdr_tune_result_t r;
    double worst = 0;
    unsigned i;

    plan_bad = 0;

    if (mirisdr_open_null(&n, "252_S16") < 0) { say("no null device"); return T_FAIL; }
    mirisdr_set_hw_flavour(n, MIRISDR_HW_RSP1B);

    mirisdr_tune_config_default(&c);
    plan_tune(n, "the default", &c, 1, &r);
    if (mirisdr_tune_check(n, 1, &c, &r) == 0) { plan_bad++; say("a second tuner accepted"); }
    if (r.bandwidth != 5000000) { plan_bad++; say("default bandwidth %u at 2.048 Msps", r.bandwidth); }
    plan_near(r.offset, 0, 10, "default offset");

    /* the bandwidths each IF allows */
    c.bandwidth = 300000;
    plan_tune(n, "zero IF, 300 kHz", &c, 0, &r);
    c.if_freq = 450000; c.bandwidth = 0;
    plan_tune(n, "450 kHz IF, widest", &c, 1, &r);
    if (r.bandwidth != 600000) { plan_bad++; say("450 kHz IF widest is %u", r.bandwidth); }
    c.bandwidth = 1536000;
    plan_tune(n, "450 kHz IF, 1536 kHz", &c, 0, &r);
    c.if_freq = 1620000;
    plan_tune(n, "1620 kHz IF, 1536 kHz", &c, 1, &r);
    c.if_freq = 2048000; c.bandwidth = 200000;
    plan_tune(n, "2048 kHz IF, 200 kHz", &c, 0, &r);
    c.if_freq = 0; c.bandwidth = 5500000;
    plan_tune(n, "a 5.5 MHz bandwidth", &c, 0, &r);
    c.bandwidth = 14000000;
    plan_tune(n, "zero IF, no filter", &c, 1, &r);

    /* a single output needs a low IF */
    mirisdr_tune_config_default(&c);
    c.iq = MIRISDR_IQ_ONLY_I;
    plan_tune(n, "zero IF, I only", &c, 0, &r);

    /* low IF auto: the LO above, the signal below it */
    mirisdr_tune_config_default(&c);
    c.frequency = 100000000; c.if_freq = 450000; c.bandwidth = 300000; c.low_if_auto = 1;
    plan_tune(n, "450 kHz IF, auto", &c, 1, &r);
    plan_near(r.lo, 100450000, 20, "low IF auto LO");
    plan_near(r.offset, -450000, 20, "low IF auto offset");
    if (r.inverted) { plan_bad++; say("a complex stream reported inverted"); }

    c.iq = MIRISDR_IQ_ONLY_I;
    plan_tune(n, "450 kHz IF, auto, I only", &c, 1, &r);
    plan_near(r.offset, 450000, 20, "single output IF");
    if (!r.inverted || r.iq != MIRISDR_IQ_ONLY_I) { plan_bad++; say("I only: inverted %d, iq %d", r.inverted, r.iq); }

    /* an offset moves the LO */
    mirisdr_tune_config_default(&c);
    c.frequency = 433920000; c.lo_offset = 200000;
    plan_tune(n, "zero IF, 200 kHz offset", &c, 1, &r);
    plan_near(r.lo, 434120000, 20, "offset LO");
    plan_near(r.offset, -200000, 20, "offset");

    /* past the plan, and the gain each band reaches */
    c.lo_offset = 0;
    c.frequency = 2500000000U;
    plan_tune(n, "2.5 GHz", &c, 0, &r);
    c.frequency = 1500000000; c.gain.mode = MIRISDR_GAIN_TOTAL; c.gain.total = 200;
    plan_tune(n, "1.5 GHz, gain 200", &c, 1, &r);
    if (r.gain.total != 82) { plan_bad++; say("L band gain %d, not 82", r.gain.total); }
    c.frequency = 100000000;
    plan_tune(n, "100 MHz, gain 200", &c, 1, &r);
    if (r.gain.total != 102) { plan_bad++; say("VHF gain %d, not 102", r.gain.total); }
    c.gain.total = -5;
    plan_tune(n, "a negative gain", &c, 0, &r);

    /* a total splits front end first, stages are checked against the band */
    mirisdr_tune_config_default(&c);
    c.frequency = 100000000; c.gain.mode = MIRISDR_GAIN_TOTAL; c.gain.total = 60;
    plan_tune(n, "VHF, 60 dB in total", &c, 1, &r);
    if (r.gain.total != 60 || !r.gain.lna || !r.gain.mixer || r.gain.baseband != 17)
    { plan_bad++; say("60 dB split to %d: lna %d mixer %d baseband %d", r.gain.total, r.gain.lna, r.gain.mixer, r.gain.baseband); }
    c.gain.mode = MIRISDR_GAIN_STAGES; c.gain.lna = 0; c.gain.mixer = 1; c.gain.mixbuffer = 0; c.gain.baseband = 30;
    plan_tune(n, "VHF, stages", &c, 1, &r);
    if (r.gain.total != 49) { plan_bad++; say("stages without the LNA give %d, not 49", r.gain.total); }
    c.gain.baseband = 60;
    plan_tune(n, "baseband 60 dB", &c, 0, &r);
    c.gain.baseband = 30; c.gain.lna = 2;
    plan_tune(n, "LNA 2", &c, 0, &r);
    /* every plan takes HF through AM2: its mixbuffer is 0 or 24 dB */
    c.gain.lna = 1; c.frequency = 1000000; c.gain.mixbuffer = 12;
    plan_tune(n, "AM2, mixbuffer 12", &c, 0, &r);
    c.gain.mixbuffer = 24;
    plan_tune(n, "AM2, mixbuffer 24", &c, 1, &r);
    if (r.gain.total != 30 + 19 + 24) { plan_bad++; say("AM2 stages give %d, not 73", r.gain.total); }
    c.gain.mode = 3;
    plan_tune(n, "gain mode 3", &c, 0, &r);

    /* the LO the synthesizer reaches, across the bands */
    mirisdr_tune_config_default(&c);
    for (i = 0; i < sizeof lo_at / sizeof lo_at[0]; i++)
    {
        c.frequency = lo_at[i];
        plan_tune(n, "LO accuracy", &c, 1, &r);
        if (fabs((double) r.lo - lo_at[i]) > worst) worst = fabs((double) r.lo - lo_at[i]);
    }
    if (worst > 20) { plan_bad++; say("LO off by up to %.0f Hz", worst); }

    /* overrides: range checked, kept with the tune, cleared by the wrapper */
    mirisdr_tune_config_default(&c);
    c.override.hold_vco = 1; c.override.coarse = 3;
    plan_tune(n, "VCO range 3 held", &c, 0, &r);
    c.override.coarse = 1; c.override.fine = 12; c.override.hold_lna = 1; c.override.lna_cal = 16;
    plan_tune(n, "LNA code 16 held", &c, 0, &r);
    c.override.lna_cal = 7;
    plan_tune(n, "VCO and LNA codes held", &c, 1, &r);
    if (mirisdr_tune(n, 0, &c, NULL) < 0) { plan_bad++; say("a tune with holds refused"); }
    mirisdr_get_tune(n, 0, &got, NULL);
    if (!got.override.hold_vco || got.override.fine != 12 || got.override.lna_cal != 7)
    { plan_bad++; say("get_tune lost the holds"); }
    if (mirisdr_set_tuner_override(n, 0, NULL) < 0) { plan_bad++; say("clearing the overrides refused"); }
    mirisdr_get_tune(n, 0, &got, NULL);
    if (got.override.hold_vco || got.override.hold_lna || got.frequency != c.frequency)
    { plan_bad++; say("clearing left hold_vco %u hold_lna %u, frequency %u",
                      got.override.hold_vco, got.override.hold_lna, got.frequency); }
    if (mirisdr_set_tuner_override(n, 1, NULL) == 0) { plan_bad++; say("an override on a second tuner accepted"); }

    mirisdr_close(n);

    /* a real stream: one converter, so the result is a real IF, and the other
       output cannot be switched off */
    if (mirisdr_open_null(&n, "504_REAL_S16") < 0) { say("no null device"); return T_FAIL; }

    mirisdr_tune_config_default(&c);
    c.frequency = 100000000; c.if_freq = 450000; c.low_if_auto = 1;
    plan_tune(n, "real stream, both outputs", &c, 1, &r);
    plan_near(r.offset, 450000, 20, "real stream IF");
    if (!r.inverted) { plan_bad++; say("real stream not inverted"); }
    c.iq = MIRISDR_IQ_ONLY_Q;
    plan_tune(n, "stream on I, tune Q only", &c, 0, &r);
    c.iq = MIRISDR_IQ_ONLY_I;
    plan_tune(n, "stream on I, tune I only", &c, 1, &r);

    mirisdr_close(n);

    if (plan_bad) return T_FAIL;

    say("IF and bandwidth rules, low IF auto, offsets, single outputs, gain limits, holds, LO within %.0f Hz", worst);

    return T_PASS;
}

static void plan_stream (mirisdr_dev_t *n, const char *what, mirisdr_stream_config_t *c, int want,
                         mirisdr_stream_result_t *r)
{
    int got = mirisdr_stream_check(n, c, r) == 0;

    if (got != want) { plan_bad++; say("%s: %s", what, got ? "accepted" : "refused"); }
    note("%-36s %s%s%s", what, got ? "accepted" : "refused", got ? ", " : "", got ? r->format : "");
}

static tres_t t_plan_stream (void)
{
    mirisdr_dev_t *n;
    mirisdr_stream_config_t c;
    mirisdr_stream_result_t r;

    plan_bad = 0;

    if (mirisdr_open_null(&n, "252_S16") < 0) { say("no null device"); return T_FAIL; }

    mirisdr_stream_config_default(&c);
    plan_stream(n, "the default", &c, 1, &r);
    if (strcmp(r.format, "252_S16") || r.adc != MIRISDR_IQ_BOTH || r.usb_bytes != 8126984)
    { plan_bad++; say("default: %s, adc %d, %u B/s", r.format, r.adc, r.usb_bytes); }

    c.format = "AUTO_REAL";
    plan_stream(n, "real, I", &c, 1, &r);
    if (strcmp(r.format, "504_REAL_S16") || r.adc != MIRISDR_IQ_ONLY_I) { plan_bad++; say("real: %s, adc %d", r.format, r.adc); }
    c.swap_iq = 1;
    plan_stream(n, "real, Q", &c, 1, &r);
    if (r.adc != MIRISDR_IQ_ONLY_Q) { plan_bad++; say("real swapped: adc %d", r.adc); }

    /* what each transfer mode carries */
    mirisdr_stream_config_default(&c);
    c.transfer = "ISOC1"; c.rate = 3000000; c.format = "252_S16";
    plan_stream(n, "ISOC1, 3 Msps in 252", &c, 0, &r);
    c.format = NULL;
    plan_stream(n, "ISOC1, 3 Msps automatic", &c, 1, &r);
    if (strcmp(r.format, "384_S16")) { plan_bad++; say("ISOC1 3 Msps picked %s", r.format); }
    c.transfer = "BULK"; c.rate = 12000000;
    plan_stream(n, "BULK, 12 Msps automatic", &c, 1, &r);
    if (strcmp(r.format, "384_S16")) { plan_bad++; say("BULK 12 Msps picked %s", r.format); }
    /* past 32 MB/s bulk takes the 8-bit format rather than refusing */
    c.rate = 20000000;
    plan_stream(n, "BULK, 20 Msps", &c, 1, &r);
    if (strcmp(r.format, "504_S16") || !r.decimation_bypassed) { plan_bad++; say("BULK 20 Msps: %s", r.format); }
    c.rate = 30000000;
    plan_stream(n, "BULK, 30 Msps, over the receiver", &c, 0, &r);
    c.transfer = "ISOC"; c.rate = 13000000;
    plan_stream(n, "ISOC, 13 Msps, over the reservation", &c, 0, &r);

    /* the rate range and the decimator */
    mirisdr_stream_config_default(&c);
    c.rate = 1000000;
    plan_stream(n, "1 Msps", &c, 0, &r);
    c.rate = 16000000; c.decimation_bypass = "OFF";
    plan_stream(n, "16 Msps, no bypass", &c, 0, &r);
    c.rate = 3000000; c.decimation_bypass = "ON";
    plan_stream(n, "3 Msps, bypass", &c, 1, &r);
    if (!r.decimation_bypassed) { plan_bad++; say("bypass not reported"); }

    /* names */
    mirisdr_stream_config_default(&c);
    c.format = "FOO";
    plan_stream(n, "an unknown format", &c, 0, &r);
    c.format = NULL; c.transfer = "ISOC9";
    plan_stream(n, "an unknown transfer", &c, 0, &r);
    c.transfer = NULL; c.decimation_bypass = "MAYBE";
    plan_stream(n, "an unknown decimation setting", &c, 0, &r);

    mirisdr_close(n);

    if (plan_bad) return T_FAIL;

    say("formats, transfer capacity, rate range, decimation and names");

    return T_PASS;
}

/* the stream following the tune, offline on the null device */
static volatile int label_adc, label_n;

static void label_cb (unsigned char *buf, uint32_t len, void *ctx)
{
    mirisdr_buffer_info_t in;

    (void) buf; (void) len;
    if (mirisdr_get_buffer_info((mirisdr_dev_t *) ctx, &in) == 0) { label_adc = in.adc; label_n++; }
}

static int plan_label (const char *format)
{
    dec_plan_t pl = { .blocks = 20, .cut = -1 };
    mirisdr_dev_t *n;
    uint32_t len;
    uint8_t *b = dec_gen(&pl, &len);

    label_adc = -1; label_n = 0;
    if (mirisdr_open_null(&n, format) < 0) { free(b); return -2; }
    mirisdr_feed_bulk(n, label_cb, n, 0, b, len);
    mirisdr_close(n);
    free(b);

    return label_n ? label_adc : -2;
}

/* The frontend goes with the tune: refused as a whole where a board has no notch,
 * reported in the result, kept by the single setters, and carried by scan hops */
static tres_t t_plan_frontend (void)
{
    mirisdr_dev_t *n;
    mirisdr_tune_config_t c, got, h[3];
    mirisdr_tune_result_t r;
    mirisdr_scan_t *s;
    mirisdr_scan_hop_t hb, hc;

    plan_bad = 0;

    if (mirisdr_open_null(&n, "252_S16") < 0) { say("no null device"); return T_FAIL; }

    /* a board without notches: the bias-T only */
    mirisdr_tune_config_default(&c);
    c.frequency = 100000000; c.frontend.notch = MIRISDR_NOTCH_FM;
    plan_tune(n, "a notch on a plain board", &c, 0, &r);
    if (mirisdr_set_notch(n, MIRISDR_NOTCH_FM) == 0) { plan_bad++; say("set_notch on a plain board accepted"); }
    c.frontend.notch = 0; c.frontend.bias = 1;
    plan_tune(n, "the bias-T on a plain board", &c, 1, &r);
    if (mirisdr_tune(n, 0, &c, &r) < 0) { plan_bad++; say("bias-T tune refused"); }
    if (!r.frontend.bias || mirisdr_get_bias(n) != 1) { plan_bad++; say("bias-T not in force"); }

    /* the RSP1B */
    mirisdr_set_hw_flavour(n, MIRISDR_HW_RSP1B);
    c.frontend.notch = MIRISDR_NOTCH_FM | MIRISDR_NOTCH_DAB; c.frontend.bias = 0;
    plan_tune(n, "both notches on the RSP1B", &c, 1, &r);
    if (mirisdr_tune(n, 0, &c, &r) < 0) { plan_bad++; say("notch tune refused"); }
    if (r.frontend.notch != (MIRISDR_NOTCH_FM | MIRISDR_NOTCH_DAB | MIRISDR_NOTCH_MW) || r.frontend.bias) { plan_bad++; say("result notch %x bias %d", r.frontend.notch, r.frontend.bias); }
    c.frontend.notch = 0x08;
    plan_tune(n, "an unknown notch", &c, 0, &r);

    /* FM and MW share a switch: either one asked, both in force */
    c.frontend.notch = MIRISDR_NOTCH_MW;
    plan_tune(n, "the MW notch", &c, 1, &r);
    if (r.frontend.notch != (MIRISDR_NOTCH_FM | MIRISDR_NOTCH_MW)) { plan_bad++; say("MW asked, %x in force", r.frontend.notch); }

    /* a check leaves what is in force */
    c.frontend.notch = 0;
    if (mirisdr_tune_check(n, 0, &c, &r) < 0 || r.frontend.notch) { plan_bad++; say("check notch %x", r.frontend.notch); }
    if (mirisdr_get_notch(n) != (MIRISDR_NOTCH_FM | MIRISDR_NOTCH_DAB)) { plan_bad++; say("a check changed the notch"); }

    /* the single setters change the tune's frontend and nothing else */
    if (mirisdr_set_notch(n, MIRISDR_NOTCH_DAB) < 0 || mirisdr_set_bias(n, 1) < 0) { plan_bad++; say("setters refused"); }
    mirisdr_get_tune(n, 0, &got, &r);
    if (got.frontend.notch != MIRISDR_NOTCH_DAB || !got.frontend.bias || got.frequency != 100000000 ||
        r.frontend.notch != MIRISDR_NOTCH_DAB || !r.frontend.bias) { plan_bad++; say("after the setters: notch %x bias %d at %u", got.frontend.notch, got.frontend.bias, got.frequency); }

    /* a hop that changes only the notch writes the expander, the same again does not */
    h[0] = h[1] = h[2] = got;
    h[0].frontend.notch = 0; h[1].frontend.notch = h[2].frontend.notch = MIRISDR_NOTCH_FM;
    if (mirisdr_scan_compile(n, 0, h, 3, 4, &s) < 0) { plan_bad++; say("scan with notches refused"); }
    else
    {
        mirisdr_scan_hop(s, 1, &hb);
        mirisdr_scan_hop(s, 2, &hc);
        if (hb.pre_entries <= hc.pre_entries || hb.res.frontend.notch != (MIRISDR_NOTCH_FM | MIRISDR_NOTCH_MW)) { plan_bad++; say("hop entries before register 2: %u then %u, notch %x", hb.pre_entries, hc.pre_entries, hb.res.frontend.notch); }
        mirisdr_scan_free(s);
    }

    mirisdr_close(n);
    if (!plan_bad) say("refused on a plain board, both notches and the bias-T on the RSP1B, the setters, scan hops");
    return plan_bad ? T_FAIL : T_PASS;
}

/* Bandwidth 0 at zero IF: the narrowest filter as wide as the rate, 8 MHz at most,
 * and it follows the rate. One asked for stays */
static tres_t t_plan_bw_auto (void)
{
    static const struct { uint32_t rate, bw; } want[] = {
        { 1300000, 1536000 }, { 1536000, 1536000 }, { 2048000, 5000000 }, { 5000000, 5000000 },
        { 5500000, 6000000 }, { 7000000, 7000000 }, { 8000000, 8000000 }, { 10000000, 8000000 },
    };
    mirisdr_dev_t *n;
    mirisdr_tune_config_t c, got;
    mirisdr_tune_result_t r;
    mirisdr_stream_config_t sc;
    unsigned i;

    plan_bad = 0;

    if (mirisdr_open_null(&n, "252_S16") < 0) { say("no null device"); return T_FAIL; }

    /* as opened: 2.048 Msps */
    mirisdr_get_tune(n, 0, &got, &r);
    if (got.bandwidth || r.bandwidth != 5000000) { plan_bad++; say("as opened: asked %u, in force %u", got.bandwidth, r.bandwidth); }

    /* the stream's rate moves it, in either order */
    for (i = 0; i < sizeof want / sizeof want[0]; i++)
    {
        mirisdr_get_stream(n, &sc, NULL);
        sc.rate = want[i].rate;
        if (mirisdr_set_stream(n, &sc, NULL) < 0) { plan_bad++; say("%u sps refused", want[i].rate); continue; }
        mirisdr_get_tune(n, 0, &got, &r);
        if (r.bandwidth != want[i].bw || mirisdr_get_bandwidth(n) != want[i].bw)
        { plan_bad++; say("%u sps: %u Hz, want %u", want[i].rate, r.bandwidth, want[i].bw); }
        note("%8u sps -> %8u Hz", want[i].rate, r.bandwidth);
    }

    /* asked for: kept at any rate, and get_tune gives it back */
    c = got;
    c.bandwidth = 8000000;
    if (mirisdr_tune(n, 0, &c, &r) < 0) { plan_bad++; say("8 MHz refused"); }
    mirisdr_set_sample_rate(n, 2048000);
    mirisdr_get_tune(n, 0, &got, &r);
    if (got.bandwidth != 8000000 || r.bandwidth != 8000000) { plan_bad++; say("8 MHz asked, %u then %u", got.bandwidth, r.bandwidth); }

    /* 0 again: from the rate */
    c.bandwidth = 0;
    if (mirisdr_tune(n, 0, &c, &r) < 0 || r.bandwidth != 5000000) { plan_bad++; say("back to 0: %u", r.bandwidth); }

    /* a low IF: its widest. Then to zero IF with a baseband stream at 256 ksps, which
       the tune moves up to the zero IF rates: the filter is the new rate's */
    c.if_freq = 2048000; c.low_if_auto = 1;
    mirisdr_get_stream(n, &sc, NULL);
    sc.baseband = 1; sc.rate = 256000;
    if (mirisdr_tune(n, 0, &c, &r) < 0 || r.bandwidth != 1536000) { plan_bad++; say("2048 kHz IF: %u", r.bandwidth); }
    if (mirisdr_set_stream(n, &sc, NULL) < 0) { plan_bad++; say("baseband 256 ksps refused"); }
    c.if_freq = 0; c.low_if_auto = 0;
    if (mirisdr_tune(n, 0, &c, &r) < 0) { plan_bad++; say("back to zero IF refused"); }
    else
    {
        uint32_t rate = mirisdr_get_sample_rate(n);
        uint32_t bw = rate <= 1536000 ? 1536000 : 5000000;

        if (r.bandwidth != bw) { plan_bad++; say("zero IF at %u sps: %u, want %u", rate, r.bandwidth, bw); }
        note("baseband to zero IF: %u sps, %u Hz", rate, r.bandwidth);
    }

    mirisdr_close(n);
    if (!plan_bad) say("zero IF follows the rate from 1.536 to 8 MHz, an asked filter stays, a low IF gets its widest");
    return plan_bad ? T_FAIL : T_PASS;
}

/* Room to grow: a config with its reserved part set is refused, results come back
 * with it at 0 whatever the memory held */
static int all_zero (const uint32_t *r, size_t n)
{
    while (n--) if (*r++) return 0;
    return 1;
}

static tres_t t_plan_reserved (void)
{
    mirisdr_dev_t *n, *o;
    mirisdr_tune_config_t c;
    mirisdr_tune_result_t r;
    mirisdr_stream_config_t s;
    mirisdr_stream_result_t sr;
    mirisdr_open_config_t oc;
    mirisdr_scan_status_t ss;

    plan_bad = 0;

    if (mirisdr_open_null(&n, "252_S16") < 0) { say("no null device"); return T_FAIL; }

    mirisdr_get_tune(n, 0, &c, NULL);
    plan_tune(n, "a tune config as it came", &c, 1, &r);
    c.reserved[MIRISDR_RESERVED - 1] = 1;
    plan_tune(n, "a tune config with reserved set", &c, 0, &r);
    mirisdr_get_tune(n, 0, &c, NULL);
    c.gain.reserved[0] = 1;
    plan_tune(n, "its gain with reserved set", &c, 0, &r);
    mirisdr_get_tune(n, 0, &c, NULL);
    c.frontend.reserved[0] = 1;
    plan_tune(n, "its frontend with reserved set", &c, 0, &r);
    mirisdr_get_tune(n, 0, &c, NULL);
    c.override.reserved[0] = 1;
    plan_tune(n, "its override with reserved set", &c, 0, &r);

    mirisdr_get_stream(n, &s, NULL);
    s.reserved[3] = 7;
    if (mirisdr_stream_check(n, &s, &sr) == 0) { plan_bad++; say("a stream config with reserved set accepted"); }

    mirisdr_open_config_default(&oc);
    oc.reserved[0] = 1;
    o = NULL;
    if (mirisdr_open_ex(&o, &oc) == 0) { plan_bad++; say("an open config with reserved set accepted"); mirisdr_close(o); }

    /* results over dirty memory */
    memset(&r, 0xA5, sizeof r);
    memset(&sr, 0xA5, sizeof sr);
    memset(&ss, 0xA5, sizeof ss);
    mirisdr_get_tune(n, 0, NULL, &r);
    mirisdr_get_stream(n, NULL, &sr);
    mirisdr_get_scan_status(n, &ss);
    if (!all_zero(r.reserved, MIRISDR_RESERVED) || !all_zero(r.gain.reserved, MIRISDR_RESERVED_NESTED) ||
        !all_zero(r.frontend.reserved, MIRISDR_RESERVED_NESTED)) { plan_bad++; say("tune result reserved not cleared"); }
    if (!all_zero(sr.reserved, MIRISDR_RESERVED)) { plan_bad++; say("stream result reserved not cleared"); }
    if (!all_zero(ss.reserved, MIRISDR_RESERVED)) { plan_bad++; say("scan status reserved not cleared"); }

    memset(&r, 0xA5, sizeof r);
    mirisdr_get_tune(n, 0, &c, NULL);
    mirisdr_tune_check(n, 0, &c, &r);
    if (!all_zero(r.reserved, MIRISDR_RESERVED)) { plan_bad++; say("check result reserved not cleared"); }

    mirisdr_close(n);
    if (!plan_bad) say("set reserved refused in the tune, its gain, frontend and override, the stream and the open config; results cleared");
    return plan_bad ? T_FAIL : T_PASS;
}

static tres_t t_plan_follow (void)
{
    mirisdr_dev_t *n;
    mirisdr_stream_config_t s;
    mirisdr_stream_result_t sr;
    mirisdr_tune_config_t c;
    mirisdr_tune_result_t r;

    plan_bad = 0;

    if (mirisdr_open_null(&n, "252_S16") < 0) { say("no null device"); return T_FAIL; }

    mirisdr_stream_config_default(&s);
    s.transfer = "BULK"; s.follow_tune = 1;
    if (mirisdr_set_stream(n, &s, &sr) < 0 || sr.adc != MIRISDR_IQ_BOTH) { plan_bad++; say("following stream: adc %d", sr.adc); }

    /* the result says what the switch will give before it happens */
    mirisdr_tune_config_default(&c);
    c.frequency = 100000000; c.if_freq = 450000; c.low_if_auto = 1; c.iq = MIRISDR_IQ_ONLY_I;
    if (mirisdr_tune_check(n, 0, &c, &r) < 0 || !r.inverted || r.offset != 450000) { plan_bad++; say("check: offset %d", r.offset); }

    if (mirisdr_tune(n, 0, &c, NULL) < 0) { plan_bad++; say("I only refused"); }
    mirisdr_get_stream(n, NULL, &sr);
    if (strcmp(sr.format, "504_REAL_S16") || sr.adc != MIRISDR_IQ_ONLY_I) { plan_bad++; say("I only: %s, adc %d", sr.format, sr.adc); }

    c.iq = MIRISDR_IQ_ONLY_Q;
    if (mirisdr_tune(n, 0, &c, NULL) < 0) { plan_bad++; say("Q only refused"); }
    mirisdr_get_stream(n, NULL, &sr);
    if (sr.adc != MIRISDR_IQ_ONLY_Q) { plan_bad++; say("Q only: adc %d", sr.adc); }

    mirisdr_tune_config_default(&c);
    if (mirisdr_tune(n, 0, &c, NULL) < 0) { plan_bad++; say("back to zero IF refused"); }
    mirisdr_get_stream(n, NULL, &sr);
    if (strcmp(sr.format, "252_S16") || sr.adc != MIRISDR_IQ_BOTH) { plan_bad++; say("zero IF: %s, adc %d", sr.format, sr.adc); }

    /* both formats given */
    s.format = "336_S16"; s.format_single = "768_REAL_S16";
    if (mirisdr_set_stream(n, &s, NULL) < 0) { plan_bad++; say("explicit formats refused"); }
    c.if_freq = 450000; c.low_if_auto = 1; c.iq = MIRISDR_IQ_ONLY_I;
    mirisdr_tune(n, 0, &c, NULL);
    mirisdr_get_stream(n, NULL, &sr);
    if (strcmp(sr.format, "768_REAL_S16")) { plan_bad++; say("explicit single: %s", sr.format); }
    mirisdr_tune_config_default(&c);
    mirisdr_tune(n, 0, &c, NULL);
    mirisdr_get_stream(n, NULL, &sr);
    if (strcmp(sr.format, "336_S16")) { plan_bad++; say("explicit both: %s", sr.format); }

    /* each format of the right kind */
    s.format = "AUTO_REAL"; s.format_single = NULL;
    plan_stream(n, "following, a real format for both", &s, 0, &sr);
    s.format = NULL; s.format_single = "252_S16";
    plan_stream(n, "following, a complex format for one", &s, 0, &sr);

    /* what bulk is allowed: the host's business, apart from the receiver's own limit */
    mirisdr_stream_config_default(&s);
    s.transfer = "BULK"; s.rate = 12000000;
    plan_stream(n, "BULK 12 Msps, default capacity", &s, 1, &sr);
    if (strcmp(sr.format, "384_S16")) { plan_bad++; say("default capacity picked %s", sr.format); }
    s.usb_capacity = 50000000;
    plan_stream(n, "BULK 12 Msps, 50 MB/s capacity", &s, 1, &sr);
    if (strcmp(sr.format, "252_S16")) { plan_bad++; say("50 MB/s capacity picked %s", sr.format); }
    s.usb_capacity = 0; s.format = "252_S16";
    plan_stream(n, "BULK 12 Msps in 252", &s, 1, &sr);
    s.rate = 14000000;
    plan_stream(n, "BULK 14 Msps in 252, over the receiver", &s, 0, &sr);
    s.transfer = "ISOC"; s.rate = 7000000;
    plan_stream(n, "ISOC 7 Msps in 252", &s, 0, &sr);

    mirisdr_close(n);

    /* each buffer says what it holds */
    if (plan_label("504_REAL_S16") != MIRISDR_IQ_ONLY_I) { plan_bad++; say("real buffers labelled %d", label_adc); }
    if (plan_label("504_S8") != MIRISDR_IQ_BOTH) { plan_bad++; say("complex buffers labelled %d", label_adc); }

    if (plan_bad) return T_FAIL;

    say("following the tune both ways and with given formats, bulk and isochronous limits, buffer labels");

    return T_PASS;
}

/* the stream following the tune, on the device and streaming */
static tres_t t_stream_follow (void)
{
    static const int seq[] = { MIRISDR_IQ_ONLY_I, MIRISDR_IQ_ONLY_Q, MIRISDR_IQ_BOTH, MIRISDR_IQ_ONLY_I, MIRISDR_IQ_BOTH };
    mirisdr_stream_config_t s;
    mirisdr_stream_result_t sr;
    mirisdr_tune_config_t c;
    mirisdr_stream_stats_t d;
    unsigned i;
    double sps;

    pump_stop();

    mirisdr_stream_config_default(&s);
    s.transfer = "BULK"; s.follow_tune = 1;
    mirisdr_tune_config_default(&c);
    if (mirisdr_tune(dev, 0, &c, NULL) < 0 || mirisdr_set_stream(dev, &s, NULL) < 0) { say("setup refused"); return T_FAIL; }

    if (pump_start() < 0) { say("stream did not start"); return T_FAIL; }

    for (i = 0; i < sizeof seq / sizeof seq[0]; i++)
    {
        mirisdr_tune_config_default(&c);
        c.frequency = 100000000; c.iq = seq[i];
        if (seq[i] != MIRISDR_IQ_BOTH) { c.if_freq = 450000; c.low_if_auto = 1; }

        if (mirisdr_tune(dev, 0, &c, NULL) < 0) { pump_stop(); say("tune %u refused", i); return T_FAIL; }
        usleep(300000);
        mirisdr_get_stream(dev, NULL, &sr);
        sps = stream_rate(0.35, &d);

        note("iq %d: stream %s adc %d, buffers adc %d, %.0f sps", seq[i], sr.format, sr.adc, pump_adc, sps);

        if (sr.adc != seq[i] || pump_adc != seq[i] || !within(sps, 2000000, 0.03))
        { pump_stop(); say("iq %d: stream adc %d, buffers %d, %.0f sps", seq[i], sr.adc, pump_adc, sps); return T_FAIL; }
    }

    pump_stop();

    if (pump_backwards) { say("%d buffers went back in position", pump_backwards); return T_FAIL; }

    mirisdr_tune_config_default(&c);
    mirisdr_stream_config_default(&s);
    s.transfer = "BULK";
    mirisdr_tune(dev, 0, &c, NULL);
    mirisdr_set_stream(dev, &s, NULL);

    say("%u tunes switched the running stream, buffers labelled, rate held, positions forward", (unsigned) (sizeof seq / sizeof seq[0]));

    return T_PASS;
}

/* ------------------------------------------------------------------ */
/* the configs on the device                                           */
/* ------------------------------------------------------------------ */

static tres_t t_tune_api (void)
{
    mirisdr_tune_config_t c, got;
    mirisdr_tune_result_t r;
    mirisdr_stream_config_t s;
    mirisdr_tuner_status_t a, b;

    pump_stop();

    mirisdr_tune_config_default(&c);
    c.frequency = 100000000; c.gain.mode = MIRISDR_GAIN_TOTAL; c.gain.total = 30;
    if (mirisdr_tune(dev, 0, &c, &r) < 0) { say("a plain tune refused"); return T_FAIL; }
    if (mirisdr_get_center_freq(dev) != 100000000 || mirisdr_get_tuner_gain(dev) != 30)
    { say("tuned to %u at %d dB", mirisdr_get_center_freq(dev), mirisdr_get_tuner_gain(dev)); return T_FAIL; }

    c.if_freq = 450000; c.bandwidth = 300000; c.low_if_auto = 1; c.iq = MIRISDR_IQ_ONLY_I; c.gain.mode = MIRISDR_GAIN_KEEP;
    if (mirisdr_tune(dev, 0, &c, &r) < 0) { say("a low IF tune refused"); return T_FAIL; }
    if (mirisdr_get_center_freq(dev) != 100450000 || mirisdr_get_if_freq(dev) != 450000 ||
        mirisdr_get_bandwidth(dev) != 300000 || r.iq != MIRISDR_IQ_ONLY_I || r.offset != 450000 || !r.inverted)
    { say("low IF: LO %u, IF %u, bandwidth %u, iq %d, offset %d", mirisdr_get_center_freq(dev),
          mirisdr_get_if_freq(dev), mirisdr_get_bandwidth(dev), r.iq, r.offset); return T_FAIL; }

    mirisdr_get_tune(dev, 0, &got, NULL);
    if (got.frequency != 100000000 || got.if_freq != 450000 || !got.low_if_auto)
    { say("get_tune gave %u, IF %u", got.frequency, got.if_freq); return T_FAIL; }

    /* a refused tune leaves everything as it was */
    c.if_freq = 0; c.bandwidth = 0;
    if (mirisdr_tune(dev, 0, &c, NULL) == 0) { say("zero IF with I only accepted"); return T_FAIL; }
    if (mirisdr_get_center_freq(dev) != 100450000) { say("a refused tune moved the LO"); return T_FAIL; }

    /* the single setters keep the rest of the tune; set_center_freq always sets the LO */
    if (mirisdr_set_center_freq(dev, 200000000) < 0 || mirisdr_get_center_freq(dev) != 200000000 ||
        mirisdr_get_if_freq(dev) != 450000)
    { say("set_center_freq gave LO %u, IF %u", mirisdr_get_center_freq(dev), mirisdr_get_if_freq(dev)); return T_FAIL; }
    mirisdr_get_tune(dev, 0, &got, NULL);
    if (got.low_if_auto || got.lo_offset) { say("set_center_freq kept low_if_auto or lo_offset"); return T_FAIL; }

    /* the stream's converter and the tuner's output: refused both ways */
    mirisdr_get_stream(dev, &s, NULL);
    s.format = "AUTO_REAL"; s.swap_iq = 1;
    if (mirisdr_set_stream(dev, &s, NULL) == 0) { say("a real stream on Q accepted with only I on"); return T_FAIL; }
    s.swap_iq = 0;
    if (mirisdr_set_stream(dev, &s, NULL) < 0) { say("a real stream on I refused"); return T_FAIL; }
    c.if_freq = 450000; c.iq = MIRISDR_IQ_ONLY_Q;
    if (mirisdr_tune(dev, 0, &c, NULL) == 0) { say("Q only accepted with the stream on I"); return T_FAIL; }

    /* holds in the tune go out with it and read back; the status is apart */
    s.format = NULL;
    if (mirisdr_set_stream(dev, &s, NULL) < 0) { say("could not go back to a complex stream"); return T_FAIL; }
    mirisdr_tune_config_default(&c);
    c.frequency = 159000000;
    if (mirisdr_tune(dev, 0, &c, NULL) < 0) { say("159 MHz refused"); return T_FAIL; }
    usleep(20000);
    if (mirisdr_get_tuner_status(dev, 0, &a) < 0 || a.coarse < 0) { say("no calibrated readback"); return T_FAIL; }
    if (mirisdr_get_tuner_status(dev, 1, &b) == 0) { say("a second tuner's status read"); return T_FAIL; }

    c.override.hold_vco = 1; c.override.coarse = a.coarse; c.override.fine = a.fine < 31 ? a.fine + 1 : 30;
    c.override.hold_upconv = 1; c.override.upconv = 5;
    if (mirisdr_tune(dev, 0, &c, NULL) < 0) { say("a tune with holds refused"); return T_FAIL; }
    usleep(20000);
    mirisdr_get_tune(dev, 0, &got, NULL);
    if (mirisdr_get_tuner_status(dev, 0, &b) < 0 || b.coarse != c.override.coarse || b.fine != c.override.fine ||
        b.upconv != 5 || !got.override.hold_vco)
    { mirisdr_set_tuner_override(dev, 0, NULL); say("held fine %u upconv %u read back fine %u upconv %u",
                                                    c.override.fine, 5, b.fine, b.upconv); return T_FAIL; }

    /* cleared, the next tune searches again */
    memset(&c.override, 0, sizeof c.override);
    c.frequency = 159100000;
    mirisdr_tune(dev, 0, &c, NULL);
    c.frequency = 159000000;
    if (mirisdr_tune(dev, 0, &c, NULL) < 0) { say("a tune without holds refused"); return T_FAIL; }
    usleep(20000);
    if (mirisdr_get_tuner_status(dev, 0, &b) < 0 || b.coarse != a.coarse || b.fine + 1 < a.fine || b.fine > a.fine + 1)
    { say("cleared, coarse %d fine %u, was coarse %d fine %u", b.coarse, b.fine, a.coarse, a.fine); return T_FAIL; }
    note("searched fine %u, held %u, searched again %u", a.fine, got.override.fine, b.fine);

    /* back to the defaults */
    mirisdr_tune_config_default(&c);
    if (mirisdr_tune(dev, 0, &c, NULL) < 0 || mirisdr_set_stream(dev, &s, NULL) < 0)
    { say("could not go back to the defaults"); return T_FAIL; }

    /* stages with the tune, kept through a retune; a total is split again per band */
    {
        mirisdr_tune_config_t g;

        mirisdr_tune_config_default(&g);
        g.frequency = 100000000; g.gain.mode = MIRISDR_GAIN_STAGES; g.gain.mixer = 1; g.gain.baseband = 20;
        if (mirisdr_tune(dev, 0, &g, &r) < 0) { say("a tune with stages refused"); return T_FAIL; }
        if (mirisdr_get_lna_gain(dev) != 0 || mirisdr_get_mixer_gain(dev) != 19 || mirisdr_get_baseband_gain(dev) != 20 ||
            r.gain.total != 39 || mirisdr_get_tuner_gain(dev) != 39)
        { say("stages: lna %d mixer %d baseband %d, total %d / %d", mirisdr_get_lna_gain(dev), mirisdr_get_mixer_gain(dev),
              mirisdr_get_baseband_gain(dev), r.gain.total, mirisdr_get_tuner_gain(dev)); return T_FAIL; }
        g.frequency = 433000000; g.gain.mode = MIRISDR_GAIN_KEEP;
        if (mirisdr_tune(dev, 0, &g, &r) < 0 || r.gain.lna || r.gain.baseband != 20)
        { say("stages after a retune: lna %d baseband %d", r.gain.lna, r.gain.baseband); return T_FAIL; }
        g.gain.mode = MIRISDR_GAIN_TOTAL; g.gain.total = 80;
        if (mirisdr_tune(dev, 0, &g, &r) < 0 || r.gain.total != 80) { say("80 dB in band IV/V gave %d", r.gain.total); return T_FAIL; }
        g.frequency = 100000000; g.gain.mode = MIRISDR_GAIN_KEEP;
        if (mirisdr_tune(dev, 0, &g, &r) < 0 || r.gain.total != 80 || !r.gain.lna)
        { say("80 dB back in VHF gave %d, lna %d", r.gain.total, r.gain.lna); return T_FAIL; }
    }

    mirisdr_tune_config_default(&c);
    c.gain.mode = MIRISDR_GAIN_TOTAL; c.gain.total = 43;    /* the library's default */
    mirisdr_tune(dev, 0, &c, NULL);

    say("tune, low IF auto, single output, holds, gain stages, refusals leave the state, setters keep the rest");

    return T_PASS;
}

static tres_t t_stream_api (void)
{
    mirisdr_stream_config_t c, got;
    mirisdr_stream_result_t r;
    mirisdr_stream_stats_t d;
    double sps;

    pump_stop();

    mirisdr_stream_config_default(&c);
    c.transfer = "BULK";
    if (mirisdr_set_stream(dev, &c, &r) < 0 || strcmp(r.format, "252_S16"))
    { say("the default stream gave %s", r.format); return T_FAIL; }

    mirisdr_get_stream(dev, &got, NULL);
    if (strcmp(got.format, "AUTO") || strcmp(got.transfer, "BULK") || strcmp(got.decimation_bypass, "AUTO"))
    { say("get_stream gave %s %s %s", got.format, got.transfer, got.decimation_bypass); return T_FAIL; }

    if (pump_start() < 0) { say("stream did not start"); return T_FAIL; }

    /* a rate change restarts the stream */
    c.rate = 4000000;
    if (mirisdr_set_stream(dev, &c, &r) < 0) { pump_stop(); say("4 Msps refused while streaming"); return T_FAIL; }
    usleep(200000);
    sps = stream_rate(0.35, &d);
    if (!within(sps, 4000000, 0.03)) { pump_stop(); say("4 Msps streams at %.0f", sps); return T_FAIL; }

    /* the same again changes nothing */
    if (mirisdr_set_stream(dev, &c, NULL) < 0) { pump_stop(); say("the same config refused"); return T_FAIL; }

    /* real while streaming: the buffers say so */
    c.format = "AUTO_REAL";
    if (mirisdr_set_stream(dev, &c, NULL) < 0) { pump_stop(); say("a real format refused while streaming"); return T_FAIL; }
    usleep(300000);
    if (pump_adc != MIRISDR_IQ_ONLY_I) { pump_stop(); say("real buffers say %d", pump_adc); return T_FAIL; }

    sps = stream_rate(0.35, &d);
    pump_stop();
    if (!within(sps, 4000000, 0.03)) { say("streams at %.0f after the switch", sps); return T_FAIL; }
    if (pump_backwards) { say("%d buffers went back in position", pump_backwards); return T_FAIL; }

    mirisdr_stream_config_default(&c);
    c.transfer = "BULK";
    mirisdr_set_stream(dev, &c, NULL);

    say("set and read back, a rate change and a real switch while streaming, buffers labelled");

    return T_PASS;
}

/* Hops from lo to hi in steps, every band's synthesizer threshold as given */
static uint32_t scan_hops (mirisdr_tune_config_t *h, uint32_t max, uint32_t lo, uint32_t hi, uint32_t step,
                           uint32_t thresh)
{
    uint32_t n = 0, f;

    for (f = lo; (f <= hi) && (n < max); f += step, n++)
    {
        mirisdr_tune_config_default(&h[n]);
        h[n].frequency = f;
        h[n].synth_thresh = thresh;
    }

    return n;
}

static tres_t t_plan_scan (void)
{
    static mirisdr_tune_config_t h[300];
    mirisdr_dev_t *n;
    mirisdr_tune_config_t c;
    mirisdr_tune_result_t r;
    mirisdr_scan_t *s;
    mirisdr_scan_hop_t hi;
    const mirisdr_list_entry_t *e;
    uint32_t nh, k, ch, two = 0, total = 0, biggest = 0;
    int ne;

    plan_bad = 0;

    if (mirisdr_open_null(&n, "252_S16") < 0) { say("no null device"); return T_FAIL; }
    mirisdr_set_hw_flavour(n, MIRISDR_HW_RSP1B);

    /* the grid: 3 MHz / 6 in VHF */
    mirisdr_tune_config_default(&c);
    c.frequency = 100300000; c.synth_thresh = 6;
    plan_tune(n, "VHF on a 500 kHz grid", &c, 1, &r);
    if (r.lo != 100500000) { plan_bad++; say("grid LO %u, not 100500000", r.lo); }
    plan_near(r.offset, -200000, 0, "grid offset");
    c.synth_thresh = 4096;
    plan_tune(n, "threshold 4096", &c, 0, &r);

    /* 52 MHz to 948 MHz in 8 MHz hops */
    nh = scan_hops(h, 300, 52000000, 948000000, 8000000, 6);
    if (mirisdr_scan_compile(n, 0, h, nh, 4, &s) < 0) { mirisdr_close(n); say("compile refused"); return T_FAIL; }

    for (ch = 0; ch < mirisdr_scan_chunks(s); ch++)
    {
        ne = mirisdr_scan_chunk(s, ch, &e);
        total += (uint32_t) ne;
        if (ne < 2 || ne > 62) { plan_bad++; say("chunk %u has %d entries", ch, ne); }
        else if (e[0].reg != 0x10) { plan_bad++; say("chunk %u starts with %02x, not the marker", ch, e[0].reg); }
        else if (e[ne - 1].reg != MIRISDR_LIST_WAIT_IRQ) { plan_bad++; say("chunk %u does not end on a wait", ch); }
    }

    for (k = 0, ch = 0; k < nh; k++)
    {
        mirisdr_scan_hop(s, k, &hi);
        if (hi.entries == 2) two++;
        if (hi.entries > biggest) biggest = hi.entries;
        if (hi.chunk != ch && hi.chunk != ch + 1) { plan_bad++; say("hop %u in chunk %u after %u", k, hi.chunk, ch); }
        ch = hi.chunk;
    }

    mirisdr_scan_hop(s, 0, &hi);
    note("%u hops in %u chunks, %u entries; %u hops of 2 entries, the first %u (register 2 after %u words), "
         "the biggest %u", nh, mirisdr_scan_chunks(s), total, two, hi.entries, hi.pre_words, biggest);
    if (two + 12 < nh) { plan_bad++; say("only %u of %u hops are register 2 and the wait", two, nh); }
    mirisdr_scan_free(s);

    /* L band needs the LNA code held */
    nh = scan_hops(h, 300, 1200000000, 1240000000, 8000000, 48);
    if (mirisdr_scan_compile(n, 0, h, nh, 4, &s) == 0) { plan_bad++; say("L band without a held LNA code compiled"); mirisdr_scan_free(s); }
    for (k = 0; k < nh; k++) { h[k].override.hold_lna = 1; h[k].override.lna_cal = 8; }
    if (mirisdr_scan_compile(n, 0, h, nh, 4, &s) < 0) { plan_bad++; say("L band with the LNA code held refused"); }
    else mirisdr_scan_free(s);

    mirisdr_close(n);

    if (plan_bad) return T_FAIL;

    say("grid tunes, chunks led by the marker and ending on a wait, hops of 2 entries in a band, "
        "L band needs its LNA code");

    return T_PASS;
}

/* Learned codes come back when replayed */
static tres_t t_scan_learn (void)
{
    static const uint32_t f[] = { 100000000, 433000000, 1300000000 };
    mirisdr_tune_config_t c;
    mirisdr_tuner_status_t st;
    unsigned i;

    pump_stop();

    for (i = 0; i < sizeof f / sizeof f[0]; i++)
    {
        mirisdr_tune_config_default(&c);
        c.frequency = f[i];
        if (mirisdr_tune_learn(dev, 0, &c) < 0) { say("%u Hz: could not learn", f[i]); return T_FAIL; }

        /* somewhere else, then back with the holds */
        mirisdr_set_center_freq(dev, 200000000);
        usleep(10000);
        if (mirisdr_tune(dev, 0, &c, NULL) < 0) { say("%u Hz: replay refused", f[i]); return T_FAIL; }
        usleep(10000);
        if (mirisdr_get_tuner_status(dev, 0, &st) < 0) { say("%u Hz: no readback", f[i]); return T_FAIL; }

        note("%10u Hz: learned coarse %u fine %2u unknown %2u upconv %2u lna %2u filter %2u, read %d %2u - %2u %2u %2u",
             f[i], c.override.coarse, c.override.fine, c.override.unknown, c.override.upconv,
             c.override.lna_cal, c.override.filter, st.coarse, st.fine, st.upconv, st.lna_cal, st.filter);

        if (st.coarse != c.override.coarse || st.fine != c.override.fine || st.upconv != c.override.upconv ||
            st.lna_cal != c.override.lna_cal)
        { say("%u Hz: replayed codes differ", f[i]); mirisdr_set_tuner_override(dev, 0, NULL); return T_FAIL; }
    }

    mirisdr_set_tuner_override(dev, 0, NULL);
    mirisdr_set_center_freq(dev, 100000000);

    say("three bands learned and replayed to the same codes");

    return T_PASS;
}

/* What the scan reports, from the stream thread */
#define SCAN_LOG_MAX    4096

static mirisdr_scan_report_t scan_log[SCAN_LOG_MAX];
static volatile uint32_t scan_logged;

static void scan_cb (const mirisdr_scan_report_t *r, void *ctx)
{
    (void) ctx;
    if (scan_logged < SCAN_LOG_MAX) scan_log[scan_logged] = *r;
    scan_logged++;
}

static tres_t t_scan_run (void)
{
    static mirisdr_tune_config_t h[300];
    mirisdr_stream_config_t sc;
    mirisdr_tune_result_t r;
    mirisdr_scan_t *s;
    mirisdr_scan_status_t st;
    uint32_t nh, k, passes = 2, learned = 0, bad = 0, flagged = 0, pkt, late = 0, shorted = 0, fl[3] = { 0 };
    int64_t shortest = INT64_MAX, longest = 0;
    int fr = 1, loops = 0;
    tres_t res = T_PASS;

    pump_stop();

    mirisdr_stream_config_default(&sc);
    sc.transfer = "BULK";
    sc.rate = 8000000;
    if (mirisdr_set_stream(dev, &sc, NULL) < 0) { say("8 Msps bulk refused"); return T_FAIL; }

    /* 52 MHz to 1948 MHz in 8 MHz hops, L band learned */
    nh = scan_hops(h, 300, 52000000, 1948000000, 8000000, 6);
    for (k = 0; k < nh; k++)
    {
        if (mirisdr_tune_check(dev, 0, &h[k], &r) < 0) { say("%u Hz refused", h[k].frequency); return T_FAIL; }
        if (r.band != MIRISDR_BAND_L) continue;
        if (mirisdr_tune_learn(dev, 0, &h[k]) < 0) { say("%u Hz: could not learn", h[k].frequency); return T_FAIL; }
        learned++;
    }

    if (mirisdr_scan_compile(dev, 0, h, nh, 8, &s) < 0) { say("compile refused"); return T_FAIL; }

    scan_logged = 0;
    if (pump_start() < 0) { mirisdr_scan_free(s); say("stream did not start"); return T_FAIL; }
    if (mirisdr_scan_start(dev, s, passes, scan_cb, NULL) < 0)
    { pump_stop(); mirisdr_scan_free(s); say("scan did not start"); return T_FAIL; }

    while ((fr = mirisdr_scan_feed(dev)) == 1 && loops++ < 20000) usleep(500);
    mirisdr_get_scan_status(dev, &st);
    usleep(50000);
    mirisdr_scan_stop(dev);
    pump_stop();

    note("%u hops (%u L band learned) in %u chunks, %u passes: %u reports, %u restarts, feed %d",
         nh, learned, mirisdr_scan_chunks(s), passes, scan_logged, st.restarts, fr);

    /* every hop gets its dwell: the next one starts no earlier than the interrupt
       period its own register 2 fell in allows, give or take a packet of toggle jitter */
    pkt = scan_log[0].length / (8 * 4);
    for (k = 0; k < scan_logged && k < SCAN_LOG_MAX; k++)
    {
        const mirisdr_scan_report_t *a = &scan_log[k];
        int64_t len;

        if (a->flags) flagged++;
        if (a->flags & MIRISDR_SCAN_MISSED) fl[0]++;
        if (a->flags & MIRISDR_SCAN_RESTART) fl[1]++;
        if (a->flags & MIRISDR_SCAN_LOST) fl[2]++;
        if (a->hop != k % nh || a->pass != k / nh) bad++;
        if (!k || (k == nh) || (k + 1 >= scan_logged)) continue;

        len = (int64_t) (scan_log[k + 1].index - a->index) / (int64_t) pkt;
        if (len < shortest) shortest = len;
        if (len > longest) longest = len;
        if (len < 8 * 4 - 4 - 1) shorted++;
        if (len > 8 * 4 + 1) late++;
    }

    note("hops %lld to %lld packets long for a dwell of 32; %u longer", (long long) shortest,
         (long long) longest, late);

    mirisdr_scan_free(s);
    mirisdr_stream_config_default(&sc);
    sc.transfer = "BULK";
    mirisdr_set_stream(dev, &sc, NULL);

    if (fr != 0) { say("the feed ended with %d", fr); res = T_FAIL; }
    else if (scan_logged != nh * passes) { say("%u reports for %u hops", scan_logged, nh * passes); res = T_FAIL; }
    else if (bad) { say("%u reports out of order", bad); res = T_FAIL; }
    else if (flagged) { say("%u reports flagged: %u missed packets, %u restarts, %u lost", flagged, fl[0], fl[1], fl[2]); res = T_FAIL; }
    else if (shorted) { say("%u hops shorter than their dwell allows", shorted); res = T_FAIL; }
    else say("%u hops x %u passes reported in order, %lld to %lld packets for a dwell of 32, %u restarts",
             nh, passes, (long long) shortest, (long long) longest, st.restarts);

    return res;
}

/* A feeder far too slow: the list runs dry, starts again with the next chunk, and the
   reports still come in order, the restarts flagged */
static tres_t t_scan_dry (void)
{
    static mirisdr_tune_config_t h[300];
    mirisdr_stream_config_t sc;
    mirisdr_scan_t *s;
    mirisdr_scan_status_t st;
    uint32_t nh, k, bad = 0, restarted = 0, lost = 0;
    int fr = 1, loops = 0;
    tres_t res = T_PASS;

    pump_stop();

    mirisdr_stream_config_default(&sc);
    sc.transfer = "BULK";
    sc.rate = 8000000;
    if (mirisdr_set_stream(dev, &sc, NULL) < 0) { say("8 Msps bulk refused"); return T_FAIL; }

    /* VHF and band III only, 8 interrupts (~1.3 ms) a hop: a chunk lasts ~40 ms */
    nh = scan_hops(h, 300, 52000000, 248000000, 2000000, 6);
    if (mirisdr_scan_compile(dev, 0, h, nh, 8, &s) < 0) { say("compile refused"); return T_FAIL; }

    scan_logged = 0;
    if (pump_start() < 0) { mirisdr_scan_free(s); say("stream did not start"); return T_FAIL; }
    if (mirisdr_scan_start(dev, s, 1, scan_cb, NULL) < 0)
    { pump_stop(); mirisdr_scan_free(s); say("scan did not start"); return T_FAIL; }

    while ((fr = mirisdr_scan_feed(dev)) == 1 && loops++ < 200) usleep(100000);
    mirisdr_get_scan_status(dev, &st);
    usleep(50000);
    mirisdr_scan_stop(dev);
    pump_stop();

    for (k = 0; k < scan_logged && k < SCAN_LOG_MAX; k++)
    {
        if (scan_log[k].hop != k) bad++;
        if (scan_log[k].flags & MIRISDR_SCAN_RESTART) restarted++;
        if (scan_log[k].flags & MIRISDR_SCAN_LOST) lost++;
    }

    note("%u hops in %u chunks: %u reports, %u restarts, %u reports flagged restarted, feed %d",
         nh, mirisdr_scan_chunks(s), scan_logged, st.restarts, restarted, fr);
    mirisdr_scan_free(s);

    mirisdr_stream_config_default(&sc);
    sc.transfer = "BULK";
    mirisdr_set_stream(dev, &sc, NULL);

    if (fr != 0) { say("the feed ended with %d", fr); res = T_FAIL; }
    else if (scan_logged != nh || bad) { say("%u reports for %u hops, %u out of order", scan_logged, nh, bad); res = T_FAIL; }
    else if (!st.restarts) { say("the list never ran dry"); res = T_FAIL; }
    else if (restarted != st.restarts || lost) { say("%u restarts, %u flagged, %u lost", st.restarts, restarted, lost); res = T_FAIL; }
    else say("ran dry %u times, every hop reported in order, each restart flagged", st.restarts);

    return res;
}

/* ------------------------------------------------------------------ */
/* baseband: the down-converter, offline on the null device            */
/* ------------------------------------------------------------------ */

/* Synthetic streams as the tuner would give them with the LO the IF above
   100 MHz: a tone at RF 100 MHz + delta lies at -IF + delta in the complex
   stream, an image tone at +IF + img. Blocks are 252_S16 (both outputs) or
   504_REAL_S16 (the I output), stamped, their counter the first sample's index. */
static struct {
    float    *x;            /* the output pairs */
    size_t   n, cap;
    uint64_t want_sample;
    uint32_t rate, len;
    int      bad_info, calls;
    uint64_t gaps, gap_samples, gap_filled, last_index_jump;
    mirisdr_dev_t *d;
} bbo;

static void bb_cb (unsigned char *buf, uint32_t len, void *ctx)
{
    mirisdr_buffer_info_t in;
    uint32_t k;

    (void) ctx;
    bbo.calls++;
    if (mirisdr_get_buffer_info(bbo.d, &in) < 0 || in.type != MIRISDR_SAMPLE_F32 || in.rate != bbo.rate ||
        in.adc != MIRISDR_IQ_BOTH || in.sample != bbo.want_sample || (bbo.len && len != bbo.len))
        bbo.bad_info++;
    for (k = 0; k < in.gaps_len; k++) {
        bbo.gaps++;
        bbo.gap_samples += in.gaps[k].samples;
        bbo.gap_filled += in.gaps[k].filled;
    }
    if (in.index > in.sample) bbo.last_index_jump = in.index - in.sample;
    bbo.want_sample += len / 8;

    if (bbo.n + len / 4 > bbo.cap) {
        bbo.cap = (bbo.n + len / 4) * 2;
        bbo.x = realloc(bbo.x, bbo.cap * sizeof(float));
    }
    memcpy(bbo.x + bbo.n, buf, len);
    bbo.n += len / 4;
}

static int bb_swap;      /* I and Q swapped by the receiver: the band at +IF */

static void bb_put14 (uint8_t *b, double v)
{
    int w = (int) lrint(v * 8191.0) & 0x3fff;

    b[0] = (uint8_t) w;
    b[1] = (uint8_t) (w >> 8);
}

/* blocks of a tone at -IF + delta (amplitude a) and one at +IF + img (amplitude ai);
   'skip' blocks from block 'skip_at' go missing */
static uint8_t *bb_gen (int real, double fs, double ifhz, double delta, double a, double img, double ai,
                        int blocks, int skip_at, int skip, uint32_t *len)
{
    int spb = real ? 504 : 252, k, i, o = 0;
    uint8_t *data = calloc((size_t) blocks, 1024);
    uint64_t idx = 0;

    for (k = 0; k < blocks; k++) {
        if (k == skip_at) idx += (uint64_t) skip * spb;
        uint8_t *h = data + 1024 * o++;
        h[0] = (uint8_t) idx; h[1] = (uint8_t) (idx >> 8); h[2] = (uint8_t) (idx >> 16); h[3] = (uint8_t) (idx >> 24);
        memcpy(h + 8, "BVDB", 4);
        for (i = 0; i < spb; i++) {
            double n = (double) (idx + (uint64_t) i);
            double p1 = 2 * M_PI * (-ifhz + delta) * n / fs, p2 = 2 * M_PI * (ifhz + img) * n / fs;
            double re = a * cos(p1) + ai * cos(p2), im = a * sin(p1) + ai * sin(p2);

            /* swapped, the receiver hands Q as I: the spectrum mirrored */
            if (bb_swap) { double t = re; re = im; im = t; }

            if (real) bb_put14(h + 16 + 2 * i, re);
            else {
                bb_put14(h + 16 + 4 * i, re);
                bb_put14(h + 18 + 4 * i, im);
            }
        }
        idx += (uint64_t) spb;
    }
    *len = (uint32_t) o * 1024;
    return data;
}

/* a tone's amplitude in the output, Hann window, f in Hz at rate fs */
static double bb_tone (const float *x, size_t n, double f, double fs)
{
    double re = 0, im = 0, w, c, s;
    size_t k;

    for (k = 0; k < n; k++) {
        w = 0.5 - 0.5 * cos(2 * M_PI * k / n);
        c = cos(2 * M_PI * f * k / fs);
        s = -sin(2 * M_PI * f * k / fs);
        re += w * (x[2 * k] * c - x[2 * k + 1] * s);
        im += w * (x[2 * k] * s + x[2 * k + 1] * c);
    }
    return sqrt(re * re + im * im) / (n / 2.0);
}

/* A null device tuned to 100 MHz with this IF and output, baseband at rate */
static int bb_open (int real, uint32_t ifhz, uint32_t rate, mirisdr_stream_result_t *sr)
{
    mirisdr_tune_config_t tc;
    mirisdr_stream_config_t sc;

    if (mirisdr_open_null(&bbo.d, real ? "504_REAL_S16" : "252_S16") < 0) return -1;
    mirisdr_get_tune(bbo.d, 0, &tc, NULL);
    tc.frequency = 100000000;
    tc.if_freq = ifhz;
    tc.bandwidth = 0;
    tc.iq = real ? MIRISDR_IQ_ONLY_I : MIRISDR_IQ_BOTH;
    if (mirisdr_tune(bbo.d, 0, &tc, NULL) < 0) return -1;
    mirisdr_get_stream(bbo.d, &sc, NULL);
    sc.baseband = 1;
    sc.rate = rate;
    sc.transfer = "BULK";
    sc.format = "252_S16";
    sc.format_single = "504_REAL_S16";
    sc.swap_iq = bb_swap;
    if (mirisdr_set_stream(bbo.d, &sc, sr) < 0) return -1;

    free(bbo.x);
    memset(&bbo, 0, sizeof bbo - sizeof bbo.d);
    bbo.rate = sr->rate;
    return 0;
}

static void bb_feed_cb (const uint8_t *data, uint32_t len, uint32_t buf, mirisdr_read_async_cb_t cb)
{
    uint32_t at, t;

    for (at = 0; at < len; at += t) {
        t = 1024 * (1 + (uint32_t) rand() % 24);
        if (t > len - at) t = len - at;
        mirisdr_feed_bulk(bbo.d, cb, NULL, buf, data + at, t);
    }
}

static void bb_feed (const uint8_t *data, uint32_t len, uint32_t buf)
{
    bb_feed_cb(data, len, buf, bb_cb);
}

static void dec_cb_count (unsigned char *buf, uint32_t len, void *ctx)
{
    (void) buf; (void) ctx;
    bbo.n += len / 4;
}

static tres_t t_bb_plan (void)
{
    mirisdr_dev_t *n;
    mirisdr_tune_config_t c;
    mirisdr_stream_config_t s;
    mirisdr_stream_result_t sr;
    mirisdr_tune_result_t tr;
    static const struct { uint32_t ifhz; int real; uint32_t rate; uint32_t adc; uint32_t decim; } ok[] = {
        { 450000, 0, 1800000, 1800000, 1 }, { 450000, 0, 900000, 1800000, 2 }, { 450000, 0, 28125, 1800000, 64 },
        { 450000, 1, 900000, 1800000, 2 },  { 450000, 1, 225000, 1800000, 8 },
        { 1620000, 0, 6480000, 6480000, 1 }, { 1620000, 0, 810000, 6480000, 8 }, { 1620000, 1, 3240000, 6480000, 2 },
        { 2048000, 0, 8192000, 8192000, 1 }, { 2048000, 0, 2048000, 8192000, 4 }, { 2048000, 0, 32000, 8192000, 256 },
        { 2048000, 1, 4096000, 8192000, 2 }, { 2048000, 1, 1024000, 8192000, 8 } };
    unsigned i;
    int bad = 0;

    if (mirisdr_open_null(&n, "252_S16") < 0) { say("no null device"); return T_FAIL; }

    for (i = 0; i < sizeof ok / sizeof ok[0]; i++) {
        mirisdr_get_tune(n, 0, &c, NULL);
        c.frequency = 100000000; c.if_freq = ok[i].ifhz; c.bandwidth = 0;
        c.iq = ok[i].real ? MIRISDR_IQ_ONLY_I : MIRISDR_IQ_BOTH;
        if (mirisdr_tune(n, 0, &c, NULL) < 0) { say("%u Hz IF refused", ok[i].ifhz); bad++; continue; }
        mirisdr_get_stream(n, &s, NULL);
        s.baseband = 1; s.rate = ok[i].rate; s.transfer = "BULK"; s.format = NULL; s.format_single = NULL;
        if (mirisdr_set_stream(n, &s, &sr) < 0 || sr.rate != ok[i].rate || sr.adc_rate != ok[i].adc ||
            sr.decimation != ok[i].decim || sr.baseband != (ok[i].real ? MIRISDR_BASEBAND_REAL : MIRISDR_BASEBAND_COMPLEX))
        { say("IF %u %s %u: rate %u adc %u decimation %u", ok[i].ifhz, ok[i].real ? "one" : "both", ok[i].rate,
              sr.rate, sr.adc_rate, sr.decimation); bad++; continue; }
        mirisdr_get_tune(n, 0, NULL, &tr);
        if (tr.lo != 100000000 + ok[i].ifhz || tr.offset || tr.inverted)
        { say("IF %u: LO %u, offset %d", ok[i].ifhz, tr.lo, tr.offset); bad++; }
    }

    /* refused: not a power of 2 down, too far down, one output at the full rate */
    s.rate = 1000000;
    if (mirisdr_set_stream(n, &s, NULL) == 0) { say("1 MHz from a 2048 kHz IF accepted"); bad++; }
    s.rate = 16000;
    if (mirisdr_set_stream(n, &s, NULL) == 0) { say("1/512 accepted"); bad++; }
    s.rate = 8192000;
    if (mirisdr_set_stream(n, &s, NULL) == 0) { say("one output at 8192 ksps accepted"); bad++; }

    /* a tune to another IF takes the stream's rate to the nearest it has */
    mirisdr_get_tune(n, 0, &c, NULL);
    c.iq = MIRISDR_IQ_BOTH; c.if_freq = 2048000; c.bandwidth = 0;
    mirisdr_tune(n, 0, &c, NULL);
    s.rate = 1024000;
    mirisdr_set_stream(n, &s, NULL);
    c.if_freq = 450000; c.bandwidth = 0;
    if (mirisdr_tune(n, 0, &c, NULL) < 0) { say("a tune to 450 kHz refused"); bad++; }
    mirisdr_get_stream(n, &s, &sr);
    if (sr.rate != 900000 || sr.adc_rate != 1800000) { say("1024 ksps at 2048 kHz became %u at 450 kHz", sr.rate); bad++; }

    /* zero IF: float as it comes, any rate; the LO on the frequency */
    c.if_freq = 0; c.bandwidth = 0;
    mirisdr_tune(n, 0, &c, NULL);
    s.rate = 3000000;
    if (mirisdr_set_stream(n, &s, &sr) < 0 || sr.baseband != MIRISDR_BASEBAND_ZERO_IF || sr.rate != 3000000 ||
        sr.decimation != 1) { say("zero IF baseband: %d at %u", sr.baseband, sr.rate); bad++; }
    mirisdr_get_tune(n, 0, NULL, &tr);
    if (tr.lo != 100000000) { say("zero IF LO %u", tr.lo); bad++; }

    /* baseband off with a low IF puts the LO back on the frequency */
    c.if_freq = 450000; c.bandwidth = 0;
    mirisdr_tune(n, 0, &c, NULL);
    s.baseband = 0; s.rate = 2000000;
    if (mirisdr_set_stream(n, &s, &sr) < 0 || sr.baseband) { say("baseband would not go off"); bad++; }
    mirisdr_get_tune(n, 0, NULL, &tr);
    if (tr.lo != 100000000) { say("without baseband the LO is at %u", tr.lo); bad++; }

    mirisdr_close(n);
    if (!bad) say("rates for each IF and output, refusals, a tune moving the rate, zero IF, the LO");
    return bad ? T_FAIL : T_PASS;
}

/* each IF and output: the tone at +delta at its level, the mirror and the image down */
static tres_t t_bb_tone (void)
{
    static const struct { uint32_t ifhz; int real; uint32_t rate; double delta, img; } cases[] = {
        { 450000, 0, 1800000, 50000, 0 },   { 450000, 0, 900000, 50000, 120000 }, { 450000, 0, 225000, 20000, 30000 },
        { 450000, 1, 900000, 50000, 0 },    { 450000, 1, 112500, -20000, 0 },
        { 1620000, 0, 1620000, -300000, 400000 }, { 1620000, 1, 3240000, 600000, 0 },
        { 2048000, 0, 2048000, 300000, 500000 },  { 2048000, 1, 2048000, -700000, 0 },
        { 2048000, 1, 256000, 60000, 0 }, { 450000, 0, 900000, 70000, 0 }, { 2048000, 0, 2048000, -400000, 0 } };
    mirisdr_stream_result_t sr;
    unsigned i;
    int bad = 0;
    double worst_mirror = 1e9, worst_image = 1e9;

    for (i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        uint32_t len;
        uint8_t *data;
        size_t m, s0;
        double a, mir, img = 0, fs = 4.0 * cases[i].ifhz;

        /* the last two with I and Q swapped */
        bb_swap = i >= sizeof cases / sizeof cases[0] - 2;
        if (bb_open(cases[i].real, cases[i].ifhz, cases[i].rate, &sr) < 0) { say("case %u: no stream", i); return T_FAIL; }
        data = bb_gen(cases[i].real, fs, cases[i].ifhz, cases[i].delta, 0.5, cases[i].img, cases[i].img ? 0.5 : 0,
                      (int) (0.2 * fs / (cases[i].real ? 504 : 252)) + 64, -1, 0, &len);
        bb_feed(data, len, 0);
        free(data);
        mirisdr_close(bbo.d);

        /* swapped, the spectrum comes out mirrored, as without baseband */
        m = bbo.n / 2 / 2;
        s0 = bbo.n / 2 - m;
        a = bb_tone(bbo.x + 2 * s0, m, bb_swap ? -cases[i].delta : cases[i].delta, sr.rate);
        mir = 20 * log10(a / (bb_tone(bbo.x + 2 * s0, m, bb_swap ? cases[i].delta : -cases[i].delta, sr.rate) + 1e-12));
        if (cases[i].img && sr.decimation > 1)
            img = 20 * log10(a / (bb_tone(bbo.x + 2 * s0, m, cases[i].img, sr.rate) + 1e-12));

        if (bb_swap) note("swapped, so the tone is looked for mirrored:");
        if (img) note("IF %u, %s output, %u sps: tone at %+.0f Hz %.2f dBFS, mirror %.1f dB down, image %.1f dB down",
                      cases[i].ifhz, cases[i].real ? "one" : "both", sr.rate, cases[i].delta, 20 * log10(a), mir, img);
        else note("IF %u, %s output, %u sps: tone at %+.0f Hz %.2f dBFS, mirror %.1f dB down", cases[i].ifhz,
                  cases[i].real ? "one" : "both", sr.rate, cases[i].delta, 20 * log10(a), mir);

        if (fabs(20 * log10(a) + 6.02) > 0.3 || mir < (cases[i].real ? 60 : 90) || (img && img < 58) || bbo.bad_info)
        { say("IF %u %s %u: %.2f dBFS, mirror %.1f dB, image %.1f dB, %d bad infos", cases[i].ifhz,
              cases[i].real ? "one" : "both", sr.rate, 20 * log10(a), mir, img, bbo.bad_info); bad++; }
        if (mir < worst_mirror) worst_mirror = mir;
        if (img && img < worst_image) worst_image = img;
    }

    bb_swap = 0;
    if (!bad) say("12 cases, 2 with I/Q swapped: each tone at its offset within 0.3 dB, mirrors >= %.0f dB down, images >= %.0f dB",
                  worst_mirror, worst_image);
    return bad ? T_FAIL : T_PASS;
}

/* sample counts, fixed-length buffers, a gap: where it lands and what fills it */
static tres_t t_bb_timing (void)
{
    mirisdr_stream_result_t sr;
    uint32_t len;
    uint8_t *data;
    int bad = 0, real;

    for (real = 0; real < 2; real++) {
        int spb = real ? 504 : 252;
        uint64_t in_samples;

        /* 2000 blocks, 30 missing after block 1000, buffers of 4096 bytes */
        if (bb_open(real, 450000, 225000, &sr) < 0) { say("no stream"); return T_FAIL; }
        bbo.len = 4096;
        data = bb_gen(real, 1800000, 450000, 10000, 0.5, 0, 0, 2000, 1000, 30, &len);
        bb_feed(data, len, 4096);
        free(data);
        mirisdr_close(bbo.d);

        /* 16 of the 30 blocks are filled, the rest a jump in the index. The fill can
           reach the callback over several buffers, so one gap may come as several */
        in_samples = (uint64_t) (2000 + 16) * spb;
        if (bbo.bad_info) { say("%s: %d buffers with a wrong info", real ? "one" : "both", bbo.bad_info); bad++; }
        if (bbo.n / 2 > in_samples / sr.decimation + 2 || bbo.n / 2 + 512 + 64 < in_samples / sr.decimation)
        { say("%s: %zu pairs out of %llu in, decimation %u", real ? "one" : "both", bbo.n / 2,
              (unsigned long long) in_samples, sr.decimation); bad++; }
        if (!bbo.gaps || bbo.gap_samples != (uint64_t) (30 * spb + sr.decimation - 1) / sr.decimation ||
            bbo.gap_filled != (uint64_t) 16 * spb / sr.decimation)
        { say("%s: %llu gaps, %llu missing %llu filled", real ? "one" : "both", (unsigned long long) bbo.gaps,
              (unsigned long long) bbo.gap_samples, (unsigned long long) bbo.gap_filled); bad++; }
        if (bbo.last_index_jump < (uint64_t) 14 * spb / sr.decimation - 1 || bbo.last_index_jump > (uint64_t) 14 * spb / sr.decimation + 1)
        { say("%s: the index ran %llu ahead, not %u", real ? "one" : "both", (unsigned long long) bbo.last_index_jump,
              14 * spb / sr.decimation); bad++; }
        note("%s output: %zu pairs in %d buffers of 512, gap of %llu in %llu parts with %llu filled, index +%llu after it",
             real ? "one" : "both", bbo.n / 2, bbo.calls, (unsigned long long) bbo.gap_samples,
             (unsigned long long) bbo.gaps,
             (unsigned long long) bbo.gap_filled, (unsigned long long) bbo.last_index_jump);
    }

    if (!bad) say("4096 byte buffers counted in output samples; a 30 block gap: 16 filled, the rest in the index");
    return bad ? T_FAIL : T_PASS;
}

/* how fast each path runs here, against what it has to keep up with */
static tres_t t_bb_speed (void)
{
    static const struct { uint32_t ifhz; int real; uint32_t rate; } cases[] = {
        { 2048000, 0, 8192000 }, { 2048000, 0, 2048000 }, { 2048000, 0, 256000 },
        { 2048000, 1, 4096000 }, { 2048000, 1, 2048000 }, { 450000, 1, 112500 } };
    mirisdr_stream_result_t sr;
    unsigned i;
    int slow = 0;
    char line[256] = "";

    for (i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        uint32_t len, blocks = 16000;
        uint8_t *data;
        double t0, dt, msps;

        if (bb_open(cases[i].real, cases[i].ifhz, cases[i].rate, &sr) < 0) { say("no stream"); return T_FAIL; }
        data = bb_gen(cases[i].real, 4.0 * cases[i].ifhz, cases[i].ifhz, 1000, 0.5, 0, 0, (int) blocks, -1, 0, &len);
        bb_feed_cb(data, len / 8, 0, dec_cb_count);     /* warm up: caches, clock */
        t0 = now();
        bb_feed_cb(data + len / 8, len - len / 8, 0, dec_cb_count);
        dt = now() - t0;
        blocks -= blocks / 8;
        free(data);
        mirisdr_close(bbo.d);

        /* converter samples a second, the 14-bit unpacking included */
        msps = (double) blocks * (cases[i].real ? 504 : 252) / dt / 1e6;
        note("IF %u, %s output, %u sps out: %.1f Msps of %.3f needed (%.0fx)", cases[i].ifhz,
             cases[i].real ? "one" : "both", cases[i].rate, msps, sr.adc_rate / 1e6, msps * 1e6 / sr.adc_rate);
        snprintf(line + strlen(line), sizeof line - strlen(line), "%s%.0fx", i ? ", " : "", msps * 1e6 / sr.adc_rate);
        if (msps * 1e6 < 1.5 * sr.adc_rate) slow++;
    }

    /* the same blocks without baseband: the unpacking every stream has */
    {
        mirisdr_dev_t *d;
        uint32_t len, blocks = 16000;
        uint8_t *data = bb_gen(0, 8192000, 2048000, 1000, 0.5, 0, 0, (int) blocks, -1, 0, &len);
        double t0, dt;

        if (mirisdr_open_null(&d, "252_S16") == 0) {
            bbo.d = d;
            bbo.rate = 0;
            t0 = now();
            for (uint32_t at = 0; at < len; at += 16384)
                mirisdr_feed_bulk(d, dec_cb_count, NULL, 0, data + at, len - at < 16384 ? len - at : 16384);
            dt = now() - t0;
            mirisdr_close(d);
            note("unpacking 252_S16 alone, no baseband: %.1f Msps", (double) blocks * 252 / dt / 1e6);
        }
        free(data);
    }

    say("real time x %s (both 8192k, 2048k, 256k; one 4096k, 2048k; 112k)", line);
    return slow ? T_FAIL : T_PASS;
}

/* ------------------------------------------------------------------ */
/* regress: what the review of 2026-10 found, each once, no device    */
/* ------------------------------------------------------------------ */

static int reg_bad;

#define REG_CHECK(cond, ...) do { if (!(cond)) { reg_bad++; say(__VA_ARGS__); } } while (0)

static tres_t t_reg_open (void)
{
    mirisdr_open_config_t cfg;
    mirisdr_dev_t *d;
    uint8_t *big;
    uint32_t size;

    reg_bad = 0;

    /* a board not in the list is refused, before it indexes the band plans */
    mirisdr_open_config_default(&cfg);
    cfg.hw_flavour = 7;
    d = (mirisdr_dev_t *) 1;
    REG_CHECK(mirisdr_open_ex(&d, &cfg) < 0, "hw_flavour 7 opened");
    REG_CHECK(d == NULL, "*out not cleared on a refused open");
    cfg.hw_flavour = -2;
    REG_CHECK(mirisdr_open_ex(&d, &cfg) < 0, "hw_flavour -2 opened");

    /* an image past the 0x1800 the firmware's RAM leaves: the built-in one padded out */
    mirisdr_open_config_default(&cfg);
    if ((big = calloc(1, 0x1900))) {
        const uint8_t *fw = mirisdr_default_firmware(&size);

        memcpy(big, fw, size);
        cfg.firmware = big;
        cfg.firmware_size = 0x1900;
        REG_CHECK(mirisdr_open_ex(&d, &cfg) < 0, "a 0x1900 byte image accepted");
        free(big);
    }

    REG_CHECK(mirisdr_get_index_by_serial("no such serial") == -1, "a serial not found is not -1");

    if (!reg_bad) say("a board out of range and an image too big refused, *out cleared, -1 for no serial");
    return reg_bad ? T_FAIL : T_PASS;
}

/* every call with no device returns its failure rather than crashing */
static tres_t t_reg_null_dev (void)
{
    mirisdr_stream_config_t sc;
    mirisdr_tune_config_t tc;
    uint8_t b[16];

    reg_bad = 0;
    mirisdr_stream_config_default(&sc);
    mirisdr_tune_config_default(&tc);

    REG_CHECK(mirisdr_get_sample_rate(NULL) == 0, "get_sample_rate");
    REG_CHECK(!strcmp(mirisdr_get_decimation_bypass(NULL), ""), "get_decimation_bypass");
    REG_CHECK(mirisdr_get_swap_iq(NULL) < 0, "get_swap_iq");
    REG_CHECK(!strcmp(mirisdr_get_sample_format(NULL), ""), "get_sample_format");
    REG_CHECK(!strcmp(mirisdr_get_sample_format_selected(NULL), ""), "get_sample_format_selected");
    REG_CHECK(mirisdr_get_center_freq(NULL) == 0, "get_center_freq");
    REG_CHECK(!strcmp(mirisdr_get_transfer(NULL), ""), "get_transfer");
    mirisdr_get_band(NULL);
    REG_CHECK(mirisdr_get_bias(NULL) < 0, "get_bias");
    REG_CHECK(mirisdr_get_tuner_gains(NULL, NULL) < 0, "get_tuner_gains");
    REG_CHECK(mirisdr_get_tuner_gain(NULL) < 0, "get_tuner_gain");
    REG_CHECK(mirisdr_set_tuner_gain(NULL, 10) < 0, "set_tuner_gain");
    REG_CHECK(mirisdr_set_tuner_gain_mode(NULL, 1) < 0, "set_tuner_gain_mode");
    REG_CHECK(mirisdr_get_tuner_gain_mode(NULL) < 0, "get_tuner_gain_mode");
    REG_CHECK(mirisdr_get_mixer_gain(NULL) < 0, "get_mixer_gain");
    REG_CHECK(mirisdr_get_mixbuffer_gain(NULL) < 0, "get_mixbuffer_gain");
    REG_CHECK(mirisdr_get_lna_gain(NULL) < 0, "get_lna_gain");
    REG_CHECK(mirisdr_get_baseband_gain(NULL) < 0, "get_baseband_gain");
    REG_CHECK(mirisdr_set_mixbuffer_gain(NULL, 6) < 0, "set_mixbuffer_gain");
    REG_CHECK(mirisdr_set_lna_gain(NULL, 1) < 0, "set_lna_gain");
    REG_CHECK(mirisdr_set_gain(NULL) < 0, "set_gain");
    REG_CHECK(mirisdr_start_async(NULL) < 0, "start_async");
    REG_CHECK(mirisdr_stop_async(NULL) < 0, "stop_async");
    REG_CHECK(mirisdr_cancel_async(NULL) < 0, "cancel_async");
    REG_CHECK(mirisdr_get_fw_id(NULL, b, sizeof b) < 0, "get_fw_id");
    REG_CHECK(mirisdr_running_from_rom(NULL) < 0, "running_from_rom");
    REG_CHECK(mirisdr_tune(NULL, 0, &tc, NULL) < 0, "tune");
    REG_CHECK(mirisdr_set_stream(NULL, &sc, NULL) < 0, "set_stream");
    REG_CHECK(mirisdr_read_sync(NULL, b, sizeof b, NULL) < 0, "read_sync");

    if (!reg_bad) say("30 getters and setters return their failure");
    return reg_bad ? T_FAIL : T_PASS;
}

static tres_t t_reg_tune (void)
{
    mirisdr_dev_t *n;
    mirisdr_tune_config_t c, got;
    mirisdr_tune_result_t r;
    mirisdr_stream_config_t sc;
    int g;

    reg_bad = 0;
    if (mirisdr_open_null(&n, "252_S16") < 0) { say("no null device"); return T_FAIL; }

    /* a refused gain leaves the one in force */
    mirisdr_set_tuner_gain(n, 60);
    g = mirisdr_get_tuner_gain(n);
    REG_CHECK(mirisdr_set_tuner_gain(n, -1) < 0, "a negative gain accepted");
    REG_CHECK(mirisdr_get_tuner_gain(n) == g, "a refused gain changed it: %d, was %d", mirisdr_get_tuner_gain(n), g);

    /* no mixbuffer outside the AM bands, so the stages add up */
    mirisdr_tune_config_default(&c);
    c.frequency = 100000000;
    c.gain.mode = MIRISDR_GAIN_TOTAL;
    c.gain.total = 90;
    REG_CHECK(mirisdr_tune(n, 0, &c, &r) == 0, "100 MHz refused");
    REG_CHECK(r.gain.mixbuffer == 0, "a %d dB mixbuffer at VHF", r.gain.mixbuffer);
    REG_CHECK(r.gain.total == (r.gain.lna ? 24 : 0) + (r.gain.mixer ? 19 : 0) + r.gain.baseband,
              "VHF stages do not add up to %d", r.gain.total);

    /* the RSP1's 261-404 MHz row is band III, not L */
    mirisdr_set_hw_flavour(n, MIRISDR_HW_SDRPLAY);
    c.frequency = 300000000;
    c.gain.total = 110;
    REG_CHECK(mirisdr_tune(n, 0, &c, &r) == 0, "300 MHz on the RSP1 refused");
    REG_CHECK(r.band == MIRISDR_BAND_3 && r.gain.total == 102, "300 MHz on the RSP1: band %d, %d dB", r.band, r.gain.total);
    mirisdr_set_hw_flavour(n, MIRISDR_HW_DEFAULT);

    /* the single setters go past the plan, but not where the synthesizer's N wraps */
    REG_CHECK(mirisdr_set_center_freq(n, 2900000000u) == 0, "2.9 GHz refused");
    REG_CHECK(mirisdr_set_center_freq(n, 3100000000u) < 0, "3.1 GHz accepted, which wraps the synthesizer");
    REG_CHECK(mirisdr_get_center_freq(n) == 2900000000u, "a refused LO changed it");
    mirisdr_set_center_freq(n, 100000000);

    /* a notch does not outlive the RSP1B */
    mirisdr_set_hw_flavour(n, MIRISDR_HW_RSP1B);
    REG_CHECK(mirisdr_set_notch(n, MIRISDR_NOTCH_FM) == 0, "notch on the RSP1B refused");
    mirisdr_set_hw_flavour(n, MIRISDR_HW_DEFAULT);
    REG_CHECK(mirisdr_get_notch(n) == 0, "notch %d left after the flavour change", mirisdr_get_notch(n));
    REG_CHECK(mirisdr_set_center_freq(n, 101000000) == 0, "tuning refused after the flavour change");

    /* a single setter leaves a tune a check takes */
    REG_CHECK(mirisdr_set_bandwidth(n, 8000000) == 0, "8 MHz filter refused");
    REG_CHECK(mirisdr_set_offset_tuning(n, 1) == 0, "offset tuning refused");
    mirisdr_get_tune(n, 0, &got, NULL);
    REG_CHECK(mirisdr_tune_check(n, 0, &got, NULL) == 0, "the setters left IF %u with a %u Hz filter", got.if_freq, got.bandwidth);
    mirisdr_set_offset_tuning(n, 0);

    /* the null device has a stream to start from */
    mirisdr_get_stream(n, &sc, NULL);
    REG_CHECK(mirisdr_set_stream(n, &sc, NULL) == 0, "get_stream then set_stream refused on the null device");

    /* a check predicts a refusal by a stream following the tune */
    mirisdr_tune_config_default(&c);
    c.frequency = 100000000;
    c.if_freq = 2048000;
    c.low_if_auto = 1;
    c.iq = MIRISDR_IQ_ONLY_I;
    REG_CHECK(mirisdr_tune(n, 0, &c, NULL) == 0, "single output tune refused");
    mirisdr_stream_config_default(&sc);
    sc.follow_tune = 1;
    sc.transfer = "ISOC1";
    sc.rate = 5000000;
    REG_CHECK(mirisdr_set_stream(n, &sc, NULL) == 0, "following stream refused");
    c.iq = MIRISDR_IQ_BOTH;
    REG_CHECK((mirisdr_tune_check(n, 0, &c, NULL) == 0) == (mirisdr_tune(n, 0, &c, NULL) == 0),
              "check and tune disagree with a stream following it");

    mirisdr_close(n);

    if (!reg_bad) say("refused gain kept, MIXBUF 0 at VHF, the RSP1 at 300 MHz, LO past 3.07 GHz refused, "
                      "notch cleared, setters and checks agree");
    return reg_bad ? T_FAIL : T_PASS;
}

static tres_t t_reg_stream (void)
{
    mirisdr_dev_t *n;
    mirisdr_tune_config_t c;
    mirisdr_stream_config_t sc;
    mirisdr_stream_result_t sr;
    uint8_t b[16];

    reg_bad = 0;
    if (mirisdr_open_null(&n, "252_S16") < 0) { say("no null device"); return T_FAIL; }

    /* a baseband stream at a low IF has no other rate than 4 x IF: with the decimation
       bypass on that is below its range, so refused rather than clamped */
    mirisdr_stream_config_default(&sc);
    sc.rate = 4000000;
    sc.baseband = 1;
    sc.decimation_bypass = "ON";
    REG_CHECK(mirisdr_set_stream(n, &sc, NULL) == 0, "baseband at zero IF with the bypass on refused");
    mirisdr_tune_config_default(&c);
    c.frequency = 100000000;
    c.if_freq = 450000;
    if (mirisdr_tune(n, 0, &c, NULL) == 0) {
        mirisdr_get_stream(n, &sc, &sr);
        REG_CHECK(sr.adc_rate == 1800000 && sr.rate * sr.decimation == sr.adc_rate,
                  "450 kHz IF with the bypass on: converters at %u, out %u, decimation %u", sr.adc_rate, sr.rate, sr.decimation);
    }

    REG_CHECK(mirisdr_cancel_async(n) == -1, "cancel without a stream: %d", mirisdr_cancel_async(n));
    REG_CHECK(mirisdr_read_sync(n, b, sizeof b, NULL) < 0, "a sync read on the null device");
    mirisdr_close(n);

    /* under baseband the legacy rate calls take and give the output's rate */
    if (bb_open(0, 450000, 900000, &sr) == 0) {
        REG_CHECK(mirisdr_get_sample_rate(bbo.d) == 900000, "get_sample_rate %u under baseband at 900k", mirisdr_get_sample_rate(bbo.d));
        REG_CHECK(mirisdr_set_sample_rate(bbo.d, 450000) == 0 && mirisdr_get_sample_rate(bbo.d) == 450000,
                  "set_sample_rate 450k under baseband gave %u", mirisdr_get_sample_rate(bbo.d));
        mirisdr_close(bbo.d);
    } else { reg_bad++; say("no baseband stream"); }

    if (!reg_bad) say("baseband with the bypass on keeps its rates, cancel without a stream -1, legacy rates under baseband");
    return reg_bad ? T_FAIL : T_PASS;
}

static struct { uint64_t gap_samples, gap_filled, first_index; int restarted, short_fill, calls; } rg;

static void reg_cb (unsigned char *buf, uint32_t len, void *ctx)
{
    mirisdr_buffer_info_t in;
    uint32_t k;

    (void) buf; (void) len; (void) ctx;
    if (mirisdr_get_buffer_info(bbo.d, &in) < 0) return;
    rg.calls++;
    rg.gap_samples += in.gap_samples;
    for (k = 0; k < in.gaps_len; k++) {
        rg.gap_filled += in.gaps[k].filled;
        /* within gap fill's reach: all of it filled */
        if (in.gaps[k].filled != in.gaps[k].samples && in.gaps[k].samples < 64) rg.short_fill++;
    }
    if (rg.restarted == 1) { rg.first_index = in.index; rg.restarted = 2; }
}

/* blocks of bb_gen with every other one of 'n' from 'at' cut out: n gaps of a block */
static uint8_t *reg_holes (uint32_t *len, int blocks, int at, int n)
{
    uint8_t *d = bb_gen(0, 1800000, 450000, 10000, 0.5, 0, 0, blocks, -1, 0, len), *o = d;
    int k;

    for (k = 0; k < blocks; k++)
        if (!(k >= at && k < at + 2 * n && ((k - at) & 1))) { memmove(o, d + 1024 * k, 1024); o += 1024; }
    *len = (uint32_t) (o - d);
    return d;
}

static tres_t t_reg_baseband (void)
{
    mirisdr_stream_result_t sr;
    mirisdr_stream_config_t sc;
    uint8_t *data;
    uint32_t len, half;
    uint64_t want;
    int k;

    reg_bad = 0;

    /* one block missing, filled: decimated, still all filled (31.5 samples is not a loss) */
    if (bb_open(0, 450000, 225000, &sr) < 0) { say("no stream"); return T_FAIL; }
    memset(&rg, 0, sizeof rg);
    data = reg_holes(&len, 400, 100, 1);
    bb_feed_cb(data, len, 0, reg_cb);
    free(data);
    REG_CHECK(rg.gap_samples && !rg.short_fill, "a filled gap reads as partly lost (%llu)", (unsigned long long) rg.gap_samples);
    mirisdr_close(bbo.d);

    /* gap fill stays on under baseband whatever is asked */
    if (bb_open(0, 450000, 225000, &sr) < 0) { say("no stream"); return T_FAIL; }
    mirisdr_set_gap_fill(bbo.d, 0);
    memset(&rg, 0, sizeof rg);
    data = reg_holes(&len, 400, 100, 1);
    bb_feed_cb(data, len, 0, reg_cb);
    free(data);
    REG_CHECK(rg.gap_filled, "set_gap_fill(0) turned the fill off under baseband");
    mirisdr_close(bbo.d);

    /* 100 gaps waiting for one big buffer: all counted */
    if (bb_open(0, 450000, 225000, &sr) < 0) { say("no stream"); return T_FAIL; }
    memset(&rg, 0, sizeof rg);
    data = reg_holes(&len, 600, 50, 100);
    bb_feed_cb(data, len, 15000 * 8, reg_cb);   /* about 600 blocks out, the gaps in the first */
    free(data);
    want = 100 * ((252 + 7) / 8);
    REG_CHECK(rg.gap_samples == want, "100 gaps in one buffer: %llu missing, not %llu",
              (unsigned long long) rg.gap_samples, (unsigned long long) want);
    mirisdr_close(bbo.d);

    /* a restart drops the buffer being filled: the index carries on past it */
    if (bb_open(0, 450000, 225000, &sr) < 0) { say("no stream"); return T_FAIL; }
    memset(&rg, 0, sizeof rg);
    data = bb_gen(0, 1800000, 450000, 10000, 0.5, 0, 0, 800, -1, 0, &len);
    /* 444 blocks are 13986 out: 986 of a 1000 sample buffer pending. The parser holds a
       transfer's last block back, so one more goes in before the restart. From that one
       the counter is a restarted stream's, which the null device expects from 2 past a
       block's step */
    half = 445 * 1024;
    for (k = 444; k < 800; k++) {
        uint32_t idx = 254 + (uint32_t) (k - 444) * 252;
        uint8_t *h = data + 1024 * k;

        h[0] = (uint8_t) idx; h[1] = (uint8_t) (idx >> 8); h[2] = (uint8_t) (idx >> 16); h[3] = (uint8_t) (idx >> 24);
    }
    bb_feed_cb(data, half, 8000, reg_cb);
    mirisdr_get_stream(bbo.d, &sc, NULL);
    sc.swap_iq = !sc.swap_iq;           /* a restream, the rate as it was */
    if (mirisdr_set_stream(bbo.d, &sc, NULL) < 0) { reg_bad++; say("restream refused"); }
    rg.restarted = 1;
    bb_feed_cb(data + half, len - half, 8000, reg_cb);
    free(data);
    want = (uint64_t) 444 * 252 / 8;
    REG_CHECK(rg.restarted == 2 && rg.first_index + 2 >= want && rg.first_index <= want + 2,
              "first index after a restart %llu, the stream was at %llu", (unsigned long long) rg.first_index,
              (unsigned long long) want);
    mirisdr_close(bbo.d);

    if (!reg_bad) say("filled gaps stay filled, fill on under baseband, 100 gaps in a buffer, the index over a restart");
    return reg_bad ? T_FAIL : T_PASS;
}

static tres_t t_reg_fw_patch (void)
{
    mirisdr_fw_patch_t pt;
    const uint8_t *fw;
    uint8_t *img, *orig;
    uint32_t size;

    reg_bad = 0;
    fw = mirisdr_default_firmware(&size);
    img = malloc(size);
    orig = malloc(size);
    if (!img || !orig) { free(img); free(orig); say("out of memory"); return T_FAIL; }
    memcpy(img, fw, size);
    memcpy(orig, fw, size);

    /* new ids with a serial too long: refused, and the ids not written either */
    memset(&pt, 0, sizeof pt);
    pt.fields = MIRISDR_FW_PATCH_IDS | MIRISDR_FW_PATCH_SERIAL;
    pt.vid = 0x1234;
    pt.pid = 0x5678;
    memset(pt.serial, 'A', sizeof pt.serial);
    REG_CHECK(mirisdr_fw_patch(img, size, &pt) < 0, "a 13 character serial accepted");
    REG_CHECK(!memcmp(img, orig, size), "a refused patch changed the image");

    /* a field this library does not know */
    memset(&pt, 0, sizeof pt);
    pt.fields = 1u << 5;
    REG_CHECK(mirisdr_fw_patch(img, size, &pt) < 0, "an unknown field accepted");

    free(img);
    free(orig);
    if (!reg_bad) say("a patch is refused as a whole, unknown fields too");
    return reg_bad ? T_FAIL : T_PASS;
}

#ifdef MIRI_TEST_SOAPY
/* ------------------------------------------------------------------ */
/* the SoapySDR module, loaded and used as an application would        */
/* ------------------------------------------------------------------ */

#include "miri_test_soapy.h"

/* MIRI_SOAPY_MODULE in the environment, else the build's, else the installed one */
static const char *soapy_module (void)
{
    const char *env = getenv("MIRI_SOAPY_MODULE");

    if (env) return env;
    if (access(MIRI_SOAPY_MODULE_BUILD, R_OK) == 0) return MIRI_SOAPY_MODULE_BUILD;
    if (access(MIRI_SOAPY_MODULE_INSTALL, R_OK) == 0) return MIRI_SOAPY_MODULE_INSTALL;
    return NULL;
}

/* the module opens the receiver itself, so ours is closed around it */
static tres_t soapy_run (const char *which)
{
    const char *module = soapy_module();
    int r;

    if (opt_rom || opt_fw) { say("the module loads the built-in firmware"); return T_SKIP; }
    if (!module) { say("module not found, set MIRI_SOAPY_MODULE"); return T_SKIP; }

    pump_stop();
    mirisdr_close(dev);
    dev = NULL;

    r = soapy_test(which, module, open_serial, opt_verbose, say, note);

    if (device_reopen() < 0) { say("the device did not come back"); return T_FAIL; }

    return r == SOAPY_T_PASS ? T_PASS : r == SOAPY_T_SKIP ? T_SKIP : T_FAIL;
}

static tres_t t_soapy_probe (void)  { return soapy_run("probe"); }
static tres_t t_soapy_stream (void) { return soapy_run("stream"); }
static tres_t t_soapy_cycles (void) { return soapy_run("cycles"); }
static tres_t t_soapy_retune (void) { return soapy_run("retune"); }
static tres_t t_soapy_band (void)   { return soapy_run("bandranges"); }
static tres_t t_soapy_regress (void) { return soapy_run("regress"); }
#endif

static const struct {
    const char *group;
    const char *name;
    tres_t    (*fn)(void);
} tests[] = {
    { "decode",   "bulk blocks, clean",         t_dec_clean           },
    { "decode",   "bulk gaps and gap fill",     t_dec_gaps            },
    { "decode",   "bulk slips",                 t_dec_slips           },

    { "plan",     "tune config rules",          t_plan_tune           },
    { "plan",     "stream config rules",        t_plan_stream         },
    { "plan",     "stream following the tune",  t_plan_follow         },
    { "plan",     "frontend with the tune",     t_plan_frontend       },
    { "plan",     "reserved room to grow",      t_plan_reserved       },
    { "plan",     "automatic filter",           t_plan_bw_auto        },
    { "plan",     "scan lists",                 t_plan_scan           },

    { "baseband", "rates, IF moves and the LO",  t_bb_plan             },
    { "baseband", "tone, mirror and image",     t_bb_tone             },
    { "baseband", "counts, buffers and gaps",   t_bb_timing           },
    { "baseband", "speed against real time",    t_bb_speed            },

    { "regress",  "open config refusals",       t_reg_open            },
    { "regress",  "calls without a device",     t_reg_null_dev        },
    { "regress",  "tune, gain and setters",     t_reg_tune            },
    { "regress",  "stream plan and rates",      t_reg_stream          },
    { "regress",  "baseband gaps and restarts", t_reg_baseband        },
    { "regress",  "firmware patch as a whole",  t_reg_fw_patch        },

    { "identity", "device enumerates",          t_enumerate           },
    { "identity", "usb descriptors",            t_usb_strings         },
    { "identity", "open by serial",             t_open_by_serial      },
    { "identity", "usb position",               t_usb_position        },
    { "identity", "firmware identity",          t_identity            },
    { "identity", "built-in firmware default",  t_firmware_precedence },
    { "identity", "firmware image patching",    t_fw_patch_roundtrip  },

    { "stream",   "bulk streams",               t_stream_bulk         },
    { "stream",   "isochronous 3x1024",         t_stream_isoc         },
    { "stream",   "isochronous 1x1024",         t_stream_isoc1        },
    { "stream",   "isochronous 2x1024",         t_stream_isoc2        },
    { "stream",   "each transport's ceiling",   t_transport_capacity  },
    { "stream",   "every packing format",       t_formats             },
    { "stream",   "synchronous read",           t_sync_read           },
    { "stream",   "rate range 1.3 to 12 Msps",  t_rate_range          },
    { "stream",   "rate clamping",              t_rate_clamping       },
    { "stream",   "automatic format choice",    t_format_auto         },
    { "stream",   "stream config",              t_stream_api          },
    { "stream",   "stream following the tune",  t_stream_follow       },
    { "stream",   "baseband on the receiver",   t_bb_device           },

    { "fixes",    "rate changes keep the stream", t_rate_changes      },
    { "fixes",    "counts over restarts",       t_restart_counts      },
    { "fixes",    "transfer changed live",      t_transfer_live       },
    { "fixes",    "sync reads and restarts",    t_sync_restart        },
    { "fixes",    "repeated stop and start",    t_stop_start          },
    { "fixes",    "stall and clear recovers",   t_stall_recovery      },
    { "fixes",    "unstall before cancel",      t_stall_clear_then_cancel },
    { "fixes",    "cancel while starting",      t_cancel_at_start     },
    { "fixes",    "cancel while restarting",    t_cancel_while_restarting },
    { "fixes",    "header stamp present",       t_header_stamp        },
    { "fixes",    "no false grid shifts",       t_no_false_resync     },
    { "fixes",    "usb reset recovers",         t_usb_reset           },
    { "fixes",    "pll stays off the band ends", t_pll_band           },
    { "fixes",    "pll recovers from no lock",  t_pll_unlockable     },
    { "fixes",    "rate recovers from the ceiling", t_rate_over_ceiling },

    { "chars",    "synthesiser limits of this part", t_pll_characterise },
    { "chars",    "engine byte rate ceiling",    t_engine_ceiling   },

    { "device",   "read registers",             t_read_regs           },
    { "device",   "write a register",           t_write_reg           },
    { "device",   "read memory",                t_read_mem            },
    { "device",   "write memory",               t_write_mem           },
    { "device",   "call gate runs code",        t_call_gate           },

    { "tuner",    "tuning across bands",        t_tuning              },
    { "tuner",    "tuner status readback",      t_tuner_status        },
    { "tuner",    "tuner overrides",            t_tuner_override      },
    { "tuner",    "unknown override",           t_tuner_unknown       },
    { "tuner",    "stream events",              t_stream_events       },
    { "tuner",    "gain steps",                 t_gain                },
    { "tuner",    "filter bandwidths",          t_bandwidth           },
    { "tuner",    "settings round trip",        t_settings_roundtrip  },
    { "tuner",    "tune config",                t_tune_api            },
    { "tuner",    "VCO limits (--characterise)", t_vco_limits         },
    { "tuner",    "gpio inputs",                t_gpio_read           },

    { "scan",     "learn and replay",           t_scan_learn          },
    { "scan",     "scan list run",              t_scan_run            },
    { "scan",     "scan list run dry",          t_scan_dry            },

    { "extras",   "eeprom size probe",          t_eeprom_probe        },
    { "extras",   "eeprom read",                t_eeprom_read         },
    { "extras",   "eeprom write back",          t_eeprom_write        },
    { "extras",   "uart transmit",              t_uart                },
    { "extras",   "i2c bus",                    t_i2c                 },
    { "extras",   "pps timestamping",           t_pps                 },
    { "extras",   "sof timestamping",           t_sof                 },

#ifdef MIRI_TEST_SOAPY
    { "soapy",    "module, ranges and gains",   t_soapy_probe         },
    { "soapy",    "each format streams",        t_soapy_stream        },
    { "soapy",    "start and stop",             t_soapy_cycles        },
    { "soapy",    "retune and rate changes",    t_soapy_retune        },
    { "soapy",    "gain ranges of the band",    t_soapy_band          },
    { "soapy",    "review fixes",               t_soapy_regress       },
#endif
};

#define NTESTS ((int)(sizeof tests / sizeof tests[0]))

static void usage (const char *me)
{
    printf("usage: %s [options]\n"
           "  -s <serial>      pick a device by serial\n"
           "  -f <image>       load this firmware instead of the built-in one\n"
           "  --rom            run against the factory ROM\n"
           "  -g <group>       run only one group of tests\n"
           "  -v               show per test detail\n"
           "  --io             allow tests that drive external pins (uart, i2c)\n"
           "  --characterise   sweep the synthesiser and report this part's limits\n"
           "  --step MHz       sweep resolution for --characterise (default 1.0)\n"
           "  --eeprom         allow the eeprom tests; on a board without an eeprom the\n"
           "                   probe reaches the tuner and leaves it deaf until power off\n"
           "  --eeprom-write   allow the eeprom write back test (implies --eeprom)\n"
           "  --pps            run the pps test, needs a 1PPS on GPIO_0\n"
           "  --list           list the tests and exit\n"
           "\ngroups: decode plan baseband regress identity stream fixes device tuner extras"
#ifdef MIRI_TEST_SOAPY
           " soapy"
#endif
           "\n", me);
}

int main (int argc, char **argv)
{
    mirisdr_open_config_t cfg;
    const char *serial = NULL;
    int i, r;

    setvbuf(stdout, NULL, _IONBF, 0);

    for (i = 1; i < argc; i++)
    {
        if (!strcmp(argv[i], "-s") && (i + 1 < argc)) serial = argv[++i];
        else if (!strcmp(argv[i], "-f") && (i + 1 < argc)) opt_fw = argv[++i];
        else if (!strcmp(argv[i], "--rom")) opt_rom = 1;
        else if (!strcmp(argv[i], "-g") && (i + 1 < argc)) opt_only = argv[++i];
        else if (!strcmp(argv[i], "-v")) opt_verbose = 1;
        else if (!strcmp(argv[i], "--io")) opt_io = 1;
        else if (!strcmp(argv[i], "--eeprom")) opt_eeprom = 1;
        else if (!strcmp(argv[i], "--eeprom-write")) opt_eeprom_write = opt_eeprom = 1;
        else if (!strcmp(argv[i], "--pps")) opt_pps = 1;
        else if (!strcmp(argv[i], "--characterise") || !strcmp(argv[i], "--characterize")) opt_chars = 1;
        else if (!strcmp(argv[i], "--step") && (i + 1 < argc)) opt_step = atof(argv[++i]);
        else if (!strcmp(argv[i], "--list")) {
            for (r = 0; r < NTESTS; r++) printf("  %-9s %s\n", tests[r].group, tests[r].name);
            return 0;
        }
        else { usage(argv[0]); return 1; }
    }

    open_serial = serial;

    if (opt_rom && opt_fw) {
        fprintf(stderr, "--rom and -f are exclusive\n");
        return 1;
    }

    /* For the ROM the part has to be put back to it first, since whatever is
       resident keeps running otherwise. */
    if (opt_rom) {
        mirisdr_open_config_default(&cfg);
        cfg.serial = serial;
        cfg.keep_running = 1;

        if (mirisdr_open_ex(&dev, &cfg) < 0) {
            fprintf(stderr, "cannot open the device\n");
            return 1;
        }

        if (mirisdr_running_from_rom(dev) != 1) mirisdr_reboot(dev, MIRISDR_BOOT_ROM);
    }

    /* the decode, plan, baseband and regress groups need no device */
    if (opt_only && (!strcmp(opt_only, "decode") || !strcmp(opt_only, "plan") || !strcmp(opt_only, "baseband") ||
                     !strcmp(opt_only, "regress"))) goto run;

    if ((opt_rom ? device_reopen() : device_open()) < 0) {
        fprintf(stderr, "cannot open the device\n");
        return 1;
    }

    {
        uint8_t id[MIRISDR_FW_ID_LEN];

        fw_ours = mirisdr_get_fw_id(dev, id, sizeof id) == MIRISDR_FW_ID_LEN;
    }

    from_rom = mirisdr_running_from_rom(dev);

    printf("\n  miri_test - %s firmware, %s\n\n",
           fw_ours ? "our" : "factory", from_rom ? "from ROM" : "from RAM");

run:

    for (i = 0; i < NTESTS; i++)
    {
        tres_t res;

        if (opt_only && strcmp(opt_only, tests[i].group)) continue;

        printf("  %-9s %-32s ", tests[i].group, tests[i].name);
        fflush(stdout);

        detail[0] = 0;
        res = tests[i].fn();

        switch (res) {
        case T_PASS: n_pass++; printf("ok  "); break;
        case T_FAIL: n_fail++; printf("FAIL"); break;
        case T_SKIP: n_skip++; printf("skip"); break;
        }

        if (detail[0]) printf("  %s", detail);
        printf("\n");

        if (!dev) continue;

        pump_stop();

        if (!device_healthy()) {
            printf("  %-9s %-32s      the device needed reopening after that\n", "", "");

            if (device_reopen() < 0) {
                printf("\n  the device is gone, cannot continue\n\n");
                return 1;
            }
        }
    }

    if (dev) {
        pump_stop();
        mirisdr_close(dev);
    }

    printf("\n  %d passed, %d failed, %d skipped\n\n", n_pass, n_fail, n_skip);

    return n_fail ? 1 : 0;
}
