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

static void stream_cb (unsigned char *buf, uint32_t len, void *ctx)
{
    (void) buf; (void) ctx;
    pump_bytes += (int) len;
}

static void *pump_main (void *arg)
{
    (void) arg;
    mirisdr_read_async(dev, stream_cb, NULL, 8, 65536);
    return NULL;
}

static int pump_start (void)
{
    if (pump_running) return 0;

    pump_bytes = 0;

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

    for (tries = 0; tries < 20; tries++) {
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

static tres_t t_rate_changes (void)
{
    static const uint32_t rates[] = {
        1300000, 2000000, 3000000, 4000000, 6000000, 8000000, 10000000, 12000000
    };
    mirisdr_stream_stats_t total;
    unsigned pass, i;
    int changes = 0, bad = 0;

    if (stream_setup("BULK", "504_S8", 2000000) < 0) return T_FAIL;

    memset(&total, 0, sizeof total);

    for (pass = 0; pass < 3; pass++)
        for (i = 0; i < sizeof rates / sizeof rates[0]; i++)
        {
            mirisdr_stream_stats_t d;
            double sps;

            if (mirisdr_set_sample_rate(dev, rates[i]) < 0) { say("rate %u refused", rates[i]); return T_FAIL; }

            usleep(200000);
            sps = stream_rate(0.35, &d);
            changes++;

            total.gaps    += d.gaps;
            total.lost    += d.lost;
            total.resyncs += d.resyncs;

            if (!within(sps, rates[i], 0.03)) {
                bad++;
                note("FAILED at %u: %.0f sps", rates[i], sps);
            }
        }

    say("%d rate changes, %d off rate, %llu gaps, %llu lost, %llu resyncs",
        changes, bad, (unsigned long long) total.gaps,
        (unsigned long long) total.lost, (unsigned long long) total.resyncs);

    if (bad) return T_FAIL;

    /* the firmware hands the pending packet over at the stop, so this is 0 */
    if (total.gaps) return T_FAIL;

    return T_PASS;
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

        if (mirisdr_read_mem(dev, addr, got, sizeof got, 1) < 0) {
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
        sleep(3);

        if (device_open() < 0) { say("parked, and it did not come back"); return T_FAIL; }
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
    uint8_t a[16], b[16];

    pump_stop();

    if (mirisdr_read_mem(dev, 0x0040, a, sizeof a, 0) < 0) { say("could not read 0x0040"); return T_FAIL; }
    if (mirisdr_read_mem(dev, 0x0040, b, sizeof b, 0) < 0) { say("second read failed"); return T_FAIL; }

    if (memcmp(a, b, sizeof a)) { say("two reads of the same address disagreed"); return T_FAIL; }

    if (fw_ours && memcmp(a, "BVDB", 4)) {
        say("our firmware is running but 0x0040 does not carry BVDB");
        return T_FAIL;
    }

    say("%02X%02X%02X%02X at 0x0040%s", a[0], a[1], a[2], a[3],
        fw_ours ? " - the information block" : "");

    return T_PASS;
}

static tres_t t_write_mem (void)
{
    uint8_t save[16], probe[16], back[16];
    unsigned i;

    if (!fw_ours) { say("needs our firmware, the ROM has nowhere safe to write"); return T_SKIP; }

    pump_stop();

    /* scratch above the image, below xdata */
    if (mirisdr_read_mem(dev, 0x1700, save, sizeof save, 0) < 0) { say("could not read the scratch area"); return T_FAIL; }

    for (i = 0; i < sizeof probe; i++) probe[i] = (uint8_t) (0xA5 ^ i);

    if (mirisdr_write_mem(dev, 0x1700, probe, sizeof probe, 0) < 0) { say("write failed"); return T_FAIL; }
    if (mirisdr_read_mem(dev, 0x1700, back, sizeof back, 0) < 0) { say("read back failed"); return T_FAIL; }

    mirisdr_write_mem(dev, 0x1700, save, sizeof save, 0);

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

    if (mirisdr_write_mem(dev, 0x1200, stub, sizeof stub, 0) < 0) { say("could not place the stub"); return T_FAIL; }

    memset(&regs, 0, sizeof regs);
    regs.r0 = 0x40;
    regs.r1 = 0x0F;

    if (mirisdr_call(dev, 0x1200, &regs) < 0) { say("the call failed"); return T_FAIL; }

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

static tres_t t_gain (void)
{
    int gains[64], n, i, before;

    pump_stop();

    before = mirisdr_get_tuner_gain(dev);
    n = mirisdr_get_tuner_gains(dev, gains);

    if (n <= 0) { say("the tuner reports no gain steps"); return T_FAIL; }

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

static tres_t t_pps (void)
{
    mirisdr_pps_t a, b;
    double t0;

    if (!fw_ours) { say("needs our firmware"); return T_SKIP; }
    if (!opt_pps) { say("needs a 1PPS on GPIO_0 and --pps"); return T_SKIP; }

    if (stream_setup("BULK", "504_S8", 2000000) < 0) return T_FAIL;

    if (mirisdr_enable_pps(dev, 1) < 0) { say("could not enable"); return T_FAIL; }

    if (mirisdr_get_pps(dev, &a) < 0) { mirisdr_enable_pps(dev, 0); say("could not read"); return T_FAIL; }

    t0 = now();

    while (now() - t0 < 3.0)
    {
        usleep(200000);

        if (mirisdr_get_pps(dev, &b) < 0) continue;
        if (b.edges != a.edges) break;
    }

    mirisdr_enable_pps(dev, 0);

    if (b.edges == a.edges) { say("no edge arrived in 3 s"); return T_FAIL; }

    say("edge %u at sample %llu, trusted %d, gapless %d", b.edges,
        (unsigned long long) b.sample, b.trusted, b.gapless);

    return T_PASS;
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
        sleep(3);
        if (device_open() < 0) { say("parked it and it did not come back"); return T_FAIL; }
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

/* The capture engine seems to have a hard bandwidth ceiling that is independent of the
 * chip or USB host, check for corruption in the internal buffer */
static int chars_stamps (void)
{
    static const uint8_t want[4] = { 'B', 'V', 'D', 'B' };
    int i, ok = 0;

    for (i = 0; i < 4; i++)
    {
        uint8_t got[4];

        if (mirisdr_read_mem(dev, (uint16_t) (0xE000 + i * 0x400 + 8), got, sizeof got, 1) < 0)
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
    sleep(3);

    if (device_open() < 0) return -1;
    if (stream_setup("BULK", "252_S16", rate) < 0) return -1;

    usleep(300000);

    return (chars_stamps() == 4) ? 0 : -1;
}

static int chars_stamped_at (uint32_t rate)
{
    int n;

    if (mirisdr_set_sample_rate(dev, rate) < 0) return -1;
    usleep(350000);
    n = chars_stamps();

    return (n < 0) ? -1 : (n == 4);
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

    if (chars_restamp(coarse_good) < 0) { say("could not restamp for the fine pass"); return T_FAIL; }

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

static const struct {
    const char *group;
    const char *name;
    tres_t    (*fn)(void);
} tests[] = {
    { "identity", "device enumerates",          t_enumerate           },
    { "identity", "usb descriptors",            t_usb_strings         },
    { "identity", "open by serial",             t_open_by_serial      },
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

    { "fixes",    "rate changes keep the stream", t_rate_changes      },
    { "fixes",    "repeated stop and start",    t_stop_start          },
    { "fixes",    "stall and clear recovers",   t_stall_recovery      },
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
    { "tuner",    "gain steps",                 t_gain                },
    { "tuner",    "filter bandwidths",          t_bandwidth           },
    { "tuner",    "settings round trip",        t_settings_roundtrip  },
    { "tuner",    "gpio inputs",                t_gpio_read           },

    { "extras",   "eeprom size probe",          t_eeprom_probe        },
    { "extras",   "eeprom read",                t_eeprom_read         },
    { "extras",   "eeprom write back",          t_eeprom_write        },
    { "extras",   "uart transmit",              t_uart                },
    { "extras",   "i2c bus",                    t_i2c                 },
    { "extras",   "pps timestamping",           t_pps                 },
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
           "  --eeprom-write   allow the eeprom write back test\n"
           "  --pps            run the pps test, needs a 1PPS on GPIO_0\n"
           "  --list           list the tests and exit\n"
           "\ngroups: identity stream fixes device tuner extras\n", me);
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
        else if (!strcmp(argv[i], "--eeprom-write")) opt_eeprom_write = 1;
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

        if (mirisdr_running_from_rom(dev) != 1) {
            mirisdr_reboot(dev, MIRISDR_BOOT_ROM);
            mirisdr_close(dev);
            dev = NULL;
            sleep(2);
        }
    }

    if (device_open() < 0) {
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

        pump_stop();

        if (!device_healthy()) {
            printf("  %-9s %-32s      the device needed reopening after that\n", "", "");

            if (device_reopen() < 0) {
                printf("\n  the device is gone, cannot continue\n\n");
                return 1;
            }
        }
    }

    pump_stop();
    mirisdr_close(dev);

    printf("\n  %d passed, %d failed, %d skipped\n\n", n_pass, n_fail, n_skip);

    return n_fail ? 1 : 0;
}
