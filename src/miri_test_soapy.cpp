/*
 * miri_test's soapy group: the SoapySDR module, loaded and used as an
 * application would. C++ because SoapySDR's C API differs between 0.7 and 0.8.
 */

#include "miri_test_soapy.h"

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Modules.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

typedef SoapySDR::Device Dev;

static double now(void)
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

/* quiet unless verbose: the module and library log refusals on purpose */
static void quietLog(const SoapySDRLogLevel, const char *) {}

static Dev *openDev(const char *module, const char *serial, soapy_say_t say)
{
    static std::string loaded;
    SoapySDR::Kwargs args;
    std::string err;

    if (loaded != module)
    {
        err = SoapySDR::loadModule(module);
        if (!err.empty() && err.find("already loaded") == std::string::npos)
        {
            say("cannot load %s: %s", module, err.c_str());
            return nullptr;
        }
        loaded = module;
    }

    args["driver"] = "mirisdr";
    if (serial) args["serial"] = serial;
    try { return Dev::make(args); }
    catch (const std::exception &e) { say("make failed: %s", e.what()); }
    return nullptr;
}

static bool has(const std::vector<std::string> &v, const char *s)
{
    return std::find(v.begin(), v.end(), s) != v.end();
}

/* names, formats, ranges and the gain stages a band has */
static int probe(Dev *d, soapy_say_t say, soapy_say_t note)
{
    std::vector<std::string> f = d->getStreamFormats(SOAPY_SDR_RX, 0), g;
    SoapySDR::RangeList fr = d->getFrequencyRange(SOAPY_SDR_RX, 0), rr = d->getSampleRateRange(SOAPY_SDR_RX, 0);
    std::vector<double> bw;
    double full;
    int bad = 0;

    if (d->getDriverKey() != "mirisdr") { say("driver %s", d->getDriverKey().c_str()); bad++; }
    if (d->getNumChannels(SOAPY_SDR_RX) != 1 || d->getNumChannels(SOAPY_SDR_TX) != 0) { say("channels wrong"); bad++; }
    if (!has(f, SOAPY_SDR_CS16) || !has(f, SOAPY_SDR_CF32) || !has(f, SOAPY_SDR_CS8)) { say("formats missing"); bad++; }
    if (d->getNativeStreamFormat(SOAPY_SDR_RX, 0, full) != SOAPY_SDR_CS16 || full != 32768) { say("native format wrong"); bad++; }
    if (fr.empty() || fr[0].minimum() > 1e6 || fr[0].maximum() < 2e9) { say("frequency range wrong"); bad++; }
    if (rr.empty() || rr[0].minimum() > 2e6 || rr[0].maximum() < 8e6) { say("rate range wrong"); bad++; }
    note("frequency %.3f-%.0f MHz, rate %.2f-%.2f Msps", fr[0].minimum() / 1e6, fr[0].maximum() / 1e6,
         rr[0].minimum() / 1e6, rr[0].maximum() / 1e6);

    g = d->listGains(SOAPY_SDR_RX, 0);
    if (g.size() != 4 || !has(g, "LNA") || !has(g, "MIX") || !has(g, "MIXBUF") || !has(g, "BB")) { say("gain stages wrong"); bad++; }

    /* VHF: LNA 24, mixer 19, no mixbuffer, baseband 59 */
    d->setFrequency(SOAPY_SDR_RX, 0, 100e6);
    if (d->getGainRange(SOAPY_SDR_RX, 0).maximum() != 102 ||
        d->getGainRange(SOAPY_SDR_RX, 0, "LNA").maximum() != 24 ||
        d->getGainRange(SOAPY_SDR_RX, 0, "MIX").maximum() != 19 ||
        d->getGainRange(SOAPY_SDR_RX, 0, "MIXBUF").maximum() != 0 ||
        d->getGainRange(SOAPY_SDR_RX, 0, "BB").maximum() != 59)
    { say("VHF gain ranges wrong, total %.0f", d->getGainRange(SOAPY_SDR_RX, 0).maximum()); bad++; }

    /* HF goes through AM2: a 24 dB mixbuffer and no LNA gain */
    d->setFrequency(SOAPY_SDR_RX, 0, 10e6);
    if (d->getGainRange(SOAPY_SDR_RX, 0, "MIXBUF").maximum() != 24 || d->getGainRange(SOAPY_SDR_RX, 0, "LNA").maximum() != 0)
    { say("HF gain ranges wrong"); bad++; }

    /* a total, then one stage: the others stay */
    d->setFrequency(SOAPY_SDR_RX, 0, 100e6);
    d->setGain(SOAPY_SDR_RX, 0, 40);
    if (d->getGain(SOAPY_SDR_RX, 0) != 40) { say("total gain 40 gave %.0f", d->getGain(SOAPY_SDR_RX, 0)); bad++; }
    d->setGain(SOAPY_SDR_RX, 0, "BB", 10);
    d->setGain(SOAPY_SDR_RX, 0, "LNA", 24);
    d->setGain(SOAPY_SDR_RX, 0, "MIX", 19);
    if (d->getGain(SOAPY_SDR_RX, 0, "BB") != 10 || d->getGain(SOAPY_SDR_RX, 0, "LNA") != 24 ||
        d->getGain(SOAPY_SDR_RX, 0) != 53)
    { say("stages BB 10, LNA and MIX on gave %.0f dB", d->getGain(SOAPY_SDR_RX, 0)); bad++; }

    /* the filters follow the IF; a request takes the narrowest that passes it */
    bw = d->listBandwidths(SOAPY_SDR_RX, 0);
    if (std::find(bw.begin(), bw.end(), 8e6) == bw.end()) { say("no 8 MHz filter at zero IF"); bad++; }
    d->setBandwidth(SOAPY_SDR_RX, 0, 5.5e6);
    if (d->getBandwidth(SOAPY_SDR_RX, 0) != 6e6) { say("5.5 MHz gave %.0f", d->getBandwidth(SOAPY_SDR_RX, 0)); bad++; }
    d->writeSetting("if_freq", "450000");
    bw = d->listBandwidths(SOAPY_SDR_RX, 0);
    if (bw.size() != 3 || bw[0] != 200e3 || bw[2] != 600e3) { say("low IF filters wrong"); bad++; }
    d->writeSetting("if_freq", "0");

    /* refused: logged, and the rate stays */
    d->setSampleRate(SOAPY_SDR_RX, 0, 2e6);
    d->setSampleRate(SOAPY_SDR_RX, 0, 100e6);
    if (d->getSampleRate(SOAPY_SDR_RX, 0) != 2e6) { say("a refused rate changed it"); bad++; }

    if (!bad) say("ranges, gain stages per band, filters per IF, refusals keep the state");
    return bad ? SOAPY_T_FAIL : SOAPY_T_PASS;
}

/* each format at 2 Msps, and 8 Msps over 8 bit USB samples: rate, time stamps, data */
static int stream(Dev *d, soapy_say_t say, soapy_say_t note)
{
    struct Case { const char *fmt; double rate; const char *usb; };
    const Case cases[] = { { SOAPY_SDR_CS16, 2e6, "AUTO" }, { SOAPY_SDR_CF32, 2e6, "AUTO" },
                           { SOAPY_SDR_CS8, 2e6, "AUTO" }, { SOAPY_SDR_CS16, 8e6, "504_S8" } };
    int bad = 0;

    d->setFrequency(SOAPY_SDR_RX, 0, 100e6);
    d->setGain(SOAPY_SDR_RX, 0, 40);

    for (const Case &c : cases)
    {
        d->writeSetting("format", c.usb);
        d->setSampleRate(SOAPY_SDR_RX, 0, c.rate);

        SoapySDR::Stream *s = d->setupStream(SOAPY_SDR_RX, c.fmt);
        size_t mtu = d->getStreamMTU(s);
        std::vector<int16_t> buf(4 * mtu);
        void *b = buf.data();
        long long total = 0, want = (long long) c.rate, next = -1, jumps = 0, ovf = 0, err = 0, notime = 0;
        double sum = 0, t0, dt;

        d->activateStream(s);
        /* the first buffers carry the start up */
        for (int i = 0; i < 20; i++) { int fl; long long t; d->readStream(s, &b, mtu, fl, t, 1000000); }
        t0 = now();
        while (total < want && !err)
        {
            int fl = 0; long long t = 0;
            int n = d->readStream(s, &b, mtu, fl, t, 1000000);

            if (n == SOAPY_SDR_OVERFLOW) { ovf++; next = -1; continue; }
            if (n < 0) { err = n; break; }
            if (!(fl & SOAPY_SDR_HAS_TIME)) notime++;
            if (next >= 0 && std::llabs(t - next) > 2) jumps++;
            next = t + (long long) std::llround(n * 1e9 / c.rate);

            /* the power, in the format's own scale */
            for (int i = 0; i < 2 * n; i += 64)
            {
                double v = c.fmt == std::string(SOAPY_SDR_CF32) ? ((float *) b)[i] :
                           c.fmt == std::string(SOAPY_SDR_CS8) ? ((int8_t *) b)[i] / 128.0 : ((int16_t *) b)[i] / 32768.0;
                sum += v * v;
            }
            total += n;
        }
        dt = now() - t0;
        d->deactivateStream(s);
        d->closeStream(s);

        double sps = total / dt, rms = std::sqrt(sum / (total / 32.0 + 1));
        note("%s at %.0f Msps over %s: %.0f sps, rms %.4f, %lld overflows, %lld time jumps",
             c.fmt, c.rate / 1e6, c.usb, sps, rms, ovf, jumps);
        if (err || ovf || jumps || notime || std::fabs(sps / c.rate - 1) > 0.05 || rms < 1e-4)
        {
            say("%s at %.0f Msps: %.0f sps, rms %.5f, %lld overflows, %lld jumps, error %lld",
                c.fmt, c.rate / 1e6, sps, rms, ovf, jumps, err);
            bad++;
        }
    }
    d->writeSetting("format", "AUTO");

    if (!bad) say("CS16, CF32, CS8 at 2 Msps and 8 Msps over 8 bit USB: full rate, time stamps continuous");
    return bad ? SOAPY_T_FAIL : SOAPY_T_PASS;
}

/* start and stop straight away, half of them after one read */
static int cycles(Dev *d, soapy_say_t say, soapy_say_t note)
{
    const int n = 100;
    int slow = 0, i;
    double worst = 0;

    d->setSampleRate(SOAPY_SDR_RX, 0, 2e6);
    SoapySDR::Stream *s = d->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16);
    std::vector<int16_t> buf(2 * d->getStreamMTU(s));
    void *b = buf.data();

    for (i = 0; i < n; i++)
    {
        double t0 = now();
        d->activateStream(s);
        if (i % 2) { int fl; long long t; d->readStream(s, &b, buf.size() / 2, fl, t, 1000000); }
        d->deactivateStream(s);
        worst = std::max(worst, now() - t0);
        if (now() - t0 > 2) { slow++; note("cycle %d took %.2f s", i, now() - t0); }
    }
    d->closeStream(s);

    say("%d start and stop cycles, %d slow, the longest %.0f ms", n, slow, worst * 1e3);
    return slow ? SOAPY_T_FAIL : SOAPY_T_PASS;
}

/* retunes while streaming: no overflow, time stamps continuous; then rate changes:
 * the stream restarts and still flows at the new rate */
static int retune(Dev *d, soapy_say_t say, soapy_say_t note)
{
    double t0, last = 0, rate;
    long long total = 0, ovf = 0, err = 0, jumps = 0, next = -1, tunes = 0, ticks = 0, rates = 0;
    int bad = 0;

    d->setSampleRate(SOAPY_SDR_RX, 0, 2e6);
    d->setFrequency(SOAPY_SDR_RX, 0, 100e6);
    SoapySDR::Stream *s = d->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32);
    std::vector<float> buf(2 * d->getStreamMTU(s));
    void *b = buf.data();

    d->activateStream(s);
    for (int phase = 0; phase < 2; phase++)
    {
        t0 = now();
        while (now() - t0 < 4)
        {
            int fl = 0; long long t = 0;
            int n = d->readStream(s, &b, buf.size() / 2, fl, t, 1000000);

            if (n == SOAPY_SDR_OVERFLOW) { ovf += !phase; next = -1; continue; }
            if (n < 0) { err++; continue; }
            if (!phase && next >= 0 && std::llabs(t - next) > 2) jumps++;
            rate = d->getSampleRate(SOAPY_SDR_RX, 0);
            next = t + (long long) std::llround(n * 1e9 / rate);
            total += n;

            if (now() - last > 0.05)
            {
                last = now();
                if (!phase) { d->setFrequency(SOAPY_SDR_RX, 0, 90e6 + (tunes % 20) * 1e6); tunes++; }
                else if ((ticks++ % 4) == 0)
                {
                    d->setSampleRate(SOAPY_SDR_RX, 0, (rates % 2) ? 2e6 : 8e6);
                    rates++;
                    next = -1;
                }
            }
        }
    }

    /* and it streams at the rate set last */
    long long got = 0;
    t0 = now();
    while (now() - t0 < 0.5)
    {
        int fl; long long t;
        int n = d->readStream(s, &b, buf.size() / 2, fl, t, 1000000);
        if (n > 0) got += n;
    }
    rate = got / (now() - t0);
    d->deactivateStream(s);
    d->closeStream(s);

    note("%lld retunes: %lld overflows, %lld time jumps; %lld rate changes, then %.0f sps at %.0f",
         tunes, ovf, jumps, rates, rate, d->getSampleRate(SOAPY_SDR_RX, 0));
    if (ovf || jumps || err || std::fabs(rate / d->getSampleRate(SOAPY_SDR_RX, 0) - 1) > 0.1) bad++;

    say("%lld retunes: %lld overflows, %lld time jumps; %lld rate changes, then %.0f sps; %lld errors",
        tunes, ovf, jumps, rates, rate, err);
    return bad ? SOAPY_T_FAIL : SOAPY_T_PASS;
}

extern "C" int soapy_test(const char *which, const char *module, const char *serial, int verbose,
                          soapy_say_t say, soapy_say_t note)
{
    Dev *d;
    int r = SOAPY_T_FAIL;

    if (!verbose) SoapySDR::registerLogHandler(quietLog);
    if (!(d = openDev(module, serial, say))) return SOAPY_T_FAIL;

    try
    {
        if (!strcmp(which, "probe")) r = probe(d, say, note);
        else if (!strcmp(which, "stream")) r = stream(d, say, note);
        else if (!strcmp(which, "cycles")) r = cycles(d, say, note);
        else if (!strcmp(which, "retune")) r = retune(d, say, note);
    }
    catch (const std::exception &e) { say("%s", e.what()); r = SOAPY_T_FAIL; }

    Dev::unmake(d);
    SoapySDR::registerLogHandler(nullptr);
    return r;
}
