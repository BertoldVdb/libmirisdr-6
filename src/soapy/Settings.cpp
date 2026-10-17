/*
 * SoapySDR module for libmirisdr: device, tuning and settings.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 */

#include "SoapyMiriSDR.hpp"

#include <SoapySDR/Logger.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

static const char *FORMATS[] = { "AUTO", "252_S16", "336_S16", "384_S16", "504_S16", "504_S8" };
static const char *TRANSFERS[] = { "BULK", "ISOC", "ISOC1", "ISOC2" };
static const char *BYPASS[] = { "AUTO", "ON", "OFF" };
static const uint32_t IFS[] = { 0, 450000, 1620000, 2048000 };
static const char *CONVERTERS[] = { "both", "I", "Q" };

/* every filter the tuner has; which ones an IF takes is asked from the library */
static const uint32_t BANDWIDTHS[] = { 200000, 300000, 600000, 1536000, 5000000, 6000000,
                                       7000000, 8000000, 14000000 };

static bool isTrue(const std::string &v)
{
    return v == "true" || v == "1" || v == "on" || v == "yes";
}

/* An IF setting as "auto" or the Hz of one the tuner has, anything else refused */
static bool parseIf(const std::string &v, std::string &mode)
{
    std::string l = v;
    char *end;
    unsigned long hz;

    std::transform(l.begin(), l.end(), l.begin(), [](unsigned char c) { return (char) std::tolower(c); });
    if (l == "auto")
    {
        mode = "auto";
        return true;
    }

    if (v.empty() || !std::isdigit((unsigned char) v[0])) return false;
    hz = std::strtoul(v.c_str(), &end, 10);
    if (*end || std::find(std::begin(IFS), std::end(IFS), (uint32_t) hz) == std::end(IFS)) return false;

    mode = std::to_string(hz);
    return true;
}

/* the IF asked for, 0 for automatic (or zero IF) */
uint32_t SoapyMiriSDR::fixedIf(void) const
{
    return ifMode == "auto" ? 0 : (uint32_t) std::strtoul(ifMode.c_str(), NULL, 10);
}

SoapyMiriSDR::SoapyMiriSDR(const SoapySDR::Kwargs &args):
    dev(nullptr), flavour(MIRISDR_HW_DEFAULT), ifFreq(0), ifMode("auto"), converters(MIRISDR_IQ_BOTH),
    baseband(true), wantRate(2048000), wantBw(0), argBw(0), bandGainRanges(false),
    freqMin(0), freqMax(0), rateMin(0), rateMax(0),
    stopping(false), rxDone(true), rxResult(0), outFormat(0),
    ringHead(0), ringTail(0), ringCount(0), dropPending(false), slotPos(0), gapPos(0),
    timeOffsetNs(0), lastIndex(0), lastRate(0), timeBaseNs(0), timeBaseIndex(0), timeBaseRate(0),
    asyncBuffers(0)
{
    mirisdr_open_config_t cfg;
    mirisdr_tune_config_t tc;
    char ser[256];

    /* the library finds the serial or port in the list it opens from */
    mirisdr_open_config_default(&cfg);
    if (args.count("serial")) cfg.serial = args.at("serial").c_str();
    if (args.count("usb_port")) cfg.usb_port = args.at("usb_port").c_str();

    if (mirisdr_open_ex(&dev, &cfg) < 0 || !dev)
        throw std::runtime_error("mirisdr: could not open the device");

    /* the device is ours from here: closed again if anything below throws */
    try {

    flavour = mirisdr_get_hw_flavour(dev);
    if (mirisdr_get_serial(dev, ser, sizeof ser) > 0) serial = ser;
    /* "1-3" here is "1:3" in the list */
    if ((mirisdr_get_usb_position(dev, NULL, NULL, ser, sizeof ser) == 0) && std::strchr(ser, '-'))
    {
        *std::strchr(ser, '-') = ':';
        usbPort = ser;
    }

    /* the IF settings may come with the device, for programs that only take a string */
    if (args.count("if_freq") && !parseIf(args.at("if_freq"), ifMode))
        SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: no IF %s, using auto", args.at("if_freq").c_str());
    if (args.count("converters"))
        converters = args.at("converters") == "I" ? MIRISDR_IQ_ONLY_I :
                     args.at("converters") == "Q" ? MIRISDR_IQ_ONLY_Q : MIRISDR_IQ_BOTH;
    if (args.count("baseband")) baseband = isTrue(args.at("baseband"));
    /* the filter, for programs that have no field for it */
    if (args.count("bandwidth"))
    {
        try { argBw = (uint32_t) std::stoul(args.at("bandwidth")); }
        catch (const std::exception &) { SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: bandwidth %s is not a number of Hz", args.at("bandwidth").c_str()); }
        wantBw = argBw;
    }
    /* the gain ranges: "max", the default, stay the same in every band, as programs
       such as GQRX read them once at open; "band" has only what the band tuned has */
    if (args.count("gain_ranges"))
    {
        if (args.at("gain_ranges") == "band") bandGainRanges = true;
        else if (args.at("gain_ranges") != "max")
            SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: gain_ranges %s is not max or band", args.at("gain_ranges").c_str());
    }

    /* the library's defaults, its filter and a known gain, the ranges at zero IF */
    mirisdr_get_stream(dev, &streamCfg, NULL);
    streamCfg.rate = 2048000;
    streamCfg.baseband = 0;
    applyStream(streamCfg);

    mirisdr_get_tune(dev, 0, &tc, NULL);
    tc.frequency = 100000000;
    tc.if_freq = 0;
    tc.bandwidth = 0;
    tc.gain.mode = MIRISDR_GAIN_TOTAL;
    tc.gain.total = 40;
    applyTune(tc);

    probeRanges();
    configure(wantRate);

    } catch (...) {
        mirisdr_close(dev);
        dev = nullptr;
        throw;
    }

    SoapySDR_logf(SOAPY_SDR_INFO, "mirisdr: opened %s at %s", serial.empty() ? "a receiver without serial" : serial.c_str(),
                  usbPort.empty() ? "an unknown port" : usbPort.c_str());
}

SoapyMiriSDR::~SoapyMiriSDR(void)
{
    if (rxThread.joinable()) deactivateStream(nullptr, 0, 0);
    mirisdr_close(dev);
}

/*******************************************************************
 * Applying the config: refused combinations are logged, not thrown,
 * since applications rarely catch, and the device keeps what it had
 ******************************************************************/

void SoapyMiriSDR::applyStream(const mirisdr_stream_config_t &cfg)
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_stream_config_t c = cfg;

    c.format = formatSetting.empty() ? NULL : formatSetting.c_str();
    c.transfer = transferSetting.empty() ? NULL : transferSetting.c_str();
    c.decimation_bypass = bypassSetting.empty() ? NULL : bypassSetting.c_str();
    /* raw, a single converter gives real samples */
    c.follow_tune = !c.baseband;

    if (mirisdr_set_stream(dev, &c, NULL) < 0)
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: stream refused (%u S/s, format %s, transfer %s)",
                      c.rate, c.format ? c.format : "AUTO", c.transfer ? c.transfer : "default");
        return;
    }
    streamCfg = c;
    streamCfg.format = streamCfg.transfer = streamCfg.decimation_bypass = NULL;
}

void SoapyMiriSDR::applyTune(const mirisdr_tune_config_t &cfg)
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);

    if (mirisdr_tune(dev, 0, &cfg, NULL) < 0)
        SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: tune refused (%u Hz, IF %u, bandwidth %u)",
                      cfg.frequency, cfg.if_freq, cfg.bandwidth);
}

/* what the library accepts, found once by asking it */
void SoapyMiriSDR::probeRanges(void)
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;
    mirisdr_stream_config_t sc;
    double lo, hi, mid;
    int i;

    mirisdr_get_tune(dev, 0, &tc, NULL);
    tc.gain.mode = MIRISDR_GAIN_KEEP;

    for (freqMin = 1000; freqMin < 100e6; freqMin *= 2)
    {
        tc.frequency = (uint32_t) freqMin;
        if (mirisdr_tune_check(dev, 0, &tc, NULL) == 0) break;
    }
    lo = 1e9; hi = 4e9;
    for (i = 0; i < 40; i++)
    {
        mid = std::floor((lo + hi) / 2);
        tc.frequency = (uint32_t) mid;
        if (mirisdr_tune_check(dev, 0, &tc, NULL) == 0) lo = mid; else hi = mid;
    }
    freqMax = lo;

    mirisdr_get_stream(dev, &sc, NULL);
    sc.baseband = 0;
    sc.follow_tune = 0;
    lo = 1; hi = 4e6;
    for (i = 0; i < 40; i++)
    {
        mid = std::floor((lo + hi) / 2);
        sc.rate = (uint32_t) mid;
        if (mirisdr_stream_check(dev, &sc, NULL) == 0) hi = mid; else lo = mid;
    }
    rateMin = hi;
    lo = 4e6; hi = 100e6;
    for (i = 0; i < 40; i++)
    {
        mid = std::floor((lo + hi) / 2);
        sc.rate = (uint32_t) mid;
        if (mirisdr_stream_check(dev, &sc, NULL) == 0) lo = mid; else hi = mid;
    }
    rateMax = lo;
}

/*******************************************************************
 * The IF and the rate. Baseband (the default) delivers the band at 0 Hz
 * whatever the IF. "auto" keeps zero IF from rateMin up, as without
 * baseband, and below it takes a low IF. A low IF takes any rate in a
 * span (see the library's baseband rates): the converters at 4 x IF up
 * to 7 Msps, else at the least the IF takes, and a resampler for what
 * halving does not reach. The cheapest are 4 x IF divided down, which
 * need neither the resampler nor the NCO
 ******************************************************************/

/* the rates of a low IF that need no resampler or NCO: 4 x IF divided by a power of 2, by
   2 at least for a single converter */
std::vector<uint32_t> SoapyMiriSDR::lowIfRates(uint32_t ifHz) const
{
    std::vector<uint32_t> rates;
    int one = converters != MIRISDR_IQ_BOTH, k;
    uint64_t base = (uint64_t) 4 * ifHz / (one ? 2 : 1);

    for (k = 0; k <= 8 - one && !(base % ((uint64_t) 1 << k)); k++) rates.push_back((uint32_t) (base >> k));
    return rates;
}

/* the converter rate a low IF runs at, as the library has it: 4 x IF up to 7 Msps, else
   the least the IF takes, 2.4 x (IF + half its widest filter) */
static uint64_t convPref(uint32_t ifHz)
{
    const uint64_t widest = ifHz > 450000 ? 1536000 : 600000;
    const uint64_t least = std::max<uint64_t>(((2 * (uint64_t) ifHz + widest) * 6 + 4) / 5, 1300000);

    return 4 * (uint64_t) ifHz <= 7000000 ? 4 * (uint64_t) ifHz : least;
}

/* the rates a low IF offers: from just above that over 512 (8 halvings and the resampler's
   2 at most), to 4 x IF (2 x IF with one converter), past which the tuner's filter leaves
   nothing. The library takes more, up to the converters' most */
void SoapyMiriSDR::lowIfRange(uint32_t ifHz, uint32_t &lo, uint32_t &hi) const
{
    const uint64_t one = converters != MIRISDR_IQ_BOTH ? 1 : 0;

    lo = (uint32_t) (convPref(ifHz) / 512 + 1);
    hi = (uint32_t) std::min<uint64_t>((uint64_t) rateMax, 4 * (uint64_t) ifHz) / (one ? 2 : 1);
}

/* "auto" below rateMin: an IF that has this rate free, as before; else the one with the
   least converter rate whose widest filter passes most of it */
uint32_t SoapyMiriSDR::pickLowIf(uint32_t rate) const
{
    const int one = converters != MIRISDR_IQ_BOTH;
    uint32_t pick = 0, lo, hi;
    uint64_t best = 0;

    for (uint32_t f : IFS)
    {
        std::vector<uint32_t> r = lowIfRates(f);
        if (f && std::find(r.begin(), r.end(), rate) != r.end()) return f;
    }

    for (uint32_t f : IFS)
    {
        uint64_t widest = f > 450000 ? 1536000 : 600000, conv;

        lowIfRange(f, lo, hi);
        if (!f || rate < lo || rate > hi || (widest < 0.8 * rate && f != 2048000)) continue;
        conv = std::max<uint64_t>(convPref(f), (uint64_t) rate << one);
        if (!pick || conv < best) { pick = f; best = conv; }
    }

    return pick ? pick : IFS[1];
}

bool SoapyMiriSDR::realOut(void) const
{
    return !baseband && ifFreq && converters != MIRISDR_IQ_BOTH;
}

void SoapyMiriSDR::configure(uint32_t rate)
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;
    mirisdr_stream_config_t c = streamCfg;
    uint32_t ifHz = 0, out = rate;

    ifHz = fixedIf();

    /* below the converters own minimum only a low IF gets there */
    if (baseband && ifMode == "auto" && rate < rateMin) ifHz = pickLowIf(rate);

    /* a low IF has a span of rates: the nearest end outside it */
    if (baseband && ifHz)
    {
        uint32_t lo, hi;

        lowIfRange(ifHz, lo, hi);
        out = std::min(std::max(rate, lo), hi);
    }

    /* the tune: the IF, one converter or both, the filter nearest to what was asked */
    mirisdr_get_tune(dev, 0, &tc, NULL);
    tc.if_freq = ifHz;
    tc.iq = ifHz ? converters : MIRISDR_IQ_BOTH;
    tc.low_if_auto = 0;
    tc.bandwidth = pickBandwidth(ifHz);
    tc.gain.mode = MIRISDR_GAIN_KEEP;
    if (mirisdr_tune(dev, 0, &tc, NULL) < 0)
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: IF %u refused", ifHz);
        return;
    }
    ifFreq = ifHz;

    c.rate = out;
    c.baseband = baseband;
    applyStream(c);
    if (out != rate) SoapySDR_logf(SOAPY_SDR_INFO, "mirisdr: %u S/s, the nearest to %u with a %u Hz IF", out, rate, ifHz);

    /* a rate that is not the converters' halved, or with them off 4 x IF: say which
       nearby ones need neither. Direct sampling always runs the NCO */
    mirisdr_stream_result_t r;
    mirisdr_tune_result_t tr;
    if (mirisdr_get_tune(dev, 0, NULL, &tr) == 0 && tr.frontend.direct_sampling) return;
    if (mirisdr_get_stream(dev, NULL, &r) == 0 && (r.nco || r.resampled))
    {
        uint32_t below = 0, above = 0;

        for (uint32_t f : lowIfRates(ifHz))
        {
            if (f < r.rate && f > below) below = f;
            if (f > r.rate && (!above || f < above)) above = f;
        }
        SoapySDR_logf(SOAPY_SDR_WARNING, "mirisdr: %u S/s with a %u Hz IF needs the %s, the receiver at %u S/s: "
                      "more CPU than %u%s%.0u S/s, which need neither", r.rate, ifHz,
                      r.nco && r.resampled ? "resampler and the NCO" : r.nco ? "NCO" : "resampler", r.adc_rate,
                      below ? below : above, below && above ? " or " : "", below && above ? above : 0);
    }
}

/*******************************************************************
 * Identification
 ******************************************************************/

std::string SoapyMiriSDR::getDriverKey(void) const
{
    return SOAPY_MIRISDR_DRIVER;
}

std::string SoapyMiriSDR::getHardwareKey(void) const
{
    switch (flavour)
    {
    case MIRISDR_HW_RSP1B:   return "RSP1B";
    case MIRISDR_HW_RSPDXR2: return "RSPdxR2";
    case MIRISDR_HW_RSPDUO:  return "RSPduo";
    case MIRISDR_HW_SDRPLAY: return "SDRplay";
    default:                 return "MSi2500";
    }
}

SoapySDR::Kwargs SoapyMiriSDR::getHardwareInfo(void) const
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    SoapySDR::Kwargs info;
    uint8_t id[MIRISDR_FW_ID_LEN];
    uint16_t vid, pid;
    char s[64];
    int i;

    if (!serial.empty()) info["serial"] = serial;
    if (!usbPort.empty()) info["usb_port"] = usbPort;
    if (mirisdr_get_usb_ids(dev, &vid, &pid) == 0)
    {
        snprintf(s, sizeof s, "%04x:%04x", vid, pid);
        info["usb_id"] = s;
    }
    if (mirisdr_running_from_rom(dev) == 1) info["firmware"] = "ROM";
    else if (mirisdr_get_fw_id(dev, id, sizeof id) == (int) sizeof id)
    {
        for (i = 0; i < (int) sizeof id; i++) snprintf(s + 2 * i, 3, "%02x", id[i]);
        info["firmware"] = s;
    }
    return info;
}

size_t SoapyMiriSDR::getNumChannels(const int direction) const
{
    return direction == SOAPY_SDR_RX ? 1 : 0;
}

/*******************************************************************
 * Antenna
 ******************************************************************/

/* The RSPdx-R2 has three ports, A, B and C, other boards one */
std::vector<std::string> SoapyMiriSDR::listAntennas(const int direction, const size_t channel) const
{
    if (flavour == MIRISDR_HW_RSPDXR2) return { "A", "B", "C" };
    return { "RX" };
}

void SoapyMiriSDR::setAntenna(const int direction, const size_t channel, const std::string &name)
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);

    if (flavour == MIRISDR_HW_RSPDXR2 && (name == "A" || name == "B" || name == "C"))
    {
        if (mirisdr_set_antenna(dev, MIRISDR_ANTENNA_A + (name[0] - 'A')) < 0)
            SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: antenna %s refused", name.c_str());
    }
    else if (name != "RX" || flavour == MIRISDR_HW_RSPDXR2)
        SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: no antenna %s", name.c_str());
}

std::string SoapyMiriSDR::getAntenna(const int direction, const size_t channel) const
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    int a;

    if (flavour != MIRISDR_HW_RSPDXR2) return "RX";
    a = mirisdr_get_antenna(dev);
    return std::string(1, (char) ('A' + ((a >= MIRISDR_ANTENNA_A && a <= MIRISDR_ANTENNA_C) ? a : 0)));
}

/*******************************************************************
 * DC offset: the library takes it off in software under baseband, by
 * default at a low IF and not at zero IF, where it is the band's
 * centre. Setting the mode turns it on or off at every IF
 ******************************************************************/

bool SoapyMiriSDR::hasDCOffsetMode(const int direction, const size_t channel) const
{
    return true;
}

void SoapyMiriSDR::setDCOffsetMode(const int direction, const size_t channel, const bool automatic)
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;

    mirisdr_get_tune(dev, 0, &tc, NULL);
    tc.dc_removal = automatic ? MIRISDR_DC_ON : MIRISDR_DC_OFF;
    tc.gain.mode = MIRISDR_GAIN_KEEP;
    applyTune(tc);
}

bool SoapyMiriSDR::getDCOffsetMode(const int direction, const size_t channel) const
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;

    mirisdr_get_tune(dev, 0, &tc, NULL);
    if (!baseband || tc.dc_removal == MIRISDR_DC_OFF) return false;
    return tc.dc_removal == MIRISDR_DC_ON || ifFreq != 0;
}

/*******************************************************************
 * Gain: the total split by the library, or each stage. The stages a
 * band has and their steps are asked from the library
 ******************************************************************/

std::vector<std::string> SoapyMiriSDR::listGains(const int direction, const size_t channel) const
{
    return { "LNA", "MIX", "MIXBUF", "BB" };
}

SoapyMiriSDR::StageRanges SoapyMiriSDR::stageRanges(void) const
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;
    mirisdr_tune_result_t r;
    StageRanges sr = { 0, 0, 0, { 0 } };
    int base, v;

    mirisdr_get_tune(dev, 0, &tc, NULL);
    tc.gain.mode = MIRISDR_GAIN_STAGES;
    tc.gain.lna = tc.gain.mixer = tc.gain.mixbuffer = tc.gain.baseband = 0;
    if (mirisdr_tune_check(dev, 0, &tc, &r) < 0) return sr;
    base = r.gain.total;

    tc.gain.lna = 1;
    if (mirisdr_tune_check(dev, 0, &tc, &r) == 0) sr.lna = r.gain.total - base;
    tc.gain.lna = 0;

    tc.gain.mixer = 1;
    if (mirisdr_tune_check(dev, 0, &tc, &r) == 0) sr.mixer = r.gain.total - base;
    tc.gain.mixer = 0;

    /* only the values that add gain: bands without the stage take any */
    for (v = 1; v <= 24; v++)
    {
        tc.gain.mixbuffer = v;
        if (mirisdr_tune_check(dev, 0, &tc, &r) == 0 && r.gain.total - base == v) sr.mixbuffer.push_back(v);
    }
    tc.gain.mixbuffer = 0;

    for (v = 59; v > 0; v--)
    {
        tc.gain.baseband = v;
        if (mirisdr_tune_check(dev, 0, &tc, &r) == 0) break;
    }
    /* none where it adds nothing: under direct sampling only the LNA counts */
    sr.baseband = (v > 0 && r.gain.total == base) ? 0 : v;

    return sr;
}

void SoapyMiriSDR::setGain(const int direction, const size_t channel, const double value)
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;

    mirisdr_get_tune(dev, 0, &tc, NULL);
    tc.gain.mode = MIRISDR_GAIN_TOTAL;
    tc.gain.total = std::max(0, (int) std::lround(value));
    applyTune(tc);
}

void SoapyMiriSDR::setGain(const int direction, const size_t channel, const std::string &name, const double value)
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;
    mirisdr_tune_result_t r;
    StageRanges sr = stageRanges();
    int v = (int) std::lround(value), best, d;

    /* the stages in force, then this one changed */
    mirisdr_get_tune(dev, 0, &tc, &r);
    tc.gain = r.gain;
    tc.gain.mode = MIRISDR_GAIN_STAGES;

    if (name == "LNA") tc.gain.lna = (sr.lna > 0 && 2 * v >= sr.lna) ? 1 : 0;
    else if (name == "MIX") tc.gain.mixer = (sr.mixer > 0 && 2 * v >= sr.mixer) ? 1 : 0;
    else if (name == "BB") tc.gain.baseband = std::min(std::max(v, 0), sr.baseband);
    else if (name == "MIXBUF")
    {
        best = 0;
        for (int m : sr.mixbuffer)
        {
            d = std::abs(m - v);
            if (d < std::abs(best - v)) best = m;
        }
        tc.gain.mixbuffer = best;
    }
    else
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: no gain %s", name.c_str());
        return;
    }
    applyTune(tc);
}

double SoapyMiriSDR::getGain(const int direction, const size_t channel) const
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_result_t r;

    if (mirisdr_get_tune(dev, 0, NULL, &r) < 0) return 0;
    return r.gain.total;
}

double SoapyMiriSDR::getGain(const int direction, const size_t channel, const std::string &name) const
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_result_t r;
    StageRanges sr;

    if (mirisdr_get_tune(dev, 0, NULL, &r) < 0) return 0;
    if (name == "MIXBUF") return r.gain.mixbuffer;
    if (name == "BB") return r.gain.baseband;

    sr = stageRanges();
    if (name == "LNA") return r.gain.lna ? sr.lna : 0;
    if (name == "MIX") return r.gain.mixer ? sr.mixer : 0;
    return 0;
}

/* The most each stage reaches in any band: the LNA below band IV/V, the mixbuffer on
   AM2, which never run together. The band plans use AM2 only, so the mixbuffer is 0 or
   24 dB (AM1 would step by 6) */
#define MAX_LNA                 24
#define MAX_MIX                 19
#define MAX_MIXBUF              24
#define MAX_BB                  59

SoapySDR::Range SoapyMiriSDR::getGainRange(const int direction, const size_t channel) const
{
    StageRanges sr;

    if (!bandGainRanges) return SoapySDR::Range(0, std::max(MAX_LNA, MAX_MIXBUF) + MAX_MIX + MAX_BB, 1);

    sr = stageRanges();
    return SoapySDR::Range(0, sr.lna + sr.mixer + sr.mixbuffer.back() + sr.baseband, 1);
}

SoapySDR::Range SoapyMiriSDR::getGainRange(const int direction, const size_t channel, const std::string &name) const
{
    StageRanges sr;

    if (!bandGainRanges)
    {
        if (name == "LNA") return SoapySDR::Range(0, MAX_LNA, MAX_LNA);
        if (name == "MIX") return SoapySDR::Range(0, MAX_MIX, MAX_MIX);
        if (name == "MIXBUF") return SoapySDR::Range(0, MAX_MIXBUF, MAX_MIXBUF);
        if (name == "BB") return SoapySDR::Range(0, MAX_BB, 1);
        return SoapySDR::Range(0, 0);
    }

    sr = stageRanges();

    if (name == "LNA") return SoapySDR::Range(0, sr.lna, sr.lna ? sr.lna : 1);
    if (name == "MIX") return SoapySDR::Range(0, sr.mixer, sr.mixer ? sr.mixer : 1);
    if (name == "BB") return SoapySDR::Range(0, sr.baseband, 1);
    if (name == "MIXBUF")
        return SoapySDR::Range(0, sr.mixbuffer.back(), sr.mixbuffer.size() > 1 ? sr.mixbuffer[1] : 1);
    return SoapySDR::Range(0, 0);
}

/*******************************************************************
 * Frequency: the LO, which is the centre of the stream. With a low IF
 * the band received lies the IF below it
 ******************************************************************/

void SoapyMiriSDR::setFrequency(const int direction, const size_t channel, const double frequency,
                                const SoapySDR::Kwargs &args)
{
    setFrequency(direction, channel, "RF", frequency, args);
}

void SoapyMiriSDR::setFrequency(const int direction, const size_t channel, const std::string &name,
                                const double frequency, const SoapySDR::Kwargs &args)
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;

    if (name != "RF")
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: no frequency %s", name.c_str());
        return;
    }
    if (frequency < 0 || frequency > 4294967295.0)
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: cannot tune to %.0f Hz", frequency);
        return;
    }

    mirisdr_get_tune(dev, 0, &tc, NULL);
    tc.frequency = (uint32_t) std::llround(frequency);
    tc.low_if_auto = 0;
    tc.gain.mode = MIRISDR_GAIN_KEEP;
    applyTune(tc);
}

double SoapyMiriSDR::getFrequency(const int direction, const size_t channel) const
{
    return getFrequency(direction, channel, "RF");
}

double SoapyMiriSDR::getFrequency(const int direction, const size_t channel, const std::string &name) const
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_result_t r;

    mirisdr_tune_config_t c;

    if (name != "RF" || mirisdr_get_tune(dev, 0, &c, &r) < 0) return 0;
    /* baseband: the band asked for is the centre; raw, the LO is */
    return baseband ? c.frequency : r.lo;
}

std::vector<std::string> SoapyMiriSDR::listFrequencies(const int direction, const size_t channel) const
{
    return { "RF" };
}

SoapySDR::RangeList SoapyMiriSDR::getFrequencyRange(const int direction, const size_t channel) const
{
    return getFrequencyRange(direction, channel, "RF");
}

SoapySDR::RangeList SoapyMiriSDR::getFrequencyRange(const int direction, const size_t channel,
                                                    const std::string &name) const
{
    return { SoapySDR::Range(freqMin, freqMax) };
}

/*******************************************************************
 * Sample rate and bandwidth
 ******************************************************************/

void SoapyMiriSDR::setSampleRate(const int direction, const size_t channel, const double rate)
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);

    wantRate = (uint32_t) std::llround(rate);
    configure(wantRate);
}

double SoapyMiriSDR::getSampleRate(const int direction, const size_t channel) const
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_stream_result_t r;

    if (mirisdr_get_stream(dev, NULL, &r) < 0) return 0;
    return r.rate;
}

/* rates within a span, once each */
static void addRates(std::vector<double> &v, const std::vector<double> &rates, double lo, double hi)
{
    for (double r : rates)
        if (r >= lo && r <= hi && std::find(v.begin(), v.end(), r) == v.end()) v.push_back(r);
}

/* A list of the usual ones. With a low IF those needing no NCO, from 25 ksps, within
   its span */
std::vector<double> SoapyMiriSDR::listSampleRates(const int direction, const size_t channel) const
{
    static const std::vector<double> common = { 1.536e6, 2e6, 2.048e6, 2.4e6, 3e6, 4e6, 5e6, 6e6, 7e6,
                                                8e6, 9e6, 10e6, 12e6, 14e6 };
    static const std::vector<double> low = { 48e3, 96e3, 192e3, 250e3, 500e3, 1e6 };
    std::vector<double> rates;
    uint32_t fixed = fixedIf(), lo, hi;

    if (baseband && fixed)
    {
        lowIfRange(fixed, lo, hi);
        for (uint32_t r : lowIfRates(fixed)) addRates(rates, { (double) r }, std::max<double>(lo, 25000), hi);
        addRates(rates, low, lo, hi);
        addRates(rates, common, lo, hi);
    }
    else
    {
        if (baseband)
            for (uint32_t f : IFS)
                if (f)
                {
                    lowIfRange(f, lo, hi);
                    for (uint32_t r : lowIfRates(f)) addRates(rates, { (double) r }, std::max<double>(lo, 25000), rateMin - 1);
                    addRates(rates, low, lo, rateMin - 1);
                }
        addRates(rates, common, rateMin, rateMax);
    }
    std::sort(rates.begin(), rates.end());
    return rates;
}

/* Any rate in a span: a low IF's, or with "auto" the lowest of them up to zero IF's most */
SoapySDR::RangeList SoapyMiriSDR::getSampleRateRange(const int direction, const size_t channel) const
{
    SoapySDR::RangeList out;
    uint32_t fixed = fixedIf(), lo, hi;

    if (baseband && fixed)
    {
        lowIfRange(fixed, lo, hi);
        out.push_back(SoapySDR::Range(lo, hi));
    }
    else if (baseband)
    {
        lowIfRange(IFS[1], lo, hi);
        out.push_back(SoapySDR::Range(std::min<double>(lo, rateMin), rateMax));
    }
    else out.push_back(SoapySDR::Range(rateMin, rateMax));
    return out;
}

std::vector<uint32_t> SoapyMiriSDR::bandwidthsFor(uint32_t ifHz) const
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    std::vector<uint32_t> bws;
    mirisdr_tune_config_t tc;

    mirisdr_get_tune(dev, 0, &tc, NULL);
    tc.if_freq = ifHz;
    tc.gain.mode = MIRISDR_GAIN_KEEP;
    for (uint32_t bw : BANDWIDTHS)
    {
        tc.bandwidth = bw;
        if (mirisdr_tune_check(dev, 0, &tc, NULL) == 0) bws.push_back(bw);
    }
    return bws;
}

/* the narrowest filter of the IF that passes wantBw, else its widest. 0 when
   nothing is asked: the library picks */
uint32_t SoapyMiriSDR::pickBandwidth(uint32_t ifHz) const
{
    std::vector<uint32_t> bws = bandwidthsFor(ifHz);

    if (!wantBw || bws.empty()) return 0;
    for (uint32_t b : bws)
        if (b >= wantBw) return b;
    return bws.back();
}

void SoapyMiriSDR::setBandwidth(const int direction, const size_t channel, const double bw)
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;

    /* remembered, so a change of IF takes the nearest it has. 0 is the device
       string's, or else the library's choice */
    wantBw = bw > 0 ? (uint32_t) bw : argBw;

    mirisdr_get_tune(dev, 0, &tc, NULL);
    tc.bandwidth = pickBandwidth(ifFreq);
    tc.gain.mode = MIRISDR_GAIN_KEEP;
    applyTune(tc);
}

double SoapyMiriSDR::getBandwidth(const int direction, const size_t channel) const
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_result_t r;

    if (mirisdr_get_tune(dev, 0, NULL, &r) < 0) return 0;
    return r.bandwidth;
}

std::vector<double> SoapyMiriSDR::listBandwidths(const int direction, const size_t channel) const
{
    std::vector<double> out;

    for (uint32_t b : bandwidthsFor(ifFreq)) out.push_back(b);
    return out;
}

SoapySDR::RangeList SoapyMiriSDR::getBandwidthRange(const int direction, const size_t channel) const
{
    SoapySDR::RangeList out;

    for (uint32_t b : bandwidthsFor(ifFreq)) out.push_back(SoapySDR::Range(b, b));
    return out;
}

/*******************************************************************
 * Settings
 ******************************************************************/

SoapySDR::ArgInfoList SoapyMiriSDR::getSettingInfo(void) const
{
    SoapySDR::ArgInfoList list;
    SoapySDR::ArgInfo a;

    a = SoapySDR::ArgInfo();
    a.key = "if_freq";
    a.name = "IF";
    a.description = "Tuner IF in Hz. auto: zero IF from 1.3 Msps up, a low IF below. A low IF has its own "
                    "sample rates and filters.";
    a.type = SoapySDR::ArgInfo::STRING;
    a.value = "auto";
    a.options.push_back("auto");
    for (uint32_t f : IFS) a.options.push_back(std::to_string(f));
    a.optionNames = { "Auto", "Zero IF", "450 kHz", "1620 kHz", "2048 kHz" };
    list.push_back(a);

    a = SoapySDR::ArgInfo();
    a.key = "converters";
    a.name = "Converters";
    a.description = "With a low IF: both, or one (I or Q), which halves the USB data and keeps more bits "
                    "at high rates, using more CPU.";
    a.type = SoapySDR::ArgInfo::STRING;
    a.value = "both";
    for (const char *c : CONVERTERS) a.options.push_back(c);
    list.push_back(a);

    a = SoapySDR::ArgInfo();
    a.key = "baseband";
    a.name = "Baseband";
    a.description = "The frequency set at the centre of the stream at any IF. Off: the stream "
                    "unmodified, the LO at the centre, real samples from a single converter.";
    a.type = SoapySDR::ArgInfo::BOOL;
    a.value = "true";
    list.push_back(a);

    a = SoapySDR::ArgInfo();
    a.key = "format";
    a.name = "USB sample format";
    a.description = "AUTO picks a format that fits automatically.";
    a.type = SoapySDR::ArgInfo::STRING;
    a.value = "AUTO";
    for (const char *f : FORMATS) a.options.push_back(f);
    list.push_back(a);

    a = SoapySDR::ArgInfo();
    a.key = "transfer";
    a.name = "USB transfer";
    a.description = "Bulk, or isochronous with 3, 1 or 2 packets per microframe.";
    a.type = SoapySDR::ArgInfo::STRING;
    a.value = readSetting("transfer");
    for (const char *t : TRANSFERS) a.options.push_back(t);
    list.push_back(a);

    a = SoapySDR::ArgInfo();
    a.key = "decimation_bypass";
    a.name = "Decimation bypass";
    a.description = "Run the ADC at the sample rate without the internal filter. AUTO above 14.5 Msps.";
    a.type = SoapySDR::ArgInfo::STRING;
    a.value = "AUTO";
    for (const char *b : BYPASS) a.options.push_back(b);
    list.push_back(a);

    a = SoapySDR::ArgInfo();
    a.key = "gap_fill";
    a.name = "Gap fill";
    a.description = "Put zeros where samples were lost on the USB, so the timing stays right. Always on with baseband, the default.";
    a.type = SoapySDR::ArgInfo::BOOL;
    a.value = "false";
    list.push_back(a);

    if (flavour == MIRISDR_HW_RSP1B || flavour == MIRISDR_HW_RSPDXR2)
    {
        a = SoapySDR::ArgInfo();
        a.key = "biastee";
        a.name = "Bias-T";
        a.description = flavour == MIRISDR_HW_RSPDXR2 ? "Power on antenna port B." : "Power on the antenna input.";
        a.type = SoapySDR::ArgInfo::BOOL;
        a.value = "false";
        list.push_back(a);
    }

    if (flavour == MIRISDR_HW_RSP1B || flavour == MIRISDR_HW_RSPDXR2 || flavour == MIRISDR_HW_RSPDUO)
    {
        a = SoapySDR::ArgInfo();
        a.key = "fm_notch";
        a.name = "FM notch";
        a.description = "Notch for the FM band (85-100 MHz).";
        a.type = SoapySDR::ArgInfo::BOOL;
        a.value = "false";
        list.push_back(a);

        a = SoapySDR::ArgInfo();
        a.key = "mw_notch";
        a.name = "MW notch";
        a.description = "Notch for medium wave (0.4-1.6 MHz).";
        a.type = SoapySDR::ArgInfo::BOOL;
        a.value = "false";
        list.push_back(a);

        a = SoapySDR::ArgInfo();
        a.key = "dab_notch";
        a.name = "DAB notch";
        a.description = "Notch for DAB (165-230 MHz).";
        a.type = SoapySDR::ArgInfo::BOOL;
        a.value = "false";
        list.push_back(a);
    }

    if (flavour == MIRISDR_HW_RSPDXR2)
    {
        a = SoapySDR::ArgInfo();
        a.key = "direct_sampling";
        a.name = "Direct sampling";
        a.description = "At 2 MHz and below, sample the antenna directly instead of through the tuner. "
                        "The gain is then its LNA alone, about 37 dB.";
        a.type = SoapySDR::ArgInfo::BOOL;
        a.value = "false";
        list.push_back(a);
    }

    return list;
}

void SoapyMiriSDR::writeSetting(const std::string &key, const std::string &value)
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;
    int n;

    if (key == "if_freq")
    {
        if (!parseIf(value, ifMode))
        {
            SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: no IF %s", value.c_str());
            return;
        }
        configure(wantRate);
    }
    else if (key == "converters")
    {
        converters = value == "I" ? MIRISDR_IQ_ONLY_I : value == "Q" ? MIRISDR_IQ_ONLY_Q : MIRISDR_IQ_BOTH;
        configure(wantRate);
    }
    else if (key == "baseband")
    {
        baseband = isTrue(value);
        configure(wantRate);
    }
    else if (key == "format" || key == "transfer" || key == "decimation_bypass")
    {
        std::string &s = key == "format" ? formatSetting : key == "transfer" ? transferSetting : bypassSetting;
        std::string was = s;

        s = (value == "AUTO" && key != "transfer") ? "" : value;
        applyStream(streamCfg);

        /* refused: keep what runs */
        mirisdr_stream_config_t now;
        mirisdr_get_stream(dev, &now, NULL);
        if (key == "format" && !s.empty() && (!now.format || s != now.format)) s = was;
        if (key == "transfer" && !s.empty() && (!now.transfer || s != now.transfer)) s = was;
        if (key == "decimation_bypass" && !s.empty() && (!now.decimation_bypass || s != now.decimation_bypass)) s = was;

        /* the rates a format and transfer reach differ */
        probeRanges();
        configure(wantRate);
    }
    else if (key == "gap_fill")
    {
        mirisdr_stream_config_t c = streamCfg;
        c.gap_fill = isTrue(value);
        applyStream(c);
    }
    else if (key == "biastee")
    {
        if (mirisdr_set_bias(dev, isTrue(value)) < 0)
            SoapySDR_log(SOAPY_SDR_ERROR, "mirisdr: bias-T refused");
    }
    else if (key == "direct_sampling")
    {
        if (mirisdr_set_direct_sampling(dev, isTrue(value) ? 1 : 0) < 0)
            SoapySDR_log(SOAPY_SDR_ERROR, "mirisdr: direct sampling refused");
    }
    else if (key == "fm_notch" || key == "mw_notch" || key == "dab_notch")
    {
        int bit = key == "fm_notch" ? MIRISDR_NOTCH_FM : key == "mw_notch" ? MIRISDR_NOTCH_MW : MIRISDR_NOTCH_DAB;

        n = mirisdr_get_notch(dev);
        if (n < 0) n = 0;
        n = isTrue(value) ? (n | bit) : (n & ~bit);
        if (mirisdr_set_notch(dev, n) < 0)
            SoapySDR_log(SOAPY_SDR_ERROR, "mirisdr: notch refused");
    }
    else SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: no setting %s", key.c_str());
}

std::string SoapyMiriSDR::readSetting(const std::string &key) const
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_stream_config_t c;
    int n;

    if (key == "if_freq") return ifMode;
    if (key == "converters") return converters == MIRISDR_IQ_ONLY_I ? "I" : converters == MIRISDR_IQ_ONLY_Q ? "Q" : "both";
    if (key == "baseband") return baseband ? "true" : "false";
    if (key == "format") return formatSetting.empty() ? "AUTO" : formatSetting;
    if (key == "transfer")
    {
        mirisdr_get_stream(dev, &c, NULL);
        return c.transfer ? c.transfer : "";
    }
    if (key == "decimation_bypass") return bypassSetting.empty() ? "AUTO" : bypassSetting;
    if (key == "gap_fill")
    {
        mirisdr_stream_result_t r;
        mirisdr_get_stream(dev, NULL, &r);
        return r.gap_fill ? "true" : "false";
    }
    if (key == "biastee") return mirisdr_get_bias(dev) > 0 ? "true" : "false";
    if (key == "direct_sampling") return mirisdr_get_direct_sampling(dev) > 0 ? "true" : "false";
    if (key == "fm_notch" || key == "mw_notch" || key == "dab_notch")
    {
        n = mirisdr_get_notch(dev);
        if (n < 0) return "false";
        return (n & (key == "fm_notch" ? MIRISDR_NOTCH_FM : key == "mw_notch" ? MIRISDR_NOTCH_MW :
                     MIRISDR_NOTCH_DAB)) ? "true" : "false";
    }
    return "";
}

/*******************************************************************
 * Time: the stream's sample count, from when it started
 ******************************************************************/

static long long spanNs(uint64_t n, uint32_t rate)
{
    if (!rate) return 0;
    return (long long) ((n / rate) * 1000000000ULL + ((n % rate) * 1000000000ULL) / rate);
}

/* From the last rate change on at its rate, before it at the rates before */
long long SoapyMiriSDR::indexToNs(uint64_t index, uint32_t rate) const
{
    uint64_t from = (uint64_t) timeBaseIndex.load();

    return timeOffsetNs + timeBaseNs + spanNs(index > from ? index - from : 0, rate);
}

/* the rate changes at index: the time so far is kept, at the rate it ran at */
void SoapyMiriSDR::timeRebase(uint64_t index, uint32_t rate)
{
    uint64_t from = (uint64_t) timeBaseIndex.load();

    if (rate == timeBaseRate) return;
    if (timeBaseRate) timeBaseNs += spanNs(index > from ? index - from : 0, timeBaseRate);
    timeBaseIndex = (long long) index;
    timeBaseRate = rate;
}

bool SoapyMiriSDR::hasHardwareTime(const std::string &what) const
{
    return what.empty();
}

long long SoapyMiriSDR::getHardwareTime(const std::string &what) const
{
    return indexToNs(lastIndex, lastRate);
}

void SoapyMiriSDR::setHardwareTime(const long long timeNs, const std::string &what)
{
    timeOffsetNs += timeNs - getHardwareTime(what);
}
