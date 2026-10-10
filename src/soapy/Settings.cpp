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
#include <cmath>
#include <cstdio>
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

SoapyMiriSDR::SoapyMiriSDR(const SoapySDR::Kwargs &args):
    dev(nullptr), flavour(MIRISDR_HW_DEFAULT), ifFreq(0), ifMode("auto"), converters(MIRISDR_IQ_BOTH),
    baseband(true), wantRate(2048000), wantBw(0), argBw(0),
    freqMin(0), freqMax(0), rateMin(0), rateMax(0),
    stopping(false), rxDone(true), rxResult(0), outFormat(0),
    ringHead(0), ringTail(0), ringCount(0), dropPending(false), slotPos(0), gapPos(0),
    timeOffsetNs(0), lastIndex(0), lastRate(0), asyncBuffers(0)
{
    mirisdr_open_config_t cfg;
    mirisdr_tune_config_t tc;
    char manufact[256], product[256], ser[256];   /* the library fills 256 each */

    mirisdr_open_config_default(&cfg);
    if (args.count("serial")) cfg.serial = args.at("serial").c_str();

    if (mirisdr_open_ex(&dev, &cfg) < 0 || !dev)
        throw std::runtime_error("mirisdr: could not open the device");

    flavour = mirisdr_get_hw_flavour(dev);
    if (mirisdr_get_usb_strings(dev, manufact, product, ser) == 0) serial = ser;

    /* the IF settings may come with the device, for programs that only take a string */
    if (args.count("if_freq")) ifMode = args.at("if_freq");
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

    SoapySDR_logf(SOAPY_SDR_INFO, "mirisdr: opened %s", serial.c_str());
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
 * baseband, and below it takes the low IF rate nearest to the one asked.
 * A fixed low IF has its own rates: the converters at 4 x IF, divided
 * by a power of 2, by 2 at least for a single converter
 ******************************************************************/

std::vector<uint32_t> SoapyMiriSDR::lowIfRates(uint32_t ifHz) const
{
    std::vector<uint32_t> rates;
    int one = converters != MIRISDR_IQ_BOTH, k;
    uint64_t base = (uint64_t) 4 * ifHz / (one ? 2 : 1);

    for (k = 0; k <= 8 - one && !(base % ((uint64_t) 1 << k)); k++) rates.push_back((uint32_t) (base >> k));
    return rates;
}

bool SoapyMiriSDR::realOut(void) const
{
    return !baseband && ifFreq && converters != MIRISDR_IQ_BOTH;
}

static double ratio(double a, double b)
{
    return a > b ? a / b : b / a;
}

void SoapyMiriSDR::configure(uint32_t rate)
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;
    mirisdr_stream_config_t c = streamCfg;
    uint32_t ifHz = 0, out = rate, pick = 0;
    double best = 0;

    if (ifMode != "auto") ifHz = (uint32_t) std::stoul(ifMode);

    if (baseband && ifMode == "auto" && rate < rateMin)
    {
        /* below the converters own minimum only a low IF gets there */
        for (uint32_t f : IFS)
            for (uint32_t r : lowIfRates(f))
                if (f && r < rateMin && (!pick || ratio(r, rate) < best)) { pick = r; best = ratio(r, rate); ifHz = f; }
        out = pick;
    }
    else if (baseband && ifHz)
    {
        for (uint32_t r : lowIfRates(ifHz))
            if (!pick || ratio(r, rate) < best) { pick = r; best = ratio(r, rate); }
        out = pick;
    }

    /* the tune: the IF, one converter or both, the filter nearest to what was asked */
    mirisdr_get_tune(dev, 0, &tc, NULL);
    tc.if_freq = ifHz;
    tc.iq = ifHz ? converters : MIRISDR_IQ_BOTH;
    tc.low_if_auto = 0;
    tc.lo_offset = 0;
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

    info["serial"] = serial;
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

std::vector<std::string> SoapyMiriSDR::listAntennas(const int direction, const size_t channel) const
{
    return { "RX" };
}

void SoapyMiriSDR::setAntenna(const int direction, const size_t channel, const std::string &name)
{
    if (name != "RX") SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: no antenna %s", name.c_str());
}

std::string SoapyMiriSDR::getAntenna(const int direction, const size_t channel) const
{
    return "RX";
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
    sr.baseband = v;

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

SoapySDR::Range SoapyMiriSDR::getGainRange(const int direction, const size_t channel) const
{
    StageRanges sr = stageRanges();

    return SoapySDR::Range(0, sr.lna + sr.mixer + sr.mixbuffer.back() + sr.baseband, 1);
}

SoapySDR::Range SoapyMiriSDR::getGainRange(const int direction, const size_t channel, const std::string &name) const
{
    StageRanges sr = stageRanges();

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
    tc.lo_offset = 0;
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

/* the low IF rates a setting allows: below rateMin with "auto", all of a fixed IF's */
static void addLowIf(std::vector<double> &v, const std::vector<uint32_t> &rates, double below)
{
    for (uint32_t r : rates)
        if (r < below && r >= 25000 && std::find(v.begin(), v.end(), (double) r) == v.end()) v.push_back(r);
}

std::vector<double> SoapyMiriSDR::listSampleRates(const int direction, const size_t channel) const
{
    static const double common[] = { 1.536e6, 2e6, 2.048e6, 2.4e6, 3e6, 4e6, 5e6, 6e6, 7e6,
                                     8e6, 9e6, 10e6, 12e6, 14e6 };
    std::vector<double> rates;
    uint32_t fixed = ifMode == "auto" ? 0 : (uint32_t) std::stoul(ifMode);

    if (baseband && fixed) addLowIf(rates, lowIfRates(fixed), 1e12);
    else
    {
        if (baseband)
            for (uint32_t f : IFS) if (f) addLowIf(rates, lowIfRates(f), rateMin);
        for (double r : common)
            if (r >= rateMin && r <= rateMax) rates.push_back(r);
    }
    std::sort(rates.begin(), rates.end());
    return rates;
}

SoapySDR::RangeList SoapyMiriSDR::getSampleRateRange(const int direction, const size_t channel) const
{
    SoapySDR::RangeList out;
    std::vector<double> low;
    uint32_t fixed = ifMode == "auto" ? 0 : (uint32_t) std::stoul(ifMode);

    if (baseband && fixed) addLowIf(low, lowIfRates(fixed), 1e12);
    else if (baseband)
        for (uint32_t f : IFS) if (f) addLowIf(low, lowIfRates(f), rateMin);
    std::sort(low.begin(), low.end());
    for (double r : low) out.push_back(SoapySDR::Range(r, r));
    if (!(baseband && fixed)) out.push_back(SoapySDR::Range(rateMin, rateMax));
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
    a.description = "Put zeros where samples were lost on the USB, so the timing stays right.";
    a.type = SoapySDR::ArgInfo::BOOL;
    a.value = "false";
    list.push_back(a);

    if (flavour == MIRISDR_HW_RSP1B)
    {
        a = SoapySDR::ArgInfo();
        a.key = "biastee";
        a.name = "Bias-T";
        a.description = "Power on the antenna input.";
        a.type = SoapySDR::ArgInfo::BOOL;
        a.value = "false";
        list.push_back(a);

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

    return list;
}

void SoapyMiriSDR::writeSetting(const std::string &key, const std::string &value)
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_tune_config_t tc;
    int n;

    if (key == "if_freq")
    {
        if (value != "auto" && std::find(std::begin(IFS), std::end(IFS), (uint32_t) std::strtoul(value.c_str(), NULL, 10))
                               == std::end(IFS))
        {
            SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: no IF %s", value.c_str());
            return;
        }
        ifMode = value;
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
    if (key == "gap_fill") return streamCfg.gap_fill ? "true" : "false";
    if (key == "biastee") return mirisdr_get_bias(dev) > 0 ? "true" : "false";
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

long long SoapyMiriSDR::indexToNs(uint64_t index, uint32_t rate) const
{
    if (!rate) return timeOffsetNs;
    return timeOffsetNs + (long long) ((index / rate) * 1000000000ULL +
                                       ((index % rate) * 1000000000ULL) / rate);
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
