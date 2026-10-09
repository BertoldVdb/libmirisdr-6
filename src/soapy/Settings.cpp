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

/* every filter the tuner has; which ones an IF takes is asked from the library */
static const uint32_t BANDWIDTHS[] = { 200000, 300000, 600000, 1536000, 5000000, 6000000,
                                       7000000, 8000000, 14000000 };

static bool isTrue(const std::string &v)
{
    return v == "true" || v == "1" || v == "on" || v == "yes";
}

SoapyMiriSDR::SoapyMiriSDR(const SoapySDR::Kwargs &args):
    dev(nullptr), flavour(MIRISDR_HW_DEFAULT), ifFreq(0),
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

    /* the library's defaults, the widest filter and a known gain */
    mirisdr_get_stream(dev, &streamCfg, NULL);
    streamCfg.rate = 2048000;
    applyStream(streamCfg);

    mirisdr_get_tune(dev, 0, &tc, NULL);
    tc.frequency = 100000000;
    tc.if_freq = 0;
    tc.bandwidth = 0;
    tc.gain.mode = MIRISDR_GAIN_TOTAL;
    tc.gain.total = 40;
    applyTune(tc);

    probeRanges();

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
    c.follow_tune = 0;

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

    if (name != "RF" || mirisdr_get_tune(dev, 0, NULL, &r) < 0) return 0;
    return r.lo;
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
    mirisdr_stream_config_t c = streamCfg;

    c.rate = (uint32_t) std::llround(rate);
    applyStream(c);
}

double SoapyMiriSDR::getSampleRate(const int direction, const size_t channel) const
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    mirisdr_stream_result_t r;

    if (mirisdr_get_stream(dev, NULL, &r) < 0) return 0;
    return r.rate;
}

std::vector<double> SoapyMiriSDR::listSampleRates(const int direction, const size_t channel) const
{
    static const double common[] = { 1e6, 1.536e6, 2e6, 2.048e6, 2.4e6, 3e6, 4e6, 5e6, 6e6, 7e6,
                                     8e6, 9e6, 10e6, 12e6, 14e6 };
    std::vector<double> rates;

    for (double r : common)
        if (r >= rateMin && r <= rateMax) rates.push_back(r);
    return rates;
}

SoapySDR::RangeList SoapyMiriSDR::getSampleRateRange(const int direction, const size_t channel) const
{
    return { SoapySDR::Range(rateMin, rateMax) };
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

/* the narrowest filter that passes bw, else the widest */
void SoapyMiriSDR::setBandwidth(const int direction, const size_t channel, const double bw)
{
    std::lock_guard<std::recursive_mutex> lock(devMutex);
    std::vector<uint32_t> bws = bandwidthsFor(ifFreq);
    mirisdr_tune_config_t tc;
    uint32_t pick = 0;

    for (uint32_t b : bws)
        if (bw > 0 && b >= bw) { pick = b; break; }

    mirisdr_get_tune(dev, 0, &tc, NULL);
    tc.bandwidth = pick;
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
    a.description = "Tuner IF in Hz. With a low IF the band received lies this far below the centre.";
    a.type = SoapySDR::ArgInfo::STRING;
    a.value = "0";
    for (uint32_t f : IFS) a.options.push_back(std::to_string(f));
    a.optionNames = { "Zero IF", "450 kHz", "1620 kHz", "2048 kHz" };
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
        a.name = "Broadcast notch";
        a.description = "Notch for the FM band (85-100 MHz) and MW (0.4-1.6 MHz).";
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
        uint32_t f = (uint32_t) std::stoul(value);

        mirisdr_get_tune(dev, 0, &tc, NULL);
        tc.if_freq = f;
        tc.bandwidth = 0;
        tc.gain.mode = MIRISDR_GAIN_KEEP;
        if (mirisdr_tune(dev, 0, &tc, NULL) < 0)
            SoapySDR_logf(SOAPY_SDR_ERROR, "mirisdr: IF %s refused", value.c_str());
        else ifFreq = f;
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
    else if (key == "fm_notch" || key == "dab_notch")
    {
        int bit = key == "fm_notch" ? MIRISDR_NOTCH_FM : MIRISDR_NOTCH_DAB;

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

    if (key == "if_freq") return std::to_string(ifFreq);
    if (key == "format") return formatSetting.empty() ? "AUTO" : formatSetting;
    if (key == "transfer")
    {
        mirisdr_get_stream(dev, &c, NULL);
        return c.transfer ? c.transfer : "";
    }
    if (key == "decimation_bypass") return bypassSetting.empty() ? "AUTO" : bypassSetting;
    if (key == "gap_fill") return streamCfg.gap_fill ? "true" : "false";
    if (key == "biastee") return mirisdr_get_bias(dev) > 0 ? "true" : "false";
    if (key == "fm_notch" || key == "dab_notch")
    {
        n = mirisdr_get_notch(dev);
        if (n < 0) return "false";
        return (n & (key == "fm_notch" ? MIRISDR_NOTCH_FM : MIRISDR_NOTCH_DAB)) ? "true" : "false";
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
