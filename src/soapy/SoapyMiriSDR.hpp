/*
 * SoapySDR module for libmirisdr: MSi2500/MSi001 receivers and the SDRplay RSP1B.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 */

#pragma once

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Types.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <mirisdr.h>

#define SOAPY_MIRISDR_DRIVER    "mirisdr"
#define SOAPY_MIRISDR_SLOTS     256     /* callback buffers the ring holds */

class SoapyMiriSDR : public SoapySDR::Device
{
public:
    explicit SoapyMiriSDR(const SoapySDR::Kwargs &args);
    ~SoapyMiriSDR(void);

    /* identification */
    std::string getDriverKey(void) const;
    std::string getHardwareKey(void) const;
    SoapySDR::Kwargs getHardwareInfo(void) const;

    /* channels */
    size_t getNumChannels(const int direction) const;

    /* stream */
    std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const;
    std::string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const;
    SoapySDR::ArgInfoList getStreamArgsInfo(const int direction, const size_t channel) const;
    SoapySDR::Stream *setupStream(const int direction, const std::string &format,
                                  const std::vector<size_t> &channels, const SoapySDR::Kwargs &args);
    void closeStream(SoapySDR::Stream *stream);
    size_t getStreamMTU(SoapySDR::Stream *stream) const;
    int activateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs, const size_t numElems);
    int deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs);
    int readStream(SoapySDR::Stream *stream, void * const *buffs, const size_t numElems,
                   int &flags, long long &timeNs, const long timeoutUs);

    /* antenna */
    std::vector<std::string> listAntennas(const int direction, const size_t channel) const;
    void setAntenna(const int direction, const size_t channel, const std::string &name);
    std::string getAntenna(const int direction, const size_t channel) const;

    /* gain */
    std::vector<std::string> listGains(const int direction, const size_t channel) const;
    void setGain(const int direction, const size_t channel, const double value);
    void setGain(const int direction, const size_t channel, const std::string &name, const double value);
    double getGain(const int direction, const size_t channel) const;
    double getGain(const int direction, const size_t channel, const std::string &name) const;
    SoapySDR::Range getGainRange(const int direction, const size_t channel) const;
    SoapySDR::Range getGainRange(const int direction, const size_t channel, const std::string &name) const;

    /* frequency */
    void setFrequency(const int direction, const size_t channel, const double frequency, const SoapySDR::Kwargs &args);
    void setFrequency(const int direction, const size_t channel, const std::string &name,
                      const double frequency, const SoapySDR::Kwargs &args);
    double getFrequency(const int direction, const size_t channel) const;
    double getFrequency(const int direction, const size_t channel, const std::string &name) const;
    std::vector<std::string> listFrequencies(const int direction, const size_t channel) const;
    SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel) const;
    SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel, const std::string &name) const;

    /* sample rate and bandwidth */
    void setSampleRate(const int direction, const size_t channel, const double rate);
    double getSampleRate(const int direction, const size_t channel) const;
    std::vector<double> listSampleRates(const int direction, const size_t channel) const;
    SoapySDR::RangeList getSampleRateRange(const int direction, const size_t channel) const;
    void setBandwidth(const int direction, const size_t channel, const double bw);
    double getBandwidth(const int direction, const size_t channel) const;
    std::vector<double> listBandwidths(const int direction, const size_t channel) const;
    SoapySDR::RangeList getBandwidthRange(const int direction, const size_t channel) const;

    /* time */
    bool hasHardwareTime(const std::string &what) const;
    long long getHardwareTime(const std::string &what) const;
    void setHardwareTime(const long long timeNs, const std::string &what);

    /* settings */
    SoapySDR::ArgInfoList getSettingInfo(void) const;
    void writeSetting(const std::string &key, const std::string &value);
    std::string readSetting(const std::string &key) const;

    /* what a slot's samples are */
    enum Kind { K_CS16, K_CS8, K_CF32, K_S16 };

    /* called from the library's stream thread */
    void rxCallback(unsigned char *buf, uint32_t len);

private:
    struct Gap
    {
        uint32_t offset;    /* samples of the slot before it */
        uint64_t missing;   /* samples the index skips here */
        bool     lost;      /* not filled in: report an overflow */
    };

    struct Slot
    {
        std::vector<uint8_t> data;
        size_t   n;         /* samples */
        int      kind;      /* Kind */
        int      bytes;     /* a sample takes */
        uint64_t index;     /* of the first sample */
        uint32_t rate;
        bool     dropped;   /* the ring was full before this one */
        std::vector<Gap> gaps;
    };

    /* the gain each stage can take in the band of a tune */
    struct StageRanges
    {
        int lna, mixer, baseband;
        std::vector<int> mixbuffer;
    };

    mirisdr_dev_t *dev;
    mutable std::recursive_mutex devMutex;
    int flavour;
    std::string serial;

    /* settings, applied with each stream or tune */
    mirisdr_stream_config_t streamCfg;
    std::string formatSetting, transferSetting, bypassSetting;
    uint32_t ifFreq;        /* the IF in force */
    std::string ifMode;     /* "auto": zero IF from rateMin up, a low IF below; or a fixed one */
    int converters;         /* MIRISDR_IQ_BOTH, _ONLY_I or _ONLY_Q */
    bool baseband;          /* the band at 0 Hz in complex float, whatever the IF */
    uint32_t wantRate, wantBw;  /* as asked, wantBw 0 for the library's choice */
    uint32_t argBw;             /* the device string's bandwidth, what 0 falls back to */
    bool bandGainRanges;    /* gain ranges of the band tuned, not the most of any band */

    /* ranges found at open */
    double freqMin, freqMax, rateMin, rateMax;

    /* stream */
    std::thread rxThread;
    std::atomic<bool> stopping, rxDone;
    std::atomic<int> rxResult;
    int outFormat;          /* 0 CS16, 1 CF32, 2 CS8 */
    std::mutex ringMutex;
    std::condition_variable ringCond;
    std::vector<Slot> ring;
    size_t ringHead, ringTail, ringCount;
    bool dropPending;
    size_t slotPos, gapPos;
    long long timeOffsetNs;
    std::atomic<long long> lastIndex;
    std::atomic<uint32_t> lastRate;
    uint32_t asyncBuffers;

    void applyStream(const mirisdr_stream_config_t &cfg);
    void configure(uint32_t rate);
    bool realOut(void) const;
    std::vector<uint32_t> lowIfRates(uint32_t ifHz) const;
    void applyTune(const mirisdr_tune_config_t &cfg);
    StageRanges stageRanges(void) const;
    std::vector<uint32_t> bandwidthsFor(uint32_t ifHz) const;
    uint32_t pickBandwidth(uint32_t ifHz) const;
    void probeRanges(void);
    long long indexToNs(uint64_t index, uint32_t rate) const;
};
