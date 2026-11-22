/*
 * SoapySDR module for libmirisdr: finding and making devices.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 */

#include "SoapyMiriSDR.hpp"

#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>

#include <algorithm>
#include <cctype>
#include <cstdlib>

/* The build sets both, see SOAPY_PROBE_SERIAL and SOAPY_ROM_ON_CLOSE in CMakeLists.txt. These
   are the same defaults for a build that does not */
#ifndef MIRISDR_SOAPY_PROBE_SERIAL
#define MIRISDR_SOAPY_PROBE_SERIAL 1
#endif
#ifndef MIRISDR_SOAPY_ROM_ON_CLOSE
#define MIRISDR_SOAPY_ROM_ON_CLOSE 0
#endif

/* a switch from the device string's key, else the environment's variable if it has one, else
   what the module was built with */
static bool argSwitch(const SoapySDR::Kwargs &args, const char *key, const char *var, bool def)
{
    const char *env = var ? std::getenv(var) : nullptr;
    std::string v = args.count(key) ? args.at(key) : (env ? env : "");

    if (v.empty()) return def;
    std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return (char) std::tolower(c); });
    return v == "true" || v == "1" || v == "on" || v == "yes";
}

bool soapyMiriSDRProbeSerial(const SoapySDR::Kwargs &args)
{
    return argSwitch(args, "probe_serial", "MIRISDR_PROBE_SERIAL", MIRISDR_SOAPY_PROBE_SERIAL != 0);
}

bool soapyMiriSDRRomOnClose(const SoapySDR::Kwargs &args)
{
    return argSwitch(args, "rom_on_close", nullptr, MIRISDR_SOAPY_ROM_ON_CLOSE != 0);
}

static std::vector<SoapySDR::Kwargs> findMiriSDR(const SoapySDR::Kwargs &args)
{
    std::vector<SoapySDR::Kwargs> results;
    mirisdr_list_config_t cfg;
    /* one look at the USB, by the library's rules, which the open matches again: with a serial or
       port only the receiver they name. The serial is the system's copy, so a receiver in use
       elsewhere is not disturbed. Only with the probe on is one that runs from ROM opened, for
       the serial in its EEPROM */
    mirisdr_list_config_default(&cfg);
    const char *want = cfg.serial = args.count("serial") ? args.at("serial").c_str() : nullptr;
    const char *at = cfg.usb_port = args.count("usb_port") ? args.at("usb_port").c_str() : nullptr;
    if (soapyMiriSDRProbeSerial(args)) cfg.flags |= MIRISDR_LIST_PROBE_SERIAL;

    /* in one call where they fit: a call to count them and one to fill them in would read each
       EEPROM twice */
    std::vector<mirisdr_receiver_t> rx(16);
    int n = mirisdr_list_receivers_ex(&cfg, rx.data(), (int) rx.size());

    if (n > (int) rx.size())
    {
        rx.resize((size_t) n);
        n = mirisdr_list_receivers_ex(&cfg, rx.data(), n);
    }
    for (int i = 0; i < n && i < (int) rx.size(); i++)
    {
        const mirisdr_receiver_t &r = rx[(size_t) i];
        SoapySDR::Kwargs dev;

        dev["driver"] = SOAPY_MIRISDR_DRIVER;
        /* asked for by serial or port, those: the device is opened by them */
        if (want || at)
        {
            if (want) dev["serial"] = want;
            if (at) dev["usb_port"] = at;
        }
        else
        {
            /* no serial until it declares one, as from ROM it does not, unless the probe read it */
            if (*r.serial) dev["serial"] = r.serial;
            dev["usb_port"] = r.usb_port;
        }
        dev["manufacturer"] = r.manufacturer ? r.manufacturer : "";
        dev["product"] = r.product ? r.product : "";
        dev["label"] = dev["product"] + " :: " + (*r.serial ? r.serial : r.usb_port);
        results.push_back(dev);
    }

    return results;
}

static SoapySDR::Device *makeMiriSDR(const SoapySDR::Kwargs &args)
{
    return new SoapyMiriSDR(args);
}

static SoapySDR::Registry registerMiriSDR(SOAPY_MIRISDR_DRIVER, &findMiriSDR, &makeMiriSDR, SOAPY_SDR_ABI_VERSION);
