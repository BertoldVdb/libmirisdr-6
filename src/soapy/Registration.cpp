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

static std::vector<SoapySDR::Kwargs> findMiriSDR(const SoapySDR::Kwargs &args)
{
    std::vector<SoapySDR::Kwargs> results;
    const char *want = args.count("serial") ? args.at("serial").c_str() : nullptr;
    const char *at = args.count("usb_port") ? args.at("usb_port").c_str() : nullptr;
    /* one look at the USB, by the library's rules, which the open matches again: with a serial or
       port only the receiver they name. The serial is the system's copy, so a receiver in use
       elsewhere is not disturbed */
    int n = mirisdr_list_receivers(want, at, nullptr, 0);
    std::vector<mirisdr_receiver_t> rx(n > 0 ? (size_t) n : 0);

    if (n > 0) n = mirisdr_list_receivers(want, at, rx.data(), n);
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
            /* no serial until it declares one, as from ROM it does not */
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
