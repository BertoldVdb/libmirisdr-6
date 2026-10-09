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
    uint32_t i, n = mirisdr_get_device_count();

    for (i = 0; i < n; i++)
    {
        char manufact[256] = "", product[256] = "", serial[256] = "";
        SoapySDR::Kwargs dev;

        if (mirisdr_get_device_usb_strings(i, manufact, product, serial) != 0) continue;

        if (args.count("serial") && args.at("serial") != serial) continue;

        dev["driver"] = SOAPY_MIRISDR_DRIVER;
        dev["serial"] = serial;
        dev["manufacturer"] = manufact;
        dev["product"] = product;
        dev["label"] = std::string(product) + " :: " + serial;
        results.push_back(dev);
    }

    return results;
}

static SoapySDR::Device *makeMiriSDR(const SoapySDR::Kwargs &args)
{
    return new SoapyMiriSDR(args);
}

static SoapySDR::Registry registerMiriSDR(SOAPY_MIRISDR_DRIVER, &findMiriSDR, &makeMiriSDR, SOAPY_SDR_ABI_VERSION);
