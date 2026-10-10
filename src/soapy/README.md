SoapySDR module
===============

This module lets SoapySDR programs use libmirisdr receivers, such as SDR++, CubicSDR, GQRX and GNU Radio. The driver is called `mirisdr`.

<h3>Building and installing</h3>

The module is built with the library when the SoapySDR development files are installed (on Debian, `libsoapysdr-dev`). Turn it off with `cmake -DENABLE_SOAPY=OFF`. `make install` puts it in SoapySDR's module folder, for example `/usr/local/lib/SoapySDR/modules0.7`.

Check that SoapySDR finds the receiver:

    SoapySDRUtil --find="driver=mirisdr"
    SoapySDRUtil --probe="driver=mirisdr"

A module only loads in the SoapySDR version it was built for, which is the version in the folder name. Examples are `modules0.7`, `modules0.8` for the 0.8.1 release and `modules0.8-3` for the development version. After an upgrade, build the module again. The same code builds and works with 0.7.2 and with the development version of October 2026.

Other modules also drive these receivers. These are the Debian package `soapysdr0.7-module-mirisdr` (driver `miri`) and SoapyMiri (driver `soapyMiri`). A program then lists the receiver once for each module. Pick this one with `driver=mirisdr`, or remove the others.

<h3>What it offers</h3>

  - One receive channel, with samples as CF32 (the native format, full scale 1.0), CS16 or CS8.
  - The frequency you set is the centre of the stream, at every IF.
  - Sample rates from 1.3 Msps at zero IF. The highest rate depends on the USB transfer. Isochronous mode reaches 12 Msps. Bulk mode reaches about 27 Msps, with 8-bit samples above 15.75 Msps, but the computer then has to keep up with up to 55 MB/s. If it cannot, samples go missing, and a lower rate or isochronous mode is the answer. Above 14.5 Msps the receiver's own filter is bypassed, so the program has to filter.
  - Lower rates, down to about 28 ksps, through a low IF. The receiver tunes a low IF and the library shifts the band back to the centre in software, then filters it down to the rate. The rates come from a list: 900, 450, 225 ksps and lower from the 450 kHz IF, 810, 405 ksps and lower from 1620 kHz, and 1024, 512, 256 ksps and lower from 2048 kHz. A rate that is not on the list gives the nearest one.
  - Gain as one total, which the library splits over the stages the way it works best. The stages can also be set one by one: `LNA`, `MIX`, `MIXBUF` (only on the AM inputs) and `BB`. Their ranges follow the band you are tuned to.
  - Bandwidths are the tuner's filters. A request picks the narrowest filter that is at least as wide. With none asked for, zero IF takes the narrowest filter as wide as the sample rate, up to 8 MHz. For programs without a bandwidth field, such as GQRX, the device string takes `bandwidth=` in Hz.
  - Time stamps on every read, counted in samples from the start of the stream.
  - Samples lost on the USB are replaced with zeros when `baseband` or `gap_fill` is on, which keeps the timing of the stream. This covers gaps of up to 16 USB packets, which is 2 ms at 2 Msps and less at higher rates. A longer gap, or any gap with both settings off, is reported as an overflow at the place where samples are missing. The next read's time stamp shows how many. An overflow is also reported when the program reads too slowly and the module's buffers run full.

<h3>Settings</h3>

  - `if_freq`: the tuner IF. `auto`, the default, uses zero IF from 1.3 Msps up and a low IF below that. 0 is always zero IF. 450000, 1620000 or 2048000 always use that IF, and then the sample rates come from its list, for example 8.192, 4.096, 2.048 and 1.024 Msps with 2048000. A low IF also has the narrower filters (200, 300 and 600 kHz with 450000, 600 kHz and 1.536 MHz with the others).
  - `converters`: with a low IF, `both` (the default) or a single one, `I` or `Q`. One converter halves the USB data and keeps 14 bits at the highest rates, but needs more CPU. Both get the same image rejection, the tuner's own, about 37 dB.
  - `baseband`: on by default. Off gives the stream as it comes from the receiver, with the tuner's LO at the centre, as in older versions. With a low IF and a single converter the samples are then real, as S16 or F32.
  - `format`: how samples cross the USB. AUTO picks the most bits that fit.
  - `transfer`: BULK, ISOC, ISOC1 or ISOC2.
  - `decimation_bypass`: AUTO, ON or OFF.
  - `gap_fill`: put zeros where samples were lost, so the timing of the stream stays right. It covers gaps of up to 16 USB packets. `baseband` always does this, because its filters need the timing.
  - On the RSP1B: `biastee`, `fm_notch`, `mw_notch` and `dab_notch`. The FM and MW notches are one switch on this board, so either turns both on.

A setting the receiver cannot do is refused and logged, and the receiver keeps what it had.

`if_freq`, `converters` and `baseband` can also be given when the device is opened, for programs that only take a device string, for example `driver=mirisdr,if_freq=2048000`.

<h3>CPU use</h3>

Measured on a Raspberry Pi Compute Module 0 (one core at 1 GHz), the low IF processing runs at 3 times real time or better with the 2048 kHz IF, and 9 times with both converters at the full 8.192 Msps. The 450 kHz IF needs much less. Zero IF needs no processing.

<h3>Not supported yet</h3>

  - Automatic gain and frequency correction.
  - Time from a PPS input.
