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

  - One receive channel, with samples as CS16, CF32 or CS8. A full scale CS16 sample is 32768.
  - Sample rates from 1.3 Msps. The highest rate depends on the USB transfer and sample format. It is 12 Msps in bulk mode with the automatic format.
  - The frequency is the tuner's LO, which is the centre of the stream.
  - Gain as one total, which the library splits over the stages the way it works best. The stages can also be set one by one: `LNA`, `MIX`, `MIXBUF` (only on the AM inputs) and `BB`. Their ranges follow the band you are tuned to.
  - Bandwidths are the tuner's filters. A request picks the narrowest filter that is at least as wide.
  - Time stamps on every read, counted in samples from the start of the stream.
  - Samples lost on the USB are replaced with zeros when `baseband` or `gap_fill` is on, which keeps the timing of the stream. This covers gaps of up to 16 USB packets, which is 2 ms at 2 Msps and less at higher rates. A longer gap, or any gap with both settings off, is reported as an overflow at the place where samples are missing. The next read's time stamp shows how many. An overflow is also reported when the program reads too slowly and the module's buffers run full.

<h3>Settings</h3>

  - `if_freq`: the tuner IF. 0 is zero IF. With 450000, 1620000 or 2048000 the band received lies that far below the centre of the stream. A low IF also allows narrower filters.
  - `format`: how samples cross the USB. AUTO picks the most bits that fit.
  - `transfer`: BULK, ISOC, ISOC1 or ISOC2.
  - `decimation_bypass`: AUTO, ON or OFF.
  - `gap_fill`: put zeros where samples were lost, so the timing of the stream stays right.
  - On the RSP1B: `biastee`, `fm_notch` (FM and MW) and `dab_notch`.

A setting the receiver cannot do is refused and logged, and the receiver keeps what it had.

<h3>Not supported yet</h3>

  - Automatic gain and frequency correction.
  - Real sample formats.
  - Time from a PPS input.
