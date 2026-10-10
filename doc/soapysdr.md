Using libmirisdr receivers with SoapySDR
========================================

The `mirisdr` SoapySDR module lets programs that use SoapySDR work with libmirisdr receivers. Examples are SDR++, CubicSDR, GQRX, GNU Radio and your own programs. It supports boards with the MSi2500 and MSi001 chips, such as the SDRplay RSP1 and RSP1B.

This guide explains how to install the module, how to pick it, and what each setting does.


<h3>Quick start</h3>

Check that SoapySDR finds the receiver:

    SoapySDRUtil --find="driver=mirisdr"
    SoapySDRUtil --probe="driver=mirisdr"

In a program, choose the device with the string `driver=mirisdr`. Then set a frequency, a sample rate and a gain as for any other receiver. The defaults work for most uses. The frequency you set is always at the centre of the samples you get.

To check that your computer keeps up at a rate:

    SoapySDRUtil --args="driver=mirisdr" --rate=8e6 --direction=RX


<h3>Installing</h3>

The module is built with the library when the SoapySDR development files are installed. On Debian and Raspberry Pi OS that is the package `libsoapysdr-dev`. Turn it off with `cmake -DENABLE_SOAPY=OFF`.

`make install` puts it in SoapySDR's module folder, for example `/usr/local/lib/SoapySDR/modules0.7`. A module only loads in the SoapySDR version it was built for, which is the version in the folder name. After a SoapySDR upgrade, build the module again. It builds and works with SoapySDR 0.7.2 and with the development version of October 2026.

Other modules can also drive these receivers. These are the Debian package `soapysdr0.7-module-mirisdr` (driver `miri`) and SoapyMiri (driver `soapyMiri`). If one of them is installed too, programs list the receiver once for each module. Pick this one with `driver=mirisdr`, or remove the others.


<h3>Choosing the device</h3>

The device string can hold these keys:

  - `driver=mirisdr`: this module.
  - `serial=...`: a receiver by its serial, when more than one is plugged in. `SoapySDRUtil --find` lists the serials.
  - `if_freq`, `converters` and `baseband`: the settings of the same name, described below. They are here for programs that only let you type a device string. An example is `driver=mirisdr,if_freq=450000`.
  - `bandwidth`: the tuner filter in Hz, for programs such as GQRX that have no field for it. See Bandwidth below.
  - `gain_ranges`: `max`, the default, or `band`. See Gain below.

When opened, the receiver is at 100 MHz with a total gain of 40 dB and a rate of 2.048 Msps.

<h4>Receivers without firmware in their EEPROM</h4>

Most receivers start from the ROM in the chip. libmirisdr loads its own firmware into them when it opens them. The ROM does not report a serial, and the module does not open/update every receiver just to list it. So until a receiver has been opened once, `SoapySDRUtil --find` shows where it sits on the USB in place of a serial:

    serial = 1:3

That is bus 1, address 3. `serial=1:3` picks the receiver, but the numbers change when you plug it in again or into another port.

After the first open the firmware keeps running until the receiver loses power. From then on the real serial shows up, whichever program opened it.

To have a fixed serial from the moment it is plugged in, write the firmware into the receiver's EEPROM with a serial of its own. The receiver needs an EEPROM for this. Unplug it and plug it in again afterwards, so it starts from the new firmware. With more than one receiver plugged in, pick the one to write with `-d` and its index, for example `miri_eeprom -d 1 writefw ...`.

**Make a backup of the EEPROM first.** Writing the firmware replaces the original data, and without the old contents you may not be able to easily go back to vendor software. Replug the receiver, so it runs from its ROM, and read the EEPROM before any program opens it:

    miri_eeprom saferead backup.bin

Keep `backup.bin` somewhere safe. `saferead` does no do anything with the receiver but dumping the EEPROM. To put the backup back later:

    miri_eeprom write backup.bin

The firmware shows up with libmirisdr-6's own USB ids, 16d0:158c. These ids do not say which board it is, so the library tells from the serial:

  - **RSP1B**: the serial must be `B-` followed by the receiver's own 10 character SDRplay serial.

        miri_eeprom writefw default -s B-1234567890

  - **Other boards**: any serial of up to 12 characters works. `random` makes one up.

        miri_eeprom writefw default -s random

After replugging the USB VID:PID will change, which may require retriggering udev to get access to the device.

<h3>Frequency</h3>

The module reports a range of 1 kHz to 2400 MHz, which is what the library accepts. The frequency you set is at 0 Hz in the samples, at every IF. You never need to add or subtract an IF yourself.

There is one tuning element, called `RF`, and one antenna, called `RX`.


<h3>Sample rates</h3>

How a rate is made depends on whether it is above or below 1.3 Msps.

<h4>From 1.3 Msps up</h4>

The receiver samples directly, at zero IF. Any rate in the range works.

  - The isochronous USB transfer, the default on Linux, reaches 12.096 Msps.
  - The bulk transfer reaches about 27.5 Msps. Above 15.75 Msps the samples cross the USB with 8 bits, and the computer has to take in up to 55 MB/s. If it cannot, samples go missing.
  - Above 14.5 Msps the receiver's own filter is bypassed, so the program gets the full width and has to filter itself.

<h4>Below 1.3 Msps</h4>

The receiver tunes to a low IF, and the library shifts the band back to the centre and filters it down in software. Only certain rates exist. They come from three IFs:

| IF | Rates |
|---|---|
| 450 kHz | 900000, 450000, 225000, 112500, 56250, 28125 |
| 1620 kHz | 810000, 405000, 202500, 101250, 50625 |
| 2048 kHz | 1024000, 512000, 256000, 128000, 64000, 32000 |

The module lists these rates together with the zero IF range. If you ask for a rate that is not on the list, you get the nearest one and the module logs which. For example 250000 gives 256000. Programs that let you type any rate can use `getSampleRate()` to see what they got.

With a fixed IF (see the `if_freq` setting) the rates of that IF are the only ones offered, including the higher ones. The 2048 kHz IF then also offers 8192000, 4096000 and 2048000.


<h3>Bandwidth</h3>

The bandwidths are the tuner's filters. A request picks the narrowest filter that is at least as wide as asked, or the widest if none is. The module remembers what you asked for, so a change of IF takes the nearest filter the new IF has.

Asking for 0, the default, lets the library choose:

  - At zero IF it takes the narrowest filter that is at least as wide as the sample rate, up to 8 MHz. It changes when the rate changes. For example 2.048 Msps gets 5 MHz, 6 Msps gets 6 MHz and 10 Msps gets 8 MHz. This passes the whole stream and keeps strong signals outside it away from the receiver's converters.
  - With a low IF it takes the widest filter the IF has. The software filters do the rest.

Some programs, have no field for the filter. For those, put it in the device string, for example `driver=mirisdr,bandwidth=1536000`. A program that asks for 0 then gets this filter.

| IF | Filters |
|---|---|
| Zero IF | 1.536, 5, 6, 7 and 8 MHz, and 14 MHz (no filter) |
| 450 kHz | 200, 300 and 600 kHz |
| 1620 kHz and 2048 kHz | 600 kHz and 1.536 MHz |


<h3>Gain</h3>

Set the gain as one total, from 0 to 102 dB on most bands. The library splits it over the stages in the way that works best, with the front end first so the noise stays low. It does this again when you tune to another band.

The stages can also be set one by one:

  - `LNA`: 0 or its full gain, which is 24 dB on most bands, 7 dB in band IV/V and about 4 dB in L band. The AM inputs have none.
  - `MIX`: 0 or 19 dB.
  - `MIXBUF`: 0 or 24 dB, only where the receiver uses its AM input: below 55 MHz, and from 255 to 420 MHz through the up-converter. On the RSP1 that is below 50 MHz only. The tuner's other AM input, with steps of 6 dB, is not used.
  - `BB`: 0 to 59 dB.

The ranges the module reports are the most each stage has in any band, because programs such as GQRX read them only once, when the device opens. A value the band you are tuned to cannot take gives the nearest one it can, and a stage the band does not have stays at 0. For ranges that follow the band, add `gain_ranges=band` to the device string. There is no automatic gain control.


<h3>Sample formats</h3>

  - `CF32`: the native format, complex float with full scale 1.0.
  - `CS16`: complex int16 with full scale 32768.
  - `CS8`: complex int8 with full scale 128.

With `baseband` off and one converter at a low IF, the samples are real instead, as `S16` or `F32`.


<h3>Settings</h3>

Programs show these as device settings. With SoapySDR's API they are `writeSetting()` and `readSetting()`.

| Setting | Values | Default | What it does |
|---|---|---|---|
| `if_freq` | `auto`, `0`, `450000`, `1620000`, `2048000` | `auto` | The tuner IF. `auto` uses zero IF from 1.3 Msps up and a low IF below. The others fix the IF. |
| `converters` | `both`, `I`, `Q` | `both` | With a low IF, use both converters or only one. |
| `baseband` | `true`, `false` | `true` | Put the frequency you set at the centre of the stream, at any IF. |
| `format` | `AUTO`, `252_S16`, `336_S16`, `384_S16`, `504_S16`, `504_S8` | `AUTO` | How samples cross the USB. `AUTO` picks the most bits that fit. |
| `transfer` | `ISOC`, `ISOC1`, `ISOC2`, `BULK` | `ISOC` on Linux, `BULK` on Windows | The USB transfer type. |
| `decimation_bypass` | `AUTO`, `ON`, `OFF` | `AUTO` | Skip the receiver's own filter. `AUTO` does this above 14.5 Msps. |
| `gap_fill` | `true`, `false` | `false` | Put zeros where samples were lost, for gaps of up to 16 USB packets. Always on with `baseband`. |
| `biastee` | `true`, `false` | `false` | RSP1B only. Power on the antenna input. |
| `fm_notch` | `true`, `false` | `false` | RSP1B only. Notch for FM broadcast (85 to 100 MHz). |
| `mw_notch` | `true`, `false` | `false` | RSP1B only. Notch for medium wave (0.4 to 1.6 MHz). |
| `dab_notch` | `true`, `false` | `false` | RSP1B only. Notch for DAB (165 to 230 MHz). |

A setting the receiver cannot do is refused and logged. The receiver keeps what it had.

<h4>More about the IF settings</h4>

  - **Zero IF** needs no processing on the computer. The LO and its DC offset sit in the middle of the band.
  - **A low IF** keeps the LO and DC away from the band and has the narrow filters. The tuner rejects the other side of its LO by about 37 dB. A strong signal at the other side can leak into the stream that much weaker.
  - **Both converters** cost very little CPU. This is the best choice in most cases.
  - **One converter** (`I` or `Q`) halves the USB data and keeps 14 bits at the highest low IF rates. It needs more CPU and halves the rates on offer. The image rejection stays the same, because it comes from the tuner.
  - **`baseband` off** gives the samples as they come from the receiver, as older versions did. The tuner's LO is then at the centre, and with a low IF the band you want lies one IF below it. Use this only if your program does its own IF handling.

<h4>How much CPU</h4>

Measured on a Raspberry Pi Compute Module 0 (one core at 1 GHz), the low IF processing runs at 3 times real time or better with the 2048 kHz IF, and 9 times with both converters at the full 8.192 Msps. The 450 kHz IF needs much less. Zero IF needs no processing.


<h3>Stream options</h3>

These go in the arguments of `setupStream()`:

  - `buffers`: how many USB buffers the module holds for the program. The default is 256. More buffers ride out longer pauses in the consumer.
  - `transfers`: USB transfers in flight. 0, the default, uses the library's choice.


<h3>Time stamps and lost samples</h3>

Every read carries a time stamp. It counts samples from the start of the stream and turns them into nanoseconds. `setHardwareTime()` moves the time stamps to a time you choose.

Samples can go missing in two places.

  - **On the USB**, when the computer does not take the data fast enough. With `baseband` on, the default, or with `gap_fill` on, short gaps are replaced with zeros. The stream keeps its timing and no overflow is reported. Gap fill covers up to 16 USB packets per gap, which is 2 ms at 2 Msps and less at higher rates. Where more is missing, and always with both settings off, the read stops at the gap and returns an overflow. The next read's time stamp shows how many samples are missing.
  - **In the module**, when the program reads too slowly and all its buffers are full. The read returns an overflow and SoapySDR prints `O`. The time stamp of the next read shows the jump.

If you see overflows, try a lower rate, more `buffers`, or the isochronous transfer. Bulk transfers at high rates depend most on the computer.


<h3>A short example in C++</h3>

```cpp
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Errors.hpp>
#include <complex>
#include <cstdio>
#include <vector>

int main()
{
    SoapySDR::Device *sdr = SoapySDR::Device::make("driver=mirisdr");

    sdr->setSampleRate(SOAPY_SDR_RX, 0, 256000);
    sdr->setFrequency(SOAPY_SDR_RX, 0, 433.5e6);
    sdr->setGain(SOAPY_SDR_RX, 0, 10);

    SoapySDR::Stream *rx = sdr->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32);
    sdr->activateStream(rx);

    std::vector<std::complex<float>> buf(4096);
    void *buffs[] = { buf.data() };
    long long total = 0;
    while (total < 256000) {
        int flags;
        long long timeNs;
        int r = sdr->readStream(rx, buffs, buf.size(), flags, timeNs, 1000000);
        if (r == SOAPY_SDR_OVERFLOW) printf("samples lost\n");
        else if (r < 0) break;
        else total += r;
    }

    sdr->deactivateStream(rx);
    sdr->closeStream(rx);
    SoapySDR::Device::unmake(sdr);
    return 0;
}
```

Build it with `g++ example.cpp -lSoapySDR`. At 256000 samples a second the module picks the 2048 kHz IF by itself, and 433.5 MHz is at 0 Hz in `buf`.


<h3>What it does not do</h3>

  - Transmit.
  - More than one channel.
  - Automatic gain control.
  - Frequency correction.
  - Time from a PPS input. The library can do this, but it is not exposed in this module as you need custom hardware.
