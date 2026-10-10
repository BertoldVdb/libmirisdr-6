The libmirisdr C API
====================

This document explains how to use libmirisdr from C. It covers the calls that set up a receiver in one go, the way samples arrive, low IF reception with the band delivered at 0 Hz, and the extra features of the receivers.

The older calls in the style of rtl-sdr, such as `mirisdr_set_center_freq()` and `mirisdr_set_sample_rate()`, still work. New programs should preferably use the calls described here, because they check a whole configuration at once and tell you what you got.

Everything is declared in `mirisdr.h`. Link with `-lmirisdr`.


<h3>The main idea</h3>

A receiver has two configurations.

  - The tune: the frequency, the IF, the filter and the gain. It lives in a `mirisdr_tune_config_t`.
  - The stream: the sample rate, how samples cross the USB, and what the program gets. It lives in a `mirisdr_stream_config_t`.

You change either one the same way. Read the one in force, change the fields you care about and apply it. The library checks the whole configuration first. If any part of it is not possible, it refuses all of it, prints the reason on stderr and keeps what it had. It never quietly picks something else.

Each apply call can fill in a result. The result says what is really running, such as the exact rate or the LO frequency the synthesizer reached. There is also a check call for each configuration, which gives the same result without changing anything.


<h3>A complete example</h3>

This program receives 433.5 MHz through the 2048 kHz IF and gets 256000 complex float samples a second, with 433.5 MHz at 0 Hz.

```c
#include <mirisdr.h>
#include <stdio.h>

static mirisdr_dev_t *dev;
static uint64_t received;

static void callback(unsigned char *buf, uint32_t len, void *ctx)
{
    mirisdr_buffer_info_t info;
    const float *iq = (const float *) buf;   /* I, Q, I, Q, ... */

    mirisdr_get_buffer_info(dev, &info);
    if (info.gap_samples)
        printf("%llu samples missing before sample %llu\n",
               (unsigned long long) info.gap_samples, (unsigned long long) info.sample);

    received += len / (2 * sizeof(float));
    if (received >= 10 * 256000)
        mirisdr_cancel_async(dev);
    (void) iq; (void) ctx;
}

int main(void)
{
    mirisdr_tune_config_t tc;
    mirisdr_tune_result_t tr;
    mirisdr_stream_config_t sc;
    mirisdr_stream_result_t sr;

    if (mirisdr_open(&dev, 0) < 0)
        return 1;

    mirisdr_get_tune(dev, 0, &tc, NULL);
    tc.frequency = 433500000;
    tc.if_freq = 2048000;
    tc.bandwidth = 0;          /* the library picks */
    tc.low_if_auto = 1;        /* LO one IF above the frequency */
    tc.gain.mode = MIRISDR_GAIN_TOTAL;
    tc.gain.total = 10;
    if (mirisdr_tune(dev, 0, &tc, &tr) < 0)
        return 1;

    mirisdr_get_stream(dev, &sc, NULL);
    sc.baseband = 1;
    sc.rate = 256000;
    if (mirisdr_set_stream(dev, &sc, &sr) < 0)
        return 1;
    printf("%u samples/s from %u, LO %u Hz, bandwidth %u Hz\n",
           sr.rate, sr.adc_rate, tr.lo, tr.bandwidth);

    mirisdr_reset_buffer(dev);
    mirisdr_read_async(dev, callback, NULL, 0, 0);
    mirisdr_close(dev);
    return 0;
}
```

It prints `256000 samples/s from 8192000, LO 435548000 Hz, bandwidth 1536000 Hz`. The rest of this document explains each part.


<h3>Opening a receiver</h3>

`mirisdr_open(&dev, index)` opens a receiver by its place in the list. `mirisdr_get_device_count()`, `mirisdr_get_device_name()` and `mirisdr_get_device_usb_strings()` describe the receivers found. `mirisdr_get_index_by_serial()` finds one by its serial.

For more control use `mirisdr_open_ex()`. Fill the config with `mirisdr_open_config_default()` first and change only what you need.

```c
mirisdr_open_config_t cfg;
mirisdr_open_config_default(&cfg);
cfg.serial = "2405278B60";
if (mirisdr_open_ex(&dev, &cfg) < 0)
    return 1;
```

The fields you are most likely to use:

  - `index`, `serial` or `fd`: which receiver. A serial wins over the index. `fd` takes a USB file descriptor that is already open, as on Android.
  - `hw_flavour`: the kind of board. `MIRISDR_HW_AUTO`, the default, tells from the USB ids. Other values are `MIRISDR_HW_DEFAULT`, `MIRISDR_HW_SDRPLAY` and `MIRISDR_HW_RSP1B`. A wrong value can switch things the board does not have, such as a bias-T.
  - `firmware`, `firmware_path` and `keep_running`: the library loads its own firmware into the receiver by default, and replaces a different one that is running. Give another image, or set `keep_running` to use what runs.

`mirisdr_close()` stops any stream and frees everything.


<h3>Tuning</h3>

A tune sets the frequency, the IF, the filter, which tuner outputs run and the gain, all in one request.

```c
mirisdr_tune_config_t tc;
mirisdr_tune_result_t tr;

mirisdr_get_tune(dev, 0, &tc, NULL);   /* start from what is in force */
tc.frequency = 145000000;
tc.gain.mode = MIRISDR_GAIN_KEEP;      /* leave the gain alone */
if (mirisdr_tune(dev, 0, &tc, &tr) < 0)
    printf("refused\n");
```

The second argument is the tuner. It is always 0 for now. It is there for receivers with two tuners, such as the RSPduo.

<h4>The tune config</h4>

  - `frequency`: the frequency to receive, in Hz.
  - `if_freq`: 0 for zero IF, or a low IF of 450000, 1620000 or 2048000.
  - `bandwidth`: the tuner's filter in Hz. 0 lets the library pick. At zero IF it takes the narrowest filter at least as wide as the sample rate, up to 8 MHz, and changes it when the rate changes. For example 2.048 Msps gets 5 MHz and 10 Msps gets 8 MHz. With a low IF it takes the widest the IF has. The filters are listed below.
  - `low_if_auto`: with a low IF, put the LO one IF above `frequency`, so `frequency` is what the tuner passes. Without it the LO sits at `frequency`.
  - `lo_offset`: moves the LO by this many Hz on top of the above.
  - `iq`: with a low IF, which tuner outputs run. `MIRISDR_IQ_BOTH`, or `MIRISDR_IQ_ONLY_I` or `MIRISDR_IQ_ONLY_Q` to switch one off.
  - `gain`: see the next section.
  - `frontend`: the bias-T and the notch filters, see below.
  - `override` and `synth_thresh`: expert settings. They hold the tuner's calibration codes and fix the synthesizer's grid. Leave them at 0 unless you know you need them.

The filters each IF has:

| IF | Filters |
|---|---|
| Zero IF | 1536000, 5000000, 6000000, 7000000, 8000000, and 14000000 (no filter) |
| 450 kHz | 200000, 300000, 600000 |
| 1620 kHz and 2048 kHz | 600000, 1536000 |

A filter the IF does not have is refused. This matters when you change the IF. `mirisdr_get_tune()` gives the bandwidth back as you asked for it. A 0 stays 0 and works at every IF. A filter you chose yourself, for example 8 MHz at zero IF, may not exist at the new IF. Then set `bandwidth` to 0, or to a filter of the new IF, when you change `if_freq`. The tune result always says which filter is in force.

<h4>The tune result</h4>

  - `lo`: the LO frequency the synthesizer reached.
  - `offset`: where `frequency` ends up in the stream. With a baseband stream this is 0, apart from the synthesizer's small error.
  - `inverted`: with one tuner output, whether the spectrum is mirrored.
  - `iq`: the outputs that run.
  - `bandwidth`: the filter in force.
  - `gain`: the gain of each stage and the total.
  - `frontend`: the bias-T and notches in force.
  - `band`: the tuner band, such as `MIRISDR_BAND_VHF`.

`mirisdr_tune_check()` takes the same arguments and fills the same result, without tuning. Use it to find out what would work.

The older `mirisdr_set_center_freq()` sets the LO directly. It switches `low_if_auto` and `lo_offset` off. Under a baseband stream at a low IF it sets the frequency put at 0 Hz instead, with the LO the IF above it, as any tune there does.

<h4>The front end</h4>

`tc.frontend` holds what sits before the tuner. It goes out with the tune, in the same request.

  - `bias`: the bias-T, which powers the antenna input.
  - `notch`: notch filters, on the RSP1B only. `MIRISDR_NOTCH_FM` covers 85 to 100 MHz, `MIRISDR_NOTCH_MW` medium wave from 0.4 to 1.6 MHz, and `MIRISDR_NOTCH_DAB` 165 to 230 MHz. Combine them with `|`.

A notch on a board without one is refused. The config keeps what you asked for, and the tune result reports both as in force.

`mirisdr_set_bias()` and `mirisdr_set_notch()` still work. They change this part of the tune in force and apply it.


<h3>Gain</h3>

The gain goes with the tune, in `tc.gain`. Its `mode` decides what happens.

  - `MIRISDR_GAIN_KEEP`: leave the gain as it is. The stages adapt to a new band by themselves.
  - `MIRISDR_GAIN_TOTAL`: set `total` in dB. The library splits it over the stages in the way that works best, with the front end first so noise stays low. It does this again for every band. A total above what the band can do gives the most it can do.
  - `MIRISDR_GAIN_STAGES`: set each stage yourself. The stages keep these values through later tunes.

The stages:

  - `lna`: on or off. It adds 24 dB on most bands, 7 dB in band IV/V and about 4 dB in L band. The AM inputs have none.
  - `mixer`: on or off. It adds 19 dB.
  - `mixbuffer`: only on the AM inputs. 0, 6, 12 or 18 dB on AM1, 0 or 24 dB on AM2.
  - `baseband`: 0 to 59 dB.

The total therefore goes from 0 to 102 dB on most bands. The tune result reports the stages and the total in force. There is no automatic gain control.


<h3>The stream</h3>

The stream config says how fast to sample, how samples cross the USB and what the program gets.

```c
mirisdr_stream_config_t sc;
mirisdr_stream_result_t sr;

mirisdr_get_stream(dev, &sc, NULL);
sc.rate = 8000000;
sc.transfer = "BULK";
if (mirisdr_set_stream(dev, &sc, &sr) < 0)
    printf("refused\n");
printf("%u samples/s as %s, %u bytes/s on the USB\n", sr.rate, sr.format, sr.usb_bytes);
```

`mirisdr_stream_config_default()` gives a fresh config at 2 Msps with everything automatic.

<h4>The stream config</h4>

  - `rate`: samples per second.
  - `format`: how samples are packed on the USB. NULL means `"AUTO"`. See below.
  - `transfer`: `"BULK"`, `"ISOC"`, `"ISOC2"` or `"ISOC1"`. NULL is the platform's default, isochronous on Linux and other Unix systems and bulk on Windows.
  - `decimation_bypass`: `"AUTO"`, `"ON"` or `"OFF"`. NULL means `"AUTO"`. See below.
  - `swap_iq`: swaps I and Q, which mirrors the spectrum. With a real format it picks the Q converter instead of I.
  - `gap_fill`: puts zeros where samples were lost, so the timing of the stream stays right.
  - `follow_tune`: the stream switches between complex and real by itself when a tune switches a tuner output off or on.
  - `format_single`: with `follow_tune`, the real format to use for one output. NULL means `"AUTO_REAL"`.
  - `usb_capacity`: the bytes per second the automatic format may plan on. 0 is the default.
  - `baseband`: deliver complex float with the tune's frequency at 0 Hz, at any IF. This has its own section below.

<h4>Sample formats</h4>

Samples travel in 1 kB USB packets. A format packs a fixed number of samples in each packet. More samples per packet means fewer bits per sample but less USB data.

| Format | Samples per packet | Kind |
|---|---|---|
| `252_S16` | 252 | complex, 14 bits |
| `336_S16` | 336 | complex, 12 bits |
| `384_S16` | 384 | complex, 10 bits plus 2 |
| `504_S16` | 504 | complex, 8 bits, delivered as int16 |
| `504_S8` | 504 | complex, 8 bits, delivered as int8 |
| `504_REAL_S16` | 504 | real, one converter |
| `672_REAL_S16` | 672 | real, one converter |
| `768_REAL_S16` | 768 | real, one converter |

The USB data rate is `rate x 1024 / samples per packet` bytes a second. Complex samples arrive as I, Q pairs. Real samples come from a single converter, chosen with `swap_iq`. They only make sense with a low IF.

`"AUTO"` picks the complex format with the most bits that fits the USB. `"AUTO_REAL"` does the same among the real formats.

<h4>USB transfers and the highest rates</h4>

| Transfer | Capacity | Highest rate with 8-bit samples |
|---|---|---|
| `ISOC` | 24.576 MB/s | about 12.1 Msps |
| `ISOC2` | 16.384 MB/s | about 8 Msps |
| `ISOC1` | 8.192 MB/s | about 4 Msps |
| `BULK` | what the computer manages | about 27.5 Msps |

Isochronous transfers reserve their bandwidth on the USB, so their capacity is a hard limit. A rate that does not fit is refused.

Bulk transfers take whatever the computer gives them. The automatic format plans on 32 MB/s in bulk, which every computer tried so far reaches. Above that it still accepts the rate and uses the 8-bit format. If the computer cannot keep up, samples go missing and show up as gaps. Only the receiver's own limit of about 56 MB/s is refused. Set `usb_capacity` if you know your computer does better or worse than 32 MB/s.

<h4>Decimation bypass</h4>

The receiver normally samples at twice the rate and filters down. Its clock reaches about 15 Msps that way. Above that the bypass skips the filter and samples at the rate itself. The program then gets the full width without the receiver's own filtering and has to filter itself.

  - `"AUTO"`: bypass only above 14.5 Msps.
  - `"OFF"`: never bypass, 1.3 to 15 Msps.
  - `"ON"`: always bypass, 2.6 to 30 Msps.

<h4>The stream result</h4>

  - `rate`: the rate the sample clock really reached.
  - `format`: the format running, never an automatic one.
  - `adc`: what is sampled, `MIRISDR_IQ_BOTH` for complex or `MIRISDR_IQ_ONLY_I` or `_ONLY_Q` for real.
  - `decimation_bypassed`: whether the bypass is on.
  - `usb_bytes`: bytes per second on the USB.
  - `usb_capacity`: what the automatic choice planned on.
  - `baseband`, `adc_rate` and `decimation`: see the baseband section.

`mirisdr_stream_check()` gives the result without changing anything.

<h4>Changing the stream while it runs</h4>

You can apply a new stream config while samples are flowing. The stream restarts with a short gap. The buffer being filled at that moment is dropped, so a buffer never mixes two formats.


<h3>Reading samples</h3>

<h4>The callback</h4>

`mirisdr_read_async(dev, callback, ctx, num, len)` streams until it is cancelled. It calls `callback(buf, len, ctx)` from its own thread for every buffer.

  - `num` is the number of USB transfers in flight. 0 means 32.
  - `len` is the buffer length in bytes. 0 means the buffers come as the USB delivers them, which is the best choice for most programs. A fixed length gathers samples into buffers of exactly that size.

Call `mirisdr_reset_buffer()` before you start. Stop with `mirisdr_cancel_async()`, from the callback or from another thread. A cancel counts from the moment `mirisdr_read_async()` is entered, even before the first transfer runs. A cancel made before that is lost, so keep a flag of your own and check it in the callback.

Keep the callback short. If it blocks, the USB transfers run dry and samples are lost.

<h4>What a buffer holds</h4>

Inside the callback, `mirisdr_get_buffer_info()` describes the buffer. It only works from inside the callback.

  - `type`: how each value is stored. `MIRISDR_SAMPLE_S16` is int16 with full scale 32768. `MIRISDR_SAMPLE_S8` is int8 with full scale 128. `MIRISDR_SAMPLE_F32` is float with full scale 1.0.
  - `adc`: `MIRISDR_IQ_BOTH` for complex I, Q pairs, or `MIRISDR_IQ_ONLY_I` or `_ONLY_Q` for real samples.
  - `rate`: samples per second.
  - `sample`: how many samples were delivered before this buffer.
  - `index`: the position of the first sample in time, counting lost samples too.
  - `gap_samples`, `gaps_len` and `gaps`: where samples are missing inside this buffer and how many.

Each gap has an `offset` (the samples of the buffer before it), `samples` (how many are missing) and `filled` (how many of those gap fill replaced with zeros).

The difference between `index` and `sample` is the number of samples lost so far and not filled. Use `index` to put samples on a time line.

<h4>Gap fill</h4>

With `gap_fill` set in the stream config (or `mirisdr_set_gap_fill()`), the library puts zeros where samples went missing. The stream then keeps its timing, which matters for demodulators and filters. The gaps are still reported, with `filled` set.

Gap fill only covers short gaps. It puts in at most 16 USB packets of zeros per gap, which is 2 ms at 2 Msps with 14-bit samples and less at higher rates. A longer gap gets those zeros and the rest stays missing, so `filled` is less than `samples` and the time line jumps there.

<h4>Stream statistics</h4>

`mirisdr_get_stream_stats()` gives totals since the stream started: samples delivered, samples lost, gaps, samples filled, and a few error counters. Call it from the callback, where the numbers are consistent. Its `index` is the position of the current buffer's first sample.

<h4>Reading without a callback</h4>

`mirisdr_read_sync()` reads into your buffer and returns. It gives no buffer information, so you cannot see gaps. It does not do baseband, and it needs the `BULK` transfer, which is not the default on Linux. Use `mirisdr_read_async()` where you can.


<h3>Baseband: the band at 0 Hz, at any IF</h3>

Set `baseband` in the stream config and the callback gets complex float samples with the tune's `frequency` at 0 Hz. This works the same at zero IF and at a low IF, so the program does not need to know which IF runs.

<h4>Why a low IF</h4>

At zero IF the receiver's lowest rate is 1.3 Msps, and the LO and the DC offset sit in the middle of the band. A low IF moves the band away from the LO, gives access to narrower tuner filters, and lets the library filter down to much lower rates.

<h4>How it works</h4>

  - At zero IF the samples are only converted to float. No other processing is done.
  - With a low IF the LO goes one IF above `frequency`. The converters run at 4 times the IF, so the band sits at a quarter of their rate. The library shifts it to 0 Hz and removes the DC offset. With both converters the shift is just a change of sign, which costs almost nothing.
  - With one converter (`iq` set to `MIRISDR_IQ_ONLY_I` or `_ONLY_Q` in the tune) a real to I/Q converter makes complex samples at half the rate. This halves the USB data and keeps more bits at high rates, but costs more CPU.
  - Half-band filters then halve the rate as many times as needed. Each is flat to 0.4 of its output rate and 61 dB down from 0.6.

The image rejection is the tuner's own, about 37 dB, with one converter or both.

<h4>Which rates</h4>

At zero IF any normal rate works, from 1.3 Msps. With a low IF the rate must be the converter rate divided by a power of 2, up to 256, as long as the result is a whole number. With one converter it must be divided by 2 at least.

| IF | Converter rate | Rates with both converters |
|---|---|---|
| 450 kHz | 1.8 Msps | 1800000, 900000, 450000, 225000, 112500, 56250, 28125 |
| 1620 kHz | 6.48 Msps | 6480000, 3240000, 1620000, 810000, 405000, 202500, 101250, 50625 |
| 2048 kHz | 8.192 Msps | 8192000, 4096000, 2048000 and so on down to 32000 |

Tune the IF first, then set the stream, because the IF decides which rates exist. A rate the IF does not have is refused with a message that says which ones it has.

A tune that changes the IF while the stream runs moves the rate to the nearest one the new IF has. Check `mirisdr_get_stream()` afterwards.

<h4>The stream result with baseband</h4>

  - `baseband`: `MIRISDR_BASEBAND_ZERO_IF`, `MIRISDR_BASEBAND_COMPLEX` (low IF, both converters) or `MIRISDR_BASEBAND_REAL` (low IF, one converter).
  - `rate`: the output rate the callback gets.
  - `adc_rate`: the converters' rate.
  - `decimation`: converter samples per output sample.

<h4>Things to know</h4>

  - The LO moves one IF above `frequency` when the stream starts. Set `low_if_auto` in the tune and it is there from the start, and the tune result shows the right LO.
  - Buffer information counts output samples. Gaps are placed to within the filters' delay. Stream statistics, stream events, PPS and scan reports still count converter samples. Divide by `decimation` to compare.
  - Gap fill is always on, because the filters need the timing. A gap longer than gap fill covers still makes the time line jump.
  - `swap_iq` still mirrors the spectrum.
  - A buffer length given to `mirisdr_read_async()` is in output bytes, 8 bytes for each complex sample.
  - `mirisdr_read_sync()` refuses baseband.
  - The processing runs in the callback's thread. On a Raspberry Pi Compute Module 0 (one core at 1 GHz) it runs at 3 times real time or better with the 2048 kHz IF, and 9 times with both converters at the full 8.192 Msps. The 450 kHz IF needs much less.
  - The same signal reads about 12 dB higher with a low IF than at zero IF. This is not explained yet, so do not compare absolute levels between the two.


<h3>Stream events and the marker</h3>

Every USB packet carries the gain in force and bits that flip when a gain, tune or rate change takes effect. `mirisdr_set_stream_events(dev, cb, ctx)` calls `cb` for each packet where something changed, before its samples are delivered. The event says which change (`MIRISDR_EVENT_GAIN`, `_TUNE`, `_RATE`, `_MARK`, or `_MISSED` when packets were lost), where in the stream it happened, and the gain of each stage.

`mirisdr_set_stream_mark(dev, on)` sets a marker bit that travels with the samples. Use it to find the exact sample where something you did took effect.


<h3>Pulse per second</h3>

A pulse on GPIO 0 can be time stamped in the stream's own samples. This needs the library's firmware, which it loads by default, and a running stream.

  - `mirisdr_enable_pps(dev, 1)` starts it. This costs one damaged packet.
  - `mirisdr_get_pps()` gives the sample of the last rising edge, an edge counter, and two flags. Ignore the sample when `trusted` or `gapless` is 0. Both repair themselves.
  - `mirisdr_set_pps_source(dev, 1, divider)` takes the edges from the USB frame clock instead of GPIO 0.


<h3>Scanning</h3>

The receiver can hop through a list of tunes by itself, one hop every few USB packets, while the computer only keeps the list fed.

  1. `mirisdr_scan_compile()` turns an array of tune configs into a scan.
  2. `mirisdr_scan_start()` starts it and calls your function for every hop as it shows in the stream, with the exact sample where it started.
  3. `mirisdr_scan_feed()` keeps the list fed. Call it often from another thread.
  4. `mirisdr_scan_stop()` ends the scan and restores the tune from before.

`mirisdr_tune_learn()` tunes once, reads back the tuner's calibration and fills the config's `override` with it. Hops with held codes settle faster. L band hops need this.

The comments in `mirisdr.h` describe the timing in detail.


<h3>Board features</h3>

  - The bias-T and the RSP1B's notches are part of the tune. See the front end under Tuning.
  - GPIO: `mirisdr_set_gpio_direction()`, `mirisdr_set_gpio_output()`, `mirisdr_get_gpio_input()` and related calls, for 4 pins.
  - I2C on GPIO 1 and 2: `mirisdr_i2c_write()`, `mirisdr_i2c_read()` and `mirisdr_i2c_transfer()`.
  - Serial output on GPIO 2: `mirisdr_uart_write()`.
  - The infrared receiver on GPIO 3: `mirisdr_set_ir()` calls a function for every pulse.
  - The EEPROM: `mirisdr_read_eeprom()`, `mirisdr_write_eeprom()` and `mirisdr_eeprom_size()`.
  - Firmware: `mirisdr_get_fw_id()` reads the id of the running image. `mirisdr_fw_get()` and `mirisdr_fw_patch()` read and change the USB ids and serial inside an image.
  - `mirisdr_get_serial()`, `mirisdr_get_usb_ids()` and `mirisdr_get_usb_position()` say which device this is and where it sits on the USB.

<h4>For experts</h4>

`mirisdr_write_reg()`, `mirisdr_write_mem()`, `mirisdr_call()`, `mirisdr_load_list()` and `mirisdr_set_tuner_override()` reach the hardware directly. Writing the wrong value can hang the receiver until it is unplugged. `mirisdr_get_tuner_status()` and `mirisdr_read_reg()` only read and are safe.


<h3>Testing without a receiver</h3>

`mirisdr_open_null(&dev, format)` opens a fake receiver. `mirisdr_feed_bulk()` pushes your own USB packets through the library as if they came from a receiver. This is how the library's own tests check the sample handling, gap reports and baseband filters.


<h3>Threads</h3>

The library follows rtl-sdr's model. A device has no lock around its calls, so the threads that use it share the work as follows:

  - **One control thread.** Tuning, gain, rate, format and the other settings, scans, GPIO, EEPROM, UART, I2C and register access all come from one thread at a time. Several of them are made of a sequence of USB requests, and the library keeps the registers it last wrote in a cache. Two control threads at once interleave those requests and let the cache go out of date. If your program has more than one thread that changes settings, hold a lock of your own around these calls.
  - **The stream thread** is the one inside `mirisdr_read_async()`. It calls your callback.
  - **From the callback**, call only `mirisdr_cancel_async()` and the calls that read the stream: `mirisdr_get_buffer_info()`, `mirisdr_get_stream_stats()` and `mirisdr_get_stream()`. A setting changed from the callback would have to stop the stream, which waits for the callback that is asking it to.
  - **`mirisdr_cancel_async()`** may come from any thread at any time, also while the control thread changes a setting that restarts the stream.
  - **`mirisdr_scan_feed()`** runs in its own thread, next to the stream and the control thread, as described under Scanning.
  - **`mirisdr_close()`** comes last, from the control thread. It cancels the stream and waits for `mirisdr_read_async()` to return, so call it outside the callback.

Settings that change the stream may be made while it runs, from the control thread: the stream stops, takes the change and starts again, so the callback never sees a setting change under it.

<h3>Room to grow</h3>

Every config and result struct ends in a `reserved` array. The register list entries and `mirisdr_call_regs_t` mirror the firmware and have none. Later versions of the library add fields there, so the struct keeps its size and programs built against this version keep working. A new field always means the old behaviour when it is 0.

  - Start a config from its default call (such as `mirisdr_tune_config_default()`) or its get call (such as `mirisdr_get_tune()`). Both leave `reserved` at 0.
  - Do not write to `reserved`. A config with anything set there is refused, with a message that the program needs a newer libmirisdr. This way a program written for a newer library fails clearly on an older one, rather than having settings quietly ignored.
  - Results come back with `reserved` at 0.
  - The structs only handed to your callbacks (stream events, scan reports and IR pulses) belong to the library. They can grow at their end.


<h3>Return values</h3>

  - 0 or more means success.
  - A negative value means the call failed or was refused. The reason is printed on stderr. The receiver keeps what it had.
  - `MIRISDR_REOPEN` (-2) means the USB handle is no longer valid and the receiver has to be opened again. This only happens after `mirisdr_reboot()` or when it was opened by file descriptor.
