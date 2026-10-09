/*
 * The half-band kernel of the Airspy real to I/Q converter (licence below), used by
 * iqconverter_float.c and by the half-band decimators in baseband.c. 47 taps, flat to
 * 0.2 fs, at least 61 dB down from 0.3 fs.
 */
/*
Copyright (C) 2014, Youssef Touil <youssef@airspy.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#ifndef MIRISDR_HB_KERNEL_H
#define MIRISDR_HB_KERNEL_H

#define MIRISDR_HB_KERNEL_LEN 47

static const float mirisdr_hb_kernel[MIRISDR_HB_KERNEL_LEN] =
{
    -0.000998606272947510f,
    0.000000000000000000f,
    0.001695637278417295f,
    0.000000000000000000f,
    -0.003054430179754289f,
    0.000000000000000000f,
    0.005055504379767936f,
    0.000000000000000000f,
    -0.007901319195893647f,
    0.000000000000000000f,
    0.011873357051047719f,
    0.000000000000000000f,
    -0.017411159379930066f,
    0.000000000000000000f,
    0.025304817427568772f,
    0.000000000000000000f,
    -0.037225225204559217f,
    0.000000000000000000f,
    0.057533286997004301f,
    0.000000000000000000f,
    -0.102327462004259350f,
    0.000000000000000000f,
    0.317034472508947400f,
    0.500000000000000000f,
    0.317034472508947400f,
    0.000000000000000000f,
    -0.102327462004259350f,
    0.000000000000000000f,
    0.057533286997004301f,
    0.000000000000000000f,
    -0.037225225204559217f,
    0.000000000000000000f,
    0.025304817427568772f,
    0.000000000000000000f,
    -0.017411159379930066f,
    0.000000000000000000f,
    0.011873357051047719f,
    0.000000000000000000f,
    -0.007901319195893647f,
    0.000000000000000000f,
    0.005055504379767936f,
    0.000000000000000000f,
    -0.003054430179754289f,
    0.000000000000000000f,
    0.001695637278417295f,
    0.000000000000000000f,
    -0.000998606272947510f
};


/* A short half-band for the stages that another follows: they only need to keep
   aliases out of the final band, at most 0.1 of their rate, so their stopband
   starts at 0.4. 15 taps, Kaiser window (beta 6.5), 65 dB down from 0.4 fs,
   flat within 0.005 dB to 0.1 fs. Made for libmirisdr, not from Airspy */
#define MIRISDR_HB_SHORT_LEN 15

static const float mirisdr_hb_short[MIRISDR_HB_SHORT_LEN] =
{
    -4.275877835e-04f,
    0.000000000e+00f,
    1.092679889e-02f,
    0.000000000e+00f,
    -5.973774928e-02f,
    0.000000000e+00f,
    2.992385382e-01f,
    5.000000000e-01f,
    2.992385382e-01f,
    0.000000000e+00f,
    -5.973774928e-02f,
    0.000000000e+00f,
    1.092679889e-02f,
    0.000000000e+00f,
    -4.275877835e-04f
};

#endif
