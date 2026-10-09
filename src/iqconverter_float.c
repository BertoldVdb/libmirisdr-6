/*
 * In libmirisdr: the real to I/Q converter from Airspy (Youssef Touil, MIT licence
 * below), reworked for lpc4370sdr by Bertold Van den Bergh: vectorised, NEON on ARM,
 * the DC removal folded into the translation, split for two threads.
 *
 * Copied from lpc4370sdr-host 0.3.1.
 */
/*
Copyright (C) 2014, Youssef Touil <youssef@airspy.com>
Copyright (c) 2026 Bertold Van den Bergh <vandenbergh@bertold.org> (modifications for lpc4370sdr)

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

#include "iqconverter_float.h"
#include <stdlib.h>
#include <string.h>

#include <stdio.h>

#if defined(__MINGW32__) && !defined(__MINGW64_VERSION_MAJOR)
  #include <malloc.h>
  #define _aligned_malloc __mingw_aligned_malloc
  #define _aligned_free  __mingw_aligned_free
  #define _inline inline
  #define FIR_STANDARD
#elif defined(__APPLE__)
  #include <malloc/malloc.h>
  #define _aligned_malloc(size, alignment) malloc(size)
  #define _aligned_free(mem) free(mem)
  #define _inline inline
  #define FIR_STANDARD
#elif defined(__FreeBSD__)
  #ifdef __SSE2__ /* only where the target has SSE2 (not on ARM or RISC-V) */
  #define USE_SSE2
  #include <immintrin.h>
  #endif
  #define _inline inline
  #define _aligned_free(mem) free(mem)
void *_aligned_malloc(size_t size, size_t alignment)
{
    void *result;
    if (posix_memalign(&result, alignment, size) == 0)
        return result;
    return 0;
}
#elif defined(__GNUC__) && !defined(__MINGW64_VERSION_MAJOR)
  #include <malloc.h>
  #define _aligned_malloc(size, alignment) memalign(alignment, size)
  #define _aligned_free(mem) free(mem)
  #define _inline inline
#else
	#if (_MSC_VER >= 1800)
		//#define USE_SSE2
		//#include <immintrin.h>
	#endif
#endif

/* The FIR and the delay line work on blocks of FIR_BLOCK I/Q pairs */
#define FIR_BLOCK 128

/* GCC and Clang vector extensions: SSE on x86, NEON on ARM, without intrinsics. On ARM, NEON
   intrinsics also split and merge the I/Q pairs around the FIR, four pairs at a time. */
#if defined(__GNUC__) || defined(__clang__)
#define HAVE_V4SF
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#define HAVE_NEON
typedef float32x4_t v4sf;
#else
typedef float v4sf __attribute__((vector_size(16)));
#endif
static _inline v4sf load4(const float *p) { v4sf v; memcpy(&v, p, sizeof(v)); return v; }
static _inline void store4(float *p, v4sf v) { memcpy(p, &v, sizeof(v)); }
#endif
#define DEFAULT_ALIGNMENT 16
#define HPF_COEFF 0.01f

#if defined(_MSC_VER)
	#define ALIGNED __declspec(align(DEFAULT_ALIGNMENT))
#else
	#define ALIGNED
#endif

iqconverter_float_t *iqconverter_float_create(const float *hb_kernel, int len)
{
	int i, j;
	size_t buffer_size;
	iqconverter_float_t *cnv;

	/* A half-band kernel has an odd length, its centre tap in the middle */
	if (hb_kernel == NULL || len < 3 || (len & 1) == 0)
	{
		return NULL;
	}

	cnv = (iqconverter_float_t *) _aligned_malloc(sizeof(iqconverter_float_t), DEFAULT_ALIGNMENT);
	if (cnv == NULL)
	{
		return NULL;
	}

	cnv->len = len / 2 + 1;
	cnv->hbc = hb_kernel[len / 2];
	cnv->dc_removal = 1;

	buffer_size = cnv->len * sizeof(float);

	cnv->fir_kernel = (float *) _aligned_malloc(buffer_size, DEFAULT_ALIGNMENT);
	cnv->fir_queue = (float *) _aligned_malloc((cnv->len - 1 + FIR_BLOCK) * sizeof(float), DEFAULT_ALIGNMENT);
	cnv->delay_line = (float *) _aligned_malloc((cnv->len / 2 + FIR_BLOCK) * sizeof(float), DEFAULT_ALIGNMENT);
	if (cnv->fir_kernel == NULL || cnv->fir_queue == NULL || cnv->delay_line == NULL)
	{
		iqconverter_float_free(cnv);
		return NULL;
	}

	iqconverter_float_reset(cnv);

	for (i = 0, j = 0; i < cnv->len; i++, j += 2)
	{
		cnv->fir_kernel[i] = hb_kernel[j];
	} 

	return cnv;
}

void iqconverter_float_free(iqconverter_float_t *cnv)
{
	if (cnv == NULL)
	{
		return;
	}
	if (cnv->fir_kernel != NULL)
		_aligned_free(cnv->fir_kernel);
	if (cnv->fir_queue != NULL)
		_aligned_free(cnv->fir_queue);
	if (cnv->delay_line != NULL)
		_aligned_free(cnv->delay_line);
	_aligned_free(cnv);
}

void iqconverter_float_reset(iqconverter_float_t *cnv)
{
	cnv->avg = 0.0f;
	cnv->phase = 0;
	cnv->fir_index = 0;
	cnv->delay_index = 0;
	memset(cnv->delay_line, 0, (cnv->len / 2 + FIR_BLOCK) * sizeof(float));
	memset(cnv->fir_queue, 0, (cnv->len - 1 + FIR_BLOCK) * sizeof(float));
}

/*
 * The half-band filter: FIR on the even (I) samples, delay line on the odd (Q) samples.
 * fir_queue holds the last len - 1 I inputs in time order followed by the block being
 * filtered, delay_line the last len / 2 Q inputs followed by the block. Outputs are computed
 * FIR_GROUP at a time with every tap applied to the whole group, so the accumulators stay in
 * (vector) registers and no output waits on a chain of dependent multiply-adds.
 */
#define FIR_GROUP 16

static void fir_delay_interleaved(iqconverter_float_t *cnv, float *samples, int len)
{
	const int taps = cnv->len;
	const int half = cnv->len / 2;
	const float *kernel = cnv->fir_kernel;
	float *x = cnv->fir_queue;
	float *q = cnv->delay_line;
	int n = len / 2;
	int done, i, k;

	for (done = 0; done < n; done += FIR_BLOCK)
	{
		int m = (n - done < FIR_BLOCK) ? n - done : FIR_BLOCK;
		float *out = samples + 2 * done;

		i = 0;
#ifdef HAVE_NEON
		for (; i + 4 <= m; i += 4)
		{
			const float32x4x2_t iq = vld2q_f32(out + 2 * i);
			vst1q_f32(x + taps - 1 + i, iq.val[0]);
			vst1q_f32(q + half + i, iq.val[1]);
		}
#endif
		for (; i < m; i++)
		{
			x[taps - 1 + i] = out[2 * i];
			q[half + i] = out[2 * i + 1];
		}
		/* The outputs: the FIR's in the I positions, the delayed Q samples in the Q positions */
		for (i = 0; i + FIR_GROUP <= m; i += FIR_GROUP)
		{
#ifdef HAVE_V4SF
			v4sf a0 = { 0 }, a1 = { 0 }, a2 = { 0 }, a3 = { 0 };
			for (k = 0; k < taps; k++)
			{
				const float h = kernel[k];
				const float *xk = x + taps - 1 - k + i;
				a0 += h * load4(xk);
				a1 += h * load4(xk + 4);
				a2 += h * load4(xk + 8);
				a3 += h * load4(xk + 12);
			}
#endif
#ifdef HAVE_NEON
			float32x4x2_t iq;
			iq.val[0] = a0;
			iq.val[1] = vld1q_f32(q + i);
			vst2q_f32(out + 2 * i, iq);
			iq.val[0] = a1;
			iq.val[1] = vld1q_f32(q + i + 4);
			vst2q_f32(out + 2 * i + 8, iq);
			iq.val[0] = a2;
			iq.val[1] = vld1q_f32(q + i + 8);
			vst2q_f32(out + 2 * i + 16, iq);
			iq.val[0] = a3;
			iq.val[1] = vld1q_f32(q + i + 12);
			vst2q_f32(out + 2 * i + 24, iq);
#else
			float acc[FIR_GROUP];
			int j;
#ifdef HAVE_V4SF
			store4(acc, a0);
			store4(acc + 4, a1);
			store4(acc + 8, a2);
			store4(acc + 12, a3);
#else
			memset(acc, 0, sizeof(acc));
			for (k = 0; k < taps; k++)
			{
				const float h = kernel[k];
				const float *xk = x + taps - 1 - k + i;
				for (j = 0; j < FIR_GROUP; j++)
				{
					acc[j] += h * xk[j];
				}
			}
#endif
			for (j = 0; j < FIR_GROUP; j++)
			{
				out[2 * (i + j)] = acc[j];
				out[2 * (i + j) + 1] = q[i + j];
			}
#endif
		}
		for (; i < m; i++)
		{
			float acc = 0.0f;
			for (k = 0; k < taps; k++)
			{
				acc += kernel[k] * x[taps - 1 - k + i];
			}
			out[2 * i] = acc;
			out[2 * i + 1] = q[i];
		}
		memmove(x, x + m, (taps - 1) * sizeof(float));
		memmove(q, q + m, half * sizeof(float));
	}
}

#define SCALE (0.01f)

/*
 * DC removal and the fs/4 translation in one pass.
 * avg[n+1] = avg[n] + SCALE * (x[n] - avg[n]) = A * avg[n] + SCALE * x[n], with A = 1 - SCALE.
 * Four samples at a time: their outputs and the next average follow from avg[n] and the four
 * inputs, so the only dependency from one group to the next is one multiply-add. The fs/4
 * translation multiplies the samples by -1, -hbc, 1, hbc in turn; cnv->phase is where the
 * next block starts in that cycle, so blocks of any length join up.
 */
static void remove_dc_translate_fs_4(iqconverter_float_t *cnv, float *samples, int len)
{
	const float a = 1.0f - SCALE;
	const float a2 = a * a, a3 = a2 * a, a4 = a3 * a;
	const float hbc = cnv->hbc;
	const float r[4] = { -1.0f, -hbc, 1.0f, hbc };
	const int ph = cnv->phase;
	const float r0 = r[ph], r1 = r[(ph + 1) & 3], r2 = r[(ph + 2) & 3], r3 = r[(ph + 3) & 3];
	float avg = cnv->avg;
	int i;

	if (!cnv->dc_removal)
	{
		/* the translation alone (see iqconverter_float_t.dc_removal) */
		for (i = 0; i + 4 <= len; i += 4)
		{
			samples[i] *= r0;
			samples[i + 1] *= r1;
			samples[i + 2] *= r2;
			samples[i + 3] *= r3;
		}
		for (; i < len; i++)
		{
			samples[i] *= r[(ph + i) & 3];
		}
		cnv->phase = (ph + (len & 3)) & 3;
		return;
	}

#ifdef HAVE_V4SF
	const v4sf pow_a = { 1.0f, a, a2, a3 };
	const v4sf c0 = { 0.0f, SCALE, SCALE * a, SCALE * a2 };
	const v4sf c1 = { 0.0f, 0.0f, SCALE, SCALE * a };
	const v4sf c2 = { 0.0f, 0.0f, 0.0f, SCALE };
	const v4sf rot = { r0, r1, r2, r3 };

	/* Two groups per iteration: an in-order core works on the second group's terms
	   while the first group's average is being computed */
	for (i = 0; i + 8 <= len; i += 8)
	{
		const v4sf x = load4(samples + i);
		const v4sf y = load4(samples + i + 4);
		const v4sf px = x[0] * c0 + x[1] * c1 + x[2] * c2;
		const v4sf py = y[0] * c0 + y[1] * c1 + y[2] * c2;
		const float p4x = SCALE * (a3 * x[0] + a2 * x[1] + a * x[2] + x[3]);
		const float p4y = SCALE * (a3 * y[0] + a2 * y[1] + a * y[2] + y[3]);
		const float avg_y = a4 * avg + p4x;

		store4(samples + i, (x - (avg * pow_a + px)) * rot);
		store4(samples + i + 4, (y - (avg_y * pow_a + py)) * rot);
		avg = a4 * avg_y + p4y;
	}
	for (; i + 4 <= len; i += 4)
	{
		const v4sf x = load4(samples + i);
		const v4sf p = x[0] * c0 + x[1] * c1 + x[2] * c2;
		const float p4 = SCALE * (a3 * x[0] + a2 * x[1] + a * x[2] + x[3]);

		store4(samples + i, (x - (avg * pow_a + p)) * rot);
		avg = a4 * avg + p4;
	}
#else
	for (i = 0; i + 4 <= len; i += 4)
	{
		const float x0 = samples[i], x1 = samples[i + 1], x2 = samples[i + 2], x3 = samples[i + 3];
		const float p1 = SCALE * x0;
		const float p2 = a * p1 + SCALE * x1;
		const float p3 = a * p2 + SCALE * x2;
		const float p4 = a * p3 + SCALE * x3;

		samples[i] = (x0 - avg) * r0;
		samples[i + 1] = (x1 - (a * avg + p1)) * r1;
		samples[i + 2] = (x2 - (a2 * avg + p2)) * r2;
		samples[i + 3] = (x3 - (a3 * avg + p3)) * r3;
		avg = a4 * avg + p4;
	}
#endif
	/* A tail of fewer than four samples, one at a time */
	for (; i < len; i++)
	{
		const float d = samples[i] - avg;
		avg += SCALE * d;
		samples[i] = d * r[(ph + i) & 3];
	}

	cnv->avg = avg;
	cnv->phase = (ph + (len & 3)) & 3;
}

/* The two halves of iqconverter_float_process(), for running them on two threads: they use
   separate parts of the state */
void iqconverter_float_process_first(iqconverter_float_t *cnv, float *samples, int len)
{
	remove_dc_translate_fs_4(cnv, samples, len);
}

void iqconverter_float_process_second(iqconverter_float_t *cnv, float *samples, int len)
{
	fir_delay_interleaved(cnv, samples, len);
}

void iqconverter_float_process(iqconverter_float_t *cnv, float *samples, int len)
{
	remove_dc_translate_fs_4(cnv, samples, len);
	fir_delay_interleaved(cnv, samples, len);
}
