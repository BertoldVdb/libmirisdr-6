/*
 * The down-converter in front of the callback (see mirisdr_stream_config_t.baseband).
 * The stream hands it the converters' samples as before, gaps filled. It gives the
 * callback complex float with the tune's frequency at 0 Hz.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 */

#include "iqconverter_float.h"
#include "hb_kernel.h"

#define MIRISDR_BB_STAGES_MAX   8       /* half-band stages, so down to 1/256 */
#define MIRISDR_BB_GAPQ         64
#define MIRISDR_BB_DC_TAU       0.05    /* seconds the DC estimate averages over */

/* GCC and Clang vector types: NEON on ARM, SSE on x86, without intrinsics */
#if defined(__GNUC__) || defined(__clang__)
#define MIRISDR_BB_V4
typedef float mirisdr_v4_t __attribute__((vector_size(16)));
static inline mirisdr_v4_t mirisdr_v4_load (const float *p) { mirisdr_v4_t v; memcpy(&v, p, sizeof v); return v; }
static inline void mirisdr_v4_store (float *p, mirisdr_v4_t v) { memcpy(p, &v, sizeof v); }
#endif

/* A half-band decimator by 2 on interleaved complex float. x holds the last len - 1
   inputs, then the block; e and o its two phases. The last stage takes the long
   kernel (flat to 0.2 fs, 61 dB from 0.3), the ones before it the short one (to 0.1,
   65 dB from 0.4), which is all they need to keep aliases out of the final band */
typedef struct mirisdr_hb
{
    const float *k;         /* the kernel */
    int     len;
    float   *x, *e, *o;
    int     cap;            /* pairs a block may have */
    int     skip;           /* 1: the block's first input is no output position */
} mirisdr_hb_t;

typedef struct mirisdr_bb
{
    int     path, stages, sign;
    uint32_t decim;         /* converter samples per output sample */
    iqconverter_float_t *cnv;
    mirisdr_hb_t hb[MIRISDR_BB_STAGES_MAX];
    float   dc_a, dc_i, dc_q;
    unsigned rot;
    float   *work;
    size_t  work_cap;       /* floats */
    float   carry;          /* one output: an odd sample waiting for its pair */
    int     have_carry;

    /* output side */
    float   *out;           /* a fixed-length buffer being filled */
    uint32_t out_cap, out_n;    /* pairs */
    uint64_t sample;        /* pairs handed out */
    uint64_t produced;      /* pairs out of the filters */
    double  lost;           /* output samples missing, from the converters' unfilled gaps */
    uint64_t lost_in;       /* the converters' unfilled samples seen */
    struct { uint64_t at, samples, filled; } gq[MIRISDR_BB_GAPQ];
    int     gq_n;
} mirisdr_bb_t;

static int mirisdr_hb_size (mirisdr_hb_t *h, int cap)
{
    const int H = h->len - 1;
    float *x = realloc(h->x, (size_t) (H + cap) * 2 * sizeof(float));
    float *e, *o;

    if (!x) return -1;
    h->x = x;
    if (!(e = realloc(h->e, (size_t) (H + cap + 8) * sizeof(float)))) return -1;
    h->e = e;
    if (!(o = realloc(h->o, (size_t) (H + cap + 8) * sizeof(float)))) return -1;
    h->o = o;
    h->cap = cap;

    return 0;
}

static int mirisdr_hb_init (mirisdr_hb_t *h, int cap, int last)
{
    h->k = last ? mirisdr_hb_kernel : mirisdr_hb_short;
    h->len = last ? MIRISDR_HB_KERNEL_LEN : MIRISDR_HB_SHORT_LEN;
    h->x = h->e = h->o = NULL;
    h->skip = 0;
    if (mirisdr_hb_size(h, cap) < 0) return -1;
    memset(h->x, 0, (size_t) (h->len - 1) * 2 * sizeof(float));

    return 0;
}

/* n pairs in, at most n / 2 + 1 out, in place.
 *
 * With the newest input at t, output m is the sum over the kernel of h[k] x[t - k].
 * The kernel is zero at even distances from its centre C (odd), so with the outputs
 * at t = H + skip + 2m the side taps meet only x[skip + 2m + 2i], i = 0..H/2, and the
 * centre x[skip + 2m + C]. Split into e (every other sample from skip) and o (from
 * skip + 1): output m = sum of h[2i] (e[m + i] + e[m + H/2 - i]) over the pairs, plus
 * h[C] o[m + C/2]. Consecutive outputs then read consecutive pairs, so four floats are
 * two outputs.
 *
 * Note: Do not try to merge the two kernels into a single function, this broke GCCs
 * unrolling and costs more performance than using the large kernel everywhere... */
#ifdef MIRISDR_BB_V4
#define MIRISDR_HB_V4(KERN, PAIRS, H, C)                                                \
    for (; m + 8 <= out; m += 8) {                                                      \
        const float *oc = o + 2 * (m + C / 2);                                          \
        mirisdr_v4_t a0 = KERN[C] * mirisdr_v4_load(oc), a1 = KERN[C] * mirisdr_v4_load(oc + 4); \
        mirisdr_v4_t a2 = KERN[C] * mirisdr_v4_load(oc + 8), a3 = KERN[C] * mirisdr_v4_load(oc + 12); \
                                                                                        \
        for (i = 0; i < PAIRS; i++) {                                                   \
            const float g = KERN[2 * i];                                                \
            const float *lo = e + 2 * (m + i), *hi = e + 2 * (m + H / 2 - i);           \
                                                                                        \
            a0 += g * (mirisdr_v4_load(lo) + mirisdr_v4_load(hi));                      \
            a1 += g * (mirisdr_v4_load(lo + 4) + mirisdr_v4_load(hi + 4));              \
            a2 += g * (mirisdr_v4_load(lo + 8) + mirisdr_v4_load(hi + 8));              \
            a3 += g * (mirisdr_v4_load(lo + 12) + mirisdr_v4_load(hi + 12));            \
        }                                                                               \
        mirisdr_v4_store(io + 2 * m, a0);                                               \
        mirisdr_v4_store(io + 2 * m + 4, a1);                                           \
        mirisdr_v4_store(io + 2 * m + 8, a2);                                           \
        mirisdr_v4_store(io + 2 * m + 12, a3);                                          \
    }
#else
#define MIRISDR_HB_V4(KERN, PAIRS, H, C)
#endif

#define MIRISDR_HB_RUN(NAME, KERN, LEN)                                                 \
static int NAME (mirisdr_hb_t *h, float *io, int n)                                     \
{                                                                                       \
    enum { C = (LEN) / 2, H = (LEN) - 1, PAIRS = ((LEN) / 2 + 1) / 2 };                 \
    float *x, *e, *o;                                                                   \
    int i, k, m, out, pe, po;                                                           \
                                                                                        \
    if (n > h->cap && mirisdr_hb_size(h, n) < 0) return 0;                              \
    x = h->x; e = h->e; o = h->o;                                                       \
                                                                                        \
    memcpy(x + 2 * H, io, (size_t) n * 2 * sizeof(float));                              \
                                                                                        \
    /* outputs at t = H + skip, H + skip + 2, ... below H + n */                        \
    out = (n - h->skip + 1) / 2;                                                        \
    if (out < 0) out = 0;                                                               \
                                                                                        \
    /* the two phases, as many as the outputs need */                                   \
    pe = out + H / 2;                                                                   \
    po = out + C / 2;                                                                   \
    {                                                                                   \
        const float *xe = x + 2 * h->skip, *xo = x + 2 * (h->skip + 1);                 \
                                                                                        \
        for (k = 0; k < pe; k++) memcpy(e + 2 * k, xe + 4 * k, 2 * sizeof(float));     \
        for (k = 0; k < po; k++) memcpy(o + 2 * k, xo + 4 * k, 2 * sizeof(float));     \
    }                                                                                   \
                                                                                        \
    m = 0;                                                                              \
    MIRISDR_HB_V4(KERN, PAIRS, H, C)                                                    \
    for (; m < out; m++) {                                                              \
        float yi = KERN[C] * o[2 * (m + C / 2)], yq = KERN[C] * o[2 * (m + C / 2) + 1]; \
                                                                                        \
        for (i = 0; i < PAIRS; i++) {                                                   \
            const float g = KERN[2 * i];                                                \
                                                                                        \
            yi += g * (e[2 * (m + i)] + e[2 * (m + H / 2 - i)]);                        \
            yq += g * (e[2 * (m + i) + 1] + e[2 * (m + H / 2 - i) + 1]);                \
        }                                                                               \
        io[2 * m] = yi;                                                                 \
        io[2 * m + 1] = yq;                                                             \
    }                                                                                   \
                                                                                        \
    h->skip = (h->skip + 2 * out) - n;                                                  \
    memmove(x, x + 2 * n, (size_t) H * 2 * sizeof(float));                              \
                                                                                        \
    return out;                                                                         \
}

MIRISDR_HB_RUN(mirisdr_hb_run_long, mirisdr_hb_kernel, MIRISDR_HB_KERNEL_LEN)
MIRISDR_HB_RUN(mirisdr_hb_run_short, mirisdr_hb_short, MIRISDR_HB_SHORT_LEN)

static int mirisdr_hb_run (mirisdr_hb_t *h, float *io, int n)
{
    return (h->len == MIRISDR_HB_KERNEL_LEN) ? mirisdr_hb_run_long(h, io, n) : mirisdr_hb_run_short(h, io, n);
}

static void mirisdr_bb_free (mirisdr_dev_t *p)
{
    mirisdr_bb_t *b = p->bb;
    int i;

    if (!b) return;
    if (b->cnv) iqconverter_float_free(b->cnv);
    for (i = 0; i < MIRISDR_BB_STAGES_MAX; i++) {
        free(b->hb[i].x);
        free(b->hb[i].e);
        free(b->hb[i].o);
    }
    free(b->work);
    free(b->out);
    free(b);
    p->bb = NULL;
}

/* the filters' history only: after a restart the samples do not follow on */
static void mirisdr_bb_restart (mirisdr_dev_t *p)
{
    mirisdr_bb_t *b = p->bb;
    int i;

    if (!b) return;
    if (b->cnv) iqconverter_float_reset(b->cnv);
    for (i = 0; i < b->stages; i++) {
        memset(b->hb[i].x, 0, (size_t) (b->hb[i].len - 1) * 2 * sizeof(float));
        b->hb[i].skip = 0;
    }
    b->dc_i = b->dc_q = 0;
    b->rot = 0;
    b->have_carry = 0;
    b->out_n = 0;
    b->gq_n = 0;
    b->produced = b->sample;
}

/* a new read: the counts from 0 */
static void mirisdr_bb_start (mirisdr_dev_t *p)
{
    if (!p->bb) return;
    mirisdr_bb_restart(p);
    p->bb->sample = p->bb->produced = 0;
    p->bb->lost = 0;
    p->bb->lost_in = 0;
}

/* For the stream the plan chose (p->bb_*), from stream_regs; the counts carry on */
static int mirisdr_bb_setup (mirisdr_dev_t *p)
{
    mirisdr_bb_t *b, *old = p->bb;
    int i;

    if (p->bb_path == MIRISDR_BASEBAND_OFF) {
        mirisdr_bb_free(p);
        return 0;
    }

    if (!(b = calloc(1, sizeof *b))) goto failed;
    b->path = p->bb_path;
    b->stages = p->bb_stages;
    b->decim = (uint32_t) (p->bb_rate ? p->rate / p->bb_rate : 1);
    if (!b->decim) b->decim = 1;

    /* the band lies the IF below the LO: up by a quarter of the rate, or down with I
       and Q swapped. The converter's output of a single one is the right way round */
    b->sign = p->swap_iq ? -1 : 1;

    /* the converter is the last stage when none follow it */
    if (b->path == MIRISDR_BASEBAND_REAL &&
        !(b->cnv = b->stages ? iqconverter_float_create(mirisdr_hb_short, MIRISDR_HB_SHORT_LEN)
                             : iqconverter_float_create(mirisdr_hb_kernel, MIRISDR_HB_KERNEL_LEN))) goto failed;

    for (i = 0; i < b->stages; i++)
        if (mirisdr_hb_init(&b->hb[i], 4096, i == b->stages - 1) < 0) goto failed;

    b->dc_a = (float) (1.0 / (MIRISDR_BB_DC_TAU * (p->rate ? p->rate : 1)));

    if (old) {
        b->sample = b->produced = old->sample;
        b->lost = old->lost;
        b->lost_in = old->lost_in;
    }
    mirisdr_bb_free(p);
    p->bb = b;

    return 0;

failed:
    if (b) {
        p->bb = b;
        mirisdr_bb_free(p);
    }
    p->bb = old;
    mirisdr_bb_free(p);
    p->bb_path = MIRISDR_BASEBAND_OFF;
    fprintf(stderr, "libmirisdr: out of memory for the baseband filters\n");

    return -1;
}

static float *mirisdr_bb_work (mirisdr_bb_t *b, size_t floats)
{
    if (floats > b->work_cap) {
        float *w = realloc(b->work, floats * sizeof(float));
        if (!w) return NULL;
        b->work = w;
        b->work_cap = floats;
    }

    return b->work;
}

/* Both outputs: one pass from the integers to float with the DC taken off and the
   shift, a quarter turn a sample, as a pattern of four from where the last buffer
   left it. The DC estimate moves once a buffer toward the buffer's mean, so the
   loop has no chain from sample to sample; after the shift it would sit at a
   quarter of the rate */
#define MIRISDR_BB_SHIFT(T)                                                             \
static int mirisdr_bb_shift_##T (mirisdr_bb_t *b, const T *in, int n, float sc, float *w) \
{                                                                                       \
    int64_t si = 0, sq = 0;                                                             \
    unsigned r = b->rot;                                                                \
    float di, dq;                                                                       \
    int k;                                                                              \
                                                                                        \
    for (k = 0; k < n; k++) {                                                           \
        si += in[2 * k];                                                                \
        sq += in[2 * k + 1];                                                            \
    }                                                                                   \
    if (n) {                                                                            \
        double f = (double) b->dc_a * n;                                                \
        if (f > 1) f = 1;                                                               \
        b->dc_i += (float) (f * ((double) si * sc / n - b->dc_i));                      \
        b->dc_q += (float) (f * ((double) sq * sc / n - b->dc_q));                      \
    }                                                                                   \
    di = b->dc_i;                                                                       \
    dq = b->dc_q;                                                                       \
                                                                                        \
    for (k = 0; k < n && (r & 3); k++, r++)                                             \
        mirisdr_bb_rot1(w + 2 * k, in[2 * k] * sc - di, in[2 * k + 1] * sc - dq, b->sign, r); \
    if (b->sign > 0)                                                                    \
        for (; k + 4 <= n; k += 4, r += 4) {                                            \
            const T *p = in + 2 * k;                                                    \
            float *q = w + 2 * k;                                                       \
            q[0] = p[0] * sc - di;   q[1] = p[1] * sc - dq;                             \
            q[2] = dq - p[3] * sc;   q[3] = p[2] * sc - di;                             \
            q[4] = di - p[4] * sc;   q[5] = dq - p[5] * sc;                             \
            q[6] = p[7] * sc - dq;   q[7] = di - p[6] * sc;                             \
        }                                                                               \
    else                                                                                \
        for (; k + 4 <= n; k += 4, r += 4) {                                            \
            const T *p = in + 2 * k;                                                    \
            float *q = w + 2 * k;                                                       \
            q[0] = p[0] * sc - di;   q[1] = p[1] * sc - dq;                             \
            q[2] = p[3] * sc - dq;   q[3] = di - p[2] * sc;                             \
            q[4] = di - p[4] * sc;   q[5] = dq - p[5] * sc;                             \
            q[6] = dq - p[7] * sc;   q[7] = p[6] * sc - di;                             \
        }                                                                               \
    for (; k < n; k++, r++)                                                             \
        mirisdr_bb_rot1(w + 2 * k, in[2 * k] * sc - di, in[2 * k + 1] * sc - dq, b->sign, r); \
                                                                                        \
    b->rot = r;                                                                         \
    return n;                                                                           \
}

static inline void mirisdr_bb_rot1 (float *q, float vi, float vq, int sign, unsigned r)
{
    switch ((sign > 0 ? r : (0u - r)) & 3) {
    case 0: q[0] = vi;  q[1] = vq;  break;
    case 1: q[0] = -vq; q[1] = vi;  break;
    case 2: q[0] = -vi; q[1] = -vq; break;
    case 3: q[0] = vq;  q[1] = -vi; break;
    }
}

MIRISDR_BB_SHIFT(int16_t)
MIRISDR_BB_SHIFT(int8_t)

/* The converters' samples to output pairs in b->work, returns how many */
static int mirisdr_bb_process (mirisdr_dev_t *p, const unsigned char *buf, uint32_t len)
{
    mirisdr_bb_t *b = p->bb;
    const int s8 = p->format == MIRISDR_FORMAT_504_S8;
    const uint32_t vals = s8 ? len : len / 2;     /* int8 or int16 values */
    const float full = s8 ? 1.0f / 128.0f : 1.0f / 32768.0f;
    float *w;
    uint32_t i;
    int n, s;

    if (!(w = mirisdr_bb_work(b, vals + 2))) return 0;

    if (b->path == MIRISDR_BASEBAND_COMPLEX) {
        n = s8 ? mirisdr_bb_shift_int8_t(b, (const int8_t *) buf, (int) vals / 2, full, w)
               : mirisdr_bb_shift_int16_t(b, (const int16_t *) buf, (int) vals / 2, full, w);
    } else {
        /* one output carries the band at half the amplitude both do: twice the scale,
           so a signal comes out at the same level either way */
        const float sc = (b->path == MIRISDR_BASEBAND_REAL ? 2.0f : 1.0f) * full;

        /* the odd sample of the last buffer first */
        i = 0;
        if (b->path == MIRISDR_BASEBAND_REAL && b->have_carry) {
            w[0] = b->carry;
            i = 1;
        }
        if (s8) {
            const int8_t *in = (const int8_t *) buf;
            for (uint32_t k = 0; k < vals; k++) w[i + k] = in[k] * sc;
        } else {
            const int16_t *in = (const int16_t *) buf;
            for (uint32_t k = 0; k < vals; k++) w[i + k] = in[k] * sc;
        }
        n = (int) (vals + i);

        if (b->path == MIRISDR_BASEBAND_REAL) {
            b->have_carry = n & 1;
            if (b->have_carry) b->carry = w[--n];
            iqconverter_float_process(b->cnv, w, n);
        }
        n /= 2;
    }

    for (s = 0; s < b->stages; s++) n = mirisdr_hb_run(&b->hb[s], w, n);

    return n;
}

/* One output buffer to the callback, with its info in output samples */
static void mirisdr_bb_call (mirisdr_dev_t *p, float *iq, uint32_t n)
{
    mirisdr_bb_t *b = p->bb;
    mirisdr_buffer_info_t *in = &p->cb_info;
    uint64_t start = b->sample, end = start + n;
    int k = 0, j;

    memset(in, 0, sizeof *in);
    in->sample = start;
    in->index = start + (uint64_t) (b->lost + 0.5);
    in->adc = MIRISDR_IQ_BOTH;
    in->rate = p->bb_rate;
    in->type = MIRISDR_SAMPLE_F32;

    while (k < b->gq_n && b->gq[k].at < end) {
        if (in->gaps_len < MIRISDR_GAPS_MAX) {
            mirisdr_gap_t *g = &in->gaps[in->gaps_len++];

            g->offset = b->gq[k].at > start ? (uint32_t) (b->gq[k].at - start) : 0;
            g->samples = b->gq[k].samples;
            g->filled = (uint32_t) b->gq[k].filled;
        }
        in->gap_samples += b->gq[k].samples;
        k++;
    }
    for (j = k; j < b->gq_n; j++) b->gq[j - k] = b->gq[j];
    b->gq_n -= k;

    b->sample = end;
    p->cb((unsigned char *) iq, n * 2 * sizeof(float), p->cb_ctx);
}

/* From mirisdr_cb_call, with the converters' buffer and its info in p->cb_info */
static void mirisdr_bb_feed (mirisdr_dev_t *p, unsigned char *buf, uint32_t len)
{
    mirisdr_bb_t *b = p->bb;
    const mirisdr_buffer_info_t *in = &p->cb_info;
    uint64_t lost_in = in->index - in->sample;
    uint32_t k, n, at, chunk;
    float *w;

    /* the unfilled gaps before this buffer, as output samples */
    if (lost_in > b->lost_in) b->lost += (double) (lost_in - b->lost_in) / b->decim;
    b->lost_in = lost_in;

    /* its gaps, where they come out */
    for (k = 0; k < in->gaps_len && b->gq_n < MIRISDR_BB_GAPQ; k++) {
        b->gq[b->gq_n].at = b->produced + in->gaps[k].offset / b->decim;
        b->gq[b->gq_n].samples = (in->gaps[k].samples + b->decim - 1) / b->decim;
        b->gq[b->gq_n].filled = in->gaps[k].filled / b->decim;
        b->gq_n++;
    }

    n = (uint32_t) mirisdr_bb_process(p, buf, len);
    w = b->work;
    b->produced += n;

    /* buffers as they come */
    if (!p->user_out_len || p->user_out_len < 2 * sizeof(float)) {
        if (n) mirisdr_bb_call(p, w, n);
        return;
    }

    /* or of the size asked for */
    if (!b->out || b->out_cap != p->user_out_len / (2 * sizeof(float))) {
        float *o = realloc(b->out, p->user_out_len);
        if (!o) return;
        b->out = o;
        b->out_cap = (uint32_t) (p->user_out_len / (2 * sizeof(float)));
        b->out_n = 0;
    }

    for (at = 0; at < n; at += chunk) {
        chunk = b->out_cap - b->out_n;
        if (chunk > n - at) chunk = n - at;

        if (!b->out_n && chunk == b->out_cap) {
            mirisdr_bb_call(p, w + 2 * at, chunk);
            continue;
        }
        memcpy(b->out + 2 * b->out_n, w + 2 * at, (size_t) chunk * 2 * sizeof(float));
        b->out_n += chunk;
        if (b->out_n == b->out_cap) {
            mirisdr_bb_call(p, b->out, b->out_cap);
            b->out_n = 0;
        }
    }
}
