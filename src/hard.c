/*
 * Copyright (C) 2013 by Miroslav Slugen <thunder.m@email.cz
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "hard.h"




/* samples in each 1 kB block, which sets what a rate costs the engine */
static uint32_t mirisdr_format_spp (int format)
{
	switch (format)
	{
	case MIRISDR_FORMAT_252_S16:      return 252;
	case MIRISDR_FORMAT_336_S16:      return 336;
	case MIRISDR_FORMAT_384_S16:      return 384;
	case MIRISDR_FORMAT_504_S16:
	case MIRISDR_FORMAT_504_S8:
	case MIRISDR_FORMAT_504_REAL_S16: return 504;
	case MIRISDR_FORMAT_672_REAL_S16: return 672;
	case MIRISDR_FORMAT_768_REAL_S16: return 768;
	}

	return 252;
}

/* Every divider whose VCO is in range, nearest the middle first. */
static unsigned mirisdr_pll_candidates (uint32_t pll_rate, uint64_t *out)
{
	uint64_t want = (MIRISDR_VCO_MIN + MIRISDR_VCO_MAX) / 2, i;
	unsigned n = 0, a, b;

	for (i = 4; i <= 16; i += 2)
	{
		uint64_t v = (uint64_t) pll_rate * i * 12;

		if (v > MIRISDR_VCO_MAX) break;
		if (v < MIRISDR_VCO_MIN) continue;

		out[n++] = i;
	}

	for (a = 0; a < n; a++)
		for (b = a + 1; b < n; b++)
		{
			uint64_t va = (uint64_t) pll_rate * out[a] * 12;
			uint64_t vb = (uint64_t) pll_rate * out[b] * 12;
			uint64_t da = (va > want) ? (va - want) : (want - va);
			uint64_t db = (vb > want) ? (vb - want) : (want - vb);

			if (db < da) { uint64_t t = out[a]; out[a] = out[b]; out[b] = t; }
		}

	return n;
}

/* nastavení parametrů které vyžadují restart */
/* parameters that require restart */
/* The converters a stream captures, MIRISDR_IQ_* */
static int mirisdr_stream_adc (mirisdr_dev_t *p)
{
    switch (p->format)
    {
    case MIRISDR_FORMAT_504_REAL_S16:
    case MIRISDR_FORMAT_672_REAL_S16:
    case MIRISDR_FORMAT_768_REAL_S16:
        return p->swap_iq ? MIRISDR_IQ_ONLY_Q : MIRISDR_IQ_ONLY_I;
    default:
        return MIRISDR_IQ_BOTH;
    }
}

/* ------------------------------------------------------------------ */
/* the stream config                                                  */
/* ------------------------------------------------------------------ */

#define MIRISDR_STREAM_ADJUST   1       /* the single setters: clamp and warn rather than refuse */
#define MIRISDR_STREAM_FORCE    2       /* write it all even if nothing changed */

static const struct { const char *name; int format; } mirisdr_formats[] = {
	{ "252_S16",      MIRISDR_FORMAT_252_S16 },
	{ "336_S16",      MIRISDR_FORMAT_336_S16 },
	{ "384_S16",      MIRISDR_FORMAT_384_S16 },
	{ "504_S16",      MIRISDR_FORMAT_504_S16 },
	{ "504_S8",       MIRISDR_FORMAT_504_S8 },
	{ "504_REAL_S16", MIRISDR_FORMAT_504_REAL_S16 },
	{ "672_REAL_S16", MIRISDR_FORMAT_672_REAL_S16 },
	{ "768_REAL_S16", MIRISDR_FORMAT_768_REAL_S16 },
};

/* The isochronous settings differ in how many 1kB slots they reserve in the microframe:
 * ISOC=3, ISOC1=1, ISOC2=2 (2 requires custom fw) */
static const struct { const char *name; int transfer; uint8_t alt; } mirisdr_transfers[] = {
	{ "BULK",  MIRISDR_TRANSFER_BULK, 3 },
	{ "ISOC",  MIRISDR_TRANSFER_ISOC, 1 },
	{ "ISOC1", MIRISDR_TRANSFER_ISOC, 2 },
	{ "ISOC2", MIRISDR_TRANSFER_ISOC, 4 },
};

/* ISOC is more stable but works only on Unix systems */
#if !defined (_WIN32) || defined(__MINGW32__)
#define MIRISDR_TRANSFER_DEFAULT        "ISOC"
#else
#define MIRISDR_TRANSFER_DEFAULT        "BULK"
#endif

static const char *mirisdr_decim_names[] = { "AUTO", "OFF", "ON" };     /* by MIRISDR_DECIMATION_BYPASS_* */

static int mirisdr_format_real (int format)
{
	return (format == MIRISDR_FORMAT_504_REAL_S16) || (format == MIRISDR_FORMAT_672_REAL_S16) ||
	       (format == MIRISDR_FORMAT_768_REAL_S16);
}

static const char *mirisdr_format_name (int format)
{
	unsigned i;

	for (i = 0; i < sizeof mirisdr_formats / sizeof mirisdr_formats[0]; i++)
		if (mirisdr_formats[i].format == format) return mirisdr_formats[i].name;

	return "";
}

/* the register 7 word of a format, without the swap, decimation and burst bits */
static uint32_t mirisdr_format_reg7 (int format)
{
	switch (format)
	{
	case MIRISDR_FORMAT_252_S16:      return 0x000014;
	case MIRISDR_FORMAT_336_S16:      return 0x000005;
	case MIRISDR_FORMAT_384_S16:      return 0x000025;
	case MIRISDR_FORMAT_504_S16:
	case MIRISDR_FORMAT_504_S8:       return 0x000c14;
	case MIRISDR_FORMAT_504_REAL_S16: return 0x000414;
	case MIRISDR_FORMAT_672_REAL_S16: return 0x000405;
	case MIRISDR_FORMAT_768_REAL_S16: return 0x000425;
	}

	return 0x000014;
}

/* the register 3 AGC field of a format */
static uint32_t mirisdr_format_agc (int format)
{
	switch (format)
	{
	case MIRISDR_FORMAT_336_S16:
	case MIRISDR_FORMAT_672_REAL_S16: return 0x05;
	case MIRISDR_FORMAT_384_S16:
	case MIRISDR_FORMAT_768_REAL_S16: return 0x09;
	case MIRISDR_FORMAT_504_S16:
	case MIRISDR_FORMAT_504_S8:       return 0x0d;
	default:                          return 0x01;
	}
}

typedef struct mirisdr_stream_plan
{
	mirisdr_stream_config_t cfg;    /* as asked, the strings canonical, the rate reached */
	uint32_t rate, pll_rate, cap;
	int format, format_auto, decimation_bypass, decim, transfer, swap;
	uint8_t alt;
	uint64_t cand[8];
	unsigned ncand;
} mirisdr_stream_plan_t;

static int mirisdr_stream_plan (mirisdr_dev_t *p, const mirisdr_stream_config_t *c, int flags,
                                mirisdr_stream_plan_t *pl)
{
	int adjust = flags & MIRISDR_STREAM_ADJUST;
	uint32_t rate_min, rate_max, spp;
	uint64_t most;
	unsigned i;

	pl->cfg = *c;

	/* format */
	if (!c->format || !strcmp(c->format, "AUTO"))
	{
		pl->format_auto = MIRISDR_FORMAT_AUTO_ON;
		pl->cfg.format = "AUTO";
	}
	else if (!strcmp(c->format, "AUTO_REAL"))
	{
		pl->format_auto = MIRISDR_FORMAT_AUTO_REAL;
		pl->cfg.format = "AUTO_REAL";
	}
	else
	{
		for (i = 0; (i < sizeof mirisdr_formats / sizeof mirisdr_formats[0]) &&
		            strcmp(c->format, mirisdr_formats[i].name); i++);

		if (i == sizeof mirisdr_formats / sizeof mirisdr_formats[0])
		{
			fprintf(stderr, "unsupported format: %s\n", c->format);
			return -1;
		}

		pl->format_auto = MIRISDR_FORMAT_AUTO_OFF;
		pl->format = mirisdr_formats[i].format;
		pl->cfg.format = mirisdr_formats[i].name;
	}

	/* transfer */
	if (!c->transfer) pl->cfg.transfer = MIRISDR_TRANSFER_DEFAULT;

	for (i = 0; (i < sizeof mirisdr_transfers / sizeof mirisdr_transfers[0]) &&
	            strcmp(pl->cfg.transfer, mirisdr_transfers[i].name); i++);

	if (i == sizeof mirisdr_transfers / sizeof mirisdr_transfers[0])
	{
		fprintf(stderr, "unsupported transfer type: %s\n", pl->cfg.transfer);
		return -1;
	}

	pl->transfer = mirisdr_transfers[i].transfer;
	pl->alt = mirisdr_transfers[i].alt;
	pl->cfg.transfer = mirisdr_transfers[i].name;

	/* decimation */
	if (!c->decimation_bypass) pl->decimation_bypass = MIRISDR_DECIMATION_BYPASS_AUTO;
	else
	{
		for (i = 0; (i < 3) && strcmp(c->decimation_bypass, mirisdr_decim_names[i]); i++);

		if (i == 3)
		{
			fprintf(stderr, "unsupported decimation bypass: %s\n", c->decimation_bypass);
			return -1;
		}

		pl->decimation_bypass = (int) i;
	}

	pl->cfg.decimation_bypass = mirisdr_decim_names[pl->decimation_bypass];

	pl->swap = pl->cfg.swap_iq = c->swap_iq ? 1 : 0;
	pl->cfg.gap_fill = c->gap_fill ? 1 : 0;

	/* rate, within what the decimation setting allows */
	pl->rate = c->rate;
	pl->decim = (pl->decimation_bypass == MIRISDR_DECIMATION_BYPASS_ON) ||
	            ((pl->decimation_bypass == MIRISDR_DECIMATION_BYPASS_AUTO) &&
	             (pl->rate > MIRISDR_DECIMATION_AUTO_RATE));

	rate_min = pl->decim ? 2 * MIRISDR_SAMPLE_RATE_MIN : MIRISDR_SAMPLE_RATE_MIN;
	rate_max = pl->decim ? 2 * MIRISDR_SAMPLE_RATE_MAX : MIRISDR_SAMPLE_RATE_MAX;

	if ((pl->rate < rate_min) || (pl->rate > rate_max))
	{
		if (!adjust)
		{
			fprintf(stderr, "rate %u is outside %u to %u sps\n", pl->rate, rate_min, rate_max);
			return -1;
		}

		pl->rate = (pl->rate < rate_min) ? rate_min : rate_max;
	}

	pl->cap = (pl->transfer == MIRISDR_TRANSFER_BULK) ? MIRISDR_BULK_CAPACITY
	                                                  : 1024 * mirisdr_alt_burst(pl->alt) * 8000;

	/* the automatic choice: the least dense packing that fits */
	if (pl->format_auto != MIRISDR_FORMAT_AUTO_OFF)
	{
		static const struct { uint32_t spp; int format; } complex[] = {
			{ 252, MIRISDR_FORMAT_252_S16 }, { 336, MIRISDR_FORMAT_336_S16 },
			{ 384, MIRISDR_FORMAT_384_S16 }, { 504, MIRISDR_FORMAT_504_S16 } },
		real[] = {
			{ 504, MIRISDR_FORMAT_504_REAL_S16 }, { 672, MIRISDR_FORMAT_672_REAL_S16 },
			{ 768, MIRISDR_FORMAT_768_REAL_S16 } };
		int is_real = pl->format_auto == MIRISDR_FORMAT_AUTO_REAL;
		unsigned n = is_real ? 3 : 4;

		for (i = 0; i < n; i++)
		{
			spp = is_real ? real[i].spp : complex[i].spp;
			pl->format = is_real ? real[i].format : complex[i].format;
			if ((uint64_t) pl->rate * 1024 / spp <= pl->cap) break;
		}

		if (i == n)
		{
			fprintf(stderr, "rate %u needs %lu B/s, more than the %u B/s this mode supports\n", pl->rate,
			        (long unsigned) ((uint64_t) pl->rate * 1024 / spp), pl->cap);

			if (!adjust) return -1;
		}
	}
	else if (!adjust && ((uint64_t) pl->rate * 1024 / mirisdr_format_spp(pl->format) > pl->cap))
	{
		fprintf(stderr, "rate %u in %s needs %lu B/s, more than the %u B/s this mode supports\n", pl->rate,
		        pl->cfg.format, (long unsigned) ((uint64_t) pl->rate * 1024 / mirisdr_format_spp(pl->format)),
		        pl->cap);
		return -1;
	}

	/* The format is settled now, so the engine's own limit can be applied: after the
	   automatic choice, which needs the rate, and before register 7, since clamping can
	   drop the rate back under the decimation threshold and change that bit. */
	spp = mirisdr_format_spp(pl->format);
	most = MIRISDR_ENGINE_BLOCK_RATE * spp / 1024;

	if ((uint64_t) pl->rate > most)
	{
		fprintf(stderr, "rate %u needs %lu B/s of blocks, more than the engine's %lu%s%lu\n", pl->rate,
		        (long unsigned) ((uint64_t) pl->rate * 1024 / spp), (long unsigned) MIRISDR_ENGINE_BLOCK_RATE,
		        adjust ? ", using " : "", adjust ? (long unsigned) most : 0UL);

		if (!adjust) return -1;

		pl->rate = (uint32_t) most;

		/* the clamp only ever lowers it, which can turn decimation off */
		pl->decim = (pl->decimation_bypass == MIRISDR_DECIMATION_BYPASS_ON) ||
		            ((pl->decimation_bypass == MIRISDR_DECIMATION_BYPASS_AUTO) &&
		             (pl->rate > MIRISDR_DECIMATION_AUTO_RATE));

		if (pl->rate < (pl->decim ? 2 * MIRISDR_SAMPLE_RATE_MIN : MIRISDR_SAMPLE_RATE_MIN))
			pl->rate = pl->decim ? 2 * MIRISDR_SAMPLE_RATE_MIN : MIRISDR_SAMPLE_RATE_MIN;
	}

	pl->pll_rate = pl->decim ? pl->rate / 2 : pl->rate;
	pl->cfg.rate = pl->rate;

	/* The sample clock: N of at least 2, or the rate cannot be switched back (see the
	   rate limits in hard.h) */
	if (!(pl->ncand = mirisdr_pll_candidates(pl->pll_rate, pl->cand)))
	{
		fprintf(stderr, "no PLL divider puts the VCO in range for %u sps\n", pl->rate);
		return -1;
	}

	if (!adjust)
	{
		/* what the callback is handed changes shape between real and complex */
		if ((p->async_status == MIRISDR_ASYNC_RUNNING) &&
		    (mirisdr_format_real(pl->format) != mirisdr_format_real(p->format)))
		{
			fprintf(stderr, "cannot switch between a real and a complex format while streaming\n");
			return -1;
		}

		/* the tune switched off the output this would capture */
		if (mirisdr_format_real(pl->format) && (p->tune_iq != MIRISDR_IQ_BOTH) &&
		    ((pl->swap ? MIRISDR_IQ_ONLY_Q : MIRISDR_IQ_ONLY_I) != p->tune_iq))
		{
			fprintf(stderr, "the tune switched off the tuner output this would capture\n");
			return -1;
		}
	}

	return 0;
}

static void mirisdr_stream_result_of (uint32_t rate, int format, int swap, int decim, uint32_t cap,
                                      mirisdr_stream_result_t *res)
{
	res->rate = rate;
	res->format = mirisdr_format_name(format);
	res->adc = !mirisdr_format_real(format) ? MIRISDR_IQ_BOTH : swap ? MIRISDR_IQ_ONLY_Q : MIRISDR_IQ_ONLY_I;
	res->decimation_bypassed = decim;
	res->usb_bytes = (uint32_t) ((uint64_t) rate * 1024 / mirisdr_format_spp(format));
	res->usb_capacity = cap;
}

static int mirisdr_stream_send (mirisdr_dev_t *p, const mirisdr_stream_plan_t *pl)
{
	int streaming = 0;
	uint32_t reg3 = 0, reg4 = 0, reg7;
	uint64_t i, vco, n, fract;
	unsigned try;

	/* registers change, so the stream stops and starts again around it */
	if (p->async_status == MIRISDR_ASYNC_RUNNING)
	{
		streaming = 1;

		if ((mirisdr_stop_async(p) < 0) || (mirisdr_adc_stop(p) < 0)) goto failed;
	}

	p->rate = pl->rate;
	p->format = pl->format;
	p->format_auto = pl->format_auto;
	p->decimation_bypass = pl->decimation_bypass;
	p->decim_on = pl->decim;
	p->swap_iq = pl->swap;
	p->transfer = pl->transfer;
	p->alt_setting = pl->alt;
	p->gap_fill = pl->cfg.gap_fill;
	p->stream = pl->cfg;

	reg7 = mirisdr_format_reg7(pl->format) | (pl->swap ? (1 << 9) : 0) | (pl->decim ? (1 << 3) : 0) |
	       ((mirisdr_alt_burst(pl->alt) - 1) << 6);
	mirisdr_write_reg(p, 0x07, reg7);
	p->addr_step = mirisdr_format_spp(pl->format);
	p->addr = p->addr_step + 2;

	i = pl->cand[0];
	vco = (uint64_t) pl->pll_rate * i * 12;

	/* N is at least 4 here */
	n = vco / 48000000UL;
	fract = 0x200000UL * (vco % 48000000UL) / 48000000UL;
#if MIRISDR_DEBUG >= 1
	fprintf(stderr, "rate: %u, pll: %u, vco: %lu (%lu), n: %lu, fraction: %lu\n", pl->rate, pl->pll_rate,
	        (long unsigned) vco, (long unsigned) (i / 2) - 1, (long unsigned) n, (long unsigned) fract);
#endif

	reg3 |= 3 << 0;                                 /* ?? */
	reg3 |= (0x07 & (i / 2 - 1)) << 2;              /* the divider */
	reg3 |= (0x01 & (fract >> 20)) << 7;            /* +0.5 */
	reg3 |= (0x0f & n) << 8;                        /* the main range */
	reg3 |= mirisdr_format_agc(pl->format) << 12;
	reg3 |= 1 << 16;                                /* ?? */

	/* The real formats digitise one converter, so gate the other off: bit 5
	   powers down I, bit 6 Q.  reg7 bit 9 chooses which one is read. */
	if (mirisdr_format_real(pl->format)) reg3 |= pl->swap ? (1 << 5) : (1 << 6);

	reg4 |= (0xfffff & fract) << 0;

	mirisdr_write_reg(p, 0x04, reg4);
	mirisdr_write_reg(p, 0x03, reg3);

	/* The part reports its VCO band selection in read index 0, low nibble.
	   Extremes: 0xF when it wants more capacitance than the bank has, 0 when it has
	   none left to remove. We try to avoid the edges to increase tolerance to
	   variation */
	for (try = 1; (pl->ncand > 1) && (try <= pl->ncand); try++)
	{
		uint8_t rd[4];
		uint64_t alt, v;

		/* Let the capacitance search settle before asking what it chose. */
#if defined (_WIN32) && !defined(__MINGW32__)
		Sleep(1);               /* 1ms */
#else
		usleep(200);
#endif

		if (mirisdr_read_reg(p, 0, rd, sizeof(rd)) != (int) sizeof(rd)) break;

		rd[0] &= 0x0f;

		if ((rd[0] > 1) && (rd[0] < 0x0e)) break;

		/* Once the list is exhausted nothing was better than where it started,
		   so put the first choice back rather than leave the last one tried. */
		alt = (try < pl->ncand) ? pl->cand[try] : pl->cand[0];
		v = (uint64_t) pl->pll_rate * alt * 12;
		n = v / 48000000UL;
		fract = 0x200000UL * (v % 48000000UL) / 48000000UL;

		reg3 = (reg3 & ~0xf9cUL)
		     | ((0x07 & (uint32_t) (alt / 2 - 1)) << 2)
		     | ((0x01 & (uint32_t) (fract >> 20)) << 7)
		     | ((0x0f & (uint32_t) n) << 8);
		reg4 = (uint32_t) (fract & 0xfffff);

#if MIRISDR_DEBUG >= 1
		fprintf(stderr, "vco band %X at %lu, trying %lu\n", rd[0],
		        (long unsigned int) vco, (long unsigned int) v);
#endif
		vco = v;

		mirisdr_write_reg(p, 0x04, reg4);
		mirisdr_write_reg(p, 0x03, reg3);

		if (try == pl->ncand) break;
	}

	if (streaming && (mirisdr_start_async(p) < 0)) goto failed;

	return 0;

failed:
	return -1;
}

static int mirisdr_stream_apply (mirisdr_dev_t *p, const mirisdr_stream_config_t *c, int flags,
                                 mirisdr_stream_result_t *res)
{
	mirisdr_stream_plan_t pl;

	if (!p || !c) return -1;
	if (mirisdr_stream_plan(p, c, flags, &pl) < 0) return -1;

	/* nothing the hardware holds changes: no restart */
	if (!(flags & MIRISDR_STREAM_FORCE) && (pl.rate == p->rate) && (pl.format == (int) p->format) &&
	    (pl.decim == p->decim_on) && (pl.swap == p->swap_iq) && (pl.alt == p->alt_setting))
	{
		p->format_auto = pl.format_auto;
		p->decimation_bypass = pl.decimation_bypass;
		p->transfer = pl.transfer;
		p->gap_fill = pl.cfg.gap_fill;
		p->stream = pl.cfg;
	}
	else if (mirisdr_stream_send(p, &pl) < 0) return -1;

	if (res) mirisdr_stream_result_of(p->rate, p->format, p->swap_iq, p->decim_on, pl.cap, res);

	return 0;
}

int mirisdr_set_hard(mirisdr_dev_t *p)
{
	return mirisdr_stream_apply(p, &p->stream, MIRISDR_STREAM_ADJUST | MIRISDR_STREAM_FORCE, NULL);
}

void mirisdr_stream_config_default (mirisdr_stream_config_t *cfg)
{
	if (!cfg) return;

	memset(cfg, 0, sizeof *cfg);
	cfg->rate = DEFAULT_RATE;
}

int mirisdr_set_stream (mirisdr_dev_t *p, const mirisdr_stream_config_t *cfg, mirisdr_stream_result_t *res)
{
	return mirisdr_stream_apply(p, cfg, 0, res);
}

int mirisdr_stream_check (mirisdr_dev_t *p, const mirisdr_stream_config_t *cfg, mirisdr_stream_result_t *res)
{
	mirisdr_stream_plan_t pl;

	if (!p || !cfg) return -1;
	if (mirisdr_stream_plan(p, cfg, 0, &pl) < 0) return -1;

	if (res) mirisdr_stream_result_of(pl.rate, pl.format, pl.swap, pl.decim, pl.cap, res);

	return 0;
}

int mirisdr_get_stream (mirisdr_dev_t *p, mirisdr_stream_config_t *cfg, mirisdr_stream_result_t *res)
{
	if (!p) return -1;

	if (cfg) *cfg = p->stream;
	if (res) mirisdr_stream_result_of(p->rate, p->format, p->swap_iq, p->decim_on,
	                                  (p->transfer == MIRISDR_TRANSFER_BULK) ? MIRISDR_BULK_CAPACITY :
	                                  1024 * mirisdr_alt_burst(p->alt_setting) * 8000, res);

	return 0;
}

/* ------------------------------------------------------------------ */
/* keep the old API                                                   */
/* ------------------------------------------------------------------ */

int mirisdr_set_sample_rate(mirisdr_dev_t *p, uint32_t rate)
{
	mirisdr_stream_config_t c;
	int r;

	if (!p) return -1;

	c = p->stream;
	c.rate = rate;
	r = mirisdr_stream_apply(p, &c, MIRISDR_STREAM_ADJUST | MIRISDR_STREAM_FORCE, NULL);

	/* the range depends on the decimation bypass, report what was reached */
	if (p->rate != rate) fprintf(stderr, "can't set rate %u, using %u\n", rate, p->rate);

	return r;
}

int mirisdr_set_sample_format(mirisdr_dev_t *p, const char *v)
{
	mirisdr_stream_config_t c;

	if (!p || !v) return -1;

	c = p->stream;
	c.format = v;

	return mirisdr_stream_apply(p, &c, MIRISDR_STREAM_ADJUST | MIRISDR_STREAM_FORCE, NULL);
}

int mirisdr_set_decimation_bypass(mirisdr_dev_t *p, const char *v)
{
	mirisdr_stream_config_t c;

	if (!p || !v) return -1;

	c = p->stream;
	c.decimation_bypass = v;

	return mirisdr_stream_apply(p, &c, MIRISDR_STREAM_ADJUST | MIRISDR_STREAM_FORCE, NULL);
}

int mirisdr_set_swap_iq(mirisdr_dev_t *p, int swap)
{
	mirisdr_stream_config_t c;

	if (!p) return -1;

	c = p->stream;
	c.swap_iq = swap;

	return mirisdr_stream_apply(p, &c, MIRISDR_STREAM_ADJUST | MIRISDR_STREAM_FORCE, NULL);
}

uint32_t mirisdr_get_sample_rate(mirisdr_dev_t *p)
{
	return p->rate;
}

const char *mirisdr_get_decimation_bypass(mirisdr_dev_t *p)
{
	switch (p->decimation_bypass)
	{
	case MIRISDR_DECIMATION_BYPASS_ON:
		return "ON";
	case MIRISDR_DECIMATION_BYPASS_OFF:
		return "OFF";
	default:
		return "AUTO";
	}
}

int mirisdr_get_swap_iq(mirisdr_dev_t *p)
{
	return p->swap_iq;
}

const char *mirisdr_get_sample_format(mirisdr_dev_t *p)
{
	if (p->format_auto == MIRISDR_FORMAT_AUTO_ON) {
		return "AUTO";
	}

	if (p->format_auto == MIRISDR_FORMAT_AUTO_REAL) {
		return "AUTO_REAL";
	}

	return mirisdr_get_sample_format_selected(p);
}

const char *mirisdr_get_sample_format_selected(mirisdr_dev_t *p)
{
	switch (p->format)
	{
	case MIRISDR_FORMAT_252_S16:
		return "252_S16";
	case MIRISDR_FORMAT_336_S16:
		return "336_S16";
	case MIRISDR_FORMAT_384_S16:
		return "384_S16";
	case MIRISDR_FORMAT_504_S16:
		return "504_S16";
	case MIRISDR_FORMAT_504_S8:
		return "504_S8";
	case MIRISDR_FORMAT_504_REAL_S16:
		return "504_REAL_S16";
	case MIRISDR_FORMAT_672_REAL_S16:
		return "672_REAL_S16";
	case MIRISDR_FORMAT_768_REAL_S16:
		return "768_REAL_S16";
	}

	return "";
}
