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


static uint32_t mirisdr_capacity(mirisdr_dev_t *p)
{
	if (p->transfer == MIRISDR_TRANSFER_BULK)
		return MIRISDR_BULK_CAPACITY;

	return 1024 * mirisdr_burst(p) * 8000;
}

static int mirisdr_format_fits(mirisdr_dev_t *p, uint32_t spp, uint32_t cap)
{
	return ((uint64_t) p->rate * 1024 / spp) <= cap;
}

/* nastavení parametrů které vyžadují restart */
/* parameters that require restart */
int mirisdr_set_hard(mirisdr_dev_t *p)
{
	int streaming = 0;
	uint32_t reg3 = 0, reg4 = 0, swap, burst, decim, pll_rate, rate_min, rate_max;
	uint64_t i, vco, n, fract;

	/* při změně registrů musíme zastavit streamování */
	/* at a registry change we must stop streaming */
	if (p->async_status == MIRISDR_ASYNC_RUNNING)
	{
		streaming = 1;

		if ((mirisdr_stop_async(p) < 0) || (mirisdr_adc_stop(p) < 0)) {
			goto failed;
		}
	}

	swap = p->swap_iq ? (1 << 9) : 0;

	burst = (mirisdr_burst(p) - 1) << 6;

	decim = ((p->decimation_bypass == MIRISDR_DECIMATION_BYPASS_ON) ||
	         ((p->decimation_bypass == MIRISDR_DECIMATION_BYPASS_AUTO) &&
	          (p->rate > MIRISDR_DECIMATION_AUTO_RATE))) ? (1 << 3) : 0;

	rate_min = decim ? 2 * MIRISDR_SAMPLE_RATE_MIN : MIRISDR_SAMPLE_RATE_MIN;
	rate_max = decim ? 2 * MIRISDR_SAMPLE_RATE_MAX : MIRISDR_SAMPLE_RATE_MAX;

	/* omezení rozsahu */
	/* limit the scope of */
	if (p->rate > rate_max)
	{
		p->rate = rate_max;
	}
	else if (p->rate < rate_min)
	{
		p->rate = rate_min;
	}

	pll_rate = decim ? p->rate / 2 : p->rate;

	/* automatická volba formátu */
	if (p->format_auto != MIRISDR_FORMAT_AUTO_OFF)
	{
		uint32_t cap = mirisdr_capacity(p);

		if (p->format_auto == MIRISDR_FORMAT_AUTO_REAL)
		{
			if (mirisdr_format_fits(p, 504, cap)) {
				p->format = MIRISDR_FORMAT_504_REAL_S16;
			} else if (mirisdr_format_fits(p, 672, cap)) {
				p->format = MIRISDR_FORMAT_672_REAL_S16;
			} else {
				p->format = MIRISDR_FORMAT_768_REAL_S16;
			}
		}
		else
		{
			if (mirisdr_format_fits(p, 252, cap)) {
				p->format = MIRISDR_FORMAT_252_S16;
			} else if (mirisdr_format_fits(p, 336, cap)) {
				p->format = MIRISDR_FORMAT_336_S16;
			} else if (mirisdr_format_fits(p, 384, cap)) {
				p->format = MIRISDR_FORMAT_384_S16;
			} else {
				p->format = MIRISDR_FORMAT_504_S16;
			}
		}

		if (!mirisdr_format_fits(p, p->format_auto == MIRISDR_FORMAT_AUTO_REAL ? 768 : 504, cap))
			fprintf(stderr, "rate %u needs %lu B/s, more than the %u B/s this mode supports\n",
			        p->rate,
			        (long unsigned int) ((uint64_t) p->rate * 1024 /
			                             (p->format_auto == MIRISDR_FORMAT_AUTO_REAL ? 768 : 504)),
			        cap);
	}

	/* typ forámtu a šířka pásma */
	/* format type and bandwidth */
	switch (p->format)
	{
	case MIRISDR_FORMAT_252_S16:
		/* maximum rate 6.048 Msps | 24.576 MB/s | 196.608 Mbit/s  */
#if MIRISDR_DEBUG >= 1
		fprintf( stderr, "format: 252\n");
#endif
		mirisdr_write_reg(p, 0x07, 0x000014 | swap | decim | burst);
		p->addr = 252 + 2;
		p->addr_step = 252;
		break;
	case MIRISDR_FORMAT_336_S16:
		/* maximum rate 8.064 Msps | 24.576 MB/s | 196.608 Mbit/s */
#if MIRISDR_DEBUG >= 1
		fprintf( stderr, "format: 336\n");
#endif
		mirisdr_write_reg(p, 0x07, 0x000005 | swap | decim | burst);
		p->addr = 336 + 2;
		p->addr_step = 336;
		break;
	case MIRISDR_FORMAT_384_S16:
		/* maximum rate 9.216 Msps | 24.576 MB/s | 196.608 Mbit/s */
#if MIRISDR_DEBUG >= 1
		fprintf( stderr, "format: 384\n");
#endif
		mirisdr_write_reg(p, 0x07, 0x000025 | swap | decim | burst);
		p->addr = 384 + 2;
		p->addr_step = 384;
		break;
	case MIRISDR_FORMAT_504_S16:
	case MIRISDR_FORMAT_504_S8:
		/* maximum rate 12.096 Msps | 24.576 MB/s | 196.608 Mbit/s */
#if MIRISDR_DEBUG >= 1
		fprintf( stderr, "format: 504\n");
#endif
		mirisdr_write_reg(p, 0x07, 0x000c14 | swap | decim | burst);
		p->addr = 504 + 2;
		p->addr_step = 504;
		break;
	case MIRISDR_FORMAT_504_REAL_S16:
#if MIRISDR_DEBUG >= 1
		fprintf( stderr, "format: 504 real\n");
#endif
		mirisdr_write_reg(p, 0x07, 0x000414 | swap | decim | burst);
		p->addr = 504 + 2;
		p->addr_step = 504;
		break;
	case MIRISDR_FORMAT_672_REAL_S16:
#if MIRISDR_DEBUG >= 1
		fprintf( stderr, "format: 672 real\n");
#endif
		mirisdr_write_reg(p, 0x07, 0x000405 | swap | decim | burst);
		p->addr = 672 + 2;
		p->addr_step = 672;
		break;
	case MIRISDR_FORMAT_768_REAL_S16:
#if MIRISDR_DEBUG >= 1
		fprintf( stderr, "format: 768 real\n");
#endif
		mirisdr_write_reg(p, 0x07, 0x000425 | swap | decim | burst);
		p->addr = 768 + 2;
		p->addr_step = 768;
		break;
	}

	/*
	 * Výpočet dělení vzorkovací frekvence
	 * Min: >= 1.3 Msps
	 * Max: <= 15 Msps, od 12,096 Msps prokládaně
	 * Poznámka: Nastavení vyšší frekvence než 15 Msps uvede tuner do speciálního
	 *           režimu kdy není možné přepnout rate zpět, stejně tak nastavení nižší
	 *           frekvence než je 571429 sps, protože pak bude N menší jak 2, což není
	 *           přípustný stav.
	 */
	/*
	 * Calculating division sampling frequency
	 * Min: >= 1.3 Msps
	 * Max: <= 15 Msps, from 12,096 Msps interpolated
	 * Note: Setting a higher frequency than 15 Msps indicate tuner into a special mode
	 * 		 where you can not switch back rate, as well as setting a lower frequency than 571,429 SPS
	 * 		 because it will be less than N 2, which is not an acceptable condition.
	 */
	for (i = 4; i < 16; i += 2)
	{
		vco = (uint64_t) pll_rate * i * 12;

		if (vco >= 202000000UL) {
			break;
		}
	}

	/* z předchozího výpočtu je N minimálně 4 */
	/* from the previous calculation N is at least 4 */
	n = vco / 48000000UL;
	fract = 0x200000UL * (vco % 48000000UL) / 48000000UL;
#if MIRISDR_DEBUG >= 1
	fprintf( stderr, "rate: %u, pll: %u, vco: %lu (%lu), n: %lu, fraction: %lu\n",
			p->rate, pll_rate, (long unsigned int)vco, (long unsigned int)(i / 2) - 1,
			(long unsigned int)n, (long unsigned int)fract);
#endif
	/* nastavení vzorkovací frekvence */
	/* Setting the sampling rate */
	reg3 |= (0x03 & 3) << 0; /* ?? */
	reg3 |= (0x07 & (i / 2 - 1)) << 2; /* rozlišení / distinction */
	reg3 |= (0x03 & 0) << 5; /* ?? */
	reg3 |= (0x01 & (fract >> 20)) << 7; /* +0.5 */
	reg3 |= (0x0f & n) << 8; /* hlavní rozsah / main range */

	switch (p->format)
	{ /* AGC */
	case MIRISDR_FORMAT_252_S16:
	case MIRISDR_FORMAT_504_REAL_S16:
		reg3 |= (0x0f & 0x01) << 12;
		break;
	case MIRISDR_FORMAT_336_S16:
	case MIRISDR_FORMAT_672_REAL_S16:
		reg3 |= (0x0f & 0x05) << 12;
		break;
	case MIRISDR_FORMAT_384_S16:
	case MIRISDR_FORMAT_768_REAL_S16:
		reg3 |= (0x0f & 0x09) << 12;
		break;
	case MIRISDR_FORMAT_504_S16:
	case MIRISDR_FORMAT_504_S8:
		reg3 |= (0x0f & 0x0d) << 12;
		break;
	}

	reg3 |= (0x01 & 1) << 16; /* ?? */

	/* The real formats digitise one converter, so gate the other off: bit 5
	   powers down I, bit 6 Q.  reg7 bit 9 chooses which one is read. */
	switch (p->format)
	{
	case MIRISDR_FORMAT_504_REAL_S16:
	case MIRISDR_FORMAT_672_REAL_S16:
	case MIRISDR_FORMAT_768_REAL_S16:
		reg3 |= (p->swap_iq ? (1 << 5) : (1 << 6));
		break;
	default:
		break;
	}

	/* registr pro detailní nastavení vzorkovací frekvence */
	/* Registry settings for detailed sampling frequency */
	reg4 |= (0xfffff & fract) << 0;

	mirisdr_write_reg(p, 0x04, reg4);
	mirisdr_write_reg(p, 0x03, reg3);

	/* opětovné spuštění streamu */
	/* restart stream */
	if ((streaming) && (mirisdr_start_async(p) < 0)) {
		goto failed;
	}

	return 0;

	failed: return -1;
}

int mirisdr_set_sample_rate(mirisdr_dev_t *p, uint32_t rate)
{
	int r;

	p->rate = rate;
	r = mirisdr_set_hard(p);

	/* the range depends on the decimation bypass, report what was reached */
	if (p->rate != rate) {
		fprintf(stderr, "can't set rate %u, using %u\n", rate, p->rate);
	}

	return r;
}

uint32_t mirisdr_get_sample_rate(mirisdr_dev_t *p)
{
	return p->rate;
}

int mirisdr_set_sample_format(mirisdr_dev_t *p, const char *v)
{
	if (!strcmp(v, "AUTO"))
	{
		p->format_auto = MIRISDR_FORMAT_AUTO_ON;
	}
	else if (!strcmp(v, "AUTO_REAL"))
	{
		p->format_auto = MIRISDR_FORMAT_AUTO_REAL;
	}
	else
	{
		p->format_auto = MIRISDR_FORMAT_AUTO_OFF;
		if (!strcmp(v, "252_S16")) {
			p->format = MIRISDR_FORMAT_252_S16;
		} else if (!strcmp(v, "336_S16")) {
			p->format = MIRISDR_FORMAT_336_S16;
		} else if (!strcmp(v, "384_S16")) {
			p->format = MIRISDR_FORMAT_384_S16;
		} else if (!strcmp(v, "504_S16")) {
			p->format = MIRISDR_FORMAT_504_S16;
		} else if (!strcmp(v, "504_S8")) {
			p->format = MIRISDR_FORMAT_504_S8;
		} else if (!strcmp(v, "504_REAL_S16")) {
			p->format = MIRISDR_FORMAT_504_REAL_S16;
		} else if (!strcmp(v, "672_REAL_S16")) {
			p->format = MIRISDR_FORMAT_672_REAL_S16;
		} else if (!strcmp(v, "768_REAL_S16")) {
			p->format = MIRISDR_FORMAT_768_REAL_S16;
		} else {
			fprintf(stderr, "unsupported format: %s\n", v);
			goto failed;
		}
	}

	return mirisdr_set_hard(p);

	failed: return -1;
}

int mirisdr_set_decimation_bypass(mirisdr_dev_t *p, const char *v)
{
	if (!strcmp(v, "AUTO")) {
		p->decimation_bypass = MIRISDR_DECIMATION_BYPASS_AUTO;
	} else if (!strcmp(v, "ON")) {
		p->decimation_bypass = MIRISDR_DECIMATION_BYPASS_ON;
	} else if (!strcmp(v, "OFF")) {
		p->decimation_bypass = MIRISDR_DECIMATION_BYPASS_OFF;
	} else {
		fprintf(stderr, "unsupported decimation bypass: %s\n", v);
		return -1;
	}

	return mirisdr_set_hard(p);
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

int mirisdr_set_swap_iq(mirisdr_dev_t *p, int swap)
{
	p->swap_iq = swap ? 1 : 0;

	return mirisdr_set_hard(p);
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

