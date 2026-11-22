/*
 * Copyright (C) 2012 by Kyle Keen <keenerd@gmail.com>
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

/* a collection of user friendly tools
 * todo: use strtol for more flexible int parsing
 * */

#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#if !defined(_WIN32) || defined(__MINGW32__)
#include <unistd.h>
#else
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#define _USE_MATH_DEFINES
#endif

#include <math.h>

#include "mirisdr.h"

double atofs(char *s)
/* standard suffixes */
{
	char last;
	int len;
	double suff = 1.0;
	len = strlen(s);
	if (!len) return 0;
	last = s[len-1];
	s[len-1] = '\0';
	switch (last) {
		case 'g':
		case 'G':
			suff *= 1e3;
			/* FALLTHRU */
		case 'm':
		case 'M':
			suff *= 1e3;
			/* FALLTHRU */
		case 'k':
		case 'K':
			suff *= 1e3;
			suff *= atof(s);
			s[len-1] = last;
			return suff;
	}
	s[len-1] = last;
	return atof(s);
}

double atoft(char *s)
/* time suffixes, returns seconds */
{
	char last;
	int len;
	double suff = 1.0;
	len = strlen(s);
	if (!len) return 0;
	last = s[len-1];
	s[len-1] = '\0';
	switch (last) {
		case 'h':
		case 'H':
			suff *= 60;
			/* FALLTHRU */
		case 'm':
		case 'M':
			suff *= 60;
			/* FALLTHRU */
		case 's':
		case 'S':
			suff *= atof(s);
			s[len-1] = last;
			return suff;
	}
	s[len-1] = last;
	return atof(s);
}

double atofp(char *s)
/* percent suffixes */
{
	char last;
	int len;
	double suff = 1.0;
	len = strlen(s);
	if (!len) return 0;
	last = s[len-1];
	s[len-1] = '\0';
	switch (last) {
		case '%':
			suff *= 0.01;
			suff *= atof(s);
			s[len-1] = last;
			return suff;
	}
	s[len-1] = last;
	return atof(s);
}

int nearest_gain(mirisdr_dev_t *dev, int target_gain)
{
	int i, r, err1, err2, count, nearest;
	int* gains;
	r = mirisdr_set_tuner_gain_mode(dev, 1);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to enable manual gain.\n");
		return r;
	}
	count = mirisdr_get_tuner_gains(dev, NULL);
	if (count <= 0) {
		return 0;
	}
	gains = malloc(sizeof(int) * count);
	if (!gains) {
		fprintf(stderr, "WARNING: Out of memory for the gain list.\n");
		return -1;
	}
	count = mirisdr_get_tuner_gains(dev, gains);
	if (count <= 0) {
		free(gains);
		return 0;
	}
	nearest = gains[0];
	for (i=0; i<count; i++) {
		err1 = abs(target_gain - nearest);
		err2 = abs(target_gain - gains[i]);
		if (err2 < err1) {
			nearest = gains[i];
		}
	}
	free(gains);
	return nearest;
}

int verbose_set_frequency(mirisdr_dev_t *dev, uint32_t frequency)
{
	int r;
	r = mirisdr_set_center_freq(dev, frequency);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to set center freq.\n");
	} else {
		/* a band's maximum can lower a manual gain set before the tune */
		int gain = mirisdr_get_tuner_gain(dev);

		if (gain >= 0) fprintf(stderr, "Tuned to %u Hz, tuner gain %d dB.\n", frequency, gain);
		else fprintf(stderr, "Tuned to %u Hz.\n", frequency);
	}
	return r;
}

int verbose_set_sample_rate(mirisdr_dev_t *dev, uint32_t samp_rate)
{
	int r;
	r = mirisdr_set_sample_rate(dev, samp_rate);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to set sample rate.\n");
	} else {
		/* the rate reached, which can differ from the one asked */
		fprintf(stderr, "Sampling at %u S/s.\n", mirisdr_get_sample_rate(dev));
	}
	return r;
}

int verbose_direct_sampling(mirisdr_dev_t *dev, int on)
{
	int r;
	r = mirisdr_set_direct_sampling(dev, on);
	if (r != 0) {
		fprintf(stderr, "WARNING: Failed to set direct sampling mode.\n");
		return r;
	}
	if (on == 0) {
		fprintf(stderr, "Direct sampling mode disabled.\n");}
	if (on == 1) {
		fprintf(stderr, "Enabled direct sampling mode, input 1/I.\n");}
	if (on == 2) {
		fprintf(stderr, "Enabled direct sampling mode, input 2/Q.\n");}
	return r;
}

int verbose_offset_tuning(mirisdr_dev_t *dev)
{
	int r;
	r = mirisdr_set_offset_tuning(dev, 1);
	if (r != 0) {
		fprintf(stderr, "WARNING: Failed to set offset tuning.\n");
	} else {
		fprintf(stderr, "Offset tuning mode enabled.\n");
	}
	return r;
}

int verbose_auto_gain(mirisdr_dev_t *dev)
{
	int r;
	r = mirisdr_set_tuner_gain_mode(dev, 0);
	if (r != 0) {
		fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
	} else {
		fprintf(stderr, "Tuner gain set to automatic.\n");
	}
	return r;
}

int verbose_gain_set(mirisdr_dev_t *dev, int gain)
{
	int r;
	r = mirisdr_set_tuner_gain_mode(dev, 1);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to enable manual gain.\n");
		return r;
	}
	r = mirisdr_set_tuner_gain(dev, gain);
	if (r != 0) {
		fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
	} else {
		fprintf(stderr, "Tuner gain set to %d dB.\n", mirisdr_get_tuner_gain(dev));
	}
	return r;
}

int verbose_ppm_set(mirisdr_dev_t *dev, int ppm_error)
{
	int r;
	if (ppm_error == 0) {
		return 0;}
	r = mirisdr_set_freq_correction(dev, ppm_error);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to set ppm error.\n");
	} else {
		fprintf(stderr, "Tuner error set to %i ppm.\n", ppm_error);
	}
	return r;
}

int verbose_ppm_eeprom(mirisdr_dev_t *dev, int *ppm_error)
{
	#define start_char ' '
	#define stop_char 'p'
	int i, r, len, status = -1;
	char vendor[256], product[256], serial[256];
	r = mirisdr_get_usb_strings(dev, vendor, product, serial);
	if (r) {
		return r;
	}
	len = strlen(serial);
	if (len <= 3) {
		return -1;}
	if (serial[len-1] != stop_char) {
		return -1;}
	serial[len-1] = '\0';
	for (i=len-3; i>=0; i--) {
		if (serial[i] != start_char) {
			continue;}
		fprintf(stderr, "PPM calibration found in eeprom.\n");
		status = 0;
		*ppm_error = atoi(serial + i + 1);
		break;
	}
	serial[len-1] = stop_char;
	return status;
}

int verbose_reset_buffer(mirisdr_dev_t *dev)
{
	int r;
	r = mirisdr_reset_buffer(dev);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to reset buffers.\n");}
	return r;
}

int verbose_device_search(char *s)
{
	int i, device_count, device, offset;
	char *s2;
	char vendor[256], product[256], serial[256];
	device_count = mirisdr_get_device_count();
	if (!device_count) {
		fprintf(stderr, "No supported devices found.\n");
		return -1;
	}
	fprintf(stderr, "Found %d device(s):\n", device_count);
	for (i = 0; i < device_count; i++) {
		mirisdr_get_device_usb_strings(i, vendor, product, serial);
		fprintf(stderr, "  %d:  %s, %s, SN: %s\n", i, vendor, product, serial);
	}
	fprintf(stderr, "\n");
	/* does string look like raw id number */
	device = (int)strtol(s, &s2, 0);
	if (s2[0] == '\0' && device >= 0 && device < device_count) {
		fprintf(stderr, "Using device %d: %s\n",
			device, mirisdr_get_device_name((uint32_t)device));
		return device;
	}
	/* does string exact match a serial */
	for (i = 0; i < device_count; i++) {
		mirisdr_get_device_usb_strings(i, vendor, product, serial);
		if (strcmp(s, serial) != 0) {
			continue;}
		device = i;
		fprintf(stderr, "Using device %d: %s\n",
			device, mirisdr_get_device_name((uint32_t)device));
		return device;
	}
	/* does string prefix match a serial */
	for (i = 0; i < device_count; i++) {
		mirisdr_get_device_usb_strings(i, vendor, product, serial);
		if (strncmp(s, serial, strlen(s)) != 0) {
			continue;}
		device = i;
		fprintf(stderr, "Using device %d: %s\n",
			device, mirisdr_get_device_name((uint32_t)device));
		return device;
	}
	/* does string suffix match a serial */
	for (i = 0; i < device_count; i++) {
		mirisdr_get_device_usb_strings(i, vendor, product, serial);
		offset = strlen(serial) - strlen(s);
		if (offset < 0) {
			continue;}
		if (strncmp(s, serial+offset, strlen(s)) != 0) {
			continue;}
		device = i;
		fprintf(stderr, "Using device %d: %s\n",
			device, mirisdr_get_device_name((uint32_t)device));
		return device;
	}
	fprintf(stderr, "No matching devices found.\n");
	return -1;
}

/* The antenna input and the bias-T, -a and -P of miri_sdr and miri_fm. Both are part of
   the tune of tuner 0, so they go into the config the program tunes with */

/* A typed name against an antenna's. Capitals and dashes do not matter, so hiz is Hi-Z */
static int antenna_same(const char *a, const char *b)
{
	for (;;) {
		while (*a == '-') a++;
		while (*b == '-') b++;
		if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return 0;
		if (!*a) return 1;
		a++;
		b++;
	}
}

/* An antenna's name, the one the SoapySDR module lists it under. The Hi-Z input is Hi-Z
   and the test oscillator TEST. A board with a second 50 ohm port has its ports' letters,
   any other the one RX */
const char *antenna_name(const mirisdr_frontend_caps_t *caps, int antenna)
{
	static const char *letters[] = { "A", "B", "C" };

	if (antenna == MIRISDR_ANTENNA_TEST) return "TEST";
	if (antenna == caps->antenna_hiz) return "Hi-Z";
	if ((caps->antennas & (1u << MIRISDR_ANTENNA_B)) && (caps->antenna_hiz != MIRISDR_ANTENNA_B) &&
	    (antenna >= MIRISDR_ANTENNA_A) && (antenna <= MIRISDR_ANTENNA_C))
		return letters[antenna];
	return "RX";
}

/* -a: the antenna named, into the tune config. A name the receiver does not have lists
   the ones it has */
int verbose_antenna(mirisdr_dev_t *dev, const char *name, mirisdr_tune_config_t *tc)
{
	mirisdr_frontend_caps_t caps;
	int a;

	if (mirisdr_get_frontend_caps(dev, 0, &caps) < 0) {
		fprintf(stderr, "Failed to read the receiver's antennas.\n");
		return -1;
	}
	for (a = 0; a < 32; a++) {
		if ((caps.antennas & (1u << a)) && antenna_same(name, antenna_name(&caps, a))) {
			tc->frontend.antenna = a;
			return 0;
		}
	}
	fprintf(stderr, "No antenna %s. This receiver has:", name);
	for (a = 0; a < 32; a++) {
		if (caps.antennas & (1u << a)) {
			fprintf(stderr, " %s", antenna_name(&caps, a));}
	}
	fprintf(stderr, "\n");
	return -1;
}

/* After the tune: the antenna in use, where the board has more than one, and the bias-T,
   where it has one or GPIO 3 stands in for one. -P sets frontend.bias in the tune config
   and the tune refuses it where there is none to turn on: the Mirics TV design, and the
   RSPduo's tuner 0, as its bias-T powers the other tuner's port. mirisdr_close() turns it
   off again */
void verbose_frontend(mirisdr_dev_t *dev, const mirisdr_tune_result_t *tr)
{
	mirisdr_frontend_caps_t caps;

	if (mirisdr_get_frontend_caps(dev, 0, &caps) < 0) {
		return;}
	if (caps.antennas & (caps.antennas - 1)) {
		fprintf(stderr, "Antenna %s.\n", antenna_name(&caps, tr->frontend.antenna));}
	if (caps.bias) {
		fprintf(stderr, "Bias-T %s.\n", tr->frontend.bias ? "on" : "off");}
	else if (tr->frontend.bias) {
		fprintf(stderr, "No bias-T known on this board. GPIO 3 is held high, which switches one on some boards.\n"
		        "On an RSP1 that pin helps select the band filter, so reception suffers in most bands.\n");}
}

// vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab
