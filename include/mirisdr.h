/*
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Dimitri Stolnikov <horiz0n@gmx.net>
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

#ifndef __MIRISDR_H
#define __MIRISDR_H

#ifdef __cplusplus
extern "C" {
#endif

// Set debug level
// 0=no debug
// 1=gain and frequency info.
// 2=extended debug (register writes)
// 3=more extended debug (function calls)
#define MIRISDR_DEBUG 0

#include <stdint.h>
#include <mirisdr_export.h>

typedef enum
{
    MIRISDR_HW_DEFAULT,
    MIRISDR_HW_SDRPLAY,
} mirisdr_hw_flavour_t;

typedef enum
{
    MIRISDR_BAND_AM1,
    MIRISDR_BAND_AM2,
    MIRISDR_BAND_VHF,
    MIRISDR_BAND_3,
    MIRISDR_BAND_45,
    MIRISDR_BAND_L,
} mirisdr_band_t;

typedef struct mirisdr_dev mirisdr_dev_t;

/* devices */
MIRISDR_API uint32_t mirisdr_get_device_count (void);
MIRISDR_API const char *mirisdr_get_device_name (uint32_t index);
MIRISDR_API int mirisdr_get_device_usb_strings (uint32_t index, char *manufact, char *product, char *serial);
/* Matches the serial of the device if it has one, otherwise a synthetic one based on USB bus location */
MIRISDR_API int mirisdr_get_index_by_serial (const char *serial);

/* main */
MIRISDR_API int mirisdr_open (mirisdr_dev_t **p, uint32_t index);
MIRISDR_API int mirisdr_open_fd (mirisdr_dev_t **p, int fd);
MIRISDR_API int mirisdr_close (mirisdr_dev_t *p);
MIRISDR_API int mirisdr_reset (mirisdr_dev_t *p);                       /* extra */
MIRISDR_API int mirisdr_reset_buffer (mirisdr_dev_t *p);
MIRISDR_API int mirisdr_get_usb_strings (mirisdr_dev_t *dev, char *manufact, char *product, char *serial);
MIRISDR_API int mirisdr_get_usb_ids (mirisdr_dev_t *p, uint16_t *vid, uint16_t *pid); /* extra */
/* The serial the device itself declares, returns 0 if none declared */
MIRISDR_API int mirisdr_get_serial (mirisdr_dev_t *p, char *out, int len); /* extra */
MIRISDR_API int mirisdr_set_hw_flavour (mirisdr_dev_t *p, mirisdr_hw_flavour_t hw_flavour);

/* sync */
MIRISDR_API int mirisdr_read_sync (mirisdr_dev_t *p, void *buf, int len, int *n_read);

/* async */
typedef void(*mirisdr_read_async_cb_t) (unsigned char *buf, uint32_t len, void *ctx);
MIRISDR_API int mirisdr_read_async (mirisdr_dev_t *p, mirisdr_read_async_cb_t cb, void *ctx, uint32_t num, uint32_t len);
MIRISDR_API int mirisdr_cancel_async (mirisdr_dev_t *p);
MIRISDR_API int mirisdr_cancel_async_now (mirisdr_dev_t *p);            /* extra */
MIRISDR_API int mirisdr_start_async (mirisdr_dev_t *p);                 /* extra */
MIRISDR_API int mirisdr_stop_async (mirisdr_dev_t *p);                  /* extra */

/* adc */
MIRISDR_API int mirisdr_adc_init (mirisdr_dev_t *p);                    /* extra */

/* rate control */
MIRISDR_API int mirisdr_set_sample_rate (mirisdr_dev_t *p, uint32_t rate);
MIRISDR_API uint32_t mirisdr_get_sample_rate (mirisdr_dev_t *p);

/*
 * The PLL of the device is limited to ~15MHz, so higher sample rates require 
 * turning off the internal decimator. This results in a doubled output rate,
 * but requires the application to do some extra filtering to get useful results.
 *
 * Three modes are available:
 *   "AUTO" (default)  bypass only above 14.5 Msps
 *   "ON"              always bypass (the PLL runs at half the sample rate)
 *   "OFF"             never bypass  (the PLL runs at the sample rate)
 */
MIRISDR_API int mirisdr_set_decimation_bypass (mirisdr_dev_t *p, const char *v);  /* extra */
MIRISDR_API const char *mirisdr_get_decimation_bypass (mirisdr_dev_t *p);         /* extra */

/*
 * Complex formats deliver interleaved I/Q, so n samples are 2n int16. The
 * REAL formats digitise a single converter. Every int16 is one real sample.
 * This means that n samples are n int16.
 *
 * The following rates are available:
 *   AUTO, 252_S16, 336_S16, 384_S16, 504_S16, 504_S8
 *   AUTO_REAL, 504_REAL_S16, 672_REAL_S16, 768_REAL_S16
 */
MIRISDR_API int mirisdr_set_sample_format (mirisdr_dev_t *p, const char *v);  /* extra */
MIRISDR_API const char *mirisdr_get_sample_format (mirisdr_dev_t *p);   /* extra */

/*
 * This function returns the selected sample format, eg:
 * set: AUTO get: 336_S16
 */
MIRISDR_API const char *mirisdr_get_sample_format_selected (mirisdr_dev_t *p);  /* extra */

/*
 * For complex formats this swaps I and Q. For real formats, this selects which
 * converter is used (swap=Q, no-swap=I).
 */
MIRISDR_API int mirisdr_set_swap_iq (mirisdr_dev_t *p, int swap);       /* extra */
MIRISDR_API int mirisdr_get_swap_iq (mirisdr_dev_t *p);                 /* extra */

/*
 * Counts are in the unit the hardware counter uses:
 * I/Q pairs in the complex formats, single samples in the real ones.
 */
typedef struct mirisdr_stream_stats
{
	uint64_t samples;   /* total number delivered to the application */
	uint64_t lost;      /* total number of samples lost */
	uint64_t gaps;      /* number of gaps in the stream */
	uint64_t jitter;    /* number of jitter events. These happen at very high rates and are harmless. */
	uint64_t resyncs;   /* byte alignment recoveries */
	uint64_t index;     /* absolute index of the first sample in the buffer */
} mirisdr_stream_stats_t;

MIRISDR_API int mirisdr_get_stream_stats (mirisdr_dev_t *p, mirisdr_stream_stats_t *s); /* extra */

/* streaming control */
MIRISDR_API int mirisdr_streaming_start (mirisdr_dev_t *p);             /* extra */
MIRISDR_API int mirisdr_streaming_stop (mirisdr_dev_t *p);              /* extra */

/* frequency */
MIRISDR_API int mirisdr_set_center_freq (mirisdr_dev_t *p, uint32_t freq);
MIRISDR_API uint32_t mirisdr_get_center_freq (mirisdr_dev_t *p);
MIRISDR_API int mirisdr_set_if_freq (mirisdr_dev_t *p, uint32_t freq);  /* extra */
MIRISDR_API uint32_t mirisdr_get_if_freq (mirisdr_dev_t *p);            /* extra */
MIRISDR_API int mirisdr_set_xtal_freq (mirisdr_dev_t *p, uint32_t freq);/* extra */
MIRISDR_API uint32_t mirisdr_get_xtal_freq (mirisdr_dev_t *p);          /* extra */
MIRISDR_API int mirisdr_set_bandwidth (mirisdr_dev_t *p, uint32_t bw);  /* extra */
MIRISDR_API uint32_t mirisdr_get_bandwidth (mirisdr_dev_t *p);          /* extra */
MIRISDR_API int mirisdr_set_offset_tuning (mirisdr_dev_t *p, int on);   /* extra */
MIRISDR_API mirisdr_band_t mirisdr_get_band (mirisdr_dev_t *p);         /* extra */

/* not implemented yet */
MIRISDR_API int mirisdr_set_freq_correction (mirisdr_dev_t *p, int ppm);
MIRISDR_API int mirisdr_set_direct_sampling (mirisdr_dev_t *p, int on);

/* transfer */
MIRISDR_API int mirisdr_set_transfer (mirisdr_dev_t *p, const char *v);       /* extra */
MIRISDR_API const char *mirisdr_get_transfer (mirisdr_dev_t *p);        /* extra */

/* gain */
MIRISDR_API int mirisdr_set_gain (mirisdr_dev_t *p);                    /* extra */
MIRISDR_API int mirisdr_get_tuner_gains (mirisdr_dev_t *dev, int *gains);
MIRISDR_API int mirisdr_set_tuner_gain (mirisdr_dev_t *p, int gain);
MIRISDR_API int mirisdr_get_tuner_gain (mirisdr_dev_t *p);              /* extra */
MIRISDR_API int mirisdr_set_tuner_gain_mode (mirisdr_dev_t *p, int mode);
MIRISDR_API int mirisdr_get_tuner_gain_mode (mirisdr_dev_t *p);         /* extra */
MIRISDR_API int mirisdr_set_mixer_gain (mirisdr_dev_t *p, int gain);    /* extra */
MIRISDR_API int mirisdr_set_mixbuffer_gain (mirisdr_dev_t *p, int gain);/* extra */
MIRISDR_API int mirisdr_set_lna_gain (mirisdr_dev_t *p, int gain);      /* extra */
MIRISDR_API int mirisdr_set_baseband_gain (mirisdr_dev_t *p, int gain); /* extra */
MIRISDR_API int mirisdr_get_mixer_gain (mirisdr_dev_t *p);              /* extra */
MIRISDR_API int mirisdr_get_mixbuffer_gain (mirisdr_dev_t *p);          /* extra */
MIRISDR_API int mirisdr_get_lna_gain (mirisdr_dev_t *p);                /* extra */
MIRISDR_API int mirisdr_get_baseband_gain (mirisdr_dev_t *p);           /* extra */
MIRISDR_API int mirisdr_set_bias (mirisdr_dev_t *p, int bias);          /* extra */
MIRISDR_API int mirisdr_get_bias (mirisdr_dev_t *p);                    /* extra */

/* GPIO control */
#define MIRISDR_GPIO_COUNT      4

MIRISDR_API int mirisdr_set_gpio_direction (mirisdr_dev_t *p, unsigned int pin, int output); /* extra */
MIRISDR_API int mirisdr_get_gpio_direction (mirisdr_dev_t *p, unsigned int pin); /* extra */
MIRISDR_API int mirisdr_set_gpio_output (mirisdr_dev_t *p, unsigned int pin, int high); /* extra */
MIRISDR_API int mirisdr_get_gpio_output (mirisdr_dev_t *p, unsigned int pin); /* extra */
MIRISDR_API int mirisdr_get_gpio_input (mirisdr_dev_t *p, unsigned int pin); /* extra */
MIRISDR_API int mirisdr_release_gpio (mirisdr_dev_t *p, unsigned int pin); /* extra */
MIRISDR_API int mirisdr_set_gpio_outputs (mirisdr_dev_t *p, unsigned int mask, unsigned int levels); /* extra */
MIRISDR_API int mirisdr_get_gpio_inputs (mirisdr_dev_t *p); /* extra */

#define MIRISDR_FW_SERIAL_MAX   12

/* mirisdr_fw_get() sets fields to what the image has, mirisdr_fw_patch() applies only
 * what is set. A serial is at most MIRISDR_FW_SERIAL_MAX characters. An empty one leaves
 * the image declaring none */
#define MIRISDR_FW_PATCH_IDS    (1u << 0)
#define MIRISDR_FW_PATCH_SERIAL (1u << 1)

typedef struct mirisdr_fw_patch
{
	unsigned int   fields;
	uint16_t       vid;
	uint16_t       pid;
	char           serial[MIRISDR_FW_SERIAL_MAX + 1];
} mirisdr_fw_patch_t;

MIRISDR_API int mirisdr_fw_get (const uint8_t *image, uint32_t size, mirisdr_fw_patch_t *out); /* extra */
MIRISDR_API int mirisdr_fw_patch (uint8_t *image, uint32_t size, const mirisdr_fw_patch_t *p); /* extra */

/* Open a device with options, including the firmware it should be running.
 * The image comes from a buffer or from a path. The image can be patched
 * at runtime to ensure the VID/PID/Serial do not change. */
#define MIRISDR_FW_IDS_IMAGE    0
#define MIRISDR_FW_IDS_DEVICE   1
#define MIRISDR_FW_IDS_SET      2

typedef struct mirisdr_open_config
{
	uint32_t       index;           /* when opening by index */
	const char    *serial;          /* or by serial, which wins over the index */
	int            fd;              /* or an already open descriptor, -1 for none */

	const uint8_t *firmware;        /* the image, or NULL */
	uint32_t       firmware_size;
	const char    *firmware_path;   /* or a file to read it from */
	int            firmware_ids;    /* one of MIRISDR_FW_IDS_* */
	mirisdr_fw_patch_t firmware_patch; /* what MIRISDR_FW_IDS_SET writes in */
	int            keep_running;    /* use running fw */
} mirisdr_open_config_t;

/* Please fill in the open_config struct using mirisdr_open_config_default(&cfg) and only
 * the edit the options you need. You will likely only need to change index or fd. */

MIRISDR_API void mirisdr_open_config_default (mirisdr_open_config_t *cfg); /* extra */
MIRISDR_API int mirisdr_open_ex (mirisdr_dev_t **p, const mirisdr_open_config_t *cfg); /* extra */

/* If a function returns this value, the USB handle is invalid, and the device needs to be reopened.
 * Currently this can only happens when opening by fd or calling mirisdr_reboot(). */
#define MIRISDR_REOPEN          (-2)

#define MIRISDR_FW_BLOCK        0x0040
#define MIRISDR_FW_ID_LEN       8

MIRISDR_API int mirisdr_get_fw_id (mirisdr_dev_t *p, uint8_t *buf, int len); /* extra */

MIRISDR_API int mirisdr_running_from_rom (mirisdr_dev_t *p);            /* extra */

/* These functions allow the host to read and write device memory. The remap argument
 * allows access to certain internal DSP and USB memories, assuming a compatible firmware
 * is loaded. Using remap is ignored on the standard firmware and can cause data corruption. */
MIRISDR_API int mirisdr_read_mem (mirisdr_dev_t *p, uint16_t addr, uint8_t *buf, int len, int remap); /* extra */
MIRISDR_API int mirisdr_write_mem (mirisdr_dev_t *p, uint16_t addr, const uint8_t *buf, int len, int remap); /* extra */

#define MIRISDR_BOOT_ROM        0
#define MIRISDR_BOOT_RAM        1
#define MIRISDR_BOOT_IGNORE_EEPROM 2 /* attempt to skip loading firmware or configuration from EEPROM */
MIRISDR_API int mirisdr_reboot (mirisdr_dev_t *p, int mode); /* extra, returns MIRISDR_REOPEN */

/* Call code on the device. The registers go in and the ones the callee left come
 * back, in the same struct. Write the code somewhere unused with mirisdr_write_mem()
 * first. Needs a firmware that supports it */
#define MIRISDR_CALL_CTX        0x1FF8
typedef struct mirisdr_call_regs
{
	uint8_t  a;
	uint8_t  b;
	uint16_t dptr;
	uint8_t  r0;
	uint8_t  r1;
} mirisdr_call_regs_t;

MIRISDR_API int mirisdr_call (mirisdr_dev_t *p, uint16_t addr, mirisdr_call_regs_t *regs); /* extra */

/* Read and write the eeprom */
MIRISDR_API int mirisdr_read_eeprom (mirisdr_dev_t *p, uint16_t addr, uint8_t *buf, int len); /* extra */
MIRISDR_API int mirisdr_write_eeprom (mirisdr_dev_t *p, uint16_t addr, const uint8_t *buf, int len); /* extra */

/* Returns the maximum EEPROM size for the strapping:
 * 0=no response, 512=9-bit, 65536=16-bit */
MIRISDR_API int mirisdr_eeprom_size (mirisdr_dev_t *p); /* extra */

/* I2C master on GPIO_1 (SDA) and GPIO_2 (SCL) */
#define MIRISDR_I2C_NO_STOP     0x01
#define MIRISDR_I2C_REPEAT      0x02
#define MIRISDR_I2C_NACK        1
MIRISDR_API int mirisdr_set_i2c_rate (mirisdr_dev_t *p, uint32_t hz); /* extra, 50 kHz until set */
MIRISDR_API int mirisdr_i2c_write (mirisdr_dev_t *p, uint8_t addr, unsigned int flags, const uint8_t *buf, int len); /* extra */
MIRISDR_API int mirisdr_i2c_read (mirisdr_dev_t *p, uint8_t addr, unsigned int flags, uint8_t *buf, int len); /* extra */
MIRISDR_API int mirisdr_i2c_transfer (mirisdr_dev_t *p, uint8_t addr, const uint8_t *out, int outlen, uint8_t *in, int inlen); /* extra */
MIRISDR_API int mirisdr_i2c_recover (mirisdr_dev_t *p); /* extra */

/* Send UART data via GPIO_2 */
MIRISDR_API int mirisdr_uart_write (mirisdr_dev_t *p, uint32_t baud, const uint8_t *buf, int len); /* extra */

/* The MSI2500 has a remote control receiver on GPIO3, this function allows to configure a
 * callback that receives IR events. If you want to sample a non-pulse based protocol, 
 * such as UART, it makes sense to set tick_ns to 1 and use only the level field */
#define MIRISDR_IR_GPIO         3
#define MIRISDR_IR_KEEPS_GOING  127

typedef struct mirisdr_ir_pulse
{
	uint32_t index;     /* the sample counter for this packet */
	uint32_t duration;  /* the run's length if it ended here, or its current duration if not (microseconds) */
	uint8_t  level;     /* high=1 */
	uint8_t  ended;     /* set when this is the end of the pulse */
	uint8_t  partial;   /* duration is a floor due to missing packets */
	uint8_t  ticks;     /* raw count as received from the device (127=ongoing) */
} mirisdr_ir_pulse_t;

typedef void (*mirisdr_ir_cb_t) (const mirisdr_ir_pulse_t *pulse, void *ctx);

MIRISDR_API int mirisdr_set_ir (mirisdr_dev_t *p, uint32_t tick_ns, mirisdr_ir_cb_t cb, void *ctx); /* extra */
MIRISDR_API uint32_t mirisdr_get_ir (mirisdr_dev_t *p);                 /* extra */

/*
 * DC Calibration
 *
 * Only raw interface for testing purposes. No sanity checks done.
 * raw format with bit length in parenthesis:
 *   [ unused (4) | period (12) | unused (6) | track (6) | speedup (1) | mode (3) ]
 * The default dc setting is:
 *   0x080001f2 (period = 0x800, track = 0x1f, speedup = 0, mode = 2)
 */
MIRISDR_API int mirisdr_set_dc_raw (mirisdr_dev_t *p, uint32_t raw);
MIRISDR_API uint32_t mirisdr_get_dc_raw (mirisdr_dev_t *p);

#ifdef __cplusplus
}
#endif

#endif /* __MIRISDR_H */
