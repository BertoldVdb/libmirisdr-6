/*
 * Copyright (C) 2013 by Miroslav Slugen <thunder.m@email.cz
 * Copyright (C) 2025 by Peter Hackenberg <170885528+Peter3579@users.noreply.github.com>
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

typedef struct mirisdr_device {
    uint16_t            vid;
    uint16_t            pid;
    const char          *name;
    const char          *manufacturer;
    const char          *product;
    int                 flavour;        /* mirisdr_hw_flavour_t picked by MIRISDR_HW_AUTO */
} mirisdr_device_t;

struct mirisdr_dev {
    libusb_context      *ctx;
    struct libusb_device_handle *dh;

    /* parametry */
    uint32_t            index;
    uint32_t            freq;
    uint32_t            rate;
    int                 gain;
    int                 gain_reduction_lna;
    int                 gain_reduction_mixbuffer;
    int                 gain_reduction_mixer;
    int                 gain_reduction_baseband;
    int                 gain_stages_set;    /* a stage set by hand: retunes keep the split */
    int                 lna_cal_mhz;        /* the L-band LNA was last calibrated for this, 0 none */
    int                 lna_cal_run;        /* this tune is the calibration one */
    mirisdr_hw_flavour_t hw_flavour;
    int                 external_tuner;
    mirisdr_band_t      band;
    enum {
        MIRISDR_FORMAT_AUTO_ON = 0,
        MIRISDR_FORMAT_AUTO_OFF,
        MIRISDR_FORMAT_AUTO_REAL
    } format_auto;
    enum {
        MIRISDR_DECIMATION_BYPASS_AUTO = 0,
        MIRISDR_DECIMATION_BYPASS_OFF,
        MIRISDR_DECIMATION_BYPASS_ON
    } decimation_bypass;
    enum {
        MIRISDR_FORMAT_252_S16 = 0,
        MIRISDR_FORMAT_336_S16,
        MIRISDR_FORMAT_384_S16,
        MIRISDR_FORMAT_504_S16,
        MIRISDR_FORMAT_504_S8,
        /* real modes use a single converter, each entry is a single I OR Q sample. */
        MIRISDR_FORMAT_504_REAL_S16,
        MIRISDR_FORMAT_672_REAL_S16,
        MIRISDR_FORMAT_768_REAL_S16
    } format;
    enum {
        MIRISDR_BW_200KHZ = 0,
        MIRISDR_BW_300KHZ,
        MIRISDR_BW_600KHZ,
        MIRISDR_BW_1536KHZ,
        MIRISDR_BW_5MHZ,
        MIRISDR_BW_6MHZ,
        MIRISDR_BW_7MHZ,
        MIRISDR_BW_8MHZ,
        MIRISDR_BW_MAX
    } bandwidth;
    enum {
        MIRISDR_IF_ZERO = 0,
        MIRISDR_IF_450KHZ,
        MIRISDR_IF_1620KHZ,
        MIRISDR_IF_2048KHZ
    } if_freq;
    enum {
        MIRISDR_XTAL_19_2M = 0,
        MIRISDR_XTAL_22M,
        MIRISDR_XTAL_24M,
        MIRISDR_XTAL_24_576M,
        MIRISDR_XTAL_26M,
        MIRISDR_XTAL_38_4M
    } xtal;
    enum {
        MIRISDR_TRANSFER_BULK = 0,
        MIRISDR_TRANSFER_ISOC
    } transfer;
    uint8_t             alt_setting;    /* the one streaming actually selects */

    /* async */
    enum {
        MIRISDR_ASYNC_INACTIVE = 0,
        MIRISDR_ASYNC_CANCELING,
        MIRISDR_ASYNC_RUNNING,
        MIRISDR_ASYNC_PAUSED,
        MIRISDR_ASYNC_FAILED
    } async_status;
    mirisdr_read_async_cb_t cb;
    void                *cb_ctx;
    size_t              xfer_buf_num;
    int                 xfer_inflight;  /* transfers submitted and not yet completed */
    int                 xfer_draining;  /* completions must not resubmit */
    struct libusb_transfer **xfer;
    unsigned char       **xfer_buf;
    int                 xfer_buf_devmem;
    int                 xfer_buf_slow;  /* reading from USB buffers is slow */
    size_t              xfer_buf_size;
    size_t              xfer_out_len;
    size_t              xfer_out_pos;
    unsigned char       *xfer_out;
    uint32_t            addr;
    uint32_t            addr_step;
    int                 swap_iq;
    int                 driver_active;
    int                 bias;
    int                 reg8;
    uint32_t            reg8_sent;      /* last word written to register 8 */
    int                 reg8_valid;     /* register 8 still holds reg8_sent */
    uint32_t            tuner_reg[16];  /* last word written to each MSi001 register, 0 if never */
    uint16_t            tuner_valid;    /* per register: the tuner holds tuner_reg */
    uint8_t             tuner_turned;   /* the next word clocks a readback and is not latched */
    uint8_t             batch[256];     /* register words held for one list request */
    int                 batch_n;
    int                 batch_depth;
    int                 batch_running;  /* a tune list may still be running */
    uint32_t            tuner_regd;     /* last register 13 data, without the override bits */
    uint32_t            tuner_ovr13;    /* override bits added to the register 13 data */
    uint32_t            tuner_ovr14;    /* register 14 data, the override bits */
    int                 filter_cal;     /* IF filter code under the real crystal's XTALSEL, -1 unknown */
    uint8_t             tuner_gap;      /* tuned with another crystal's XTALSEL: hold filter_cal */
    uint8_t             dc_n;           /* DC calibration divider N of the XTALSEL tuned, 0 before */
    uint16_t            exp_iodir;      /* RSP1B expander direction word sent, 0 bits pulled low */
    int                 exp_valid;      /* the expander holds exp_iodir */
    int                 notch;          /* MIRISDR_NOTCH_* asked for */
    int                 fw_ours;
    uint8_t             fw_pps_at;      /* status blocks, from the firmware block: internal RAM, */
    uint16_t            fw_anchor_at;   /* xdata, */
    uint8_t             fw_list_at;     /* internal RAM; 0 when the firmware has none */
    int                 sync_ready;     /* the sync path has started the stream */
    uint8_t             *sync_in;       /* one bulk transfer as read */
    uint8_t             *sync_out;      /* it converted, not yet handed out */
    int                 sync_len;
    int                 sync_pos;
    int                 sync_xlen;      /* next read length, short once to shift the grid */
    int64_t             pps_base;       /* stream index where the packet count was zeroed */
    uint64_t            pps_lost0;
    uint8_t             pps_edge0;
    uint8_t             pps_stale;      /* the latched edge predates the anchor */
    uint32_t            pps_irq_last;
    uint32_t            pps_irq_high;
    uint8_t             pps_anchor_valid;
    int                 pps_src_sof;    /* watch USB frames, not GPIO_0 */
    int                 pps_div;        /* latch every Nth edge, 1 to 255 */
    uint8_t             pps_seen;       /* an interval matching the clock has been seen */
    double              pps_k_min;      /* least a SOF handler was seen to cost */
    uint16_t            i2c_delay;
    uint32_t            i2c_clock_ns;
    int                 ee_size;        /* what the EEPROM's address reaches, 0 if none */

    /* pins the application has taken over, and what it wants them doing */
    uint8_t             gpio_mask;
    uint8_t             gpio_dir;
    uint8_t             gpio_val;

    /* IR receiver, see ir.c */
    uint32_t            ir_tick_ns;     /* counter period */
    uint32_t            ir_start;       /* sample counter when this pulse started */
    uint8_t             ir_prev;        /* last header byte 6 */
    uint8_t             ir_have;        
    uint8_t             ir_level;       /* the level of the current pulse */
    uint8_t             ir_partial;     /* pulse beginning was missing */
    mirisdr_ir_cb_t     ir_cb;
    void                *ir_ctx;
    mirisdr_stream_event_cb_t ev_cb;
    void                *ev_ctx;
    uint16_t            ev_last;        /* the previous packet's header bytes 4-5 */
    uint8_t             ev_valid;       /* ev_last holds a packet of this stream */
    uint8_t             ev_missed;      /* packets were lost since ev_last */

    uint8_t             *samples;
    int                 samples_size;
    mirisdr_stream_stats_t stats;
    int                 stats_head;    /* next block starts a delivered buffer */
    int                 sync_run;      /* consecutive bad blocks, drives resync */
    uint8_t             bulk_carry[2 * 1024 + 48];  /* a leftover and the next transfer's head */
    int                 bulk_carry_n;  /* leftover, under 1036 bytes */
    int                 bulk_lost;     /* off the grid, counted as one resync */
    int                 bulk_wait;     /* transfers still queued with the old phase */
    /* gaps, from the conversion that finds them to the callback that delivers them */
    int                 gap_fill;
    int                 gap_track;     /* read_async: keep the queue and insert fills */
    struct {
        uint64_t at;                   /* delivered byte position, fills included */
        uint64_t samples;
        uint32_t filled;
    }                   gapq[64];
    int                 gapq_n;
    struct {
        uint32_t off;                  /* byte offset in this conversion's output */
        uint32_t n;                    /* samples */
    }                   fills[64];
    int                 fills_n;
    uint64_t            conv_samples;  /* stats.samples when this conversion began */
    uint64_t            conv_filled;
    uint64_t            fed_bytes;     /* bytes passed on to the callbacks' buffering, fills included */
    uint64_t            cb_bytes;      /* bytes handed to the application before this buffer */
    uint64_t            cb_lost;       /* samples missing, not filled, before this buffer */
    mirisdr_buffer_info_t cb_info;
    int                 addr_valid;
    int                 addr_restart;  /* stream start since the last block */

    /* dc offset calibration */
    enum {
        MIRISDR_DC_STATIC = 0,
        MIRISDR_DC_PERIODIC1,
        MIRISDR_DC_PERIODIC2,
        MIRISDR_DC_PERIODIC3,
        MIRISDR_DC_ONE_SHOT,
        MIRISDR_DC_CONTINUOUS
    } dc_mode;
    int                 dc_speedup;
    int                 dc_track;   // tracking duration
    int                 dc_period;  // refresh period
};

