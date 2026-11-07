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
    int                 self_powered;   /* the power the firmware declares: self powered, */
    int                 max_ma;         /* and what it takes from VBUS, mA. Both 0 keeps the
                                           image's own, bus powered at 500 mA */
    int                 ds_lna_db;      /* the direct sampling amplifier's gain, dB, 0 for
                                           MIRISDR_DS_LNA_DB */
} mirisdr_device_t;

/* the lock on the transfers, recursive: a stop holds it while it frees and allocates them
   (start_async_held), and the run's finish while it drains. The events' lock, recursive
   so the event callback may change the callback */
#if defined(_WIN32)
typedef CRITICAL_SECTION mirisdr_lock_t;
#else
#include <pthread.h>
typedef pthread_mutex_t mirisdr_lock_t;
#endif

enum {
    MIRISDR_ASYNC_INACTIVE = 0,
    MIRISDR_ASYNC_CANCELING,
    MIRISDR_ASYNC_RUNNING,
    MIRISDR_ASYNC_PAUSED,
    MIRISDR_ASYNC_FAILED,
    MIRISDR_ASYNC_STARTING      /* read_async entered, the transfers not yet running */
};

/* What a tuner keeps of its own. On a board with two (the RSPduo) the device's fields
   hold the selected tuner's, the others' wait here: mirisdr_tuner_select() */
typedef struct mirisdr_tuner_ctx
{
    uint32_t            freq;
    int                 gain;
    int                 gain_reduction_lna;
    int                 gain_reduction_mixbuffer;
    int                 gain_reduction_mixer;
    int                 gain_reduction_baseband;
    int                 gain_stages_set;
    int                 gain_reduction_amp;
    int                 amp_db;
    int                 attenuator;
    int                 lna_cal_mhz;
    mirisdr_band_t      band;
    int                 bandwidth;
    int                 if_freq;
    int                 bias;
    mirisdr_tune_config_t tune;
    uint32_t            tune_lo;
    int                 tune_iq;
    uint16_t            tune_exp;
    uint32_t            tuner_reg[16];
    uint16_t            tuner_valid;
    uint8_t             tuner_turned;
    uint8_t             tuner_last;
    uint32_t            tuner_regd;
    uint32_t            tuner_ovr13;
    uint32_t            tuner_ovr14;
    int                 filter_cal;
    uint8_t             tuner_gap;
    uint8_t             dc_n;
    int                 notch;
    int                 antenna;
    int                 ds;
    int                 ds_lna;
    uint32_t            ds_edge;
    int                 dc_mode;
    int                 dc_speedup;
    int                 dc_track;
    int                 dc_period;
} mirisdr_tuner_ctx_t;

/* its fields, as the device has them too (tuner_reg apart, an array) */
#define MIRISDR_TUNER_CTX(X) X(freq) X(gain) X(gain_reduction_lna) X(gain_reduction_mixbuffer) \
    X(gain_reduction_mixer) X(gain_reduction_baseband) X(gain_stages_set) X(gain_reduction_amp) X(amp_db) X(attenuator) X(lna_cal_mhz) X(band) \
    X(bandwidth) X(if_freq) X(bias) X(tune) X(tune_lo) X(tune_iq) X(tune_exp) X(tuner_valid) X(tuner_turned) X(tuner_last) \
    X(tuner_regd) X(tuner_ovr13) X(tuner_ovr14) X(filter_cal) X(tuner_gap) X(dc_n) X(notch) X(antenna) \
    X(ds) X(ds_lna) X(ds_edge) X(dc_mode) X(dc_speedup) X(dc_track) X(dc_period)

/* A marker flip written but not yet reported clean (events.c) */
#define MIRISDR_CHG_MAX         32

typedef struct mirisdr_chg
{
    uint32_t            tag;            /* the tune's change_tag */
    uint8_t             report;         /* a tune's flip: report it */
    uint8_t             level;          /* the marker after it */
    uint8_t             seen;           /* seen in the stream */
    uint8_t             known;          /* its tune returned */
    uint8_t             failed;
    uint8_t             dirty_sent;
    uint32_t            flags;          /* MIRISDR_CHANGE_* */
    int                 tuner;
    uint64_t            at_ns;          /* when written (host clock) */
    uint64_t            pos;            /* stream index when written */
    uint64_t            dirty;          /* DIRTY index */
    uint64_t            from;           /* the marker's packet: CLEAN counts from here */
    uint32_t            settle_us;
    mirisdr_tune_result_t res;
} mirisdr_chg_t;

struct mirisdr_dev {
    /* the device */
    libusb_context      *ctx;
    struct libusb_device_handle *dh;
    uint32_t            index;
    mirisdr_hw_flavour_t hw_flavour;
    int                 external_tuner;
    int                 fake;           /* mirisdr_open_null(): writes succeed without a device */
    int                 driver_active;
    int                 checking;       /* in a *_check(): refusals are not printed */
    int                 fw_ours;
    uint8_t             fw_pps_at;      /* status blocks, from the firmware block: internal RAM, */
    uint16_t            fw_anchor_at;   /* xdata, */
    uint8_t             fw_list_at;     /* internal RAM; 0 when the firmware has none */
    uint16_t            i2c_delay;
    uint32_t            i2c_clock_ns;
    int                 ee_size;        /* what the EEPROM's address reaches, 0 if none */

    /* the stream's format, see mirisdr_set_stream() */
    mirisdr_stream_config_t stream;     /* the stream asked for */
    uint32_t            rate;
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
    int                 decim_on;       /* the decimator is bypassed */
    int                 swap_iq;
    uint32_t            addr_step;

    /* The selected tuner: mirisdr_tuner_ctx_t's fields, in its order (tctx below keeps every
       tuner's, mirisdr_tuner_select()) */
    uint32_t            freq;
    int                 gain;
    int                 gain_reduction_lna;
    int                 gain_reduction_mixbuffer;
    int                 gain_reduction_mixer;
    int                 gain_reduction_baseband;
    int                 gain_stages_set;    /* a stage set by hand: retunes keep the split */
    int                 gain_reduction_amp; /* the board's amplifier before the tuner bypassed */
    int                 amp_db;             /* that amplifier's gain in the band in force, 0 none */
    int                 attenuator;         /* dB of the board's pad, 0 none (frontend) */
    int                 lna_cal_mhz;        /* the L-band LNA was last calibrated for this, 0 none */
    mirisdr_band_t      band;
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
    int                 bias;
    mirisdr_tune_config_t tune;         /* the tune asked for, see mirisdr_tune() */
    uint32_t            tune_lo;        /* Hz the LO reaches, as the received frequency */
    int                 tune_iq;        /* the tuner outputs running, MIRISDR_IQ_* */
    uint16_t            tune_exp;       /* the plan row's expander word of the tune in force, 0 none */
    uint32_t            tuner_reg[16];  /* last word written to each MSi001 register, 0 if never */
    uint16_t            tuner_valid;    /* per register: the tuner holds tuner_reg */
    uint8_t             tuner_turned;   /* the next word clocks a readback and is not latched */
    uint8_t             tuner_last;     /* the register of the last word the tuner took */
    uint32_t            tuner_regd;     /* last register 13 data, without the override bits */
    uint32_t            tuner_ovr13;    /* override bits added to the register 13 data */
    uint32_t            tuner_ovr14;    /* register 14 data, the override bits */
    int                 filter_cal;     /* IF filter code under the real crystal's XTALSEL, -1 unknown */
    uint8_t             tuner_gap;      /* tuned with another crystal's XTALSEL: hold filter_cal */
    uint8_t             dc_n;           /* DC calibration divider N of the XTALSEL tuned, 0 before */
    int                 notch;          /* MIRISDR_NOTCH_* asked for */
    int                 antenna;        /* MIRISDR_ANTENNA_* */
    int                 ds;             /* RSPdx-R2 direct sampling in force: GPIO_2 high */
    int                 ds_lna;         /* its amplifier on (expander A0 released) */
    uint32_t            ds_edge;        /* Hz, the low pass it runs through, 0 without it */
    enum {
        MIRISDR_DC_STATIC = 0,
        MIRISDR_DC_PERIODIC1,
        MIRISDR_DC_PERIODIC2,
        MIRISDR_DC_PERIODIC3,
        MIRISDR_DC_ONE_SHOT,
        MIRISDR_DC_CONTINUOUS
    } dc_mode;                          /* dc offset calibration */
    int                 dc_speedup;
    int                 dc_track;       /* tracking duration */
    int                 dc_period;      /* refresh period */

    /* the tuners: the selected one lives in the fields above, every one's here as last
       left (the stream's thread reads a tuner's DC removal from here) */
    mirisdr_tuner_ctx_t tctx[MIRISDR_TUNERS];
    int                 tuner_cur;

    /* the board around the tuners: its front end, register 8 and the pins */
    uint32_t            fe_flags;       /* frontend.flags in force: the board's, not a tuner's */
    mirisdr_frontend_raw_t fe;          /* the front end words in force, as the tune result has them */
    uint32_t            fe_raw;         /* MIRISDR_FRONTEND_RAW_* of them replaced by the tune */
    uint16_t            exp_iodir[2];   /* SDRplay expanders' direction words sent, 0 bits pulled low */
    int                 exp_valid;      /* bit n: expander n holds exp_iodir[n] */
    int                 exp_stale;      /* bit n: a list wrote expander n, send both bytes again, no setup */
    int                 ds_lna_db;      /* the board's, from its table entry: 0 for MIRISDR_DS_LNA_DB */
    uint32_t            edge_pin[2];    /* LO Hz where a plan row's VCO search pinned: [0] the lowest at its
                                           high end, [1] the highest at its low end, 0 none seen. What a
                                           tune check or a scan list takes, without the hardware to ask */
    int                 edge_try;       /* a tune is trying an edge row's own band: no fallback */
    int                 reg8;
    uint32_t            reg8_sent;      /* last word written to register 8 */
    int                 reg8_valid;     /* register 8 still holds reg8_sent */
    uint8_t             gpio_hold;      /* the open config's gpio_input_mask */
    /* pins the application has taken over, and what it wants them doing */
    uint8_t             gpio_mask;
    uint8_t             gpio_dir;
    uint8_t             gpio_val;

    /* register writes: the list being built for one request (reg.c), and the running
       mirisdr_tune()'s settling and marker */
    uint8_t             batch[256];     /* register words held for one list request */
    int                 batch_n;
    int                 batch_depth;
    int                 batch_running;  /* a tune list may still be running */
    uint32_t            batch_us;       /* the device's time to the list's end so far */
    uint32_t            batch_ready_us; /* its settling, from the list start */
    int                 batch_timed;    /* it holds a tune's words */
    int                 batch_mark;     /* it holds the marker, batch_mark_us in */
    uint32_t            batch_mark_us;
    uint32_t            kind_us;        /* settling of front-end words being written */
    uint32_t            synth_us;       /* settling of the synthesizer word */
    int                 call_on;        /* a mirisdr_tune() is running */
    int                 call_mark;      /* the running mirisdr_tune() flips the marker */
    int                 call_marked;    /* it flipped the marker */
    uint64_t            call_ref_ns;    /* its marker reached the device, 0 not yet */
    uint64_t            call_ready_ns;  /* when its words will have settled */
    uint32_t            call_lo;        /* LO of the last synthesizer word, 0 unknown */
    int                 mark_level;     /* the stream marker as last written */
    uint64_t            mark_sent_ns;   /* last flip sent (host clock) */
    int                 mark_pending;   /* a flip waits in the list being built */

    /* change reports (events.c), under ev_lock */
    struct {
        mirisdr_change_cb_t cb;
        void            *ctx;
        mirisdr_chg_t   ring[MIRISDR_CHG_MAX];  /* the flips, oldest first */
        uint32_t        head, n;
        int             req;            /* the next flip is the running tune's */
        uint32_t        tag;
        int             slot;           /* the running tune's entry, -1 none */
        uint64_t        pos;            /* stream index after the last packet */
        uint64_t        last_ns;        /* last flip written (host clock) */
        int             unsure;         /* flips cannot be matched: report UNCERTAIN */
        uint64_t        unsure_from;
    }                   chg;

    /* streaming (async.c). async_status: MIRISDR_ASYNC_*, changed from more than one
       thread, so only through mirisdr_async_get(), _set() and _move() below */
    int                 async_status;
    mirisdr_read_async_cb_t cb;
    void                *cb_ctx;
    size_t              xfer_buf_num;
    int                 xfer_inflight;  /* transfers submitted and not yet completed */
    int                 xfer_draining;  /* completions must not resubmit */
    int                 xfer_abandoned; /* a run gave up on xfer while some were in flight */
    struct libusb_transfer **xfer;
    mirisdr_lock_t      xfer_lock;      /* held by whatever uses or frees xfer from more than
                                           one thread: a stop, a restart, the run's last drain */
    int                 xfer_kind;      /* the transfer and alt setting xfer was made for */
    uint8_t             xfer_alt;
    unsigned char       **xfer_buf;
    int                 xfer_buf_devmem;
    int                 xfer_buf_slow;  /* reading from USB buffers is slow */
    size_t              xfer_buf_size;
    size_t              xfer_out_len;   /* 0 under baseband, which keeps its own */
    size_t              user_out_len;   /* the callback's buffer size as asked */
    size_t              xfer_out_pos;
    unsigned char       *xfer_out;
    unsigned char       fill_zeros[4096];   /* gap fill handed to the callback */

    /* the packets: their counter, conversion and statistics (convert/) */
    uint32_t            addr;
    uint32_t            addr_seen;      /* the last header's counter, as read */
    int                 addr_valid;
    int                 addr_restart;   /* stream start since the last block */
    int                 addr_dropped;   /* blocks dropped since that start */
    uint8_t             *samples;
    int                 samples_size;
    mirisdr_stream_stats_t stats;
    int                 stats_head;     /* next block starts a delivered buffer */
    int                 sync_run;       /* consecutive bad blocks, drives resync */
    uint8_t             bulk_carry[2 * 1024 + 48];  /* a leftover and the next transfer's head */
    int                 bulk_carry_n;   /* leftover, under 1036 bytes */
    int                 bulk_lost;      /* off the grid, counted as one resync */
    int                 bulk_wait;      /* transfers still queued with the old phase */
    /* the stream's count as of its last block header, for the control thread: count_seq odd
       while the stream thread writes them, so a reader sees all three of one block */
    int                 count_seq;
    uint64_t            count_index;    /* stats.samples + stats.lost */
    uint32_t            count_addr;     /* addr */
    uint64_t            count_gaps;

    /* gaps, from the conversion that finds them to the callback that delivers them */
    int                 gap_fill;
    int                 gap_track;      /* read_async: keep the queue and insert fills */
    struct {
        uint64_t at;                    /* delivered byte position, fills included */
        uint64_t samples;
        uint32_t filled;
    }                   gapq[64];
    int                 gapq_n;
    struct {
        uint32_t off;                   /* byte offset in this conversion's output */
        uint32_t n;                     /* samples */
    }                   fills[64];
    int                 fills_n;
    uint64_t            conv_samples;   /* stats.samples when this conversion began */
    uint64_t            conv_filled;
    uint64_t            fed_bytes;      /* bytes passed on to the callbacks' buffering, fills included */
    uint64_t            cb_bytes;       /* bytes handed to the application before this buffer, since cb_base */
    uint64_t            cb_base;        /* samples handed out before the last restart */
    uint64_t            cb_lost;        /* samples missing, not filled, before this buffer */
    mirisdr_buffer_info_t cb_info;

    /* the down-converter in front of the callback (baseband.c): NULL without baseband. With
       two tuners bb takes tuner 0 off the I converter and bb2 tuner 1 off the Q one */
    struct mirisdr_bb   *bb;
    struct mirisdr_bb   *bb2;
    uint8_t             *bb_split;      /* two tuners: a buffer's I values, then its Q values */
    size_t              bb_split_cap;
    int                 bb_path;        /* MIRISDR_BASEBAND_* the stream runs */
    int                 bb_stages;      /* half-band stages after the shift or the converter */
    uint32_t            bb_rate;        /* output samples per second */
    uint32_t            bb_if;          /* Hz, the low IF the stream was planned for: the NCO's */
    uint32_t            bb_if2;         /* Hz, tuner 1's IF with two tuners (MIRISDR_BASEBAND_DUAL) */
    int                 bb_ds;          /* bb_if is a directly sampled frequency, not mirrored */
    int                 bb_filling;     /* the samples on their way are gap fill's zeros */

    /* the sync read path (sync.c) */
    int                 sync_ready;     /* the sync path has started the stream */
    uint8_t             *sync_in;       /* one bulk transfer as read */
    uint8_t             *sync_out;      /* it converted, not yet handed out */
    int                 sync_len;
    int                 sync_pos;
    int                 sync_xlen;      /* next read length, short once to shift the grid */

    /* stream events from the packet headers, with ev_ctx under ev_lock, a pair */
    mirisdr_stream_event_cb_t ev_cb;
    void                *ev_ctx;
    mirisdr_lock_t      ev_lock;
    uint16_t            ev_last;        /* the previous packet's header bytes 4-5 */
    uint8_t             ev_valid;       /* ev_last holds a packet of this stream */
    uint8_t             ev_missed;      /* packets were lost since ev_last */
    int                 ev_mark_base;   /* first packet's marker is compared with this */

    /* IR receiver, see ir.c */
    uint32_t            ir_tick_ns;     /* counter period */
    uint32_t            ir_start;       /* sample counter when this pulse started */
    uint8_t             ir_prev;        /* last header byte 6 */
    uint8_t             ir_have;
    uint8_t             ir_level;       /* the level of the current pulse */
    uint8_t             ir_partial;     /* pulse beginning was missing */
    mirisdr_ir_cb_t     ir_cb;
    void                *ir_ctx;

    /* PPS and USB SOF timestamps (pps.c) */
    struct {
        int64_t         base;           /* stream index where the packet count was zeroed */
        uint64_t        gaps0;          /* stats.gaps at the anchor */
        uint8_t         edge0;
        uint8_t         stale;          /* the latched edge predates the anchor */
        uint32_t        irq_last;
        uint32_t        irq_high;
        uint8_t         anchor_valid;
        int             src_sof;        /* watch USB frames, not GPIO_0 */
        uint8_t         seen;           /* an interval matching the clock has been seen */
        double          k_min;          /* least a SOF handler was seen to cost */
        /* the SOF source's line: edges on it, the oldest, one between and the last, with
           their microframes counted from the oldest */
        int             grid_have;
        int             grid_off;       /* edges off it in a row */
        int64_t         grid_a, grid_m, grid_b;
        int64_t         grid_ka, grid_km, grid_kb;
        int             grid_lb;        /* the last edge's frame and microframe */
        int64_t         grid_cand;      /* the last edge off it, and its */
        int             grid_lc;
        double          chk_expect;     /* the packet interval the grid is for */
        int             chk_valid;      /* the last edge checked, and what was found */
        uint8_t         chk_edges;
        uint32_t        chk_packet;
        uint8_t         chk_guard;
    }                   pps;
    /* the blocks an anchor may have damaged, see mirisdr_anchor_open() */
    struct {
        int             open;           /* anchors asked for, by the control thread */
        int             known;          /* the last of them placed in the count */
        uint32_t        from;           /* its first block that may be damaged */
        uint32_t        to;             /* and its last */
        int             shut;           /* the last the stream thread is past, its own */
    }                   anc;

    /* scan lists, see scan.c */
    struct {
        mirisdr_list_entry_t *rec;      /* compiling: register writes are kept here instead */
        uint32_t        rec_n;
        uint32_t        rec_max;
        const struct mirisdr_scan *run; /* running */
        volatile int    on;             /* the stream side follows it */
        uint64_t        total;          /* chunks to load, 0 forever */
        uint64_t        loaded;         /* chunks loaded so far */
        volatile uint32_t restarts;     /* the list ran dry and was loaded again */
        uint32_t        restarts_seen;
        uint32_t        burst;          /* packets per stream interrupt */
        uint64_t        g;              /* stream side: chunks seen, counting the marks */
        uint32_t        in;             /* tune events seen in the chunk */
        uint8_t         flags;          /* for the next report */
        uint64_t        pos;            /* stream index after the last packet */
        int             unsure;         /* hops not reported until the next chunk */
        mirisdr_scan_cb_t cb;
        void            *ctx;
    }                   scan;
};

/* The SDRplay boards with the front end on an SPI expander (sdrplay.c) */
#define MIRISDR_HW_EXPANDER(f)  (((f) == MIRISDR_HW_RSP1B) || ((f) == MIRISDR_HW_RSPDXR2) || ((f) == MIRISDR_HW_RSPDUO) || \
                                 ((f) == MIRISDR_HW_RSP2) || ((f) == MIRISDR_HW_RSP1A))

#if defined(_WIN32)
static void mirisdr_xfer_lock_init (mirisdr_dev_t *p)    { InitializeCriticalSection(&p->xfer_lock);
                                                           InitializeCriticalSection(&p->ev_lock); }
static void mirisdr_xfer_lock_destroy (mirisdr_dev_t *p) { DeleteCriticalSection(&p->xfer_lock);
                                                           DeleteCriticalSection(&p->ev_lock); }
static void mirisdr_xfer_lock (mirisdr_dev_t *p)         { EnterCriticalSection(&p->xfer_lock); }
static void mirisdr_xfer_unlock (mirisdr_dev_t *p)       { LeaveCriticalSection(&p->xfer_lock); }
static void mirisdr_ev_lock (mirisdr_dev_t *p)           { EnterCriticalSection(&p->ev_lock); }
static void mirisdr_ev_unlock (mirisdr_dev_t *p)         { LeaveCriticalSection(&p->ev_lock); }
#else
static void mirisdr_xfer_lock_init (mirisdr_dev_t *p)
{
    pthread_mutexattr_t a;

    pthread_mutexattr_init(&a);
    pthread_mutexattr_settype(&a, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&p->xfer_lock, &a);
    pthread_mutex_init(&p->ev_lock, &a);
    pthread_mutexattr_destroy(&a);
}
static void mirisdr_xfer_lock_destroy (mirisdr_dev_t *p) { pthread_mutex_destroy(&p->xfer_lock);
                                                           pthread_mutex_destroy(&p->ev_lock); }
static void mirisdr_xfer_lock (mirisdr_dev_t *p)         { pthread_mutex_lock(&p->xfer_lock); }
static void mirisdr_xfer_unlock (mirisdr_dev_t *p)       { pthread_mutex_unlock(&p->xfer_lock); }
static void mirisdr_ev_lock (mirisdr_dev_t *p)           { pthread_mutex_lock(&p->ev_lock); }
static void mirisdr_ev_unlock (mirisdr_dev_t *p)         { pthread_mutex_unlock(&p->ev_lock); }
#endif

/* A monotonic clock in ns, and usleep where the compiler has none */
#if defined(_WIN32)
static uint64_t mirisdr_now_ns (void)
{
    LARGE_INTEGER f, c;

    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    return (uint64_t) ((double) c.QuadPart * 1e9 / (double) f.QuadPart);
}
#else
static uint64_t mirisdr_now_ns (void)
{
    struct timespec t;

    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t) t.tv_sec * 1000000000u + (uint64_t) t.tv_nsec;
}
#endif
#if defined(_MSC_VER)
static void usleep (unsigned us) { Sleep((us + 999) / 1000); }
#endif

/* A change of the stream state that depends on the state it leaves is one compare
   and swap, so a cancel cannot land between the check and the change */
#if defined(_MSC_VER)
#include <intrin.h>
static int mirisdr_async_get (mirisdr_dev_t *p)
{
    return (int) _InterlockedCompareExchange((volatile long *) &p->async_status, 0, 0);
}
static void mirisdr_async_set (mirisdr_dev_t *p, int s)
{
    _InterlockedExchange((volatile long *) &p->async_status, s);
}
static int mirisdr_async_move (mirisdr_dev_t *p, int from, int to)
{
    return _InterlockedCompareExchange((volatile long *) &p->async_status, to, from) == from;
}

static int mirisdr_atomic_xchg (int *at, int v)
{
    return (int) _InterlockedExchange((volatile long *) at, v);
}

/* adds d, returns the new value, d 0 reads it */
static int mirisdr_atomic_add (int *at, int d)
{
    return (int) _InterlockedExchangeAdd((volatile long *) at, d) + d;
}
#else
static int mirisdr_async_get (mirisdr_dev_t *p)
{
    return __atomic_load_n(&p->async_status, __ATOMIC_SEQ_CST);
}
static void mirisdr_async_set (mirisdr_dev_t *p, int s)
{
    __atomic_store_n(&p->async_status, s, __ATOMIC_SEQ_CST);
}
static int mirisdr_async_move (mirisdr_dev_t *p, int from, int to)
{
    return __atomic_compare_exchange_n(&p->async_status, &from, to, 0, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
}

static int mirisdr_atomic_xchg (int *at, int v)
{
    return __atomic_exchange_n(at, v, __ATOMIC_SEQ_CST);
}

/* adds d, returns the new value, d 0 reads it */
static int mirisdr_atomic_add (int *at, int d)
{
    return __atomic_add_fetch(at, d, __ATOMIC_SEQ_CST);
}
#endif
