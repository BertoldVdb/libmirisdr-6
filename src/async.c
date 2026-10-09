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

#include "async.h"

static int mirisdr_stream_adc (mirisdr_dev_t *p);

static void mirisdr_cb_call (mirisdr_dev_t *p, unsigned char *buf, uint32_t len) {
    mirisdr_buffer_info_t *in = &p->cb_info;
    uint64_t ub = mirisdr_unit_bytes(p), start = p->cb_bytes, end = start + len, later = 0;
    int k = 0, j;

    memset(in, 0, sizeof *in);
    in->sample = p->cb_base + start / ub;
    in->adc = mirisdr_stream_adc(p);
    in->rate = p->rate;
    in->type = (p->format == MIRISDR_FORMAT_504_S8) ? MIRISDR_SAMPLE_S8 : MIRISDR_SAMPLE_S16;

    /* the queue's gaps up to this buffer's end, in order */
    while (k < p->gapq_n && p->gapq[k].at < end) {
        uint64_t unfilled = p->gapq[k].samples - p->gapq[k].filled, keep = 0;

        /* zeros running past the end go on in the next buffer, as a gap of
           nothing but fill at its start */
        if (p->gapq[k].at + (uint64_t) p->gapq[k].filled * ub > end)
            keep = (p->gapq[k].at + (uint64_t) p->gapq[k].filled * ub - end) / ub;

        if (in->gaps_len < MIRISDR_GAPS_MAX) {
            mirisdr_gap_t *g = &in->gaps[in->gaps_len++];

            g->offset  = p->gapq[k].at > start ? (uint32_t) ((p->gapq[k].at - start) / ub) : 0;
            g->filled  = p->gapq[k].filled - (uint32_t) keep;
            g->samples = p->gapq[k].samples - keep;
        }
        in->gap_samples += p->gapq[k].samples - keep;

        /* a gap at the start lies before its first sample */
        if (p->gapq[k].at <= start) p->cb_lost += unfilled;
        else later += unfilled;

        if (keep) {
            p->gapq[k].at = end;
            p->gapq[k].samples = keep;
            p->gapq[k].filled = (uint32_t) keep;
            break;
        }
        k++;
    }
    for (j = k; j < p->gapq_n; j++) p->gapq[j - k] = p->gapq[j];
    p->gapq_n -= k;

    in->index = in->sample + p->cb_lost;

    if (p->bb) mirisdr_bb_feed(p, buf, len);
    else p->cb(buf, len, p->cb_ctx);

    p->cb_lost += later;
    p->cb_bytes = end;
}

/* uložení dat */
static int mirisdr_feed_async (mirisdr_dev_t *p, unsigned char *samples, uint32_t bytes) {
    uint32_t i;

    if (!p) goto failed;
    if (!p->cb) goto failed;

    /* automatická velikost */
    if (!p->xfer_out_len) {
        /* přímé zaslání */
        mirisdr_cb_call(p, samples, bytes);
    /* fixní velikost bufferu bez předchozích dat */
    } else if (p->xfer_out_pos == 0) {
        /* buffer přesně odpovídá - málo časté, přímé zaslání */
        if (bytes == p->xfer_out_len) {
            mirisdr_cb_call(p, samples, bytes);
        /* buffer je kratší */
        } else if (bytes < p->xfer_out_len) {
            memcpy(p->xfer_out, samples, bytes);
            p->xfer_out_pos = bytes;
        /* buffer je delší */
        } else {
            /* muže být i x násobkem délky */
            for (i = 0;; i+= p->xfer_out_len) {
                if (i + p->xfer_out_len > bytes) {
                    if (bytes > i) {
                        memcpy(p->xfer_out, samples + i, bytes - i);
                        p->xfer_out_pos = bytes - i;
                    }
                    break;
                }
                mirisdr_cb_call(p, samples + i, p->xfer_out_len);
            }
        }
    /* data jsou přesně, využije se interní buffer */
    } else if (p->xfer_out_pos + bytes == p->xfer_out_len) {
        memcpy(p->xfer_out + p->xfer_out_pos, samples, bytes);
        mirisdr_cb_call(p, p->xfer_out, p->xfer_out_len);
        p->xfer_out_pos = 0;
    /* není dostatek dat */
    } else if (p->xfer_out_pos + bytes < p->xfer_out_len) {
        memcpy(p->xfer_out + p->xfer_out_pos, samples, bytes);
        p->xfer_out_pos+= bytes;
    /* dat je více než potřebujeme, nejsložitější případ */
    } else {
        memcpy(p->xfer_out + p->xfer_out_pos, samples, p->xfer_out_len - p->xfer_out_pos);
        mirisdr_cb_call(p, p->xfer_out, p->xfer_out_len);
        for (i = p->xfer_out_len - p->xfer_out_pos;; i+= p->xfer_out_len) {
            if (i + p->xfer_out_len > bytes) {
                if (bytes > i) {
                    memcpy(p->xfer_out, samples + i, bytes - i);
                    p->xfer_out_pos = bytes - i;
                } else {
                    p->xfer_out_pos = 0;
                }
                break;
            }
            mirisdr_cb_call(p, samples + i, p->xfer_out_len);
        }
    }
    return 0;

failed:
    return -1;
}

static void mirisdr_feed_converted (mirisdr_dev_t *p, unsigned char *samples, uint32_t bytes) {
    static unsigned char zeros[4096];
    uint32_t at = 0, ub = mirisdr_unit_bytes(p), k;
    uint64_t z;
    int i;

    for (i = 0; i < p->fills_n; i++) {
        if (p->fills[i].off > at) {
            mirisdr_feed_async(p, samples + at, p->fills[i].off - at);
            p->fed_bytes += p->fills[i].off - at;
            at = p->fills[i].off;
        }
        for (z = (uint64_t) p->fills[i].n * ub; z; z -= k) {
            k = z > sizeof zeros ? (uint32_t) sizeof zeros : (uint32_t) z;
            mirisdr_feed_async(p, zeros, k);
            p->fed_bytes += k;
        }
    }
    if (bytes > at) {
        mirisdr_feed_async(p, samples + at, bytes - at);
        p->fed_bytes += bytes - at;
    }
    p->fills_n = 0;
}

static uint8_t *samples_realloc(mirisdr_dev_t *p, int size)
{
    if(p->samples_size < size)
    {
        if(p->samples)
            free(p->samples);
        p->samples=malloc(size);
        p->samples_size=p->samples ? size : 0;
    }
    return p->samples;
}

#define MIRISDR_BLOCK_OUT_MAX   2016

static int mirisdr_convert_bulk (mirisdr_dev_t *p, uint8_t *src, uint8_t *dst, int cnt)
{
    switch (p->format) {
    case MIRISDR_FORMAT_252_S16:
    case MIRISDR_FORMAT_504_REAL_S16:
        return mirisdr_samples_convert_252_s16(p, src, dst, cnt);
    case MIRISDR_FORMAT_336_S16:
    case MIRISDR_FORMAT_672_REAL_S16:
        return mirisdr_samples_convert_336_s16(p, src, dst, cnt);
    case MIRISDR_FORMAT_384_S16:
    case MIRISDR_FORMAT_768_REAL_S16:
        return mirisdr_samples_convert_384_s16(p, src, dst, cnt);
    case MIRISDR_FORMAT_504_S16:
        return mirisdr_samples_convert_504_s16(p, src, dst, cnt);
    case MIRISDR_FORMAT_504_S8:
        return mirisdr_samples_convert_504_s8(p, src, dst, cnt);
    default:
        return 0;
    }
}

/* Our firmware stamps header bytes 8-11 of every buffer at startup - bytes the
   capture engine never writes - so the mark rides along in every packet the
   part sends.  Byte 12 carries which of the four ring buffers it came from,
   and 13-15 are spare. */
static const uint8_t mirisdr_hdr_magic[4] = { 'B', 'V', 'D', 'B' };

static int mirisdr_bulk_stamped (const uint8_t *block)
{
    return !memcmp(block + 8, mirisdr_hdr_magic, sizeof(mirisdr_hdr_magic));
}

/* A block with the next block's stamp in place behind it */
static int mirisdr_bulk_pair (const uint8_t *b)
{
    return mirisdr_bulk_stamped(b) && mirisdr_bulk_stamped(b + 1024);
}

static void mirisdr_bulk_slip (mirisdr_dev_t *p)
{
    if (!p->bulk_lost) {
        p->bulk_lost = 1;
        p->stats.resyncs++;
    }
}

static int mirisdr_parse_bulk (mirisdr_dev_t *p, const uint8_t *src, int n, uint8_t *dst)
{
    uint8_t *st = p->bulk_carry;
    int out = 0, pos = 0, k, c, h, l;

    if (p->bulk_carry_n) {
        h = n < 1024 + 12 ? n : 1024 + 12;
        memcpy(st + p->bulk_carry_n, src, (size_t) h);
        l = p->bulk_carry_n + h;

        /* the blocks that start in the leftover */
        while (pos < p->bulk_carry_n && pos + 1024 + 12 <= l) {
            if (mirisdr_bulk_pair(st + pos)) {
                out += mirisdr_convert_bulk(p, st + pos, dst + out, 1024);
                p->bulk_lost = 0;
                pos += 1024;
                continue;
            }
            mirisdr_bulk_slip(p);
            for (pos++; pos + 1024 + 12 <= l && !mirisdr_bulk_pair(st + pos); pos++) ;
        }

        /* a transfer too short to get past them: keep it all */
        if (pos < p->bulk_carry_n) {
            memmove(st, st + pos, (size_t) (l - pos));
            p->bulk_carry_n = l - pos;
            return out;
        }

        pos -= p->bulk_carry_n;
        p->bulk_carry_n = 0;
    }

    while (pos + 1024 <= n) {
        if (pos + 1024 + 12 > n) {
            /* the last block, its successor in the next transfer */
            if (!p->bulk_lost && pos + 1024 == n && n == DEFAULT_BULK_BUFFER && mirisdr_bulk_stamped(src + pos)) {
                out += mirisdr_convert_bulk(p, (uint8_t *) src + pos, dst + out, 1024);
                pos += 1024;
            }
            break;
        }

        if (mirisdr_bulk_pair(src + pos)) {
            for (c = 1; pos + (c + 1) * 1024 + 12 <= n && mirisdr_bulk_stamped(src + pos + (c + 1) * 1024); c++) ;
            out += mirisdr_convert_bulk(p, (uint8_t *) src + pos, dst + out, c * 1024);
            p->bulk_lost = 0;
            pos += c * 1024;
            continue;
        }

        mirisdr_bulk_slip(p);
        for (k = pos + 1; k + 1024 + 12 <= n && !mirisdr_bulk_pair(src + k); k++) ;
        if (k + 1024 + 12 > n) {
            /* none: keep the tail, a pair may straddle into the next transfer */
            if (pos < n - (1024 + 11)) pos = n - (1024 + 11);
            break;
        }
        fprintf(stderr, "libmirisdr: off the block grid, %d bytes skipped\n", k - pos);
        pos = k;
    }

    if (pos < n) {
        memcpy(st, src + pos, (size_t) (n - pos));
        p->bulk_carry_n = n - pos;
    }

    return out;
}

static int mirisdr_bulk_next_len (mirisdr_dev_t *p)
{
    if (!p->bulk_lost && p->bulk_carry_n && !(p->bulk_carry_n % 512))
        return DEFAULT_BULK_BUFFER - p->bulk_carry_n;

    return DEFAULT_BULK_BUFFER;
}

/* Where does the 1 kB block grid really start in this buffer?
   Returns -1 when nothing in it looks like the grid. */
static int mirisdr_bulk_phase (mirisdr_dev_t *p, const uint8_t *b, int n)
{
	int k;

	if (n < 3 * 1024) return -1;

	/* The stamp settles it outright: two of them a block apart cannot be
	   sample data. */
	for (k = 0; k < 1024; k++)
		if (!memcmp(b + k + 8, mirisdr_hdr_magic, sizeof(mirisdr_hdr_magic)) &&
		    !memcmp(b + k + 1024 + 8, mirisdr_hdr_magic, sizeof(mirisdr_hdr_magic)))
			return k;

	/* Firmware that does not stamp so fall back to the counter:
     * a run one block apart stepping by addr_step is the grid. */
	for (k = 0; k < 1024; k++)
	{
		uint32_t c0 = b[k]        | b[k+1]<<8    | b[k+2]<<16    | (uint32_t) b[k+3]<<24;
		uint32_t c1 = b[k+1024]   | b[k+1025]<<8 | b[k+1026]<<16 | (uint32_t) b[k+1027]<<24;
		uint32_t c2 = b[k+2048]   | b[k+2049]<<8 | b[k+2050]<<16 | (uint32_t) b[k+2051]<<24;

		if ((c1 - c0 == p->addr_step) && (c2 - c1 == p->addr_step)) return k;
	}

	return -1;
}

/*
 * It is possible to configure how many 1kB blocks at a time the DSP will
 * hand over to the USB controller. If this value is too low not all buffer
 * memory is used and the chance of gaps increases. If it is too high (eg 4
 * packets in a 1 slot isochronous transfer), the stream stops.
 */
static uint32_t mirisdr_alt_burst(uint8_t alt)
{
	switch (alt)
	{
	case 1:  return 3;          /* isochronous, 3 x 1024 per microframe */
	case 2:  return 1;          /* isochronous, 1 x 1024 */
	case 4:  return 2;          /* isochronous, 2 x 1024 (requires custom firmware) */
	default: return 4;          /* bulk */
	}
}

static uint32_t mirisdr_burst(mirisdr_dev_t *p)
{
	return mirisdr_alt_burst(p->alt_setting);
}

/* The host polls an isochronous endpoint only a few hundred us after the transfers
   are submitted. A stream started before then overflows the device's at 7-8 Msps
   and it skips samples, so wait a bit with starting :) */
static void mirisdr_iso_settle (mirisdr_dev_t *p) {
    if (p->transfer != MIRISDR_TRANSFER_ISOC) return;
#if defined (_WIN32) && !defined(__MINGW32__)
    Sleep(1);
#else
    usleep(1000);
#endif
}

static int mirisdr_async_drain (mirisdr_dev_t *p, int rounds) {
    size_t i;
    int r;
    struct timeval tv = {1, 0};
    p->xfer_draining = 1;
    while (p->xfer_inflight > 0) {
        if (rounds-- <= 0) return -1;
        for (i = 0; i < p->xfer_buf_num; i++)
            if (p->xfer[i]) libusb_cancel_transfer(p->xfer[i]);
        if ((r = libusb_handle_events_timeout(p->ctx, &tv)) < 0) {
            fprintf( stderr, "libusb_handle_events returned: %d\n", r);
            if (r == LIBUSB_ERROR_INTERRUPTED) continue; /* stray */
            return -1;
        }
    }
    return 0;
}

/* volání pro zasílání dat */
static void LIBUSB_CALL _libusb_callback (struct libusb_transfer *xfer) {
    size_t i;
    int len, bytes = 0;
    static unsigned char *iso_packet_buf;
    mirisdr_dev_t *p = (mirisdr_dev_t*) xfer->user_data;
    uint8_t *samples = p->samples;

    if (!p) goto failed;
    /* one completion per submission, whatever its status */
    p->xfer_inflight--;

    /* zpracujeme pouze kompletní přenos */
    if (xfer->status == LIBUSB_TRANSFER_COMPLETED) {
        p->stats_head = 1;
        p->conv_samples = p->stats.samples;
        p->conv_filled = 0;
        p->fills_n = 0;
        /*
         * Určení správné velikosti bufferu, tato část musí být provedena
         * v jednom kroku, jinak může dojít ke změně formátu uprostřed procesu,
         * druhá možnost je používat lock.
         */
        switch (xfer->type) {
        case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
            switch (p->format) {
            case MIRISDR_FORMAT_252_S16:
                if (!(samples = samples_realloc(p, 504 * DEFAULT_ISO_BUFFERS * DEFAULT_ISO_PACKETS * 2))) goto failed;
                for (i = 0; i < DEFAULT_ISO_PACKETS; i++) {
                    struct libusb_iso_packet_descriptor *packet = &xfer->iso_packet_desc[i];

                    /* buffer_simple je pouze pro stejně velké pakety */
                    if ((packet->actual_length > 0) &&
                        (iso_packet_buf = libusb_get_iso_packet_buffer_simple(xfer, i))) {
                        /* menší velikost než 3072 nevadí, je běžný násobek 1024, cokoliv jiného je chyba */
                        len = mirisdr_samples_convert_252_s16(p, iso_packet_buf, samples + bytes, packet->actual_length);
                        bytes+= len;
                    }
                }
                break;
            case MIRISDR_FORMAT_336_S16:
                if (!(samples = samples_realloc(p, 672 * DEFAULT_ISO_BUFFERS * DEFAULT_ISO_PACKETS * 2))) goto failed;
                for (i = 0; i < DEFAULT_ISO_PACKETS; i++) {
                    struct libusb_iso_packet_descriptor *packet = &xfer->iso_packet_desc[i];
                    if ((packet->actual_length > 0) &&
                        (iso_packet_buf = libusb_get_iso_packet_buffer_simple(xfer, i))) {
                        len = mirisdr_samples_convert_336_s16(p, iso_packet_buf, samples + bytes, packet->actual_length);
                        bytes+= len;
                    }
                }
                break;
            case MIRISDR_FORMAT_384_S16:
                if (!(samples = samples_realloc(p, 768 * DEFAULT_ISO_BUFFERS * DEFAULT_ISO_PACKETS * 2))) goto failed;
                for (i = 0; i < DEFAULT_ISO_PACKETS; i++) {
                    struct libusb_iso_packet_descriptor *packet = &xfer->iso_packet_desc[i];
                    if ((packet->actual_length > 0) &&
                        (iso_packet_buf = libusb_get_iso_packet_buffer_simple(xfer, i))) {
                        len = mirisdr_samples_convert_384_s16(p, iso_packet_buf, samples + bytes, packet->actual_length);
                        bytes+= len;
                    }
                }
                break;
            case MIRISDR_FORMAT_504_S16:
                if (!(samples = samples_realloc(p, 1008 * DEFAULT_ISO_BUFFERS * DEFAULT_ISO_PACKETS * 2))) goto failed;
                for (i = 0; i < DEFAULT_ISO_PACKETS; i++) {
                    struct libusb_iso_packet_descriptor *packet = &xfer->iso_packet_desc[i];
                    if ((packet->actual_length > 0) &&
                        (iso_packet_buf = libusb_get_iso_packet_buffer_simple(xfer, i))) {
                        len = mirisdr_samples_convert_504_s16(p, iso_packet_buf, samples + bytes, packet->actual_length);
                        bytes+= len;
                    }
                }
                break;
            case MIRISDR_FORMAT_504_S8:
                if (!(samples = samples_realloc(p, 1008 * DEFAULT_ISO_BUFFERS * DEFAULT_ISO_PACKETS))) goto failed;
                for (i = 0; i < DEFAULT_ISO_PACKETS; i++) {
                    struct libusb_iso_packet_descriptor *packet = &xfer->iso_packet_desc[i];
                    if ((packet->actual_length > 0) &&
                        (iso_packet_buf = libusb_get_iso_packet_buffer_simple(xfer, i))) {
                        len = mirisdr_samples_convert_504_s8(p, iso_packet_buf, samples + bytes, packet->actual_length);
                        bytes+= len;
                    }
                }
                break;
            case MIRISDR_FORMAT_504_REAL_S16:
                if (!(samples = samples_realloc(p, 504 * DEFAULT_ISO_BUFFERS * DEFAULT_ISO_PACKETS * 2))) goto failed;
                for (i = 0; i < DEFAULT_ISO_PACKETS; i++) {
                    struct libusb_iso_packet_descriptor *packet = &xfer->iso_packet_desc[i];
                    if ((packet->actual_length > 0) &&
                        (iso_packet_buf = libusb_get_iso_packet_buffer_simple(xfer, i))) {
                        len = mirisdr_samples_convert_252_s16(p, iso_packet_buf, samples + bytes, packet->actual_length);
                        bytes+= len;
                    }
                }
                break;
            case MIRISDR_FORMAT_672_REAL_S16:
                if (!(samples = samples_realloc(p, 672 * DEFAULT_ISO_BUFFERS * DEFAULT_ISO_PACKETS * 2))) goto failed;
                for (i = 0; i < DEFAULT_ISO_PACKETS; i++) {
                    struct libusb_iso_packet_descriptor *packet = &xfer->iso_packet_desc[i];
                    if ((packet->actual_length > 0) &&
                        (iso_packet_buf = libusb_get_iso_packet_buffer_simple(xfer, i))) {
                        len = mirisdr_samples_convert_336_s16(p, iso_packet_buf, samples + bytes, packet->actual_length);
                        bytes+= len;
                    }
                }
                break;
            case MIRISDR_FORMAT_768_REAL_S16:
                if (!(samples = samples_realloc(p, 768 * DEFAULT_ISO_BUFFERS * DEFAULT_ISO_PACKETS * 2))) goto failed;
                for (i = 0; i < DEFAULT_ISO_PACKETS; i++) {
                    struct libusb_iso_packet_descriptor *packet = &xfer->iso_packet_desc[i];
                    if ((packet->actual_length > 0) &&
                        (iso_packet_buf = libusb_get_iso_packet_buffer_simple(xfer, i))) {
                        len = mirisdr_samples_convert_384_s16(p, iso_packet_buf, samples + bytes, packet->actual_length);
                        bytes+= len;
                    }
                }
                break;
            }
            break;
        case LIBUSB_TRANSFER_TYPE_BULK:
            if (!(samples = samples_realloc(p, (DEFAULT_BULK_BUFFER / 1024 + 2) * MIRISDR_BLOCK_OUT_MAX))) goto failed;
            if (p->fw_ours)
                bytes = mirisdr_parse_bulk(p, xfer->buffer, xfer->actual_length, samples);
            else
                bytes = mirisdr_convert_bulk(p, xfer->buffer, samples, xfer->actual_length);
            break;
        default:
            fprintf( stderr, "not isoc or bulk transfer type on usb device: %u\n", p->index);
            goto failed;
        }

        if (bytes > 0 || p->fills_n) mirisdr_feed_converted(p, samples, bytes);
        /* draining: the transfer is done, and should not be reused */
        if (p->xfer_draining || (p->async_status != MIRISDR_ASYNC_RUNNING)) return;

        /* with the stamp: back onto the grid once the queue has drained of the old phase */
        if ((xfer->type == LIBUSB_TRANSFER_TYPE_BULK) && p->fw_ours)
        {
            xfer->length = DEFAULT_BULK_BUFFER;
            if (p->bulk_wait > 0) {
                p->bulk_wait--;
            } else if ((xfer->length = mirisdr_bulk_next_len(p)) != DEFAULT_BULK_BUFFER) {
                p->bulk_wait = (int) p->xfer_buf_num;
            }
        }

        /* without the stamp, find the grid from the counters and shift the transfers onto it */
        if ((xfer->type == LIBUSB_TRANSFER_TYPE_BULK) && !p->fw_ours)
        {
            if(p->sync_run > (int)p->xfer_buf_num)
            {
                /* Rarely, packets containing a count of zero are received, which should not
                 * be seen as desync */
                int phase = mirisdr_bulk_phase(p, xfer->buffer, xfer->actual_length);

                /* A shift only reaches the stream once the transfers already
                   queued behind it have drained, and every one of those still
                   carries the old phase. Counting that wait in transfers is
                   a bug: sync_run counts blocks, and one transfer holds
                   sixteen of them, so the next buffer re-triggered the shift
                   and it oscillates. */
                p->sync_run = -(int) (p->xfer_buf_num * (DEFAULT_BULK_BUFFER / 1024));

                /* Blocks start at phase in every transfer, so one transfer
                   1024 - phase short puts the next on the grid. */
                if (phase > 0) {
                    p->stats.resyncs++;
                    xfer->length = DEFAULT_BULK_BUFFER - 1024 + phase;
                    fprintf(stderr,"libmirisdr: block grid is %d bytes out, shifting.\n", phase);
                } else {
                    xfer->length = DEFAULT_BULK_BUFFER;
                }
            }else
                xfer->length = DEFAULT_BULK_BUFFER;
        }
                /* pokračujeme dalším přenosem */
        if (libusb_submit_transfer(xfer) < 0) {
            fprintf( stderr, "error re-submitting URB on device %u\n", p->index);
            goto failed;
        }
        p->xfer_inflight++;
    } else if (xfer->status != LIBUSB_TRANSFER_CANCELLED) {
        fprintf( stderr, "error async transfer status %d on device %u\n", xfer->status, p->index);
        goto failed;
    }

    return;

failed:
    mirisdr_cancel_async(p);
    /* stav failed má absolutní přednost */
    p->async_status = MIRISDR_ASYNC_FAILED;
}

/* ukončení async části */
int mirisdr_cancel_async (mirisdr_dev_t *p) {
    if (!p) goto failed;

    switch (p->async_status) {
    case MIRISDR_ASYNC_INACTIVE:
        if (p->async_starting) {
            p->cancel_pending = 1;
            return 0;
        }
        goto canceled;
    case MIRISDR_ASYNC_CANCELING:
        goto canceled;
    case MIRISDR_ASYNC_RUNNING:
    case MIRISDR_ASYNC_PAUSED:
        p->async_status = MIRISDR_ASYNC_CANCELING;
        break;
    case MIRISDR_ASYNC_FAILED:
        goto failed;
    }

    return 0;

failed:
    return -1;

canceled:
    return -2;
}

/* ukončení async části včetně čekání */
int mirisdr_cancel_async_now (mirisdr_dev_t *p) {
    if (!p) goto failed;

    switch (p->async_status) {
    case MIRISDR_ASYNC_INACTIVE:
        if (!p->async_starting) goto done;
        p->cancel_pending = 1;
        break;
    case MIRISDR_ASYNC_CANCELING:
        break;
    case MIRISDR_ASYNC_RUNNING:
    case MIRISDR_ASYNC_PAUSED:
        p->async_status = MIRISDR_ASYNC_CANCELING;
        break;
    case MIRISDR_ASYNC_FAILED:
        goto failed;
    }

    /* cyklujeme dokud není vše ukončeno */
    while (p->async_starting ||
           ((p->async_status != MIRISDR_ASYNC_INACTIVE) &&
            (p->async_status != MIRISDR_ASYNC_FAILED)))
#if defined (_WIN32) && !defined(__MINGW32__)
    Sleep(20);
#else
    usleep(20000);
#endif

done:
    return 0;

failed:
    return -1;
}

/* alokace asynchronních bufferů */
/* Byte reads from a usbfs buffer cost less than a nanosecond where the controller
   is IO coherent and hundreds where it is not. The platform does not expose this
   information to the application, so we have to measure it. */
#define MIRISDR_PROBE_BYTES     2048
#define MIRISDR_PROBE_ROUNDS    5
#define MIRISDR_PROBE_RATIO     8

static double mirisdr_read_cost (const volatile uint8_t *buf, int len, int rounds) {
    volatile uint32_t sum = 0;
    double best = 0;
    int i, r;

    for (i = 0; i < len; i++) sum+= buf[i];        /* map the pages first */

    for (r = 0; r < rounds; r++) {
        struct timespec a, b;
        double ns;

        clock_gettime(CLOCK_MONOTONIC, &a);
        for (i = 0; i < len; i++) sum+= buf[i];
        clock_gettime(CLOCK_MONOTONIC, &b);

        ns = ((double) (b.tv_sec - a.tv_sec) * 1e9 + (b.tv_nsec - a.tv_nsec)) / len;

        if (!r || (ns < best)) best = ns;
    }

    return best;
}

/* Slow usbfs buffers are swapped for malloc'd ones: usbfs then copies in the
   kernel, instead of the converters (or a memcpy) reading uncached memory. */
static void mirisdr_probe_buffers (mirisdr_dev_t *p) {
    uint8_t *cached;
    double slow, fast;
    size_t i;

    p->xfer_buf_slow = 0;

    if (!p->xfer_buf_devmem || !p->xfer_buf || !p->xfer_buf[0]) return;
    if (!(cached = malloc(MIRISDR_PROBE_BYTES))) return;

    memset(cached, 0, MIRISDR_PROBE_BYTES);

    fast = mirisdr_read_cost(cached, MIRISDR_PROBE_BYTES, MIRISDR_PROBE_ROUNDS);
    slow = mirisdr_read_cost(p->xfer_buf[0], MIRISDR_PROBE_BYTES, MIRISDR_PROBE_ROUNDS);

    free(cached);

    p->xfer_buf_slow = slow > fast * MIRISDR_PROBE_RATIO;

#if MIRISDR_DEBUG >= 1
    fprintf(stderr, "transfer buffers: %.1f ns/byte against %.1f cached, %s\n",
            slow, fast, p->xfer_buf_slow ? "using malloc buffers" : "read in place");
#endif

    if (!p->xfer_buf_slow) return;

    for (i = 0; i < p->xfer_buf_num; i++) {
        libusb_dev_mem_free(p->dh, p->xfer_buf[i], p->xfer_buf_size);
        p->xfer_buf[i] = NULL;
    }

    p->xfer_buf_devmem = 0;

    /* a failure leaves NULL, which the caller checks */
    for (i = 0; i < p->xfer_buf_num; i++)
        p->xfer_buf[i] = malloc(p->xfer_buf_size);
}

static int mirisdr_async_free (mirisdr_dev_t *p);

static int mirisdr_async_alloc (mirisdr_dev_t *p) {
    size_t i;

    if (!p->xfer) {
        if (!(p->xfer = calloc(p->xfer_buf_num, sizeof(*p->xfer)))) goto failed;

        for (i = 0; i < p->xfer_buf_num; i++) {
            switch (p->transfer) {
            case MIRISDR_TRANSFER_BULK:
                p->xfer[i] = libusb_alloc_transfer(0);
                break;
            case MIRISDR_TRANSFER_ISOC:
                p->xfer[i] = libusb_alloc_transfer(DEFAULT_ISO_PACKETS);
                break;
            }
            if (!p->xfer[i]) goto failed;
        }
    }

    if (!p->xfer_buf) {
        size_t j, bufsz;

        if (!(p->xfer_buf = calloc(p->xfer_buf_num, sizeof(*p->xfer_buf)))) goto failed;

        switch (p->transfer) {
        case MIRISDR_TRANSFER_ISOC:
            bufsz = (size_t) DEFAULT_ISO_BUFFER * mirisdr_burst(p) * DEFAULT_ISO_PACKETS;
            break;
        default:
            bufsz = DEFAULT_BULK_BUFFER;
            break;
        }
        p->xfer_buf_size = bufsz;

        /* Prefer usbfs DMA-coherent buffers: saves ~5% CPU on N150 */
        p->xfer_buf_devmem = (getenv("MIRISDR_NO_ZEROCOPY") == NULL);
        for (i = 0; p->xfer_buf_devmem && i < p->xfer_buf_num; i++) {
            p->xfer_buf[i] = libusb_dev_mem_alloc(p->dh, bufsz);
            if (!p->xfer_buf[i]) { p->xfer_buf_devmem = 0; break; }
        }
        if (!p->xfer_buf_devmem) {
            for (j = 0; j < i; j++) {
                libusb_dev_mem_free(p->dh, p->xfer_buf[j], bufsz);
                p->xfer_buf[j] = NULL;
            }
            for (i = 0; i < p->xfer_buf_num; i++)
                if (!(p->xfer_buf[i] = malloc(bufsz))) goto failed;
        }

        mirisdr_probe_buffers(p);

        for (i = 0; i < p->xfer_buf_num; i++)
            if (!p->xfer_buf[i]) goto failed;
    }

    if ((!p->xfer_out) &&
        (p->user_out_len)) {
        if (!(p->xfer_out = malloc(p->user_out_len * sizeof(*p->xfer_out)))) goto failed;
    }

    return 0;

failed:
    fprintf(stderr, "libmirisdr: out of memory for the transfer buffers\n");
    mirisdr_async_free(p);
    return -1;
}

/* uvolnění asynchronních bufferů */
static int mirisdr_async_free (mirisdr_dev_t *p) {
    size_t i;

    if (p->xfer) {
        for (i = 0; i < p->xfer_buf_num; i++) {
            if (p->xfer[i]) libusb_free_transfer(p->xfer[i]);
        }

        free(p->xfer);
        p->xfer = NULL;
    }

    if (p->xfer_buf) {
        for (i = 0; i < p->xfer_buf_num; i++) {
            if (!p->xfer_buf[i]) continue;
            if (p->xfer_buf_devmem)
                libusb_dev_mem_free(p->dh, p->xfer_buf[i], p->xfer_buf_size);
            else
                free(p->xfer_buf[i]);
        }

        free(p->xfer_buf);
        p->xfer_buf = NULL;
    }

    if (p->xfer_out) {
        free(p->xfer_out);
        p->xfer_out = NULL;
    }

    return 0;
}

/* spuštění async části */
static int mirisdr_read_async_run (mirisdr_dev_t *p, mirisdr_read_async_cb_t cb, void *ctx, uint32_t num, uint32_t len);

/* TODO: this cancel_pending is a slight race condition, it solves the test problem now but I will fix properly later */
int mirisdr_read_async (mirisdr_dev_t *p, mirisdr_read_async_cb_t cb, void *ctx, uint32_t num, uint32_t len) {
    int r;

    if (!p) return -1;
    if (p->async_status != MIRISDR_ASYNC_INACTIVE) return -1;

    p->cancel_pending = 0;
    p->async_starting = 1;
    r = mirisdr_read_async_run(p, cb, ctx, num, len);
    p->async_starting = 0;
    p->cancel_pending = 0;

    return r;
}

static int mirisdr_read_async_run (mirisdr_dev_t *p, mirisdr_read_async_cb_t cb, void *ctx, uint32_t num, uint32_t len) {
    size_t i;
    int r;
    int transfer_failed = 0;
    int stop_asked = 0;
    struct timeval tv = {1, 0};

    if (!p) goto failed;
    if (!p->dh) goto failed;

    /* nedovolíme spustit jiný stav než neaktivní */
    if (p->async_status != MIRISDR_ASYNC_INACTIVE) goto failed;

    p->cb = cb;
    p->cb_ctx = ctx;

    p->xfer_buf_num = (num == 0) ? DEFAULT_BUF_NUMBER : num;
    /* jde o fixní velikost výstupního bufferu; baseband keeps its own */
    p->user_out_len = len;
    p->xfer_out_len = p->bb ? 0 : len;
    mirisdr_bb_start(p);
    p->xfer_out_pos = 0;
#if MIRISDR_DEBUG >= 1
    fprintf( stderr, "async read on device %u, buffers: %lu, output size: ",
                                p->index, (long)p->xfer_buf_num);
    if (p->xfer_out_len) {
        fprintf( stderr, "%lu", (long)p->xfer_out_len);
    } else {
        fprintf( stderr, "auto");
    }
#endif
    memset(&p->stats, 0, sizeof(p->stats));
    p->sync_run = 0;
    p->addr_valid = 0;
    p->bulk_carry_n = 0;
    p->bulk_lost = 0;
    p->bulk_wait = 0;
    p->gap_track = 1;
    p->gapq_n = 0;
    p->fills_n = 0;
    p->fed_bytes = 0;
    p->cb_bytes = 0;
    p->cb_base = 0;
    p->cb_lost = 0;
    p->ev_valid = 0;
    p->sync_ready = 0;
    /* použití správného rozhraní které zasílá data - není kritické */
    switch (p->transfer) {
    case MIRISDR_TRANSFER_BULK:
#if MIRISDR_DEBUG >= 1
        fprintf( stderr, ", transfer: bulk\n");
#endif
        if ((r = libusb_set_interface_alt_setting(p->dh, 0, p->alt_setting)) < 0) {
            fprintf( stderr, "failed to use alternate setting for Bulk mode on miri usb device %u with code %d\n", p->index, r);
        }
        break;
    case MIRISDR_TRANSFER_ISOC:
#if MIRISDR_DEBUG >= 1
        fprintf( stderr, ", transfer: isochronous\n");
#endif
        if ((r = libusb_set_interface_alt_setting(p->dh, 0, p->alt_setting)) < 0) {
            fprintf( stderr, "failed to use alternate setting for Isochronous mode on miri usb device %u with code %d\n", p->index, r);
        }
        break;
    default:
        fprintf( stderr, "\nunsupported transfer type on miri usb device %u\n", p->index);
        goto failed;
    }

    if (mirisdr_async_alloc(p) < 0) goto failed;

    /* spustíme přenosy */
    for (i = 0; i < p->xfer_buf_num; i++) {
        switch (p->transfer) {
        case MIRISDR_TRANSFER_BULK:
            libusb_fill_bulk_transfer(p->xfer[i],
                                      p->dh,
                                      0x81,
                                      p->xfer_buf[i],
                                      DEFAULT_BULK_BUFFER,
                                      _libusb_callback,
                                      (void*) p,
                                      DEFAULT_BULK_TIMEOUT);
            break;
        case MIRISDR_TRANSFER_ISOC:
            libusb_fill_iso_transfer(p->xfer[i],
                                     p->dh,
                                     0x81,
                                     p->xfer_buf[i],
                                     DEFAULT_ISO_BUFFER * mirisdr_burst(p) * DEFAULT_ISO_PACKETS,
                                     DEFAULT_ISO_PACKETS,
                                     _libusb_callback,
                                     (void*) p,
                                     DEFAULT_ISO_TIMEOUT);
            libusb_set_iso_packet_lengths(p->xfer[i], DEFAULT_ISO_BUFFER * mirisdr_burst(p));
            break;
        default:
            fprintf( stderr, "unsupported transfer type\n");
            goto failed_free;
        }

                r = libusb_submit_transfer(p->xfer[i]);
        if (r < 0) {
            fprintf(stderr, "Failed to submit transfer %lu reason: %d\n", i, r);
            /* the ones before it are in flight: get them back first */
            if (mirisdr_async_drain(p, 5) < 0) goto failed;
            goto failed_free;
        }
        p->xfer_inflight++;
    }

    /* spustíme streamování dat */
    p->xfer_draining = 0;
    mirisdr_iso_settle(p);
    mirisdr_streaming_start(p);
    p->async_status = MIRISDR_ASYNC_RUNNING;

    while (p->async_status != MIRISDR_ASYNC_INACTIVE) {
        if (p->cancel_pending) {
            p->cancel_pending = 0;
            if (p->async_status != MIRISDR_ASYNC_FAILED) p->async_status = MIRISDR_ASYNC_CANCELING;
        }

        /* počkáme na další událost */
        if ((r = libusb_handle_events_timeout(p->ctx, &tv)) < 0) {
            fprintf( stderr, "libusb_handle_events returned: %d\n", r);
            if (r == LIBUSB_ERROR_INTERRUPTED) continue; /* stray */
            goto failed_free;
        }


        /* dochází k ukončení */
        if (p->async_status == MIRISDR_ASYNC_CANCELING) {
            if (!p->xfer) {
                p->async_status = MIRISDR_ASYNC_INACTIVE;
                break;
            }

            /* Ask the firmware to stop while the transfers are still queued,
             * so the engine can drain */
            if (!stop_asked) {
                mirisdr_streaming_stop(p);
                stop_asked = 1;
            }

            /* A stalled endpoint may never return its transfers, so do not wait
             * for them for ever -- that hangs the caller's stream thread. Give
             * up after a few seconds and leave without freeing: the transfers
             * are still owned by libusb, and freeing an in-flight transfer
             * corrupts memory. libusb_close()/libusb_exit() in mirisdr_close()
             * cleans up from here. */
                        if (mirisdr_async_drain(p, 5) < 0) {
                fprintf(stderr, "libmirisdr: transfers would not cancel, "
                                "abandoning them\n");
                p->async_status = MIRISDR_ASYNC_INACTIVE;
                return -1;
            }
            /* every completion is in: nothing of ours is on libusb's list */
            p->async_status = MIRISDR_ASYNC_INACTIVE;
            break;
        } else if (p->async_status == MIRISDR_ASYNC_FAILED) {
            /* Do NOT free the transfers here: on this path some of them are
             * still submitted, and libusb_free_transfer() on an in-flight
             * transfer corrupts memory (it crashed the host process every time
             * a bulk endpoint stalled). Convert the failure into a normal
             * cancel so the branch above drains every transfer properly, and
             * remember that it failed so we still return an error. */
            transfer_failed = 1;
            p->async_status = MIRISDR_ASYNC_CANCELING;
        }
    }

    /* dealokujeme buffer */
    mirisdr_async_free(p);

    /* ukončíme streamování dat */
#if defined (_WIN32) && !defined(__MINGW32__)
    Sleep(20);
#else
    usleep(20000);
#endif
    mirisdr_streaming_stop(p);
    /* je vhodné ukončit i adc, jenže pak by při dalším otevření bylo nutné provést inicializaci */

    if (transfer_failed) {
        p->async_status = MIRISDR_ASYNC_INACTIVE;
        return -1;
    }

    return 0;

failed_free:
    mirisdr_async_free(p);

failed:
    return -1;
}

/* spuštění streamování */
int mirisdr_start_async (mirisdr_dev_t *p) {
    size_t i;

    /* nedovolíme jiný stav než pozastavený */
    if (p->async_status != MIRISDR_ASYNC_PAUSED) goto failed;

        /* reset interního bufferu */
    p->xfer_out_pos = 0;
    p->xfer_draining = 0;

    for (i = 0; i < p->xfer_buf_num; i++) {
        if (!p->xfer[i]) continue;
        if (libusb_submit_transfer(p->xfer[i])< 0) {
            goto failed;
        }
        p->xfer_inflight++;
    }

    if (p->async_status != MIRISDR_ASYNC_PAUSED) goto failed;

    mirisdr_bb_restart(p);
    mirisdr_iso_settle(p);
    mirisdr_streaming_start(p);

    p->async_status = MIRISDR_ASYNC_RUNNING;

    return 0;

failed:
    return -1;
}

/* The buffer being filled is dropped at a restart, and the next format may take
   other bytes a sample: count what was not handed out as lost and start the byte
   positions again. The buffers' sample and index carry on across it. */
static void mirisdr_cb_rebase (mirisdr_dev_t *p) {
    uint64_t ub = mirisdr_unit_bytes(p), dropped = p->xfer_out_pos / ub, filled = 0, missing = 0;
    int k;

    for (k = 0; k < p->gapq_n; k++) {
        filled += p->gapq[k].filled;
        missing += p->gapq[k].samples;
    }

    /* the real samples of the dropped part, and all of the gaps not reached */
    p->cb_lost += (dropped > filled ? dropped - filled : 0) + missing;
    p->cb_base += p->cb_bytes / ub;
    p->cb_bytes = 0;
    p->fed_bytes = 0;
    p->gapq_n = 0;
    p->xfer_out_pos = 0;
}

/* zastavení streamování */
int mirisdr_stop_async (mirisdr_dev_t *p) {

    /* nedovolíme jiný stav než spuštěný */
    if (p->async_status != MIRISDR_ASYNC_RUNNING) goto failed;

    /* ask the firmware to stop while transfers are still queued, as otherwise the ISR never
     * fires, and we can't disable the capture engine */
    mirisdr_streaming_stop(p);

    /* The firmware sends the buffer it holds when it stops, as two 512 byte packets.
       Cancelling between them leaves the second in its FIFO, to start the next
       stream half a block out. */
#if defined(_WIN32) && !defined(__MINGW32__)
    Sleep(2);
#else
    usleep(2000);
#endif

        /* every transfer back from libusb before the pause is declared */
    if (mirisdr_async_drain(p, 10) < 0) goto failed;

    if (p->async_status != MIRISDR_ASYNC_RUNNING) goto failed;

    mirisdr_cb_rebase(p);

    p->async_status = MIRISDR_ASYNC_PAUSED;

    return 0;

failed:
    return -1;
}

int mirisdr_get_buffer_info (mirisdr_dev_t *p, mirisdr_buffer_info_t *info) {
    if (!p || !info) return -1;

    *info = p->cb_info;

    return 0;
}

int mirisdr_set_gap_fill (mirisdr_dev_t *p, int on) {
    if (!p) return -1;

    p->gap_fill = !!on;
    p->stream.gap_fill = p->gap_fill;

    return 0;
}
