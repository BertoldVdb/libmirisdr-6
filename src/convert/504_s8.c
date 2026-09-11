
/*
 * 8 bitový formát
 * 1024 bajtů odpovídá 1008 hodnotám
 * struktura:
 *   16b hlavička
 *  1008b bloků obsahujících 1008 8b hodnot
 */
static int mirisdr_samples_convert_504_s8 (mirisdr_dev_t *p, unsigned char* src, uint8_t *dst, int cnt) {
    int i, ret = 0;

    /* only whole blocks: a transfer cut short by a resynchronisation ends in a
       partial one, and the header of the next block is not where it looks */
    for (i = 0; i + 1024 <= cnt; i+= 1024, ret+= 1008) {
        unsigned char *b = src + i;

        /* ztracená data */
        mirisdr_addr_next(p, b[3] << 24 | b[2] << 16 | b[1] << 8 | b[0] << 0, 504);

        memcpy(dst + ret, b + 16, 1008);
    }

    return ret;
}
