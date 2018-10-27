/*
 * Copyright © 2018, VideoLAN and dav1d authors
 * Copyright © 2018, Two Orioles, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Copyright (c) 2001-2016, Alliance for Open Media. All rights reserved
 *
 * This source code is subject to the terms of the BSD 2 Clause License and
 * the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
 * was not distributed with this source code in the LICENSE file, you can
 * obtain it at www.aomedia.org/license/software. If the Alliance for Open
 * Media Patent License 1.0 was not distributed with this source code in the
 * PATENTS file, you can obtain it at www.aomedia.org/license/patent.
 */

#include "config.h"

#include <assert.h>
#include <stdlib.h>

#include "common/intops.h"

#include "src/cdef.h"

static const int8_t cdef_directions4[8 /* dir */][2 /* pass */] = {
    { -1 * 8 + 1, -2 * 8 + 2 },
    {  0 * 8 + 1, -1 * 8 + 2 },
    {  0 * 8 + 1,  0 * 8 + 2 },
    {  0 * 8 + 1,  1 * 8 + 2 },
    {  1 * 8 + 1,  2 * 8 + 2 },
    {  1 * 8 + 0,  2 * 8 + 1 },
    {  1 * 8 + 0,  2 * 8 + 0 },
    {  1 * 8 + 0,  2 * 8 - 1 }
};

static const int8_t cdef_directions8[8 /* dir */][2 /* pass */] = {
    { -1 * 16 + 1, -2 * 16 + 2 },
    {  0 * 16 + 1, -1 * 16 + 2 },
    {  0 * 16 + 1,  0 * 16 + 2 },
    {  0 * 16 + 1,  1 * 16 + 2 },
    {  1 * 16 + 1,  2 * 16 + 2 },
    {  1 * 16 + 0,  2 * 16 + 1 },
    {  1 * 16 + 0,  2 * 16 + 0 },
    {  1 * 16 + 0,  2 * 16 - 1 }
};
static const uint8_t cdef_pri_taps[2][2] = { { 4, 2 }, { 3, 3 } };
static const uint8_t cdef_sec_taps[2][2] = { { 2, 1 }, { 2, 1 } };

static inline int constrain(const int diff, const int threshold,
                            const int damping)
{
    if (!threshold) return 0;
    const int shift = imax(0, damping - ulog2(threshold));
    return apply_sign(imin(abs(diff), imax(0, threshold - (abs(diff) >> shift))),
                      diff);
}

/*
 * <code partially copied from libaom>
 */

#define CDEF_VERY_LARGE (30000)

static void fill(uint16_t *tmp, const ptrdiff_t stride,
                 const int w, const int h)
{
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++)
            tmp[x] = CDEF_VERY_LARGE;
        tmp += stride;
    }
}

/* Smooth in the direction detected. */
static void cdef_filter_block_c(pixel *const dst, const ptrdiff_t dst_stride,
                                /*const*/ pixel *const top[2],
                                const int w, const int h, const int pri_strength,
                                const int sec_strength, const int dir,
                                const int damping, const enum CdefEdgeFlags edges)
{
    const ptrdiff_t tmp_stride = 16 >> (w == 4);
    assert((w == 4 || w == 8) && (h == 4 || h == 8));
    uint16_t tmp[192];  // 16*12 is the maximum value of tmp_stride * (h + 4)
    uint16_t *tmp2 = tmp + 2 * tmp_stride + 2;
    const uint8_t *const pri_taps = cdef_pri_taps[(pri_strength >> (BITDEPTH - 8)) & 1];
    const uint8_t *const sec_taps = cdef_sec_taps[(pri_strength >> (BITDEPTH - 8)) & 1];
    const int8_t (*cdef_directions)[2];

    assert(w == 4 || w == 8);
    cdef_directions = w == 4 ? cdef_directions4 : cdef_directions8;

    // fill extended input buffer
    int x_start = -2, x_end = w + 2, y_start = -2, y_end = h + 2;
    if (!(edges & HAVE_TOP)) {
        fill(tmp, tmp_stride, w + 4, 2);
        y_start = 0;
    }
    if (!(edges & HAVE_BOTTOM)) {
        fill(tmp + (h + 2) * tmp_stride, tmp_stride, w + 4, 2);
        y_end -= 2;
    }
    if (!(edges & HAVE_LEFT)) {
        fill(tmp + (2 + y_start) * tmp_stride, tmp_stride, 2, y_end - y_start);
        x_start = 0;
    }
    if (!(edges & HAVE_RIGHT)) {
        fill(tmp + (2 + y_start) * tmp_stride + w + 2, tmp_stride,
             2, y_end - y_start);
        x_end -= 2;
    }
    for (int y = y_start; y < 0; y++)
        for (int x = x_start; x < x_end; x++)
            tmp2[y * tmp_stride + x] = top[y & 1][x];
    for (int y = 0; y < y_end; y++)
        for (int x = x_start; x < x_end; x++)
            tmp2[y * tmp_stride + x] = dst[y * PXSTRIDE(dst_stride) + x];

    // run actual filter
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int sum = 0;
            const int px = dst[y * PXSTRIDE(dst_stride) + x];
            int max = px, min = px;
            for (int k = 0; k < 2; k++) {
                const int8_t off1 = cdef_directions[dir][k];
                const int p0 = tmp2[y * tmp_stride + x + off1];
                const int p1 = tmp2[y * tmp_stride + x - off1];
                sum += pri_taps[k] * constrain(p0 - px, pri_strength, damping);
                sum += pri_taps[k] * constrain(p1 - px, pri_strength, damping);
                if (p0 != CDEF_VERY_LARGE) max = imax(p0, max);
                if (p1 != CDEF_VERY_LARGE) max = imax(p1, max);
                min = imin(p0, min);
                min = imin(p1, min);
                const int8_t off2 = cdef_directions[(dir + 2) & 7][k];
                const int s0 = tmp2[y * tmp_stride + x + off2];
                const int s1 = tmp2[y * tmp_stride + x - off2];
                const int8_t off3 = cdef_directions[(dir + 6) & 7][k];
                const int s2 = tmp2[y * tmp_stride + x + off3];
                const int s3 = tmp2[y * tmp_stride + x - off3];
                if (s0 != CDEF_VERY_LARGE) max = imax(s0, max);
                if (s1 != CDEF_VERY_LARGE) max = imax(s1, max);
                if (s2 != CDEF_VERY_LARGE) max = imax(s2, max);
                if (s3 != CDEF_VERY_LARGE) max = imax(s3, max);
                min = imin(s0, min);
                min = imin(s1, min);
                min = imin(s2, min);
                min = imin(s3, min);
                sum += sec_taps[k] * constrain(s0 - px, sec_strength, damping);
                sum += sec_taps[k] * constrain(s1 - px, sec_strength, damping);
                sum += sec_taps[k] * constrain(s2 - px, sec_strength, damping);
                sum += sec_taps[k] * constrain(s3 - px, sec_strength, damping);
            }
            dst[y * PXSTRIDE(dst_stride) + x] =
                iclip(px + ((8 + sum - (sum < 0)) >> 4), min, max);
        }
    }
}

/*
 * </code partially copied from libaom>
 */

#define cdef_fn(w, h) \
static void cdef_filter_block_##w##x##h##_c(pixel *const dst, \
                                            const ptrdiff_t stride, \
                                            /*const*/ pixel *const top[2], \
                                            const int pri_strength, \
                                            const int sec_strength, \
                                            const int dir, \
                                            const int damping, \
                                            const enum CdefEdgeFlags edges) \
{ \
    cdef_filter_block_c(dst, stride, top, w, h, pri_strength, sec_strength, \
                        dir, damping, edges); \
}

cdef_fn(4, 4);
cdef_fn(4, 8);
cdef_fn(8, 8);

static int cdef_find_dir_c(const pixel *img, const ptrdiff_t stride,
                           unsigned *const var)
{
    int partial_sum_hv[2][8] = { { 0 } };
    int partial_sum_diag[2][15] = { { 0 } };
    int partial_sum_alt[4][11] = { { 0 } };

    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 8; x++) {
            const int px = (img[x] >> (BITDEPTH - 8)) - 128;

            partial_sum_diag[0][     y       +  x      ] += px;
            partial_sum_alt [0][     y       + (x >> 1)] += px;
            partial_sum_hv  [0][     y                 ] += px;
            partial_sum_alt [1][3 +  y       - (x >> 1)] += px;
            partial_sum_diag[1][7 +  y       -  x      ] += px;
            partial_sum_alt [2][3 - (y >> 1) +  x      ] += px;
            partial_sum_hv  [1][                x      ] += px;
            partial_sum_alt [3][    (y >> 1) +  x      ] += px;
        }
        img += PXSTRIDE(stride);
    }

    unsigned cost[8] = { 0 };
    for (int n = 0; n < 8; n++) {
        cost[2] += partial_sum_hv[0][n] * partial_sum_hv[0][n];
        cost[6] += partial_sum_hv[1][n] * partial_sum_hv[1][n];
    }
    cost[2] *= 105;
    cost[6] *= 105;

    static const uint16_t div_table[7] = { 840, 420, 280, 210, 168, 140, 120 };
    for (int n = 0; n < 7; n++) {
        const int d = div_table[n];
        cost[0] += (partial_sum_diag[0][n]      * partial_sum_diag[0][n] +
                    partial_sum_diag[0][14 - n] * partial_sum_diag[0][14 - n]) * d;
        cost[4] += (partial_sum_diag[1][n]      * partial_sum_diag[1][n] +
                    partial_sum_diag[1][14 - n] * partial_sum_diag[1][14 - n]) * d;
    }
    cost[0] += partial_sum_diag[0][7] * partial_sum_diag[0][7] * 105;
    cost[4] += partial_sum_diag[1][7] * partial_sum_diag[1][7] * 105;

    for (int n = 0; n < 4; n++) {
        unsigned *const cost_ptr = &cost[n * 2 + 1];
        for (int m = 0; m < 5; m++)
            *cost_ptr += partial_sum_alt[n][3 + m] * partial_sum_alt[n][3 + m];
        *cost_ptr *= 105;
        for (int m = 0; m < 3; m++) {
            const int d = div_table[2 * m + 1];
            *cost_ptr += (partial_sum_alt[n][m]      * partial_sum_alt[n][m] +
                          partial_sum_alt[n][10 - m] * partial_sum_alt[n][10 - m]) * d;
        }
    }

    int best_dir = 0;
    unsigned best_cost = cost[0];
    for (int n = 1; n < 8; n++) {
        if (cost[n] > best_cost) {
            best_cost = cost[n];
            best_dir = n;
        }
    }

    *var = (best_cost - (cost[best_dir ^ 4])) >> 10;
    return best_dir;
}

void bitfn(dav1d_cdef_dsp_init)(Dav1dCdefDSPContext *const c) {
    c->dir = cdef_find_dir_c;
    c->fb[0] = cdef_filter_block_8x8_c;
    c->fb[1] = cdef_filter_block_4x8_c;
    c->fb[2] = cdef_filter_block_4x4_c;

#if HAVE_ASM && ARCH_X86 && BITDEPTH == 8
    bitfn(dav1d_cdef_dsp_init_x86)(c);
#endif
}
