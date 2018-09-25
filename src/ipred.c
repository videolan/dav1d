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

#include "config.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "common/intops.h"

#include "src/ipred.h"

#define sz_grid(l_fn) \
l_fn( 4,  4) \
l_fn( 4,  8) \
l_fn( 4, 16) \
l_fn( 8,  4) \
l_fn( 8,  8) \
l_fn( 8, 16) \
l_fn( 8, 32) \
l_fn(16,  4) \
l_fn(16,  8) \
l_fn(16, 16) \
l_fn(16, 32) \
l_fn(16, 64) \
l_fn(32,  8) \
l_fn(32, 16) \
l_fn(32, 32) \
l_fn(32, 64) \
l_fn(64, 16) \
l_fn(64, 32) \
l_fn(64, 64)

static __attribute__((noinline)) void
splat_dc_c(pixel *dst, const ptrdiff_t stride,
           const int w, const int h, const unsigned dc)
{
    assert(dc <= (1 << BITDEPTH) - 1);
#if BITDEPTH == 8
    if (w > 4) {
        const uint64_t dcN = dc * 0x0101010101010101ULL;
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x += sizeof(dcN))
                *((uint64_t *) &dst[x]) = dcN;
            dst += PXSTRIDE(stride);
        }
    } else {
        const unsigned dcN = dc * 0x01010101U;
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x += sizeof(dcN))
                *((unsigned *) &dst[x]) = dcN;
            dst += PXSTRIDE(stride);
        }
    }
#else
    const uint64_t dcN = dc * 0x0001000100010001ULL;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x += sizeof(dcN) >> 1)
            *((uint64_t *) &dst[x]) = dcN;
        dst += PXSTRIDE(stride);
    }
#endif
}

#define dc_lfn(w, h, dir, dc_gen) \
static void dc##dir##_##w##x##h##_c(pixel *dst, const ptrdiff_t stride, \
                                    const pixel *const topleft, const int a) \
{ \
    dc_gen; \
    splat_dc_c(dst, stride, w, h, dc); \
}

#define dc1d_lfns(width, height, sh1, sh2) \
dc_lfn(width, height, top, unsigned dc = width >> 1; \
                           for (int i = 0; i < width; i++) \
                               dc += topleft[1 + i]; \
                           dc >>= sh1) \
dc_lfn(width, height, left, unsigned dc = height >> 1; \
                            for (int i = 0; i < height; i++) \
                                dc += topleft[-(1 + i)]; \
                            dc >>= sh2)

dc1d_lfns( 4,  4, 2, 2)
dc1d_lfns( 4,  8, 2, 3)
dc1d_lfns( 4, 16, 2, 4)
dc1d_lfns( 8,  4, 3, 2)
dc1d_lfns( 8,  8, 3, 3)
dc1d_lfns( 8, 16, 3, 4)
dc1d_lfns( 8, 32, 3, 5)
dc1d_lfns(16,  4, 4, 2)
dc1d_lfns(16,  8, 4, 3)
dc1d_lfns(16, 16, 4, 4)
dc1d_lfns(16, 32, 4, 5)
dc1d_lfns(16, 64, 4, 6)
dc1d_lfns(32,  8, 5, 3)
dc1d_lfns(32, 16, 5, 4)
dc1d_lfns(32, 32, 5, 5)
dc1d_lfns(32, 64, 5, 6)
dc1d_lfns(64, 16, 6, 4)
dc1d_lfns(64, 32, 6, 5)
dc1d_lfns(64, 64, 6, 6)

#define dc2d_lfn(width, height, dc_gen) \
dc_lfn(width, height,, unsigned dc = (width + height) >> 1; \
                       for (int i = 0; i < width; i++) \
                           dc += topleft[i + 1]; \
                       for (int i = 0; i < height; i++) \
                           dc += topleft[-(i + 1)]; \
                       dc_gen)

dc2d_lfn( 4,  4, dc >>= 3)
dc2d_lfn( 4,  8, dc = iclip_pixel(0x5556 * dc >> 18))
dc2d_lfn( 4, 16, dc = iclip_pixel(0x3334 * dc >> 18))
dc2d_lfn( 8,  4, dc = iclip_pixel(0x5556 * dc >> 18))
dc2d_lfn( 8,  8, dc >>= 4)
dc2d_lfn( 8, 16, dc = iclip_pixel(0x5556 * dc >> 19))
dc2d_lfn( 8, 32, dc = iclip_pixel(0x3334 * dc >> 19))
dc2d_lfn(16,  4, dc = iclip_pixel(0x3334 * dc >> 18))
dc2d_lfn(16,  8, dc = iclip_pixel(0x5556 * dc >> 19))
dc2d_lfn(16, 16, dc >>= 5)
dc2d_lfn(16, 32, dc = iclip_pixel(0x5556 * dc >> 20))
dc2d_lfn(16, 64, dc = iclip_pixel(0x3334 * dc >> 20))
dc2d_lfn(32,  8, dc = iclip_pixel(0x3334 * dc >> 19))
dc2d_lfn(32, 16, dc = iclip_pixel(0x5556 * dc >> 20))
dc2d_lfn(32, 32, dc >>= 6)
dc2d_lfn(32, 64, dc = iclip_pixel(0x5556 * dc >> 21))
dc2d_lfn(64, 16, dc = iclip_pixel(0x3334 * dc >> 20))
dc2d_lfn(64, 32, dc = iclip_pixel(0x5556 * dc >> 21))
dc2d_lfn(64, 64, dc >>= 7)

#define dc128_lfn(width, height) \
dc_lfn(width, height, 128, const unsigned dc = (1 << BITDEPTH) >> 1)

sz_grid(dc128_lfn)

static __attribute__((noinline)) void
v_c(pixel *dst, const ptrdiff_t stride,
    const pixel *const topleft, const int width, const int height)
{
    for (int y = 0; y < height; y++) {
        pixel_copy(dst, topleft + 1, width);
        dst += PXSTRIDE(stride);
    }
}

#define v_lfn(width, height) \
static void v_##width##x##height##_##c(pixel *dst, const ptrdiff_t stride, \
                                       const pixel *const topleft, const int a) \
{ \
    v_c(dst, stride, topleft, width, height); \
}

sz_grid(v_lfn)

static __attribute__((noinline)) void
h_c(pixel *dst, const ptrdiff_t stride,
    const pixel *const topleft, const int width, const int height)
{
    for (int y = 0; y < height; y++) {
        pixel_set(dst, topleft[-(1 + y)], width);
        dst += PXSTRIDE(stride);
    }
}

#define h_lfn(width, height) \
static void h_##width##x##height##_c(pixel *dst, const ptrdiff_t stride, \
                                     const pixel *const topleft, const int a) \
{ \
    h_c(dst, stride, topleft, width, height); \
}

sz_grid(h_lfn)

static __attribute__((noinline)) void
paeth_c(pixel *dst, const ptrdiff_t stride, const pixel *const tl_ptr,
        const int width, const int height)
{
    const int topleft = tl_ptr[0];
    for (int y = 0; y < height; y++) {
        const int left = tl_ptr[-(y + 1)];
        for (int x = 0; x < width; x++) {
            const int top = tl_ptr[1 + x];
            const int base = left + top - topleft;
            const int ldiff = abs(left - base);
            const int tdiff = abs(top - base);
            const int tldiff = abs(topleft - base);

            dst[x] = ldiff <= tdiff && ldiff <= tldiff ? left :
                     tdiff <= tldiff ? top : topleft;
        }
        dst += PXSTRIDE(stride);
    }
}

#define paeth_lfn(width, height) \
static void paeth_##width##x##height##_c(pixel *dst, const ptrdiff_t stride, \
                                         const pixel *const topleft, \
                                         const int a) \
{ \
    paeth_c(dst, stride, topleft, width, height); \
}

sz_grid(paeth_lfn)

static const uint8_t sm_weight_arrays[] = {
    // Unused, because we always offset by bs, which is at least 2.
    0, 0,
    // bs = 2
    255, 128,
    // bs = 4
    255, 149, 85, 64,
    // bs = 8
    255, 197, 146, 105, 73, 50, 37, 32,
    // bs = 16
    255, 225, 196, 170, 145, 123, 102, 84, 68, 54, 43, 33, 26, 20, 17, 16,
    // bs = 32
    255, 240, 225, 210, 196, 182, 169, 157, 145, 133, 122, 111, 101, 92, 83, 74,
    66, 59, 52, 45, 39, 34, 29, 25, 21, 17, 14, 12, 10, 9, 8, 8,
    // bs = 64
    255, 248, 240, 233, 225, 218, 210, 203, 196, 189, 182, 176, 169, 163, 156,
    150, 144, 138, 133, 127, 121, 116, 111, 106, 101, 96, 91, 86, 82, 77, 73, 69,
    65, 61, 57, 54, 50, 47, 44, 41, 38, 35, 32, 29, 27, 25, 22, 20, 18, 16, 15,
    13, 12, 10, 9, 8, 7, 6, 6, 5, 5, 4, 4, 4,
};

static __attribute__((noinline)) void
smooth_c(pixel *dst, const ptrdiff_t stride, const pixel *const topleft,
         const int width, const int height)
{
    const uint8_t *const weights_hor = &sm_weight_arrays[width];
    const uint8_t *const weights_ver = &sm_weight_arrays[height];
    const int right = topleft[width], bottom = topleft[-height];

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            const int pred = weights_ver[y]  * topleft[1 + x] +
                      (256 - weights_ver[y]) * bottom +
                             weights_hor[x]  * topleft[-(1 + y)] +
                      (256 - weights_hor[x]) * right;
            dst[x] = (pred + 256) >> 9;
        }
        dst += PXSTRIDE(stride);
    }
}

#define smooth_lfn(width, height) \
static void smooth_##width##x##height##_c(pixel *dst, const ptrdiff_t stride, \
                                          const pixel *const topleft, \
                                          const int a) \
{ \
    smooth_c(dst, stride, topleft, width, height); \
}

sz_grid(smooth_lfn)

static __attribute__((noinline)) void
smooth_v_c(pixel *dst, const ptrdiff_t stride, const pixel *const topleft,
           const int width, const int height)
{
    const uint8_t *const weights_ver = &sm_weight_arrays[height];
    const int bottom = topleft[-height];

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            const int pred = weights_ver[y]  * topleft[1 + x] +
                      (256 - weights_ver[y]) * bottom;
            dst[x] = (pred + 128) >> 8;
        }
        dst += PXSTRIDE(stride);
    }
}

#define smooth_v_lfn(width, height) \
static void smooth_v_##width##x##height##_c(pixel *dst, const ptrdiff_t stride, \
                                            const pixel *const topleft, \
                                            const int a) \
{ \
    smooth_v_c(dst, stride, topleft, width, height); \
}

sz_grid(smooth_v_lfn)

static __attribute__((noinline)) void
smooth_h_c(pixel *dst, const ptrdiff_t stride, const pixel *const topleft,
           const int width, const int height)
{
    const uint8_t *const weights_hor = &sm_weight_arrays[width];
    const int right = topleft[width];

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            const int pred = weights_hor[x]  * topleft[-(y + 1)] +
                      (256 - weights_hor[x]) * right;
            dst[x] = (pred + 128) >> 8;
        }
        dst += PXSTRIDE(stride);
    }
}

#define smooth_h_lfn(width, height) \
static void smooth_h_##width##x##height##_c(pixel *dst, const ptrdiff_t stride, \
                                            const pixel *const topleft, \
                                            const int a) \
{ \
    smooth_h_c(dst, stride, topleft, width, height); \
}

sz_grid(smooth_h_lfn)

static const int16_t dr_intra_derivative[90] = {
  // More evenly spread out angles and limited to 10-bit
  // Values that are 0 will never be used
  //                    Approx angle
  0,    0, 0,        //
  1023, 0, 0,        // 3, ...
  547,  0, 0,        // 6, ...
  372,  0, 0, 0, 0,  // 9, ...
  273,  0, 0,        // 14, ...
  215,  0, 0,        // 17, ...
  178,  0, 0,        // 20, ...
  151,  0, 0,        // 23, ... (113 & 203 are base angles)
  132,  0, 0,        // 26, ...
  116,  0, 0,        // 29, ...
  102,  0, 0, 0,     // 32, ...
  90,   0, 0,        // 36, ...
  80,   0, 0,        // 39, ...
  71,   0, 0,        // 42, ...
  64,   0, 0,        // 45, ... (45 & 135 are base angles)
  57,   0, 0,        // 48, ...
  51,   0, 0,        // 51, ...
  45,   0, 0, 0,     // 54, ...
  40,   0, 0,        // 58, ...
  35,   0, 0,        // 61, ...
  31,   0, 0,        // 64, ...
  27,   0, 0,        // 67, ... (67 & 157 are base angles)
  23,   0, 0,        // 70, ...
  19,   0, 0,        // 73, ...
  15,   0, 0, 0, 0,  // 76, ...
  11,   0, 0,        // 81, ...
  7,    0, 0,        // 84, ...
  3,    0, 0,        // 87, ...
};

static int get_filter_strength(const unsigned blk_wh, const unsigned d,
                               const int type)
{
    int strength = 0;

    if (type == 0) {
        if (blk_wh <= 8) {
            if (d >= 56) strength = 1;
        } else if (blk_wh <= 12) {
            if (d >= 40) strength = 1;
        } else if (blk_wh <= 16) {
            if (d >= 40) strength = 1;
        } else if (blk_wh <= 24) {
            if (d >= 8) strength = 1;
            if (d >= 16) strength = 2;
            if (d >= 32) strength = 3;
        } else if (blk_wh <= 32) {
            if (d >= 1) strength = 1;
            if (d >= 4) strength = 2;
            if (d >= 32) strength = 3;
        } else {
            if (d >= 1) strength = 3;
        }
    } else {
        if (blk_wh <= 8) {
            if (d >= 40) strength = 1;
            if (d >= 64) strength = 2;
        } else if (blk_wh <= 16) {
            if (d >= 20) strength = 1;
            if (d >= 48) strength = 2;
        } else if (blk_wh <= 24) {
            if (d >= 4) strength = 3;
        } else {
            if (d >= 1) strength = 3;
        }
    }

    return strength;
}

static void filter_edge(pixel *const out, const int sz,
                        const pixel *const in, const int from, const int to,
                        const unsigned strength)
{
    const uint8_t kernel[3][5] = {
        { 0, 4, 8, 4, 0 },
        { 0, 5, 6, 5, 0 },
        { 2, 4, 4, 4, 2 }
    };

    assert(strength > 0);
    for (int i = 0; i < sz; i++) {
        int s = 0;
        for (int j = 0; j < 5; j++)
            s += in[iclip(i - 2 + j, from, to - 1)] * kernel[strength - 1][j];
        out[i] = (s + 8) >> 4;
    }
}

static int get_upsample(const int blk_wh, const unsigned d, const int type) {
    if (d >= 40) return 0;
    return type ? (blk_wh <= 8) : (blk_wh <= 16);
}

static void upsample_edge(pixel *const out, const int hsz,
                          const pixel *const in, const int from, const int to)
{
    const int8_t kernel[4] = { -1, 9, 9, -1 };
    int i;
    for (i = 0; i < hsz - 1; i++) {
        out[i * 2] = in[iclip(i, from, to - 1)];

        int s = 0;
        for (int j = 0; j < 4; j++)
            s += in[iclip(i + j - 1, from, to - 1)] * kernel[j];
        out[i * 2 + 1] = iclip_pixel((s + 8) >> 4);
    }
    out[i * 2] = in[iclip(i, from, to - 1)];
}

static __attribute__((noinline)) void
z1_c(pixel *dst, const ptrdiff_t stride, const pixel *const topleft_in,
     int angle, const int width, const int height)
{
    const int is_sm = angle >> 9;
    angle &= 511;
    assert(angle < 90);
    const int dx = dr_intra_derivative[angle];
    pixel top_out[(64 + 64) * 2];
    const pixel *top;
    int max_base_x;
    const int upsample_above = get_upsample(width + height, 90 - angle, is_sm);
    if (upsample_above) {
        upsample_edge(top_out, width + height,
                      &topleft_in[1], -1, width + imin(width, height));
        top = top_out;
        max_base_x = 2 * (width + height) - 2;
    } else {
        const int filter_strength =
            get_filter_strength(width + height, 90 - angle, is_sm);

        if (filter_strength) {
            filter_edge(top_out, width + height,
                        &topleft_in[1], -1, width + imin(width, height),
                        filter_strength);
            top = top_out;
            max_base_x = width + height - 1;
        } else {
            top = &topleft_in[1];
            max_base_x = width + imin(width, height) - 1;
        }
    }
    const int frac_bits = 6 - upsample_above;
    const int base_inc = 1 << upsample_above;
    for (int y = 0, xpos = dx; y < height;
         y++, dst += PXSTRIDE(stride), xpos += dx)
    {
        int base = xpos >> frac_bits;
        const int frac = ((xpos << upsample_above) & 0x3F) >> 1;

        for (int x = 0; x < width; x++, base += base_inc) {
            if (base < max_base_x) {
                const int v = top[base] * (32 - frac) + top[base + 1] * frac;
                dst[x] = iclip_pixel((v + 16) >> 5);
            } else {
                pixel_set(&dst[x], top[max_base_x], width - x);
                break;
            }
        }
    }
}

#define z1_lfn(width, height) \
static void z1_##width##x##height##_c(pixel *dst, const ptrdiff_t stride, \
                                      const pixel *const topleft, \
                                      const int angle) \
{ \
    z1_c(dst, stride, topleft, angle, width, height); \
}

sz_grid(z1_lfn)

static __attribute__((noinline)) void
z2_c(pixel *dst, const ptrdiff_t stride, const pixel *const topleft_in,
     int angle, const int width, const int height)
{
    const int is_sm = angle >> 9;
    angle &= 511;
    assert(angle > 90 && angle < 180);
    const int dy = dr_intra_derivative[angle - 90];
    const int dx = dr_intra_derivative[180 - angle];
    const int upsample_left = get_upsample(width + height, 180 - angle, is_sm);
    const int upsample_above = get_upsample(width + height, angle - 90, is_sm);
    pixel edge[64 * 2 + 64 * 2 + 1];
    pixel *const topleft = &edge[height * 2];

    if (upsample_above) {
        upsample_edge(topleft, width + 1, topleft_in, 0, width + 1);
    } else {
        const int filter_strength =
            get_filter_strength(width + height, angle - 90, is_sm);

        if (filter_strength) {
            filter_edge(&topleft[1], width, &topleft_in[1], -1, width,
                        filter_strength);
        } else {
            pixel_copy(&topleft[1], &topleft_in[1], width);
        }
    }
    if (upsample_left) {
        upsample_edge(edge, height + 1, &topleft_in[-height], 0, height + 1);
    } else {
        const int filter_strength =
            get_filter_strength(width + height, 180 - angle, is_sm);

        if (filter_strength) {
            filter_edge(&topleft[-height], height, &topleft_in[-height],
                        0, height + 1, filter_strength);
        } else {
            pixel_copy(&topleft[-height], &topleft_in[-height], height);
        }
    }
    *topleft = *topleft_in;

    const int min_base_x = -(1 << upsample_above);
    const int frac_bits_y = 6 - upsample_left, frac_bits_x = 6 - upsample_above;
    const int base_inc_x = 1 << upsample_above;
    const pixel *const left = &topleft[-(1 << upsample_left)];
    const pixel *const top = &topleft[1 << upsample_above];
    for (int y = 0, xpos = -dx; y < height;
         y++, xpos -= dx, dst += PXSTRIDE(stride))
    {
        int base_x = xpos >> frac_bits_x;
        const int frac_x = ((xpos * (1 << upsample_above)) & 0x3F) >> 1;

        for (int x = 0, ypos = (y << 6) - dy; x < width;
             x++, base_x += base_inc_x, ypos -= dy)
        {
            int v;

            if (base_x >= min_base_x) {
                v = top[base_x] * (32 - frac_x) + top[base_x + 1] * frac_x;
            } else {
                const int base_y = ypos >> frac_bits_y;
                assert(base_y >= -(1 << upsample_left));
                const int frac_y = ((ypos * (1 << upsample_left)) & 0x3F) >> 1;
                v = left[-base_y] * (32 - frac_y) + left[-(base_y + 1)] * frac_y;
            }
            dst[x] = iclip_pixel((v + 16) >> 5);
        }
    }
}

#define z2_lfn(width, height) \
static void z2_##width##x##height##_c(pixel *dst, const ptrdiff_t stride, \
                                      const pixel *const topleft, \
                                      const int angle) \
{ \
    z2_c(dst, stride, topleft, angle, width, height); \
}

sz_grid(z2_lfn)

static __attribute__((noinline)) void
z3_c(pixel *dst, const ptrdiff_t stride, const pixel *const topleft_in,
     int angle, const int width, const int height)
{
    const int is_sm = angle >> 9;
    angle &= 511;
    assert(angle > 180);
    const int dy = dr_intra_derivative[270 - angle];
    pixel left_out[(64 + 64) * 2];
    const pixel *left;
    int max_base_y;
    const int upsample_left = get_upsample(width + height, angle - 180, is_sm);
    if (upsample_left) {
        upsample_edge(left_out, width + height,
                      &topleft_in[-(width + height)],
                      imax(width - height, 0), width + height + 1);
        left = &left_out[2 * (width + height) - 2];
        max_base_y = 2 * (width + height) - 2;
    } else {
        const int filter_strength =
            get_filter_strength(width + height, angle - 180, is_sm);

        if (filter_strength) {
            filter_edge(left_out, width + height,
                        &topleft_in[-(width + height)],
                        imax(width - height, 0), width + height + 1,
                        filter_strength);
            left = &left_out[width + height - 1];
            max_base_y = width + height - 1;
        } else {
            left = &topleft_in[-1];
            max_base_y = height + imin(width, height) - 1;
        }
    }
    const int frac_bits = 6 - upsample_left;
    const int base_inc = 1 << upsample_left;
    for (int x = 0, ypos = dy; x < width; x++, ypos += dy) {
        int base = ypos >> frac_bits;
        const int frac = ((ypos << upsample_left) & 0x3F) >> 1;

        for (int y = 0; y < height; y++, base += base_inc) {
            if (base < max_base_y) {
                const int v = left[-base] * (32 - frac) +
                              left[-(base + 1)] * frac;
                dst[y * PXSTRIDE(stride) + x] = iclip_pixel((v + 16) >> 5);
            } else {
                do {
                    dst[y * PXSTRIDE(stride) + x] = left[-max_base_y];
                } while (++y < height);
                break;
            }
        }
    }
}

#define z3_lfn(width, height) \
static void z3_##width##x##height##_c(pixel *dst, const ptrdiff_t stride, \
                                      const pixel *const topleft, \
                                      const int angle) \
{ \
    z3_c(dst, stride, topleft, angle, width, height); \
}

sz_grid(z3_lfn)

static const int8_t av1_filter_intra_taps[5][8][8] = {
    {
        { -6, 10,  0,  0,  0, 12,  0, 0 },
        { -5,  2, 10,  0,  0,  9,  0, 0 },
        { -3,  1,  1, 10,  0,  7,  0, 0 },
        { -3,  1,  1,  2, 10,  5,  0, 0 },
        { -4,  6,  0,  0,  0,  2, 12, 0 },
        { -3,  2,  6,  0,  0,  2,  9, 0 },
        { -3,  2,  2,  6,  0,  2,  7, 0 },
        { -3,  1,  2,  2,  6,  3,  5, 0 },
    }, {
        { -10, 16,  0,  0,  0, 10,  0, 0 },
        {  -6,  0, 16,  0,  0,  6,  0, 0 },
        {  -4,  0,  0, 16,  0,  4,  0, 0 },
        {  -2,  0,  0,  0, 16,  2,  0, 0 },
        { -10, 16,  0,  0,  0,  0, 10, 0 },
        {  -6,  0, 16,  0,  0,  0,  6, 0 },
        {  -4,  0,  0, 16,  0,  0,  4, 0 },
        {  -2,  0,  0,  0, 16,  0,  2, 0 },
    }, {
        { -8, 8, 0, 0, 0, 16,  0, 0 },
        { -8, 0, 8, 0, 0, 16,  0, 0 },
        { -8, 0, 0, 8, 0, 16,  0, 0 },
        { -8, 0, 0, 0, 8, 16,  0, 0 },
        { -4, 4, 0, 0, 0,  0, 16, 0 },
        { -4, 0, 4, 0, 0,  0, 16, 0 },
        { -4, 0, 0, 4, 0,  0, 16, 0 },
        { -4, 0, 0, 0, 4,  0, 16, 0 },
    }, {
        { -2, 8, 0, 0, 0, 10,  0, 0 },
        { -1, 3, 8, 0, 0,  6,  0, 0 },
        { -1, 2, 3, 8, 0,  4,  0, 0 },
        {  0, 1, 2, 3, 8,  2,  0, 0 },
        { -1, 4, 0, 0, 0,  3, 10, 0 },
        { -1, 3, 4, 0, 0,  4,  6, 0 },
        { -1, 2, 3, 4, 0,  4,  4, 0 },
        { -1, 2, 2, 3, 4,  3,  3, 0 },
    }, {
        { -12, 14,  0,  0,  0, 14,  0, 0 },
        { -10,  0, 14,  0,  0, 12,  0, 0 },
        {  -9,  0,  0, 14,  0, 11,  0, 0 },
        {  -8,  0,  0,  0, 14, 10,  0, 0 },
        { -10, 12,  0,  0,  0,  0, 14, 0 },
        {  -9,  1, 12,  0,  0,  0, 12, 0 },
        {  -8,  0,  0, 12,  0,  1, 11, 0 },
        {  -7,  0,  0,  1, 12,  1,  9, 0 },
    },
};

static __attribute__((noinline)) void
filter_intra_c(pixel *dst, const ptrdiff_t stride,
               const pixel *const topleft_in,
               int filt_idx, const int width, const int height)
{
    filt_idx &= 511;
    assert(filt_idx < 5);

    const int8_t (*const filter)[8] = av1_filter_intra_taps[filt_idx];
    int x, y;
    ptrdiff_t left_stride;
    const pixel *left, *topleft, *top;

    top = &topleft_in[1];
    for (y = 0; y < height; y += 2) {
        topleft = &topleft_in[-y];
        left = &topleft[-1];
        left_stride = -1;
        for (x = 0; x < width; x += 4) {
            const int p0 = *topleft;
            const int p1 = top[0], p2 = top[1], p3 = top[2], p4 = top[3];
            const int p5 = left[0 * left_stride], p6 = left[1 * left_stride];
            pixel *ptr = &dst[x];
            const int8_t (*flt_ptr)[8] = filter;

            for (int yy = 0; yy < 2; yy++) {
                for (int xx = 0; xx < 4; xx++, flt_ptr++) {
                    int acc = flt_ptr[0][0] * p0 + flt_ptr[0][1] * p1 +
                              flt_ptr[0][2] * p2 + flt_ptr[0][3] * p3 +
                              flt_ptr[0][4] * p4 + flt_ptr[0][5] * p5 +
                              flt_ptr[0][6] * p6;
                    ptr[xx] = iclip_pixel((acc + 8) >> 4);
                }
                ptr += PXSTRIDE(stride);
            }

            left = &dst[x + 4 - 1];
            left_stride = PXSTRIDE(stride);
            top += 4;
            topleft = &top[-1];
        }
        top = &dst[PXSTRIDE(stride)];
        dst = &dst[PXSTRIDE(stride) * 2];
    }
}

#define filter_lfn(width, height) \
static void filter_##width##x##height##_c(pixel *const dst, \
                                          const ptrdiff_t stride, \
                                          const pixel *const topleft, \
                                          const int filt_idx) \
{ \
    filter_intra_c(dst, stride, topleft, filt_idx, width, height); \
}

filter_lfn( 4,  4)
filter_lfn( 8,  4)
filter_lfn(16,  4)
filter_lfn( 4,  8)
filter_lfn( 8,  8)
filter_lfn(16,  8)
filter_lfn(32,  8)
filter_lfn( 4, 16)
filter_lfn( 8, 16)
filter_lfn(16, 16)
filter_lfn(32, 16)
filter_lfn( 8, 32)
filter_lfn(16, 32)
filter_lfn(32, 32)

static __attribute__((noinline)) void
cfl_ac_c(int16_t *ac, const pixel *ypx, const ptrdiff_t stride,
         const int w_pad, const int h_pad, const int width, const int height,
         const int ss_hor, const int ss_ver, const int log2sz)
{
    int y, x;
    int16_t *const ac_orig = ac;

    assert(w_pad >= 0 && w_pad * 4 < width);
    assert(h_pad >= 0 && h_pad * 4 < height);

    for (y = 0; y < height - 4 * h_pad; y++) {
        for (x = 0; x < width - 4 * w_pad; x++) {
            int ac_sum = ypx[x << ss_hor];
            if (ss_hor) ac_sum += ypx[x * 2 + 1];
            if (ss_ver) {
                ac_sum += ypx[(x << ss_hor) + PXSTRIDE(stride)];
                if (ss_hor) ac_sum += ypx[x * 2 + 1 + PXSTRIDE(stride)];
            }
            ac[x] = ac_sum << (1 + !ss_ver + !ss_hor);
        }
        for (; x < width; x++)
            ac[x] = ac[x - 1];
        ac += width;
        ypx += PXSTRIDE(stride) << ss_ver;
    }
    for (; y < height; y++) {
        memcpy(ac, &ac[-width], width * sizeof(*ac));
        ac += width;
    }

    int sum = (1 << log2sz) >> 1;
    for (ac = ac_orig, y = 0; y < height; y++) {
        for (x = 0; x < width; x++)
            sum += ac[x];
        ac += width;
    }
    sum >>= log2sz;

    // subtract DC
    for (ac = ac_orig, y = 0; y < height; y++) {
        for (x = 0; x < width; x++)
            ac[x] -= sum;
        ac += width;
    }
}

#define cfl_ac_fn(lw, lh, cw, ch, ss_hor, ss_ver, log2sz) \
static void cfl_ac_##lw##x##lh##_to_##cw##x##ch##_c(int16_t *const ac, \
                                                    const pixel *const ypx, \
                                                    const ptrdiff_t stride, \
                                                    const int w_pad, \
                                                    const int h_pad) \
{ \
    cfl_ac_c(ac, ypx, stride, w_pad, h_pad, cw, ch, ss_hor, ss_ver, log2sz); \
}

cfl_ac_fn( 8,  8,  4,  4, 1, 1, 4)
cfl_ac_fn( 8, 16,  4,  8, 1, 1, 5)
cfl_ac_fn( 8, 32,  4, 16, 1, 1, 6)
cfl_ac_fn(16,  8,  8,  4, 1, 1, 5)
cfl_ac_fn(16, 16,  8,  8, 1, 1, 6)
cfl_ac_fn(16, 32,  8, 16, 1, 1, 7)
cfl_ac_fn(32,  8, 16,  4, 1, 1, 6)
cfl_ac_fn(32, 16, 16,  8, 1, 1, 7)
cfl_ac_fn(32, 32, 16, 16, 1, 1, 8)

cfl_ac_fn( 8,  4,  4,  4, 1, 0, 4)
cfl_ac_fn( 8,  8,  4,  8, 1, 0, 5)
cfl_ac_fn(16,  4,  8,  4, 1, 0, 5)
cfl_ac_fn(16,  8,  8,  8, 1, 0, 6)
cfl_ac_fn(16, 16,  8, 16, 1, 0, 7)
cfl_ac_fn(32,  8, 16,  8, 1, 0, 7)
cfl_ac_fn(32, 16, 16, 16, 1, 0, 8)
cfl_ac_fn(32, 32, 16, 32, 1, 0, 9)

cfl_ac_fn( 4,  4,  4,  4, 0, 0, 4)
cfl_ac_fn( 4,  8,  4,  8, 0, 0, 5)
cfl_ac_fn( 4, 16,  4, 16, 0, 0, 6)
cfl_ac_fn( 8,  4,  8,  4, 0, 0, 5)
cfl_ac_fn( 8,  8,  8,  8, 0, 0, 6)
cfl_ac_fn( 8, 16,  8, 16, 0, 0, 7)
cfl_ac_fn( 8, 32,  8, 32, 0, 0, 8)
cfl_ac_fn(16,  4, 16,  4, 0, 0, 6)
cfl_ac_fn(16,  8, 16,  8, 0, 0, 7)
cfl_ac_fn(16, 16, 16, 16, 0, 0, 8)
cfl_ac_fn(16, 32, 16, 32, 0, 0, 9)
cfl_ac_fn(32,  8, 32,  8, 0, 0, 8)
cfl_ac_fn(32, 16, 32, 16, 0, 0, 9)
cfl_ac_fn(32, 32, 32, 32, 0, 0, 10)

static __attribute__((noinline)) void
cfl_pred_c(pixel *dstU, pixel *dstV, const ptrdiff_t stride,
           const int16_t *ac, const pixel *const dc_pred,
           const int8_t *const alphas, const int width, const int height)
{
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            const int diff1 = alphas[0] * ac[x];
            dstU[x] = iclip_pixel(dc_pred[ 0] + apply_sign((abs(diff1) + 32) >> 6,
                                                           diff1));
            const int diff2 = alphas[1] * ac[x];
            dstV[x] = iclip_pixel(dc_pred[32] + apply_sign((abs(diff2) + 32) >> 6,
                                                           diff2));
        }
        ac += width;
        dstU += PXSTRIDE(stride);
        dstV += PXSTRIDE(stride);
    }
}

#define cfl_pred_fn(width) \
static void cfl_pred_##width##xN_c(pixel *const dstU, \
                                   pixel *const dstV, \
                                   const ptrdiff_t stride, \
                                   const int16_t *const ac, \
                                   const pixel *const dc_pred, \
                                   const int8_t *const alphas, \
                                   const int height) \
{ \
    cfl_pred_c(dstU, dstV, stride, ac, dc_pred, alphas, width, height); \
}

cfl_pred_fn( 4)
cfl_pred_fn( 8)
cfl_pred_fn(16)
cfl_pred_fn(32)

static void pal_pred_c(pixel *dst, const ptrdiff_t stride,
                       const uint16_t *const pal, const uint8_t *idx,
                       const int w, const int h)
{
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++)
            dst[x] = pal[idx[x]];
        idx += w;
        dst += PXSTRIDE(stride);
    }
}

void bitfn(dav1d_intra_pred_dsp_init)(Dav1dIntraPredDSPContext *const c) {
#define assign_lfn(w, h, p1, p2, pfx) \
    c->intra_pred[pfx##TX_##w##X##h][p1##_PRED] = p2##_##w##x##h##_c
#define assign_fns(p1, p2) \
    assign_lfn( 4,  4, p1, p2,); \
    assign_lfn( 4,  8, p1, p2, R); \
    assign_lfn( 4, 16, p1, p2, R); \
    assign_lfn( 8,  4, p1, p2, R); \
    assign_lfn( 8,  8, p1, p2,); \
    assign_lfn( 8, 16, p1, p2, R); \
    assign_lfn( 8, 32, p1, p2, R); \
    assign_lfn(16,  4, p1, p2, R); \
    assign_lfn(16,  8, p1, p2, R); \
    assign_lfn(16, 16, p1, p2,); \
    assign_lfn(16, 32, p1, p2, R); \
    assign_lfn(16, 64, p1, p2, R); \
    assign_lfn(32,  8, p1, p2, R); \
    assign_lfn(32, 16, p1, p2, R); \
    assign_lfn(32, 32, p1, p2,); \
    assign_lfn(32, 64, p1, p2, R); \
    assign_lfn(64, 16, p1, p2, R); \
    assign_lfn(64, 32, p1, p2, R); \
    assign_lfn(64, 64, p1, p2,); \

    assign_fns(DC, dc);
    assign_fns(DC_128, dc128);
    assign_fns(TOP_DC,  dctop);
    assign_fns(LEFT_DC, dcleft);
    assign_fns(HOR, h);
    assign_fns(VERT, v);
    assign_fns(PAETH, paeth);
    assign_fns(SMOOTH, smooth);
    assign_fns(SMOOTH_V, smooth_v);
    assign_fns(SMOOTH_H, smooth_h);
    assign_fns(Z1, z1);
    assign_fns(Z2, z2);
    assign_fns(Z3, z3);

    assign_lfn( 4,  4, FILTER, filter,);
    assign_lfn( 8,  4, FILTER, filter, R);
    assign_lfn(16,  4, FILTER, filter, R);
    assign_lfn( 4,  8, FILTER, filter, R);
    assign_lfn( 8,  8, FILTER, filter,);
    assign_lfn(16,  8, FILTER, filter, R);
    assign_lfn(32,  8, FILTER, filter, R);
    assign_lfn( 4, 16, FILTER, filter, R);
    assign_lfn( 8, 16, FILTER, filter, R);
    assign_lfn(16, 16, FILTER, filter,);
    assign_lfn(32, 16, FILTER, filter, R);
    assign_lfn( 8, 32, FILTER, filter, R);
    assign_lfn(16, 32, FILTER, filter, R);
    assign_lfn(32, 32, FILTER, filter,);

    // cfl functions are split per chroma subsampling type
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I420 - 1][ TX_4X4  ] = cfl_ac_8x8_to_4x4_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I420 - 1][RTX_4X8  ] = cfl_ac_8x16_to_4x8_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I420 - 1][RTX_4X16 ] = cfl_ac_8x32_to_4x16_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I420 - 1][RTX_8X4  ] = cfl_ac_16x8_to_8x4_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I420 - 1][ TX_8X8  ] = cfl_ac_16x16_to_8x8_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I420 - 1][RTX_8X16 ] = cfl_ac_16x32_to_8x16_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I420 - 1][RTX_16X4 ] = cfl_ac_32x8_to_16x4_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I420 - 1][RTX_16X8 ] = cfl_ac_32x16_to_16x8_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I420 - 1][ TX_16X16] = cfl_ac_32x32_to_16x16_c;

    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I422 - 1][ TX_4X4  ] = cfl_ac_8x4_to_4x4_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I422 - 1][RTX_4X8  ] = cfl_ac_8x8_to_4x8_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I422 - 1][RTX_8X4 ] = cfl_ac_16x4_to_8x4_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I422 - 1][ TX_8X8 ] = cfl_ac_16x8_to_8x8_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I422 - 1][RTX_8X16] = cfl_ac_16x16_to_8x16_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I422 - 1][RTX_16X8 ] = cfl_ac_32x8_to_16x8_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I422 - 1][ TX_16X16] = cfl_ac_32x16_to_16x16_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I422 - 1][RTX_16X32] = cfl_ac_32x32_to_16x32_c;

    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I444 - 1][ TX_4X4  ] = cfl_ac_4x4_to_4x4_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I444 - 1][RTX_4X8  ] = cfl_ac_4x8_to_4x8_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I444 - 1][RTX_4X16 ] = cfl_ac_4x16_to_4x16_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I444 - 1][RTX_8X4  ] = cfl_ac_8x4_to_8x4_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I444 - 1][ TX_8X8  ] = cfl_ac_8x8_to_8x8_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I444 - 1][RTX_8X16 ] = cfl_ac_8x16_to_8x16_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I444 - 1][RTX_8X32 ] = cfl_ac_8x32_to_8x32_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I444 - 1][RTX_16X4 ] = cfl_ac_16x4_to_16x4_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I444 - 1][RTX_16X8 ] = cfl_ac_16x8_to_16x8_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I444 - 1][ TX_16X16] = cfl_ac_16x16_to_16x16_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I444 - 1][RTX_16X32] = cfl_ac_16x32_to_16x32_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I444 - 1][RTX_32X8 ] = cfl_ac_32x8_to_32x8_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I444 - 1][RTX_32X16] = cfl_ac_32x16_to_32x16_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I444 - 1][ TX_32X32] = cfl_ac_32x32_to_32x32_c;

    c->cfl_pred[0] = cfl_pred_4xN_c;
    c->cfl_pred[1] = cfl_pred_8xN_c;
    c->cfl_pred[2] = cfl_pred_16xN_c;
    c->cfl_pred[3] = cfl_pred_32xN_c;

    c->pal_pred = pal_pred_c;
}
