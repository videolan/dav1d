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

#include <stdlib.h>

#include "common/intops.h"

#include "src/looprestoration.h"
#include "src/tables.h"


// TODO Reuse p when no padding is needed (add and remove lpf pixels in p)
// TODO Chroma only requires 2 rows of padding.
static void padding(pixel *dst, const ptrdiff_t dst_stride,
                    const pixel *p, const ptrdiff_t p_stride,
                    const pixel *lpf, const ptrdiff_t lpf_stride,
                    int unit_w, const int stripe_h, const enum LrEdgeFlags edges)
{
    const int have_left = !!(edges & LR_HAVE_LEFT);
    const int have_right = !!(edges & LR_HAVE_RIGHT);

    // Copy more pixels if we don't have to pad them
    unit_w += 3 * have_left + 3 * have_right;
    pixel *dst_l = dst + 3 * !have_left;
    p -= 3 * have_left;
    lpf -= 3 * have_left;

    if (edges & LR_HAVE_TOP) {
        // Copy previous loop filtered rows
        const pixel *const above_1 = lpf;
        const pixel *const above_2 = above_1 + PXSTRIDE(lpf_stride);
        pixel_copy(dst_l, above_1, unit_w);
        pixel_copy(dst_l + PXSTRIDE(dst_stride), above_1, unit_w);
        pixel_copy(dst_l + 2 * PXSTRIDE(dst_stride), above_2, unit_w);
    } else {
        // Pad with first row
        pixel_copy(dst_l, p, unit_w);
        pixel_copy(dst_l + PXSTRIDE(dst_stride), p, unit_w);
        pixel_copy(dst_l + 2 * PXSTRIDE(dst_stride), p, unit_w);
    }

    pixel *dst_tl = dst_l + 3 * PXSTRIDE(dst_stride);
    if (edges & LR_HAVE_BOTTOM) {
        // Copy next loop filtered rows
        const pixel *const below_1 = lpf + 6 * PXSTRIDE(lpf_stride);
        const pixel *const below_2 = below_1 + PXSTRIDE(lpf_stride);
        pixel_copy(dst_tl + stripe_h * PXSTRIDE(dst_stride), below_1, unit_w);
        pixel_copy(dst_tl + (stripe_h + 1) * PXSTRIDE(dst_stride), below_2, unit_w);
        pixel_copy(dst_tl + (stripe_h + 2) * PXSTRIDE(dst_stride), below_2, unit_w);
    } else {
        // Pad with last row
        const pixel *const src = p + (stripe_h - 1) * PXSTRIDE(p_stride);
        pixel_copy(dst_tl + stripe_h * PXSTRIDE(dst_stride), src, unit_w);
        pixel_copy(dst_tl + (stripe_h + 1) * PXSTRIDE(dst_stride), src, unit_w);
        pixel_copy(dst_tl + (stripe_h + 2) * PXSTRIDE(dst_stride), src, unit_w);
    }

    // Inner UNIT_WxSTRIPE_H
    for (int j = 0; j < stripe_h; j++) {
        pixel_copy(dst_tl, p, unit_w);
        dst_tl += PXSTRIDE(dst_stride);
        p += PXSTRIDE(p_stride);
    }

    if (!have_right) {
        pixel *pad = dst_l + unit_w;
        pixel *row_last = &dst_l[unit_w - 1];
        // Pad 3x(STRIPE_H+6) with last column
        for (int j = 0; j < stripe_h + 6; j++) {
            pixel_set(pad, *row_last, 3);
            pad += PXSTRIDE(dst_stride);
            row_last += PXSTRIDE(dst_stride);
        }
    }

    if (!have_left) {
        // Pad 3x(STRIPE_H+6) with first column
        for (int j = 0; j < stripe_h + 6; j++) {
            pixel_set(dst, *dst_l, 3);
            dst += PXSTRIDE(dst_stride);
            dst_l += PXSTRIDE(dst_stride);
        }
    }
}

// FIXME Could split into luma and chroma specific functions,
// (since first and last tops are always 0 for chroma)
// FIXME Could implement a version that requires less temporary memory
// (should be possible to implement with only 6 rows of temp storage)
static void wiener_c(pixel *p, const ptrdiff_t p_stride,
                     const pixel *lpf, const ptrdiff_t lpf_stride,
                     const int w, const int h,
                     const int16_t filterh[7], const int16_t filterv[7],
                     const enum LrEdgeFlags edges)
{
    // padding is 3 pixels above and 3 pixels below
    const ptrdiff_t tmp_stride = sizeof(pixel) * (w + 6);
    pixel tmp[(h + 6) * PXSTRIDE(tmp_stride)];
    pixel *tmp_ptr = tmp;

    padding(tmp, tmp_stride, p, p_stride, lpf, lpf_stride, w, h, edges);

    // Values stored between horizontal and vertical filtering don't
    // fit in a uint8_t.
    uint16_t hor[(h + 6 /*padding*/) * w];
    uint16_t *hor_ptr = hor;

    const int round_bits_h = 3 + (BITDEPTH == 12) * 2;
    const int rounding_off_h = 1 << (round_bits_h - 1);
    const int clip_limit = 1 << ((BITDEPTH) + 1 + 7 - round_bits_h);
    for (int j = 0; j < h + 6; j++) {
        for (int i = 0; i < w; i++) {
            int sum = (tmp_ptr[i + 3] << 7) + (1 << (BITDEPTH + 6));

            for (int k = 0; k < 7; k++) {
                sum += tmp_ptr[i + k] * filterh[k];
            }

            hor_ptr[i] =
                iclip((sum + rounding_off_h) >> round_bits_h, 0, clip_limit);
        }
        tmp_ptr += PXSTRIDE(tmp_stride);
        hor_ptr += w;
    }

    const int round_bits_v = 11 - (BITDEPTH == 12) * 2;
    const int rounding_off_v = 1 << (round_bits_v - 1);
    const int round_offset = 1 << (BITDEPTH + (round_bits_v - 1));
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
            int sum = (hor[w * (j + 3) + i] << 7) - round_offset;

            for (int k = 0; k < 7; k++) {
                sum += hor[(j + k) * w + i] * filterv[k];
            }

            p[j * PXSTRIDE(p_stride) + i] =
                iclip_pixel((sum + rounding_off_v) >> round_bits_v);
        }
    }
}

// Sum over a 3x3 area
// The dst and src pointers are positioned 3 pixels above and 3 pixels to the
// left of the top left corner. However, the self guided filter only needs 1
// pixel above and one pixel to the left. As for the pixels below and to the
// right they must be computed in the sums, but don't need to be stored.
//
// Example for a 4x4 block:
//      x x x x x x x x x x
//      x c c c c c c c c x
//      x i s s s s s s i x
//      x i s s s s s s i x
//      x i s s s s s s i x
//      x i s s s s s s i x
//      x i s s s s s s i x
//      x i s s s s s s i x
//      x c c c c c c c c x
//      x x x x x x x x x x
//
// s: Pixel summed and stored
// i: Pixel summed and stored (between loops)
// c: Pixel summed not stored
// x: Pixel not summed not stored
static void boxsum3(coef *dst, const ptrdiff_t dst_stride,
                    const pixel *src, ptrdiff_t src_stride,
                    const int w, const int h)
{
    src_stride = PXSTRIDE(src_stride);
    // We skip the first row, as it is never used
    src += src_stride;
    dst += dst_stride;

    // We skip the first and last columns, as they are never used
    for (int x = 1; x < w - 1; x++) {
        coef *ds = dst + x;
        const pixel *s = src + x;
        int a = s[0], b = s[src_stride];

        // We skip the first 2 rows, as they are skipped in the next loop and
        // we don't need the last 2 row as it is skipped in the next loop
        for (int y = 2; y < h - 2; y++) {
            s += src_stride;
            const int c = s[src_stride];
            ds += dst_stride;
            *ds = a + b + c;
            a = b;
            b = c;
        }
     }

    // We skip the first 2 rows as they are never read
    dst += dst_stride;
    // We skip the last 2 rows as it is never read
    for (int y = 2; y < h - 2; y++) {
        int a = dst[1], b = dst[2];

        // We don't store the first column as it is never read and
        // we don't store the last 2 columns as they are never read
        for (int x = 2; x < w - 2; x++) {
            const int c = dst[x + 1];
            dst[x] = a + b + c;
            a = b;
            b = c;
        }
        dst += dst_stride;
    }
}

// Sum over a 5x5 area
// The dst and src pointers are positioned 3 pixels above and 3 pixels to the
// left of the top left corner. However, the self guided filter only needs 1
// pixel above and one pixel to the left. As for the pixels below and to the
// right they must be computed in the sums, but don't need to be stored.
//
// Example for a 4x4 block:
//      c c c c c c c c c c
//      c c c c c c c c c c
//      i i s s s s s s i i
//      i i s s s s s s i i
//      i i s s s s s s i i
//      i i s s s s s s i i
//      i i s s s s s s i i
//      i i s s s s s s i i
//      c c c c c c c c c c
//      c c c c c c c c c c
//
// s: Pixel summed and stored
// i: Pixel summed and stored (between loops)
// c: Pixel summed not stored
// x: Pixel not summed not stored
static void boxsum5(coef *dst, const ptrdiff_t dst_stride,
                    const pixel *const src, ptrdiff_t src_stride,
                    const int w, const int h)
{
    src_stride = PXSTRIDE(src_stride);

    // We skip the first row, as it is never used
    dst += dst_stride;

    for (int x = 0; x < w; x++) {
        coef *ds = dst + x;
        const pixel *s = src + 3 * src_stride + x;
        int a = s[-3 * src_stride];
        int b = s[-2 * src_stride];
        int c = s[-1 * src_stride];
        int d = s[0];

        // We skip the first 2 rows, as they are skipped in the next loop and
        // we don't need the last 2 row as it is skipped in the next loop
        for (int y = 2; y < h - 2; y++) {
            s += src_stride;
            const int e = *s;
            ds += dst_stride;
            *ds = a + b + c + d + e;
            a = b;
            b = c;
            c = d;
            d = e;
        }
    }

    // We skip the first 2 rows as they are never read
    dst += dst_stride;
    for (int y = 2; y < h - 2; y++) {
        int a = dst[0];
        int b = dst[1];
        int c = dst[2];
        int d = dst[3];

        for (int x = 2; x < w - 2; x++) {
            const int e = dst[x + 2];
            dst[x] = a + b + c + d + e;
            a = b;
            b = c;
            c = d;
            d = e;
        }
        dst += dst_stride;
    }
}

// See boxsum3 function comments for details on row and column skipping
static void boxsum3sqr(int32_t *dst, const ptrdiff_t dst_stride,
                       const pixel *src, ptrdiff_t src_stride,
                       const int w, const int h)
{
    src_stride = PXSTRIDE(src_stride);

    // We skip the first row, as it is never used
    src += src_stride;
    dst += dst_stride;

    // We skip the first and last columns, as they are never used
    for (int x = 1; x < w - 1; x++) {
        int *ds = dst + x;
        const pixel *s = src + x;
        int a = s[0] * s[0];
        int b = s[src_stride] * s[src_stride];

        // We skip the first row, as it is skipped in the next loop and
        // we don't need the last row as it is skipped in the next loop
        for (int y = 2; y < h - 2; y++) {
            s += src_stride;
            const int c = s[src_stride] * s[src_stride];
            ds += dst_stride;
            *ds = a + b + c;
            a = b;
            b = c;
        }
     }

    // We skip the first row as it is never read
    dst += dst_stride;
    // We skip the last row as it is never read
    for (int y = 2; y < h - 2; y++) {
        int a = dst[1], b = dst[2];

        // We don't store the first column as it is never read and
        // we don't store the last 2 columns as they are never read
        for (int x = 2; x < w - 2; x++) {
            const int c = dst[x + 1];
            dst[x] = a + b + c;
            a = b;
            b = c;
        }
        dst += dst_stride;
    }
}

// See boxsum5 function comments for details on row and column skipping
static void boxsum5sqr(int32_t *dst, const ptrdiff_t dst_stride,
                       const pixel *const src, ptrdiff_t src_stride,
                       const int w, const int h)
{
    src_stride = PXSTRIDE(src_stride);

    // We skip the first row, as it is never used
    dst += dst_stride;

    for (int x = 0; x < w; x++) {
        int *ds = dst + x;
        const pixel *s = src + 3 * src_stride + x;
        int a = s[-3 * src_stride] * s[-3 * src_stride];
        int b = s[-2 * src_stride] * s[-2 * src_stride];
        int c = s[-1 * src_stride] * s[-1 * src_stride];
        int d = s[0] * s[0];

        // We skip the first 2 rows, as they are skipped in the next loop and
        // we don't need the last 2 row as it is skipped in the next loop
        for (int y = 2; y < h - 2; y++) {
            s += src_stride;
            const int e = s[0] * s[0];
            ds += dst_stride;
            *ds = a + b + c + d + e;
            a = b;
            b = c;
            c = d;
            d = e;
        }
    }

    // We skip the first 2 rows as they are never read
    dst += dst_stride;
    for (int y = 2; y < h - 2; y++) {
        int a = dst[0];
        int b = dst[1];
        int c = dst[2];
        int d = dst[3];

        for (int x = 2; x < w - 2; x++) {
            const int e = dst[x + 2];
            dst[x] = a + b + c + d + e;
            a = b;
            b = c;
            c = d;
            d = e;
        }
        dst += dst_stride;
    }
}

static void selfguided_filter(int32_t *dst, const ptrdiff_t dst_stride,
                              const pixel *src, const ptrdiff_t src_stride,
                              const int w, const int h, const int n, const int s)
{
    const int tmp_stride = w + 6;
    // FIXME Replace array with scratch memory
    int32_t A_[(h + 6) * tmp_stride];
    int32_t *A = A_ + 3 * tmp_stride + 3;
    // By inverting A and B after the boxsums, B can be of size coef instead
    // of int32_t
    coef B_[(h + 6) * tmp_stride];
    coef *B = B_ + 3 * tmp_stride + 3;

    const int step = (n == 25) + 1;
    if (n == 25) {
        boxsum5(B_, tmp_stride, src, src_stride, w + 6, h + 6);
        boxsum5sqr(A_, tmp_stride, src, src_stride, w + 6, h + 6);
    } else {
        boxsum3(B_, tmp_stride, src, src_stride, w + 6, h + 6);
        boxsum3sqr(A_, tmp_stride, src, src_stride, w + 6, h + 6);
    }

    int32_t *AA = A - tmp_stride;
    coef *BB = B - tmp_stride;
    for (int j = -1; j < h + 1; j+= step) {
        for (int i = -1; i < w + 1; i++) {
            const int a =
                (AA[i] + (1 << (2 * (BITDEPTH - 8)) >> 1)) >> (2 * (BITDEPTH - 8));
            const int b =
                (BB[i] + (1 << (BITDEPTH - 8) >> 1)) >> (BITDEPTH - 8);

            const uint32_t p = (a * n >= b * b) * (a * n - b * b);
            const uint32_t z = (p * s + (1 << 19)) >> 20;

            const int x = sgr_x_by_xplus1[imin(z, 255)];
            // This is where we invert A and B, so that B is of size coef.
            AA[i] = (((1 << 8) - x) * BB[i] * sgr_one_by_x[n - 1] + (1 << 11)) >> 12;
            BB[i] = x;
        }
        AA += step * tmp_stride;
        BB += step * tmp_stride;
    }

    src += 3 * PXSTRIDE(src_stride) + 3;
    if (n == 25) {
        int j = 0;
#define SIX_NEIGHBORS(P, i, stride)\
    ((P[i - stride]     + P[i + stride]) * 6 +   \
     (P[i - 1 - stride] + P[i - 1 + stride] +    \
      P[i + 1 - stride] + P[i + 1 + stride]) * 5)
        for (; j < h - 1; j+=2) {
            for (int i = 0; i < w; i++) {
                const int32_t a = SIX_NEIGHBORS(B, i, tmp_stride);
                const int32_t b = SIX_NEIGHBORS(A, i, tmp_stride);
                dst[i] = (a * src[i] + b + (1 << 8)) >> 9;
            }
            dst += dst_stride;
            src += PXSTRIDE(src_stride);
            B += tmp_stride;
            A += tmp_stride;
            for (int i = 0; i < w; i++) {
                const int32_t a = B[i] * 6 + (B[i - 1] + B[i + 1]) * 5;
                const int32_t b = A[i] * 6 + (A[i - 1] + A[i + 1]) * 5;
                dst[i] = (a * src[i] + b + (1 << 7)) >> 8;
            }
            dst += dst_stride;
            src += PXSTRIDE(src_stride);
            B += tmp_stride;
            A += tmp_stride;
        }
        if (j + 1 == h) { // Last row, when number of rows is odd
            for (int i = 0; i < w; i++) {
                const int32_t a = SIX_NEIGHBORS(B, i, tmp_stride);
                const int32_t b = SIX_NEIGHBORS(A, i, tmp_stride);
                dst[i] = (a * src[i] + b + (1 << 8)) >> 9;
            }
        }
#undef SIX_NEIGHBORS
    } else {
#define EIGHT_NEIGHBORS(P, i, stride)\
    ((P[i] + P[i - 1] + P[i + 1] + P[i - tmp_stride] + P[i + tmp_stride]) * 4 + \
     (P[i - 1 - tmp_stride] + P[i - 1 + tmp_stride] +                           \
      P[i + 1 - tmp_stride] + P[i + 1 + tmp_stride]) * 3)
        for (int j = 0; j < h; j++) {
            for (int i = 0; i < w; i++) {
                const int32_t a = EIGHT_NEIGHBORS(B, i, stride);
                const int32_t b = EIGHT_NEIGHBORS(A, i, stride);
                dst[i] = (a * src[i] + b + (1 << 8)) >> 9;
            }
            dst += dst_stride;
            src += PXSTRIDE(src_stride);
            B += tmp_stride;
            A += tmp_stride;
        }
    }
#undef NINE_NEIGHBORS
}

static void selfguided_c(pixel *p, const ptrdiff_t p_stride,
                         const pixel *lpf, const ptrdiff_t lpf_stride,
                         const int w, const int h, const int sgr_idx,
                         const int16_t sgr_w[2], const enum LrEdgeFlags edges)
{
    // padding is 3 pixels above and 3 pixels below
    const int tmp_stride = sizeof(pixel) * (w + 6);
    pixel tmp[(h + 6) * PXSTRIDE(tmp_stride)];

    padding(tmp, tmp_stride, p, p_stride, lpf, lpf_stride, w, h, edges);

    // both r1 and r0 can't be zero
    if (!sgr_params[sgr_idx][0]) {
        int32_t dst[h * w];
        const int s1 = sgr_params[sgr_idx][3];
        selfguided_filter(dst, w, tmp, tmp_stride, w, h, 9, s1);
        const int w1 = (1 << 7) - sgr_w[1];
        for (int j = 0; j < h; j++) {
            for (int i = 0; i < w; i++) {
                const int32_t u = (p[i] << 4);
                const int32_t v = (u << 7) + w1 * (dst[j * w + i] - u);
                p[i] = iclip_pixel((v + (1 << 10)) >> 11);
            }
            p += PXSTRIDE(p_stride);
        }
    } else if (!sgr_params[sgr_idx][1]) {
        int32_t dst[h * w];
        const int s0 = sgr_params[sgr_idx][2];
        selfguided_filter(dst, w, tmp, tmp_stride, w, h, 25, s0);
        const int w0 = sgr_w[0];
        for (int j = 0; j < h; j++) {
            for (int i = 0; i < w; i++) {
                const int32_t u = (p[i] << 4);
                const int32_t v = (u << 7) + w0 * (dst[j * w + i] - u);
                p[i] = iclip_pixel((v + (1 << 10)) >> 11);
            }
            p += PXSTRIDE(p_stride);
        }
    } else {
        int32_t dst0[h * w];
        int32_t dst1[h * w];
        const int s0 = sgr_params[sgr_idx][2];
        const int s1 = sgr_params[sgr_idx][3];
        const int w0 = sgr_w[0];
        const int w1 = (1 << 7) - w0 - sgr_w[1];
        selfguided_filter(dst0, w, tmp, tmp_stride, w, h, 25, s0);
        selfguided_filter(dst1, w, tmp, tmp_stride, w, h, 9, s1);
        for (int j = 0; j < h; j++) {
            for (int i = 0; i < w; i++) {
                const int32_t u = (p[i] << 4);
                const int32_t v = (u << 7) + w0 * (dst0[j * w + i] - u) +
                                  w1 * (dst1[j * w + i] - u);
                p[i] = iclip_pixel((v + (1 << 10)) >> 11);
            }
            p += PXSTRIDE(p_stride);
        }
    }
}

void bitfn(dav1d_loop_restoration_dsp_init)(Dav1dLoopRestorationDSPContext *const c) {
    c->wiener = wiener_c;
    c->selfguided = selfguided_c;
}
