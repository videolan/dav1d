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

#include "tests/checkasm/checkasm.h"

#include <string.h>

#include "src/levels.h"
#include "src/loopfilter.h"

static void init_lpf_border(pixel *const dst, const ptrdiff_t stride,
                            int E, int I, int H)
{
    const int F = 1 << (BITDEPTH - 8);
    E <<= BITDEPTH - 8;
    I <<= BITDEPTH - 8;
    H <<= BITDEPTH - 8;

    const int filter_type = rand() % 4;
    const int edge_diff = rand() % ((E + 2) * 4) - 2 * (E + 2);
    switch (filter_type) {
    case 0: // random, unfiltered
        for (int i = -8; i < 8; i++)
            dst[i * stride] = rand() & ((1 << BITDEPTH) - 1);
        break;
    case 1: // long flat
        dst[-8 * stride] = rand() & ((1 << BITDEPTH) - 1);
        dst[+7 * stride] = rand() & ((1 << BITDEPTH) - 1);
        dst[+0 * stride] = rand() & ((1 << BITDEPTH) - 1);
        dst[-1 * stride] = iclip_pixel(dst[+0 * stride] + edge_diff);
        for (int i = 1; i < 7; i++) {
            dst[-(1 + i) * stride] = iclip_pixel(dst[-1 * stride] +
                                                 rand() % (2 * (F + 1)) - (F + 1));
            dst[+(0 + i) * stride] = iclip_pixel(dst[+0 * stride] +
                                                 rand() % (2 * (F + 1)) - (F + 1));
        }
        break;
    case 2: // short flat
        for (int i = 4; i < 8; i++) {
            dst[-(1 + i) * stride] = rand() & ((1 << BITDEPTH) - 1);
            dst[+(0 + i) * stride] = rand() & ((1 << BITDEPTH) - 1);
        }
        dst[+0 * stride] = rand() & ((1 << BITDEPTH) - 1);
        dst[-1 * stride] = iclip_pixel(dst[+0 * stride] + edge_diff);
        for (int i = 1; i < 4; i++) {
            dst[-(1 + i) * stride] = iclip_pixel(dst[-1 * stride] +
                                                 rand() % (2 * (F + 1)) - (F + 1));
            dst[+(0 + i) * stride] = iclip_pixel(dst[+0 * stride] +
                                                 rand() % (2 * (F + 1)) - (F + 1));
        }
        break;
    case 3: // normal or hev
        for (int i = 4; i < 8; i++) {
            dst[-(1 + i) * stride] = rand() & ((1 << BITDEPTH) - 1);
            dst[+(0 + i) * stride] = rand() & ((1 << BITDEPTH) - 1);
        }
        dst[+0 * stride] = rand() & ((1 << BITDEPTH) - 1);
        dst[-1 * stride] = iclip_pixel(dst[+0 * stride] + edge_diff);
        for (int i = 1; i < 4; i++) {
            dst[-(1 + i) * stride] = iclip_pixel(dst[-(0 + i) * stride] +
                                                 rand() % (2 * (I + 1)) - (I + 1));
            dst[+(0 + i) * stride] = iclip_pixel(dst[+(i - 1) * stride] +
                                                 rand() % (2 * (I + 1)) - (I + 1));
        }
        break;
    }
}

static void check_lpf_sb(loopfilter_sb_fn fn, const char *const name,
                         const int n_strengths, const int n_blks,
                         const int sb_idx)
{
    ALIGN_STK_32(pixel, c_dst, 128 * 16,);
    ALIGN_STK_32(pixel, a_dst, 128 * 16,);

    declare_func(void, pixel *dst, ptrdiff_t dst_stride, const uint32_t *mask,
                 const uint8_t (*l)[4], ptrdiff_t b4_stride,
                 const Av1FilterLUT *lut, int w);

    Av1FilterLUT lut;
    const int sharp = rand() & 7;
    for (int level = 0; level < 64; level++) {
        int limit = level;

        if (sharp > 0) {
            limit >>= (sharp + 3) >> 2;
            limit = imin(limit, 9 - sharp);
        }
        limit = imax(limit, 1);

        lut.i[level] = limit;
        lut.e[level] = 2 * (level + 2) + limit;
    }
    lut.sharp[0] = (sharp + 3) >> 2;
    lut.sharp[1] = sharp ? 9 - sharp : 0xff;

    for (int i = 0; i < n_strengths; i++) {
        if (check_func(fn, "%s_w%d_%dbpc", name,
                       n_strengths == 3 ? 4 << i : 4 * 2 * i, BITDEPTH))
        {
            uint32_t vmask[4] = { 0 };
            uint8_t l[32 * 2][4];

            for (int j = 0; j < n_blks; j++) {
                const int idx = rand() % (i + 2);
                if (idx) vmask[idx - 1] |= 1 << j;
                l[j][sb_idx] = rand() & 63;
                l[32 + j][sb_idx] = rand() & 63;
            }

            for (int i = 0; i < 128; i++) {
                const int x = i >> 2;
                const int L = l[x + 32][sb_idx] ? l[x + 32][sb_idx] : l[x][sb_idx];
                init_lpf_border(c_dst + 128 * 8 + i, 128,
                                lut.e[L], lut.i[L], L >> 4);
            }
            memcpy(a_dst, c_dst, 128 * sizeof(pixel) * 16);

            call_ref(c_dst + 128 * 8, 128 * sizeof(pixel),
                     vmask, (const uint8_t(*)[4]) &l[32][sb_idx], 32,
                     &lut, n_blks);
            call_new(a_dst + 128 * 8, 128 * sizeof(pixel),
                     vmask, (const uint8_t(*)[4]) &l[32][sb_idx], 32,
                     &lut, n_blks);
            if (memcmp(c_dst, a_dst, 128 * 16 * sizeof(*a_dst))) fail();

            bench_new(a_dst + 128 * 8, 128 * sizeof(pixel),
                      vmask, (const uint8_t(*)[4]) &l[32][sb_idx], 32,
                      &lut, n_blks);
        }
    }
    report(name);
}

void bitfn(checkasm_check_loopfilter)(void) {
    Dav1dLoopFilterDSPContext c;

    bitfn(dav1d_loop_filter_dsp_init)(&c);

    check_lpf_sb(c.loop_filter_sb128y, "lpf_v_sb128y", 3, 32, 1);
    check_lpf_sb(c.loop_filter_sb128uv, "lpf_v_sb128uv", 2, 16, 2);
}
