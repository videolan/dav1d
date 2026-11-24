/*
 * Copyright © 2026, VideoLAN and dav1d authors
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

#include "tests/checkasm/internal.h"
#include "src/lf_mask.h"

#include <stdlib.h>
#include <string.h>

static void check_fill_levelcache_block(Dav1dLfMaskDSPContext *c)
{
    declare_func(void, uint8_t (*level_cache_ptr)[2], ptrdiff_t b_stride,
                 int bw, int bh, unsigned v0, unsigned v1);

    ALIGN_STK_16(uint8_t, c_dst, 32 * 64, [2]);
    ALIGN_STK_16(uint8_t, a_dst, 32 * 64, [2]);

    unsigned v0 = rnd() & 0xff, v1 = rnd() & 0xff;
    for (int w = 1; w <= 32; ++w) {
        int h = rnd() % 32 + 1;
        if (!check_func(c->fill_levelcache_block, "fill_levelcache_block"))
            continue;

        ptrdiff_t stride = w + rnd() % 32;

        memset(c_dst, 0xab, stride * h);
        memset(a_dst, 0xab, stride * h);

        checkasm_call_ref(c_dst, stride, w, h, v0, v1);
        checkasm_call_new(a_dst, stride, w, h, v0, v1);

        checkasm_check1d(uint8_t, (uint8_t*)c_dst, (uint8_t*)a_dst,
                         (int)(stride * h), "level_cache_ptr");
    }

    for (int w = 1; w <= 32; w <<= 1) {
        if (check_func(c->fill_levelcache_block,
                       "fill_levelcache_block_w%d", w)) {
            ptrdiff_t stride = 32;
            int h = w;

            bench_new(alternate(c_dst, a_dst), stride, w, h, v0, v1);
        }
    }


    checkasm_report("fill_levelcache_block");
}

static void check_mask_edges_inter_outer(Dav1dLfMaskDSPContext *c)
{
    declare_func(void, uint8_t *a, uint8_t *l, int by4, int bx4, int w4, int h4,
                 uint16_t (*masks)[32][3][2], const uint8_t txa[2][2][32][32]);

    ALIGN_STK_16(uint16_t, c_masks_dst, 2, [32][3][2]);
    ALIGN_STK_16(uint16_t, a_masks_dst, 2, [32][3][2]);
    ALIGN_STK_32(uint8_t, c_a, 32,);
    ALIGN_STK_32(uint8_t, a_a, 32,);
    ALIGN_STK_16(uint8_t, c_l, 32,);
    ALIGN_STK_16(uint8_t, a_l, 32,);
    ALIGN_STK_64(uint8_t, txa, 2, [2][32][32]);

    for (int w4 = 1; w4 <= 32; w4 <<= 1) {
        if (!check_func(c->mask_edges_inter_outer,
                        "mask_edges_inter_outer_w%d", w4))
            continue;
        // blocks at the right frame edge might be clipped to a non-power-of-two
        // width, so test widths in (w4 / 2, w4]
        const int w = w4 - (rnd() & ((w4 - 1) >> 1));
        const int h4 = rnd() % 32 + 1;
        // blocks are aligned to their (unclipped) width
        const int bx4 = (rnd() % (32 / w4)) * w4, by4 = rnd() % (33 - h4);

        for (size_t i = 0; i < sizeof(c_masks_dst) / sizeof(uint16_t); i++)
            ((uint16_t*)c_masks_dst)[i] = rnd();
        memcpy(a_masks_dst, c_masks_dst, sizeof(a_masks_dst));

        for (int i = 0; i < 32; i++) {
            int r1 = rnd() % 3, r2 = rnd() % 3;
            c_a[i] = a_a[i] = r1;
            c_l[i] = a_l[i] = r2;
        }

        for (size_t i = 0; i < sizeof(txa); i++)
            ((uint8_t*)txa)[i] = rnd() % 3;

        checkasm_call_ref(c_a + bx4, c_l, by4, bx4, w, h4, c_masks_dst, txa);
        checkasm_call_new(a_a + bx4, a_l, by4, bx4, w, h4, a_masks_dst, txa);

        checkasm_check1d(uint16_t, (uint16_t*)c_masks_dst, (uint16_t*)a_masks_dst,
                         sizeof(a_masks_dst) / sizeof(uint16_t), "masks");
        checkasm_check1d(uint8_t, c_l, a_l, sizeof(c_l), "l");
        // entries of a past w (but within w4) may be overwritten for clipped blocks
        checkasm_check1d(uint8_t, c_a, a_a, bx4 + w, "a");
        checkasm_check1d(uint8_t, c_a + bx4 + w4, a_a + bx4 + w4,
                         32 - bx4 - w4, "a (past w4)");

        bench_new(alternate(c_a, a_a), alternate(c_l, a_l), 0, 0, w4, w4,
                  alternate(c_masks_dst, a_masks_dst), txa);
    }

    checkasm_report("mask_edges_inter_outer");
}

void checkasm_check_lf_mask(void)
{
    Dav1dLfMaskDSPContext c;
    dav1d_lf_mask_dsp_init(&c);

    check_fill_levelcache_block(&c);
    check_mask_edges_inter_outer(&c);
}
