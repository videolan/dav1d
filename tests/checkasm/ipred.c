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
#include "src/ipred.h"
#include "src/levels.h"

static const char *const intra_pred_mode_names[N_IMPL_INTRA_PRED_MODES] = {
    [DC_PRED]       = "dc",
    [DC_128_PRED]   = "dc_128",
    [TOP_DC_PRED]   = "dc_top",
    [LEFT_DC_PRED]  = "dc_left",
    [HOR_PRED]      = "h",
    [VERT_PRED]     = "v",
    [PAETH_PRED]    = "paeth",
    [SMOOTH_PRED]   = "smooth",
    [SMOOTH_V_PRED] = "smooth_v",
    [SMOOTH_H_PRED] = "smooth_h",
    [Z1_PRED]       = "z1",
    [Z2_PRED]       = "z2",
    [Z3_PRED]       = "z3",
    [FILTER_PRED]   = "filter"
};

static const uint8_t z_angles[27] = {
     3,  6,  9,
    14, 17, 20, 23, 26, 29, 32,
    36, 39, 42, 45, 48, 51, 54,
    58, 61, 64, 67, 70, 73, 76,
    81, 84, 87
};

static void check_intra_pred(Dav1dIntraPredDSPContext *const c) {
    ALIGN_STK_32(pixel, c_dst, 64 * 64,);
    ALIGN_STK_32(pixel, a_dst, 64 * 64,);
    ALIGN_STK_32(pixel, topleft_buf, 257,);
    pixel *const topleft = topleft_buf + 128;

    declare_func(void, pixel *dst, ptrdiff_t stride, const pixel *topleft,
                 int width, int height, int angle);

    for (int mode = 0; mode < N_IMPL_INTRA_PRED_MODES; mode++)
        for (int w = 4; w <= (mode == FILTER_PRED ? 32 : 64); w <<= 1)
            if (check_func(c->intra_pred[mode], "intra_pred_%s_w%d_%dbpc",
                intra_pred_mode_names[mode], w, BITDEPTH))
            {
                for (int h = imax(w / 4, 4); h <= imin(w * 4,
                    (mode == FILTER_PRED ? 32 : 64)); h <<= 1)
                {
                    const ptrdiff_t stride = w * sizeof(pixel);

                    int a = 0;
                    if (mode >= Z1_PRED && mode <= Z3_PRED) /* angle */
                        a = 90 * (mode - Z1_PRED) + z_angles[rand() % 27];
                    else if (mode == FILTER_PRED) /* filter_idx */
                        a = rand() % 5;

                    for (int i = -h * 2; i <= w * 2; i++)
                        topleft[i] = rand() & ((1 << BITDEPTH) - 1);

                    call_ref(c_dst, stride, topleft, w, h, a);
                    call_new(a_dst, stride, topleft, w, h, a);
                    if (memcmp(c_dst, a_dst, w * h * sizeof(*c_dst)))
                        fail();

                    bench_new(a_dst, stride, topleft, w, h, a);
                }
            }
    report("intra_pred");
}

void bitfn(checkasm_check_ipred)(void) {
    Dav1dIntraPredDSPContext c;
    bitfn(dav1d_intra_pred_dsp_init)(&c);

    check_intra_pred(&c);
}
