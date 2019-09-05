/*
 * Copyright © 2019, VideoLAN and dav1d authors
 * Copyright © 2019, Two Orioles, LLC
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
#include "src/film_grain.h"
#define UNIT_TEST 1
#include "src/fg_apply_tmpl.c"

static void check_gen_grny(const Dav1dFilmGrainDSPContext *const dsp) {
    entry grain_lut_c[GRAIN_HEIGHT][GRAIN_WIDTH];
    entry grain_lut_a[GRAIN_HEIGHT + 1][GRAIN_WIDTH];

    declare_func(void, entry grain_lut[][GRAIN_WIDTH],
                 const Dav1dFilmGrainData *data HIGHBD_DECL_SUFFIX);

    for (int i = 0; i < 4; i++) {
        if (check_func(dsp->generate_grain_y, "gen_grain_y_ar%d_%dbpc", i, BITDEPTH)) {
            Dav1dFilmGrainData fg_data;
            fg_data.seed = rnd() & 0xFFFF;

#if BITDEPTH == 16
            const int bitdepth_max = rnd() & 1 ? 0x3ff : 0xfff;
#endif

            fg_data.grain_scale_shift = rnd() & 3;
            fg_data.ar_coeff_shift = (rnd() & 3) + 6;
            fg_data.ar_coeff_lag = i;
            const int num_y_pos = 2 * fg_data.ar_coeff_lag * (fg_data.ar_coeff_lag + 1);
            for (int n = 0; n < num_y_pos; n++)
                fg_data.ar_coeffs_y[n] = (rnd() & 0xff) - 128;

            call_ref(grain_lut_c, &fg_data HIGHBD_TAIL_SUFFIX);
            call_new(grain_lut_a, &fg_data HIGHBD_TAIL_SUFFIX);
            if (memcmp(grain_lut_c, grain_lut_a,
                       GRAIN_WIDTH * GRAIN_HEIGHT * sizeof(entry)))
            {
                fail();
            }

            bench_new(grain_lut_a, &fg_data HIGHBD_TAIL_SUFFIX);
        }
    }

    report("gen_grain_y");
}

static void check_fgy_sbrow(const Dav1dFilmGrainDSPContext *const dsp) {
    ALIGN_STK_32(pixel, c_dst, 128 * 32,);
    ALIGN_STK_32(pixel, a_dst, 128 * 32,);
    ALIGN_STK_32(pixel, src, 128 * 32,);
    const ptrdiff_t stride = 128 * sizeof(pixel);

    declare_func(void, pixel *dst_row, const pixel *src_row, ptrdiff_t stride,
                 const Dav1dFilmGrainData *data, size_t pw,
                 const uint8_t scaling[SCALING_SIZE],
                 const entry grain_lut[][GRAIN_WIDTH],
                 int bh, int row_num HIGHBD_DECL_SUFFIX);

    if (check_func(dsp->fgy_32x32xn, "fgy_32x32xn_%dbpc", BITDEPTH)) {
        Dav1dFilmGrainData fg_data;
        fg_data.seed = rnd() & 0xFFFF;

#if BITDEPTH == 16
        const int bitdepth_max = rnd() & 1 ? 0x3ff : 0xfff;
#else
        const int bitdepth_max = 0xff;
#endif

        uint8_t scaling[SCALING_SIZE];
        entry grain_lut[GRAIN_HEIGHT + 1][GRAIN_WIDTH];
        fg_data.grain_scale_shift = rnd() & 3;
        fg_data.ar_coeff_shift = (rnd() & 3) + 6;
        fg_data.ar_coeff_lag = rnd() & 3;
        const int num_y_pos = 2 * fg_data.ar_coeff_lag * (fg_data.ar_coeff_lag + 1);
        for (int n = 0; n < num_y_pos; n++)
            fg_data.ar_coeffs_y[n] = (rnd() & 0xff) - 128;
        dsp->generate_grain_y(grain_lut, &fg_data HIGHBD_TAIL_SUFFIX);

        fg_data.num_y_points = 2 + (rnd() % 13);
        const int pad = 0xff / fg_data.num_y_points;
        for (int n = 0; n < fg_data.num_y_points; n++) {
            fg_data.y_points[n][0] = 0xff * n / fg_data.num_y_points;
            fg_data.y_points[n][0] += rnd() % pad;
            fg_data.y_points[n][1] = rnd() & 0xff;
        }
        generate_scaling(bitdepth_from_max(bitdepth_max), fg_data.y_points,
                         fg_data.num_y_points, scaling);

        const int w = 1 + (rnd() & 127);
        const int h = 1 + (rnd() & 31);

        for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++)
                src[y * PXSTRIDE(stride) + x] = rnd() & bitdepth_max;
        memcpy(a_dst, src, stride * h);
        memcpy(c_dst, src, stride * h);
        const int row_num = rnd() & 0x7ff;

        fg_data.clip_to_restricted_range = rnd() & 1;
        fg_data.scaling_shift = (rnd() & 3) + 8;
        for (fg_data.overlap_flag = 0; fg_data.overlap_flag <= 1;
             fg_data.overlap_flag++)
        {
            call_ref(c_dst, src, stride, &fg_data, w, scaling, grain_lut, h,
                     row_num HIGHBD_TAIL_SUFFIX);
            call_new(a_dst, src, stride, &fg_data, w, scaling, grain_lut, h,
                     row_num HIGHBD_TAIL_SUFFIX);

            checkasm_check_pixel(c_dst, stride, a_dst, stride, w, h, "dst");
        }
        fg_data.overlap_flag = 1;
        bench_new(a_dst, src, stride, &fg_data, 64, scaling, grain_lut, 32,
                  row_num HIGHBD_TAIL_SUFFIX);
    }

    report("fgy_32x32xn");
}

void bitfn(checkasm_check_filmgrain)(void) {
    Dav1dFilmGrainDSPContext c;

    bitfn(dav1d_film_grain_dsp_init)(&c);

    check_gen_grny(&c);
    check_fgy_sbrow(&c);
}
