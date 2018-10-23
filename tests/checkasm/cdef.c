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
#include "src/cdef.h"

static void init_tmp(pixel *buf, const ptrdiff_t stride,
                     const int w, const int h)
{
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++)
            buf[x] = rand() & ((1 << BITDEPTH) - 1);
        buf += PXSTRIDE(stride);
    }
}

static void check_cdef_direction(const cdef_dir_fn fn) {
    ALIGN_STK_32(pixel, src, 8 * 8,);

    declare_func(int, pixel *src, ptrdiff_t dst_stride, unsigned *var);

    init_tmp(src, 8 * sizeof(pixel), 8, 8);

    if (check_func(fn, "cdef_dir_%dbpc", BITDEPTH)) {
        unsigned c_var, a_var;

        const int c_dir = call_ref(src, 8 * sizeof(pixel), &c_var);
        const int a_dir = call_new(src, 8 * sizeof(pixel), &a_var);
        if (c_var != a_var || c_dir != a_dir) fail();
        bench_new(src, 8 * sizeof(pixel), &a_var);
    }
    report("cdef_dir");
}

void bitfn(checkasm_check_cdef)(void) {
    Dav1dCdefDSPContext c;

    bitfn(dav1d_cdef_dsp_init)(&c);

    check_cdef_direction(c.dir);
}
