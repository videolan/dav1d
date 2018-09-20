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

#ifndef __DAV1D_SRC_LOOPFILTER_H__
#define __DAV1D_SRC_LOOPFILTER_H__

#include <stdint.h>
#include <stddef.h>

#include "common/bitdepth.h"

#include "src/levels.h"

#define decl_loopfilter_fn(name) \
void (name)(pixel *dst, ptrdiff_t stride, int mb_lim, int lim, int hev_thr)
typedef decl_loopfilter_fn(*loopfilter_fn);

typedef struct Dav1dLoopFilterDSPContext {
    /*
     * dimension 1: filter taps (0=4, 1=8, 2=16 for luma; 0=4, 1=6 for chroma)
     * dimension 2: 0=col-edge filter (h), 1=row-edge filter (v)
     *
     * dst/stride are aligned by 4
     */
    loopfilter_fn loop_filter[3][2];
    loopfilter_fn loop_filter_uv[2][2];
} Dav1dLoopFilterDSPContext;

void dav1d_loop_filter_dsp_init_8bpc(Dav1dLoopFilterDSPContext *c);
void dav1d_loop_filter_dsp_init_10bpc(Dav1dLoopFilterDSPContext *c);

#endif /* __DAV1D_SRC_LOOPFILTER_H__ */
