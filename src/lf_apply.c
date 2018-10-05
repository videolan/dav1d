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
#include <string.h>

#include "common/intops.h"

#include "src/lf_apply.h"

static inline void filter_plane_cols_y(const Dav1dFrameContext *const f,
                                       const int have_left,
                                       const uint8_t (*lvl)[4],
                                       const ptrdiff_t b4_stride,
                                       const uint32_t (*const mask)[3],
                                       pixel *dst, const ptrdiff_t ls,
                                       const int w,
                                       const int starty4, const int endy4)
{
    const Dav1dDSPContext *const dsp = f->dsp;

    // filter edges between columns (e.g. block1 | block2)
    for (int x = 0; x < w; x++) {
        if (!have_left && !x) continue;
        dsp->lf.loop_filter_sb[0][0](&dst[x * 4], ls,
                                     starty4 ? (const uint32_t[3]) {
                                         mask[x][0] >> starty4,
                                         mask[x][1] >> starty4,
                                         mask[x][2] >> starty4,
                                     } : mask[x],
                                     (const uint8_t(*)[4]) &lvl[x][0], b4_stride,
                                     &f->lf.lim_lut, endy4 - starty4);
    }
}

static inline void filter_plane_rows_y(const Dav1dFrameContext *const f,
                                       const int have_top,
                                       const uint8_t (*lvl)[4],
                                       const ptrdiff_t b4_stride,
                                       const uint32_t (*const mask)[3],
                                       pixel *dst, const ptrdiff_t ls,
                                       const int w,
                                       const int starty4, const int endy4)
{
    const Dav1dDSPContext *const dsp = f->dsp;

    //                                 block1
    // filter edges between rows (e.g. ------)
    //                                 block2
    for (int y = starty4; y < endy4;
         y++, dst += 4 * PXSTRIDE(ls), lvl += b4_stride)
    {
        if (!have_top && !y) continue;
        dsp->lf.loop_filter_sb[0][1](dst, ls, mask[y],
                                     (const uint8_t(*)[4]) &lvl[0][1], b4_stride,
                                     &f->lf.lim_lut, w);
    }
}

static inline void filter_plane_cols_uv(const Dav1dFrameContext *const f,
                                        const int have_left,
                                        const uint8_t (*lvl)[4],
                                        const ptrdiff_t b4_stride,
                                        const uint32_t (*const mask)[2],
                                        pixel *const u, pixel *const v,
                                        const ptrdiff_t ls, const int w,
                                        const int starty4, const int endy4)
{
    const Dav1dDSPContext *const dsp = f->dsp;

    // filter edges between columns (e.g. block1 | block2)
    for (int x = 0; x < w; x++) {
        if (!have_left && !x) continue;
        dsp->lf.loop_filter_sb[1][0](&u[x * 4], ls,
                                     starty4 ? (const uint32_t[2]) {
                                         mask[x][0] >> starty4,
                                         mask[x][1] >> starty4,
                                     } : mask[x],
                                     (const uint8_t(*)[4]) &lvl[x][2], b4_stride,
                                     &f->lf.lim_lut, endy4 - starty4);
        dsp->lf.loop_filter_sb[1][0](&v[x * 4], ls,
                                     starty4 ? (const uint32_t[2]) {
                                         mask[x][0] >> starty4,
                                         mask[x][1] >> starty4,
                                     } : mask[x],
                                     (const uint8_t(*)[4]) &lvl[x][3], b4_stride,
                                     &f->lf.lim_lut, endy4 - starty4);
    }
}

static inline void filter_plane_rows_uv(const Dav1dFrameContext *const f,
                                        const int have_top,
                                        const uint8_t (*lvl)[4],
                                        const ptrdiff_t b4_stride,
                                        const uint32_t (*const mask)[2],
                                        pixel *const u, pixel *const v,
                                        const ptrdiff_t ls, const int w,
                                        const int starty4, const int endy4)
{
    const Dav1dDSPContext *const dsp = f->dsp;
    int y;
    ptrdiff_t off_l;

    //                                 block1
    // filter edges between rows (e.g. ------)
    //                                 block2
    for (off_l = 0, y = starty4; y < endy4;
         y++, off_l += 4 * PXSTRIDE(ls), lvl += b4_stride)
    {
        if (!have_top && !y) continue;
        dsp->lf.loop_filter_sb[1][1](&u[off_l], ls, mask[y],
                                     (const uint8_t(*)[4]) &lvl[0][2], b4_stride,
                                     &f->lf.lim_lut, w);
        dsp->lf.loop_filter_sb[1][1](&v[off_l], ls, mask[y],
                                     (const uint8_t(*)[4]) &lvl[0][3], b4_stride,
                                     &f->lf.lim_lut, w);
    }
}

void bytefn(dav1d_loopfilter_sbrow)(const Dav1dFrameContext *const f,
                                    pixel *const p[3], Av1Filter *const lflvl,
                                    int sby, const int start_of_tile_row)
{
    int x, have_left;
    // Don't filter outside the frame
    const int hy4 = (f->cur.p.p.h + 3) >> 2;
    const int have_top = sby > 0;
    const int is_sb64 = !f->seq_hdr.sb128;
    const int starty4 = (sby & is_sb64) << 4;
    const int sbsz = 32 >> is_sb64;
    const int sbl2 = 5 - is_sb64;
    const int halign = (f->bh + 31) & ~31;
    const int ss_ver = f->cur.p.p.layout == DAV1D_PIXEL_LAYOUT_I420;
    const int ss_hor = f->cur.p.p.layout != DAV1D_PIXEL_LAYOUT_I444;
    const int endy4 = starty4 + imin(hy4 - sby * f->sb_step, sbsz);
    const int uv_endy4 = (endy4 + ss_ver) >> ss_ver;

    // fix lpf strength at tile col boundaries
    const uint8_t *lpf_y = &f->lf.tx_lpf_right_edge[0][sby << sbl2];
    const uint8_t *lpf_uv = &f->lf.tx_lpf_right_edge[1][sby << (sbl2 - ss_ver)];
    for (int tile_col = 1;; tile_col++) {
        x = f->frame_hdr.tiling.col_start_sb[tile_col];
        if ((x << sbl2) >= f->bw) break;
        const int bx4 = x & is_sb64 ? 16 : 0, cbx4 = bx4 >> ss_hor;
        x >>= is_sb64;
        for (unsigned y = starty4, mask = 1 << y; y < endy4; y++, mask <<= 1) {
            const int idx = 2 * !!(lflvl[x].filter_y[0][bx4][2] & mask) +
                                !!(lflvl[x].filter_y[0][bx4][1] & mask);
            lflvl[x].filter_y[0][bx4][2] &= ~mask;
            lflvl[x].filter_y[0][bx4][1] &= ~mask;
            lflvl[x].filter_y[0][bx4][0] &= ~mask;
            lflvl[x].filter_y[0][bx4][imin(idx, lpf_y[y - starty4])] |= mask;
        }
        for (unsigned y = starty4 >> ss_ver, uv_mask = 1 << y; y < uv_endy4;
             y++, uv_mask <<= 1)
        {
            const int idx = !!(lflvl[x].filter_uv[0][cbx4][1] & uv_mask);
            lflvl[x].filter_uv[0][cbx4][1] &= ~uv_mask;
            lflvl[x].filter_uv[0][cbx4][0] &= ~uv_mask;
            lflvl[x].filter_uv[0][cbx4][imin(idx, lpf_uv[y - (starty4 >> ss_ver)])] |= uv_mask;
        }
        lpf_y  += halign;
        lpf_uv += halign >> ss_ver;
    }

    // fix lpf strength at tile row boundaries
    if (start_of_tile_row) {
        const BlockContext *a;
        for (x = 0, a = &f->a[f->sb128w * (start_of_tile_row - 1)];
             x < f->sb128w; x++, a++)
        {
            uint32_t *const y_vmask = lflvl[x].filter_y[1][starty4];
            const unsigned y_vm = y_vmask[0] | y_vmask[1] | y_vmask[2];

            for (unsigned mask = 1, i = 0; i < 32; mask <<= 1, i++) {
                if (!(y_vm & mask)) continue;
                const int idx = 2 * !!(y_vmask[2] & mask) + !!(y_vmask[1] & mask);
                y_vmask[2] &= ~mask;
                y_vmask[1] &= ~mask;
                y_vmask[0] &= ~mask;
                y_vmask[imin(idx, a->tx_lpf_y[i])] |= mask;
            }

            uint32_t *const uv_vmask = lflvl[x].filter_uv[1][starty4 >> ss_ver];
            const unsigned uv_vm = uv_vmask[0] | uv_vmask[1];
            for (unsigned mask = 1, i = 0; i < (32U >> ss_hor); mask <<= 1, i++) {
                if (!(uv_vm & mask)) continue;
                const int idx = !!(uv_vmask[1] & mask);
                uv_vmask[1] &= ~mask;
                uv_vmask[0] &= ~mask;
                uv_vmask[imin(idx, a->tx_lpf_uv[i])] |= mask;
            }
        }
    }

    pixel *ptr;
    uint8_t (*level_ptr)[4] = f->lf.level + f->b4_stride * sby * sbsz;
    for (ptr = p[0], have_left = 0, x = 0; x < f->sb128w;
         x++, have_left = 1, ptr += 128, level_ptr += 32)
    {
        filter_plane_cols_y(f, have_left, level_ptr, f->b4_stride,
                            lflvl[x].filter_y[0], ptr, f->cur.p.stride[0],
                            imin(32, f->bw - x * 32), starty4, endy4);
    }

    level_ptr = f->lf.level + f->b4_stride * sby * sbsz;
    for (ptr = p[0], x = 0; x < f->sb128w; x++, ptr += 128, level_ptr += 32) {
        filter_plane_rows_y(f, have_top, level_ptr, f->b4_stride,
                            lflvl[x].filter_y[1], ptr, f->cur.p.stride[0],
                            imin(32, f->bw - x * 32), starty4, endy4);
    }

    if (!f->frame_hdr.loopfilter.level_u && !f->frame_hdr.loopfilter.level_v)
        return;

    ptrdiff_t uv_off;
    level_ptr = f->lf.level + f->b4_stride * (sby * sbsz >> ss_ver);
    for (uv_off = 0, have_left = 0, x = 0; x < f->sb128w;
         x++, have_left = 1, uv_off += 128 >> ss_hor, level_ptr += 32 >> ss_hor)
    {
        filter_plane_cols_uv(f, have_left, level_ptr, f->b4_stride,
                             lflvl[x].filter_uv[0],
                             &p[1][uv_off], &p[2][uv_off], f->cur.p.stride[1],
                             (imin(32, f->bw - x * 32) + ss_hor) >> ss_hor,
                             starty4 >> ss_ver, uv_endy4);
    }

    level_ptr = f->lf.level + f->b4_stride * (sby * sbsz >> ss_ver);
    for (uv_off = 0, x = 0; x < f->sb128w;
         x++, uv_off += 128 >> ss_hor, level_ptr += 32 >> ss_hor)
    {
        filter_plane_rows_uv(f, have_top, level_ptr, f->b4_stride,
                             lflvl[x].filter_uv[1],
                             &p[1][uv_off], &p[2][uv_off], f->cur.p.stride[1],
                             (imin(32, f->bw - x * 32) + ss_hor) >> ss_hor,
                             starty4 >> ss_ver, uv_endy4);
    }
}
