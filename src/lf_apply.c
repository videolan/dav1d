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

static inline int maxifzero(const uint8_t (*const a)[4], const int have_b,
                            const uint8_t (*const b)[4], const int diridx)
{
    const int a_val = (*a)[diridx];
    if (a_val) return a_val;
    if (!have_b) return a_val;
    return (*b)[diridx];
}

static inline void filter_plane_cols_y(const Dav1dFrameContext *const f,
                                       const int have_left,
                                       const uint8_t (*lvl)[4],
                                       const ptrdiff_t b4_stride,
                                       const uint32_t (*const mask)[3],
                                       pixel *dst, const ptrdiff_t ls,
                                       const int starty4, const int endy4)
{
    const Dav1dDSPContext *const dsp = f->dsp;

    // filter edges between columns (e.g. block1 | block2)
    for (int y = starty4; y < endy4;
         y++, dst += 4 * PXSTRIDE(ls), lvl += b4_stride)
    {
        pixel *ptr = dst;
        const uint8_t (*l)[4] = lvl;
        const uint32_t *const hmask = mask[y];
        const unsigned hm = hmask[0] | hmask[1] | hmask[2];

        for (unsigned x = 1; hm & ~(x - 1); l++, x <<= 1, ptr += 4) {
            if ((have_left || x > 1) && (hm & x)) {
                const int L = maxifzero(l, have_left || x > 1, &l[-1], 0);
                if (!L) continue;
                const int H = L >> 4;
                const int E = f->lf.lim_lut.e[L], I = f->lf.lim_lut.i[L];
                const int idx = (hmask[2] & x) ? 2 : !!(hmask[1] & x);

                dsp->lf.loop_filter[idx][0](ptr, ls, E, I, H);
            }
        }
    }
}

static inline void filter_plane_rows_y(const Dav1dFrameContext *const f,
                                       const int have_top,
                                       const uint8_t (*lvl)[4],
                                       const ptrdiff_t b4_stride,
                                       const uint32_t (*const mask)[3],
                                       pixel *dst, const ptrdiff_t ls,
                                       const int starty4, const int endy4)
{
    const Dav1dDSPContext *const dsp = f->dsp;

    //                                 block1
    // filter edges between rows (e.g. ------)
    //                                 block2
    for (int y = starty4; y < endy4;
         y++, dst += 4 * PXSTRIDE(ls), lvl += b4_stride)
    {
        pixel *ptr = dst;
        const uint8_t (*l)[4] = lvl;
        const uint32_t *const vmask = mask[y];
        const unsigned vm = vmask[0] | vmask[1] | vmask[2];

        for (unsigned x = 1; vm & ~(x - 1); x <<= 1, ptr += 4, l++) {
            if ((have_top || y) && (vm & x)) {
                const int L = maxifzero(l, have_top || y, &l[-b4_stride], 1);
                if (!L) continue;
                const int H = L >> 4;
                const int E = f->lf.lim_lut.e[L], I = f->lf.lim_lut.i[L];
                const int idx = (vmask[2] & x) ? 2 : !!(vmask[1] & x);

                dsp->lf.loop_filter[idx][1](ptr, ls, E, I, H);
            }
        }
    }
}

static inline void filter_plane_cols_uv(const Dav1dFrameContext *const f,
                                        const int have_left,
                                        const uint8_t (*lvl)[4],
                                        const ptrdiff_t b4_stride,
                                        const uint32_t (*const mask)[2],
                                        pixel *const u, pixel *const v,
                                        const ptrdiff_t ls,
                                        const int starty4, const int endy4)
{
    const Dav1dDSPContext *const dsp = f->dsp;
    int y;
    ptrdiff_t off_l;
    const int ss_ver = f->cur.p.p.layout == DAV1D_PIXEL_LAYOUT_I420;
    const int ss_hor = f->cur.p.p.layout != DAV1D_PIXEL_LAYOUT_I444;
    const int hstep = 1 << ss_hor;

    // filter edges between columns (e.g. block1 | block2)
    lvl += ss_hor + ss_ver * b4_stride;
    for (off_l = 0, y = starty4; y < endy4;
         y++, off_l += 4 * PXSTRIDE(ls), lvl += b4_stride << ss_ver)
    {
        ptrdiff_t off = off_l;
        const uint8_t (*l)[4] = lvl;
        const uint32_t *const hmask = mask[y];
        const unsigned hm = hmask[0] | hmask[1];

        for (unsigned x = 1; hm & ~(x - 1); l += hstep, x <<= 1, off += 4) {
            if ((have_left || x > 1) && (hm & x)) {
                const int idx = !!(hmask[1] & x);

                const int Lu = maxifzero(l, have_left || x > 1, &l[-hstep], 2);
                if (Lu) {
                    const int H = Lu >> 4;
                    const int E = f->lf.lim_lut.e[Lu], I = f->lf.lim_lut.i[Lu];

                    dsp->lf.loop_filter_uv[idx][0](&u[off], ls, E, I, H);
                }

                const int Lv = maxifzero(l, have_left || x > 1, &l[-hstep], 3);
                if (Lv) {
                    const int H = Lv >> 4;
                    const int E = f->lf.lim_lut.e[Lv], I = f->lf.lim_lut.i[Lv];

                    dsp->lf.loop_filter_uv[idx][0](&v[off], ls, E, I, H);
                }
            }
        }
    }
}

static inline void filter_plane_rows_uv(const Dav1dFrameContext *const f,
                                        const int have_top,
                                        const uint8_t (*lvl)[4],
                                        const ptrdiff_t b4_stride,
                                        const uint32_t (*const mask)[2],
                                        pixel *const u, pixel *const v,
                                        const ptrdiff_t ls,
                                        const int starty4, const int endy4)
{
    const Dav1dDSPContext *const dsp = f->dsp;
    int y;
    ptrdiff_t off_l;
    const int ss_ver = f->cur.p.p.layout == DAV1D_PIXEL_LAYOUT_I420;
    const int ss_hor = f->cur.p.p.layout != DAV1D_PIXEL_LAYOUT_I444;
    const int hstep = 1 << ss_hor;

    //                                 block1
    // filter edges between rows (e.g. ------)
    //                                 block2
    lvl += ss_ver * b4_stride + ss_hor;
    for (off_l = 0, y = starty4; y < endy4;
         y++, off_l += 4 * PXSTRIDE(ls), lvl += b4_stride << ss_ver)
    {
        ptrdiff_t off = off_l;
        const uint8_t (*l)[4] = lvl;
        const uint32_t *const vmask = mask[y];
        const unsigned vm = vmask[0] | vmask[1];

        for (unsigned x = 1; vm & ~(x - 1); x <<= 1, off += 4, l += hstep) {
            if ((have_top || y) && (vm & x)) {
                const int idx = !!(vmask[1] & x);

                const int Lu = maxifzero(l, have_top || y,
                                         &l[-(b4_stride << ss_ver)], 2);
                if (Lu) {
                    const int H = Lu >> 4;
                    const int E = f->lf.lim_lut.e[Lu], I = f->lf.lim_lut.i[Lu];

                    dsp->lf.loop_filter_uv[idx][1](&u[off], ls, E, I, H);
                }

                const int Lv = maxifzero(l, have_top || y,
                                         &l[-(b4_stride << ss_ver)], 3);
                if (Lv) {
                    const int H = Lv >> 4;
                    const int E = f->lf.lim_lut.e[Lv], I = f->lf.lim_lut.i[Lv];

                    dsp->lf.loop_filter_uv[idx][1](&v[off], ls, E, I, H);
                }
            }
        }
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
    const int endy4 = starty4 + imin(hy4 - sby * f->sb_step, sbsz);
    const int halign = (f->bh + 31) & ~31;
    const int ss_ver = f->cur.p.p.layout == DAV1D_PIXEL_LAYOUT_I420;
    const int ss_hor = f->cur.p.p.layout != DAV1D_PIXEL_LAYOUT_I444;

    // fix lpf strength at tile col boundaries
    const uint8_t *lpf_y = &f->lf.tx_lpf_right_edge[0][sby << sbl2];
    const uint8_t *lpf_uv = &f->lf.tx_lpf_right_edge[1][sby << (sbl2 - ss_ver)];
    for (int tile_col = 1;; tile_col++) {
        x = f->frame_hdr.tiling.col_start_sb[tile_col];
        if ((x << sbl2) >= f->bw) break;
        const int mask = x & is_sb64 ? 1 << 16 : 1;
        const int uv_mask = x & is_sb64 ? 1 << (16 >> ss_hor) : 1;
        x >>= is_sb64;
        for (int y = starty4; y < endy4; y++) {
            const int idx = 2 * !!(lflvl[x].filter_y[0][y][2] & mask) +
                                !!(lflvl[x].filter_y[0][y][1] & mask);
            lflvl[x].filter_y[0][y][2] &= ~mask;
            lflvl[x].filter_y[0][y][1] &= ~mask;
            lflvl[x].filter_y[0][y][0] &= ~mask;
            lflvl[x].filter_y[0][y][imin(idx, lpf_y[y - starty4])] |= mask;
        }
        for (int y = starty4 >> ss_ver; y < ((endy4 + ss_ver) >> ss_ver); y++) {
            const int idx = !!(lflvl[x].filter_uv[0][y][1] & uv_mask);
            lflvl[x].filter_uv[0][y][1] &= ~uv_mask;
            lflvl[x].filter_uv[0][y][0] &= ~uv_mask;
            lflvl[x].filter_uv[0][y][imin(idx, lpf_uv[y - (starty4 >> ss_ver)])] |= uv_mask;
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
            for (unsigned mask = 1, i = 0; i < (32 >> ss_hor); mask <<= 1, i++) {
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
                            lflvl[x].filter_y[0],
                            ptr, f->cur.p.stride[0], starty4, endy4);
    }

    level_ptr = f->lf.level + f->b4_stride * sby * sbsz;
    for (ptr = p[0], x = 0; x < f->sb128w; x++, ptr += 128, level_ptr += 32) {
        filter_plane_rows_y(f, have_top, level_ptr, f->b4_stride,
                            lflvl[x].filter_y[1],
                            ptr, f->cur.p.stride[0], starty4, endy4);
    }

    if (!f->frame_hdr.loopfilter.level_u && !f->frame_hdr.loopfilter.level_v)
        return;

    ptrdiff_t uv_off;
    level_ptr = f->lf.level + f->b4_stride * sby * sbsz;
    for (uv_off = 0, have_left = 0, x = 0; x < f->sb128w;
         x++, have_left = 1, uv_off += 128 >> ss_hor, level_ptr += 32)
    {
        filter_plane_cols_uv(f, have_left, level_ptr, f->b4_stride,
                             lflvl[x].filter_uv[0],
                             &p[1][uv_off], &p[2][uv_off], f->cur.p.stride[1],
                             starty4 >> ss_ver, endy4 >> ss_ver);
    }

    level_ptr = f->lf.level + f->b4_stride * sby * sbsz;
    for (uv_off = 0, x = 0; x < f->sb128w;
         x++, uv_off += 128 >> ss_hor, level_ptr += 32)
    {
        filter_plane_rows_uv(f, have_top, level_ptr, f->b4_stride,
                             lflvl[x].filter_uv[1],
                             &p[1][uv_off], &p[2][uv_off], f->cur.p.stride[1],
                             starty4 >> ss_ver, endy4 >> ss_ver);
    }
}
