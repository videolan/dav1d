/*
 * Copyright © 2021, VideoLAN and dav1d authors
 * Copyright © 2021, Martin Storsjo
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

#include "src/arm/asm.S"
#include "util.S"
#include "src/arm/asm-offsets.h"

#define GRAIN_WIDTH 82

.macro gather_interleaved dst1, dst2, src1, src2, off
        umov            w14, \src1[0+\off]
        umov            w15, \src2[1+\off]
        umov            w16, \src1[2+\off]
        add             x14, x14, x3
        umov            w17, \src2[3+\off]
        add             x15, x15, x3
        ld1             {\dst1}[0+\off], [x14]
        umov            w14, \src1[4+\off]
        add             x16, x16, x3
        ld1             {\dst2}[1+\off], [x15]
        umov            w15, \src2[5+\off]
        add             x17, x17, x3
        ld1             {\dst1}[2+\off], [x16]
        umov            w16, \src1[6+\off]
        add             x14, x14, x3
        ld1             {\dst2}[3+\off], [x17]
        umov            w17, \src2[7+\off]
        add             x15, x15, x3
        ld1             {\dst1}[4+\off], [x14]
        add             x16, x16, x3
        ld1             {\dst2}[5+\off], [x15]
        add             x17, x17, x3
        ld1             {\dst1}[6+\off], [x16]
        ld1             {\dst2}[7+\off], [x17]
.endm

.macro gather dst1, dst2, src1, src2
        gather_interleaved \dst1, \dst2, \src1, \src2, 0
        gather_interleaved \dst2, \dst1, \src2, \src1, 0
        gather_interleaved \dst1, \dst2, \src1, \src2, 8
        gather_interleaved \dst2, \dst1, \src2, \src1, 8
.endm

function gather_neon
        gather          v4.b, v5.b, v0.b, v1.b
        ret
endfunc

const overlap_coeffs_0, align=4
        .byte 27, 17, 0,  0,  0,  0,  0,  0
        .byte 17, 27, 32, 32, 32, 32, 32, 32
endconst

const overlap_coeffs_1, align=4
        .byte 23, 0,  0,  0,  0,  0,  0,  0
        .byte 22, 32, 32, 32, 32, 32, 32, 32
endconst

.macro calc_offset offx, offy, src, sx, sy
        and             \offy, \src,  #0xF     // randval & 0xF
        lsr             \offx, \src,  #4       // randval >> 4
.if \sy == 0
        add             \offy, \offy, \offy    // 2 * (randval & 0xF)
.endif
.if \sx == 0
        add             \offx, \offx, \offx    // 2 * (randval >> 4)
.endif
.endm

.macro add_offset dst, offx, offy, src, stride
        madd            \dst, \stride, \offy, \src // grain_lut += grain_stride * offy
        add             \dst, \dst, \offx, uxtw // grain_lut += offx
.endm

// void dav1d_fgy_32x32_XY_8bpc_neon(pixel *const dst, const pixel *const src,
//                                   const ptrdiff_t stride,
//                                   const uint8_t scaling[SCALING_SIZE],
//                                   const int scaling_shift,
//                                   const entry grain_lut[][GRAIN_WIDTH],
//                                   const int offsets[][2],
//                                   const int h, const ptrdiff_t clip);
.macro fgy ox, oy
function fgy_32x32_\ox\oy\()_8bpc_neon, export=1
        str             x30, [sp, #-16]!
.if \ox
        ldr             w11, [x6, #8]          // offsets[1][0]
.endif
.if \oy
        ldr             w13, [x6, #4]          // offsets[0][1]
.endif
.if \ox && \oy
        ldr             w15, [x6, #12]         // offsets[1][1]
.endif
        ldr             w6,  [x6]              // offsets[0][0]
        ldr             w8,  [sp, #16]         // clip
        mov             x9,  #GRAIN_WIDTH      // grain_lut stride

        neg             w4,  w4
        dup             v29.8h, w4             // -scaling_shift

.if \ox || \oy
        movrel          x16, overlap_coeffs_0
.endif

        cbz             w8,  1f
        // clip
        movi            v30.16b, #16
        movi            v31.16b, #235
        b               2f
1:
        // no clip
        movi            v30.16b, #0
        movi            v31.16b, #255
2:

.if \ox || \oy
        ld1             {v27.8b, v28.8b}, [x16] // overlap_coeffs
.endif

        add             x5,  x5,  #9           // grain_lut += 9
        add             x5,  x5,  x9,  lsl #3  // grain_lut += 8 * grain_stride
        add             x5,  x5,  x9           // grain_lut += grain_stride

.if \ox
        calc_offset     w11, w12, w11, 0,  0
.endif
.if \oy
        calc_offset     w13, w14, w13, 0,  0
.endif
.if \ox && \oy
        calc_offset     w15, w16, w15, 0,  0
.endif
        calc_offset     w6,  w10, w6,  0,  0

.if \ox
        add_offset      x12, w11, x12, x5,  x9
.endif
.if \oy
        add_offset      x14, w13, x14, x5,  x9
.endif
.if \ox && \oy
        add_offset      x16, w15, x16, x5,  x9
.endif
        add_offset      x5,  w6,  x10, x5,  x9

.if \ox
        add             x4,  x12, #32          // grain_lut += BLOCK_SIZE * bx
.endif
.if \oy
        add             x6,  x14, x9,  lsl #5  // grain_lut += grain_stride * BLOCK_SIZE * by
.endif
.if \ox && \oy
        add             x8,  x16, x9,  lsl #5  // grain_lut += grain_stride * BLOCK_SIZE * by
        add             x8,  x8,  #32          // grain_lut += BLOCK_SIZE * bx
.endif

.if \oy
        dup             v6.16b,  v27.b[0]
        dup             v7.16b,  v27.b[1]
        mov             w10, w7                // backup actual h
        mov             w7,  #2
.endif

L(loop_\ox\oy):
1:
        ld1             {v0.16b,  v1.16b},  [x1],  x2 // src
.if \ox
        ld1             {v20.8b},           [x4],  x9 // grain_lut old
.endif
.if \oy
        ld1             {v22.16b, v23.16b}, [x6],  x9 // grain_lut top
.endif
.if \ox && \oy
        ld1             {v21.8b},           [x8],  x9 // grain_lut top old
.endif
        ld1             {v18.16b, v19.16b}, [x5],  x9 // grain_lut

        bl              gather_neon

.if \ox
        smull           v20.8h,  v20.8b,  v27.8b
        smlal           v20.8h,  v18.8b,  v28.8b
.endif

.if \oy
.if \ox
        smull           v21.8h,  v21.8b,  v27.8b
        smlal           v21.8h,  v22.8b,  v28.8b
        sqrshrn         v20.8b,  v20.8h,  #5
        sqrshrn         v21.8b,  v21.8h,  #5
.endif

.if \ox
        smull           v16.8h,  v20.8b,  v7.8b
.else
        smull           v16.8h,  v18.8b,  v7.8b
.endif
        smull2          v17.8h,  v18.16b, v7.16b
        smull           v18.8h,  v19.8b,  v7.8b
        smull2          v19.8h,  v19.16b, v7.16b
.if \ox
        smlal           v16.8h,  v21.8b,  v6.8b
.else
        smlal           v16.8h,  v22.8b,  v6.8b
.endif
        smlal2          v17.8h,  v22.16b, v6.16b
        smlal           v18.8h,  v23.8b,  v6.8b
        smlal2          v19.8h,  v23.16b, v6.16b
        sqrshrn         v22.8b,  v16.8h,  #5
        sqrshrn2        v22.16b, v17.8h,  #5
        sqrshrn         v23.8b,  v18.8h,  #5
        sqrshrn2        v23.16b, v19.8h,  #5
.endif

        // sxtl of grain
.if \oy
        sxtl            v16.8h,  v22.8b
        sxtl2           v17.8h,  v22.16b
        sxtl            v18.8h,  v23.8b
        sxtl2           v19.8h,  v23.16b
.elseif \ox
        sqrshrn         v20.8b,  v20.8h,  #5
        sxtl2           v17.8h,  v18.16b
        sxtl            v18.8h,  v19.8b
        sxtl2           v19.8h,  v19.16b
        sxtl            v16.8h,  v20.8b
.else
        sxtl            v16.8h,  v18.8b
        sxtl2           v17.8h,  v18.16b
        sxtl            v18.8h,  v19.8b
        sxtl2           v19.8h,  v19.16b
.endif

        uxtl            v2.8h,   v4.8b   // scaling
        uxtl2           v3.8h,   v4.16b
        uxtl            v4.8h,   v5.8b
        uxtl2           v5.8h,   v5.16b

        mul             v16.8h,  v16.8h,  v2.8h   // scaling * grain
        mul             v17.8h,  v17.8h,  v3.8h
        mul             v18.8h,  v18.8h,  v4.8h
        mul             v19.8h,  v19.8h,  v5.8h

        srshl           v16.8h,  v16.8h,  v29.8h  // round2(scaling * grain, scaling_shift)
        srshl           v17.8h,  v17.8h,  v29.8h
        srshl           v18.8h,  v18.8h,  v29.8h
        srshl           v19.8h,  v19.8h,  v29.8h

        uaddw           v16.8h,  v16.8h,  v0.8b   // *src + noise
        uaddw2          v17.8h,  v17.8h,  v0.16b
        uaddw           v18.8h,  v18.8h,  v1.8b
        uaddw2          v19.8h,  v19.8h,  v1.16b

        sqxtun          v0.8b,   v16.8h
        sqxtun2         v0.16b,  v17.8h
        sqxtun          v1.8b,   v18.8h
        sqxtun2         v1.16b,  v19.8h

        umax            v0.16b,  v0.16b,  v30.16b
        umax            v1.16b,  v1.16b,  v30.16b
        umin            v0.16b,  v0.16b,  v31.16b
        umin            v1.16b,  v1.16b,  v31.16b

        subs            w7,  w7,  #1
.if \oy
        dup             v6.16b,  v28.b[0]
        dup             v7.16b,  v28.b[1]
.endif
        st1             {v0.16b,  v1.16b},  [x0], x2 // dst
        b.gt            1b

.if \oy
        cmp             w10, #2
        sub             w7,  w10, #2           // restore actual remaining h
        b.gt            L(loop_\ox\()0)
.endif
        ldr             x30, [sp], #16
        ret
endfunc
.endm

fgy 0, 0
fgy 0, 1
fgy 1, 0
fgy 1, 1

// void dav1d_fguv_32x32_420_8bpc_neon(pixel *const dst,
//                                     const pixel *const src,
//                                     const ptrdiff_t stride,
//                                     const uint8_t scaling[SCALING_SIZE],
//                                     const Dav1dFilmGrainData *const data,
//                                     const entry grain_lut[][GRAIN_WIDTH],
//                                     const pixel *const luma_row,
//                                     const ptrdiff_t luma_stride,
//                                     const int offsets[][2],
//                                     const ptrdiff_t h, const ptrdiff_t uv,
//                                     const ptrdiff_t is_id,
//                                     const ptrdiff_t type);
.macro fguv layout, sx, sy
function fguv_32x32_\layout\()_8bpc_neon, export=1
        str             x30,      [sp, #-32]!
        stp             d8,  d9,  [sp, #16]
        ldp             x8,  x9,  [sp, #32]    // offsets, h
        ldp             x10, x11, [sp, #48]    // uv, is_id

        ldr             w13, [x4, #FGD_SCALING_SHIFT]
        ldr             w12, [x4, #FGD_CLIP_TO_RESTRICTED_RANGE]
        neg             w13, w13               // -scaling_shift

        // !csfl
        add             x10, x4,  x10, lsl #2  // + 4*uv
        add             x14, x10, #FGD_UV_LUMA_MULT
        add             x15, x10, #FGD_UV_MULT
        add             x10, x10, #FGD_UV_OFFSET
        ld1             {v8.h}[0], [x14] // uv_luma_mult
        ld1r            {v24.8h},  [x10] // uv_offset
        ld1             {v8.h}[1], [x15] // uv_mult

        dup             v29.8h, w13            // -scaling_shift

        cbz             w12, 1f
        // clip
        movi            v30.16b, #16
        movi            v31.16b, #240
        cbz             w11, 2f
        // is_id
        movi            v31.16b, #235
        b               2f
1:
        // no clip
        movi            v30.16b, #0
        movi            v31.16b, #255
2:

        ldr             w12, [x8, #8]          // offsets[1][0]
        ldr             w14, [x8, #4]          // offsets[0][1]
        ldr             w16, [x8, #12]         // offsets[1][1]
        ldr             w8,  [x8]              // offsets[0][0]

        mov             x10, #GRAIN_WIDTH      // grain_lut stride

        add             x5,  x5,  #(3 + (2 >> \sx)*3) // grain_lut += 9 or 6
.if \sy
        add             x5,  x5,  x10, lsl #2  // grain_lut += 4 * grain_stride
        add             x5,  x5,  x10, lsl #1  // grain_lut += 2 * grain_stride
.else
        add             x5,  x5,  x10, lsl #3  // grain_lut += 8 * grain_stride
        add             x5,  x5,  x10          // grain_lut += grain_stride
.endif

        calc_offset     w12, w13, w12, \sx, \sy
        calc_offset     w14, w15, w14, \sx, \sy
        calc_offset     w16, w17, w16, \sx, \sy
        calc_offset     w8,  w11, w8,  \sx, \sy

        add_offset      x13, w12, x13, x5,  x10
        add_offset      x15, w14, x15, x5,  x10
        add_offset      x17, w16, x17, x5,  x10
        add_offset      x5,  w8,  x11, x5,  x10

        add             x4,  x13, #(32 >> \sx) // grain_lut += BLOCK_SIZE * bx
        add             x8,  x15, x10, lsl #(5 - \sy) // grain_lut += grain_stride * BLOCK_SIZE * by
        add             x11, x17, x10, lsl #(5 - \sy) // grain_lut += grain_stride * BLOCK_SIZE * by
        add             x11, x11, #(32 >> \sx) // grain_lut += BLOCK_SIZE * bx

        ldr             w13, [sp, #64]         // type

        movrel          x16, overlap_coeffs_\sx
        adr             x14, L(fguv_loop_sx\sx\()_tbl)

        ld1             {v27.8b, v28.8b}, [x16] // overlap_coeffs
        tst             w13, #1
        ldrh            w13, [x14, w13, uxtw #1]

        b.eq            1f
        // y overlap
        sub             w12, w9,  #(2 >> \sy)  // backup remaining h
        mov             w9,  #(2 >> \sy)

1:
        sub             x13, x14, w13, uxtw

.if \sy
        movi            v25.16b, #23
        movi            v26.16b, #22
.else
        movi            v25.16b, #27
        movi            v26.16b, #17
.endif

.if \sy
        add             x7,  x7,  x7           // luma_stride *= 2
.endif

        br              x13
endfunc
.endm

fguv 420, 1, 1
fguv 422, 1, 0
fguv 444, 0, 0

function fguv_loop_sx0_neon
.macro fguv_loop_sx0 csfl, ox, oy
L(fguv_loop_sx0_csfl\csfl\()_\ox\oy):
1:
        ld1             {v0.16b,  v1.16b},  [x6],  x7  // luma
        ld1             {v6.16b,  v7.16b},  [x1],  x2  // src
.if \ox
        ld1             {v20.8b},           [x4],  x10 // grain_lut old
.endif
.if \oy
        ld1             {v22.16b, v23.16b}, [x8],  x10 // grain_lut top
.endif
.if \ox && \oy
        ld1             {v21.8b},           [x11], x10 // grain_lut top old
.endif
        ld1             {v18.16b, v19.16b}, [x5],  x10 // grain_lut

.if !\csfl
        uxtl            v2.8h,   v0.8b
        uxtl2           v3.8h,   v0.16b
        uxtl            v4.8h,   v1.8b
        uxtl2           v5.8h,   v1.16b
        uxtl            v0.8h,   v6.8b
        uxtl2           v1.8h,   v6.16b
        uxtl            v16.8h,  v7.8b
        uxtl2           v17.8h,  v7.16b
        mul             v2.8h,   v2.8h,   v8.h[0]
        mul             v3.8h,   v3.8h,   v8.h[0]
        mul             v4.8h,   v4.8h,   v8.h[0]
        mul             v5.8h,   v5.8h,   v8.h[0]
        mul             v0.8h,   v0.8h,   v8.h[1]
        mul             v1.8h,   v1.8h,   v8.h[1]
        mul             v16.8h,  v16.8h,  v8.h[1]
        mul             v17.8h,  v17.8h,  v8.h[1]
        sqadd           v2.8h,   v2.8h,   v0.8h
        sqadd           v3.8h,   v3.8h,   v1.8h
        sqadd           v4.8h,   v4.8h,   v16.8h
        sqadd           v5.8h,   v5.8h,   v17.8h
        sshr            v2.8h,   v2.8h,   #6
        sshr            v3.8h,   v3.8h,   #6
        sshr            v4.8h,   v4.8h,   #6
        sshr            v5.8h,   v5.8h,   #6
        add             v2.8h,   v2.8h,   v24.8h
        add             v3.8h,   v3.8h,   v24.8h
        add             v4.8h,   v4.8h,   v24.8h
        add             v5.8h,   v5.8h,   v24.8h
        sqxtun          v0.8b,   v2.8h
        sqxtun2         v0.16b,  v3.8h
        sqxtun          v1.8b,   v4.8h
        sqxtun2         v1.16b,  v5.8h
.endif

        bl              gather_neon

.if \ox
        smull           v20.8h,  v20.8b,  v27.8b
        smlal           v20.8h,  v18.8b,  v28.8b
.endif

.if \oy
.if \ox
        smull           v21.8h,  v21.8b,  v27.8b
        smlal           v21.8h,  v22.8b,  v28.8b
        sqrshrn         v20.8b,  v20.8h,  #5
        sqrshrn         v21.8b,  v21.8h,  #5
.endif

.if \ox
        smull           v16.8h,  v20.8b,  v26.8b
.else
        smull           v16.8h,  v18.8b,  v26.8b
.endif
        smull2          v17.8h,  v18.16b, v26.16b
        smull           v18.8h,  v19.8b,  v26.8b
        smull2          v19.8h,  v19.16b, v26.16b
.if \ox
        smlal           v16.8h,  v21.8b,  v25.8b
.else
        smlal           v16.8h,  v22.8b,  v25.8b
.endif
        smlal2          v17.8h,  v22.16b, v25.16b
        smlal           v18.8h,  v23.8b,  v25.8b
        smlal2          v19.8h,  v23.16b, v25.16b
        sqrshrn         v22.8b,  v16.8h,  #5
        sqrshrn2        v22.16b, v17.8h,  #5
        sqrshrn         v23.8b,  v18.8h,  #5
        sqrshrn2        v23.16b, v19.8h,  #5
.endif

        // sxtl of grain
.if \oy
        sxtl            v16.8h,  v22.8b
        sxtl2           v17.8h,  v22.16b
        sxtl            v18.8h,  v23.8b
        sxtl2           v19.8h,  v23.16b
.elseif \ox
        sqrshrn         v20.8b,  v20.8h,  #5
        sxtl2           v17.8h,  v18.16b
        sxtl            v18.8h,  v19.8b
        sxtl2           v19.8h,  v19.16b
        sxtl            v16.8h,  v20.8b
.else
        sxtl            v16.8h,  v18.8b
        sxtl2           v17.8h,  v18.16b
        sxtl            v18.8h,  v19.8b
        sxtl2           v19.8h,  v19.16b
.endif

        uxtl            v2.8h,   v4.8b   // scaling
        uxtl2           v3.8h,   v4.16b
        uxtl            v4.8h,   v5.8b
        uxtl2           v5.8h,   v5.16b

        mul             v16.8h,  v16.8h,  v2.8h   // scaling * grain
        mul             v17.8h,  v17.8h,  v3.8h
        mul             v18.8h,  v18.8h,  v4.8h
        mul             v19.8h,  v19.8h,  v5.8h

        srshl           v16.8h,  v16.8h,  v29.8h  // round2(scaling * grain, scaling_shift)
        srshl           v17.8h,  v17.8h,  v29.8h
        srshl           v18.8h,  v18.8h,  v29.8h
        srshl           v19.8h,  v19.8h,  v29.8h

        uaddw           v16.8h,  v16.8h,  v6.8b   // *src + noise
        uaddw2          v17.8h,  v17.8h,  v6.16b
        uaddw           v18.8h,  v18.8h,  v7.8b
        uaddw2          v19.8h,  v19.8h,  v7.16b

        sqxtun          v0.8b,   v16.8h
        sqxtun2         v0.16b,  v17.8h
        sqxtun          v1.8b,   v18.8h
        sqxtun2         v1.16b,  v19.8h

        umax            v0.16b,  v0.16b,  v30.16b
        umax            v1.16b,  v1.16b,  v30.16b
        umin            v0.16b,  v0.16b,  v31.16b
        umin            v1.16b,  v1.16b,  v31.16b

        subs            w9,  w9,  #1
.if \oy
        dup             v25.16b, v28.b[0]
        dup             v26.16b, v28.b[1]
.endif
        st1             {v0.16b,  v1.16b},  [x0], x2 // dst
        b.gt            1b

.if \oy
        cmp             w12, #0
        mov             w9,  w12               // restore actual remaining h
        b.gt            L(fguv_loop_sx0_csfl\csfl\()_\ox\()0)
.endif
        b               9f
.endm
        fguv_loop_sx0   0, 0, 0
        fguv_loop_sx0   0, 0, 1
        fguv_loop_sx0   0, 1, 0
        fguv_loop_sx0   0, 1, 1
        fguv_loop_sx0   1, 0, 0
        fguv_loop_sx0   1, 0, 1
        fguv_loop_sx0   1, 1, 0
        fguv_loop_sx0   1, 1, 1

9:
        ldp             d8,  d9,  [sp, #16]
        ldr             x30,      [sp], #32
        ret

L(fguv_loop_sx0_tbl):
        .hword L(fguv_loop_sx0_tbl) - L(fguv_loop_sx0_csfl0_00)
        .hword L(fguv_loop_sx0_tbl) - L(fguv_loop_sx0_csfl0_01)
        .hword L(fguv_loop_sx0_tbl) - L(fguv_loop_sx0_csfl0_10)
        .hword L(fguv_loop_sx0_tbl) - L(fguv_loop_sx0_csfl0_11)
        .hword L(fguv_loop_sx0_tbl) - L(fguv_loop_sx0_csfl1_00)
        .hword L(fguv_loop_sx0_tbl) - L(fguv_loop_sx0_csfl1_01)
        .hword L(fguv_loop_sx0_tbl) - L(fguv_loop_sx0_csfl1_10)
        .hword L(fguv_loop_sx0_tbl) - L(fguv_loop_sx0_csfl1_11)
endfunc

function fguv_loop_sx1_neon
.macro fguv_loop_sx1 csfl, ox, oy
L(fguv_loop_sx1_csfl\csfl\()_\ox\oy):
1:
        ld1             {v0.16b, v1.16b},  [x6],  x7  // luma
        ld1             {v6.16b},          [x1],  x2  // src
.if \ox
        ld1             {v20.8b},          [x4],  x10 // grain_lut old
.endif
.if \oy
        ld1             {v22.16b},         [x8],  x10 // grain_lut top
.endif
.if \ox && \oy
        ld1             {v21.8b},          [x11], x10 // grain_lut top old
.endif
        ld1             {v18.16b},         [x5],  x10 // grain_lut

        uaddlp          v2.8h,   v0.16b
        uaddlp          v3.8h,   v1.16b
.if \csfl
        rshrn           v0.8b,   v2.8h,   #1
        rshrn2          v0.16b,  v3.8h,   #1
.else
        urshr           v2.8h,   v2.8h,   #1
        urshr           v3.8h,   v3.8h,   #1
        uxtl            v0.8h,   v6.8b
        uxtl2           v1.8h,   v6.16b
        mul             v2.8h,   v2.8h,   v8.h[0]
        mul             v3.8h,   v3.8h,   v8.h[0]
        mul             v0.8h,   v0.8h,   v8.h[1]
        mul             v1.8h,   v1.8h,   v8.h[1]
        sqadd           v2.8h,   v2.8h,   v0.8h
        sqadd           v3.8h,   v3.8h,   v1.8h
        sshr            v2.8h,   v2.8h,   #6
        sshr            v3.8h,   v3.8h,   #6
        add             v2.8h,   v2.8h,   v24.8h
        add             v3.8h,   v3.8h,   v24.8h
        sqxtun          v0.8b,   v2.8h
        sqxtun2         v0.16b,  v3.8h
.endif

        bl              gather_neon

.if \ox
        smull           v20.8h,  v20.8b,  v27.8b
        smlal           v20.8h,  v18.8b,  v28.8b
.endif

.if \oy
.if \ox
        smull           v21.8h,  v21.8b,  v27.8b
        smlal           v21.8h,  v22.8b,  v28.8b
        sqrshrn         v20.8b,  v20.8h,  #5
        sqrshrn         v21.8b,  v21.8h,  #5
.endif

.if \ox
        smull           v16.8h,  v20.8b,  v26.8b
.else
        smull           v16.8h,  v18.8b,  v26.8b
.endif
        smull2          v17.8h,  v18.16b, v26.16b
.if \ox
        smlal           v16.8h,  v21.8b,  v25.8b
.else
        smlal           v16.8h,  v22.8b,  v25.8b
.endif
        smlal2          v17.8h,  v22.16b, v25.16b
        sqrshrn         v22.8b,  v16.8h,  #5
        sqrshrn2        v22.16b, v17.8h,  #5
.endif

        // sxtl of grain
.if \oy
        sxtl            v16.8h,  v22.8b
        sxtl2           v17.8h,  v22.16b
.elseif \ox
        sqrshrn         v20.8b,  v20.8h,  #5
        sxtl2           v17.8h,  v18.16b
        sxtl            v16.8h,  v20.8b
.else
        sxtl            v16.8h,  v18.8b
        sxtl2           v17.8h,  v18.16b
.endif

        uxtl            v2.8h,   v4.8b   // scaling
        uxtl2           v3.8h,   v4.16b

        mul             v16.8h,  v16.8h,  v2.8h   // scaling * grain
        mul             v17.8h,  v17.8h,  v3.8h

        srshl           v16.8h,  v16.8h,  v29.8h  // round2(scaling * grain, scaling_shift)
        srshl           v17.8h,  v17.8h,  v29.8h

        uaddw           v16.8h,  v16.8h,  v6.8b   // *src + noise
        uaddw2          v17.8h,  v17.8h,  v6.16b

        sqxtun          v0.8b,   v16.8h
        sqxtun2         v0.16b,  v17.8h

        umax            v0.16b,  v0.16b,  v30.16b
        umin            v0.16b,  v0.16b,  v31.16b

.if \oy
        mov             v16.16b, v25.16b
.endif
        subs            w9,  w9,  #1
.if \oy
        mov             v25.16b, v26.16b
        mov             v26.16b, v16.16b
.endif
        st1             {v0.16b},  [x0], x2 // dst
        b.gt            1b

.if \oy
        cmp             w12, #0
        mov             w9,  w12               // restore actual remaining h
        b.gt            L(fguv_loop_sx1_csfl\csfl\()_\ox\()0)
.endif

        b               9f
.endm
        fguv_loop_sx1   0, 0, 0
        fguv_loop_sx1   0, 0, 1
        fguv_loop_sx1   0, 1, 0
        fguv_loop_sx1   0, 1, 1
        fguv_loop_sx1   1, 0, 0
        fguv_loop_sx1   1, 0, 1
        fguv_loop_sx1   1, 1, 0
        fguv_loop_sx1   1, 1, 1

9:
        ldp             d8,  d9,  [sp, #16]
        ldr             x30,      [sp], #32
        ret

L(fguv_loop_sx1_tbl):
        .hword L(fguv_loop_sx1_tbl) - L(fguv_loop_sx1_csfl0_00)
        .hword L(fguv_loop_sx1_tbl) - L(fguv_loop_sx1_csfl0_01)
        .hword L(fguv_loop_sx1_tbl) - L(fguv_loop_sx1_csfl0_10)
        .hword L(fguv_loop_sx1_tbl) - L(fguv_loop_sx1_csfl0_11)
        .hword L(fguv_loop_sx1_tbl) - L(fguv_loop_sx1_csfl1_00)
        .hword L(fguv_loop_sx1_tbl) - L(fguv_loop_sx1_csfl1_01)
        .hword L(fguv_loop_sx1_tbl) - L(fguv_loop_sx1_csfl1_10)
        .hword L(fguv_loop_sx1_tbl) - L(fguv_loop_sx1_csfl1_11)
endfunc
