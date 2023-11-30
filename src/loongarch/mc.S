/*
 * Copyright © 2023, VideoLAN and dav1d authors
 * Copyright © 2023, Loongson Technology Corporation Limited
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

#include "src/loongarch/loongson_asm.S"

/*
static void warp_affine_8x8_c(pixel *dst, const ptrdiff_t dst_stride,
                              const pixel *src, const ptrdiff_t src_stride,
                              const int16_t *const abcd, int mx, int my
                              HIGHBD_DECL_SUFFIX)
*/
.macro FILTER_WARP_RND_P_LSX in0, in1, in2, in3, out0, out1, out2, out3
    vbsrl.v         vr2,    \in0,     \in1
    vbsrl.v         vr20,   \in0,     \in2
    addi.w          t4,     \in3,     512
    srai.w          t4,     t4,       10
    addi.w          t4,     t4,       64
    slli.w          t4,     t4,       3
    vldx            vr1,    t5,       t4
    add.w           t3,     t3,       t0   // tmx += abcd[0]

    addi.w          t4,     t3,       512
    srai.w          t4,     t4,       10
    addi.w          t4,     t4,       64
    slli.w          t4,     t4,       3
    vldx            vr29,   t5,       t4
    add.w           t3,     t3,       t0   // tmx += abcd[0]

    vilvl.d         vr2,    vr20,     vr2
    vilvl.d         vr1,    vr29,     vr1
    vmulwev.h.bu.b  vr3,    vr2,      vr1
    vmulwod.h.bu.b  vr20,   vr2,      vr1
    vilvl.d         vr2,    vr20,     vr3
    vhaddw.w.h      vr2,    vr2,      vr2
    vhaddw.d.w      vr2,    vr2,      vr2
    vhaddw.q.d      vr2,    vr2,      vr2
    vilvh.d         vr3,    vr20,     vr3
    vhaddw.w.h      vr3,    vr3,      vr3
    vhaddw.d.w      vr3,    vr3,      vr3
    vhaddw.q.d      vr3,    vr3,      vr3
    vextrins.w      \out0,  vr2,      \out1
    vextrins.w      \out2,  vr3,      \out3
.endm

.macro FILTER_WARP_CLIP_LSX in0, in1, in2, out0, out1
    add.w           \in0,     \in0,    \in1
    addi.w          t6,       \in0,    512
    srai.w          t6,       t6,      10
    addi.w          t6,       t6,      64
    slli.w          t6,       t6,      3
    fldx.d          f1,       t5,      t6
    vsllwil.h.b     vr1,      vr1,     0
    vmulwev.w.h     vr3,      \in2,    vr1
    vmaddwod.w.h    vr3,      \in2,    vr1
    vhaddw.d.w      vr3,      vr3,     vr3
    vhaddw.q.d      vr3,      vr3,     vr3
    vextrins.w      \out0,    vr3,     \out1
.endm

const warp_sh
.rept 2
.byte 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17
.endr
.rept 2
.byte 18, 19, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
.endr
endconst

.macro warp_lsx t, shift
function warp_affine_8x8\t\()_8bpc_lsx
    addi.d          sp,       sp,      -64
    fst.d           f24,      sp,      0
    fst.d           f25,      sp,      8
    fst.d           f26,      sp,      16
    fst.d           f27,      sp,      24
    fst.d           f28,      sp,      32
    fst.d           f29,      sp,      40
    fst.d           f30,      sp,      48
    fst.d           f31,      sp,      56

    la.local        t4,       warp_sh
    ld.h            t0,       a4,      0   // abcd[0]
    ld.h            t1,       a4,      2   // abcd[1]

    alsl.w          t2,       a3,      a3,     1
    addi.w          t3,       a5,      0
    la.local        t5,       dav1d_mc_warp_filter
    sub.d           a2,       a2,      t2
    addi.d          a2,       a2,      -3
    vld             vr0,      a2,      0
    vld             vr30,     t4,      0
    vld             vr31,     t4,      32

    FILTER_WARP_RND_P_LSX   vr0, 0, 1, a5, vr4, 0x00, vr5, 0x00
    FILTER_WARP_RND_P_LSX   vr0, 2, 3, t3, vr6, 0x00, vr7, 0x00
    FILTER_WARP_RND_P_LSX   vr0, 4, 5, t3, vr8, 0x00, vr9, 0x00
    FILTER_WARP_RND_P_LSX   vr0, 6, 7, t3, vr10, 0x00, vr11, 0x00

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    FILTER_WARP_RND_P_LSX   vr0, 0, 1, a5, vr4, 0x10, vr5, 0x10
    FILTER_WARP_RND_P_LSX   vr0, 2, 3, t3, vr6, 0x10, vr7, 0x10
    FILTER_WARP_RND_P_LSX   vr0, 4, 5, t3, vr8, 0x10, vr9, 0x10
    FILTER_WARP_RND_P_LSX   vr0, 6, 7, t3, vr10, 0x10, vr11, 0x10

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    FILTER_WARP_RND_P_LSX   vr0, 0, 1, a5, vr4, 0x20, vr5, 0x20
    FILTER_WARP_RND_P_LSX   vr0, 2, 3, t3, vr6, 0x20, vr7, 0x20
    FILTER_WARP_RND_P_LSX   vr0, 4, 5, t3, vr8, 0x20, vr9, 0x20
    FILTER_WARP_RND_P_LSX   vr0, 6, 7, t3, vr10, 0x20, vr11, 0x20

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    FILTER_WARP_RND_P_LSX   vr0, 0, 1, a5, vr4, 0x30, vr5, 0x30
    FILTER_WARP_RND_P_LSX   vr0, 2, 3, t3, vr6, 0x30, vr7, 0x30
    FILTER_WARP_RND_P_LSX   vr0, 4, 5, t3, vr8, 0x30, vr9, 0x30
    FILTER_WARP_RND_P_LSX   vr0, 6, 7, t3, vr10, 0x30, vr11, 0x30

    add.w           a5,       t1,      a5
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    FILTER_WARP_RND_P_LSX   vr0, 0, 1, a5, vr12, 0x00, vr13, 0x00
    FILTER_WARP_RND_P_LSX   vr0, 2, 3, t3, vr14, 0x00, vr15, 0x00
    FILTER_WARP_RND_P_LSX   vr0, 4, 5, t3, vr16, 0x00, vr17, 0x00
    FILTER_WARP_RND_P_LSX   vr0, 6, 7, t3, vr18, 0x00, vr19, 0x00

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    FILTER_WARP_RND_P_LSX   vr0, 0, 1, a5, vr12, 0x10, vr13, 0x10
    FILTER_WARP_RND_P_LSX   vr0, 2, 3, t3, vr14, 0x10, vr15, 0x10
    FILTER_WARP_RND_P_LSX   vr0, 4, 5, t3, vr16, 0x10, vr17, 0x10
    FILTER_WARP_RND_P_LSX   vr0, 6, 7, t3, vr18, 0x10, vr19, 0x10

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    FILTER_WARP_RND_P_LSX   vr0, 0, 1, a5, vr12, 0x20, vr13, 0x20
    FILTER_WARP_RND_P_LSX   vr0, 2, 3, t3, vr14, 0x20, vr15, 0x20
    FILTER_WARP_RND_P_LSX   vr0, 4, 5, t3, vr16, 0x20, vr17, 0x20
    FILTER_WARP_RND_P_LSX   vr0, 6, 7, t3, vr18, 0x20, vr19, 0x20

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    FILTER_WARP_RND_P_LSX   vr0, 0, 1, a5, vr12, 0x30, vr13, 0x30
    FILTER_WARP_RND_P_LSX   vr0, 2, 3, t3, vr14, 0x30, vr15, 0x30
    FILTER_WARP_RND_P_LSX   vr0, 4, 5, t3, vr16, 0x30, vr17, 0x30
    FILTER_WARP_RND_P_LSX   vr0, 6, 7, t3, vr18, 0x30, vr19, 0x30

    vsrarni.h.w       vr12,     vr4,     3
    vsrarni.h.w       vr13,     vr5,     3
    vsrarni.h.w       vr14,     vr6,     3
    vsrarni.h.w       vr15,     vr7,     3
    vsrarni.h.w       vr16,     vr8,     3
    vsrarni.h.w       vr17,     vr9,     3
    vsrarni.h.w       vr18,     vr10,    3
    vsrarni.h.w       vr19,     vr11,    3

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    FILTER_WARP_RND_P_LSX   vr0, 0, 1, a5, vr4, 0x00, vr5, 0x00
    FILTER_WARP_RND_P_LSX   vr0, 2, 3, t3, vr6, 0x00, vr7, 0x00
    FILTER_WARP_RND_P_LSX   vr0, 4, 5, t3, vr8, 0x00, vr9, 0x00
    FILTER_WARP_RND_P_LSX   vr0, 6, 7, t3, vr10, 0x00, vr11, 0x00

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    FILTER_WARP_RND_P_LSX   vr0, 0, 1, a5, vr4, 0x10, vr5, 0x10
    FILTER_WARP_RND_P_LSX   vr0, 2, 3, t3, vr6, 0x10, vr7, 0x10
    FILTER_WARP_RND_P_LSX   vr0, 4, 5, t3, vr8, 0x10, vr9, 0x10
    FILTER_WARP_RND_P_LSX   vr0, 6, 7, t3, vr10, 0x10, vr11, 0x10

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    FILTER_WARP_RND_P_LSX   vr0, 0, 1, a5, vr4, 0x20, vr5, 0x20
    FILTER_WARP_RND_P_LSX   vr0, 2, 3, t3, vr6, 0x20, vr7, 0x20
    FILTER_WARP_RND_P_LSX   vr0, 4, 5, t3, vr8, 0x20, vr9, 0x20
    FILTER_WARP_RND_P_LSX   vr0, 6, 7, t3, vr10, 0x20, vr11, 0x20

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    FILTER_WARP_RND_P_LSX   vr0, 0, 1, a5, vr4, 0x30, vr5, 0x30
    FILTER_WARP_RND_P_LSX   vr0, 2, 3, t3, vr6, 0x30, vr7, 0x30
    FILTER_WARP_RND_P_LSX   vr0, 4, 5, t3, vr8, 0x30, vr9, 0x30
    FILTER_WARP_RND_P_LSX   vr0, 6, 7, t3, vr10, 0x30, vr11, 0x30

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    FILTER_WARP_RND_P_LSX   vr0, 0, 1, a5, vr21, 0x00, vr22, 0x00
    FILTER_WARP_RND_P_LSX   vr0, 2, 3, t3, vr23, 0x00, vr24, 0x00
    FILTER_WARP_RND_P_LSX   vr0, 4, 5, t3, vr25, 0x00, vr26, 0x00
    FILTER_WARP_RND_P_LSX   vr0, 6, 7, t3, vr27, 0x00, vr28, 0x00

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    FILTER_WARP_RND_P_LSX   vr0, 0, 1, a5, vr21, 0x10, vr22, 0x10
    FILTER_WARP_RND_P_LSX   vr0, 2, 3, t3, vr23, 0x10, vr24, 0x10
    FILTER_WARP_RND_P_LSX   vr0, 4, 5, t3, vr25, 0x10, vr26, 0x10
    FILTER_WARP_RND_P_LSX   vr0, 6, 7, t3, vr27, 0x10, vr28, 0x10

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    FILTER_WARP_RND_P_LSX   vr0, 0, 1, a5, vr21, 0x20, vr22, 0x20
    FILTER_WARP_RND_P_LSX   vr0, 2, 3, t3, vr23, 0x20, vr24, 0x20
    FILTER_WARP_RND_P_LSX   vr0, 4, 5, t3, vr25, 0x20, vr26, 0x20
    FILTER_WARP_RND_P_LSX   vr0, 6, 7, t3, vr27, 0x20, vr28, 0x20

    vsrarni.h.w     vr21,     vr4,     3
    vsrarni.h.w     vr22,     vr5,     3
    vsrarni.h.w     vr23,     vr6,     3
    vsrarni.h.w     vr24,     vr7,     3
    vsrarni.h.w     vr25,     vr8,     3
    vsrarni.h.w     vr26,     vr9,     3
    vsrarni.h.w     vr27,     vr10,    3
    vsrarni.h.w     vr28,     vr11,    3

    addi.w          t2,       a6,      0   // my
    ld.h            t7,       a4,      4   // abcd[2]
    ld.h            t8,       a4,      6   // abcd[3]

.ifnb \t
    slli.d          a1,       a1,      1
.endif

    FILTER_WARP_CLIP_LSX  t2, zero, vr12,  vr4, 0x00
    FILTER_WARP_CLIP_LSX  t2, t7,   vr13,  vr4, 0x10
    FILTER_WARP_CLIP_LSX  t2, t7,   vr14,  vr4, 0x20
    FILTER_WARP_CLIP_LSX  t2, t7,   vr15,  vr4, 0x30
    FILTER_WARP_CLIP_LSX  t2, t7,   vr16,  vr5, 0x00
    FILTER_WARP_CLIP_LSX  t2, t7,   vr17,  vr5, 0x10
    FILTER_WARP_CLIP_LSX  t2, t7,   vr18,  vr5, 0x20
    FILTER_WARP_CLIP_LSX  t2, t7,   vr19,  vr5, 0x30
.ifnb \t
    vssrarni.h.w    vr5,      vr4,     \shift
    vst             vr5,      a0,      0
.else
    vssrarni.hu.w   vr5,      vr4,     \shift
    vssrlni.bu.h    vr5,      vr5,     0
    fst.d           f5,       a0,      0
.endif

    vshuf.b         vr12,     vr21,    vr12,   vr30
    vshuf.b         vr13,     vr22,    vr13,   vr30
    vshuf.b         vr14,     vr23,    vr14,   vr30
    vshuf.b         vr15,     vr24,    vr15,   vr30
    vshuf.b         vr16,     vr25,    vr16,   vr30
    vshuf.b         vr17,     vr26,    vr17,   vr30
    vshuf.b         vr18,     vr27,    vr18,   vr30
    vshuf.b         vr19,     vr28,    vr19,   vr30
    vextrins.h      vr30,     vr31,    0x70

    add.w           a6,       a6,      t8
    addi.w          t2,       a6,      0
    FILTER_WARP_CLIP_LSX  t2, zero, vr12,  vr4, 0x00
    FILTER_WARP_CLIP_LSX  t2, t7,   vr13,  vr4, 0x10
    FILTER_WARP_CLIP_LSX  t2, t7,   vr14,  vr4, 0x20
    FILTER_WARP_CLIP_LSX  t2, t7,   vr15,  vr4, 0x30
    FILTER_WARP_CLIP_LSX  t2, t7,   vr16,  vr5, 0x00
    FILTER_WARP_CLIP_LSX  t2, t7,   vr17,  vr5, 0x10
    FILTER_WARP_CLIP_LSX  t2, t7,   vr18,  vr5, 0x20
    FILTER_WARP_CLIP_LSX  t2, t7,   vr19,  vr5, 0x30
.ifnb \t
    vssrarni.h.w    vr5,      vr4,     \shift
    vstx            vr5,      a0,      a1
.else
    vssrarni.hu.w   vr5,      vr4,     \shift
    vssrlni.bu.h    vr5,      vr5,     0
    fstx.d          f5,       a0,      a1
.endif

    vaddi.bu        vr31,     vr31,    2
    vshuf.b         vr12,     vr21,    vr12,   vr30
    vshuf.b         vr13,     vr22,    vr13,   vr30
    vshuf.b         vr14,     vr23,    vr14,   vr30
    vshuf.b         vr15,     vr24,    vr15,   vr30
    vshuf.b         vr16,     vr25,    vr16,   vr30
    vshuf.b         vr17,     vr26,    vr17,   vr30
    vshuf.b         vr18,     vr27,    vr18,   vr30
    vshuf.b         vr19,     vr28,    vr19,   vr30
    vextrins.h      vr30,     vr31,    0x70

    add.w           a6,       a6,      t8
    addi.w          t2,       a6,      0
    FILTER_WARP_CLIP_LSX  t2, zero, vr12,  vr4, 0x00
    FILTER_WARP_CLIP_LSX  t2, t7,   vr13,  vr4, 0x10
    FILTER_WARP_CLIP_LSX  t2, t7,   vr14,  vr4, 0x20
    FILTER_WARP_CLIP_LSX  t2, t7,   vr15,  vr4, 0x30
    FILTER_WARP_CLIP_LSX  t2, t7,   vr16,  vr5, 0x00
    FILTER_WARP_CLIP_LSX  t2, t7,   vr17,  vr5, 0x10
    FILTER_WARP_CLIP_LSX  t2, t7,   vr18,  vr5, 0x20
    FILTER_WARP_CLIP_LSX  t2, t7,   vr19,  vr5, 0x30
    alsl.d          a0,       a1,      a0,   1
.ifnb \t
    vssrarni.h.w    vr5,      vr4,     \shift
    vst             vr5,      a0,      0
.else
    vssrarni.hu.w   vr5,      vr4,     \shift
    vssrlni.bu.h    vr5,      vr5,     0
    fst.d           f5,       a0,      0
.endif

    vaddi.bu        vr31,     vr31,    2
    vshuf.b         vr12,     vr21,    vr12,   vr30
    vshuf.b         vr13,     vr22,    vr13,   vr30
    vshuf.b         vr14,     vr23,    vr14,   vr30
    vshuf.b         vr15,     vr24,    vr15,   vr30
    vshuf.b         vr16,     vr25,    vr16,   vr30
    vshuf.b         vr17,     vr26,    vr17,   vr30
    vshuf.b         vr18,     vr27,    vr18,   vr30
    vshuf.b         vr19,     vr28,    vr19,   vr30
    vextrins.h      vr30,     vr31,    0x70

    add.w           a6,       a6,       t8
    addi.w          t2,       a6,       0
    FILTER_WARP_CLIP_LSX  t2, zero, vr12,  vr4, 0x00
    FILTER_WARP_CLIP_LSX  t2, t7,   vr13,  vr4, 0x10
    FILTER_WARP_CLIP_LSX  t2, t7,   vr14,  vr4, 0x20
    FILTER_WARP_CLIP_LSX  t2, t7,   vr15,  vr4, 0x30
    FILTER_WARP_CLIP_LSX  t2, t7,   vr16,  vr5, 0x00
    FILTER_WARP_CLIP_LSX  t2, t7,   vr17,  vr5, 0x10
    FILTER_WARP_CLIP_LSX  t2, t7,   vr18,  vr5, 0x20
    FILTER_WARP_CLIP_LSX  t2, t7,   vr19,  vr5, 0x30
.ifnb \t
    vssrarni.h.w    vr5,      vr4,      \shift
    vstx            vr5,      a0,       a1
.else
    vssrarni.hu.w   vr5,      vr4,      \shift
    vssrlni.bu.h    vr5,      vr5,      0
    fstx.d          f5,       a0,       a1
.endif

    vaddi.bu        vr31,     vr31,    2
    vshuf.b         vr12,     vr21,    vr12,   vr30
    vshuf.b         vr13,     vr22,    vr13,   vr30
    vshuf.b         vr14,     vr23,    vr14,   vr30
    vshuf.b         vr15,     vr24,    vr15,   vr30
    vshuf.b         vr16,     vr25,    vr16,   vr30
    vshuf.b         vr17,     vr26,    vr17,   vr30
    vshuf.b         vr18,     vr27,    vr18,   vr30
    vshuf.b         vr19,     vr28,    vr19,   vr30
    vextrins.h      vr30,     vr31,    0x70

    add.w           a6,       a6,       t8
    addi.w          t2,       a6,       0
    FILTER_WARP_CLIP_LSX  t2, zero, vr12,  vr4, 0x00
    FILTER_WARP_CLIP_LSX  t2, t7,   vr13,  vr4, 0x10
    FILTER_WARP_CLIP_LSX  t2, t7,   vr14,  vr4, 0x20
    FILTER_WARP_CLIP_LSX  t2, t7,   vr15,  vr4, 0x30
    FILTER_WARP_CLIP_LSX  t2, t7,   vr16,  vr5, 0x00
    FILTER_WARP_CLIP_LSX  t2, t7,   vr17,  vr5, 0x10
    FILTER_WARP_CLIP_LSX  t2, t7,   vr18,  vr5, 0x20
    FILTER_WARP_CLIP_LSX  t2, t7,   vr19,  vr5, 0x30
    alsl.d          a0,       a1,       a0,   1
.ifnb \t
    vssrarni.h.w    vr5,      vr4,      \shift
    vst             vr5,      a0,       0
.else
    vssrarni.hu.w   vr5,      vr4,      \shift
    vssrlni.bu.h    vr5,      vr5,      0
    fst.d           f5,       a0,       0
.endif

    vaddi.bu        vr31,     vr31,    2
    vshuf.b         vr12,     vr21,    vr12,   vr30
    vshuf.b         vr13,     vr22,    vr13,   vr30
    vshuf.b         vr14,     vr23,    vr14,   vr30
    vshuf.b         vr15,     vr24,    vr15,   vr30
    vshuf.b         vr16,     vr25,    vr16,   vr30
    vshuf.b         vr17,     vr26,    vr17,   vr30
    vshuf.b         vr18,     vr27,    vr18,   vr30
    vshuf.b         vr19,     vr28,    vr19,   vr30
    vextrins.h      vr30,     vr31,    0x70

    add.w           a6,       a6,       t8
    addi.w          t2,       a6,       0
    FILTER_WARP_CLIP_LSX  t2, zero, vr12,  vr4, 0x00
    FILTER_WARP_CLIP_LSX  t2, t7,   vr13,  vr4, 0x10
    FILTER_WARP_CLIP_LSX  t2, t7,   vr14,  vr4, 0x20
    FILTER_WARP_CLIP_LSX  t2, t7,   vr15,  vr4, 0x30
    FILTER_WARP_CLIP_LSX  t2, t7,   vr16,  vr5, 0x00
    FILTER_WARP_CLIP_LSX  t2, t7,   vr17,  vr5, 0x10
    FILTER_WARP_CLIP_LSX  t2, t7,   vr18,  vr5, 0x20
    FILTER_WARP_CLIP_LSX  t2, t7,   vr19,  vr5, 0x30
.ifnb \t
    vssrarni.h.w    vr5,      vr4,      \shift
    vstx            vr5,      a0,       a1
.else
    vssrarni.hu.w   vr5,      vr4,      \shift
    vssrlni.bu.h    vr5,      vr5,      0
    fstx.d          f5,       a0,       a1
.endif

    vaddi.bu        vr31,     vr31,    2
    vshuf.b         vr12,     vr21,    vr12,   vr30
    vshuf.b         vr13,     vr22,    vr13,   vr30
    vshuf.b         vr14,     vr23,    vr14,   vr30
    vshuf.b         vr15,     vr24,    vr15,   vr30
    vshuf.b         vr16,     vr25,    vr16,   vr30
    vshuf.b         vr17,     vr26,    vr17,   vr30
    vshuf.b         vr18,     vr27,    vr18,   vr30
    vshuf.b         vr19,     vr28,    vr19,   vr30
    vextrins.h      vr30,     vr31,    0x70

    add.w           a6,       a6,       t8
    addi.w          t2,       a6,       0
    FILTER_WARP_CLIP_LSX  t2, zero, vr12,  vr4, 0x00
    FILTER_WARP_CLIP_LSX  t2, t7,   vr13,  vr4, 0x10
    FILTER_WARP_CLIP_LSX  t2, t7,   vr14,  vr4, 0x20
    FILTER_WARP_CLIP_LSX  t2, t7,   vr15,  vr4, 0x30
    FILTER_WARP_CLIP_LSX  t2, t7,   vr16,  vr5, 0x00
    FILTER_WARP_CLIP_LSX  t2, t7,   vr17,  vr5, 0x10
    FILTER_WARP_CLIP_LSX  t2, t7,   vr18,  vr5, 0x20
    FILTER_WARP_CLIP_LSX  t2, t7,   vr19,  vr5, 0x30
    alsl.d          a0,       a1,       a0,   1
.ifnb \t
    vssrarni.h.w    vr5,      vr4,      \shift
    vst             vr5,      a0,       0
.else
    vssrarni.hu.w   vr5,      vr4,      \shift
    vssrlni.bu.h    vr5,      vr5,      0
    fst.d           f5,       a0,       0
.endif

    vshuf.b         vr12,     vr21,    vr12,   vr30
    vshuf.b         vr13,     vr22,    vr13,   vr30
    vshuf.b         vr14,     vr23,    vr14,   vr30
    vshuf.b         vr15,     vr24,    vr15,   vr30
    vshuf.b         vr16,     vr25,    vr16,   vr30
    vshuf.b         vr17,     vr26,    vr17,   vr30
    vshuf.b         vr18,     vr27,    vr18,   vr30
    vshuf.b         vr19,     vr28,    vr19,   vr30

    add.w           a6,       a6,       t8
    addi.w          t2,       a6,       0
    FILTER_WARP_CLIP_LSX  t2, zero, vr12,  vr4, 0x00
    FILTER_WARP_CLIP_LSX  t2, t7,   vr13,  vr4, 0x10
    FILTER_WARP_CLIP_LSX  t2, t7,   vr14,  vr4, 0x20
    FILTER_WARP_CLIP_LSX  t2, t7,   vr15,  vr4, 0x30
    FILTER_WARP_CLIP_LSX  t2, t7,   vr16,  vr5, 0x00
    FILTER_WARP_CLIP_LSX  t2, t7,   vr17,  vr5, 0x10
    FILTER_WARP_CLIP_LSX  t2, t7,   vr18,  vr5, 0x20
    FILTER_WARP_CLIP_LSX  t2, t7,   vr19,  vr5, 0x30
.ifnb \t
    vssrarni.h.w    vr5,      vr4,      \shift
    vstx            vr5,      a0,       a1
.else
    vssrarni.hu.w   vr5,      vr4,      \shift
    vssrlni.bu.h    vr5,      vr5,      0
    fstx.d          f5,       a0,       a1
.endif

    fld.d           f24,      sp,       0
    fld.d           f25,      sp,       8
    fld.d           f26,      sp,       16
    fld.d           f27,      sp,       24
    fld.d           f28,      sp,       32
    fld.d           f29,      sp,       40
    fld.d           f30,      sp,       48
    fld.d           f31,      sp,       56
    addi.d          sp,       sp,       64
endfunc
.endm

warp_lsx , 11
warp_lsx t, 7

.macro FILTER_WARP_RND_P_LASX in0, in1, in2, out0, out1, out2, out3
    xvshuf.b        xr2,    \in0,     \in0,     \in2

    addi.w          t4,     \in1,     512
    srai.w          t4,     t4,       10
    addi.w          t4,     t4,       64
    slli.w          t4,     t4,       3
    vldx            vr3,    t5,       t4
    add.w           t3,     t3,       t0   // tmx += abcd[0]

    addi.w          t4,     t3,       512
    srai.w          t4,     t4,       10
    addi.w          t4,     t4,       64
    slli.w          t4,     t4,       3
    vldx            vr4,    t5,       t4
    add.w           t3,     t3,       t0   // tmx += abcd[0]

    addi.w          t4,     t3,       512
    srai.w          t4,     t4,       10
    addi.w          t4,     t4,       64
    slli.w          t4,     t4,       3
    vldx            vr5,    t5,       t4
    add.w           t3,     t3,       t0   // tmx += abcd[0]

    addi.w          t4,     t3,       512
    srai.w          t4,     t4,       10
    addi.w          t4,     t4,       64
    slli.w          t4,     t4,       3
    vldx            vr6,    t5,       t4
    add.w           t3,     t3,       t0   // tmx += abcd[0]

    xvinsve0.d      xr3,    xr5,      1
    xvinsve0.d      xr3,    xr4,      2
    xvinsve0.d      xr3,    xr6,      3

    xvmulwev.h.bu.b xr4,    xr2,      xr3
    xvmulwod.h.bu.b xr5,    xr2,      xr3
    xvilvl.d        xr2,    xr5,      xr4
    xvilvh.d        xr3,    xr5,      xr4
    xvhaddw.w.h     xr2,    xr2,      xr2
    xvhaddw.w.h     xr3,    xr3,      xr3
    xvhaddw.d.w     xr2,    xr2,      xr2
    xvhaddw.d.w     xr3,    xr3,      xr3
    xvhaddw.q.d     xr2,    xr2,      xr2
    xvhaddw.q.d     xr3,    xr3,      xr3

    xvextrins.w     \out0,  xr2,      \out1
    xvextrins.w     \out2,  xr3,      \out3
.endm

.macro FILTER_WARP_CLIP_LASX in0, in1, in2, out0, out1
    add.w           \in0,     \in0,    \in1
    addi.w          t6,       \in0,    512
    srai.w          t6,       t6,      10
    addi.w          t6,       t6,      64
    slli.w          t6,       t6,      3
    fldx.d          f1,       t5,      t6

    add.w           t2,       t2,      t7
    addi.w          t6,       t2,      512
    srai.w          t6,       t6,      10
    addi.w          t6,       t6,      64
    slli.w          t6,       t6,      3
    fldx.d          f2,       t5,      t6

    vilvl.d         vr0,      vr2,     vr1
    vext2xv.h.b     xr0,      xr0
    xvmulwev.w.h    xr3,      \in2,    xr0
    xvmaddwod.w.h   xr3,      \in2,    xr0
    xvhaddw.d.w     xr3,      xr3,     xr3
    xvhaddw.q.d     xr3,      xr3,     xr3
    xvextrins.w     \out0,    xr3,     \out1
.endm

const shuf0
.byte  0, 1, 2, 3, 4, 5, 6, 7, 2, 3, 4, 5, 6, 7, 8, 9
.byte  1, 2, 3, 4, 5, 6, 7, 8, 3, 4, 5, 6, 7, 8, 9, 10
endconst

.macro warp_lasx t, shift
function warp_affine_8x8\t\()_8bpc_lasx
    addi.d          sp,       sp,      -16
    ld.h            t0,       a4,      0   // abcd[0]
    ld.h            t1,       a4,      2   // abcd[1]
    fst.d           f24,      sp,      0
    fst.d           f25,      sp,      8

    alsl.w          t2,       a3,      a3,     1
    addi.w          t3,       a5,      0
    la.local        t4,       warp_sh
    la.local        t5,       dav1d_mc_warp_filter
    sub.d           a2,       a2,      t2
    addi.d          a2,       a2,      -3
    vld             vr0,      a2,      0
    xvld            xr24,     t4,      0
    xvld            xr25,     t4,      32
    la.local        t2,       shuf0
    xvld            xr1,      t2,      0
    xvpermi.q       xr0,      xr0,     0x00
    xvaddi.bu        xr9,    xr1,      4
    FILTER_WARP_RND_P_LASX xr0, a5, xr1, xr7, 0x00, xr8, 0x00
    FILTER_WARP_RND_P_LASX xr0, t3, xr9, xr10, 0x00, xr11, 0x00

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    xvpermi.q       xr0,      xr0,     0x00
    FILTER_WARP_RND_P_LASX xr0, a5, xr1, xr7, 0x10, xr8, 0x10
    FILTER_WARP_RND_P_LASX xr0, t3, xr9, xr10, 0x10, xr11, 0x10

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    xvpermi.q       xr0,      xr0,     0x00
    FILTER_WARP_RND_P_LASX xr0, a5, xr1, xr7, 0x20, xr8, 0x20
    FILTER_WARP_RND_P_LASX xr0, t3, xr9, xr10, 0x20, xr11, 0x20

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    xvpermi.q       xr0,      xr0,     0x00
    FILTER_WARP_RND_P_LASX xr0, a5, xr1, xr7, 0x30, xr8, 0x30
    FILTER_WARP_RND_P_LASX xr0, t3, xr9, xr10, 0x30, xr11, 0x30

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    xvpermi.q       xr0,      xr0,     0x00
    FILTER_WARP_RND_P_LASX xr0, a5, xr1, xr12, 0x00, xr13, 0x00
    FILTER_WARP_RND_P_LASX xr0, t3, xr9, xr14, 0x00, xr15, 0x00

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    xvpermi.q       xr0,      xr0,     0x00
    FILTER_WARP_RND_P_LASX xr0, a5, xr1, xr12, 0x10, xr13, 0x10
    FILTER_WARP_RND_P_LASX xr0, t3, xr9, xr14, 0x10, xr15, 0x10

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    xvpermi.q       xr0,      xr0,     0x00
    FILTER_WARP_RND_P_LASX xr0, a5, xr1, xr12, 0x20, xr13, 0x20
    FILTER_WARP_RND_P_LASX xr0, t3, xr9, xr14, 0x20, xr15, 0x20

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    xvpermi.q       xr0,      xr0,     0x00
    FILTER_WARP_RND_P_LASX xr0, a5, xr1, xr12, 0x30, xr13, 0x30
    FILTER_WARP_RND_P_LASX xr0, t3, xr9, xr14, 0x30, xr15, 0x30

    xvsrarni.h.w    xr12,     xr7,     3
    xvsrarni.h.w    xr13,     xr8,     3
    xvsrarni.h.w    xr14,     xr10,    3
    xvsrarni.h.w    xr15,     xr11,    3

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    xvpermi.q       xr0,      xr0,     0x00
    FILTER_WARP_RND_P_LASX xr0, a5, xr1, xr7, 0x00, xr8, 0x00
    FILTER_WARP_RND_P_LASX xr0, t3, xr9, xr10, 0x00, xr11, 0x00

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    xvpermi.q       xr0,      xr0,     0x00
    FILTER_WARP_RND_P_LASX xr0, a5, xr1, xr7, 0x10, xr8, 0x10
    FILTER_WARP_RND_P_LASX xr0, t3, xr9, xr10, 0x10, xr11, 0x10

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    xvpermi.q       xr0,      xr0,     0x00
    FILTER_WARP_RND_P_LASX xr0, a5, xr1, xr7, 0x20, xr8, 0x20
    FILTER_WARP_RND_P_LASX xr0, t3, xr9, xr10, 0x20, xr11, 0x20

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    xvpermi.q       xr0,      xr0,     0x00
    FILTER_WARP_RND_P_LASX xr0, a5, xr1, xr7, 0x30, xr8, 0x30
    FILTER_WARP_RND_P_LASX xr0, t3, xr9, xr10, 0x30, xr11, 0x30

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    xvpermi.q       xr0,      xr0,     0x00
    FILTER_WARP_RND_P_LASX xr0, a5, xr1, xr16, 0x00, xr17, 0x00
    FILTER_WARP_RND_P_LASX xr0, t3, xr9, xr18, 0x00, xr19, 0x00

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    xvpermi.q       xr0,      xr0,     0x00
    FILTER_WARP_RND_P_LASX xr0, a5, xr1, xr16, 0x10, xr17, 0x10
    FILTER_WARP_RND_P_LASX xr0, t3, xr9, xr18, 0x10, xr19, 0x10

    add.w           a5,       a5,      t1
    or              t3,       a5,      a5
    add.d           a2,       a2,      a3
    vld             vr0,      a2,      0
    xvpermi.q       xr0,      xr0,     0x00
    FILTER_WARP_RND_P_LASX xr0, a5, xr1, xr16, 0x20, xr17, 0x20
    FILTER_WARP_RND_P_LASX xr0, t3, xr9, xr18, 0x20, xr19, 0x20

    xvsrarni.h.w    xr16,     xr7,     3
    xvsrarni.h.w    xr17,     xr8,     3
    xvsrarni.h.w    xr18,     xr10,    3
    xvsrarni.h.w    xr19,     xr11,    3

    addi.w          t2,       a6,      0   // my
    ld.h            t7,       a4,      4   // abcd[2]
    ld.h            t8,       a4,      6   // abcd[3]

.ifnb \t
    slli.d          a1,       a1,      1
.endif

    // y = 0
    FILTER_WARP_CLIP_LASX  t2, zero, xr12,  xr20, 0x00
    FILTER_WARP_CLIP_LASX  t2, t7,   xr13,  xr20, 0x10
    FILTER_WARP_CLIP_LASX  t2, t7,   xr14,  xr20, 0x20
    FILTER_WARP_CLIP_LASX  t2, t7,   xr15,  xr20, 0x30

    xvshuf.b         xr12,     xr16,    xr12,   xr24
    xvshuf.b         xr13,     xr17,    xr13,   xr24
    xvshuf.b         xr14,     xr18,    xr14,   xr24
    xvshuf.b         xr15,     xr19,    xr15,   xr24
    xvextrins.h      xr24,     xr25,    0x70

    add.w           a6,       a6,      t8
    addi.w          t2,       a6,      0
    FILTER_WARP_CLIP_LASX  t2, zero, xr12,  xr21, 0x00
    FILTER_WARP_CLIP_LASX  t2, t7,   xr13,  xr21, 0x10
    FILTER_WARP_CLIP_LASX  t2, t7,   xr14,  xr21, 0x20
    FILTER_WARP_CLIP_LASX  t2, t7,   xr15,  xr21, 0x30

.ifnb \t
    xvssrarni.h.w   xr21,     xr20,     \shift
    xvpermi.q       xr22,     xr21,     0x01
    vilvl.h         vr23,     vr22,     vr21
    vilvh.h         vr21,     vr22,     vr21
    vst             vr23,     a0,       0
    vstx            vr21,     a0,       a1
.else
    xvssrarni.hu.w   xr21,    xr20,     \shift
    xvssrlni.bu.h    xr22,    xr21,     0
    xvpermi.q        xr23,    xr22,     0x01
    vilvl.b          vr21,    vr23,     vr22
    fst.d            f21,     a0,       0
    add.d            a0,      a0,       a1
    vstelm.d         vr21,    a0,       0,     1
.endif

    xvaddi.bu        xr25,     xr25,    2
    xvshuf.b         xr12,     xr16,    xr12,   xr24
    xvshuf.b         xr13,     xr17,    xr13,   xr24
    xvshuf.b         xr14,     xr18,    xr14,   xr24
    xvshuf.b         xr15,     xr19,    xr15,   xr24
    xvextrins.h      xr24,     xr25,    0x70

    add.w           a6,       a6,      t8
    addi.w          t2,       a6,      0
    FILTER_WARP_CLIP_LASX  t2, zero, xr12,  xr20, 0x00
    FILTER_WARP_CLIP_LASX  t2, t7,   xr13,  xr20, 0x10
    FILTER_WARP_CLIP_LASX  t2, t7,   xr14,  xr20, 0x20
    FILTER_WARP_CLIP_LASX  t2, t7,   xr15,  xr20, 0x30

    xvaddi.bu        xr25,     xr25,    2
    xvshuf.b         xr12,     xr16,    xr12,   xr24
    xvshuf.b         xr13,     xr17,    xr13,   xr24
    xvshuf.b         xr14,     xr18,    xr14,   xr24
    xvshuf.b         xr15,     xr19,    xr15,   xr24
    xvextrins.h      xr24,     xr25,    0x70

    add.w           a6,       a6,      t8
    addi.w          t2,       a6,      0
    FILTER_WARP_CLIP_LASX  t2, zero, xr12,  xr21, 0x00
    FILTER_WARP_CLIP_LASX  t2, t7,   xr13,  xr21, 0x10
    FILTER_WARP_CLIP_LASX  t2, t7,   xr14,  xr21, 0x20
    FILTER_WARP_CLIP_LASX  t2, t7,   xr15,  xr21, 0x30

.ifnb \t
    xvssrarni.h.w   xr21,     xr20,     \shift
    alsl.d          a0,       a1,       a0,     1
    xvpermi.q       xr22,     xr21,     0x01
    vilvl.h         vr23,     vr22,     vr21
    vilvh.h         vr21,     vr22,     vr21
    vst             vr23,     a0,       0
    vstx            vr21,     a0,       a1
.else
    xvssrarni.hu.w   xr21,    xr20,     11
    xvssrlni.bu.h    xr22,    xr21,     0
    xvpermi.q        xr23,    xr22,     0x01
    vilvl.b          vr21,    vr23,     vr22
    add.d            a0,      a0,       a1
    fst.d            f21,     a0,       0
    add.d            a0,      a0,       a1
    vstelm.d         vr21,    a0,       0,     1
.endif

    xvaddi.bu        xr25,     xr25,    2
    xvshuf.b         xr12,     xr16,    xr12,   xr24
    xvshuf.b         xr13,     xr17,    xr13,   xr24
    xvshuf.b         xr14,     xr18,    xr14,   xr24
    xvshuf.b         xr15,     xr19,    xr15,   xr24
    xvextrins.h      xr24,     xr25,    0x70

    add.w           a6,       a6,      t8
    addi.w          t2,       a6,      0
    FILTER_WARP_CLIP_LASX  t2, zero, xr12,  xr20, 0x00
    FILTER_WARP_CLIP_LASX  t2, t7,   xr13,  xr20, 0x10
    FILTER_WARP_CLIP_LASX  t2, t7,   xr14,  xr20, 0x20
    FILTER_WARP_CLIP_LASX  t2, t7,   xr15,  xr20, 0x30

    xvaddi.bu        xr25,     xr25,    2
    xvshuf.b         xr12,     xr16,    xr12,   xr24
    xvshuf.b         xr13,     xr17,    xr13,   xr24
    xvshuf.b         xr14,     xr18,    xr14,   xr24
    xvshuf.b         xr15,     xr19,    xr15,   xr24
    xvextrins.h      xr24,     xr25,    0x70

    add.w           a6,       a6,      t8
    addi.w          t2,       a6,      0
    FILTER_WARP_CLIP_LASX  t2, zero, xr12,  xr21, 0x00
    FILTER_WARP_CLIP_LASX  t2, t7,   xr13,  xr21, 0x10
    FILTER_WARP_CLIP_LASX  t2, t7,   xr14,  xr21, 0x20
    FILTER_WARP_CLIP_LASX  t2, t7,   xr15,  xr21, 0x30

.ifnb \t
    xvssrarni.h.w   xr21,     xr20,     \shift
    alsl.d          a0,       a1,       a0,     1
    xvpermi.q       xr22,     xr21,     0x01
    vilvl.h         vr23,     vr22,     vr21
    vilvh.h         vr21,     vr22,     vr21
    vst             vr23,     a0,       0
    vstx            vr21,     a0,       a1
.else
    xvssrarni.hu.w   xr21,    xr20,     11
    xvssrlni.bu.h    xr22,    xr21,     0
    xvpermi.q        xr23,    xr22,     0x01
    vilvl.b          vr21,    vr23,     vr22
    add.d            a0,      a0,       a1
    fst.d            f21,     a0,       0
    add.d            a0,      a0,       a1
    vstelm.d         vr21,    a0,       0,     1
.endif

    xvaddi.bu        xr25,     xr25,    2
    xvshuf.b         xr12,     xr16,    xr12,   xr24
    xvshuf.b         xr13,     xr17,    xr13,   xr24
    xvshuf.b         xr14,     xr18,    xr14,   xr24
    xvshuf.b         xr15,     xr19,    xr15,   xr24
    xvextrins.h      xr24,     xr25,    0x70

    add.w           a6,       a6,      t8
    addi.w          t2,       a6,      0
    FILTER_WARP_CLIP_LASX  t2, zero, xr12,  xr20, 0x00
    FILTER_WARP_CLIP_LASX  t2, t7,   xr13,  xr20, 0x10
    FILTER_WARP_CLIP_LASX  t2, t7,   xr14,  xr20, 0x20
    FILTER_WARP_CLIP_LASX  t2, t7,   xr15,  xr20, 0x30

    xvshuf.b         xr12,     xr16,    xr12,   xr24
    xvshuf.b         xr13,     xr17,    xr13,   xr24
    xvshuf.b         xr14,     xr18,    xr14,   xr24
    xvshuf.b         xr15,     xr19,    xr15,   xr24

    add.w           a6,       a6,      t8
    addi.w          t2,       a6,      0
    FILTER_WARP_CLIP_LASX  t2, zero, xr12,  xr21, 0x00
    FILTER_WARP_CLIP_LASX  t2, t7,   xr13,  xr21, 0x10
    FILTER_WARP_CLIP_LASX  t2, t7,   xr14,  xr21, 0x20
    FILTER_WARP_CLIP_LASX  t2, t7,   xr15,  xr21, 0x30

.ifnb \t
    xvssrarni.h.w   xr21,     xr20,     \shift
    alsl.d          a0,       a1,       a0,     1
    xvpermi.q       xr22,     xr21,     0x01
    vilvl.h         vr23,     vr22,     vr21
    vilvh.h         vr21,     vr22,     vr21
    vst             vr23,     a0,       0
    vstx            vr21,     a0,       a1
.else
    xvssrarni.hu.w   xr21,    xr20,     11
    xvssrlni.bu.h    xr22,    xr21,     0
    xvpermi.q        xr23,    xr22,     0x01
    vilvl.b          vr21,    vr23,     vr22
    add.d            a0,      a0,       a1
    fst.d            f21,     a0,       0
    add.d            a0,      a0,       a1
    vstelm.d         vr21,    a0,       0,     1
.endif
    fld.d            f24,     sp,       0
    fld.d            f25,     sp,       8
    addi.d           sp,      sp,       16
endfunc
.endm

warp_lasx , 11
warp_lasx t, 7
