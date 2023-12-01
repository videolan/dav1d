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

/*
static void w_avg_c(pixel *dst, const ptrdiff_t dst_stride,
                    const int16_t *tmp1, const int16_t *tmp2,
                    const int w, int h,
                    const int weight HIGHBD_DECL_SUFFIX)
*/

#define bpc8_sh     5     // sh = intermediate_bits + 1
#define bpcw8_sh    8     // sh = intermediate_bits + 4

#define bpc_sh   bpc8_sh
#define bpcw_sh  bpcw8_sh

function avg_8bpc_lsx
    addi.d        t8,     a0,     0

    clz.w         t0,     a4
    li.w          t1,     24
    sub.w         t0,     t0,      t1
    la.local      t1,     .AVG_LSX_JRTABLE
    alsl.d        t0,     t0,      t1,    1
    ld.h          t2,     t0,      0  // The jump addresses are relative to AVG_LSX_JRTABLE
    add.d         t1,     t1,      t2 // Get absolute address
    jirl          $r0,    t1,      0

    .align   3
.AVG_LSX_JRTABLE:
    .hword .AVG_W128_LSX - .AVG_LSX_JRTABLE
    .hword .AVG_W64_LSX  - .AVG_LSX_JRTABLE
    .hword .AVG_W32_LSX  - .AVG_LSX_JRTABLE
    .hword .AVG_W16_LSX  - .AVG_LSX_JRTABLE
    .hword .AVG_W8_LSX   - .AVG_LSX_JRTABLE
    .hword .AVG_W4_LSX   - .AVG_LSX_JRTABLE

.AVG_W4_LSX:
    vld           vr0,    a2,     0
    vld           vr1,    a3,     0
    vadd.h        vr2,    vr0,    vr1
    vssrarni.bu.h vr3,    vr2,    bpc_sh
    vstelm.w      vr3,    a0,     0,    0
    add.d         a0,     a0,     a1
    vstelm.w      vr3,    a0,     0,    1
    addi.w        a5,     a5,     -2
    addi.d        a2,     a2,     16
    addi.d        a3,     a3,     16
    add.d         a0,     a0,     a1
    blt           zero,   a5,     .AVG_W4_LSX
    b             .AVG_END_LSX

.AVG_W8_LSX:
    vld           vr0,    a2,     0
    vld           vr2,    a2,     16
    vld           vr1,    a3,     0
    vld           vr3,    a3,     16
    vadd.h        vr4,    vr0,    vr1
    vadd.h        vr5,    vr2,    vr3
    vssrarni.bu.h vr5,    vr4,    bpc_sh
    addi.w        a5,     a5,     -2
    addi.d        a2,     a2,     32
    vstelm.d      vr5,    a0,     0,    0
    add.d         a0,     a0,     a1
    vstelm.d      vr5,    a0,     0,    1
    addi.d        a3,     a3,     32
    add.d         a0,     a0,     a1
    blt           zero,   a5,     .AVG_W8_LSX
    b             .AVG_END_LSX

.AVG_W16_LSX:
    vld           vr0,    a2,     0
    vld           vr2,    a2,     16
    vld           vr1,    a3,     0
    vld           vr3,    a3,     16
    vadd.h        vr4,    vr0,    vr1
    vadd.h        vr5,    vr2,    vr3
    vssrarni.bu.h vr5,    vr4,    bpc_sh
    addi.w        a5,     a5,     -1
    addi.d        a2,     a2,     32
    vst           vr5,    a0,     0
    addi.d        a3,     a3,     32
    add.d         a0,     a0,     a1
    blt           zero,   a5,     .AVG_W16_LSX
    b             .AVG_END_LSX

.AVG_W32_LSX:
    vld           vr0,    a2,     0
    vld           vr2,    a2,     16
    vld           vr4,    a2,     32
    vld           vr6,    a2,     48
    vld           vr1,    a3,     0
    vld           vr3,    a3,     16
    vld           vr5,    a3,     32
    vld           vr7,    a3,     48
    vadd.h        vr0,    vr0,    vr1
    vadd.h        vr2,    vr2,    vr3
    vadd.h        vr4,    vr4,    vr5
    vadd.h        vr6,    vr6,    vr7
    vssrarni.bu.h vr2,    vr0,    bpc_sh
    vssrarni.bu.h vr6,    vr4,    bpc_sh
    addi.w        a5,     a5,     -1
    addi.d        a2,     a2,     64
    vst           vr2,    a0,     0
    vst           vr6,    a0,     16
    addi.d        a3,     a3,     64
    add.d         a0,     a0,     a1
    blt           zero,   a5,     .AVG_W32_LSX
    b             .AVG_END_LSX

.AVG_W64_LSX:
.rept 4
    vld           vr0,    a2,     0
    vld           vr2,    a2,     16
    vld           vr1,    a3,     0
    vld           vr3,    a3,     16
    vadd.h        vr0,    vr0,    vr1
    vadd.h        vr2,    vr2,    vr3
    vssrarni.bu.h vr2,    vr0,    bpc_sh
    addi.d        a2,     a2,     32
    addi.d        a3,     a3,     32
    vst           vr2,    a0,     0
    addi.d        a0,     a0,     16
.endr
    addi.w        a5,     a5,     -1
    add.d         t8,     t8,     a1
    add.d         a0,     t8,     zero
    blt           zero,   a5,     .AVG_W64_LSX
    b             .AVG_END_LSX

.AVG_W128_LSX:
.rept 8
    vld           vr0,    a2,     0
    vld           vr2,    a2,     16
    vld           vr1,    a3,     0
    vld           vr3,    a3,     16
    vadd.h        vr0,    vr0,    vr1
    vadd.h        vr2,    vr2,    vr3
    vssrarni.bu.h vr2,    vr0,    bpc_sh
    addi.d        a2,     a2,     32
    addi.d        a3,     a3,     32
    vst           vr2,    a0,     0
    addi.d        a0,     a0,     16
.endr
    addi.w        a5,     a5,     -1
    add.d         t8,     t8,     a1
    add.d         a0,     t8,     zero
    blt           zero,   a5,     .AVG_W128_LSX
.AVG_END_LSX:
endfunc

function avg_8bpc_lasx
    clz.w         t0,     a4
    li.w          t1,     24
    sub.w         t0,     t0,      t1
    la.local      t1,     .AVG_LASX_JRTABLE
    alsl.d        t0,     t0,      t1,    1
    ld.h          t2,     t0,      0
    add.d         t1,     t1,      t2
    jirl          $r0,    t1,      0

    .align   3
.AVG_LASX_JRTABLE:
    .hword .AVG_W128_LASX - .AVG_LASX_JRTABLE
    .hword .AVG_W64_LASX  - .AVG_LASX_JRTABLE
    .hword .AVG_W32_LASX  - .AVG_LASX_JRTABLE
    .hword .AVG_W16_LASX  - .AVG_LASX_JRTABLE
    .hword .AVG_W8_LASX   - .AVG_LASX_JRTABLE
    .hword .AVG_W4_LASX   - .AVG_LASX_JRTABLE

.AVG_W4_LASX:
    vld            vr0,    a2,     0
    vld            vr1,    a3,     0
    vadd.h         vr0,    vr0,    vr1
    vssrarni.bu.h  vr1,    vr0,    bpc_sh
    vstelm.w       vr1,    a0,     0,    0
    add.d          a0,     a0,     a1
    vstelm.w       vr1,    a0,     0,    1
    addi.w         a5,     a5,     -2
    addi.d         a2,     a2,     16
    addi.d         a3,     a3,     16
    add.d          a0,     a0,     a1
    blt            zero,   a5,     .AVG_W4_LASX
    b              .AVG_END_LASX
.AVG_W8_LASX:
    xvld           xr0,    a2,     0
    xvld           xr1,    a3,     0
    xvadd.h        xr2,    xr0,    xr1
    xvssrarni.bu.h xr1,    xr2,    bpc_sh
    xvstelm.d      xr1,    a0,     0,    0
    add.d          a0,     a0,     a1
    xvstelm.d      xr1,    a0,     0,    2
    addi.w         a5,     a5,     -2
    addi.d         a2,     a2,     32
    addi.d         a3,     a3,     32
    add.d          a0,     a1,     a0
    blt            zero,   a5,     .AVG_W8_LASX
    b              .AVG_END_LASX
.AVG_W16_LASX:
    xvld           xr0,    a2,     0
    xvld           xr2,    a2,     32
    xvld           xr1,    a3,     0
    xvld           xr3,    a3,     32
    xvadd.h        xr4,    xr0,    xr1
    xvadd.h        xr5,    xr2,    xr3
    xvssrarni.bu.h xr5,    xr4,    bpc_sh
    xvpermi.d      xr2,    xr5,    0xd8
    xvpermi.d      xr3,    xr5,    0x8d
    vst            vr2,    a0,     0
    vstx           vr3,    a0,     a1
    addi.w         a5,     a5,     -2
    addi.d         a2,     a2,     64
    addi.d         a3,     a3,     64
    alsl.d         a0,     a1,     a0,   1
    blt            zero,   a5,     .AVG_W16_LASX
    b              .AVG_END_LASX
.AVG_W32_LASX:
    xvld           xr0,    a2,     0
    xvld           xr2,    a2,     32
    xvld           xr1,    a3,     0
    xvld           xr3,    a3,     32
    xvadd.h        xr4,    xr0,    xr1
    xvadd.h        xr5,    xr2,    xr3
    xvssrarni.bu.h xr5,    xr4,    bpc_sh
    xvpermi.d      xr6,    xr5,    0xd8
    xvst           xr6,    a0,     0
    addi.w         a5,     a5,     -1
    addi.d         a2,     a2,     64
    addi.d         a3,     a3,     64
    add.d          a0,     a0,     a1
    blt            zero,   a5,     .AVG_W32_LASX
    b              .AVG_END_LASX
.AVG_W64_LASX:
    xvld           xr0,    a2,     0
    xvld           xr2,    a2,     32
    xvld           xr4,    a2,     64
    xvld           xr6,    a2,     96
    xvld           xr1,    a3,     0
    xvld           xr3,    a3,     32
    xvld           xr5,    a3,     64
    xvld           xr7,    a3,     96
    xvadd.h        xr0,    xr0,    xr1
    xvadd.h        xr2,    xr2,    xr3
    xvadd.h        xr4,    xr4,    xr5
    xvadd.h        xr6,    xr6,    xr7
    xvssrarni.bu.h xr2,    xr0,    bpc_sh
    xvssrarni.bu.h xr6,    xr4,    bpc_sh
    xvpermi.d      xr1,    xr2,    0xd8
    xvpermi.d      xr3,    xr6,    0xd8
    xvst           xr1,    a0,     0
    xvst           xr3,    a0,     32
    addi.w         a5,     a5,     -1
    addi.d         a2,     a2,     128
    addi.d         a3,     a3,     128
    add.d          a0,     a0,     a1
    blt            zero,   a5,     .AVG_W64_LASX
    b              .AVG_END_LASX
.AVG_W128_LASX:
    xvld           xr0,    a2,     0
    xvld           xr2,    a2,     32
    xvld           xr4,    a2,     64
    xvld           xr6,    a2,     96
    xvld           xr8,    a2,     128
    xvld           xr10,   a2,     160
    xvld           xr12,   a2,     192
    xvld           xr14,   a2,     224
    xvld           xr1,    a3,     0
    xvld           xr3,    a3,     32
    xvld           xr5,    a3,     64
    xvld           xr7,    a3,     96
    xvld           xr9,    a3,     128
    xvld           xr11,   a3,     160
    xvld           xr13,   a3,     192
    xvld           xr15,   a3,     224
    xvadd.h        xr0,    xr0,    xr1
    xvadd.h        xr2,    xr2,    xr3
    xvadd.h        xr4,    xr4,    xr5
    xvadd.h        xr6,    xr6,    xr7
    xvadd.h        xr8,    xr8,    xr9
    xvadd.h        xr10,   xr10,   xr11
    xvadd.h        xr12,   xr12,   xr13
    xvadd.h        xr14,   xr14,   xr15
    xvssrarni.bu.h xr2,    xr0,    bpc_sh
    xvssrarni.bu.h xr6,    xr4,    bpc_sh
    xvssrarni.bu.h xr10,   xr8,    bpc_sh
    xvssrarni.bu.h xr14,   xr12,   bpc_sh
    xvpermi.d      xr1,    xr2,    0xd8
    xvpermi.d      xr3,    xr6,    0xd8
    xvpermi.d      xr5,    xr10,   0xd8
    xvpermi.d      xr7,    xr14,   0xd8
    xvst           xr1,    a0,     0
    xvst           xr3,    a0,     32
    xvst           xr5,    a0,     64
    xvst           xr7,    a0,     96
    addi.w         a5,     a5,     -1
    addi.d         a2,     a2,     256
    addi.d         a3,     a3,     256
    add.d          a0,     a0,     a1
    blt            zero,   a5,     .AVG_W128_LASX
.AVG_END_LASX:
endfunc

function w_avg_8bpc_lsx
    addi.d        t8,     a0,     0
    li.w          t2,     16
    sub.w         t2,     t2,     a6  // 16 - weight
    vreplgr2vr.h  vr21,   a6
    vreplgr2vr.h  vr22,   t2

    clz.w         t0,     a4
    li.w          t1,     24
    sub.w         t0,     t0,      t1
    la.local      t1,     .W_AVG_LSX_JRTABLE
    alsl.d        t0,     t0,      t1,    1
    ld.h          t2,     t0,      0
    add.d         t1,     t1,      t2
    jirl          $r0,    t1,      0

    .align   3
.W_AVG_LSX_JRTABLE:
    .hword .W_AVG_W128_LSX - .W_AVG_LSX_JRTABLE
    .hword .W_AVG_W64_LSX  - .W_AVG_LSX_JRTABLE
    .hword .W_AVG_W32_LSX  - .W_AVG_LSX_JRTABLE
    .hword .W_AVG_W16_LSX  - .W_AVG_LSX_JRTABLE
    .hword .W_AVG_W8_LSX   - .W_AVG_LSX_JRTABLE
    .hword .W_AVG_W4_LSX   - .W_AVG_LSX_JRTABLE

.W_AVG_W4_LSX:
    vld           vr0,    a2,     0
    vld           vr1,    a3,     0
    vmulwev.w.h   vr2,    vr0,    vr21
    vmulwod.w.h   vr3,    vr0,    vr21
    vmaddwev.w.h  vr2,    vr1,    vr22
    vmaddwod.w.h  vr3,    vr1,    vr22
    vssrarni.hu.w vr3,    vr2,    bpcw_sh
    vssrlni.bu.h  vr1,    vr3,    0
    vpickod.w     vr4,    vr2,    vr1
    vilvl.b       vr0,    vr4,    vr1
    fst.s         f0,     a0,     0
    add.d         a0,     a0,     a1
    vstelm.w      vr0,    a0,     0,   1
    addi.w        a5,     a5,     -2
    addi.d        a2,     a2,     16
    addi.d        a3,     a3,     16
    add.d         a0,     a1,     a0
    blt           zero,   a5,     .W_AVG_W4_LSX
    b             .W_AVG_END_LSX
.W_AVG_W8_LSX:
    vld           vr0,    a2,     0
    vld           vr1,    a3,     0
    vmulwev.w.h   vr2,    vr0,    vr21
    vmulwod.w.h   vr3,    vr0,    vr21
    vmaddwev.w.h  vr2,    vr1,    vr22
    vmaddwod.w.h  vr3,    vr1,    vr22
    vssrarni.hu.w vr3,    vr2,    bpcw_sh
    vssrlni.bu.h  vr1,    vr3,    0
    vpickod.w     vr4,    vr2,    vr1
    vilvl.b       vr0,    vr4,    vr1
    fst.d         f0,     a0,     0
    addi.w        a5,     a5,     -1
    addi.d        a2,     a2,     16
    addi.d        a3,     a3,     16
    add.d         a0,     a0,     a1
    blt           zero,   a5,     .W_AVG_W8_LSX
    b             .W_AVG_END_LSX
.W_AVG_W16_LSX:
    vld           vr0,    a2,     0
    vld           vr2,    a2,     16
    vld           vr1,    a3,     0
    vld           vr3,    a3,     16
    vmulwev.w.h   vr4,    vr0,    vr21
    vmulwod.w.h   vr5,    vr0,    vr21
    vmulwev.w.h   vr6,    vr2,    vr21
    vmulwod.w.h   vr7,    vr2,    vr21
    vmaddwev.w.h  vr4,    vr1,    vr22
    vmaddwod.w.h  vr5,    vr1,    vr22
    vmaddwev.w.h  vr6,    vr3,    vr22
    vmaddwod.w.h  vr7,    vr3,    vr22
    vssrarni.hu.w vr6,    vr4,    bpcw_sh
    vssrarni.hu.w vr7,    vr5,    bpcw_sh
    vssrlrni.bu.h vr7,    vr6,    0
    vshuf4i.w     vr8,    vr7,    0x4E
    vilvl.b       vr0,    vr8,    vr7
    vst           vr0,    a0,     0
    addi.w        a5,     a5,     -1
    addi.d        a2,     a2,     32
    addi.d        a3,     a3,     32
    add.d         a0,     a0,     a1
    blt           zero,   a5,     .W_AVG_W16_LSX
    b             .W_AVG_END_LSX
.W_AVG_W32_LSX:
.rept 2
    vld           vr0,    a2,     0
    vld           vr2,    a2,     16
    vld           vr1,    a3,     0
    vld           vr3,    a3,     16
    vmulwev.w.h   vr4,    vr0,    vr21
    vmulwod.w.h   vr5,    vr0,    vr21
    vmulwev.w.h   vr6,    vr2,    vr21
    vmulwod.w.h   vr7,    vr2,    vr21
    vmaddwev.w.h  vr4,    vr1,    vr22
    vmaddwod.w.h  vr5,    vr1,    vr22
    vmaddwev.w.h  vr6,    vr3,    vr22
    vmaddwod.w.h  vr7,    vr3,    vr22
    vssrarni.hu.w vr6,    vr4,    bpcw_sh
    vssrarni.hu.w vr7,    vr5,    bpcw_sh
    vssrlrni.bu.h vr7,    vr6,    0
    vshuf4i.w     vr8,    vr7,    0x4E
    vilvl.b       vr0,    vr8,    vr7
    vst           vr0,    a0,     0
    addi.d        a2,     a2,     32
    addi.d        a3,     a3,     32
    addi.d        a0,     a0,     16
.endr
    addi.w        a5,     a5,     -1
    add.d         t8,     t8,     a1
    add.d         a0,     t8,     zero
    blt           zero,   a5,     .W_AVG_W32_LSX
    b             .W_AVG_END_LSX

.W_AVG_W64_LSX:
.rept 4
    vld           vr0,    a2,     0
    vld           vr2,    a2,     16
    vld           vr1,    a3,     0
    vld           vr3,    a3,     16
    vmulwev.w.h   vr4,    vr0,    vr21
    vmulwod.w.h   vr5,    vr0,    vr21
    vmulwev.w.h   vr6,    vr2,    vr21
    vmulwod.w.h   vr7,    vr2,    vr21
    vmaddwev.w.h  vr4,    vr1,    vr22
    vmaddwod.w.h  vr5,    vr1,    vr22
    vmaddwev.w.h  vr6,    vr3,    vr22
    vmaddwod.w.h  vr7,    vr3,    vr22
    vssrarni.hu.w vr6,    vr4,    bpcw_sh
    vssrarni.hu.w vr7,    vr5,    bpcw_sh
    vssrlrni.bu.h vr7,    vr6,    0
    vshuf4i.w     vr8,    vr7,    0x4E
    vilvl.b       vr0,    vr8,    vr7
    vst           vr0,    a0,     0
    addi.d        a2,     a2,     32
    addi.d        a3,     a3,     32
    addi.d        a0,     a0,     16
.endr
    addi.w        a5,     a5,     -1
    add.d         t8,     t8,     a1
    add.d         a0,     t8,     zero
    blt           zero,   a5,     .W_AVG_W64_LSX
    b             .W_AVG_END_LSX

.W_AVG_W128_LSX:
.rept 8
    vld           vr0,    a2,     0
    vld           vr2,    a2,     16
    vld           vr1,    a3,     0
    vld           vr3,    a3,     16
    vmulwev.w.h   vr4,    vr0,    vr21
    vmulwod.w.h   vr5,    vr0,    vr21
    vmulwev.w.h   vr6,    vr2,    vr21
    vmulwod.w.h   vr7,    vr2,    vr21
    vmaddwev.w.h  vr4,    vr1,    vr22
    vmaddwod.w.h  vr5,    vr1,    vr22
    vmaddwev.w.h  vr6,    vr3,    vr22
    vmaddwod.w.h  vr7,    vr3,    vr22
    vssrarni.hu.w vr6,    vr4,    bpcw_sh
    vssrarni.hu.w vr7,    vr5,    bpcw_sh
    vssrlrni.bu.h vr7,    vr6,    0
    vshuf4i.w     vr8,    vr7,    0x4E
    vilvl.b       vr0,    vr8,    vr7
    vst           vr0,    a0,     0
    addi.d        a2,     a2,     32
    addi.d        a3,     a3,     32
    addi.d        a0,     a0,     16
.endr
    addi.w        a5,     a5,     -1
    add.d         t8,     t8,     a1
    add.d         a0,     t8,     zero
    blt           zero,   a5,     .W_AVG_W128_LSX
.W_AVG_END_LSX:
endfunc

function w_avg_8bpc_lasx
    addi.d        t8,     a0,     0
    li.w          t2,     16
    sub.w         t2,     t2,     a6  // 16 - weight
    xvreplgr2vr.h xr21,   a6
    xvreplgr2vr.h xr22,   t2

    clz.w         t0,     a4
    li.w          t1,     24
    sub.w         t0,     t0,      t1
    la.local      t1,     .W_AVG_LASX_JRTABLE
    alsl.d        t0,     t0,      t1,    1
    ld.h          t2,     t0,      0
    add.d         t1,     t1,      t2
    jirl          $r0,    t1,      0

    .align   3
.W_AVG_LASX_JRTABLE:
    .hword .W_AVG_W128_LASX - .W_AVG_LASX_JRTABLE
    .hword .W_AVG_W64_LASX  - .W_AVG_LASX_JRTABLE
    .hword .W_AVG_W32_LASX  - .W_AVG_LASX_JRTABLE
    .hword .W_AVG_W16_LASX  - .W_AVG_LASX_JRTABLE
    .hword .W_AVG_W8_LASX   - .W_AVG_LASX_JRTABLE
    .hword .W_AVG_W4_LASX   - .W_AVG_LASX_JRTABLE

.W_AVG_W4_LASX:
    vld            vr0,    a2,     0
    vld            vr1,    a3,     0
    xvpermi.d      xr2,    xr0,    0xD8
    xvpermi.d      xr3,    xr1,    0xD8
    xvilvl.h       xr4,    xr3,    xr2
    xvmulwev.w.h   xr0,    xr4,    xr21
    xvmaddwod.w.h  xr0,    xr4,    xr22
    xvssrarni.hu.w xr1,    xr0,    bpcw_sh
    xvssrlni.bu.h  xr0,    xr1,    0
    fst.s          f0,     a0,     0
    add.d          a0,     a0,     a1
    xvstelm.w      xr0,    a0,     0,     4
    addi.w         a5,     a5,     -2
    addi.d         a2,     a2,     16
    addi.d         a3,     a3,     16
    add.d          a0,     a1,     a0
    blt            zero,   a5,     .W_AVG_W4_LASX
    b              .W_AVG_END_LASX

.W_AVG_W8_LASX:
    xvld           xr0,    a2,     0
    xvld           xr1,    a3,     0
    xvmulwev.w.h   xr2,    xr0,    xr21
    xvmulwod.w.h   xr3,    xr0,    xr21
    xvmaddwev.w.h  xr2,    xr1,    xr22
    xvmaddwod.w.h  xr3,    xr1,    xr22
    xvssrarni.hu.w xr3,    xr2,    bpcw_sh
    xvssrlni.bu.h  xr1,    xr3,    0
    xvpickod.w     xr4,    xr2,    xr1
    xvilvl.b       xr0,    xr4,    xr1
    xvstelm.d      xr0,    a0,     0,     0
    add.d          a0,     a0,     a1
    xvstelm.d      xr0,    a0,     0,     2
    addi.w         a5,     a5,     -2
    addi.d         a2,     a2,     32
    addi.d         a3,     a3,     32
    add.d          a0,     a0,     a1
    blt            zero,   a5,     .W_AVG_W8_LASX
    b              .W_AVG_END_LASX

.W_AVG_W16_LASX:
    xvld           xr0,    a2,     0
    xvld           xr1,    a3,     0
    xvmulwev.w.h   xr2,    xr0,    xr21
    xvmulwod.w.h   xr3,    xr0,    xr21
    xvmaddwev.w.h  xr2,    xr1,    xr22
    xvmaddwod.w.h  xr3,    xr1,    xr22
    xvssrarni.hu.w xr3,    xr2,    bpcw_sh
    xvssrlni.bu.h  xr1,    xr3,    0
    xvpickod.w     xr4,    xr2,    xr1
    xvilvl.b       xr0,    xr4,    xr1
    xvpermi.d      xr1,    xr0,    0xD8
    vst            vr1,    a0,     0
    addi.w         a5,     a5,     -1
    addi.d         a2,     a2,     32
    addi.d         a3,     a3,     32
    add.d          a0,     a0,     a1
    blt            zero,   a5,     .W_AVG_W16_LASX
    b              .W_AVG_END_LSX

.W_AVG_W32_LASX:
    xvld           xr0,    a2,     0
    xvld           xr2,    a2,     32
    xvld           xr1,    a3,     0
    xvld           xr3,    a3,     32
    xvmulwev.w.h   xr4,    xr0,    xr21
    xvmulwod.w.h   xr5,    xr0,    xr21
    xvmulwev.w.h   xr6,    xr2,    xr21
    xvmulwod.w.h   xr7,    xr2,    xr21
    xvmaddwev.w.h  xr4,    xr1,    xr22
    xvmaddwod.w.h  xr5,    xr1,    xr22
    xvmaddwev.w.h  xr6,    xr3,    xr22
    xvmaddwod.w.h  xr7,    xr3,    xr22
    xvssrarni.hu.w xr6,    xr4,    bpcw_sh
    xvssrarni.hu.w xr7,    xr5,    bpcw_sh
    xvssrlni.bu.h  xr7,    xr6,    0
    xvshuf4i.w     xr8,    xr7,    0x4E
    xvilvl.b       xr9,    xr8,    xr7
    xvpermi.d      xr0,    xr9,    0xD8
    xvst           xr0,    a0,     0
    addi.w         a5,     a5,     -1
    addi.d         a2,     a2,     64
    addi.d         a3,     a3,     64
    add.d          a0,     a0,     a1
    blt            zero,   a5,     .W_AVG_W32_LASX
    b              .W_AVG_END_LASX

.W_AVG_W64_LASX:
.rept 2
    xvld           xr0,    a2,     0
    xvld           xr2,    a2,     32
    xvld           xr1,    a3,     0
    xvld           xr3,    a3,     32
    xvmulwev.w.h   xr4,    xr0,    xr21
    xvmulwod.w.h   xr5,    xr0,    xr21
    xvmulwev.w.h   xr6,    xr2,    xr21
    xvmulwod.w.h   xr7,    xr2,    xr21
    xvmaddwev.w.h  xr4,    xr1,    xr22
    xvmaddwod.w.h  xr5,    xr1,    xr22
    xvmaddwev.w.h  xr6,    xr3,    xr22
    xvmaddwod.w.h  xr7,    xr3,    xr22
    xvssrarni.hu.w xr6,    xr4,    bpcw_sh
    xvssrarni.hu.w xr7,    xr5,    bpcw_sh
    xvssrlni.bu.h  xr7,    xr6,    0
    xvshuf4i.w     xr8,    xr7,    0x4E
    xvilvl.b       xr9,    xr8,    xr7
    xvpermi.d      xr0,    xr9,    0xD8
    xvst           xr0,    a0,     0
    addi.d         a2,     a2,     64
    addi.d         a3,     a3,     64
    addi.d         a0,     a0,     32
.endr
    addi.w         a5,     a5,     -1
    add.d          t8,     t8,     a1
    add.d          a0,     t8,     zero
    blt            zero,   a5,     .W_AVG_W64_LASX
    b              .W_AVG_END_LASX

.W_AVG_W128_LASX:
.rept 4
    xvld           xr0,    a2,     0
    xvld           xr2,    a2,     32
    xvld           xr1,    a3,     0
    xvld           xr3,    a3,     32
    xvmulwev.w.h   xr4,    xr0,    xr21
    xvmulwod.w.h   xr5,    xr0,    xr21
    xvmulwev.w.h   xr6,    xr2,    xr21
    xvmulwod.w.h   xr7,    xr2,    xr21
    xvmaddwev.w.h  xr4,    xr1,    xr22
    xvmaddwod.w.h  xr5,    xr1,    xr22
    xvmaddwev.w.h  xr6,    xr3,    xr22
    xvmaddwod.w.h  xr7,    xr3,    xr22
    xvssrarni.hu.w xr6,    xr4,    bpcw_sh
    xvssrarni.hu.w xr7,    xr5,    bpcw_sh
    xvssrlni.bu.h  xr7,    xr6,    0
    xvshuf4i.w     xr8,    xr7,    0x4E
    xvilvl.b       xr9,    xr8,    xr7
    xvpermi.d      xr0,    xr9,    0xD8
    xvst           xr0,    a0,     0
    addi.d         a2,     a2,     64
    addi.d         a3,     a3,     64
    addi.d         a0,     a0,     32
.endr

    addi.w         a5,     a5,     -1
    add.d          t8,     t8,     a1
    add.d          a0,     t8,     zero
    blt            zero,   a5,     .W_AVG_W128_LASX
.W_AVG_END_LASX:
endfunc

#undef bpc_sh
#undef bpcw_sh

#define mask_sh         10
/*
static void mask_c(pixel *dst, const ptrdiff_t dst_stride,
                   const int16_t *tmp1, const int16_t *tmp2, const int w, int h,
                   const uint8_t *mask HIGHBD_DECL_SUFFIX)
*/
function mask_8bpc_lsx
    vldi          vr21,   0x440   // 64
    vxor.v        vr19,   vr19,   vr19
    addi.d        t8,     a0,     0
    clz.w         t0,     a4
    li.w          t1,     24
    sub.w         t0,     t0,      t1
    la.local      t1,     .MASK_LSX_JRTABLE
    alsl.d        t0,     t0,      t1,    1
    ld.h          t2,     t0,      0
    add.d         t1,     t1,      t2
    jirl          $r0,    t1,      0

    .align   3
.MASK_LSX_JRTABLE:
    .hword .MASK_W128_LSX - .MASK_LSX_JRTABLE
    .hword .MASK_W64_LSX  - .MASK_LSX_JRTABLE
    .hword .MASK_W32_LSX  - .MASK_LSX_JRTABLE
    .hword .MASK_W16_LSX  - .MASK_LSX_JRTABLE
    .hword .MASK_W8_LSX   - .MASK_LSX_JRTABLE
    .hword .MASK_W4_LSX   - .MASK_LSX_JRTABLE

.MASK_W4_LSX:
    vld           vr0,     a2,     0
    vld           vr1,     a3,     0
    fld.d         f22,     a6,     0

    vilvl.b       vr2,    vr19,   vr22
    vsub.h        vr3,    vr21,   vr2

    vmulwev.w.h   vr4,    vr0,    vr2
    vmulwod.w.h   vr5,    vr0,    vr2
    vmaddwev.w.h  vr4,    vr1,    vr3
    vmaddwod.w.h  vr5,    vr1,    vr3
    vssrarni.hu.w vr5,    vr4,    mask_sh
    vssrlrni.bu.h vr1,    vr5,    0
    vpickod.w     vr4,    vr2,    vr1
    vilvl.b       vr0,    vr4,    vr1
    fst.s         f0,     a0,     0
    add.d         a0,     a0,     a1
    vstelm.w      vr0,    a0,     0,    1
    addi.d        a2,     a2,     16
    addi.d        a3,     a3,     16
    addi.d        a6,     a6,     8
    add.d         a0,     a0,     a1
    addi.w        a5,     a5,     -2
    blt           zero,   a5,     .MASK_W4_LSX
    b             .MASK_END_LSX
.MASK_W8_LSX:
    vld           vr0,    a2,     0
    vld           vr10,   a2,     16
    vld           vr1,    a3,     0
    vld           vr11,   a3,     16
    vld           vr22,   a6,     0

    vilvl.b       vr2,    vr19,   vr22
    vilvh.b       vr12,   vr19,   vr22
    vsub.h        vr3,    vr21,   vr2
    vsub.h        vr13,   vr21,   vr12

    vmulwev.w.h   vr4,    vr0,    vr2
    vmulwod.w.h   vr5,    vr0,    vr2
    vmulwev.w.h   vr14,   vr10,   vr12
    vmulwod.w.h   vr15,   vr10,   vr12
    vmaddwev.w.h  vr4,    vr1,    vr3
    vmaddwod.w.h  vr5,    vr1,    vr3
    vmaddwev.w.h  vr14,   vr11,   vr13
    vmaddwod.w.h  vr15,   vr11,   vr13
    vssrarni.hu.w vr14,   vr4,    mask_sh
    vssrarni.hu.w vr15,   vr5,    mask_sh
    vssrlrni.bu.h vr15,   vr14,   0
    vshuf4i.w     vr6,    vr15,   0x4E
    vilvl.b       vr0,    vr6,    vr15
    fst.d         f0,     a0,     0
    add.d         a0,     a0,     a1
    vstelm.d      vr0,    a0,     0,   1
    addi.d        a2,     a2,     32
    addi.d        a3,     a3,     32
    addi.d        a6,     a6,     16
    add.d         a0,     a0,     a1
    addi.w        a5,     a5,     -2
    blt           zero,   a5,     .MASK_W8_LSX
    b             .MASK_END_LSX

.MASK_W16_LSX:
    vld           vr0,    a2,     0
    vld           vr10,   a2,     16
    vld           vr1,    a3,     0
    vld           vr11,   a3,     16
    vld           vr22,   a6,     0

    vilvl.b       vr2,    vr19,   vr22
    vilvh.b       vr12,   vr19,   vr22
    vsub.h        vr3,    vr21,   vr2
    vsub.h        vr13,   vr21,   vr12

    vmulwev.w.h   vr4,    vr0,    vr2
    vmulwod.w.h   vr5,    vr0,    vr2
    vmulwev.w.h   vr14,   vr10,   vr12
    vmulwod.w.h   vr15,   vr10,   vr12
    vmaddwev.w.h  vr4,    vr1,    vr3
    vmaddwod.w.h  vr5,    vr1,    vr3
    vmaddwev.w.h  vr14,   vr11,   vr13
    vmaddwod.w.h  vr15,   vr11,   vr13
    vssrarni.hu.w vr14,   vr4,    mask_sh
    vssrarni.hu.w vr15,   vr5,    mask_sh
    vssrlrni.bu.h vr15,   vr14,   0
    vshuf4i.w     vr6,    vr15,   0x4E
    vilvl.b       vr0,    vr6,    vr15
    vst           vr0,    a0,     0
    addi.d        a2,     a2,     32
    addi.d        a3,     a3,     32
    addi.d        a6,     a6,     16
    add.d         a0,     a0,     a1
    addi.w        a5,     a5,     -1
    blt           zero,   a5,     .MASK_W16_LSX
    b             .MASK_END_LSX
.MASK_W32_LSX:
.rept 2
    vld           vr0,    a2,     0
    vld           vr10,   a2,     16
    vld           vr1,    a3,     0
    vld           vr11,   a3,     16
    vld           vr22,   a6,     0
    vilvl.b       vr2,    vr19,   vr22
    vilvh.b       vr12,   vr19,   vr22
    vsub.h        vr3,    vr21,   vr2
    vsub.h        vr13,   vr21,   vr12
    vmulwev.w.h   vr4,    vr0,    vr2
    vmulwod.w.h   vr5,    vr0,    vr2
    vmulwev.w.h   vr14,   vr10,   vr12
    vmulwod.w.h   vr15,   vr10,   vr12
    vmaddwev.w.h  vr4,    vr1,    vr3
    vmaddwod.w.h  vr5,    vr1,    vr3
    vmaddwev.w.h  vr14,   vr11,   vr13
    vmaddwod.w.h  vr15,   vr11,   vr13
    vssrarni.hu.w vr14,   vr4,    mask_sh
    vssrarni.hu.w vr15,   vr5,    mask_sh
    vssrlrni.bu.h vr15,   vr14,   0
    vshuf4i.w     vr6,    vr15,   0x4E
    vilvl.b       vr0,    vr6,    vr15
    vst           vr0,    a0,     0
    addi.d        a2,     a2,     32
    addi.d        a3,     a3,     32
    addi.d        a6,     a6,     16
    addi.d        a0,     a0,     16
.endr
    add.d         t8,     t8,     a1
    add.d         a0,     t8,     zero
    addi.w        a5,     a5,     -1
    blt           zero,   a5,     .MASK_W32_LSX
    b             .MASK_END_LSX
.MASK_W64_LSX:
.rept 4
    vld           vr0,    a2,     0
    vld           vr10,   a2,     16
    vld           vr1,    a3,     0
    vld           vr11,   a3,     16
    vld           vr22,   a6,     0
    vilvl.b       vr2,    vr19,   vr22
    vilvh.b       vr12,   vr19,   vr22
    vsub.h        vr3,    vr21,   vr2
    vsub.h        vr13,   vr21,   vr12
    vmulwev.w.h   vr4,    vr0,    vr2
    vmulwod.w.h   vr5,    vr0,    vr2
    vmulwev.w.h   vr14,   vr10,   vr12
    vmulwod.w.h   vr15,   vr10,   vr12
    vmaddwev.w.h  vr4,    vr1,    vr3
    vmaddwod.w.h  vr5,    vr1,    vr3
    vmaddwev.w.h  vr14,   vr11,   vr13
    vmaddwod.w.h  vr15,   vr11,   vr13
    vssrarni.hu.w vr14,   vr4,    mask_sh
    vssrarni.hu.w vr15,   vr5,    mask_sh
    vssrlrni.bu.h vr15,   vr14,   0
    vshuf4i.w     vr6,    vr15,   0x4E
    vilvl.b       vr0,    vr6,    vr15
    vst           vr0,    a0,     0
    addi.d        a2,     a2,     32
    addi.d        a3,     a3,     32
    addi.d        a6,     a6,     16
    addi.d        a0,     a0,     16
.endr
    add.d         t8,     t8,     a1
    add.d         a0,     t8,     zero
    addi.w        a5,     a5,     -1
    blt           zero,   a5,     .MASK_W64_LSX
    b             .MASK_END_LSX
.MASK_W128_LSX:
.rept 8
    vld           vr0,    a2,     0
    vld           vr10,   a2,     16
    vld           vr1,    a3,     0
    vld           vr11,   a3,     16
    vld           vr22,   a6,     0
    vilvl.b       vr2,    vr19,   vr22
    vilvh.b       vr12,   vr19,   vr22
    vsub.h        vr3,    vr21,   vr2
    vsub.h        vr13,   vr21,   vr12
    vmulwev.w.h   vr4,    vr0,    vr2
    vmulwod.w.h   vr5,    vr0,    vr2
    vmulwev.w.h   vr14,   vr10,   vr12
    vmulwod.w.h   vr15,   vr10,   vr12
    vmaddwev.w.h  vr4,    vr1,    vr3
    vmaddwod.w.h  vr5,    vr1,    vr3
    vmaddwev.w.h  vr14,   vr11,   vr13
    vmaddwod.w.h  vr15,   vr11,   vr13
    vssrarni.hu.w vr14,   vr4,    mask_sh
    vssrarni.hu.w vr15,   vr5,    mask_sh
    vssrlrni.bu.h vr15,   vr14,   0
    vshuf4i.w     vr6,    vr15,   0x4E
    vilvl.b       vr0,    vr6,    vr15
    vst           vr0,    a0,     0
    addi.d        a2,     a2,     32
    addi.d        a3,     a3,     32
    addi.d        a6,     a6,     16
    addi.d        a0,     a0,     16
.endr
    add.d         t8,     t8,     a1
    add.d         a0,     t8,     zero
    addi.w        a5,     a5,     -1
    blt           zero,   a5,     .MASK_W128_LSX
.MASK_END_LSX:
endfunc

function mask_8bpc_lasx
    xvldi         xr21,   0x440   // 64
    xvxor.v       xr19,   xr19,   xr19
    addi.d        t8,     a0,     0
    clz.w         t0,     a4
    li.w          t1,     24
    sub.w         t0,     t0,      t1
    la.local      t1,     .MASK_LASX_JRTABLE
    alsl.d        t0,     t0,      t1,    1
    ld.h          t2,     t0,      0
    add.d         t1,     t1,      t2
    jirl          $r0,    t1,      0

    .align   3
.MASK_LASX_JRTABLE:
    .hword .MASK_W128_LASX - .MASK_LASX_JRTABLE
    .hword .MASK_W64_LASX  - .MASK_LASX_JRTABLE
    .hword .MASK_W32_LASX  - .MASK_LASX_JRTABLE
    .hword .MASK_W16_LASX  - .MASK_LASX_JRTABLE
    .hword .MASK_W8_LASX   - .MASK_LASX_JRTABLE
    .hword .MASK_W4_LASX   - .MASK_LASX_JRTABLE

.MASK_W4_LASX:
    vld            vr0,    a2,     0
    vld            vr1,    a3,     0
    fld.d          f22,    a6,     0

    vilvl.h        vr4,    vr1,    vr0
    vilvh.h        vr14,   vr1,    vr0
    vilvl.b        vr2,    vr19,   vr22
    vsub.h         vr3,    vr21,   vr2
    xvpermi.q      xr14,   xr4,    0x20
    vilvl.h        vr5,    vr3,    vr2
    vilvh.h        vr15,   vr3,    vr2
    xvpermi.q      xr15,   xr5,    0x20
    xvmulwev.w.h   xr0,    xr14,   xr15
    xvmaddwod.w.h  xr0,    xr14,   xr15
    xvssrarni.hu.w xr1,    xr0,    mask_sh
    xvssrlni.bu.h  xr2,    xr1,    0
    fst.s          f2,     a0,     0
    add.d          a0,     a0,     a1
    xvstelm.w      xr2,    a0,     0,    4

    addi.d         a2,     a2,     16
    addi.d         a3,     a3,     16
    addi.d         a6,     a6,     8
    add.d          a0,     a0,     a1
    addi.w         a5,     a5,     -2
    blt            zero,   a5,     .MASK_W4_LASX
    b              .MASK_END_LASX

.MASK_W8_LASX:
    xvld           xr0,    a2,      0
    xvld           xr1,    a3,      0
    vld            vr22,   a6,      0

    vext2xv.hu.bu  xr2,    xr22
    xvsub.h        xr3,    xr21,    xr2
    xvmulwev.w.h   xr4,    xr0,     xr2
    xvmulwod.w.h   xr5,    xr0,     xr2
    xvmaddwev.w.h  xr4,    xr1,     xr3
    xvmaddwod.w.h  xr5,    xr1,     xr3
    xvssrarni.hu.w xr5,    xr4,     mask_sh
    xvssrlni.bu.h  xr1,    xr5,     0
    xvpickod.w     xr4,    xr2,     xr1
    xvilvl.b       xr0,    xr4,     xr1
    fst.d          f0,     a0,      0
    add.d          a0,     a0,      a1
    xvstelm.d      xr0,    a0,      0,    2

    addi.d         a2,     a2,      32
    addi.d         a3,     a3,      32
    addi.d         a6,     a6,      16
    add.d          a0,     a0,      a1
    addi.w         a5,     a5,      -2
    blt            zero,   a5,      .MASK_W8_LASX
    b              .MASK_END_LASX

.MASK_W16_LASX:
    xvld           xr0,    a2,      0
    xvld           xr1,    a3,      0
    vld            vr22,   a6,      0

    vext2xv.hu.bu  xr2,    xr22
    xvsub.h        xr3,    xr21,    xr2
    xvmulwev.w.h   xr4,    xr0,     xr2
    xvmulwod.w.h   xr5,    xr0,     xr2
    xvmaddwev.w.h  xr4,    xr1,     xr3
    xvmaddwod.w.h  xr5,    xr1,     xr3
    xvssrarni.hu.w xr5,    xr4,     mask_sh
    xvssrlni.bu.h  xr1,    xr5,     0
    xvpickod.w     xr4,    xr2,    xr1
    xvilvl.b       xr0,    xr4,    xr1
    xvpermi.d      xr1,    xr0,     0xD8
    vst            vr1,    a0,      0

    addi.d         a2,     a2,      32
    addi.d         a3,     a3,      32
    addi.d         a6,     a6,      16
    add.d          a0,     a0,      a1
    addi.w         a5,     a5,      -1
    blt            zero,   a5,      .MASK_W16_LASX
    b              .MASK_END_LASX
.MASK_W32_LASX:
    xvld           xr0,    a2,      0
    xvld           xr10,   a2,      32
    xvld           xr1,    a3,      0
    xvld           xr11,   a3,      32
    xvld           xr22,   a6,      0
    vext2xv.hu.bu  xr2,    xr22
    xvpermi.q      xr4,    xr22,    0x01
    vext2xv.hu.bu  xr12,   xr4
    xvsub.h        xr3,    xr21,    xr2
    xvsub.h        xr13,   xr21,    xr12

    xvmulwev.w.h   xr4,    xr0,     xr2
    xvmulwod.w.h   xr5,    xr0,     xr2
    xvmulwev.w.h   xr14,   xr10,    xr12
    xvmulwod.w.h   xr15,   xr10,    xr12
    xvmaddwev.w.h  xr4,    xr1,     xr3
    xvmaddwod.w.h  xr5,    xr1,     xr3
    xvmaddwev.w.h  xr14,   xr11,    xr13
    xvmaddwod.w.h  xr15,   xr11,    xr13
    xvssrarni.hu.w xr14,   xr4,     mask_sh
    xvssrarni.hu.w xr15,   xr5,     mask_sh
    xvssrlni.bu.h  xr15,   xr14,    0
    xvshuf4i.w     xr6,    xr15,    0x4E
    xvilvl.b       xr1,    xr6,     xr15
    xvpermi.d      xr0,    xr1,     0xD8
    xvst           xr0,    a0,      0

    addi.d         a2,     a2,      64
    addi.d         a3,     a3,      64
    addi.d         a6,     a6,      32
    add.d          a0,     a0,      a1
    addi.w         a5,     a5,      -1
    blt            zero,   a5,      .MASK_W32_LASX
    b              .MASK_END_LASX

.MASK_W64_LASX:
.rept 2
    xvld           xr0,    a2,      0
    xvld           xr10,   a2,      32
    xvld           xr1,    a3,      0
    xvld           xr11,   a3,      32
    xvld           xr22,   a6,      0
    vext2xv.hu.bu  xr2,    xr22
    xvpermi.q      xr4,    xr22,    0x01
    vext2xv.hu.bu  xr12,   xr4
    xvsub.h        xr3,    xr21,    xr2
    xvsub.h        xr13,   xr21,    xr12

    xvmulwev.w.h   xr4,    xr0,     xr2
    xvmulwod.w.h   xr5,    xr0,     xr2
    xvmulwev.w.h   xr14,   xr10,    xr12
    xvmulwod.w.h   xr15,   xr10,    xr12
    xvmaddwev.w.h  xr4,    xr1,     xr3
    xvmaddwod.w.h  xr5,    xr1,     xr3
    xvmaddwev.w.h  xr14,   xr11,    xr13
    xvmaddwod.w.h  xr15,   xr11,    xr13
    xvssrarni.hu.w xr14,   xr4,     mask_sh
    xvssrarni.hu.w xr15,   xr5,     mask_sh
    xvssrlni.bu.h  xr15,   xr14,    0
    xvshuf4i.w     xr6,    xr15,    0x4E
    xvilvl.b       xr1,    xr6,     xr15
    xvpermi.d      xr0,    xr1,     0xD8
    xvst           xr0,    a0,      0
    addi.d         a2,     a2,      64
    addi.d         a3,     a3,      64
    addi.d         a6,     a6,      32
    addi.d         a0,     a0,      32
.endr
    add.d          t8,     t8,     a1
    add.d          a0,     t8,     zero
    addi.w         a5,     a5,      -1
    blt            zero,   a5,      .MASK_W64_LASX
    b              .MASK_END_LASX

.MASK_W128_LASX:
.rept 4
    xvld           xr0,    a2,      0
    xvld           xr10,   a2,      32
    xvld           xr1,    a3,      0
    xvld           xr11,   a3,      32
    xvld           xr22,   a6,      0
    vext2xv.hu.bu  xr2,    xr22
    xvpermi.q      xr4,    xr22,    0x01
    vext2xv.hu.bu  xr12,   xr4
    xvsub.h        xr3,    xr21,    xr2
    xvsub.h        xr13,   xr21,    xr12

    xvmulwev.w.h   xr4,    xr0,     xr2
    xvmulwod.w.h   xr5,    xr0,     xr2
    xvmulwev.w.h   xr14,   xr10,    xr12
    xvmulwod.w.h   xr15,   xr10,    xr12
    xvmaddwev.w.h  xr4,    xr1,     xr3
    xvmaddwod.w.h  xr5,    xr1,     xr3
    xvmaddwev.w.h  xr14,   xr11,    xr13
    xvmaddwod.w.h  xr15,   xr11,    xr13
    xvssrarni.hu.w xr14,   xr4,     mask_sh
    xvssrarni.hu.w xr15,   xr5,     mask_sh
    xvssrlni.bu.h  xr15,   xr14,    0
    xvshuf4i.w     xr6,    xr15,    0x4E
    xvilvl.b       xr1,    xr6,     xr15
    xvpermi.d      xr0,    xr1,     0xD8
    xvst           xr0,    a0,      0

    addi.d         a2,     a2,      64
    addi.d         a3,     a3,      64
    addi.d         a6,     a6,      32
    addi.d         a0,     a0,      32
.endr
    add.d          t8,     t8,     a1
    add.d          a0,     t8,     zero
    addi.w         a5,     a5,      -1
    blt            zero,   a5,      .MASK_W128_LASX
.MASK_END_LASX:
endfunc

/*
static void w_mask_c(pixel *dst, const ptrdiff_t dst_stride,
                     const int16_t *tmp1, const int16_t *tmp2, const int w, int h,
                     uint8_t *mask, const int sign,
                     const int ss_hor, const int ss_ver HIGHBD_DECL_SUFFIX)
*/
function w_mask_420_8bpc_lsx
    addi.d        sp,      sp,    -24
    fst.d         f24,     sp,    0
    fst.d         f25,     sp,    8
    fst.d         f26,     sp,    16
    vldi          vr20,    0x440
    vreplgr2vr.h  vr21,    a7
    vldi          vr22,    0x426

    clz.w         t0,      a4
    li.w          t1,      24
    sub.w         t0,      t0,      t1
    la.local      t1,      .WMASK420_LSX_JRTABLE
    alsl.d        t0,      t0,      t1,    1
    ld.h          t8,      t0,      0
    add.d         t1,      t1,      t8
    jirl          $r0,     t1,      0

    .align   3
.WMASK420_LSX_JRTABLE:
    .hword .WMASK420_W128_LSX - .WMASK420_LSX_JRTABLE
    .hword .WMASK420_W64_LSX  - .WMASK420_LSX_JRTABLE
    .hword .WMASK420_W32_LSX  - .WMASK420_LSX_JRTABLE
    .hword .WMASK420_W16_LSX  - .WMASK420_LSX_JRTABLE
    .hword .WMASK420_W8_LSX   - .WMASK420_LSX_JRTABLE
    .hword .WMASK420_W4_LSX   - .WMASK420_LSX_JRTABLE

.WMASK420_W4_LSX:
    vld           vr0,     a2,       0
    vld           vr1,     a2,       16
    vld           vr2,     a3,       0
    vld           vr3,     a3,       16
    addi.w        a5,      a5,       -4

    vabsd.h       vr4,     vr0,      vr2
    vabsd.h       vr5,     vr1,      vr3
    vaddi.hu      vr4,     vr4,      8
    vaddi.hu      vr5,     vr5,      8
    vsrli.h       vr4,     vr4,      8
    vsrli.h       vr5,     vr5,      8
    vadd.h        vr4,     vr4,      vr22
    vadd.h        vr5,     vr5,      vr22
    vmin.hu       vr6,     vr4,      vr20
    vmin.hu       vr7,     vr5,      vr20
    vsub.h        vr8,     vr20,     vr6
    vsub.h        vr9,     vr20,     vr7
    vmulwev.w.h   vr4,     vr6,      vr0
    vmulwod.w.h   vr5,     vr6,      vr0
    vmulwev.w.h   vr10,    vr7,      vr1
    vmulwod.w.h   vr11,    vr7,      vr1
    vmaddwev.w.h  vr4,     vr8,      vr2
    vmaddwod.w.h  vr5,     vr8,      vr2
    vmaddwev.w.h  vr10,    vr9,      vr3
    vmaddwod.w.h  vr11,    vr9,      vr3
    vilvl.w       vr0,     vr5,      vr4
    vilvh.w       vr1,     vr5,      vr4
    vilvl.w       vr2,     vr11,     vr10
    vilvh.w       vr3,     vr11,     vr10
    vssrarni.hu.w vr1,     vr0,      10
    vssrarni.hu.w vr3,     vr2,      10
    vssrlni.bu.h  vr3,     vr1,      0
    vstelm.w      vr3,     a0,       0,    0
    add.d         a0,      a0,       a1
    vstelm.w      vr3,     a0,       0,    1
    add.d         a0,      a0,       a1
    vstelm.w      vr3,     a0,       0,    2
    add.d         a0,      a0,       a1
    vstelm.w      vr3,     a0,       0,    3
    add.d         a0,      a0,       a1
    vpickev.h     vr0,     vr7,      vr6
    vpickod.h     vr1,     vr7,      vr6
    vadd.h        vr0,     vr0,      vr1
    vshuf4i.h     vr0,     vr0,      0xd8
    vhaddw.w.h    vr2,     vr0,      vr0
    vpickev.h     vr2,     vr2,      vr2
    vsub.h        vr2,     vr2,      vr21
    vaddi.hu      vr2,     vr2,      2
    vssrani.bu.h  vr2,     vr2,      2
    vstelm.w      vr2,     a6,       0,    0

    addi.d        a2,      a2,       32
    addi.d        a3,      a3,       32
    addi.d        a6,      a6,       4
    blt           zero,    a5,       .WMASK420_W4_LSX
    b             .END_W420

.WMASK420_W8_LSX:
    vld           vr0,     a2,       0
    vld           vr1,     a2,       16
    vld           vr2,     a3,       0
    vld           vr3,     a3,       16
    addi.w        a5,      a5,       -2

    vabsd.h       vr4,     vr0,      vr2
    vabsd.h       vr5,     vr1,      vr3
    vaddi.hu      vr4,     vr4,      8
    vaddi.hu      vr5,     vr5,      8
    vsrli.h       vr4,     vr4,      8
    vsrli.h       vr5,     vr5,      8
    vadd.h        vr4,     vr4,      vr22
    vadd.h        vr5,     vr5,      vr22
    vmin.hu       vr6,     vr4,      vr20
    vmin.hu       vr7,     vr5,      vr20
    vsub.h        vr8,     vr20,     vr6
    vsub.h        vr9,     vr20,     vr7
    vmulwev.w.h   vr4,     vr6,      vr0
    vmulwod.w.h   vr5,     vr6,      vr0
    vmulwev.w.h   vr10,    vr7,      vr1
    vmulwod.w.h   vr11,    vr7,      vr1
    vmaddwev.w.h  vr4,     vr8,      vr2
    vmaddwod.w.h  vr5,     vr8,      vr2
    vmaddwev.w.h  vr10,    vr9,      vr3
    vmaddwod.w.h  vr11,    vr9,      vr3
    vssrarni.hu.w vr10,    vr4,      10
    vssrarni.hu.w vr11,    vr5,      10
    vssrlni.bu.h  vr11,    vr10,     0
    vshuf4i.w     vr0,     vr11,     0x4E
    vilvl.b       vr3,     vr0,      vr11
    vstelm.d      vr3,     a0,       0,     0
    add.d         a0,      a0,       a1
    vstelm.d      vr3,     a0,       0,     1
    add.d         a0,      a0,       a1
    vpickev.h     vr0,     vr7,      vr6
    vpickod.h     vr1,     vr7,      vr6
    vadd.h        vr0,     vr0,      vr1
    vilvh.d       vr2,     vr0,      vr0
    vadd.h        vr2,     vr2,      vr0
    vsub.h        vr2,     vr2,      vr21
    vaddi.hu      vr2,     vr2,      2
    vssrani.bu.h  vr2,     vr2,      2
    vstelm.w      vr2,     a6,       0,     0

    addi.d        a2,      a2,       32
    addi.d        a3,      a3,       32
    addi.d        a6,      a6,       4
    blt           zero,    a5,       .WMASK420_W8_LSX
    b             .END_W420

.WMASK420_W16_LSX:
    vld           vr0,     a2,       0
    vld           vr1,     a2,       16
    alsl.d        a2,      a4,       a2,    1
    vld           vr2,     a2,       0
    vld           vr3,     a2,       16
    vld           vr4,     a3,       0
    vld           vr5,     a3,       16
    alsl.d        a3,      a4,       a3,    1
    vld           vr6,     a3,       0
    vld           vr7,     a3,       16

    vabsd.h       vr8,     vr0,      vr4
    vabsd.h       vr9,     vr1,      vr5
    vabsd.h       vr10,    vr2,      vr6
    vabsd.h       vr11,    vr3,      vr7
    vaddi.hu      vr8,     vr8,      8
    vaddi.hu      vr9,     vr9,      8
    vaddi.hu      vr10,    vr10,     8
    vaddi.hu      vr11,    vr11,     8
    vsrli.h       vr8,     vr8,      8
    vsrli.h       vr9,     vr9,      8
    vsrli.h       vr10,    vr10,     8
    vsrli.h       vr11,    vr11,     8
    vadd.h        vr8,     vr8,      vr22
    vadd.h        vr9,     vr9,      vr22
    vadd.h        vr10,    vr10,     vr22
    vadd.h        vr11,    vr11,     vr22
    vmin.hu       vr12,    vr8,      vr20
    vmin.hu       vr13,    vr9,      vr20
    vmin.hu       vr14,    vr10,     vr20
    vmin.hu       vr15,    vr11,     vr20
    vsub.h        vr16,    vr20,     vr12
    vsub.h        vr17,    vr20,     vr13
    vsub.h        vr18,    vr20,     vr14
    vsub.h        vr19,    vr20,     vr15
    vmulwev.w.h   vr8,     vr12,     vr0
    vmulwod.w.h   vr9,     vr12,     vr0
    vmulwev.w.h   vr10,    vr13,     vr1
    vmulwod.w.h   vr11,    vr13,     vr1
    vmulwev.w.h   vr23,    vr14,     vr2
    vmulwod.w.h   vr24,    vr14,     vr2
    vmulwev.w.h   vr25,    vr15,     vr3
    vmulwod.w.h   vr26,    vr15,     vr3
    vmaddwev.w.h  vr8,     vr16,     vr4
    vmaddwod.w.h  vr9,     vr16,     vr4
    vmaddwev.w.h  vr10,    vr17,     vr5
    vmaddwod.w.h  vr11,    vr17,     vr5
    vmaddwev.w.h  vr23,    vr18,     vr6
    vmaddwod.w.h  vr24,    vr18,     vr6
    vmaddwev.w.h  vr25,    vr19,     vr7
    vmaddwod.w.h  vr26,    vr19,     vr7
    vssrarni.hu.w vr10,    vr8,      10
    vssrarni.hu.w vr11,    vr9,      10
    vssrarni.hu.w vr25,    vr23,     10
    vssrarni.hu.w vr26,    vr24,     10
    vssrlni.bu.h  vr11,    vr10,     0
    vssrlni.bu.h  vr26,    vr25,     0
    vshuf4i.w     vr0,     vr11,     0x4E
    vshuf4i.w     vr1,     vr26,     0x4E
    vilvl.b       vr3,     vr0,      vr11
    vilvl.b       vr7,     vr1,      vr26
    vst           vr3,     a0,       0
    vstx          vr7,     a0,       a1
    vpickev.h     vr0,     vr13,     vr12
    vpickod.h     vr1,     vr13,     vr12
    vpickev.h     vr2,     vr15,     vr14
    vpickod.h     vr3,     vr15,     vr14
    vadd.h        vr4,     vr0,      vr1
    vadd.h        vr5,     vr2,      vr3
    vadd.h        vr4,     vr4,      vr5
    vsub.h        vr4,     vr4,      vr21
    vssrarni.bu.h vr4,     vr4,      2
    vstelm.d      vr4,     a6,       0,    0

    alsl.d        a2,      a4,       a2,   1
    alsl.d        a3,      a4,       a3,   1
    alsl.d        a0,      a1,       a0,   1
    addi.d        a6,      a6,       8
    addi.w        a5,      a5,       -2
    blt           zero,    a5,       .WMASK420_W16_LSX
    b    .END_W420

.WMASK420_W32_LSX:
.WMASK420_W64_LSX:
.WMASK420_W128_LSX:

.LOOP_W32_420_LSX:
    add.d         t1,       a2,       zero
    add.d         t2,       a3,       zero
    add.d         t3,       a0,       zero
    add.d         t4,       a6,       zero
    alsl.d        t5,       a4,       t1,     1
    alsl.d        t6,       a4,       t2,     1
    or            t7,       a4,       a4

.W32_420_LSX:
    vld           vr0,      t1,       0
    vld           vr1,      t1,       16
    vld           vr2,      t2,       0
    vld           vr3,      t2,       16
    vld           vr4,      t5,       0
    vld           vr5,      t5,       16
    vld           vr6,      t6,       0
    vld           vr7,      t6,       16
    addi.d        t1,       t1,       32
    addi.d        t2,       t2,       32
    addi.d        t5,       t5,       32
    addi.d        t6,       t6,       32
    addi.w        t7,       t7,       -16
    vabsd.h       vr8,      vr0,      vr2
    vabsd.h       vr9,      vr1,      vr3
    vabsd.h       vr10,     vr4,      vr6
    vabsd.h       vr11,     vr5,      vr7
    vaddi.hu      vr8,      vr8,      8
    vaddi.hu      vr9,      vr9,      8
    vaddi.hu      vr10,     vr10,     8
    vaddi.hu      vr11,     vr11,     8
    vsrli.h       vr8,      vr8,      8
    vsrli.h       vr9,      vr9,      8
    vsrli.h       vr10,     vr10,     8
    vsrli.h       vr11,     vr11,     8
    vadd.h        vr8,      vr8,      vr22
    vadd.h        vr9,      vr9,      vr22
    vadd.h        vr10,     vr10,     vr22
    vadd.h        vr11,     vr11,     vr22
    vmin.hu       vr12,     vr8,      vr20
    vmin.hu       vr13,     vr9,      vr20
    vmin.hu       vr14,     vr10,     vr20
    vmin.hu       vr15,     vr11,     vr20
    vsub.h        vr16,     vr20,     vr12
    vsub.h        vr17,     vr20,     vr13
    vsub.h        vr18,     vr20,     vr14
    vsub.h        vr19,     vr20,     vr15
    vmulwev.w.h   vr8,      vr12,     vr0
    vmulwod.w.h   vr9,      vr12,     vr0
    vmulwev.w.h   vr10,     vr13,     vr1
    vmulwod.w.h   vr11,     vr13,     vr1
    vmulwev.w.h   vr23,     vr14,     vr4
    vmulwod.w.h   vr24,     vr14,     vr4
    vmulwev.w.h   vr25,     vr15,     vr5
    vmulwod.w.h   vr26,     vr15,     vr5
    vmaddwev.w.h  vr8,      vr16,     vr2
    vmaddwod.w.h  vr9,      vr16,     vr2
    vmaddwev.w.h  vr10,     vr17,     vr3
    vmaddwod.w.h  vr11,     vr17,     vr3
    vmaddwev.w.h  vr23,     vr18,     vr6
    vmaddwod.w.h  vr24,     vr18,     vr6
    vmaddwev.w.h  vr25,     vr19,     vr7
    vmaddwod.w.h  vr26,     vr19,     vr7
    vssrarni.hu.w vr10,     vr8,      10
    vssrarni.hu.w vr11,     vr9,      10
    vssrarni.hu.w vr25,     vr23,     10
    vssrarni.hu.w vr26,     vr24,     10
    vssrlni.bu.h  vr11,     vr10,     0
    vssrlni.bu.h  vr26,     vr25,     0
    vshuf4i.w     vr8,      vr11,     0x4E
    vshuf4i.w     vr9,      vr26,     0x4E
    vilvl.b       vr3,      vr8,      vr11
    vilvl.b       vr7,      vr9,      vr26
    vst           vr3,      t3,       0
    vstx          vr7,      a1,       t3
    addi.d        t3,       t3,       16
    vpickev.h     vr8,      vr13,     vr12
    vpickod.h     vr9,      vr13,     vr12
    vpickev.h     vr10,     vr15,     vr14
    vpickod.h     vr11,     vr15,     vr14
    vadd.h        vr8,      vr8,      vr9
    vadd.h        vr10,     vr10,     vr11
    vadd.h        vr12,     vr8,      vr10
    vsub.h        vr12,     vr12,     vr21
    vssrarni.bu.h vr12,     vr12,     2
    vstelm.d      vr12,     t4,       0,     0
    addi.d        t4,       t4,       8
    bne           t7,       zero,     .W32_420_LSX

    alsl.d        a2,       a4,       a2,     2
    alsl.d        a3,       a4,       a3,     2
    alsl.d        a0,       a1,       a0,     1
    srai.w        t8,       a4,       1
    add.d         a6,       a6,       t8
    addi.w        a5,       a5,       -2
    blt           zero,     a5,       .LOOP_W32_420_LSX

.END_W420:
    fld.d            f24,     sp,    0
    fld.d            f25,     sp,    8
    fld.d            f26,     sp,    16
    addi.d           sp,      sp,    24
endfunc

function w_mask_420_8bpc_lasx
    xvldi          xr20,    0x440
    xvreplgr2vr.h  xr21,    a7
    xvldi          xr22,    0x426

    clz.w          t0,      a4
    li.w           t1,      24
    sub.w          t0,      t0,      t1
    la.local       t1,      .WMASK420_LASX_JRTABLE
    alsl.d         t0,      t0,      t1,    1
    ld.h           t8,      t0,      0
    add.d          t1,      t1,      t8
    jirl           $r0,     t1,      0

    .align   3
.WMASK420_LASX_JRTABLE:
    .hword .WMASK420_W128_LASX - .WMASK420_LASX_JRTABLE
    .hword .WMASK420_W64_LASX  - .WMASK420_LASX_JRTABLE
    .hword .WMASK420_W32_LASX  - .WMASK420_LASX_JRTABLE
    .hword .WMASK420_W16_LASX  - .WMASK420_LASX_JRTABLE
    .hword .WMASK420_W8_LASX   - .WMASK420_LASX_JRTABLE
    .hword .WMASK420_W4_LASX   - .WMASK420_LASX_JRTABLE

.WMASK420_W4_LASX:
    xvld           xr0,     a2,     0
    xvld           xr1,     a3,     0
    addi.w         a5,      a5,     -4

    xvabsd.h       xr2,     xr0,    xr1
    xvaddi.hu      xr2,     xr2,    8
    xvsrli.h       xr2,     xr2,    8
    xvadd.h        xr2,     xr2,    xr22
    xvmin.hu       xr3,     xr2,    xr20
    xvsub.h        xr4,     xr20,   xr3
    xvmulwev.w.h   xr5,     xr3,    xr0
    xvmulwod.w.h   xr6,     xr3,    xr0
    xvmaddwev.w.h  xr5,     xr4,    xr1
    xvmaddwod.w.h  xr6,     xr4,    xr1
    xvilvl.w       xr7,     xr6,    xr5
    xvilvh.w       xr8,     xr6,    xr5
    xvssrarni.hu.w xr8,     xr7,    10
    xvssrlni.bu.h  xr9,     xr8,    0
    vstelm.w       vr9,     a0,     0,     0
    add.d          a0,      a0,     a1
    vstelm.w       vr9,     a0,     0,     1
    add.d          a0,      a0,     a1
    xvstelm.w      xr9,     a0,     0,     4
    add.d          a0,      a0,     a1
    xvstelm.w      xr9,     a0,     0,     5
    add.d          a0,      a0,     a1

    xvhaddw.w.h    xr3,     xr3,    xr3
    xvpermi.d      xr4,     xr3,    0xb1
    xvadd.h        xr3,     xr3,    xr4
    xvpickev.h     xr3,     xr3,    xr3
    xvsub.h        xr3,     xr3,    xr21
    xvssrarni.bu.h xr3,     xr3,    2
    vstelm.h       vr3,     a6,     0,     0
    xvstelm.h      xr3,     a6,     2,     8

    addi.d         a2,     a2,      32
    addi.d         a3,     a3,      32
    addi.d         a6,     a6,      4
    blt            zero,   a5,      .WMASK420_W4_LASX
    b              .END_W420_LASX

.WMASK420_W8_LASX:
    xvld           xr0,      a2,     0
    xvld           xr1,      a2,     32
    xvld           xr2,      a3,     0
    xvld           xr3,      a3,     32
    addi.w         a5,       a5,     -4

    xvabsd.h       xr4,      xr0,    xr2
    xvabsd.h       xr5,      xr1,    xr3
    xvaddi.hu      xr4,      xr4,    8
    xvaddi.hu      xr5,      xr5,    8
    xvsrli.h       xr4,      xr4,    8
    xvsrli.h       xr5,      xr5,    8
    xvadd.h        xr4,      xr4,    xr22
    xvadd.h        xr5,      xr5,    xr22
    xvmin.hu       xr6,      xr4,    xr20
    xvmin.hu       xr7,      xr5,    xr20
    xvsub.h        xr8,      xr20,   xr6
    xvsub.h        xr9,      xr20,   xr7
    xvmulwev.w.h   xr10,     xr6,    xr0
    xvmulwod.w.h   xr11,     xr6,    xr0
    xvmulwev.w.h   xr12,     xr7,    xr1
    xvmulwod.w.h   xr13,     xr7,    xr1
    xvmaddwev.w.h  xr10,     xr8,    xr2
    xvmaddwod.w.h  xr11,     xr8,    xr2
    xvmaddwev.w.h  xr12,     xr9,    xr3
    xvmaddwod.w.h  xr13,     xr9,    xr3
    xvssrarni.hu.w xr12,     xr10,   10
    xvssrarni.hu.w xr13,     xr11,   10
    xvssrlni.bu.h  xr13,     xr12,   0
    xvshuf4i.w     xr1,      xr13,   0x4E
    xvilvl.b       xr17,     xr1,    xr13
    vstelm.d       vr17,     a0,     0,     0
    add.d          a0,       a0,     a1
    xvstelm.d      xr17,     a0,     0,     2
    add.d          a0,       a0,     a1
    xvstelm.d      xr17,     a0,     0,     1
    add.d          a0,       a0,     a1
    xvstelm.d      xr17,     a0,     0,     3
    add.d          a0,       a0,     a1

    xvhaddw.w.h    xr6,      xr6,    xr6
    xvhaddw.w.h    xr7,      xr7,    xr7
    xvpickev.h     xr8,      xr7,    xr6
    xvpermi.q      xr9,      xr8,    0x01
    vadd.h         vr8,      vr8,    vr9
    vsub.h         vr8,      vr8,    vr21
    vssrarni.bu.h  vr8,      vr8,    2
    vstelm.d       vr8,      a6,     0,    0
    addi.d         a2,       a2,     64
    addi.d         a3,       a3,     64
    addi.d         a6,       a6,     8
    blt            zero,     a5,     .WMASK420_W8_LASX
    b              .END_W420_LASX

.WMASK420_W16_LASX:
    xvld           xr0,      a2,     0
    xvld           xr1,      a2,     32
    xvld           xr2,      a3,     0
    xvld           xr3,      a3,     32
    addi.w         a5,       a5,     -2

    xvabsd.h       xr4,      xr0,    xr2
    xvabsd.h       xr5,      xr1,    xr3
    xvaddi.hu      xr4,      xr4,    8
    xvaddi.hu      xr5,      xr5,    8
    xvsrli.h       xr4,      xr4,    8
    xvsrli.h       xr5,      xr5,    8
    xvadd.h        xr4,      xr4,    xr22
    xvadd.h        xr5,      xr5,    xr22
    xvmin.hu       xr4,      xr4,    xr20
    xvmin.hu       xr5,      xr5,    xr20
    xvsub.h        xr6,      xr20,   xr4
    xvsub.h        xr7,      xr20,   xr5
    xvmulwev.w.h   xr8,      xr4,    xr0
    xvmulwod.w.h   xr9,      xr4,    xr0
    xvmulwev.w.h   xr10,     xr5,    xr1
    xvmulwod.w.h   xr11,     xr5,    xr1
    xvmaddwev.w.h  xr8,      xr6,    xr2
    xvmaddwod.w.h  xr9,      xr6,    xr2
    xvmaddwev.w.h  xr10,     xr7,    xr3
    xvmaddwod.w.h  xr11,     xr7,    xr3
    xvssrarni.hu.w xr10,     xr8,    10
    xvssrarni.hu.w xr11,     xr9,    10
    xvssrlni.bu.h  xr11,     xr10,   0
    xvshuf4i.w     xr8,      xr11,   0x4E
    xvilvl.b       xr15,     xr8,    xr11
    xvpermi.d      xr16,     xr15,   0xd8
    vst            vr16,     a0,     0
    add.d          a0,       a0,     a1
    xvpermi.q      xr16,     xr16,   0x01
    vst            vr16,     a0,     0
    add.d          a0,       a0,     a1

    xvhaddw.w.h    xr4,      xr4,    xr4
    xvhaddw.w.h    xr5,      xr5,    xr5
    xvadd.h        xr4,      xr5,    xr4
    xvpickev.h     xr6,      xr4,    xr4
    xvpermi.d      xr7,      xr6,    0x08
    vsub.h         vr7,      vr7,    vr21
    vssrarni.bu.h  vr7,      vr7,    2
    vstelm.d       vr7,      a6,     0,    0

    addi.d         a2,       a2,     64
    addi.d         a3,       a3,     64
    addi.d         a6,       a6,     8
    blt            zero,     a5,     .WMASK420_W16_LASX
    b              .END_W420_LASX

.WMASK420_W32_LASX:
.WMASK420_W64_LASX:
.WMASK420_W128_LASX:

.LOOP_W32_420_LASX:
    add.d          t1,       a2,       zero
    add.d          t2,       a3,       zero
    add.d          t3,       a0,       zero
    add.d          t4,       a6,       zero
    alsl.d         t5,       a4,       t1,     1
    alsl.d         t6,       a4,       t2,     1
    or             t7,       a4,       a4
.W32_420_LASX:
    xvld           xr0,      t1,       0
    xvld           xr1,      t2,       0
    xvld           xr2,      t5,       0
    xvld           xr3,      t6,       0
    addi.d         t1,       t1,       32
    addi.d         t2,       t2,       32
    addi.d         t5,       t5,       32
    addi.d         t6,       t6,       32
    addi.w         t7,       t7,       -16
    xvabsd.h       xr4,      xr0,      xr1
    xvabsd.h       xr5,      xr2,      xr3
    xvaddi.hu      xr4,      xr4,      8
    xvaddi.hu      xr5,      xr5,      8
    xvsrli.h       xr4,      xr4,      8
    xvsrli.h       xr5,      xr5,      8
    xvadd.h        xr4,      xr4,      xr22
    xvadd.h        xr5,      xr5,      xr22
    xvmin.hu       xr6,      xr4,      xr20
    xvmin.hu       xr7,      xr5,      xr20
    xvsub.h        xr8,      xr20,     xr6
    xvsub.h        xr9,      xr20,     xr7
    xvmulwev.w.h   xr10,     xr6,      xr0
    xvmulwod.w.h   xr11,     xr6,      xr0
    xvmulwev.w.h   xr12,     xr7,      xr2
    xvmulwod.w.h   xr13,     xr7,      xr2
    xvmaddwev.w.h  xr10,     xr8,      xr1
    xvmaddwod.w.h  xr11,     xr8,      xr1
    xvmaddwev.w.h  xr12,     xr9,      xr3
    xvmaddwod.w.h  xr13,     xr9,      xr3
    xvssrarni.hu.w xr12,     xr10,     10
    xvssrarni.hu.w xr13,     xr11,     10
    xvssrlni.bu.h  xr13,     xr12,     0
    xvshuf4i.w     xr10,     xr13,     0x4E
    xvilvl.b       xr17,     xr10,     xr13
    xvpermi.d      xr18,     xr17,     0x08
    xvpermi.d      xr19,     xr17,     0x0d
    vst            vr18,     t3,       0
    vstx           vr19,     t3,       a1
    addi.d         t3,       t3,       16

    xvhaddw.w.h    xr6,      xr6,      xr6
    xvhaddw.w.h    xr7,      xr7,      xr7
    xvadd.h        xr6,      xr7,      xr6
    xvpickev.h     xr7,      xr6,      xr6
    xvpermi.d      xr8,      xr7,      0x08
    vsub.h         vr9,      vr8,      vr21
    vssrarni.bu.h  vr9,      vr9,      2
    vstelm.d       vr9,      t4,       0,      0
    addi.d         t4,       t4,       8
    bne            t7,       zero,     .W32_420_LASX

    alsl.d         a2,       a4,       a2,     2
    alsl.d         a3,       a4,       a3,     2
    alsl.d         a0,       a1,       a0,     1
    srai.w         t8,       a4,       1
    add.d          a6,       a6,       t8
    addi.w         a5,       a5,       -2
    blt            zero,     a5,       .LOOP_W32_420_LASX

.END_W420_LASX:
endfunc

#undef bpc_sh
#undef bpcw_sh
