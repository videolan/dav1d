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
void inv_txfm_add_wht_wht_4x4_c(pixel *dst, const ptrlowff_t stride,
                                coef *const coeff, const int eob
                                HIGHBD_DECL_SUFFIX)
*/
function inv_txfm_add_wht_wht_4x4_8bpc_lsx
    vld           vr0,       a2,      0
    vld           vr2,       a2,      16

    vreplgr2vr.h  vr20,      zero

    vsrai.h       vr0,       vr0,     2
    vsrai.h       vr2,       vr2,     2

    vst           vr20,      a2,      0

    vpickod.d     vr1,       vr0,     vr0
    vpickod.d     vr3,       vr2,     vr2

    vadd.h        vr4,       vr0,     vr1
    vsub.h        vr5,       vr2,     vr3
    vsub.h        vr6,       vr4,     vr5
    vsrai.h       vr6,       vr6,     1
    vsub.h        vr0,       vr6,     vr3
    vsub.h        vr2,       vr6,     vr1
    vsub.h        vr1,       vr4,     vr0
    vadd.h        vr3,       vr5,     vr2

    vst           vr20,      a2,      16

    vilvl.h       vr4,       vr0,     vr1
    vilvl.h       vr5,       vr3,     vr2
    vilvl.w       vr0,       vr5,     vr4
    vilvh.w       vr2,       vr5,     vr4
    vilvh.d       vr1,       vr0,     vr0
    vilvh.d       vr3,       vr2,     vr2

    vadd.h        vr4,       vr0,     vr1
    vsub.h        vr5,       vr2,     vr3
    vsub.h        vr6,       vr4,     vr5
    vsrai.h       vr6,       vr6,     1
    vsub.h        vr0,       vr6,     vr3
    vsub.h        vr2,       vr6,     vr1
    vsub.h        vr1,       vr4,     vr0
    vadd.h        vr3,       vr5,     vr2

    vld           vr4,       a0,      0
    vldx          vr5,       a0,      a1
    alsl.d        t0,        a1,      a0,    1
    vld           vr6,       t0,      0
    vldx          vr7,       t0,      a1

    vsllwil.hu.bu vr4,       vr4,     0
    vsllwil.hu.bu vr5,       vr5,     0
    vsllwil.hu.bu vr6,       vr6,     0
    vsllwil.hu.bu vr7,       vr7,     0
    vilvl.d       vr1,       vr0,     vr1
    vilvl.d       vr2,       vr3,     vr2
    vilvl.d       vr4,       vr5,     vr4
    vilvl.d       vr6,       vr7,     vr6
    vadd.h        vr1,       vr1,     vr4
    vadd.h        vr2,       vr2,     vr6
    vssrani.bu.h  vr2,       vr1,     0

    vstelm.w      vr2,       a0,      0,     0
    add.d         a0,        a0,      a1
    vstelm.w      vr2,       a0,      0,     1
    add.d         a0,        a0,      a1
    vstelm.w      vr2,       a0,      0,     2
    add.d         a0,        a0,      a1
    vstelm.w      vr2,       a0,      0,     3
endfunc

const idct_coeffs, align=4
    // idct4
    .word          2896, 2896*8, 1567, 3784
    // idct8
    .word          799, 4017, 3406, 2276
    // idct16
    .word          401, 4076, 3166, 2598
    .word          1931, 3612, 3920, 1189
    // idct32
    .word          201, 4091, 3035, 2751
    .word          1751, 3703, 3857, 1380
    .word          995, 3973, 3513, 2106
    .word          2440, 3290, 4052, 601
endconst

.macro vld_x8 src, start, stride, in0, in1, in2, in3, in4, in5, in6, in7
    vld           \in0,     \src,     \start
    vld           \in1,     \src,     \start+(\stride*1)
    vld           \in2,     \src,     \start+(\stride*2)
    vld           \in3,     \src,     \start+(\stride*3)
    vld           \in4,     \src,     \start+(\stride*4)
    vld           \in5,     \src,     \start+(\stride*5)
    vld           \in6,     \src,     \start+(\stride*6)
    vld           \in7,     \src,     \start+(\stride*7)
.endm

.macro vst_x8 src, start, stride, in0, in1, in2, in3, in4, in5, in6, in7
    vst           \in0,     \src,     \start
    vst           \in1,     \src,     \start+(\stride*1)
    vst           \in2,     \src,     \start+(\stride*2)
    vst           \in3,     \src,     \start+(\stride*3)
    vst           \in4,     \src,     \start+(\stride*4)
    vst           \in5,     \src,     \start+(\stride*5)
    vst           \in6,     \src,     \start+(\stride*6)
    vst           \in7,     \src,     \start+(\stride*7)
.endm

.macro vld_x16 src, start, stride, in0, in1, in2, in3, in4, in5, in6, in7, \
               in8, in9, in10, in11, in12, in13, in14, in15

    vld_x8 \src, \start, \stride, \in0, \in1, \in2, \in3, \in4, \in5, \in6, \in7

    vld           \in8,     \src,     \start+(\stride*8)
    vld           \in9,     \src,     \start+(\stride*9)
    vld           \in10,    \src,     \start+(\stride*10)
    vld           \in11,    \src,     \start+(\stride*11)
    vld           \in12,    \src,     \start+(\stride*12)
    vld           \in13,    \src,     \start+(\stride*13)
    vld           \in14,    \src,     \start+(\stride*14)
    vld           \in15,    \src,     \start+(\stride*15)
.endm

.macro vst_x16 src, start, stride, in0, in1, in2, in3, in4, in5, in6, in7, \
               in8, in9, in10, in11, in12, in13, in14, in15

    vst_x8 \src, \start, \stride, \in0, \in1, \in2, \in3, \in4, \in5, \in6, \in7

    vst           \in8,     \src,     \start+(\stride*8)
    vst           \in9,     \src,     \start+(\stride*9)
    vst           \in10,    \src,     \start+(\stride*10)
    vst           \in11,    \src,     \start+(\stride*11)
    vst           \in12,    \src,     \start+(\stride*12)
    vst           \in13,    \src,     \start+(\stride*13)
    vst           \in14,    \src,     \start+(\stride*14)
    vst           \in15,    \src,     \start+(\stride*15)
.endm

.macro DST_ADD_W4 in0, in1, in2, in3, in4, in5
    vilvl.w       vr10,     \in1,     \in0  // 0 1  2  3  4  5  6  7 x ...
    vilvl.w       vr12,     \in3,     \in2  // 8 9 10 11 12 13 14 15 x ...
    vsllwil.hu.bu vr10,     vr10,     0
    vsllwil.hu.bu vr12,     vr12,     0
    vadd.h        vr10,     \in4,     vr10
    vadd.h        vr12,     \in5,     vr12
    vssrani.bu.h  vr12,     vr10,     0
    vstelm.w      vr12,     a0,       0,    0
    add.d         t8,       a0,       a1
    vstelm.w      vr12,     t8,       0,    1
    vstelm.w      vr12,     t2,       0,    2
    add.d         t8,       t2,       a1
    vstelm.w      vr12,     t8,       0,    3
.endm

.macro VLD_DST_ADD_W4 in0, in1
    vld           vr0,      a0,       0
    vldx          vr1,      a0,       a1
    vld           vr2,      t2,       0
    vldx          vr3,      t2,       a1

    DST_ADD_W4    vr0, vr1, vr2, vr3, \in0, \in1
.endm

.macro dct_4x4_core_lsx in0, in1, in2, in3, in4, in5, in6, in7, out0, out1
    vexth.w.h     vr4,      \in0            // in1
    vexth.w.h     vr5,      \in1            // in3
    vmul.w        vr6,      vr4,      \in4
    vmul.w        vr7,      vr4,      \in5
    vmadd.w       vr6,      vr5,      \in5  // t3
    vmsub.w       vr7,      vr5,      \in4  // t2
    vsllwil.w.h   vr4,      \in2,     0     // in0
    vsllwil.w.h   vr5,      \in3,     0     // in2
    vmul.w        vr9,      vr4,      \in6
    vmul.w        vr10,     vr4,      \in7
    vmadd.w       vr9,      vr5,      \in7  // t0
    vmsub.w       vr10,     vr5,      \in6  // t1
    vssrarni.h.w  vr10,     vr9,      12    // t0 t1
    vssrarni.h.w  vr7,      vr6,      12    // t3 t2
    vsadd.h       \out0,    vr10,     vr7   // 0 4  8 12 1 5  9 13  c[0] c[1]
    vssub.h       \out1,    vr10,     vr7   // 3 7 11 15 2 6 10 14  c[3] c[2]
.endm

.macro inv_dct_dct_4x4_lsx
    la.local      t0,       idct_coeffs

    vld           vr0,      a2,       0    // 0 1  2  3  4  5  6  7
    vld           vr1,      a2,       16   // 8 9 10 11 12 13 14 15

    vldrepl.w     vr2,      t0,       8    // 1567
    vldrepl.w     vr3,      t0,       12   // 3784
    vldrepl.w     vr8,      t0,       0    // 2896

    dct_4x4_core_lsx vr0, vr1, vr0, vr1, vr3, vr2, vr8, vr8, vr11, vr12

    vreplgr2vr.h  vr15,     zero
    vshuf4i.d     vr12,     vr12,     0x01 // 2 6 10 14 3 7 11 15
    vst           vr15,     a2,       0
    vst           vr15,     a2,       16

    vilvl.h       vr4,      vr12,     vr11 // 0 2 4 6 8 10 12 14
    vilvh.h       vr5,      vr12,     vr11 // 1 3 5 7 9 11 13 15
    vilvl.h       vr0,      vr5,      vr4  // 0 1  2  3  4  5  6  7
    vilvh.h       vr1,      vr5,      vr4  // 8 9 10 11 12 13 14 15

    dct_4x4_core_lsx vr0, vr1, vr0, vr1, vr3, vr2, vr8, vr8, vr13, vr14
    vsrari.h      vr13,     vr13,     4
    vsrari.h      vr14,     vr14,     4
    vshuf4i.d     vr14,     vr14,     0x01

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W4 vr13, vr14
.endm

.macro identity_4x4_lsx in0, in1, in2, in3, out0
    vsllwil.w.h   vr2,      \in0,    0
    vexth.w.h     vr3,      \in1
    vmul.w        vr4,      vr2,     \in2
    vmul.w        vr5,      vr3,     \in2
    vssrarni.h.w  vr5,      vr4,     12
    vsadd.h       \out0,    vr5,     \in3
.endm

.macro inv_identity_identity_4x4_lsx
    vld           vr0,      a2,       0    // 0 1  2  3  4  5  6  7
    vld           vr1,      a2,       16   // 8 9 10 11 12 13 14 15

    li.w          t0,       1697
    vreplgr2vr.w  vr20,     t0

    identity_4x4_lsx vr0, vr0, vr20, vr0, vr0
    identity_4x4_lsx vr1, vr1, vr20, vr1, vr1
    vreplgr2vr.h  vr15,     zero
    vst           vr15,     a2,      0
    vst           vr15,     a2,      16
    identity_4x4_lsx vr0, vr0, vr20, vr0, vr6
    identity_4x4_lsx vr1, vr1, vr20, vr1, vr7

    vsrari.h      vr6,      vr6,     4
    vsrari.h      vr7,      vr7,     4
    vilvh.d       vr8,      vr6,     vr6
    vilvh.d       vr9,      vr7,     vr7
    vilvl.h       vr4,      vr8,     vr6
    vilvl.h       vr5,      vr9,     vr7
    vilvl.w       vr6,      vr5,     vr4
    vilvh.w       vr7,      vr5,     vr4

    alsl.d        t2,       a1,      a0,   1
    VLD_DST_ADD_W4 vr6, vr7
.endm

const iadst4_coeffs, align=4
    .word          1321, 3803, 2482, 3344
endconst

.macro adst4x4_1d_lsx in0, in1, in2, in3, out0, out1, out2, out3
    vsub.w        vr6,      \in0,   \in2  // in0-in2
    vmul.w        vr7,      \in0,   vr20  // in0*1321
    vmadd.w       vr7,      \in2,   vr21  // in0*1321+in2*3803
    vmadd.w       vr7,      \in3,   vr22  // in0*1321+in2*3803+in3*2482
    vmul.w        vr8,      \in1,   vr23  // in1*3344
    vadd.w        vr6,      vr6,    \in3  // in0-in2+in3
    vmul.w        vr9,      \in0,   vr22  // in0*2482
    vmsub.w       vr9,      \in2,   vr20  // in2*1321
    vmsub.w       vr9,      \in3,   vr21  // in0*2482-in2*1321-in3*3803
    vadd.w        vr5,      vr7,    vr9
    vmul.w        \out2,    vr6,    vr23  // out[2] 8  9  10 11
    vadd.w        \out0,    vr7,    vr8   // out[0] 0  1  2  3
    vadd.w        \out1,    vr9,    vr8   // out[1] 4  5  6  7
    vsub.w        \out3,    vr5,    vr8   // out[3] 12 13 14 15
.endm

.macro inv_adst_dct_4x4_lsx
    vld           vr0,      a2,     0
    vld           vr1,      a2,     16

    la.local      t0,       iadst4_coeffs
    vsllwil.w.h   vr2,      vr0,    0     // in0
    vexth.w.h     vr3,      vr0           // in1
    vsllwil.w.h   vr4,      vr1,    0     // in2
    vexth.w.h     vr5,      vr1           // in3
    vldrepl.w     vr20,     t0,     0     // 1321
    vldrepl.w     vr21,     t0,     4     // 3803
    vldrepl.w     vr22,     t0,     8     // 2482
    vldrepl.w     vr23,     t0,     12    // 3344

    adst4x4_1d_lsx vr2, vr3, vr4, vr5, vr0, vr1, vr2, vr3

    LSX_TRANSPOSE4x4_W vr0, vr1, vr2, vr3, vr11, vr13, vr12, vr14, vr6, vr7
    vssrarni.h.w  vr13,     vr11,    12
    vssrarni.h.w  vr14,     vr12,    12

    vreplgr2vr.h  vr15,     zero
    la.local      t0,       idct_coeffs
    vst           vr15,     a2,      0
    vst           vr15,     a2,      16
    vldrepl.w     vr20,     t0,      8    // 1567
    vldrepl.w     vr21,     t0,      12   // 3784
    vldrepl.w     vr22,     t0,      0    // 2896

    dct_4x4_core_lsx vr13, vr14, vr13, vr14, vr21, vr20, vr22, vr22, vr13, vr14

    vshuf4i.d     vr14,     vr14,    0x01
    vsrari.h      vr13,     vr13,    4
    vsrari.h      vr14,     vr14,    4

    alsl.d        t2,       a1,      a0,   1
    VLD_DST_ADD_W4 vr13, vr14
.endm

.macro inv_adst_adst_4x4_lsx
    vld           vr0,      a2,     0
    vld           vr1,      a2,     16

    la.local      t0,       iadst4_coeffs
    vsllwil.w.h   vr2,      vr0,    0     // in0
    vexth.w.h     vr3,      vr0           // in1
    vsllwil.w.h   vr4,      vr1,    0     // in2
    vexth.w.h     vr5,      vr1           // in3
    vldrepl.w     vr20,     t0,     0     // 1321
    vldrepl.w     vr21,     t0,     4     // 3803
    vldrepl.w     vr22,     t0,     8     // 2482
    vldrepl.w     vr23,     t0,     12    // 3344

    adst4x4_1d_lsx vr2, vr3, vr4, vr5, vr0, vr1, vr2, vr3

    LSX_TRANSPOSE4x4_W vr0, vr1, vr2, vr3, vr11, vr13, vr12, vr14, vr6, vr7

    vsrari.w      vr11,     vr11,    12
    vsrari.w      vr13,     vr13,    12
    vsrari.w      vr12,     vr12,    12
    vsrari.w      vr14,     vr14,    12

    vreplgr2vr.h  vr15,     zero
    vst           vr15,     a2,      0
    vst           vr15,     a2,      16

    adst4x4_1d_lsx vr11, vr13, vr12, vr14, vr11, vr13, vr12, vr14

    vssrarni.h.w  vr13,     vr11,    12
    vssrarni.h.w  vr14,     vr12,    12
    vsrari.h      vr13,     vr13,    4
    vsrari.h      vr14,     vr14,    4

    alsl.d        t2,       a1,      a0,   1
    VLD_DST_ADD_W4 vr13, vr14
.endm

.macro inv_dct_adst_4x4_lsx
    la.local      t0,       idct_coeffs

    vld           vr0,      a2,       0    // 0 1  2  3  4  5  6  7
    vld           vr1,      a2,       16   // 8 9 10 11 12 13 14 15

    vldrepl.w     vr20,     t0,       8    // 1567
    vldrepl.w     vr21,     t0,       12   // 3784
    vldrepl.w     vr22,     t0,       0    // 2896

    dct_4x4_core_lsx  vr0, vr1, vr0, vr1, vr21, vr20, vr22, vr22, vr11, vr12

    vreplgr2vr.h  vr15,     zero
    vst           vr15,     a2,       0
    vst           vr15,     a2,       16

    vshuf4i.d     vr12,     vr12,     0x01 // 3 7 11 15 2 6 10 14

    vilvl.h       vr4,      vr12,     vr11 // 0 2 4 6 8 10 12 14
    vilvh.h       vr5,      vr12,     vr11 // 1 3 5 7 9 11 13 15
    vilvl.h       vr11,     vr5,      vr4  // 0 1  2  3  4  5  6  7
    vilvh.h       vr12,     vr5,      vr4  // 8 9 10 11 12 13 14 15

    vsllwil.w.h   vr2,      vr11,     0     // in0
    vexth.w.h     vr3,      vr11            // in1
    vsllwil.w.h   vr4,      vr12,     0     // in2
    vexth.w.h     vr5,      vr12            // in3

    la.local      t0,       iadst4_coeffs

    vldrepl.w     vr20,     t0,       0     // 1321
    vldrepl.w     vr21,     t0,       4     // 3803
    vldrepl.w     vr22,     t0,       8     // 2482
    vldrepl.w     vr23,     t0,       12    // 3344

    adst4x4_1d_lsx vr2, vr3, vr4, vr5, vr11, vr13, vr12, vr14

    vssrarni.h.w  vr13,     vr11,     12
    vssrarni.h.w  vr14,     vr12,     12
    vsrari.h      vr13,     vr13,     4
    vsrari.h      vr14,     vr14,     4

    alsl.d        t2,       a1,       a0,   1
    VLD_DST_ADD_W4 vr13, vr14
.endm

.macro inv_dct_flipadst_4x4_lsx
    la.local      t0,       idct_coeffs

    vld           vr0,      a2,       0    // 0 1  2  3  4  5  6  7
    vld           vr1,      a2,       16   // 8 9 10 11 12 13 14 15

    vldrepl.w     vr20,     t0,       8    // 1567
    vldrepl.w     vr21,     t0,       12   // 3784
    vldrepl.w     vr22,     t0,       0    // 2896

    dct_4x4_core_lsx  vr0, vr1, vr0, vr1, vr21, vr20, vr22, vr22, vr11, vr12

    vreplgr2vr.h  vr15,     zero
    vst           vr15,     a2,       0
    vst           vr15,     a2,       16

    vshuf4i.d     vr12,     vr12,     0x01 // 3 7 11 15 2 6 10 14

    vilvl.h       vr4,      vr12,     vr11 // 0 2 4 6 8 10 12 14
    vilvh.h       vr5,      vr12,     vr11 // 1 3 5 7 9 11 13 15
    vilvl.h       vr11,     vr5,      vr4  // 0 1  2  3  4  5  6  7
    vilvh.h       vr12,     vr5,      vr4  // 8 9 10 11 12 13 14 15
    vsllwil.w.h   vr2,      vr11,     0    // in0
    vexth.w.h     vr3,      vr11           // in1
    vsllwil.w.h   vr4,      vr12,     0    // in2
    vexth.w.h     vr5,      vr12           // in3

    la.local      t0,       iadst4_coeffs

    vldrepl.w     vr20,     t0,       0     // 1321
    vldrepl.w     vr21,     t0,       4     // 3803
    vldrepl.w     vr22,     t0,       8     // 2482
    vldrepl.w     vr23,     t0,       12    // 3344

    adst4x4_1d_lsx vr2, vr3, vr4, vr5, vr11, vr12, vr13, vr14

    vssrarni.h.w  vr11,     vr12,     12    // 0 1  2  3  4  5  6  7
    vssrarni.h.w  vr13,     vr14,     12    // 8 9 10 11 12 13 14 15
    vsrari.h      vr11,     vr11,     4
    vsrari.h      vr13,     vr13,     4

    alsl.d        t2,       a1,       a0,   1
    VLD_DST_ADD_W4 vr13, vr11
.endm

.macro inv_flipadst_adst_4x4_lsx
    vld           vr0,      a2,       0
    vld           vr1,      a2,       16

    la.local      t0,       iadst4_coeffs
    vsllwil.w.h   vr2,      vr0,      0     // in0
    vexth.w.h     vr3,      vr0             // in1
    vsllwil.w.h   vr4,      vr1,      0     // in2
    vexth.w.h     vr5,      vr1             // in3
    vldrepl.w     vr20,     t0,       0     // 1321
    vldrepl.w     vr21,     t0,       4     // 3803
    vldrepl.w     vr22,     t0,       8     // 2482
    vldrepl.w     vr23,     t0,       12    // 3344

    adst4x4_1d_lsx vr2, vr3, vr4, vr5, vr0, vr1, vr2, vr3

    vsrari.w      vr0,      vr0,      12
    vsrari.w      vr1,      vr1,      12
    vsrari.w      vr2,      vr2,      12
    vsrari.w      vr3,      vr3,      12

    vilvl.w       vr4,      vr0,      vr1
    vilvh.w       vr5,      vr0,      vr1
    vilvl.w       vr6,      vr2,      vr3
    vilvh.w       vr7,      vr2,      vr3
    vilvl.d       vr11,     vr4,      vr6
    vilvh.d       vr12,     vr4,      vr6
    vilvl.d       vr13,     vr5,      vr7
    vilvh.d       vr14,     vr5,      vr7

    vreplgr2vr.h  vr15,     zero
    vst           vr15,     a2,       0
    vst           vr15,     a2,       16

    adst4x4_1d_lsx vr11, vr12, vr13, vr14, vr11, vr13, vr12, vr14

    vssrarni.h.w  vr13,     vr11,     12
    vssrarni.h.w  vr14,     vr12,     12
    vsrari.h      vr13,     vr13,     4
    vsrari.h      vr14,     vr14,     4

    alsl.d        t2,       a1,       a0,   1
    VLD_DST_ADD_W4 vr13, vr14
.endm

.macro inv_adst_flipadst_4x4_lsx
    vld           vr0,      a2,      0
    vld           vr1,      a2,      16

    la.local      t0,       iadst4_coeffs
    vsllwil.w.h   vr2,      vr0,      0     // in0
    vexth.w.h     vr3,      vr0             // in1
    vsllwil.w.h   vr4,      vr1,      0     // in2
    vexth.w.h     vr5,      vr1             // in3
    vldrepl.w     vr20,     t0,       0     // 1321
    vldrepl.w     vr21,     t0,       4     // 3803
    vldrepl.w     vr22,     t0,       8     // 2482
    vldrepl.w     vr23,     t0,       12    // 3344

    adst4x4_1d_lsx vr2, vr3, vr4, vr5, vr0, vr1, vr2, vr3
    LSX_TRANSPOSE4x4_W vr0, vr1, vr2, vr3, vr11, vr13, vr12, vr14, vr6, vr7
    vsrari.w      vr11,     vr11,     12
    vsrari.w      vr12,     vr12,     12
    vsrari.w      vr13,     vr13,     12
    vsrari.w      vr14,     vr14,     12

    vreplgr2vr.h  vr15,     zero
    vst           vr15,     a2,       0
    vst           vr15,     a2,       16

    adst4x4_1d_lsx vr11, vr13, vr12, vr14, vr11, vr12, vr13, vr14

    vssrarni.h.w  vr11,     vr12,     12
    vssrarni.h.w  vr13,     vr14,     12
    vsrari.h      vr11,     vr11,     4
    vsrari.h      vr13,     vr13,     4

    alsl.d        t2,       a1,       a0,   1
    VLD_DST_ADD_W4 vr13, vr11
.endm

.macro inv_flipadst_dct_4x4_lsx
    vld           vr0,      a2,       0
    vld           vr1,      a2,       16

    la.local      t0,       iadst4_coeffs
    vsllwil.w.h   vr2,      vr0,      0     // in0
    vexth.w.h     vr3,      vr0             // in1
    vsllwil.w.h   vr4,      vr1,      0     // in2
    vexth.w.h     vr5,      vr1             // in3
    vldrepl.w     vr20,     t0,       0     // 1321
    vldrepl.w     vr21,     t0,       4     // 3803
    vldrepl.w     vr22,     t0,       8     // 2482
    vldrepl.w     vr23,     t0,       12    // 3344

    adst4x4_1d_lsx vr2, vr3, vr4, vr5, vr0, vr1, vr2, vr3

    vilvl.w       vr4,      vr0,      vr1
    vilvh.w       vr5,      vr0,      vr1
    vilvl.w       vr6,      vr2,      vr3
    vilvh.w       vr7,      vr2,      vr3

    vilvl.d       vr11,     vr4,      vr6
    vilvh.d       vr12,     vr4,      vr6
    vilvl.d       vr13,     vr5,      vr7
    vilvh.d       vr14,     vr5,      vr7

    vssrarni.h.w  vr12,     vr11,     12
    vssrarni.h.w  vr14,     vr13,     12

    vreplgr2vr.h  vr15,     zero
    la.local      t0,       idct_coeffs
    vst           vr15,     a2,       0
    vst           vr15,     a2,       16
    vldrepl.w     vr20,     t0,       8    // 1567
    vldrepl.w     vr21,     t0,       12   // 3784
    vldrepl.w     vr22,     t0,       0    // 2896

    dct_4x4_core_lsx vr12, vr14, vr12, vr14, vr21, vr20, vr22, vr22, vr13, vr14

    vshuf4i.d     vr14,     vr14,     0x01
    vsrari.h      vr13,     vr13,     4
    vsrari.h      vr14,     vr14,     4

    alsl.d        t2,       a1,       a0,   1
    VLD_DST_ADD_W4 vr13, vr14
.endm

.macro inv_flipadst_flipadst_4x4_lsx
    vld           vr0,      a2,       0
    vld           vr1,      a2,       16

    la.local      t0,       iadst4_coeffs
    vsllwil.w.h   vr2,      vr0,      0     // in0
    vexth.w.h     vr3,      vr0             // in1
    vsllwil.w.h   vr4,      vr1,      0     // in2
    vexth.w.h     vr5,      vr1             // in3
    vldrepl.w     vr20,     t0,       0     // 1321
    vldrepl.w     vr21,     t0,       4     // 3803
    vldrepl.w     vr22,     t0,       8     // 2482
    vldrepl.w     vr23,     t0,       12    // 3344

    adst4x4_1d_lsx vr2, vr3, vr4, vr5, vr0, vr1, vr2, vr3

    vilvl.w       vr4,      vr0,      vr1
    vilvh.w       vr5,      vr0,      vr1
    vilvl.w       vr6,      vr2,      vr3
    vilvh.w       vr7,      vr2,      vr3
    vilvl.d       vr11,     vr4,      vr6
    vilvh.d       vr12,     vr4,      vr6
    vilvl.d       vr13,     vr5,      vr7
    vilvh.d       vr14,     vr5,      vr7

    vsrari.w      vr11,     vr11,     12
    vsrari.w      vr12,     vr12,     12
    vsrari.w      vr13,     vr13,     12
    vsrari.w      vr14,     vr14,     12

    vreplgr2vr.h  vr15,     zero
    vst           vr15,     a2,       0
    vst           vr15,     a2,       16

    adst4x4_1d_lsx vr11, vr12, vr13, vr14, vr11, vr12, vr13, vr14

    vssrarni.h.w  vr11,     vr12,     12
    vssrarni.h.w  vr13,     vr14,     12
    vsrari.h      vr11,     vr11,     4
    vsrari.h      vr13,     vr13,     4

    alsl.d        t2,       a1,       a0,   1
    VLD_DST_ADD_W4 vr13, vr11
.endm

.macro inv_dct_identity_4x4_lsx
    la.local      t0,       idct_coeffs

    vld           vr0,      a2,       0
    vld           vr1,      a2,       16

    vldrepl.w     vr2,      t0,       8    // 1567
    vldrepl.w     vr3,      t0,       12   // 3784
    vldrepl.w     vr8,      t0,       0    // 2896

    dct_4x4_core_lsx vr0, vr1, vr0, vr1, vr3, vr2, vr8, vr8, vr11, vr12
    vshuf4i.d     vr12,     vr12,     0x01 // 2 6 10 14 3 7 11 15

    vreplgr2vr.h  vr15,     zero
    li.w          t0,       1697

    vilvl.h       vr4,      vr12,     vr11 // 0 2 4 6 8 10 12 14
    vilvh.h       vr5,      vr12,     vr11 // 1 3 5 7 9 11 13 15
    vilvl.h       vr10,     vr5,      vr4  // 0 1  2  3  4  5  6  7
    vilvh.h       vr12,     vr5,      vr4  // 8 9 10 11 12 13 14 15

    vst           vr15,     a2,       0
    vst           vr15,     a2,       16
    vreplgr2vr.w  vr20,     t0

    identity_4x4_lsx vr10, vr10, vr20, vr10, vr6
    identity_4x4_lsx vr12, vr12, vr20, vr12, vr7
    vsrari.h      vr11,      vr6,     4
    vsrari.h      vr13,      vr7,     4

    alsl.d        t2,       a1,       a0,   1
    VLD_DST_ADD_W4 vr11, vr13
.endm

.macro inv_identity_dct_4x4_lsx
    vld           vr0,      a2,       0
    vld           vr1,      a2,       16

    li.w          t0,       1697
    vreplgr2vr.w  vr20,     t0

    identity_4x4_lsx vr0, vr0, vr20, vr0, vr0
    identity_4x4_lsx vr1, vr1, vr20, vr1, vr1

    vreplgr2vr.h  vr15,     zero

    vilvl.h       vr4,      vr1,      vr0  // 0 2 4 6 8 10 12 14
    vilvh.h       vr5,      vr1,      vr0  // 1 3 5 7 9 11 13 15
    vilvl.h       vr13,     vr5,      vr4  // 0 1  2  3  4  5  6  7
    vilvh.h       vr14,     vr5,      vr4  // 8 9 10 11 12 13 14 15

    vst           vr15,     a2,       0
    vst           vr15,     a2,       16

    la.local      t0,       idct_coeffs

    vldrepl.w     vr20,     t0,       8    // 1567
    vldrepl.w     vr21,     t0,       12   // 3784
    vldrepl.w     vr22,     t0,       0    // 2896

    dct_4x4_core_lsx vr13, vr14, vr13, vr14, vr21, vr20, vr22, vr22, vr13, vr14

    vshuf4i.d     vr14,     vr14,     0x01
    vsrari.h      vr13,     vr13,     4
    vsrari.h      vr14,     vr14,     4

    alsl.d        t2,       a1,       a0,   1
    VLD_DST_ADD_W4 vr13, vr14
.endm

.macro inv_flipadst_identity_4x4_lsx
    vld           vr0,      a2,       0
    vld           vr1,      a2,       16

    la.local      t0,       iadst4_coeffs
    vsllwil.w.h   vr2,      vr0,      0     // in0
    vexth.w.h     vr3,      vr0             // in1
    vsllwil.w.h   vr4,      vr1,      0     // in2
    vexth.w.h     vr5,      vr1             // in3
    vldrepl.w     vr20,     t0,       0     // 1321
    vldrepl.w     vr21,     t0,       4     // 3803
    vldrepl.w     vr22,     t0,       8     // 2482
    vldrepl.w     vr23,     t0,       12    // 3344

    adst4x4_1d_lsx vr2, vr3, vr4, vr5, vr10, vr11, vr12, vr13

    vssrarni.h.w  vr12,     vr13,     12
    vssrarni.h.w  vr10,     vr11,     12

    vilvl.h       vr4,      vr10,     vr12  // 0 2 4 6 8 10 12 14
    vilvh.h       vr5,      vr10,     vr12  // 1 3 5 7 9 11 13 15
    vilvl.h       vr11,     vr5,      vr4   // 0 1  2  3  4  5  6  7
    vilvh.h       vr13,     vr5,      vr4   // 8 9 10 11 12 13 14 15

    vreplgr2vr.h  vr15,     zero
    li.w          t0,       1697

    vst           vr15,     a2,       0
    vst           vr15,     a2,       16
    vreplgr2vr.w  vr20,     t0

    identity_4x4_lsx vr11, vr11, vr20, vr11, vr6
    identity_4x4_lsx vr13, vr13, vr20, vr13, vr7
    vsrari.h      vr11,     vr6,     4
    vsrari.h      vr13,     vr7,     4

    alsl.d        t2,       a1,      a0,   1
    VLD_DST_ADD_W4 vr11, vr13
.endm

.macro inv_identity_flipadst_4x4_lsx
    vld           vr0,      a2,       0
    vld           vr1,      a2,       16

    li.w          t0,       1697
    vreplgr2vr.w  vr20,     t0

    identity_4x4_lsx vr0, vr0, vr20, vr0, vr0
    identity_4x4_lsx vr1, vr1, vr20, vr1, vr1

    vilvl.h       vr4,      vr1,      vr0
    vilvh.h       vr5,      vr1,      vr0
    vilvl.h       vr11,     vr5,      vr4
    vilvh.h       vr13,     vr5,      vr4

    vreplgr2vr.h  vr15,     zero
    vst           vr15,     a2,       0
    vst           vr15,     a2,       16

    la.local      t0,       iadst4_coeffs
    vsllwil.w.h   vr2,      vr11,     0   // in0
    vexth.w.h     vr3,      vr11          // in1
    vsllwil.w.h   vr4,      vr13,     0   // in2
    vexth.w.h     vr5,      vr13          // in3
    vldrepl.w     vr20,     t0,       0   // 1321
    vldrepl.w     vr21,     t0,       4   // 3803
    vldrepl.w     vr22,     t0,       8   // 2482
    vldrepl.w     vr23,     t0,       12  // 3344

    adst4x4_1d_lsx vr2, vr3, vr4, vr5, vr0, vr1, vr2, vr3

    vssrarni.h.w  vr0,      vr1,      12  // 8 9 10 11 12 13 14 15
    vssrarni.h.w  vr2,      vr3,      12  // 0 1  2  3  4  5  6  7
    vsrari.h      vr11,     vr0,      4
    vsrari.h      vr13,     vr2,      4

    alsl.d        t2,       a1,       a0,   1
    VLD_DST_ADD_W4 vr13, vr11
.endm

.macro inv_identity_adst_4x4_lsx
    vld           vr0,      a2,       0
    vld           vr1,      a2,       16

    li.w          t0,       1697
    vreplgr2vr.w  vr20,     t0

    identity_4x4_lsx vr0, vr0, vr20, vr0, vr0
    identity_4x4_lsx vr1, vr1, vr20, vr1, vr1

    vilvl.h       vr4,      vr1,      vr0
    vilvh.h       vr5,      vr1,      vr0
    vilvl.h       vr11,     vr5,      vr4
    vilvh.h       vr13,     vr5,      vr4

    vreplgr2vr.h  vr15,     zero
    vst           vr15,     a2,       0
    vst           vr15,     a2,       16

    la.local      t0,       iadst4_coeffs
    vsllwil.w.h   vr2,      vr11,     0     // in0
    vexth.w.h     vr3,      vr11            // in1
    vsllwil.w.h   vr4,      vr13,     0     // in2
    vexth.w.h     vr5,      vr13            // in3
    vldrepl.w     vr20,     t0,       0     // 1321
    vldrepl.w     vr21,     t0,       4     // 3803
    vldrepl.w     vr22,     t0,       8     // 2482
    vldrepl.w     vr23,     t0,       12    // 3344

    adst4x4_1d_lsx vr2, vr3, vr4, vr5, vr0, vr1, vr2, vr3

    vssrarni.h.w  vr1,      vr0,      12
    vssrarni.h.w  vr3,      vr2,      12
    vsrari.h      vr11,     vr1,      4
    vsrari.h      vr13,     vr3,      4

    alsl.d        t2,       a1,       a0,   1
    VLD_DST_ADD_W4 vr11, vr13
.endm

.macro inv_adst_identity_4x4_lsx
    vld           vr0,      a2,       0
    vld           vr1,      a2,       16

    la.local      t0,       iadst4_coeffs
    vsllwil.w.h   vr2,      vr0,      0     // in0
    vexth.w.h     vr3,      vr0             // in1
    vsllwil.w.h   vr4,      vr1,      0     // in2
    vexth.w.h     vr5,      vr1             // in3
    vldrepl.w     vr20,     t0,       0     // 1321
    vldrepl.w     vr21,     t0,       4     // 3803
    vldrepl.w     vr22,     t0,       8     // 2482
    vldrepl.w     vr23,     t0,       12    // 3344

    adst4x4_1d_lsx vr2, vr3, vr4, vr5, vr0, vr1, vr2, vr3

    LSX_TRANSPOSE4x4_W vr0, vr1, vr2, vr3, vr11, vr13, vr12, vr14, vr6, vr7

    vssrarni.h.w  vr13,     vr11,     12
    vssrarni.h.w  vr14,     vr12,     12

    vreplgr2vr.h  vr15,     zero
    li.w          t0,       1697

    vst           vr15,     a2,       0
    vst           vr15,     a2,       16
    vreplgr2vr.w  vr20,     t0

    identity_4x4_lsx vr13, vr13, vr20, vr13, vr6
    identity_4x4_lsx vr14, vr14, vr20, vr14, vr7
    vsrari.h      vr11,     vr6,      4
    vsrari.h      vr13,     vr7,      4

    alsl.d        t2,       a1,       a0,   1
    VLD_DST_ADD_W4 vr11, vr13
.endm

.macro fun4x4 type1, type2
function inv_txfm_add_\type1\()_\type2\()_4x4_8bpc_lsx
.ifc \type1\()_\type2, dct_dct
    bnez          a3,       .LLL

    vldi          vr0,      0x8b5            // 181
    ld.h          t2,       a2,       0      // dc
    st.h          zero,     a2,       0
    vreplgr2vr.w  vr1,      t2
    vldi          vr3,      0x880            // 128
    vmul.w        vr2,      vr0,      vr1
    vld           vr10,     a0,       0
    vsrari.w      vr2,      vr2,      8
    vldx          vr11,     a0,       a1
    vmadd.w       vr3,      vr2,      vr0
    alsl.d        t2,       a1,       a0,    1
    vssrarni.h.w  vr3,      vr3,      12
    vld           vr12,     t2,       0
    vldx          vr13,     t2,       a1

    DST_ADD_W4    vr10, vr11, vr12, vr13, vr3, vr3

    b             .IDST_\type1\()_\type2\()_4X4_END
.LLL:
.endif

    inv_\type1\()_\type2\()_4x4_lsx
.IDST_\type1\()_\type2\()_4X4_END:
endfunc
.endm

fun4x4 dct, dct
fun4x4 identity, identity
fun4x4 adst, dct
fun4x4 dct, adst
fun4x4 adst, adst
fun4x4 dct, flipadst
fun4x4 flipadst, adst
fun4x4 adst, flipadst
fun4x4 flipadst, dct
fun4x4 flipadst, flipadst
fun4x4 dct, identity
fun4x4 identity, dct
fun4x4 flipadst, identity
fun4x4 identity, flipadst
fun4x4 identity, adst
fun4x4 adst, identity

.macro rect2_w4_lsx in0, in1, in2, out0, out1
    vsllwil.w.h   vr22,     \in0,     0
    vexth.w.h     vr23,     \in1
    vmul.w        vr22,     vr22,     \in2
    vmul.w        vr23,     vr23,     \in2
    vsrari.w      \out0,    vr22,     12
    vsrari.w      \out1,    vr23,     12
.endm

.macro dct_8x4_core_lsx1 out0, out1, out2, out3
    // dct4 stride=1<<1
    vmul.w        vr0,      vr6,      vr21
    vmul.w        vr1,      vr6,      vr20
    vmadd.w       vr0,      vr10,     vr20  // t3
    vmsub.w       vr1,      vr10,     vr21  // t2
    vmul.w        vr2,      vr18,     vr22
    vmul.w        vr3,      vr18,     vr22
    vmadd.w       vr2,      vr8,      vr22  // t0
    vmsub.w       vr3,      vr8,      vr22  // t1
    vssrarni.h.w  vr1,      vr0,      12    // t3 t2
    vssrarni.h.w  vr3,      vr2,      12    // t0 t1
    vsadd.h       vr8,      vr3,      vr1   // t0 t1
    vssub.h       vr10,     vr3,      vr1   // t3 t2

    vldrepl.w     vr20,     t0,       16    // 799
    vldrepl.w     vr21,     t0,       20    // 4017
    vldrepl.w     vr22,     t0,       24    // 3406
    vldrepl.w     vr23,     t0,       28    // 2276

    vmul.w        vr0,      vr19,     vr21  // in1 * 4017
    vmul.w        vr1,      vr19,     vr20  // in1 * 799
    vmadd.w       vr0,      vr11,     vr20  // in7 * 799   // t7a
    vmsub.w       vr1,      vr11,     vr21  // in7 * 4017  // t4a
    vmul.w        vr2,      vr9,      vr23  // in5 * 1138
    vmul.w        vr3,      vr9,      vr22  // in5 * 1703
    vmadd.w       vr2,      vr7,      vr22  // in3 * 1703  // t6a
    vmsub.w       vr3,      vr7,      vr23  // in3 * 1138  // t5a
    vssrarni.h.w  vr0,      vr1,      12    // t4a t7a
    vssrarni.h.w  vr2,      vr3,      12    // t5a t6a
    vsadd.h       vr9,      vr0,      vr2   // t4  t7
    vssub.h       vr11,     vr0,      vr2   // t5a t6a

    vldrepl.w     vr22,     t0,       0     // 2896
    vexth.w.h     vr18,     vr11            // t6a
    vsllwil.w.h   vr19,     vr11,     0     // t5a
    vmul.w        vr6,      vr18,     vr22
    vmul.w        vr7,      vr18,     vr22
    vmadd.w       vr6,      vr19,     vr22  // t6
    vmsub.w       vr7,      vr19,     vr22  // t5
    vssrarni.h.w  vr6,      vr7,      12    // t5 t6

    vilvh.d       vr11,     vr6,      vr9   // t7 t6
    vilvl.d       vr9,      vr6,      vr9   // t4 t5

    vsadd.h       \out0,    vr8,      vr11  // c[0] c[1]
    vsadd.h       \out1,    vr10,     vr9   // c[3] c[2]
    vssub.h       \out2,    vr10,     vr9   // c[4] c[5]
    vssub.h       \out3,    vr8,      vr11  // c[7] c[6]
.endm

.macro dct_8x4_core_lsx2 in0, in1, in2, in3, in4, in5, in6, in7, \
                         out0, out1, out2, out3
    vexth.w.h     vr4,      \in0            // in1
    vexth.w.h     vr5,      \in1            // in3
    vmul.w        vr6,      vr4,      \in4
    vmul.w        vr7,      vr4,      \in5
    vmadd.w       vr6,      vr5,      \in5  // t3
    vmsub.w       vr7,      vr5,      \in4  // t2
    vexth.w.h     vr4,      \in2            // in1
    vexth.w.h     vr5,      \in3            // in3
    vmul.w        vr8,      vr4,      \in4
    vmul.w        vr9,      vr4,      \in5
    vmadd.w       vr8,      vr5,      \in5  // t3
    vmsub.w       vr9,      vr5,      \in4  // t2
    vssrarni.h.w  vr8,      vr6,      12    // t3
    vssrarni.h.w  vr9,      vr7,      12    // t2

    vsllwil.w.h   vr4,      \in0,     0
    vsllwil.w.h   vr5,      \in1,     0
    vmul.w        vr11,     vr4,      \in6
    vmul.w        vr12,     vr4,      \in7
    vmadd.w       vr11,     vr5,      \in7  // t0
    vmsub.w       vr12,     vr5,      \in6  // t1
    vsllwil.w.h   vr4,      \in2,     0
    vsllwil.w.h   vr5,      \in3,     0
    vmul.w        vr13,     vr4,      \in6
    vmul.w        vr14,     vr4,      \in7
    vmadd.w       vr13,     vr5,      \in7  // t0
    vmsub.w       vr14,     vr5,      \in6  // t1
    vssrarni.h.w  vr13,     vr11,     12    // t0
    vssrarni.h.w  vr14,     vr12,     12    // t1

    vsadd.h       \out0,    vr13,     vr8
    vsadd.h       \out1,    vr14,     vr9
    vssub.h       \out2,    vr14,     vr9
    vssub.h       \out3,    vr13,     vr8
.endm

.macro DST_ADD_W8 in0, in1, in2, in3, in4, in5, in6, in7
    vsllwil.hu.bu vr10,     \in0,     0
    vsllwil.hu.bu vr11,     \in1,     0
    vsllwil.hu.bu vr12,     \in2,     0
    vsllwil.hu.bu vr13,     \in3,     0
    vadd.h        vr10,     \in4,     vr10
    vadd.h        vr11,     \in5,     vr11
    vadd.h        vr12,     \in6,     vr12
    vadd.h        vr13,     \in7,     vr13
    vssrani.bu.h  vr11,     vr10,     0
    vssrani.bu.h  vr13,     vr12,     0
    vstelm.d      vr11,     a0,       0,    0
    add.d         t8,       a0,       a1
    vstelm.d      vr11,     t8,       0,    1
    vstelm.d      vr13,     t2,       0,    0
    add.d         t8,       t2,       a1
    vstelm.d      vr13,     t8,       0,    1
.endm

.macro VLD_DST_ADD_W8 in0, in1, in2, in3
    vld           vr0,      a0,       0
    vldx          vr1,      a0,       a1
    vld           vr2,      t2,       0
    vldx          vr3,      t2,       a1

    DST_ADD_W8 vr0, vr1, vr2, vr3, \in0, \in1, \in2, \in3
.endm

function inv_txfm_add_dct_dct_8x4_8bpc_lsx
    bnez          a3,       .NO_HAS_DCONLY_8x4

    ld.h          t2,       a2,       0      // dc
    vldi          vr0,      0x8b5            // 181
    vreplgr2vr.w  vr1,      t2
    vldi          vr5,      0x880            // 128
    vmul.w        vr2,      vr0,      vr1
    st.h          zero,     a2,       0
    vsrari.w      vr2,      vr2,      8
    vld           vr10,     a0,       0
    vmul.w        vr2,      vr2,      vr0
    vldx          vr11,     a0,       a1
    vsrari.w      vr2,      vr2,      8
    alsl.d        t2,       a1,       a0,    1
    vmadd.w       vr5,      vr2,      vr0
    vld           vr12,     t2,       0
    vssrarni.h.w  vr5,      vr5,      12
    vldx          vr13,     t2,       a1

    DST_ADD_W8 vr10, vr11, vr12, vr13, vr5, vr5, vr5, vr5

    b             .DCT_DCT_8X4_END

.NO_HAS_DCONLY_8x4:
    la.local      t0,       idct_coeffs

    vld           vr0,      a2,       0
    vld           vr1,      a2,       16
    vld           vr2,      a2,       32
    vld           vr3,      a2,       48

    vldrepl.w     vr20,     t0,       0     // 2896

    rect2_w4_lsx vr0, vr0, vr20, vr18, vr19
    rect2_w4_lsx vr1, vr1, vr20, vr6, vr7
    rect2_w4_lsx vr2, vr2, vr20, vr8, vr9
    rect2_w4_lsx vr3, vr3, vr20, vr10, vr11

    vldrepl.w     vr20,     t0,       8     // 1567
    vldrepl.w     vr21,     t0,       12    // 3784
    vldrepl.w     vr22,     t0,       0     // 2896

    dct_8x4_core_lsx1 vr0, vr1, vr2, vr3

    vshuf4i.d     vr1,      vr1,      0x01
    vshuf4i.d     vr3,      vr3,      0x01

    vilvl.h       vr4,      vr1,      vr0   // 0 2 4 6 8 10 12 14
    vilvh.h       vr5,      vr1,      vr0   // 1 3 5 7 9 11 13 15
    vilvl.h       vr0,      vr5,      vr4   // 0 1  2  3  4  5  6  7 in0
    vilvh.h       vr1,      vr5,      vr4   // 8 9 10 11 12 13 14 15 in1
    vilvl.h       vr4,      vr3,      vr2   // 0 2 4 6 8 10 12 14
    vilvh.h       vr5,      vr3,      vr2   // 1 3 5 7 9 11 13 15
    vilvl.h       vr2,      vr5,      vr4   // 16 - 23  in2
    vilvh.h       vr3,      vr5,      vr4   // 24 - 31  in3

    la.local      t0,       idct_coeffs

    vreplgr2vr.h  vr23,     zero
    vst           vr23,     a2,       0
    vst           vr23,     a2,       16
    vst           vr23,     a2,       32
    vst           vr23,     a2,       48

    vldrepl.w     vr20,     t0,       8     // 1567
    vldrepl.w     vr21,     t0,       12    // 3784

    dct_8x4_core_lsx2 vr0, vr1, vr2, vr3, vr21, vr20, vr22, \
                      vr22, vr15, vr16, vr17, vr18

    vsrari.h      vr15,     vr15,     4
    vsrari.h      vr16,     vr16,     4
    vsrari.h      vr17,     vr17,     4
    vsrari.h      vr18,     vr18,     4

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr15, vr16, vr17, vr18

.DCT_DCT_8X4_END:
endfunc

.macro identity8_lsx in0, in1, in2, in3, in4, in5, in6, in7, \
                     out0, out1, out2, out3
    vssrarni.h.w  \in1,     \in0,     0
    vssrarni.h.w  \in3,     \in2,     0
    vssrarni.h.w  \in5,     \in4,     0
    vssrarni.h.w  \in7,     \in6,     0
    vsadd.h       \out0,    \in1,     \in1
    vsadd.h       \out1,    \in3,     \in3
    vsadd.h       \out2,    \in5,     \in5
    vsadd.h       \out3,    \in7,     \in7
.endm

function inv_txfm_add_identity_identity_8x4_8bpc_lsx
    la.local      t0,       idct_coeffs

    vld           vr0,      a2,       0    //  0  1  2  3  4  5  6  7  in0
    vld           vr1,      a2,       16   //  8  9 10 11 12 13 14 15  in1
    vld           vr2,      a2,       32   // 16 17 18 19 20 21 22 23  in2
    vld           vr3,      a2,       48   // 24 25 26 27 28 29 30 31  in3

    vldrepl.w     vr20,     t0,       0    // 2896

    rect2_w4_lsx vr0, vr0, vr20, vr18, vr19
    rect2_w4_lsx vr1, vr1, vr20, vr6, vr7
    rect2_w4_lsx vr2, vr2, vr20, vr8, vr9
    rect2_w4_lsx vr3, vr3, vr20, vr10, vr11

    identity8_lsx vr18, vr19, vr6, vr7, vr8, vr9, vr10, vr11, \
                  vr19, vr7, vr9, vr11

    vreplgr2vr.h  vr23,     zero
    vst           vr23,     a2,       0
    vst           vr23,     a2,       16
    vst           vr23,     a2,       32
    vst           vr23,     a2,       48

    li.w          t0,       1697
    vreplgr2vr.w  vr20,     t0
    identity_4x4_lsx vr19, vr19, vr20, vr19, vr19
    identity_4x4_lsx vr7, vr7, vr20, vr7, vr7
    identity_4x4_lsx vr9, vr9, vr20, vr9, vr9
    identity_4x4_lsx vr11, vr11, vr20, vr11, vr11

    vsrari.h      vr15,     vr19,     4
    vsrari.h      vr16,     vr7,      4
    vsrari.h      vr17,     vr9,      4
    vsrari.h      vr18,     vr11,     4

    vilvl.h       vr4,      vr16,     vr15
    vilvh.h       vr5,      vr16,     vr15
    vilvl.h       vr11,     vr5,      vr4
    vilvh.h       vr12,     vr5,      vr4
    vilvl.h       vr4,      vr18,     vr17
    vilvh.h       vr5,      vr18,     vr17
    vilvl.h       vr13,     vr5,      vr4
    vilvh.h       vr14,     vr5,      vr4
    vilvl.d       vr15,     vr13,     vr11
    vilvh.d       vr16,     vr13,     vr11
    vilvl.d       vr17,     vr14,     vr12
    vilvh.d       vr18,     vr14,     vr12

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr15, vr16, vr17, vr18
endfunc

const iadst8_coeffs, align=4
    .word          4076, 401, 3612, 1931
    .word          2598, 3166, 1189, 3920
    // idct_coeffs
    .word          2896, 0, 1567, 3784, 0, 0, 0, 0
endconst

.macro vmadd_vmsub_vssrarni_hw_12 in0, in1, in2, in3, in4, in5, in6, in7, \
                                  in8, in9, in10, in11, out0, out1, out2, out3
    vmul.w        \out0,    \in0,     \in4
    vmul.w        \out1,    \in0,     \in5
    vmadd.w       \out0,    \in1,     \in6   // t0a
    vmsub.w       \out1,    \in1,     \in7   // t1a
    vmul.w        \out2,    \in2,     \in8
    vmul.w        \out3,    \in2,     \in9
    vmadd.w       \out2,    \in3,     \in10  // t2a
    vmsub.w       \out3,    \in3,     \in11  // t3a
    vssrarni.h.w  \out1,    \out0,    12     // t0a t1a
    vssrarni.h.w  \out3,    \out2,    12     // t2a t3a
.endm

.macro adst8x4_1d_lsx
    la.local      t0,       iadst8_coeffs

    vldrepl.w     vr20,     t0,       0     // 4076
    vldrepl.w     vr21,     t0,       4     // 401
    vldrepl.w     vr22,     t0,       8     // 3612
    vldrepl.w     vr23,     t0,       12    // 1931

    // vr13 t0a t1a    vr15 t2a t3a
    vmadd_vmsub_vssrarni_hw_12 vr11, vr18, vr9, vr6, vr20, vr21, vr21, vr20, \
                               vr22, vr23, vr23, vr22, vr12, vr13, vr14, vr15
    vldrepl.w     vr20,     t0,       16    // 2598
    vldrepl.w     vr21,     t0,       20    // 3166
    vldrepl.w     vr22,     t0,       24    // 1189
    vldrepl.w     vr23,     t0,       28    // 3920

    // vr18 t4a t5a     vr6 t6a t7a
    vmadd_vmsub_vssrarni_hw_12 vr7, vr8, vr19, vr10, vr20, vr21, vr21, vr20, \
                               vr22, vr23, vr23, vr22, vr11, vr18, vr9, vr6

    vsadd.h       vr12,     vr13,     vr18  // t0 t1
    vsadd.h       vr14,     vr15,     vr6   // t2 t3
    vssub.h       vr16,     vr13,     vr18  // t4 t5
    vssub.h       vr18,     vr15,     vr6   // t6 t7

    la.local      t0,       idct_coeffs

    vldrepl.w     vr20,     t0,       8     // 1567
    vldrepl.w     vr21,     t0,       12    // 3784
    vldrepl.w     vr22,     t0,       0     // 2896

    vsllwil.w.h   vr7,      vr16,     0     // t4
    vexth.w.h     vr8,      vr16            // t5
    vsllwil.w.h   vr10,     vr18,     0     // t6
    vexth.w.h     vr11,     vr18            // t7

    // vr13 out0 out7   vr17 out1 out6
    vmadd_vmsub_vssrarni_hw_12 vr7, vr8, vr11, vr10, vr21, vr20, vr20, vr21, \
                               vr20, vr21, vr21, vr20, vr13, vr15, vr17, vr19
    vshuf4i.d     vr19,     vr19,     0x01

    vsadd.h       vr13,     vr12,     vr14  // out0 out7
    vssub.h       vr16,     vr12,     vr14  // t2 t3
    vsadd.h       vr17,     vr15,     vr19  // out1 out6
    vssub.h       vr18,     vr15,     vr19  // t6 t7

    vexth.w.h     vr20,     vr13            // out7
    vsllwil.w.h   vr21,     vr17,     0     // out1
    vneg.w        vr20,     vr20
    vneg.w        vr21,     vr21
    vssrarni.h.w  vr21,     vr20,     0     // out7 out1
    vilvl.d       vr13,     vr21,     vr13  // out0 out7
    vilvh.d       vr17,     vr17,     vr21  // out1 out6

    vsllwil.w.h   vr7,      vr16,     0     // t2
    vexth.w.h     vr8,      vr16            // t3
    vsllwil.w.h   vr10,     vr18,     0     // t6
    vexth.w.h     vr11,     vr18            // t7

    // vr15 out[3] out[4]    vr18 out[2] out[5]
    vmadd_vmsub_vssrarni_hw_12 vr7, vr8, vr10, vr11, vr22, vr22, vr22, vr22, \
                               vr22, vr22, vr22, vr22, vr14, vr15, vr19, vr18

    vexth.w.h     vr20,     vr18            // out5
    vsllwil.w.h   vr21,     vr15,     0     // out3
    vneg.w        vr20,     vr20
    vneg.w        vr21,     vr21
    vssrarni.h.w  vr21,     vr20,     0     // out5 out3
    vilvl.d       vr18,     vr21,     vr18  // out2 out5
    vilvh.d       vr15,     vr15,     vr21  // out3 out4
.endm

function inv_txfm_add_adst_dct_8x4_8bpc_lsx
    vld           vr0,      a2,       0     //  0  1  2  3  4  5  6  7  in0
    vld           vr1,      a2,       16    //  8  9 10 11 12 13 14 15  in1
    vld           vr2,      a2,       32    // 16 17 18 19 20 21 22 23  in2
    vld           vr3,      a2,       48    // 24 25 26 27 28 29 30 31  in3

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       0     // 2896

    rect2_w4_lsx vr0, vr0, vr20, vr18, vr19
    rect2_w4_lsx vr1, vr1, vr20, vr6, vr7
    rect2_w4_lsx vr2, vr2, vr20, vr8, vr9
    rect2_w4_lsx vr3, vr3, vr20, vr10, vr11

    adst8x4_1d_lsx

    vilvl.h       vr4,      vr17,     vr13
    vilvl.h       vr5,      vr15,     vr18
    vilvl.w       vr0,      vr5,      vr4
    vilvh.w       vr1,      vr5,      vr4
    vilvh.h       vr4,      vr18,     vr15
    vilvh.h       vr5,      vr13,     vr17
    vilvl.w       vr2,      vr5,      vr4
    vilvh.w       vr3,      vr5,      vr4

    vreplgr2vr.h  vr23,     zero
    vst           vr23,     a2,       0
    vst           vr23,     a2,       16
    vst           vr23,     a2,       32
    vst           vr23,     a2,       48

    la.local      t0,       idct_coeffs

    vldrepl.w     vr20,     t0,       8     // 1567
    vldrepl.w     vr21,     t0,       12    // 3784
    vldrepl.w     vr22,     t0,       0     // 2896

    dct_8x4_core_lsx2 vr0, vr1, vr2, vr3, vr21, vr20, vr22, \
                      vr22, vr15, vr16, vr17, vr18

    vsrari.h      vr15,     vr15,     4
    vsrari.h      vr16,     vr16,     4
    vsrari.h      vr17,     vr17,     4
    vsrari.h      vr18,     vr18,     4

    alsl.d        t2,       a1,       a0,    1

    VLD_DST_ADD_W8 vr15, vr16, vr17, vr18
endfunc

function inv_txfm_add_dct_adst_8x4_8bpc_lsx
    vld           vr0,      a2,       0     //  0  1  2  3  4  5  6  7  in0
    vld           vr1,      a2,       16    //  8  9 10 11 12 13 14 15  in1
    vld           vr2,      a2,       32    // 16 17 18 19 20 21 22 23  in2
    vld           vr3,      a2,       48    // 24 25 26 27 28 29 30 31  in3

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       0     // 2896

    rect2_w4_lsx vr0, vr0, vr20, vr18, vr19
    rect2_w4_lsx vr1, vr1, vr20, vr6, vr7
    rect2_w4_lsx vr2, vr2, vr20, vr8, vr9
    rect2_w4_lsx vr3, vr3, vr20, vr10, vr11

    vldrepl.w     vr20,      t0,       8    // 1567
    vldrepl.w     vr21,      t0,       12   // 3784
    vldrepl.w     vr22,      t0,       0    // 2896

    dct_8x4_core_lsx1 vr0, vr1, vr2, vr3

    vshuf4i.d     vr1,      vr1,      0x01
    vshuf4i.d     vr3,      vr3,      0x01

    vilvl.h       vr4,      vr1,      vr0
    vilvh.h       vr5,      vr1,      vr0
    vilvl.h       vr0,      vr5,      vr4
    vilvh.h       vr1,      vr5,      vr4
    vilvl.h       vr4,      vr3,      vr2
    vilvh.h       vr5,      vr3,      vr2
    vilvl.h       vr2,      vr5,      vr4
    vilvh.h       vr3,      vr5,      vr4

    la.local      t0,       iadst4_coeffs

    vreplgr2vr.h  vr23,     zero
    vst           vr23,     a2,       0
    vst           vr23,     a2,       16
    vst           vr23,     a2,       32
    vst           vr23,     a2,       48

    vldrepl.w     vr20,     t0,       0     // 1321
    vldrepl.w     vr21,     t0,       4     // 3803
    vldrepl.w     vr22,     t0,       8     // 2482
    vldrepl.w     vr23,     t0,       12    // 3344

    vsllwil.w.h   vr10,     vr0,      0
    vexth.w.h     vr11,     vr0
    vsllwil.w.h   vr12,     vr1,      0
    vexth.w.h     vr13,     vr1

    adst4x4_1d_lsx vr10, vr11, vr12, vr13, vr10, vr11, vr12, vr13

    vsllwil.w.h   vr14,     vr2,      0
    vexth.w.h     vr15,     vr2
    vsllwil.w.h   vr16,     vr3,      0
    vexth.w.h     vr17,     vr3

    adst4x4_1d_lsx vr14, vr15, vr16, vr17, vr14, vr15, vr16, vr17

    vssrarni.h.w  vr14,     vr10,     12
    vssrarni.h.w  vr15,     vr11,     12
    vssrarni.h.w  vr16,     vr12,     12
    vssrarni.h.w  vr17,     vr13,     12

    vsrari.h      vr14,     vr14,     4
    vsrari.h      vr15,     vr15,     4
    vsrari.h      vr16,     vr16,     4
    vsrari.h      vr17,     vr17,     4

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr14, vr15, vr16, vr17
endfunc

function inv_txfm_add_adst_adst_8x4_8bpc_lsx
    vld           vr0,      a2,       0     //  0  1  2  3  4  5  6  7  in0
    vld           vr1,      a2,       16    //  8  9 10 11 12 13 14 15  in1
    vld           vr2,      a2,       32    // 16 17 18 19 20 21 22 23  in2
    vld           vr3,      a2,       48    // 24 25 26 27 28 29 30 31  in3

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       0     // 2896

    rect2_w4_lsx vr0, vr0, vr20, vr18, vr19
    rect2_w4_lsx vr1, vr1, vr20, vr6, vr7
    rect2_w4_lsx vr2, vr2, vr20, vr8, vr9
    rect2_w4_lsx vr3, vr3, vr20, vr10, vr11

    adst8x4_1d_lsx

    vilvl.h       vr4,      vr17,     vr13
    vilvl.h       vr5,      vr15,     vr18
    vilvl.w       vr0,      vr5,      vr4
    vilvh.w       vr1,      vr5,      vr4
    vilvh.h       vr4,      vr18,     vr15
    vilvh.h       vr5,      vr13,     vr17
    vilvl.w       vr2,      vr5,      vr4
    vilvh.w       vr3,      vr5,      vr4

    la.local      t0,       iadst4_coeffs

    vreplgr2vr.h  vr23,     zero
    vst           vr23,     a2,       0
    vst           vr23,     a2,       16
    vst           vr23,     a2,       32
    vst           vr23,     a2,       48

    vldrepl.w     vr20,     t0,       0     // 1321
    vldrepl.w     vr21,     t0,       4     // 3803
    vldrepl.w     vr22,     t0,       8     // 2482
    vldrepl.w     vr23,     t0,       12    // 3344

    vsllwil.w.h   vr10,     vr0,      0
    vexth.w.h     vr11,     vr0
    vsllwil.w.h   vr12,     vr1,      0
    vexth.w.h     vr13,     vr1

    adst4x4_1d_lsx vr10, vr11, vr12, vr13, vr10, vr11, vr12, vr13

    vsllwil.w.h   vr14,     vr2,      0
    vexth.w.h     vr15,     vr2
    vsllwil.w.h   vr16,     vr3,      0
    vexth.w.h     vr17,     vr3

    adst4x4_1d_lsx vr14, vr15, vr16, vr17, vr14, vr15, vr16, vr17

    vssrarni.h.w  vr14,     vr10,     12
    vssrarni.h.w  vr15,     vr11,     12
    vssrarni.h.w  vr16,     vr12,     12
    vssrarni.h.w  vr17,     vr13,     12

    vsrari.h      vr14,     vr14,     4
    vsrari.h      vr15,     vr15,     4
    vsrari.h      vr16,     vr16,     4
    vsrari.h      vr17,     vr17,     4

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr14, vr15, vr16, vr17
endfunc

function inv_txfm_add_flipadst_adst_8x4_8bpc_lsx
    vld           vr0,      a2,       0    //  0  1  2  3  4  5  6  7  in0
    vld           vr1,      a2,       16   //  8  9 10 11 12 13 14 15  in1
    vld           vr2,      a2,       32   // 16 17 18 19 20 21 22 23  in2
    vld           vr3,      a2,       48   // 24 25 26 27 28 29 30 31  in3

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       0    // 2896

    rect2_w4_lsx vr0, vr0, vr20, vr18, vr19
    rect2_w4_lsx vr1, vr1, vr20, vr6, vr7
    rect2_w4_lsx vr2, vr2, vr20, vr8, vr9
    rect2_w4_lsx vr3, vr3, vr20, vr10, vr11

    adst8x4_1d_lsx

    vilvl.h       vr20,     vr15,     vr13
    vilvl.h       vr21,     vr18,     vr17
    vilvl.w       vr0,      vr21,     vr20
    vilvh.w       vr1,      vr21,     vr20
    vilvh.h       vr20,     vr15,     vr13
    vilvh.h       vr21,     vr18,     vr17
    vilvl.w       vr2,      vr21,     vr20
    vilvh.w       vr3,      vr21,     vr20
    vshuf4i.h     vr0,      vr0,      0x2d
    vshuf4i.h     vr1,      vr1,      0x2d
    vshuf4i.h     vr2,      vr2,      0x78
    vshuf4i.h     vr3,      vr3,      0x78

    la.local      t0,       iadst4_coeffs

    vreplgr2vr.h  vr23,     zero
    vst           vr23,     a2,       0
    vst           vr23,     a2,       16
    vst           vr23,     a2,       32
    vst           vr23,     a2,       48

    vldrepl.w     vr20,     t0,       0     // 1321
    vldrepl.w     vr21,     t0,       4     // 3803
    vldrepl.w     vr22,     t0,       8     // 2482
    vldrepl.w     vr23,     t0,       12    // 3344

    vsllwil.w.h   vr10,     vr2,      0
    vexth.w.h     vr11,     vr2
    vsllwil.w.h   vr12,     vr3,      0
    vexth.w.h     vr13,     vr3

    adst4x4_1d_lsx vr10, vr11, vr12, vr13, vr10, vr11, vr12, vr13

    vsllwil.w.h   vr14,     vr0,      0
    vexth.w.h     vr15,     vr0
    vsllwil.w.h   vr16,     vr1,      0
    vexth.w.h     vr17,     vr1

    adst4x4_1d_lsx vr14, vr15, vr16, vr17, vr14, vr15, vr16, vr17

    vssrarni.h.w  vr14,     vr10,     12
    vssrarni.h.w  vr15,     vr11,     12
    vssrarni.h.w  vr16,     vr12,     12
    vssrarni.h.w  vr17,     vr13,     12

    vsrari.h      vr14,     vr14,     4
    vsrari.h      vr15,     vr15,     4
    vsrari.h      vr16,     vr16,     4
    vsrari.h      vr17,     vr17,     4

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr14, vr15, vr16, vr17
endfunc

function inv_txfm_add_adst_flipadst_8x4_8bpc_lsx
    vld           vr0,      a2,       0      // in0
    vld           vr1,      a2,       16     // in1
    vld           vr2,      a2,       32     // in2
    vld           vr3,      a2,       48     // in3

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       0      // 2896

    rect2_w4_lsx vr0, vr0, vr20, vr18, vr19  // 0  8 16 24 1  9 17 25 in0 in1
    rect2_w4_lsx vr1, vr1, vr20, vr6, vr7    // 2 10 18 26 3 11 19 27 in2 in3
    rect2_w4_lsx vr2, vr2, vr20, vr8, vr9    // 4 12 20 28 5 13 21 29 in4 in5
    rect2_w4_lsx vr3, vr3, vr20, vr10, vr11  // 6 14 22 30 7 15 23 31 in6 in7

    adst8x4_1d_lsx

    vilvl.h       vr4,      vr17,     vr13
    vilvl.h       vr5,      vr15,     vr18
    vilvl.w       vr0,      vr5,      vr4
    vilvh.w       vr1,      vr5,      vr4
    vilvh.h       vr4,      vr18,     vr15
    vilvh.h       vr5,      vr13,     vr17
    vilvl.w       vr2,      vr5,      vr4
    vilvh.w       vr3,      vr5,      vr4

    la.local      t0,       iadst4_coeffs

    vreplgr2vr.h  vr23,     zero
    vst           vr23,     a2,       0
    vst           vr23,     a2,       16
    vst           vr23,     a2,       32
    vst           vr23,     a2,       48

    vldrepl.w     vr20,     t0,       0     // 1321
    vldrepl.w     vr21,     t0,       4     // 3803
    vldrepl.w     vr22,     t0,       8     // 2482
    vldrepl.w     vr23,     t0,       12    // 3344

    vsllwil.w.h   vr10,     vr0,      0
    vexth.w.h     vr11,     vr0
    vsllwil.w.h   vr12,     vr1,      0
    vexth.w.h     vr13,     vr1

    adst4x4_1d_lsx vr10, vr11, vr12, vr13, vr10, vr11, vr12, vr13

    vsllwil.w.h   vr14,     vr2,      0
    vexth.w.h     vr15,     vr2
    vsllwil.w.h   vr16,     vr3,      0
    vexth.w.h     vr17,     vr3

    adst4x4_1d_lsx vr14, vr15, vr16, vr17, vr14, vr15, vr16, vr17

    vssrarni.h.w  vr14,     vr10,     12
    vssrarni.h.w  vr15,     vr11,     12
    vssrarni.h.w  vr16,     vr12,     12
    vssrarni.h.w  vr17,     vr13,     12

    vsrari.h      vr14,     vr14,     4
    vsrari.h      vr15,     vr15,     4
    vsrari.h      vr16,     vr16,     4
    vsrari.h      vr17,     vr17,     4

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr17, vr16, vr15, vr14
endfunc

function inv_txfm_add_flipadst_dct_8x4_8bpc_lsx
    vld           vr0,      a2,       0      // in0
    vld           vr1,      a2,       16     // in1
    vld           vr2,      a2,       32     // in2
    vld           vr3,      a2,       48     // in3

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       0      // 2896

    rect2_w4_lsx vr0, vr0, vr20, vr18, vr19  // 0  8 16 24 1  9 17 25 in0 in1
    rect2_w4_lsx vr1, vr1, vr20, vr6, vr7    // 2 10 18 26 3 11 19 27 in2 in3
    rect2_w4_lsx vr2, vr2, vr20, vr8, vr9    // 4 12 20 28 5 13 21 29 in4 in5
    rect2_w4_lsx vr3, vr3, vr20, vr10, vr11  // 6 14 22 30 7 15 23 31 in6 in7

    adst8x4_1d_lsx

    vilvl.h       vr20,     vr15,     vr13
    vilvl.h       vr21,     vr18,     vr17
    vilvl.w       vr0,      vr21,     vr20
    vilvh.w       vr1,      vr21,     vr20
    vilvh.h       vr20,     vr15,     vr13
    vilvh.h       vr21,     vr18,     vr17
    vilvl.w       vr2,      vr21,     vr20
    vilvh.w       vr3,      vr21,     vr20
    vshuf4i.h     vr0,      vr0,      0x2d
    vshuf4i.h     vr1,      vr1,      0x2d
    vshuf4i.h     vr2,      vr2,      0x78
    vshuf4i.h     vr3,      vr3,      0x78

    vreplgr2vr.h  vr23,     zero
    vst           vr23,     a2,       0
    vst           vr23,     a2,       16
    vst           vr23,     a2,       32
    vst           vr23,     a2,       48

    la.local      t0,       idct_coeffs

    vldrepl.w     vr20,     t0,       8     // 1567
    vldrepl.w     vr21,     t0,       12    // 3784
    vldrepl.w     vr22,     t0,       0     // 2896

    dct_8x4_core_lsx2 vr2, vr3, vr0, vr1, vr21, vr20, vr22, \
                      vr22, vr15, vr16, vr17, vr18

    vsrari.h      vr15,     vr15,     4
    vsrari.h      vr16,     vr16,     4
    vsrari.h      vr17,     vr17,     4
    vsrari.h      vr18,     vr18,     4

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr15, vr16, vr17, vr18
endfunc

function inv_txfm_add_dct_flipadst_8x4_8bpc_lsx
    la.local      t0,       idct_coeffs

    vld           vr0,      a2,       0     // in0
    vld           vr1,      a2,       16    // in1
    vld           vr2,      a2,       32    // in2
    vld           vr3,      a2,       48    // in3

    vldrepl.w     vr20,     t0,       0     // 2896

    rect2_w4_lsx vr0, vr0, vr20, vr18, vr19  // in0 0 - 7
    rect2_w4_lsx vr1, vr1, vr20, vr6, vr7    // in1 8 - 15
    rect2_w4_lsx vr2, vr2, vr20, vr8, vr9    // in2 16 - 23
    rect2_w4_lsx vr3, vr3, vr20, vr10, vr11  // in3 24 - 31

    vldrepl.w     vr20,     t0,       8      // 1567
    vldrepl.w     vr21,     t0,       12     // 3784
    vldrepl.w     vr22,     t0,       0      // 2896

    dct_8x4_core_lsx1 vr0, vr1, vr2, vr3

    vshuf4i.d     vr1,      vr1,      0x01
    vshuf4i.d     vr3,      vr3,      0x01

    vilvl.h       vr4,      vr1,      vr0
    vilvh.h       vr5,      vr1,      vr0
    vilvl.h       vr0,      vr5,      vr4
    vilvh.h       vr1,      vr5,      vr4
    vilvl.h       vr4,      vr3,      vr2
    vilvh.h       vr5,      vr3,      vr2
    vilvl.h       vr2,      vr5,      vr4
    vilvh.h       vr3,      vr5,      vr4

    la.local      t0,       iadst4_coeffs

    vreplgr2vr.h  vr23,     zero
    vst           vr23,     a2,       0
    vst           vr23,     a2,       16
    vst           vr23,     a2,       32
    vst           vr23,     a2,       48

    vldrepl.w     vr20,     t0,       0     // 1321
    vldrepl.w     vr21,     t0,       4     // 3803
    vldrepl.w     vr22,     t0,       8     // 2482
    vldrepl.w     vr23,     t0,       12    // 3344

    vsllwil.w.h   vr10,     vr0,      0     // in0
    vexth.w.h     vr11,     vr0             // in1
    vsllwil.w.h   vr12,     vr1,      0     // in2
    vexth.w.h     vr13,     vr1             // in3
    adst4x4_1d_lsx vr10, vr11, vr12, vr13, vr10, vr11, vr12, vr13

    vsllwil.w.h   vr14,     vr2,      0
    vexth.w.h     vr15,     vr2
    vsllwil.w.h   vr16,     vr3,      0
    vexth.w.h     vr17,     vr3
    adst4x4_1d_lsx vr14, vr15, vr16, vr17, vr14, vr15, vr16, vr17

    vssrarni.h.w  vr14,     vr10,     12
    vssrarni.h.w  vr15,     vr11,     12
    vssrarni.h.w  vr16,     vr12,     12
    vssrarni.h.w  vr17,     vr13,     12
    vsrari.h      vr14,     vr14,     4
    vsrari.h      vr15,     vr15,     4
    vsrari.h      vr16,     vr16,     4
    vsrari.h      vr17,     vr17,     4

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr17, vr16, vr15, vr14
endfunc

function inv_txfm_add_flipadst_flipadst_8x4_8bpc_lsx
    vld           vr0,      a2,       0      // in0
    vld           vr1,      a2,       16     // in1
    vld           vr2,      a2,       32     // in2
    vld           vr3,      a2,       48     // in3

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       0      // 2896

    rect2_w4_lsx vr0, vr0, vr20, vr18, vr19  // 0  8 16 24 1  9 17 25 in0 in1
    rect2_w4_lsx vr1, vr1, vr20, vr6, vr7    // 2 10 18 26 3 11 19 27 in2 in3
    rect2_w4_lsx vr2, vr2, vr20, vr8, vr9    // 4 12 20 28 5 13 21 29 in4 in5
    rect2_w4_lsx vr3, vr3, vr20, vr10, vr11  // 6 14 22 30 7 15 23 31 in6 in7

    adst8x4_1d_lsx

    vilvl.h       vr20,     vr15,     vr13
    vilvl.h       vr21,     vr18,     vr17
    vilvl.w       vr0,      vr21,     vr20
    vilvh.w       vr1,      vr21,     vr20
    vilvh.h       vr20,     vr15,     vr13
    vilvh.h       vr21,     vr18,     vr17
    vilvl.w       vr2,      vr21,     vr20
    vilvh.w       vr3,      vr21,     vr20
    vshuf4i.h     vr0,      vr0,      0x2d
    vshuf4i.h     vr1,      vr1,      0x2d
    vshuf4i.h     vr2,      vr2,      0x78
    vshuf4i.h     vr3,      vr3,      0x78

    la.local      t0,       iadst4_coeffs

    vreplgr2vr.h  vr23,     zero
    vst           vr23,     a2,       0
    vst           vr23,     a2,       16
    vst           vr23,     a2,       32
    vst           vr23,     a2,       48

    vldrepl.w     vr20,     t0,       0     // 1321
    vldrepl.w     vr21,     t0,       4     // 3803
    vldrepl.w     vr22,     t0,       8     // 2482
    vldrepl.w     vr23,     t0,       12    // 3344

    vsllwil.w.h   vr10,     vr2,      0     // in0
    vexth.w.h     vr11,     vr2             // in1
    vsllwil.w.h   vr12,     vr3,      0     // in2
    vexth.w.h     vr13,     vr3             // in3
    adst4x4_1d_lsx vr10, vr11, vr12, vr13, vr10, vr11, vr12, vr13

    vsllwil.w.h   vr14,     vr0,      0
    vexth.w.h     vr15,     vr0
    vsllwil.w.h   vr16,     vr1,      0
    vexth.w.h     vr17,     vr1
    adst4x4_1d_lsx vr14, vr15, vr16, vr17, vr14, vr15, vr16, vr17

    vssrarni.h.w  vr14,     vr10,     12
    vssrarni.h.w  vr15,     vr11,     12
    vssrarni.h.w  vr16,     vr12,     12
    vssrarni.h.w  vr17,     vr13,     12

    vsrari.h      vr14,     vr14,     4
    vsrari.h      vr15,     vr15,     4
    vsrari.h      vr16,     vr16,     4
    vsrari.h      vr17,     vr17,     4

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr17, vr16, vr15, vr14
endfunc

function inv_txfm_add_dct_identity_8x4_8bpc_lsx
    vld           vr0,      a2,       0     // in0
    vld           vr1,      a2,       16    // in1
    vld           vr2,      a2,       32    // in2
    vld           vr3,      a2,       48    // in3

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       0     // 2896

    rect2_w4_lsx vr0, vr0, vr20, vr18, vr19 // in0 0 - 7
    rect2_w4_lsx vr1, vr1, vr20, vr6, vr7   // in1 8 - 15
    rect2_w4_lsx vr2, vr2, vr20, vr8, vr9   // in2 16 - 23
    rect2_w4_lsx vr3, vr3, vr20, vr10, vr11 // in3 24 - 31

    vldrepl.w     vr20,     t0,       8     // 1567
    vldrepl.w     vr21,     t0,       12    // 3784
    vldrepl.w     vr22,     t0,       0     // 2896

    dct_8x4_core_lsx1 vr0, vr1, vr2, vr3

    vshuf4i.d     vr1,      vr1,      0x01
    vshuf4i.d     vr3,      vr3,      0x01

    vilvl.h       vr4,      vr1,      vr0
    vilvh.h       vr5,      vr1,      vr0
    vilvl.h       vr0,      vr5,      vr4
    vilvh.h       vr1,      vr5,      vr4
    vilvl.h       vr4,      vr3,      vr2
    vilvh.h       vr5,      vr3,      vr2
    vilvl.h       vr2,      vr5,      vr4
    vilvh.h       vr3,      vr5,      vr4
    vilvl.d       vr14,     vr2,      vr0
    vilvh.d       vr15,     vr2,      vr0
    vilvl.d       vr16,     vr3,      vr1
    vilvh.d       vr17,     vr3,      vr1

    vreplgr2vr.h  vr23,     zero
    vst           vr23,     a2,       0
    vst           vr23,     a2,       16
    vst           vr23,     a2,       32
    vst           vr23,     a2,       48

    li.w          t0,       1697
    vreplgr2vr.w  vr20,     t0

    identity_4x4_lsx vr14, vr14, vr20, vr14, vr14
    identity_4x4_lsx vr15, vr15, vr20, vr15, vr15
    identity_4x4_lsx vr16, vr16, vr20, vr16, vr16
    identity_4x4_lsx vr17, vr17, vr20, vr17, vr17

    vsrari.h      vr14,     vr14,     4
    vsrari.h      vr15,     vr15,     4
    vsrari.h      vr16,     vr16,     4
    vsrari.h      vr17,     vr17,     4

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr14, vr15, vr16, vr17
endfunc

function inv_txfm_add_identity_dct_8x4_8bpc_lsx
    vld           vr0,      a2,       0     // in0
    vld           vr1,      a2,       16    // in1
    vld           vr2,      a2,       32    // in2
    vld           vr3,      a2,       48    // in3

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       0     // 2896

    rect2_w4_lsx vr0, vr0, vr20, vr18, vr19 // in0 0 - 7
    rect2_w4_lsx vr1, vr1, vr20, vr6, vr7   // in1 8 - 15
    rect2_w4_lsx vr2, vr2, vr20, vr8, vr9   // in2 16 - 23
    rect2_w4_lsx vr3, vr3, vr20, vr10, vr11 // in3 24 - 31

    identity8_lsx vr18, vr19, vr6, vr7, vr8, vr9, vr10, vr11, \
                  vr19, vr7, vr9, vr11

    vreplgr2vr.h  vr23,     zero
    vst           vr23,     a2,       0
    vst           vr23,     a2,       16
    vst           vr23,     a2,       32
    vst           vr23,     a2,       48

    vilvl.h       vr4,      vr7,      vr19
    vilvh.h       vr5,      vr7,      vr19
    vilvl.h       vr0,      vr5,      vr4
    vilvh.h       vr1,      vr5,      vr4
    vilvl.h       vr4,      vr11,     vr9
    vilvh.h       vr5,      vr11,     vr9
    vilvl.h       vr2,      vr5,      vr4
    vilvh.h       vr3,      vr5,      vr4

    la.local      t0,       idct_coeffs

    vldrepl.w     vr20,     t0,       8    // 1567
    vldrepl.w     vr21,     t0,       12   // 3784
    vldrepl.w     vr22,     t0,       0    // 2896

    dct_8x4_core_lsx2 vr0, vr1, vr2, vr3, vr21, vr20, vr22, \
                      vr22, vr15, vr16, vr17, vr18

    vsrari.h      vr15,     vr15,     4
    vsrari.h      vr16,     vr16,     4
    vsrari.h      vr17,     vr17,     4
    vsrari.h      vr18,     vr18,     4

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr15, vr16, vr17, vr18
endfunc

function inv_txfm_add_flipadst_identity_8x4_8bpc_lsx
    vld           vr0,      a2,       0      // in0
    vld           vr1,      a2,       16     // in1
    vld           vr2,      a2,       32     // in2
    vld           vr3,      a2,       48     // in3

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       0      // 2896

    rect2_w4_lsx vr0, vr0, vr20, vr18, vr19  // 0  8 16 24 1  9 17 25 in0 in1
    rect2_w4_lsx vr1, vr1, vr20, vr6, vr7    // 2 10 18 26 3 11 19 27 in2 in3
    rect2_w4_lsx vr2, vr2, vr20, vr8, vr9    // 4 12 20 28 5 13 21 29 in4 in5
    rect2_w4_lsx vr3, vr3, vr20, vr10, vr11  // 6 14 22 30 7 15 23 31 in6 in7

    adst8x4_1d_lsx

    vilvl.h       vr20,     vr15,     vr13
    vilvl.h       vr21,     vr18,     vr17
    vilvl.w       vr0,      vr21,     vr20
    vilvh.w       vr1,      vr21,     vr20
    vilvh.h       vr20,     vr15,     vr13
    vilvh.h       vr21,     vr18,     vr17
    vilvl.w       vr2,      vr21,     vr20
    vilvh.w       vr3,      vr21,     vr20
    vshuf4i.h     vr0,      vr0,      0x2d
    vshuf4i.h     vr1,      vr1,      0x2d
    vshuf4i.h     vr2,      vr2,      0x78
    vshuf4i.h     vr3,      vr3,      0x78
    vilvl.d       vr14,     vr0,      vr2    // in0
    vilvh.d       vr15,     vr0,      vr2    // in1
    vilvl.d       vr16,     vr1,      vr3    // in2
    vilvh.d       vr17,     vr1,      vr3    // in3

    vreplgr2vr.h  vr23,     zero
    vst           vr23,     a2,       0
    vst           vr23,     a2,       16
    vst           vr23,     a2,       32
    vst           vr23,     a2,       48

    li.w          t0,       1697
    vreplgr2vr.w  vr20,     t0

    identity_4x4_lsx vr14, vr14, vr20, vr14, vr14
    identity_4x4_lsx vr15, vr15, vr20, vr15, vr15
    identity_4x4_lsx vr16, vr16, vr20, vr16, vr16
    identity_4x4_lsx vr17, vr17, vr20, vr17, vr17

    vsrari.h      vr14,     vr14,     4
    vsrari.h      vr15,     vr15,     4
    vsrari.h      vr16,     vr16,     4
    vsrari.h      vr17,     vr17,     4

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr14, vr15, vr16, vr17
endfunc

function inv_txfm_add_identity_flipadst_8x4_8bpc_lsx
    vld           vr0,      a2,       0     // in0
    vld           vr1,      a2,       16    // in1
    vld           vr2,      a2,       32    // in2
    vld           vr3,      a2,       48    // in3

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       0     // 2896

    rect2_w4_lsx vr0, vr0, vr20, vr18, vr19 // in0 0 - 7
    rect2_w4_lsx vr1, vr1, vr20, vr6, vr7   // in1 8 - 15
    rect2_w4_lsx vr2, vr2, vr20, vr8, vr9   // in2 16 - 23
    rect2_w4_lsx vr3, vr3, vr20, vr10, vr11 // in3 24 - 31

    identity8_lsx vr18, vr19, vr6, vr7, vr8, vr9, vr10, vr11, \
                  vr19, vr7, vr9, vr11

    vreplgr2vr.h  vr23,     zero
    vst           vr23,     a2,       0
    vst           vr23,     a2,       16
    vst           vr23,     a2,       32
    vst           vr23,     a2,       48

    vilvl.h       vr4,      vr7,      vr19
    vilvh.h       vr5,      vr7,      vr19
    vilvl.h       vr0,      vr5,      vr4
    vilvh.h       vr1,      vr5,      vr4
    vilvl.h       vr4,      vr11,     vr9
    vilvh.h       vr5,      vr11,     vr9
    vilvl.h       vr2,      vr5,      vr4
    vilvh.h       vr3,      vr5,      vr4

    la.local      t0,       iadst4_coeffs

    vreplgr2vr.h  vr23,     zero
    vst           vr23,     a2,       0
    vst           vr23,     a2,       16
    vst           vr23,     a2,       32
    vst           vr23,     a2,       48

    vldrepl.w     vr20,     t0,       0     // 1321
    vldrepl.w     vr21,     t0,       4     // 3803
    vldrepl.w     vr22,     t0,       8     // 2482
    vldrepl.w     vr23,     t0,       12    // 3344

    vsllwil.w.h   vr10,     vr0,      0     // in0
    vexth.w.h     vr11,     vr0             // in1
    vsllwil.w.h   vr12,     vr1,      0     // in2
    vexth.w.h     vr13,     vr1             // in3
    adst4x4_1d_lsx vr10, vr11, vr12, vr13, vr10, vr11, vr12, vr13

    vsllwil.w.h   vr14,     vr2,      0
    vexth.w.h     vr15,     vr2
    vsllwil.w.h   vr16,     vr3,      0
    vexth.w.h     vr17,     vr3
    adst4x4_1d_lsx vr14, vr15, vr16, vr17, vr14, vr15, vr16, vr17

    vssrarni.h.w  vr14,     vr10,     12
    vssrarni.h.w  vr15,     vr11,     12
    vssrarni.h.w  vr16,     vr12,     12
    vssrarni.h.w  vr17,     vr13,     12

    vsrari.h      vr14,     vr14,     4
    vsrari.h      vr15,     vr15,     4
    vsrari.h      vr16,     vr16,     4
    vsrari.h      vr17,     vr17,     4

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr17, vr16, vr15, vr14
endfunc

function inv_txfm_add_adst_identity_8x4_8bpc_lsx
    vld           vr0,      a2,       0     // in0
    vld           vr1,      a2,       16    // in1
    vld           vr2,      a2,       32    // in2
    vld           vr3,      a2,       48    // in3

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       0     // 2896

    rect2_w4_lsx vr0, vr0, vr20, vr18, vr19 // 0  8 16 24 1  9 17 25 in0 in1
    rect2_w4_lsx vr1, vr1, vr20, vr6, vr7   // 2 10 18 26 3 11 19 27 in2 in3
    rect2_w4_lsx vr2, vr2, vr20, vr8, vr9   // 4 12 20 28 5 13 21 29 in4 in5
    rect2_w4_lsx vr3, vr3, vr20, vr10, vr11 // 6 14 22 30 7 15 23 31 in6 in7

    adst8x4_1d_lsx

    vilvl.h       vr4,      vr17,     vr13
    vilvl.h       vr5,      vr15,     vr18
    vilvl.w       vr14,     vr5,      vr4   // in0 in1
    vilvh.w       vr16,     vr5,      vr4   // in2 in3
    vilvh.h       vr4,      vr18,     vr15
    vilvh.h       vr5,      vr13,     vr17
    vilvl.w       vr17,     vr5,      vr4
    vilvh.w       vr18,     vr5,      vr4
    vilvl.d       vr10,     vr17,     vr14  // in0
    vilvh.d       vr11,     vr17,     vr14  // in1
    vilvl.d       vr12,     vr18,     vr16  // in2
    vilvh.d       vr13,     vr18,     vr16  // in3

    vreplgr2vr.h  vr23,     zero
    vst           vr23,     a2,       0
    vst           vr23,     a2,       16
    vst           vr23,     a2,       32
    vst           vr23,     a2,       48

    li.w          t0,       1697
    vreplgr2vr.w  vr20,     t0

    identity_4x4_lsx vr10, vr10, vr20, vr10, vr15
    identity_4x4_lsx vr11, vr11, vr20, vr11, vr16
    identity_4x4_lsx vr12, vr12, vr20, vr12, vr17
    identity_4x4_lsx vr13, vr13, vr20, vr13, vr18

    vsrari.h      vr15,     vr15,     4
    vsrari.h      vr16,     vr16,     4
    vsrari.h      vr17,     vr17,     4
    vsrari.h      vr18,     vr18,     4

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr15, vr16, vr17, vr18
endfunc

function inv_txfm_add_identity_adst_8x4_8bpc_lsx
    vld           vr0,      a2,       0     // in0
    vld           vr1,      a2,       16    // in1
    vld           vr2,      a2,       32    // in2
    vld           vr3,      a2,       48    // in3

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       0     // 2896

    rect2_w4_lsx vr0, vr0, vr20, vr18, vr19 // in0 0 - 7
    rect2_w4_lsx vr1, vr1, vr20, vr6, vr7   // in1 8 - 15
    rect2_w4_lsx vr2, vr2, vr20, vr8, vr9   // in2 16 - 23
    rect2_w4_lsx vr3, vr3, vr20, vr10, vr11 // in3 24 - 31

    identity8_lsx vr18, vr19, vr6, vr7, vr8, vr9, vr10, vr11, \
                  vr0, vr1, vr2, vr3

    vilvl.h       vr4,      vr1,      vr0   // 0 2 4 6 8 10 12 14
    vilvh.h       vr5,      vr1,      vr0   // 1 3 5 7 9 11 13 15
    vilvl.h       vr0,      vr5,      vr4   // 0 1  2  3  4  5  6  7
    vilvh.h       vr1,      vr5,      vr4   // 8 9 10 11 12 13 14 15
    vilvl.h       vr4,      vr3,      vr2   // 0 2 4 6 8 10 12 14
    vilvh.h       vr5,      vr3,      vr2   // 1 3 5 7 9 11 13 15
    vilvl.h       vr2,      vr5,      vr4   // 0 1  2  3  4  5  6  7
    vilvh.h       vr3,      vr5,      vr4   // 8 9 10 11 12 13 14 15

    vreplgr2vr.h  vr23,     zero
    vst           vr23,     a2,       0
    vst           vr23,     a2,       16
    vst           vr23,     a2,       32
    vst           vr23,     a2,       48

    la.local      t0,       iadst4_coeffs

    vldrepl.w     vr20,     t0,       0     // 1321
    vldrepl.w     vr21,     t0,       4     // 3803
    vldrepl.w     vr22,     t0,       8     // 2482
    vldrepl.w     vr23,     t0,       12    // 3344

    vsllwil.w.h   vr10,     vr0,      0
    vexth.w.h     vr11,     vr0
    vsllwil.w.h   vr12,     vr1,      0
    vexth.w.h     vr13,     vr1

    adst4x4_1d_lsx vr10, vr11, vr12, vr13, vr10, vr11, vr12, vr13

    vsllwil.w.h   vr14,     vr2,      0
    vexth.w.h     vr15,     vr2
    vsllwil.w.h   vr16,     vr3,      0
    vexth.w.h     vr17,     vr3

    adst4x4_1d_lsx vr14, vr15, vr16, vr17, vr14, vr15, vr16, vr17

    vssrarni.h.w  vr14,     vr10,     12
    vssrarni.h.w  vr15,     vr11,     12
    vssrarni.h.w  vr16,     vr12,     12
    vssrarni.h.w  vr17,     vr13,     12

    vsrari.h      vr14,     vr14,     4
    vsrari.h      vr15,     vr15,     4
    vsrari.h      vr16,     vr16,     4
    vsrari.h      vr17,     vr17,     4

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr14, vr15, vr16, vr17
endfunc

function inv_txfm_add_identity_identity_8x8_8bpc_lsx

    vld_x8 a2, 0, 16, vr0, vr1, vr2, vr3, vr4, vr5, vr14, vr15

    // identity8
    vsllwil.w.h   vr6,      vr0,      1
    vsllwil.w.h   vr7,      vr1,      1
    vsllwil.w.h   vr8,      vr2,      1
    vsllwil.w.h   vr9,      vr3,      1
    vsllwil.w.h   vr10,     vr4,      1
    vsllwil.w.h   vr11,     vr5,      1
    vsllwil.w.h   vr12,     vr14,     1
    vsllwil.w.h   vr13,     vr15,     1

.irp i, vr0, vr1, vr2, vr3, vr4, vr5, vr14, vr15
    vexth.w.h     \i,       \i
.endr

.irp i, vr0, vr1, vr2, vr3, vr4, vr5, vr14, vr15
    vslli.w       \i,       \i,       1
.endr

    vssrarni.h.w  vr0,      vr6,      1     // in0
    vssrarni.h.w  vr1,      vr7,      1     // in1
    vssrarni.h.w  vr2,      vr8,      1     // in2
    vssrarni.h.w  vr3,      vr9,      1     // in3
    vssrarni.h.w  vr4,      vr10,     1     // in4
    vssrarni.h.w  vr5,      vr11,     1     // in5
    vssrarni.h.w  vr14,     vr12,     1     // in6
    vssrarni.h.w  vr15,     vr13,     1     // in7

    vreplgr2vr.h  vr23,     zero
.irp i, 0, 16, 32, 48, 64, 80, 96, 112
    vst           vr23,     a2,       \i
.endr

    LSX_TRANSPOSE8x8_H vr0, vr1, vr2, vr3, vr4, vr5, vr14, vr15, \
                       vr16, vr17, vr18, vr19, vr20, vr21, vr22, vr23, \
                       vr6, vr7, vr8, vr9, vr10, vr11, vr12 vr13

    vsllwil.w.h   vr6,      vr16,     1
    vsllwil.w.h   vr7,      vr17,     1
    vsllwil.w.h   vr8,      vr18,     1
    vsllwil.w.h   vr9,      vr19,     1
    vsllwil.w.h   vr10,     vr20,     1
    vsllwil.w.h   vr11,     vr21,     1
    vsllwil.w.h   vr12,     vr22,     1
    vsllwil.w.h   vr13,     vr23,     1

.irp i, vr16, vr17, vr18, vr19, vr20, vr21, vr22, vr23
    vexth.w.h     \i,       \i
.endr

.irp i, vr16, vr17, vr18, vr19, vr20, vr21, vr22, vr23
    vslli.w       \i,       \i,       1
.endr

    vssrarni.h.w  vr16,     vr6,      4     // in0
    vssrarni.h.w  vr17,     vr7,      4     // in1
    vssrarni.h.w  vr18,     vr8,      4     // in2
    vssrarni.h.w  vr19,     vr9,      4     // in3
    vssrarni.h.w  vr20,     vr10,     4     // in4
    vssrarni.h.w  vr21,     vr11,     4     // in5
    vssrarni.h.w  vr22,     vr12,     4     // in6
    vssrarni.h.w  vr23,     vr13,     4     // in7

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr16, vr17, vr18, vr19

    alsl.d        a0,       a1,       a0,     2
    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr20, vr21, vr22, vr23

endfunc

.macro adst8x8_1d_lsx out0, out1, out2, out3
    la.local      t0,       iadst8_coeffs

    vldrepl.w     vr20,     t0,       0     // 4076
    vldrepl.w     vr21,     t0,       4     // 401
    vldrepl.w     vr22,     t0,       8     // 3612
    vldrepl.w     vr23,     t0,       12    // 1931

    // vr13 t0a t1a    vr15 t2a t3a
    vmadd_vmsub_vssrarni_hw_12 vr11, vr18, vr9, vr6, vr20, vr21, vr21, vr20, \
                               vr22, vr23, vr23, vr22, vr12, vr13, vr14, vr15
    vldrepl.w     vr20,     t0,       16    // 2598
    vldrepl.w     vr21,     t0,       20    // 3166
    vldrepl.w     vr22,     t0,       24    // 1189
    vldrepl.w     vr23,     t0,       28    // 3920

    // vr18 t4a t5a     vr6 t6a t7a
    vmadd_vmsub_vssrarni_hw_12 vr7, vr8, vr19, vr10, vr20, vr21, vr21, vr20, \
                               vr22, vr23, vr23, vr22, vr11, vr18, vr9, vr6

    vsadd.h       vr12,     vr13,     vr18  // t0 t1
    vsadd.h       vr14,     vr15,     vr6   // t2 t3
    vssub.h       vr9,      vr13,     vr18  // t4 t5
    vssub.h       vr18,     vr15,     vr6   // t6 t7

    la.local      t0,       idct_coeffs

    vldrepl.w     vr20,     t0,       8     // 1567
    vldrepl.w     vr21,     t0,       12    // 3784
    vldrepl.w     vr22,     t0,       0     // 2896

    vsllwil.w.h   vr7,      vr9,      0     // t4
    vexth.w.h     vr8,      vr9             // t5
    vsllwil.w.h   vr10,     vr18,     0     // t6
    vexth.w.h     vr11,     vr18            // t7

    // vr13 out0 out7   vr17 out1 out6
    vmadd_vmsub_vssrarni_hw_12 vr7, vr8, vr11, vr10, vr21, vr20, vr20, vr21, \
                               vr20, vr21, vr21, vr20, vr13, vr15, vr18, vr19
    vshuf4i.d     vr19,     vr19,     0x01

    vsadd.h       vr13,     vr12,     vr14  // out0 out7
    vssub.h       vr6,      vr12,     vr14  // t2 t3
    vsadd.h       vr7,      vr15,     vr19  // out1 out6
    vssub.h       vr18,     vr15,     vr19  // t6 t7

    vexth.w.h     vr20,     vr13            // out7
    vsllwil.w.h   vr21,     vr7,      0     // out1
    vneg.w        vr20,     vr20
    vneg.w        vr21,     vr21
    vssrarni.h.w  vr21,     vr20,     0     // out7 out1
    vilvl.d       \out0,    vr21,     vr13  // out0 out7
    vilvh.d       \out1,    vr7,      vr21  // out1 out6

    vsllwil.w.h   vr7,      vr6,      0     // t2
    vexth.w.h     vr8,      vr6             // t3
    vsllwil.w.h   vr10,     vr18,     0     // t6
    vexth.w.h     vr11,     vr18            // t7

    // vr15 out[3] out[4]    vr18 out[2] out[5]
    vmadd_vmsub_vssrarni_hw_12 vr7, vr8, vr10, vr11, vr22, vr22, vr22, vr22, \
                               vr22, vr22, vr22, vr22, vr14, vr15, vr19, vr18

    vexth.w.h     vr20,     vr18            // out5
    vsllwil.w.h   vr21,     vr15,     0     // out3
    vneg.w        vr20,     vr20
    vneg.w        vr21,     vr21
    vssrarni.h.w  vr21,     vr20,     0     // out5 out3
    vilvl.d       \out2,    vr21,     vr18  // out2 out5
    vilvh.d       \out3,    vr15,     vr21  // out3 out4
.endm

function inv_txfm_add_adst_dct_8x8_8bpc_lsx
    addi.d        sp,       sp,       -32
    fst.d         f24,      sp,       0
    fst.d         f25,      sp,       8
    fst.d         f26,      sp,       16
    fst.d         f27,      sp,       24

    vld_x8 a2, 0, 16, vr0, vr1, vr2, vr3, vr4, vr5, vr16, vr17

    vsllwil.w.h   vr18,     vr0,      0
    vsllwil.w.h   vr19,     vr1,      0
    vsllwil.w.h   vr6,      vr2,      0
    vsllwil.w.h   vr7,      vr3,      0
    vsllwil.w.h   vr8,      vr4,      0
    vsllwil.w.h   vr9,      vr5,      0
    vsllwil.w.h   vr10,     vr16,     0
    vsllwil.w.h   vr11,     vr17,     0
    adst8x8_1d_lsx vr24, vr25, vr26, vr27

    vexth.w.h     vr18,     vr0
    vexth.w.h     vr19,     vr1
    vexth.w.h     vr6,      vr2
    vexth.w.h     vr7,      vr3
    vexth.w.h     vr8,      vr4
    vexth.w.h     vr9,      vr5
    vexth.w.h     vr10,     vr16
    vexth.w.h     vr11,     vr17
    adst8x8_1d_lsx vr0, vr1, vr2, vr3

    vreplgr2vr.h  vr23,     zero
.irp i, 0, 16, 32, 48, 64, 80, 96, 112
    vst           vr23,     a2,       \i
.endr

.irp i, vr24, vr25, vr26, vr27, vr0, vr1, vr2, vr3
    vsrari.h        \i,       \i,     1
.endr

    LSX_TRANSPOSE8x8_H vr24, vr25, vr26, vr27, vr0, vr1, vr2, vr3, \
                       vr4, vr5, vr12, vr13, vr14, vr15, vr24, vr25, \
                       vr6, vr7, vr8, vr9, vr10, vr11, vr16, vr17

    vshuf4i.h     vr14,     vr14,     0x1b
    vshuf4i.h     vr15,     vr15,     0x1b
    vshuf4i.h     vr24,     vr24,     0x1b
    vshuf4i.h     vr25,     vr25,     0x1b

    vsllwil.w.h   vr18,     vr4,      0
    vsllwil.w.h   vr19,     vr5,      0
    vsllwil.w.h   vr6,      vr12,     0
    vsllwil.w.h   vr7,      vr13,     0
    vexth.w.h     vr8,      vr4
    vexth.w.h     vr9,      vr5
    vexth.w.h     vr10,     vr12
    vexth.w.h     vr11,     vr13

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       8    // 1567
    vldrepl.w     vr21,     t0,       12   // 3784
    vldrepl.w     vr22,     t0,       0    // 2896

    dct_8x4_core_lsx1 vr4, vr5, vr12, vr13

    vshuf4i.d     vr5,      vr5,      0x01
    vshuf4i.d     vr13,     vr13,     0x01

    vsllwil.w.h   vr18,     vr14,     0
    vsllwil.w.h   vr19,     vr15,     0
    vsllwil.w.h   vr6,      vr24,     0
    vsllwil.w.h   vr7,      vr25,     0
    vexth.w.h     vr8,      vr14
    vexth.w.h     vr9,      vr15
    vexth.w.h     vr10,     vr24
    vexth.w.h     vr11,     vr25

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       8     // 1567
    vldrepl.w     vr21,     t0,       12    // 3784
    vldrepl.w     vr22,     t0,       0     // 2896

    dct_8x4_core_lsx1 vr14, vr15, vr24, vr25

    vshuf4i.d     vr15,     vr15,     0x01
    vshuf4i.d     vr25,     vr25,     0x01

    vilvl.d       vr20,     vr14,     vr4
    vilvh.d       vr21,     vr14,     vr4
    vilvl.d       vr22,     vr15,     vr5
    vilvh.d       vr23,     vr15,     vr5
    vilvl.d       vr16,     vr24,     vr12
    vilvh.d       vr17,     vr24,     vr12
    vilvl.d       vr18,     vr25,     vr13
    vilvh.d       vr19,     vr25,     vr13

.irp i, vr20, vr21, vr22, vr23, vr16, vr17, vr18, vr19
    vsrari.h      \i,       \i,       4
.endr

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr20, vr21, vr22, vr23

    alsl.d        a0,       a1,       a0,     2
    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr16, vr17, vr18, vr19

    fld.d            f24,     sp,    0
    fld.d            f25,     sp,    8
    fld.d            f26,     sp,    16
    fld.d            f27,     sp,    24
    addi.d           sp,      sp,    32
endfunc

function inv_txfm_add_dct_adst_8x8_8bpc_lsx
    addi.d        sp,       sp,       -48
    fst.d         f24,      sp,       0
    fst.d         f25,      sp,       8
    fst.d         f26,      sp,       16
    fst.d         f27,      sp,       24
    fst.d         f28,      sp,       32
    fst.d         f29,      sp,       40

    vld_x8 a2, 0, 16, vr4, vr5, vr12, vr13, vr14, vr15, vr24, vr25

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       8    // 1567
    vldrepl.w     vr21,     t0,       12   // 3784
    vldrepl.w     vr22,     t0,       0    // 2896

    vsllwil.w.h   vr18,     vr4,      0
    vsllwil.w.h   vr19,     vr5,      0
    vsllwil.w.h   vr6,      vr12,     0
    vsllwil.w.h   vr7,      vr13,     0
    vsllwil.w.h   vr8,      vr14,     0
    vsllwil.w.h   vr9,      vr15,     0
    vsllwil.w.h   vr10,     vr24,     0
    vsllwil.w.h   vr11,     vr25,     0

    dct_8x4_core_lsx1 vr26, vr27, vr28, vr29

    vshuf4i.d     vr27,     vr27,     0x01
    vshuf4i.d     vr29,     vr29,     0x01

    vilvl.h       vr8,      vr27,     vr26  // 0 2 4 6 8 10 12 14
    vilvh.h       vr9,      vr27,     vr26  // 1 3 5 7 9 11 13 15
    vilvl.h       vr26,     vr9,      vr8   // 0 - 7 in0
    vilvh.h       vr27,     vr9,      vr8   // 8 - 15 in1
    vilvl.h       vr8,      vr29,     vr28  // 0 2 4 6 8 10 12 14
    vilvh.h       vr9,      vr29,     vr28  // 1 3 5 7 9 11 13 15
    vilvl.h       vr28,     vr9,      vr8   // 16 - 23  in2
    vilvh.h       vr29,     vr9,      vr8   // 24 - 31  in3

    vsrari.h      vr26,     vr26,     1     // in0low in1low
    vsrari.h      vr27,     vr27,     1     // in2low in3low
    vsrari.h      vr28,     vr28,     1     // in0high in1high
    vsrari.h      vr29,     vr29,     1     // in2high in3high

    vexth.w.h     vr18,     vr4
    vexth.w.h     vr19,     vr5
    vexth.w.h     vr6,      vr12
    vexth.w.h     vr7,      vr13
    vexth.w.h     vr8,      vr14
    vexth.w.h     vr9,      vr15
    vexth.w.h     vr10,     vr24
    vexth.w.h     vr11,     vr25

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       8     // 1567
    vldrepl.w     vr21,     t0,       12    // 3784
    vldrepl.w     vr22,     t0,       0     // 2896

    dct_8x4_core_lsx1 vr12, vr13, vr14, vr15

    vshuf4i.d     vr13,     vr13,     0x01
    vshuf4i.d     vr15,     vr15,     0x01

    vilvl.h       vr8,      vr13,     vr12  // 0 2 4 6 8 10 12 14
    vilvh.h       vr9,      vr13,     vr12  // 1 3 5 7 9 11 13 15
    vilvl.h       vr12,     vr9,      vr8   // 0 - 7 in0
    vilvh.h       vr13,     vr9,      vr8   // 8 - 15 in1
    vilvl.h       vr8,      vr15,     vr14  // 0 2 4 6 8 10 12 14
    vilvh.h       vr9,      vr15,     vr14  // 1 3 5 7 9 11 13 15
    vilvl.h       vr14,     vr9,      vr8   // 16 - 23  in2
    vilvh.h       vr15,     vr9,      vr8   // 24 - 31  in3

    vsrari.h      vr0,      vr12,     1     // in4low in5low
    vsrari.h      vr1,      vr13,     1     // in6low in7low
    vsrari.h      vr2,      vr14,     1     // in4high in5high
    vsrari.h      vr3,      vr15,     1     // in6high in7high

    vreplgr2vr.h  vr23,     zero
.irp i, 0, 16, 32, 48, 64, 80, 96, 112
    vst           vr23,     a2,       \i
.endr

    vsllwil.w.h   vr18,     vr26,     0     // in0
    vexth.w.h     vr19,     vr26            // in1
    vsllwil.w.h   vr6,      vr27,     0     // in2
    vexth.w.h     vr7,      vr27            // in3
    vsllwil.w.h   vr8,      vr0,      0     // in3
    vexth.w.h     vr9,      vr0             // in4
    vsllwil.w.h   vr10,     vr1,      0     // in5
    vexth.w.h     vr11,     vr1             // in6
    adst8x8_1d_lsx vr26, vr27, vr0, vr1

    vsllwil.w.h   vr18,     vr28,     0     // in0
    vexth.w.h     vr19,     vr28            // in1
    vsllwil.w.h   vr6,      vr29,     0     // in2
    vexth.w.h     vr7,      vr29            // in3
    vsllwil.w.h   vr8,      vr2,      0     // in4
    vexth.w.h     vr9,      vr2             // in5
    vsllwil.w.h   vr10,     vr3,      0     // in6
    vexth.w.h     vr11,     vr3             // in7
    adst8x8_1d_lsx vr28, vr29, vr16, vr17

    vilvl.d       vr4,      vr28,     vr26  // 0 ... 7
    vilvl.d       vr5,      vr29,     vr27  // 8 ... 15
    vilvl.d       vr6,      vr16,     vr0   // 16 ... 23
    vilvl.d       vr7,      vr17,     vr1   // 24 ... 31
    vilvh.d       vr14,     vr17,     vr1   // 32 ... 39
    vilvh.d       vr15,     vr16,     vr0   // 40 ... 47
    vilvh.d       vr16,     vr29,     vr27  // 48 ... 55
    vilvh.d       vr17,     vr28,     vr26  // 56 ... 63

.irp i, vr4, vr5, vr6, vr7, vr14, vr15, vr16, vr17
    vsrari.h      \i,       \i,       4
.endr

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr4, vr5, vr6, vr7

    alsl.d        a0,       a1,       a0,     2
    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr14, vr15, vr16, vr17

    fld.d         f24,      sp,       0
    fld.d         f25,      sp,       8
    fld.d         f26,      sp,       16
    fld.d         f27,      sp,       24
    fld.d         f28,      sp,       32
    fld.d         f29,      sp,       40
    addi.d        sp,       sp,       48
endfunc

function inv_txfm_add_adst_adst_8x8_8bpc_lsx
    addi.d        sp,       sp,       -32
    fst.d         f24,      sp,       0
    fst.d         f25,      sp,       8
    fst.d         f26,      sp,       16
    fst.d         f27,      sp,       24

    vld_x8 a2, 0, 16, vr0, vr1, vr2, vr3, vr4, vr5, vr16, vr17

    vsllwil.w.h   vr18,     vr0,      0
    vsllwil.w.h   vr19,     vr1,      0
    vsllwil.w.h   vr6,      vr2,      0
    vsllwil.w.h   vr7,      vr3,      0
    vsllwil.w.h   vr8,      vr4,      0
    vsllwil.w.h   vr9,      vr5,      0
    vsllwil.w.h   vr10,     vr16,     0
    vsllwil.w.h   vr11,     vr17,     0
    adst8x8_1d_lsx vr24, vr25, vr26, vr27

    vexth.w.h     vr18,     vr0            // in0
    vexth.w.h     vr19,     vr1            // in1
    vexth.w.h     vr6,      vr2            // in2
    vexth.w.h     vr7,      vr3            // in3
    vexth.w.h     vr8,      vr4            // in3
    vexth.w.h     vr9,      vr5            // in4
    vexth.w.h     vr10,     vr16           // in5
    vexth.w.h     vr11,     vr17           // in6
    adst8x8_1d_lsx vr0, vr1, vr2, vr3

    vreplgr2vr.h  vr23,     zero
.irp i, 0, 16, 32, 48, 64, 80, 96, 112
    vst           vr23,     a2,       \i
.endr

.irp i, vr24, vr25, vr26, vr27, vr0, vr1, vr2, vr3
    vsrari.h        \i,       \i,     1
.endr

    LSX_TRANSPOSE8x8_H vr24, vr25, vr26, vr27, vr0, vr1, vr2, vr3, \
                       vr14, vr15, vr12, vr13, vr4, vr5, vr24, vr25, \
                       vr6, vr7, vr8, vr9, vr10, vr11, vr16, vr17

    vshuf4i.h     vr4,      vr4,      0x1b
    vshuf4i.h     vr5,      vr5,      0x1b
    vshuf4i.h     vr24,     vr24,     0x1b
    vshuf4i.h     vr25,     vr25,     0x1b

    vsllwil.w.h   vr18,     vr14,     0
    vsllwil.w.h   vr19,     vr15,     0
    vsllwil.w.h   vr6,      vr12,     0
    vsllwil.w.h   vr7,      vr13,     0
    vexth.w.h     vr8,      vr14            // in3
    vexth.w.h     vr9,      vr15            // in4
    vexth.w.h     vr10,     vr12            // in5
    vexth.w.h     vr11,     vr13            // in6

    adst8x8_1d_lsx vr26, vr27, vr0, vr1

    vsllwil.w.h   vr18,     vr4,     0
    vsllwil.w.h   vr19,     vr5,     0
    vsllwil.w.h   vr6,      vr24,    0
    vsllwil.w.h   vr7,      vr25,    0
    vexth.w.h     vr8,      vr4             // in3
    vexth.w.h     vr9,      vr5             // in4
    vexth.w.h     vr10,     vr24            // in5
    vexth.w.h     vr11,     vr25            // in6

    adst8x8_1d_lsx vr24, vr25, vr16, vr17

    vilvl.d       vr4,      vr24,     vr26  // 0 ... 7
    vilvl.d       vr5,      vr25,     vr27  // 8 ... 15
    vilvl.d       vr6,      vr16,     vr0   // 16 ... 23
    vilvl.d       vr7,      vr17,     vr1   // 24 ... 31
    vilvh.d       vr14,     vr17,     vr1   // 32 ... 39
    vilvh.d       vr15,     vr16,     vr0   // 40 ... 47
    vilvh.d       vr16,     vr25,     vr27  // 48 ... 55
    vilvh.d       vr17,     vr24,     vr26  // 56 ... 63

.irp i, vr4, vr5, vr6, vr7, vr14, vr15, vr16, vr17
    vsrari.h        \i,       \i,     4
.endr

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr4, vr5, vr6, vr7

    alsl.d        a0,       a1,       a0,     2
    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr14, vr15, vr16, vr17

    fld.d         f24,      sp,       0
    fld.d         f25,      sp,       8
    fld.d         f26,      sp,       16
    fld.d         f27,      sp,       24
    addi.d        sp,       sp,       32
endfunc

function inv_txfm_add_flipadst_adst_8x8_8bpc_lsx
    addi.d        sp,       sp,       -32
    fst.d         f24,      sp,       0
    fst.d         f25,      sp,       8
    fst.d         f26,      sp,       16
    fst.d         f27,      sp,       24

    vld_x8 a2, 0, 16, vr0, vr1, vr2, vr3, vr4, vr5, vr16, vr17

    vsllwil.w.h   vr18,     vr0,      0
    vsllwil.w.h   vr19,     vr1,      0
    vsllwil.w.h   vr6,      vr2,      0
    vsllwil.w.h   vr7,      vr3,      0
    vsllwil.w.h   vr8,      vr4,      0
    vsllwil.w.h   vr9,      vr5,      0
    vsllwil.w.h   vr10,     vr16,     0
    vsllwil.w.h   vr11,     vr17,     0
    adst8x8_1d_lsx vr12, vr13, vr14, vr15

    vilvl.h       vr20,     vr12,     vr13
    vilvl.h       vr21,     vr14,     vr15
    vilvl.w       vr24,     vr20,     vr21
    vilvh.w       vr25,     vr20,     vr21
    vilvh.h       vr20,     vr12,     vr13
    vilvh.h       vr21,     vr14,     vr15
    vilvl.w       vr26,     vr20,     vr21
    vilvh.w       vr27,     vr20,     vr21
    vshuf4i.h     vr26,     vr26,     0x1b
    vshuf4i.h     vr27,     vr27,     0x1b

    vexth.w.h     vr18,     vr0
    vexth.w.h     vr19,     vr1
    vexth.w.h     vr6,      vr2
    vexth.w.h     vr7,      vr3
    vexth.w.h     vr8,      vr4
    vexth.w.h     vr9,      vr5
    vexth.w.h     vr10,     vr16
    vexth.w.h     vr11,     vr17
    adst8x8_1d_lsx vr12, vr13, vr14, vr15

    vilvl.h       vr20,     vr12,     vr13
    vilvl.h       vr21,     vr14,     vr15
    vilvl.w       vr0,      vr20,     vr21
    vilvh.w       vr1,      vr20,     vr21
    vilvh.h       vr20,     vr12,     vr13
    vilvh.h       vr21,     vr14,     vr15
    vilvl.w       vr2,      vr20,     vr21
    vilvh.w       vr3,      vr20,     vr21
    vshuf4i.h     vr2,      vr2,      0x1b
    vshuf4i.h     vr3,      vr3,      0x1b

    vreplgr2vr.h  vr23,     zero
.irp i, 0, 16, 32, 48, 64, 80, 96, 112
    vst           vr23,     a2,       \i
.endr

.irp i, vr24, vr25, vr26, vr27, vr0, vr1, vr2, vr3
    vsrari.h      \i,       \i,       1
.endr

    vsllwil.w.h   vr18,     vr26,     0    // in0
    vexth.w.h     vr19,     vr26           // in1
    vsllwil.w.h   vr6,      vr27,     0    // in2
    vexth.w.h     vr7,      vr27           // in3
    vsllwil.w.h   vr8,      vr2,      0    // in4
    vexth.w.h     vr9,      vr2            // in5
    vsllwil.w.h   vr10,     vr3,      0    // in6
    vexth.w.h     vr11,     vr3            // in7
    adst8x8_1d_lsx vr4, vr5, vr16, vr17

    vsllwil.w.h   vr18,     vr24,     0    // in0
    vexth.w.h     vr19,     vr24           // in1
    vsllwil.w.h   vr6,      vr25,     0    // in2
    vexth.w.h     vr7,      vr25           // in3
    vsllwil.w.h   vr8,      vr0,      0    // in4
    vexth.w.h     vr9,      vr0            // in5
    vsllwil.w.h   vr10,     vr1,      0    // in6
    vexth.w.h     vr11,     vr1            // in7
    adst8x8_1d_lsx vr0, vr1, vr2, vr3

    vilvl.d       vr20,     vr0,     vr4   // 0 ... 7
    vilvl.d       vr21,     vr1,     vr5   // 8 ... 15
    vilvl.d       vr22,     vr2,     vr16  // 16 ... 23
    vilvl.d       vr23,     vr3,     vr17  // 24 ... 31
    vilvh.d       vr14,     vr3,     vr17  // 32 ... 39
    vilvh.d       vr15,     vr2,     vr16  // 40 ... 47
    vilvh.d       vr16,     vr1,     vr5   // 48 ... 55
    vilvh.d       vr17,     vr0,     vr4   // 56 ... 63

.irp i, vr20, vr21, vr22, vr23, vr14, vr15, vr16, vr17
    vsrari.h      \i,       \i,      4
.endr

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr20, vr21, vr22, vr23

    alsl.d        a0,       a1,       a0,     2
    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr14, vr15, vr16, vr17
    fld.d         f24,      sp,       0
    fld.d         f25,      sp,       8
    fld.d         f26,      sp,       16
    fld.d         f27,      sp,       24
    addi.d        sp,       sp,       32
endfunc

function inv_txfm_add_adst_flipadst_8x8_8bpc_lsx
    addi.d        sp,       sp,       -32
    fst.d         f24,      sp,       0
    fst.d         f25,      sp,       8
    fst.d         f26,      sp,       16
    fst.d         f27,      sp,       24

    vld_x8 a2, 0, 16, vr0, vr1, vr2, vr3, vr4, vr5, vr16, vr17

    vsllwil.w.h   vr18,     vr0,      0
    vsllwil.w.h   vr19,     vr1,      0
    vsllwil.w.h   vr6,      vr2,      0
    vsllwil.w.h   vr7,      vr3,      0
    vsllwil.w.h   vr8,      vr4,      0
    vsllwil.w.h   vr9,      vr5,      0
    vsllwil.w.h   vr10,     vr16,     0
    vsllwil.w.h   vr11,     vr17,     0
    adst8x8_1d_lsx vr24, vr25, vr26, vr27

    vexth.w.h     vr18,     vr0
    vexth.w.h     vr19,     vr1
    vexth.w.h     vr6,      vr2
    vexth.w.h     vr7,      vr3
    vexth.w.h     vr8,      vr4
    vexth.w.h     vr9,      vr5
    vexth.w.h     vr10,     vr16
    vexth.w.h     vr11,     vr17
    adst8x8_1d_lsx vr0, vr1, vr2, vr3

    vreplgr2vr.h  vr23,     zero
.irp i, 0, 16, 32, 48, 64, 80, 96, 112
    vst           vr23,     a2,       \i
.endr

.irp i, vr24, vr25, vr26, vr27, vr0, vr1, vr2, vr3
    vsrari.h      \i,       \i,       1
.endr

    LSX_TRANSPOSE8x8_H vr24, vr25, vr26, vr27, vr0, vr1, vr2, vr3, \
                       vr24, vr25, vr26, vr27, vr0, vr1, vr2, vr3, \
                       vr6, vr7, vr8, vr9, vr10, vr11, vr16, vr17

    vshuf4i.h     vr0,      vr0,      0x1b
    vshuf4i.h     vr1,      vr1,      0x1b
    vshuf4i.h     vr2,      vr2,      0x1b
    vshuf4i.h     vr3,      vr3,      0x1b

    vsllwil.w.h   vr18,     vr0,      0    // in0
    vsllwil.w.h   vr19,     vr1,      0    // in1
    vsllwil.w.h   vr6,      vr2,      0    // in2
    vsllwil.w.h   vr7,      vr3,      0    // in3
    vexth.w.h     vr8,      vr0            // in4
    vexth.w.h     vr9,      vr1            // in5
    vexth.w.h     vr10,     vr2            // in6
    vexth.w.h     vr11,     vr3            // in7
    adst8x8_1d_lsx vr4, vr5, vr16, vr17

    vsllwil.w.h   vr18,     vr24,     0    // in0
    vsllwil.w.h   vr19,     vr25,     0    // in1
    vsllwil.w.h   vr6,      vr26,     0    // in2
    vsllwil.w.h   vr7,      vr27,     0    // in3
    vexth.w.h     vr8,      vr24           // in4
    vexth.w.h     vr9,      vr25           // in5
    vexth.w.h     vr10,     vr26           // in6
    vexth.w.h     vr11,     vr27           // in7
    adst8x8_1d_lsx vr0, vr1, vr2, vr3

    vilvh.d       vr20,     vr4,      vr0
    vilvh.d       vr21,     vr5,      vr1
    vilvh.d       vr22,     vr16,     vr2
    vilvh.d       vr23,     vr17,     vr3
    vilvl.d       vr14,     vr17,     vr3
    vilvl.d       vr15,     vr16,     vr2
    vilvl.d       vr18,     vr5,      vr1
    vilvl.d       vr19,     vr4,      vr0

.irp i, vr20, vr21, vr22, vr23, vr14, vr15, vr18, vr19
    vsrari.h      \i,       \i,       4
.endr

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr20, vr21, vr22, vr23

    alsl.d        a0,       a1,       a0,     2
    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr14, vr15, vr18, vr19

    fld.d         f24,      sp,       0
    fld.d         f25,      sp,       8
    fld.d         f26,      sp,       16
    fld.d         f27,      sp,       24
    addi.d        sp,       sp,       32
endfunc

function inv_txfm_add_flipadst_dct_8x8_8bpc_lsx
    addi.d        sp,       sp,       -32
    fst.d         f24,      sp,       0
    fst.d         f25,      sp,       8
    fst.d         f26,      sp,       16
    fst.d         f27,      sp,       24

    vld_x8 a2, 0, 16, vr0, vr1, vr2, vr3, vr4, vr5, vr16, vr17

    vsllwil.w.h   vr18,     vr0,      0
    vsllwil.w.h   vr19,     vr1,      0
    vsllwil.w.h   vr6,      vr2,      0
    vsllwil.w.h   vr7,      vr3,      0
    vsllwil.w.h   vr8,      vr4,      0
    vsllwil.w.h   vr9,      vr5,      0
    vsllwil.w.h   vr10,     vr16,     0
    vsllwil.w.h   vr11,     vr17,     0
    adst8x8_1d_lsx vr12, vr13, vr14, vr15

    vilvl.h       vr20,     vr12,     vr13
    vilvl.h       vr21,     vr14,     vr15
    vilvl.w       vr24,     vr20,     vr21
    vilvh.w       vr25,     vr20,     vr21
    vilvh.h       vr20,     vr12,     vr13
    vilvh.h       vr21,     vr14,     vr15
    vilvl.w       vr26,     vr20,     vr21
    vilvh.w       vr27,     vr20,     vr21

    vexth.w.h     vr18,     vr0
    vexth.w.h     vr19,     vr1
    vexth.w.h     vr6,      vr2
    vexth.w.h     vr7,      vr3
    vexth.w.h     vr8,      vr4
    vexth.w.h     vr9,      vr5
    vexth.w.h     vr10,     vr16
    vexth.w.h     vr11,     vr17
    adst8x8_1d_lsx vr12, vr13, vr14, vr15

    vilvl.h       vr20,     vr12,     vr13
    vilvl.h       vr21,     vr14,     vr15
    vilvl.w       vr0,      vr20,     vr21
    vilvh.w       vr1,      vr20,     vr21
    vilvh.h       vr20,     vr12,     vr13
    vilvh.h       vr21,     vr14,     vr15
    vilvl.w       vr2,      vr20,     vr21
    vilvh.w       vr3,      vr20,     vr21

    vreplgr2vr.h  vr23,     zero

.irp i, 0, 16, 32, 48, 64, 80, 96, 112
    vst           vr23,     a2,       \i
.endr

    vsrari.h      vr24,     vr24,     1
    vsrari.h      vr25,     vr25,     1
    vsrari.h      vr26,     vr26,     1
    vsrari.h      vr27,     vr27,     1
    vsrari.h      vr14,     vr0,      1
    vsrari.h      vr15,     vr1,      1
    vsrari.h      vr16,     vr2,      1
    vsrari.h      vr17,     vr3,      1

    vsllwil.w.h   vr18,     vr26,     0
    vexth.w.h     vr19,     vr26
    vsllwil.w.h   vr6,      vr27,     0
    vexth.w.h     vr7,      vr27
    vsllwil.w.h   vr8,      vr16,     0
    vexth.w.h     vr9,      vr16
    vsllwil.w.h   vr10,     vr17,     0
    vexth.w.h     vr11,     vr17

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       8    // 1567
    vldrepl.w     vr21,     t0,       12   // 3784
    vldrepl.w     vr22,     t0,       0    // 2896

    dct_8x4_core_lsx1 vr26, vr27, vr16, vr17

    vshuf4i.h     vr26,     vr26,     0x1b
    vshuf4i.h     vr27,     vr27,     0x1b
    vshuf4i.h     vr16,     vr16,     0x1b
    vshuf4i.h     vr17,     vr17,     0x1b

    vsllwil.w.h   vr18,     vr24,     0
    vexth.w.h     vr19,     vr24
    vsllwil.w.h   vr6,      vr25,     0
    vexth.w.h     vr7,      vr25
    vsllwil.w.h   vr8,      vr14,     0
    vexth.w.h     vr9,      vr14
    vsllwil.w.h   vr10,     vr15,     0
    vexth.w.h     vr11,     vr15

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       8    // 1567
    vldrepl.w     vr21,     t0,       12   // 3784
    vldrepl.w     vr22,     t0,       0    // 2896

    dct_8x4_core_lsx1 vr24, vr25, vr14, vr15

    vilvl.d       vr4,      vr24,     vr26
    vilvh.d       vr5,      vr24,     vr26
    vilvh.d       vr6,      vr25,     vr27
    vilvl.d       vr7,      vr25,     vr27
    vilvl.d       vr24,     vr14,     vr16
    vilvh.d       vr25,     vr14,     vr16
    vilvh.d       vr26,     vr15,     vr17
    vilvl.d       vr27,     vr15,     vr17

.irp i, vr4, vr5, vr6, vr7, vr24, vr25, vr26, vr27
    vsrari.h      \i,       \i,       4
.endr

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr4, vr5, vr6, vr7

    alsl.d        a0,       a1,       a0,     2
    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr24, vr25, vr26, vr27

    fld.d         f24,      sp,       0
    fld.d         f25,      sp,       8
    fld.d         f26,      sp,       16
    fld.d         f27,      sp,       24
    addi.d        sp,       sp,       32
endfunc

function inv_txfm_add_dct_flipadst_8x8_8bpc_lsx
    addi.d        sp,       sp,       -48
    fst.d         f24,      sp,       0
    fst.d         f25,      sp,       8
    fst.d         f26,      sp,       16
    fst.d         f27,      sp,       24
    fst.d         f28,      sp,       32
    fst.d         f29,      sp,       40

    vld_x8 a2, 0, 16, vr4, vr5, vr12, vr13, vr14, vr15, vr24, vr25

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       8    // 1567
    vldrepl.w     vr21,     t0,       12   // 3784
    vldrepl.w     vr22,     t0,       0    // 2896

    vsllwil.w.h   vr18,     vr4,      0
    vsllwil.w.h   vr19,     vr5,      0
    vsllwil.w.h   vr6,      vr12,     0
    vsllwil.w.h   vr7,      vr13,     0
    vsllwil.w.h   vr8,      vr14,     0
    vsllwil.w.h   vr9,      vr15,     0
    vsllwil.w.h   vr10,     vr24,     0
    vsllwil.w.h   vr11,     vr25,     0
    dct_8x4_core_lsx1 vr26, vr27, vr28, vr29
    vshuf4i.d     vr27,     vr27,     0x01
    vshuf4i.d     vr29,     vr29,     0x01

    vilvl.h       vr8,      vr27,     vr26
    vilvh.h       vr9,      vr27,     vr26
    vilvl.h       vr26,     vr9,      vr8
    vilvh.h       vr27,     vr9,      vr8
    vilvl.h       vr8,      vr29,     vr28
    vilvh.h       vr9,      vr29,     vr28
    vilvl.h       vr28,     vr9,      vr8
    vilvh.h       vr29,     vr9,      vr8

    vsrari.h      vr26,     vr26,     1     // in0low in1low
    vsrari.h      vr27,     vr27,     1     // in2low in3low
    vsrari.h      vr28,     vr28,     1     // in0high in1high
    vsrari.h      vr29,     vr29,     1     // in2high in3high

    vexth.w.h     vr18,     vr4
    vexth.w.h     vr19,     vr5
    vexth.w.h     vr6,      vr12
    vexth.w.h     vr7,      vr13
    vexth.w.h     vr8,      vr14
    vexth.w.h     vr9,      vr15
    vexth.w.h     vr10,     vr24
    vexth.w.h     vr11,     vr25
    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       8    // 1567
    vldrepl.w     vr21,     t0,       12   // 3784
    vldrepl.w     vr22,     t0,       0    // 2896
    dct_8x4_core_lsx1 vr12, vr13, vr14, vr15
    vshuf4i.d     vr13,     vr13,     0x01
    vshuf4i.d     vr15,     vr15,     0x01

    vilvl.h       vr8,      vr13,     vr12
    vilvh.h       vr9,      vr13,     vr12
    vilvl.h       vr12,     vr9,      vr8
    vilvh.h       vr13,     vr9,      vr8
    vilvl.h       vr8,      vr15,     vr14
    vilvh.h       vr9,      vr15,     vr14
    vilvl.h       vr14,     vr9,      vr8
    vilvh.h       vr15,     vr9,      vr8

    vsrari.h      vr0,      vr12,     1
    vsrari.h      vr1,      vr13,     1
    vsrari.h      vr2,      vr14,     1
    vsrari.h      vr3,      vr15,     1

    vreplgr2vr.h  vr23,     zero
.irp i, 0, 16, 32, 48, 64, 80, 96, 112
    vst           vr23,     a2,       \i
.endr

    vsllwil.w.h   vr18,     vr28,     0    // in0
    vexth.w.h     vr19,     vr28           // in1
    vsllwil.w.h   vr6,      vr29,     0    // in2
    vexth.w.h     vr7,      vr29           // in3
    vsllwil.w.h   vr8,      vr2,      0    // in4
    vexth.w.h     vr9,      vr2            // in5
    vsllwil.w.h   vr10,     vr3,      0    // in6
    vexth.w.h     vr11,     vr3            // in7
    adst8x8_1d_lsx vr4, vr5, vr16, vr17

    vsllwil.w.h   vr18,     vr26,     0    // in0
    vexth.w.h     vr19,     vr26           // in1
    vsllwil.w.h   vr6,      vr27,     0    // in2
    vexth.w.h     vr7,      vr27           // in3
    vsllwil.w.h   vr8,      vr0,      0    // in4
    vexth.w.h     vr9,      vr0            // in5
    vsllwil.w.h   vr10,     vr1,      0    // in6
    vexth.w.h     vr11,     vr1            // in7
    adst8x8_1d_lsx vr0, vr1, vr2, vr3

    vilvh.d       vr26,     vr4,      vr0
    vilvh.d       vr27,     vr5,      vr1
    vilvh.d       vr28,     vr16,     vr2
    vilvh.d       vr29,     vr17,     vr3
    vilvl.d       vr20,     vr17,     vr3
    vilvl.d       vr21,     vr16,     vr2
    vilvl.d       vr22,     vr5,      vr1
    vilvl.d       vr23,     vr4,      vr0

.irp i, vr26, vr27, vr28, vr29, vr20, vr21, vr22, vr23
    vsrari.h      \i,       \i,       4
.endr

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr26, vr27, vr28, vr29

    alsl.d        a0,       a1,       a0,     2
    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr20, vr21, vr22, vr23

    fld.d         f24,      sp,       0
    fld.d         f25,      sp,       8
    fld.d         f26,      sp,       16
    fld.d         f27,      sp,       24
    fld.d         f28,      sp,       32
    fld.d         f29,      sp,       40
    addi.d        sp,       sp,       48
endfunc

function inv_txfm_add_flipadst_flipadst_8x8_8bpc_lsx
    addi.d        sp,       sp,       -32
    fst.d         f24,      sp,       0
    fst.d         f25,      sp,       8
    fst.d         f26,      sp,       16
    fst.d         f27,      sp,       24

    vld_x8 a2, 0, 16, vr0, vr1, vr2, vr3, vr4, vr5, vr16, vr17

    vsllwil.w.h   vr18,     vr0,      0
    vsllwil.w.h   vr19,     vr1,      0
    vsllwil.w.h   vr6,      vr2,      0
    vsllwil.w.h   vr7,      vr3,      0
    vsllwil.w.h   vr8,      vr4,      0
    vsllwil.w.h   vr9,      vr5,      0
    vsllwil.w.h   vr10,     vr16,     0
    vsllwil.w.h   vr11,     vr17,     0
    adst8x8_1d_lsx vr12, vr13, vr14, vr15

    vilvl.h       vr20,     vr12,     vr13
    vilvl.h       vr21,     vr14,     vr15
    vilvl.w       vr24,     vr20,     vr21
    vilvh.w       vr25,     vr20,     vr21
    vilvh.h       vr20,     vr12,     vr13
    vilvh.h       vr21,     vr14,     vr15
    vilvl.w       vr26,     vr20,     vr21
    vilvh.w       vr27,     vr20,     vr21
    vshuf4i.h     vr26,     vr26,     0x1b
    vshuf4i.h     vr27,     vr27,     0x1b

    vexth.w.h     vr18,     vr0
    vexth.w.h     vr19,     vr1
    vexth.w.h     vr6,      vr2
    vexth.w.h     vr7,      vr3
    vexth.w.h     vr8,      vr4
    vexth.w.h     vr9,      vr5
    vexth.w.h     vr10,     vr16
    vexth.w.h     vr11,     vr17
    adst8x8_1d_lsx vr12, vr13, vr14, vr15

    vilvl.h       vr20,     vr12,     vr13
    vilvl.h       vr21,     vr14,     vr15
    vilvl.w       vr0,      vr20,     vr21
    vilvh.w       vr1,      vr20,     vr21
    vilvh.h       vr20,     vr12,     vr13
    vilvh.h       vr21,     vr14,     vr15
    vilvl.w       vr2,      vr20,     vr21
    vilvh.w       vr3,      vr20,     vr21
    vshuf4i.h     vr2,      vr2,      0x1b
    vshuf4i.h     vr3,      vr3,      0x1b

.irp i, vr24, vr25, vr26, vr27, vr0, vr1, vr2, vr3
    vsrari.h      \i,       \i,       1
.endr

    vreplgr2vr.h  vr23,     zero
.irp i, 0, 16, 32, 48, 64, 80, 96, 112
    vst           vr23,     a2,       \i
.endr

    vsllwil.w.h   vr18,     vr26,     0    // in0
    vexth.w.h     vr19,     vr26           // in1
    vsllwil.w.h   vr6,      vr27,     0    // in2
    vexth.w.h     vr7,      vr27           // in3
    vsllwil.w.h   vr8,      vr2,      0    // in4
    vexth.w.h     vr9,      vr2            // in5
    vsllwil.w.h   vr10,     vr3,      0    // in6
    vexth.w.h     vr11,     vr3            // in7
    adst8x8_1d_lsx vr4, vr5, vr16, vr17

    vsllwil.w.h   vr18,     vr24,     0    // in0
    vexth.w.h     vr19,     vr24           // in1
    vsllwil.w.h   vr6,      vr25,     0    // in2
    vexth.w.h     vr7,      vr25           // in3
    vsllwil.w.h   vr8,      vr0,      0    // in4
    vexth.w.h     vr9,      vr0            // in5
    vsllwil.w.h   vr10,     vr1,      0    // in6
    vexth.w.h     vr11,     vr1            // in7
    adst8x8_1d_lsx vr0, vr1, vr2, vr3

    vilvh.d       vr24,     vr0,      vr4
    vilvh.d       vr25,     vr1,      vr5
    vilvh.d       vr26,     vr2,      vr16
    vilvh.d       vr27,     vr3,      vr17
    vilvl.d       vr20,     vr3,      vr17
    vilvl.d       vr21,     vr2,      vr16
    vilvl.d       vr22,     vr1,      vr5
    vilvl.d       vr23,     vr0,      vr4

.irp i, vr24, vr25, vr26, vr27, vr20, vr21, vr22, vr23
    vsrari.h      \i,       \i,       4
.endr

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr24, vr25, vr26, vr27

    alsl.d        a0,       a1,       a0,     2
    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr20, vr21, vr22, vr23

    fld.d         f24,      sp,       0
    fld.d         f25,      sp,       8
    fld.d         f26,      sp,       16
    fld.d         f27,      sp,       24
    addi.d        sp,       sp,       32
endfunc

function inv_txfm_add_dct_identity_8x8_8bpc_lsx
    addi.d        sp,       sp,       -48
    fst.d         f24,      sp,       0
    fst.d         f25,      sp,       8
    fst.d         f26,      sp,       16
    fst.d         f27,      sp,       24
    fst.d         f28,      sp,       32
    fst.d         f29,      sp,       40

    vld_x8 a2, 0, 16, vr4, vr5, vr12, vr13, vr14, vr15, vr24, vr25

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       8    // 1567
    vldrepl.w     vr21,     t0,       12   // 3784
    vldrepl.w     vr22,     t0,       0    // 2896

    vsllwil.w.h   vr18,     vr4,      0
    vsllwil.w.h   vr19,     vr5,      0
    vsllwil.w.h   vr6,      vr12,     0
    vsllwil.w.h   vr7,      vr13,     0
    vsllwil.w.h   vr8,      vr14,     0
    vsllwil.w.h   vr9,      vr15,     0
    vsllwil.w.h   vr10,     vr24,     0
    vsllwil.w.h   vr11,     vr25,     0
    dct_8x4_core_lsx1 vr26, vr27, vr28, vr29
    vshuf4i.d     vr27,     vr27,     0x01
    vshuf4i.d     vr29,     vr29,     0x01

    vilvl.h       vr8,      vr27,     vr26
    vilvh.h       vr9,      vr27,     vr26
    vilvl.h       vr26,     vr9,      vr8
    vilvh.h       vr27,     vr9,      vr8
    vilvl.h       vr8,      vr29,     vr28
    vilvh.h       vr9,      vr29,     vr28
    vilvl.h       vr28,     vr9,      vr8
    vilvh.h       vr29,     vr9,      vr8

    vsrari.h      vr26,     vr26,     1     // in0low in1low
    vsrari.h      vr27,     vr27,     1     // in2low in3low
    vsrari.h      vr28,     vr28,     1     // in0high in1high
    vsrari.h      vr29,     vr29,     1     // in2high in3high

    vexth.w.h     vr18,     vr4
    vexth.w.h     vr19,     vr5
    vexth.w.h     vr6,      vr12
    vexth.w.h     vr7,      vr13
    vexth.w.h     vr8,      vr14
    vexth.w.h     vr9,      vr15
    vexth.w.h     vr10,     vr24
    vexth.w.h     vr11,     vr25

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       8    // 1567
    vldrepl.w     vr21,     t0,       12   // 3784
    vldrepl.w     vr22,     t0,       0    // 2896

    dct_8x4_core_lsx1 vr12, vr13, vr14, vr15

    vshuf4i.d     vr13,     vr13,     0x01
    vshuf4i.d     vr15,     vr15,     0x01

    vilvl.h       vr8,      vr13,     vr12
    vilvh.h       vr9,      vr13,     vr12
    vilvl.h       vr12,     vr9,      vr8
    vilvh.h       vr13,     vr9,      vr8
    vilvl.h       vr8,      vr15,     vr14
    vilvh.h       vr9,      vr15,     vr14
    vilvl.h       vr14,     vr9,      vr8
    vilvh.h       vr15,     vr9,      vr8

    vsrari.h      vr20,     vr12,     1
    vsrari.h      vr21,     vr13,     1
    vsrari.h      vr22,     vr14,     1
    vsrari.h      vr23,     vr15,     1

    vreplgr2vr.h  vr19,     zero
.irp i, 0, 16, 32, 48, 64, 80, 96, 112
    vst           vr19,     a2,       \i
.endr
    // identity8
    vsllwil.w.h   vr10,     vr26,     1
    vsllwil.w.h   vr11,     vr27,     1
    vsllwil.w.h   vr16,     vr28,     1
    vsllwil.w.h   vr17,     vr29,     1
    vsllwil.w.h   vr6,      vr20,     1
    vsllwil.w.h   vr7,      vr21,     1
    vsllwil.w.h   vr18,     vr22,     1
    vsllwil.w.h   vr19,     vr23,     1

.irp i, vr26, vr27, vr28, vr29, vr20, vr21, vr22, vr23
    vexth.w.h     \i,       \i
.endr

.irp i, vr26, vr27, vr28, vr29, vr20, vr21, vr22, vr23
    vslli.w       \i,       \i,       1
.endr

    vssrarni.h.w  vr16,     vr10,     4   // in0
    vssrarni.h.w  vr28,     vr26,     4   // in1
    vssrarni.h.w  vr17,     vr11,     4   // in2
    vssrarni.h.w  vr29,     vr27,     4   // in3
    vssrarni.h.w  vr18,     vr6,      4   // in4
    vssrarni.h.w  vr22,     vr20,     4   // in5
    vssrarni.h.w  vr19,     vr7,      4   // in6
    vssrarni.h.w  vr23,     vr21,     4   // in7

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr16, vr28, vr17, vr29

    alsl.d        a0,       a1,       a0,     2
    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr18, vr22, vr19, vr23

    fld.d         f24,      sp,       0
    fld.d         f25,      sp,       8
    fld.d         f26,      sp,       16
    fld.d         f27,      sp,       24
    fld.d         f28,      sp,       32
    fld.d         f29,      sp,       40
    addi.d        sp,       sp,       48
endfunc

function inv_txfm_add_identity_dct_8x8_8bpc_lsx
    addi.d        sp,       sp,       -48
    fst.d         f24,      sp,       0
    fst.d         f25,      sp,       8
    fst.d         f26,      sp,       16
    fst.d         f27,      sp,       24
    fst.d         f28,      sp,       32
    fst.d         f29,      sp,       40

    vld_x8 a2, 0, 16, vr0, vr1, vr2, vr3, vr4, vr5, vr24, vr25

    // identity8
    vsllwil.w.h   vr6,      vr0,      1
    vsllwil.w.h   vr7,      vr1,      1
    vsllwil.w.h   vr8,      vr2,      1
    vsllwil.w.h   vr9,      vr3,      1
    vsllwil.w.h   vr10,     vr4,      1
    vsllwil.w.h   vr11,     vr5,      1
    vsllwil.w.h   vr12,     vr24,     1
    vsllwil.w.h   vr13,     vr25,     1

.irp i, vr0, vr1, vr2, vr3, vr4, vr5, vr24, vr25
    vexth.w.h     \i,       \i
.endr

.irp i, vr0, vr1, vr2, vr3, vr4, vr5, vr24, vr25
    vslli.w       \i,       \i,       1
.endr
    vssrarni.h.w  vr0,      vr6,      1   // in0
    vssrarni.h.w  vr1,      vr7,      1   // in1
    vssrarni.h.w  vr2,      vr8,      1   // in2
    vssrarni.h.w  vr3,      vr9,      1   // in3
    vssrarni.h.w  vr4,      vr10,     1   // in4
    vssrarni.h.w  vr5,      vr11,     1   // in5
    vssrarni.h.w  vr24,     vr12,     1   // in6
    vssrarni.h.w  vr25,     vr13,     1   // in7

    vreplgr2vr.h  vr23,     zero
.irp i, 0, 16, 32, 48, 64, 80, 96, 112
    vst           vr23,     a2,       \i
.endr

    LSX_TRANSPOSE8x8_H vr0, vr1, vr2, vr3, vr4, vr5, vr24, vr25, \
                       vr4, vr5, vr12, vr13, vr14, vr15, vr24, vr25, \
                       vr6, vr7, vr8, vr9, vr10, vr11, vr16, vr17

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       8    // 1567
    vldrepl.w     vr21,     t0,       12   // 3784
    vldrepl.w     vr22,     t0,       0    // 2896

    // dct4 in0 in2 in4 in6
    vsllwil.w.h   vr18,     vr4,      0
    vsllwil.w.h   vr19,     vr5,      0
    vsllwil.w.h   vr6,      vr12,     0
    vsllwil.w.h   vr7,      vr13,     0
    vsllwil.w.h   vr8,      vr14,     0
    vsllwil.w.h   vr9,      vr15,     0
    vsllwil.w.h   vr10,     vr24,     0
    vsllwil.w.h   vr11,     vr25,     0
    dct_8x4_core_lsx1 vr16, vr17, vr26, vr27

    vexth.w.h     vr18,     vr4
    vexth.w.h     vr19,     vr5
    vexth.w.h     vr6,      vr12
    vexth.w.h     vr7,      vr13
    vexth.w.h     vr8,      vr14
    vexth.w.h     vr9,      vr15
    vexth.w.h     vr10,     vr24
    vexth.w.h     vr11,     vr25

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       8     // 1567
    vldrepl.w     vr21,     t0,       12    // 3784
    vldrepl.w     vr22,     t0,       0     // 2896
    dct_8x4_core_lsx1 vr4, vr5, vr24, vr25

    vilvl.d       vr8,      vr4,      vr16
    vilvh.d       vr9,      vr4,      vr16
    vilvh.d       vr6,      vr5,      vr17
    vilvl.d       vr7,      vr5,      vr17
    vilvl.d       vr16,     vr24,     vr26
    vilvh.d       vr17,     vr24,     vr26
    vilvh.d       vr18,     vr25,     vr27
    vilvl.d       vr19,     vr25,     vr27

.irp i, vr8, vr9, vr6, vr7, vr16, vr17, vr18, vr19
    vsrari.h      \i,       \i,       4
.endr

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr8, vr9, vr6, vr7

    alsl.d        a0,       a1,       a0,     2
    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr16, vr17, vr18, vr19

    fld.d         f24,      sp,       0
    fld.d         f25,      sp,       8
    fld.d         f26,      sp,       16
    fld.d         f27,      sp,       24
    fld.d         f28,      sp,       32
    fld.d         f29,      sp,       40
    addi.d        sp,       sp,       48
endfunc

function inv_txfm_add_flipadst_identity_8x8_8bpc_lsx
    addi.d        sp,       sp,       -32
    fst.d         f24,      sp,       0
    fst.d         f25,      sp,       8
    fst.d         f26,      sp,       16
    fst.d         f27,      sp,       24

    vld_x8 a2, 0, 16, vr0, vr1, vr2, vr3, vr4, vr5, vr16, vr17

    vsllwil.w.h   vr18,     vr0,      0
    vsllwil.w.h   vr19,     vr1,      0
    vsllwil.w.h   vr6,      vr2,      0
    vsllwil.w.h   vr7,      vr3,      0
    vsllwil.w.h   vr8,      vr4,      0
    vsllwil.w.h   vr9,      vr5,      0
    vsllwil.w.h   vr10,     vr16,     0
    vsllwil.w.h   vr11,     vr17,     0
    adst8x8_1d_lsx vr12, vr13, vr14, vr15

    vilvl.h       vr20,     vr12,     vr13
    vilvl.h       vr21,     vr14,     vr15
    vilvl.w       vr24,     vr20,     vr21
    vilvh.w       vr25,     vr20,     vr21
    vilvh.h       vr20,     vr12,     vr13
    vilvh.h       vr21,     vr14,     vr15
    vilvl.w       vr26,     vr20,     vr21
    vilvh.w       vr27,     vr20,     vr21
    vshuf4i.h     vr26,     vr26,     0x1b
    vshuf4i.h     vr27,     vr27,     0x1b

    vexth.w.h     vr18,     vr0            // in0
    vexth.w.h     vr19,     vr1            // in1
    vexth.w.h     vr6,      vr2            // in2
    vexth.w.h     vr7,      vr3            // in3
    vexth.w.h     vr8,      vr4            // in3
    vexth.w.h     vr9,      vr5            // in4
    vexth.w.h     vr10,     vr16           // in5
    vexth.w.h     vr11,     vr17           // in6
    adst8x8_1d_lsx vr12, vr13, vr14, vr15

    vilvl.h       vr20,     vr12,     vr13
    vilvl.h       vr21,     vr14,     vr15
    vilvl.w       vr16,     vr20,     vr21
    vilvh.w       vr17,     vr20,     vr21
    vilvh.h       vr20,     vr12,     vr13
    vilvh.h       vr21,     vr14,     vr15
    vilvl.w       vr18,     vr20,     vr21
    vilvh.w       vr19,     vr20,     vr21
    vshuf4i.h     vr18,     vr18,     0x1b
    vshuf4i.h     vr19,     vr19,     0x1b

    vreplgr2vr.h  vr23,     zero
.irp i, 0, 16, 32, 48, 64, 80, 96, 112
    vst           vr23,     a2,       \i
.endr

.irp i, vr24, vr25, vr26, vr27, vr16, vr17, vr18, vr19
    vsrari.h      \i,       \i,       1
.endr

    // identity8
    vsllwil.w.h   vr20,     vr24,     1
    vsllwil.w.h   vr21,     vr25,     1
    vsllwil.w.h   vr12,     vr26,     1
    vsllwil.w.h   vr13,     vr27,     1
    vsllwil.w.h   vr22,     vr16,     1
    vsllwil.w.h   vr23,     vr17,     1
    vsllwil.w.h   vr14,     vr18,     1
    vsllwil.w.h   vr15,     vr19,     1

.irp i, vr24, vr25, vr26, vr27, vr16, vr17, vr18, vr19
    vexth.w.h     \i,       \i
.endr

.irp i, vr24, vr25, vr26, vr27, vr16, vr17, vr18, vr19
     vslli.w      \i,       \i,       1
.endr

    vssrarni.h.w  vr20,     vr12,     4   // in0
    vssrarni.h.w  vr24,     vr26,     4   // in1
    vssrarni.h.w  vr21,     vr13,     4   // in2
    vssrarni.h.w  vr25,     vr27,     4   // in3
    vssrarni.h.w  vr22,     vr14,     4   // in4
    vssrarni.h.w  vr16,     vr18,     4   // in5
    vssrarni.h.w  vr23,     vr15,     4   // in6
    vssrarni.h.w  vr17,     vr19,     4   // in7

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr20, vr24, vr21, vr25

    alsl.d        a0,       a1,       a0,     2
    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr22, vr16, vr23, vr17

    fld.d         f24,      sp,       0
    fld.d         f25,      sp,       8
    fld.d         f26,      sp,       16
    fld.d         f27,      sp,       24
    addi.d        sp,       sp,       32
endfunc

function inv_txfm_add_identity_flipadst_8x8_8bpc_lsx
    addi.d        sp,       sp,       -48
    fst.d         f24,      sp,       0
    fst.d         f25,      sp,       8
    fst.d         f26,      sp,       16
    fst.d         f27,      sp,       24
    fst.d         f28,      sp,       32
    fst.d         f29,      sp,       40

    vld_x8 a2, 0, 16, vr0, vr1, vr2, vr3, vr4, vr5, vr24, vr25

    // identity8
    vsllwil.w.h   vr6,      vr0,      1
    vsllwil.w.h   vr7,      vr1,      1
    vsllwil.w.h   vr8,      vr2,      1
    vsllwil.w.h   vr9,      vr3,      1
    vsllwil.w.h   vr10,     vr4,      1
    vsllwil.w.h   vr11,     vr5,      1
    vsllwil.w.h   vr12,     vr24,     1
    vsllwil.w.h   vr13,     vr25,     1

.irp i, vr0, vr1, vr2, vr3, vr4, vr5, vr24, vr25
    vexth.w.h     \i,       \i
.endr

.irp i, vr0, vr1, vr2, vr3, vr4, vr5, vr24, vr25
    vslli.w       \i,       \i,       1
.endr

    vssrarni.h.w  vr0,      vr6,      1   // in0
    vssrarni.h.w  vr1,      vr7,      1   // in1
    vssrarni.h.w  vr2,      vr8,      1   // in2
    vssrarni.h.w  vr3,      vr9,      1   // in3
    vssrarni.h.w  vr4,      vr10,     1   // in4
    vssrarni.h.w  vr5,      vr11,     1   // in5
    vssrarni.h.w  vr24,     vr12,     1   // in6
    vssrarni.h.w  vr25,     vr13,     1   // in7

    LSX_TRANSPOSE8x8_H vr0, vr1, vr2, vr3, vr4, vr5, vr24, vr25, \
                       vr0, vr1, vr2, vr3, vr4, vr5, vr24, vr25, \
                       vr6, vr7, vr8, vr9, vr10, vr11, vr12, vr13

    vreplgr2vr.h  vr23,     zero
.irp i, 0, 16, 32, 48, 64, 80, 96, 112
    vst           vr23,     a2,       \i
.endr

    vsllwil.w.h   vr18,     vr0,      0    // in0
    vsllwil.w.h   vr19,     vr1,      0    // in1
    vsllwil.w.h   vr6,      vr2,      0    // in2
    vsllwil.w.h   vr7,      vr3,      0    // in3
    vsllwil.w.h   vr8,      vr4,      0    // in3
    vsllwil.w.h   vr9,      vr5,      0    // in4
    vsllwil.w.h   vr10,     vr24,     0    // in5
    vsllwil.w.h   vr11,     vr25,     0    // in6
    adst8x8_1d_lsx vr26, vr27, vr28, vr29

    vexth.w.h     vr18,     vr0            // in0
    vexth.w.h     vr19,     vr1            // in1
    vexth.w.h     vr6,      vr2            // in2
    vexth.w.h     vr7,      vr3            // in3
    vexth.w.h     vr8,      vr4            // in3
    vexth.w.h     vr9,      vr5            // in4
    vexth.w.h     vr10,     vr24           // in5
    vexth.w.h     vr11,     vr25           // in6
    adst8x8_1d_lsx vr0, vr1, vr2, vr3

    vilvh.d       vr4,      vr0,      vr26
    vilvh.d       vr5,      vr1,      vr27
    vilvh.d       vr6,      vr2,      vr28
    vilvh.d       vr7,      vr3,      vr29
    vilvl.d       vr14,     vr3,      vr29
    vilvl.d       vr15,     vr2,      vr28
    vilvl.d       vr16,     vr1,      vr27
    vilvl.d       vr17,     vr0,      vr26

.irp i, vr4, vr5, vr6, vr7, vr14, vr15, vr16, vr17
    vsrari.h      \i,        \i,      4
.endr

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr4, vr5, vr6, vr7

    alsl.d        a0,       a1,       a0,     2
    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr14, vr15, vr16, vr17

    fld.d         f24,      sp,       0
    fld.d         f25,      sp,       8
    fld.d         f26,      sp,       16
    fld.d         f27,      sp,       24
    fld.d         f28,      sp,       32
    fld.d         f29,      sp,       40
    addi.d        sp,       sp,       48

endfunc

function inv_txfm_add_adst_identity_8x8_8bpc_lsx
    addi.d        sp,       sp,       -32
    fst.d         f24,      sp,       0
    fst.d         f25,      sp,       8
    fst.d         f26,      sp,       16
    fst.d         f27,      sp,       24

    vld_x8 a2, 0, 16, vr0, vr1, vr2, vr3, vr4, vr5, vr16, vr17

    vsllwil.w.h   vr18,     vr0,      0
    vsllwil.w.h   vr19,     vr1,      0
    vsllwil.w.h   vr6,      vr2,      0
    vsllwil.w.h   vr7,      vr3,      0
    vsllwil.w.h   vr8,      vr4,      0
    vsllwil.w.h   vr9,      vr5,      0
    vsllwil.w.h   vr10,     vr16,     0
    vsllwil.w.h   vr11,     vr17,     0
    adst8x8_1d_lsx vr24, vr25, vr26, vr27

    vexth.w.h     vr18,     vr0
    vexth.w.h     vr19,     vr1
    vexth.w.h     vr6,      vr2
    vexth.w.h     vr7,      vr3
    vexth.w.h     vr8,      vr4
    vexth.w.h     vr9,      vr5
    vexth.w.h     vr10,     vr16
    vexth.w.h     vr11,     vr17
    adst8x8_1d_lsx vr0, vr1, vr2, vr3

    vreplgr2vr.h  vr23,     zero
.irp i, 0, 16, 32, 48, 64, 80, 96, 112
    vst           vr23,     a2,       \i
.endr

.irp i, vr24, vr25, vr26, vr27, vr0, vr1, vr2, vr3
    vsrari.h      \i,       \i,       1
.endr

    LSX_TRANSPOSE8x8_H vr24, vr25, vr26, vr27, vr0, vr1, vr2, vr3, \
                       vr24, vr25, vr20, vr21, vr26, vr27, vr22, vr23, \
                       vr6, vr7, vr8, vr9, vr10, vr11, vr16, vr17

    vshuf4i.h     vr26,     vr26,     0x1b
    vshuf4i.h     vr27,     vr27,     0x1b
    vshuf4i.h     vr22,     vr22,     0x1b
    vshuf4i.h     vr23,     vr23,     0x1b

    // identity8
    vsllwil.w.h   vr16,     vr24,     1
    vsllwil.w.h   vr17,     vr25,     1
    vsllwil.w.h   vr10,     vr20,     1
    vsllwil.w.h   vr11,     vr21,     1
    vsllwil.w.h   vr18,     vr26,     1
    vsllwil.w.h   vr19,     vr27,     1
    vsllwil.w.h   vr14,     vr22,     1
    vsllwil.w.h   vr15,     vr23,     1

.irp i, vr24, vr25, vr20, vr21, vr26, vr27, vr22, vr23
    vexth.w.h     \i,       \i
.endr

.irp i, vr24, vr25, vr20, vr21, vr26, vr27, vr22, vr23
    vslli.w       \i,       \i,       1
.endr

    vssrarni.h.w  vr18,     vr16,     4    // in0
    vssrarni.h.w  vr19,     vr17,     4    // in1
    vssrarni.h.w  vr14,     vr10,     4    // in2
    vssrarni.h.w  vr15,     vr11,     4    // in3
    vssrarni.h.w  vr26,     vr24,     4    // in4
    vssrarni.h.w  vr27,     vr25,     4    // in5
    vssrarni.h.w  vr22,     vr20,     4    // in6
    vssrarni.h.w  vr23,     vr21,     4    // in7

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr18, vr19, vr14, vr15

    alsl.d        a0,       a1,       a0,     2
    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr26, vr27, vr22, vr23

    fld.d         f24,      sp,       0
    fld.d         f25,      sp,       8
    fld.d         f26,      sp,       16
    fld.d         f27,      sp,       24
    addi.d        sp,       sp,       32
endfunc

function inv_txfm_add_identity_adst_8x8_8bpc_lsx
    addi.d        sp,       sp,       -48
    fst.d         f24,      sp,       0
    fst.d         f25,      sp,       8
    fst.d         f26,      sp,       16
    fst.d         f27,      sp,       24
    fst.d         f28,      sp,       32
    fst.d         f29,      sp,       40

    vld_x8 a2, 0, 16, vr0, vr1, vr2, vr3, vr4, vr5, vr24, vr25

    // identity8
    vsllwil.w.h   vr6,      vr0,      1
    vsllwil.w.h   vr7,      vr1,      1
    vsllwil.w.h   vr8,      vr2,      1
    vsllwil.w.h   vr9,      vr3,      1
    vsllwil.w.h   vr10,     vr4,      1
    vsllwil.w.h   vr11,     vr5,      1
    vsllwil.w.h   vr12,     vr24,     1
    vsllwil.w.h   vr13,     vr25,     1

.irp i, vr0, vr1, vr2, vr3, vr4, vr5, vr24, vr25
    vexth.w.h     \i,       \i
.endr

.irp i, vr0, vr1, vr2, vr3, vr4, vr5, vr24, vr25
    vslli.w       \i,       \i,       1
.endr

    vssrarni.h.w  vr0,      vr6,      1   // in0
    vssrarni.h.w  vr1,      vr7,      1   // in1
    vssrarni.h.w  vr2,      vr8,      1   // in2
    vssrarni.h.w  vr3,      vr9,      1   // in3
    vssrarni.h.w  vr4,      vr10,     1   // in4
    vssrarni.h.w  vr5,      vr11,     1   // in5
    vssrarni.h.w  vr24,     vr12,     1   // in6
    vssrarni.h.w  vr25,     vr13,     1   // in7

    LSX_TRANSPOSE8x8_H vr0, vr1, vr2, vr3, vr4, vr5, vr24, vr25, \
                       vr0, vr1, vr2, vr3, vr4, vr5, vr24, vr25, \
                       vr6, vr7, vr8, vr9, vr10, vr11, vr12, vr13

    vreplgr2vr.h  vr23,     zero

.irp i, 0, 16, 32, 48, 64, 80, 96, 112
    vst           vr23,     a2,       \i
.endr

    vsllwil.w.h   vr18,     vr0,      0
    vsllwil.w.h   vr19,     vr1,      0
    vsllwil.w.h   vr6,      vr2,      0
    vsllwil.w.h   vr7,      vr3,      0
    vsllwil.w.h   vr8,      vr4,      0
    vsllwil.w.h   vr9,      vr5,      0
    vsllwil.w.h   vr10,     vr24,     0
    vsllwil.w.h   vr11,     vr25,     0
    adst8x8_1d_lsx vr26, vr27, vr28, vr29

    vexth.w.h     vr18,     vr0
    vexth.w.h     vr19,     vr1
    vexth.w.h     vr6,      vr2
    vexth.w.h     vr7,      vr3
    vexth.w.h     vr8,      vr4
    vexth.w.h     vr9,      vr5
    vexth.w.h     vr10,     vr24
    vexth.w.h     vr11,     vr25

    adst8x8_1d_lsx vr0, vr1, vr2, vr3

    vilvl.d       vr4,      vr0,      vr26  // 0 ... 7
    vilvl.d       vr5,      vr1,      vr27  // 8 ... 15
    vilvl.d       vr6,      vr2,      vr28  // 16 ... 23
    vilvl.d       vr7,      vr3,      vr29  // 24 ... 31
    vilvh.d       vr14,     vr3,      vr29  // 32 ... 39
    vilvh.d       vr15,     vr2,      vr28  // 40 ... 47
    vilvh.d       vr16,     vr1,      vr27  // 48 ... 55
    vilvh.d       vr17,     vr0,      vr26  // 56 ... 63

.irp i, vr4, vr5, vr6, vr7, vr14, vr15, vr16, vr17
    vsrari.h      \i,       \i,       4
.endr

    alsl.d        t2,       a1,       a0,    1

    VLD_DST_ADD_W8 vr4, vr5, vr6, vr7

    alsl.d        a0,       a1,       a0,     2
    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr14, vr15, vr16, vr17

    fld.d         f24,      sp,       0
    fld.d         f25,      sp,       8
    fld.d         f26,      sp,       16
    fld.d         f27,      sp,       24
    fld.d         f28,      sp,       32
    fld.d         f29,      sp,       40
    addi.d        sp,       sp,       48
endfunc

.macro vmul_vmadd_w in0, in1, in2, in3, out0, out1
    vsllwil.w.h   vr22,     \in0,     0
    vexth.w.h     vr23,     \in0
    vmul.w        \out0,    vr22,     \in2
    vmul.w        \out1,    vr23,     \in2
    vsllwil.w.h   vr22,     \in1,     0
    vexth.w.h     vr23,     \in1
    vmadd.w       \out0,    vr22,     \in3
    vmadd.w       \out1,    vr23,     \in3
.endm

.macro vmul_vmsub_w in0, in1, in2, in3, out0, out1
    vsllwil.w.h   vr22,     \in0,     0
    vexth.w.h     vr23,     \in0
    vmul.w        \out0,    vr22,     \in2
    vmul.w        \out1,    vr23,     \in2
    vsllwil.w.h   vr22,     \in1,     0
    vexth.w.h     vr23,     \in1
    vmsub.w       \out0,    vr22,     \in3
    vmsub.w       \out1,    vr23,     \in3
.endm

.macro rect2_lsx in0, in1, out0
    vsllwil.w.h   vr22,     \in0,     0     // in1
    vexth.w.h     \in0,     \in0            // in1
    vmul.w        vr22,     vr22,     \in1
    vmul.w        \out0,    \in0,     \in1
    vssrarni.h.w  \out0,    vr22,     12
.endm

.macro dct_8x8_core_lsx in0, in1, in2, in3, in4, in5, in6, in7, out0, \
                        out1, out2, out3, out4, out5, out6, out7, rect2

    la.local      t0,       idct_coeffs

.ifc \rect2, rect2_lsx
    vldrepl.w     vr23,      t0,       0        // 2896
.irp i, \in0, \in1, \in2, \in3, \in4, \in5, \in6, \in7
    rect2_lsx \i, vr23, \i
.endr
.endif
    vldrepl.w     vr20,      t0,       8        // 1567
    vldrepl.w     vr21,      t0,       12       // 3784

    vmul_vmadd_w  \in2, \in6, vr21, vr20, vr8, vr9
    vssrarni.h.w  vr9,       vr8,      12       // t3
    vmul_vmsub_w  \in2, \in6, vr20, vr21, vr8, vr10
    vssrarni.h.w  vr10,      vr8,      12       // t2

    vldrepl.w     vr20,      t0,       0        // 2896
    vmul_vmadd_w  \in0, \in4, vr20, vr20, vr8, \in2
    vssrarni.h.w  \in2,      vr8,      12       // t0
    vmul_vmsub_w  \in0, \in4, vr20, vr20, vr8, \in6
    vssrarni.h.w  \in6,      vr8,      12       // t1

    vsadd.h       vr8,       \in2,     vr9      // c[0]
    vssub.h       vr9,       \in2,     vr9      // c[3]
    vsadd.h       \in0,      \in6,     vr10     // c[1]
    vssub.h       vr10,      \in6,     vr10     // c[2]

    vldrepl.w     vr20,     t0,        16       // 799
    vldrepl.w     vr21,     t0,        20       // 4017
    vmul_vmadd_w  \in1, \in7, vr21, vr20, \in2, \in4
    vssrarni.h.w  \in4,     \in2,      12       // t7a
    vmul_vmsub_w  \in1, \in7, vr20, vr21, \in2, \in6
    vssrarni.h.w  \in6,     \in2,      12       // t4a

    vldrepl.w     vr20,     t0,        24       // 3406
    vldrepl.w     vr21,     t0,        28       // 2276
    vmul_vmadd_w  \in5, \in3, vr21, vr20, \in2, \in1
    vssrarni.h.w  \in1,     \in2,      12       // t6a
    vmul_vmsub_w  \in5, \in3, vr20, vr21, \in2, \in7
    vssrarni.h.w  \in7,     \in2,      12       // t5a

    vsadd.h       \in3,     \in6,      \in7     // t4
    vssub.h       \in6,     \in6,      \in7     // t5a
    vsadd.h       \in5,     \in4,      \in1     // t7
    vssub.h       \in4,     \in4,      \in1     // t6a

    vldrepl.w     vr20,     t0,        0        // 2896
    vmul_vmadd_w  \in4, \in6, vr20, vr20, \in2, \in1
    vssrarni.h.w  \in1,     \in2,      12       // t6
    vmul_vmsub_w  \in4, \in6, vr20, vr20, \in2, \in7
    vssrarni.h.w  \in7,     \in2,      12       // t5

    vsadd.h       \out0,    vr8,       \in5     // c[0]
    vssub.h       \out7,    vr8,       \in5     // c[7]
    vsadd.h       \out1,    \in0,      \in1     // c[1]
    vssub.h       \out6,    \in0,      \in1     // c[6]
    vsadd.h       \out2,    vr10,      \in7     // c[2]
    vssub.h       \out5,    vr10,      \in7     // c[5]
    vsadd.h       \out3,    vr9,       \in3     // c[3]
    vssub.h       \out4,    vr9,       \in3     // c[4]
.endm

function inv_txfm_add_dct_dct_8x8_8bpc_lsx
    bnez          a3,       .NO_HAS_DCONLY_8x8

    ld.h          t2,       a2,       0      // dc
    vldi          vr0,      0x8b5            // 181
    vreplgr2vr.w  vr1,      t2
    vldi          vr5,      0x880            // 128
    vmul.w        vr2,      vr0,      vr1    // dc * 181
    st.h          zero,     a2,       0
    vsrari.w      vr2,      vr2,      8      // (dc * 181 + 128) >> 8
    vld           vr10,     a0,       0      // 0 1 2 3 4 5 6 7
    vsrari.w      vr2,      vr2,      1      // (dc + rnd) >> shift
    vldx          vr11,     a0,       a1     // 8 9 10 11 12 13 14 15
    alsl.d        t2,       a1,       a0,    1
    vmadd.w       vr5,      vr2,      vr0
    vld           vr12,     t2,       0      // 16 17 18 19 20 21 22 23
    vssrarni.h.w  vr5,      vr5,      12
    vldx          vr13,     t2,       a1     // 24 25 26 27 28 29 30 31

    DST_ADD_W8 vr10, vr11, vr12, vr13, vr5, vr5, vr5, vr5

    alsl.d        a0,       a1,       a0,     2
    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr5, vr5, vr5, vr5

    b             .DCT_DCT_8X8_END

.NO_HAS_DCONLY_8x8:

    vld_x8 a2, 0, 16, vr0, vr1, vr2, vr3, vr4, vr5, vr6, vr7

    la.local      t0,       idct_coeffs

    dct_8x8_core_lsx vr0, vr1, vr2, vr3, vr4, vr5, vr6, vr7, \
                     vr11, vr12, vr13, vr14, vr15, vr16, vr17, vr18, no_rect2

    LSX_TRANSPOSE8x8_H vr11, vr12, vr13, vr14, vr15, vr16, vr17, vr18, \
                       vr11, vr12, vr13, vr14, vr15, vr16, vr17, vr18, \
                       vr0, vr1, vr2, vr3, vr4, vr5, vr6, vr7

.irp i, vr11, vr12, vr13, vr14, vr15, vr16, vr17, vr18
    vsrari.h      \i,       \i,       1
.endr

    vreplgr2vr.h  vr23,     zero

.irp i, 0, 16, 32, 48, 64, 80, 96, 112
    vst           vr23,     a2,       \i
.endr

    dct_8x8_core_lsx vr11, vr12, vr13, vr14, vr15, vr16, vr17, vr18, \
                     vr4, vr5, vr6, vr7, vr20, vr21, vr22, vr23,  no_rect2

.irp i, vr4, vr5, vr6, vr7, vr20, vr21, vr22, vr23
    vsrari.h      \i,       \i,       4
.endr

    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr4, vr5, vr6, vr7

    alsl.d        a0,       a1,       a0,     2
    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr20, vr21, vr22, vr23

.DCT_DCT_8X8_END:

endfunc

.macro dct_8x16_core_lsx
    dct_8x8_core_lsx vr0, vr2, vr4, vr6, vr19, vr25, vr27, vr29, \
                     vr11, vr12, vr13, vr14, vr15, vr16, vr17, vr18, no_rect2

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       32        // 401
    vldrepl.w     vr21,     t0,       36        // 4076
    vmul_vmadd_w vr1, vr30, vr21, vr20, vr0, vr10
    vssrarni.h.w  vr10,     vr0,      12        // t15a
    vmul_vmsub_w vr1, vr30, vr20, vr21, vr0, vr29
    vssrarni.h.w  vr29,     vr0,      12        // t8a

    vldrepl.w     vr20,     t0,       40        // 3166 -> 1583
    vldrepl.w     vr21,     t0,       44        // 2598 -> 1299
    vmul_vmadd_w vr24, vr7, vr21, vr20, vr0, vr30
    vssrarni.h.w  vr30,     vr0,      12        // t14a
    vmul_vmsub_w vr24, vr7, vr20, vr21, vr0, vr31
    vssrarni.h.w  vr31,     vr0,      12        // t9a

    vldrepl.w     vr20,     t0,       48        // 1931
    vldrepl.w     vr21,     t0,       52        // 3612
    vmul_vmadd_w vr5, vr26, vr21, vr20, vr0, vr24
    vssrarni.h.w  vr24,     vr0,      12        // t13a
    vmul_vmsub_w vr5, vr26, vr20, vr21, vr0, vr25
    vssrarni.h.w  vr25,     vr0,      12        // t10a

    vldrepl.w     vr20,     t0,       56        // 3920
    vldrepl.w     vr21,     t0,       60        // 1189
    vmul_vmadd_w vr28, vr3, vr21, vr20, vr0, vr26
    vssrarni.h.w  vr26,     vr0,      12        // t12a
    vmul_vmsub_w vr28, vr3, vr20, vr21, vr0, vr27
    vssrarni.h.w  vr27,     vr0,      12        // t11a

    // vr22 vr23 vr30 vr31 vr24 vr25 vr26 vr27
    vsadd.h       vr28,     vr29,      vr31     // t8
    vssub.h       vr19,     vr29,      vr31     // t9
    vssub.h       vr29,     vr27,      vr25     // t10
    vsadd.h       vr9,      vr27,      vr25     // t11
    vsadd.h       vr31,     vr26,      vr24     // t12
    vssub.h       vr25,     vr26,      vr24     // t13
    vssub.h       vr27,     vr10,      vr30     // t14
    vsadd.h       vr24,     vr10,      vr30     // t15

    vldrepl.w     vr20,     t0,       8         // 1567
    vldrepl.w     vr21,     t0,       12        // 3784
    vmul_vmadd_w vr27, vr19, vr21, vr20, vr0, vr26
    vssrarni.h.w  vr26,     vr0,       12       // t14a
    vmul_vmsub_w vr27, vr19, vr20, vr21, vr0, vr30
    vssrarni.h.w  vr30,     vr0,       12       // t9a

    vmul_vmadd_w vr25, vr29, vr21, vr20, vr0, vr19
    vneg.w        vr0,      vr0
    vneg.w        vr19,     vr19
    vssrarni.h.w  vr19,     vr0,       12       // t10a
    vmul_vmsub_w vr25, vr29, vr20, vr21, vr0, vr27
    vssrarni.h.w  vr27,     vr0,       12       // t13a

    vsadd.h       vr25,     vr28,     vr9       // t8a
    vssub.h       vr29,     vr28,     vr9       // t11a
    vssub.h       vr28,     vr24,     vr31      // t12a
    vsadd.h       vr10,     vr24,     vr31      // t15a
    vsadd.h       vr9,      vr30,     vr19      // t9
    vssub.h       vr31,     vr30,     vr19      // t10
    vssub.h       vr30,     vr26,     vr27      // t13
    vsadd.h       vr24,     vr26,     vr27      // t14

    vldrepl.w     vr20,     t0,       0         // 2896
    vmul_vmadd_w vr30, vr31, vr20, vr20, vr0, vr26
    vssrarni.h.w  vr26,     vr0,      12        // t13a
    vmul_vmsub_w vr30, vr31, vr20, vr20, vr0, vr27
    vssrarni.h.w  vr27,     vr0,      12        // t10a

    vmul_vmadd_w vr28, vr29, vr20, vr20, vr0, vr31
    vssrarni.h.w  vr31,     vr0,      12        // t12
    vmul_vmsub_w vr28, vr29, vr20, vr20, vr0, vr30
    vssrarni.h.w  vr30,     vr0,      12        // t11

    // vr11 vr12 ... vr18
    vsadd.h       vr28,     vr14,     vr31      // c[3]
    vssub.h       vr29,     vr14,     vr31      // c[12]
    vsadd.h       vr20,     vr15,     vr30      // c[4]
    vssub.h       vr21,     vr15,     vr30      // c[11]
    vsadd.h       vr14,     vr16,     vr27      // c[5]
    vssub.h       vr23,     vr16,     vr27      // c[10]
    vsadd.h       vr15,     vr17,     vr9       // c[6]
    vssub.h       vr30,     vr17,     vr9       // c[9]
    vsadd.h       vr16,     vr18,     vr25      // c[7]
    vssub.h       vr27,     vr18,     vr25      // c[8]
    vsadd.h       vr17,     vr13,     vr26      // c[2]
    vssub.h       vr26,     vr13,     vr26      // c[13]
    vsadd.h       vr18,     vr12,     vr24      // c[1]
    vssub.h       vr25,     vr12,     vr24      // c[14]
    vsadd.h       vr22,     vr11,     vr10      // c[0]
    vssub.h       vr24,     vr11,     vr10      // c[15]
.endm

function inv_txfm_add_dct_dct_8x16_8bpc_lsx
    bnez          a3,       .NO_HAS_DCONLY_8x16

    ld.h          t2,       a2,       0      // dc
    vldi          vr0,      0x8b5            // 181
    vreplgr2vr.w  vr1,      t2
    vldi          vr5,      0x880            // 128
    vmul.w        vr2,      vr0,      vr1    // dc * 181
    st.h          zero,     a2,       0
    vsrari.w      vr2,      vr2,      8      // (dc * 181 + 128) >> 8
    vld           vr10,     a0,       0      // 0 1 2 3 4 5 6 7
    vmul.w        vr2,      vr0,      vr2
    vsrari.w      vr2,      vr2,      8      // (dc * 181 + 128) >> 8
    vsrari.w      vr2,      vr2,      1      // (dc + rnd) >> shift
    vldx          vr11,     a0,       a1     // 8 9 10 11 12 13 14 15
    alsl.d        t2,       a1,       a0,    1
    vmadd.w       vr5,      vr2,      vr0
    vld           vr12,     t2,       0      // 16 17 18 19 20 21 22 23
    vssrarni.h.w  vr5,      vr5,      12
    vldx          vr13,     t2,       a1     // 24 25 26 27 28 29 30 31

    DST_ADD_W8 vr10, vr11, vr12, vr13, vr5, vr5, vr5, vr5

    alsl.d        a0,       a1,       a0,     2
    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr5, vr5, vr5, vr5

    alsl.d        a0,       a1,       a0,     2
    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr5, vr5, vr5, vr5

    alsl.d        a0,       a1,       a0,     2
    alsl.d        t2,       a1,       a0,     1

    VLD_DST_ADD_W8 vr5, vr5, vr5, vr5

    b             .DCT_DCT_8X16_END

.NO_HAS_DCONLY_8x16:
    addi.d        sp,       sp,       -64
    fst.d         f24,      sp,       0
    fst.d         f25,      sp,       8
    fst.d         f26,      sp,       16
    fst.d         f27,      sp,       24
    fst.d         f28,      sp,       32
    fst.d         f29,      sp,       40
    fst.d         f30,      sp,       48
    fst.d         f31,      sp,       56

    vld_x8 a2, 0, 32, vr0, vr1, vr2, vr3, vr4, vr5, vr6, vr7

    la.local      t0,       idct_coeffs

    dct_8x8_core_lsx vr0, vr1, vr2, vr3, vr4, vr5, vr6, vr7, \
                     vr11, vr12, vr13, vr14, vr15, vr16, vr17, vr18, rect2_lsx

    vld_x8 a2, 16, 32, vr0, vr1, vr2, vr3, vr4, vr5, vr6, vr7

    dct_8x8_core_lsx vr0, vr1, vr2, vr3, vr4, vr5, vr6, vr7, \
                     vr19, vr24, vr25, vr26, vr27, vr28, vr29, vr30, rect2_lsx

.irp i, vr11, vr12, vr13, vr14, vr15, vr16, vr17, vr18, \
        vr19, vr24, vr25, vr26, vr27, vr28, vr29, vr30
    vsrari.h      \i,       \i,       1
.endr

    vreplgr2vr.h  vr23,     zero

.irp i, 0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240
    vst           vr23,     a2,       \i
.endr

    LSX_TRANSPOSE8x8_H vr11, vr12, vr13, vr14, vr15, vr16, vr17, vr18, \
                       vr0, vr1, vr2, vr3, vr4, vr5, vr6, vr7, \
                       vr8, vr9, vr10, vr20, vr21, vr22, vr23, vr31

    LSX_TRANSPOSE8x8_H vr19, vr24, vr25, vr26, vr27, vr28, vr29, vr30, \
                       vr19, vr24, vr25, vr26, vr27, vr28, vr29, vr30, \
                       vr8, vr9, vr10, vr20, vr21, vr22, vr23, vr31

    dct_8x16_core_lsx

.irp i, vr22, vr18, vr17, vr28, vr20, vr14, vr15, vr16, \
        vr27, vr30, vr23, vr21, vr29, vr26, vr25, vr24
    vsrari.h     \i,       \i,       4
.endr

    alsl.d        t2,       a1,       a0,    1

    VLD_DST_ADD_W8 vr22, vr18, vr17, vr28

    alsl.d        a0,       a1,       a0,    2
    alsl.d        t2,       a1,       a0,    1

    VLD_DST_ADD_W8 vr20, vr14, vr15, vr16

    alsl.d        a0,       a1,       a0,    2
    alsl.d        t2,       a1,       a0,    1

    VLD_DST_ADD_W8 vr27, vr30, vr23, vr21

    alsl.d        a0,       a1,       a0,    2
    alsl.d        t2,       a1,       a0,    1

    VLD_DST_ADD_W8 vr29, vr26, vr25, vr24

    fld.d         f24,      sp,       0
    fld.d         f25,      sp,       8
    fld.d         f26,      sp,       16
    fld.d         f27,      sp,       24
    fld.d         f28,      sp,       32
    fld.d         f29,      sp,       40
    fld.d         f30,      sp,       48
    fld.d         f31,      sp,       56
    addi.d        sp,       sp,       64
.DCT_DCT_8X16_END:
endfunc

.macro identity_8x8_core_lsx in0, in1, in2, in3, in4, in5, in6, in7, rect2

    la.local      t0,       idct_coeffs

.ifc \rect2, rect2_lsx
    vldrepl.w     vr23,      t0,       0       // 2896
.irp i, \in0, \in1, \in2, \in3, \in4, \in5, \in6, \in7
    rect2_lsx \i, vr23, \i
.endr
.endif
    vsllwil.w.h   vr8,      \in0,     1
    vsllwil.w.h   vr9,      \in1,     1
    vsllwil.w.h   vr10,     \in2,     1
    vsllwil.w.h   vr11,     \in3,     1
    vsllwil.w.h   vr12,     \in4,     1
    vsllwil.w.h   vr13,     \in5,     1
    vsllwil.w.h   vr14,     \in6,     1
    vsllwil.w.h   vr15,     \in7,     1

.irp i, \in0, \in1, \in2, \in3, \in4, \in5, \in6, \in7
    vexth.w.h     \i,       \i
.endr

.irp i, \in0, \in1, \in2, \in3, \in4, \in5, \in6, \in7
    vslli.w       \i,       \i,       1
.endr

    vssrarni.h.w  \in0,     vr8,      1
    vssrarni.h.w  \in1,     vr9,      1
    vssrarni.h.w  \in2,     vr10,     1
    vssrarni.h.w  \in3,     vr11,     1
    vssrarni.h.w  \in4,     vr12,     1
    vssrarni.h.w  \in5,     vr13,     1
    vssrarni.h.w  \in6,     vr14,     1
    vssrarni.h.w  \in7,     vr15,     1
.endm

.macro identity_8x16_core_lsx in0, out0
    vsadd.h       vr10,     \in0,     \in0
    vsllwil.w.h   vr8,      \in0,     0
    vexth.w.h     \out0,    \in0
    vmul.w        vr8,      vr8,      vr20
    vmul.w        \out0,    \out0,    vr20
    vssrarni.h.w  \out0,    vr8,      11
    vsadd.h       \out0,    \out0,    vr10
.endm

function inv_txfm_add_identity_identity_8x16_8bpc_lsx
    addi.d        sp,       sp,       -64
    fst.d         f24,      sp,       0
    fst.d         f25,      sp,       8
    fst.d         f26,      sp,       16
    fst.d         f27,      sp,       24
    fst.d         f28,      sp,       32
    fst.d         f29,      sp,       40
    fst.d         f30,      sp,       48
    fst.d         f31,      sp,       56

    vld_x8 a2, 0, 16, vr0, vr1, vr2, vr3, vr4, vr5, vr6, vr7

    identity_8x8_core_lsx vr0, vr1, vr2, vr3, vr4, vr5, vr6, vr7, rect2_lsx

    vld_x8 a2, 128, 16, vr16, vr17, vr18, vr19, vr24, vr25, vr26, vr27

    identity_8x8_core_lsx vr16, vr17, vr18, vr19, vr24, vr25, vr26, vr27, rect2_lsx

    vreplgr2vr.h  vr23,     zero

.irp i, 0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240
    vst           vr23,     a2,       \i
.endr


    LSX_TRANSPOSE8x8_H vr0, vr2, vr4, vr6, vr16, vr18, vr24, vr26, \
                       vr14, vr15, vr22, vr23, vr16, vr18, vr24, vr26, \
                       vr8, vr9, vr10, vr11, vr12, vr13, vr20, vr21

    LSX_TRANSPOSE8x8_H vr1, vr3, vr5, vr7, vr17, vr19, vr25, vr27, \
                       vr28, vr29, vr30, vr31, vr17, vr19, vr25, vr27, \
                       vr8, vr9, vr10, vr11, vr12, vr13, vr20, vr21

    li.w          t0,       1697
    vreplgr2vr.w  vr20,     t0

.irp i, vr14, vr15, vr22, vr23, vr16, vr18, vr24, vr26, \
        vr28, vr29, vr30, vr31, vr17, vr19, vr25, vr27
    identity_8x16_core_lsx \i, \i
    vsrari.h      \i,       \i,       4
.endr

    alsl.d        t2,       a1,       a0,    1

    VLD_DST_ADD_W8 vr14, vr15, vr22, vr23

    alsl.d        a0,       a1,       a0,    2
    alsl.d        t2,       a1,       a0,    1

    VLD_DST_ADD_W8 vr16, vr18, vr24, vr26

    alsl.d        a0,       a1,       a0,    2
    alsl.d        t2,       a1,       a0,    1

    VLD_DST_ADD_W8 vr28, vr29, vr30, vr31

    alsl.d        a0,       a1,       a0,    2
    alsl.d        t2,       a1,       a0,    1

    VLD_DST_ADD_W8 vr17, vr19, vr25, vr27

    fld.d         f24,      sp,       0
    fld.d         f25,      sp,       8
    fld.d         f26,      sp,       16
    fld.d         f27,      sp,       24
    fld.d         f28,      sp,       32
    fld.d         f29,      sp,       40
    fld.d         f30,      sp,       48
    fld.d         f31,      sp,       56
    addi.d        sp,       sp,       64
endfunc

.macro adst_8x8_core_lsx in0, in1, in2, in3, in4, in5, in6, in7, out0, out1, \
                         out2, out3, out4, out5, out6, out7, rect2

    la.local      t0,       iadst8_coeffs

.ifc \rect2, rect2_lsx
    vldrepl.w     vr23,      t0,       32       // 2896
.irp i, \in0, \in1, \in2, \in3, \in4, \in5, \in6, \in7
    rect2_lsx \i, vr23, \i
.endr
.endif

    vldrepl.w     vr20,     t0,       0         // 4076
    vldrepl.w     vr21,     t0,       4         // 401

    vmul_vmadd_w vr7, vr0, vr20, vr21, vr8, vr9
    vssrarni.h.w  vr9,      vr8,      12        // t0a low
    vmul_vmsub_w vr7, vr0, vr21, vr20, vr8, vr10
    vssrarni.h.w  vr10,     vr8,      12        // t1a low

    vldrepl.w     vr20,     t0,       8         // 3612
    vldrepl.w     vr21,     t0,       12        // 1931
    vmul_vmadd_w vr5, vr2, vr20, vr21, vr8, vr0
    vssrarni.h.w  vr0,      vr8,      12        // t2a low
    vmul_vmsub_w vr5, vr2, vr21, vr20, vr8, vr7
    vssrarni.h.w  vr7,      vr8,      12        // t3a low

    vldrepl.w     vr20,     t0,       16        // 2598 -> 1299
    vldrepl.w     vr21,     t0,       20        // 3166 -> 1583
    vmul_vmadd_w vr3, vr4, vr20, vr21, vr8, vr2
    vssrarni.h.w  vr2,      vr8,      12        // t4a low
    vmul_vmsub_w vr3, vr4, vr21, vr20, vr8, vr5
    vssrarni.h.w  vr5,      vr8,      12        // t5a low

    vldrepl.w     vr20,     t0,       24        // 1189
    vldrepl.w     vr21,     t0,       28        // 3920
    vmul_vmadd_w vr1, vr6, vr20, vr21, vr8, vr3
    vssrarni.h.w  vr3,      vr8,      12        // t6a low
    vmul_vmsub_w vr1, vr6, vr21, vr20, vr8, vr4
    vssrarni.h.w  vr4,      vr8,      12        // t7a low

    vsadd.h       vr1,      vr9,      vr2       // t0
    vssub.h       vr6,      vr9,      vr2       // t4
    vsadd.h       vr8,      vr10,     vr5       // t1
    vssub.h       vr2,      vr10,     vr5       // t5
    vsadd.h       vr9,      vr0,      vr3       // t2
    vssub.h       vr5,      vr0,      vr3       // t6
    vsadd.h       vr10,     vr7,      vr4       // t3
    vssub.h       vr0,      vr7,      vr4       // t7

    vldrepl.w     vr20,     t0,       40        // 1567
    vldrepl.w     vr21,     t0,       44        // 3784
    vmul_vmadd_w vr6, vr2, vr21, vr20, vr3, vr4
    vssrarni.h.w  vr4,      vr3,      12        // t4a low
    vmul_vmsub_w vr6, vr2, vr20, vr21, vr3, vr7
    vssrarni.h.w  vr7,      vr3,      12        // t5a low

    vmul_vmadd_w vr0, vr5, vr20, vr21, vr3, vr2
    vssrarni.h.w  vr2,      vr3,      12        // t7a low
    vmul_vmsub_w vr0, vr5, vr21, vr20, vr3, vr6
    vssrarni.h.w  vr6,      vr3,      12        // t6a low

    vsadd.h       \out0,    vr1,      vr9       // out[0]
    vssub.h       vr5,      vr1,      vr9       // t2
    vsadd.h       vr3,      vr8,      vr10      // out[7]
    vssub.h       vr1,      vr8,      vr10      // t3
    vexth.w.h     vr9,      vr3
    vsllwil.w.h   vr21,     vr3,      0
    vneg.w        \out7,    vr9
    vneg.w        vr21,     vr21
    vssrarni.h.w  \out7,    vr21,     0         // out[7]

    vsadd.h       vr8,      vr4,      vr6       // out[1]
    vssub.h       vr10,     vr4,      vr6       // t6
    vexth.w.h     vr20,     vr8
    vsllwil.w.h   vr21,     vr8,      0
    vneg.w        \out1,    vr20
    vneg.w        vr21,     vr21
    vssrarni.h.w  \out1,    vr21,     0         // out[1]
    vsadd.h       \out6,    vr7,      vr2       // out[6]
    vssub.h       vr4,      vr7,      vr2       // t7

    vldrepl.w     vr20,     t0,       32        // 2896
    vmul_vmadd_w vr5, vr1, vr20, vr20, vr9, vr6
    vssrarni.h.w  vr6,      vr9,      12        // out[3]
    vmul_vmsub_w vr5, vr1, vr20, vr20, vr9, \out4
    vssrarni.h.w  \out4,    vr9,      12        // out[4]

    vmul_vmadd_w vr10, vr4, vr20, vr20, vr9, \out2
    vssrarni.h.w  \out2,    vr9,      12        // out[2]
    vmul_vmsub_w vr10, vr4, vr20, vr20, vr9, vr5
    vssrarni.h.w  vr5,      vr9,      12        // out[5]

    vexth.w.h     vr20,     vr6
    vsllwil.w.h   vr21,     vr6,      0
    vneg.w        \out3,    vr20
    vneg.w        vr21,     vr21
    vssrarni.h.w  \out3,    vr21,     0         // out[3]

    vexth.w.h     vr20,     vr5
    vsllwil.w.h   vr21,     vr5,      0
    vneg.w        \out5,    vr20
    vneg.w        vr21,     vr21
    vssrarni.h.w  \out5,    vr21,     0         // out[5]
.endm

function inv_txfm_add_adst_dct_8x16_8bpc_lsx
    addi.d        sp,       sp,       -64
    fst.d         f24,      sp,       0
    fst.d         f25,      sp,       8
    fst.d         f26,      sp,       16
    fst.d         f27,      sp,       24
    fst.d         f28,      sp,       32
    fst.d         f29,      sp,       40
    fst.d         f30,      sp,       48
    fst.d         f31,      sp,       56

    vld_x8 a2, 0, 32, vr0, vr1, vr2, vr3, vr4, vr5, vr6, vr7

    adst_8x8_core_lsx vr0, vr1, vr2, vr3, vr4, vr5, vr6, vr7, \
                      vr11, vr12, vr13, vr14, vr15, vr16, vr17, vr18, rect2_lsx

    vld_x8 a2, 16, 32, vr0, vr1, vr2, vr3, vr4, vr5, vr6, vr7

    adst_8x8_core_lsx vr0, vr1, vr2, vr3, vr4, vr5, vr6, vr7, \
                      vr19, vr24, vr25, vr26, vr27, vr28, vr29, vr30, rect2_lsx

.irp i, vr11, vr12, vr13, vr14, vr15, vr16, vr17, vr18, \
        vr19, vr24, vr25, vr26, vr27, vr28, vr29, vr30
    vsrari.h     \i,       \i,       1
.endr

    vreplgr2vr.h  vr23,     zero

.irp i, 0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240
    vst           vr23,     a2,       \i
.endr

    LSX_TRANSPOSE8x8_H vr11, vr12, vr13, vr14, vr15, vr16, vr17, vr18, \
                       vr0, vr1, vr2, vr3, vr4, vr5, vr6, vr7, \
                       vr8, vr9, vr10, vr20, vr21, vr22, vr23, vr31

    LSX_TRANSPOSE8x8_H vr19, vr24, vr25, vr26, vr27, vr28, vr29, vr30, \
                       vr19, vr24, vr25, vr26, vr27, vr28, vr29, vr30, \
                       vr8, vr9, vr10, vr20, vr21, vr22, vr23, vr31

    dct_8x8_core_lsx vr0, vr2, vr4, vr6, vr19, vr25, vr27, vr29, \
                     vr11, vr12, vr13, vr14, vr15, vr16, vr17, vr18, no_rect2

    la.local      t0,       idct_coeffs
    vldrepl.w     vr20,     t0,       32        // 401
    vldrepl.w     vr21,     t0,       36        // 4076
    vmul_vmadd_w vr1, vr30, vr21, vr20, vr0, vr10
    vssrarni.h.w  vr10,     vr0,      12        // t15a
    vmul_vmsub_w vr1, vr30, vr20, vr21, vr0, vr29
    vssrarni.h.w  vr29,     vr0,      12        // t8a

    vldrepl.w     vr20,     t0,       40        // 3166 -> 1583
    vldrepl.w     vr21,     t0,       44        // 2598 -> 1299
    vmul_vmadd_w vr24, vr7, vr21, vr20, vr0, vr30
    vssrarni.h.w  vr30,     vr0,      12        // t14a
    vmul_vmsub_w vr24, vr7, vr20, vr21, vr0, vr31
    vssrarni.h.w  vr31,     vr0,      12        // t9a

    vldrepl.w     vr20,     t0,       48        // 1931
    vldrepl.w     vr21,     t0,       52        // 3612
    vmul_vmadd_w vr5, vr26, vr21, vr20, vr0, vr24
    vssrarni.h.w  vr24,     vr0,      12        // t13a
    vmul_vmsub_w vr5, vr26, vr20, vr21, vr0, vr25
    vssrarni.h.w  vr25,     vr0,      12        // t10a

    vldrepl.w     vr20,     t0,       56        // 3920
    vldrepl.w     vr21,     t0,       60        // 1189
    vmul_vmadd_w vr28, vr3, vr21, vr20, vr0, vr26
    vssrarni.h.w  vr26,     vr0,      12        // t12a
    vmul_vmsub_w vr28, vr3, vr20, vr21, vr0, vr27
    vssrarni.h.w  vr27,     vr0,      12        // t11a

    // vr22 vr23 vr30 vr31 vr24 vr25 vr26 vr27
    vsadd.h       vr28,     vr29,      vr31     // t8
    vssub.h       vr19,     vr29,      vr31     // t9
    vssub.h       vr29,     vr27,      vr25     // t10
    vsadd.h       vr9,      vr27,      vr25     // t11
    vsadd.h       vr31,     vr26,      vr24     // t12
    vssub.h       vr25,     vr26,      vr24     // t13
    vssub.h       vr27,     vr10,      vr30     // t14
    vsadd.h       vr24,     vr10,      vr30     // t15

    vldrepl.w     vr20,     t0,       8         // 1567
    vldrepl.w     vr21,     t0,       12        // 3784
    vmul_vmadd_w vr27, vr19, vr21, vr20, vr0, vr26
    vssrarni.h.w  vr26,     vr0,       12       // t14a
    vmul_vmsub_w vr27, vr19, vr20, vr21, vr0, vr30
    vssrarni.h.w  vr30,     vr0,       12       // t9a

    vmul_vmadd_w vr25, vr29, vr21, vr20, vr0, vr19
    vneg.w        vr0,      vr0
    vneg.w        vr19,     vr19
    vssrarni.h.w  vr19,     vr0,       12       // t10a
    vmul_vmsub_w vr25, vr29, vr20, vr21, vr0, vr27
    vssrarni.h.w  vr27,     vr0,       12       // t13a

    vsadd.h       vr25,     vr28,     vr9       // t8a
    vssub.h       vr29,     vr28,     vr9       // t11a
    vssub.h       vr28,     vr24,     vr31      // t12a
    vsadd.h       vr10,     vr24,     vr31      // t15a
    vsadd.h       vr9,      vr30,     vr19      // t9
    vssub.h       vr31,     vr30,     vr19      // t10
    vssub.h       vr30,     vr26,     vr27      // t13
    vsadd.h       vr24,     vr26,     vr27      // t14

    vldrepl.w     vr20,     t0,       0         // 2896
    vmul_vmadd_w vr30, vr31, vr20, vr20, vr0, vr26
    vssrarni.h.w  vr26,     vr0,      12        // t13a
    vmul_vmsub_w vr30, vr31, vr20, vr20, vr0, vr27
    vssrarni.h.w  vr27,     vr0,      12        // t10a

    vmul_vmadd_w vr28, vr29, vr20, vr20, vr0, vr31
    vssrarni.h.w  vr31,     vr0,      12        // t12
    vmul_vmsub_w vr28, vr29, vr20, vr20, vr0, vr30
    vssrarni.h.w  vr30,     vr0,      12        // t11

    // vr11 vr12 ... vr18
    vsadd.h       vr28,     vr14,     vr31      // c[3]
    vssub.h       vr29,     vr14,     vr31      // c[12]
    vsadd.h       vr20,     vr15,     vr30      // c[4]
    vssub.h       vr21,     vr15,     vr30      // c[11]
    vsadd.h       vr14,     vr16,     vr27      // c[5]
    vssub.h       vr23,     vr16,     vr27      // c[10]
    vsadd.h       vr15,     vr17,     vr9       // c[6]
    vssub.h       vr30,     vr17,     vr9       // c[9]
    vsadd.h       vr16,     vr18,     vr25      // c[7]
    vssub.h       vr27,     vr18,     vr25      // c[8]
    vsadd.h       vr17,     vr13,     vr26      // c[2]
    vssub.h       vr26,     vr13,     vr26      // c[13]
    vsadd.h       vr18,     vr12,     vr24      // c[1]
    vssub.h       vr25,     vr12,     vr24      // c[14]
    vsadd.h       vr22,     vr11,     vr10      // c[0]
    vssub.h       vr24,     vr11,     vr10      // c[15]

.irp i, vr22, vr18, vr17, vr28, vr20, vr14, vr15, vr16, \
        vr27, vr30, vr23, vr21, vr29, vr26, vr25, vr24
    vsrari.h     \i,       \i,       4
.endr

    alsl.d        t2,       a1,       a0,    1

    VLD_DST_ADD_W8 vr22, vr18, vr17, vr28

    alsl.d        a0,       a1,       a0,    2
    alsl.d        t2,       a1,       a0,    1

    VLD_DST_ADD_W8 vr20, vr14, vr15, vr16

    alsl.d        a0,       a1,       a0,    2
    alsl.d        t2,       a1,       a0,    1

    VLD_DST_ADD_W8 vr27, vr30, vr23, vr21

    alsl.d        a0,       a1,       a0,    2
    alsl.d        t2,       a1,       a0,    1

    VLD_DST_ADD_W8 vr29, vr26, vr25, vr24

    fld.d         f24,      sp,       0
    fld.d         f25,      sp,       8
    fld.d         f26,      sp,       16
    fld.d         f27,      sp,       24
    fld.d         f28,      sp,       32
    fld.d         f29,      sp,       40
    fld.d         f30,      sp,       48
    fld.d         f31,      sp,       56
    addi.d        sp,       sp,       64
endfunc

const iadst16_coeffs, align=4
    .word         4091, 201, 3973, 995
    .word         3703, 1751, 3290, 2440
    .word         2751, 3035, 2106, 3513
    .word         1380, 3857, 601, 4052
endconst

.macro adst16_core_lsx transpose8x8, shift, vst
    la.local      t0,       iadst16_coeffs
    vldrepl.w     vr20,     t0,        0        // 4091
    vldrepl.w     vr21,     t0,        4        // 201

    vmul_vmadd_w vr15, vr0, vr20, vr21, vr16, vr18
    vmul_vmsub_w vr15, vr0, vr21, vr20, vr17, vr19
    vssrarni.h.w  vr18,     vr16,      12       // t0
    vssrarni.h.w  vr19,     vr17,      12       // t1

    vldrepl.w     vr20,     t0,        8        // 3973
    vldrepl.w     vr21,     t0,        12       // 995
    vmul_vmadd_w vr13, vr2, vr20, vr21, vr16, vr0
    vmul_vmsub_w vr13, vr2, vr21, vr20, vr17, vr15
    vssrarni.h.w  vr0,      vr16,      12       // t2
    vssrarni.h.w  vr15,     vr17,      12       // t3

    vldrepl.w     vr20,     t0,        16       // 3703
    vldrepl.w     vr21,     t0,        20       // 1751
    vmul_vmadd_w vr11, vr4, vr20, vr21, vr16, vr2
    vmul_vmsub_w vr11, vr4, vr21, vr20, vr17, vr13
    vssrarni.h.w  vr2,      vr16,      12       // t4
    vssrarni.h.w  vr13,     vr17,      12       // t5

    vldrepl.w     vr20,     t0,        24       // 3290 -> 1645
    vldrepl.w     vr21,     t0,        28       // 2440 -> 1220
    vmul_vmadd_w vr9, vr6, vr20, vr21, vr16, vr4
    vmul_vmsub_w vr9, vr6, vr21, vr20, vr17, vr11
    vssrarni.h.w  vr4,      vr16,      12       // t6
    vssrarni.h.w  vr11,     vr17,      12       // t7

    vldrepl.w     vr20,     t0,        32       // 2751
    vldrepl.w     vr21,     t0,        36       // 3035
    vmul_vmadd_w vr7, vr8, vr20, vr21, vr16, vr6
    vmul_vmsub_w vr7, vr8, vr21, vr20, vr17, vr9
    vssrarni.h.w  vr6,      vr16,      12       // t8
    vssrarni.h.w  vr9,      vr17,      12       // t9

    vldrepl.w     vr20,     t0,        40       // 2106
    vldrepl.w     vr21,     t0,        44       // 3513
    vmul_vmadd_w vr5, vr10, vr20, vr21, vr16, vr7
    vmul_vmsub_w vr5, vr10, vr21, vr20, vr17, vr8
    vssrarni.h.w  vr7,      vr16,      12       // t10
    vssrarni.h.w  vr8,      vr17,      12       // t11

    vldrepl.w     vr20,     t0,        48       // 1380
    vldrepl.w     vr21,     t0,        52       // 3857
    vmul_vmadd_w vr3, vr12, vr20, vr21, vr16, vr5
    vmul_vmsub_w vr3, vr12, vr21, vr20, vr17, vr10
    vssrarni.h.w  vr5,      vr16,      12       // t12
    vssrarni.h.w  vr10,     vr17,      12       // t13

    vldrepl.w     vr20,     t0,        56       // 601
    vldrepl.w     vr21,     t0,        60       // 4052
    vmul_vmadd_w vr1, vr14, vr20, vr21, vr16, vr3
    vmul_vmsub_w vr1, vr14, vr21, vr20, vr17, vr12
    vssrarni.h.w  vr3,      vr16,      12       // t14
    vssrarni.h.w  vr12,     vr17,      12       // t15

    vsadd.h       vr1,      vr18,      vr6      // t0a
    vssub.h       vr14,     vr18,      vr6      // t8a
    vsadd.h       vr16,     vr19,      vr9      // t1a
    vssub.h       vr17,     vr19,      vr9      // t9a
    vsadd.h       vr6,      vr0,       vr7      // t2a
    vssub.h       vr18,     vr0,       vr7      // t10a
    vsadd.h       vr9,      vr15,      vr8      // t3a
    vssub.h       vr19,     vr15,      vr8      // t11a
    vsadd.h       vr0,      vr2,       vr5      // t4a
    vssub.h       vr7,      vr2,       vr5      // t12a
    vsadd.h       vr8,      vr13,      vr10     // t5a
    vssub.h       vr15,     vr13,      vr10     // t13a
    vsadd.h       vr2,      vr4,       vr3      // t6a
    vssub.h       vr5,      vr4,       vr3      // t14a
    vsadd.h       vr10,     vr11,      vr12     // t7a
    vssub.h       vr13,     vr11,      vr12     // t15a

    la.local      t0,       idct_coeffs

    vldrepl.w     vr20,     t0,        16       // 799
    vldrepl.w     vr21,     t0,        20       // 4017
    vmul_vmadd_w vr14, vr17, vr21, vr20, vr3, vr11
    vmul_vmsub_w vr14, vr17, vr20, vr21, vr4, vr12
    vssrarni.h.w  vr11,     vr3,       12       // t8
    vssrarni.h.w  vr12,     vr4,       12       // t9

    vmul_vmadd_w vr15, vr7, vr20, vr21, vr3, vr14
    vmul_vmsub_w vr15, vr7, vr21, vr20, vr4, vr17
    vssrarni.h.w  vr14,     vr3,       12       // t13
    vssrarni.h.w  vr17,     vr4,       12       // t12

    vldrepl.w     vr20,     t0,        24       // 3406
    vldrepl.w     vr21,     t0,        28       // 2276
    vmul_vmadd_w vr18, vr19, vr21, vr20, vr3, vr7
    vmul_vmsub_w vr18, vr19, vr20, vr21, vr4, vr15
    vssrarni.h.w  vr7,      vr3,       12       // t10
    vssrarni.h.w  vr15,     vr4,       12       // t11

    vmul_vmadd_w vr13, vr5, vr20, vr21, vr3, vr18
    vmul_vmsub_w vr13, vr5, vr21, vr20, vr4, vr19
    vssrarni.h.w  vr18,     vr3,       12       // t15
    vssrarni.h.w  vr19,     vr4,       12       // t14

    vsadd.h       vr5,      vr1,       vr0      // t0
    vssub.h       vr13,     vr1,       vr0      // t4
    vsadd.h       vr3,      vr16,      vr8      // t1
    vssub.h       vr4,      vr16,      vr8      // t5
    vsadd.h       vr0,      vr6,       vr2      // t2
    vssub.h       vr1,      vr6,       vr2      // t6
    vsadd.h       vr8,      vr9,       vr10     // t3
    vssub.h       vr16,     vr9,       vr10     // t7
    vsadd.h       vr2,      vr11,      vr17     // t8a
    vssub.h       vr6,      vr11,      vr17     // t12a
    vsadd.h       vr9,      vr12,      vr14     // t9a
    vssub.h       vr10,     vr12,      vr14     // t13a
    vsadd.h       vr11,     vr7,       vr19     // t10a
    vssub.h       vr17,     vr7,       vr19     // t14a
    vsadd.h       vr12,     vr15,      vr18     // t11a
    vssub.h       vr14,     vr15,      vr18     // t15a

    la.local      t0,       idct_coeffs

    vldrepl.w     vr20,     t0,        8        // 1567
    vldrepl.w     vr21,     t0,        12       // 3784
    vmul_vmadd_w vr13, vr4, vr21, vr20, vr7, vr18
    vmul_vmsub_w vr13, vr4, vr20, vr21, vr15, vr19
    vssrarni.h.w  vr18,     vr7,       12       // t4a
    vssrarni.h.w  vr19,     vr15,      12       // t5a

    vmul_vmadd_w vr16, vr1, vr20, vr21, vr7, vr4
    vmul_vmsub_w vr16, vr1, vr21, vr20, vr15, vr13
    vssrarni.h.w  vr4,      vr7,       12       // t7a
    vssrarni.h.w  vr13,     vr15,      12       // t6a

    vmul_vmadd_w vr6, vr10, vr21, vr20, vr7, vr1
    vmul_vmsub_w vr6, vr10, vr20, vr21, vr15, vr16
    vssrarni.h.w  vr1,      vr7,       12       // t12
    vssrarni.h.w  vr16,     vr15,      12       // t13

    vmul_vmadd_w vr14, vr17, vr20, vr21, vr7, vr6
    vmul_vmsub_w vr14, vr17, vr21, vr20, vr15, vr10
    vssrarni.h.w  vr6,      vr7,       12       // t15
    vssrarni.h.w  vr10,     vr15,      12       // t14

    vsadd.h       vr14,     vr5,       vr0      // out[0]
    vssub.h       vr17,     vr5,       vr0      // t2a
    vssub.h       vr7,      vr3,       vr8      // t3a
    vsadd.h       vr15,     vr3,       vr8      // out[15]
    vsllwil.w.h   vr22,     vr15,      0
    vexth.w.h     vr15,     vr15
    vneg.w        vr22,     vr22
    vneg.w        vr15,     vr15
    vssrarni.h.w  vr15,     vr22,      0        // out[15]
    vsadd.h       vr14,     vr5,       vr0      // out[0]
    vssub.h       vr17,     vr5,       vr0      // t2a
    vssub.h       vr7,      vr3,       vr8      // t3a

    vsadd.h       vr3,      vr19,      vr4      // out[12]
    vssub.h       vr8,      vr19,      vr4      // t7
    vssub.h       vr0,      vr18,      vr13     // t6
    vsadd.h       vr5,      vr18,      vr13     // out[3]
    vsllwil.w.h   vr22,     vr5,       0
    vexth.w.h     vr5,      vr5
    vneg.w        vr22,     vr22
    vneg.w        vr5,      vr5
    vssrarni.h.w  vr5,      vr22,      0        // out[3]

    vsadd.h       vr13,     vr9,       vr12     // out[14]
    vssub.h       vr19,     vr9,       vr12     // t11
    vssub.h       vr4,      vr2,       vr11     // t10
    vsadd.h       vr18,     vr2,       vr11     // out[1]
    vsllwil.w.h   vr22,     vr18,      0
    vexth.w.h     vr18,     vr18
    vneg.w        vr22,     vr22
    vneg.w        vr18,     vr18
    vssrarni.h.w  vr18,     vr22,      0        // out[1]

    vsadd.h       vr2,      vr1,       vr10     // out[2]
    vssub.h       vr11,     vr1,       vr10     // t14a
    vssub.h       vr12,     vr16,      vr6      // t15a
    vsadd.h       vr9,      vr16,      vr6      // out[13]
    vsllwil.w.h   vr22,     vr9,       0
    vexth.w.h     vr9,      vr9
    vneg.w        vr22,     vr22
    vneg.w        vr9,      vr9
    vssrarni.h.w  vr9,      vr22,      0        // out[13]

    vldrepl.w     vr20,     t0,        0        // 2896
    vmul_vmadd_w vr17, vr7, vr20, vr20, vr6, vr10
    vmul_vmsub_w vr17, vr7, vr20, vr20, vr16, vr1
    vssrarni.h.w  vr10,     vr6,       12       // out[7]

    vsllwil.w.h   vr7,      vr10,      0
    vexth.w.h     vr10,     vr10
    vneg.w        vr7,      vr7
    vneg.w        vr10,     vr10
    vssrarni.h.w  vr10,     vr7,       0
    vssrarni.h.w  vr1,      vr16,      12       // out[8]

    vmul_vmsub_w vr0, vr8, vr20, vr20, vr16, vr17
    vmul_vmadd_w vr0, vr8, vr20, vr20, vr6, vr7
    vssrarni.h.w  vr17,     vr16,      12       // out[11]

    vsllwil.w.h   vr0,      vr17,      0
    vexth.w.h     vr17,     vr17
    vneg.w        vr0,      vr0
    vneg.w        vr17,     vr17
    vssrarni.h.w  vr17,     vr0,       0
    vssrarni.h.w  vr7,      vr6,       12       // out[4]

    vmul_vmsub_w vr4, vr19, vr20, vr20, vr16, vr0
    vmul_vmadd_w vr4, vr19, vr20, vr20, vr6, vr8
    vssrarni.h.w  vr0,      vr16,      12       // out[9]

    vsllwil.w.h   vr4,      vr0,       0
    vexth.w.h     vr0,      vr0
    vneg.w        vr4,      vr4
    vneg.w        vr0,      vr0
    vssrarni.h.w  vr0,      vr4,       0
    vssrarni.h.w  vr8,      vr6,       12       // out[6]

    vmul_vmadd_w vr11, vr12, vr20, vr20, vr6, vr4
    vmul_vmsub_w vr11, vr12, vr20, vr20, vr16, vr19
    vssrarni.h.w  vr4,      vr6,       12       // out[5]

    vsllwil.w.h   vr24,     vr4,       0
    vexth.w.h     vr4,      vr4
    vneg.w        vr24,     vr24
    vneg.w        vr4,      vr4
    vssrarni.h.w  vr4,      vr24,      0
    vssrarni.h.w  vr19,     vr16,      12       // out[10]

.ifnb \transpose8x8
    LSX_TRANSPOSE8x8_H vr14, vr18, vr2, vr5, vr7, vr4, vr8, vr10, \
                       vr14, vr18, vr2, vr5, vr7, vr4, vr8, vr10, \
                       vr6, vr11, vr12, vr16, vr20, vr21, vr22, vr23

    LSX_TRANSPOSE8x8_H vr1, vr0, vr19, vr17, vr3, vr9, vr13, vr15, \
                       vr1, vr0, vr19, vr17, vr3, vr9, vr13, vr15, \
                       vr6, vr11, vr12, vr16, vr20, vr21, vr22, vr23
.endif

.ifnb \shift
.irp i, vr14, vr18, vr2, vr5, vr7, vr4, vr8, vr10, \
    vr1, vr0, vr19, vr17, vr3, vr9, vr13, vr15
    vsrari.h      \i,       \i,       \shift
.endr
.endif

.ifnb \vst
    vst_x16 t1, 0, 16, vr14, vr18, vr2, vr5, vr7, vr4, vr8, vr10, \
            vr1, vr0, vr19, vr17, vr3, vr9, vr13, vr15
.endif
// out0 out1 out2 out3 out4 out5 out6 out7
// vr14 vr18 vr2  vr5  vr7  vr4  vr8  vr10
// out8 out9 out10 out11 out12 out13 out14 out15
// vr1  vr0  vr19  vr17  vr3   vr9   vr13  vr15
.endm // adst16_core_lsx

.macro adst16_core_finish_lsx in0, in1, in2, in3, in4, in5, in6, in7
    fld.d         f20,      t2,       0
    fldx.d        f21,      t2,       a1
    fld.d         f22,      t3,       0
    fldx.d        f23,      t3,       a1

    alsl.d        t2,       a1,       t2,     2
    alsl.d        t3,       a1,       t3,     2

    fld.d         f24,      t2,       0
    fldx.d        f25,      t2,       a1
    fld.d         f26,      t3,       0
    fldx.d        f27,      t3,       a1

.irp i, vr20, vr21, vr22, vr23, vr24, vr25, vr26, vr27
    vsllwil.hu.bu \i,       \i,       0
.endr

.irp i, \in0, \in1, \in2, \in3, \in4, \in5, \in6, \in7
    vsrari.h      \i,       \i,       4
.endr

    vadd.h        vr20,     vr20,     \in0
    vadd.h        vr21,     vr21,     \in1
    vadd.h        vr22,     vr22,     \in2
    vadd.h        vr23,     vr23,     \in3
    vadd.h        vr24,     vr24,     \in4
    vadd.h        vr25,     vr25,     \in5
    vadd.h        vr26,     vr26,     \in6
    vadd.h        vr27,     vr27,     \in7

    vssrani.bu.h  vr21,     vr20,     0
    vssrani.bu.h  vr23,     vr22,     0
    vssrani.bu.h  vr25,     vr24,     0
    vssrani.bu.h  vr27,     vr26,     0

    vstelm.d      vr21,     t4,       0,     0
    vstelm.d      vr21,     t5,       0,     1

    alsl.d        t4,       a1,       t4,    1
    alsl.d        t5,       a1,       t5,    1
    vstelm.d      vr23,     t4,       0,     0
    vstelm.d      vr23,     t5,       0,     1

    alsl.d        t4,       a1,       t4,    1
    alsl.d        t5,       a1,       t5,    1
    vstelm.d      vr25,     t4,       0,     0
    vstelm.d      vr25,     t5,       0,     1

    alsl.d        t4,       a1,       t4,    1
    alsl.d        t5,       a1,       t5,    1
    vstelm.d      vr27,     t4,       0,     0
    vstelm.d      vr27,     t5,       0,     1

.endm // adst16_core_finish_lsx

function inv_txfm_add_dct_adst_8x16_8bpc_lsx
    addi.d        sp,       sp,       -64
    fst.d         f24,      sp,       0
    fst.d         f25,      sp,       8
    fst.d         f26,      sp,       16
    fst.d         f27,      sp,       24
    fst.d         f28,      sp,       32
    fst.d         f29,      sp,       40
    fst.d         f30,      sp,       48
    fst.d         f31,      sp,       56

    vld_x8 a2, 0, 32, vr0, vr1, vr2, vr3, vr4, vr5, vr6, vr7

    la.local      t0,       idct_coeffs

    dct_8x8_core_lsx vr0, vr1, vr2, vr3, vr4, vr5, vr6, vr7, \
                     vr11, vr12, vr13, vr14, vr15, vr16, vr17, vr18, rect2_lsx

    vld_x8 a2, 16, 32, vr0, vr1, vr2, vr3, vr4, vr5, vr6, vr7

    dct_8x8_core_lsx vr0, vr1, vr2, vr3, vr4, vr5, vr6, vr7, \
                     vr19, vr24, vr25, vr26, vr27, vr28, vr29, vr30, rect2_lsx

.irp i, vr11, vr12, vr13, vr14, vr15, vr16, vr17, vr18, \
        vr19, vr24, vr25, vr26, vr27, vr28, vr29, vr30
    vsrari.h      \i,       \i,       1
.endr

    vreplgr2vr.h  vr23,     zero

.irp i, 0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240
    vst           vr23,     a2,       \i
.endr

    LSX_TRANSPOSE8x8_H vr11, vr12, vr13, vr14, vr15, vr16, vr17, vr18, \
                       vr0, vr1, vr2, vr3, vr4, vr5, vr6, vr7, \
                       vr8, vr9, vr10, vr20, vr21, vr22, vr23, vr31

    LSX_TRANSPOSE8x8_H vr19, vr24, vr25, vr26, vr27, vr28, vr29, vr30, \
                       vr8, vr9, vr10, vr11, vr12, vr13, vr14, vr15, \
                       vr16, vr17, vr18, vr20, vr21, vr22, vr23, vr31

    adst16_core_lsx , ,

    addi.d        t2,       a0,       0
    alsl.d        t3,       a1,       a0,     1
    addi.d        t4,       a0,       0
    add.d         t5,       a1,       a0

    adst16_core_finish_lsx vr14, vr18, vr2, vr5, vr7, vr4, vr8, vr10

    alsl.d        t2,       a1,       t2,    2
    alsl.d        t3,       a1,       t3,    2

    alsl.d        t4,       a1,       t4,    1
    alsl.d        t5,       a1,       t5,    1

    adst16_core_finish_lsx vr1, vr0, vr19, vr17, vr3, vr9, vr13, vr15

    fld.d         f24,      sp,       0
    fld.d         f25,      sp,       8
    fld.d         f26,      sp,       16
    fld.d         f27,      sp,       24
    fld.d         f28,      sp,       32
    fld.d         f29,      sp,       40
    fld.d         f30,      sp,       48
    fld.d         f31,      sp,       56
    addi.d        sp,       sp,       64
endfunc
