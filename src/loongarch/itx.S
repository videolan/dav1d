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
