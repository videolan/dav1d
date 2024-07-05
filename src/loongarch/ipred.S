/*
 * Copyright © 2024, VideoLAN and dav1d authors
 * Copyright © 2024, Loongson Technology Corporation Limited
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

.macro ipred_dc_gen topleft, width, height
    add.d          t0,      \width,  \height //dc
    srai.d         t0,      t0,      1
    addi.d         t3,      \topleft,1

    or             t1,      zero,    zero  //data index
    srai.d         t2,      \width,  4     //loop param
    beqz           t2,      2f

1:  // width/16
    vldx           vr0,     t3,      t1
    vhaddw.hu.bu   vr0,     vr0,     vr0
    vhaddw.wu.hu   vr0,     vr0,     vr0
    vhaddw.du.wu   vr0,     vr0,     vr0
    vhaddw.qu.du   vr0,     vr0,     vr0

    vpickve2gr.du  t4,      vr0,     0
    add.d          t0,      t0,      t4

    addi.d         t1,      t1,      16
    addi.d         t2,      t2,      -1
    bnez           t2,      1b
    b              4f

2:  // &8
    andi           t2,      \width,  8
    beqz           t2,      3f

    vxor.v         vr0,     vr0,     vr0
    fldx.d         f0,      t3,      t1

    vhaddw.hu.bu   vr0,     vr0,     vr0
    vhaddw.wu.hu   vr0,     vr0,     vr0
    vhaddw.du.wu   vr0,     vr0,     vr0

    vpickve2gr.du  t4,      vr0,     0
    add.d          t0,      t0,      t4
    addi.d         t1,      t1,      8
    b              4f

3:  // &4
    andi           t2,      \width,  4
    beqz           t2,      4f

    vxor.v         vr0,     vr0,     vr0
    fldx.s         f0,      t3,      t1

    vhaddw.hu.bu   vr0,     vr0,     vr0
    vhaddw.wu.hu   vr0,     vr0,     vr0

    vpickve2gr.wu  t4,      vr0,     0
    add.d          t0,      t0,      t4
    addi.d         t1,      t1,      4

4:
    addi.d         t3,      \topleft,0
    srai.d         t2,      \height, 4     //loop param
    beqz           t2,      8f

7:  // height/16
    addi.d         t3,      t3,      -16
    vld            vr0,     t3,      0

    vhaddw.hu.bu   vr0,     vr0,     vr0
    vhaddw.wu.hu   vr0,     vr0,     vr0
    vhaddw.du.wu   vr0,     vr0,     vr0
    vhaddw.qu.du   vr0,     vr0,     vr0

    vpickve2gr.du  t4,      vr0,     0
    add.d          t0,      t0,      t4

    addi.d         t2,      t2,      -1
    bnez           t2,      7b
    b              10f

8:  // &8
    andi           t2,      \height, 8
    beqz           t2,      9f

    addi.d         t3,      t3,      -8
    vxor.v         vr0,     vr0,     vr0
    fld.d          f0,      t3,      0

    vhaddw.hu.bu   vr0,     vr0,     vr0
    vhaddw.wu.hu   vr0,     vr0,     vr0
    vhaddw.du.wu   vr0,     vr0,     vr0

    vpickve2gr.du  t4,      vr0,     0
    add.d          t0,      t0,      t4
    b              10f

9:  // &4
    andi           t2,      \height, 4
    beqz           t2,      10f

    addi.d         t3,      t3,      -4
    vxor.v         vr0,     vr0,     vr0
    fld.s          f0,      t3,      0

    vhaddw.hu.bu   vr0,     vr0,     vr0
    vhaddw.wu.hu   vr0,     vr0,     vr0

    vpickve2gr.wu  t4,      vr0,     0
    add.d          t0,      t0,      t4

10:
    add.d          t1,      \width,  \height
    ctz.w          t1,      t1
    sra.w          t0,      t0,      t1

    // w != h
    beq            \width,  \height, 16f
    add.d          t2,      \height, \height
    add.d          t3,      \width,  \width
    slt            t2,      t2,      \width
    slt            t3,      t3,      \height
    or             t2,      t2,      t3
    li.w           t3,      0x3334
    maskeqz        t1,      t3,      t2
    li.w           t3,      0x5556
    masknez        t2,      t3,      t2
    or             t1,      t1,      t2
    mul.w          t0,      t0,      t1
    srai.w         t0,      t0,      16

16:
.endm

.macro ipred_splat_dc dst, stride, width, height, dc
    li.w           t1,      4
    blt            t1,      \width,  2f

    li.w           t1,      0x01010101
    mulw.d.wu      t1,      \dc,     t1
    beqz           \height, 7f
    or             t2,      \dst,    \dst
1:  // width <= 4
    st.w           t1,      t2,      0
    add.d          t2,      t2,      \stride
    addi.d         \height, \height, -1
    bnez           \height, 1b
    b              7f

2:  //width > 4
    li.d           t1,      0x0101010101010101
    mul.d          t1,      \dc,     t1
    vreplgr2vr.d   vr0,     t1
    or             t4,      \dst,    \dst
    beqz           \height, 7f

3:
    andi           t5,      \width,  64
    beqz           t5,      4f
    vst            vr0,     t4,      0
    vst            vr0,     t4,      16
    vst            vr0,     t4,      32
    vst            vr0,     t4,      48
    b              6f

4:
    andi           t5,      \width,  32
    beqz           t5,      41f
    vst            vr0,     t4,      0
    vst            vr0,     t4,      16
    b              6f

41:
    andi           t5,      \width,  16
    beqz           t5,      5f
    vst            vr0,     t4,      0
    b              6f

5:
    fst.d          f0,      t4,      0

6:
    add.d          t4,      t4,      \stride
    addi.d         \height, \height, -1
    bnez           \height, 3b

7:
.endm

.macro ipred_dc_gen_top topleft, width
    srai.d         t0,      \width,  1
    addi.d         t1,      \topleft,1

    srai.d         t2,      \width,  4
    beqz           t2,      2f
1:
    vld            vr0,     t1,      0
    vhaddw.hu.bu   vr0,     vr0,     vr0
    vhaddw.wu.hu   vr0,     vr0,     vr0
    vhaddw.du.wu   vr0,     vr0,     vr0
    vhaddw.qu.du   vr0,     vr0,     vr0

    vpickve2gr.du  t3,      vr0,     0
    add.d          t0,      t0,      t3

    addi.d         t1,      t1,      16
    addi.d         t2,      t2,      -1
    bnez           t2,      1b
    b              4f

2:  // &8
    andi           t2,      \width,  8
    beqz           t2,      3f

    vxor.v         vr0,     vr0,     vr0
    fld.d          f0,      t1,      0

    vhaddw.hu.bu   vr0,     vr0,     vr0
    vhaddw.wu.hu   vr0,     vr0,     vr0
    vhaddw.du.wu   vr0,     vr0,     vr0

    vpickve2gr.du  t2,      vr0,     0
    add.d          t0,      t0,      t2

    addi.d         t1,      t1,      8
    b              4f

3:  // &4
    andi           t2,      \width,  4
    beqz           t2,      4f

    vxor.v         vr0,     vr0,     vr0
    fld.s          f0,      t1,      0

    vhaddw.hu.bu   vr0,     vr0,     vr0
    vhaddw.wu.hu   vr0,     vr0,     vr0

    vpickve2gr.du  t2,      vr0,     0
    add.d          t0,      t0,      t2
    addi.d         t1,      t1,      4

4:
    ctz.w          t1,      \width
    sra.w          t0,      t0,      t1
.endm

.macro ipred_dc_gen_left topleft, height
    srai.d         t0,      \height, 1
    srai.d         t2,      \height, 4     //loop param
    beqz           t2,      8f

7:  // height/16
    addi.d         \topleft,\topleft,-16
    vld            vr0,     \topleft,0

    vhaddw.hu.bu   vr0,     vr0,     vr0
    vhaddw.wu.hu   vr0,     vr0,     vr0
    vhaddw.du.wu   vr0,     vr0,     vr0
    vhaddw.qu.du   vr0,     vr0,     vr0

    vpickve2gr.du  t4,      vr0,     0
    add.d          t0,      t0,      t4

    addi.d         t2,      t2,      -1
    bnez           t2,      7b
    b              10f

8:  // &8
    andi           t2,      \height, 8
    beqz           t2,      9f

    addi.d         \topleft,\topleft,-8
    vxor.v         vr0,     vr0,     vr0
    fld.d          f0,      \topleft,0

    vhaddw.hu.bu   vr0,     vr0,     vr0
    vhaddw.wu.hu   vr0,     vr0,     vr0
    vhaddw.du.wu   vr0,     vr0,     vr0

    vpickve2gr.du  t4,      vr0,     0
    add.d          t0,      t0,      t4
    b              10f

9:  // &4
    andi           t2,      \height, 4
    beqz           t2,      10f

    addi.d         \topleft,\topleft,-4
    vxor.v         vr0,     vr0,     vr0
    fld.s          f0,      \topleft,0

    vhaddw.hu.bu   vr0,     vr0,     vr0
    vhaddw.wu.hu   vr0,     vr0,     vr0

    vpickve2gr.wu  t4,      vr0,     0
    add.d          t0,      t0,      t4

10:
    ctz.w          t1,      \height
    sra.w          t0,      t0,      t1

.endm

// void ipred_dc_lsx(pixel *dst, const ptrdiff_t stride,
//                   const pixel *const topleft,
//                   const int width, const int height, const int a,
//                   const int max_width, const int max_height
//                   HIGHBD_DECL_SUFFIX)
function ipred_dc_8bpc_lsx
    ipred_dc_gen   a2, a3, a4
    ipred_splat_dc a0, a1, a3, a4, t0

endfunc

// void ipred_dc_128_lsx(pixel *dst, const ptrdiff_t stride,
//                       const pixel *const topleft,
//                       const int width, const int height, const int a,
//                       const int max_width, const int max_height
//                       HIGHBD_DECL_SUFFIX)
function ipred_dc_128_8bpc_lsx
    li.w           t0,      128
    ipred_splat_dc a0, a1, a3, a4, t0

endfunc

// void ipred_dc_top_c(pixel *dst, const ptrdiff_t stride,
//                     const pixel *const topleft,
//                     const int width, const int height, const int a,
//                     const int max_width, const int max_height
//                     HIGHBD_DECL_SUFFIX)
function ipred_dc_top_8bpc_lsx
    ipred_dc_gen_top a2, a3
    ipred_splat_dc   a0, a1, a3, a4, t0

endfunc

// void ipred_dc_left_c(pixel *dst, const ptrdiff_t stride,
//                      const pixel *const topleft,
//                      const int width, const int height, const int a,
//                      const int max_width, const int max_height
//                      HIGHBD_DECL_SUFFIX)
function ipred_dc_left_8bpc_lsx
    ipred_dc_gen_left a2, a4
    ipred_splat_dc    a0, a1, a3, a4, t0

endfunc

.macro pixel_set_8bpc dst_ptr, src_ptr, width
    vldrepl.b      vr0,     \src_ptr, 0
1:
    andi           a5,      \width,   64
    beqz           a5,      2f

    vst            vr0,     \dst_ptr, 0
    vst            vr0,     \dst_ptr, 16
    vst            vr0,     \dst_ptr, 32
    vst            vr0,     \dst_ptr, 48
    b              6f
2:
    andi           a5,      \width,   32
    beqz           a5,      3f

    vst            vr0,     \dst_ptr, 0
    vst            vr0,     \dst_ptr, 16
    b              6f
3:
    andi           a5,      \width,   16
    beqz           a5,      4f

    vst            vr0,     \dst_ptr, 0
    b              6f
4:
    andi           a5,      \width,   8
    beqz           a5,      5f

    fst.d          f0,      \dst_ptr, 0
    b              6f
5:
    andi           a5,      \width,   4
    beqz           a5,      6f

    fst.s          f0,      \dst_ptr, 0
6:
.endm

// void ipred_h_c(pixel *dst, const ptrdiff_t stride,
//                const pixel *const topleft,
//                const int width, const int height, const int a,
//                const int max_width, const int max_height
//                HIGHBD_DECL_SUFFIX)
function ipred_h_8bpc_lsx
    beqz           a4,      .IPRED_H_END
.IPRED_H_LOOP:
    addi.d         a2,      a2,      -1

    pixel_set_8bpc a0, a2, a3

    add.d          a0,      a0,      a1
    addi.d         a4,      a4,      -1
    bnez           a4,      .IPRED_H_LOOP

.IPRED_H_END:
endfunc

.macro pixel_copy_8bpc dst_ptr, src_ptr, width
1:
    andi           a5,      \width,   64
    beqz           a5,      2f

    vld            vr0,     \src_ptr, 0
    vld            vr1,     \src_ptr, 16
    vld            vr2,     \src_ptr, 32
    vld            vr3,     \src_ptr, 48

    vst            vr0,     \dst_ptr, 0
    vst            vr1,     \dst_ptr, 16
    vst            vr2,     \dst_ptr, 32
    vst            vr3,     \dst_ptr, 48

    b              6f
2:
    andi           a5,      \width,   32
    beqz           a5,      3f

    vld            vr0,     \src_ptr, 0
    vld            vr1,     \src_ptr, 16

    vst            vr0,     \dst_ptr, 0
    vst            vr1,     \dst_ptr, 16

    b              6f
3:
    andi           a5,      \width,   16
    beqz           a5,      4f

    vld            vr0,     \src_ptr, 0
    vst            vr0,     \dst_ptr, 0

    b              6f
4:
    andi           a5,      \width,   8
    beqz           a5,      5f

    fld.d          f0,      \src_ptr, 0
    fst.d          f0,      \dst_ptr, 0

    b              6f
5:
    andi           a5,      \width,   4
    beqz           a5,      6f

    fld.s          f0,      \src_ptr, 0
    fst.s          f0,      \dst_ptr, 0
6:
.endm

// void ipred_v_lsx(pixel *dst, const ptrdiff_t stride,
//                  const pixel *const topleft,
//                  const int width, const int height, const int a,
//                  const int max_width, const int max_height
//                  HIGHBD_DECL_SUFFIX)
function ipred_v_8bpc_lsx
    beqz           a4,      .IPRED_V_END
    addi.d         a2,      a2,      1
.IPRED_V_LOOP:
    pixel_copy_8bpc  a0, a2, a3

    add.d          a0,      a0,      a1
    addi.d         a4,      a4,      -1
    bnez           a4,      .IPRED_V_LOOP

.IPRED_V_END:
endfunc