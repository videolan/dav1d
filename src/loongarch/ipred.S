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

// void ipred_paeth_lsx(pixel *dst, const ptrdiff_t stride,
//                      const pixel *const tl_ptr,
//                      const int width, const int height, const int a,
//                      const int max_width, const int max_height
//                      HIGHBD_DECL_SUFFIX)
function ipred_paeth_8bpc_lsx
    vldrepl.b      vr0,     a2,      0    //topleft
    vsllwil.hu.bu  vr0,     vr0,     0
    or             a6,      a2,      a2
    addi.d         a7,      a2,      1

.IPRED_PAETH_H_LOOP:
    addi.d         a6,      a6,      -1
    vldrepl.b      vr1,     a6,      0   //left
    vsllwil.hu.bu  vr1,     vr1,     0

.IPRED_PAETH_W_LOOP64:
    andi           a5,      a3,      64
    beqz           a5,      .IPRED_PAETH_W_LOOP32

    vld            vr2,     a7,      0   //top
    vpermi.w       vr9,     vr2,     0x0e
    vsllwil.hu.bu  vr2,     vr2,     0
    vsllwil.hu.bu  vr9,     vr9,     0

    vabsd.hu       vr5,     vr0,     vr1  //tdiff
    vabsd.hu       vr4,     vr0,     vr2  //ldiff
    vabsd.hu       vr10,    vr0,     vr9

    vadd.h         vr3,     vr0,     vr0
    vadd.h         vr6,     vr1,     vr2
    vadd.h         vr11,    vr1,     vr9
    vabsd.hu       vr6,     vr3,     vr6  //tldiff
    vabsd.hu       vr11,    vr3,     vr11 //tldiff

    vsle.hu        vr3,     vr5,     vr6
    vbitsel.v      vr7,     vr0,     vr2,    vr3
    vsle.hu        vr3,     vr4,     vr5
    vsle.hu        vr8,     vr4,     vr6
    vand.v         vr3,     vr3,     vr8
    vbitsel.v      vr3,     vr7,     vr1,    vr3
    vsrlni.b.h     vr3,     vr3,     0

    vsle.hu        vr12,    vr5,     vr11
    vbitsel.v      vr7,     vr0,     vr9,    vr12
    vsle.hu        vr12,    vr10,    vr5
    vsle.hu        vr8,     vr10,    vr11
    vand.v         vr12,    vr12,    vr8
    vbitsel.v      vr12,    vr7,     vr1,    vr12
    vsrlni.b.h     vr12,    vr12,    0

    vpermi.w       vr12,    vr3,     0x44

    vst            vr12,    a0,      0

    vld            vr2,     a7,      16   //top
    vpermi.w       vr9,     vr2,     0x0e
    vsllwil.hu.bu  vr2,     vr2,     0
    vsllwil.hu.bu  vr9,     vr9,     0

    vabsd.hu       vr5,     vr0,     vr1  //tdiff
    vabsd.hu       vr4,     vr0,     vr2  //ldiff
    vabsd.hu       vr10,    vr0,     vr9

    vadd.h         vr3,     vr0,     vr0
    vadd.h         vr6,     vr1,     vr2
    vadd.h         vr11,    vr1,     vr9
    vabsd.hu       vr6,     vr3,     vr6  //tldiff
    vabsd.hu       vr11,    vr3,     vr11 //tldiff

    vsle.hu        vr3,     vr5,     vr6
    vbitsel.v      vr7,     vr0,     vr2,    vr3
    vsle.hu        vr3,     vr4,     vr5
    vsle.hu        vr8,     vr4,     vr6
    vand.v         vr3,     vr3,     vr8
    vbitsel.v      vr3,     vr7,     vr1,    vr3
    vsrlni.b.h     vr3,     vr3,     0

    vsle.hu        vr12,    vr5,     vr11
    vbitsel.v      vr7,     vr0,     vr9,    vr12
    vsle.hu        vr12,    vr10,    vr5
    vsle.hu        vr8,     vr10,    vr11
    vand.v         vr12,    vr12,    vr8
    vbitsel.v      vr12,    vr7,     vr1,    vr12
    vsrlni.b.h     vr12,    vr12,    0

    vpermi.w       vr12,    vr3,     0x44

    vst            vr12,    a0,      16

    vld            vr2,     a7,      32   //top
    vpermi.w       vr9,     vr2,     0x0e
    vsllwil.hu.bu  vr2,     vr2,     0
    vsllwil.hu.bu  vr9,     vr9,     0

    vabsd.hu       vr5,     vr0,     vr1  //tdiff
    vabsd.hu       vr4,     vr0,     vr2  //ldiff
    vabsd.hu       vr10,    vr0,     vr9

    vadd.h         vr3,     vr0,     vr0
    vadd.h         vr6,     vr1,     vr2
    vadd.h         vr11,    vr1,     vr9
    vabsd.hu       vr6,     vr3,     vr6  //tldiff
    vabsd.hu       vr11,    vr3,     vr11 //tldiff

    vsle.hu        vr3,     vr5,     vr6
    vbitsel.v      vr7,     vr0,     vr2,    vr3
    vsle.hu        vr3,     vr4,     vr5
    vsle.hu        vr8,     vr4,     vr6
    vand.v         vr3,     vr3,     vr8
    vbitsel.v      vr3,     vr7,     vr1,    vr3
    vsrlni.b.h     vr3,     vr3,     0

    vsle.hu        vr12,    vr5,     vr11
    vbitsel.v      vr7,     vr0,     vr9,    vr12
    vsle.hu        vr12,    vr10,    vr5
    vsle.hu        vr8,     vr10,    vr11
    vand.v         vr12,    vr12,    vr8
    vbitsel.v      vr12,    vr7,     vr1,    vr12
    vsrlni.b.h     vr12,    vr12,    0

    vpermi.w       vr12,    vr3,     0x44

    vst            vr12,    a0,      32

    vld            vr2,     a7,      48   //top
    vpermi.w       vr9,     vr2,     0x0e
    vsllwil.hu.bu  vr2,     vr2,     0
    vsllwil.hu.bu  vr9,     vr9,     0

    vabsd.hu       vr5,     vr0,     vr1  //tdiff
    vabsd.hu       vr4,     vr0,     vr2  //ldiff
    vabsd.hu       vr10,    vr0,     vr9

    vadd.h         vr3,     vr0,     vr0
    vadd.h         vr6,     vr1,     vr2
    vadd.h         vr11,    vr1,     vr9
    vabsd.hu       vr6,     vr3,     vr6  //tldiff
    vabsd.hu       vr11,    vr3,     vr11 //tldiff

    vsle.hu        vr3,     vr5,     vr6
    vbitsel.v      vr7,     vr0,     vr2,    vr3
    vsle.hu        vr3,     vr4,     vr5
    vsle.hu        vr8,     vr4,     vr6
    vand.v         vr3,     vr3,     vr8
    vbitsel.v      vr3,     vr7,     vr1,    vr3
    vsrlni.b.h     vr3,     vr3,     0

    vsle.hu        vr12,    vr5,     vr11
    vbitsel.v      vr7,     vr0,     vr9,    vr12
    vsle.hu        vr12,    vr10,    vr5
    vsle.hu        vr8,     vr10,    vr11
    vand.v         vr12,    vr12,    vr8
    vbitsel.v      vr12,    vr7,     vr1,    vr12
    vsrlni.b.h     vr12,    vr12,    0

    vpermi.w       vr12,    vr3,     0x44

    vst            vr12,    a0,      48

    b              .IPRED_PAETH_W_LOOPEND

.IPRED_PAETH_W_LOOP32:
    andi           a5,      a3,      32
    beqz           a5,      .IPRED_PAETH_W_LOOP16

    vld            vr2,     a7,      0   //top
    vpermi.w       vr9,     vr2,     0x0e
    vsllwil.hu.bu  vr2,     vr2,     0
    vsllwil.hu.bu  vr9,     vr9,     0

    vabsd.hu       vr5,     vr0,     vr1  //tdiff
    vabsd.hu       vr4,     vr0,     vr2  //ldiff
    vabsd.hu       vr10,    vr0,     vr9

    vadd.h         vr3,     vr0,     vr0
    vadd.h         vr6,     vr1,     vr2
    vadd.h         vr11,    vr1,     vr9
    vabsd.hu       vr6,     vr3,     vr6  //tldiff
    vabsd.hu       vr11,    vr3,     vr11 //tldiff

    vsle.hu        vr3,     vr5,     vr6
    vbitsel.v      vr7,     vr0,     vr2,    vr3
    vsle.hu        vr3,     vr4,     vr5
    vsle.hu        vr8,     vr4,     vr6
    vand.v         vr3,     vr3,     vr8
    vbitsel.v      vr3,     vr7,     vr1,    vr3
    vsrlni.b.h     vr3,     vr3,     0

    vsle.hu        vr12,    vr5,     vr11
    vbitsel.v      vr7,     vr0,     vr9,    vr12
    vsle.hu        vr12,    vr10,    vr5
    vsle.hu        vr8,     vr10,    vr11
    vand.v         vr12,    vr12,    vr8
    vbitsel.v      vr12,    vr7,     vr1,    vr12
    vsrlni.b.h     vr12,    vr12,    0

    vpermi.w       vr12,    vr3,     0x44

    vst            vr12,    a0,      0

    vld            vr2,     a7,      16   //top
    vpermi.w       vr9,     vr2,     0x0e
    vsllwil.hu.bu  vr2,     vr2,     0
    vsllwil.hu.bu  vr9,     vr9,     0

    vabsd.hu       vr5,     vr0,     vr1  //tdiff
    vabsd.hu       vr4,     vr0,     vr2  //ldiff
    vabsd.hu       vr10,    vr0,     vr9

    vadd.h         vr3,     vr0,     vr0
    vadd.h         vr6,     vr1,     vr2
    vadd.h         vr11,    vr1,     vr9
    vabsd.hu       vr6,     vr3,     vr6  //tldiff
    vabsd.hu       vr11,    vr3,     vr11 //tldiff

    vsle.hu        vr3,     vr5,     vr6
    vbitsel.v      vr7,     vr0,     vr2,    vr3
    vsle.hu        vr3,     vr4,     vr5
    vsle.hu        vr8,     vr4,     vr6
    vand.v         vr3,     vr3,     vr8
    vbitsel.v      vr3,     vr7,     vr1,    vr3
    vsrlni.b.h     vr3,     vr3,     0

    vsle.hu        vr12,    vr5,     vr11
    vbitsel.v      vr7,     vr0,     vr9,    vr12
    vsle.hu        vr12,    vr10,    vr5
    vsle.hu        vr8,     vr10,    vr11
    vand.v         vr12,    vr12,    vr8
    vbitsel.v      vr12,    vr7,     vr1,    vr12
    vsrlni.b.h     vr12,    vr12,    0

    vpermi.w       vr12,    vr3,     0x44

    vst            vr12,    a0,      16

    b              .IPRED_PAETH_W_LOOPEND

.IPRED_PAETH_W_LOOP16:
    andi           a5,      a3,      16
    beqz           a5,      .IPRED_PAETH_W_LOOP8

    vld            vr2,     a7,      0   //top
    vpermi.w       vr9,     vr2,     0x0e
    vsllwil.hu.bu  vr2,     vr2,     0
    vsllwil.hu.bu  vr9,     vr9,     0

    vabsd.hu       vr5,     vr0,     vr1  //tdiff
    vabsd.hu       vr4,     vr0,     vr2  //ldiff
    vabsd.hu       vr10,    vr0,     vr9

    vadd.h         vr3,     vr0,     vr0
    vadd.h         vr6,     vr1,     vr2
    vadd.h         vr11,    vr1,     vr9
    vabsd.hu       vr6,     vr3,     vr6  //tldiff
    vabsd.hu       vr11,    vr3,     vr11 //tldiff

    vsle.hu        vr3,     vr5,     vr6
    vbitsel.v      vr7,     vr0,     vr2,    vr3
    vsle.hu        vr3,     vr4,     vr5
    vsle.hu        vr8,     vr4,     vr6
    vand.v         vr3,     vr3,     vr8
    vbitsel.v      vr3,     vr7,     vr1,    vr3
    vsrlni.b.h     vr3,     vr3,     0

    vsle.hu        vr12,    vr5,     vr11
    vbitsel.v      vr7,     vr0,     vr9,    vr12
    vsle.hu        vr12,    vr10,    vr5
    vsle.hu        vr8,     vr10,    vr11
    vand.v         vr12,    vr12,    vr8
    vbitsel.v      vr12,    vr7,     vr1,    vr12
    vsrlni.b.h     vr12,    vr12,    0

    vpermi.w       vr12,    vr3,     0x44

    vst            vr12,    a0,      0

    b              .IPRED_PAETH_W_LOOPEND

.IPRED_PAETH_W_LOOP8:
    andi           a5,      a3,      8
    beqz           a5,      .IPRED_PAETH_W_LOOP4

    fld.d          f2,      a7,      0   //top
    vsllwil.hu.bu  vr2,     vr2,     0

    vabsd.hu       vr5,     vr0,     vr1  //tdiff
    vabsd.hu       vr4,     vr0,     vr2  //ldiff

    vadd.h         vr3,     vr0,     vr0
    vadd.h         vr6,     vr1,     vr2
    vabsd.hu       vr6,     vr3,     vr6 //tldiff

    vsle.hu        vr3,     vr5,     vr6
    vbitsel.v      vr7,     vr0,     vr2,    vr3
    vsle.hu        vr3,     vr4,     vr5
    vsle.hu        vr8,     vr4,     vr6
    vand.v         vr3,     vr3,     vr8
    vbitsel.v      vr3,     vr7,     vr1,    vr3
    vsrlni.b.h     vr3,     vr3,     0
    fst.d          f3,      a0,      0

    b              .IPRED_PAETH_W_LOOPEND

.IPRED_PAETH_W_LOOP4:
    andi           a5,      a3,      4
    beqz           a5,      .IPRED_PAETH_W_LOOPEND

    fld.s          f2,      a7,      0   //top
    vsllwil.hu.bu  vr2,     vr2,     0

    vabsd.hu       vr5,     vr0,     vr1  //tdiff
    vabsd.hu       vr4,     vr0,     vr2  //ldiff

    vadd.h         vr3,     vr0,     vr0
    vadd.h         vr6,     vr1,     vr2
    vabsd.hu       vr6,     vr3,     vr6 //tldiff

    vsle.hu        vr3,     vr5,     vr6
    vbitsel.v      vr7,     vr0,     vr2,    vr3
    vsle.hu        vr3,     vr4,     vr5
    vsle.hu        vr8,     vr4,     vr6
    vand.v         vr3,     vr3,     vr8
    vbitsel.v      vr3,     vr7,     vr1,    vr3
    vsrlni.b.h     vr3,     vr3,     0
    fst.s          f3,      a0,      0

    b              .IPRED_PAETH_W_LOOPEND

.IPRED_PAETH_W_LOOPEND:
    add.d         a0,       a0,      a1
    addi.d        a4,       a4,      -1
    bnez          a4,       .IPRED_PAETH_H_LOOP
endfunc

const dav1d_sm_weights
    .byte  0,   0
    // bs = 2
    .byte  255, 128
    // bs = 4
    .byte  255, 149,  85,  64
    // bs = 8
    .byte  255, 197, 146, 105,  73,  50,  37,  32
    // bs = 16
    .byte  255, 225, 196, 170, 145, 123, 102,  84
    .byte  68,  54,  43,  33,  26,  20,  17,  16
    // bs = 32
    .byte  255, 240, 225, 210, 196, 182, 169, 157
    .byte  145, 133, 122, 111, 101,  92,  83,  74
    .byte  66,  59,  52,  45,  39,  34,  29,  25
    .byte  21,  17,  14,  12,  10,   9,   8,   8
    // bs = 64
    .byte  255, 248, 240, 233, 225, 218, 210, 203
    .byte  196, 189, 182, 176, 169, 163, 156, 150
    .byte  144, 138, 133, 127, 121, 116, 111, 106
    .byte  101,  96,  91,  86,  82,  77,  73,  69
    .byte  65,  61,  57,  54,  50,  47,  44,  41
    .byte  38,  35,  32,  29,  27,  25,  22,  20
    .byte  18,  16,  15,  13,  12,  10,   9,   8
    .byte  7,   6,   6,   5,   5,   4,   4,   4
endconst

// void ipred_smooth_lsx(pixel *dst, const ptrdiff_t stride,
//                       const pixel *const topleft,
//                       const int width, const int height, const int a,
//                       const int max_width, const int max_height
//                       HIGHBD_DECL_SUFFIX)
function ipred_smooth_8bpc_lsx
    la.local       a5,      dav1d_sm_weights
    add.d          a6,      a5,      a3  //hor
    add.d          a5,      a5,      a4  //ver

    add.d          a7,      a2,      a3
    sub.d          t0,      a2,      a4

    vldrepl.b      vr0,     a7,      0  //right
    vldrepl.b      vr1,     t0,      0  //bottom

    vsllwil.hu.bu  vr0,     vr0,     0
    vsllwil.wu.hu  vr0,     vr0,     0
    vsllwil.hu.bu  vr1,     vr1,     0
    vsllwil.wu.hu  vr1,     vr1,     0

    li.w           t0,      256
    vreplgr2vr.w   vr6,     t0

    addi.d         t0,      a2,      1   //ptr topleft[x]
    addi.d         t3,      a2,      -1  //ptr topleft[y]

.IPRED_SMOOTH_H_LOOP:
    vldrepl.b      vr2,     a5,      0  //ver[y]
    vldrepl.b      vr3,     t3,      0  //topleft[y]

    vsllwil.hu.bu  vr2,     vr2,     0
    vsllwil.wu.hu  vr2,     vr2,     0
    vsllwil.hu.bu  vr3,     vr3,     0
    vsllwil.wu.hu  vr3,     vr3,     0

    vsub.w         vr7,     vr6,     vr2  //256-ver[y]

    or             t1,      zero,    zero  //xx
    srai.d         t2,      a3,      2     //loop max

.IPRED_SMOOTH_W_LOOP:
    fldx.s         f4,      t0,      t1   //topleft[x]
    fldx.s         f5,      a6,      t1   //hor[x]

    vsllwil.hu.bu  vr4,     vr4,     0
    vsllwil.wu.hu  vr4,     vr4,     0
    vsllwil.hu.bu  vr5,     vr5,     0
    vsllwil.wu.hu  vr5,     vr5,     0

    vsub.w         vr8,     vr6,     vr5  //256-hor[x]

    vmul.w         vr9,     vr8,     vr0
    vmadd.w        vr9,     vr5,     vr3
    vmadd.w        vr9,     vr7,     vr1
    vmadd.w        vr9,     vr2,     vr4  //pred

    vadd.w         vr9,     vr9,     vr6
    vsrlni.h.w     vr9,     vr9,     9
    vsrlni.b.h     vr9,     vr9,     0

    fstx.s         f9,      a0,      t1

    addi.d         t1,      t1,      4
    addi.d         t2,      t2,      -1
    bnez           t2,      .IPRED_SMOOTH_W_LOOP

.IPRED_SMOOTH_W_LOOP_END:
    addi.d         t3,      t3,      -1
    addi.d         a5,      a5,      1
    add.d          a0,      a0,      a1
    addi.d         a4,      a4,      -1
    bnez           a4,      .IPRED_SMOOTH_H_LOOP

endfunc

// void ipred_smooth_v_lsx(pixel *dst, const ptrdiff_t stride,
//                         const pixel *const topleft,
//                         const int width, const int height, const int a,
//                         const int max_width, const int max_height
//                         HIGHBD_DECL_SUFFIX)
function ipred_smooth_v_8bpc_lsx
    la.local       a5,      dav1d_sm_weights
    add.d          a5,      a5,      a4  //ver

    sub.d          t0,      a2,      a4
    vldrepl.b      vr0,     t0,      0  //bottom
    vsllwil.hu.bu  vr0,     vr0,     0

    li.w           t0,      256
    vreplgr2vr.h   vr2,     t0
    li.w           t0,      128
    vreplgr2vr.h   vr3,     t0

    addi.d         t0,      a2,      1   //ptr topleft[x]

.IPRED_SMOOTH_V_H_LOOP:
    vldrepl.b      vr1,     a5,      0  //ver[y]
    vsllwil.hu.bu  vr1,     vr1,     0
    vsub.h         vr5,     vr2,     vr1  //256-ver[y]

    or             t1,      zero,    zero  //xx
    srai.d         t2,      a3,      3     //loop max
    beqz           t2,      .IPRED_SMOOTH_V_W_LOOP4

.IPRED_SMOOTH_V_W_LOOP8:
    fldx.d         f4,      t0,      t1   //topleft[x]
    vsllwil.hu.bu  vr4,     vr4,     0

    vmul.h         vr6,     vr5,     vr0
    vmadd.h        vr6,     vr1,     vr4  //pred
    vadd.h         vr6,     vr6,     vr3
    vsrlni.b.h     vr6,     vr6,     8

    fstx.d         f6,      a0,      t1

    addi.d         t1,      t1,      8
    addi.d         t2,      t2,      -1
    bnez           t2,      .IPRED_SMOOTH_V_W_LOOP8
    b              .IPRED_SMOOTH_V_W_LOOP_END

.IPRED_SMOOTH_V_W_LOOP4:
    fldx.s         f4,      t0,      t1   //topleft[x]
    vsllwil.hu.bu  vr4,     vr4,     0

    vmul.h         vr6,     vr5,     vr0
    vmadd.h        vr6,     vr1,     vr4  //pred
    vadd.h         vr6,     vr6,     vr3
    vsrai.h        vr6,     vr6,     8
    vsrlni.b.h     vr6,     vr6,     0

    fstx.s         f6,      a0,      t1

    addi.d         t1,      t1,      4

.IPRED_SMOOTH_V_W_LOOP_END:
    addi.d         a5,      a5,      1
    add.d          a0,      a0,      a1
    addi.d         a4,      a4,      -1
    bnez           a4,      .IPRED_SMOOTH_V_H_LOOP

endfunc

// void ipred_smooth_h_lsx(pixel *dst, const ptrdiff_t stride,
//                         const pixel *const topleft,
//                         const int width, const int height, const int a,
//                         const int max_width, const int max_height
//                         HIGHBD_DECL_SUFFIX)
function ipred_smooth_h_8bpc_lsx
    la.local       a5,      dav1d_sm_weights
    add.d          a6,      a5,      a3  //hor

    add.d          a7,      a2,      a3
    vldrepl.b      vr0,     a7,      0  //right
    vsllwil.hu.bu  vr0,     vr0,     0

    li.w           t0,      256
    vreplgr2vr.h   vr1,     t0
    li.w           t0,      128
    vreplgr2vr.h   vr2,     t0

    addi.d         t3,      a2,      -1  //ptr topleft[y]

.IPRED_SMOOTH_H_H_LOOP:
    vldrepl.b      vr3,     t3,      0  //topleft[y]
    vsllwil.hu.bu  vr3,     vr3,     0

    or             t1,      zero,    zero  //xx
    srai.d         t2,      a3,      3     //loop max
    beqz           t2,      .IPRED_SMOOTH_H_W_LOOP4

.IPRED_SMOOTH_H_W_LOOP8:
    fldx.d         f5,      a6,      t1   //hor[x]
    vsllwil.hu.bu  vr5,     vr5,     0
    vsub.h         vr4,     vr1,     vr5  //256-hor[x]

    vmul.h         vr6,     vr4,     vr0
    vmadd.h        vr6,     vr5,     vr3  //pred
    vadd.h         vr6,     vr6,     vr2
    vsrlni.b.h     vr6,     vr6,     8

    fstx.d         f6,      a0,      t1

    addi.d         t1,      t1,      8
    addi.d         t2,      t2,      -1
    bnez           t2,      .IPRED_SMOOTH_H_W_LOOP8
    b              .IPRED_SMOOTH_W_H_LOOP_END

.IPRED_SMOOTH_H_W_LOOP4:
    fldx.s         f5,      a6,      t1   //hor[x]
    vsllwil.hu.bu  vr5,     vr5,     0
    vsub.h         vr4,     vr1,     vr5  //256-hor[x]

    vmul.h         vr6,     vr4,     vr0
    vmadd.h        vr6,     vr5,     vr3  //pred
    vadd.h         vr6,     vr6,     vr2
    vsrai.h        vr6,     vr6,     8
    vsrlni.b.h     vr6,     vr6,     0

    fstx.s         f6,      a0,      t1

    addi.d         t1,      t1,      4

.IPRED_SMOOTH_W_H_LOOP_END:
    addi.d         t3,      t3,      -1
    add.d          a0,      a0,      a1
    addi.d         a4,      a4,      -1
    bnez           a4,      .IPRED_SMOOTH_H_H_LOOP

endfunc

// void pal_pred_lsx(pixel *dst, const ptrdiff_t stride,
//                   const pixel *const pal, const uint8_t *idx,
//                   const int w, const int h)
function pal_pred_8bpc_lsx
    srai.d         a7,      a5,      2

.PAL_PRED_WLOOP4:
    andi           a6,      a4,      4
    beqz           a6,      .PAL_PRED_WLOOP8
    fld.d          f0,      a3,      0
    vsrli.b        vr1,     vr0,     4
    vandi.b        vr2,     vr0,     7
    vilvl.b        vr0,     vr1,     vr2
    fld.d          f1,      a2,      0
    vshuf.b        vr2,     vr1,     vr1,    vr0

    vstelm.w       vr2,     a0,      0,      0
    add.d          a0,      a0,      a1
    vstelm.w       vr2,     a0,      0,      1
    add.d          a0,      a0,      a1
    vstelm.w       vr2,     a0,      0,      2
    add.d          a0,      a0,      a1
    vstelm.w       vr2,     a0,      0,      3
    add.d          a0,      a0,      a1

    addi.d         a3,      a3,      8
    addi.d         a7,      a7,      -1
    bnez           a7,      .PAL_PRED_WLOOP4
    b              .PAL_PRED_END

.PAL_PRED_WLOOP8:
    andi           a6,      a4,      8
    beqz           a6,      .PAL_PRED_WLOOP16

    vld            vr0,     a3,      0
    vsrli.b        vr1,     vr0,     4
    vandi.b        vr2,     vr0,     7
    vilvl.b        vr0,     vr1,     vr2
    vilvh.b        vr3,     vr1,     vr2
    fld.d          f1,      a2,      0
    vshuf.b        vr0,     vr1,     vr1,    vr0
    vshuf.b        vr3,     vr1,     vr1,    vr3

    vstelm.d       vr0,     a0,      0,      0
    add.d          a0,      a0,      a1
    vstelm.d       vr0,     a0,      0,      1
    add.d          a0,      a0,      a1

    vstelm.d       vr3,     a0,      0,      0
    add.d          a0,      a0,      a1
    vstelm.d       vr3,     a0,      0,      1
    add.d          a0,      a0,      a1

    addi.d         a3,      a3,      16
    addi.d         a7,      a7,      -1
    bnez           a7,      .PAL_PRED_WLOOP8
    b              .PAL_PRED_END

.PAL_PRED_WLOOP16:
    andi           a6,      a4,      16
    beqz           a6,      .PAL_PRED_WLOOP32

    vld            vr0,     a3,      0
    vld            vr1,     a3,      16
    fld.d          f6,      a2,      0
    vsrli.b        vr2,     vr0,     4
    vandi.b        vr3,     vr0,     7
    vsrli.b        vr4,     vr1,     4
    vandi.b        vr5,     vr1,     7
    vilvl.b        vr0,     vr2,     vr3
    vilvh.b        vr1,     vr2,     vr3
    vilvl.b        vr2,     vr4,     vr5
    vilvh.b        vr3,     vr4,     vr5
    vshuf.b        vr0,     vr6,     vr6,    vr0
    vshuf.b        vr1,     vr6,     vr6,    vr1
    vshuf.b        vr2,     vr6,     vr6,    vr2
    vshuf.b        vr3,     vr6,     vr6,    vr3

    vst            vr0,     a0,      0
    add.d          a0,      a0,      a1
    vst            vr1,     a0,      0
    add.d          a0,      a0,      a1
    vst            vr2,     a0,      0
    add.d          a0,      a0,      a1
    vst            vr3,     a0,      0
    add.d          a0,      a0,      a1

    addi.d         a3,      a3,      32
    addi.d         a7,      a7,      -1
    bnez           a7,      .PAL_PRED_WLOOP16
    b              .PAL_PRED_END

.PAL_PRED_WLOOP32:
    andi           a6,      a4,      32
    beqz           a6,      .PAL_PRED_WLOOP64

    vld            vr0,     a3,      0
    vld            vr1,     a3,      16
    vld            vr2,     a3,      32
    vld            vr3,     a3,      48
    fld.d          f4,      a2,      0
    vsrli.b        vr5,     vr0,     4
    vandi.b        vr6,     vr0,     7
    vsrli.b        vr7,     vr1,     4
    vandi.b        vr8,     vr1,     7
    vsrli.b        vr9,     vr2,     4
    vandi.b        vr10,    vr2,     7
    vsrli.b        vr11,    vr3,     4
    vandi.b        vr12,    vr3,     7
    vilvl.b        vr0,     vr5,     vr6
    vilvh.b        vr1,     vr5,     vr6
    vilvl.b        vr2,     vr7,     vr8
    vilvh.b        vr3,     vr7,     vr8
    vilvl.b        vr5,     vr9,     vr10
    vilvh.b        vr6,     vr9,     vr10
    vilvl.b        vr7,     vr11,    vr12
    vilvh.b        vr8,     vr11,    vr12
    vshuf.b        vr0,     vr4,     vr4,    vr0
    vshuf.b        vr1,     vr4,     vr4,    vr1
    vshuf.b        vr2,     vr4,     vr4,    vr2
    vshuf.b        vr3,     vr4,     vr4,    vr3
    vshuf.b        vr5,     vr4,     vr4,    vr5
    vshuf.b        vr6,     vr4,     vr4,    vr6
    vshuf.b        vr7,     vr4,     vr4,    vr7
    vshuf.b        vr8,     vr4,     vr4,    vr8

    vst            vr0,     a0,      0
    vst            vr1,     a0,      16
    add.d          a0,      a0,      a1
    vst            vr2,     a0,      0
    vst            vr3,     a0,      16
    add.d          a0,      a0,      a1
    vst            vr5,     a0,      0
    vst            vr6,     a0,      16
    add.d          a0,      a0,      a1
    vst            vr7,     a0,      0
    vst            vr8,     a0,      16
    add.d          a0,      a0,      a1

    addi.d         a3,      a3,      64
    addi.d         a7,      a7,      -1
    bnez           a7,      .PAL_PRED_WLOOP32
    b              .PAL_PRED_END

.PAL_PRED_WLOOP64:
    vld            vr0,     a3,      0
    vld            vr1,     a3,      16
    fld.d          f2,      a2,      0
    vsrli.b        vr3,     vr0,     4
    vandi.b        vr4,     vr0,     7
    vsrli.b        vr5,     vr1,     4
    vandi.b        vr6,     vr1,     7
    vilvl.b        vr0,     vr3,     vr4
    vilvh.b        vr1,     vr3,     vr4
    vilvl.b        vr3,     vr5,     vr6
    vilvh.b        vr4,     vr5,     vr6
    vshuf.b        vr0,     vr2,     vr2,    vr0
    vshuf.b        vr1,     vr2,     vr2,    vr1
    vshuf.b        vr3,     vr2,     vr2,    vr3
    vshuf.b        vr4,     vr2,     vr2,    vr4

    vst            vr0,     a0,      0
    vst            vr1,     a0,      16
    vst            vr3,     a0,      32
    vst            vr4,     a0,      48

    add.d          a0,      a0,      a1
    addi.d         a3,      a3,      32
    addi.d         a5,      a5,      -1
    bnez           a5,      .PAL_PRED_WLOOP64

.PAL_PRED_END:
endfunc

.macro apply_sign_vrh v, s, vrzero, vrt0 ,out
    vslt.h         \vrt0,   \s,      \vrzero
    vandn.v        \s,      \vrt0,   \v
    vsigncov.h     \v,      \vrt0,   \v
    vor.v          \out,    \s,      \v
.endm

.macro iclip_pixel_vrh in0, in1, in2, tmp0, tmp1, out
    vmin.h         \tmp0,   \in2,    \in0
    vslt.h         \in0,    \in0,    \in1
    vand.v         \tmp1,   \in0,    \in1
    vandn.v        \tmp0,   \in0,    \tmp0
    vor.v          \out,    \tmp1,   \tmp0
.endm

.macro ipred_cfl_pred dst, stride, w, h, dc, ac, alpha
    vreplgr2vr.h   vr2,     \alpha
    vreplgr2vr.h   vr7,     \dc
    li.w           t1,      32
    vreplgr2vr.h   vr3,     t1
    vxor.v         vr4,     vr4,     vr4
    li.w           t1,      255
    vreplgr2vr.h   vr6,     t1
    add.d          t4,      \w,      \w

1:
    or             t1,      zero,    zero
    or             t2,      zero,    zero
    srai.d         t3,      \w,      3
    beqz           t3,      3f

2:
    vldx           vr0,     \ac,     t1
    vmul.h         vr1,     vr2,     vr0
    vadda.h        vr0,     vr1,     vr3
    vsrai.h        vr0,     vr0,     6
    apply_sign_vrh vr0, vr1, vr4, vr5, vr0
    vadd.h         vr1,     vr0,     vr7
    iclip_pixel_vrh vr1, vr4, vr6, vr5, vr8, vr0
    vsrlni.b.h     vr0,     vr0,     0
    fstx.d         f0,      \dst,    t2

    addi.d         t1,      t1,      16
    addi.d         t2,      t2,      8
    addi.d         t3,      t3,      -1
    bnez           t3,      2b
    b              4f

3:
    fld.d          f0,      \ac,     0
    vmul.h         vr1,     vr2,     vr0
    vadda.h        vr0,     vr1,     vr3
    vsrai.h        vr0,     vr0,     6
    apply_sign_vrh vr0, vr1, vr4, vr5, vr0
    vadd.h         vr1,     vr0,     vr7
    iclip_pixel_vrh vr1, vr4, vr6, vr5, vr8, vr0
    vsrlni.b.h     vr0,     vr0,     0
    fst.s          f0,      \dst,    0

4:
    add.d          \ac,     \ac,     t4
    add.d          \dst,    \dst,    \stride
    addi.d         \h,      \h,      -1
    bnez           \h,      1b
.endm

function ipred_cfl_8bpc_lsx
    ipred_dc_gen   a2, a3, a4
    ipred_cfl_pred a0, a1, a3, a4, t0, a5, a6
endfunc

function ipred_cfl_top_8bpc_lsx
    ipred_dc_gen_top   a2, a3
    ipred_cfl_pred a0, a1, a3, a4, t0, a5, a6
endfunc

function ipred_cfl_left_8bpc_lsx
    ipred_dc_gen_left   a2, a4
    ipred_cfl_pred a0, a1, a3, a4, t0, a5, a6
endfunc

function ipred_cfl_128_8bpc_lsx
    li.w           t0,      128
    ipred_cfl_pred a0, a1, a3, a4, t0, a5, a6
endfunc