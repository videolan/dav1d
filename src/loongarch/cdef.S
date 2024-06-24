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

// static int cdef_find_dir_lsx(const pixel *img, const ptrdiff_t stride,
//                            unsigned *const var HIGHBD_DECL_SUFFIX)
// param: img: a0, stride: a1, var: a2
function cdef_find_dir_8bpc_lsx
    addi.d         sp,    sp,    -64
    fst.d          f24,   sp,    0
    fst.d          f25,   sp,    8
    fst.d          f26,   sp,    16
    fst.d          f27,   sp,    24
    fst.d          f28,   sp,    32
    fst.d          f29,   sp,    40
    fst.d          f30,   sp,    48
    fst.d          f31,   sp,    56

    li.d           a3,    128
    vreplgr2vr.w   vr31,  a3

    // hv: vr0-vr3  diag: vr4-vr11  alt: vr12-vr23
.irp i, vr0, vr1, vr2, vr3, vr4, vr5, vr6, vr7, vr8, vr9, vr10, \
        vr11, vr12, vr13, vr14, vr15, vr16, vr17, vr18, vr19, \
        vr20, vr21, vr22, vr23
    vxor.v      \i,       \i,       \i
.endr

.CFDL01:  // 8
    // 0
    fld.d          f24,   a0,    0  //img
    vpermi.w       vr25,  vr24,  0x01

    vsllwil.hu.bu  vr24,  vr24,  0
    vsllwil.hu.bu  vr24,  vr24,  0
    vsllwil.hu.bu  vr25,  vr25,  0
    vsllwil.hu.bu  vr25,  vr25,  0

    vsub.w         vr24,  vr24,  vr31  //px
    vsub.w         vr25,  vr25,  vr31

    vadd.w         vr4,   vr4,   vr24  //diag[0][y+x]
    vadd.w         vr5,   vr5,   vr25

    vpackev.w      vr26,  vr25,  vr24
    vpackod.w      vr27,  vr25,  vr24
    vpermi.w       vr26,  vr26,  0xd8 //px0246
    vpermi.w       vr27,  vr27,  0xd8 //px1357
    vadd.w         vr12,  vr12,  vr26
    vadd.w         vr12,  vr12,  vr27  //alt[0][y+(x>>1)]

    vhaddw.d.w     vr28,  vr24,  vr24
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a3,    vr28,  0
    vhaddw.d.w     vr28,  vr25,  vr25
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a4,    vr28,  0
    add.d          a3,    a3,    a4
    vinsgr2vr.w    vr0,   a3,    0    //hv[0][y]

    vadd.w         vr15,  vr15,  vr26
    vadd.w         vr15,  vr15,  vr27  //alt[1][3+y-(x>>1)]
    vpermi.w       vr15,  vr15,  0x1b

    vadd.w         vr9,   vr9,   vr24
    vadd.w         vr8,   vr8,   vr25
    vpermi.w       vr8,   vr8,   0x1b
    vpermi.w       vr9,   vr9,   0x1b  //diag[1][7+y-x]

    vxor.v         vr28,  vr28,  vr28
    vxor.v         vr29,  vr29,  vr29
    vadd.w         vr28,  vr28,  vr24
    vadd.w         vr29,  vr29,  vr25
    vextrins.w     vr18,  vr28,  0x30
    vshuf4i.w      vr19,  vr28,  0x39
    vextrins.w     vr19,  vr29,  0x30
    vshuf4i.w      vr20,  vr29,  0x39  //alt[2][3-(y>>1)+7]
    vinsgr2vr.w    vr20,  zero,  3

    vadd.w         vr2,   vr2,   vr24
    vadd.w         vr3,   vr3,   vr25  //hv[1][x]

    vadd.w         vr21,  vr21,  vr24
    vadd.w         vr22,  vr22,  vr25  //alt[3][(y>>1)+x]

    add.d          a0,    a0,    a1

    // 1
    fld.d          f24,   a0,    0  //img
    vpermi.w       vr25,  vr24,  0x01

    vsllwil.hu.bu  vr24,  vr24,  0
    vsllwil.hu.bu  vr24,  vr24,  0
    vsllwil.hu.bu  vr25,  vr25,  0
    vsllwil.hu.bu  vr25,  vr25,  0

    vsub.w         vr24,  vr24,  vr31  //px
    vsub.w         vr25,  vr25,  vr31

    vbsrl.v        vr28,  vr4,   4  //1-4
    vbsrl.v        vr29,  vr5,   4  //5-8
    vextrins.w     vr28,  vr5,   0x30
    vadd.w         vr28,  vr28,  vr24  //diag[0][y+x]
    vadd.w         vr29,  vr29,  vr25
    vbsll.v        vr5,   vr29,  4
    vextrins.w     vr5,   vr28,  0x03
    vextrins.w     vr6,   vr29,  0x03
    vextrins.w     vr28,  vr4,   0x30
    vshuf4i.w      vr4,   vr28,  0x93

    vbsrl.v        vr28,  vr12,  4
    vextrins.w     vr28,  vr13,  0x30
    vpackev.w      vr26,  vr25,  vr24
    vpackod.w      vr27,  vr25,  vr24
    vpermi.w       vr26,  vr26,  0xd8 //px0246
    vpermi.w       vr27,  vr27,  0xd8 //px1357
    vadd.w         vr28,  vr28,  vr26
    vadd.w         vr28,  vr28,  vr27  //alt[0][y+(x>>1)]
    vextrins.w     vr13,  vr28,  0x03
    vextrins.w     vr28,  vr12,  0x30
    vshuf4i.w      vr12,  vr28,  0x93

    vhaddw.d.w     vr28,  vr24,  vr24
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a3,    vr28,  0
    vhaddw.d.w     vr28,  vr25,  vr25
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a4,    vr28,  0
    add.d          a3,    a3,    a4
    vinsgr2vr.w    vr0,   a3,    1    //hv[0][y]

    vbsrl.v        vr28,  vr15,  4
    vextrins.w     vr28,  vr16,  0x30
    vpermi.w       vr28,  vr28,  0x1b
    vadd.w         vr28,  vr28,  vr26
    vadd.w         vr28,  vr28,  vr27  //alt[1][3+y-(x>>1)]
    vextrins.w     vr16,  vr28,  0x00
    vextrins.w     vr28,  vr15,  0x00
    vshuf4i.w      vr15,  vr28,  0x6c

    vbsrl.v        vr28,  vr8,   4     //4321
    vbsrl.v        vr29,  vr9,   4     //8765
    vextrins.w     vr28,  vr9,   0x30
    vpermi.w       vr28,  vr28,  0x1b
    vpermi.w       vr29,  vr29,  0x1b
    vadd.w         vr29,  vr29,  vr24
    vadd.w         vr28,  vr28,  vr25  //diag[1][7+y-x]
    vextrins.w     vr10,  vr29,  0x00
    vextrins.w     vr29,  vr28,  0x00
    vshuf4i.w      vr9,   vr29,  0x6c
    vextrins.w     vr28,  vr8,   0x00
    vshuf4i.w      vr8,   vr28,  0x6c

    vbsll.v        vr28,  vr19,  4
    vextrins.w     vr28,  vr18,  0x03
    vbsll.v        vr29,  vr20,  4
    vextrins.w     vr29,  vr19,  0x03
    vadd.w         vr28,  vr28,  vr24
    vadd.w         vr29,  vr29,  vr25  //alt[2][3-(y>>1)+7]
    vextrins.w     vr18,  vr28,  0x30
    vextrins.w     vr28,  vr29,  0x00
    vshuf4i.w      vr19,  vr28,  0x39
    vbsrl.v        vr20,  vr29,  4

    vadd.w         vr2,   vr2,   vr24
    vadd.w         vr3,   vr3,   vr25  //hv[1][x]

    vadd.w         vr21,  vr21,  vr24
    vadd.w         vr22,  vr22,  vr25  //alt[3][(y>>1)+x]

    add.d          a0,    a0,    a1

    // 2
    fld.d          f24,   a0,    0  //img
    vpermi.w       vr25,  vr24,  0x01

    vsllwil.hu.bu  vr24,  vr24,  0
    vsllwil.hu.bu  vr24,  vr24,  0
    vsllwil.hu.bu  vr25,  vr25,  0
    vsllwil.hu.bu  vr25,  vr25,  0

    vsub.w         vr24,  vr24,  vr31  //px
    vsub.w         vr25,  vr25,  vr31

    vbsrl.v        vr28,  vr4,   8
    vbsrl.v        vr29,  vr5,   8
    vextrins.d     vr28,  vr5,   0x10  //2-5
    vextrins.d     vr29,  vr6,   0x10  //6-9
    vadd.w         vr28,  vr28,  vr24  //diag[0][y+x]
    vadd.w         vr29,  vr29,  vr25
    vextrins.d     vr4,   vr28,  0x10
    vextrins.d     vr5,   vr28,  0x01
    vextrins.d     vr5,   vr29,  0x10
    vextrins.d     vr6,   vr29,  0x01

    vbsrl.v        vr28,  vr12,  8
    vextrins.d     vr28,  vr13,  0x10
    vpackev.w      vr26,  vr25,  vr24
    vpackod.w      vr27,  vr25,  vr24
    vpermi.w       vr26,  vr26,  0xd8 //px0246
    vpermi.w       vr27,  vr27,  0xd8 //px1357
    vadd.w         vr28,  vr28,  vr26
    vadd.w         vr28,  vr28,  vr27  //alt[0][y+(x>>1)]
    vextrins.d     vr12,  vr28,  0x10
    vextrins.d     vr13,  vr28,  0x01

    vhaddw.d.w     vr28,  vr24,  vr24
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a3,    vr28,  0
    vhaddw.d.w     vr28,  vr25,  vr25
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a4,    vr28,  0
    add.d          a3,    a3,    a4
    vinsgr2vr.w    vr0,   a3,    2    //hv[0][y]

    vbsrl.v        vr28,  vr15,  8
    vextrins.d     vr28,  vr16,  0x10
    vpermi.w       vr28,  vr28,  0x1b
    vadd.w         vr28,  vr28,  vr26
    vadd.w         vr28,  vr28,  vr27  //alt[1][3+y-(x>>1)]
    vpermi.w       vr28,  vr28,  0x1b
    vextrins.d     vr15,  vr28,  0x10
    vextrins.d     vr16,  vr28,  0x01

    vbsrl.v        vr28,  vr8,   8
    vextrins.d     vr28,  vr9,   0x10
    vbsrl.v        vr29,  vr9,   8
    vextrins.d     vr29,  vr10,  0x10
    vpermi.w       vr28,  vr28,  0x1b  //5432
    vpermi.w       vr29,  vr29,  0x1b  //9876
    vadd.w         vr29,  vr29,  vr24
    vadd.w         vr28,  vr28,  vr25
    vpermi.w       vr28,  vr28,  0x1b
    vpermi.w       vr29,  vr29,  0x1b
    vextrins.d     vr8,   vr28,  0x10
    vextrins.d     vr9,   vr28,  0x01
    vextrins.d     vr9,   vr29,  0x10
    vextrins.d     vr10,  vr29,  0x01  //diag[1][7+y-x]

    vbsrl.v        vr28,  vr18,  8
    vextrins.d     vr28,  vr19,  0x10 //2345
    vbsrl.v        vr29,  vr19,  8
    vextrins.d     vr29,  vr20,  0x10 //6789
    vadd.w         vr28,  vr28,  vr24
    vadd.w         vr29,  vr29,  vr25
    vextrins.d     vr18,  vr28,  0x10
    vextrins.d     vr19,  vr28,  0x01
    vextrins.d     vr19,  vr29,  0x10
    vextrins.d     vr20,  vr29,  0x01   //alt[2][3-(y>>1)+7]

    vadd.w         vr2,   vr2,   vr24
    vadd.w         vr3,   vr3,   vr25  //hv[1][x]

    vbsrl.v        vr28,  vr21,  4
    vextrins.w     vr28,  vr22,  0x30  //1234
    vbsrl.v        vr29,  vr22,  4     //5678
    vadd.w         vr28,  vr28,  vr24
    vadd.w         vr29,  vr29,  vr25  //alt[3][(y>>1)+x]
    vextrins.w     vr23,  vr29,  0x03
    vextrins.w     vr29,  vr28,  0x33
    vshuf4i.w      vr22,  vr29,  0x93
    vextrins.w     vr28,  vr21,  0x30
    vshuf4i.w      vr21,  vr28,  0x93

    add.d          a0,    a0,    a1

    // 3
    fld.d          f24,   a0,    0  //img
    vpermi.w       vr25,  vr24,  0x01

    vsllwil.hu.bu  vr24,  vr24,  0
    vsllwil.hu.bu  vr24,  vr24,  0
    vsllwil.hu.bu  vr25,  vr25,  0
    vsllwil.hu.bu  vr25,  vr25,  0

    vsub.w         vr24,  vr24,  vr31  //px
    vsub.w         vr25,  vr25,  vr31

    vbsll.v        vr28,  vr5,   4
    vextrins.w     vr28,  vr4,   0x03 //3456
    vbsll.v        vr29,  vr6,   4
    vextrins.w     vr29,  vr5,   0x03 //78910
    vadd.w         vr28,  vr28,  vr24  //diag[0][y+x]
    vadd.w         vr29,  vr29,  vr25
    vextrins.w     vr4,   vr28,  0x30
    vextrins.w     vr28,  vr29,  0x00
    vshuf4i.w      vr5,   vr28,  0x39
    vbsrl.v        vr6,   vr29,  4

    vbsll.v        vr28,  vr13,  4
    vextrins.w     vr28,  vr12,  0x03
    vpackev.w      vr26,  vr25,  vr24
    vpackod.w      vr27,  vr25,  vr24
    vpermi.w       vr26,  vr26,  0xd8 //px0246
    vpermi.w       vr27,  vr27,  0xd8 //px1357
    vadd.w         vr28,  vr28,  vr26
    vadd.w         vr28,  vr28,  vr27  //alt[0][y+(x>>1)]
    vextrins.w     vr12,  vr28,  0x30
    vbsrl.v        vr13,  vr28,  4

    vhaddw.d.w     vr28,  vr24,  vr24
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a3,    vr28,  0
    vhaddw.d.w     vr28,  vr25,  vr25
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a4,    vr28,  0
    add.d          a3,    a3,    a4
    vinsgr2vr.w    vr0,   a3,    3    //hv[0][y]

    vbsll.v        vr28,  vr16,  4
    vextrins.w     vr28,  vr15,  0x03
    vpermi.w       vr28,  vr28,  0x1b  //6543
    vadd.w         vr28,  vr28,  vr26
    vadd.w         vr28,  vr28,  vr27  //alt[1][3+y-(x>>1)]
    vextrins.w     vr15,  vr28,  0x33
    vshuf4i.w      vr16,  vr28,  0xc6
    vinsgr2vr.w    vr16,  zero,  3

    vbsll.v        vr28,  vr9,   4
    vextrins.w     vr28,  vr8,   0x03  //3456
    vbsll.v        vr29,  vr10,  4
    vextrins.w     vr29,  vr9,   0x03  //78910
    vpermi.w       vr28,  vr28,  0x1b  //6543
    vpermi.w       vr29,  vr29,  0x1b  //10987
    vadd.w         vr29,  vr29,  vr24
    vadd.w         vr28,  vr28,  vr25  //diag[1][7+y-x]
    vextrins.w     vr8,   vr28,  0x33
    vextrins.w     vr28,  vr29,  0x33
    vshuf4i.w      vr9,   vr28,  0xc6
    vshuf4i.w      vr10,  vr29,  0xc6
    vinsgr2vr.w    vr10,  zero,  3

    vbsrl.v        vr28,  vr18,  8
    vextrins.d     vr28,  vr19,  0x10 //2345
    vbsrl.v        vr29,  vr19,  8
    vextrins.d     vr29,  vr20,  0x10 //6789
    vadd.w         vr28,  vr28,  vr24
    vadd.w         vr29,  vr29,  vr25
    vextrins.d     vr18,  vr28,  0x10
    vextrins.d     vr19,  vr28,  0x01
    vextrins.d     vr19,  vr29,  0x10
    vextrins.d     vr20,  vr29,  0x01   //alt[2][3-(y>>1)+7]

    vadd.w         vr2,   vr2,   vr24
    vadd.w         vr3,   vr3,   vr25  //hv[1][x]

    vbsrl.v        vr28,  vr21,  4
    vextrins.w     vr28,  vr22,  0x30  //1234
    vbsrl.v        vr29,  vr22,  4     //5678
    vextrins.w     vr29,  vr23,  0x30
    vadd.w         vr28,  vr28,  vr24
    vadd.w         vr29,  vr29,  vr25  //alt[3][(y>>1)+x]
    vextrins.w     vr23,  vr29,  0x03
    vextrins.w     vr29,  vr28,  0x33
    vshuf4i.w      vr22,  vr29,  0x93
    vextrins.w     vr28,  vr21,  0x30
    vshuf4i.w      vr21,  vr28,  0x93

    add.d          a0,    a0,    a1

    // 4
    fld.d          f24,   a0,    0  //img
    vpermi.w       vr25,  vr24,  0x01

    vsllwil.hu.bu  vr24,  vr24,  0
    vsllwil.hu.bu  vr24,  vr24,  0
    vsllwil.hu.bu  vr25,  vr25,  0
    vsllwil.hu.bu  vr25,  vr25,  0

    vsub.w         vr24,  vr24,  vr31  //px
    vsub.w         vr25,  vr25,  vr31

    vadd.w         vr5,   vr5,   vr24  //diag[0][y+x]
    vadd.w         vr6,   vr6,   vr25

    vpackev.w      vr26,  vr25,  vr24
    vpackod.w      vr27,  vr25,  vr24
    vpermi.w       vr26,  vr26,  0xd8 //px0246
    vpermi.w       vr27,  vr27,  0xd8 //px1357
    vadd.w         vr13,  vr13,  vr26
    vadd.w         vr13,  vr13,  vr27  //alt[0][y+(x>>1)]

    vhaddw.d.w     vr28,  vr24,  vr24
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a3,    vr28,  0
    vhaddw.d.w     vr28,  vr25,  vr25
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a4,    vr28,  0
    add.d          a3,    a3,    a4
    vinsgr2vr.w    vr1,   a3,    0    //hv[0][y]

    vpermi.w       vr16,  vr16,  0x1b
    vadd.w         vr16,  vr16,  vr26
    vadd.w         vr16,  vr16,  vr27  //alt[1][3+y-(x>>1)]
    vpermi.w       vr16,  vr16,  0x1b

    vpermi.w       vr9,   vr9,   0x1b
    vpermi.w       vr10,  vr10,  0x1b
    vadd.w         vr10,  vr10,  vr24
    vadd.w         vr9,   vr9,   vr25
    vpermi.w       vr9,   vr9,   0x1b
    vpermi.w       vr10,  vr10,  0x1b  //diag[1][7+y-x]

    vbsrl.v        vr28,  vr18,  4
    vextrins.w     vr28,  vr19,  0x30  //1234
    vbsrl.v        vr29,  vr19,  4
    vextrins.w     vr29,  vr20,  0x30  //5678
    vadd.w         vr28,  vr28,  vr24
    vadd.w         vr29,  vr29,  vr25  //alt[2][3-(y>>1)+7]
    vextrins.w     vr20,  vr29,  0x03
    vextrins.w     vr29,  vr28,  0x33
    vshuf4i.w      vr19,  vr29,  0x93
    vbsll.v        vr18,  vr28,  4

    vadd.w         vr2,   vr2,   vr24
    vadd.w         vr3,   vr3,   vr25  //hv[1][x]

    vbsrl.v        vr28,  vr21,  8
    vextrins.d     vr28,  vr22,  0x10
    vbsrl.v        vr29,  vr22,  8
    vextrins.d     vr29,  vr23,  0x10
    vadd.w         vr28,  vr28,  vr24
    vadd.w         vr29,  vr29,  vr25
    vextrins.d     vr21,  vr28,  0x10
    vextrins.d     vr22,  vr28,  0x01
    vextrins.d     vr22,  vr29,  0x10
    vextrins.d     vr23,  vr29,  0x01  //alt[3][(y>>1)+x]

    add.d          a0,    a0,    a1

    // 5
    fld.d          f24,   a0,    0  //img
    vpermi.w       vr25,  vr24,  0x01

    vsllwil.hu.bu  vr24,  vr24,  0
    vsllwil.hu.bu  vr24,  vr24,  0
    vsllwil.hu.bu  vr25,  vr25,  0
    vsllwil.hu.bu  vr25,  vr25,  0

    vsub.w         vr24,  vr24,  vr31  //px
    vsub.w         vr25,  vr25,  vr31

    vbsrl.v        vr28,  vr5,   4  //5-8
    vbsrl.v        vr29,  vr6,   4  //9-12
    vextrins.w     vr28,  vr6,   0x30
    vadd.w         vr28,  vr28,  vr24  //diag[0][y+x]
    vadd.w         vr29,  vr29,  vr25
    vextrins.w     vr7,   vr29,  0x03
    vextrins.w     vr29,  vr28,  0x33
    vshuf4i.w      vr6,   vr29,  0x93
    vextrins.w     vr28,  vr5,   0x30
    vshuf4i.w      vr5,   vr28,  0x93

    vbsrl.v        vr28,  vr13,  4
    vextrins.w     vr28,  vr14,  0x30
    vpackev.w      vr26,  vr25,  vr24
    vpackod.w      vr27,  vr25,  vr24
    vpermi.w       vr26,  vr26,  0xd8 //px0246
    vpermi.w       vr27,  vr27,  0xd8 //px1357
    vadd.w         vr28,  vr28,  vr26
    vadd.w         vr28,  vr28,  vr27  //alt[0][y+(x>>1)]
    vextrins.w     vr14,  vr28,  0x03
    vextrins.w     vr28,  vr13,  0x30
    vshuf4i.w      vr13,  vr28,  0x93

    vhaddw.d.w     vr28,  vr24,  vr24
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a3,    vr28,  0
    vhaddw.d.w     vr28,  vr25,  vr25
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a4,    vr28,  0
    add.d          a3,    a3,    a4
    vinsgr2vr.w    vr1,   a3,    1    //hv[0][y]

    vbsrl.v        vr28,  vr16,  4
    vextrins.w     vr28,  vr17,  0x30
    vpermi.w       vr28,  vr28,  0x1b
    vadd.w         vr28,  vr28,  vr26
    vadd.w         vr28,  vr28,  vr27  //alt[1][3+y-(x>>1)]
    vextrins.w     vr17,  vr28,  0x00
    vextrins.w     vr28,  vr16,  0x00
    vshuf4i.w      vr16,  vr28,  0x6c

    vbsrl.v        vr28,  vr9,   4
    vbsrl.v        vr29,  vr10,  4
    vextrins.w     vr28,  vr10,  0x30
    vpermi.w       vr28,  vr28,  0x1b  //8-5
    vpermi.w       vr29,  vr29,  0x1b  //12-9
    vadd.w         vr29,  vr29,  vr24
    vadd.w         vr28,  vr28,  vr25  //diag[1][7+y-x]
    vextrins.w     vr11,  vr29,  0x00
    vextrins.w     vr29,  vr28,  0x00
    vshuf4i.w      vr10,  vr29,  0x6c
    vextrins.w     vr28,  vr9,   0x00
    vshuf4i.w      vr9,   vr28,  0x6c

    vbsrl.v        vr28,  vr18,  4
    vextrins.w     vr28,  vr19,  0x30  //1234
    vbsrl.v        vr29,  vr19,  4
    vextrins.w     vr29,  vr20,  0x30  //5678
    vadd.w         vr28,  vr28,  vr24
    vadd.w         vr29,  vr29,  vr25  //alt[2][3-(y>>1)+7]
    vextrins.w     vr20,  vr29,  0x03
    vextrins.w     vr29,  vr28,  0x33
    vshuf4i.w      vr19,  vr29,  0x93
    vbsll.v        vr18,  vr28,  4

    vadd.w         vr2,   vr2,   vr24
    vadd.w         vr3,   vr3,   vr25  //hv[1][x]

    vbsrl.v        vr28,  vr21,  8
    vextrins.d     vr28,  vr22,  0x10
    vbsrl.v        vr29,  vr22,  8
    vextrins.d     vr29,  vr23,  0x10
    vadd.w         vr28,  vr28,  vr24
    vadd.w         vr29,  vr29,  vr25
    vextrins.d     vr21,  vr28,  0x10
    vextrins.d     vr22,  vr28,  0x01
    vextrins.d     vr22,  vr29,  0x10
    vextrins.d     vr23,  vr29,  0x01  //alt[3][(y>>1)+x]

    add.d          a0,    a0,    a1

    // 6
    fld.d          f24,   a0,    0  //img
    vpermi.w       vr25,  vr24,  0x01

    vsllwil.hu.bu  vr24,  vr24,  0
    vsllwil.hu.bu  vr24,  vr24,  0
    vsllwil.hu.bu  vr25,  vr25,  0
    vsllwil.hu.bu  vr25,  vr25,  0

    vsub.w         vr24,  vr24,  vr31  //px
    vsub.w         vr25,  vr25,  vr31

    vbsrl.v        vr28,  vr5,   8
    vbsrl.v        vr29,  vr6,   8
    vextrins.d     vr28,  vr6,   0x10  //6-9
    vextrins.d     vr29,  vr7,   0x10  //10-13
    vadd.w         vr28,  vr28,  vr24  //diag[0][y+x]
    vadd.w         vr29,  vr29,  vr25
    vextrins.d     vr5,   vr28,  0x10
    vextrins.d     vr6,   vr28,  0x01
    vextrins.d     vr6,   vr29,  0x10
    vextrins.d     vr7,   vr29,  0x01

    vbsrl.v        vr28,  vr13,  8
    vextrins.d     vr28,  vr14,  0x10
    vpackev.w      vr26,  vr25,  vr24
    vpackod.w      vr27,  vr25,  vr24
    vpermi.w       vr26,  vr26,  0xd8 //px0246
    vpermi.w       vr27,  vr27,  0xd8 //px1357
    vadd.w         vr28,  vr28,  vr26
    vadd.w         vr28,  vr28,  vr27  //alt[0][y+(x>>1)]
    vextrins.d     vr13,  vr28,  0x10
    vextrins.d     vr14,  vr28,  0x01

    vhaddw.d.w     vr28,  vr24,  vr24
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a3,    vr28,  0
    vhaddw.d.w     vr28,  vr25,  vr25
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a4,    vr28,  0
    add.d          a3,    a3,    a4
    vinsgr2vr.w    vr1,   a3,    2    //hv[0][y]

    vbsrl.v        vr28,  vr16,  8
    vextrins.d     vr28,  vr17,  0x10
    vpermi.w       vr28,  vr28,  0x1b
    vadd.w         vr28,  vr28,  vr26
    vadd.w         vr28,  vr28,  vr27  //alt[1][3+y-(x>>1)]
    vpermi.w       vr28,  vr28,  0x1b
    vextrins.d     vr16,  vr28,  0x10
    vextrins.d     vr17,  vr28,  0x01

    vbsrl.v        vr28,  vr9,   8
    vextrins.d     vr28,  vr10,  0x10
    vbsrl.v        vr29,  vr10,  8
    vextrins.d     vr29,  vr11,  0x10
    vpermi.w       vr28,  vr28,  0x1b  //9876
    vpermi.w       vr29,  vr29,  0x1b  //13-10
    vadd.w         vr29,  vr29,  vr24
    vadd.w         vr28,  vr28,  vr25
    vpermi.w       vr28,  vr28,  0x1b
    vpermi.w       vr29,  vr29,  0x1b
    vextrins.d     vr9,   vr28,  0x10
    vextrins.d     vr10,  vr28,  0x01
    vextrins.d     vr10,  vr29,  0x10
    vextrins.d     vr11,  vr29,  0x01  //diag[1][7+y-x]

    vadd.w         vr18,  vr18,  vr24 //0123
    vadd.w         vr19,  vr19,  vr25 //4567 alt[2][3-(y>>1)+7]

    vadd.w         vr2,   vr2,   vr24
    vadd.w         vr3,   vr3,   vr25  //hv[1][x]

    vbsll.v        vr28,  vr22,  4
    vextrins.w     vr28,  vr21,  0x03  //3456
    vbsll.v        vr29,  vr23,  4
    vextrins.w     vr29,  vr22,  0x03  //78910
    vadd.w         vr28,  vr28,  vr24
    vadd.w         vr29,  vr29,  vr25  //alt[3][(y>>1)+x]
    vextrins.w     vr21,  vr28,  0x30
    vextrins.w     vr28,  vr29,  0x00
    vshuf4i.w      vr22,  vr28,  0x39
    vbsrl.v        vr23,  vr29,  4

    add.d          a0,    a0,    a1

    // 7
    fld.d          f24,   a0,    0  //img
    vpermi.w       vr25,  vr24,  0x01

    vsllwil.hu.bu  vr24,  vr24,  0
    vsllwil.hu.bu  vr24,  vr24,  0
    vsllwil.hu.bu  vr25,  vr25,  0
    vsllwil.hu.bu  vr25,  vr25,  0

    vsub.w         vr24,  vr24,  vr31  //px
    vsub.w         vr25,  vr25,  vr31

    vbsll.v        vr28,  vr6,   4
    vextrins.w     vr28,  vr5,   0x03 //78910
    vbsll.v        vr29,  vr7,   4
    vextrins.w     vr29,  vr6,   0x03 //11-14
    vadd.w         vr28,  vr28,  vr24  //diag[0][y+x]
    vadd.w         vr29,  vr29,  vr25
    vextrins.w     vr5,   vr28,  0x30
    vextrins.w     vr28,  vr29,  0x00
    vshuf4i.w      vr6,   vr28,  0x39
    vbsrl.v        vr7,   vr29,  4

    vbsll.v        vr28,  vr14,  4
    vextrins.w     vr28,  vr13,  0x03
    vpackev.w      vr26,  vr25,  vr24
    vpackod.w      vr27,  vr25,  vr24
    vpermi.w       vr26,  vr26,  0xd8 //px0246
    vpermi.w       vr27,  vr27,  0xd8 //px1357
    vadd.w         vr28,  vr28,  vr26
    vadd.w         vr28,  vr28,  vr27  //alt[0][y+(x>>1)]
    vextrins.w     vr13,  vr28,  0x30
    vbsrl.v        vr14,  vr28,  4

    vhaddw.d.w     vr28,  vr24,  vr24
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a3,    vr28,  0
    vhaddw.d.w     vr28,  vr25,  vr25
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a4,    vr28,  0
    add.d          a3,    a3,    a4
    vinsgr2vr.w    vr1,   a3,    3    //hv[0][y]

    vbsll.v        vr28,  vr17,  4
    vextrins.w     vr28,  vr16,  0x03
    vpermi.w       vr28,  vr28,  0x1b  //10987
    vadd.w         vr28,  vr28,  vr26
    vadd.w         vr28,  vr28,  vr27  //alt[1][3+y-(x>>1)]
    vextrins.w     vr16,  vr28,  0x33
    vshuf4i.w      vr17,  vr28,  0xc6
    vinsgr2vr.w    vr17,  zero,  3

    vbsll.v        vr28,  vr10,  4
    vextrins.w     vr28,  vr9,   0x03  //7-10
    vbsll.v        vr29,  vr11,  4
    vextrins.w     vr29,  vr10,  0x03  //11-14
    vpermi.w       vr28,  vr28,  0x1b  //10-7
    vpermi.w       vr29,  vr29,  0x1b  //14-11
    vadd.w         vr29,  vr29,  vr24
    vadd.w         vr28,  vr28,  vr25  //diag[1][7+y-x]
    vextrins.w     vr9,   vr28,  0x33
    vextrins.w     vr28,  vr29,  0x33
    vshuf4i.w      vr10,  vr28,  0xc6
    vshuf4i.w      vr11,  vr29,  0xc6
    vinsgr2vr.w    vr11,  zero,  3

    vadd.w         vr18,  vr18,  vr24 //0123
    vadd.w         vr19,  vr19,  vr25 //4567 alt[2][3-(y>>1)+7]

    vadd.w         vr2,   vr2,   vr24
    vadd.w         vr3,   vr3,   vr25  //hv[1][x]

    vbsll.v        vr28,  vr22,  4
    vextrins.w     vr28,  vr21,  0x03  //3456
    vbsll.v        vr29,  vr23,  4
    vextrins.w     vr29,  vr22,  0x03  //78910
    vadd.w         vr28,  vr28,  vr24
    vadd.w         vr29,  vr29,  vr25  //alt[3][(y>>1)+x]
    vextrins.w     vr21,  vr28,  0x30
    vextrins.w     vr28,  vr29,  0x00
    vshuf4i.w      vr22,  vr28,  0x39
    vbsrl.v        vr23,  vr29,  4

    add.d          a0,    a0,    a1

    vxor.v         vr24,  vr24,  vr24  //unsigned cost[8]
    vxor.v         vr25,  vr25,  vr25

    vmul.w         vr26,  vr0,   vr0
    vmul.w         vr27,  vr1,   vr1
    vhaddw.d.w     vr28,  vr26,  vr26
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a3,    vr28,  0
    vhaddw.d.w     vr28,  vr27,  vr27
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a4,    vr28,  0
    add.d          a3,    a3,    a4

    vmul.w         vr26,  vr2,   vr2
    vmul.w         vr27,  vr3,   vr3
    vhaddw.d.w     vr28,  vr26,  vr26
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a4,    vr28,  0
    vhaddw.d.w     vr28,  vr27,  vr27
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a5,    vr28,  0
    add.d          a4,    a4,    a5

    li.d           a6,    105
    mul.w          a3,    a3,    a6
    mul.w          a4,    a4,    a6
    vinsgr2vr.w    vr24,  a3,    2
    vinsgr2vr.w    vr25,  a4,    2

    vxor.v         vr30,  vr30,  vr30  //div_table
    vxor.v         vr31,  vr31,  vr31
    li.d           t0,    840
    vinsgr2vr.w    vr30,  t0,    0
    li.d           t0,    420
    vinsgr2vr.w    vr30,  t0,    1
    li.d           t0,    280
    vinsgr2vr.w    vr30,  t0,    2
    li.d           t0,    210
    vinsgr2vr.w    vr30,  t0,    3
    li.d           t0,    168
    vinsgr2vr.w    vr31,  t0,    0
    li.d           t0,    140
    vinsgr2vr.w    vr31,  t0,    1
    li.d           t0,    120
    vinsgr2vr.w    vr31,  t0,    2

    vbsll.v        vr27,  vr7,   4
    vextrins.w     vr27,  vr6,   0x03
    vpermi.w       vr27,  vr27,  0x1b
    vmul.w         vr26,  vr4,   vr4
    vmadd.w        vr26,  vr27,  vr27
    vmul.w         vr26,  vr26,  vr30
    vhaddw.d.w     vr28,  vr26,  vr26
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a3,    vr28,  0
    vbsll.v        vr27,  vr6,   4
    vpermi.w       vr27,  vr27,  0x1b
    vmul.w         vr26,  vr5,   vr5
    vmadd.w        vr26,  vr27,  vr27
    vmul.w         vr26,  vr26,  vr31
    vextrins.w     vr26,  vr31,  0x33
    vhaddw.d.w     vr28,  vr26,  vr26
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a4,    vr28,  0
    add.d          a3,    a3,    a4   //cost[0]

    vbsll.v        vr27,  vr11,  4
    vextrins.w     vr27,  vr10,  0x03
    vpermi.w       vr27,  vr27,  0x1b
    vmul.w         vr26,  vr8,   vr8
    vmadd.w        vr26,  vr27,  vr27
    vmul.w         vr26,  vr26,  vr30
    vhaddw.d.w     vr28,  vr26,  vr26
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a4,    vr28,  0
    vbsll.v        vr27,  vr10,  4
    vpermi.w       vr27,  vr27,  0x1b
    vmul.w         vr26,  vr9,   vr9
    vmadd.w        vr26,  vr27,  vr27
    vmul.w         vr26,  vr26,  vr31
    vextrins.w     vr26,  vr31,  0x33
    vhaddw.d.w     vr28,  vr26,  vr26
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a5,    vr28,  0
    add.d          a4,    a4,    a5   //cost[4]

    vpickve2gr.w   a5,    vr5,   3
    mul.w          a5,    a5,    a5
    mul.w          a5,    a5,    a6
    add.w          a3,    a3,    a5
    vinsgr2vr.w    vr24,  a3,    0
    vpickve2gr.w   a5,    vr9,   3
    mul.w          a5,    a5,    a5
    mul.w          a5,    a5,    a6
    add.w          a4,    a4,    a5
    vinsgr2vr.w    vr25,  a4,    0

    //n=0
    vpickve2gr.w   a3,    vr24,  1
    vmul.w         vr26,  vr13,  vr13
    vhaddw.d.w     vr28,  vr26,  vr26
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a4,    vr28,  0
    vpickve2gr.w   a5,    vr12,  3
    mul.w          a5,    a5,    a5
    add.d          a3,    a3,    a4
    add.d          a3,    a3,    a5
    mul.w          a3,    a3,    a6  //*cost_ptr

    vextrins.w     vr29,  vr30,  0x01
    vextrins.w     vr29,  vr30,  0x13
    vextrins.w     vr29,  vr31,  0x21
    vextrins.w     vr29,  vr31,  0x33
    vbsll.v        vr27,  vr14,  4
    vpermi.w       vr27,  vr27,  0x1b
    vmul.w         vr28,  vr12,  vr12
    vextrins.w     vr28,  vr31,  0x33
    vmadd.w        vr28,  vr27,  vr27
    vmul.w         vr26,  vr28,  vr29
    vhaddw.d.w     vr28,  vr26,  vr26
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a4,    vr28,  0
    add.d          a3,    a3,    a4
    vinsgr2vr.w    vr24,  a3,    1

    //n=1
    vpickve2gr.w   a3,    vr24,  3
    vmul.w         vr26,  vr16,  vr16
    vhaddw.d.w     vr28,  vr26,  vr26
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a4,    vr28,  0
    vpickve2gr.w   a5,    vr15,  3
    mul.w          a5,    a5,    a5
    add.d          a3,    a3,    a4
    add.d          a3,    a3,    a5
    mul.w          a3,    a3,    a6  //*cost_ptr

    vbsll.v        vr27,  vr17,  4
    vpermi.w       vr27,  vr27,  0x1b
    vmul.w         vr28,  vr15,  vr15
    vextrins.w     vr28,  vr31,  0x33
    vmadd.w        vr28,  vr27,  vr27
    vmul.w         vr26,  vr28,  vr29
    vhaddw.d.w     vr28,  vr26,  vr26
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a4,    vr28,  0
    add.d          a3,    a3,    a4
    vinsgr2vr.w    vr24,  a3,    3

    //n=2
    vpickve2gr.w   a3,    vr25,  1
    vmul.w         vr26,  vr19,  vr19
    vhaddw.d.w     vr28,  vr26,  vr26
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a4,    vr28,  0
    vpickve2gr.w   a5,    vr18,  3
    mul.w          a5,    a5,    a5
    add.d          a3,    a3,    a4
    add.d          a3,    a3,    a5
    mul.w          a3,    a3,    a6  //*cost_ptr

    vbsll.v        vr27,  vr20,  4
    vpermi.w       vr27,  vr27,  0x1b
    vmul.w         vr28,  vr18,  vr18
    vextrins.w     vr28,  vr31,  0x33
    vmadd.w        vr28,  vr27,  vr27
    vmul.w         vr26,  vr28,  vr29
    vhaddw.d.w     vr28,  vr26,  vr26
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a4,    vr28,  0
    add.d          a3,    a3,    a4
    vinsgr2vr.w    vr25,  a3,    1

    //n=3
    vpickve2gr.w   a3,    vr25,  3
    vmul.w         vr26,  vr22,  vr22
    vhaddw.d.w     vr28,  vr26,  vr26
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a4,    vr28,  0
    vpickve2gr.w   a5,    vr21,  3
    mul.w          a5,    a5,    a5
    add.d          a3,    a3,    a4
    add.d          a3,    a3,    a5
    mul.w          a3,    a3,    a6  //*cost_ptr

    vbsll.v        vr27,  vr23,  4
    vpermi.w       vr27,  vr27,  0x1b
    vmul.w         vr28,  vr21,  vr21
    vextrins.w     vr28,  vr31,  0x33
    vmadd.w        vr28,  vr27,  vr27
    vmul.w         vr26,  vr28,  vr29
    vhaddw.d.w     vr28,  vr26,  vr26
    vhaddw.q.d     vr28,  vr28,  vr28
    vpickve2gr.d   a4,    vr28,  0
    add.d          a3,    a3,    a4
    vinsgr2vr.w    vr25,  a3,    3

    xor            a3,    a3,    a3  //best_dir
    vpickve2gr.w   a4,    vr24,  0   //best_cost
.BSETDIR01:
    vpickve2gr.w   a5,    vr24,  1
    bge            a4,    a5,    .BSETDIR02
    or             a4,    a5,    a5
    ori            a3,    zero,  1
.BSETDIR02:
    vpickve2gr.w   a5,    vr24,  2
    bge            a4,    a5,    .BSETDIR03
    or             a4,    a5,    a5
    ori            a3,    zero,  2
.BSETDIR03:
    vpickve2gr.w   a5,    vr24,  3
    bge            a4,    a5,    .BSETDIR04
    or             a4,    a5,    a5
    ori            a3,    zero,  3
.BSETDIR04:
    vpickve2gr.w   a5,    vr25,  0
    bge            a4,    a5,    .BSETDIR05
    or             a4,    a5,    a5
    ori            a3,    zero,  4
.BSETDIR05:
    vpickve2gr.w   a5,    vr25,  1
    bge            a4,    a5,    .BSETDIR06
    or             a4,    a5,    a5
    ori            a3,    zero,  5
.BSETDIR06:
    vpickve2gr.w   a5,    vr25,  2
    bge            a4,    a5,    .BSETDIR07
    or             a4,    a5,    a5
    ori            a3,    zero,  6
.BSETDIR07:
    vpickve2gr.w   a5,    vr25,  3
    bge            a4,    a5,    .BSETDIREND
    or             a4,    a5,    a5
    ori            a3,    zero,  7
.BSETDIREND:
    xori           a5,    a3,    4
    li.d           a1,    4
    bge            a5,    a1,    .GETCOST01
    vreplve.w      vr26,  vr24,  a5
    b              .GETCOST02
.GETCOST01:
    vreplve.w      vr26,  vr25,  a5
.GETCOST02:
    vpickve2gr.w   a5,    vr26,  0
    sub.w          a5,    a4,    a5
    srai.d         a5,    a5,    10
    st.w           a5,    a2,    0
    or             a0,    a3,    a3

    fld.d          f24,   sp,    0
    fld.d          f25,   sp,    8
    fld.d          f26,   sp,    16
    fld.d          f27,   sp,    24
    fld.d          f28,   sp,    32
    fld.d          f29,   sp,    40
    fld.d          f30,   sp,    48
    fld.d          f31,   sp,    56
    addi.d         sp,    sp,    64

endfunc