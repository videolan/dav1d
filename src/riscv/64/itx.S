/******************************************************************************
 * Copyright © 2018, VideoLAN and dav1d authors
 * Copyright © 2023, Nathan Egge
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
 *****************************************************************************/

#include "src/riscv/asm.S"

function inv_txfm_add_4x4_rvv, export=1, ext=v
  csrw vxrm, zero

  vsetivli zero, 4, e16, mf2, ta, ma
  vle16.v v0, (a2)
  addi t0, a2, 8
  vle16.v v1, (t0)
  addi t0, t0, 8
  vle16.v v2, (t0)
  addi t0, t0, 8
  vle16.v v3, (t0)

  jalr t0, a4

  vmv.v.x v4, zero

  vsseg4e16.v v0, (a2)
  vle16.v v0, (a2)
  vse16.v v4, (a2)
  addi t0, a2, 8
  vle16.v v1, (t0)
  vse16.v v4, (t0)
  addi t0, t0, 8
  vle16.v v2, (t0)
  vse16.v v4, (t0)
  addi t0, t0, 8
  vle16.v v3, (t0)
  vse16.v v4, (t0)

  jalr t0, a5

  vssra.vi v0, v0, 4
  vssra.vi v1, v1, 4
  vssra.vi v2, v2, 4
  vssra.vi v3, v3, 4

itx_4x4_end:
  vsetvli zero, zero, e8, mf4, ta, ma
  vle8.v v4, (a0)
  add t0, a0, a1
  vle8.v v5, (t0)
  add t0, t0, a1
  vle8.v v6, (t0)
  add t0, t0, a1
  vle8.v v7, (t0)

  vwaddu.wv v0, v0, v4
  vwaddu.wv v1, v1, v5
  vwaddu.wv v2, v2, v6
  vwaddu.wv v3, v3, v7

  vsetvli zero, zero, e16, mf2, ta, ma
  vmax.vx v0, v0, zero
  vmax.vx v1, v1, zero
  vmax.vx v2, v2, zero
  vmax.vx v3, v3, zero

  vsetvli zero, zero, e8, mf4, ta, ma

  vnclipu.wi v4, v0, 0
  vnclipu.wi v5, v1, 0
  vnclipu.wi v6, v2, 0
  vnclipu.wi v7, v3, 0

  vse8.v v4, (a0)
  add a0, a0, a1
  vse8.v v5, (a0)
  add a0, a0, a1
  vse8.v v6, (a0)
  add a0, a0, a1
  vse8.v v7, (a0)

  ret
endfunc

function inv_identity_e16_x4_rvv, export=1, ext=v
  li t1, (5793-4096)*8
  vsmul.vx v4, v0, t1
  vsmul.vx v5, v1, t1
  vsmul.vx v6, v2, t1
  vsmul.vx v7, v3, t1

  vsadd.vv v0, v0, v4
  vsadd.vv v1, v1, v5
  vsadd.vv v2, v2, v6
  vsadd.vv v3, v3, v7

  jr t0
endfunc

function inv_dct_e16_x4_rvv, export=1, ext=v
  li t1, 2896
  li t2, 1567
  li t3, 3784

  vwmul.vx v8, v0, t1
  vwmul.vx v10, v0, t1
  vwmacc.vx v8, t1, v2
  neg t1, t1
  vwmacc.vx v10, t1, v2

  vwmul.vx v12, v1, t3
  neg t3, t3
  vwmul.vx v14, v1, t2
  vwmacc.vx v12, t2, v3
  vwmacc.vx v14, t3, v3

  li t1, 2048

  vwadd.wx v8, v8, t1
  vwadd.wx v10, v10, t1
  vwadd.wx v12, v12, t1
  vwadd.wx v14, v14, t1

  vnsra.wi v8, v8, 12
  vnsra.wi v10, v10, 12
  vnsra.wi v12, v12, 12
  vnsra.wi v14, v14, 12

  vsadd.vv v0, v8, v12
  vsadd.vv v1, v10, v14
  vssub.vv v2, v10, v14
  vssub.vv v3, v8, v12

  jr t0
endfunc

.macro def_fn_4x4 txfm1, txfm2
function inv_txfm_add_\txfm1\()_\txfm2\()_4x4_8bpc_rvv, export=1, ext=v
.ifc \txfm1\()_\txfm2, dct_dct
  beqz a3, 1f
.endif
  la a4, inv_\txfm1\()_e16_x4_rvv
  la a5, inv_\txfm2\()_e16_x4_rvv
  j inv_txfm_add_4x4_rvv
.ifc \txfm1\()_\txfm2, dct_dct
1:
  csrw vxrm, zero
  vsetivli zero, 4, e16, mf2, ta, ma
  ld t2, (a2)
  li t1, 2896*8
  vmv.v.x v0, t2
  vsmul.vx v0, v0, t1
  sd x0, (a2)
  vsmul.vx v0, v0, t1
  vssra.vi v0, v0, 4
  vmv.v.v v1, v0
  vmv.v.v v2, v0
  vmv.v.v v3, v0
  j itx_4x4_end
.endif
endfunc
.endm

def_fn_4x4 dct, dct
def_fn_4x4 identity, identity
def_fn_4x4 dct, identity
def_fn_4x4 identity, dct

.macro def_fn_8x8_base variant
function inv_txfm_\variant\()add_8x8_rvv, export=1, ext=v
  csrw vxrm, zero

  vsetivli zero, 8, e16, m1, ta, ma
  vle16.v v0, (a2)
  addi t0, a2, 16
  vle16.v v1, (t0)
  addi t0, t0, 16
  vle16.v v2, (t0)
  addi t0, t0, 16
  vle16.v v3, (t0)
  addi t0, t0, 16
  vle16.v v4, (t0)
  addi t0, t0, 16
  vle16.v v5, (t0)
  addi t0, t0, 16
  vle16.v v6, (t0)
  addi t0, t0, 16
  vle16.v v7, (t0)

.ifc \variant, identity_
  // The identity vsadd.vv and downshift vssra.vi 1 cancel out
.else
  jalr t0, a4

  vssra.vi v0, v0, 1
  vssra.vi v1, v1, 1
  vssra.vi v2, v2, 1
  vssra.vi v3, v3, 1
  vssra.vi v4, v4, 1
  vssra.vi v5, v5, 1
  vssra.vi v6, v6, 1
  vssra.vi v7, v7, 1
.endif

  vsseg8e16.v v0, (a2)
  vle16.v v0, (a2)
  addi t0, a2, 16
  vle16.v v1, (t0)
  addi t0, t0, 16
  vle16.v v2, (t0)
  addi t0, t0, 16
  vle16.v v3, (t0)
  addi t0, t0, 16
  vle16.v v4, (t0)
  addi t0, t0, 16
  vle16.v v5, (t0)
  addi t0, t0, 16
  vle16.v v6, (t0)
  addi t0, t0, 16
  vle16.v v7, (t0)

  jalr t0, a5

  vssra.vi v0, v0, 4
  vssra.vi v1, v1, 4
  vssra.vi v2, v2, 4
  vssra.vi v3, v3, 4
  vssra.vi v4, v4, 4
  vssra.vi v5, v5, 4
  vssra.vi v6, v6, 4
  vssra.vi v7, v7, 4

  vsetvli zero, zero, e8, mf2, ta, ma
  vle8.v v8, (a0)
  add t0, a0, a1
  vle8.v v9, (t0)
  add t0, t0, a1
  vle8.v v10, (t0)
  add t0, t0, a1
  vle8.v v11, (t0)
  add t0, t0, a1
  vle8.v v12, (t0)
  add t0, t0, a1
  vle8.v v13, (t0)
  add t0, t0, a1
  vle8.v v14, (t0)
  add t0, t0, a1
  vle8.v v15, (t0)

  vwaddu.wv v0, v0, v8
  vwaddu.wv v1, v1, v9
  vwaddu.wv v2, v2, v10
  vwaddu.wv v3, v3, v11
  vwaddu.wv v4, v4, v12
  vwaddu.wv v5, v5, v13
  vwaddu.wv v6, v6, v14
  vwaddu.wv v7, v7, v15

  vsetvli zero, zero, e16, m1
  vmax.vx v0, v0, zero
  vmax.vx v1, v1, zero
  vmax.vx v2, v2, zero
  vmax.vx v3, v3, zero
  vmax.vx v4, v4, zero
  vmax.vx v5, v5, zero
  vmax.vx v6, v6, zero
  vmax.vx v7, v7, zero

  vsetvli zero, zero, e8, mf2, ta, ma

  vnclipu.wi v8, v0, 0
  vnclipu.wi v9, v1, 0
  vnclipu.wi v10, v2, 0
  vnclipu.wi v11, v3, 0
  vnclipu.wi v12, v4, 0
  vnclipu.wi v13, v5, 0
  vnclipu.wi v14, v6, 0
  vnclipu.wi v15, v7, 0

  vse8.v v8, (a0)
  add a0, a0, a1
  vse8.v v9, (a0)
  add a0, a0, a1
  vse8.v v10, (a0)
  add a0, a0, a1
  vse8.v v11, (a0)
  add a0, a0, a1
  vse8.v v12, (a0)
  add a0, a0, a1
  vse8.v v13, (a0)
  add a0, a0, a1
  vse8.v v14, (a0)
  add a0, a0, a1
  vse8.v v15, (a0)

  li t1, 64
  vsetvli zero, t1, e16, m8, ta, ma
  vmv.v.x v0, zero
  vse16.v v0, (a2)

  ret
endfunc
.endm

def_fn_8x8_base
def_fn_8x8_base identity_

function inv_identity_e16_x8_rvv, export=1, ext=v
  vsadd.vv v0, v0, v0
  vsadd.vv v1, v1, v1
  vsadd.vv v2, v2, v2
  vsadd.vv v3, v3, v3
  vsadd.vv v4, v4, v4
  vsadd.vv v5, v5, v5
  vsadd.vv v6, v6, v6
  vsadd.vv v7, v7, v7

  jr t0
endfunc

.macro def_fn_8x8 txfm1, txfm2
function inv_txfm_add_\txfm1\()_\txfm2\()_8x8_8bpc_rvv, export=1
  la a5, inv_\txfm2\()_e16_x8_rvv
.ifc \txfm1, identity
  j inv_txfm_identity_add_8x8_rvv
.else
  la a4, inv_\txfm1\()_e16_x8_rvv
  j inv_txfm_add_8x8_rvv
.endif
endfunc
.endm

def_fn_8x8 identity, identity
