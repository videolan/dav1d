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

function inv_txfm_add_4x4_rvv, export=1
  csrw vxrm, zero

  vsetivli zero, 4, e16, m1, ta, ma
  vle16.v v0, (a2)
  addi t0, a2, 8
  vle16.v v1, (t0)
  addi t0, t0, 8
  vle16.v v2, (t0)
  addi t0, t0, 8
  vle16.v v3, (t0)

  jalr t0, a4

  vsseg4e16.v v0, (a2)
  vle16.v v0, (a2)
  addi t0, a2, 8
  vle16.v v1, (t0)
  addi t0, t0, 8
  vle16.v v2, (t0)
  addi t0, t0, 8
  vle16.v v3, (t0)

  jalr t0, a5

  vssra.vi v0, v0, 4
  vssra.vi v1, v1, 4
  vssra.vi v2, v2, 4
  vssra.vi v3, v3, 4

  vsetvli zero, zero, e8, mf2, ta, ma
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

  vsetvli t0, zero, e16, m4
  vmax.vx v0, v0, x0

  vsetivli zero, 4, e8, mf2, ta, ma

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

  vsetivli zero, 16, e16, m2, ta, ma
  vmv.v.x v0, zero
  vse16.v v0, (a2)

  ret
endfunc

function inv_identity_e16_x4_rvv, export=1
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

.macro def_fn_4x4 txfm1, txfm2
function inv_txfm_add_\txfm1\()_\txfm2\()_4x4_8bpc_rvv, export=1
  la a4, inv_\txfm1\()_e16_x4_rvv
  la a5, inv_\txfm2\()_e16_x4_rvv
  j inv_txfm_add_4x4_rvv
endfunc
.endm

def_fn_4x4 identity, identity
