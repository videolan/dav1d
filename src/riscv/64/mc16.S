/******************************************************************************
 * Copyright © 2018, VideoLAN and dav1d authors
 * Copyright © 2024, Nathan Egge
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

function blend_vl256_16bpc_rvv, export=1, ext=v
  li t0, 4
  beq a3, t0, 4f
  li t0, 8
  beq a3, t0, 8f
  li t0, 16
  beq a3, t0, 16f
32:
  vsetvli zero, a3, e16, m2, ta, ma
  j L(blend_epilog)
16:
  vsetvli zero, a3, e16, m1, ta, ma
  j L(blend_epilog)
8:
  vsetvli zero, a3, e16, mf2, ta, ma
  j L(blend_epilog)
4:
  vsetvli zero, a3, e16, mf4, ta, ma
  j L(blend_epilog)
endfunc

function blend_16bpc_rvv, export=1, ext="v,zbb"
  li t0, 4
  beq a3, t0, 4f
  li t0, 8
  beq a3, t0, 8f
  li t0, 16
  beq a3, t0, 16f
32:
  vsetvli zero, a3, e16, m4, ta, ma
  j L(blend_epilog)
16:
  vsetvli zero, a3, e16, m2, ta, ma
  j L(blend_epilog)
8:
  vsetvli zero, a3, e16, m1, ta, ma
  j L(blend_epilog)
4:
  vsetvli zero, a3, e16, mf2, ta, ma
L(blend_epilog):
  csrw vxrm, zero
  li t1, 64
  add a6, a3, a3
1:
  addi a4, a4, -1
  vle8.v v8, (a5)
  add a5, a5, a3
  vle16.v v4, (a2)
  add a2, a2, a6
  vzext.vf2 v12, v8
  vle16.v v0, (a0)
  vwmulu.vv v16, v4, v12
  vrsub.vx v12, v12, t1
  vwmaccu.vv v16, v0, v12
  vnclipu.wi v0, v16, 6
  vse16.v v0, (a0)
  add a0, a0, a1
  bnez a4, 1b
  ret
endfunc
