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

function blend_8bpc_rvv, export=1, ext=v
  li t0, 4
  beq a3, t0, 4f
  li t0, 8
  beq a3, t0, 8f
  li t0, 16
  beq a3, t0, 16f
32:
  vsetvli zero, a3, e8, m2, ta, ma
  j 0f
16:
  vsetvli zero, a3, e8, m1, ta, ma
  j 0f
8:
  vsetvli zero, a3, e8, mf2, ta, ma
  j 0f
4:
  vsetvli zero, a3, e8, mf4, ta, ma
0:
  csrw vxrm, zero
  li t1, 64
1:
  addi a4, a4, -2
  vle8.v v4, (a2)
  add a2, a2, a3
  vle8.v v6, (a2)
  add a2, a2, a3
  vle8.v v8, (a5)
  add a5, a5, a3
  vle8.v v10, (a5)
  add a5, a5, a3
  vle8.v v0, (a0)
  add t0, a0, a1
  vle8.v v2, (t0)
  vwmulu.vv v16, v4, v8
  vwmulu.vv v20, v6, v10
  vrsub.vx v8, v8, t1
  vrsub.vx v10, v10, t1
  vwmaccu.vv v16, v0, v8
  vwmaccu.vv v20, v2, v10
  vnclipu.wi v0, v16, 6
  vnclipu.wi v2, v20, 6
  vse8.v v0, (a0)
  vse8.v v2, (t0)
  add a0, t0, a1
  bnez a4, 1b
  ret
endfunc

function blend_h_8bpc_rvv, export=1, ext=v
  csrw vxrm, zero
  la t1, dav1d_obmc_masks
  srai t0, a4, 2
  add t1, t1, a4
  sub a4, a4, t0
  li t0, 2
  li t2, 64
  beq a3, t0, 2f
  li t0, 4
  beq a3, t0, 4f
  li t0, 8
  beq a3, t0, 8f
  li t0, 16
  beq a3, t0, 16f
  li t0, 32
  beq a3, t0, 32f
64:
  vsetvli zero, t2, e8, m4, ta, ma
  beq a3, t2, 0f
128:
  mv a5, a0
  mv a6, a2
  mv a7, a4
  jal t5, 1f
  add t1, t1, a4
  add a0, a5, t2
  add a2, a6, t2
  mv a4, a7
  sub t1, t1, a4
  j 0f
32:
  vsetvli zero, a3, e8, m2, ta, ma
  j 0f
16:
  vsetivli zero, 16, e8, m1, ta, ma
  j 0f
8:
  vsetivli zero, 8, e8, mf2, ta, ma
  j 0f
4:
  vsetivli zero, 4, e8, mf4, ta, ma
  j 0f
2:
  vsetivli zero, 2, e8, mf8, ta, ma
0:
  mv t5, ra
1:
  addi a4, a4, -2
  lbu t3, (t1)
  addi t1, t1, 1
  lbu t4, (t1)
  addi t1, t1, 1
  vle8.v v8, (a2)
  add a2, a2, a3
  vle8.v v12, (a2)
  add a2, a2, a3
  vle8.v v0, (a0)
  add t0, a0, a1
  vle8.v v4, (t0)
  vwmulu.vx v16, v8, t3
  vwmulu.vx v24, v12, t4
  sub t3, t2, t3
  sub t4, t2, t4
  vwmaccu.vx v16, t3, v0
  vwmaccu.vx v24, t4, v4
  vnclipu.wi v0, v16, 6
  vnclipu.wi v4, v24, 6
  vse8.v v0, (a0)
  vse8.v v4, (t0)
  add a0, t0, a1
  bgtz a4, 1b
  jr t5
endfunc
