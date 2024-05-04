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
