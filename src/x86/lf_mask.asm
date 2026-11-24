; Copyright © 2026, VideoLAN and dav1d authors
; All rights reserved.
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions are met:
;
; 1. Redistributions of source code must retain the above copyright notice, this
;    list of conditions and the following disclaimer.
;
; 2. Redistributions in binary form must reproduce the above copyright notice,
;    this list of conditions and the following disclaimer in the documentation
;    and/or other materials provided with the distribution.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
; ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
; WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
; DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
; ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
; (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
; ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
; (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
; SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

%include "config.asm"
%include "ext/x86/x86inc.asm"

%if ARCH_X86_64

SECTION_RODATA

%macro JMP_TABLE 2-*
    %1_table:
    %xdefine %%base %1_table
    %xdefine %%current_base %1
    %xdefine %%prefix mangle(private_prefix %+ _ %+ %%current_base)
    %rotate 1
    %rep %0 - 1
        %ifnum %1
            dd %%prefix %+ .w%1 - %%base
        %else
            %xdefine %%current_base %1
            %xdefine %%prefix mangle(private_prefix %+ _ %+ %%current_base)
        %endif
        %rotate 1
    %endrep
%endmacro

JMP_TABLE fill_levelcache_block_sse2, 1, 2, 4, 8, 16, 32
JMP_TABLE fill_levelcache_block_avx2, \
          fill_levelcache_block_sse2, 1, 2, 4, \
          fill_levelcache_block_avx2, 8, 16, 32

SECTION .text

%macro LC_PTR_LOOP 1
    add  lc_ptrq, strideq
    dec      bhd
    jnz %1
    RET
%endmacro

INIT_XMM sse2
; uint8_t (*level_cache_ptr)[4], ptrdiff_t b_stride, int chroma, int bw, int bh,
; unsigned v0, unsigned v1
cglobal fill_levelcache_block, 4, 7, 5, lc_ptr, stride, bw, bh, v0, v1, tmp
    movifnidn  v1d, v1m
    shl        v1d, 8
    or         v1d, v0m        ; v0 | v1
    add    strideq, strideq

    lea       tmpd, [bwq-1]
    test      tmpd, bwd
    jnz .scalar                ; non-powers of two

    bsr       tmpd, bwd
    lea        v0q, [fill_levelcache_block_sse2_table]
    movsx     tmpq, dword [v0q+4*tmpq]
    add        v0q, tmpq
    jmp        v0q

.scalar:
    xor              tmpd, tmpd
.scalar_row:
    mov  [lc_ptrq+2*tmpq], v1w
    inc              tmpd
    cmp              tmpd, bwd
    jne .scalar_row
    LC_PTR_LOOP .scalar

.w1:
    mov [lc_ptrq], v1w
    LC_PTR_LOOP .w1

.w2:
    imul              v1d, v1d, 0x10001
.w2_loop:
    mov         [lc_ptrq], v1d
    LC_PTR_LOOP .w2_loop

.w4:
    mov              tmpq, 0x1000100010001
    imul              v1q, tmpq
.w4_loop:
    mov         [lc_ptrq], v1q
    LC_PTR_LOOP .w4_loop

.w8:
    movd               m0, v1d
    pshuflw            m0, m0, q0000
    punpcklqdq         m0, m0
.w8_loop:
    movu        [lc_ptrq], m0
    LC_PTR_LOOP .w8_loop

.w16:
    movd               m0, v1d
    pshuflw            m0, m0, q0000
    punpcklqdq         m0, m0
.w16_loop:
    movu        [lc_ptrq], m0
    movu     [lc_ptrq+16], m0
    LC_PTR_LOOP .w16_loop

.w32:
    movd               m0, v1d
    pshuflw            m0, m0, q0000
    punpcklqdq         m0, m0
.w32_loop:
    movu        [lc_ptrq], m0
    movu     [lc_ptrq+16], m0
    movu     [lc_ptrq+32], m0
    movu     [lc_ptrq+48], m0
    LC_PTR_LOOP .w32_loop


INIT_YMM avx2
; uint8_t (*level_cache_ptr)[4], ptrdiff_t b_stride, int chroma, int bw, int bh,
; unsigned v0, unsigned v1
cglobal fill_levelcache_block, 4, 7, 5, lc_ptr, stride, bw, bh, v0, v1, tmp
    rorx       v1d, v1m, 24
    or         v1d, v0m        ; v0 | v1
    add    strideq, strideq

    blsr      tmpd, bwd
    jnz mangle(private_prefix %+ _fill_levelcache_block_sse2).scalar
    tzcnt     tmpd, bwd

    lea        v0q, [fill_levelcache_block_avx2_table]
    movsx     tmpq, dword [v0q+4*tmpq]
    add        v0q, tmpq
    jmp        v0q

; (shared path with sse2 for scalar, w1, w2, w4)

.w8:
    movd              xm0, v1d
    vpbroadcastw      xm0, xm0
.w8_loop:
    movu        [lc_ptrq], xm0
    LC_PTR_LOOP .w8_loop

.w16:
    movd              xm0, v1d
    vpbroadcastw       m0, xm0
.w16_loop:
    movu        [lc_ptrq], m0
    LC_PTR_LOOP .w16_loop

.w32:
    movd              xm0, v1d
    vpbroadcastw       m0, xm0
.w32_loop:
    movu        [lc_ptrq], m0
    movu     [lc_ptrq+32], m0
    LC_PTR_LOOP .w32_loop

%macro SCALAR_MASK_EDGE 1
    mov            masksq, masksm
    mov              txaq, txam
; masks holds pointer into masks[0][bx4] or masks[1][by4] and is advanced one
; byte when crossing a 8-bit mask boundary
%ifidn %1, left
%define               laq  lq
%define           la_offq  h4q
    lea                ad, [bx4q*3]
    add                lq, h4q
    mov             tmp1d, by4d
    neg               h4q
    mov             tmp2d, by4d
    lea            masksq, [masksq+aq*4]
%else
%define               laq  aq
%define           la_offq  tmp4q
    lea             tmp3d, [by4q*3+96]
    mov             tmp4d, w4d
    add                aq, w4q
    mov             tmp1d, bx4d
    neg             tmp4q
    mov             tmp2d, bx4d
    lea            masksq, [masksq+tmp3q*4]
    mov             tmp3d, h4d
    add              txaq, 2048
    shl             tmp3d, 5
%endif
    shr             tmp1d, 3
    and             tmp2d, 7
    xor             maskd, maskd
    add            masksq, tmp1q                ; pre-advance masks
    bts             maskd, tmp2d                ; mask = 1 << (by4 % 8)
%%loop:
    movzx           tmp1d, byte [laq+la_offq]
    movzx           tmp2d, byte [txaq]
    cmp             tmp1d, tmp2d                ; imin(txa[0][0][y][0], l[y])
    cmovae          tmp1d, tmp2d                ; or imin(txa[1][0][0][x], a[x])
%ifidn %1, left
    movzx           tmp2d, byte [txaq+w4q-1]    ; l[y] = txa[0][0][y][w4-1]
    add              txaq, 32
%else
    movzx           tmp2d, byte [txaq+tmp3q-32] ; a[x] = txa[1][0][h4-1][x]
    inc              txaq
%endif
    mov     [laq+la_offq], tmp2b
    or   [masksq+tmp1q*4], maskb
    rol             maskb, 1
    adc            masksq, 0                    ; advance to the next byte if necessary
    inc           la_offq
    jl %%loop

%endmacro

INIT_XMM sse2
; void mask_edges_inter_outer(uint8_t *a, uint8_t *l, int by4, int bx4, int w4, int h4,
;                             uint16_t (*masks)[32][3][2], uint8_t txa[2][2][32][32])
cglobal mask_edges_inter_outer, 4, 11, 4, a, l, by4, bx4, w4, h4, masks, \
                                          txa, mask, tmp1, tmp2, tmp3, tmp4
    mov               w4d, w4m
    mov               h4d, h4m
.scalar_top:
    PUSH            tmp3q
    PUSH            tmp4q
    SCALAR_MASK_EDGE top
    POP             tmp4q
    POP             tmp3q
.scalar_left:
    SCALAR_MASK_EDGE left
    RET

; Vectorized top edge for blocks at least 8 4x4 units wide
; (possibly clipped at the right frame edge).
%macro MASK_EDGE_TOP_VEC 2 ; aq_sz, mask_sz
    mov%1              m0, [aq]
%ifidn %1, q
    movq               m3, [txaq]
    movq               m2, [txaq+tmp1q-32]
    pminub             m0, m3
%else
    pminub             m0, [txaq]          ; imin(txa[1][0][0][x], a[x])
    mova               m2, [txaq+tmp1q-32] ; a[x] = txa[1][0][h4-1][x]
%endif
    pcmpeqb            m1, m0
    psllw              m0, 7
    mov%1            [aq], m2 ; partial overwrite at right frame edge is ok
    pmovmskb           ad, m1 ; index 0
    pmovmskb        tmp1d, m0 ; index 1
%if mmsize == 32
    vzeroupper
%endif
    and                ad, maskd
    and             tmp1d, maskd ; clear oob bits in the masks
    xor             maskd, ad
    xor             maskd, tmp1d ; index 2
    or       [masksq+4*0], a%2
    or       [masksq+4*1], tmp1%2
    or       [masksq+4*2], mask%2
    jmp mangle(private_prefix %+ _mask_edges_inter_outer_sse2).scalar_left
%endmacro

INIT_XMM avx2
; void mask_edges_inter_outer(uint8_t *a, uint8_t *l, int by4, int bx4, int w4, int h4,
;                             uint16_t (*masks)[32][3][2], uint8_t txa[2][2][32][32])
cglobal mask_edges_inter_outer, 4, 11, 4, a, l, by4, bx4, w4, h4, masks, \
                                          txa, tmp2, tmp1, mask
    mov               w4d, w4m
    mov               h4d, h4m
    cmp               w4d, 4
    jle mangle(private_prefix %+ _mask_edges_inter_outer_sse2).scalar_top
    mov            masksq, masksm
    mov              txaq, txam
    lea             tmp2d, [by4q*3+96]
    rorx            tmp1d, h4d, 27
    mov             maskd, -1
    pxor               m1, m1
    lea            masksq, [masksq+tmp2q*4] ; &masks[1][by4][0][0]
    add              txaq, 2048
    bzhi            maskd, maskd, w4d
    cmp               w4d, 16
    jg .top_w32
    rorx            tmp2d, bx4d, 3
    add            masksq, tmp2q
    cmp               w4d, 8
    jg .top_w16
.top_w8:
    MASK_EDGE_TOP_VEC   q, b
.top_w16:
    MASK_EDGE_TOP_VEC   a, w
INIT_YMM avx2
.top_w32:
    MASK_EDGE_TOP_VEC   u, d

%endif  ; ARCH_X86_64
