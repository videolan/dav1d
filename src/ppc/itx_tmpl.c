/*
 * Copyright © 2024, VideoLAN and dav1d authors
 * Copyright © 2024, Luca Barbato
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

#include "src/ppc/dav1d_types.h"
#include "src/ppc/itx.h"
#include "src/ppc/utils.h"

#if BITDEPTH == 8

#define LOAD_4(src, stride, a, b, c, d) \
{  \
    uint8_t *s = src; \
    a = vec_xl(0, s); \
    s += stride; \
    b = vec_xl(0, s); \
    s += stride; \
    c = vec_xl(0, s); \
    s += stride; \
    d = vec_xl(0, s); \
}

#define LOAD_DECLARE_2_I16(src, a, b) \
    i16x8 a = vec_xl(0, src); \
    i16x8 b = vec_xl(0, src + 8);

#define UNPACK_DECLARE_4_I16_I32(sa, sb, a, b, c, d) \
    i32x4 a = i16h_to_i32(sa); \
    i32x4 b = i16l_to_i32(sa); \
    i32x4 c = i16h_to_i32(sb); \
    i32x4 d = i16l_to_i32(sb);

#define LOAD_COEFF_4(coeff) \
    LOAD_DECLARE_2_I16(coeff, c01, c23) \
    UNPACK_DECLARE_4_I16_I32(c01, c23, c0, c1, c2, c3)

#define LOAD_SCALE_COEFF_4x8(coeff, scale) \
    LOAD_DECLARE_2_I16(coeff, c04, c15) \
    LOAD_DECLARE_2_I16(coeff+16, c26, c37) \
    i16x8 c01 = (i16x8)vec_mergeh((i64x2)c04, (i64x2)c15); \
    i16x8 c23 = (i16x8)vec_mergeh((i64x2)c26, (i64x2)c37); \
    i16x8 c45 = (i16x8)vec_mergel((i64x2)c04, (i64x2)c15); \
    i16x8 c67 = (i16x8)vec_mergel((i64x2)c26, (i64x2)c37); \
    c01 = vec_mradds(c01, scale, vec_splat_s16(0)); \
    c23 = vec_mradds(c23, scale, vec_splat_s16(0)); \
    UNPACK_DECLARE_4_I16_I32(c01, c23, c0, c1, c2, c3) \
    c45 = vec_mradds(c45, scale, vec_splat_s16(0)); \
    c67 = vec_mradds(c67, scale, vec_splat_s16(0)); \
    UNPACK_DECLARE_4_I16_I32(c45, c67, c4, c5, c6, c7)

#define LOAD_SCALE_COEFF_8x4(coeff, scale) \
    LOAD_DECLARE_2_I16(coeff, c01, c23) \
    LOAD_DECLARE_2_I16(coeff+16, c45, c67) \
    c01 = vec_mradds(c01, scale, vec_splat_s16(0)); \
    c23 = vec_mradds(c23, scale, vec_splat_s16(0)); \
    UNPACK_DECLARE_4_I16_I32(c01, c23, c0, c1, c2, c3) \
    c45 = vec_mradds(c45, scale, vec_splat_s16(0)); \
    c67 = vec_mradds(c67, scale, vec_splat_s16(0)); \
    UNPACK_DECLARE_4_I16_I32(c45, c67, c4, c5, c6, c7)

#define LOAD_COEFF_8x8(coeff) \
    LOAD_DECLARE_2_I16(coeff, c0, c1) \
    LOAD_DECLARE_2_I16(coeff+16, c2, c3) \
    LOAD_DECLARE_2_I16(coeff+32, c4, c5) \
    LOAD_DECLARE_2_I16(coeff+48, c6, c7) \
    UNPACK_DECLARE_4_I16_I32(c0, c1, c0h, c0l, c1h, c1l) \
    UNPACK_DECLARE_4_I16_I32(c2, c3, c2h, c2l, c3h, c3l) \
    UNPACK_DECLARE_4_I16_I32(c4, c5, c4h, c4l, c5h, c5l) \
    UNPACK_DECLARE_4_I16_I32(c6, c7, c6h, c6l, c7h, c7l) \

#define LOAD_DECLARE_4(src, stride, a, b, c, d) \
    u8x16 a, b, c, d; \
    LOAD_4(src, stride, a, b, c, d)

#define STORE_LEN(l, dst, stride, a, b, c, d) \
{ \
    uint8_t *dst2 = dst; \
    vec_xst_len(a, dst2, l); \
    dst2 += stride; \
    vec_xst_len(b, dst2, l); \
    dst2 += stride; \
    vec_xst_len(c, dst2, l); \
    dst2 += stride; \
    vec_xst_len(d, dst2, l); \
}

#define STORE_4(dst, stride, a, b, c, d) \
    STORE_LEN(4, dst, stride, a, b, c, d)

#define STORE_8(dst, stride, ab, cd, ef, gh) \
    STORE_LEN(8, dst, stride, ab, cd, ef, gh)

#define APPLY_COEFF_4(a, b, c, d, c01, c23) \
{ \
    u8x16 ab = (u8x16)vec_mergeh((u32x4)a, (u32x4)b); \
    u8x16 cd = (u8x16)vec_mergeh((u32x4)c, (u32x4)d); \
 \
    c01 = vec_adds(c01, vec_splat_s16(8)); \
    c23 = vec_adds(c23, vec_splat_s16(8)); \
    c01 = vec_sra(c01, vec_splat_u16(4)); \
    c23 = vec_sra(c23, vec_splat_u16(4)); \
 \
    i16x8 abs = u8h_to_i16(ab); \
    i16x8 cds = u8h_to_i16(cd); \
 \
    abs = vec_adds(abs, c01); \
    cds = vec_adds(cds, c23); \
 \
    a = vec_packsu(abs, abs); \
    c = vec_packsu(cds, cds); \
 \
    b = (u8x16)vec_mergeo((u32x4)a, (u32x4)a); \
    d = (u8x16)vec_mergeo((u32x4)c, (u32x4)c); \
}

#define APPLY_COEFF_8x4(ab, cd, c01, c23) \
{ \
    i16x8 abs = u8h_to_i16(ab); \
    i16x8 cds = u8h_to_i16(cd); \
    c01 = vec_adds(c01, vec_splat_s16(8)); \
    c23 = vec_adds(c23, vec_splat_s16(8)); \
    c01 = vec_sra(c01, vec_splat_u16(4)); \
    c23 = vec_sra(c23, vec_splat_u16(4)); \
 \
    abs = vec_adds(abs, c01); \
    cds = vec_adds(cds, c23); \
 \
    ab = vec_packsu(abs, abs); \
    cd = vec_packsu(cds, cds); \
}

#define IDCT_4_INNER(c0, c1, c2, c3) \
{ \
    i32x4 o0 = vec_add(c0, c2); \
    i32x4 o1 = vec_sub(c0, c2); \
 \
    i32x4 v2896 = vec_splats(2896); \
    i32x4 v1567 = vec_splats(1567); \
    i32x4 v3784 = vec_splats(3784); \
    i32x4 v2048 = vec_splats(2048); \
 \
    o0 = vec_mul(o0, v2896); \
    o1 = vec_mul(o1, v2896); \
 \
    i32x4 o2a = vec_mul(c1, v1567); \
    i32x4 o2b = vec_mul(c3, v3784); \
    i32x4 o3a = vec_mul(c1, v3784); \
    i32x4 o3b = vec_mul(c3, v1567); \
 \
    i32x4 o2 = vec_sub(o2a, o2b); \
    i32x4 o3 = vec_add(o3a, o3b); \
 \
    u32x4 v12 = vec_splat_u32(12); \
 \
    o0 = vec_add(o0, v2048); \
    o1 = vec_add(o1, v2048); \
    o2 = vec_add(o2, v2048); \
    o3 = vec_add(o3, v2048); \
 \
    o0 = vec_sra(o0, v12); \
    o1 = vec_sra(o1, v12); \
    o2 = vec_sra(o2, v12); \
    o3 = vec_sra(o3, v12); \
 \
    c0 = vec_add(o0, o3); \
    c1 = vec_add(o1, o2); \
    c2 = vec_sub(o1, o2); \
    c3 = vec_sub(o0, o3); \
 \
}

#define dct4_for_dct8(c0, c1, c2, c3, c03, c12) \
    IDCT_4_INNER(c0, c1, c2, c3) \
    c03 = vec_packs(c0, c3); \
    c12 = vec_packs(c1, c2); \

#define dct_4_in(c0, c1, c2, c3, c01, c23) \
{ \
    IDCT_4_INNER(c0, c1, c2, c3) \
    c01 = vec_packs(c0, c1); \
    c23 = vec_packs(c2, c3); \
    c0 = i16h_to_i32(c01); \
    c1 = i16l_to_i32(c01); \
    c2 = i16h_to_i32(c23); \
    c3 = i16l_to_i32(c23); \
}

#define dct_4_out(c0, c1, c2, c3, c01, c23) \
    IDCT_4_INNER(c0, c1, c2, c3) \
    c01 = vec_packs(c0, c1); \
    c23 = vec_packs(c2, c3); \


#define IDENTITY_4(c0, c1, c2, c3, c01, c23) \
{ \
    i16x8 v1697 = vec_splats((int16_t)(1697*8)); \
    i16x8 o01 = vec_mradds(c01, v1697, vec_splat_s16(0)); \
    i16x8 o23 = vec_mradds(c23, v1697, vec_splat_s16(0)); \
    c01 = vec_adds(c01, o01); \
    c23 = vec_adds(c23, o23); \
}

#define identity_4_in(c0, c1, c2, c3, c01, c23) \
{ \
    IDENTITY_4(c0, c1, c2, c3, c01, c23) \
    c0 = i16h_to_i32(c01); \
    c1 = i16l_to_i32(c01); \
    c2 = i16h_to_i32(c23); \
    c3 = i16l_to_i32(c23); \
}

#define identity_4_out(c0, c1, c2, c3, c01, c23) \
{ \
    c01 = vec_packs(c0, c1); \
    c23 = vec_packs(c2, c3); \
    IDENTITY_4(c0, c1, c2, c3, c01, c23) \
}

#define ADST_INNER_4(c0, c1, c2, c3, oc0, oc1, oc2, oc3) \
{ \
    i32x4 v1321 = vec_splats(1321); \
    i32x4 v3803 = vec_splats(3803); \
    i32x4 v2482 = vec_splats(2482); \
    i32x4 v3344 = vec_splats(3344); \
    i32x4 v2048 = vec_splats(2048); \
    i32x4 i0_v1321 = vec_mul(c0, v1321); \
    i32x4 i0_v2482 = vec_mul(c0, v2482); \
    i32x4 i0_v3803 = vec_mul(c0, v3803); \
    i32x4 i1 = vec_mul(c1, v3344); \
    i32x4 i2_v1321 = vec_mul(c2, v1321); \
    i32x4 i2_v2482 = vec_mul(c2, v2482); \
    i32x4 i2_v3803 = vec_mul(c2, v3803); \
    i32x4 i3_v1321 = vec_mul(c3, v1321); \
    i32x4 i3_v2482 = vec_mul(c3, v2482); \
    i32x4 i3_v3803 = vec_mul(c3, v3803); \
 \
    i32x4 n1 = vec_sub(i1, v2048); \
    i1 = vec_add(i1, v2048); \
 \
 \
    i32x4 o0 = vec_add(i0_v1321, i2_v3803); \
    i32x4 o1 = vec_sub(i0_v2482, i2_v1321); \
    i32x4 o2 = vec_sub(c0, c2); \
    i32x4 o3 = vec_add(i0_v3803, i2_v2482); \
 \
    o0 = vec_add(o0, i3_v2482); \
    o1 = vec_sub(o1, i3_v3803); \
    o2 = vec_add(o2, c3); \
    o3 = vec_sub(o3, i3_v1321); \
 \
    o0 = vec_add(o0, i1); \
    o1 = vec_add(o1, i1); \
    o2 = vec_mul(o2, v3344); \
    o3 = vec_sub(o3, n1); \
 \
    o2 = vec_add(o2, v2048); \
 \
    oc0 = vec_sra(o0, vec_splat_u32(12)); \
    oc1 = vec_sra(o1, vec_splat_u32(12)); \
    oc2 = vec_sra(o2, vec_splat_u32(12)); \
    oc3 = vec_sra(o3, vec_splat_u32(12)); \
}

#define adst_4_in(c0, c1, c2, c3, c01, c23) \
{ \
    ADST_INNER_4(c0, c1, c2, c3, c0, c1, c2, c3) \
}

#define flipadst_4_in(c0, c1, c2, c3, c01, c23) \
{ \
    ADST_INNER_4(c0, c1, c2, c3, c3, c2, c1, c0) \
}

#define adst_4_out(c0, c1, c2, c3, c01, c23) \
{ \
    ADST_INNER_4(c0, c1, c2, c3, c0, c1, c2, c3) \
    c01 = vec_packs(c0, c1); \
    c23 = vec_packs(c2, c3); \
}

#define flipadst_4_out(c0, c1, c2, c3, c01, c23) \
{ \
    ADST_INNER_4(c0, c1, c2, c3, c3, c2, c1, c0) \
    c01 = vec_packs(c0, c1); \
    c23 = vec_packs(c2, c3); \
}

void dav1d_inv_txfm_add_dct_dct_4x4_8bpc_pwr9(uint8_t *dst, const ptrdiff_t stride,
                                              int16_t *const coeff, const int eob
                                              HIGHBD_DECL_SUFFIX)
{
    assert(eob >= 0);

    if (eob < 1) {
        int dc = coeff[0];
        i16x8 vdc = vec_splats((int16_t)dc);
        i16x8 v = vec_splats((int16_t)(2896*8));
        coeff[0] = 0;
        vdc = vec_mradds(vdc, v, vec_splat_s16(0));
        vdc = vec_mradds(vdc, v, vec_splat_s16(8));
        vdc = vec_sra(vdc, vec_splat_u16(4));
        LOAD_DECLARE_4(dst, stride, a, b, c, d)

        i16x8 as = u8h_to_i16(a);
        i16x8 bs = u8h_to_i16(b);
        i16x8 cs = u8h_to_i16(c);
        i16x8 ds = u8h_to_i16(d);

        as = vec_adds(as, vdc);
        bs = vec_adds(bs, vdc);
        cs = vec_adds(cs, vdc);
        ds = vec_adds(ds, vdc);

        a = vec_packsu(as, as);
        b = vec_packsu(bs, bs);
        c = vec_packsu(cs, cs);
        d = vec_packsu(ds, ds);

        STORE_4(dst, stride, a, b, c, d)

        return;
    }

    LOAD_COEFF_4(coeff)

    dct_4_in(c0, c1, c2, c3, c01, c23)

    TRANSPOSE4_I32(c0, c1, c2, c3)

    memset(coeff, 0, sizeof(*coeff) * 4 * 4);

    dct_4_out(c0, c1, c2, c3, c01, c23)

    LOAD_DECLARE_4(dst, stride, a, b, c, d)

    APPLY_COEFF_4(a, b, c, d, c01, c23)

    STORE_4(dst, stride, a, b, c, d)
}

void dav1d_inv_txfm_add_wht_wht_4x4_8bpc_pwr9(pixel *dst, const ptrdiff_t stride,
                                              coef *const coeff, const int eob
                                              HIGHBD_DECL_SUFFIX)
{
    LOAD_COEFF_4(coeff)

    u32x4 v2 = vec_splat_u32(2);

    c0 = vec_sra(c0, v2);
    c1 = vec_sra(c1, v2);
    c2 = vec_sra(c2, v2);
    c3 = vec_sra(c3, v2);

    i32x4 t0 = vec_add(c0, c1);
    i32x4 t2 = vec_sub(c2, c3);
    i32x4 t4 = vec_sra(vec_sub(t0, t2), vec_splat_u32(1));
    i32x4 t3 = vec_sub(t4, c3);
    i32x4 t1 = vec_sub(t4, c1);
    c0 = vec_sub(t0, t3);
    c1 = t3;
    c2 = t1;
    c3 = vec_add(t2, t1);

    memset(coeff, 0, sizeof(*coeff) * 4 * 4);

    TRANSPOSE4_I32(c0, c1, c2, c3)

    t0 = vec_add(c0, c1);
    t2 = vec_sub(c2, c3);
    t4 = vec_sra(vec_sub(t0, t2), vec_splat_u32(1));
    t3 = vec_sub(t4, c3);
    t1 = vec_sub(t4, c1);
    c0 = vec_sub(t0, t3);
    c1 = t3;
    c2 = t1;
    c3 = vec_add(t2, t1);

    c01 = vec_packs(c0, c1);
    c23 = vec_packs(c2, c3);

    LOAD_DECLARE_4(dst, stride, a, b, c, d)

    u8x16 ab = (u8x16)vec_mergeh((u32x4)a, (u32x4)b);
    u8x16 cd = (u8x16)vec_mergeh((u32x4)c, (u32x4)d);

    i16x8 abs = u8h_to_i16(ab);
    i16x8 cds = u8h_to_i16(cd);

    abs = vec_adds(abs, c01);
    cds = vec_adds(cds, c23);

    a = vec_packsu(abs, abs);
    c = vec_packsu(cds, cds);

    b = (u8x16)vec_mergeo((u32x4)a, (u32x4)a);
    d = (u8x16)vec_mergeo((u32x4)c, (u32x4)c);

    STORE_4(dst, stride, a, b, c, d)
}

#define inv_txfm_fn4x4(type1, type2) \
void dav1d_inv_txfm_add_##type1##_##type2##_4x4_8bpc_pwr9(uint8_t *dst, const ptrdiff_t stride, \
                                                          int16_t *const coeff, const int eob \
                                                          HIGHBD_DECL_SUFFIX) \
{ \
    LOAD_COEFF_4(coeff) \
    type1##_4_in(c0, c1, c2, c3, c01, c23) \
    memset(coeff, 0, sizeof(*coeff) * 4 * 4); \
    TRANSPOSE4_I32(c0, c1, c2, c3) \
    type2##_4_out(c0, c1, c2, c3, c01, c23) \
    LOAD_DECLARE_4(dst, stride, a, b, c, d) \
    APPLY_COEFF_4(a, b, c, d, c01, c23) \
    STORE_4(dst, stride, a, b, c, d) \
}

inv_txfm_fn4x4(adst,     dct     )
inv_txfm_fn4x4(dct,      adst    )
inv_txfm_fn4x4(dct,      flipadst)
inv_txfm_fn4x4(flipadst, dct     )
inv_txfm_fn4x4(adst,     flipadst)
inv_txfm_fn4x4(flipadst, adst    )
inv_txfm_fn4x4(identity, dct     )
inv_txfm_fn4x4(dct,      identity)
inv_txfm_fn4x4(identity, flipadst)
inv_txfm_fn4x4(flipadst, identity)
inv_txfm_fn4x4(identity, adst   )
inv_txfm_fn4x4(adst,     identity)
inv_txfm_fn4x4(identity, identity)
inv_txfm_fn4x4(adst,     adst    )
inv_txfm_fn4x4(flipadst, flipadst)

#define IDCT_8_INNER(c0, c1, c2, c3, c4, c5, c6, c7, c03, c12, c74, c65) \
    dct4_for_dct8(c0, c2, c4, c6, c03, c12) \
 \
    i32x4 v799 = vec_splats(799); \
    i32x4 v4017 = vec_splats(4017); \
    i32x4 v3406 = vec_splats(3406); \
    i32x4 v2276 = vec_splats(2276); \
    i32x4 v2048 = vec_splats(2048); \
    u32x4 v12 = vec_splat_u32(12); \
 \
    i32x4 c1v799 = vec_mul(c1, v799); \
    i32x4 c7v4017 = vec_mul(c7, v4017); \
    i32x4 c5v3406 = vec_mul(c5, v3406); \
    i32x4 c3v2276 = vec_mul(c3, v2276); \
    i32x4 c5v2276 = vec_mul(c5, v2276); \
    i32x4 c3v3406 = vec_mul(c3, v3406); \
    i32x4 c1v4017 = vec_mul(c1, v4017); \
    i32x4 c7v799 = vec_mul(c7, v799); \
 \
    i32x4 t4a = vec_subs(c1v799, c7v4017); \
    i32x4 t5a = vec_subs(c5v3406, c3v2276); \
    i32x4 t6a = vec_adds(c5v2276, c3v3406); \
    i32x4 t7a = vec_adds(c1v4017, c7v799); \
 \
    t4a = vec_adds(t4a, v2048); \
    t5a = vec_adds(t5a, v2048); \
    t6a = vec_adds(t6a, v2048); \
    t7a = vec_adds(t7a, v2048); \
 \
    t4a = vec_sra(t4a, v12); \
    t7a = vec_sra(t7a, v12); \
    t5a = vec_sra(t5a, v12); \
    t6a = vec_sra(t6a, v12); \
 \
    i16x8 t7at4a = vec_packs(t7a, t4a); \
    i16x8 t6at5a = vec_packs(t6a, t5a); \
 \
    i16x8 t7t4 = vec_adds(t7at4a, t6at5a); \
    t6at5a = vec_subs(t7at4a, t6at5a); \
 \
    t6a = i16h_to_i32(t6at5a); \
    t5a = i16l_to_i32(t6at5a); \
 \
    i32x4 t6 = vec_add(t6a, t5a); \
    i32x4 t5 = vec_sub(t6a, t5a); \
 \
    t6 = vec_mul(t6, vec_splats(181)); \
    t5 = vec_mul(t5, vec_splats(181)); \
    t6 = vec_add(t6, vec_splats(128)); \
    t5 = vec_add(t5, vec_splats(128)); \
 \
    t6 = vec_sra(t6, vec_splat_u32(8)); \
    t5 = vec_sra(t5, vec_splat_u32(8)); \
 \
    i16x8 t6t5 = vec_packs(t6, t5); \
 \
    c74 = vec_subs(c03, t7t4); \
    c65 = vec_subs(c12, t6t5); \
    c03 = vec_adds(c03, t7t4); \
    c12 = vec_adds(c12, t6t5); \

#define UNPACK_4_I16_I32(t0, t1, t2, t3) \
    t0 = i16h_to_i32(t0##t1); \
    t1 = i16l_to_i32(t0##t1); \
    t2 = i16h_to_i32(t2##t3); \
    t3 = i16l_to_i32(t2##t3);

#define UNPACK_PAIR_I16_I32(hi, lo, v) \
    hi = i16h_to_i32(v); \
    lo = i16l_to_i32(v); \


#define dct_8_in(c0, c1, c2, c3, c4, c5, c6, c7, ...) \
{ \
    i16x8 c0##c3, c1##c2, c7##c4, c6##c5; \
    IDCT_8_INNER(c0, c1, c2, c3, c4, c5, c6, c7, c0##c3, c1##c2, c7##c4, c6##c5) \
    UNPACK_4_I16_I32(c0, c3, c1, c2) \
    UNPACK_4_I16_I32(c7, c4, c6, c5) \
}

#define dct_8_out(c0, c1, c2, c3, c4, c5, c6, c7, c01, c23, c45, c67) \
{ \
    i16x8 c03, c12, c74, c65; \
    IDCT_8_INNER(c0, c1, c2, c3, c4, c5, c6, c7, c03, c12, c74, c65) \
    c01 = (i16x8)vec_mergeh((u64x2)c03, (u64x2)c12); \
    c23 = (i16x8)vec_mergel((u64x2)c12, (u64x2)c03); \
    c45 = (i16x8)vec_mergel((u64x2)c74, (u64x2)c65); \
    c67 = (i16x8)vec_mergeh((u64x2)c65, (u64x2)c74); \
}

#define dct_8x2_in(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h, \
                   c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l, \
                   c0, c1, c2, c3, c4, c5, c6, c7) \
{ \
    dct_8_in(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h,) \
    dct_8_in(c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l,) \
}

#define dct_8x2_out(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h, \
                    c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l, \
                    c0, c1, c2, c3, c4, c5, c6, c7) \
{ \
    i16x8 c03h, c12h, c74h, c65h; \
    i16x8 c03l, c12l, c74l, c65l; \
    { \
        IDCT_8_INNER(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h, c03h, c12h, c74h, c65h) \
    } \
    { \
        IDCT_8_INNER(c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l, c03l, c12l, c74l, c65l) \
    } \
    c0 = (i16x8)vec_mergeh((u64x2)c03h, (u64x2)c03l); \
    c3 = (i16x8)vec_mergel((u64x2)c03h, (u64x2)c03l); \
    c1 = (i16x8)vec_mergeh((u64x2)c12h, (u64x2)c12l); \
    c2 = (i16x8)vec_mergel((u64x2)c12h, (u64x2)c12l); \
    c7 = (i16x8)vec_mergeh((u64x2)c74h, (u64x2)c74l); \
    c4 = (i16x8)vec_mergel((u64x2)c74h, (u64x2)c74l); \
    c6 = (i16x8)vec_mergeh((u64x2)c65h, (u64x2)c65l); \
    c5 = (i16x8)vec_mergel((u64x2)c65h, (u64x2)c65l); \
}

#define IDENTITY_8(c01, c23, c45, c67) \
{ \
    c01 = vec_adds(c01, c01); \
    c23 = vec_adds(c23, c23); \
    c45 = vec_adds(c45, c45); \
    c67 = vec_adds(c67, c67); \
}

#define identity_8_in(c0, c1, c2, c3, c4, c5, c6, c7, c01, c23, c45, c67) \
{ \
    IDENTITY_8(c01, c23, c45, c67) \
    UNPACK_PAIR_I16_I32(c0, c1, c01) \
    UNPACK_PAIR_I16_I32(c2, c3, c23) \
    UNPACK_PAIR_I16_I32(c4, c5, c45) \
    UNPACK_PAIR_I16_I32(c6, c7, c67) \
}

#define identity_8_out(c0, c1, c2, c3, c4, c5, c6, c7, c01, c23, c45, c67) \
    c01 = vec_packs(c0, c1); \
    c23 = vec_packs(c2, c3); \
    c45 = vec_packs(c4, c5); \
    c67 = vec_packs(c6, c7); \
    IDENTITY_8(c01, c23, c45, c67)

#define identity_8x2_in(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h, \
                        c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l, \
                        c0, c1, c2, c3, c4, c5, c6, c7) \
{ \
    IDENTITY_8(c0, c1, c2, c3) \
    IDENTITY_8(c4, c5, c6, c7) \
    UNPACK_PAIR_I16_I32(c0h, c0l, c0) \
    UNPACK_PAIR_I16_I32(c1h, c1l, c1) \
    UNPACK_PAIR_I16_I32(c2h, c2l, c2) \
    UNPACK_PAIR_I16_I32(c3h, c3l, c3) \
    UNPACK_PAIR_I16_I32(c4h, c4l, c4) \
    UNPACK_PAIR_I16_I32(c5h, c5l, c5) \
    UNPACK_PAIR_I16_I32(c6h, c6l, c6) \
    UNPACK_PAIR_I16_I32(c7h, c7l, c7) \
}

#define PACK_8(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h, \
               c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l, \
               c0, c1, c2, c3, c4, c5, c6, c7) \
{ \
    c0 = vec_packs(c0h, c0l); \
    c1 = vec_packs(c1h, c1l); \
    c2 = vec_packs(c2h, c2l); \
    c3 = vec_packs(c3h, c3l); \
    c4 = vec_packs(c4h, c4l); \
    c5 = vec_packs(c5h, c5l); \
    c6 = vec_packs(c6h, c6l); \
    c7 = vec_packs(c7h, c7l); \
}

#define identity_8x2_out(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h, \
                         c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l, \
                         c0, c1, c2, c3, c4, c5, c6, c7) \
{ \
    PACK_8(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h, \
           c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l, \
           c0, c1, c2, c3, c4, c5, c6, c7) \
    IDENTITY_8(c0, c1, c2, c3) \
    IDENTITY_8(c4, c5, c6, c7) \
}

#define DECLARE_SPLAT_I32(val) \
    i32x4 v##val = vec_splats(val);

#define DECLARE_MUL_PAIR_I32(ca, cb, va, vb) \
    i32x4 ca##va = vec_mul(ca, va); \
    i32x4 cb##vb = vec_mul(cb, vb); \
    i32x4 ca##vb = vec_mul(ca, vb); \
    i32x4 cb##va = vec_mul(cb, va);

#define ADD_SUB_PAIR(r0, r1, ca, cb, va, vb) \
    r0 = vec_add(ca##va, cb##vb); \
    r1 = vec_sub(ca##vb, cb##va);

#define DECLARE_ADD_SUB_PAIR(r0, r1, ca, cb, va, vb) \
    i32x4 r0, r1; \
    ADD_SUB_PAIR(r0, r1, ca, cb, va, vb)

#define SCALE_ROUND_4(a, b, c, d, rnd, shift) \
    a = vec_add(a, rnd); \
    b = vec_add(b, rnd); \
    c = vec_add(c, rnd); \
    d = vec_add(d, rnd); \
    a = vec_sra(a, shift); \
    b = vec_sra(b, shift); \
    c = vec_sra(c, shift); \
    d = vec_sra(d, shift);

#define ADST_INNER_8(c0, c1, c2, c3, c4, c5, c6, c7, \
                     o0, o1, o2, o3, o4, o5, o6, o7) \
{ \
    DECLARE_SPLAT_I32(4076) \
    DECLARE_SPLAT_I32(401) \
 \
    DECLARE_SPLAT_I32(3612) \
    DECLARE_SPLAT_I32(1931) \
 \
    DECLARE_SPLAT_I32(2598) \
    DECLARE_SPLAT_I32(3166) \
 \
    DECLARE_SPLAT_I32(1189) \
    DECLARE_SPLAT_I32(3920) \
 \
    DECLARE_SPLAT_I32(3784) \
    DECLARE_SPLAT_I32(1567) \
 \
    DECLARE_SPLAT_I32(2048) \
    u32x4 v12 = vec_splat_u32(12); \
 \
    DECLARE_MUL_PAIR_I32(c7, c0, v4076, v401) \
    DECLARE_MUL_PAIR_I32(c5, c2, v3612, v1931) \
    DECLARE_MUL_PAIR_I32(c3, c4, v2598, v3166) \
    DECLARE_MUL_PAIR_I32(c1, c6, v1189, v3920) \
 \
    DECLARE_ADD_SUB_PAIR(t0a, t1a, c7, c0, v4076, v401) \
    DECLARE_ADD_SUB_PAIR(t2a, t3a, c5, c2, v3612, v1931) \
    DECLARE_ADD_SUB_PAIR(t4a, t5a, c3, c4, v2598, v3166) \
    DECLARE_ADD_SUB_PAIR(t6a, t7a, c1, c6, v1189, v3920) \
 \
    SCALE_ROUND_4(t0a, t1a, t2a, t3a, v2048, v12) \
    SCALE_ROUND_4(t4a, t5a, t6a, t7a, v2048, v12) \
 \
    i32x4 t0 = vec_add(t0a, t4a); \
    i32x4 t1 = vec_add(t1a, t5a); \
    i32x4 t2 = vec_add(t2a, t6a); \
    i32x4 t3 = vec_add(t3a, t7a); \
    i32x4 t4 = vec_sub(t0a, t4a); \
    i32x4 t5 = vec_sub(t1a, t5a); \
    i32x4 t6 = vec_sub(t2a, t6a); \
    i32x4 t7 = vec_sub(t3a, t7a); \
 \
    i16x8 t0t1 = vec_packs(t0, t1); \
    i16x8 t2t3 = vec_packs(t2, t3); \
    i16x8 t4t5 = vec_packs(t4, t5); \
    i16x8 t6t7 = vec_packs(t6, t7); \
 \
    UNPACK_4_I16_I32(t4, t5, t6, t7) \
    UNPACK_4_I16_I32(t0, t1, t2, t3) \
 \
    DECLARE_MUL_PAIR_I32(t4, t5, v3784, v1567) \
    DECLARE_MUL_PAIR_I32(t7, t6, v3784, v1567) \
 \
    ADD_SUB_PAIR(t4a, t5a, t4, t5, v3784, v1567) \
    ADD_SUB_PAIR(t7a, t6a, t7, t6, v1567, v3784) \
 \
    SCALE_ROUND_4(t4a, t5a, t6a, t7a, v2048, v12) \
  \
    o0 = vec_add(t0, t2); \
    o1 = vec_add(t4a, t6a); \
    o7 = vec_add(t1, t3); \
    o6 = vec_add(t5a, t7a); \
    t2 = vec_sub(t0, t2); \
    t3 = vec_sub(t1, t3); \
    t6 = vec_sub(t4a, t6a); \
    t7 = vec_sub(t5a, t7a); \
 \
    i16x8 o7##o1 = vec_packs(o7, o1); \
    i16x8 o0##o6 = vec_packs(o0, o6); \
    t2t3 = vec_packs(t2, t3); \
    t6t7 = vec_packs(t6, t7); \
 \
    UNPACK_4_I16_I32(t2, t3, t6, t7) \
    UNPACK_4_I16_I32(o7, o1, o0, o6) \
 \
    o7 = -o7; \
    o1 = -o1; \
 \
    o3 = vec_add(t2, t3); \
    o4 = vec_sub(t2, t3); \
    o5 = vec_sub(t6, t7); \
    o2 = vec_add(t6, t7); \
 \
    i32x4 v181 = vec_splats(181); \
    i32x4 v128 = vec_splats(128); \
    u32x4 v8 = vec_splat_u32(8); \
 \
    o2 = vec_mul(o2, v181); \
    o3 = vec_mul(o3, v181); \
    o4 = vec_mul(o4, v181); \
    o5 = vec_mul(o5, v181); \
 \
    SCALE_ROUND_4(o2, o3, o4, o5, v128, v8) \
 \
    o3 = -o3; \
    o5 = -o5; \
}

#define adst_8_in(c0, c1, c2, c3, c4, c5, c6, c7, c01, c23, c45, c67) \
{\
    ADST_INNER_8(c0, c1, c2, c3, c4, c5, c6, c7, \
                 c0, c1, c2, c3, c4, c5, c6, c7) \
    c01 = vec_packs(c0, c1); \
    c23 = vec_packs(c2, c3); \
    c45 = vec_packs(c4, c5); \
    c67 = vec_packs(c6, c7); \
    UNPACK_PAIR_I16_I32(c0, c1, c01) \
    UNPACK_PAIR_I16_I32(c2, c3, c23) \
    UNPACK_PAIR_I16_I32(c4, c5, c45) \
    UNPACK_PAIR_I16_I32(c6, c7, c67) \
}

#define adst_8_out(c0, c1, c2, c3, c4, c5, c6, c7, c01, c23, c45, c67) \
{\
    ADST_INNER_8(c0, c1, c2, c3, c4, c5, c6, c7, \
                 c0, c1, c2, c3, c4, c5, c6, c7) \
    c01 = vec_packs(c0, c1); \
    c23 = vec_packs(c2, c3); \
    c45 = vec_packs(c4, c5); \
    c67 = vec_packs(c6, c7); \
}

#define adst_8x2_in(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h, \
                    c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l, \
                    c0, c1, c2, c3, c4, c5, c6, c7) \
{ \
    ADST_INNER_8(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h, \
                 c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h) \
    ADST_INNER_8(c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l, \
                 c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l) \
}

#define adst_8x2_out(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h, \
                    c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l, \
                    c0, c1, c2, c3, c4, c5, c6, c7) \
{ \
    ADST_INNER_8(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h, \
                 c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h) \
    ADST_INNER_8(c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l, \
                 c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l) \
    PACK_8(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h, \
           c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l, \
           c0, c1, c2, c3, c4, c5, c6, c7) \
}

#define flipadst_8_in(c0, c1, c2, c3, c4, c5, c6, c7, c01, c23, c45, c67) \
{\
    ADST_INNER_8(c0, c1, c2, c3, c4, c5, c6, c7, \
                 c7, c6, c5, c4, c3, c2, c1, c0) \
    c01 = vec_packs(c0, c1); \
    c23 = vec_packs(c2, c3); \
    c45 = vec_packs(c4, c5); \
    c67 = vec_packs(c6, c7); \
    UNPACK_PAIR_I16_I32(c0, c1, c01) \
    UNPACK_PAIR_I16_I32(c2, c3, c23) \
    UNPACK_PAIR_I16_I32(c4, c5, c45) \
    UNPACK_PAIR_I16_I32(c6, c7, c67) \
}

#define flipadst_8_out(c0, c1, c2, c3, c4, c5, c6, c7, c01, c23, c45, c67) \
{\
    ADST_INNER_8(c0, c1, c2, c3, c4, c5, c6, c7, \
                 c7, c6, c5, c4, c3, c2, c1, c0) \
    c01 = vec_packs(c0, c1); \
    c23 = vec_packs(c2, c3); \
    c45 = vec_packs(c4, c5); \
    c67 = vec_packs(c6, c7); \
}

#define flipadst_8x2_in(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h, \
                        c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l, \
                        c0, c1, c2, c3, c4, c5, c6, c7) \
{ \
    ADST_INNER_8(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h, \
                 c7h, c6h, c5h, c4h, c3h, c2h, c1h, c0h) \
    ADST_INNER_8(c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l, \
                 c7l, c6l, c5l, c4l, c3l, c2l, c1l, c0l) \
}

#define flipadst_8x2_out(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h, \
                         c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l, \
                         c0, c1, c2, c3, c4, c5, c6, c7) \
{ \
    ADST_INNER_8(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h, \
                 c7h, c6h, c5h, c4h, c3h, c2h, c1h, c0h) \
    ADST_INNER_8(c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l, \
                 c7l, c6l, c5l, c4l, c3l, c2l, c1l, c0l) \
    PACK_8(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h, \
           c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l, \
           c0, c1, c2, c3, c4, c5, c6, c7) \
}

void dav1d_inv_txfm_add_dct_dct_4x8_8bpc_pwr9(uint8_t *dst, const ptrdiff_t stride,
                                              int16_t *const coeff, const int eob
                                              HIGHBD_DECL_SUFFIX)
{
    i16x8 v = vec_splats((int16_t)(2896*8));

    if (eob < 1) {
        int dc = coeff[0];
        i16x8 vdc = vec_splats((int16_t)dc);
        coeff[0] = 0;
        vdc = vec_mradds(vdc, v, vec_splat_s16(0));
        vdc = vec_mradds(vdc, v, vec_splat_s16(0));
        vdc = vec_mradds(vdc, v, vec_splat_s16(8));
        vdc = vec_sra(vdc, vec_splat_u16(4));

        for (int i = 0; i < 2; i++, dst += 4 * stride) {
            LOAD_DECLARE_4(dst, stride, a, b, c, d)

            i16x8 as = u8h_to_i16(a);
            i16x8 bs = u8h_to_i16(b);
            i16x8 cs = u8h_to_i16(c);
            i16x8 ds = u8h_to_i16(d);

            as = vec_adds(as, vdc);
            bs = vec_adds(bs, vdc);
            cs = vec_adds(cs, vdc);
            ds = vec_adds(ds, vdc);

            a = vec_packsu(as, as);
            b = vec_packsu(bs, bs);
            c = vec_packsu(cs, cs);
            d = vec_packsu(ds, ds);

            STORE_4(dst, stride, a, b, c, d)
        }

        return;
    }

    LOAD_SCALE_COEFF_4x8(coeff, v)

    dct_4_in(c0, c1, c2, c3, c01, c23)
    dct_4_in(c4, c5, c6, c7, c45, c67)


    memset(coeff, 0, sizeof(*coeff) * 4 * 8);

    TRANSPOSE4_I32(c0, c1, c2, c3);
    TRANSPOSE4_I32(c4, c5, c6, c7);

    dct_8_out(c0, c1, c2, c3, c4, c5, c6, c7, c01, c23, c45, c67)

    LOAD_DECLARE_4(dst, stride, a, b, cc, d)
    LOAD_DECLARE_4(dst + 4 * stride, stride, e, f, g, hh)

    APPLY_COEFF_4(a, b, cc, d, c01, c23)
    APPLY_COEFF_4(e, f, g, hh, c45, c67)

    STORE_4(dst, stride, a, b, cc, d)
    STORE_4(dst + 4 * stride, stride, e, f, g, hh)
}


#define inv_txfm_fn4x8(type1, type2) \
void dav1d_inv_txfm_add_##type1##_##type2##_4x8_8bpc_pwr9(uint8_t *dst, const ptrdiff_t stride, \
                                                          int16_t *const coeff, const int eob \
                                                          HIGHBD_DECL_SUFFIX) \
{ \
    i16x8 v = vec_splats((int16_t)(2896*8)); \
    LOAD_SCALE_COEFF_4x8(coeff, v) \
    type1##_4_in(c0, c1, c2, c3, c01, c23) \
    type1##_4_in(c4, c5, c6, c7, c45, c67) \
    memset(coeff, 0, sizeof(*coeff) * 4 * 8); \
    TRANSPOSE4_I32(c0, c1, c2, c3); \
    TRANSPOSE4_I32(c4, c5, c6, c7); \
    type2##_8_out(c0, c1, c2, c3, c4, c5, c6, c7, c01, c23, c45, c67) \
    LOAD_DECLARE_4(dst, stride, a, b, c, d) \
    LOAD_DECLARE_4(dst + 4 * stride, stride, e, f, g, h) \
    APPLY_COEFF_4(a, b, c, d, c01, c23) \
    APPLY_COEFF_4(e, f, g, h, c45, c67) \
    STORE_4(dst, stride, a, b, c, d) \
    STORE_4(dst + 4 * stride, stride, e, f, g, h) \
}

inv_txfm_fn4x8(adst,     dct     )
inv_txfm_fn4x8(dct,      adst    )
inv_txfm_fn4x8(dct,      flipadst)
inv_txfm_fn4x8(flipadst, dct     )
inv_txfm_fn4x8(adst,     flipadst)
inv_txfm_fn4x8(flipadst, adst    )
inv_txfm_fn4x8(identity, dct     )
inv_txfm_fn4x8(dct,      identity)
inv_txfm_fn4x8(identity, flipadst)
inv_txfm_fn4x8(flipadst, identity)
inv_txfm_fn4x8(identity, adst   )
inv_txfm_fn4x8(adst,     identity)
inv_txfm_fn4x8(identity, identity)
inv_txfm_fn4x8(adst,     adst    )
inv_txfm_fn4x8(flipadst, flipadst)


void dav1d_inv_txfm_add_dct_dct_8x4_8bpc_pwr9(uint8_t *dst, const ptrdiff_t stride,
                                              int16_t *const coeff, const int eob
                                              HIGHBD_DECL_SUFFIX)
{
    i16x8 v = vec_splats((int16_t)(2896*8));

    if (eob < 1) {
        int dc = coeff[0];
        i16x8 vdc = vec_splats((int16_t)dc);
        coeff[0] = 0;
        vdc = vec_mradds(vdc, v, vec_splat_s16(0));
        vdc = vec_mradds(vdc, v, vec_splat_s16(0));
        vdc = vec_mradds(vdc, v, vec_splat_s16(8));
        vdc = vec_sra(vdc, vec_splat_u16(4));

        dc = vdc[0];

        LOAD_DECLARE_4(dst, stride, a, b, c, d)

        i16x8 as = u8h_to_i16(a);
        i16x8 bs = u8h_to_i16(b);
        i16x8 cs = u8h_to_i16(c);
        i16x8 ds = u8h_to_i16(d);

        as = vec_adds(as, vdc);
        bs = vec_adds(bs, vdc);
        cs = vec_adds(cs, vdc);
        ds = vec_adds(ds, vdc);

        a = vec_packsu(as, as);
        b = vec_packsu(bs, bs);
        c = vec_packsu(cs, cs);
        d = vec_packsu(ds, ds);

        STORE_8(dst, stride, a, b, c, d)

        return;
    }

    LOAD_SCALE_COEFF_8x4(coeff, v)

    dct_8_in(c0, c1, c2, c3, c4, c5, c6, c7, c01, c23, c45, c67)

    memset(coeff, 0, sizeof(*coeff) * 8 * 4);

    TRANSPOSE4_I32(c0, c1, c2, c3)
    TRANSPOSE4_I32(c4, c5, c6, c7)

    dct_4_out(c0, c1, c2, c3, c01, c23)
    dct_4_out(c4, c5, c6, c7, c45, c67)

    LOAD_DECLARE_4(dst, stride, ae, bf, cg, dh)

    i16x8 c04 = (i16x8)vec_mergeh((u64x2)c01, (u64x2)c45);
    i16x8 c15 = (i16x8)vec_mergel((u64x2)c01, (u64x2)c45);
    i16x8 c26 = (i16x8)vec_mergeh((u64x2)c23, (u64x2)c67);
    i16x8 c37 = (i16x8)vec_mergel((u64x2)c23, (u64x2)c67);

    APPLY_COEFF_8x4(ae, bf, c04, c15)
    APPLY_COEFF_8x4(cg, dh, c26, c37)

    STORE_8(dst, stride, ae, bf, cg, dh)
}


#define inv_txfm_fn8x4(type1, type2) \
void dav1d_inv_txfm_add_##type1##_##type2##_8x4_8bpc_pwr9(uint8_t *dst, const ptrdiff_t stride, \
                                                          int16_t *const coeff, const int eob \
                                                          HIGHBD_DECL_SUFFIX) \
{ \
    i16x8 v = vec_splats((int16_t)(2896*8)); \
    LOAD_SCALE_COEFF_8x4(coeff, v) \
    type1##_8_in(c0, c1, c2, c3, c4, c5, c6, c7, c01, c23, c45, c67) \
    memset(coeff, 0, sizeof(*coeff) * 8 * 4); \
    TRANSPOSE4_I32(c0, c1, c2, c3) \
    TRANSPOSE4_I32(c4, c5, c6, c7) \
    type2##_4_out(c0, c1, c2, c3, c01, c23) \
    type2##_4_out(c4, c5, c6, c7, c45, c67) \
    LOAD_DECLARE_4(dst, stride, ae, bf, cg, dh) \
    i16x8 c04 = (i16x8)vec_mergeh((u64x2)c01, (u64x2)c45); \
    i16x8 c15 = (i16x8)vec_mergel((u64x2)c01, (u64x2)c45); \
    i16x8 c26 = (i16x8)vec_mergeh((u64x2)c23, (u64x2)c67); \
    i16x8 c37 = (i16x8)vec_mergel((u64x2)c23, (u64x2)c67); \
    APPLY_COEFF_8x4(ae, bf, c04, c15) \
    APPLY_COEFF_8x4(cg, dh, c26, c37) \
    STORE_8(dst, stride, ae, bf, cg, dh) \
}
inv_txfm_fn8x4(adst,     dct     )
inv_txfm_fn8x4(dct,      adst    )
inv_txfm_fn8x4(dct,      flipadst)
inv_txfm_fn8x4(flipadst, dct     )
inv_txfm_fn8x4(adst,     flipadst)
inv_txfm_fn8x4(flipadst, adst    )
inv_txfm_fn8x4(identity, dct     )
inv_txfm_fn8x4(dct,      identity)
inv_txfm_fn8x4(identity, flipadst)
inv_txfm_fn8x4(flipadst, identity)
inv_txfm_fn8x4(identity, adst   )
inv_txfm_fn8x4(adst,     identity)
inv_txfm_fn8x4(identity, identity)
inv_txfm_fn8x4(adst,     adst    )
inv_txfm_fn8x4(flipadst, flipadst)

void dav1d_inv_txfm_add_dct_dct_8x8_8bpc_pwr9(uint8_t *dst, const ptrdiff_t stride,
                                              int16_t *const coeff, const int eob
                                              HIGHBD_DECL_SUFFIX)
{
    i16x8 v = vec_splats((int16_t)(2896*8));

    if (eob < 1) {
        int dc = coeff[0];
        i16x8 vdc = vec_splats((int16_t)dc);
        coeff[0] = 0;
        vdc = vec_mradds(vdc, v, vec_splat_s16(1));
        vdc = vec_sra(vdc, vec_splat_u16(1));
        vdc = vec_mradds(vdc, v, vec_splat_s16(8));
        vdc = vec_sra(vdc, vec_splat_u16(4));

        for (int i = 0; i < 2; i++, dst += 4 * stride) {
            LOAD_DECLARE_4(dst, stride, a, b, c, d)

            i16x8 as = u8h_to_i16(a);
            i16x8 bs = u8h_to_i16(b);
            i16x8 cs = u8h_to_i16(c);
            i16x8 ds = u8h_to_i16(d);

            as = vec_adds(as, vdc);
            bs = vec_adds(bs, vdc);
            cs = vec_adds(cs, vdc);
            ds = vec_adds(ds, vdc);

            a = vec_packsu(as, as);
            b = vec_packsu(bs, bs);
            c = vec_packsu(cs, cs);
            d = vec_packsu(ds, ds);

            STORE_8(dst, stride, a, b, c, d)
        }
        return;
    }

    LOAD_COEFF_8x8(coeff)

    dct_8x2_in(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h,
               c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l,
               c0, c1, c2, c3, c4, c5, c6, c7)

    memset(coeff, 0, sizeof(*coeff) * 8 * 8);

    SCALE_ROUND_4(c0h, c1h, c2h, c3h, vec_splat_s32(1), vec_splat_u32(1))
    SCALE_ROUND_4(c4h, c5h, c6h, c7h, vec_splat_s32(1), vec_splat_u32(1))
    SCALE_ROUND_4(c0l, c1l, c2l, c3l, vec_splat_s32(1), vec_splat_u32(1))
    SCALE_ROUND_4(c4l, c5l, c6l, c7l, vec_splat_s32(1), vec_splat_u32(1))

    TRANSPOSE8_I32(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h,
                   c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l)

    dct_8x2_out(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h,
                c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l,
                c0, c1, c2, c3, c4, c5, c6, c7)

    LOAD_DECLARE_4(dst, stride, a, b, cc, d)
    LOAD_DECLARE_4(dst + 4 * stride, stride, e, f, g, hh)

    APPLY_COEFF_8x4(a, b, c0, c1)
    APPLY_COEFF_8x4(cc, d, c2, c3)
    APPLY_COEFF_8x4(e, f, c4, c5)
    APPLY_COEFF_8x4(g, hh, c6, c7)

    STORE_8(dst, stride, a, b, cc, d)
    STORE_8(dst + 4 * stride, stride, e, f, g, hh)
}

#define inv_txfm_fn8x8(type1, type2) \
void dav1d_inv_txfm_add_##type1##_##type2##_8x8_8bpc_pwr9(uint8_t *dst, const ptrdiff_t stride, \
                                                          int16_t *const coeff, const int eob \
                                                          HIGHBD_DECL_SUFFIX) \
{ \
    LOAD_COEFF_8x8(coeff) \
    type1##_8x2_in(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h, \
                   c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l, \
                   c0, c1, c2, c3, c4, c5, c6, c7) \
    SCALE_ROUND_4(c0h, c1h, c2h, c3h, vec_splat_s32(1), vec_splat_u32(1)) \
    SCALE_ROUND_4(c4h, c5h, c6h, c7h, vec_splat_s32(1), vec_splat_u32(1)) \
    SCALE_ROUND_4(c0l, c1l, c2l, c3l, vec_splat_s32(1), vec_splat_u32(1)) \
    SCALE_ROUND_4(c4l, c5l, c6l, c7l, vec_splat_s32(1), vec_splat_u32(1)) \
    memset(coeff, 0, sizeof(*coeff) * 8 * 8); \
    TRANSPOSE8_I32(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h, \
                   c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l) \
    type2##_8x2_out(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h, \
                    c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l, \
                    c0, c1, c2, c3, c4, c5, c6, c7) \
    LOAD_DECLARE_4(dst, stride, a, b, c, d) \
    LOAD_DECLARE_4(dst + 4 * stride, stride, e, f, g, h) \
    APPLY_COEFF_8x4(a, b, c0, c1) \
    APPLY_COEFF_8x4(c, d, c2, c3) \
    APPLY_COEFF_8x4(e, f, c4, c5) \
    APPLY_COEFF_8x4(g, h, c6, c7) \
    STORE_8(dst, stride, a, b, c, d) \
    STORE_8(dst + 4 * stride, stride, e, f, g, h) \
}
inv_txfm_fn8x8(adst,     dct     )
inv_txfm_fn8x8(dct,      adst    )
inv_txfm_fn8x8(dct,      flipadst)
inv_txfm_fn8x8(flipadst, dct     )
inv_txfm_fn8x8(adst,     flipadst)
inv_txfm_fn8x8(flipadst, adst    )
inv_txfm_fn8x8(dct,      identity)
inv_txfm_fn8x8(flipadst, identity)
inv_txfm_fn8x8(adst,     identity)
inv_txfm_fn8x8(adst,     adst    )
inv_txfm_fn8x8(flipadst, flipadst)

// identity + scale is a no op
#define inv_txfm_fn8x8_identity(type2) \
void dav1d_inv_txfm_add_identity_##type2##_8x8_8bpc_pwr9(uint8_t *dst, const ptrdiff_t stride, \
                                                         int16_t *const coeff, const int eob \
                                                         HIGHBD_DECL_SUFFIX) \
{ \
    LOAD_COEFF_8x8(coeff) \
    memset(coeff, 0, sizeof(*coeff) * 8 * 8); \
    TRANSPOSE8_I32(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h, \
                   c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l) \
    type2##_8x2_out(c0h, c1h, c2h, c3h, c4h, c5h, c6h, c7h, \
                    c0l, c1l, c2l, c3l, c4l, c5l, c6l, c7l, \
                    c0, c1, c2, c3, c4, c5, c6, c7) \
    LOAD_DECLARE_4(dst, stride, a, b, c, d) \
    LOAD_DECLARE_4(dst + 4 * stride, stride, e, f, g, h) \
    APPLY_COEFF_8x4(a, b, c0, c1) \
    APPLY_COEFF_8x4(c, d, c2, c3) \
    APPLY_COEFF_8x4(e, f, c4, c5) \
    APPLY_COEFF_8x4(g, h, c6, c7) \
    STORE_8(dst, stride, a, b, c, d) \
    STORE_8(dst + 4 * stride, stride, e, f, g, h) \
}
inv_txfm_fn8x8_identity(dct     )
inv_txfm_fn8x8_identity(flipadst)
inv_txfm_fn8x8_identity(adst    )
inv_txfm_fn8x8_identity(identity)

#endif // BITDEPTH
