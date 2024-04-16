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

#define LOAD_COEFF_4(coeff) \
    i16x8 c01 = vec_xl(0, coeff); \
    i16x8 c23 = vec_xl(0, coeff + 8); \
    i32x4 c0 = i16h_to_i32(c01); \
    i32x4 c1 = i16l_to_i32(c01); \
    i32x4 c2 = i16h_to_i32(c23); \
    i32x4 c3 = i16l_to_i32(c23);

#define LOAD_DECLARE_4(src, stride, a, b, c, d) \
    u8x16 a, b, c, d; \
    LOAD_4(src, stride, a, b, c, d)

#define STORE_4(dst, stride, a, b, c, d) \
{ \
    uint8_t *dst2 = dst; \
    vec_xst_len(a, dst2, 4); \
    dst2 += stride; \
    vec_xst_len(b, dst2, 4); \
    dst2 += stride; \
    vec_xst_len(c, dst2, 4); \
    dst2 += stride; \
    vec_xst_len(d, dst2, 4); \
}

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

#define IDCT_4(c0, c1, c2, c3) \
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
    c01 = vec_packs(c0, c1); \
    c23 = vec_packs(c2, c3); \
 \
}

#define dct_4_in(c0, c1, c2, c3) \
{ \
    IDCT_4(c0, c1, c2, c3) \
    c0 = i16h_to_i32(c01); \
    c1 = i16l_to_i32(c01); \
    c2 = i16h_to_i32(c23); \
    c3 = i16l_to_i32(c23); \
}

#define dct_4_out(c0, c1, c2, c3) \
    IDCT_4(c0, c1, c2, c3)

#define IDENTITY_4(c0, c1, c2, c3) \
{ \
    i16x8 v1697 = vec_splats((int16_t)(1697*8)); \
    i16x8 o01 = vec_mradds(c01, v1697, vec_splat_s16(0)); \
    i16x8 o23 = vec_mradds(c23, v1697, vec_splat_s16(0)); \
    c01 = vec_adds(c01, o01); \
    c23 = vec_adds(c23, o23); \
}

#define identity_4_in(c0, c1, c2, c3) \
{ \
    IDENTITY_4(c0, c1, c2, c3) \
    c0 = i16h_to_i32(c01); \
    c1 = i16l_to_i32(c01); \
    c2 = i16h_to_i32(c23); \
    c3 = i16l_to_i32(c23); \
}

#define identity_4_out(c0, c1, c2, c3) \
{ \
    c01 = vec_packs(c0, c1); \
    c23 = vec_packs(c2, c3); \
    IDENTITY_4(c0, c1, c2, c3) \
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

#define adst_4_in(c0, c1, c2, c3) \
{ \
    ADST_INNER_4(c0, c1, c2, c3, c0, c1, c2, c3) \
}

#define flipadst_4_in(c0, c1, c2, c3) \
{ \
    ADST_INNER_4(c0, c1, c2, c3, c3, c2, c1, c0) \
}

#define adst_4_out(c0, c1, c2, c3) \
{ \
    ADST_INNER_4(c0, c1, c2, c3, c0, c1, c2, c3) \
    c01 = vec_packs(c0, c1); \
    c23 = vec_packs(c2, c3); \
}

#define flipadst_4_out(c0, c1, c2, c3) \
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

    dct_4_in(c0, c1, c2, c3)

    TRANSPOSE4_I32(c0, c1, c2, c3)

    memset(coeff, 0, sizeof(*coeff) * 4 * 4);

    dct_4_out(c0, c1, c2, c3)

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
    type1##_4_in(c0, c1, c2, c3) \
    memset(coeff, 0, sizeof(*coeff) * 4 * 4); \
    TRANSPOSE4_I32(c0, c1, c2, c3) \
    type2##_4_out(c0, c1, c2, c3) \
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

#endif // BITDEPTH
