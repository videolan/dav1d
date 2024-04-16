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

#ifndef DAV1D_SRC_PPC_UTILS_H
#define DAV1D_SRC_PPC_UTILS_H

#include "src/ppc/dav1d_types.h"

#define assert_eq(a, b) \
    if ((a) != (b)) \
        printf("%d vs %d\n", a, b); \
    assert((a) == (b));

// Transpose a 4x4 matrix of i32x4 vectors
#define TRANSPOSE4_I32(c0, c1, c2, c3) \
{ \
    i32x4 v02h = vec_mergeh(c0, c2); \
    i32x4 v02l = vec_mergel(c0, c2); \
    i32x4 v13h = vec_mergeh(c1, c3); \
    i32x4 v13l = vec_mergel(c1, c3); \
\
    c0 = vec_mergeh(v02h, v13h); \
    c1 = vec_mergel(v02h, v13h); \
    c2 = vec_mergeh(v02l, v13l); \
    c3 = vec_mergel(v02l, v13l); \
}


#endif // DAV1D_SRC_PPC_UTILS_H
