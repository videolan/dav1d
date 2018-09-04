/*
 * Copyright © 2018, VideoLAN and dav1d authors
 * Copyright © 2018, Two Orioles, LLC
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

#ifndef __DAV1D_COMMON_ATTRIBUTES_H__
#define __DAV1D_COMMON_ATTRIBUTES_H__

#include <stddef.h>

/*
 * API for variables, struct members (ALIGN()) like:
 * uint8_t var[1][2][3][4]
 * becomes:
 * ALIGN(uint8_t var[1][2][3][4], alignment).
 */
#define ALIGN(line, align) \
    line __attribute__((aligned(align)))

/*
 * API for stack alignment (ALIGN_STK_$align()) of variables like:
 * uint8_t var[1][2][3][4]
 * becomes:
 * ALIGN_STK_$align(uint8_t, var, 1, [2][3][4])
 */
#define ALIGN_STK_32(type, var, sz1d, sznd) \
    ALIGN(type var[sz1d]sznd, 32)
// as long as stack is itself 16-byte aligned, this works (win64, gcc)
#define ALIGN_STK_16(type, var, sz1d, sznd) \
    ALIGN(type var[sz1d]sznd, 16)

#endif /* __DAV1D_COMMON_ATTRIBUTES_H__ */
