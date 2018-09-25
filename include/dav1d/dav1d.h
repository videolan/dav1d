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

#ifndef __DAV1D_H__
#define __DAV1D_H__

#include "common.h"
#include "picture.h"
#include "data.h"

typedef struct Dav1dContext Dav1dContext;
typedef struct Dav1dRef Dav1dRef;

typedef struct Dav1dSettings {
    int n_frame_threads;
    int n_tile_threads;
} Dav1dSettings;

/*
 * Init the library.
 */
DAV1D_API void dav1d_init(void);

/**
 * Get library version.
 */
DAV1D_API const char *dav1d_version(void);

/**
 * Initialize settings to default values.
 */
DAV1D_API void dav1d_default_settings(Dav1dSettings *s);

/**
 * Open/allocate decoder instance.
 *
 * The resulting instance context will be placed in $c_out and can be used in
 * iterative calls to dav1d_decode().
 *
 * You should free the context using dav1d_close() when you're done decoding.
 *
 * This returns < 0 (a negative errno code) on error, or 0 on success.
 */
DAV1D_API int dav1d_open(Dav1dContext **c_out, const Dav1dSettings *s);

/**
 * Decode one input frame. Library takes ownership of the passed-in reference.
 * After that, it will return < 0 (a negative errno code, but not -EAGAIN) on
 * failure, or 0 on success. If any decoded output frames are available, they
 * will be placed in $out. The caller assumes ownership of the returned output
 * picture.
 *
 * To flush the decoder (i.e. all input is finished), feed it NULL input data
 * until it returns -EAGAIN.
 */
DAV1D_API int dav1d_decode(Dav1dContext *c, Dav1dData *in, Dav1dPicture *out);

/**
 * Close decoder instance, free all associated memory, and set $c_out to NULL.
 */
DAV1D_API void dav1d_close(Dav1dContext **c_out);

#endif /* __DAV1D_H__ */
