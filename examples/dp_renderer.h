/*
 * Copyright © 2020, VideoLAN and dav1d authors
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

/**
 * Renderer info
 */
typedef struct rdr_info
{
    // Cookie passed to the renderer implementation callbacks
    void *cookie;
    // Callback to create the renderer
    void* (*create_renderer)(void *data);
    // Callback to destroy the renderer
    void (*destroy_renderer)(void *cookie);
    // Callback to the render function that renders a prevously sent frame
    void (*render)(void *cookie, const Dav1dPlaySettings *settings);
    // Callback to the send frame function
    int (*update_frame)(void *cookie, Dav1dPicture *dav1d_pic,
                        const Dav1dPlaySettings *settings);
    // Callback for alloc/release pictures (optional)
    int (*alloc_pic)(Dav1dPicture *pic, void *cookie);
    void (*release_pic)(Dav1dPicture *pic, void *cookie);
} Dav1dPlayRenderInfo;

#ifdef HAVE_RENDERER_PLACEBO
# include "dp_renderer_placebo.h"
#else
# include "dp_renderer_sdl.h"
#endif
