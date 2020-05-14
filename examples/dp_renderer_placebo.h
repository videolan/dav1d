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

#include <libplacebo/renderer.h>
#include <libplacebo/utils/upload.h>

#if defined(HAVE_PLACEBO_VULKAN)
# include <libplacebo/vulkan.h>
# include <SDL_vulkan.h>
#elif defined(HAVE_PLACEBO_OPENGL)
# include <libplacebo/opengl.h>
# include <SDL_opengl.h>
#else
# error Placebo rendering only implemented for Vulkan or OpenGL!
#endif


/**
 * Renderer context for libplacebo
 */
typedef struct renderer_priv_ctx
{
    // Placebo context
    struct pl_context *ctx;
    // Placebo renderer
    struct pl_renderer *renderer;
#if defined(HAVE_PLACEBO_VULKAN)
    // Placebo Vulkan handle
    const struct pl_vulkan *vk;
    // Placebo Vulkan instance
    const struct pl_vk_inst *vk_inst;
    // Vulkan surface
    VkSurfaceKHR surf;
#elif defined(HAVE_PLACEBO_OPENGL)
    // Placebo OpenGL handle
    const struct pl_opengl *gl;
#endif
    // Placebo GPU
    const struct pl_gpu *gpu;
    // Placebo swapchain
    const struct pl_swapchain *swapchain;
    // Lock protecting access to the texture
    SDL_mutex *lock;
    // Planes to render
    struct pl_plane y_plane;
    struct pl_plane u_plane;
    struct pl_plane v_plane;
    // Textures to render
    const struct pl_tex *y_tex;
    const struct pl_tex *u_tex;
    const struct pl_tex *v_tex;
} Dav1dPlayRendererPrivateContext;

static void *placebo_renderer_create(void *data)
{
    fprintf(stderr, "Using placebo renderer\n");
    // Alloc
    Dav1dPlayRendererPrivateContext *rd_priv_ctx = malloc(sizeof(Dav1dPlayRendererPrivateContext));
    if (rd_priv_ctx == NULL) {
        return NULL;
    }

    // Init libplacebo
    rd_priv_ctx->ctx = pl_context_create(PL_API_VER, &(struct pl_context_params) {
        .log_cb     = pl_log_color,
#ifndef NDEBUG
        .log_level  = PL_LOG_DEBUG,
#else
        .log_level  = PL_LOG_WARN,
#endif
    });
    if (rd_priv_ctx->ctx == NULL) {
        free(rd_priv_ctx);
        return NULL;
    }

    // Create Mutex
    rd_priv_ctx->lock = SDL_CreateMutex();
    if (rd_priv_ctx->lock == NULL) {
        fprintf(stderr, "SDL_CreateMutex failed: %s\n", SDL_GetError());
        pl_context_destroy(&(rd_priv_ctx->ctx));
        free(rd_priv_ctx);
        return NULL;
    }


#if defined(HAVE_PLACEBO_VULKAN)
    // Init Vulkan
    struct pl_vk_inst_params iparams = pl_vk_inst_default_params;

    SDL_Window *sdlwin = data;

    unsigned num = 0;
    if (!SDL_Vulkan_GetInstanceExtensions(sdlwin, &num, NULL)) {
        fprintf(stderr, "Failed enumerating Vulkan extensions: %s\n", SDL_GetError());
        exit(1);
    }

    iparams.extensions = malloc(num * sizeof(const char *));
    iparams.num_extensions = num;
    assert(iparams.extensions);

    SDL_bool ok = SDL_Vulkan_GetInstanceExtensions(sdlwin, &num, iparams.extensions);
    if (!ok) {
        fprintf(stderr, "Failed getting Vk instance extensions\n");
        exit(1);
    }

    if (num > 0) {
        printf("Requesting %d additional Vulkan extensions:\n", num);
        for (unsigned i = 0; i < num; i++)
            printf("    %s\n", iparams.extensions[i]);
    }

    rd_priv_ctx->vk_inst = pl_vk_inst_create(rd_priv_ctx->ctx, &iparams);
    if (!rd_priv_ctx->vk_inst) {
        fprintf(stderr, "Failed creating Vulkan instance!\n");
        exit(1);
    }
    free(iparams.extensions);

    if (!SDL_Vulkan_CreateSurface(sdlwin, rd_priv_ctx->vk_inst->instance, &rd_priv_ctx->surf)) {
        fprintf(stderr, "Failed creating vulkan surface: %s\n", SDL_GetError());
        exit(1);
    }

    struct pl_vulkan_params params = pl_vulkan_default_params;
    params.instance = rd_priv_ctx->vk_inst->instance;
    params.surface = rd_priv_ctx->surf;
    params.allow_software = true;

    rd_priv_ctx->vk = pl_vulkan_create(rd_priv_ctx->ctx, &params);
    if (!rd_priv_ctx->vk) {
        fprintf(stderr, "Failed creating vulkan device!\n");
        exit(2);
    }

    // Create swapchain
    rd_priv_ctx->swapchain = pl_vulkan_create_swapchain(rd_priv_ctx->vk,
        &(struct pl_vulkan_swapchain_params) {
            .surface = rd_priv_ctx->surf,
            .present_mode = VK_PRESENT_MODE_IMMEDIATE_KHR,
        });

    if (!rd_priv_ctx->swapchain) {
        fprintf(stderr, "Failed creating vulkan swapchain!\n");
        exit(2);
    }

    int w = WINDOW_WIDTH, h = WINDOW_HEIGHT;
    if (!pl_swapchain_resize(rd_priv_ctx->swapchain, &w, &h)) {
        fprintf(stderr, "Failed resizing vulkan swapchain!\n");
        exit(2);
    }

    rd_priv_ctx->gpu = rd_priv_ctx->vk->gpu;
#elif defined(HAVE_PLACEBO_OPENGL)
    // Init OpenGL
    struct pl_opengl_params params = pl_opengl_default_params;
# ifndef NDEBUG
    params.debug = true;
# endif

    SDL_Window *sdlwin = data;
    SDL_GLContext glcontext = SDL_GL_CreateContext(sdlwin);
    SDL_GL_MakeCurrent(sdlwin, glcontext);

    rd_priv_ctx->gl = pl_opengl_create(rd_priv_ctx->ctx, &params);
    if (!rd_priv_ctx->gl) {
        fprintf(stderr, "Failed creating opengl device!\n");
        exit(2);
    }

    rd_priv_ctx->swapchain = pl_opengl_create_swapchain(rd_priv_ctx->gl,
        &(struct pl_opengl_swapchain_params) {
            .swap_buffers = (void (*)(void *)) SDL_GL_SwapWindow,
            .priv = sdlwin,
        });

    if (!rd_priv_ctx->swapchain) {
        fprintf(stderr, "Failed creating opengl swapchain!\n");
        exit(2);
    }

    int w = WINDOW_WIDTH, h = WINDOW_HEIGHT;
    SDL_GL_GetDrawableSize(sdlwin, &w, &h);

    if (!pl_swapchain_resize(rd_priv_ctx->swapchain, &w, &h)) {
        fprintf(stderr, "Failed resizing vulkan swapchain!\n");
        exit(2);
    }

    rd_priv_ctx->gpu = rd_priv_ctx->gl->gpu;
#endif

    if (w != WINDOW_WIDTH || h != WINDOW_HEIGHT)
        printf("Note: window dimensions differ (got %dx%d)\n", w, h);

    rd_priv_ctx->y_tex = NULL;
    rd_priv_ctx->u_tex = NULL;
    rd_priv_ctx->v_tex = NULL;

    rd_priv_ctx->renderer = NULL;

    return rd_priv_ctx;
}

static void placebo_renderer_destroy(void *cookie)
{
    Dav1dPlayRendererPrivateContext *rd_priv_ctx = cookie;
    assert(rd_priv_ctx != NULL);

    pl_renderer_destroy(&(rd_priv_ctx->renderer));
    pl_tex_destroy(rd_priv_ctx->gpu, &(rd_priv_ctx->y_tex));
    pl_tex_destroy(rd_priv_ctx->gpu, &(rd_priv_ctx->u_tex));
    pl_tex_destroy(rd_priv_ctx->gpu, &(rd_priv_ctx->v_tex));
    pl_swapchain_destroy(&(rd_priv_ctx->swapchain));

#if defined(HAVE_PLACEBO_VULKAN)
    pl_vulkan_destroy(&(rd_priv_ctx->vk));
    vkDestroySurfaceKHR(rd_priv_ctx->vk_inst->instance, rd_priv_ctx->surf, NULL);
    pl_vk_inst_destroy(&(rd_priv_ctx->vk_inst));
#elif defined(HAVE_PLACEBO_OPENGL)
    pl_opengl_destroy(&(rd_priv_ctx->gl));
#endif

    pl_context_destroy(&(rd_priv_ctx->ctx));
}

static void placebo_render(void *cookie, const Dav1dPlaySettings *settings)
{
    Dav1dPlayRendererPrivateContext *rd_priv_ctx = cookie;
    assert(rd_priv_ctx != NULL);

    SDL_LockMutex(rd_priv_ctx->lock);
    if (rd_priv_ctx->y_tex == NULL) {
        SDL_UnlockMutex(rd_priv_ctx->lock);
        return;
    }

    // Prepare rendering
    if (rd_priv_ctx->renderer == NULL) {
        rd_priv_ctx->renderer = pl_renderer_create(rd_priv_ctx->ctx, rd_priv_ctx->gpu);
    }

    struct pl_swapchain_frame frame;
    bool ok = pl_swapchain_start_frame(rd_priv_ctx->swapchain, &frame);
    if (!ok) {
        SDL_UnlockMutex(rd_priv_ctx->lock);
        return;
    }

    const struct pl_tex *img = rd_priv_ctx->y_plane.texture;
    struct pl_image image = {
        .num_planes = 3,
        .planes     = { rd_priv_ctx->y_plane, rd_priv_ctx->u_plane, rd_priv_ctx->v_plane },
        .repr       = pl_color_repr_hdtv,
        .color      = pl_color_space_unknown,
        .width      = img->params.w,
        .height     = img->params.h,
    };

    struct pl_render_params render_params = {0};
    if (settings->highquality)
        render_params = pl_render_default_params;

    struct pl_render_target target;
    pl_render_target_from_swapchain(&target, &frame);
    target.profile = (struct pl_icc_profile) {
        .data = NULL,
        .len = 0,
    };

    if (!pl_render_image(rd_priv_ctx->renderer, &image, &target, &render_params)) {
        fprintf(stderr, "Failed rendering frame!\n");
        SDL_UnlockMutex(rd_priv_ctx->lock);
        return;
    }

    ok = pl_swapchain_submit_frame(rd_priv_ctx->swapchain);
    if (!ok) {
        fprintf(stderr, "Failed submitting frame!\n");
        SDL_UnlockMutex(rd_priv_ctx->lock);
        return;
    }

    pl_swapchain_swap_buffers(rd_priv_ctx->swapchain);
    SDL_UnlockMutex(rd_priv_ctx->lock);
}

static int placebo_upload_planes(void *cookie, Dav1dPicture *dav1d_pic,
                                 const Dav1dPlaySettings *settings)
{
    Dav1dPlayRendererPrivateContext *rd_priv_ctx = cookie;
    assert(rd_priv_ctx != NULL);

    SDL_LockMutex(rd_priv_ctx->lock);

    if (dav1d_pic == NULL) {
        SDL_UnlockMutex(rd_priv_ctx->lock);
        return 0;
    }

    int width = dav1d_pic->p.w;
    int height = dav1d_pic->p.h;

    enum Dav1dPixelLayout dav1d_layout = dav1d_pic->p.layout;

    if (DAV1D_PIXEL_LAYOUT_I420 != dav1d_layout || dav1d_pic->p.bpc != 8) {
        fprintf(stderr, "Unsupported pixel format, only 8bit 420 supported so far.\n");
        exit(50);
    }

    struct pl_plane_data data_y = {
        .type           = PL_FMT_UNORM,
        .width          = width,
        .height         = height,
        .pixel_stride   = 1,
        .row_stride     = dav1d_pic->stride[0],
        .component_size = {8},
        .component_map  = {0},
    };

    struct pl_plane_data data_u = {
        .type           = PL_FMT_UNORM,
        .width          = width/2,
        .height         = height/2,
        .pixel_stride   = 1,
        .row_stride     = dav1d_pic->stride[1],
        .component_size = {8},
        .component_map  = {1},
    };

    struct pl_plane_data data_v = {
        .type           = PL_FMT_UNORM,
        .width          = width/2,
        .height         = height/2,
        .pixel_stride   = 1,
        .row_stride     = dav1d_pic->stride[1],
        .component_size = {8},
        .component_map  = {2},
    };

    if (settings->zerocopy) {
        const struct pl_buf *buf = dav1d_pic->allocator_data;
        assert(buf);
        data_y.buf = data_u.buf = data_v.buf = buf;
        data_y.buf_offset = (uintptr_t) dav1d_pic->data[0] - (uintptr_t) buf->data;
        data_u.buf_offset = (uintptr_t) dav1d_pic->data[1] - (uintptr_t) buf->data;
        data_v.buf_offset = (uintptr_t) dav1d_pic->data[2] - (uintptr_t) buf->data;
    } else {
        data_y.pixels = dav1d_pic->data[0];
        data_u.pixels = dav1d_pic->data[1];
        data_v.pixels = dav1d_pic->data[2];
    }

    bool ok = true;
    ok &= pl_upload_plane(rd_priv_ctx->gpu, &(rd_priv_ctx->y_plane), &(rd_priv_ctx->y_tex), &data_y);
    ok &= pl_upload_plane(rd_priv_ctx->gpu, &(rd_priv_ctx->u_plane), &(rd_priv_ctx->u_tex), &data_u);
    ok &= pl_upload_plane(rd_priv_ctx->gpu, &(rd_priv_ctx->v_plane), &(rd_priv_ctx->v_tex), &data_v);

    pl_chroma_location_offset(PL_CHROMA_LEFT, &rd_priv_ctx->u_plane.shift_x, &rd_priv_ctx->u_plane.shift_y);
    pl_chroma_location_offset(PL_CHROMA_LEFT, &rd_priv_ctx->v_plane.shift_x, &rd_priv_ctx->v_plane.shift_y);

    if (!ok) {
        fprintf(stderr, "Failed uploading planes!\n");
    }

    SDL_UnlockMutex(rd_priv_ctx->lock);
    return !ok;
}

// Align to power of 2
#define ALIGN2(x, align) (((x) + (align) - 1) & ~((align) - 1))

static int placebo_alloc_pic(Dav1dPicture *const p, void *cookie)
{
    Dav1dPlayRendererPrivateContext *rd_priv_ctx = cookie;
    assert(rd_priv_ctx != NULL);
    SDL_LockMutex(rd_priv_ctx->lock);

    const struct pl_gpu *gpu = rd_priv_ctx->gpu;
    int ret = DAV1D_ERR(ENOMEM);

    // Copied from dav1d_default_picture_alloc
    const int hbd = p->p.bpc > 8;
    const int aligned_w = ALIGN2(p->p.w, 128);
    const int aligned_h = ALIGN2(p->p.h, 128);
    const int has_chroma = p->p.layout != DAV1D_PIXEL_LAYOUT_I400;
    const int ss_ver = p->p.layout == DAV1D_PIXEL_LAYOUT_I420;
    const int ss_hor = p->p.layout != DAV1D_PIXEL_LAYOUT_I444;
    p->stride[0] = aligned_w << hbd;
    p->stride[1] = has_chroma ? (aligned_w >> ss_hor) << hbd : 0;

    // Align strides up to multiples of the GPU performance hints
    p->stride[0] = ALIGN2(p->stride[0], gpu->limits.align_tex_xfer_stride);
    p->stride[1] = ALIGN2(p->stride[1], gpu->limits.align_tex_xfer_stride);

    // Aligning offsets to 4 also implicity aligns to the texel size (1 or 2)
    size_t off_align = ALIGN2(gpu->limits.align_tex_xfer_offset, 4);
    const size_t y_sz = ALIGN2(p->stride[0] * aligned_h, off_align);
    const size_t uv_sz = ALIGN2(p->stride[1] * (aligned_h >> ss_ver), off_align);

    // The extra DAV1D_PICTURE_ALIGNMENTs are to brute force plane alignment,
    // even in the case that the driver gives us insane alignments
    const size_t pic_size = y_sz + 2 * uv_sz;
    const size_t total_size = pic_size + DAV1D_PICTURE_ALIGNMENT * 4;

    // Validate size limitations
    if (total_size > gpu->limits.max_xfer_size) {
        printf("alloc of %zu bytes exceeds limits\n", total_size);
        goto err;
    }

    const struct pl_buf *buf = pl_buf_create(gpu, &(struct pl_buf_params) {
        .type = PL_BUF_TEX_TRANSFER,
        .host_mapped = true,
        .size = total_size,
        .memory_type = PL_BUF_MEM_HOST,
        .user_data = p,
    });

    if (!buf) {
        printf("alloc of GPU mapped buffer failed\n");
        goto err;
    }

    assert(buf->data);
    uintptr_t base = (uintptr_t) buf->data, data[3];
    data[0] = ALIGN2(base, DAV1D_PICTURE_ALIGNMENT);
    data[1] = ALIGN2(data[0] + y_sz, DAV1D_PICTURE_ALIGNMENT);
    data[2] = ALIGN2(data[1] + uv_sz, DAV1D_PICTURE_ALIGNMENT);

    // Sanity check offset alignment for the sake of debugging
    if (data[0] - base != ALIGN2(data[0] - base, off_align) ||
        data[1] - base != ALIGN2(data[1] - base, off_align) ||
        data[2] - base != ALIGN2(data[2] - base, off_align))
    {
        printf("GPU buffer horribly misaligned, expect slowdown!\n");
    }

    p->allocator_data = (void *) buf;
    p->data[0] = (void *) data[0];
    p->data[1] = (void *) data[1];
    p->data[2] = (void *) data[2];
    ret = 0;

    // fall through
err:
    SDL_UnlockMutex(rd_priv_ctx->lock);
    return ret;
}

static void placebo_release_pic(Dav1dPicture *pic, void *cookie)
{
    Dav1dPlayRendererPrivateContext *rd_priv_ctx = cookie;
    assert(rd_priv_ctx != NULL);
    assert(pic->allocator_data);

    SDL_LockMutex(rd_priv_ctx->lock);
    const struct pl_gpu *gpu = rd_priv_ctx->gpu;
    pl_buf_destroy(gpu, (const struct pl_buf **) &pic->allocator_data);
    SDL_UnlockMutex(rd_priv_ctx->lock);
}

static const Dav1dPlayRenderInfo renderer_info = {
    .create_renderer = placebo_renderer_create,
    .destroy_renderer = placebo_renderer_destroy,
    .render = placebo_render,
    .update_frame = placebo_upload_planes,
    .alloc_pic = placebo_alloc_pic,
    .release_pic = placebo_release_pic,
};
