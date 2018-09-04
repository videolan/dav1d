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

#include "config.h"

#include <stdint.h>

#include "src/levels.h"
#include "src/tables.h"

const uint8_t av1_al_part_ctx[2][N_BL_LEVELS][N_PARTITIONS] = {
    {
        // partitions:
        // none,  h,    v, split,  tts,  tbs,  tls,  trs,   h4,   v4
        { 0x00, 0x00, 0x10,   -1, 0x00, 0x10, 0x10, 0x10,   -1,   -1 }, // bl128
        { 0x10, 0x10, 0x18,   -1, 0x10, 0x18, 0x18, 0x18, 0x10, 0x1c }, // bl64
        { 0x18, 0x18, 0x1c,   -1, 0x18, 0x1c, 0x1c, 0x1c, 0x18, 0x1e }, // bl32
        { 0x1c, 0x1c, 0x1e,   -1, 0x1c, 0x1e, 0x1e, 0x1e, 0x1c, 0x1f }, // bl16
        { 0x1e, 0x1e, 0x1f, 0x1f,   -1,   -1,   -1,   -1,   -1,   -1 }, // bl8
    }, {
        { 0x00, 0x10, 0x00,   -1, 0x10, 0x10, 0x00, 0x10,   -1,   -1 }, // bl128
        { 0x10, 0x18, 0x10,   -1, 0x18, 0x18, 0x10, 0x18, 0x1c, 0x10 }, // bl64
        { 0x18, 0x1c, 0x18,   -1, 0x1c, 0x1c, 0x18, 0x1c, 0x1e, 0x18 }, // bl32
        { 0x1c, 0x1e, 0x1c,   -1, 0x1e, 0x1e, 0x1c, 0x1e, 0x1f, 0x1c }, // bl16
        { 0x1e, 0x1f, 0x1e, 0x1f,   -1,   -1,   -1,   -1,   -1,   -1 }, // bl8
    }
};

const uint8_t /* enum BlockSize */
    av1_block_sizes[N_BL_LEVELS][N_PARTITIONS][2] =
{
    [BL_128X128] = {
        [PARTITION_NONE]           = { BS_128x128 },
        [PARTITION_H]              = { BS_128x64 },
        [PARTITION_V]              = { BS_64x128 },
        [PARTITION_T_TOP_SPLIT]    = { BS_64x64, BS_128x64 },
        [PARTITION_T_BOTTOM_SPLIT] = { BS_128x64, BS_64x64 },
        [PARTITION_T_LEFT_SPLIT]   = { BS_64x64, BS_64x128 },
        [PARTITION_T_RIGHT_SPLIT]  = { BS_64x128, BS_64x64 },
    }, [BL_64X64] = {
        [PARTITION_NONE]           = { BS_64x64 },
        [PARTITION_H]              = { BS_64x32 },
        [PARTITION_V]              = { BS_32x64 },
        [PARTITION_T_TOP_SPLIT]    = { BS_32x32, BS_64x32 },
        [PARTITION_T_BOTTOM_SPLIT] = { BS_64x32, BS_32x32 },
        [PARTITION_T_LEFT_SPLIT]   = { BS_32x32, BS_32x64 },
        [PARTITION_T_RIGHT_SPLIT]  = { BS_32x64, BS_32x32 },
        [PARTITION_H4]             = { BS_64x16 },
        [PARTITION_V4]             = { BS_16x64 },
    }, [BL_32X32] = {
        [PARTITION_NONE]           = { BS_32x32 },
        [PARTITION_H]              = { BS_32x16 },
        [PARTITION_V]              = { BS_16x32 },
        [PARTITION_T_TOP_SPLIT]    = { BS_16x16, BS_32x16 },
        [PARTITION_T_BOTTOM_SPLIT] = { BS_32x16, BS_16x16 },
        [PARTITION_T_LEFT_SPLIT]   = { BS_16x16, BS_16x32 },
        [PARTITION_T_RIGHT_SPLIT]  = { BS_16x32, BS_16x16 },
        [PARTITION_H4]             = { BS_32x8  },
        [PARTITION_V4]             = { BS_8x32  },
    }, [BL_16X16] = {
        [PARTITION_NONE]           = { BS_16x16 },
        [PARTITION_H]              = { BS_16x8  },
        [PARTITION_V]              = { BS_8x16  },
        [PARTITION_T_TOP_SPLIT]    = { BS_8x8,   BS_16x8  },
        [PARTITION_T_BOTTOM_SPLIT] = { BS_16x8,  BS_8x8   },
        [PARTITION_T_LEFT_SPLIT]   = { BS_8x8,   BS_8x16  },
        [PARTITION_T_RIGHT_SPLIT]  = { BS_8x16,  BS_8x8   },
        [PARTITION_H4]             = { BS_16x4  },
        [PARTITION_V4]             = { BS_4x16  },
    }, [BL_8X8] = {
        [PARTITION_NONE]           = { BS_8x8   },
        [PARTITION_H]              = { BS_8x4   },
        [PARTITION_V]              = { BS_4x8   },
        [PARTITION_SPLIT]          = { BS_4x4   },
    }
};

const uint8_t av1_block_dimensions[N_BS_SIZES][4] = {
    [BS_128x128] = { 32, 32, 5, 5 },
    [BS_128x64]  = { 32, 16, 5, 4 },
    [BS_64x128]  = { 16, 32, 4, 5 },
    [BS_64x64]   = { 16, 16, 4, 4 },
    [BS_64x32]   = { 16,  8, 4, 3 },
    [BS_64x16]   = { 16,  4, 4, 2 },
    [BS_32x64]   = {  8, 16, 3, 4 },
    [BS_32x32]   = {  8,  8, 3, 3 },
    [BS_32x16]   = {  8,  4, 3, 2 },
    [BS_32x8]    = {  8,  2, 3, 1 },
    [BS_16x64]   = {  4, 16, 2, 4 },
    [BS_16x32]   = {  4,  8, 2, 3 },
    [BS_16x16]   = {  4,  4, 2, 2 },
    [BS_16x8]    = {  4,  2, 2, 1 },
    [BS_16x4]    = {  4,  1, 2, 0 },
    [BS_8x32]    = {  2,  8, 1, 3 },
    [BS_8x16]    = {  2,  4, 1, 2 },
    [BS_8x8]     = {  2,  2, 1, 1 },
    [BS_8x4]     = {  2,  1, 1, 0 },
    [BS_4x16]    = {  1,  4, 0, 2 },
    [BS_4x8]     = {  1,  2, 0, 1 },
    [BS_4x4]     = {  1,  1, 0, 0 },
};

const TxfmInfo av1_txfm_dimensions[N_RECT_TX_SIZES] = {
    [ TX_4X4]   = { .w = 1, .h = 1, .lw = 0, .lh = 0,
                    .min = 0, .max = 0, .ctx = 0 },
    [ TX_8X8]   = { .w = 2, .h = 2, .lw = 1, .lh = 1,
                    .min = 1, .max = 1, .sub = TX_4X4, .ctx = 1 },
    [ TX_16X16] = { .w = 4, .h = 4, .lw = 2, .lh = 2,
                    .min = 2, .max = 2, .sub = TX_8X8, .ctx = 2 },
    [ TX_32X32] = { .w = 8, .h = 8, .lw = 3, .lh = 3,
                    .min = 3, .max = 3, .sub = TX_16X16, .ctx = 3 },
    [ TX_64X64] = { .w = 16, .h = 16, .lw = 4, .lh = 4,
                    .min = 4, .max = 4, .sub = TX_32X32, .ctx = 4 },
    [RTX_4X8]   = { .w = 1, .h = 2, .lw = 0, .lh = 1,
                    .min = 0, .max = 1, .sub = TX_4X4, .ctx = 1 },
    [RTX_8X4]   = { .w = 2, .h = 1, .lw = 1, .lh = 0,
                    .min = 0, .max = 1, .sub = TX_4X4, .ctx = 1 },
    [RTX_8X16]  = { .w = 2, .h = 4, .lw = 1, .lh = 2,
                    .min = 1, .max = 2, .sub = TX_8X8, .ctx = 2 },
    [RTX_16X8]  = { .w = 4, .h = 2, .lw = 2, .lh = 1,
                    .min = 1, .max = 2, .sub = TX_8X8, .ctx = 2 },
    [RTX_16X32] = { .w = 4, .h = 8, .lw = 2, .lh = 3,
                    .min = 2, .max = 3, .sub = TX_16X16, .ctx = 3 },
    [RTX_32X16] = { .w = 8, .h = 4, .lw = 3, .lh = 2,
                    .min = 2, .max = 3, .sub = TX_16X16, .ctx = 3 },
    [RTX_32X64] = { .w = 8, .h = 16, .lw = 3, .lh = 4,
                    .min = 3, .max = 4, .sub = TX_32X32, .ctx = 4 },
    [RTX_64X32] = { .w = 16, .h = 8, .lw = 4, .lh = 3,
                    .min = 3, .max = 4, .sub = TX_32X32, .ctx = 4 },
    [RTX_4X16]  = { .w = 1, .h = 4, .lw = 0, .lh = 2,
                    .min = 0, .max = 2, .sub = RTX_4X8, .ctx = 1 },
    [RTX_16X4]  = { .w = 4, .h = 1, .lw = 2, .lh = 0,
                    .min = 0, .max = 2, .sub = RTX_8X4, .ctx = 1 },
    [RTX_8X32]  = { .w = 2, .h = 8, .lw = 1, .lh = 3,
                    .min = 1, .max = 3, .sub = RTX_8X16, .ctx = 2 },
    [RTX_32X8]  = { .w = 8, .h = 2, .lw = 3, .lh = 1,
                    .min = 1, .max = 3, .sub = RTX_16X8, .ctx = 2 },
    [RTX_16X64] = { .w = 4, .h = 16, .lw = 2, .lh = 4,
                    .min = 2, .max = 4, .sub = RTX_16X32, .ctx = 3 },
    [RTX_64X16] = { .w = 16, .h = 4, .lw = 4, .lh = 2,
                    .min = 2, .max = 4, .sub = RTX_32X16, .ctx = 3 },
};

const uint8_t /* enum (Rect)TxfmSize */
    av1_max_txfm_size_for_bs[N_BS_SIZES][4 /* y, 420, 422, 444 */] =
{
    [BS_128x128] = {  TX_64X64,  TX_32X32,  TX_32X32,  TX_32X32 },
    [BS_128x64]  = {  TX_64X64,  TX_32X32,  TX_32X32,  TX_32X32 },
    [BS_64x128]  = {  TX_64X64,  TX_32X32,       0,    TX_32X32 },
    [BS_64x64]   = {  TX_64X64,  TX_32X32,  TX_32X32,  TX_32X32 },
    [BS_64x32]   = { RTX_64X32, RTX_32X16,  TX_32X32,  TX_32X32 },
    [BS_64x16]   = { RTX_64X16, RTX_32X8,  RTX_32X16, RTX_32X16 },
    [BS_32x64]   = { RTX_32X64, RTX_16X32,       0,    TX_32X32 },
    [BS_32x32]   = {  TX_32X32,  TX_16X16, RTX_16X32,  TX_32X32 },
    [BS_32x16]   = { RTX_32X16, RTX_16X8,   TX_16X16, RTX_32X16 },
    [BS_32x8]    = { RTX_32X8,  RTX_16X4,  RTX_16X8,  RTX_32X8  },
    [BS_16x64]   = { RTX_16X64, RTX_8X32,        0,   RTX_16X32 },
    [BS_16x32]   = { RTX_16X32, RTX_8X16,        0,   RTX_16X32 },
    [BS_16x16]   = {  TX_16X16,  TX_8X8,   RTX_8X16,   TX_16X16 },
    [BS_16x8]    = { RTX_16X8,  RTX_8X4,    TX_8X8,   RTX_16X8  },
    [BS_16x4]    = { RTX_16X4,  RTX_8X4,   RTX_8X4,   RTX_16X4  },
    [BS_8x32]    = { RTX_8X32,  RTX_4X16,       0,    RTX_8X32  },
    [BS_8x16]    = { RTX_8X16,  RTX_4X8,        0,    RTX_8X16  },
    [BS_8x8]     = {  TX_8X8,    TX_4X4,   RTX_4X8,    TX_8X8   },
    [BS_8x4]     = { RTX_8X4,    TX_4X4,    TX_4X4,   RTX_8X4   },
    [BS_4x16]    = { RTX_4X16,  RTX_4X8,        0,    RTX_4X16  },
    [BS_4x8]     = { RTX_4X8,    TX_4X4,        0,    RTX_4X8   },
    [BS_4x4]     = {  TX_4X4,    TX_4X4,    TX_4X4,    TX_4X4   },
};

const uint8_t /* enum TxfmType */
    av1_txtp_from_uvmode[N_UV_INTRA_PRED_MODES] =
{
    [DC_PRED]              = DCT_DCT,
    [VERT_PRED]            = ADST_DCT,
    [HOR_PRED]             = DCT_ADST,
    [DIAG_DOWN_LEFT_PRED]  = DCT_DCT,
    [DIAG_DOWN_RIGHT_PRED] = ADST_ADST,
    [VERT_RIGHT_PRED]      = ADST_DCT,
    [HOR_DOWN_PRED]        = DCT_ADST,
    [HOR_UP_PRED]          = DCT_ADST,
    [VERT_LEFT_PRED]       = ADST_DCT,
    [SMOOTH_PRED]          = ADST_ADST,
    [SMOOTH_V_PRED]        = ADST_DCT,
    [SMOOTH_H_PRED]        = DCT_ADST,
    [PAETH_PRED]           = ADST_ADST,
};

const uint8_t /* enum InterPredMode */
    av1_comp_inter_pred_modes[N_COMP_INTER_PRED_MODES][2] =
{
    [NEARESTMV_NEARESTMV] = { NEARESTMV, NEARESTMV },
    [NEARMV_NEARMV]       = { NEARMV,    NEARMV    },
    [NEWMV_NEWMV]         = { NEWMV,     NEWMV     },
    [GLOBALMV_GLOBALMV]   = { GLOBALMV,  GLOBALMV  },
    [NEWMV_NEARESTMV]     = { NEWMV,     NEARESTMV },
    [NEWMV_NEARMV]        = { NEWMV,     NEARMV    },
    [NEARESTMV_NEWMV]     = { NEARESTMV, NEWMV     },
    [NEARMV_NEWMV]        = { NEARMV,    NEWMV     },
};

const uint8_t av1_tx_type_count[N_TXTP_SETS] = {
    [TXTP_SET_DCT] = 1,
    [TXTP_SET_DCT_ID] = 2,
    [TXTP_SET_DT4_ID] = 5,
    [TXTP_SET_DT4_ID_1D] = 7,
    [TXTP_SET_DT9_ID_1D] = 12,
    [TXTP_SET_ALL] = 16,
    [TXTP_SET_LOSSLESS] = 1,
};

const uint8_t /* enum TxfmType */
              av1_tx_types_per_set[N_TXTP_SETS][N_TX_TYPES] =
{
    [TXTP_SET_DCT]       = { DCT_DCT },
    [TXTP_SET_DCT_ID]    = { IDTX, DCT_DCT },
    [TXTP_SET_DT4_ID]    = { IDTX, DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST },
    [TXTP_SET_DT4_ID_1D] = { IDTX, DCT_DCT, V_DCT, H_DCT, ADST_ADST, ADST_DCT,
                             DCT_ADST },
    [TXTP_SET_DT9_ID_1D] = { IDTX, V_DCT, H_DCT, DCT_DCT, ADST_DCT, DCT_ADST,
                             FLIPADST_DCT, DCT_FLIPADST, ADST_ADST,
                             FLIPADST_FLIPADST, ADST_FLIPADST, FLIPADST_ADST },
    [TXTP_SET_ALL]       = { IDTX, V_DCT, H_DCT, V_ADST, H_ADST, V_FLIPADST,
                             H_FLIPADST, DCT_DCT, ADST_DCT, DCT_ADST,
                             FLIPADST_DCT, DCT_FLIPADST, ADST_ADST,
                             FLIPADST_FLIPADST, ADST_FLIPADST, FLIPADST_ADST },
    [TXTP_SET_LOSSLESS]  = { WHT_WHT },
};

const uint8_t av1_tx_type_set_index[2][N_TXTP_SETS] = {
    { 0, -1,  2,  1, -1, -1, 3 },
    { 0,  3, -1, -1,  2,  1, 4 },
};

const uint8_t av1_ymode_size_context[N_BS_SIZES] = {
    [BS_128x128] = 3,
    [BS_128x64]  = 3,
    [BS_64x128]  = 3,
    [BS_64x64]   = 3,
    [BS_64x32]   = 3,
    [BS_64x16]   = 2,
    [BS_32x64]   = 3,
    [BS_32x32]   = 3,
    [BS_32x16]   = 2,
    [BS_32x8 ]   = 1,
    [BS_16x64]   = 2,
    [BS_16x32]   = 2,
    [BS_16x16]   = 2,
    [BS_16x8 ]   = 1,
    [BS_16x4 ]   = 0,
    [BS_8x32 ]   = 1,
    [BS_8x16 ]   = 1,
    [BS_8x8  ]   = 1,
    [BS_8x4  ]   = 0,
    [BS_4x16 ]   = 0,
    [BS_4x8  ]   = 0,
    [BS_4x4  ]   = 0,
};

const uint8_t av1_nz_map_ctx_offset[N_RECT_TX_SIZES][5][5] = {
    [TX_4X4] = {
        { 0, 1, 6, 6 },
        { 1, 6, 6, 21 },
        { 6, 6, 21, 21 },
        { 6, 21, 21, 21 },
    }, [TX_8X8] = {
        { 0, 1, 6, 6, 21 },
        { 1, 6, 6, 21, 21 },
        { 6, 6, 21, 21, 21 },
        { 6, 21, 21, 21, 21 },
        { 21, 21, 21, 21, 21 }
    }, [TX_16X16] = {
        { 0, 1, 6, 6, 21 },
        { 1, 6, 6, 21, 21 },
        { 6, 6, 21, 21, 21 },
        { 6, 21, 21, 21, 21 },
        { 21, 21, 21, 21, 21 }
    }, [TX_32X32] = {
        { 0, 1, 6, 6, 21 },
        { 1, 6, 6, 21, 21 },
        { 6, 6, 21, 21, 21 },
        { 6, 21, 21, 21, 21 },
        { 21, 21, 21, 21, 21 }
    }, [TX_64X64] = {
        { 0, 1, 6, 6, 21 },
        { 1, 6, 6, 21, 21 },
        { 6, 6, 21, 21, 21 },
        { 6, 21, 21, 21, 21 },
        { 21, 21, 21, 21, 21 }
    }, [RTX_4X8] = {
        { 0, 11, 11, 11 },
        { 11, 11, 11, 11 },
        { 6, 6, 21, 21 },
        { 6, 21, 21, 21 },
        { 21, 21, 21, 21 }
    }, [RTX_8X4] = {
        { 0, 16, 6, 6, 21 },
        { 16, 16, 6, 21, 21 },
        { 16, 16, 21, 21, 21 },
        { 16, 16, 21, 21, 21 },
    }, [RTX_8X16] = {
        { 0, 11, 11, 11, 11 },
        { 11, 11, 11, 11, 11 },
        { 6, 6, 21, 21, 21 },
        { 6, 21, 21, 21, 21 },
        { 21, 21, 21, 21, 21 }
    }, [RTX_16X8] = {
        { 0, 16, 6, 6, 21 },
        { 16, 16, 6, 21, 21 },
        { 16, 16, 21, 21, 21 },
        { 16, 16, 21, 21, 21 },
        { 16, 16, 21, 21, 21 }
    }, [RTX_16X32] = {
        { 0, 11, 11, 11, 11 },
        { 11, 11, 11, 11, 11 },
        { 6, 6, 21, 21, 21 },
        { 6, 21, 21, 21, 21 },
        { 21, 21, 21, 21, 21 }
    }, [RTX_32X16] = {
        { 0, 16, 6, 6, 21 },
        { 16, 16, 6, 21, 21 },
        { 16, 16, 21, 21, 21 },
        { 16, 16, 21, 21, 21 },
        { 16, 16, 21, 21, 21 }
    }, [RTX_32X64] = {
        { 0, 11, 11, 11, 11 },
        { 11, 11, 11, 11, 11 },
        { 6, 6, 21, 21, 21 },
        { 6, 21, 21, 21, 21 },
        { 21, 21, 21, 21, 21 }
    }, [RTX_64X32] = {
        { 0, 16, 6, 6, 21 },
        { 16, 16, 6, 21, 21 },
        { 16, 16, 21, 21, 21 },
        { 16, 16, 21, 21, 21 },
        { 16, 16, 21, 21, 21 }
    }, [RTX_4X16] = {
        { 0, 11, 11, 11 },
        { 11, 11, 11, 11 },
        { 6, 6, 21, 21 },
        { 6, 21, 21, 21 },
        { 21, 21, 21, 21 }
    }, [RTX_16X4] = {
        { 0, 16, 6, 6, 21 },
        { 16, 16, 6, 21, 21 },
        { 16, 16, 21, 21, 21 },
        { 16, 16, 21, 21, 21 },
    }, [RTX_8X32] = {
        { 0, 11, 11, 11, 11 },
        { 11, 11, 11, 11, 11 },
        { 6, 6, 21, 21, 21 },
        { 6, 21, 21, 21, 21 },
        { 21, 21, 21, 21, 21 }
    }, [RTX_32X8] = {
        { 0, 16, 6, 6, 21 },
        { 16, 16, 6, 21, 21 },
        { 16, 16, 21, 21, 21 },
        { 16, 16, 21, 21, 21 },
        { 16, 16, 21, 21, 21 }
    }, [RTX_16X64] = {
        { 0, 11, 11, 11, 11 },
        { 11, 11, 11, 11, 11 },
        { 6, 6, 21, 21, 21 },
        { 6, 21, 21, 21, 21 },
        { 21, 21, 21, 21, 21 }
    }, [RTX_64X16] = {
        { 0, 16, 6, 6, 21 },
        { 16, 16, 6, 21, 21 },
        { 16, 16, 21, 21, 21 },
        { 16, 16, 21, 21, 21 },
        { 16, 16, 21, 21, 21 }
    }
};

const uint8_t /* enum TxClass */ av1_tx_type_class[N_TX_TYPES_PLUS_LL] = {
    [DCT_DCT]           = TX_CLASS_2D,
    [ADST_DCT]          = TX_CLASS_2D,
    [DCT_ADST]          = TX_CLASS_2D,
    [ADST_ADST]         = TX_CLASS_2D,
    [FLIPADST_DCT]      = TX_CLASS_2D,
    [DCT_FLIPADST]      = TX_CLASS_2D,
    [FLIPADST_FLIPADST] = TX_CLASS_2D,
    [ADST_FLIPADST]     = TX_CLASS_2D,
    [FLIPADST_ADST]     = TX_CLASS_2D,
    [IDTX]              = TX_CLASS_2D,
    [V_DCT]             = TX_CLASS_V,
    [H_DCT]             = TX_CLASS_H,
    [V_ADST]            = TX_CLASS_V,
    [H_ADST]            = TX_CLASS_H,
    [V_FLIPADST]        = TX_CLASS_V,
    [H_FLIPADST]        = TX_CLASS_H,
    [WHT_WHT]           = TX_CLASS_2D,
};

const uint8_t /* enum Filter2d */ av1_filter_2d[N_FILTERS][N_FILTERS] = {
    [FILTER_8TAP_REGULAR] = {
        [FILTER_8TAP_REGULAR] = FILTER_2D_8TAP_REGULAR,
        [FILTER_8TAP_SHARP]   = FILTER_2D_8TAP_REGULAR_SHARP,
        [FILTER_8TAP_SMOOTH]  = FILTER_2D_8TAP_REGULAR_SMOOTH,
    }, [FILTER_8TAP_SHARP] = {
        [FILTER_8TAP_REGULAR] = FILTER_2D_8TAP_SHARP_REGULAR,
        [FILTER_8TAP_SHARP]   = FILTER_2D_8TAP_SHARP,
        [FILTER_8TAP_SMOOTH]  = FILTER_2D_8TAP_SHARP_SMOOTH,
    }, [FILTER_8TAP_SMOOTH] = {
        [FILTER_8TAP_REGULAR] = FILTER_2D_8TAP_SMOOTH_REGULAR,
        [FILTER_8TAP_SHARP]   = FILTER_2D_8TAP_SMOOTH_SHARP,
        [FILTER_8TAP_SMOOTH]  = FILTER_2D_8TAP_SMOOTH,
    }, [FILTER_BILINEAR] = {
        [FILTER_BILINEAR]     = FILTER_2D_BILINEAR,
    }
};

const uint8_t /* enum FilterMode */ eve_av1_filter_dir[N_2D_FILTERS][2] = {
    [FILTER_2D_8TAP_REGULAR]        = { FILTER_8TAP_REGULAR, FILTER_8TAP_REGULAR },
    [FILTER_2D_8TAP_REGULAR_SMOOTH] = { FILTER_8TAP_SMOOTH,  FILTER_8TAP_REGULAR },
    [FILTER_2D_8TAP_REGULAR_SHARP]  = { FILTER_8TAP_SHARP,   FILTER_8TAP_REGULAR },
    [FILTER_2D_8TAP_SHARP_REGULAR]  = { FILTER_8TAP_REGULAR, FILTER_8TAP_SHARP   },
    [FILTER_2D_8TAP_SHARP_SMOOTH]   = { FILTER_8TAP_SMOOTH,  FILTER_8TAP_SHARP   },
    [FILTER_2D_8TAP_SHARP]          = { FILTER_8TAP_SHARP,   FILTER_8TAP_SHARP   },
    [FILTER_2D_8TAP_SMOOTH_REGULAR] = { FILTER_8TAP_REGULAR, FILTER_8TAP_SMOOTH  },
    [FILTER_2D_8TAP_SMOOTH]         = { FILTER_8TAP_SMOOTH,  FILTER_8TAP_SMOOTH  },
    [FILTER_2D_8TAP_SMOOTH_SHARP]   = { FILTER_8TAP_SHARP,   FILTER_8TAP_SMOOTH  },
    [FILTER_2D_BILINEAR]            = { FILTER_2D_BILINEAR,  FILTER_2D_BILINEAR  },
};

const uint8_t av1_filter_mode_to_y_mode[5] = {
    DC_PRED, VERT_PRED, HOR_PRED, HOR_DOWN_PRED, DC_PRED
};

const uint8_t intra_mode_context[N_INTRA_PRED_MODES] = {
    [DC_PRED]              = 0,
    [VERT_PRED]            = 1,
    [HOR_PRED]             = 2,
    [DIAG_DOWN_LEFT_PRED]  = 3,
    [DIAG_DOWN_RIGHT_PRED] = 4,
    [VERT_RIGHT_PRED]      = 4,
    [HOR_DOWN_PRED]        = 4,
    [HOR_UP_PRED]          = 4,
    [VERT_LEFT_PRED]       = 3,
    [SMOOTH_PRED]          = 0,
    [SMOOTH_V_PRED]        = 1,
    [SMOOTH_H_PRED]        = 2,
    [PAETH_PRED]           = 0,
};

const unsigned cfl_allowed_mask =
    (1 << BS_32x32) |
    (1 << BS_32x16) |
    (1 << BS_32x8) |
    (1 << BS_16x32) |
    (1 << BS_16x16) |
    (1 << BS_16x8) |
    (1 << BS_16x4) |
    (1 << BS_8x32) |
    (1 << BS_8x16) |
    (1 << BS_8x8) |
    (1 << BS_8x4) |
    (1 << BS_4x16) |
    (1 << BS_4x8) |
    (1 << BS_4x4);

const unsigned wedge_allowed_mask =
    (1 << BS_32x32) |
    (1 << BS_32x16) |
    (1 << BS_32x8) |
    (1 << BS_16x32) |
    (1 << BS_16x16) |
    (1 << BS_16x8) |
    (1 << BS_8x32) |
    (1 << BS_8x16) |
    (1 << BS_8x8);

const unsigned interintra_allowed_mask =
    (1 << BS_32x32) |
    (1 << BS_32x16) |
    (1 << BS_16x32) |
    (1 << BS_16x16) |
    (1 << BS_16x8) |
    (1 << BS_8x16) |
    (1 << BS_8x8);

const uint8_t av1_wedge_ctx_lut[N_BS_SIZES] = {
    [BS_32x32] = 6,
    [BS_32x16] = 5,
    [BS_32x8]  = 8,
    [BS_16x32] = 4,
    [BS_16x16] = 3,
    [BS_16x8]  = 2,
    [BS_8x32]  = 7,
    [BS_8x16]  = 1,
    [BS_8x8]   = 0,
};

const WarpedMotionParams default_wm_params = {
    .type = WM_TYPE_IDENTITY,
    .matrix = {
        0, 0, 1 << 16,
        0, 0, 1 << 16,
    },
    .alpha = 0,
    .beta = 0,
    .gamma = 0,
    .delta = 0,
};

const int16_t sgr_params[16][4] = { // r0, r1, e0, e1
    { 2, 1, 140, 3236 }, { 2, 1, 112, 2158 }, { 2, 1, 93, 1618 },
    { 2, 1,  80, 1438 }, { 2, 1,  70, 1295 }, { 2, 1, 58, 1177 },
    { 2, 1,  47, 1079 }, { 2, 1,  37,  996 }, { 2, 1, 30,  925 },
    { 2, 1,  25,  863 }, { 0, 1,  -1, 2589 }, { 0, 1, -1, 1618 },
    { 0, 1,  -1, 1177 }, { 0, 1,  -1,  925 }, { 2, 0, 56,   -1 },
    { 2, 0,  22,   -1 },
};

const int16_t sgr_x_by_xplus1[256] = {
  1,   128, 171, 192, 205, 213, 219, 224, 228, 230, 233, 235, 236, 238, 239,
  240, 241, 242, 243, 243, 244, 244, 245, 245, 246, 246, 247, 247, 247, 247,
  248, 248, 248, 248, 249, 249, 249, 249, 249, 250, 250, 250, 250, 250, 250,
  250, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 252, 252, 252, 252,
  252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 253, 253,
  253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253,
  253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 254, 254, 254,
  254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
  254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
  254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
  254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
  254, 254, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
  256,
};

const int16_t sgr_one_by_x[] = {
  4096, 2048, 1365, 1024, 819, 683, 585, 512, 455, 410, 372, 341, 315,
  293,  273,  256,  241,  228, 216, 205, 195, 186, 178, 171, 164,
};

const int8_t dav1d_mc_subpel_filters[5][15][8] = {
    [FILTER_8TAP_REGULAR] = {
        { 0, 2,  -6, 126,   8,  -2, 0, 0 },
        { 0, 2, -10, 122,  18,  -4, 0, 0 },
        { 0, 2, -12, 116,  28,  -8, 2, 0 },
        { 0, 2, -14, 110,  38, -10, 2, 0 },
        { 0, 2, -14, 102,  48, -12, 2, 0 },
        { 0, 2, -16,  94,  58, -12, 2, 0 },
        { 0, 2, -14,  84,  66, -12, 2, 0 },
        { 0, 2, -14,  76,  76, -14, 2, 0 },
        { 0, 2, -12,  66,  84, -14, 2, 0 },
        { 0, 2, -12,  58,  94, -16, 2, 0 },
        { 0, 2, -12,  48, 102, -14, 2, 0 },
        { 0, 2, -10,  38, 110, -14, 2, 0 },
        { 0, 2,  -8,  28, 116, -12, 2, 0 },
        { 0, 0,  -4,  18, 122, -10, 2, 0 },
        { 0, 0,  -2,   8, 126,  -6, 2, 0 }
    }, [FILTER_8TAP_SHARP] = {
        { -2,  2,  -6, 126,   8,  -2,  2,  0 },
        { -2,  6, -12, 124,  16,  -6,  4, -2 },
        { -2,  8, -18, 120,  26, -10,  6, -2 },
        { -4, 10, -22, 116,  38, -14,  6, -2 },
        { -4, 10, -22, 108,  48, -18,  8, -2 },
        { -4, 10, -24, 100,  60, -20,  8, -2 },
        { -4, 10, -24,  90,  70, -22, 10, -2 },
        { -4, 12, -24,  80,  80, -24, 12, -4 },
        { -2, 10, -22,  70,  90, -24, 10, -4 },
        { -2,  8, -20,  60, 100, -24, 10, -4 },
        { -2,  8, -18,  48, 108, -22, 10, -4 },
        { -2,  6, -14,  38, 116, -22, 10, -4 },
        { -2,  6, -10,  26, 120, -18,  8, -2 },
        { -2,  4,  -6,  16, 124, -12,  6, -2 },
        {  0,  2,  -2,   8, 126,  -6,  2, -2 }
    }, [FILTER_8TAP_SMOOTH] = {
        { 0,  2, 28,  62, 34,  2,  0, 0 },
        { 0,  0, 26,  62, 36,  4,  0, 0 },
        { 0,  0, 22,  62, 40,  4,  0, 0 },
        { 0,  0, 20,  60, 42,  6,  0, 0 },
        { 0,  0, 18,  58, 44,  8,  0, 0 },
        { 0,  0, 16,  56, 46, 10,  0, 0 },
        { 0, -2, 16,  54, 48, 12,  0, 0 },
        { 0, -2, 14,  52, 52, 14, -2, 0 },
        { 0,  0, 12,  48, 54, 16, -2, 0 },
        { 0,  0, 10,  46, 56, 16,  0, 0 },
        { 0,  0,  8,  44, 58, 18,  0, 0 },
        { 0,  0,  6,  42, 60, 20,  0, 0 },
        { 0,  0,  4,  40, 62, 22,  0, 0 },
        { 0,  0,  4,  36, 62, 26,  0, 0 },
        { 0,  0,  2,  34, 62, 28,  2, 0 },
    },
    /* width <= 4 */
    [3 + FILTER_8TAP_REGULAR] = {
        { 0, 0,  -4, 126,   8,  -2, 0, 0 },
        { 0, 0,  -8, 122,  18,  -4, 0, 0 },
        { 0, 0, -10, 116,  28,  -6, 0, 0 },
        { 0, 0, -12, 110,  38,  -8, 0, 0 },
        { 0, 0, -12, 102,  48, -10, 0, 0 },
        { 0, 0, -14,  94,  58, -10, 0, 0 },
        { 0, 0, -12,  84,  66, -10, 0, 0 },
        { 0, 0, -12,  76,  76, -12, 0, 0 },
        { 0, 0, -10,  66,  84, -12, 0, 0 },
        { 0, 0, -10,  58,  94, -14, 0, 0 },
        { 0, 0, -10,  48, 102, -12, 0, 0 },
        { 0, 0,  -8,  38, 110, -12, 0, 0 },
        { 0, 0,  -6,  28, 116, -10, 0, 0 },
        { 0, 0,  -4,  18, 122,  -8, 0, 0 },
        { 0, 0,  -2,   8, 126,  -4, 0, 0 }
    }, [3 + FILTER_8TAP_SMOOTH] = {
        { 0, 0, 30,  62, 34,  2, 0, 0 },
        { 0, 0, 26,  62, 36,  4, 0, 0 },
        { 0, 0, 22,  62, 40,  4, 0, 0 },
        { 0, 0, 20,  60, 42,  6, 0, 0 },
        { 0, 0, 18,  58, 44,  8, 0, 0 },
        { 0, 0, 16,  56, 46, 10, 0, 0 },
        { 0, 0, 14,  54, 48, 12, 0, 0 },
        { 0, 0, 12,  52, 52, 12, 0, 0 },
        { 0, 0, 12,  48, 54, 14, 0, 0 },
        { 0, 0, 10,  46, 56, 16, 0, 0 },
        { 0, 0,  8,  44, 58, 18, 0, 0 },
        { 0, 0,  6,  42, 60, 20, 0, 0 },
        { 0, 0,  4,  40, 62, 22, 0, 0 },
        { 0, 0,  4,  36, 62, 26, 0, 0 },
        { 0, 0,  2,  34, 62, 30, 0, 0 }
    }
};

const int8_t dav1d_mc_warp_filter[][8] = {
    // [-1, 0)
    { 0,   0, 127,   1,   0, 0, 0, 0 }, { 0, - 1, 127,   2,   0, 0, 0, 0 },
    { 1, - 3, 127,   4, - 1, 0, 0, 0 }, { 1, - 4, 126,   6, - 2, 1, 0, 0 },
    { 1, - 5, 126,   8, - 3, 1, 0, 0 }, { 1, - 6, 125,  11, - 4, 1, 0, 0 },
    { 1, - 7, 124,  13, - 4, 1, 0, 0 }, { 2, - 8, 123,  15, - 5, 1, 0, 0 },
    { 2, - 9, 122,  18, - 6, 1, 0, 0 }, { 2, -10, 121,  20, - 6, 1, 0, 0 },
    { 2, -11, 120,  22, - 7, 2, 0, 0 }, { 2, -12, 119,  25, - 8, 2, 0, 0 },
    { 3, -13, 117,  27, - 8, 2, 0, 0 }, { 3, -13, 116,  29, - 9, 2, 0, 0 },
    { 3, -14, 114,  32, -10, 3, 0, 0 }, { 3, -15, 113,  35, -10, 2, 0, 0 },
    { 3, -15, 111,  37, -11, 3, 0, 0 }, { 3, -16, 109,  40, -11, 3, 0, 0 },
    { 3, -16, 108,  42, -12, 3, 0, 0 }, { 4, -17, 106,  45, -13, 3, 0, 0 },
    { 4, -17, 104,  47, -13, 3, 0, 0 }, { 4, -17, 102,  50, -14, 3, 0, 0 },
    { 4, -17, 100,  52, -14, 3, 0, 0 }, { 4, -18,  98,  55, -15, 4, 0, 0 },
    { 4, -18,  96,  58, -15, 3, 0, 0 }, { 4, -18,  94,  60, -16, 4, 0, 0 },
    { 4, -18,  91,  63, -16, 4, 0, 0 }, { 4, -18,  89,  65, -16, 4, 0, 0 },
    { 4, -18,  87,  68, -17, 4, 0, 0 }, { 4, -18,  85,  70, -17, 4, 0, 0 },
    { 4, -18,  82,  73, -17, 4, 0, 0 }, { 4, -18,  80,  75, -17, 4, 0, 0 },
    { 4, -18,  78,  78, -18, 4, 0, 0 }, { 4, -17,  75,  80, -18, 4, 0, 0 },
    { 4, -17,  73,  82, -18, 4, 0, 0 }, { 4, -17,  70,  85, -18, 4, 0, 0 },
    { 4, -17,  68,  87, -18, 4, 0, 0 }, { 4, -16,  65,  89, -18, 4, 0, 0 },
    { 4, -16,  63,  91, -18, 4, 0, 0 }, { 4, -16,  60,  94, -18, 4, 0, 0 },
    { 3, -15,  58,  96, -18, 4, 0, 0 }, { 4, -15,  55,  98, -18, 4, 0, 0 },
    { 3, -14,  52, 100, -17, 4, 0, 0 }, { 3, -14,  50, 102, -17, 4, 0, 0 },
    { 3, -13,  47, 104, -17, 4, 0, 0 }, { 3, -13,  45, 106, -17, 4, 0, 0 },
    { 3, -12,  42, 108, -16, 3, 0, 0 }, { 3, -11,  40, 109, -16, 3, 0, 0 },
    { 3, -11,  37, 111, -15, 3, 0, 0 }, { 2, -10,  35, 113, -15, 3, 0, 0 },
    { 3, -10,  32, 114, -14, 3, 0, 0 }, { 2, - 9,  29, 116, -13, 3, 0, 0 },
    { 2, - 8,  27, 117, -13, 3, 0, 0 }, { 2, - 8,  25, 119, -12, 2, 0, 0 },
    { 2, - 7,  22, 120, -11, 2, 0, 0 }, { 1, - 6,  20, 121, -10, 2, 0, 0 },
    { 1, - 6,  18, 122, - 9, 2, 0, 0 }, { 1, - 5,  15, 123, - 8, 2, 0, 0 },
    { 1, - 4,  13, 124, - 7, 1, 0, 0 }, { 1, - 4,  11, 125, - 6, 1, 0, 0 },
    { 1, - 3,   8, 126, - 5, 1, 0, 0 }, { 1, - 2,   6, 126, - 4, 1, 0, 0 },
    { 0, - 1,   4, 127, - 3, 1, 0, 0 }, { 0,   0,   2, 127, - 1, 0, 0, 0 },

    // [0, 1)
    { 0,  0,   0, 127,   1,   0,  0,  0}, { 0,  0,  -1, 127,   2,   0,  0,  0},
    { 0,  1,  -3, 127,   4,  -2,  1,  0}, { 0,  1,  -5, 127,   6,  -2,  1,  0},
    { 0,  2,  -6, 126,   8,  -3,  1,  0}, {-1,  2,  -7, 126,  11,  -4,  2, -1},
    {-1,  3,  -8, 125,  13,  -5,  2, -1}, {-1,  3, -10, 124,  16,  -6,  3, -1},
    {-1,  4, -11, 123,  18,  -7,  3, -1}, {-1,  4, -12, 122,  20,  -7,  3, -1},
    {-1,  4, -13, 121,  23,  -8,  3, -1}, {-2,  5, -14, 120,  25,  -9,  4, -1},
    {-1,  5, -15, 119,  27, -10,  4, -1}, {-1,  5, -16, 118,  30, -11,  4, -1},
    {-2,  6, -17, 116,  33, -12,  5, -1}, {-2,  6, -17, 114,  35, -12,  5, -1},
    {-2,  6, -18, 113,  38, -13,  5, -1}, {-2,  7, -19, 111,  41, -14,  6, -2},
    {-2,  7, -19, 110,  43, -15,  6, -2}, {-2,  7, -20, 108,  46, -15,  6, -2},
    {-2,  7, -20, 106,  49, -16,  6, -2}, {-2,  7, -21, 104,  51, -16,  7, -2},
    {-2,  7, -21, 102,  54, -17,  7, -2}, {-2,  8, -21, 100,  56, -18,  7, -2},
    {-2,  8, -22,  98,  59, -18,  7, -2}, {-2,  8, -22,  96,  62, -19,  7, -2},
    {-2,  8, -22,  94,  64, -19,  7, -2}, {-2,  8, -22,  91,  67, -20,  8, -2},
    {-2,  8, -22,  89,  69, -20,  8, -2}, {-2,  8, -22,  87,  72, -21,  8, -2},
    {-2,  8, -21,  84,  74, -21,  8, -2}, {-2,  8, -22,  82,  77, -21,  8, -2},
    {-2,  8, -21,  79,  79, -21,  8, -2}, {-2,  8, -21,  77,  82, -22,  8, -2},
    {-2,  8, -21,  74,  84, -21,  8, -2}, {-2,  8, -21,  72,  87, -22,  8, -2},
    {-2,  8, -20,  69,  89, -22,  8, -2}, {-2,  8, -20,  67,  91, -22,  8, -2},
    {-2,  7, -19,  64,  94, -22,  8, -2}, {-2,  7, -19,  62,  96, -22,  8, -2},
    {-2,  7, -18,  59,  98, -22,  8, -2}, {-2,  7, -18,  56, 100, -21,  8, -2},
    {-2,  7, -17,  54, 102, -21,  7, -2}, {-2,  7, -16,  51, 104, -21,  7, -2},
    {-2,  6, -16,  49, 106, -20,  7, -2}, {-2,  6, -15,  46, 108, -20,  7, -2},
    {-2,  6, -15,  43, 110, -19,  7, -2}, {-2,  6, -14,  41, 111, -19,  7, -2},
    {-1,  5, -13,  38, 113, -18,  6, -2}, {-1,  5, -12,  35, 114, -17,  6, -2},
    {-1,  5, -12,  33, 116, -17,  6, -2}, {-1,  4, -11,  30, 118, -16,  5, -1},
    {-1,  4, -10,  27, 119, -15,  5, -1}, {-1,  4,  -9,  25, 120, -14,  5, -2},
    {-1,  3,  -8,  23, 121, -13,  4, -1}, {-1,  3,  -7,  20, 122, -12,  4, -1},
    {-1,  3,  -7,  18, 123, -11,  4, -1}, {-1,  3,  -6,  16, 124, -10,  3, -1},
    {-1,  2,  -5,  13, 125,  -8,  3, -1}, {-1,  2,  -4,  11, 126,  -7,  2, -1},
    { 0,  1,  -3,   8, 126,  -6,  2,  0}, { 0,  1,  -2,   6, 127,  -5,  1,  0},
    { 0,  1,  -2,   4, 127,  -3,  1,  0}, { 0,  0,   0,   2, 127,  -1,  0,  0},

    // [1, 2)
    { 0, 0, 0,   1, 127,   0,   0, 0 }, { 0, 0, 0, - 1, 127,   2,   0, 0 },
    { 0, 0, 1, - 3, 127,   4, - 1, 0 }, { 0, 0, 1, - 4, 126,   6, - 2, 1 },
    { 0, 0, 1, - 5, 126,   8, - 3, 1 }, { 0, 0, 1, - 6, 125,  11, - 4, 1 },
    { 0, 0, 1, - 7, 124,  13, - 4, 1 }, { 0, 0, 2, - 8, 123,  15, - 5, 1 },
    { 0, 0, 2, - 9, 122,  18, - 6, 1 }, { 0, 0, 2, -10, 121,  20, - 6, 1 },
    { 0, 0, 2, -11, 120,  22, - 7, 2 }, { 0, 0, 2, -12, 119,  25, - 8, 2 },
    { 0, 0, 3, -13, 117,  27, - 8, 2 }, { 0, 0, 3, -13, 116,  29, - 9, 2 },
    { 0, 0, 3, -14, 114,  32, -10, 3 }, { 0, 0, 3, -15, 113,  35, -10, 2 },
    { 0, 0, 3, -15, 111,  37, -11, 3 }, { 0, 0, 3, -16, 109,  40, -11, 3 },
    { 0, 0, 3, -16, 108,  42, -12, 3 }, { 0, 0, 4, -17, 106,  45, -13, 3 },
    { 0, 0, 4, -17, 104,  47, -13, 3 }, { 0, 0, 4, -17, 102,  50, -14, 3 },
    { 0, 0, 4, -17, 100,  52, -14, 3 }, { 0, 0, 4, -18,  98,  55, -15, 4 },
    { 0, 0, 4, -18,  96,  58, -15, 3 }, { 0, 0, 4, -18,  94,  60, -16, 4 },
    { 0, 0, 4, -18,  91,  63, -16, 4 }, { 0, 0, 4, -18,  89,  65, -16, 4 },
    { 0, 0, 4, -18,  87,  68, -17, 4 }, { 0, 0, 4, -18,  85,  70, -17, 4 },
    { 0, 0, 4, -18,  82,  73, -17, 4 }, { 0, 0, 4, -18,  80,  75, -17, 4 },
    { 0, 0, 4, -18,  78,  78, -18, 4 }, { 0, 0, 4, -17,  75,  80, -18, 4 },
    { 0, 0, 4, -17,  73,  82, -18, 4 }, { 0, 0, 4, -17,  70,  85, -18, 4 },
    { 0, 0, 4, -17,  68,  87, -18, 4 }, { 0, 0, 4, -16,  65,  89, -18, 4 },
    { 0, 0, 4, -16,  63,  91, -18, 4 }, { 0, 0, 4, -16,  60,  94, -18, 4 },
    { 0, 0, 3, -15,  58,  96, -18, 4 }, { 0, 0, 4, -15,  55,  98, -18, 4 },
    { 0, 0, 3, -14,  52, 100, -17, 4 }, { 0, 0, 3, -14,  50, 102, -17, 4 },
    { 0, 0, 3, -13,  47, 104, -17, 4 }, { 0, 0, 3, -13,  45, 106, -17, 4 },
    { 0, 0, 3, -12,  42, 108, -16, 3 }, { 0, 0, 3, -11,  40, 109, -16, 3 },
    { 0, 0, 3, -11,  37, 111, -15, 3 }, { 0, 0, 2, -10,  35, 113, -15, 3 },
    { 0, 0, 3, -10,  32, 114, -14, 3 }, { 0, 0, 2, - 9,  29, 116, -13, 3 },
    { 0, 0, 2, - 8,  27, 117, -13, 3 }, { 0, 0, 2, - 8,  25, 119, -12, 2 },
    { 0, 0, 2, - 7,  22, 120, -11, 2 }, { 0, 0, 1, - 6,  20, 121, -10, 2 },
    { 0, 0, 1, - 6,  18, 122, - 9, 2 }, { 0, 0, 1, - 5,  15, 123, - 8, 2 },
    { 0, 0, 1, - 4,  13, 124, - 7, 1 }, { 0, 0, 1, - 4,  11, 125, - 6, 1 },
    { 0, 0, 1, - 3,   8, 126, - 5, 1 }, { 0, 0, 1, - 2,   6, 126, - 4, 1 },
    { 0, 0, 0, - 1,   4, 127, - 3, 1 }, { 0, 0, 0,   0,   2, 127, - 1, 0 },

    // dummy (replicate row index 191)
    { 0, 0, 0,   0,   2, 127, - 1, 0 },
};
