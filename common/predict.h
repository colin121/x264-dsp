/*****************************************************************************
 * predict.h: intra prediction
 *****************************************************************************/

#ifndef X264_PREDICT_H
#define X264_PREDICT_H

typedef void (*x264_predict_t)( pixel *src );

enum intra_chroma_pred_e
{
    I_PRED_CHROMA_DC = 0,
    I_PRED_CHROMA_H  = 1,
    I_PRED_CHROMA_V  = 2,
    I_PRED_CHROMA_P  = 3,

    I_PRED_CHROMA_DC_LEFT = 4,
    I_PRED_CHROMA_DC_TOP  = 5,
    I_PRED_CHROMA_DC_128  = 6
};
static const uint8_t x264_mb_chroma_pred_mode_fix[7] =
{
    I_PRED_CHROMA_DC, I_PRED_CHROMA_H, I_PRED_CHROMA_V, I_PRED_CHROMA_P,
    I_PRED_CHROMA_DC, I_PRED_CHROMA_DC,I_PRED_CHROMA_DC
};

enum intra16x16_pred_e
{
    I_PRED_16x16_V  = 0,
    I_PRED_16x16_H  = 1,
    I_PRED_16x16_DC = 2,
    I_PRED_16x16_P  = 3,

    I_PRED_16x16_DC_LEFT = 4,
    I_PRED_16x16_DC_TOP  = 5,
    I_PRED_16x16_DC_128  = 6,
};
static const uint8_t x264_mb_pred_mode16x16_fix[7] =
{
    I_PRED_16x16_V, I_PRED_16x16_H, I_PRED_16x16_DC, I_PRED_16x16_P,
    I_PRED_16x16_DC,I_PRED_16x16_DC,I_PRED_16x16_DC
};

enum intra4x4_pred_e
{
    I_PRED_4x4_V  = 0,
    I_PRED_4x4_H  = 1,
    I_PRED_4x4_DC = 2,
    I_PRED_4x4_DDL= 3,
    I_PRED_4x4_DDR= 4,
    I_PRED_4x4_VR = 5,
    I_PRED_4x4_HD = 6,
    I_PRED_4x4_VL = 7,
    I_PRED_4x4_HU = 8,

    I_PRED_4x4_DC_LEFT = 9,
    I_PRED_4x4_DC_TOP  = 10,
    I_PRED_4x4_DC_128  = 11,
};
static const int8_t x264_mb_pred_mode4x4_fix[13] =
{
    -1,
    I_PRED_4x4_V,   I_PRED_4x4_H,   I_PRED_4x4_DC,
    I_PRED_4x4_DDL, I_PRED_4x4_DDR, I_PRED_4x4_VR,
    I_PRED_4x4_HD,  I_PRED_4x4_VL,  I_PRED_4x4_HU,
    I_PRED_4x4_DC,  I_PRED_4x4_DC,  I_PRED_4x4_DC
};
#define x264_mb_pred_mode4x4_fix(t) x264_mb_pred_mode4x4_fix[(t)+1]

void x264_predict_4x4_dc_c  ( pixel *src );
void x264_predict_4x4_h_c   ( pixel *src );
void x264_predict_4x4_v_c   ( pixel *src );
void x264_predict_16x16_dc_c( pixel *src );
void x264_predict_16x16_h_c ( pixel *src );
void x264_predict_16x16_v_c ( pixel *src );
void x264_predict_16x16_p_c ( pixel *src );
void x264_predict_8x8c_dc_c ( pixel *src );
void x264_predict_8x8c_h_c  ( pixel *src );
void x264_predict_8x8c_v_c  ( pixel *src );
void x264_predict_8x8c_p_c  ( pixel *src );

void x264_predict_16x16_init ( int cpu, x264_predict_t pf[7] );
void x264_predict_8x8c_init  ( int cpu, x264_predict_t pf[7] );
void x264_predict_4x4_init   ( int cpu, x264_predict_t pf[12] );

#endif
