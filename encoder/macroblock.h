/*****************************************************************************
 * macroblock.h: macroblock encoding
 *****************************************************************************/

#ifndef X264_ENCODER_MACROBLOCK_H
#define X264_ENCODER_MACROBLOCK_H

#include "common/macroblock.h"

extern const int x264_lambda2_tab[QP_MAX_MAX+1];
extern const uint16_t x264_lambda_tab[QP_MAX_MAX+1];
extern const int x264_dequant_div_lut[QP_MAX+1][2];

int  x264_macroblock_probe_pskip ( x264_t *h );
void x264_macroblock_encode      ( x264_t *h );
void x264_macroblock_write_cabac ( x264_t *h, x264_cabac_t *cb );
void x264_macroblock_write_cavlc ( x264_t *h );

void x264_cabac_mb_skip( x264_t *h, int b_skip );

static ALWAYS_INLINE int x264_quant_4x4( x264_t *h, dctcoef dct[16], int i_qp, int ctx_block_cat, int b_intra, int idx )
{
    int i_quant_cat = b_intra ? CQM_4IY:CQM_4PY; /* quantization category */
    if( h->mb.b_noise_reduction )
        h->quantf.denoise_dct( dct, h->nr_residual_sum[0], h->nr_offset[0], 16 );
    return h->quantf.quant_4x4( dct, h->quant4_mf[i_quant_cat][i_qp], h->quant4_bias[i_quant_cat][i_qp] );
}

/************************************************************
 * x264_mb_encode_i4x4: encode intra 4x4 block.             *
 * This includes dct, quant, zigzag, dequant, idct.         *
 * idx represents block index in macroblock. range: [0-15]  *
 * i_qp represents block quantization parameter.            *
 * i_mode represents block intra predict mode.              *
 * b_predict represents whether need predict here.          *
 ************************************************************/
static ALWAYS_INLINE void x264_mb_encode_i4x4( x264_t *h, int idx, int i_qp, int i_mode, int b_predict )
{
    int nz; /* non-zero count */
    pixel *p_src = &h->mb.pic.p_fenc[0][block_idx_xy_fenc[idx]];
    pixel *p_dst = &h->mb.pic.p_fdec[0][block_idx_xy_fdec[idx]];
    ALIGNED_ARRAY_16( dctcoef, dct4x4,[16] ); /* dct coefficients */

    if( b_predict )
    {
        h->predict_4x4[i_mode]( p_dst );
    }

    h->dctf.sub4x4_dct( dct4x4, p_src, p_dst );
    nz = x264_quant_4x4( h, dct4x4, i_qp, DCT_LUMA_4x4, 1, idx );
    h->mb.cache.non_zero_count[x264_scan8[idx]] = nz;
    if( nz )
    {
    	/* Set CBP luma corresponding bit to 1  */
    	/* This indicates transform coefficient levels of belonging 8x8 block shall be non-zero. */
        h->mb.i_cbp_luma |= 1<<(idx>>2);
        h->zigzagf.scan_4x4( h->dct.luma4x4[idx], dct4x4 ); /* zigzag scan */
        h->quantf.dequant_4x4( dct4x4, h->dequant4_mf[CQM_4IY], i_qp ); /* dequantization */
        h->dctf.add4x4_idct( p_dst, dct4x4 ); /* inverse dct */
    }
}

#endif
