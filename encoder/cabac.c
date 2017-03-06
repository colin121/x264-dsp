/*****************************************************************************
 * cabac.c: cabac bitstream writing
 *****************************************************************************/

#include "common/common.h"
#include "macroblock.h"

/**************************************************************
 * <ITU-T-REC-H.264-201201.pdf> Page 252                      *
 * Table 9-36 ¨C Binarization for macroblock types in I slices *
 * Value (name) of mb_type | Bin string                       *
 * 0 (I_NxN)               | 0                                *
 * 1 (I_16x16_0_0_0)       | 1 0 0 0 0 0                      *
 * 2 (I_16x16_1_0_0)       | 1 0 0 0 0 1                      *
 * 3 (I_16x16_2_0_0)       | 1 0 0 0 1 0                      *
 * 4 (I_16x16_3_0_0)       | 1 0 0 0 1 1                      *
 * 5 (I_16x16_0_1_0)       | 1 0 0 1 0 0 0                    *
 * 6 (I_16x16_1_1_0)       | 1 0 0 1 0 0 1                    *
 * 7 (I_16x16_2_1_0)       | 1 0 0 1 0 1 0                    *
 * 8 (I_16x16_3_1_0)       | 1 0 0 1 0 1 1                    *
 * 9 (I_16x16_0_2_0)       | 1 0 0 1 1 0 0                    *
 * 10 (I_16x16_1_2_0)      | 1 0 0 1 1 0 1                    *
 * 11 (I_16x16_2_2_0)      | 1 0 0 1 1 1 0                    *
 * 12 (I_16x16_3_2_0)      | 1 0 0 1 1 1 1                    *
 * 13 (I_16x16_0_0_1)      | 1 0 1 0 0 0                      *
 * 14 (I_16x16_1_0_1)      | 1 0 1 0 0 1                      *
 * 15 (I_16x16_2_0_1)      | 1 0 1 0 1 0                      *
 * 16 (I_16x16_3_0_1)      | 1 0 1 0 1 1                      *
 * 17 (I_16x16_0_1_1)      | 1 0 1 1 0 0 0                    *
 * 18 (I_16x16_1_1_1)      | 1 0 1 1 0 0 1                    *
 * 19 (I_16x16_2_1_1)      | 1 0 1 1 0 1 0                    *
 * 20 (I_16x16_3_1_1)      | 1 0 1 1 0 1 1                    *
 * 21 (I_16x16_0_2_1)      | 1 0 1 1 1 0 0                    *
 * 22 (I_16x16_1_2_1)      | 1 0 1 1 1 0 1                    *
 * 23 (I_16x16_2_2_1)      | 1 0 1 1 1 1 0                    *
 * 24 (I_16x16_3_2_1)      | 1 0 1 1 1 1 1                    *
 **************************************************************/
static inline void x264_cabac_mb_type_intra( x264_t *h, x264_cabac_t *cb, int i_mb_type,
                    int ctx0, int ctx1, int ctx2, int ctx3, int ctx4, int ctx5 )
{
    if( i_mb_type == I_4x4 || i_mb_type == I_8x8 )
    {
        x264_cabac_encode_decision_noup( cb, ctx0, 0 );
    }
    else /* I_16x16 */
    {
        int i_pred = x264_mb_pred_mode16x16_fix[h->mb.i_intra16x16_pred_mode];

        x264_cabac_encode_decision_noup( cb, ctx0, 1 );
        x264_cabac_encode_terminal( cb );

        x264_cabac_encode_decision_noup( cb, ctx1, !!h->mb.i_cbp_luma );
        if( h->mb.i_cbp_chroma == 0 )
            x264_cabac_encode_decision_noup( cb, ctx2, 0 );
        else
        {
            x264_cabac_encode_decision( cb, ctx2, 1 );
            x264_cabac_encode_decision_noup( cb, ctx3, h->mb.i_cbp_chroma>>1 );
        }
        x264_cabac_encode_decision( cb, ctx4, i_pred>>1 );
        x264_cabac_encode_decision_noup( cb, ctx5, i_pred&1 );
    }
}

static void x264_cabac_intra4x4_pred_mode( x264_cabac_t *cb, int i_pred, int i_mode )
{
    if( i_pred == i_mode )
        x264_cabac_encode_decision( cb, 68, 1 );
    else
    {
        x264_cabac_encode_decision( cb, 68, 0 );
        if( i_mode > i_pred  )
            i_mode--;

        /* FL(Fixed-Length) binarization of Intra4x4PredMode with cMax = 7 */
        x264_cabac_encode_decision( cb, 69, (i_mode     )&0x01 );
        x264_cabac_encode_decision( cb, 69, (i_mode >> 1)&0x01 );
        x264_cabac_encode_decision( cb, 69, (i_mode >> 2)      );
    }
}

static void x264_cabac_intra_chroma_pred_mode( x264_t *h, x264_cabac_t *cb )
{
    int i_mode = x264_mb_chroma_pred_mode_fix[h->mb.i_chroma_pred_mode];
    int ctx = 0;

    /* No need to test for I4x4 or I_16x16 as cache_save handle that */
    if( (h->mb.i_neighbour & MB_LEFT) && h->mb.chroma_pred_mode[h->mb.i_mb_left_xy[0]] != 0 )
        ctx++;
    if( (h->mb.i_neighbour & MB_TOP) && h->mb.chroma_pred_mode[h->mb.i_mb_top_xy] != 0 )
        ctx++;

    /* TU(Truncated-Unary) binarization of IntraChromaPredMode with cMax = 3 */
    x264_cabac_encode_decision_noup( cb, 64 + ctx, i_mode > 0 );
    if( i_mode > 0 )
    {
        x264_cabac_encode_decision( cb, 64 + 3, i_mode > 1 );
        if( i_mode > 1 )
            x264_cabac_encode_decision_noup( cb, 64 + 3, i_mode > 2 );
    }
}

/****************************************************************************
 * The binarization of coded_block_pattern consists of a prefix part        *
 * and (when present) a suffix part. The prefix part of the binarization    *
 * is given by the FL binarization of CodedBlockPatternLuma with cMax = 15. *
 * When ChromaArrayType is not equal to 0 or 3, the suffix part is present  *
 * and consists of the TU binarization of CodedBlockPatternChroma with      *
 * cMax = 2.                                                                *
 ****************************************************************************/
static void x264_cabac_cbp_luma( x264_t *h, x264_cabac_t *cb )
{
    int cbp = h->mb.i_cbp_luma;
    int cbp_l = h->mb.cache.i_cbp_left;
    int cbp_t = h->mb.cache.i_cbp_top;

    /* FL(Fixed-Length) binarization of CodedBlockPatternLuma with cMax = 15 */
    x264_cabac_encode_decision     ( cb, 76 - ((cbp_l >> 1) & 1) - ((cbp_t >> 1) & 2), (cbp >> 0) & 1 );
    x264_cabac_encode_decision     ( cb, 76 - ((cbp   >> 0) & 1) - ((cbp_t >> 2) & 2), (cbp >> 1) & 1 );
    x264_cabac_encode_decision     ( cb, 76 - ((cbp_l >> 3) & 1) - ((cbp   << 1) & 2), (cbp >> 2) & 1 );
    x264_cabac_encode_decision_noup( cb, 76 - ((cbp   >> 2) & 1) - ((cbp   >> 0) & 2), (cbp >> 3) & 1 );
}

static void x264_cabac_cbp_chroma( x264_t *h, x264_cabac_t *cb )
{
    int cbp_a = h->mb.cache.i_cbp_left & 0x30;
    int cbp_b = h->mb.cache.i_cbp_top  & 0x30;
    int ctx = 0;

    if( cbp_a && h->mb.cache.i_cbp_left != -1 ) ctx++;
    if( cbp_b && h->mb.cache.i_cbp_top  != -1 ) ctx+=2;

    /* TU(Truncated-Unary) binarization of CodedBlockPatternChroma with cMax = 2 */
    if( h->mb.i_cbp_chroma == 0 )
        x264_cabac_encode_decision_noup( cb, 77 + ctx, 0 );
    else
    {
        x264_cabac_encode_decision_noup( cb, 77 + ctx, 1 );

        ctx = 4;
        if( cbp_a == 0x20 ) ctx++;
        if( cbp_b == 0x20 ) ctx += 2;
        x264_cabac_encode_decision_noup( cb, 77 + ctx, h->mb.i_cbp_chroma >> 1 );
    }
}

/********************************************************************
 * The bin string of mb_qp_delta is derived by the U binarization   *
 * of the mapped value of the syntax element mb_qp_delta, where the *
 * assignment rule between the signed value of mb_qp_delta and its  *
 * mapped value is given as specified in Table 9-3.                 *
 *                                                                  *
 * Table 9-3 ¨C Assignment of syntax element to codeNum              *
 * for signed Exp-Golomb coded syntax elements se(v)                *
 * | codeNum | syntax element value |                               *
 * | 0       |  0                   |                               *
 * | 1       |  1                   |                               *
 * | 2       | -1                   |                               *
 * | 3       |  2                   |                               *
 * | 4       | -2                   |                               *
 * | 5       |  3                   |                               *
 * | 6       | -3                   |                               *
 * | k       | (-1)k+1 Ceil(k/2)    |                               *
 ********************************************************************/
static void x264_cabac_qp_delta( x264_t *h, x264_cabac_t *cb )
{
    int i_dqp = h->mb.i_qp - h->mb.i_last_qp;
    int ctx;

    /* Avoid writing a delta quant if we have an empty i16x16 block, e.g. in a completely flat background area */
    if( h->mb.i_type == I_16x16 && !h->mb.cbp[h->mb.i_mb_xy] )
    {
        h->mb.i_qp = h->mb.i_last_qp;
        i_dqp = 0;
    }

    /* Since, per the above, empty-CBP I16x16 blocks never have delta quants,
     * we don't have to check for them. */
    ctx = h->mb.i_last_dqp && h->mb.cbp[h->mb.i_mb_prev_xy];

    /* U(Unary) binarization of the mapped value of the syntax element mb_qp_delta */
    if( i_dqp != 0 )
    {
        /* Faster than (i_dqp <= 0 ? (-2*i_dqp) : (2*i_dqp-1)).
         * If you so much as sneeze on these lines, gcc will compile this suboptimally. */
		int val;
        i_dqp <<= 1;
        val = 1 - i_dqp;
        if( val < 0 ) val = i_dqp;
        val--;
        /* dqp is interpreted modulo (QP_MAX_SPEC+1) */
        if( val >= QP_MAX_SPEC && val != QP_MAX_SPEC+1 )
            val = (QP_MAX_SPEC<<1)+1 - val;
        do
        {
            x264_cabac_encode_decision( cb, 60 + ctx, 1 );
            ctx = 2+(ctx>>1);
        } while( --val );
    }
    x264_cabac_encode_decision_noup( cb, 60 + ctx, 0 );
}

void x264_cabac_mb_skip( x264_t *h, int b_skip )
{
    int ctx = h->mb.cache.i_neighbour_skip + 11;
    if( h->sh.i_type != SLICE_TYPE_P )
       ctx += 13;
    x264_cabac_encode_decision( &h->cabac, ctx, b_skip );
}

static NOINLINE void x264_cabac_ref_p( x264_t *h, x264_cabac_t *cb, int idx )
{
    const int i8 = x264_scan8[idx];
    const int i_refa = h->mb.cache.ref[0][i8 - 1]; /* ref of left block */
    const int i_refb = h->mb.cache.ref[0][i8 - 8]; /* ref of top block */
    int ctx = 0;
	int i_ref;

    if( i_refa > 0 )
        ctx++;
    if( i_refb > 0 )
        ctx += 2;

    for( i_ref = h->mb.cache.ref[0][i8]; i_ref > 0; i_ref-- )
    {
        x264_cabac_encode_decision( cb, 54 + ctx, 1 );
        ctx = (ctx>>2)+4;
    }
    x264_cabac_encode_decision( cb, 54 + ctx, 0 );
}

/*****************************************************************
 * For the syntax elements mvd_l0[ ][ ][ ] and mvd_l1[ ][ ][ ],  *
 * a UEG3 binarization is used (k is equal to 3).                *
 *                                                               *
 * A UEGk bin string is a concatenation of a prefix bit string   *
 * and a suffix bit string. The prefix of the binarization is    *
 * specified by invoking the TU binarization process for the     *
 * prefix part Min( uCoff, Abs( synElVal ) ) of a syntax element *
 * value synElVal as specified in clause 9.3.2.2 with            *
 * cMax = uCoff, where uCoff > 0.                                *
 *****************************************************************/
static ALWAYS_INLINE int x264_cabac_mvd_cpn( x264_t *h, x264_cabac_t *cb, int i_list, int idx, int l, int mvd, int ctx )
{
    int ctxbase = l ? 47 : 40;
	int i, i_abs;
	static const uint8_t ctxes[8] = { 3,4,5,6,6,6,6,6 };

    if( mvd == 0 )
    {
        x264_cabac_encode_decision( cb, ctxbase + ctx, 0 );
        return 0;
    }

    i_abs = abs( mvd );
    x264_cabac_encode_decision( cb, ctxbase + ctx, 1 );

    /* prefix and suffix as given by UEG3 with signedValFlag=1, uCoff=9 */
    if( i_abs < 9 )
    {
        for( i = 1; i < i_abs; i++ )
            x264_cabac_encode_decision( cb, ctxbase + ctxes[i-1], 1 );
        x264_cabac_encode_decision( cb, ctxbase + ctxes[i_abs-1], 0 );
    }
    else
    {
        for( i = 1; i < 9; i++ )
            x264_cabac_encode_decision( cb, ctxbase + ctxes[i-1], 1 );
        x264_cabac_encode_ue_bypass( cb, 3, i_abs - 9 );
    }
    x264_cabac_encode_bypass( cb, mvd >> 31 );

    /* Since we don't need to keep track of MVDs larger than 66, just cap the value.
     * This lets us store MVDs as 8-bit values instead of 16-bit. */
    return X264_MIN( i_abs, 66 );
}

static NOINLINE void x264_cabac_mvd( x264_t *h, x264_cabac_t *cb, int i_list, int idx, int width, int height )
{
    ALIGNED_4( int16_t mvp[2] );
    int mdx, mdy, amvd0, amvd1;
    uint8_t * mvdleft, * mvdtop;

    /* Calculate mvd */
    x264_mb_predict_mv( h, i_list, idx, width, mvp );
    mdx = h->mb.cache.mv[i_list][x264_scan8[idx]][0] - mvp[0];
    mdy = h->mb.cache.mv[i_list][x264_scan8[idx]][1] - mvp[1];
    /* Calculate cabac context index offset of mvd by neighbour mv */
    mvdleft = h->mb.cache.mvd[i_list][x264_scan8[idx] - 1];
    mvdtop  = h->mb.cache.mvd[i_list][x264_scan8[idx] - 8];
    amvd0 = mvdleft[0] + mvdtop[0];
    amvd1 = mvdleft[1] + mvdtop[1];
    amvd0 = (amvd0 > 2) + (amvd0 > 32);
    amvd1 = (amvd1 > 2) + (amvd1 > 32);

    /* encode */
    mdx = x264_cabac_mvd_cpn( h, cb, i_list, idx, 0, mdx, amvd0 );
    mdy = x264_cabac_mvd_cpn( h, cb, i_list, idx, 1, mdy, amvd1 );
    x264_macroblock_cache_mvd( h, block_idx_x[idx], block_idx_y[idx], width, height, i_list, pack8to16(mdx, mdy) );
}

static ALWAYS_INLINE void x264_cabac_mb_header_i( x264_t *h, x264_cabac_t *cb, int i_mb_type, int slice_type )
{
    if( slice_type == SLICE_TYPE_I )
    {
        int ctx = 0;
        if( (h->mb.i_neighbour & MB_LEFT) && h->mb.i_mb_type_left[0] != I_4x4 )
            ctx++;
        if( (h->mb.i_neighbour & MB_TOP) && h->mb.i_mb_type_top != I_4x4 )
            ctx++;

        x264_cabac_mb_type_intra( h, cb, i_mb_type, 3+ctx, 3+3, 3+4, 3+5, 3+6, 3+7 );
    }
    else if( slice_type == SLICE_TYPE_P )
    {
        /* prefix */
        x264_cabac_encode_decision_noup( cb, 14, 1 );

        /* suffix */
        x264_cabac_mb_type_intra( h, cb, i_mb_type, 17+0, 17+1, 17+2, 17+2, 17+3, 17+3 );
    }

    if( i_mb_type != I_16x16 )
    {
		int i;
        for( i = 0; i < 16; i++ )
        {
            const int i_pred = x264_mb_predict_intra4x4_mode( h, i );
            const int i_mode = x264_mb_pred_mode4x4_fix( h->mb.cache.intra4x4_pred_mode[x264_scan8[i]] );
            x264_cabac_intra4x4_pred_mode( cb, i_pred, i_mode );
        }
    }

    x264_cabac_intra_chroma_pred_mode( h, cb );
}

/*************************************************************************
 * <ITU-T-REC-H.264-201201.pdf> Page 253                                 *
 * Table 9-37 ¨C Binarization for macroblock types in P, SP, and B slices *
 * Value (name) of mb_type | Bin string                                  *
 * 0 (P_L0_16x16)          | 0 0 0                                       *
 * 1 (P_L0_L0_16x8)        | 0 1 1                                       *
 * 2 (P_L0_L0_8x16)        | 0 1 0                                       *
 * 3 (P_8x8)               | 0 0 1                                       *
 *************************************************************************/
static ALWAYS_INLINE void x264_cabac_mb_header_p( x264_t *h, x264_cabac_t *cb, int i_mb_type )
{
    if( i_mb_type == P_L0 )
    {
        x264_cabac_encode_decision_noup( cb, 14, 0 );
        if( h->mb.i_partition == D_16x16 )
        {
        	/* (P_L0_16x16) 0 0 0 */
            x264_cabac_encode_decision_noup( cb, 15, 0 );
            x264_cabac_encode_decision_noup( cb, 16, 0 );
            if( h->mb.pic.i_fref[0] > 1 )
                x264_cabac_ref_p( h, cb, 0 );
            x264_cabac_mvd( h, cb, 0, 0, 4, 4 );
        }
        else if( h->mb.i_partition == D_16x8 )
        {
            x264_cabac_encode_decision_noup( cb, 15, 1 );
            x264_cabac_encode_decision_noup( cb, 17, 1 );
            if( h->mb.pic.i_fref[0] > 1 )
            {
                x264_cabac_ref_p( h, cb, 0 );
                x264_cabac_ref_p( h, cb, 8 );
            }
            x264_cabac_mvd( h, cb, 0, 0, 4, 2 );
            x264_cabac_mvd( h, cb, 0, 8, 4, 2 );
        }
        else //if( h->mb.i_partition == D_8x16 )
        {
            x264_cabac_encode_decision_noup( cb, 15, 1 );
            x264_cabac_encode_decision_noup( cb, 17, 0 );
            if( h->mb.pic.i_fref[0] > 1 )
            {
                x264_cabac_ref_p( h, cb, 0 );
                x264_cabac_ref_p( h, cb, 4 );
            }
            x264_cabac_mvd( h, cb, 0, 0, 2, 4 );
            x264_cabac_mvd( h, cb, 0, 4, 2, 4 );
        }
    }
    else if( i_mb_type == P_8x8 )
    {
        x264_cabac_encode_decision_noup( cb, 14, 0 );
        x264_cabac_encode_decision_noup( cb, 15, 0 );
        x264_cabac_encode_decision_noup( cb, 16, 1 );

        /* sub mb type == D_L0_8x8 */
        x264_cabac_encode_decision( cb, 21, 1 );
        x264_cabac_encode_decision( cb, 21, 1 );
        x264_cabac_encode_decision( cb, 21, 1 );
        x264_cabac_encode_decision( cb, 21, 1 );

        /* ref 0 */
        if( h->mb.pic.i_fref[0] > 1 )
        {
            x264_cabac_ref_p( h, cb,  0 );
            x264_cabac_ref_p( h, cb,  4 );
            x264_cabac_ref_p( h, cb,  8 );
            x264_cabac_ref_p( h, cb, 12 );
        }

        x264_cabac_mvd( h, cb, 0, 0, 2, 2 );
        x264_cabac_mvd( h, cb, 0, 4, 2, 2 );
        x264_cabac_mvd( h, cb, 0, 8, 2, 2 );
        x264_cabac_mvd( h, cb, 0, 12, 2, 2 );
    }
    else /* intra */
        x264_cabac_mb_header_i( h, cb, i_mb_type, SLICE_TYPE_P );
}

/* x264_cabac_cbf_ctxidxinc calculates cabac context index offset of coded block flag by neighbour nnz */
static int ALWAYS_INLINE x264_cabac_cbf_ctxidxinc( x264_t *h, int i_cat, int i_idx, int b_intra, int b_dc )
{
    static const uint16_t base_ctx[14] = {85,89,93,97,101,1012,460,464,468,1016,472,476,480,1020};

    if( b_dc )
    {
        i_idx -= LUMA_DC;
        if( i_cat == DCT_CHROMA_DC )
        {
            int i_nza = h->mb.cache.i_cbp_left != -1 ? (h->mb.cache.i_cbp_left >> (8 + i_idx)) & 1 : b_intra;
            int i_nzb = h->mb.cache.i_cbp_top  != -1 ? (h->mb.cache.i_cbp_top  >> (8 + i_idx)) & 1 : b_intra;
            return base_ctx[i_cat] + (i_nzb<<1) + i_nza;
        }
        else
        {
            int i_nza = (h->mb.cache.i_cbp_left >> (8 + i_idx)) & 1;
            int i_nzb = (h->mb.cache.i_cbp_top  >> (8 + i_idx)) & 1;
            return base_ctx[i_cat] + (i_nzb<<1) + i_nza;
        }
    }
    else
    {
        int i_nza = h->mb.cache.non_zero_count[x264_scan8[i_idx] - 1];
        int i_nzb = h->mb.cache.non_zero_count[x264_scan8[i_idx] - 8];
        if( x264_constant_p(b_intra) && !b_intra )
            return base_ctx[i_cat] + (((i_nzb<<1) + i_nza)&0x7f);
        else
        {
            i_nza &= 0x7f + (b_intra << 7);
            i_nzb &= 0x7f + (b_intra << 7);
            return base_ctx[i_cat] + ((!!i_nzb)<<1) + !!i_nza;
        }
    }
}

/**********************************************************************
 * 14 context index offset is ordered by block category:              *
 * LUMA_DC, LUMA_AC, LUMA_4x4, CHROMA_DC, CHROMA_AC,                  *
 * LUMA_8x8, (8x8 transform enabled only)                             *
 * CHROMAU_DC, CHROMAU_AC, CHROMAU_4x4, CHROMAU_8x8, (Chroma444 only) *
 * CHROMAV_DC, CHROMAV_AC, CHROMAV_4x4, CHROMAV_8x8  (Chroma444 only) *
 **********************************************************************/
static const uint16_t significant_coeff_flag_offset[14] =
{
    105+0, 105+15, 105+29, 105+44, 105+47, 402, 484+0, 484+15, 484+29, 660, 528+0, 528+15, 528+29, 718
};
static const uint16_t last_coeff_flag_offset[14] =
{
    166+0, 166+15, 166+29, 166+44, 166+47, 417, 572+0, 572+15, 572+29, 690, 616+0, 616+15, 616+29, 748
};
static const uint16_t coeff_abs_level_m1_offset[14] =
{
    227+0, 227+10, 227+20, 227+30, 227+39, 426, 952+0, 952+10, 952+20, 708, 982+0, 982+10, 982+20, 766
};
static const uint8_t count_cat_m1[14] =
{
	15, 14, 15, 3, 14, 63, 15, 14, 15, 63, 15, 14, 15, 63
};

// node ctx: 0..3: abslevel1 (with abslevelgt1 == 0).
//           4..7: abslevelgt1 + 3 (and abslevel1 doesn't matter).
/* map node ctx => cabac ctx for level=1 */
static const uint8_t coeff_abs_level1_ctx[8] = { 1, 2, 3, 4, 0, 0, 0, 0 };
/* map node ctx => cabac ctx for level>1 */
static const uint8_t coeff_abs_levelgt1_ctx[8] = { 5, 5, 5, 5, 6, 7, 8, 9 };
static const uint8_t coeff_abs_level_transition[2][8] =
{
/* update node ctx after coding a level=1 */
    { 1, 2, 3, 3, 4, 5, 6, 7 },
/* update node ctx after coding a level>1 */
    { 4, 4, 4, 4, 5, 6, 7, 7 }
};

static void x264_cabac_block_residual( x264_t *h, x264_cabac_t *cb, int ctx_block_cat, dctcoef *l )
{
    int ctx_sig   = significant_coeff_flag_offset[ctx_block_cat];
    int ctx_last  = last_coeff_flag_offset[ctx_block_cat];
    int ctx_level = coeff_abs_level_m1_offset[ctx_block_cat];
    int count_m1  = count_cat_m1[ctx_block_cat];
    int coeff_idx = -1, node_ctx = 0;
    int last      = h->quantf.coeff_last[ctx_block_cat]( l );
    const uint8_t *levelgt1_ctx = coeff_abs_levelgt1_ctx;
    dctcoef coeffs[64];
    int i = 0;

    /* write significant coeff flags and last coeff flag */
	while( 1 )
	{
		if( l[i] )
		{
			coeffs[++coeff_idx] = l[i];
			x264_cabac_encode_decision_c( cb, ctx_sig + i, 1 );
			if( i == last )
			{
				x264_cabac_encode_decision_c( cb, ctx_last + i, 1 );
				break;
			}
			else
				x264_cabac_encode_decision_c( cb, ctx_last + i, 0 );
		}
		else
			x264_cabac_encode_decision_c( cb, ctx_sig + i, 0 );
		if( ++i == count_m1 )
		{
			coeffs[++coeff_idx] = l[i];
			break;
		}
	}

	/* write coeff abs values and coeff sign flags */
    do
    {
        /* write coeff_abs - 1 */
        int coeff = coeffs[coeff_idx];
        int abs_coeff = abs(coeff);
        int coeff_sign = coeff >> 31;
        int ctx = coeff_abs_level1_ctx[node_ctx] + ctx_level;

        if( abs_coeff > 1 )
        {
            x264_cabac_encode_decision( cb, ctx, 1 );
            ctx = levelgt1_ctx[node_ctx] + ctx_level;
            for( i = X264_MIN( abs_coeff, 15 ) - 2; i > 0; i-- )
                x264_cabac_encode_decision( cb, ctx, 1 );
            if( abs_coeff < 15 )
                x264_cabac_encode_decision( cb, ctx, 0 );
            else
                x264_cabac_encode_ue_bypass( cb, 0, abs_coeff - 15 );

            node_ctx = coeff_abs_level_transition[1][node_ctx];
        }
        else
        {
            x264_cabac_encode_decision( cb, ctx, 0 );
            node_ctx = coeff_abs_level_transition[0][node_ctx];
        }

        /* write coeff sign flag after coeff_abs */
        x264_cabac_encode_bypass( cb, coeff_sign );
    } while( --coeff_idx >= 0 );
}

#define x264_cabac_block_residual_cbf( h, cb, ctx_block_cat, i_idx, l, b_intra, b_dc )\
do\
{\
    int ctxidxinc = x264_cabac_cbf_ctxidxinc( h, ctx_block_cat, i_idx, b_intra, b_dc );\
    if( h->mb.cache.non_zero_count[x264_scan8[i_idx]] )\
    {\
        x264_cabac_encode_decision( cb, ctxidxinc, 1 );\
        x264_cabac_block_residual( h, cb, ctx_block_cat, l );\
    }\
    else\
        x264_cabac_encode_decision( cb, ctxidxinc, 0 );\
} while(0)

void x264_macroblock_write_cabac( x264_t *h, x264_cabac_t *cb )
{
    const int i_mb_type = h->mb.i_type;
    const int i_mb_pos_start = x264_cabac_pos( cb );
    int       i_mb_pos_tex;
    int       i;

    if( h->sh.i_type == SLICE_TYPE_P )
        x264_cabac_mb_header_p( h, cb, i_mb_type );
    else //if( h->sh.i_type == SLICE_TYPE_I )
        x264_cabac_mb_header_i( h, cb, i_mb_type, SLICE_TYPE_I );

    i_mb_pos_tex = x264_cabac_pos( cb );
    h->stat.frame.i_mv_bits += i_mb_pos_tex - i_mb_pos_start;

    if( i_mb_type != I_16x16 )
    {
        x264_cabac_cbp_luma( h, cb );
        x264_cabac_cbp_chroma( h, cb );
    }

    if( h->mb.i_cbp_luma || h->mb.i_cbp_chroma || i_mb_type == I_16x16 )
    {
        const int b_intra = IS_INTRA( i_mb_type );
        x264_cabac_qp_delta( h, cb );

        /* write luma residual */
        if( i_mb_type == I_16x16 )
        {
            /* DC Luma */
        	x264_cabac_block_residual_cbf( h, cb, DCT_LUMA_DC, LUMA_DC, h->dct.luma16x16_dc[0], 1, 1 );
        	/* AC Luma */
        	if( h->mb.i_cbp_luma )
        		for( i = 0; i < 16; i++ )
        			x264_cabac_block_residual_cbf( h, cb, DCT_LUMA_AC, i, h->dct.luma4x4[i]+1, 1, 0 );
        }
        else
        {
        	for( i = 0; i < 16; i++ )
        		if( h->mb.i_cbp_luma & ( 1 << ( i >> 2 ) ) )
        			x264_cabac_block_residual_cbf( h, cb, DCT_LUMA_4x4, i, h->dct.luma4x4[i], b_intra, 0 );
        }

        /* write chroma residual */
        if( h->mb.i_cbp_chroma ) /* Chroma DC residual present */
        {
        	x264_cabac_block_residual_cbf( h, cb, DCT_CHROMA_DC, CHROMA_DC+0, h->dct.chroma_dc[0], b_intra, 1 );
        	x264_cabac_block_residual_cbf( h, cb, DCT_CHROMA_DC, CHROMA_DC+1, h->dct.chroma_dc[1], b_intra, 1 );

            if( h->mb.i_cbp_chroma == 2 ) /* Chroma AC residual present */
            {
            	/* write chroma u and v residual separately */
            	for( i = 0; i < 4; i++ )
            		x264_cabac_block_residual_cbf( h, cb, DCT_CHROMA_AC, 16+i, h->dct.luma4x4[16+i]+1, b_intra, 0 );
            	for( i = 0; i < 4; i++ )
            		x264_cabac_block_residual_cbf( h, cb, DCT_CHROMA_AC, 32+i, h->dct.luma4x4[32+i]+1, b_intra, 0 );
            }
        }
    }

    h->stat.frame.i_tex_bits += x264_cabac_pos( cb ) - i_mb_pos_tex;
}
