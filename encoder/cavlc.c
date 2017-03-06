/*****************************************************************************
 * cavlc.c: cavlc bitstream writing
 *****************************************************************************/

#include "common/common.h"
#include "macroblock.h"

/* [400,420][inter,intra] */
static const uint8_t cbp_to_golomb[2][2][48] =
{
    {{ 0,  1,  2,  5,  3,  6, 14, 10,  4, 15,  7, 11,  8, 12, 13,  9 },
     { 1, 10, 11,  6, 12,  7, 14,  2, 13, 15,  8,  3,  9,  4,  5,  0 }},
    {{ 0,  2,  3,  7,  4,  8, 17, 13,  5, 18,  9, 14, 10, 15, 16, 11,
       1, 32, 33, 36, 34, 37, 44, 40, 35, 45, 38, 41, 39, 42, 43, 19,
       6, 24, 25, 20, 26, 21, 46, 28, 27, 47, 22, 29, 23, 30, 31, 12 },
     { 3, 29, 30, 17, 31, 18, 37,  8, 32, 38, 19,  9, 20, 10, 11,  2,
      16, 33, 34, 21, 35, 22, 39,  4, 36, 40, 23,  5, 24,  6,  7,  1,
      41, 42, 43, 25, 44, 26, 46, 12, 45, 47, 27, 13, 28, 14, 15,  0 }}
};

#define bs_write_vlc(s,v) bs_write( s, (v).i_size, (v).i_bits )

/****************************************************************************
 * x264_cavlc_block_residual:
 ****************************************************************************/
static inline int x264_cavlc_block_residual_escape( x264_t *h, int i_suffix_length, int level )
{
    bs_t *s = &h->out.bs;
    static const uint16_t next_suffix[7] = { 0, 3, 6, 12, 24, 48, 0xffff };
    int i_level_prefix = 15;
    int mask = level >> 31;
    int abs_level = (level^mask)-mask;
    int i_level_code = abs_level*2-mask-2;
    if( ( i_level_code >> i_suffix_length ) < 15 )
    {
        bs_write( s, (i_level_code >> i_suffix_length) + 1 + i_suffix_length,
                 (1<<i_suffix_length) + (i_level_code & ((1<<i_suffix_length)-1)) );
    }
    else
    {
        i_level_code -= 15 << i_suffix_length;
        if( i_suffix_length == 0 )
            i_level_code -= 15;

        /* If the prefix size exceeds 15, High Profile is required. */
        if( i_level_code >= 1<<12 )
        {
            if( h->sps->i_profile_idc >= PROFILE_HIGH )
            {
                while( i_level_code > 1<<(i_level_prefix-3) )
                {
                    i_level_code -= 1<<(i_level_prefix-3);
                    i_level_prefix++;
                }
            }
            else
            {
                /* We've had an overflow; note it down and re-encode the MB later. */
                h->mb.b_overflow = 1;
            }
        }
        bs_write( s, i_level_prefix + 1, 1 );
        bs_write( s, i_level_prefix - 3, i_level_code & ((1<<(i_level_prefix-3))-1) );
    }
    if( i_suffix_length == 0 )
        i_suffix_length++;
    if( abs_level > next_suffix[i_suffix_length] )
        i_suffix_length++;
    return i_suffix_length;
}

static int x264_cavlc_block_residual_internal( x264_t *h, int ctx_block_cat, dctcoef *l, int nC )
{
    bs_t *s = &h->out.bs;
    static const uint8_t ctz_index[8] = {3,0,1,0,2,0,1,0};
    static const uint8_t count_cat[14] = {16, 15, 16, 0, 15, 64, 16, 15, 16, 64, 16, 15, 16, 64};
    x264_run_level_t runlevel;
    int i_total, i_trailing, i_total_zero, i_suffix_length, i;
    unsigned int i_sign;
	int zero_run_code;

    /* level and run and total */
    /* set these to 2 to allow branchless i_trailing calculation */
    runlevel.level[1] = 2;
    runlevel.level[2] = 2;
    i_total = h->quantf.coeff_level_run[ctx_block_cat]( l, &runlevel );
    x264_prefetch( &x264_run_before[runlevel.mask] );
    i_total_zero = runlevel.last + 1 - i_total;

    i_trailing = ((((runlevel.level[0]+1) | (1-runlevel.level[0])) >> 31) & 1) // abs(runlevel.level[0])>1
               | ((((runlevel.level[1]+1) | (1-runlevel.level[1])) >> 31) & 2)
               | ((((runlevel.level[2]+1) | (1-runlevel.level[2])) >> 31) & 4);
    i_trailing = ctz_index[i_trailing];
    i_sign = ((runlevel.level[2] >> 31) & 1)
           | ((runlevel.level[1] >> 31) & 2)
           | ((runlevel.level[0] >> 31) & 4);
    i_sign >>= 3-i_trailing;

    /* total/trailing */
    bs_write_vlc( s, x264_coeff_token[nC][i_total-1][i_trailing] );

    i_suffix_length = i_total > 10 && i_trailing < 3;
    bs_write( s, i_trailing, i_sign );

    if( i_trailing < i_total )
    {
        int val = runlevel.level[i_trailing];
        int val_original = runlevel.level[i_trailing]+LEVEL_TABLE_SIZE/2;
        val -= ((val>>31)|1) & -(i_trailing < 3); /* as runlevel.level[i] can't be 1 for the first one if i_trailing < 3 */
        val += LEVEL_TABLE_SIZE/2;

        if( (unsigned)val_original < LEVEL_TABLE_SIZE )
        {
            bs_write_vlc( s, x264_level_token[i_suffix_length][val] );
            i_suffix_length = x264_level_token[i_suffix_length][val_original].i_next;
        }
        else
            i_suffix_length = x264_cavlc_block_residual_escape( h, i_suffix_length, val-LEVEL_TABLE_SIZE/2 );
        for( i = i_trailing+1; i < i_total; i++ )
        {
            val = runlevel.level[i] + LEVEL_TABLE_SIZE/2;
            if( (unsigned)val < LEVEL_TABLE_SIZE )
            {
                bs_write_vlc( s, x264_level_token[i_suffix_length][val] );
                i_suffix_length = x264_level_token[i_suffix_length][val].i_next;
            }
            else
                i_suffix_length = x264_cavlc_block_residual_escape( h, i_suffix_length, val-LEVEL_TABLE_SIZE/2 );
        }
    }

    if( ctx_block_cat == DCT_CHROMA_DC )
    {
        if( i_total < 8>>CHROMA_V_SHIFT )
        {
            vlc_t total_zeros = CHROMA_FORMAT == CHROMA_420 ? x264_total_zeros_2x2_dc[i_total-1][i_total_zero]
                                                            : x264_total_zeros_2x4_dc[i_total-1][i_total_zero];
            bs_write_vlc( s, total_zeros );
        }
    }
    else if( (uint8_t)i_total < count_cat[ctx_block_cat] )
        bs_write_vlc( s, x264_total_zeros[i_total-1][i_total_zero] );

    zero_run_code = x264_run_before[runlevel.mask];
    bs_write( s, zero_run_code&0x1f, zero_run_code>>5 );

    return i_total;
}

static const uint8_t ct_index[17] = {0,0,1,1,2,2,2,2,3,3,3,3,3,3,3,3,3};

#define x264_cavlc_block_residual(h,cat,idx,l)\
{\
    int nC = cat == DCT_CHROMA_DC ? 5 - CHROMA_V_SHIFT\
                                  : ct_index[x264_mb_predict_non_zero_code( h, cat == DCT_LUMA_DC ? (idx - LUMA_DC)*16 : idx )];\
    uint8_t *nnz = &h->mb.cache.non_zero_count[x264_scan8[idx]];\
    if( !*nnz )\
        bs_write_vlc( &h->out.bs, x264_coeff0_token[nC] );\
    else\
        *nnz = x264_cavlc_block_residual_internal(h,cat,l,nC);\
}

static void x264_cavlc_qp_delta( x264_t *h )
{
    bs_t *s = &h->out.bs;
    int i_dqp = h->mb.i_qp - h->mb.i_last_qp;

    /* Avoid writing a delta quant if we have an empty i16x16 block, e.g. in a completely flat background area */
    if( h->mb.i_type == I_16x16 && !(h->mb.i_cbp_luma | h->mb.i_cbp_chroma)
        && !h->mb.cache.non_zero_count[x264_scan8[LUMA_DC]]
        && !h->mb.cache.non_zero_count[x264_scan8[CHROMA_DC+0]]
        && !h->mb.cache.non_zero_count[x264_scan8[CHROMA_DC+1]] )
    {
        h->mb.i_qp = h->mb.i_last_qp;
        i_dqp = 0;
    }

    if( i_dqp )
    {
        if( i_dqp < -(QP_MAX_SPEC+1)/2 )
            i_dqp += QP_MAX_SPEC+1;
        else if( i_dqp > QP_MAX_SPEC/2 )
            i_dqp -= QP_MAX_SPEC+1;
    }
    bs_write_se( s, i_dqp );
}

static void x264_cavlc_mvd( x264_t *h, int i_list, int idx, int width )
{
    bs_t *s = &h->out.bs;
    ALIGNED_4( int16_t mvp[2] );
    x264_mb_predict_mv( h, i_list, idx, width, mvp );
    bs_write_se( s, h->mb.cache.mv[i_list][x264_scan8[idx]][0] - mvp[0] );
    bs_write_se( s, h->mb.cache.mv[i_list][x264_scan8[idx]][1] - mvp[1] );
}

static inline void x264_cavlc_macroblock_luma_residual( x264_t *h, int i8start, int i8end )
{
	int i8, i4;
    for( i8 = i8start; i8 <= i8end; i8++ )
        if( h->mb.i_cbp_luma & (1 << (i8&3)) )
            for( i4 = 0; i4 < 4; i4++ )
                x264_cavlc_block_residual( h, DCT_LUMA_4x4, i4+i8*4, h->dct.luma4x4[i4+i8*4] );
}

static void x264_cavlc_mb_header_i( x264_t *h, int i_mb_type, int i_mb_i_offset )
{
    bs_t *s = &h->out.bs;
    if( i_mb_type == I_16x16 )
    {
        bs_write_ue( s, i_mb_i_offset + 1 + x264_mb_pred_mode16x16_fix[h->mb.i_intra16x16_pred_mode] +
                        h->mb.i_cbp_chroma * 4 + ( h->mb.i_cbp_luma == 0 ? 0 : 12 ) );
    }
    else //if( i_mb_type == I_4x4 )
    {
		int i;
        bs_write_ue( s, i_mb_i_offset + 0 );

        /* Prediction: Luma */
        for( i = 0; i < 16; i++ )
        {
            int i_pred = x264_mb_predict_intra4x4_mode( h, i );
            int i_mode = x264_mb_pred_mode4x4_fix( h->mb.cache.intra4x4_pred_mode[x264_scan8[i]] );

            if( i_pred == i_mode )
                bs_write1( s, 1 );  /* b_prev_intra4x4_pred_mode */
            else
                bs_write( s, 4, i_mode - (i_mode > i_pred) );
        }
    }

    bs_write_ue( s, x264_mb_chroma_pred_mode_fix[h->mb.i_chroma_pred_mode] );
}

static ALWAYS_INLINE void x264_cavlc_mb_header_p( x264_t *h, int i_mb_type )
{
    bs_t *s = &h->out.bs;
    if( i_mb_type == P_L0 )
    {
        if( h->mb.i_partition == D_16x16 )
        {
            bs_write1( s, 1 );

            if( h->mb.pic.i_fref[0] > 1 )
                bs_write_te( s, h->mb.pic.i_fref[0] - 1, h->mb.cache.ref[0][x264_scan8[0]] );
            x264_cavlc_mvd( h, 0, 0, 4 );
        }
        else if( h->mb.i_partition == D_16x8 )
        {
            bs_write_ue( s, 1 );
            if( h->mb.pic.i_fref[0] > 1 )
            {
                bs_write_te( s, h->mb.pic.i_fref[0] - 1, h->mb.cache.ref[0][x264_scan8[0]] );
                bs_write_te( s, h->mb.pic.i_fref[0] - 1, h->mb.cache.ref[0][x264_scan8[8]] );
            }
            x264_cavlc_mvd( h, 0, 0, 4 );
            x264_cavlc_mvd( h, 0, 8, 4 );
        }
        else if( h->mb.i_partition == D_8x16 )
        {
            bs_write_ue( s, 2 );
            if( h->mb.pic.i_fref[0] > 1 )
            {
                bs_write_te( s, h->mb.pic.i_fref[0] - 1, h->mb.cache.ref[0][x264_scan8[0]] );
                bs_write_te( s, h->mb.pic.i_fref[0] - 1, h->mb.cache.ref[0][x264_scan8[4]] );
            }
            x264_cavlc_mvd( h, 0, 0, 2 );
            x264_cavlc_mvd( h, 0, 4, 2 );
        }
    }
    else if( i_mb_type == P_8x8 )
    {
        int b_sub_ref;
        if( (h->mb.cache.ref[0][x264_scan8[0]] | h->mb.cache.ref[0][x264_scan8[ 4]] |
             h->mb.cache.ref[0][x264_scan8[8]] | h->mb.cache.ref[0][x264_scan8[12]]) == 0 )
        {
            bs_write_ue( s, 4 );
            b_sub_ref = 0;
        }
        else
        {
            bs_write_ue( s, 3 );
            b_sub_ref = 1;
        }

        /* sub mb type == D_L0_8x8 */
        bs_write( s, 4, 0xf );

        /* ref0 */
        if( b_sub_ref )
        {
            bs_write_te( s, h->mb.pic.i_fref[0] - 1, h->mb.cache.ref[0][x264_scan8[0]] );
            bs_write_te( s, h->mb.pic.i_fref[0] - 1, h->mb.cache.ref[0][x264_scan8[4]] );
            bs_write_te( s, h->mb.pic.i_fref[0] - 1, h->mb.cache.ref[0][x264_scan8[8]] );
            bs_write_te( s, h->mb.pic.i_fref[0] - 1, h->mb.cache.ref[0][x264_scan8[12]] );
        }

        x264_cavlc_mvd( h, 0, 0, 2 );
        x264_cavlc_mvd( h, 0, 4, 2 );
        x264_cavlc_mvd( h, 0, 8, 2 );
        x264_cavlc_mvd( h, 0, 12, 2 );
    }
    else //if( IS_INTRA( i_mb_type ) )
        x264_cavlc_mb_header_i( h, i_mb_type, 5 );
}

/*****************************************************************************
 * x264_macroblock_write:
 *****************************************************************************/
void x264_macroblock_write_cavlc( x264_t *h )
{
    bs_t *s = &h->out.bs;
    const int i_mb_type = h->mb.i_type;
	int i;

    const int i_mb_pos_start = bs_pos( s );
    int       i_mb_pos_tex;

    if( h->sh.i_type == SLICE_TYPE_P )
        x264_cavlc_mb_header_p( h, i_mb_type );
    else //if( h->sh.i_type == SLICE_TYPE_I )
        x264_cavlc_mb_header_i( h, i_mb_type, 0 );

    i_mb_pos_tex = bs_pos( s );
    h->stat.frame.i_mv_bits += i_mb_pos_tex - i_mb_pos_start;

    /* Coded block pattern */
    if( i_mb_type != I_16x16 )
        bs_write_ue( s, cbp_to_golomb[1][IS_INTRA(i_mb_type)][(h->mb.i_cbp_chroma << 4)|h->mb.i_cbp_luma] );

    if( i_mb_type == I_16x16 )
    {
        x264_cavlc_qp_delta( h );

        /* DC Luma */
		x264_cavlc_block_residual( h, DCT_LUMA_DC, LUMA_DC, h->dct.luma16x16_dc[0] );

		/* AC Luma */
		if( h->mb.i_cbp_luma )
			for( i = 0; i < 16; i++ )
				x264_cavlc_block_residual( h, DCT_LUMA_AC, i, h->dct.luma4x4[i]+1 );
    }
    else if( h->mb.i_cbp_luma | h->mb.i_cbp_chroma )
    {
        x264_cavlc_qp_delta( h );
        x264_cavlc_macroblock_luma_residual( h, 0, 3 );
    }
    if( h->mb.i_cbp_chroma )
    {
        /* Chroma DC residual present */
        x264_cavlc_block_residual( h, DCT_CHROMA_DC, CHROMA_DC+0, h->dct.chroma_dc[0] );
        x264_cavlc_block_residual( h, DCT_CHROMA_DC, CHROMA_DC+1, h->dct.chroma_dc[1] );

        if( h->mb.i_cbp_chroma == 2 ) /* Chroma AC residual present */
        {
        	/* write chroma u and v residual separately */
        	for( i = 0; i < 4; i++ )
        		x264_cavlc_block_residual( h, DCT_CHROMA_AC, 16+i, h->dct.luma4x4[16+i]+1 );
        	for( i = 0; i < 4; i++ )
        		x264_cavlc_block_residual( h, DCT_CHROMA_AC, 32+i, h->dct.luma4x4[32+i]+1 );
        }
    }

    h->stat.frame.i_tex_bits += bs_pos(s) - i_mb_pos_tex;
}
