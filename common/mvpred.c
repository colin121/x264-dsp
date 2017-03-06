/*****************************************************************************
 * mvpred.c: motion vector prediction
 *****************************************************************************/

#include "common.h"

/**********************************************************************
 * motion vector prediction                                           *
 *                                                                    *
 *        ----- -----                                                 *
 *       |  B  |  C  |    B: top block, C: top-right block            *
 *  ----- ----- -----                                                 *
 * |  A  |  E  |          A: left block, E: current block             *
 *  ----- -----                                                       *
 *                                                                    *
 * if current mb partition is:                                        *
 * 1. neither 16x8 nor 8x16, MVp is median of A, B, C                 *
 * 2. 16x8, the top part is equal to B, the bottom part is equal to A *
 * 3. 8x16, the left part is equal to A, the right part is equal to C *
 * 4. skip, MVp is median of A, B, C                                  *
 **********************************************************************/
void x264_mb_predict_mv( x264_t *h, int i_list, int idx, int i_width, int16_t mvp[2] )
{
    const int i8 = x264_scan8[idx];
    const int i_ref= h->mb.cache.ref[i_list][i8];
    int     i_refa = h->mb.cache.ref[i_list][i8 - 1];
    int16_t *mv_a  = h->mb.cache.mv[i_list][i8 - 1];
    int     i_refb = h->mb.cache.ref[i_list][i8 - 8];
    int16_t *mv_b  = h->mb.cache.mv[i_list][i8 - 8];
    int     i_refc = h->mb.cache.ref[i_list][i8 - 8 + i_width];
    int16_t *mv_c  = h->mb.cache.mv[i_list][i8 - 8 + i_width];
	int i_count;

    // Partitions not yet reached in scan order are unavailable.
	/* if top-right mb is unavailable, use ref and mv of top-left mb instead. */
    if( (idx&3) >= 2 + (i_width&1) || i_refc == -2 )
    {
        i_refc = h->mb.cache.ref[i_list][i8 - 8 - 1];
        mv_c   = h->mb.cache.mv[i_list][i8 - 8 - 1];
    }
    if( h->mb.i_partition == D_16x8 )
    {
        if( idx == 0 )
        {
            if( i_refb == i_ref )
            {
                CP32( mvp, mv_b );
                return;
            }
        }
        else
        {
            if( i_refa == i_ref )
            {
                CP32( mvp, mv_a );
                return;
            }
        }
    }
    else if( h->mb.i_partition == D_8x16 )
    {
        if( idx == 0 )
        {
            if( i_refa == i_ref )
            {
                CP32( mvp, mv_a );
                return;
            }
        }
        else
        {
            if( i_refc == i_ref )
            {
                CP32( mvp, mv_c );
                return;
            }
        }
    }

    i_count = (i_refa == i_ref) + (i_refb == i_ref) + (i_refc == i_ref);

    if( i_count > 1 )
    {
median:
        x264_median_mv( mvp, mv_a, mv_b, mv_c );
    }
    else if( i_count == 1 )
    {
        if( i_refa == i_ref )
            CP32( mvp, mv_a );
        else if( i_refb == i_ref )
            CP32( mvp, mv_b );
        else
            CP32( mvp, mv_c );
    }
    else if( i_refb == -2 && i_refc == -2 && i_refa != -2 )
        CP32( mvp, mv_a );
    else
        goto median;
}

/* motion vector prediction only for 16x16 */
void x264_mb_predict_mv_16x16( x264_t *h, int i_list, int i_ref, int16_t mvp[2] )
{
    int     i_refa = h->mb.cache.ref[i_list][X264_SCAN8_0 - 1];
    int16_t *mv_a  = h->mb.cache.mv[i_list][X264_SCAN8_0 - 1];
    int     i_refb = h->mb.cache.ref[i_list][X264_SCAN8_0 - 8];
    int16_t *mv_b  = h->mb.cache.mv[i_list][X264_SCAN8_0 - 8];
    int     i_refc = h->mb.cache.ref[i_list][X264_SCAN8_0 - 8 + 4];
    int16_t *mv_c  = h->mb.cache.mv[i_list][X264_SCAN8_0 - 8 + 4];
	int i_count;

    /* if top-right mb is unavailable, use ref and mv of top-left mb instead. */
	if( i_refc == -2 )
    {
        i_refc = h->mb.cache.ref[i_list][X264_SCAN8_0 - 8 - 1];
        mv_c   = h->mb.cache.mv[i_list][X264_SCAN8_0 - 8 - 1];
    }

    i_count = (i_refa == i_ref) + (i_refb == i_ref) + (i_refc == i_ref);

    if( i_count > 1 )
    {
median:
        x264_median_mv( mvp, mv_a, mv_b, mv_c );
    }
    else if( i_count == 1 )
    {
        if( i_refa == i_ref )
            CP32( mvp, mv_a );
        else if( i_refb == i_ref )
            CP32( mvp, mv_b );
        else
            CP32( mvp, mv_c );
    }
    else if( i_refb == -2 && i_refc == -2 && i_refa != -2 )
        CP32( mvp, mv_a );
    else
        goto median;
}

/* motion vector prediction only for pskip */
void x264_mb_predict_mv_pskip( x264_t *h, int16_t mv[2] )
{
    int     i_refa = h->mb.cache.ref[0][X264_SCAN8_0 - 1];
    int     i_refb = h->mb.cache.ref[0][X264_SCAN8_0 - 8];
    int16_t *mv_a  = h->mb.cache.mv[0][X264_SCAN8_0 - 1];
    int16_t *mv_b  = h->mb.cache.mv[0][X264_SCAN8_0 - 8];

    if( i_refa == -2 || i_refb == -2 ||
        !( i_refa | M32( mv_a ) ) ||
        !( i_refb | M32( mv_b ) ) )
    {
        M32( mv ) = 0;
    }
    else
        x264_mb_predict_mv_16x16( h, 0, 0, mv );
}

/*************************************************************************
 * x264_mb_predict_mv_ref16x16 is called by x264_mb_analyse_inter_p16x16 *
 * This function finds all possible mv candidates and save them to mvc   *
 * array, number of mvc is also saved to i_mvc.                          *
 *                                                                       *
 * This just improves encoder performance, it's not part of the spec.    *
 *************************************************************************/
void x264_mb_predict_mv_ref16x16( x264_t *h, int i_list, int i_ref, int16_t mvc[9][2], int *i_mvc )
{
    int16_t (*mvr)[2] = h->mb.mvr[i_list][i_ref];
    int i = 0;

    if( i_ref == 0 && h->frames.b_have_lowres )
    {
        int idx = i_list ? h->fref[1][0]->i_frame-h->fenc->i_frame-1
                         : h->fenc->i_frame-h->fref[0][0]->i_frame-1;
        if( idx <= h->param.i_bframe )
        {
            int16_t (*lowres_mv)[2] = h->fenc->lowres_mvs[i_list][idx];
            if( lowres_mv[0][0] != 0x7fff )
            {
                M32( mvc[i++] ) = (M32( lowres_mv[h->mb.i_mb_xy] )*2)&0xfffeffff;
            }
        }
    }

    /* spatial predictors */
    CP32( mvc[i++], mvr[h->mb.i_mb_left_xy[0]] );
    CP32( mvc[i++], mvr[h->mb.i_mb_top_xy] );
    CP32( mvc[i++], mvr[h->mb.i_mb_topleft_xy] );
    CP32( mvc[i++], mvr[h->mb.i_mb_topright_xy] );

    /* temporal predictors */
    if( h->fref[0][0]->i_ref[0] > 0 )
    {
        x264_frame_t *l0 = h->fref[0][0];
        int field = h->mb.i_mb_y&1;
        int curpoc = h->fdec->i_poc + h->fdec->i_delta_poc[field];
        int refpoc = h->fref[i_list][i_ref>>SLICE_MBAFF]->i_poc;
        refpoc += l0->i_delta_poc[field^(i_ref&1)];

#define SET_TMVP( dx, dy ) \
        { \
            int mb_index = h->mb.i_mb_xy + dx + dy*h->mb.i_mb_stride; \
            int scale = (curpoc - refpoc) * l0->inv_ref_poc[MB_INTERLACED&field]; \
            mvc[i][0] = (l0->mv16x16[mb_index][0]*scale + 128) >> 8; \
            mvc[i][1] = (l0->mv16x16[mb_index][1]*scale + 128) >> 8; \
            i++; \
        }

        SET_TMVP(0,0);
        if( h->mb.i_mb_x < h->mb.i_mb_width-1 )
            SET_TMVP(1,0);
        if( h->mb.i_mb_y < h->mb.i_mb_height-1 )
            SET_TMVP(0,1);
#undef SET_TMVP
    }

    *i_mvc = i;
}
