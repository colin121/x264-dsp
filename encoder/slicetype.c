/*****************************************************************************
 * slicetype.c: lookahead analysis
 *****************************************************************************/

#include "common/common.h"
#include "macroblock.h"
#include "me.h"
#include "ratecontrol.h"
#include "analyse.h"

/************************************************************************
 * output buffers (output_intra and output_inter) of frame costs        *
 * use x264_t->scratch_buffer2 to avoid repeated malloc.                *
 *                                                                      *
 * output buffers save costs as following order:                        *
 * [0]: cost estimation                                                 *
 * [1]: cost estimation if AQ enabled                                   *
 * [2]: number of intra mbs                                             *
 * [3]: number of rows (i_mb_height)                                    *
 * [4]: satd of row 1 (only used if vbv enabled)                        *
 * [5]: satd of row 2 (only used if vbv enabled)                        *
 * ......                                                               *
 * [h->mb.i_mb_height + 3]: satd of last row (only used if vbv enabled) *
 ************************************************************************/
#define COST_EST    0
#define COST_EST_AQ 1
#define INTRA_MBS   2
#define NUM_ROWS    3
#define NUM_INTS    4
#define ROW_SATD    (NUM_INTS + (h->mb.i_mb_y - h->i_threadslice_start))

/*******************************************************************************************
 * x264_slicetype_frame_cost operates by calling x264_slicetype_mb_cost                    *
 * for each macroblock in the frame. As the frame is half-resolution,                      *
 * each ¡°macroblock¡± is 8x8 pixels instead of 16x16. x264_slicetype_mb_cost                *
 * performs a motion search for each reference frame                                       *
 * (past for P-frames, past and future for B-frames).                                      *
 * This motion search is typically a hexagon motion search with subpel refinement.         *
 *                                                                                         *
 * For B-frames it also checks a few possible bidirectional modes:                         *
 * a mode similar to H.264/AVC's ¡°temporal direct¡±, the zero motion vector,                *
 * and a mode using the motion vectors resulting from the list0 and list1 motion searches. *
 *                                                                                         *
 * x264_slicetype_mb_cost also calculates an approximate intra cost. All of these costs    *
 * are stored for potential future usage. This is important for mb-tree or qcomp rate      *
 * control mode, which will need this information for its calculations.                    *
 *******************************************************************************************/
static void x264_slicetype_mb_cost( x264_t *h, x264_frame_t **frames, int p0, int p1, int b,
                                    int do_search, const x264_weight_t *w,
                                    int *output_inter, int *output_intra )
{
    x264_frame_t *fref0 = frames[p0];
    x264_frame_t *fenc  = frames[b];
    const int i_mb_x = h->mb.i_mb_x;
    const int i_mb_y = h->mb.i_mb_y;
    const int i_mb_stride = h->mb.i_mb_width;
    const int i_mb_xy = i_mb_x + i_mb_y * i_mb_stride;
    const int i_stride = fenc->i_stride_lowres;
    const int i_pel_offset = (i_mb_x + i_mb_y * i_stride) << 3;
    int16_t (*fenc_mvs)[2] = &fenc->lowres_mvs[0][b-p0-1][i_mb_xy];
    int *fenc_costs = &fenc->lowres_mv_costs[0][b-p0-1][i_mb_xy];
    ALIGNED_ARRAY_16( pixel, pix1, [9*FDEC_STRIDE] );

    x264_me_t m;
    int i_bcost = COST_MAX, i_icost = COST_MAX;
    int i_bcost_aq, i_icost_aq;
    int b_intra;
    /* A small, arbitrary bias to avoid VBV problems caused by zero-residual lookahead blocks. */
    int lowres_penalty = 4;
	int i;

    h->mb.pic.p_fenc[0] = h->mb.pic.fenc_buf;
    h->mc.copy[PIXEL_8x8]( h->mb.pic.p_fenc[0], FENC_STRIDE, &fenc->lowres[0][i_pel_offset], i_stride, 8 );

    /* do motion search for P/B frame */
    if( p0 != p1 )
	{
		// no need for h->mb.mv_min[]
		h->mb.mv_min_fpel[0] = -(h->mb.i_mb_x << 3) - 4;
		h->mb.mv_max_fpel[0] = (( h->mb.i_mb_width - h->mb.i_mb_x - 1 ) << 3) + 4;
		h->mb.mv_min_spel[0] = ( h->mb.mv_min_fpel[0] - 8 ) << 2;
		h->mb.mv_max_spel[0] = ( h->mb.mv_max_fpel[0] + 8 ) << 2;
		if( h->mb.i_mb_x >= h->mb.i_mb_width - 2 )
		{
			h->mb.mv_min_fpel[1] = -(h->mb.i_mb_y << 3) - 4;
			h->mb.mv_max_fpel[1] = (( h->mb.i_mb_height - h->mb.i_mb_y - 1 ) << 3) + 4;
			h->mb.mv_min_spel[1] = ( h->mb.mv_min_fpel[1] - 8 ) << 2;
			h->mb.mv_max_spel[1] = ( h->mb.mv_max_fpel[1] + 8 ) << 2;
		}

		m.i_pixel = PIXEL_8x8;
		m.p_cost_mv = h->cost_mv[X264_LOOKAHEAD_QP];
		m.i_stride[0] = i_stride;
		m.p_fenc[0] = h->mb.pic.p_fenc[0];
		m.weight = w;
		m.i_ref = 0;
		m.p_fref[0] = &(fref0->lowres)[0][i_pel_offset];
		m.p_fref[1] = &(fref0->lowres)[1][i_pel_offset];
		m.p_fref[2] = &(fref0->lowres)[2][i_pel_offset];
		m.p_fref[3] = &(fref0->lowres)[3][i_pel_offset];
		m.p_fref_w = m.p_fref[0];

		if( do_search )
		{
			int i_mvc = 0;
			ALIGNED_4( int16_t mvc[4][2] );

			/* Reverse-order MV prediction. 4 mv candidates are available. */
			CP32( mvc[i_mvc++], fenc_mvs[1] );
			CP32( mvc[i_mvc++], fenc_mvs[i_mb_stride] );
			CP32( mvc[i_mvc++], fenc_mvs[i_mb_stride-1] );
			CP32( mvc[i_mvc++], fenc_mvs[i_mb_stride+1] );
			x264_median_mv( m.mvp, mvc[0], mvc[1], mvc[2] );

			/* Fast skip for cases of near-zero residual.  Shortcut: don't bother except in the mv0 case,
			 * since anything else is likely to have enough residual to not trigger the skip. */
			if( !M32( m.mvp ) )
			{
				m.cost = h->pixf.mbcmp[PIXEL_8x8]( m.p_fenc[0], FENC_STRIDE, m.p_fref[0], m.i_stride[0] );
				if( m.cost < 64 )
				{
					M32( m.mv ) = 0;
					goto skip_motionest;
				}
			}

			x264_me_search( h, &m, mvc, i_mvc );
			m.cost -= 1; /* h->cost_mv[X264_LOOKAHEAD_QP][0] = 1 */ // remove mvcost from skip mbs
			if( M32( m.mv ) )
				m.cost += 5; /* (5*x264_lambda_tab[X264_LOOKAHEAD_QP]) = 5*1 = 5 */

	skip_motionest:
			CP32( fenc_mvs, m.mv );
			*fenc_costs = m.cost;
		}
		else
		{
			CP32( m.mv, fenc_mvs );
			m.cost = *fenc_costs;
		}
		COPY1_IF_LT( i_bcost, m.cost );
	}

    /* do intra predict and calculate cost for I/P frame */
    if( !fenc->b_intra_calculated )
    {
        pixel *pix = &pix1[8+FDEC_STRIDE - 1];
        pixel *src = &fenc->lowres[0][i_pel_offset - 1];
        const int intra_penalty = 5; /* (5*x264_lambda_tab[X264_LOOKAHEAD_QP]) = 5*1 = 5 */
        int satds[4];

		/* load top and left neighbour pixels */
        memcpy( pix-FDEC_STRIDE, src-i_stride, 17 * sizeof(pixel) );
#pragma MUST_ITERATE(8, 8, 8)
        for( i = 0; i < 8; i++ )
            pix[i*FDEC_STRIDE] = src[i*i_stride];
        pix++;

        /*********************************************************************
         * As the frame is half-resolution, each ¡°macroblock¡± is 8x8 pixels  *
         * instead of 16x16.                                                 *
         * We use 8x8c predict modes: H, V, DC, P to estimate intra cost.    *
         * FIXME: To avoid 8x8 predict, DDL/DDR/VR/HD/VL/HU is removed here. *
         *********************************************************************/
        h->pixf.intra_mbcmp_x3_8x8c( h->mb.pic.p_fenc[0], pix, satds );
        i_icost = X264_MIN3( satds[0], satds[1], satds[2] );
        /*
        h->predict_8x8c[I_PRED_CHROMA_P]( pix );
        satds[3] = h->pixf.mbcmp[PIXEL_8x8]( pix, FDEC_STRIDE, h->mb.pic.p_fenc[0], FENC_STRIDE );
        i_icost = X264_MIN( i_icost, satds[3] );
        */

        /* sum up intra cost */
        i_icost += intra_penalty + lowres_penalty;
        i_icost_aq = i_icost;
        output_intra[ROW_SATD] += i_icost_aq;
        output_intra[COST_EST] += i_icost;
        output_intra[COST_EST_AQ] += i_icost_aq;
        /*fenc->i_intra_cost[i_mb_xy] = i_icost;*/
    }
    i_bcost += lowres_penalty;

    /* forbid intra-mbs in B-frames, because it's rare and not worth checking */
    b_intra = i_icost < i_bcost;
    if( b_intra ) i_bcost = i_icost;
    output_inter[INTRA_MBS] += b_intra;

    /* In an I-frame, we've already added the results above in the intra section. */
    /* Don't use AQ-weighted costs for slicetype decision, only for ratecontrol. */
    if( p0 != p1 )
    {
        i_bcost_aq = i_bcost;
        output_inter[ROW_SATD] += i_bcost_aq;
        output_inter[COST_EST] += i_bcost;
        output_inter[COST_EST_AQ] += i_bcost_aq;
    }

    /* store each mb cost and list used (0 for I-mb, 1 for P-mb, 2 for B-mb) for mb_tree rate control */
    /* fenc->lowres_costs[b-p0][p1-b][i_mb_xy] = X264_MIN( i_bcost, LOWRES_COST_MASK ) + (list_used << LOWRES_COST_SHIFT); */
}

/**************************************************************************
 * x264_slicetype_frame_cost is the core of the lookahead module,         *
 * which is called repeatedly to calculate the cost of a frame            *
 * given p0, p1, and b values.                                            *
 *                                                                        *
 * p0 is the list-0 (past) reference frame of the frame to be analyzed.   *
 * p1 is the list-1 (future) reference frame of the frame to be analyzed. *
 * b is the frame to be analyzed.                                         *
 * If p1 is equal to b, the frame is inferred to be a P-frame.            *
 * If p0 is equal to b, the frame is inferred to be an I-frame.           *
 *                                                                        *
 * As x264_slicetype_frame_cost may be called repeatedly on the same      *
 * arguments as part of the algorithms that use it, the results of        *
 * each call are cached for future usage.                                 *
 *                                                                        *
 * call hierarchy of frame cost calculation:                              *
 * ->-> x264_slicetype_frame_cost: cost of each frame                     *
 * ->->->-> x264_slicetype_mb_cost: cost of each mb                       *
 * ->->->->->-> x264_me_search: motion search of inter mb(8x8)            *
 * ->->->->->-> x264_predict_8x8c: predict of intra mb(8x8)               *
 **************************************************************************/
static int x264_slicetype_frame_cost( x264_t *h, x264_frame_t **frames, int p0, int p1, int b )
{
    int i_score = 0;
    int do_search;
    const x264_weight_t *w = x264_weight_none;
    x264_frame_t *fenc = frames[b];
    int do_edges, start_x, end_x, start_y, end_y;
	int output_buf_size;
	int *output_inter;
	int *output_intra;

    /* Check whether we already evaluated this frame
     * If we have tried this frame as P, then we have also tried
     * the preceding frames as B. (is this still true?) */
    /* Also check that we already calculated the row SATDs for the current frame. */
    if( fenc->i_cost_est[b-p0][p1-b] >= 0 && (!h->param.rc.i_vbv_buffer_size || fenc->i_row_satds[b-p0][p1-b][0] != -1) )
        i_score = fenc->i_cost_est[b-p0][p1-b];
    else
    {
		/************************************************************************
		 * Init motion search context for half-resolution version of the frame, *
		 * typically a hexagon motion search with subpel refinement.            *
		 * (me = DIA, subme = 2, qp = 12<X264_LOOKAHEAD_QP>, lambda = 1 )       *
		 ************************************************************************/
		/*
	    if( h->param.analyse.i_subpel_refine > 1 )
	    {
	        h->mb.i_me_method = X264_MIN( X264_ME_HEX, h->param.analyse.i_me_method );
	        h->mb.i_subpel_refine = 4;
	    }
	    else
	    {
	        h->mb.i_me_method = X264_ME_DIA;
	        h->mb.i_subpel_refine = 2;
	    }
	    */
		h->mb.i_me_method = X264_ME_DIA;
		h->mb.i_subpel_refine = 2;
		h->mb.b_chroma_me = 0;

        /* For each list, check to see whether we have lowres motion-searched this reference frame before. */
        do_search = (b != p0) && fenc->lowres_mvs[0][b-p0-1][0][0] == 0x7FFF; /* whether do past search */
        if( do_search ) fenc->lowres_mvs[0][b-p0-1][0][0] = 0;

        /* Init output_intra and output_inter */
        output_buf_size = h->mb.i_mb_height + NUM_INTS;
        output_inter = h->scratch_buffer2;
        output_intra = output_inter + output_buf_size;
		memset( output_inter, 0, output_buf_size * sizeof(int) );
		memset( output_intra, 0, output_buf_size * sizeof(int) );
		output_inter[NUM_ROWS] = output_intra[NUM_ROWS] = h->mb.i_mb_height;
		h->i_threadslice_start = 0;
		h->i_threadslice_end = h->mb.i_mb_height;

        /*******************************************************************
		 * Lowres lookahead goes backwards because the MVs are used        *
		 * as predictors in the main encode.                               *
		 * This considerably improves MV prediction overall.               *
		 *                                                                 *
		 * The edge mbs seem to reduce the predictive quality of the       *
		 * whole frame's score, but are needed for a spatial distribution. *
		 *******************************************************************/
		do_edges = h->param.rc.b_mb_tree || h->param.rc.i_vbv_buffer_size || h->mb.i_mb_width <= 2 || h->mb.i_mb_height <= 2;
		start_y = X264_MIN( h->i_threadslice_end - 1, h->mb.i_mb_height - 2 + do_edges );
		end_y = X264_MAX( h->i_threadslice_start, 1 - do_edges );
		start_x = h->mb.i_mb_width - 2 + do_edges;
		end_x = 1 - do_edges;

		for( h->mb.i_mb_y = start_y; h->mb.i_mb_y >= end_y; h->mb.i_mb_y-- )
			for( h->mb.i_mb_x = start_x; h->mb.i_mb_x >= end_x; h->mb.i_mb_x-- )
				x264_slicetype_mb_cost( h, frames, p0, p1, b, do_search, w, output_inter, output_intra );

        /* Sum up accumulators: cost_est, cost_est_aq, intra_mbs, row_satds (if vbv enabled). */
        if( !fenc->b_intra_calculated )
        {
            fenc->i_cost_est[0][0] = 0;
            fenc->i_cost_est_aq[0][0] = 0;
        }
        fenc->i_cost_est[b-p0][p1-b] = 0;
        fenc->i_cost_est_aq[b-p0][p1-b] = 0;
        if( !fenc->b_intra_calculated )
        {
        	fenc->i_cost_est[0][0] += output_intra[COST_EST];
        	fenc->i_cost_est_aq[0][0] += output_intra[COST_EST_AQ];
        }
        fenc->i_cost_est[b-p0][p1-b] += output_inter[COST_EST];
        fenc->i_cost_est_aq[b-p0][p1-b] += output_inter[COST_EST_AQ];
        fenc->i_intra_mbs[b-p0] = output_inter[INTRA_MBS];
        if( h->param.rc.i_vbv_buffer_size )
        {
        	memcpy( fenc->i_row_satds[b-p0][p1-b], output_inter + NUM_INTS, output_inter[NUM_ROWS] * sizeof(int) );
        	if( !fenc->b_intra_calculated )
        		memcpy( fenc->i_row_satds[0][0], output_intra + NUM_INTS, output_intra[NUM_ROWS] * sizeof(int) );
        }
        fenc->b_intra_calculated = 1;
        i_score = fenc->i_cost_est[b-p0][p1-b];
    }

    return i_score;
}

static int scenecut( x264_t *h, x264_frame_t **frames, int p0, int p1 )
{
    x264_frame_t *frame = frames[p1];
	int res, icost, pcost, i_gop_size;
	int i_bias, i_thresh_max, i_thresh_min;

	/* calculate frame cost and get I-cost and P-cost */
    x264_slicetype_frame_cost( h, frames, p0, p1, p1 );

    icost = frame->i_cost_est[0][0];
    pcost = frame->i_cost_est[p1-p0][0];
    i_gop_size = frame->i_frame - h->lookahead->i_last_keyframe;

    /* magic numbers pulled out of thin air */
    i_thresh_max = h->param.i_scenecut_threshold;
    i_thresh_min = i_thresh_max >> 2;
    if( h->param.i_keyint_min == h->param.i_keyint_max )
    	i_thresh_min = i_thresh_max;

    /* compute bias based on gop size */
    if( i_gop_size <= (h->param.i_keyint_min >> 2) )
    	i_bias = i_thresh_min >> 2;
    else if( i_gop_size <= h->param.i_keyint_min )
    	i_bias = i_thresh_min * i_gop_size / h->param.i_keyint_min;
    else
    	i_bias = i_thresh_min + ( i_thresh_max - i_thresh_min ) * ( i_gop_size - h->param.i_keyint_min ) / ( h->param.i_keyint_max - h->param.i_keyint_min );

    /* compare cost as I-frame and cost as P-frame */
    res = (100 * pcost) >= ((100 - i_bias) * icost);
#ifdef _DEBUG
    if( res )
    {
    	int num_mb = (h->mb.i_mb_width > 2 && h->mb.i_mb_height > 2) ?
    			(h->mb.i_mb_width - 2) * (h->mb.i_mb_height - 2) :
    			(h->mb.i_mb_width * h->mb.i_mb_height);
        int imb = frame->i_intra_mbs[p1-p0];
        int pmb = num_mb - imb;
        x264_log( h, X264_LOG_DEBUG, "scene cut at %d Icost:%d Pcost:%d ratio:%.4f bias:%d gop:%d (imb:%d pmb:%d)\n",
                  frame->i_frame, icost, pcost, 1. - (double)pcost / icost, i_bias, i_gop_size, imb, pmb );
    }
#endif
    return res;
}

static void x264_calculate_durations( x264_t *h, x264_frame_t *cur_frame, x264_frame_t *prev_frame, int64_t *i_cpb_delay, int64_t *i_coded_fields )
{
    cur_frame->i_cpb_delay = *i_cpb_delay;
    cur_frame->i_dpb_output_delay = cur_frame->i_field_cnt - *i_coded_fields;

    // add a correction term for frame reordering
    cur_frame->i_dpb_output_delay += h->sps->vui.i_num_reorder_frames*2;

    // fix possible negative dpb_output_delay because of pulldown changes and reordering
    if( cur_frame->i_dpb_output_delay < 0 )
    {
        cur_frame->i_cpb_delay += cur_frame->i_dpb_output_delay;
        cur_frame->i_dpb_output_delay = 0;
        if( prev_frame )
            prev_frame->i_cpb_duration += cur_frame->i_dpb_output_delay;
    }

    // don't reset cpb delay for IDR frames when using intra-refresh
    if( cur_frame->b_keyframe && !h->param.b_intra_refresh )
        *i_cpb_delay = 0;

    *i_cpb_delay += cur_frame->i_duration;
    *i_coded_fields += cur_frame->i_duration;
    cur_frame->i_cpb_duration = cur_frame->i_duration;
}

static void x264_slicetype_analyse( x264_t *h, int keyframe )
{
    x264_frame_t *frames[X264_LOOKAHEAD_MAX+3] = { NULL, };
    int num_frames, keyint_limit, framecnt;
    int i_max_search = X264_MIN( h->lookahead->next.i_size, X264_LOOKAHEAD_MAX );
	int i;

	/* terminate analysis if it's the first frame */
    if( !h->lookahead->last_nonb )
        return;

    /*******************************************************
     * framecnt specifies number of frames to decide.      *
     * i_max_search specifies maximum number of frames to  *
     * analyse, <= size of lookahead->next                 *
     *******************************************************/
    frames[0] = h->lookahead->last_nonb;
    for( framecnt = 0; framecnt < i_max_search && h->lookahead->next.list[framecnt]->i_type == X264_TYPE_AUTO; framecnt++ )
        frames[framecnt+1] = h->lookahead->next.list[framecnt];
    if( !framecnt )
        return;

    /* limit num_frames to the scope of current gop. */
    keyint_limit = h->param.i_keyint_max - frames[0]->i_frame + h->lookahead->i_last_keyframe - 1;
    num_frames = h->param.b_intra_refresh ? framecnt : X264_MIN( framecnt, keyint_limit );
    if( num_frames == 0 )
    {
        frames[1]->i_type = X264_TYPE_I;
        return;
    }

    /* scenecut detection. if I-cost is less than P-cost, use I-frame. */
    if( h->param.i_scenecut_threshold && scenecut( h, frames, 0, 1 ) )
    {
        frames[1]->i_type = X264_TYPE_I;
        return;
    }

	/* if B-frames is disabled, default to P-frames for later frames. */
	for( i = 1; i <= num_frames; i++ )
		frames[i]->i_type = X264_TYPE_P;
}

/* x264_slicetype_decide decides frame type (I/P/B) for frames in lookahead->next list */
void x264_slicetype_decide( x264_t *h )
{
    x264_frame_t *frames[X264_BFRAME_MAX+2];
    x264_frame_t *frm;
    int bframes = 0;
	int i, i_coded, lookahead_size;

    if( !h->lookahead->next.i_size )
        return;

    /* calculate duration of frames in lookahead->next list */
    lookahead_size = h->lookahead->next.i_size;
    for( i = 0; i < h->lookahead->next.i_size; i++ )
    {
        if( h->param.b_vfr_input )
        {
            if( lookahead_size-- > 1 )
                h->lookahead->next.list[i]->i_duration = 2 * (h->lookahead->next.list[i+1]->i_pts - h->lookahead->next.list[i]->i_pts);
            else
                h->lookahead->next.list[i]->i_duration = h->i_prev_duration;
        }
        else
        {
        	/***************************************************************
        	 * delta_tfi_divisor[h->lookahead->next.list[i]->i_pic_struct] *
        	 * delta_tfi_divisor: Picture duration in                      *
        	 * SPS time_scale units, indexed by pic_struct values          *
        	 * 0 <= PIC_STRUCT_AUTO                                        *
        	 * 2 <= PIC_STRUCT_PROGRESSIVE                                 *
        	 * 2 <= PIC_STRUCT_TOP_BOTTOM                                  *
        	 * 2 <= PIC_STRUCT_BOTTOM_TOP                                  *
        	 * 3 <= PIC_STRUCT_TOP_BOTTOM_TOP                              *
        	 * 3 <= PIC_STRUCT_BOTTOM_TOP_BOTTOM                           *
        	 * 4 <= PIC_STRUCT_DOUBLE                                      *
        	 * 6 <= PIC_STRUCT_TRIPLE                                      *
        	 ***************************************************************/
            h->lookahead->next.list[i]->i_duration = 2;
        }
        h->i_prev_duration = h->lookahead->next.list[i]->i_duration;
        h->lookahead->next.list[i]->f_duration = (double)h->lookahead->next.list[i]->i_duration * h->sps->vui.i_num_units_in_tick / h->sps->vui.i_time_scale;

        if( h->lookahead->next.list[i]->i_frame > h->i_disp_fields_last_frame && lookahead_size > 0 )
        {
            h->lookahead->next.list[i]->i_field_cnt = h->i_disp_fields;
            h->i_disp_fields += h->lookahead->next.list[i]->i_duration;
            h->i_disp_fields_last_frame = h->lookahead->next.list[i]->i_frame;
        }
        else if( lookahead_size == 0 )
        {
            h->lookahead->next.list[i]->i_field_cnt = h->i_disp_fields;
            h->lookahead->next.list[i]->i_duration = h->i_prev_duration;
        }
    }

    /************************************************************
     * analyse and decide I/P/B if following conditions happen: *
     * 1. B-frame adaptive is enabled ?                         *
     * 2. or scenecut threshold is enabled ?                    *
     * 3. or mb_tree rate control is enabled ?                  *
     * 4. or vbv lookahead is enabled ?                         *
     ************************************************************/
    if( (h->param.i_bframe && h->param.i_bframe_adaptive)
    		|| h->param.i_scenecut_threshold
    		|| h->param.rc.b_mb_tree
    		|| (h->param.rc.i_vbv_buffer_size && h->param.rc.i_lookahead) )
        x264_slicetype_analyse( h, 0 );

    /* examine the frames list until first Non-B frame occurs */
    /* if b-frames is disabled, then loop is no sense, bframes is constant to zero. */
    /* for( bframes = 0;; bframes++ ) */
    {
        frm = h->lookahead->next.list[bframes];

        /* Use IDR for key frame if open GOP is disabled. Otherwise use normal I-frame. */
        if( frm->i_type == X264_TYPE_KEYFRAME )
            frm->i_type = h->param.b_open_gop ? X264_TYPE_I : X264_TYPE_IDR;

        /* Limit GOP size between min-keyint and max-keyint */
        if( (!h->param.b_intra_refresh || frm->i_frame == 0) && (frm->i_frame - h->lookahead->i_last_keyframe >= h->param.i_keyint_max) )
        {
            if( frm->i_type == X264_TYPE_AUTO || frm->i_type == X264_TYPE_I )
                frm->i_type = h->param.b_open_gop && h->lookahead->i_last_keyframe >= 0 ? X264_TYPE_I : X264_TYPE_IDR;
        }
        if( frm->i_type == X264_TYPE_I && (frm->i_frame - h->lookahead->i_last_keyframe >= h->param.i_keyint_min) )
        {
            if( h->param.b_open_gop )
            {
                h->lookahead->i_last_keyframe = frm->i_frame; // Use display order
                frm->b_keyframe = 1;
            }
            else
                frm->i_type = X264_TYPE_IDR;
        }

        /* If IDR is decided, change the previous B-frame to P-frame to make sure GOP is closed. */
        if( frm->i_type == X264_TYPE_IDR )
        {
            h->lookahead->i_last_keyframe = frm->i_frame;
            frm->b_keyframe = 1;
        }

        /* Switch to P-frame if we have reached max B-frames */
        if( bframes == h->param.i_bframe || !h->lookahead->next.list[bframes+1] )
        {
            if( frm->i_type == X264_TYPE_AUTO || IS_X264_TYPE_B( frm->i_type ) )
                frm->i_type = X264_TYPE_P;
        }

        /* if type is still not decided, then default to P-frame */
        if( frm->i_type == X264_TYPE_AUTO )
            frm->i_type = X264_TYPE_P;
    }

    h->lookahead->next.list[bframes]->i_bframes = bframes;

    /*****************************************************************
     * If we run in ABR/CBR/CRF rate control mode, then calculate    *
     * the frame costs ahead of time using x264_slicetype_frame_cost *
     * for x264_rc_analyse_slice while we still have lowres.         *
     * x264_rc_analyse_slice is called by rate control module later. *
     *****************************************************************/
    if( h->param.rc.i_rc_method != X264_RC_CQP )
    {
        int p0, p1, b;
        p1 = b = bframes + 1;

        frames[0] = h->lookahead->last_nonb;
        memcpy( &frames[1], h->lookahead->next.list, (bframes+1) * sizeof(x264_frame_t*) );
        if( IS_X264_TYPE_I( h->lookahead->next.list[bframes]->i_type ) )
            p0 = bframes + 1;
        else // P
            p0 = 0;

        /* calculate frame cost here */
        x264_slicetype_frame_cost( h, frames, p0, p1, b );

        /****************************************************************
         * If VBV-compliant constant bitrate (CBR) is enabled,          *
         * the overflow compensation is the same algorithm as in ABR,   *
         * but runs after each row of macroblocks instead of per-frame. *
         * We need the intra costs for row SATDs here.                  *
         ****************************************************************/
        if( (p0 != p1) && h->param.rc.i_vbv_buffer_size )
            x264_slicetype_frame_cost( h, frames, b, b, b );
    }

    /******************************************************************************
     * shift sequence to coded order.                                             *
     * use a small temporary list to avoid shifting the entire next buffer around *
     * for example: original order (play order): B0, B1, B2, P                    *
     * if B-frame pyramid mode is disabled, then coded order: P, B0, B1, B2       *
     * if B-frame pyramid mode is enabled, then coded order: P, B1, B0, B2        *
     ******************************************************************************/
    i_coded = h->lookahead->next.list[0]->i_frame;
    h->lookahead->next.list[0]->i_coded = i_coded++;
    x264_calculate_durations( h, h->lookahead->next.list[0], NULL, &h->i_cpb_delay, &h->i_coded_fields );
    h->lookahead->next.list[0]->f_planned_cpb_duration[0] = (double)h->lookahead->next.list[0]->i_cpb_duration * h->sps->vui.i_num_units_in_tick / h->sps->vui.i_time_scale;
}

/***************************************************************************
 * x264_rc_analyse_slice returns cost (satd) of current slice.             *
 * This is called by rate_estimate_qscale in ABR/CBR/CRF rate control mode *
 *                                                                         *
 * We run a fast motion estimation algorithm over a half-resolution        *
 * version of the frame, and use the Sum of Absolute Hadamard Transformed  *
 * Differences (SATD) of the residuals as the complexity of frame.         *
 ***************************************************************************/
int x264_rc_analyse_slice( x264_t *h )
{
    int p0 = 0, p1, b;
    int cost;
	x264_frame_t **frames;

    if( IS_X264_TYPE_I(h->fenc->i_type) ) /* I-frame */
        p1 = b = 0;
    else if( h->fenc->i_type == X264_TYPE_P ) /* P-frame */
        p1 = b = h->fenc->i_bframes + 1;
    else /* B-frame */
    {
        p1 = (h->fref_nearest[1]->i_poc - h->fref_nearest[0]->i_poc)/2;
        b  = (h->fenc->i_poc - h->fref_nearest[0]->i_poc)/2;
    }
    /* We don't need to assign p0/p1 since we are not performing any real analysis here. */
    frames = &h->fenc - b;

    /**********************************************************************
     * cost should have already been calculated by x264_slicetype_decide. *
     * see x264_slicetype_frame_cost in x264_slicetype_decide.            *
     **********************************************************************/
    cost = frames[b]->i_cost_est[b-p0][p1-b];
    /*assert( cost >= 0 );*/

    /* In AQ, use the weighted score instead. */
    if( h->param.rc.i_aq_mode )
        cost = frames[b]->i_cost_est_aq[b-p0][p1-b];

    h->fenc->i_row_satd = h->fenc->i_row_satds[b-p0][p1-b];
    h->fdec->i_row_satd = h->fdec->i_row_satds[b-p0][p1-b];
    h->fdec->i_satd = cost;
    memcpy( h->fdec->i_row_satd, h->fenc->i_row_satd, h->mb.i_mb_height * sizeof(int) );
    if( !IS_X264_TYPE_I(h->fenc->i_type) )
        memcpy( h->fdec->i_row_satds[0][0], h->fenc->i_row_satds[0][0], h->mb.i_mb_height * sizeof(int) );

    return cost;
}
