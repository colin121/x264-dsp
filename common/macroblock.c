/*****************************************************************************
 * macroblock.c: macroblock common functions
 *****************************************************************************/

#include "common.h"
#include "encoder/me.h"

static NOINLINE void x264_mb_mc_xywh( x264_t *h, int x, int y, int width, int height )
{
    int i8     = x264_scan8[0] + x + (y<<3);
    int i_ref  = h->mb.cache.ref[0][i8];
    int mvx    = x264_clip3( h->mb.cache.mv[0][i8][0], h->mb.mv_min[0], h->mb.mv_max[0] ) + (x<<4);
    int mvy    = x264_clip3( h->mb.cache.mv[0][i8][1], h->mb.mv_min[1], h->mb.mv_max[1] ) + (y<<4);
	int offset = (y*FDEC_STRIDE + x)<<1;

    h->mc.mc_luma( &h->mb.pic.p_fdec[0][offset<<1], FDEC_STRIDE,
    		&h->mb.pic.p_fref[0][i_ref][0], h->mb.pic.i_stride[0],
    		mvx, mvy, width<<2, height<<2, &h->sh.weight[i_ref][0] );

	h->mc.mc_chroma( &h->mb.pic.p_fdec[1][offset], &h->mb.pic.p_fdec[2][offset], FDEC_STRIDE,
			h->mb.pic.p_fref[0][i_ref][4], h->mb.pic.i_stride[1],
			mvx, mvy, width<<1, height<<1 );
}

void x264_mb_mc( x264_t *h )
{
	if( h->mb.i_partition == D_16x16 )
	{
		x264_mb_mc_xywh( h, 0, 0, 4, 4 );
	}
	else if( h->mb.i_partition == D_16x8 )
	{
		x264_mb_mc_xywh( h, 0, 0, 4, 2 );
		x264_mb_mc_xywh( h, 0, 2, 4, 2 );
	}
	else if( h->mb.i_partition == D_8x16 )
	{
		x264_mb_mc_xywh( h, 0, 0, 2, 4 );
		x264_mb_mc_xywh( h, 2, 0, 2, 4 );
	}
	else if ( h->mb.i_partition == D_8x8 )
	{
		x264_mb_mc_xywh( h, 0, 0, 2, 2 );
		x264_mb_mc_xywh( h, 2, 0, 2, 2 );
		x264_mb_mc_xywh( h, 0, 2, 2, 2 );
		x264_mb_mc_xywh( h, 2, 2, 2, 2 );
	}
}

int x264_macroblock_cache_allocate( x264_t *h )
{
    int i_mb_count = h->mb.i_mb_count;
	int i, j;

    h->mb.i_mb_stride = h->mb.i_mb_width;
    h->mb.i_b8_stride = h->mb.i_mb_width << 1;
    h->mb.i_b4_stride = h->mb.i_mb_width << 2;
    h->mb.b_interlaced = PARAM_INTERLACED;

    CHECKED_MALLOC( h->mb.qp, i_mb_count * sizeof(int8_t) );
    CHECKED_MALLOC( h->mb.cbp, i_mb_count * sizeof(int16_t) );
    CHECKED_MALLOC( h->mb.mb_transform_size, i_mb_count * sizeof(int8_t) );
    CHECKED_MALLOC( h->mb.slice_table, i_mb_count * sizeof(uint16_t) );
    memset( h->mb.slice_table, -1, i_mb_count * sizeof(uint16_t) );

    /* 0 -> 3 top(4), 4 -> 6 : left(3) */
    CHECKED_MALLOC( h->mb.intra4x4_pred_mode, i_mb_count * 8 * sizeof(int8_t) );

    /* all coeffs */
    CHECKED_MALLOC( h->mb.non_zero_count, i_mb_count * 48 * sizeof(uint8_t) );

    if( h->param.b_cabac )
    {
        CHECKED_MALLOC( h->mb.skipbp, i_mb_count * sizeof(int8_t) );
        CHECKED_MALLOC( h->mb.chroma_pred_mode, i_mb_count * sizeof(int8_t) );
        CHECKED_MALLOC( h->mb.mvd[0], i_mb_count * sizeof( **h->mb.mvd ) );
        if( h->param.i_bframe )
            CHECKED_MALLOC( h->mb.mvd[1], i_mb_count * sizeof( **h->mb.mvd ) );
    }

    for( i = 0; i < 2; i++ )
    {
        int i_refs = X264_MIN(X264_REF_MAX, (i ? 1 + !!h->param.i_bframe_pyramid : h->param.i_frame_reference) ) << PARAM_INTERLACED;

        for( j = !i; j < i_refs; j++ )
        {
            CHECKED_MALLOC( h->mb.mvr[i][j], 2 * (i_mb_count + 1) * sizeof(int16_t) );
            M32( h->mb.mvr[i][j][0] ) = 0;
            h->mb.mvr[i][j]++;
        }
    }

    return 0;
fail:
    return -1;
}

void x264_macroblock_cache_free( x264_t *h )
{
	int i, j;
    for( i = 0; i < 2; i++ )
        for( j = !i; j < X264_REF_MAX*2; j++ )
            if( h->mb.mvr[i][j] )
                x264_free( h->mb.mvr[i][j]-1 );
    for( i = 0; i < X264_REF_MAX; i++ )
        x264_free( h->mb.p_weight_buf[i] );

    if( h->param.b_cabac )
    {
        x264_free( h->mb.skipbp );
        x264_free( h->mb.chroma_pred_mode );
        x264_free( h->mb.mvd[0] );
        x264_free( h->mb.mvd[1] );
    }
    x264_free( h->mb.slice_table );
    x264_free( h->mb.intra4x4_pred_mode );
    x264_free( h->mb.non_zero_count );
    x264_free( h->mb.mb_transform_size );
    x264_free( h->mb.cbp );
    x264_free( h->mb.qp );
}

int x264_macroblock_thread_allocate( x264_t *h, int b_lookahead )
{
	int i, j, buf_mbtree;
	int scratch_size = 0;

    if( !b_lookahead )
    {
        for( i = 0; i < 2; i++ )
            for( j = 0; j < 2; j++ )
            {
                CHECKED_MALLOC( h->intra_border_backup[i][j], (h->sps->i_mb_width*16+32) * sizeof(pixel) );
                h->intra_border_backup[i][j] += 16;
            }

        CHECKED_MALLOC( h->deblock_strength[0], sizeof(**h->deblock_strength) * h->mb.i_mb_width );
        h->deblock_strength[1] = h->deblock_strength[0];
    }

    /* Allocate scratch buffer */
    if( !b_lookahead )
    {
        int buf_hpel = (h->thread[0]->fdec->i_width[0]+48) * sizeof(int16_t);
        int buf_ssim = h->param.analyse.b_ssim * 8 * (h->param.i_width/4+3) * sizeof(int);
        scratch_size = X264_MAX( buf_hpel, buf_ssim );
    }
    buf_mbtree = h->param.rc.b_mb_tree * ((h->mb.i_mb_width+7)&~7) * sizeof(int);
    scratch_size = X264_MAX( scratch_size, buf_mbtree );
    if( scratch_size )
        CHECKED_MALLOC( h->scratch_buffer, scratch_size );
    else
        h->scratch_buffer = NULL;
    CHECKED_MALLOC( h->scratch_buffer2, (h->mb.i_mb_height + 4) * sizeof(int) * 2 );

    return 0;
fail:
    return -1;
}

void x264_macroblock_thread_free( x264_t *h, int b_lookahead )
{
	int i, j;
    if( !b_lookahead )
    {
    	if( !h->param.b_sliced_threads || (h == h->thread[0]) )
    		x264_free( h->deblock_strength[0] );
        for( i = 0; i < 2; i++ )
            for( j = 0; j < 2; j++ )
                x264_free( h->intra_border_backup[i][j] - 16 );
    }
    x264_free( h->scratch_buffer );
    x264_free( h->scratch_buffer2 );
}

void x264_macroblock_slice_init( x264_t *h )
{
	int i;
    h->mb.mv[0] = h->fdec->mv[0];
    h->mb.mv[1] = h->fdec->mv[1];
    h->mb.mvr[0][0] = h->fdec->mv16x16;
    h->mb.ref[0] = h->fdec->ref[0];
    h->mb.ref[1] = h->fdec->ref[1];
    h->mb.type = h->fdec->mb_type;
    h->mb.partition = h->fdec->mb_partition;
    h->mb.field = h->fdec->field;

    h->fdec->i_ref[0] = h->i_ref[0];
    h->fdec->i_ref[1] = h->i_ref[1];
    for( i = 0; i < h->i_ref[0]; i++ )
        h->fdec->ref_poc[0][i] = h->fref[0][i]->i_poc;

    /* init with not available (for top right idx=7,15) */
    memset( h->mb.cache.ref, -2, sizeof( h->mb.cache.ref ) );
    if( h->i_ref[0] > 0 )
	{
    	int curpoc = h->fdec->i_poc + h->fdec->i_delta_poc[0];
    	int refpoc = h->fref[0][0]->i_poc + h->fref[0][0]->i_delta_poc[0];
    	int delta = curpoc - refpoc;
    	if( delta > 0 )
    		h->fdec->inv_ref_poc[0] = (256 + (delta >> 1)) / delta;
	}

    /*********************************************************
     * At the beginning of 8x8 block coding:                 *
     * 1. available neighbours of block 3:                   *
     * MB_LEFT | MB_TOP | MB_TOPLEFT                         *
     * 2. available neighbours of other blocks is uncertain. *
     *                                                       *
     * At the beginning of 4x4 block coding:                 *
     * 1. available neighbours of block 6, 9, 12, 14:        *
     * MB_LEFT | MB_TOP | MB_TOPLEFT | MB_TOPRIGHT           *
     * 2. available neighbours of block 3, 7, 11, 13, 15:    *
     * MB_LEFT | MB_TOP | MB_TOPLEFT                         *
     * 3. available neighbours of other blocks is uncertain. *
     *********************************************************/
    h->mb.i_neighbour4[6] =
    h->mb.i_neighbour4[9] =
    h->mb.i_neighbour4[12] =
    h->mb.i_neighbour4[14] = MB_LEFT|MB_TOP|MB_TOPLEFT|MB_TOPRIGHT;
    h->mb.i_neighbour4[3] =
    h->mb.i_neighbour4[7] =
    h->mb.i_neighbour4[11] =
    h->mb.i_neighbour4[13] =
    h->mb.i_neighbour4[15] =
    h->mb.i_neighbour8[3] = MB_LEFT|MB_TOP|MB_TOPLEFT;
}

void x264_macroblock_thread_init( x264_t *h )
{
    h->mb.i_me_method = h->param.analyse.i_me_method;
    h->mb.i_subpel_refine = h->param.analyse.i_subpel_refine;
    if( h->sh.i_type == SLICE_TYPE_B && (h->mb.i_subpel_refine == 6 || h->mb.i_subpel_refine == 8) )
        h->mb.i_subpel_refine--;
    h->mb.b_chroma_me = h->param.analyse.b_chroma_me &&
                        ((h->sh.i_type == SLICE_TYPE_P && h->mb.i_subpel_refine >= 5) || /* subme >= 5 if P-slice */
                         (h->sh.i_type == SLICE_TYPE_B && h->mb.i_subpel_refine >= 9));  /* subme >= 9 if B-slice */
    h->mb.b_dct_decimate = h->sh.i_type == SLICE_TYPE_B ||
                          (h->param.analyse.b_dct_decimate && h->sh.i_type != SLICE_TYPE_I);
    h->mb.i_mb_prev_xy = -1;

    /*          4:2:0                      4:2:2                      4:4:4
     * fdec            fenc       fdec            fenc       fdec            fenc
     * y y y y y y y   Y Y Y Y    y y y y y y y   Y Y Y Y    y y y y y y y   Y Y Y Y
     * y Y Y Y Y       Y Y Y Y    y Y Y Y Y       Y Y Y Y    y Y Y Y Y       Y Y Y Y
     * y Y Y Y Y       Y Y Y Y    y Y Y Y Y       Y Y Y Y    y Y Y Y Y       Y Y Y Y
     * y Y Y Y Y       Y Y Y Y    y Y Y Y Y       Y Y Y Y    y Y Y Y Y       Y Y Y Y
     * y Y Y Y Y       U U V V    y Y Y Y Y       U U V V    y Y Y Y Y       U U U U
     * u u u   v v v   U U V V    u u u   v v v   U U V V    u u u u u u u   U U U U
     * u U U   v V V              u U U   v V V   U U V V    u U U U U       U U U U
     * u U U   v V V              u U U   v V V   U U V V    u U U U U       U U U U
     *                            u U U   v V V              u U U U U       V V V V
     *                            u U U   v V V              u U U U U       V V V V
     *                                                       v v v v v v v   V V V V
     *                                                       v V V V V       V V V V
     *                                                       v V V V V
     *                                                       v V V V V
     *                                                       v V V V V
     */
    h->mb.pic.p_fenc[0] = h->mb.pic.fenc_buf;
    h->mb.pic.p_fdec[0] = h->mb.pic.fdec_buf + 2*FDEC_STRIDE;
    h->mb.pic.p_fenc[1] = h->mb.pic.fenc_buf + 16*FENC_STRIDE;
    h->mb.pic.p_fdec[1] = h->mb.pic.fdec_buf + 19*FDEC_STRIDE;
    h->mb.pic.p_fenc[2] = h->mb.pic.fenc_buf + 16*FENC_STRIDE + 8;
    h->mb.pic.p_fdec[2] = h->mb.pic.fdec_buf + 19*FDEC_STRIDE + 16;
}

void x264_prefetch_fenc( x264_t *h, x264_frame_t *fenc, int i_mb_x, int i_mb_y )
{
    int stride_y  = fenc->i_stride[0];
    int stride_uv = fenc->i_stride[1];
    int off_y     = (i_mb_x + i_mb_y * stride_y) << 4;
    int off_uv    = (i_mb_x + (i_mb_y * stride_uv >> CHROMA_V_SHIFT)) << 4;
    h->mc.prefetch_fenc( fenc->plane[0]+off_y, stride_y, fenc->plane[1]+off_uv, stride_uv, i_mb_x );
}

static const x264_left_table_t left_indices[4] =
{
    /* Current is progressive */
    {{ 4, 4, 5, 5}, { 3,  3,  7,  7}, {16+1, 16+1, 32+1, 32+1}, {0, 0, 1, 1}, {0, 0, 0, 0}},
    {{ 6, 6, 3, 3}, {11, 11, 15, 15}, {16+5, 16+5, 32+5, 32+5}, {2, 2, 3, 3}, {1, 1, 1, 1}},
    /* Current is interlaced */
    {{ 4, 6, 4, 6}, { 3, 11,  3, 11}, {16+1, 16+1, 32+1, 32+1}, {0, 2, 0, 2}, {0, 1, 0, 1}},
    /* Both same */
    {{ 4, 5, 6, 3}, { 3,  7, 11, 15}, {16+1, 16+5, 32+1, 32+5}, {0, 1, 2, 3}, {0, 0, 1, 1}}
};

static void ALWAYS_INLINE x264_macroblock_cache_load_neighbours( x264_t *h, int mb_x, int mb_y )
{
    int top_y = mb_y - 1;
    int top   = top_y * h->mb.i_mb_stride + mb_x;

    h->mb.i_mb_x = mb_x;
    h->mb.i_mb_y = mb_y;
    h->mb.i_mb_xy = mb_y * h->mb.i_mb_stride + mb_x;
    h->mb.i_b8_xy = (mb_y * h->mb.i_b8_stride + mb_x) << 1;
    h->mb.i_b4_xy = (mb_y * h->mb.i_b4_stride + mb_x) << 2;
    h->mb.left_b8[0] = h->mb.left_b8[1] = h->mb.i_b8_xy - 2;
    h->mb.left_b4[0] = h->mb.left_b4[1] = h->mb.i_b4_xy - 4;

    h->mb.i_neighbour = 0;
    h->mb.i_neighbour_intra = 0;
    h->mb.i_neighbour_frame = 0;
    h->mb.i_mb_top_xy = -1;
    h->mb.i_mb_top_y = -1;
    h->mb.i_mb_left_xy[0] = h->mb.i_mb_left_xy[1] = -1;
    h->mb.i_mb_topleft_xy = -1;
    h->mb.i_mb_topright_xy = -1;
    h->mb.i_mb_type_top = -1;
    h->mb.i_mb_type_left[0] = h->mb.i_mb_type_left[1] = -1;
    h->mb.i_mb_type_topleft = -1;
    h->mb.i_mb_type_topright = -1;
    h->mb.left_index_table = &left_indices[3];
    h->mb.topleft_partition = 0;

    /* load left neighbour if exists */
    if( mb_x > 0 )
    {
        h->mb.i_neighbour |= MB_LEFT;
        h->mb.i_neighbour_intra |= MB_LEFT;
        h->mb.i_neighbour_frame |= MB_LEFT;
        h->mb.i_mb_left_xy[0] = h->mb.i_mb_xy - 1;
        h->mb.i_mb_left_xy[1] = h->mb.i_mb_xy - 1;
        h->mb.i_mb_type_left[0] = h->mb.type[h->mb.i_mb_left_xy[0]];
        h->mb.i_mb_type_left[1] = h->mb.type[h->mb.i_mb_left_xy[1]];
    }

    /* We can't predict from the previous threadslice since it hasn't been encoded yet. */
    if( h->i_threadslice_start != mb_y )
    {
    	/* load top neighbour if exists */
        if( top >= 0 )
        {
            h->mb.i_neighbour |= MB_TOP;
            h->mb.i_neighbour_intra |= MB_TOP;
            h->mb.i_neighbour_frame |= MB_TOP;
            h->mb.i_mb_top_xy = top;
            h->mb.i_mb_top_y = top_y;
            h->mb.i_mb_type_top = h->mb.type[h->mb.i_mb_top_xy];

			/* We only need to prefetch the top blocks because the left was just written
			 * to as part of the previous cache_save.  Since most target CPUs use write-allocate
			 * caches, left blocks are near-guaranteed to be in L1 cache.  Top--not so much. */
			x264_prefetch( &h->mb.cbp[top] );
			x264_prefetch( h->mb.intra4x4_pred_mode[top] );
			x264_prefetch( &h->mb.non_zero_count[top][12] );
			/* These aren't always allocated, but prefetching an invalid address can't hurt. */
			x264_prefetch( &h->mb.mb_transform_size[top] );
			x264_prefetch( &h->mb.skipbp[top] );
        }
        /* load top-left neighbour if exists */
        if( mb_x > 0 && top_y >= 0 )
        {
            h->mb.i_neighbour |= MB_TOPLEFT;
            h->mb.i_neighbour_intra |= MB_TOPLEFT;
            h->mb.i_neighbour_frame |= MB_TOPLEFT;
            h->mb.i_mb_topleft_xy = top - 1;
            h->mb.i_mb_topleft_y = top_y;
            h->mb.i_mb_type_topleft = h->mb.type[h->mb.i_mb_topleft_xy];
        }
        /* load top-right neighbour if exists */
        if( mb_x < h->mb.i_mb_width - 1 && top_y >= 0 )
        {
            h->mb.i_neighbour |= MB_TOPRIGHT;
            h->mb.i_neighbour_intra |= MB_TOPRIGHT;
            h->mb.i_neighbour_frame |= MB_TOPRIGHT;
            h->mb.i_mb_topright_xy = top + 1;
            h->mb.i_mb_topright_y = top_y;
            h->mb.i_mb_type_topright = h->mb.type[h->mb.i_mb_topright_xy];
        }
    }
}

static void ALWAYS_INLINE x264_macroblock_load_pic( x264_t *h, int mb_x, int mb_y )
{
    int i_offset_luma   = (mb_x << 4) + ((mb_y * h->fdec->i_stride[0]) << 4);
    int i_offset_chroma = (mb_x << 4) + ((mb_y * h->fdec->i_stride[1]) << 3);
    int i;

    /* load luma and chroma of current mb from frame plane buffer */
    h->mb.pic.i_stride[0] = h->fdec->i_stride[0];
    h->mb.pic.i_stride[1] = h->fdec->i_stride[1];
    h->mb.pic.p_fenc_plane[0] = &h->fenc->plane[0][i_offset_luma];
    h->mb.pic.p_fenc_plane[1] = &h->fenc->plane[1][i_offset_chroma];
    h->mc.copy[PIXEL_16x16]( h->mb.pic.p_fenc[0], FENC_STRIDE, h->mb.pic.p_fenc_plane[0], h->mb.pic.i_stride[0], 16 );
    h->mc.load_deinterleave_chroma_fenc( h->mb.pic.p_fenc[1], h->mb.pic.p_fenc_plane[1], h->mb.pic.i_stride[1], 8 );

    if( h->sh.i_type == SLICE_TYPE_I )
    {
        pixel *intra_fdec_luma   = &h->intra_border_backup[!(mb_y&1)][0][mb_x<<4];
        pixel *intra_fdec_chroma = &h->intra_border_backup[!(mb_y&1)][1][mb_x<<4];

    	/* copy right edge column to the left edge for later intra predict. */
#pragma MUST_ITERATE(16, 16, 16)
        for( i = 0; i < 16; i++ )
        	h->mb.pic.p_fdec[0][i*FDEC_STRIDE-1] = h->mb.pic.p_fdec[0][i*FDEC_STRIDE+15]; /* 16 pixels of luma */
#pragma MUST_ITERATE(8, 8, 8)
        for( i = 0; i < 8; i++ )
        {
        	h->mb.pic.p_fdec[1][i*FDEC_STRIDE-1] = h->mb.pic.p_fdec[1][i*FDEC_STRIDE+7];  /* 8 pixels of chroma u */
        	h->mb.pic.p_fdec[2][i*FDEC_STRIDE-1] = h->mb.pic.p_fdec[2][i*FDEC_STRIDE+7];  /* 8 pixels of chroma v */
        }

        /* load previous mb row for later intra predict */
        memcpy( h->mb.pic.p_fdec[0] - FDEC_STRIDE, intra_fdec_luma, 24 );
        memcpy( h->mb.pic.p_fdec[1] - FDEC_STRIDE, intra_fdec_chroma, 8 );
        memcpy( h->mb.pic.p_fdec[2] - FDEC_STRIDE, intra_fdec_chroma + 8, 8 );
        h->mb.pic.p_fdec[0][-FDEC_STRIDE - 1] = intra_fdec_luma[-1];
        h->mb.pic.p_fdec[1][-FDEC_STRIDE - 1] = intra_fdec_chroma[-9];
        h->mb.pic.p_fdec[2][-FDEC_STRIDE - 1] = intra_fdec_chroma[-1];
    }

    /* set data pointers of reference frames for later inter predict */
	for( i = 0; i < h->mb.pic.i_fref[0]; i++ )
	{
		h->mb.pic.p_fref[0][i][0] = h->mb.pic.p_fref_w[i] = h->fref[0][i]->plane[0] + i_offset_luma;
		h->mb.pic.p_fref[0][i][1] = h->fref[0][i]->filtered[0][1] + i_offset_luma; /* 1/2 H */
		h->mb.pic.p_fref[0][i][2] = h->fref[0][i]->filtered[0][2] + i_offset_luma; /* 1/2 V */
		h->mb.pic.p_fref[0][i][3] = h->fref[0][i]->filtered[0][3] + i_offset_luma; /* 1/2 C */
		h->mb.pic.p_fref[0][i][4] = h->fref[0][i]->plane[1] + i_offset_chroma;     /* Chroma */
	}
}

void x264_macroblock_cache_load( x264_t *h, int mb_x, int mb_y )
{
	int *left;
	int top, top_y, s8x8, s4x4, top_8x8, top_4x4, lists;
	int8_t (*i4x4)[8];
	uint8_t (*nnz)[48];
	int16_t *cbp;
	int i, l;
	const x264_left_table_t *left_index_table;

    x264_macroblock_cache_load_neighbours( h, mb_x, mb_y );

    left = h->mb.i_mb_left_xy;
    top  = h->mb.i_mb_top_xy;
    top_y = h->mb.i_mb_top_y;
    s8x8 = h->mb.i_b8_stride;
    s4x4 = h->mb.i_b4_stride;
    top_8x8 = ((top_y<<1) + 1) * s8x8 + (mb_x<<1);
    top_4x4 = ((top_y<<2) + 3) * s4x4 + (mb_x<<2);
    lists = (1 << h->sh.i_type) & 3; /* I: 0, P: 1, B: 2 */

    /* GCC pessimizes direct loads from heap-allocated arrays due to aliasing. */
    /* By only dereferencing them once, we avoid this issue. */
    i4x4 = h->mb.intra4x4_pred_mode;
    nnz = h->mb.non_zero_count;
    cbp = h->mb.cbp;
    left_index_table = h->mb.left_index_table;
    h->mb.cache.deblock_strength = h->deblock_strength[mb_y&1][mb_x];

    /* load cache */
    if( h->mb.i_neighbour & MB_TOP )
    {
        h->mb.cache.i_cbp_top = cbp[top];
        /* load predict_mode of 4 blocks(4x4) of top mb, which are nearest to current mb,
           to first reserved line of intra4x4_pred_mode table according to Scan8 order. */
        CP32( &h->mb.cache.intra4x4_pred_mode[x264_scan8[0] - 8], &i4x4[top][0] );

        /* load non_zero_count of 4 blocks(4x4) of top mb, which are nearest to current mb,
           to first reserved line of non_zero_count table according to Scan8 order. */
        CP32( &h->mb.cache.non_zero_count[x264_scan8[ 0] - 8], &nnz[top][12] ); /* Y */
        CP32( &h->mb.cache.non_zero_count[x264_scan8[16] - 8], &nnz[top][16-4 + (16>>CHROMA_V_SHIFT)] ); /* U */
        CP32( &h->mb.cache.non_zero_count[x264_scan8[32] - 8], &nnz[top][32-4 + (16>>CHROMA_V_SHIFT)] ); /* V */

        /* Finish the prefetching of reference lists 0/1 */
        for( l = 0; l < lists; l++ )
        {
            x264_prefetch( &h->mb.mv[l][top_4x4-1] );
            /* Top right being not in the same cacheline as top left will happen
             * once every 4 MBs, so one extra prefetch is worthwhile */
            x264_prefetch( &h->mb.mv[l][top_4x4+4] );
            x264_prefetch( &h->mb.ref[l][top_8x8-1] );
            x264_prefetch( &h->mb.mvd[l][top] );
        }
    }
    else
    {
        h->mb.cache.i_cbp_top = -1;

        /* load intra4x4 */
        M32( &h->mb.cache.intra4x4_pred_mode[x264_scan8[0] - 8] ) = 0xFFFFFFFFU;

        /* load non_zero_count */
        M32( &h->mb.cache.non_zero_count[x264_scan8[ 0] - 8] ) = 0x80808080U;
        M32( &h->mb.cache.non_zero_count[x264_scan8[16] - 8] ) = 0x80808080U;
        M32( &h->mb.cache.non_zero_count[x264_scan8[32] - 8] ) = 0x80808080U;
    }

    if( h->mb.i_neighbour & MB_LEFT )
    {
        int ltop = left[0];
        int lbot = ltop;
        h->mb.cache.i_cbp_left = cbp[ltop];

        /* load predict_mode of 4 blocks(4x4) of left mb, which are nearest to current mb,
           to first reserved column of intra4x4_pred_mode table according to Scan8 order. */
        h->mb.cache.intra4x4_pred_mode[x264_scan8[ 0] - 1] = i4x4[ltop][left_index_table->intra[0]];
        h->mb.cache.intra4x4_pred_mode[x264_scan8[ 2] - 1] = i4x4[ltop][left_index_table->intra[1]];
        h->mb.cache.intra4x4_pred_mode[x264_scan8[ 8] - 1] = i4x4[lbot][left_index_table->intra[2]];
        h->mb.cache.intra4x4_pred_mode[x264_scan8[10] - 1] = i4x4[lbot][left_index_table->intra[3]];

        /* load non_zero_count of 4 blocks(4x4) of left mb, which are nearest to current mb,
           to first reserved column of non_zero_count table according to Scan8 order. */
        h->mb.cache.non_zero_count[x264_scan8[ 0] - 1] = nnz[ltop][left_index_table->nnz[0]];
        h->mb.cache.non_zero_count[x264_scan8[ 2] - 1] = nnz[ltop][left_index_table->nnz[1]];
        h->mb.cache.non_zero_count[x264_scan8[ 8] - 1] = nnz[lbot][left_index_table->nnz[2]];
        h->mb.cache.non_zero_count[x264_scan8[10] - 1] = nnz[lbot][left_index_table->nnz[3]];
        h->mb.cache.non_zero_count[x264_scan8[16] - 1] = nnz[ltop][left_index_table->nnz_chroma[0]];
        h->mb.cache.non_zero_count[x264_scan8[18] - 1] = nnz[lbot][left_index_table->nnz_chroma[1]];
        h->mb.cache.non_zero_count[x264_scan8[32] - 1] = nnz[ltop][left_index_table->nnz_chroma[2]];
        h->mb.cache.non_zero_count[x264_scan8[34] - 1] = nnz[lbot][left_index_table->nnz_chroma[3]];
    }
    else
    {
        h->mb.cache.i_cbp_left = -1;

        h->mb.cache.intra4x4_pred_mode[x264_scan8[ 0] - 1] =
        h->mb.cache.intra4x4_pred_mode[x264_scan8[ 2] - 1] =
        h->mb.cache.intra4x4_pred_mode[x264_scan8[ 8] - 1] =
        h->mb.cache.intra4x4_pred_mode[x264_scan8[10] - 1] = -1;

        /* load non_zero_count */
        h->mb.cache.non_zero_count[x264_scan8[ 0] - 1] =
        h->mb.cache.non_zero_count[x264_scan8[ 2] - 1] =
        h->mb.cache.non_zero_count[x264_scan8[ 8] - 1] =
        h->mb.cache.non_zero_count[x264_scan8[10] - 1] =
        h->mb.cache.non_zero_count[x264_scan8[16] - 1] =
        h->mb.cache.non_zero_count[x264_scan8[18] - 1] =
        h->mb.cache.non_zero_count[x264_scan8[32] - 1] =
        h->mb.cache.non_zero_count[x264_scan8[34] - 1] = 0x80;
    }

	x264_macroblock_load_pic( h, mb_x, mb_y );
    x264_prefetch_fenc( h, h->fenc, mb_x, mb_y );

    /* load ref/mv/mvd */
    for( l = 0; l < lists; l++ )
    {
        int16_t (*mv)[2] = h->mb.mv[l];
        int8_t *ref = h->mb.ref[l];

        int i8 = x264_scan8[0] - 1 - 8;
        if( h->mb.i_neighbour & MB_TOPLEFT )
        {
            h->mb.cache.ref[l][i8] = ref[top_8x8 - 1];
            CP32( h->mb.cache.mv[l][i8], mv[top_4x4 - 1] );
        }
        else
        {
            h->mb.cache.ref[l][i8] = -2;
            M32( h->mb.cache.mv[l][i8] ) = 0;
        }

        i8 = x264_scan8[0] - 8;
        if( h->mb.i_neighbour & MB_TOP )
        {
            h->mb.cache.ref[l][i8+0] =
            h->mb.cache.ref[l][i8+1] = ref[top_8x8 + 0];
            h->mb.cache.ref[l][i8+2] =
            h->mb.cache.ref[l][i8+3] = ref[top_8x8 + 1];
            CP128( h->mb.cache.mv[l][i8], mv[top_4x4] );
        }
        else
        {
            M128( h->mb.cache.mv[l][i8] ) = M128_ZERO;
            M32( &h->mb.cache.ref[l][i8] ) = 0xFEFEFEFEU; /* (uint8_t)(-2) * 0x01010101U */;
        }

        i8 = x264_scan8[0] + 4 - 8;
        if( h->mb.i_neighbour & MB_TOPRIGHT )
        {
            h->mb.cache.ref[l][i8] = ref[top_8x8 + 2];
            CP32( h->mb.cache.mv[l][i8], mv[top_4x4 + 4] );
        }
        else
             h->mb.cache.ref[l][i8] = -2;

        i8 = x264_scan8[0] - 1;
        if( h->mb.i_neighbour & MB_LEFT )
        {
			const int ir = h->mb.i_b8_xy - 1;
			const int iv = h->mb.i_b4_xy - 1;
			h->mb.cache.ref[l][i8+ 0] =
			h->mb.cache.ref[l][i8+ 8] = ref[ir];
			h->mb.cache.ref[l][i8+16] =
			h->mb.cache.ref[l][i8+24] = ref[ir + s8x8];

			CP32( h->mb.cache.mv[l][i8+ 0], mv[iv + 0*s4x4] );
			CP32( h->mb.cache.mv[l][i8+ 8], mv[iv + 1*s4x4] );
			CP32( h->mb.cache.mv[l][i8+16], mv[iv + 2*s4x4] );
			CP32( h->mb.cache.mv[l][i8+24], mv[iv + 3*s4x4] );
        }
        else
        {
#pragma MUST_ITERATE(4, 4, 4)
            for( i = 0; i < 4; i++ )
            {
                h->mb.cache.ref[l][i8+(i<<3)] = -2;
                M32( h->mb.cache.mv[l][i8+(i<<3)] ) = 0;
            }
        }

        if( h->param.b_cabac )
        {
            uint8_t (*mvd)[8][2] = h->mb.mvd[l];
            if( h->mb.i_neighbour & MB_TOP )
                CP64( h->mb.cache.mvd[l][x264_scan8[0] - 8], mvd[top][0] );
            else
                M64( h->mb.cache.mvd[l][x264_scan8[0] - 8] ) = 0;

            if( h->mb.i_neighbour & MB_LEFT )
            {
                CP16( h->mb.cache.mvd[l][x264_scan8[0 ] - 1], mvd[left[0]][left_index_table->intra[0]] );
                CP16( h->mb.cache.mvd[l][x264_scan8[2 ] - 1], mvd[left[0]][left_index_table->intra[1]] );
                CP16( h->mb.cache.mvd[l][x264_scan8[8 ] - 1], mvd[left[0]][left_index_table->intra[2]] );
                CP16( h->mb.cache.mvd[l][x264_scan8[10] - 1], mvd[left[0]][left_index_table->intra[3]] );
            }
            else
            {
                M16( h->mb.cache.mvd[l][x264_scan8[0]-1+ 0] ) = 0;
                M16( h->mb.cache.mvd[l][x264_scan8[0]-1+ 8] ) = 0;
                M16( h->mb.cache.mvd[l][x264_scan8[0]-1+16] ) = 0;
                M16( h->mb.cache.mvd[l][x264_scan8[0]-1+24] ) = 0;
            }
        }
    }

    if( h->param.b_cabac )
    {
    	h->mb.cache.i_neighbour_skip = ((h->mb.i_neighbour & MB_LEFT) && !IS_SKIP( h->mb.i_mb_type_left[0] ))
    			+ ((h->mb.i_neighbour & MB_TOP)  && !IS_SKIP( h->mb.i_mb_type_top ));
    }

    /* load skip */
    if( h->sh.i_type == SLICE_TYPE_P )
        x264_mb_predict_mv_pskip( h, h->mb.cache.pskip_mv );

    /*******************************************************************
     * code order of 4x4 block in one macro-block:                     *
     * 0  1  4  5                                                      *
     * 2  3  6  7                                                      *
     * 8  9  12 13                                                     *
     * 10 11 14 15                                                     *
     *                                                                 *
     * block 0: top|topright if we have top neighbour_intra            *
     * block 1, 4: top|topleft|topright if we have top neighbour_intra *
     * block 5: top|topleft if we have top neighbour_intra             *
     * block 2, 8, 10: left|topleft if we hvae left neighbour_intra    *
     * block 3, 6, 7, 9, 12, 13, 11, 14, 15:                           *
     * determined by x264_macroblock_slice_init()                      *
     *                                                                 *
     * code order of 8x8 block in one macro-block:                     *
     * 0  1                                                            *
     * 2  3                                                            *
     *                                                                 *
     * block 0: top|topright if we have top neighbour_intra            *
     * block 1: top|topleft if we have top neighbour_intra             *
     * block 2: left|topleft if we have left neighbour_intra           *
     * block 3: determined by x264_macroblock_slice_init()             *
     *******************************************************************/
    h->mb.i_neighbour4[0] =
    h->mb.i_neighbour8[0] = (h->mb.i_neighbour_intra & (MB_TOP|MB_LEFT|MB_TOPLEFT))
                            | ((h->mb.i_neighbour_intra & MB_TOP) ? MB_TOPRIGHT : 0);
    h->mb.i_neighbour4[4] =
    h->mb.i_neighbour4[1] = MB_LEFT | ((h->mb.i_neighbour_intra & MB_TOP) ? (MB_TOP|MB_TOPLEFT|MB_TOPRIGHT) : 0);
    h->mb.i_neighbour4[2] =
    h->mb.i_neighbour4[8] =
    h->mb.i_neighbour4[10] =
    h->mb.i_neighbour8[2] = MB_TOP|MB_TOPRIGHT | ((h->mb.i_neighbour_intra & MB_LEFT) ? (MB_LEFT|MB_TOPLEFT) : 0);
    h->mb.i_neighbour4[5] =
    h->mb.i_neighbour8[1] = MB_LEFT | (h->mb.i_neighbour_intra & MB_TOPRIGHT)
                            | ((h->mb.i_neighbour_intra & MB_TOP) ? MB_TOP|MB_TOPLEFT : 0);
}

void x264_macroblock_deblock_strength( x264_t *h )
{
    uint8_t (*bs)[8][4] = h->mb.cache.deblock_strength;

    /* set bs = 3 for intra predict mb */
    if( IS_INTRA( h->mb.i_type ) )
    {
        memset( bs[0][1], 3, 12 );
        memset( bs[1][1], 3, 12 );
        return;
    }

    h->mb.i_neighbour = h->mb.i_neighbour_frame;
    h->loopf.deblock_strength( h->mb.cache.non_zero_count, h->mb.cache.ref, h->mb.cache.mv, bs );
}

static void ALWAYS_INLINE x264_macroblock_store_pic( x264_t *h, int mb_x, int mb_y )
{
    int i_offset_luma   = (mb_x << 4) + ((mb_y * h->fdec->i_stride[0]) << 4);
    int i_offset_chroma = (mb_x << 4) + ((mb_y * h->fdec->i_stride[1]) << 3);

    if( h->sh.i_type == SLICE_TYPE_I )
    {
        /* In MBAFF we store the last two rows in intra_border_backup[0] and [1].
         * For progressive mbs this is the bottom two rows, and for interlaced the
         * bottom row of each field. We also store samples needed for the next
         * mbpair in intra_border_backup[2]. */
        memcpy( &h->intra_border_backup[mb_y&1][0][mb_x<<4],       h->mb.pic.p_fdec[0] + 15 * FDEC_STRIDE, 16 );
        memcpy( &h->intra_border_backup[mb_y&1][1][mb_x<<4],       h->mb.pic.p_fdec[1] + (15>>CHROMA_V_SHIFT) * FDEC_STRIDE, 8 );
        memcpy( &h->intra_border_backup[mb_y&1][1][(mb_x<<4) + 8], h->mb.pic.p_fdec[2] + (15>>CHROMA_V_SHIFT) * FDEC_STRIDE, 8 );
    }

    h->mc.copy[PIXEL_16x16]( &h->fdec->plane[0][i_offset_luma], h->fdec->i_stride[0], h->mb.pic.p_fdec[0], FDEC_STRIDE, 16 );
    h->mc.store_interleave_chroma( &h->fdec->plane[1][i_offset_chroma], h->fdec->i_stride[1], h->mb.pic.p_fdec[1], h->mb.pic.p_fdec[2], 8 );
}

void x264_macroblock_cache_save( x264_t *h )
{
    const int i_mb_xy = h->mb.i_mb_xy;
    const int i_mb_type = x264_mb_type_fix[h->mb.i_type];
    const int s8x8 = h->mb.i_b8_stride;
    const int s4x4 = h->mb.i_b4_stride;
    const int i_mb_4x4 = h->mb.i_b4_xy;
    const int i_mb_8x8 = h->mb.i_b8_xy;

    /* GCC pessimizes direct stores to heap-allocated arrays due to aliasing. */
    /* By only dereferencing them once, we avoid this issue. */
    int8_t *i4x4 = h->mb.intra4x4_pred_mode[i_mb_xy];
    uint8_t *nnz = h->mb.non_zero_count[i_mb_xy];

    x264_macroblock_store_pic( h, h->mb.i_mb_x, h->mb.i_mb_y );
    x264_prefetch_fenc( h, h->fdec, h->mb.i_mb_x, h->mb.i_mb_y );

    h->mb.type[i_mb_xy] = i_mb_type;
    h->mb.slice_table[i_mb_xy] = h->sh.i_first_mb;
    h->mb.partition[i_mb_xy] = IS_INTRA( i_mb_type ) ? D_16x16 : h->mb.i_partition;
    h->mb.i_mb_prev_xy = i_mb_xy;

    /* save intra4x4 */
    if( i_mb_type == I_4x4 )
    {
        CP32( &i4x4[0], &h->mb.cache.intra4x4_pred_mode[x264_scan8[10]] );
        M32( &i4x4[4] ) = pack8to32( h->mb.cache.intra4x4_pred_mode[x264_scan8[5] ],
                                     h->mb.cache.intra4x4_pred_mode[x264_scan8[7] ],
                                     h->mb.cache.intra4x4_pred_mode[x264_scan8[13] ], 0);
    }
    else
        M64( i4x4 ) = 0x0202020202020202ULL; /* I_PRED_4x4_DC * 0x0101010101010101ULL */

	if( h->mb.i_type != I_16x16 && h->mb.i_cbp_luma == 0 && h->mb.i_cbp_chroma == 0 )
		h->mb.i_qp = h->mb.i_last_qp;
	h->mb.qp[i_mb_xy] = h->mb.i_qp;
	h->mb.i_last_dqp = h->mb.i_qp - h->mb.i_last_qp;
	h->mb.i_last_qp = h->mb.i_qp;

    /* save non zero count */
    CP32( &nnz[ 0], &h->mb.cache.non_zero_count[x264_scan8[ 0]] );
    CP32( &nnz[ 4], &h->mb.cache.non_zero_count[x264_scan8[ 2]] );
    CP32( &nnz[ 8], &h->mb.cache.non_zero_count[x264_scan8[ 8]] );
    CP32( &nnz[12], &h->mb.cache.non_zero_count[x264_scan8[10]] );
    CP32( &nnz[16], &h->mb.cache.non_zero_count[x264_scan8[16]] );
    CP32( &nnz[20], &h->mb.cache.non_zero_count[x264_scan8[18]] );
    CP32( &nnz[32], &h->mb.cache.non_zero_count[x264_scan8[32]] );
    CP32( &nnz[36], &h->mb.cache.non_zero_count[x264_scan8[34]] );

    if( h->mb.i_cbp_luma == 0 && h->mb.i_type != I_8x8 )
        h->mb.b_transform_8x8 = 0;
    h->mb.mb_transform_size[i_mb_xy] = h->mb.b_transform_8x8;

    if( h->sh.i_type != SLICE_TYPE_I )
    {
        int16_t (*mv0)[2] = &h->mb.mv[0][i_mb_4x4];
        int8_t *ref0 = &h->mb.ref[0][i_mb_8x8];
        if( !IS_INTRA( i_mb_type ) )
        {
            ref0[0]      = h->mb.cache.ref[0][x264_scan8[0]];
            ref0[1]      = h->mb.cache.ref[0][x264_scan8[4]];
            ref0[0+s8x8] = h->mb.cache.ref[0][x264_scan8[8]];
            ref0[1+s8x8] = h->mb.cache.ref[0][x264_scan8[12]];
            CP128( &mv0[0*s4x4], h->mb.cache.mv[0][x264_scan8[0]+ 0] );
            CP128( &mv0[1*s4x4], h->mb.cache.mv[0][x264_scan8[0]+ 8] );
            CP128( &mv0[2*s4x4], h->mb.cache.mv[0][x264_scan8[0]+16] );
            CP128( &mv0[3*s4x4], h->mb.cache.mv[0][x264_scan8[0]+24] );
        }
        else
        {
            M16( &ref0[0] )      = 0xFFFFU; /* (uint8_t)(-1) * 0x0101 */
            M16( &ref0[s8x8] )   = 0xFFFFU; /* (uint8_t)(-1) * 0x0101 */
            M128( &mv0[0*s4x4] ) = M128_ZERO;
            M128( &mv0[1*s4x4] ) = M128_ZERO;
            M128( &mv0[2*s4x4] ) = M128_ZERO;
            M128( &mv0[3*s4x4] ) = M128_ZERO;
        }
    }

    if( h->param.b_cabac )
    {
        uint8_t (*mvd0)[2] = h->mb.mvd[0][i_mb_xy];
        if( IS_INTRA(i_mb_type) )
            h->mb.chroma_pred_mode[i_mb_xy] = x264_mb_chroma_pred_mode_fix[h->mb.i_chroma_pred_mode];
        else
            h->mb.chroma_pred_mode[i_mb_xy] = I_PRED_CHROMA_DC;

        if( (0x3FF30 >> i_mb_type) & 1 ) /* !INTRA && !SKIP && !DIRECT */
        {
            CP64( mvd0[0], h->mb.cache.mvd[0][x264_scan8[10]] );
            CP16( mvd0[4], h->mb.cache.mvd[0][x264_scan8[5 ]] );
            CP16( mvd0[5], h->mb.cache.mvd[0][x264_scan8[7 ]] );
            CP16( mvd0[6], h->mb.cache.mvd[0][x264_scan8[13]] );
        }
        else
        {
            M128( mvd0[0] ) = M128_ZERO;
        }
    }
}
