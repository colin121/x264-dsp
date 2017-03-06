/*****************************************************************************
 * frame.c: frame handling
 *****************************************************************************/

#include "common.h"

static int align_stride( int x, int align, int disalign )
{
    x = ALIGN( x, align );
    if( !(x&(disalign-1)) )
        x += align;
    return x;
}

static int align_plane_size( int x, int disalign )
{
    if( !(x&(disalign-1)) )
        x += 128;
    return x;
}

static x264_frame_t *x264_frame_new( x264_t *h, int b_fdec )
{
    x264_frame_t *frame;
    int i_mb_count = h->mb.i_mb_count;
    int i_stride, i_width, i_lines;
    int luma_padv = PADV;
    int chroma_padv = PADV >> 1;
    int luma_plane_size;
    int chroma_plane_size;
    int align = 16;
    int disalign = 1<<10;
	int i, j;

    CHECKED_MALLOCZERO( frame, sizeof(x264_frame_t) );

    /* allocate frame data (+64 for extra data for me) */
    i_width  = h->mb.i_mb_width << 4;
    i_lines  = h->mb.i_mb_height << 4;
    i_stride = align_stride( i_width + (PADH << 1), align, disalign );

	frame->i_plane = 2; /* luma and chroma */
	for( i = 0; i < 2; i++ )
	{
		frame->i_width[i] = i_width >> i;
		frame->i_lines[i] = i_lines >> i;
		frame->i_stride[i] = i_stride;
	}

    frame->i_csp = X264_CSP_I420;
    frame->i_width_lowres = frame->i_width[0] >> 1;
    frame->i_lines_lowres = frame->i_lines[0] >> 1;
    frame->i_stride_lowres = align_stride( frame->i_width_lowres + (PADH << 1), align, disalign<<1 );

    for( i = 0; i < h->param.i_bframe + 2; i++ )
        for( j = 0; j < h->param.i_bframe + 2; j++ )
            CHECKED_MALLOC( frame->i_row_satds[i][j], (i_lines >> 4) * sizeof(int) );

    frame->i_poc = -1;
    frame->i_type = X264_TYPE_AUTO;
    frame->i_qpplus1 = X264_QP_AUTO;
    frame->i_pts = -1;
    frame->i_frame = -1;
    frame->i_frame_num = -1;
    frame->i_lines_completed = -1;
    frame->b_fdec = b_fdec;
    frame->i_pic_struct = PIC_STRUCT_AUTO;
    frame->i_field_cnt = -1;
    frame->i_duration =
    frame->i_cpb_duration =
    frame->i_dpb_output_delay =
    frame->i_cpb_delay = 0;
    frame->i_coded_fields_lookahead =
    frame->i_cpb_delay_lookahead = -1;
    frame->orig = frame;

	/* chroma plane allocation */
	chroma_plane_size = (frame->i_stride[1] * (frame->i_lines[1] + 2*chroma_padv));
	CHECKED_MALLOC( frame->buffer[1], chroma_plane_size * sizeof(pixel) );
	frame->plane[1] = frame->buffer[1] + frame->i_stride[1] * chroma_padv + PADH;

	/* luma plane allocation */
	luma_plane_size = align_plane_size( frame->i_stride[0] * (frame->i_lines[0] + 2*luma_padv), disalign );
	if( h->param.analyse.i_subpel_refine && b_fdec )
	{
	    /* all 4 luma planes allocated together, since the cacheline split code
	     * requires them to be in-phase wrt cacheline alignment. */
		CHECKED_MALLOC( frame->buffer[0], 4*luma_plane_size * sizeof(pixel) );
		for( i = 0; i < 4; i++ )
			frame->filtered[0][i] = frame->buffer[0] + i*luma_plane_size + frame->i_stride[0] * luma_padv + PADH;
		frame->plane[0] = frame->filtered[0][0];
	}
	else
	{
		CHECKED_MALLOC( frame->buffer[0], luma_plane_size * sizeof(pixel) );
		frame->filtered[0][0] = frame->plane[0] = frame->buffer[0] + frame->i_stride[0] * luma_padv + PADH;
	}

    frame->b_duplicate = 0;

    if( b_fdec ) /* fdec frame */
    {
        CHECKED_MALLOC( frame->mb_type, i_mb_count * sizeof(int8_t));
        CHECKED_MALLOC( frame->mb_partition, i_mb_count * sizeof(uint8_t));
        CHECKED_MALLOC( frame->mv[0], 2*16 * i_mb_count * sizeof(int16_t) );
        CHECKED_MALLOC( frame->mv16x16, 2*(i_mb_count+1) * sizeof(int16_t) );
        M32( frame->mv16x16[0] ) = 0;
        frame->mv16x16++;
        CHECKED_MALLOC( frame->ref[0], 4 * i_mb_count * sizeof(int8_t) );
        if( h->param.i_bframe )
        {
            CHECKED_MALLOC( frame->mv[1], 2*16 * i_mb_count * sizeof(int16_t) );
            CHECKED_MALLOC( frame->ref[1], 4 * i_mb_count * sizeof(int8_t) );
        }
        else
        {
            frame->mv[1]  = NULL;
            frame->ref[1] = NULL;
        }
        CHECKED_MALLOC( frame->i_row_bits, (i_lines >> 4) * sizeof(int) );
        CHECKED_MALLOC( frame->f_row_qp, (i_lines >> 4) * sizeof(float) );
        CHECKED_MALLOC( frame->f_row_qscale, (i_lines >> 4) * sizeof(float) );
    }
    else /* fenc frame */
    {
        if( h->frames.b_have_lowres )
        {
        	/* half-resolution luma plane allocation */
            int luma_plane_size = align_plane_size( frame->i_stride_lowres * ((frame->i_lines[0]>>1) + (PADV<<1)), disalign );
            CHECKED_MALLOC( frame->buffer_lowres[0], 4 * luma_plane_size * sizeof(pixel) );
            for( i = 0; i < 4; i++ )
                frame->lowres[i] = frame->buffer_lowres[0] + (frame->i_stride_lowres * PADV + PADH) + i * luma_plane_size;

            for( j = 0; j <= !!h->param.i_bframe; j++ )
                for( i = 0; i <= h->param.i_bframe; i++ )
                {
                    CHECKED_MALLOCZERO( frame->lowres_mvs[j][i], 2*h->mb.i_mb_count*sizeof(int16_t) );
                    CHECKED_MALLOC( frame->lowres_mv_costs[j][i], h->mb.i_mb_count*sizeof(int) );
                }
        }
        if( h->param.rc.i_aq_mode )
        {
            CHECKED_MALLOC( frame->f_qp_offset, h->mb.i_mb_count * sizeof(float) );
            CHECKED_MALLOC( frame->f_qp_offset_aq, h->mb.i_mb_count * sizeof(float) );
        }
    }

    return frame;

fail:
    x264_free( frame );
    return NULL;
}

void x264_frame_delete( x264_frame_t *frame )
{
	int i, j;
    /* Duplicate frames are blank copies of real frames (including pointers),
     * so freeing those pointers would cause a double free later. */
    if( !frame->b_duplicate )
    {
        for( i = 0; i < 4; i++ )
            x264_free( frame->buffer[i] );
        for( i = 0; i < 4; i++ )
            x264_free( frame->buffer_lowres[i] );
        for( i = 0; i < X264_BFRAME_MAX+2; i++ )
            for( j = 0; j < X264_BFRAME_MAX+2; j++ )
                x264_free( frame->i_row_satds[i][j] );
        for( j = 0; j < 2; j++ )
            for( i = 0; i <= X264_BFRAME_MAX; i++ )
            {
                x264_free( frame->lowres_mvs[j][i] );
                x264_free( frame->lowres_mv_costs[j][i] );
            }
        x264_free( frame->f_qp_offset );
        x264_free( frame->f_qp_offset_aq );
        x264_free( frame->i_row_bits );
        x264_free( frame->f_row_qp );
        x264_free( frame->f_row_qscale );
        x264_free( frame->field );
        x264_free( frame->effective_qp );
        x264_free( frame->mb_type );
        x264_free( frame->mb_partition );
        x264_free( frame->mv[0] );
        x264_free( frame->mv[1] );
        if( frame->mv16x16 )
            x264_free( frame->mv16x16-1 );
        x264_free( frame->ref[0] );
        x264_free( frame->ref[1] );
        if( frame->param && frame->param->param_free )
            frame->param->param_free( frame->param );
        if( frame->mb_info_free )
            frame->mb_info_free( frame->mb_info );
    }
    x264_free( frame );
}

int x264_frame_copy_picture( x264_t *h, x264_frame_t *dst, x264_picture_t *src )
{
    dst->i_type       = src->i_type;
    dst->i_qpplus1    = src->i_qpplus1;
    dst->i_pts        = dst->i_reordered_pts = src->i_pts;
    dst->param        = src->param;
    dst->i_pic_struct = src->i_pic_struct;
    dst->opaque       = src->opaque;
    dst->mb_info      = h->param.analyse.b_mb_info ? src->prop.mb_info : NULL;
    dst->mb_info_free = h->param.analyse.b_mb_info ? src->prop.mb_info_free : NULL;

    if( src->img.i_csp == X264_CSP_NV16 )
    {
    	/* NV16(YUV422 Semi-Planar) => NV12(YUV420 Semi-Planar) with motion-detect deinterlacing */
    	h->mc.plane_copy_deinterlace(
    			dst->plane[0], dst->i_stride[0], src->img.plane[0], src->img.i_stride[0],
    			dst->plane[1], dst->i_stride[1], src->img.plane[1], src->img.i_stride[1],
    			h->param.i_width, h->param.i_height );
    }
    else if( src->img.i_csp == X264_CSP_NV12 )
	{
		/* copy luma data directly and chroma data with uv interleaved */
		h->mc.plane_copy( dst->plane[0], dst->i_stride[0], src->img.plane[0], src->img.i_stride[0], h->param.i_width, h->param.i_height );
        h->mc.plane_copy( dst->plane[1], dst->i_stride[1], src->img.plane[1], src->img.i_stride[1], h->param.i_width, h->param.i_height>>1 );
        /* motion-detect deinterlacing used only for TI DM6467 */
        /* h->mc.plane_deinterlace( dst->plane[0], dst->i_stride[0], dst->plane[1], dst->i_stride[1], h->param.i_width, h->param.i_height ); */
	}
	else /* X264_CSP_I420 */
	{
		h->mc.plane_copy( dst->plane[0], dst->i_stride[0], src->img.plane[0], src->img.i_stride[0], h->param.i_width, h->param.i_height );
		h->mc.plane_copy_interleave( dst->plane[1], dst->i_stride[1], src->img.plane[1], src->img.i_stride[1], src->img.plane[2], src->img.i_stride[2], h->param.i_width>>1, h->param.i_height>>1 );
	}

    return 0;
}

static void ALWAYS_INLINE pixel_memset( pixel *dst, pixel *src, int len, int size )
{
    uint8_t *dstp = (uint8_t*)dst;
    uint32_t v1 = *src;
    uint32_t v2 = size == 1 ? v1 + (v1 <<  8) : M16( src );
    uint32_t v4 = size <= 2 ? v2 + (v2 << 16) : M32( src );
    int i = 0;
    len *= size;

    /* Align the input pointer if it isn't already */
    if( (intptr_t)dstp & (WORD_SIZE - 1) )
    {
        if( size <= 2 && ((intptr_t)dstp & 3) )
        {
            if( size == 1 && ((intptr_t)dstp & 1) )
                dstp[i++] = v1;
            if( (intptr_t)dstp & 2 )
            {
                M16( dstp+i ) = v2;
                i += 2;
            }
        }
        if( WORD_SIZE == 8 && (intptr_t)dstp & 4 )
        {
            M32( dstp+i ) = v4;
            i += 4;
        }
    }

    /* Main copy loop */
    if( WORD_SIZE == 8 )
    {
        uint64_t v8 = v4 + ((uint64_t)v4<<32);
        for( ; i < len - 7; i+=8 )
            M64( dstp+i ) = v8;
    }
    for( ; i < len - 3; i+=4 )
        M32( dstp+i ) = v4;

    /* Finish up the last few bytes */
    if( size <= 2 )
    {
        if( i < len - 1 )
        {
            M16( dstp+i ) = v2;
            i += 2;
        }
        if( size == 1 && i != len )
            dstp[i] = v1;
    }
}

#ifdef __TI_COMPILER_VERSION__
static void ALWAYS_INLINE plane_expand_border( pixel *pix, int i_stride, int i_width, int i_height, int i_padh, int i_padv, int b_pad_top, int b_pad_bottom, int b_chroma )
{
#define PPIXEL(x, y) ( pix + (x) + (y)*i_stride )
	uint32_t pixl_v1, pixr_v1, pixl_v2, pixr_v2, pixl_v4, pixr_v4;
	uint64_t pixl_v8, pixr_v8, * pixl_src, * pixr_src;
	int y;

	/* left/right band: copy left/right edge pixel value to left/right paddings */
	/* NOTE: here we suppose left/right band lenght is constant to 32 (PADH).   */
	if( b_chroma )
	{
#pragma MUST_ITERATE(8, , 2)
	    for( y = 0; y < i_height; y++ )
	    {
	    	pixl_v2 = *((uint16_t*)PPIXEL(0, y));
	    	pixr_v2 = *((uint16_t*)PPIXEL(i_width-2, y));
	    	pixl_v4 = _pack2(pixl_v2, pixl_v2);
	    	pixr_v4 = _pack2(pixr_v2, pixr_v2);
	    	pixl_v8 = _itoll(pixl_v4, pixl_v4);
	    	pixr_v8 = _itoll(pixr_v4, pixr_v4);
	    	pixl_src = (uint64_t*)PPIXEL(-i_padh, y);
	    	pixr_src = (uint64_t*)PPIXEL(i_width, y);
	    	_mem8(pixl_src++) = pixl_v8;
	    	_mem8(pixr_src++) = pixr_v8;
	    	_mem8(pixl_src++) = pixl_v8;
	    	_mem8(pixr_src++) = pixr_v8;
	    	_mem8(pixl_src++) = pixl_v8;
	    	_mem8(pixr_src++) = pixr_v8;
	    	_mem8(pixl_src++) = pixl_v8;
	    	_mem8(pixr_src++) = pixr_v8;
	    }
	}
	else
	{
#pragma MUST_ITERATE(16, , 4)
	    for( y = 0; y < i_height; y++ )
	    {
	    	pixl_v1 = *((uint8_t*)PPIXEL(0, y));
	    	pixr_v1 = *((uint8_t*)PPIXEL(i_width-1, y));
	    	pixl_v2 = _pack2(pixl_v1, pixl_v1);
	    	pixr_v2 = _pack2(pixr_v1, pixr_v1);
	    	pixl_v4 = _packl4(pixl_v2, pixl_v2);
	    	pixr_v4 = _packl4(pixr_v2, pixr_v2);
	    	pixl_v8 = _itoll(pixl_v4, pixl_v4);
	    	pixr_v8 = _itoll(pixr_v4, pixr_v4);
	    	pixl_src = (uint64_t*)PPIXEL(-i_padh, y);
	    	pixr_src = (uint64_t*)PPIXEL(i_width, y);
	    	_mem8(pixl_src++) = pixl_v8;
	    	_mem8(pixr_src++) = pixr_v8;
	    	_mem8(pixl_src++) = pixl_v8;
	    	_mem8(pixr_src++) = pixr_v8;
	    	_mem8(pixl_src++) = pixl_v8;
	    	_mem8(pixr_src++) = pixr_v8;
	    	_mem8(pixl_src++) = pixl_v8;
	    	_mem8(pixr_src++) = pixr_v8;
	    }
	}

    if( b_pad_top )
    {
    	/* upper band: copy top line pixel value to top paddings */
#pragma MUST_ITERATE(16, 32, 8)
        for( y = 0; y < i_padv; y++ )
            memcpy( PPIXEL(-i_padh, -y-1), PPIXEL(-i_padh, 0), i_width+(i_padh<<1) );
    }

    if( b_pad_bottom )
    {
    	/* lower band: copy bottom line pixel value to bottom paddings */
#pragma MUST_ITERATE(16, 32, 8)
        for( y = 0; y < i_padv; y++ )
            memcpy( PPIXEL(-i_padh, i_height+y), PPIXEL(-i_padh, i_height-1), i_width+(i_padh<<1) );
    }
#undef PPIXEL
}
#else
static void ALWAYS_INLINE plane_expand_border( pixel *pix, int i_stride, int i_width, int i_height, int i_padh, int i_padv, int b_pad_top, int b_pad_bottom, int b_chroma )
{
#define PPIXEL(x, y) ( pix + (x) + (y)*i_stride )
	int y;
    for( y = 0; y < i_height; y++ )
    {
        /* left band: copy left edge pixel value to left paddings */
        pixel_memset( PPIXEL(-i_padh, y), PPIXEL(0, y), i_padh>>b_chroma, sizeof(pixel)<<b_chroma );
        /* right band: copy right edge pixel value to right paddings */
        pixel_memset( PPIXEL(i_width, y), PPIXEL(i_width-1-b_chroma, y), i_padh>>b_chroma, sizeof(pixel)<<b_chroma );
    }
    /* upper band: copy top line pixel value to top paddings */
    if( b_pad_top )
        for( y = 0; y < i_padv; y++ )
            memcpy( PPIXEL(-i_padh, -y-1), PPIXEL(-i_padh, 0), (i_width+2*i_padh) * sizeof(pixel) );
    /* lower band: copy bottom line pixel value to bottom paddings */
    if( b_pad_bottom )
        for( y = 0; y < i_padv; y++ )
            memcpy( PPIXEL(-i_padh, i_height+y), PPIXEL(-i_padh, i_height-1), (i_width+2*i_padh) * sizeof(pixel) );
#undef PPIXEL
}
#endif

void x264_frame_expand_border( x264_t *h, x264_frame_t *frame, int mb_y )
{
    int pad_top = mb_y == 0;
    int pad_bot = mb_y == h->mb.i_mb_height - 1;
    int width   = h->mb.i_mb_width<<4;
    int height  = pad_bot ? 20 : 16; // buffer: 2 chroma, 3 luma (rounded to 4) because deblocking goes beyond the top of the mb
    int starty  = (mb_y<<4) - ((!pad_top)<<2);

    plane_expand_border( frame->plane[0] + (starty*frame->i_stride[0]),      frame->i_stride[0], width, height,    PADH, PADV,    pad_top, pad_bot, 0 );
    plane_expand_border( frame->plane[1] + ((starty*frame->i_stride[1])>>1), frame->i_stride[1], width, height>>1, PADH, PADV>>1, pad_top, pad_bot, 1 );
}

void x264_frame_expand_border_filtered( x264_t *h, x264_frame_t *frame, int mb_y, int b_end )
{
    /****************************************************************
     * during filtering, 8 extra pixels were filtered on each edge, *
     * but up to 3 of the horizontal ones may be wrong.             *
     * we want to expand border from the last filtered pixel.       *
     ****************************************************************/
    int b_start = !mb_y;
    int width   = (h->mb.i_mb_width<<4) + 8;
    int height  = b_end ? ((h->mb.i_mb_height - mb_y)<<4) + 16 : 16;
    int offset  = ((mb_y<<4) - 8) * frame->i_stride[0]/* - 4*/; // buffer: 8 luma, to match the hpel filter

	plane_expand_border( frame->filtered[0][1]+offset, frame->i_stride[0], width, height, PADH/*-4*/, PADV-8, b_start, b_end, 0 );
	plane_expand_border( frame->filtered[0][2]+offset, frame->i_stride[0], width, height, PADH/*-4*/, PADV-8, b_start, b_end, 0 );
	plane_expand_border( frame->filtered[0][3]+offset, frame->i_stride[0], width, height, PADH/*-4*/, PADV-8, b_start, b_end, 0 );
}

void x264_frame_expand_border_lowres( x264_frame_t *frame )
{
    plane_expand_border( frame->lowres[0], frame->i_stride_lowres, frame->i_width_lowres, frame->i_lines_lowres, PADH, PADV, 1, 1, 0 );
    plane_expand_border( frame->lowres[1], frame->i_stride_lowres, frame->i_width_lowres, frame->i_lines_lowres, PADH, PADV, 1, 1, 0 );
    plane_expand_border( frame->lowres[2], frame->i_stride_lowres, frame->i_width_lowres, frame->i_lines_lowres, PADH, PADV, 1, 1, 0 );
    plane_expand_border( frame->lowres[3], frame->i_stride_lowres, frame->i_width_lowres, frame->i_lines_lowres, PADH, PADV, 1, 1, 0 );
}

void x264_frame_expand_border_mod16( x264_t *h, x264_frame_t *frame )
{
	int i, y;
    for( i = 0; i < frame->i_plane; i++ )
    {
        int i_width = h->param.i_width;
        int h_shift = i && CHROMA_H_SHIFT;
        int v_shift = i && CHROMA_V_SHIFT;
        int i_height = h->param.i_height >> v_shift;
        int i_padx = (h->mb.i_mb_width * 16 - h->param.i_width);
        int i_pady = (h->mb.i_mb_height * 16 - h->param.i_height) >> v_shift;

        if( i_padx )
        {
            for( y = 0; y < i_height; y++ )
                pixel_memset( &frame->plane[i][y*frame->i_stride[i] + i_width],
                              &frame->plane[i][y*frame->i_stride[i] + i_width - 1-h_shift],
                              i_padx>>h_shift, sizeof(pixel)<<h_shift );
        }
        if( i_pady )
        {
            for( y = i_height; y < i_height + i_pady; y++ )
                memcpy( &frame->plane[i][y*frame->i_stride[i]],
                        &frame->plane[i][(i_height-(~y&PARAM_INTERLACED)-1)*frame->i_stride[i]],
                        (i_width + i_padx) * sizeof(pixel) );
        }
    }
}

/* list (variable array) operators */

void x264_frame_push( x264_frame_t **list, x264_frame_t *frame )
{
    int i = 0;
    while( list[i] ) i++;
    list[i] = frame;
}

x264_frame_t *x264_frame_pop( x264_frame_t **list )
{
    x264_frame_t *frame;
    int i = 0;
    /*assert( list[0] );*/
    while( list[i+1] ) i++;
    frame = list[i];
    list[i] = NULL;
    return frame;
}

void x264_frame_unshift( x264_frame_t **list, x264_frame_t *frame )
{
    int i = 0;
    while( list[i] ) i++;
    while( i-- )
        list[i+1] = list[i];
    list[0] = frame;
}

x264_frame_t *x264_frame_shift( x264_frame_t **list )
{
    x264_frame_t *frame = list[0];
    int i;
    for( i = 0; list[i]; i++ )
        list[i] = list[i+1];
    /*assert(frame);*/
    return frame;
}

void x264_frame_push_unused( x264_t *h, x264_frame_t *frame )
{
    /*assert( frame->i_reference_count > 0 );*/
    frame->i_reference_count--;
    if( frame->i_reference_count == 0 )
        x264_frame_push( h->frames.unused[frame->b_fdec], frame );
}

x264_frame_t *x264_frame_pop_unused( x264_t *h, int b_fdec )
{
    x264_frame_t *frame;
    if( h->frames.unused[b_fdec][0] )
        frame = x264_frame_pop( h->frames.unused[b_fdec] );
    else
        frame = x264_frame_new( h, b_fdec );
    if( !frame )
        return NULL;

    frame->b_last_minigop_bframe = 0;
    frame->i_reference_count = 1;
    frame->b_intra_calculated = 0;
    frame->b_keyframe = 0;
    frame->b_corrupt = 0;
    memset( frame->weight, 0, sizeof(frame->weight) );

    return frame;
}

void x264_frame_push_blank_unused( x264_t *h, x264_frame_t *frame )
{
    /*assert( frame->i_reference_count > 0 );*/
    frame->i_reference_count--;
    if( frame->i_reference_count == 0 )
        x264_frame_push( h->frames.blank_unused, frame );
}

x264_frame_t *x264_frame_pop_blank_unused( x264_t *h )
{
    x264_frame_t *frame;
    if( h->frames.blank_unused[0] )
        frame = x264_frame_pop( h->frames.blank_unused );
    else
        frame = x264_malloc( sizeof(x264_frame_t) );
    if( !frame )
        return NULL;
    frame->b_duplicate = 1;
    frame->i_reference_count = 1;
    return frame;
}

void x264_frame_delete_list( x264_frame_t **list )
{
    int i = 0;
    if( !list )
        return;
    while( list[i] )
        x264_frame_delete( list[i++] );
    x264_free( list );
}

int x264_sync_frame_list_init( x264_sync_frame_list_t *slist, int max_size )
{
    if( max_size < 0 )
        return -1;
    slist->i_max_size = max_size;
    slist->i_size = 0;
    CHECKED_MALLOCZERO( slist->list, (max_size+1) * sizeof(x264_frame_t*) );
    return 0;
fail:
    return -1;
}

void x264_sync_frame_list_delete( x264_sync_frame_list_t *slist )
{
    x264_frame_delete_list( slist->list );
}

void x264_sync_frame_list_push( x264_sync_frame_list_t *slist, x264_frame_t *frame )
{
    slist->list[ slist->i_size++ ] = frame;
}

x264_frame_t *x264_sync_frame_list_pop( x264_sync_frame_list_t *slist )
{
    x264_frame_t *frame;
    frame = slist->list[ --slist->i_size ];
    slist->list[ slist->i_size ] = NULL;
    return frame;
}
