/*****************************************************************************
 * lookahead.c: high-level lookahead functions
 *****************************************************************************/

/******************************************************************************************
 * LOOKAHEAD (threaded and non-threaded mode)                                             *
 *                                                                                        *
 * Lookahead types:                                                                       *
 *     [1] Slice type / scene cut;                                                        *
 *                                                                                        *
 * In non-threaded mode, we run the existing slicetype decision code as it was.           *
 * In threaded mode, we run in a separate thread, that lives between the calls            *
 * to x264_encoder_open() and x264_encoder_close(), and performs lookahead for            *
 * the number of frames specified in rc_lookahead.  Recommended setting is                *
 * # of bframes + # of threads.                                                           *
 *                                                                                        *
 * x264 has a complex lookahead module designed to estimate the coding cost of            *
 * frames that have not yet been analyzed by the main encoder module. It uses             *
 * these estimations to make a variety of decisions, such as adaptive B-frame             *
 * placement, explicit weighted prediction, and bit allocation for buffer-constrained     *
 * ratecontrol. For performance reasons, it operates on a half-resolution version of      *
 * the frame and calculates SATD residuals only, doing no quantization or reconstruction. *
 ******************************************************************************************/
#include "common/common.h"
#include "analyse.h"

int x264_lookahead_init( x264_t *h, int i_slicetype_length )
{
    x264_lookahead_t *look;
	int i;
    CHECKED_MALLOCZERO( look, sizeof(x264_lookahead_t) );
    for( i = 0; i < h->param.i_threads; i++ )
        h->thread[i]->lookahead = look;

    look->i_last_keyframe = - h->param.i_keyint_max;
    look->b_analyse_keyframe = h->param.rc.b_mb_tree || (h->param.rc.i_vbv_buffer_size && h->param.rc.i_lookahead);
    look->i_slicetype_length = i_slicetype_length; /* h->frames.i_delay */

    /* init frame lists */
    if( x264_sync_frame_list_init( &look->next, h->frames.i_delay+3 ) ||
        x264_sync_frame_list_init( &look->ofbuf, h->frames.i_delay+3 ) )
        goto fail;

    return 0;
fail:
    x264_free( look );
    return -1;
}

void x264_lookahead_delete( x264_t *h )
{
    x264_sync_frame_list_delete( &h->lookahead->next );
    if( h->lookahead->last_nonb )
        x264_frame_push_unused( h, h->lookahead->last_nonb );
    x264_sync_frame_list_delete( &h->lookahead->ofbuf );
    x264_free( h->lookahead );
}

void x264_lookahead_put_frame( x264_t *h, x264_frame_t *frame )
{
    x264_sync_frame_list_push( &h->lookahead->next, frame );
}

int x264_lookahead_is_empty( x264_t *h )
{
    return !h->lookahead->next.i_size && !h->lookahead->ofbuf.i_size;
}

static void x264_lookahead_encoder_shift( x264_t *h )
{
	int i_frames;
    if( !h->lookahead->ofbuf.i_size )
        return;
    i_frames = h->lookahead->ofbuf.list[0]->i_bframes + 1;
    while( i_frames-- )
    {
        x264_frame_push( h->frames.current, x264_frame_shift( h->lookahead->ofbuf.list ) );
        h->lookahead->ofbuf.i_size--;
    }
}

static void x264_lookahead_shift( x264_sync_frame_list_t *dst, x264_sync_frame_list_t *src, int count )
{
    int i = count;
    while( i-- )
    {
        /*assert( dst->i_size < dst->i_max_size );*/
        /*assert( src->i_size );*/
        dst->list[ dst->i_size++ ] = x264_frame_shift( src->list );
        src->i_size--;
    }
}

static void x264_lookahead_update_last_nonb( x264_t *h, x264_frame_t *new_nonb )
{
    if( h->lookahead->last_nonb )
        x264_frame_push_unused( h, h->lookahead->last_nonb ); /* recycle old nonb */
    h->lookahead->last_nonb = new_nonb;
    new_nonb->i_reference_count++;
}

void x264_lookahead_get_frames( x264_t *h )
{
	/* We are not running a lookahead thread, so perform all the slicetype decide on the fly */
	if( h->frames.current[0] || !h->lookahead->next.i_size )
		return;

	/* x264_slicetype_decide runs the core decision of slice type  */
	x264_stack_align( x264_slicetype_decide, h );
	x264_lookahead_update_last_nonb( h, h->lookahead->next.list[0] );
	/* shift frame from h->lookahead->next to h->lookahead->ofbuf */
	x264_lookahead_shift( &h->lookahead->ofbuf, &h->lookahead->next, h->lookahead->next.list[0]->i_bframes + 1 );
	/* shift frame from h->lookahead->ofbuf to h->frames.current for later encoding */
	x264_lookahead_encoder_shift( h );
}
