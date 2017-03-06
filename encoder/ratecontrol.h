/*****************************************************************************
 * ratecontrol.h: ratecontrol
 *****************************************************************************/

#ifndef X264_RATECONTROL_H
#define X264_RATECONTROL_H

/**********************************************************************************
 * Completely arbitrary. Ratecontrol lowers relative quality at higher framerates *
 * and the reverse at lower framerates; this serves as the center of the curve.   *
 * Halve all the values for frame-packed 3D to compensate for the "doubled"       *
 * framerate. (frame_packing == 5)                                                *
 * Max and Min frame duration is Arbitrary limitations as a sanity check.         *
 **********************************************************************************/
#define BASE_FRAME_DURATION (0.04f) /* (0.04f / ((h->param.i_frame_packing == 5)+1)) */
#define MAX_FRAME_DURATION  (1.00f) /* (1.00f / ((h->param.i_frame_packing == 5)+1)) */
#define MIN_FRAME_DURATION  (0.01f) /* (0.01f / ((h->param.i_frame_packing == 5)+1)) */
#define CLIP_DURATION(f)    x264_clip3f(f, MIN_FRAME_DURATION, MAX_FRAME_DURATION)

int  x264_ratecontrol_new   ( x264_t * );
void x264_ratecontrol_delete( x264_t * );
void x264_ratecontrol_init_reconfigurable( x264_t *h, int b_init );
void x264_adaptive_quant_frame( x264_t *h, x264_frame_t *frame, float *quant_offsets );
void x264_ratecontrol_start( x264_t *, int i_force_qp, int overhead );
int  x264_ratecontrol_mb( x264_t *, int bits );
int  x264_ratecontrol_qp( x264_t * );
int  x264_ratecontrol_mb_qp( x264_t *h );
int  x264_ratecontrol_end( x264_t *, int bits, int *filler );
void x264_ratecontrol_summary( x264_t * );
void x264_ratecontrol_set_estimated_size( x264_t *, int bits );
int  x264_ratecontrol_get_estimated_size( x264_t const *);
int  x264_rc_analyse_slice( x264_t *h );
#endif

