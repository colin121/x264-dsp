/*****************************************************************************
 * ratecontrol.c: ratecontrol
 *****************************************************************************/

#include "common/common.h"
#include "ratecontrol.h"
#include "me.h"

/**********************************************************************************
 * Here are some important ways ratecontrol works now:                            *
 * - By default, MB-tree is used instead of qcomp for weighting frame quality     *
 * based on complexity. MB-tree is effectively a generalization of qcomp to       *
 * the macroblock level. MB-tree also replaces the constant offsets for B-frame   *
 * quantizers. The legacy algorithm is still available for low-latency scenarios. *
 * - Adaptive quantization is now used to distribute quality among mbs of each    *
 * frame by computing AC energy of mb. frames are no longer constant quantizer,   *
 * even if MB-tree is off.                                                        *
 * - VBV runs per-row rather than per-frame to improve accuracy.                  *
 *                                                                                *
 * Call hierarchy of rate control:                                                *
 * 1. x264_encoder_open()                                                         *
 *    ->-> x264_ratecontrol_new(): init rate control module for encoder.          *
 *                                                                                *
 * 2. x264_encoder_encode()                                                       *
 *    ->-> x264_adaptive_quant_frame(): compute AC energy and deduce qp offset    *
 *         for all mbs of current frame. preparation of Adaptive Quantization.    *
 *                                                                                *
 *    ->-> x264_ratecontrol_start(): before encoding, choose a qp for this frame. *
 *         a. compute complexity of frame by weighting avg of satd of slices.     *
 *         b. compute qscale = complexity ^ (1 - qcomp)                           *
 *         c. compute qscale = qscale / rate_factor                               *
 *         d. compute qp = 12 + 6 * log2 (qscale / 0.85)                          *
 *                                                                                *
 *    ->-> x264_ratecontrol_qp(): load qp of current frame.                       *
 *                                                                                *
 *    ->-> x264_slice_write()                                                     *
 *         ->-> x264_ratecontrol_mb_qp(): load qp for each mb. return qp of       *
 *              current frame, plus qp offset if AQ is enabled.                   *
 *         ->-> x264_ratecontrol_mb(): update mb bits per row for VBV mode.       *
 *                                                                                *
 *    ->-> x264_ratecontrol_end(): After encoding a frame, update and save stats  *
 *                                                                                *
 * 3. x264_encoder_close()                                                        *
 *    ->-> x264_ratecontrol_summary()                                             *
 *    ->-> x264_ratecontrol_delete()                                              *
 **********************************************************************************/

typedef struct
{
    int pict_type;
    int frame_type;
    int kept_as_ref;
    double qscale;
    int mv_bits;   /* MV bits (MV+Ref+Block Type) */
    int tex_bits;  /* Texture bits (DCT coefs) */
    int misc_bits; /* Miscellaneous bits */
    uint64_t expected_bits; /*total expected bits up to the current frame (current one excluded)*/
    double expected_vbv;
    double new_qscale;
    int new_qp;
    int i_count;
    int p_count;
    int s_count;
    float blurred_complexity;
    char direct_mode;
    int16_t weight[3][2];
    int16_t i_weight_denom[2];
    int refcount[16];
    int refs;
    int64_t i_duration;
    int64_t i_cpb_duration;
} ratecontrol_entry_t;

typedef struct
{
    float coeff_min;
    float coeff;
    float count;
    float decay;
    float offset;
} predictor_t;

struct x264_ratecontrol_t
{
    /* constants */
    int b_abr;   /* whether use abr. all modes except CQP use abr as base mode. */
    int b_2pass; /* 2 pass mode */
    int b_vbv;   /* whether enables VBV */
    int b_vbv_min_rate;
    double fps;
    double bitrate; /* bits per second */
    double rate_tolerance;
    double qcompress;
    int nmb;                    /* number of macroblocks in a frame */
    int qp_constant[3];         /* constant qp for I/P/B. used only in CQP mode. default: 20/23/25 */

    /* current frame */
    ratecontrol_entry_t *rce;
    int qp;                     /* qp for current frame */
    float qpm;                  /* qp for current macroblock: precise float for AQ */
    float qpa_rc;               /* average of macroblocks' qp before aq */
    float qpa_rc_prev;
    int   qpa_aq;               /* average of macroblocks' qp after aq */
    int   qpa_aq_prev;
    float qp_novbv;             /* QP for the current frame if 1-pass VBV was disabled. */

    /* VBV stuff */
    double buffer_size;
    int64_t buffer_fill_final;
    double buffer_fill;         /* planned buffer, if all in-progress frames hit their bit budget */
    double buffer_rate;         /* # of bits added to buffer_fill after each frame */
    double vbv_max_rate;        /* # of bits added to buffer_fill per second */
    predictor_t *pred;          /* predict frame size from satd */
    int single_frame_vbv;
    double rate_factor_max_increment; /* Don't allow RF above (CRF + this value). */

    /* ABR stuff */
    int    last_satd;
    double last_rceq;
    double cplxr_sum;           /* sum of bits*qscale/rceq */
    double expected_bits_sum;   /* sum of qscale2bits after rceq, ratefactor, and overflow, only includes finished frames */
    int64_t filler_bits_sum;    /* sum in bits of finished frames' filler data */
    double wanted_bits_window;  /* target bitrate * window */
    double cbr_decay;
    double short_term_cplxsum;
    double short_term_cplxcount;
    double rate_factor_constant;
    double ip_offset;           /* constant qp offset between I and P */
    double pb_offset;           /* constant qp offset between p and B */

    int num_entries;            /* number of ratecontrol_entry_ts */
    ratecontrol_entry_t *entry; /* FIXME: copy needed data and free this once init is done */
    double last_qscale;
    double last_qscale_for[3];  /* last qscale for a specific pict type, used for max_diff & ipb factor stuff */
    int last_non_b_pict_type;
    double accum_p_qp;          /* for determining I-frame quant */
    double accum_p_norm;
    double last_accum_p_norm;
    double lmin[3];             /* min qscale by frame type I/P/B */
    double lmax[3];             /* max qscale by frame type I/P/B */
    double lstep;               /* max change (multiply) in qscale per frame */
    struct
    {
        uint16_t *qp_buffer[2]; /* Global buffers for converting MB-tree quantizer data. */
        int qpbuf_pos;          /* In order to handle pyramid reordering, QP buffer acts as a stack.
                                 * This value is the current position (0 or 1). */
        int src_mb_count;
    } mbtree;

    /* MBRC stuff */
    float frame_size_estimated; /* Access to this variable must be atomic: double is
                                 * not atomic on all arches we care about */
    double frame_size_maximum;  /* Maximum frame size due to MinCR */
    double frame_size_planned;
    double slice_size_planned;
    predictor_t (*row_pred)[2];
    predictor_t row_preds[3][2];
    predictor_t *pred_b_from_p; /* predict B-frame size from P-frame satd */
    int bframes;                /* # consecutive B-frames before this P-frame */
    int bframe_bits;            /* total cost of those frames */

    /* hrd stuff */
    int initial_cpb_removal_delay;
    int initial_cpb_removal_delay_offset;
    double nrt_first_access_unit; /* nominal removal time */
    double previous_cpb_final_arrival_time;
    uint64_t hrd_multiply_denom;
};

static float rate_estimate_qscale( x264_t *h );
static int update_vbv( x264_t *h, int bits );
static void update_vbv_plan( x264_t *h, int overhead );
static float predict_size( predictor_t *p, float q, float var );
static void update_predictor( predictor_t *p, float q, float var, float bits );

/******************************************************
 * Terminology:                                       *
 * qp: h.264's quantizer parameter. range: [0-51]     *
 * qscale: linearized quantizer = Lagrange multiplier *
 * qscale = 0.85 * 2 ^ ((qp - 12) / 6)                *
 * qp = 12 + 6 * log2 (qscale / 0.85)                 *
 ******************************************************/
static inline float qp2qscale( float qp )
{
    return 0.85f * powf( 2.0f, ( qp - 12.0f ) / 6.0f );
}
static inline float qscale2qp( float qscale )
{
    return 12.0f + 6.0f * log2f( qscale / 0.85f );
}

static ALWAYS_INLINE uint32_t ac_energy_var( uint64_t sum_ssd, int shift, x264_frame_t *frame, int i, int b_store )
{
    uint32_t sum = sum_ssd;       /* 32 low bits: sum of pixel value */
    uint32_t ssd = sum_ssd >> 32; /* 32 high bits: sum of squared pixel value */
    if( b_store )
    {
        frame->i_pixel_sum[i] += sum;
        frame->i_pixel_ssd[i] += ssd;
    }
    /* sum of squared pixel value - square of summed pixel value >> shift */
    return ssd - ((uint64_t)sum * sum >> shift);
}

static ALWAYS_INLINE uint32_t ac_energy_plane( x264_t *h, int mb_x, int mb_y, x264_frame_t *frame, int i, int b_chroma, int b_store )
{
    int height = b_chroma ? 16>>CHROMA_V_SHIFT : 16;
    int stride = frame->i_stride[i];
    int offset = (mb_x<<4) + height * mb_y * stride;

    if( b_chroma )
    {
        ALIGNED_ARRAY_16( pixel, pix, [FENC_STRIDE*16] );
        int shift = 7 - CHROMA_V_SHIFT;

        /* chroma data are stored as uv interleaved mode, we need to deinterleave and compute ac energy separately */
        h->mc.load_deinterleave_chroma_fenc( pix, frame->plane[1] + offset, stride, height );
        return ac_energy_var( h->pixf.var[PIXEL_8x8]( pix,               FENC_STRIDE ), shift, frame, 1, b_store )  /* ac energy of chroma u */
             + ac_energy_var( h->pixf.var[PIXEL_8x8]( pix+FENC_STRIDE/2, FENC_STRIDE ), shift, frame, 2, b_store ); /* ac energy of chroma v */
    }
    else
        return ac_energy_var( h->pixf.var[PIXEL_16x16]( frame->plane[i] + offset, stride ), 8, frame, i, b_store );
}

/* Find the total AC energy of the block in all planes. */
static NOINLINE uint32_t x264_ac_energy_mb( x264_t *h, int mb_x, int mb_y, x264_frame_t *frame )
{
    uint32_t var;
    x264_prefetch_fenc( h, frame, mb_x, mb_y );

    var  = ac_energy_plane( h, mb_x, mb_y, frame, 0, 0, 1 ); /* ac energy of luma */
    var += ac_energy_plane( h, mb_x, mb_y, frame, 1, 1, 1 ); /* ac energy of chroma */

    return var;
}

/********************************************************************
 * Initialization for adaptive quantization of current frame.       *
 * quant_offsets is default to NULL unless input picture specifies. *
 ********************************************************************/
void x264_adaptive_quant_frame( x264_t *h, x264_frame_t *frame, float *quant_offsets )
{
    /* constants chosen to result in approximately the same overall bitrate as without AQ.
     * FIXME: while they're written in 5 significant digits, they're only tuned to 2. */
    int mb_x, mb_y, i;
	float strength;

    /* Initialize frame stats */
    for( i = 0; i < 3; i++ )
    {
        frame->i_pixel_sum[i] = 0;
        frame->i_pixel_ssd[i] = 0;
    }

    /* Degenerate cases: AQ mode is disabled */
    if( h->param.rc.i_aq_mode == X264_AQ_NONE || h->param.rc.f_aq_strength == 0 )
    {
        /* Need to init it anyway for MB tree */
        if( h->param.rc.i_aq_mode && h->param.rc.f_aq_strength == 0 )
        {
        	/* set qp offset to 0 for each mb */
        	memset( frame->f_qp_offset,    0, h->mb.i_mb_count * sizeof(float) );
        	memset( frame->f_qp_offset_aq, 0, h->mb.i_mb_count * sizeof(float) );
        }

        /* Need variance data for weighted prediction */
        if( h->param.analyse.i_weighted_pred )
        {
            for( mb_y = 0; mb_y < h->mb.i_mb_height; mb_y++ )
                for( mb_x = 0; mb_x < h->mb.i_mb_width; mb_x++ )
                    x264_ac_energy_mb( h, mb_x, mb_y, frame );
        }
        else
            return;
    }
    else
    {
    	/* Actual adaptive quantization */
    	/* qp_offset = aq_strength x 1.0397f x [log2 engergy - 14.427f] */
    	strength = h->param.rc.f_aq_strength * 1.0397f;

        /* calculate AC energy for each mb, then deduce and save qp offset. */
    	for( mb_y = 0; mb_y < h->mb.i_mb_height; mb_y++ )
            for( mb_x = 0; mb_x < h->mb.i_mb_width; mb_x++ )
            {
                int mb_xy = mb_x + mb_y*h->mb.i_mb_stride;
                uint32_t energy = x264_ac_energy_mb( h, mb_x, mb_y, frame );
                frame->f_qp_offset[mb_xy] =
                frame->f_qp_offset_aq[mb_xy] = strength * (x264_log2( X264_MAX(energy, 1) ) - 14.427f);
            }
    }

    /* Remove mean from SSD calculation of y/u/v plane */
    for( i = 0; i < 3; i++ )
    {
        uint64_t ssd = frame->i_pixel_ssd[i];
        uint64_t sum = frame->i_pixel_sum[i];
        int width  = (h->mb.i_mb_width  << 4) >> (i && CHROMA_H_SHIFT);
        int height = (h->mb.i_mb_height << 4) >> (i && CHROMA_V_SHIFT);
        frame->i_pixel_ssd[i] = ssd - (sum * sum + (width * height >> 1)) / (width * height);
    }
}

void x264_ratecontrol_init_reconfigurable( x264_t *h, int b_init )
{
    x264_ratecontrol_t *rc = h->rc;
	int vbv_buffer_size, vbv_max_bitrate;

    if( h->param.rc.i_rc_method == X264_RC_CRF )
    {
        /* Arbitrary rescaling to make CRF somewhat similar to QP.
         * Try to compensate for MB-tree's effects as well. */
        double base_cplx = h->mb.i_mb_count * (h->param.i_bframe ? 120 : 80);
        double mbtree_offset = h->param.rc.b_mb_tree ? (1.0-h->param.rc.f_qcompress)*13.5 : 0;
        rc->rate_factor_constant = pow( base_cplx, 1 - rc->qcompress )
                                 / qp2qscale( h->param.rc.f_rf_constant + mbtree_offset + QP_BD_OFFSET );
    }

    if( h->param.rc.i_vbv_max_bitrate > 0 && h->param.rc.i_vbv_buffer_size > 0 )
    {
        /* We don't support changing the ABR bitrate right now,
           so if the stream starts as CBR, keep it CBR. */
        if( rc->b_vbv_min_rate )
            h->param.rc.i_vbv_max_bitrate = h->param.rc.i_bitrate;

        if( h->param.rc.i_vbv_buffer_size < (int)(h->param.rc.i_vbv_max_bitrate / rc->fps) )
        {
            h->param.rc.i_vbv_buffer_size = h->param.rc.i_vbv_max_bitrate / rc->fps;
            x264_log( h, X264_LOG_WARNING, "VBV buffer size cannot be smaller than one frame, using %d kbit\n",
                      h->param.rc.i_vbv_buffer_size );
        }

        vbv_buffer_size = h->param.rc.i_vbv_buffer_size * 1000;
        vbv_max_bitrate = h->param.rc.i_vbv_max_bitrate * 1000;

        h->sps->vui.hrd.i_bit_rate_unscaled = vbv_max_bitrate;
        h->sps->vui.hrd.i_cpb_size_unscaled = vbv_buffer_size;

        if( rc->b_vbv_min_rate )
            rc->bitrate = h->param.rc.i_bitrate * 1000.;
        rc->buffer_rate = vbv_max_bitrate / rc->fps;
        rc->vbv_max_rate = vbv_max_bitrate;
        rc->buffer_size = vbv_buffer_size;
        rc->single_frame_vbv = rc->buffer_rate * 1.1 > rc->buffer_size;
        rc->cbr_decay = 1.0 - rc->buffer_rate / rc->buffer_size
                      * 0.5 * X264_MAX(0, 1.5 - rc->buffer_rate * rc->fps / rc->bitrate);
        if( h->param.rc.i_rc_method == X264_RC_CRF && h->param.rc.f_rf_constant_max )
        {
            rc->rate_factor_max_increment = h->param.rc.f_rf_constant_max - h->param.rc.f_rf_constant;
            if( rc->rate_factor_max_increment <= 0 )
            {
                x264_log( h, X264_LOG_WARNING, "CRF max must be greater than CRF\n" );
                rc->rate_factor_max_increment = 0;
            }
        }
        if( b_init )
        {
            if( h->param.rc.f_vbv_buffer_init > 1. )
                h->param.rc.f_vbv_buffer_init = x264_clip3f( h->param.rc.f_vbv_buffer_init / h->param.rc.i_vbv_buffer_size, 0, 1 );
            h->param.rc.f_vbv_buffer_init = x264_clip3f( X264_MAX( h->param.rc.f_vbv_buffer_init, rc->buffer_rate / rc->buffer_size ), 0, 1);
            rc->buffer_fill_final = rc->buffer_size * h->param.rc.f_vbv_buffer_init * h->sps->vui.i_time_scale;
            rc->b_vbv = 1;
            rc->b_vbv_min_rate = !rc->b_2pass
                          && h->param.rc.i_rc_method == X264_RC_ABR
                          && h->param.rc.i_vbv_max_bitrate <= h->param.rc.i_bitrate;
        }
    }
}

int x264_ratecontrol_new( x264_t *h )
{
    x264_ratecontrol_t *rc;
	int num_preds;
	int i, j;

    /* malloc memory for x264_ratecontrol_t struct */
	CHECKED_MALLOCZERO( h->rc, h->param.i_threads * sizeof(x264_ratecontrol_t) );
    rc = h->rc;

    rc->b_abr = h->param.rc.i_rc_method != X264_RC_CQP && !h->param.rc.b_stat_read;
    rc->b_2pass = h->param.rc.i_rc_method == X264_RC_ABR && h->param.rc.b_stat_read;

    /* FIXME: use integers */
    if( h->param.i_fps_num > 0 && h->param.i_fps_den > 0 )
        rc->fps = (float) h->param.i_fps_num / h->param.i_fps_den;
    else
        rc->fps = 25.0;

    /* use mb tree for rate control if possible. otherwise, fall back to qcomp. */
    if( h->param.rc.b_mb_tree )
    {
        h->param.rc.f_pb_factor = 1;
        rc->qcompress = 1;
    }
    else
        rc->qcompress = h->param.rc.f_qcompress; /* default: 0.6 */

    rc->bitrate = h->param.rc.i_bitrate * 1000.;
    rc->rate_tolerance = h->param.rc.f_rate_tolerance; /* default: 1.0 */
    rc->nmb = h->mb.i_mb_count;
    rc->last_non_b_pict_type = -1;
    rc->cbr_decay = 1.0;

    x264_ratecontrol_init_reconfigurable( h, 1 );

    if( rc->rate_tolerance < 0.01 )
    {
        x264_log( h, X264_LOG_WARNING, "bitrate tolerance too small, using .01\n" );
        rc->rate_tolerance = 0.01;
    }

    /* QP is allowed to vary per mb if VBV or AQ enabled. */
    h->mb.b_variable_qp = rc->b_vbv || h->param.rc.i_aq_mode;

    if( rc->b_abr )
    {
        /* FIXME ABR_INIT_QP is actually used only in CRF */
#define ABR_INIT_QP (( h->param.rc.i_rc_method == X264_RC_CRF ? h->param.rc.f_rf_constant : 24 ) + QP_BD_OFFSET)
        rc->accum_p_norm = .01;
        rc->accum_p_qp = ABR_INIT_QP * rc->accum_p_norm;
        /* estimated ratio that produces a reasonable QP for the first I-frame */
        rc->cplxr_sum = .01 * pow( 7.0e5, rc->qcompress ) * pow( h->mb.i_mb_count, 0.5 );
        rc->wanted_bits_window = 1.0 * rc->bitrate / rc->fps;
        rc->last_non_b_pict_type = SLICE_TYPE_I;
    }

    /* init for CQP mode */
    rc->ip_offset = 6.0 * log2f( h->param.rc.f_ip_factor );
    rc->pb_offset = 6.0 * log2f( h->param.rc.f_pb_factor );
    rc->qp_constant[SLICE_TYPE_P] = h->param.rc.i_qp_constant; /* default: 23 */
    rc->qp_constant[SLICE_TYPE_I] = x264_clip3( h->param.rc.i_qp_constant - rc->ip_offset + 0.5, 0, QP_MAX ); /* default: 20 */
    rc->qp_constant[SLICE_TYPE_B] = x264_clip3( h->param.rc.i_qp_constant + rc->pb_offset + 0.5, 0, QP_MAX ); /* default: 25 */
    h->mb.ip_offset = rc->ip_offset + 0.5;

    rc->lstep = pow( 2, h->param.rc.i_qp_step / 6.0 );
    rc->last_qscale = qp2qscale( 26 );
    /* predicators */
    num_preds = h->param.b_sliced_threads * h->param.i_threads + 1;
    CHECKED_MALLOC( rc->pred, 5 * sizeof(predictor_t) * num_preds );
    CHECKED_MALLOC( rc->pred_b_from_p, sizeof(predictor_t) );
    for( i = 0; i < 3; i++ ) /* I/P/B */
    {
        rc->last_qscale_for[i] = qp2qscale( ABR_INIT_QP );
        rc->lmin[i] = qp2qscale( h->param.rc.i_qp_min );
        rc->lmax[i] = qp2qscale( h->param.rc.i_qp_max );
        for( j = 0; j < num_preds; j++ )
        {
            rc->pred[i+j*5].coeff_min = 2.0 / 4;
            rc->pred[i+j*5].coeff = 2.0;
            rc->pred[i+j*5].count = 1.0;
            rc->pred[i+j*5].decay = 0.5;
            rc->pred[i+j*5].offset = 0.0;
        }
        for( j = 0; j < 2; j++ )
        {
            rc->row_preds[i][j].coeff_min = .25 / 4;
            rc->row_preds[i][j].coeff = .25;
            rc->row_preds[i][j].count = 1.0;
            rc->row_preds[i][j].decay = 0.5;
            rc->row_preds[i][j].offset = 0.0;
        }
    }
    *rc->pred_b_from_p = rc->pred[0];

    for( i = 0; i<h->param.i_threads; i++ )
    {
        h->thread[i]->rc = rc+i;
        if( i )
        {
            rc[i] = rc[0];
            h->thread[i]->param = h->param;
            h->thread[i]->mb.b_variable_qp = h->mb.b_variable_qp;
            h->thread[i]->mb.ip_offset = h->mb.ip_offset;
        }
    }

    return 0;
fail:
    return -1;
}

void x264_ratecontrol_summary( x264_t *h )
{
    x264_ratecontrol_t *rc = h->rc;
    if( rc->b_abr && h->param.rc.i_rc_method == X264_RC_ABR && rc->cbr_decay > .9999 )
    {
        double base_cplx = h->mb.i_mb_count * (h->param.i_bframe ? 120 : 80);
        double mbtree_offset = h->param.rc.b_mb_tree ? (1.0-h->param.rc.f_qcompress)*13.5 : 0;
        x264_log( h, X264_LOG_INFO, "final ratefactor: %.2f\n",
                  qscale2qp( pow( base_cplx, 1 - rc->qcompress )
                             * rc->cplxr_sum / rc->wanted_bits_window ) - mbtree_offset - QP_BD_OFFSET );
    }
}

void x264_ratecontrol_delete( x264_t *h )
{
    x264_ratecontrol_t *rc = h->rc;

    x264_free( rc->pred );
    x264_free( rc->pred_b_from_p );
    x264_free( rc->entry );
    x264_free( rc );
}

static void accum_p_qp_update( x264_t *h, float qp )
{
    x264_ratecontrol_t *rc = h->rc;
    rc->accum_p_qp   *= .95;
    rc->accum_p_norm *= .95;
    rc->accum_p_norm += 1;
    if( h->sh.i_type == SLICE_TYPE_I )
        rc->accum_p_qp += qp + rc->ip_offset;
    else
        rc->accum_p_qp += qp;
}

/* Before encoding a frame, choose a QP for it */
void x264_ratecontrol_start( x264_t *h, int i_force_qp, int overhead )
{
    x264_ratecontrol_t *rc = h->rc;
    ratecontrol_entry_t *rce = NULL;
    float q;

    /* only for VBV enabled */
    if( rc->b_vbv )
    {
		const x264_level_t *l;
		int mincr;
        memset( h->fdec->i_row_bits, 0, h->mb.i_mb_height * sizeof(int) );
        memset( h->fdec->f_row_qp, 0, h->mb.i_mb_height * sizeof(float) );
        memset( h->fdec->f_row_qscale, 0, h->mb.i_mb_height * sizeof(float) );
        rc->row_pred = &rc->row_preds[h->sh.i_type];
        rc->buffer_rate = h->fenc->i_cpb_duration * rc->vbv_max_rate * h->sps->vui.i_num_units_in_tick / h->sps->vui.i_time_scale;
        update_vbv_plan( h, overhead );

        l = x264_levels;
        while( l->level_idc != 0 && l->level_idc != h->param.i_level_idc )
            l++;

        /* minimum compress ratio defined by H.264 level */
        mincr = l->mincr;

        /* Profiles above High don't require minCR, so just set the maximum to a large value. */
        if( h->sps->i_profile_idc > PROFILE_HIGH )
            rc->frame_size_maximum = 1e9;
        else
        {
            /* The spec has a bizarre special case for the first frame. */
            if( h->i_frame == 0 )
            {
                //384 * ( Max( PicSizeInMbs, fR * MaxMBPS ) + MaxMBPS * ( tr( 0 ) - tr,n( 0 ) ) ) / MinCR
                double fr = 1. / 172;
                int pic_size_in_mbs = h->mb.i_mb_width * h->mb.i_mb_height;
                rc->frame_size_maximum = 384 * BIT_DEPTH * X264_MAX( pic_size_in_mbs, fr*l->mbps ) / mincr;
            }
            else
            {
                //384 * MaxMBPS * ( tr( n ) - tr( n - 1 ) ) / MinCR
                rc->frame_size_maximum = 384 * BIT_DEPTH * ((double)h->fenc->i_cpb_duration * h->sps->vui.i_num_units_in_tick / h->sps->vui.i_time_scale) * l->mbps / mincr;
            }
        }
    }

    if( h->sh.i_type != SLICE_TYPE_B )
        rc->bframes = h->fenc->i_bframes;

    if( rc->b_abr )
    {
    	/* compute qscale and deduce qp for current frame */
        q = qscale2qp( rate_estimate_qscale( h ) );
    }
    else /* CQP */
    {
        if( h->sh.i_type == SLICE_TYPE_B && h->fdec->b_kept_as_ref )
            q = ( rc->qp_constant[ SLICE_TYPE_B ] + rc->qp_constant[ SLICE_TYPE_P ] ) / 2;
        else
            q = rc->qp_constant[ h->sh.i_type ];
    }
    if( i_force_qp != X264_QP_AUTO )
        q = i_force_qp - 1;

    q = x264_clip3f( q, h->param.rc.i_qp_min, h->param.rc.i_qp_max );

    rc->qpa_rc = rc->qpa_rc_prev =
    rc->qpa_aq = rc->qpa_aq_prev = 0;
    rc->qp = x264_clip3( q + 0.5f, 0, QP_MAX );
    h->fdec->f_qp_avg_rc =
    h->fdec->f_qp_avg_aq =
    rc->qpm = q; /* save qp of current frame to rc->qpm */
    if( rce )
        rce->new_qp = rc->qp;

    accum_p_qp_update( h, rc->qpm );

    if( h->sh.i_type != SLICE_TYPE_B )
        rc->last_non_b_pict_type = h->sh.i_type;
}

static float predict_row_size( x264_t *h, int y, float qscale )
{
    /* average between two predictors:
     * absolute SATD, and scaled bit cost of the colocated row in the previous frame */
    x264_ratecontrol_t *rc = h->rc;
    float pred_s = predict_size( rc->row_pred[0], qscale, h->fdec->i_row_satd[y] );
    if( h->sh.i_type == SLICE_TYPE_I || qscale >= h->fref[0][0]->f_row_qscale[y] )
    {
        if( h->sh.i_type == SLICE_TYPE_P
            && h->fref[0][0]->i_type == h->fdec->i_type
            && h->fref[0][0]->f_row_qscale[y] > 0
            && h->fref[0][0]->i_row_satd[y] > 0
            && (abs(h->fref[0][0]->i_row_satd[y] - h->fdec->i_row_satd[y]) < h->fdec->i_row_satd[y]/2))
        {
            float pred_t = h->fref[0][0]->i_row_bits[y] * h->fdec->i_row_satd[y] / h->fref[0][0]->i_row_satd[y]
                         * h->fref[0][0]->f_row_qscale[y] / qscale;
            return (pred_s + pred_t) * 0.5f;
        }
        return pred_s;
    }
    /* Our QP is lower than the reference! */
    else
    {
        float pred_intra = predict_size( rc->row_pred[1], qscale, h->fdec->i_row_satds[0][0][y] );
        /* Sum: better to overestimate than underestimate by using only one of the two predictors. */
        return pred_intra + pred_s;
    }
}

static int row_bits_so_far( x264_t *h, int y )
{
    int bits = 0;
	int i;
    for( i = h->i_threadslice_start; i <= y; i++ )
        bits += h->fdec->i_row_bits[i];
    return bits;
}

static float predict_row_size_sum( x264_t *h, int y, float qp )
{
	int i;
    float qscale = qp2qscale( qp );
    float bits = row_bits_so_far( h, y );
    for( i = y+1; i < h->i_threadslice_end; i++ )
        bits += predict_row_size( h, i, qscale );
    return bits;
}

/* TODO:
 *  eliminate all use of qp in row ratecontrol: make it entirely qscale-based.
 *  make this function stop being needlessly O(N^2)
 *  update more often than once per row? */
int x264_ratecontrol_mb( x264_t *h, int bits )
{
    x264_ratecontrol_t *rc = h->rc;
    const int y = h->mb.i_mb_y;
	float qscale, prev_row_qp, qp_absolute_max, qp_max, qp_min, step_size;
	float buffer_left_planned, slice_size_planned, max_frame_error, size_of_other_slices;
	float rc_tol, b1;
	int can_reencode_row;

    h->fdec->i_row_bits[y] += bits; /* accumulate mb bits */
    rc->qpa_aq += h->mb.i_qp;       /* accumulate mb qp */

    /* return back if we have not reached the end of a row. */
    if( h->mb.i_mb_x != h->mb.i_mb_width - 1 )
        return 0;

    rc->qpa_rc += rc->qpm * h->mb.i_mb_width; /* accumulate mb qp for each row */

    /* return back if VBV is diabled. */
    if( !rc->b_vbv )
        return 0;

    qscale = qp2qscale( rc->qpm );
    h->fdec->f_row_qp[y] = rc->qpm;
    h->fdec->f_row_qscale[y] = qscale;

    update_predictor( rc->row_pred[0], qscale, h->fdec->i_row_satd[y], h->fdec->i_row_bits[y] );
    if( h->sh.i_type == SLICE_TYPE_P && rc->qpm < h->fref[0][0]->f_row_qp[y] )
        update_predictor( rc->row_pred[1], qscale, h->fdec->i_row_satds[0][0][y], h->fdec->i_row_bits[y] );

    /* update ratecontrol per-mbpair in MBAFF */
    if( SLICE_MBAFF && !(y&1) )
        return 0;

    /* FIXME: We don't currently support the case where there's a slice
     * boundary in between. */
    can_reencode_row = h->sh.i_first_mb <= ((h->mb.i_mb_y - SLICE_MBAFF) * h->mb.i_mb_stride);

    /* tweak quality based on difference from predicted size */
    prev_row_qp = h->fdec->f_row_qp[y];
    qp_absolute_max = h->param.rc.i_qp_max;
    if( rc->rate_factor_max_increment )
        qp_absolute_max = X264_MIN( qp_absolute_max, rc->qp_novbv + rc->rate_factor_max_increment );
    qp_max = X264_MIN( prev_row_qp + h->param.rc.i_qp_step, qp_absolute_max );
    qp_min = X264_MAX( prev_row_qp - h->param.rc.i_qp_step, h->param.rc.i_qp_min );
    step_size = 0.5f;
    buffer_left_planned = rc->buffer_fill - rc->frame_size_planned;
    slice_size_planned = h->param.b_sliced_threads ? rc->slice_size_planned : rc->frame_size_planned;
    max_frame_error = X264_MAX( 0.05f, 1.0f / h->mb.i_mb_height );
    size_of_other_slices = 0;

    if( y < h->i_threadslice_end-1 )
    {
        /* B-frames shouldn't use lower QP than their reference frames. */
        if( h->sh.i_type == SLICE_TYPE_B )
        {
            qp_min = X264_MAX( qp_min, X264_MAX( h->fref[0][0]->f_row_qp[y+1], h->fref[1][0]->f_row_qp[y+1] ) );
            rc->qpm = X264_MAX( rc->qpm, qp_min );
        }

        /* More threads means we have to be more cautious in letting ratecontrol use up extra bits. */
        rc_tol = buffer_left_planned / h->param.i_threads * rc->rate_tolerance;
        b1 = predict_row_size_sum( h, y, rc->qpm ) + size_of_other_slices;

        /* Don't increase the row QPs until a sufficent amount of the bits of the frame have been processed, in case a flat */
        /* area at the top of the frame was measured inaccurately. */
        if( row_bits_so_far( h, y ) < 0.05f * slice_size_planned )
            qp_max = qp_absolute_max = prev_row_qp;

        if( h->sh.i_type != SLICE_TYPE_I )
            rc_tol *= 0.5f;

        if( !rc->b_vbv_min_rate )
            qp_min = X264_MAX( qp_min, rc->qp_novbv );

        while( rc->qpm < qp_max
               && ((b1 > rc->frame_size_planned + rc_tol) ||
                   (rc->buffer_fill - b1 < buffer_left_planned * 0.5f) ||
                   (b1 > rc->frame_size_planned && rc->qpm < rc->qp_novbv)) )
        {
            rc->qpm += step_size;
            b1 = predict_row_size_sum( h, y, rc->qpm ) + size_of_other_slices;
        }

        while( rc->qpm > qp_min
               && (rc->qpm > h->fdec->f_row_qp[0] || rc->single_frame_vbv)
               && ((b1 < rc->frame_size_planned * 0.8f && rc->qpm <= prev_row_qp)
               || b1 < (rc->buffer_fill - rc->buffer_size + rc->buffer_rate) * 1.1f) )
        {
            rc->qpm -= step_size;
            b1 = predict_row_size_sum( h, y, rc->qpm ) + size_of_other_slices;
        }

        /* avoid VBV underflow or MinCR violation */
        while( (rc->qpm < qp_absolute_max)
               && ((rc->buffer_fill - b1 < rc->buffer_rate * max_frame_error) ||
                   (rc->frame_size_maximum - b1 < rc->frame_size_maximum * max_frame_error)))
        {
            rc->qpm += step_size;
            b1 = predict_row_size_sum( h, y, rc->qpm ) + size_of_other_slices;
        }

        h->rc->frame_size_estimated = b1 - size_of_other_slices;

        /* If the current row was large enough to cause a large QP jump, try re-encoding it. */
        if( rc->qpm > qp_max && prev_row_qp < qp_max && can_reencode_row )
        {
            /* Bump QP to halfway in between... close enough. */
            rc->qpm = x264_clip3f( (prev_row_qp + rc->qpm)*0.5f, prev_row_qp + 1.0f, qp_max );
            rc->qpa_rc = rc->qpa_rc_prev;
            rc->qpa_aq = rc->qpa_aq_prev;
            h->fdec->i_row_bits[y] = h->fdec->i_row_bits[y-SLICE_MBAFF] = 0;
            return -1;
        }
    }
    else
    {
        h->rc->frame_size_estimated = predict_row_size_sum( h, y, rc->qpm );

        /* Last-ditch attempt: if the last row of the frame underflowed the VBV,
         * try again. */
        if( (h->rc->frame_size_estimated + size_of_other_slices) > (rc->buffer_fill - rc->buffer_rate * max_frame_error) &&
             rc->qpm < qp_max && can_reencode_row )
        {
            rc->qpm = qp_max;
            rc->qpa_rc = rc->qpa_rc_prev;
            rc->qpa_aq = rc->qpa_aq_prev;
            h->fdec->i_row_bits[y] = h->fdec->i_row_bits[y-SLICE_MBAFF] = 0;
            return -1;
        }
    }

    rc->qpa_rc_prev = rc->qpa_rc;
    rc->qpa_aq_prev = rc->qpa_aq;

    return 0;
}

/* x264_ratecontrol_qp: load qp for current frame */
int x264_ratecontrol_qp( x264_t *h )
{
    return x264_clip3( h->rc->qpm + 0.5f, h->param.rc.i_qp_min, h->param.rc.i_qp_max );
}

/* x264_ratecontrol_mb_qp: load qp for current mb. return qp of current frame, plus qp offset if AQ is enabled. */
int x264_ratecontrol_mb_qp( x264_t *h )
{
	float qp;
    qp = h->rc->qpm;
    if( h->param.rc.i_aq_mode )
    {
         /* MB-tree currently doesn't adjust quantizers in unreferenced frames. */
        float qp_offset = h->fdec->b_kept_as_ref ? h->fenc->f_qp_offset[h->mb.i_mb_xy] : h->fenc->f_qp_offset_aq[h->mb.i_mb_xy];
        /* Scale AQ's effect towards zero in emergency mode. */
        if( qp > QP_MAX_SPEC )
            qp_offset *= (QP_MAX - qp) / (QP_MAX - QP_MAX_SPEC);
        qp += qp_offset;
    }
    return x264_clip3( qp + 0.5f, h->param.rc.i_qp_min, h->param.rc.i_qp_max );
}

/* After encoding one frame, save stats and update ratecontrol state */
int x264_ratecontrol_end( x264_t *h, int bits, int *filler )
{
    x264_ratecontrol_t *rc = h->rc;
    const int *mbs = h->stat.frame.i_mb_count;
	int i;

    h->stat.frame.i_mb_count_skip = mbs[P_SKIP] + mbs[B_SKIP];
    h->stat.frame.i_mb_count_i = mbs[I_16x16] + mbs[I_8x8] + mbs[I_4x4];
    h->stat.frame.i_mb_count_p = mbs[P_L0] + mbs[P_8x8];
    for( i = B_DIRECT; i < B_8x8; i++ )
        h->stat.frame.i_mb_count_p += mbs[i];

    h->fdec->f_qp_avg_rc = rc->qpa_rc /= h->mb.i_mb_count;
    h->fdec->f_qp_avg_aq = (float)rc->qpa_aq / h->mb.i_mb_count;
    h->fdec->f_crf_avg = h->param.rc.f_rf_constant + h->fdec->f_qp_avg_rc - rc->qp_novbv;

    if( rc->b_abr )
    {
        if( h->sh.i_type != SLICE_TYPE_B )
            rc->cplxr_sum += bits * qp2qscale( rc->qpa_rc ) / rc->last_rceq;
        else
        {
            /* Depends on the fact that B-frame's QP is an offset from the following P-frame's.
             * Not perfectly accurate with B-refs, but good enough. */
            rc->cplxr_sum += bits * qp2qscale( rc->qpa_rc ) / (rc->last_rceq * fabs( h->param.rc.f_pb_factor ));
        }
        rc->cplxr_sum *= rc->cbr_decay;
        rc->wanted_bits_window += h->fenc->f_duration * rc->bitrate;
        rc->wanted_bits_window *= rc->cbr_decay;
    }

    if( h->mb.b_variable_qp )
    {
        if( h->sh.i_type == SLICE_TYPE_B )
        {
            rc->bframe_bits += bits;
            if( h->fenc->b_last_minigop_bframe )
            {
                update_predictor( rc->pred_b_from_p, qp2qscale( rc->qpa_rc ),
                                  h->fref[1][h->i_ref[1]-1]->i_satd, rc->bframe_bits / rc->bframes );
                rc->bframe_bits = 0;
            }
        }
    }

    *filler = update_vbv( h, bits );
    rc->filler_bits_sum += *filler * 8;

    return 0;
}

/**
 * modify the bitrate curve from pass1 for one frame
 */
static double get_qscale(x264_t *h, ratecontrol_entry_t *rce, double rate_factor, int frame_num)
{
    x264_ratecontrol_t *rcc= h->rc;
    double q;
    if( h->param.rc.b_mb_tree )
    {
        double timescale = (double)h->sps->vui.i_num_units_in_tick / h->sps->vui.i_time_scale;
        q = pow( BASE_FRAME_DURATION / CLIP_DURATION(rce->i_duration * timescale), 1 - h->param.rc.f_qcompress );
    }
    else
    {
    	/* qscale = complexity ^ (1 - qcomp) */
        q = pow( rce->blurred_complexity, 1 - rcc->qcompress );
    }

    // avoid NaN's in the rc_eq
    if( !isfinite(q) || rce->tex_bits + rce->mv_bits == 0 )
        q = rcc->last_qscale_for[rce->pict_type];
    else
    {
    	/* qscale = qscale / rate_factor */
        rcc->last_rceq = q;
        q /= rate_factor;
        rcc->last_qscale = q;
    }

    return q;
}

static float predict_size( predictor_t *p, float q, float var )
{
    return (p->coeff*var + p->offset) / (q*p->count);
}

static void update_predictor( predictor_t *p, float q, float var, float bits )
{
    float range = 1.5;
	float old_coeff, new_coeff, new_coeff_clipped, new_offset;
    if( var < 10 )
        return;
    old_coeff = p->coeff / p->count;
    new_coeff = X264_MAX( bits*q / var, p->coeff_min );
    new_coeff_clipped = x264_clip3f( new_coeff, old_coeff/range, old_coeff*range );
    new_offset = bits*q - new_coeff_clipped * var;
    if( new_offset >= 0 )
        new_coeff = new_coeff_clipped;
    else
        new_offset = 0;
    p->count  *= p->decay;
    p->coeff  *= p->decay;
    p->offset *= p->decay;
    p->count  ++;
    p->coeff  += new_coeff;
    p->offset += new_offset;
}

// update VBV after encoding a frame
static int update_vbv( x264_t *h, int bits )
{
    int filler = 0;
    int bitrate = h->sps->vui.hrd.i_bit_rate_unscaled;
    x264_ratecontrol_t *rcc = h->rc;
    x264_ratecontrol_t *rct = h->thread[0]->rc;
    uint64_t buffer_size = (uint64_t)h->sps->vui.hrd.i_cpb_size_unscaled * h->sps->vui.i_time_scale;

    if( rcc->last_satd >= h->mb.i_mb_count )
        update_predictor( &rct->pred[h->sh.i_type], qp2qscale( rcc->qpa_rc ), rcc->last_satd, bits );

    if( !rcc->b_vbv )
        return filler;

    rct->buffer_fill_final -= (uint64_t)bits * h->sps->vui.i_time_scale;

    if( rct->buffer_fill_final < 0 )
        x264_log( h, X264_LOG_WARNING, "VBV underflow (frame %d, %.0f bits)\n", h->i_frame, (double)rct->buffer_fill_final / h->sps->vui.i_time_scale );
    rct->buffer_fill_final = X264_MAX( rct->buffer_fill_final, 0 );
    rct->buffer_fill_final += (uint64_t)bitrate * h->sps->vui.i_num_units_in_tick * h->fenc->i_cpb_duration;

    if( h->sps->vui.hrd.b_cbr_hrd && rct->buffer_fill_final > buffer_size )
    {
        int64_t scale = (int64_t)h->sps->vui.i_time_scale * 8;
        filler = (rct->buffer_fill_final - buffer_size + scale - 1) / scale;
        bits = X264_MAX( (FILLER_OVERHEAD - h->param.b_annexb), filler ) * 8;
        rct->buffer_fill_final -= (uint64_t)bits * h->sps->vui.i_time_scale;
    }
    else
        rct->buffer_fill_final = X264_MIN( rct->buffer_fill_final, buffer_size );

    return filler;
}

// provisionally update VBV according to the planned size of all frames currently in progress
static void update_vbv_plan( x264_t *h, int overhead )
{
    x264_ratecontrol_t *rcc = h->rc;
    rcc->buffer_fill = h->thread[0]->rc->buffer_fill_final / h->sps->vui.i_time_scale;
    rcc->buffer_fill = X264_MIN( rcc->buffer_fill, rcc->buffer_size );
    rcc->buffer_fill -= overhead;
}

// apply VBV constraints and clip qscale to between lmin and lmax
static double clip_qscale( x264_t *h, int pict_type, double q )
{
    x264_ratecontrol_t *rcc = h->rc;
    double lmin = rcc->lmin[pict_type];
    double lmax = rcc->lmax[pict_type];
	double q0 = q;
    if( rcc->rate_factor_max_increment )
        lmax = X264_MIN( lmax, qp2qscale( rcc->qp_novbv + rcc->rate_factor_max_increment ) );

    /* B-frames are not directly subject to VBV,
     * since they are controlled by the P-frames' QPs. */

    if( rcc->b_vbv && rcc->last_satd > 0 )
    {
		double bits;
        /* Lookahead VBV: raise the quantizer as necessary such that no frames in
         * the lookahead overflow and such that the buffer is in a reasonable state
         * by the end of the lookahead. */
        if( h->param.rc.i_lookahead )
        {
            int j, iterations, terminate = 0;

            /* Avoid an infinite loop. */
            for( iterations = 0; iterations < 1000 && terminate != 3; iterations++ )
            {
                double frame_q[3];
                double cur_bits = predict_size( &rcc->pred[h->sh.i_type], q, rcc->last_satd );
                double buffer_fill_cur = rcc->buffer_fill - cur_bits;
                double target_fill;
                double total_duration = 0;
                frame_q[0] = h->sh.i_type == SLICE_TYPE_I ? q * h->param.rc.f_ip_factor : q;
                frame_q[1] = frame_q[0] * h->param.rc.f_pb_factor;
                frame_q[2] = frame_q[0] / h->param.rc.f_ip_factor;

                /* Loop over the planned future frames. */
                for( j = 0; buffer_fill_cur >= 0 && buffer_fill_cur <= rcc->buffer_size; j++ )
                {
					int i_type, i_satd;
                    total_duration += h->fenc->f_planned_cpb_duration[j];
                    buffer_fill_cur += rcc->vbv_max_rate * h->fenc->f_planned_cpb_duration[j];
                    i_type = h->fenc->i_planned_type[j];
                    i_satd = h->fenc->i_planned_satd[j];
                    if( i_type == X264_TYPE_AUTO )
                        break;
                    i_type = IS_X264_TYPE_I( i_type ) ? SLICE_TYPE_I : IS_X264_TYPE_B( i_type ) ? SLICE_TYPE_B : SLICE_TYPE_P;
                    cur_bits = predict_size( &rcc->pred[i_type], frame_q[i_type], i_satd );
                    buffer_fill_cur -= cur_bits;
                }
                /* Try to get to get the buffer at least 50% filled, but don't set an impossible goal. */
                target_fill = X264_MIN( rcc->buffer_fill + total_duration * rcc->vbv_max_rate * 0.5, rcc->buffer_size * 0.5 );
                if( buffer_fill_cur < target_fill )
                {
                    q *= 1.01;
                    terminate |= 1;
                    continue;
                }
                /* Try to get the buffer no more than 80% filled, but don't set an impossible goal. */
                target_fill = x264_clip3f( rcc->buffer_fill - total_duration * rcc->vbv_max_rate * 0.5, rcc->buffer_size * 0.8, rcc->buffer_size );
                if( rcc->b_vbv_min_rate && buffer_fill_cur > target_fill )
                {
                    q /= 1.01;
                    terminate |= 2;
                    continue;
                }
                break;
            }
        }
        /* Fallback to old purely-reactive algorithm: no lookahead. */
        else
        {
			double bits, qf = 1.0, max_fill_factor, min_fill_factor;
            if( ( pict_type == SLICE_TYPE_P ||
                ( pict_type == SLICE_TYPE_I && rcc->last_non_b_pict_type == SLICE_TYPE_I ) ) &&
                rcc->buffer_fill/rcc->buffer_size < 0.5 )
            {
                q /= x264_clip3f( 2.0*rcc->buffer_fill/rcc->buffer_size, 0.5, 1.0 );
            }

            /* Now a hard threshold to make sure the frame fits in VBV.
             * This one is mostly for I-frames. */
            bits = predict_size( &rcc->pred[h->sh.i_type], q, rcc->last_satd );
            qf = 1.0;
            /* For small VBVs, allow the frame to use up the entire VBV. */
            max_fill_factor = h->param.rc.i_vbv_buffer_size >= 5*h->param.rc.i_vbv_max_bitrate / rcc->fps ? 2 : 1;
            /* For single-frame VBVs, request that the frame use up the entire VBV. */
            min_fill_factor = rcc->single_frame_vbv ? 1 : 2;

            if( bits > rcc->buffer_fill/max_fill_factor )
                qf = x264_clip3f( rcc->buffer_fill/(max_fill_factor*bits), 0.2, 1.0 );
            q /= qf;
            bits *= qf;
            if( bits < rcc->buffer_rate/min_fill_factor )
                q *= bits*min_fill_factor/rcc->buffer_rate;
            q = X264_MAX( q0, q );
        }

        /* Apply MinCR restrictions */
        bits = predict_size( &rcc->pred[h->sh.i_type], q, rcc->last_satd );
        if( bits > rcc->frame_size_maximum )
            q *= bits / rcc->frame_size_maximum;
        bits = predict_size( &rcc->pred[h->sh.i_type], q, rcc->last_satd );

        /* Check B-frame complexity, and use up any bits that would
         * overflow before the next P-frame. */
        if( h->sh.i_type == SLICE_TYPE_P && !rcc->single_frame_vbv )
        {
            int nb = rcc->bframes;
            double pbbits = bits;
            double bbits = predict_size( rcc->pred_b_from_p, q * h->param.rc.f_pb_factor, rcc->last_satd );
            double space;
            double bframe_cpb_duration = 0;
            double minigop_cpb_duration;
			int i;
            for( i = 0; i < nb; i++ )
                bframe_cpb_duration += h->fenc->f_planned_cpb_duration[1+i];

            if( bbits * nb > bframe_cpb_duration * rcc->vbv_max_rate )
                nb = 0;
            pbbits += nb * bbits;

            minigop_cpb_duration = bframe_cpb_duration + h->fenc->f_planned_cpb_duration[0];
            space = rcc->buffer_fill + minigop_cpb_duration*rcc->vbv_max_rate - rcc->buffer_size;
            if( pbbits < space )
            {
                q *= X264_MAX( pbbits / space, bits / (0.5 * rcc->buffer_size) );
            }
            q = X264_MAX( q0/2, q );
        }

        if( !rcc->b_vbv_min_rate )
            q = X264_MAX( q0, q );
    }

    if( lmin==lmax )
        return lmin;
    else
        return x264_clip3f( q, lmin, lmax );
}

// update qscale for 1 frame based on actual bits used so far
static float rate_estimate_qscale( x264_t *h )
{
    float q;
    x264_ratecontrol_t *rcc = h->rc;
    ratecontrol_entry_t rce;

    int pict_type = h->sh.i_type;
    int64_t total_bits = 8*(h->stat.i_frame_size[SLICE_TYPE_I]
                          + h->stat.i_frame_size[SLICE_TYPE_P]
                          + h->stat.i_frame_size[SLICE_TYPE_B])
                       - rcc->filler_bits_sum;

    /* if( pict_type == SLICE_TYPE_B ) */

	/* Calculate the quantizer which would have produced the desired
	 * average bitrate if it had been applied to all frames so far.
	 * Then modulate that quant based on the current frame's complexity
	 * relative to the average complexity so far (using the 2pass RCEQ).
	 * Then bias the quant up or down if total size so far was far from
	 * the target.
	 * Result: Depending on the value of rate_tolerance, there is a
	 * tradeoff between quality and bitrate precision. But at large
	 * tolerances, the bit distribution approaches that of 2pass. */

	double abr_buffer = 2 * rcc->rate_tolerance * rcc->bitrate;
	double wanted_bits, overflow = 1;

	/* complexity of picture is calculated as weighting average of cost (satd) of last frames.
	 * e.g. SATD1 = 100262, SATD2 = 12812, SATD3 = 12022.
	 * complexity = ((100262 * 0.5 + 12812) * 0.5 + 12022) / (0.5 * 0.5 + 0.5 + 1) = 43493.5
	 */
	rcc->last_satd = x264_rc_analyse_slice( h );
	rcc->short_term_cplxsum *= 0.5;
	rcc->short_term_cplxcount *= 0.5;
	rcc->short_term_cplxsum += rcc->last_satd / (CLIP_DURATION(h->fenc->f_duration) / BASE_FRAME_DURATION);
	rcc->short_term_cplxcount ++;

	rce.tex_bits = rcc->last_satd;
	rce.blurred_complexity = rcc->short_term_cplxsum / rcc->short_term_cplxcount;
	rce.mv_bits = 0;
	rce.p_count = rcc->nmb;
	rce.i_count = 0;
	rce.s_count = 0;
	rce.qscale = 1;
	rce.pict_type = pict_type;
	rce.i_duration = h->fenc->i_duration;

	if( h->param.rc.i_rc_method == X264_RC_CRF )
	{
		q = get_qscale( h, &rce, rcc->rate_factor_constant, h->fenc->i_frame );
	}
	else
	{
		q = get_qscale( h, &rce, rcc->wanted_bits_window / rcc->cplxr_sum, h->fenc->i_frame );

		/* ABR code can potentially be counterproductive in CBR, so just don't bother.
		 * Don't run it if the frame complexity is zero either. */
		if( !rcc->b_vbv_min_rate && rcc->last_satd )
		{
			// FIXME is it simpler to keep track of wanted_bits in ratecontrol_end?
			int i_frame_done = h->i_frame + 1 - h->i_thread_frames;
			double time_done = i_frame_done / rcc->fps;
			if( h->param.b_vfr_input && i_frame_done > 0 )
				time_done = ((double)(h->fenc->i_reordered_pts - h->i_reordered_pts_delay)) * h->param.i_timebase_num / h->param.i_timebase_den;
			wanted_bits = time_done * rcc->bitrate;
			if( wanted_bits > 0 )
			{
				abr_buffer *= X264_MAX( 1, sqrt( time_done ) );
				overflow = x264_clip3f( 1.0 + (total_bits - wanted_bits) / abr_buffer, .5, 2 );
				q *= overflow;
			}
		}
	}

	if( pict_type == SLICE_TYPE_I && h->param.i_keyint_max > 1
		/* should test _next_ pict type, but that isn't decided yet */
		&& rcc->last_non_b_pict_type != SLICE_TYPE_I )
	{
		q = qp2qscale( rcc->accum_p_qp / rcc->accum_p_norm );
		q /= fabs( h->param.rc.f_ip_factor );
	}
	else if( h->i_frame > 0 )
	{
		if( h->param.rc.i_rc_method != X264_RC_CRF )
		{
			/* Asymmetric clipping, because symmetric would prevent
			 * overflow control in areas of rapidly oscillating complexity */
			double lmin = rcc->last_qscale_for[pict_type] / rcc->lstep;
			double lmax = rcc->last_qscale_for[pict_type] * rcc->lstep;
			if( overflow > 1.1 && h->i_frame > 3 )
				lmax *= rcc->lstep;
			else if( overflow < 0.9 )
				lmin /= rcc->lstep;

			q = x264_clip3f(q, lmin, lmax);
		}
	}
	else if( h->param.rc.i_rc_method == X264_RC_CRF && rcc->qcompress != 1 )
	{
		q = qp2qscale( ABR_INIT_QP ) / fabs( h->param.rc.f_ip_factor );
	}
	rcc->qp_novbv = qscale2qp( q );

	//FIXME use get_diff_limited_q() ?
	q = clip_qscale( h, pict_type, q );

	rcc->last_qscale_for[pict_type] =
	rcc->last_qscale = q;

	if( !(rcc->b_2pass && !rcc->b_vbv) && h->fenc->i_frame == 0 )
		rcc->last_qscale_for[SLICE_TYPE_P] = q * fabs( h->param.rc.f_ip_factor );

	rcc->frame_size_planned = predict_size( &rcc->pred[h->sh.i_type], q, rcc->last_satd );

	/* Always use up the whole VBV in this case. */
	if( rcc->single_frame_vbv )
		rcc->frame_size_planned = rcc->buffer_rate;
	/* Limit planned size by MinCR */
	if( rcc->b_vbv )
		rcc->frame_size_planned = X264_MIN( rcc->frame_size_planned, rcc->frame_size_maximum );
	h->rc->frame_size_estimated = rcc->frame_size_planned;
	return q;
}
