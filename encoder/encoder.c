/*****************************************************************************
 * encoder.c: top-level encoder functions
 *****************************************************************************/

#include "common/common.h"

#include "set.h"
#include "analyse.h"
#include "ratecontrol.h"
#include "macroblock.h"
#include "me.h"

#define bs_write_ue bs_write_ue_big

static int x264_validate_parameters( x264_t *h, int b_open )
{
	int i_csp, max_slices;
	float fps;

    if( h->param.i_width <= 0 || h->param.i_height <= 0 )
    {
        x264_log( h, X264_LOG_ERROR, "invalid width x height (%dx%d)\n",
                  h->param.i_width, h->param.i_height );
        return -1;
    }

    i_csp = h->param.i_csp & X264_CSP_MASK;
    if( i_csp <= X264_CSP_NONE || i_csp >= X264_CSP_MAX )
    {
        x264_log( h, X264_LOG_ERROR, "invalid CSP (only I420 supported)\n" );
        return -1;
    }

    if( i_csp < X264_CSP_I444 && h->param.i_width % 2 )
    {
        x264_log( h, X264_LOG_ERROR, "width not divisible by 2 (%dx%d)\n",
                  h->param.i_width, h->param.i_height );
        return -1;
    }

    if( i_csp < X264_CSP_I422 && h->param.i_height % 2 )
    {
        x264_log( h, X264_LOG_ERROR, "height not divisible by 2 (%dx%d)\n",
                  h->param.i_width, h->param.i_height );
        return -1;
    }

    if( (h->param.crop_rect.i_left + h->param.crop_rect.i_right ) >= h->param.i_width ||
        (h->param.crop_rect.i_top  + h->param.crop_rect.i_bottom) >= h->param.i_height )
    {
        x264_log( h, X264_LOG_ERROR, "invalid crop-rect %u,%u,%u,%u\n", h->param.crop_rect.i_left,
                  h->param.crop_rect.i_top, h->param.crop_rect.i_right,  h->param.crop_rect.i_bottom );
        return -1;
    }

    /* Limit threads to 1 to force single-thread mode */
    h->param.i_threads = 1;
    if( h->param.i_threads == 1 )
    {
        h->param.b_sliced_threads = 0;
        h->param.i_lookahead_threads = 1;
    }
    h->i_thread_frames = h->param.b_sliced_threads ? 1 : h->param.i_threads;
    if( h->i_thread_frames > 1 )
        h->param.nalu_process = NULL;

    h->param.i_keyint_max = x264_clip3( h->param.i_keyint_max, 1, X264_KEYINT_MAX_INFINITE );
    if( h->param.i_keyint_max == 1 ) /* All frames are key-frame */
    {
        h->param.b_intra_refresh = 0;
        h->param.analyse.i_weighted_pred = 0;
    }

    h->param.i_frame_packing = x264_clip3( h->param.i_frame_packing, -1, 5 );

    /* rate control parameters */
    if( h->param.rc.i_rc_method < 0 || h->param.rc.i_rc_method > 2 )
    {
        x264_log( h, X264_LOG_ERROR, "no ratecontrol method specified\n" );
        return -1;
    }
    h->param.rc.f_rf_constant = x264_clip3f( h->param.rc.f_rf_constant, -QP_BD_OFFSET, 51 );
    h->param.rc.f_rf_constant_max = x264_clip3f( h->param.rc.f_rf_constant_max, -QP_BD_OFFSET, 51 );
    h->param.rc.i_qp_constant = x264_clip3( h->param.rc.i_qp_constant, 0, QP_MAX );
    h->param.analyse.i_subpel_refine = x264_clip3( h->param.analyse.i_subpel_refine, 0, 11 );
    h->param.rc.f_ip_factor = X264_MAX( h->param.rc.f_ip_factor, 0.01f );
    h->param.rc.f_pb_factor = X264_MAX( h->param.rc.f_pb_factor, 0.01f );
    if( h->param.rc.i_rc_method == X264_RC_CRF ) /* Constant rate factor */
    {
        h->param.rc.i_qp_constant = h->param.rc.f_rf_constant + QP_BD_OFFSET;
        h->param.rc.i_bitrate = 0;
    }
    if( h->param.rc.i_rc_method == X264_RC_CQP )
    {
        float qp_p = h->param.rc.i_qp_constant;                 /* constant qp for P-slice */
        float qp_i = qp_p - 6*log2f( h->param.rc.f_ip_factor ); /* constant qp for I-slice */
        float qp_b = qp_p + 6*log2f( h->param.rc.f_pb_factor ); /* constant qp for B-slice */
        h->param.rc.i_qp_min = x264_clip3( (int)(X264_MIN3( qp_p, qp_i, qp_b )), 0, QP_MAX );
        h->param.rc.i_qp_max = x264_clip3( (int)(X264_MAX3( qp_p, qp_i, qp_b ) + .999), 0, QP_MAX );
        h->param.rc.i_aq_mode = 0;
        h->param.rc.b_mb_tree = 0;
        h->param.rc.i_bitrate = 0;
    }
    h->param.rc.i_qp_max = x264_clip3( h->param.rc.i_qp_max, 0, QP_MAX );
    h->param.rc.i_qp_min = x264_clip3( h->param.rc.i_qp_min, 0, h->param.rc.i_qp_max );
    h->param.rc.i_qp_step = x264_clip3( h->param.rc.i_qp_step, 2, QP_MAX );
    h->param.rc.i_bitrate = x264_clip3( h->param.rc.i_bitrate, 0, 2000000 );
    if( h->param.rc.i_rc_method == X264_RC_ABR && !h->param.rc.i_bitrate )
    {
        x264_log( h, X264_LOG_ERROR, "bitrate not specified\n" );
        return -1;
    }
    h->param.rc.i_vbv_buffer_size = x264_clip3( h->param.rc.i_vbv_buffer_size, 0, 2000000 );
    h->param.rc.i_vbv_max_bitrate = x264_clip3( h->param.rc.i_vbv_max_bitrate, 0, 2000000 );
    h->param.rc.f_vbv_buffer_init = x264_clip3f( h->param.rc.f_vbv_buffer_init, 0, 2000000 );
    if( h->param.rc.i_vbv_buffer_size )
    {
        if( h->param.rc.i_rc_method == X264_RC_CQP )
        {
            x264_log( h, X264_LOG_WARNING, "VBV is incompatible with constant QP, ignored.\n" );
            h->param.rc.i_vbv_max_bitrate = 0;
            h->param.rc.i_vbv_buffer_size = 0;
        }
        else if( h->param.rc.i_vbv_max_bitrate == 0 )
        {
            if( h->param.rc.i_rc_method == X264_RC_ABR )
            {
                x264_log( h, X264_LOG_WARNING, "VBV maxrate unspecified, assuming CBR\n" );
                h->param.rc.i_vbv_max_bitrate = h->param.rc.i_bitrate;
            }
            else
            {
                x264_log( h, X264_LOG_WARNING, "VBV bufsize set but maxrate unspecified, ignored\n" );
                h->param.rc.i_vbv_buffer_size = 0;
            }
        }
        else if( h->param.rc.i_vbv_max_bitrate < h->param.rc.i_bitrate &&
                 h->param.rc.i_rc_method == X264_RC_ABR )
        {
            x264_log( h, X264_LOG_WARNING, "max bitrate less than average bitrate, assuming CBR\n" );
            h->param.rc.i_vbv_max_bitrate = h->param.rc.i_bitrate;
        }
    }
    else if( h->param.rc.i_vbv_max_bitrate )
    {
        x264_log( h, X264_LOG_WARNING, "VBV maxrate specified, but no bufsize, ignored\n" );
        h->param.rc.i_vbv_max_bitrate = 0;
    }

    /* slicing parameters */
    h->param.i_slice_max_size = X264_MAX( h->param.i_slice_max_size, 0 );
    h->param.i_slice_max_mbs = X264_MAX( h->param.i_slice_max_mbs, 0 );

    max_slices = (h->param.i_height + 15) >> 4;
    if( h->param.b_sliced_threads )
        h->param.i_slice_count = x264_clip3( h->param.i_threads, 0, max_slices );
    else
    {
        h->param.i_slice_count = x264_clip3( h->param.i_slice_count, 0, max_slices );
        if( h->param.i_slice_max_mbs || h->param.i_slice_max_size )
            h->param.i_slice_count = 0;
    }

    /* frame type parameters */
    h->param.i_frame_reference = x264_clip3( h->param.i_frame_reference, 1, X264_REF_MAX );
    h->param.i_dpb_size = x264_clip3( h->param.i_dpb_size, 1, X264_REF_MAX );
    if( h->param.i_scenecut_threshold < 0 )
        h->param.i_scenecut_threshold = 0;
    h->param.analyse.i_direct_mv_pred = x264_clip3( h->param.analyse.i_direct_mv_pred, X264_DIRECT_PRED_NONE, X264_DIRECT_PRED_AUTO );
    if( !h->param.analyse.i_subpel_refine && h->param.analyse.i_direct_mv_pred > X264_DIRECT_PRED_SPATIAL )
    {
        x264_log( h, X264_LOG_WARNING, "subme=0 + direct=temporal is not supported\n" );
        h->param.analyse.i_direct_mv_pred = X264_DIRECT_PRED_SPATIAL;
    }
    h->param.i_bframe = x264_clip3( h->param.i_bframe, 0, X264_MIN( X264_BFRAME_MAX, h->param.i_keyint_max-1 ) );
    h->param.i_bframe_bias = x264_clip3( h->param.i_bframe_bias, -90, 100 );
    if( h->param.i_bframe <= 1 )
        h->param.i_bframe_pyramid = X264_B_PYRAMID_NONE;
    h->param.i_bframe_pyramid = x264_clip3( h->param.i_bframe_pyramid, X264_B_PYRAMID_NONE, X264_B_PYRAMID_NORMAL );
    h->param.i_bframe_adaptive = x264_clip3( h->param.i_bframe_adaptive, X264_B_ADAPT_NONE, X264_B_ADAPT_TRELLIS );
    if( !h->param.i_bframe )
    {
        h->param.i_bframe_adaptive = X264_B_ADAPT_NONE;
        h->param.analyse.i_direct_mv_pred = 0;
        h->param.analyse.b_weighted_bipred = 0;
        h->param.b_open_gop = 0;
    }
    if( h->param.b_intra_refresh && h->param.i_bframe_pyramid == X264_B_PYRAMID_NORMAL )
    {
        x264_log( h, X264_LOG_WARNING, "b-pyramid normal + intra-refresh is not supported\n" );
        h->param.i_bframe_pyramid = X264_B_PYRAMID_STRICT;
    }
    if( h->param.b_intra_refresh && (h->param.i_frame_reference > 1 || h->param.i_dpb_size > 1) )
    {
        x264_log( h, X264_LOG_WARNING, "ref > 1 + intra-refresh is not supported\n" );
        h->param.i_frame_reference = 1;
        h->param.i_dpb_size = 1;
    }
    if( h->param.b_intra_refresh && h->param.b_open_gop )
    {
        x264_log( h, X264_LOG_WARNING, "intra-refresh is not compatible with open-gop\n" );
        h->param.b_open_gop = 0;
    }
    if( !h->param.i_fps_num || !h->param.i_fps_den )
    {
        h->param.i_fps_num = 25;
        h->param.i_fps_den = 1;
    }
    fps = (float) h->param.i_fps_num / h->param.i_fps_den;
    if( h->param.i_keyint_min == X264_KEYINT_MIN_AUTO )
        h->param.i_keyint_min = X264_MIN( h->param.i_keyint_max / 10, fps );
    h->param.i_keyint_min = x264_clip3( h->param.i_keyint_min, 1, h->param.i_keyint_max/2+1 );
    h->param.rc.i_lookahead = x264_clip3( h->param.rc.i_lookahead, 0, X264_LOOKAHEAD_MAX );
    {
        int maxrate = X264_MAX( h->param.rc.i_vbv_max_bitrate, h->param.rc.i_bitrate );
        float bufsize = maxrate ? (float)h->param.rc.i_vbv_buffer_size / maxrate : 0;
        h->param.rc.i_lookahead = X264_MIN( h->param.rc.i_lookahead, X264_MAX( h->param.i_keyint_max, bufsize*fps ) );
    }

    if( !h->param.i_timebase_num || !h->param.i_timebase_den || !(h->param.b_vfr_input || h->param.b_pulldown) )
    {
        h->param.i_timebase_num = h->param.i_fps_den;
        h->param.i_timebase_den = h->param.i_fps_num;
    }

    h->param.rc.f_qcompress = x264_clip3f( h->param.rc.f_qcompress, 0.0, 1.0 );
    if( h->param.i_keyint_max == 1 || h->param.rc.f_qcompress == 1 )
        h->param.rc.b_mb_tree = 0;
    if( (!h->param.b_intra_refresh && h->param.i_keyint_max != X264_KEYINT_MAX_INFINITE) &&
        !h->param.rc.i_lookahead && h->param.rc.b_mb_tree )
    {
        x264_log( h, X264_LOG_WARNING, "lookaheadless mb-tree requires intra refresh or infinite keyint\n" );
        h->param.rc.b_mb_tree = 0;
    }
    if( b_open && h->param.rc.b_stat_read )
        h->param.rc.i_lookahead = 0;

    h->param.i_sync_lookahead = 0;

    h->param.i_deblocking_filter_alphac0 = x264_clip3( h->param.i_deblocking_filter_alphac0, -6, 6 );
    h->param.i_deblocking_filter_beta    = x264_clip3( h->param.i_deblocking_filter_beta, -6, 6 );
    h->param.analyse.i_luma_deadzone[0] = x264_clip3( h->param.analyse.i_luma_deadzone[0], 0, 32 );
    h->param.analyse.i_luma_deadzone[1] = x264_clip3( h->param.analyse.i_luma_deadzone[1], 0, 32 );

    h->param.i_cabac_init_idc = x264_clip3( h->param.i_cabac_init_idc, 0, 2 );

    if( h->param.i_cqm_preset < X264_CQM_FLAT || h->param.i_cqm_preset > X264_CQM_CUSTOM )
        h->param.i_cqm_preset = X264_CQM_FLAT;

    /* analyse parameters */
    if( h->param.analyse.i_me_method < X264_ME_DIA ||
        h->param.analyse.i_me_method > X264_ME_TESA )
        h->param.analyse.i_me_method = X264_ME_HEX;
    h->param.analyse.i_me_range = x264_clip3( h->param.analyse.i_me_range, 4, 1024 );
    if( h->param.analyse.i_me_range > 16 && h->param.analyse.i_me_method <= X264_ME_HEX )
        h->param.analyse.i_me_range = 16;
    if( h->param.analyse.i_me_method == X264_ME_TESA &&
        (h->mb.b_lossless || h->param.analyse.i_subpel_refine <= 1) )
        h->param.analyse.i_me_method = X264_ME_ESA;
    h->param.analyse.b_mixed_references = h->param.analyse.b_mixed_references && h->param.i_frame_reference > 1;
    h->param.analyse.inter &= X264_ANALYSE_PSUB16x16|X264_ANALYSE_PSUB8x8|X264_ANALYSE_BSUB16x16|
                              X264_ANALYSE_I4x4|X264_ANALYSE_I8x8;
    h->param.analyse.intra &= X264_ANALYSE_I4x4|X264_ANALYSE_I8x8;
    if( !(h->param.analyse.inter & X264_ANALYSE_PSUB16x16) )
        h->param.analyse.inter &= ~X264_ANALYSE_PSUB8x8;
    if( !h->param.analyse.b_transform_8x8 )
    {
        h->param.analyse.inter &= ~X264_ANALYSE_I8x8;
        h->param.analyse.intra &= ~X264_ANALYSE_I8x8;
    }
    h->param.analyse.i_trellis = x264_clip3( h->param.analyse.i_trellis, 0, 2 );
    h->param.rc.i_aq_mode = x264_clip3( h->param.rc.i_aq_mode, 0, 2 );
    h->param.rc.f_aq_strength = x264_clip3f( h->param.rc.f_aq_strength, 0, 3 );
    if( h->param.rc.f_aq_strength == 0 )
        h->param.rc.i_aq_mode = 0;

    if( h->param.i_log_level < X264_LOG_INFO )
    {
        h->param.analyse.b_psnr = 0;
        h->param.analyse.b_ssim = 0;
    }
    if( !h->param.analyse.b_psy )
    {
        h->param.analyse.f_psy_rd = 0;
        h->param.analyse.f_psy_trellis = 0;
    }
    h->param.analyse.f_psy_rd = x264_clip3f( h->param.analyse.f_psy_rd, 0, 10 );
    h->param.analyse.f_psy_trellis = x264_clip3f( h->param.analyse.f_psy_trellis, 0, 10 );
    h->mb.i_psy_rd = h->param.analyse.i_subpel_refine >= 6 ? FIX8( h->param.analyse.f_psy_rd ) : 0;
    h->mb.i_psy_trellis = h->param.analyse.i_trellis ? FIX8( h->param.analyse.f_psy_trellis / 4 ) : 0;
    h->param.analyse.i_chroma_qp_offset = x264_clip3(h->param.analyse.i_chroma_qp_offset, -32, 32);
    /* In 4:4:4 mode, chroma gets twice as much resolution, so we can halve its quality. */
    if( b_open && i_csp >= X264_CSP_I444 && i_csp < X264_CSP_BGR && h->param.analyse.b_psy )
        h->param.analyse.i_chroma_qp_offset += 6;
    /* Psy RDO increases overall quantizers to improve the quality of luma--this indirectly hurts chroma quality */
    /* so we lower the chroma QP offset to compensate */
    if( b_open && h->mb.i_psy_rd )
        h->param.analyse.i_chroma_qp_offset -= h->param.analyse.f_psy_rd < 0.25 ? 1 : 2;
    /* Psy trellis has a similar effect. */
    if( b_open && h->mb.i_psy_trellis )
        h->param.analyse.i_chroma_qp_offset -= h->param.analyse.f_psy_trellis < 0.25 ? 1 : 2;
    h->param.analyse.i_chroma_qp_offset = x264_clip3(h->param.analyse.i_chroma_qp_offset, -12, 12);
    /* MB-tree requires AQ to be on, even if the strength is zero. */
    if( !h->param.rc.i_aq_mode && h->param.rc.b_mb_tree )
    {
        h->param.rc.i_aq_mode = 1;
        h->param.rc.f_aq_strength = 0;
    }
    h->param.analyse.i_noise_reduction = x264_clip3( h->param.analyse.i_noise_reduction, 0, 1<<16 );
    if( h->param.analyse.i_subpel_refine >= 10 && (h->param.analyse.i_trellis != 2 || !h->param.rc.i_aq_mode) )
        h->param.analyse.i_subpel_refine = 9;

    /* h.264 profile and level indication */
    {
        const x264_level_t *l = x264_levels;
        if( h->param.i_level_idc < 0 )
        {
            int maxrate_bak = h->param.rc.i_vbv_max_bitrate;
            if( h->param.rc.i_rc_method == X264_RC_ABR && h->param.rc.i_vbv_buffer_size <= 0 )
                h->param.rc.i_vbv_max_bitrate = h->param.rc.i_bitrate * 2;
            /* init SPS and profile indication */
            x264_sps_init( h->sps, h->param.i_sps_id, &h->param );
            /* validate level indication */
            do h->param.i_level_idc = l->level_idc;
                while( l[1].level_idc && x264_validate_levels( h, 0 ) && l++ );
            h->param.rc.i_vbv_max_bitrate = maxrate_bak;
        }
        else
        {
            while( l->level_idc && l->level_idc != h->param.i_level_idc )
                l++;
            if( l->level_idc == 0 )
            {
                x264_log( h, X264_LOG_ERROR, "invalid level_idc: %d\n", h->param.i_level_idc );
                return -1;
            }
        }

        /* if mv_range is not setted by user, use value defined by current level. */
        if( h->param.analyse.i_mv_range <= 0 )
            h->param.analyse.i_mv_range = l->mv_range;
        else
            h->param.analyse.i_mv_range = x264_clip3(h->param.analyse.i_mv_range, 32, 512);
    }

    h->param.analyse.i_weighted_pred = x264_clip3( h->param.analyse.i_weighted_pred, X264_WEIGHTP_NONE, X264_WEIGHTP_SMART );

    if( !h->param.analyse.i_weighted_pred && h->param.rc.b_mb_tree && h->param.analyse.b_psy )
        h->param.analyse.i_weighted_pred = X264_WEIGHTP_FAKE;

    if( h->param.rc.f_rate_tolerance < 0 )
        h->param.rc.f_rate_tolerance = 0;
    if( h->param.rc.f_qblur < 0 )
        h->param.rc.f_qblur = 0;
    if( h->param.rc.f_complexity_blur < 0 )
        h->param.rc.f_complexity_blur = 0;

    h->param.i_sps_id &= 31;

    h->param.i_nal_hrd = x264_clip3( h->param.i_nal_hrd, X264_NAL_HRD_NONE, X264_NAL_HRD_CBR );

    if( h->param.i_nal_hrd && !h->param.rc.i_vbv_buffer_size )
    {
        x264_log( h, X264_LOG_WARNING, "NAL HRD parameters require VBV parameters\n" );
        h->param.i_nal_hrd = X264_NAL_HRD_NONE;
    }

    if( h->param.i_nal_hrd == X264_NAL_HRD_CBR &&
       (h->param.rc.i_bitrate != h->param.rc.i_vbv_max_bitrate || !h->param.rc.i_vbv_max_bitrate) )
    {
        x264_log( h, X264_LOG_WARNING, "CBR HRD requires constant bitrate\n" );
        h->param.i_nal_hrd = X264_NAL_HRD_VBR;
    }

    /* ensure the booleans are 0 or 1 so they can be used in math */
#define BOOLIFY(x) h->param.x = !!h->param.x
    BOOLIFY( b_cabac );
    BOOLIFY( b_constrained_intra );
    BOOLIFY( b_deblocking_filter );
    BOOLIFY( b_deterministic );
    BOOLIFY( b_sliced_threads );
    BOOLIFY( b_interlaced );
    BOOLIFY( b_intra_refresh );
    BOOLIFY( b_aud );
    BOOLIFY( b_repeat_headers );
    BOOLIFY( b_annexb );
    BOOLIFY( b_vfr_input );
    BOOLIFY( b_pulldown );
    BOOLIFY( b_tff );
    BOOLIFY( b_pic_struct );
    BOOLIFY( b_fake_interlaced );
    BOOLIFY( b_open_gop );
    BOOLIFY( b_bluray_compat );
    BOOLIFY( analyse.b_transform_8x8 );
    BOOLIFY( analyse.b_weighted_bipred );
    BOOLIFY( analyse.b_chroma_me );
    BOOLIFY( analyse.b_mixed_references );
    BOOLIFY( analyse.b_fast_pskip );
    BOOLIFY( analyse.b_dct_decimate );
    BOOLIFY( analyse.b_psy );
    BOOLIFY( analyse.b_psnr );
    BOOLIFY( analyse.b_ssim );
    BOOLIFY( rc.b_stat_write );
    BOOLIFY( rc.b_stat_read );
    BOOLIFY( rc.b_mb_tree );
#undef BOOLIFY

    return 0;
}

/* macro-block comparison functions init */
static void mbcmp_init( x264_t *h )
{
	/******************************************************************
	 * SAD (Sum of Absolute Difference) = SAE (Sum of Absolute Error) *
	 * SATD (Sum of Absolute Transformed Difference)                  *
	 *                                                                *
	 * Use SATD for Intra except subme == 0.                          *
	 * Use SAD for me full-pixel search except TESA.                  *
	 * Use SATD for me sub-pixel refinement.                          *
	 ******************************************************************/
    int satd = h->param.analyse.i_subpel_refine > 0;
    memcpy( h->pixf.mbcmp, satd ? h->pixf.satd : h->pixf.sad_aligned, sizeof(h->pixf.mbcmp) );
    memcpy( h->pixf.mbcmp_unaligned, satd ? h->pixf.satd : h->pixf.sad, sizeof(h->pixf.mbcmp_unaligned) );
    h->pixf.intra_mbcmp_x3_16x16 = satd ? h->pixf.intra_satd_x3_16x16 : h->pixf.intra_sad_x3_16x16;
    h->pixf.intra_mbcmp_x3_8x8c  = satd ? h->pixf.intra_satd_x3_8x8c  : h->pixf.intra_sad_x3_8x8c;
    h->pixf.intra_mbcmp_x3_4x4 = satd ? h->pixf.intra_satd_x3_4x4 : h->pixf.intra_sad_x3_4x4;
    h->pixf.intra_mbcmp_x4_4x4_h = satd ? h->pixf.intra_satd_x4_4x4_h : h->pixf.intra_sad_x4_4x4_h;
    h->pixf.intra_mbcmp_x4_4x4_v = satd ? h->pixf.intra_satd_x4_4x4_v : h->pixf.intra_sad_x4_4x4_v;
    h->pixf.intra_mbcmp_x9_4x4 = h->param.b_cpu_independent ? NULL : satd ? h->pixf.intra_satd_x9_4x4 : h->pixf.intra_sad_x9_4x4;

    satd &= h->param.analyse.i_me_method == X264_ME_TESA;
    memcpy( h->pixf.fpelcmp, satd ? h->pixf.satd : h->pixf.sad, sizeof(h->pixf.fpelcmp) );
    memcpy( h->pixf.fpelcmp_x3, satd ? h->pixf.satd_x3 : h->pixf.sad_x3, sizeof(h->pixf.fpelcmp_x3) );
    memcpy( h->pixf.fpelcmp_x4, satd ? h->pixf.satd_x4 : h->pixf.sad_x4, sizeof(h->pixf.fpelcmp_x4) );
}

/* chroma-dependent dsp functions init */
static void chroma_dsp_init( x264_t *h )
{
    /* copy luma -> chroma pixel table of current format to h->luma2chroma_pixel */
	memcpy( h->luma2chroma_pixel, x264_luma2chroma_pixel[CHROMA_FORMAT], sizeof(h->luma2chroma_pixel) );

    switch( CHROMA_FORMAT )
    {
        case CHROMA_420:
            memcpy( h->predict_chroma, h->predict_8x8c, sizeof(h->predict_chroma) );
            h->mc.prefetch_fenc = h->mc.prefetch_fenc_420;
            h->pixf.intra_mbcmp_x3_chroma = h->pixf.intra_mbcmp_x3_8x8c;
            h->quantf.coeff_last[DCT_CHROMA_DC] = h->quantf.coeff_last4;
            h->quantf.coeff_level_run[DCT_CHROMA_DC] = h->quantf.coeff_level_run4;
            break;
        case CHROMA_422:
        case CHROMA_444:
            break;
    }
}

/****************************************************************************
 * x264_encoder_open:
 ****************************************************************************/
x264_t *x264_encoder_open( x264_param_t *param )
{
    x264_t *h;
    int i_slicetype_length, i;
	const char *profile;
	char level[4];

    CHECKED_MALLOCZERO( h, sizeof(x264_t) );

    /* Create a copy of param */
    memcpy( &h->param, param, sizeof(x264_param_t) );

    if( param->param_free )
        param->param_free( param );

    if( x264_validate_parameters( h, 1 ) < 0 )
        goto fail;

    /* Init x264_t */
    h->i_frame = -1;
    h->i_frame_num = 0;
    h->i_idr_pic_id = 0;

    x264_sps_init( h->sps, h->param.i_sps_id, &h->param );
    x264_pps_init( h->pps, h->param.i_sps_id, &h->param, h->sps );

    x264_validate_levels( h, 1 );

    /* set qp mapping of luma => chroma for encoding */
    h->chroma_qp_table = i_chroma_qp_table + 12 + h->pps->i_chroma_qp_index_offset;

    if( x264_cqm_init( h ) < 0 )
        goto fail;

    h->mb.i_mb_width = h->sps->i_mb_width;
    h->mb.i_mb_height = h->sps->i_mb_height;
    h->mb.i_mb_count = h->mb.i_mb_width * h->mb.i_mb_height;

    h->mb.chroma_h_shift = CHROMA_FORMAT == CHROMA_420;
    h->mb.chroma_v_shift = CHROMA_FORMAT == CHROMA_420;

    /* Adaptive MBAFF and subme 0 are not supported as we require halving motion
     * vectors during prediction, resulting in hpel mvs.
     * The chosen solution is to make MBAFF non-adaptive in this case. */
    h->mb.b_adaptive_mbaff = PARAM_INTERLACED && h->param.analyse.i_subpel_refine;

    /* Init frames. */
    if( h->param.i_bframe_adaptive == X264_B_ADAPT_TRELLIS && !h->param.rc.b_stat_read )
        h->frames.i_delay = X264_MAX(h->param.i_bframe,3)*4;
    else
        h->frames.i_delay = h->param.i_bframe;
    if( h->param.rc.b_mb_tree || h->param.rc.i_vbv_buffer_size )
        h->frames.i_delay = X264_MAX( h->frames.i_delay, h->param.rc.i_lookahead );
    i_slicetype_length = h->frames.i_delay;
    h->frames.i_delay += h->i_thread_frames - 1;
    h->frames.i_delay += h->param.i_sync_lookahead;
    h->frames.i_delay += h->param.b_vfr_input;
    h->frames.i_bframe_delay = h->param.i_bframe ? (h->param.i_bframe_pyramid ? 2 : 1) : 0;

    h->frames.i_max_ref0 = h->param.i_frame_reference;
    h->frames.i_max_ref1 = X264_MIN( h->sps->vui.i_num_reorder_frames, h->param.i_frame_reference );
    h->frames.i_max_dpb  = h->sps->vui.i_max_dec_frame_buffering;
    h->frames.b_have_lowres =                     /* Have half-resolution frame only if: */
        ( h->param.rc.i_rc_method == X264_RC_ABR  /* 1. ABR/CBR mode. (need frame cost)  */
        || h->param.rc.i_rc_method == X264_RC_CRF /* 2. or CRF mode.  (need frame cost)  */
        || h->param.i_bframe_adaptive             /* 3. or B-frame adaptive is enabled.  */
        || h->param.i_scenecut_threshold          /* 4. or scenecut threshold is enabled.*/
        || h->param.rc.b_mb_tree                  /* 5. or mb_tree rc is enabled.        */
        || h->param.analyse.i_weighted_pred );    /* 6. or weighted predict is enabled.  */
    h->frames.b_have_sub8x8_esa = !!(h->param.analyse.inter & X264_ANALYSE_PSUB8x8);

    h->frames.i_last_idr =
    h->frames.i_last_keyframe = - h->param.i_keyint_max;
    h->frames.i_input    = 0;
    h->frames.i_largest_pts = h->frames.i_second_largest_pts = -1;
    h->frames.i_poc_last_open_gop = -1;

    CHECKED_MALLOCZERO( h->frames.unused[0], (h->frames.i_delay + 3) * sizeof(x264_frame_t *) );
    /* Allocate room for max refs plus a few extra just in case. */
    CHECKED_MALLOCZERO( h->frames.unused[1], (h->i_thread_frames + X264_REF_MAX + 4) * sizeof(x264_frame_t *) );
    CHECKED_MALLOCZERO( h->frames.current, (h->param.i_sync_lookahead + h->param.i_bframe + h->i_thread_frames + 3) * sizeof(x264_frame_t *) );
    if( h->param.analyse.i_weighted_pred > 0 )
        CHECKED_MALLOCZERO( h->frames.blank_unused, h->i_thread_frames * 4 * sizeof(x264_frame_t *) );
    h->i_ref[0] = h->i_ref[1] = 0;
    h->i_cpb_delay = h->i_coded_fields = h->i_disp_fields = 0;
    h->i_prev_duration = ((uint64_t)h->param.i_fps_den * h->sps->vui.i_time_scale) / ((uint64_t)h->param.i_fps_num * h->sps->vui.i_num_units_in_tick);
    h->i_disp_fields_last_frame = -1;

    /* init CPU functions */
    x264_predict_16x16_init( h->param.cpu, h->predict_16x16 );
    x264_predict_8x8c_init( h->param.cpu, h->predict_8x8c );
    x264_predict_4x4_init( h->param.cpu, h->predict_4x4 );
    x264_pixel_init( h->param.cpu, &h->pixf );
    x264_dct_init( h->param.cpu, &h->dctf );
    x264_zigzag_init( h->param.cpu, &h->zigzagf );
    x264_mc_init( h->param.cpu, &h->mc );
    x264_quant_init( h, h->param.cpu, &h->quantf );
    x264_deblock_init( h->param.cpu, &h->loopf );
    x264_bitstream_init( h->param.cpu, &h->bsf );
    if( h->param.b_cabac )
        x264_cabac_init( h );
    else
        x264_stack_align( x264_cavlc_init, h );

    /* init pixel metrics functions */
    mbcmp_init( h );
    chroma_dsp_init( h );

    /* init analyse cost for each qp between qp_min and qp_max */
    x264_analyse_init_costs( h );

    /* init bitstream output */
    h->out.i_nal = 0;
    h->out.i_bitstream = X264_MAX( 1000000, h->param.i_width * h->param.i_height * 4
        * ( h->param.rc.i_rc_method == X264_RC_ABR ? pow( 0.95, h->param.rc.i_qp_min )
          : pow( 0.95, h->param.rc.i_qp_constant ) * X264_MAX( 1, h->param.rc.f_ip_factor )));

    h->nal_buffer_size = h->out.i_bitstream * 3/2 + 4; /* at least 1.5 MB */
    CHECKED_MALLOC( h->nal_buffer, h->nal_buffer_size );

    /* set main thread struct, point to global struct */
    h->thread[0] = h;
    for( i = 1; i < h->param.i_threads + !!h->param.i_sync_lookahead; i++ )
        CHECKED_MALLOC( h->thread[i], sizeof(x264_t) );

    for( i = 0; i < h->param.i_threads; i++ )
    {
        int init_nal_count = h->param.i_slice_count + 3;
        int allocate_threadlocal_data = !h->param.b_sliced_threads || !i;
        if( i > 0 )
            *h->thread[i] = *h;

        if( allocate_threadlocal_data )
        {
            h->thread[i]->fdec = x264_frame_pop_unused( h, 1 );
            if( !h->thread[i]->fdec )
                goto fail;
        }
        else
            h->thread[i]->fdec = h->thread[0]->fdec;

        CHECKED_MALLOC( h->thread[i]->out.p_bitstream, h->out.i_bitstream ); /* at least 1 MB */
        /* Start each thread with room for init_nal_count NAL units; it'll realloc later if needed. */
        CHECKED_MALLOC( h->thread[i]->out.nal, init_nal_count*sizeof(x264_nal_t) );
        h->thread[i]->out.i_nals_allocated = init_nal_count;

        if( allocate_threadlocal_data && x264_macroblock_cache_allocate( h->thread[i] ) < 0 )
            goto fail;
    }

    /* init lookahead module */
    if( x264_lookahead_init( h, i_slicetype_length ) )
        goto fail;

    for( i = 0; i < h->param.i_threads; i++ )
        if( x264_macroblock_thread_allocate( h->thread[i], 0 ) < 0 )
            goto fail;

    /* init rate control module */
    if( x264_ratecontrol_new( h ) < 0 )
        goto fail;

    profile = h->sps->i_profile_idc == PROFILE_BASELINE ? "Constrained Baseline" :
		h->sps->i_profile_idc == PROFILE_MAIN ? "Main" : "High";
    snprintf( level, sizeof(level), "%d.%d", h->sps->i_level_idc/10, h->sps->i_level_idc%10 );
    x264_log( h, X264_LOG_INFO, "profile %s, level %s\n", profile, level );

    return h;
fail:
    x264_free( h );
    return NULL;
}

/****************************************************************************
 * x264_encoder_parameters:
 ****************************************************************************/
void x264_encoder_parameters( x264_t *h, x264_param_t *param )
{
    memcpy( param, &h->thread[h->i_thread_phase]->param, sizeof(x264_param_t) );
}

/* internal usage */
static void x264_nal_start( x264_t *h, int i_type, int i_ref_idc )
{
    x264_nal_t *nal = &h->out.nal[h->out.i_nal];

    nal->i_ref_idc        = i_ref_idc;
    nal->i_type           = i_type;
    nal->b_long_startcode = 1;

    nal->i_payload= 0;
    nal->p_payload= &h->out.p_bitstream[bs_pos( &h->out.bs ) >> 3];
}

/* if number of allocated nals is not enough, re-allocate a larger one, twice as last allocated. */
static int x264_nal_check_buffer( x264_t *h )
{
    if( h->out.i_nal >= h->out.i_nals_allocated )
    {
        x264_nal_t *new_out = x264_malloc( sizeof(x264_nal_t) * (h->out.i_nals_allocated*2) );
        if( !new_out )
            return -1;
        memcpy( new_out, h->out.nal, sizeof(x264_nal_t) * (h->out.i_nals_allocated) );
        x264_free( h->out.nal );
        h->out.nal = new_out;
        h->out.i_nals_allocated *= 2;
    }
    return 0;
}

static int x264_nal_end( x264_t *h )
{
    x264_nal_t *nal = &h->out.nal[h->out.i_nal];
    uint8_t *end = &h->out.p_bitstream[bs_pos( &h->out.bs ) >> 3];
    nal->i_payload = end - nal->p_payload;
    /* nal_escape_mmx reads past the end of the input.
     * While undefined padding wouldn't actually affect the output, it makes valgrind unhappy. */
    memset( end, 0xff, 32 );
    if( h->param.nalu_process )
        h->param.nalu_process( h, nal, h->fenc->opaque );
    h->out.i_nal++;

    return x264_nal_check_buffer( h );
}

static int x264_encoder_encapsulate_nals( x264_t *h, int start )
{
    int nal_size = 0, previous_nal_size = 0;
	int i, necessary_size;
	uint8_t *nal_buffer;

    if( h->param.nalu_process )
    {
        for( i = start; i < h->out.i_nal; i++ )
            nal_size += h->out.nal[i].i_payload;
        return nal_size;
    }

    for( i = 0; i < start; i++ )
        previous_nal_size += h->out.nal[i].i_payload;

    for( i = start; i < h->out.i_nal; i++ )
        nal_size += h->out.nal[i].i_payload;

    /* Worst-case NAL unit escaping: reallocate the buffer if it's too small. */
    necessary_size = ((nal_size * 3) >> 1) + (h->out.i_nal << 2);
    if( h->nal_buffer_size < necessary_size )
    {
		uint8_t *buf;
        h->nal_buffer_size = necessary_size << 1;
        buf = x264_malloc( h->nal_buffer_size );
        if( !buf )
            return -1;
        if( previous_nal_size )
            memcpy( buf, h->nal_buffer, previous_nal_size );
        x264_free( h->nal_buffer );
        h->nal_buffer = buf;
    }

    nal_buffer = h->nal_buffer + previous_nal_size;

    for( i = start; i < h->out.i_nal; i++ )
    {
        /* h->out.nal[i].b_long_startcode = !i || h->out.nal[i].i_type == NAL_SPS || h->out.nal[i].i_type == NAL_PPS; */
        x264_nal_encode( h, nal_buffer, &h->out.nal[i] );
        nal_buffer += h->out.nal[i].i_payload; /* accumulate size of encapsulated nals */
    }

    return nal_buffer - (h->nal_buffer + previous_nal_size);
}

/****************************************************************************
 * x264_encoder_headers:
 ****************************************************************************/
int x264_encoder_headers( x264_t *h, x264_nal_t **pp_nal, int *pi_nal )
{
    int frame_size = 0;
    /* init bitstream context */
    h->out.i_nal = 0;
    bs_init( &h->out.bs, h->out.p_bitstream, h->out.i_bitstream );

    /* Write SEI, SPS and PPS. */

    /* generate sequence parameters */
    x264_nal_start( h, NAL_SPS, NAL_PRIORITY_HIGHEST );
    x264_sps_write( &h->out.bs, h->sps );
    if( x264_nal_end( h ) )
        return -1;

    /* generate picture parameters */
    x264_nal_start( h, NAL_PPS, NAL_PRIORITY_HIGHEST );
    x264_pps_write( &h->out.bs, h->sps, h->pps );
    if( x264_nal_end( h ) )
        return -1;

    /* identify ourselves */
    x264_nal_start( h, NAL_SEI, NAL_PRIORITY_DISPOSABLE );
    if( x264_sei_version_write( h, &h->out.bs ) )
        return -1;
    if( x264_nal_end( h ) )
        return -1;

    frame_size = x264_encoder_encapsulate_nals( h, 0 );
    if( frame_size < 0 )
        return -1;

    /* now set output*/
    *pi_nal = h->out.i_nal;
    *pp_nal = &h->out.nal[0];
    h->out.i_nal = 0;

    return frame_size;
}

/* Check to see whether we have chosen a reference list ordering different
 * from the standard's default. */
static inline void x264_reference_check_reorder( x264_t *h )
{
	int i, list;
    /* The reorder check doesn't check for missing frames, so just
     * force a reorder if one of the reference list is corrupt. */
    for( i = 0; h->frames.reference[i]; i++ )
        if( h->frames.reference[i]->b_corrupt )
        {
            h->b_ref_reorder[0] = 1;
            return;
        }
    for( list = 0; list <= (h->sh.i_type == SLICE_TYPE_B); list++ )
        for( i = 0; i < h->i_ref[list] - 1; i++ )
        {
            int framenum_diff = h->fref[list][i+1]->i_frame_num - h->fref[list][i]->i_frame_num;
            int poc_diff = h->fref[list][i+1]->i_poc - h->fref[list][i]->i_poc;
            /* P and B-frames use different default orders. */
            if( h->sh.i_type == SLICE_TYPE_P ? framenum_diff > 0 : list == 1 ? poc_diff < 0 : poc_diff > 0 )
            {
                h->b_ref_reorder[list] = 1;
                return;
            }
        }
}

/* Calculate absolute distance between specified frame and currently encoding frame */
static inline int x264_reference_distance( x264_t *h, x264_frame_t *frame )
{
    if( h->param.i_frame_packing == 5 ) /* 3D video by one view per frame */
        return abs((h->fenc->i_frame&~1) - (frame->i_frame&~1)) +
                  ((h->fenc->i_frame&1) != (frame->i_frame&1));
    else
        return abs(h->fenc->i_frame - frame->i_frame);
}

static inline void x264_reference_build_list( x264_t *h, int i_poc )
{
    int i, list, b_ok;

    /* build ref list 0/1 */
    h->mb.pic.i_fref[0] = h->i_ref[0] = 0;
    h->mb.pic.i_fref[1] = h->i_ref[1] = 0;
    if( h->sh.i_type == SLICE_TYPE_I )
        return;

    for( i = 0; h->frames.reference[i]; i++ )
    {
        if( h->frames.reference[i]->b_corrupt )
            continue;
        if( h->frames.reference[i]->i_poc < i_poc )
            h->fref[0][h->i_ref[0]++] = h->frames.reference[i];
        else if( h->frames.reference[i]->i_poc > i_poc )
            h->fref[1][h->i_ref[1]++] = h->frames.reference[i];
    }

    /* Order reference lists by distance from the current frame. */
    for( list = 0; list < 2; list++ )
    {
        h->fref_nearest[list] = h->fref[list][0];
        do
        {
            b_ok = 1;
            for( i = 0; i < h->i_ref[list] - 1; i++ )
            {
                if( list ? h->fref[list][i+1]->i_poc < h->fref_nearest[list]->i_poc   /* for l1, the frame of smallest poc is the nearest. */
                         : h->fref[list][i+1]->i_poc > h->fref_nearest[list]->i_poc ) /* for l0, the frame of largest  poc is the nearest. */
                    h->fref_nearest[list] = h->fref[list][i+1];
                if( x264_reference_distance( h, h->fref[list][i] ) > x264_reference_distance( h, h->fref[list][i+1] ) )
                {
                    XCHG( x264_frame_t*, h->fref[list][i], h->fref[list][i+1] );
                    b_ok = 0;
                    break;
                }
            }
        } while( !b_ok );
    }

    if( h->sh.i_mmco_remove_from_end )
        for( i = h->i_ref[0]-1; i >= h->i_ref[0] - h->sh.i_mmco_remove_from_end; i-- )
        {
            int diff = h->i_frame_num - h->fref[0][i]->i_frame_num;
            h->sh.mmco[h->sh.i_mmco_command_count].i_poc = h->fref[0][i]->i_poc;
            h->sh.mmco[h->sh.i_mmco_command_count++].i_difference_of_pic_nums = diff;
        }

    x264_reference_check_reorder( h );

    h->i_ref[1] = X264_MIN( h->i_ref[1], h->frames.i_max_ref1 );
    h->i_ref[0] = X264_MIN( h->i_ref[0], h->frames.i_max_ref0 );
    h->i_ref[0] = X264_MIN( h->i_ref[0], h->param.i_frame_reference ); // if reconfig() has lowered the limit

    /* add duplicates */
    if( h->fenc->i_type == X264_TYPE_P )
        h->mb.ref_blind_dupe = -1;

    /*assert( h->i_ref[0] + h->i_ref[1] <= X264_REF_MAX );*/
    h->mb.pic.i_fref[0] = h->i_ref[0];
    h->mb.pic.i_fref[1] = h->i_ref[1];
}

static inline int x264_reference_update( x264_t *h )
{
	int i, j;
    if( !h->fdec->b_kept_as_ref )
    {
        if( h->i_thread_frames > 1 )
        {
            x264_frame_push_unused( h, h->fdec );
            h->fdec = x264_frame_pop_unused( h, 1 );
            if( !h->fdec )
                return -1;
        }
        return 0;
    }

    /* apply mmco from previous frame. */
    for( i = 0; i < h->sh.i_mmco_command_count; i++ )
        for( j = 0; h->frames.reference[j]; j++ )
            if( h->frames.reference[j]->i_poc == h->sh.mmco[i].i_poc )
                x264_frame_push_unused( h, x264_frame_shift( &h->frames.reference[j] ) );

    /* move frame in the buffer */
    x264_frame_push( h->frames.reference, h->fdec );
    if( h->frames.reference[h->sps->i_num_ref_frames] )
        x264_frame_push_unused( h, x264_frame_shift( h->frames.reference ) );
    h->fdec = x264_frame_pop_unused( h, 1 );
    if( !h->fdec )
        return -1;
    return 0;
}

static inline void x264_reference_reset( x264_t *h )
{
    while( h->frames.reference[0] )
        x264_frame_push_unused( h, x264_frame_pop( h->frames.reference ) );
    h->fdec->i_poc =
    h->fenc->i_poc = 0;
}

static inline void x264_reference_hierarchy_reset( x264_t *h )
{
    int ref, i;
    int b_hasdelayframe = 0;

    /* look for delay frames -- chain must only contain frames that are disposable */
    for( i = 0; h->frames.current[i] && IS_DISPOSABLE( h->frames.current[i]->i_type ); i++ )
        b_hasdelayframe |= h->frames.current[i]->i_coded
                        != h->frames.current[i]->i_frame + h->sps->vui.i_num_reorder_frames;

    /* This function must handle b-pyramid and clear frames for open-gop */
    if( h->param.i_bframe_pyramid != X264_B_PYRAMID_STRICT && !b_hasdelayframe && h->frames.i_poc_last_open_gop == -1 )
        return;

    /* Remove last BREF. There will never be old BREFs in the
     * dpb during a BREF decode when pyramid == STRICT */
    for( ref = 0; h->frames.reference[ref]; ref++ )
    {
        if( ( h->param.i_bframe_pyramid == X264_B_PYRAMID_STRICT
            && h->frames.reference[ref]->i_type == X264_TYPE_BREF )
            || ( h->frames.reference[ref]->i_poc < h->frames.i_poc_last_open_gop
            && h->sh.i_type != SLICE_TYPE_B ) )
        {
            int diff = h->i_frame_num - h->frames.reference[ref]->i_frame_num;
            h->sh.mmco[h->sh.i_mmco_command_count].i_difference_of_pic_nums = diff;
            h->sh.mmco[h->sh.i_mmco_command_count++].i_poc = h->frames.reference[ref]->i_poc;
            x264_frame_push_unused( h, x264_frame_shift( &h->frames.reference[ref] ) );
            h->b_ref_reorder[0] = 1;
            ref--;
        }
    }

    /* Prepare room in the dpb for the delayed display time of the later b-frame's */
    if( h->param.i_bframe_pyramid )
        h->sh.i_mmco_remove_from_end = X264_MAX( ref + 2 - h->frames.i_max_dpb, 0 );
}

/* Fill "default" values */
static void x264_slice_header_init( x264_t *h, x264_slice_header_t *sh,
                                    x264_sps_t *sps, x264_pps_t *pps,
                                    int i_idr_pic_id, int i_frame, int i_qp )
{
    x264_param_t *param = &h->param;
	int i, deblock_thresh, list;

    /* First we fill all fields */
    sh->sps = sps;
    sh->pps = pps;

    sh->i_first_mb  = 0;
    sh->i_last_mb   = h->mb.i_mb_count - 1;
    sh->i_pps_id    = pps->i_id;

    sh->i_frame_num = i_frame;

    sh->b_mbaff = PARAM_INTERLACED; /* enable MBAFF in interlaced mode, otherwise disable it. */
    sh->b_field_pic = 0;    /* no field support for now */
    sh->b_bottom_field = 0; /* not yet used */

    sh->i_idr_pic_id = i_idr_pic_id;

    /* poc stuff, fixed later */
    sh->i_poc = 0;
    sh->i_delta_poc_bottom = 0;
    sh->i_delta_poc[0] = 0;
    sh->i_delta_poc[1] = 0;

    sh->i_redundant_pic_cnt = 0;

    h->mb.b_direct_auto_write = h->param.analyse.i_direct_mv_pred == X264_DIRECT_PRED_AUTO
                                && h->param.i_bframe
                                && ( h->param.rc.b_stat_write || !h->param.rc.b_stat_read );

    if( !h->mb.b_direct_auto_read && sh->i_type == SLICE_TYPE_B )
    {
        if( h->fref[1][0]->i_poc_l0ref0 == h->fref[0][0]->i_poc )
        {
            if( h->mb.b_direct_auto_write )
                sh->b_direct_spatial_mv_pred = ( h->stat.i_direct_score[1] > h->stat.i_direct_score[0] );
            else
                sh->b_direct_spatial_mv_pred = ( param->analyse.i_direct_mv_pred == X264_DIRECT_PRED_SPATIAL );
        }
        else
        {
            h->mb.b_direct_auto_write = 0;
            sh->b_direct_spatial_mv_pred = 1;
        }
    }
    /* else b_direct_spatial_mv_pred was read from the 2pass statsfile */

    sh->b_num_ref_idx_override = 0;
    sh->i_num_ref_idx_l0_active = 1;
    sh->i_num_ref_idx_l1_active = 1;

    sh->b_ref_pic_list_reordering[0] = h->b_ref_reorder[0];
    sh->b_ref_pic_list_reordering[1] = h->b_ref_reorder[1];

    /* If the ref list isn't in the default order, construct reordering header */
    for( list = 0; list < 2; list++ )
    {
        if( sh->b_ref_pic_list_reordering[list] )
        {
            int pred_frame_num = i_frame;
            for( i = 0; i < h->i_ref[list]; i++ )
            {
                int diff = h->fref[list][i]->i_frame_num - pred_frame_num;
                sh->ref_pic_list_order[list][i].idc = ( diff > 0 );
                sh->ref_pic_list_order[list][i].arg = (abs(diff) - 1) & ((1 << sps->i_log2_max_frame_num) - 1);
                pred_frame_num = h->fref[list][i]->i_frame_num;
            }
        }
    }

    sh->i_cabac_init_idc = param->i_cabac_init_idc;

    sh->i_qp = SPEC_QP(i_qp);
    sh->i_qp_delta = sh->i_qp - pps->i_pic_init_qp;
    sh->b_sp_for_swidth = 0;
    sh->i_qs_delta = 0;

    deblock_thresh = i_qp + 2 * X264_MIN(param->i_deblocking_filter_alphac0, param->i_deblocking_filter_beta);
    /* If effective qp <= 15, deblocking would have no effect anyway */
    if( param->b_deblocking_filter && (h->mb.b_variable_qp || 15 < deblock_thresh ) )
        sh->i_disable_deblocking_filter_idc = 0;
    else
        sh->i_disable_deblocking_filter_idc = 1;
    sh->i_alpha_c0_offset = param->i_deblocking_filter_alphac0 << 1;
    sh->i_beta_offset = param->i_deblocking_filter_beta << 1;
}

static void x264_slice_header_write( bs_t *s, x264_slice_header_t *sh, int i_nal_ref_idc )
{
	int i;
    if( sh->b_mbaff )
    {
        int first_x = sh->i_first_mb % sh->sps->i_mb_width;
        int first_y = sh->i_first_mb / sh->sps->i_mb_width;
        /*assert( (first_y&1) == 0 );*/
        bs_write_ue( s, (2*first_x + sh->sps->i_mb_width*(first_y&~1) + (first_y&1)) >> 1 );
    }
    else
        bs_write_ue( s, sh->i_first_mb );

    bs_write_ue( s, sh->i_type + 5 );   /* same type things */
    bs_write_ue( s, sh->i_pps_id );
    bs_write( s, sh->sps->i_log2_max_frame_num, sh->i_frame_num & ((1<<sh->sps->i_log2_max_frame_num)-1) );

    if( !sh->sps->b_frame_mbs_only )
    {
        bs_write1( s, sh->b_field_pic );
        if( sh->b_field_pic )
            bs_write1( s, sh->b_bottom_field );
    }

    if( sh->i_idr_pic_id >= 0 ) /* NAL IDR */
        bs_write_ue( s, sh->i_idr_pic_id );

    if( sh->sps->i_poc_type == 0 )
    {
        bs_write( s, sh->sps->i_log2_max_poc_lsb, sh->i_poc & ((1<<sh->sps->i_log2_max_poc_lsb)-1) );
        if( sh->pps->b_pic_order && !sh->b_field_pic )
            bs_write_se( s, sh->i_delta_poc_bottom );
    }

    if( sh->pps->b_redundant_pic_cnt )
        bs_write_ue( s, sh->i_redundant_pic_cnt );

    if( sh->i_type == SLICE_TYPE_B )
        bs_write1( s, sh->b_direct_spatial_mv_pred );

    if( sh->i_type == SLICE_TYPE_P || sh->i_type == SLICE_TYPE_B )
    {
        bs_write1( s, sh->b_num_ref_idx_override );
        if( sh->b_num_ref_idx_override )
        {
            bs_write_ue( s, sh->i_num_ref_idx_l0_active - 1 );
            if( sh->i_type == SLICE_TYPE_B )
                bs_write_ue( s, sh->i_num_ref_idx_l1_active - 1 );
        }
    }

    /* ref pic list reordering */
    if( sh->i_type != SLICE_TYPE_I )
    {
        bs_write1( s, sh->b_ref_pic_list_reordering[0] );
        if( sh->b_ref_pic_list_reordering[0] )
        {
            for( i = 0; i < sh->i_num_ref_idx_l0_active; i++ )
            {
                bs_write_ue( s, sh->ref_pic_list_order[0][i].idc );
                bs_write_ue( s, sh->ref_pic_list_order[0][i].arg );
            }
            bs_write_ue( s, 3 );
        }
    }
    if( sh->i_type == SLICE_TYPE_B )
    {
        bs_write1( s, sh->b_ref_pic_list_reordering[1] );
        if( sh->b_ref_pic_list_reordering[1] )
        {
            for( i = 0; i < sh->i_num_ref_idx_l1_active; i++ )
            {
                bs_write_ue( s, sh->ref_pic_list_order[1][i].idc );
                bs_write_ue( s, sh->ref_pic_list_order[1][i].arg );
            }
            bs_write_ue( s, 3 );
        }
    }

    sh->b_weighted_pred = 0;
    if( sh->pps->b_weighted_pred && sh->i_type == SLICE_TYPE_P )
    {
        sh->b_weighted_pred = sh->weight[0][0].weightfn || sh->weight[0][1].weightfn || sh->weight[0][2].weightfn;
        /* pred_weight_table() */
        bs_write_ue( s, sh->weight[0][0].i_denom );
        bs_write_ue( s, sh->weight[0][1].i_denom );
        for( i = 0; i < sh->i_num_ref_idx_l0_active; i++ )
        {
            int luma_weight_l0_flag = !!sh->weight[i][0].weightfn;
            int chroma_weight_l0_flag = !!sh->weight[i][1].weightfn || !!sh->weight[i][2].weightfn;
            bs_write1( s, luma_weight_l0_flag );
            if( luma_weight_l0_flag )
            {
                bs_write_se( s, sh->weight[i][0].i_scale );
                bs_write_se( s, sh->weight[i][0].i_offset );
            }
            bs_write1( s, chroma_weight_l0_flag );
            if( chroma_weight_l0_flag )
            {
				int j;
                for( j = 1; j < 3; j++ )
                {
                    bs_write_se( s, sh->weight[i][j].i_scale );
                    bs_write_se( s, sh->weight[i][j].i_offset );
                }
            }
        }
    }
    else if( sh->pps->b_weighted_bipred == 1 && sh->i_type == SLICE_TYPE_B )
    {
      /* TODO */
    }

    if( i_nal_ref_idc != 0 )
    {
        if( sh->i_idr_pic_id >= 0 )
        {
            bs_write1( s, 0 );  /* no output of prior pics flag */
            bs_write1( s, 0 );  /* long term reference flag */
        }
        else
        {
            bs_write1( s, sh->i_mmco_command_count > 0 ); /* adaptive_ref_pic_marking_mode_flag */
            if( sh->i_mmco_command_count > 0 )
            {
                for( i = 0; i < sh->i_mmco_command_count; i++ )
                {
                    bs_write_ue( s, 1 ); /* mark short term ref as unused */
                    bs_write_ue( s, sh->mmco[i].i_difference_of_pic_nums - 1 );
                }
                bs_write_ue( s, 0 ); /* end command list */
            }
        }
    }

    if( sh->pps->b_cabac && sh->i_type != SLICE_TYPE_I )
        bs_write_ue( s, sh->i_cabac_init_idc );

    bs_write_se( s, sh->i_qp_delta );      /* slice qp delta */

    if( sh->pps->b_deblocking_filter_control )
    {
        bs_write_ue( s, sh->i_disable_deblocking_filter_idc );
        if( sh->i_disable_deblocking_filter_idc != 1 )
        {
            bs_write_se( s, sh->i_alpha_c0_offset >> 1 );
            bs_write_se( s, sh->i_beta_offset >> 1 );
        }
    }
}

static inline void x264_slice_init( x264_t *h, int i_nal_type, int i_global_qp )
{
    /* ------------------------ Create slice header  ----------------------- */
    if( i_nal_type == NAL_SLICE_IDR )
    {
        x264_slice_header_init( h, &h->sh, h->sps, h->pps, h->i_idr_pic_id, h->i_frame_num, i_global_qp );

        /* alternate id */
        h->i_idr_pic_id ^= 1;
    }
    else
    {
        x264_slice_header_init( h, &h->sh, h->sps, h->pps, -1, h->i_frame_num, i_global_qp );

        h->sh.i_num_ref_idx_l0_active = h->i_ref[0] <= 0 ? 1 : h->i_ref[0];
        h->sh.i_num_ref_idx_l1_active = h->i_ref[1] <= 0 ? 1 : h->i_ref[1];
        if( h->sh.i_num_ref_idx_l0_active != h->pps->i_num_ref_idx_l0_default_active ||
            (h->sh.i_type == SLICE_TYPE_B && h->sh.i_num_ref_idx_l1_active != h->pps->i_num_ref_idx_l1_default_active) )
        {
            h->sh.b_num_ref_idx_override = 1;
        }
    }

    if( h->fenc->i_type == X264_TYPE_BREF && h->param.b_bluray_compat && h->sh.i_mmco_command_count )
    {
        h->b_sh_backup = 1;
        h->sh_backup = h->sh;
    }

    h->fdec->i_frame_num = h->sh.i_frame_num;

    if( h->sps->i_poc_type == 0 )
    {
        h->sh.i_poc = h->fdec->i_poc;
        h->sh.i_delta_poc_bottom = 0;
        h->fdec->i_delta_poc[0] = h->sh.i_delta_poc_bottom == -1;
        h->fdec->i_delta_poc[1] = h->sh.i_delta_poc_bottom ==  1;
    }
    else
    {
        /* Nothing to do ? */
    }

    /* mb init for future slice encoding */
    x264_macroblock_slice_init( h );
}

typedef struct
{
    int skip;
    uint8_t cabac_prevbyte;
    bs_t bs;
    x264_cabac_t cabac;
    x264_frame_stat_t stat;
    int last_qp;
    int last_dqp;
    int field_decoding_flag;
} x264_bs_bak_t;

/* If we are within a reasonable distance of the end of the memory allocated for the bitstream, */
/* reallocate, adding an arbitrary amount of space. */
static int x264_bitstream_check_buffer( x264_t *h )
{
    uint8_t *bs_bak = h->out.p_bitstream;
    int max_row_size = (2500 << SLICE_MBAFF) * h->mb.i_mb_width;
    if( (h->param.b_cabac && (h->cabac.p_end - h->cabac.p < max_row_size)) ||
        (h->out.bs.p_end - h->out.bs.p < max_row_size) )
    {
		intptr_t delta;
		int i;
        h->out.i_bitstream += max_row_size;
        CHECKED_MALLOC( h->out.p_bitstream, h->out.i_bitstream );
        h->mc.memcpy_aligned( h->out.p_bitstream, bs_bak, (h->out.i_bitstream - max_row_size) & ~15 );
        delta = h->out.p_bitstream - bs_bak;

        h->out.bs.p_start += delta;
        h->out.bs.p += delta;
        h->out.bs.p_end = h->out.p_bitstream + h->out.i_bitstream;

        h->cabac.p_start += delta;
        h->cabac.p += delta;
        h->cabac.p_end = h->out.p_bitstream + h->out.i_bitstream;

        for( i = 0; i <= h->out.i_nal; i++ )
            h->out.nal[i].p_payload += delta;
        x264_free( bs_bak );
    }
    return 0;
fail:
    x264_free( bs_bak );
    return -1;
}

static ALWAYS_INLINE void x264_bitstream_backup( x264_t *h, x264_bs_bak_t *bak, int i_skip, int full )
{
    if( full )
    {
        bak->stat = h->stat.frame;
        bak->last_qp = h->mb.i_last_qp;
        bak->last_dqp = h->mb.i_last_dqp;
        bak->field_decoding_flag = h->mb.field_decoding_flag;
    }
    else
    {
        bak->stat.i_mv_bits = h->stat.frame.i_mv_bits;
        bak->stat.i_tex_bits = h->stat.frame.i_tex_bits;
    }
    /* In the per-MB backup, we don't need the contexts because flushing the CABAC
     * encoder has no context dependency and in this case, a slice is ended (and
     * thus the content of all contexts are thrown away). */
    if( h->param.b_cabac )
    {
        if( full )
            memcpy( &bak->cabac, &h->cabac, sizeof(x264_cabac_t) );
        else
            memcpy( &bak->cabac, &h->cabac, offsetof(x264_cabac_t, f8_bits_encoded) );
        /* x264's CABAC writer modifies the previous byte during carry, so it has to be
         * backed up. */
        bak->cabac_prevbyte = h->cabac.p[-1];
    }
    else
    {
        bak->bs = h->out.bs;
        bak->skip = i_skip;
    }
}

static ALWAYS_INLINE void x264_bitstream_restore( x264_t *h, x264_bs_bak_t *bak, int *skip, int full )
{
    if( full )
    {
        h->stat.frame = bak->stat;
        h->mb.i_last_qp = bak->last_qp;
        h->mb.i_last_dqp = bak->last_dqp;
        h->mb.field_decoding_flag = bak->field_decoding_flag;
    }
    else
    {
        h->stat.frame.i_mv_bits = bak->stat.i_mv_bits;
        h->stat.frame.i_tex_bits = bak->stat.i_tex_bits;
    }
    if( h->param.b_cabac )
    {
        if( full )
            memcpy( &h->cabac, &bak->cabac, sizeof(x264_cabac_t) );
        else
            memcpy( &h->cabac, &bak->cabac, offsetof(x264_cabac_t, f8_bits_encoded) );
        h->cabac.p[-1] = bak->cabac_prevbyte;
    }
    else
    {
        h->out.bs = bak->bs;
        *skip = bak->skip;
    }
}

/* x264_fdec_filter_row
 * apply deblocking and hpel filter to rows of reconstructing frame.
 * x264_t *h: handle of x264 instance
 * int mb_y:  row index to be encoded next of current frame
 */
static void x264_fdec_filter_row( x264_t *h, int mb_y )
{
	/* mb_y is the mb to be encoded next, not the mb to be filtered here */
    int b_deblock = (h->sh.i_disable_deblocking_filter_idc != 1) & h->fdec->b_kept_as_ref; /* do deblocking only if kept as ref. */
    int min_y = mb_y - 1;

    if( min_y < h->i_threadslice_start )
        return;

    /* call deblocking filter for mb row */
    if( b_deblock )
    	x264_frame_deblock_row( h, min_y );

    /* call hpel filter for mb row */
    if( h->fdec->b_kept_as_ref )
    {
    	/* expand borders first */
    	x264_frame_expand_border( h, h->fdec, min_y );

        /* Can't do hpel until the previous slice is done encoding. */
        if( h->param.analyse.i_subpel_refine )
        {
        	int end = mb_y == h->mb.i_mb_height;
        	x264_frame_filter( h, h->fdec, min_y, end );
        	x264_frame_expand_border_filtered( h, h->fdec, min_y, end );
        }
    }

    /* psnr and ssim calculation */
#ifdef _DEBUG
    {
    	int b_end = mb_y == h->i_threadslice_end;      /* whether last mb line of current slice */
    	int b_start = min_y == h->i_threadslice_start; /* whether first mb line of current slice */
        /* Even in interlaced mode, deblocking never modifies more than 4 pixels
         * above each MB, as bS=4 doesn't happen for the top of interlaced mbpairs. */
        int minpix_y = (min_y << 4) - ((!b_start) << 2); /* min_y*16 - 4 * !b_start */
        int maxpix_y = (mb_y  << 4) - ((!b_end)   << 2); /* mb_y*16 - 4 * !b_end */

        maxpix_y = X264_MIN( maxpix_y, h->param.i_height );
        if( h->param.analyse.b_psnr )
        {
			uint64_t ssd_u, ssd_v;
			int v_shift = CHROMA_V_SHIFT;

			h->stat.frame.i_ssd[0] += x264_pixel_ssd_wxh( &h->pixf,
				h->fdec->plane[0] + minpix_y * h->fdec->i_stride[0], h->fdec->i_stride[0],
				h->fenc->plane[0] + minpix_y * h->fenc->i_stride[0], h->fenc->i_stride[0],
				h->param.i_width, maxpix_y-minpix_y );

			x264_pixel_ssd_nv12( &h->pixf,
				h->fdec->plane[1] + (minpix_y>>v_shift) * h->fdec->i_stride[1], h->fdec->i_stride[1],
				h->fenc->plane[1] + (minpix_y>>v_shift) * h->fenc->i_stride[1], h->fenc->i_stride[1],
				h->param.i_width>>1, (maxpix_y-minpix_y)>>v_shift, &ssd_u, &ssd_v );
			h->stat.frame.i_ssd[1] += ssd_u;
			h->stat.frame.i_ssd[2] += ssd_v;
        }

        if( h->param.analyse.b_ssim )
        {
            int ssim_cnt;
            /* offset by 2 pixels to avoid alignment of ssim blocks with dct blocks,
             * and overlap by 4 */
            minpix_y += b_start ? 2 : -6;
            h->stat.frame.f_ssim +=
                x264_pixel_ssim_wxh( &h->pixf,
                    h->fdec->plane[0] + 2+minpix_y*h->fdec->i_stride[0], h->fdec->i_stride[0],
                    h->fenc->plane[0] + 2+minpix_y*h->fenc->i_stride[0], h->fenc->i_stride[0],
                    h->param.i_width-2, maxpix_y-minpix_y, h->scratch_buffer, &ssim_cnt );
            h->stat.frame.i_ssim_cnt += ssim_cnt;
        }
    }
#endif
}

static int x264_slice_write( x264_t *h )
{
    int i_skip;
    int mb_xy, i_mb_x, i_mb_y;
    int back_up_bitstream = !h->param.b_cabac && h->sps->i_profile_idc < PROFILE_HIGH;
    int starting_bits = bs_pos(&h->out.bs);
    int b_deblock = (h->sh.i_disable_deblocking_filter_idc != 1) & h->fdec->b_kept_as_ref; /* do deblocking only if kept as ref. */
    int orig_last_mb = h->sh.i_last_mb;
    x264_bs_bak_t bs_bak[2];
	int total_bits, mb_size, mb_spos;
#ifdef _DEBUG
	int b_intra, b_skip;
#endif

    bs_realign( &h->out.bs );

    /* init stats */
    memset( &h->stat.frame, 0, sizeof(h->stat.frame) );
    h->mb.b_reencode_mb = 0;

    /* Slice */
    x264_nal_start( h, h->i_nal_type, h->i_nal_ref_idc );
    h->out.nal[h->out.i_nal].i_first_mb = h->sh.i_first_mb;

    /* Slice header */
    x264_macroblock_thread_init( h );

    /* If this isn't the first slice in the threadslice, set the slice QP
     * equal to the last QP in the previous slice for more accurate
     * CABAC initialization. */
    if( h->sh.i_first_mb != h->i_threadslice_start * h->mb.i_mb_width )
    {
        h->sh.i_qp = h->mb.i_last_qp;
        h->sh.i_qp_delta = h->sh.i_qp - h->pps->i_pic_init_qp;
    }

    x264_slice_header_write( &h->out.bs, &h->sh, h->i_nal_ref_idc );
    if( h->param.b_cabac )
    {
        /* alignment needed */
        bs_align_1( &h->out.bs );

        /* init cabac */
        x264_cabac_context_init( h, &h->cabac, h->sh.i_type, x264_clip3( h->sh.i_qp-QP_BD_OFFSET, 0, 51 ), h->sh.i_cabac_init_idc );
        /* pass start and end position of bitstream to cabac */
        x264_cabac_encode_init ( &h->cabac, h->out.bs.p, h->out.bs.p_end );
        /* last_emu_check = h->cabac.p; */
    }
    h->mb.i_last_qp = h->sh.i_qp;
    h->mb.i_last_dqp = 0;
    h->mb.field_decoding_flag = 0;

    i_mb_y = h->sh.i_first_mb / h->mb.i_mb_width; /* start position of coordinate-x */
    i_mb_x = h->sh.i_first_mb % h->mb.i_mb_width; /* start position of coordinate-y */
    i_skip = 0;

    /*********************************************************************
     * Routine of encoding all mbs of current slice.                     *
     * begin mb: h->sh.i_first_mb => end mb: h->sh.i_last_mb             *
     *                                                                   *
     * 0. deblocking per-row along with hpel by x264_fdec_filter_row()   *
     * 1. load cache of previous mbs by x264_macroblock_cache_load()     *
     * 2. analyse current mb by x264_macroblock_analyse()                *
     * 3. encode current mb by x264_macroblock_encode()                  *
     * 4. write cabac/cavlc by x264_macroblock_write_***()               *
     * 5. save cache of current mb by x264_macroblock_cache_save()       *
     * 6. update ratecontrol of mb by x264_ratecontrol_mb()              *
     * 7. accumulate mb stats                                            *
     * 8. check if we reach the end, otherwise update i_mb_x, i_mb_y     *
     *********************************************************************/
    while( 1 )
    {
		mb_spos = bs_pos(&h->out.bs) + x264_cabac_pos(&h->cabac);
        mb_xy = i_mb_x + i_mb_y * h->mb.i_mb_width; /* current index of macro-block */

        if( i_mb_x == 0 ) /* if we are at the start mb of each row */
        {
            if( x264_bitstream_check_buffer( h ) )
                return -1;
            if( h->param.rc.i_vbv_buffer_size )
                x264_bitstream_backup( h, &bs_bak[1], i_skip, 1 );
            /* if we are not re-encode a macro-block, do deblocking and hpel filter for current row */
            if( !h->mb.b_reencode_mb )
                x264_fdec_filter_row( h, i_mb_y );
        }

        if( back_up_bitstream )
            x264_bitstream_backup( h, &bs_bak[0], i_skip, 0 );

        /* load cache of previous macroblocks */
        x264_macroblock_cache_load( h, i_mb_x, i_mb_y );

        /* analyse this macroblock. */
        x264_macroblock_analyse( h );

        /* encode this macroblock -> be careful it can change the mb type to P_SKIP if needed */
reencode:
        x264_macroblock_encode( h );

        if( h->param.b_cabac )
        {
            if( mb_xy > h->sh.i_first_mb )
                x264_cabac_encode_terminal( &h->cabac );

            if( IS_SKIP( h->mb.i_type ) )
                x264_cabac_mb_skip( h, 1 );
            else
            {
                if( h->sh.i_type != SLICE_TYPE_I )
                    x264_cabac_mb_skip( h, 0 );
                /* write cabac stream */
                x264_macroblock_write_cabac( h, &h->cabac );
            }
        }
        else
        {
            if( IS_SKIP( h->mb.i_type ) )
                i_skip++;
            else
            {
                if( h->sh.i_type != SLICE_TYPE_I )
                {
                    bs_write_ue( &h->out.bs, i_skip );  /* skip run */
                    i_skip = 0;
                }
                /* write cavlc stream */
                x264_macroblock_write_cavlc( h );
                /* If there was a CAVLC level code overflow, try again at a higher QP. */
                if( h->mb.b_overflow )
                {
                    h->mb.i_chroma_qp = h->chroma_qp_table[++h->mb.i_qp];
                    h->mb.i_skip_intra = 0;
                    h->mb.b_skip_mc = 0;
                    h->mb.b_overflow = 0;
                    x264_bitstream_restore( h, &bs_bak[0], &i_skip, 0 );
                    goto reencode;
                }
            }
        }

        total_bits = bs_pos(&h->out.bs) + x264_cabac_pos(&h->cabac);
        mb_size = total_bits - mb_spos;
        h->mb.b_reencode_mb = 0;

        /* save cache */
        x264_macroblock_cache_save( h );

        if( x264_ratecontrol_mb( h, mb_size ) < 0 )
        {
        	/* re-encode current mb line if rate control failed */
            x264_bitstream_restore( h, &bs_bak[1], &i_skip, 1 );
            h->mb.b_reencode_mb = 1;
            i_mb_x = 0;
            h->mb.i_mb_prev_xy = i_mb_y * h->mb.i_mb_stride - 1;
            h->sh.i_last_mb = orig_last_mb;
            continue;
        }

        /* accumulate mb stats */
#ifdef _DEBUG
        h->stat.frame.i_mb_count[h->mb.i_type]++;
        b_intra = IS_INTRA( h->mb.i_type );
        b_skip = IS_SKIP( h->mb.i_type );
        if( h->param.i_log_level >= X264_LOG_INFO )
        {
            if( !b_intra && !b_skip && !IS_DIRECT( h->mb.i_type ) )
            {
				int i, i_list;
                if( h->mb.i_partition != D_8x8 )
                        h->stat.frame.i_mb_partition[h->mb.i_partition] += 4;
                    else
                        for( i = 0; i < 4; i++ )
                            h->stat.frame.i_mb_partition[h->mb.i_sub_partition[i]] ++;
                if( h->param.i_frame_reference > 1 )
                    for( i_list = 0; i_list <= (h->sh.i_type == SLICE_TYPE_B); i_list++ )
                        for( i = 0; i < 4; i++ )
                        {
                            int i_ref = h->mb.cache.ref[i_list][ x264_scan8[4*i] ];
                            if( i_ref >= 0 )
                                h->stat.frame.i_mb_count_ref[i_list][i_ref] ++;
                        }
            }
        }

        if( h->param.i_log_level >= X264_LOG_INFO )
        {
            if( h->mb.i_cbp_luma | h->mb.i_cbp_chroma )
            {
				int cbpsum = (h->mb.i_cbp_luma&1) + ((h->mb.i_cbp_luma>>1)&1)
						   + ((h->mb.i_cbp_luma>>2)&1) + (h->mb.i_cbp_luma>>3);
				h->stat.frame.i_mb_cbp[!b_intra + 0] += cbpsum;
				h->stat.frame.i_mb_cbp[!b_intra + 2] += !!h->mb.i_cbp_chroma;
				h->stat.frame.i_mb_cbp[!b_intra + 4] += h->mb.i_cbp_chroma >> 1;
            }
            if( h->mb.i_cbp_luma && !b_intra )
            {
                h->stat.frame.i_mb_count_8x8dct[0] ++;
                h->stat.frame.i_mb_count_8x8dct[1] += h->mb.b_transform_8x8;
            }
            if( b_intra && h->mb.i_type != I_PCM )
            {
				int i;
                if( h->mb.i_type == I_16x16 )
                    h->stat.frame.i_mb_pred_mode[0][h->mb.i_intra16x16_pred_mode]++;
                else if( h->mb.i_type == I_8x8 )
                    for( i = 0; i < 16; i += 4 )
                        h->stat.frame.i_mb_pred_mode[1][h->mb.cache.intra4x4_pred_mode[x264_scan8[i]]]++;
                else //if( h->mb.i_type == I_4x4 )
                    for( i = 0; i < 16; i++ )
                        h->stat.frame.i_mb_pred_mode[2][h->mb.cache.intra4x4_pred_mode[x264_scan8[i]]]++;
                h->stat.frame.i_mb_pred_mode[3][x264_mb_chroma_pred_mode_fix[h->mb.i_chroma_pred_mode]]++;
            }
            h->stat.frame.i_mb_field[b_intra?0:b_skip?2:1] += MB_INTERLACED;
        }
#endif

        /* calculate deblock strength values (actual deblocking is done per-row along with hpel) */
        if( b_deblock )
            x264_macroblock_deblock_strength( h );

        /* check if we are at the end of slice */
        if( mb_xy == h->sh.i_last_mb )
            break;

        /* update i_mb_x and i_mb_y for the next mb */
        i_mb_x++;
        if( i_mb_x == h->mb.i_mb_width )
        {
            i_mb_y++;
            i_mb_x = 0;
        }
    } // end of while( 1 )

    h->out.nal[h->out.i_nal].i_last_mb = h->sh.i_last_mb;

    if( h->param.b_cabac )
    {
        x264_cabac_encode_flush( h, &h->cabac );
        h->out.bs.p = h->cabac.p;
    }
    else
    {
        if( i_skip > 0 )
            bs_write_ue( &h->out.bs, i_skip );  /* last skip run */
        /* rbsp_slice_trailing_bits */
        bs_rbsp_trailing( &h->out.bs );
        bs_flush( &h->out.bs );
    }
    if( x264_nal_end( h ) )
        return -1;

    if( h->sh.i_last_mb == (h->i_threadslice_end * h->mb.i_mb_width - 1) )
    {
        h->stat.frame.i_misc_bits = bs_pos( &h->out.bs )
                                  + (h->out.i_nal*NALU_OVERHEAD * 8)
                                  - h->stat.frame.i_tex_bits
                                  - h->stat.frame.i_mv_bits;
        x264_fdec_filter_row( h, h->i_threadslice_end );

        /* Free mb info after the last thread's done using it */
        if( h->fdec->mb_info_free && (!h->param.b_sliced_threads || h->i_thread_idx == (h->param.i_threads-1)) )
        {
            h->fdec->mb_info_free( h->fdec->mb_info );
            h->fdec->mb_info = NULL;
            h->fdec->mb_info_free = NULL;
        }
    }

    return 0;
}

static int x264_encoder_frame_end( x264_t *h, x264_nal_t **pp_nal, int *pi_nal, x264_picture_t *pic_out );

/****************************************************************************
 * x264_encoder_encode:
 *  XXX: i_poc   : is the poc of the current given picture
 *       i_frame : is the number of the frame being coded
 *  ex:  type frame poc
 *       I      0   2*0
 *       P      1   2*3
 *       B      2   2*1
 *       B      3   2*2
 *       P      4   2*6
 *       B      5   2*4
 *       B      6   2*5
 ****************************************************************************/
int     x264_encoder_encode( x264_t *h, x264_nal_t **pp_nal, int *pi_nal, x264_picture_t *pic_in, x264_picture_t *pic_out )
{
    int i_nal_type, i_nal_ref_idc, i_global_qp, i;
    int overhead = NALU_OVERHEAD;

    h->i_cpb_delay_pir_offset = h->i_cpb_delay_pir_offset_next;

    /* no data out */
    *pi_nal = 0;
    *pp_nal = NULL;

    /* ------------------- Setup new frame from picture -------------------- */
    if( pic_in != NULL )
    {
        /* 1: Copy the picture to a frame and move it to a buffer */
        x264_frame_t *fenc = x264_frame_pop_unused( h, 0 );
        if( !fenc )
            return -1;

        if( x264_frame_copy_picture( h, fenc, pic_in ) < 0 )
            return -1;

        if( h->param.i_width != (h->mb.i_mb_width<<4) ||
            h->param.i_height != (h->mb.i_mb_height<<4) )
            x264_frame_expand_border_mod16( h, fenc );

        fenc->i_frame = h->frames.i_input++;

        if( fenc->i_frame == 0 )
            h->frames.i_first_pts = fenc->i_pts;
        if( h->frames.i_bframe_delay && fenc->i_frame == h->frames.i_bframe_delay )
            h->frames.i_bframe_delay_time = fenc->i_pts - h->frames.i_first_pts;

        if( h->param.b_vfr_input && fenc->i_pts <= h->frames.i_largest_pts )
            x264_log( h, X264_LOG_WARNING, "non-strictly-monotonic PTS\n" );

        h->frames.i_second_largest_pts = h->frames.i_largest_pts;
        h->frames.i_largest_pts = fenc->i_pts;
        fenc->i_pic_struct = PIC_STRUCT_PROGRESSIVE;

        /********************************************************************
         * Initialization for adaptive quantization of current frame.       *
         * quant_offsets is default to NULL unless input picture specifies. *
         ********************************************************************/
        x264_stack_align( x264_adaptive_quant_frame, h, fenc, pic_in->prop.quant_offsets );

        if( pic_in->prop.quant_offsets_free )
            pic_in->prop.quant_offsets_free( pic_in->prop.quant_offsets );

        /* Initialization for low-resolution frame */
        if( h->frames.b_have_lowres )
            x264_frame_init_lowres( h, fenc );

        /* 2: Place the frame into the queue for its slice type decision */
        x264_lookahead_put_frame( h, fenc );

        if( h->frames.i_input <= h->frames.i_delay + 1 - h->i_thread_frames )
        {
            /* Nothing yet to encode, waiting for filling of buffers */
            pic_out->i_type = X264_TYPE_AUTO;
            return 0;
        }
    }
    else
    {
        /* signal kills for lookahead thread */
        h->lookahead->b_exit_thread = 1;
    }

    h->i_frame++;
    /* 3: The picture is analyzed in the lookahead */
    if( !h->frames.current[0] )
        x264_lookahead_get_frames( h );

    if( !h->frames.current[0] && x264_lookahead_is_empty( h ) )
        return x264_encoder_frame_end( h, pp_nal, pi_nal, pic_out );

    /* ------------------- Get frame to be encoded ------------------------- */
    /* 4: get picture to encode */
    h->fenc = x264_frame_shift( h->frames.current );

    if( h->i_frame == h->i_thread_frames - 1 )
        h->i_reordered_pts_delay = h->fenc->i_reordered_pts;

    // ok to call this before encoding any frames, since the initial values of fdec have b_kept_as_ref=0
    if( x264_reference_update( h ) )
        return -1;
    h->fdec->i_lines_completed = -1;

    if( !IS_X264_TYPE_I( h->fenc->i_type ) )
    {
        int valid_refs_left = 0;
        for( i = 0; h->frames.reference[i]; i++ )
            if( !h->frames.reference[i]->b_corrupt )
                valid_refs_left++;
        /* No valid reference frames left: force an IDR. */
        if( !valid_refs_left )
        {
            h->fenc->b_keyframe = 1;
            h->fenc->i_type = X264_TYPE_IDR;
        }
    }

    if( h->fenc->b_keyframe )
    {
        h->frames.i_last_keyframe = h->fenc->i_frame;
        if( h->fenc->i_type == X264_TYPE_IDR )
        {
            h->i_frame_num = 0;
            h->frames.i_last_idr = h->fenc->i_frame;
        }
    }
    h->sh.i_mmco_command_count =
    h->sh.i_mmco_remove_from_end = 0;
    h->b_ref_reorder[0] =
    h->b_ref_reorder[1] = 0;
    h->fdec->i_poc =
    h->fenc->i_poc = 2 * ( h->fenc->i_frame - X264_MAX( h->frames.i_last_idr, 0 ) );

    /* ------------------- Setup frame context ----------------------------- */
    /* 5: Init data dependent of frame type */
    if( h->fenc->i_type == X264_TYPE_IDR )
    {
        /* reset ref pictures */
        i_nal_type    = NAL_SLICE_IDR;
        i_nal_ref_idc = NAL_PRIORITY_HIGHEST;
        h->sh.i_type = SLICE_TYPE_I;
        x264_reference_reset( h );
        h->frames.i_poc_last_open_gop = -1;
    }
    else if( h->fenc->i_type == X264_TYPE_I )
    {
        i_nal_type    = NAL_SLICE;
        i_nal_ref_idc = NAL_PRIORITY_HIGH; /* Not completely true but for now it is (as all I/P are kept as ref)*/
        h->sh.i_type = SLICE_TYPE_I;
        x264_reference_hierarchy_reset( h );
        if( h->param.b_open_gop )
            h->frames.i_poc_last_open_gop = h->fenc->b_keyframe ? h->fenc->i_poc : -1;
    }
    else if( h->fenc->i_type == X264_TYPE_P )
    {
        i_nal_type    = NAL_SLICE;
        i_nal_ref_idc = NAL_PRIORITY_HIGH; /* Not completely true but for now it is (as all I/P are kept as ref)*/
        h->sh.i_type = SLICE_TYPE_P;
        x264_reference_hierarchy_reset( h );
        h->frames.i_poc_last_open_gop = -1;
    }
    else if( h->fenc->i_type == X264_TYPE_BREF )
    {
        i_nal_type    = NAL_SLICE;
        i_nal_ref_idc = h->param.i_bframe_pyramid == X264_B_PYRAMID_STRICT ? NAL_PRIORITY_LOW : NAL_PRIORITY_HIGH;
        h->sh.i_type = SLICE_TYPE_B;
        x264_reference_hierarchy_reset( h );
    }
    else    /* B frame */
    {
        i_nal_type    = NAL_SLICE;
        i_nal_ref_idc = NAL_PRIORITY_DISPOSABLE;
        h->sh.i_type = SLICE_TYPE_B;
    }

    h->fdec->i_type = h->fenc->i_type;
    h->fdec->i_frame = h->fenc->i_frame;
    h->fenc->b_kept_as_ref =
    h->fdec->b_kept_as_ref = i_nal_ref_idc != NAL_PRIORITY_DISPOSABLE && h->param.i_keyint_max > 1;

    h->fdec->mb_info = h->fenc->mb_info;
    h->fdec->mb_info_free = h->fenc->mb_info_free;
    h->fenc->mb_info = NULL;
    h->fenc->mb_info_free = NULL;

    h->fdec->i_pts = h->fenc->i_pts;
    if( h->frames.i_bframe_delay )
    {
        int64_t *prev_reordered_pts = h->frames.i_prev_reordered_pts;
        h->fdec->i_dts = h->i_frame > h->frames.i_bframe_delay
                       ? prev_reordered_pts[ (h->i_frame - h->frames.i_bframe_delay) % h->frames.i_bframe_delay ]
                       : h->fenc->i_reordered_pts - h->frames.i_bframe_delay_time;
        prev_reordered_pts[ h->i_frame % h->frames.i_bframe_delay ] = h->fenc->i_reordered_pts;
    }
    else
        h->fdec->i_dts = h->fenc->i_reordered_pts;
    if( h->fenc->i_type == X264_TYPE_IDR )
        h->i_last_idr_pts = h->fdec->i_pts;

    /* ------------------- Init                ----------------------------- */
    /* build ref list 0/1 */
    x264_reference_build_list( h, h->fdec->i_poc );

    /* ---------------------- Write the bitstream -------------------------- */
    /* Init bitstream context */
    bs_init( &h->out.bs, h->out.p_bitstream, h->out.i_bitstream );
    h->out.i_nal = 0;

    h->i_nal_type = i_nal_type;
    h->i_nal_ref_idc = i_nal_ref_idc;

    if( h->fenc->b_keyframe )
    {
        /* Write SPS, PPS and SEI version for the first frame */
        if( h->param.b_repeat_headers && h->fenc->i_frame == 0 )
        {
            /* generate sequence parameters */
            x264_nal_start( h, NAL_SPS, NAL_PRIORITY_HIGHEST );
            x264_sps_write( &h->out.bs, h->sps );
            if( x264_nal_end( h ) )
                return -1;
            overhead += h->out.nal[h->out.i_nal-1].i_payload + NALU_OVERHEAD;

            /* generate picture parameters */
            x264_nal_start( h, NAL_PPS, NAL_PRIORITY_HIGHEST );
            x264_pps_write( &h->out.bs, h->sps, h->pps );
            if( x264_nal_end( h ) )
                return -1;
            overhead += h->out.nal[h->out.i_nal-1].i_payload + NALU_OVERHEAD;

            /* identify ourself */
#if 0
            x264_nal_start( h, NAL_SEI, NAL_PRIORITY_DISPOSABLE );
            if( x264_sei_version_write( h, &h->out.bs ) )
                return -1;
            if( x264_nal_end( h ) )
                return -1;
            overhead += h->out.nal[h->out.i_nal-1].i_payload + NALU_OVERHEAD - (h->param.b_annexb && h->out.i_nal-1);
#endif
        }
    }

    if( h->fenc->b_keyframe && h->param.b_intra_refresh )
        h->i_cpb_delay_pir_offset_next = h->fenc->i_cpb_delay;

    /* Init the rate control */
    /* FIXME: Include slice header bit cost. */
    x264_ratecontrol_start( h, h->fenc->i_qpplus1, overhead*8 );
    i_global_qp = x264_ratecontrol_qp( h );

    pic_out->i_qpplus1 =
    h->fdec->i_qpplus1 = i_global_qp + 1;

    if( h->i_ref[0] )
        h->fdec->i_poc_l0ref0 = h->fref[0][0]->i_poc;

    /* ------------------------ Create slice header  ----------------------- */
    x264_slice_init( h, i_nal_type, i_global_qp );

    /*------------------------- Weights -------------------------------------*/
    {
    	int i_ref;

    	/* for now no analysis and set all weights to nothing */
    	for( i_ref = 0; i_ref < h->i_ref[0]; i_ref++ )
    		h->fenc->weighted[i_ref] = h->fref[0][i_ref]->filtered[0][0];

    	// FIXME: This only supports weighting of one reference frame
    	// and duplicates of that frame.
    	h->fenc->i_lines_weighted = 0;

    	for( i_ref = 0; i_ref < h->i_ref[0]; i_ref++ )
    		for( i = 0; i < 3; i++ )
    			h->sh.weight[i_ref][i].weightfn = NULL;
    }

    if( i_nal_ref_idc != NAL_PRIORITY_DISPOSABLE )
        h->i_frame_num++;

    /* Write frame */
    h->i_threadslice_start = 0;
    h->i_threadslice_end = h->mb.i_mb_height;

    if( x264_slice_write( h ) )
        return -1;

    return x264_encoder_frame_end( h, pp_nal, pi_nal, pic_out );
}

#ifdef _DEBUG
static double x264_psnr( double sqe, double size )
{
	/* PSNR estimation method: mse = plane_ssd / plane_size, range: [0-1] */
	double mse = sqe / (PIXEL_MAX*PIXEL_MAX * size);
    if( mse <= 0.0000000001 ) /* Max 100dB */
        return 100;

    return -10.0 * log10( mse );
}

static double x264_ssim( double ssim )
{
    double inv_ssim = 1 - ssim;
    if( inv_ssim <= 0.0000000001 ) /* Max 100dB */
        return 100;

    return -10.0 * log10( inv_ssim );
}
#endif

static int x264_encoder_frame_end( x264_t *h, x264_nal_t **pp_nal, int *pi_nal, x264_picture_t *pic_out )
{
	int i, frame_size;
	int filler = 0;
#ifdef _DEBUG
	int j, i_list;
    char psz_message[80];
    double dur;
#endif

    if( !h->out.i_nal )
    {
        pic_out->i_type = X264_TYPE_AUTO;
        return 0;
    }

    frame_size = x264_encoder_encapsulate_nals( h, 0 );
    if( frame_size < 0 )
        return -1;

    /* Set output picture properties */
    pic_out->i_type = h->fenc->i_type;

    pic_out->b_keyframe = h->fenc->b_keyframe;
    pic_out->i_pic_struct = h->fenc->i_pic_struct;

    pic_out->i_pts = h->fdec->i_pts;
    pic_out->i_dts = h->fdec->i_dts;

    if( pic_out->i_pts < pic_out->i_dts )
        x264_log( h, X264_LOG_WARNING, "invalid DTS: PTS is less than DTS\n" );

    pic_out->opaque = h->fenc->opaque;

    /* copy encoded plane data to output picture */
    pic_out->img.i_csp = h->fdec->i_csp;
    pic_out->img.i_plane = h->fdec->i_plane;
    for( i = 0; i < pic_out->img.i_plane; i++ )
    {
        pic_out->img.i_stride[i] = h->fdec->i_stride[i] * sizeof(pixel);
        pic_out->img.plane[i] = (uint8_t*)h->fdec->plane[i];
    }

    x264_frame_push_unused( h, h->fenc );

    /* ---------------------- Update encoder state ------------------------- */

    /* update rc */
    filler = 0;
    if( x264_ratecontrol_end( h, frame_size * 8, &filler ) < 0 )
        return -1;

    pic_out->hrd_timing = h->fenc->hrd_timing;
    pic_out->prop.f_crf_avg = h->fdec->f_crf_avg;

    /* End bitstream, set output  */
    *pi_nal = h->out.i_nal;
    *pp_nal = h->out.nal;
    h->out.i_nal = 0;

    /* ---------------------- Compute/Print statistics --------------------- */
    /* Slice stat */
    h->stat.i_frame_count[h->sh.i_type]++;
    h->stat.i_frame_size[h->sh.i_type] += frame_size;
    h->stat.f_frame_qp[h->sh.i_type] += h->fdec->f_qp_avg_aq;

    /* mbs stat */
#ifdef _DEBUG
    for( i = 0; i < X264_MBTYPE_MAX; i++ )
        h->stat.i_mb_count[h->sh.i_type][i] += h->stat.frame.i_mb_count[i];
    for( i = 0; i < X264_PARTTYPE_MAX; i++ )
        h->stat.i_mb_partition[h->sh.i_type][i] += h->stat.frame.i_mb_partition[i];
    for( i = 0; i < 2; i++ )
        h->stat.i_mb_count_8x8dct[i] += h->stat.frame.i_mb_count_8x8dct[i];
    for( i = 0; i < 6; i++ )
        h->stat.i_mb_cbp[i] += h->stat.frame.i_mb_cbp[i];
    for( i = 0; i < 4; i++ )
        for( j = 0; j < 13; j++ )
            h->stat.i_mb_pred_mode[i][j] += h->stat.frame.i_mb_pred_mode[i][j];
    if( h->sh.i_type != SLICE_TYPE_I )
        for( i_list = 0; i_list < 2; i_list++ )
            for( i = 0; i < X264_REF_MAX*2; i++ )
                h->stat.i_mb_count_ref[h->sh.i_type][i_list][i] += h->stat.frame.i_mb_count_ref[i_list][i];
    for( i = 0; i < 3; i++ )
        h->stat.i_mb_field[i] += h->stat.frame.i_mb_field[i];
#endif

#ifdef _DEBUG
    h->stat.i_consecutive_bframes[h->fenc->i_bframes]++;
    dur = h->fenc->f_duration;
    h->stat.f_frame_duration[h->sh.i_type] += dur;
#endif

#ifdef _DEBUG
    psz_message[0] = '\0';
    if( h->param.analyse.b_psnr )
    {
        int64_t ssd[3] =
        {
            h->stat.frame.i_ssd[0],
            h->stat.frame.i_ssd[1],
            h->stat.frame.i_ssd[2],
        };
        int luma_size = h->param.i_width * h->param.i_height;
        int chroma_size = CHROMA_SIZE( luma_size );
        pic_out->prop.f_psnr[0] = x264_psnr( ssd[0], luma_size );
        pic_out->prop.f_psnr[1] = x264_psnr( ssd[1], chroma_size );
        pic_out->prop.f_psnr[2] = x264_psnr( ssd[2], chroma_size );
        pic_out->prop.f_psnr_avg = x264_psnr( ssd[0] + ssd[1] + ssd[2], luma_size + chroma_size*2 );

        h->stat.f_ssd_global[h->sh.i_type]   += dur * (ssd[0] + ssd[1] + ssd[2]);
        h->stat.f_psnr_average[h->sh.i_type] += dur * pic_out->prop.f_psnr_avg;
        h->stat.f_psnr_mean_y[h->sh.i_type]  += dur * pic_out->prop.f_psnr[0];
        h->stat.f_psnr_mean_u[h->sh.i_type]  += dur * pic_out->prop.f_psnr[1];
        h->stat.f_psnr_mean_v[h->sh.i_type]  += dur * pic_out->prop.f_psnr[2];

        snprintf( psz_message, 80, " PSNR Y:%5.2f U:%5.2f V:%5.2f", pic_out->prop.f_psnr[0],
                                                                    pic_out->prop.f_psnr[1],
                                                                    pic_out->prop.f_psnr[2] );
    }

    if( h->param.analyse.b_ssim )
    {
        pic_out->prop.f_ssim = h->stat.frame.f_ssim / h->stat.frame.i_ssim_cnt;
        h->stat.f_ssim_mean_y[h->sh.i_type] += pic_out->prop.f_ssim * dur;
        snprintf( psz_message + strlen(psz_message), 80 - strlen(psz_message),
                  " SSIM Y:%.5f", pic_out->prop.f_ssim );
    }
    psz_message[79] = '\0';

    x264_log( h, X264_LOG_DEBUG,
                  "frame=%4d QP=%.2f NAL=%d Slice:%c Poc:%-3d I:%-4d P:%-4d SKIP:%-4d size=%d bytes%s\n",
              h->i_frame,
              h->fdec->f_qp_avg_aq,
              h->i_nal_ref_idc,
              h->sh.i_type == SLICE_TYPE_I ? 'I' : (h->sh.i_type == SLICE_TYPE_P ? 'P' : 'B' ),
              h->fdec->i_poc,
              h->stat.frame.i_mb_count_i,
              h->stat.frame.i_mb_count_p,
              h->stat.frame.i_mb_count_skip,
              frame_size,
              psz_message );
#endif

    /* Remove duplicates, must be done near the end as breaks h->fref0 array
     * by freeing some of its pointers. */
    for( i = 0; i < h->i_ref[0]; i++ )
        if( h->fref[0][i] && h->fref[0][i]->b_duplicate )
        {
            x264_frame_push_blank_unused( h, h->fref[0][i] );
            h->fref[0][i] = 0;
        }

    return frame_size;
}

/****************************************************************************
 * x264_encoder_close:
 ****************************************************************************/
void    x264_encoder_close  ( x264_t *h )
{
	int i, j;
#ifdef _DEBUG
	int i_list, i_type;
	int64_t i_mb_count_size[2][7] = {{0}};
	char buf[200];
    int64_t i_yuv_size = FRAME_SIZE( h->param.i_width * h->param.i_height );
#endif

    /* destroy lookahead module */
    x264_lookahead_delete( h );

    h->i_frame++;

    /* frame count, average qp and psnr for I/P/B */
    for( i = 0; i < 3; i++ )
    {
        static const uint8_t slice_order[] = { SLICE_TYPE_I, SLICE_TYPE_P, SLICE_TYPE_B };
        int i_slice = slice_order[i];
        if( h->stat.i_frame_count[i_slice] > 0 )
        {
            int i_count = h->stat.i_frame_count[i_slice];
#ifdef _DEBUG
            if( h->param.analyse.b_psnr )
            {
            	double dur =  h->stat.f_frame_duration[i_slice];
                x264_log( h, X264_LOG_INFO,
                          "frame %c:%-5d Avg QP:%5.2f  size:%6.0f  PSNR Mean Y:%5.2f U:%5.2f V:%5.2f Avg:%5.2f Global:%5.2f\n",
                          slice_type_to_char[i_slice],
                          i_count,
                          h->stat.f_frame_qp[i_slice] / i_count,
                          (double)h->stat.i_frame_size[i_slice] / i_count,
                          h->stat.f_psnr_mean_y[i_slice] / dur, h->stat.f_psnr_mean_u[i_slice] / dur, h->stat.f_psnr_mean_v[i_slice] / dur,
                          h->stat.f_psnr_average[i_slice] / dur,
                          x264_psnr( h->stat.f_ssd_global[i_slice], dur * i_yuv_size ) );
            }
            else
#endif
            {
                x264_log( h, X264_LOG_INFO,
                          "frame %c:%-5d Avg QP:%5.2f  size:%6.0f\n",
                          slice_type_to_char[i_slice],
                          i_count,
                          h->stat.f_frame_qp[i_slice] / i_count,
                          (double)h->stat.i_frame_size[i_slice] / i_count );
            }
        }
    }

#ifdef _DEBUG
    /* accumulate mbs by size (16x16, 16x8, 8x16, 8x8, 8x4, 4x8, 4x4) for P/B frames */
    for( i_type = 0; i_type < 2; i_type++ )
        for( i = 0; i < X264_PARTTYPE_MAX; i++ )
        {
            if( i == D_DIRECT_8x8 ) continue; /* direct is counted as its own type */
            i_mb_count_size[i_type][x264_mb_partition_pixel_table[i]] += h->stat.i_mb_partition[i_type][i];
        }

    /* MB types used */
    /* I frames: I16x16, I8x8, I4x4 */
    if( h->stat.i_frame_count[SLICE_TYPE_I] > 0 )
    {
        int64_t *i_mb_count = h->stat.i_mb_count[SLICE_TYPE_I];
        double i_count = h->stat.i_frame_count[SLICE_TYPE_I] * h->mb.i_mb_count / 100.0;
        x264_log( h, X264_LOG_INFO, "mb I  I16..4: %4.1f%% %4.1f%% %4.1f%%\n",
        		i_mb_count[I_16x16]/ i_count,
        		i_mb_count[I_8x8]  / i_count,
        		i_mb_count[I_4x4]  / i_count);
    }
    /* P frames: I16x16, I8x8, I4x4, P16x16, P16x8, P8x16, P8x8, P8x4, P4x8, P4x4, P_SKIP */
    if( h->stat.i_frame_count[SLICE_TYPE_P] > 0 )
    {
        int64_t *i_mb_count = h->stat.i_mb_count[SLICE_TYPE_P];
        double i_count = h->stat.i_frame_count[SLICE_TYPE_P] * h->mb.i_mb_count / 100.0;
        int64_t *i_mb_size = i_mb_count_size[SLICE_TYPE_P];
        x264_log( h, X264_LOG_INFO,
                  "mb P  I16..4: %4.1f%% %4.1f%% %4.1f%%  P16..4: %4.1f%% %4.1f%% %4.1f%% %4.1f%% %4.1f%%    skip:%4.1f%%\n",
                  i_mb_count[I_16x16]/ i_count,
                  i_mb_count[I_8x8]  / i_count,
                  i_mb_count[I_4x4]  / i_count,
                  i_mb_size[PIXEL_16x16] / (i_count*4),
                  (i_mb_size[PIXEL_16x8] + i_mb_size[PIXEL_8x16]) / (i_count*4),
                  i_mb_size[PIXEL_8x8] / (i_count*4),
                  (i_mb_size[PIXEL_8x4] + i_mb_size[PIXEL_4x8]) / (i_count*4),
                  i_mb_size[PIXEL_4x4] / (i_count*4),
                  i_mb_count[P_SKIP] / i_count );
    }
    /* B frames: I16x16, I8x8, I4x4, B16x16, B16x8, B8x16, B8x8, B_DIRECT, B_SKIP */
#endif

    x264_ratecontrol_summary( h );

#ifdef _DEBUG
    if( h->stat.i_frame_count[SLICE_TYPE_I] + h->stat.i_frame_count[SLICE_TYPE_P] + h->stat.i_frame_count[SLICE_TYPE_B] > 0 )
    {
#define SUM3(p) (p[SLICE_TYPE_I] + p[SLICE_TYPE_P] + p[SLICE_TYPE_B])
#define SUM3b(p,o) (p[SLICE_TYPE_I][o] + p[SLICE_TYPE_P][o] + p[SLICE_TYPE_B][o])
        int64_t i_i8x8 = SUM3b( h->stat.i_mb_count, I_8x8 );
        int64_t i_intra = i_i8x8 + SUM3b( h->stat.i_mb_count, I_4x4 )
                                 + SUM3b( h->stat.i_mb_count, I_16x16 );
        int64_t i_all_intra = i_intra + SUM3b( h->stat.i_mb_count, I_PCM);
        const int i_count = h->stat.i_frame_count[SLICE_TYPE_I] +
                            h->stat.i_frame_count[SLICE_TYPE_P] +
                            h->stat.i_frame_count[SLICE_TYPE_B];
        int64_t i_mb_count = (int64_t)i_count * h->mb.i_mb_count;
        const double duration = h->stat.f_frame_duration[SLICE_TYPE_I] +
                                h->stat.f_frame_duration[SLICE_TYPE_P] +
                                h->stat.f_frame_duration[SLICE_TYPE_B];
        float f_bitrate = SUM3(h->stat.i_frame_size) / duration / 125;
		int64_t fixed_pred_modes[4][9] = {{0}}; /* predict mode count for [i16x16, i8x8, i4x4, i8x8c] */
		int64_t sum_pred_modes[4] = {0};        /* predict mode sum for [i16x16, i8x8, i4x4, i8x8c] */

        /* cbp of y/u/v of all frames */
        buf[0] = 0;
        if( i_mb_count != i_all_intra )
            sprintf( buf, " inter: %.1f%% %.1f%% %.1f%%",
                     h->stat.i_mb_cbp[1] * 100.0 / ((i_mb_count - i_all_intra)*4),
                     h->stat.i_mb_cbp[3] * 100.0 / ((i_mb_count - i_all_intra)),
                     h->stat.i_mb_cbp[5] * 100.0 / ((i_mb_count - i_all_intra)) );
        x264_log( h, X264_LOG_INFO, "coded y,uvDC,uvAC intra: %.1f%% %.1f%% %.1f%%%s\n",
                  h->stat.i_mb_cbp[0] * 100.0 / (i_all_intra*4),
                  h->stat.i_mb_cbp[2] * 100.0 / (i_all_intra),
                  h->stat.i_mb_cbp[4] * 100.0 / (i_all_intra), buf );

        /* mbs of all predict mode (v,h,dc,p) for I16x16 */
        for( i = 0; i <= I_PRED_16x16_DC_128; i++ )
        {
            fixed_pred_modes[0][x264_mb_pred_mode16x16_fix[i]] += h->stat.i_mb_pred_mode[0][i];
            sum_pred_modes[0] += h->stat.i_mb_pred_mode[0][i];
        }
        if( sum_pred_modes[0] )
            x264_log( h, X264_LOG_INFO, "i16 v,h,dc,p: %2.0f%% %2.0f%% %2.0f%% %2.0f%%\n",
                      fixed_pred_modes[0][0] * 100.0 / sum_pred_modes[0],
                      fixed_pred_modes[0][1] * 100.0 / sum_pred_modes[0],
                      fixed_pred_modes[0][2] * 100.0 / sum_pred_modes[0],
                      fixed_pred_modes[0][3] * 100.0 / sum_pred_modes[0] );

        /* mbs of all predict mode (v,h,dc,ddl,ddr,vr,hd,vl,hu) for I8x8/I4x4 */
        for( i = 1; i <= 2; i++ )
        {
            for( j = 0; j <= I_PRED_4x4_DC_128; j++ )
            {
                fixed_pred_modes[i][x264_mb_pred_mode4x4_fix(j)] += h->stat.i_mb_pred_mode[i][j];
                sum_pred_modes[i] += h->stat.i_mb_pred_mode[i][j];
            }
            if( sum_pred_modes[i] )
                x264_log( h, X264_LOG_INFO, "i%d v,h,dc,ddl,ddr,vr,hd,vl,hu: %2.0f%% %2.0f%% %2.0f%% %2.0f%% %2.0f%% %2.0f%% %2.0f%% %2.0f%% %2.0f%%\n", (3-i)*4,
                          fixed_pred_modes[i][0] * 100.0 / sum_pred_modes[i],
                          fixed_pred_modes[i][1] * 100.0 / sum_pred_modes[i],
                          fixed_pred_modes[i][2] * 100.0 / sum_pred_modes[i],
                          fixed_pred_modes[i][3] * 100.0 / sum_pred_modes[i],
                          fixed_pred_modes[i][4] * 100.0 / sum_pred_modes[i],
                          fixed_pred_modes[i][5] * 100.0 / sum_pred_modes[i],
                          fixed_pred_modes[i][6] * 100.0 / sum_pred_modes[i],
                          fixed_pred_modes[i][7] * 100.0 / sum_pred_modes[i],
                          fixed_pred_modes[i][8] * 100.0 / sum_pred_modes[i] );
        }

        /* mbs of all predict mode (dc,h,v,p) for I8x8 CHROMA */
        for( i = 0; i <= I_PRED_CHROMA_DC_128; i++ )
        {
            fixed_pred_modes[3][x264_mb_chroma_pred_mode_fix[i]] += h->stat.i_mb_pred_mode[3][i];
            sum_pred_modes[3] += h->stat.i_mb_pred_mode[3][i];
        }
        if( sum_pred_modes[3] )
            x264_log( h, X264_LOG_INFO, "i8c dc,h,v,p: %2.0f%% %2.0f%% %2.0f%% %2.0f%%\n",
                      fixed_pred_modes[3][0] * 100.0 / sum_pred_modes[3],
                      fixed_pred_modes[3][1] * 100.0 / sum_pred_modes[3],
                      fixed_pred_modes[3][2] * 100.0 / sum_pred_modes[3],
                      fixed_pred_modes[3][3] * 100.0 / sum_pred_modes[3] );

        /* reference frames of List 0/1 for P/B */
        for( i_list = 0; i_list < 2; i_list++ )
		{
			int i_slice;
            for( i_slice = 0; i_slice < 2; i_slice++ )
            {
                char *p = buf;
                int64_t i_den = 0;
                int i_max = 0;
                for( i = 0; i < X264_REF_MAX*2; i++ )
                    if( h->stat.i_mb_count_ref[i_slice][i_list][i] )
                    {
                        i_den += h->stat.i_mb_count_ref[i_slice][i_list][i];
                        i_max = i;
                    }
                if( i_max == 0 )
                    continue;
                for( i = 0; i <= i_max; i++ )
                    p += sprintf( p, " %4.1f%%", 100. * h->stat.i_mb_count_ref[i_slice][i_list][i] / i_den );
                x264_log( h, X264_LOG_INFO, "ref %c L%d:%s\n", "PB"[i_slice], i_list, buf );
            }
		}

        /* global ssim, psnr and bitrate */
        if( h->param.analyse.b_ssim )
        {
            float ssim = SUM3( h->stat.f_ssim_mean_y ) / duration;
            x264_log( h, X264_LOG_INFO, "SSIM Mean Y:%.7f (%6.3fdb)\n", ssim, x264_ssim( ssim ) );
        }
        if( h->param.analyse.b_psnr )
        {
            x264_log( h, X264_LOG_INFO,
                      "PSNR Mean Y:%6.3f U:%6.3f V:%6.3f Avg:%6.3f Global:%6.3f kb/s:%.2f\n",
                      SUM3( h->stat.f_psnr_mean_y ) / duration,
                      SUM3( h->stat.f_psnr_mean_u ) / duration,
                      SUM3( h->stat.f_psnr_mean_v ) / duration,
                      SUM3( h->stat.f_psnr_average ) / duration,
                      x264_psnr( SUM3( h->stat.f_ssd_global ), duration * i_yuv_size ),
                      f_bitrate );
        }
        else
            x264_log( h, X264_LOG_INFO, "kb/s:%.2f\n", f_bitrate );
    }
#endif

    /* rc */
    x264_ratecontrol_delete( h );

    x264_cqm_delete( h );
    x264_free( h->nal_buffer );
    x264_analyse_free_costs( h );

    /* frames */
    x264_frame_delete_list( h->frames.unused[0] );
    x264_frame_delete_list( h->frames.unused[1] );
    x264_frame_delete_list( h->frames.current );
    x264_frame_delete_list( h->frames.blank_unused );

    for( i = 0; i < h->i_thread_frames; i++ )
        if( h->thread[i]->b_thread_active )
            for( j = 0; j < h->thread[i]->i_ref[0]; j++ )
                if( h->thread[i]->fref[0][j] && h->thread[i]->fref[0][j]->b_duplicate )
                    x264_frame_delete( h->thread[i]->fref[0][j] );

    if( h->param.i_lookahead_threads > 1 )
        for( i = 0; i < h->param.i_lookahead_threads; i++ )
            x264_free( h->lookahead_thread[i] );

    for( i = h->param.i_threads - 1; i >= 0; i-- )
    {
        x264_frame_t **frame;

        if( !h->param.b_sliced_threads || i == 0 )
        {
            for( frame = h->thread[i]->frames.reference; *frame; frame++ )
            {
                /*assert( (*frame)->i_reference_count > 0 );*/
                (*frame)->i_reference_count--;
                if( (*frame)->i_reference_count == 0 )
                    x264_frame_delete( *frame );
            }
            frame = &h->thread[i]->fdec;
            if( *frame )
            {
                /*assert( (*frame)->i_reference_count > 0 );*/
                (*frame)->i_reference_count--;
                if( (*frame)->i_reference_count == 0 )
                    x264_frame_delete( *frame );
            }
            x264_macroblock_cache_free( h->thread[i] );
        }
        x264_macroblock_thread_free( h->thread[i], 0 );
        x264_free( h->thread[i]->out.p_bitstream );
        x264_free( h->thread[i]->out.nal );
        x264_free( h->thread[i] );
    }
}
