/*****************************************************************************
 * common.c: misc common functions
 *****************************************************************************/

#include "common.h"

#include <stdarg.h>
#include <ctype.h>

const int x264_bit_depth = BIT_DEPTH;

const int x264_chroma_format = X264_CHROMA_FORMAT;

static void x264_log_default( void *, int, const char *, va_list );

/****************************************************************************
 * x264_param_default:
 ****************************************************************************/
void x264_param_default( x264_param_t *param )
{
    memset( param, 0, sizeof( x264_param_t ) );

    param->cpu = 0; /* disable CPU auto-detect */
    param->i_threads = X264_THREADS_AUTO;
    param->i_lookahead_threads = X264_THREADS_AUTO;
    param->b_deterministic = 0; /**** default: 1 => 0 ****/
    param->i_sync_lookahead = X264_SYNC_LOOKAHEAD_AUTO;

    /* Video properties */
    param->i_csp           = X264_CHROMA_FORMAT ? X264_CHROMA_FORMAT : X264_CSP_I420;
    param->i_width         = 0;
    param->i_height        = 0;
    param->vui.i_sar_width = 0;
    param->vui.i_sar_height= 0;
    param->vui.i_overscan  = 0;  /* undef */
    param->vui.i_vidformat = 5;  /* undef */
    param->vui.b_fullrange = -1; /* default depends on input */
    param->vui.i_colorprim = 2;  /* undef */
    param->vui.i_transfer  = 2;  /* undef */
    param->vui.i_colmatrix = -1; /* default depends on input */
    param->vui.i_chroma_loc= 0;  /* left center */
    param->i_fps_num       = 25;
    param->i_fps_den       = 1;
    param->i_level_idc     = -1;
    param->i_slice_max_size = 0;
    param->i_slice_max_mbs = 0;
    param->i_slice_count = 0;

    /* Encoder parameters */
    param->i_frame_reference = 1; /**** default: 3 => 1 ****/
    param->i_keyint_max = 50;     /**** default: 250 => 50 ****/
    param->i_keyint_min = X264_KEYINT_MIN_AUTO;
    param->i_bframe = 0;          /**** default: 3 => 0 ****/
    param->i_scenecut_threshold = 20; /**** default: 40 => 20 ****/
    param->i_bframe_adaptive = X264_B_ADAPT_FAST;
    param->i_bframe_bias = 0;
    param->i_bframe_pyramid = X264_B_PYRAMID_NORMAL;
    param->b_interlaced = 0;
    param->b_constrained_intra = 0;

    param->b_deblocking_filter = 1;
    param->i_deblocking_filter_alphac0 = 0;
    param->i_deblocking_filter_beta = 0;

    param->b_cabac = 1;
    param->i_cabac_init_idc = 0;

    /* rc */
    param->rc.i_rc_method = X264_RC_CRF;
    param->rc.i_bitrate = 0;
    param->rc.f_rate_tolerance = 1.0;
    param->rc.i_vbv_max_bitrate = 0;
    param->rc.i_vbv_buffer_size = 0;
    param->rc.f_vbv_buffer_init = 0.9;
    param->rc.i_qp_constant = 23 + QP_BD_OFFSET;
    param->rc.f_rf_constant = 28; /**** default: 23 => 28 ****/
    param->rc.i_qp_min = 0;
    param->rc.i_qp_max = QP_MAX;
    param->rc.i_qp_step = 4;
    param->rc.f_ip_factor = 1.4;
    param->rc.f_pb_factor = 1.3;
    param->rc.i_aq_mode = X264_AQ_NONE; /**** default: VARIANCE => NONE ****/
    param->rc.f_aq_strength = 1.0;
    param->rc.i_lookahead = 0; /**** default: 40 => 0 ****/

    param->rc.b_stat_write = 0;
    param->rc.psz_stat_out = NULL;
    param->rc.b_stat_read = 0;
    param->rc.psz_stat_in = NULL;
    param->rc.f_qcompress = 0.6;
    param->rc.f_qblur = 0.5;
    param->rc.f_complexity_blur = 20;
    param->rc.b_mb_tree = 0;    /**** default: 1 => 0 ****/

    /* Log */
    param->pf_log = x264_log_default;
    param->p_log_private = NULL;
#ifdef _DEBUG
    param->i_log_level = X264_LOG_DEBUG;
#else
    param->i_log_level = X264_LOG_INFO;
#endif

    /* analyse */
    param->analyse.intra = X264_ANALYSE_I4x4;
    param->analyse.inter = 0; /**** default: X264_ANALYSE_PSUB16x16 => 0 ****/
    param->analyse.i_direct_mv_pred = X264_DIRECT_PRED_SPATIAL;
    param->analyse.i_me_method = X264_ME_DIA; /**** default: HEX => DIA ****/
    param->analyse.f_psy_rd = 1.0;
    param->analyse.b_psy = 0;   /**** default: 1 => 0 ****/
    param->analyse.f_psy_trellis = 0;
    param->analyse.i_me_range = 16;
    param->analyse.i_subpel_refine = 1; /**** default: 7 => 1 ****/
    param->analyse.b_mixed_references = 0; /**** default: 1 => 0 ****/
    param->analyse.b_chroma_me = 0;   /**** default: 1 => 0 ****/
    param->analyse.i_mv_range_thread = -1;
    param->analyse.i_mv_range = -1; // set from level_idc
    param->analyse.i_chroma_qp_offset = 0;
    param->analyse.b_fast_pskip = 1;
    param->analyse.b_weighted_bipred = 1;
    param->analyse.i_weighted_pred = X264_WEIGHTP_NONE; /**** default: SMART => NONE ****/
    param->analyse.b_dct_decimate = 1;
    param->analyse.b_transform_8x8 = 0; /**** default: 1 => 0 ****/
    param->analyse.i_trellis = 0;       /**** default: 1 => 0 ****/
    param->analyse.i_luma_deadzone[0] = 21;
    param->analyse.i_luma_deadzone[1] = 11;
#ifdef _DEBUG
    param->analyse.b_psnr = 1;
#else
    param->analyse.b_psnr = 0;
#endif
    param->analyse.b_ssim = 0;

    /* cqm */
    param->i_cqm_preset = X264_CQM_FLAT;

    /* misc */
    param->b_repeat_headers = 1;
    param->b_annexb = 1;
    param->b_aud = 0;
    param->b_vfr_input = 0;        /**** default: 1 => 0 ****/
    param->i_nal_hrd = X264_NAL_HRD_NONE;
    param->b_tff = 1;
    param->b_pic_struct = 0;
    param->b_fake_interlaced = 0;
    param->i_frame_packing = -1;
}

/****************************************************************************
 * x264_log:
 ****************************************************************************/
void x264_log( x264_t *h, int i_level, const char *psz_fmt, ... )
{
    if( !h || i_level <= h->param.i_log_level )
    {
        va_list arg;
        va_start( arg, psz_fmt );
        if( !h )
            x264_log_default( NULL, i_level, psz_fmt, arg );
        else
            h->param.pf_log( h->param.p_log_private, i_level, psz_fmt, arg );
        va_end( arg );
    }
}

static void x264_log_default( void *p_unused, int i_level, const char *psz_fmt, va_list arg )
{
    char *psz_prefix;
    switch( i_level )
    {
        case X264_LOG_ERROR:
            psz_prefix = "error";
            break;
        case X264_LOG_WARNING:
            psz_prefix = "warning";
            break;
        case X264_LOG_INFO:
            psz_prefix = "info";
            break;
        case X264_LOG_DEBUG:
            psz_prefix = "debug";
            break;
        default:
            psz_prefix = "unknown";
            break;
    }
    printf( "x264 [%s]: ", psz_prefix );
    vprintf( psz_fmt, arg );
}

/****************************************************************************
 * x264_picture_init:
 ****************************************************************************/
void x264_picture_init( x264_picture_t *pic )
{
    memset( pic, 0, sizeof( x264_picture_t ) );
    pic->i_type = X264_TYPE_AUTO;
    pic->i_qpplus1 = X264_QP_AUTO;
    pic->i_pic_struct = PIC_STRUCT_AUTO;
}

/****************************************************************************
 * x264_picture_alloc:
 ****************************************************************************/
int x264_picture_alloc( x264_picture_t *pic, int i_csp, int i_width, int i_height )
{
    typedef struct
    {
        int planes;
        int width_fix8[3];
        int height_fix8[3];
    } x264_csp_tab_t;

    static const x264_csp_tab_t x264_csp_tab[] =
    {
		[X264_CSP_I420] = { 3, { 256*1, 256/2, 256/2 }, { 256*1, 256/2, 256/2 } },
		[X264_CSP_YV12] = { 3, { 256*1, 256/2, 256/2 }, { 256*1, 256/2, 256/2 } },
		[X264_CSP_NV12] = { 2, { 256*1, 256*1 },        { 256*1, 256/2 },       },
		[X264_CSP_I422] = { 3, { 256*1, 256/2, 256/2 }, { 256*1, 256*1, 256*1 } },
		[X264_CSP_YV16] = { 3, { 256*1, 256/2, 256/2 }, { 256*1, 256*1, 256*1 } },
		[X264_CSP_NV16] = { 2, { 256*1, 256*1 },        { 256*1, 256*1 },       },
		[X264_CSP_I444] = { 3, { 256*1, 256*1, 256*1 }, { 256*1, 256*1, 256*1 } },
		[X264_CSP_YV24] = { 3, { 256*1, 256*1, 256*1 }, { 256*1, 256*1, 256*1 } },
		[X264_CSP_BGR]  = { 1, { 256*3 },               { 256*1 },              },
		[X264_CSP_BGRA] = { 1, { 256*4 },               { 256*1 },              },
		[X264_CSP_RGB]  = { 1, { 256*3 },               { 256*1 },              },
    };

    int csp = i_csp & X264_CSP_MASK;
	int depth_factor = i_csp & X264_CSP_HIGH_DEPTH ? 2 : 1;
	int plane_offset[3] = {0};
	int frame_size = 0;
	int i;
    if( csp <= X264_CSP_NONE || csp >= X264_CSP_MAX )
        return -1;
    x264_picture_init( pic );
    pic->img.i_csp = i_csp;
    pic->img.i_plane = x264_csp_tab[csp].planes;
    for( i = 0; i < pic->img.i_plane; i++ )
    {
        int stride = (((int64_t)i_width * x264_csp_tab[csp].width_fix8[i]) >> 8) * depth_factor;
        int plane_size = (((int64_t)i_height * x264_csp_tab[csp].height_fix8[i]) >> 8) * stride;
        pic->img.i_stride[i] = stride;
        plane_offset[i] = frame_size;
        frame_size += plane_size;
    }
    pic->img.plane[0] = x264_malloc( frame_size );
    if( !pic->img.plane[0] )
        return -1;
    for( i = 1; i < pic->img.i_plane; i++ )
        pic->img.plane[i] = pic->img.plane[0] + plane_offset[i];
    return 0;
}

/****************************************************************************
 * x264_picture_clean:
 ****************************************************************************/
void x264_picture_clean( x264_picture_t *pic )
{
    x264_free( pic->img.plane[0] );

    /* just to be safe */
    memset( pic, 0, sizeof( x264_picture_t ) );
}

/****************************************************************************
 * x264_malloc:
 ****************************************************************************/
void *x264_malloc( int i_size )
{
    uint8_t *align_buf = NULL;
#if HAVE_MEM_ALIGN
    align_buf = memalign( 16, i_size );
#else
    uint8_t *buf = malloc( i_size + 15 + sizeof(void **) );
    if( buf )
    {
        align_buf = buf + 15 + sizeof(void **);
        align_buf -= (intptr_t) align_buf & 15;
        *( (void **) ( align_buf - sizeof(void **) ) ) = buf;
    }
#endif
    if( !align_buf )
        x264_log( NULL, X264_LOG_ERROR, "malloc of size %d failed\n", i_size );
    return align_buf;
}

/****************************************************************************
 * x264_free:
 ****************************************************************************/
void x264_free( void *p )
{
    if( p )
    {
#if HAVE_MEM_ALIGN
        free( p );
#else
        free( *( ( ( void **) p ) - 1 ) );
#endif
    }
}

/****************************************************************************
 * x264_param2string:
 ****************************************************************************/
char *x264_param2string( x264_param_t *p, int b_res )
{
    int len = 1000;
    char *buf, *s;
    buf = s = x264_malloc( len );
    if( !buf )
        return NULL;

    if( b_res )
    {
        s += sprintf( s, "%dx%d ", p->i_width, p->i_height );
        s += sprintf( s, "fps=%u/%u ", p->i_fps_num, p->i_fps_den );
        s += sprintf( s, "timebase=%u/%u ", p->i_timebase_num, p->i_timebase_den );
        s += sprintf( s, "bitdepth=%d ", BIT_DEPTH );
    }

    s += sprintf( s, "cabac=%d", p->b_cabac );
    s += sprintf( s, " ref=%d", p->i_frame_reference );
    s += sprintf( s, " deblock=%d:%d:%d", p->b_deblocking_filter,
                  p->i_deblocking_filter_alphac0, p->i_deblocking_filter_beta );
    s += sprintf( s, " analyse=%#x:%#x", p->analyse.intra, p->analyse.inter );
    s += sprintf( s, " me=%d", p->analyse.i_me_method );
    s += sprintf( s, " subme=%d", p->analyse.i_subpel_refine );
    s += sprintf( s, " psy=%d", p->analyse.b_psy );
    if( p->analyse.b_psy )
        s += sprintf( s, " psy_rd=%.2f:%.2f", p->analyse.f_psy_rd, p->analyse.f_psy_trellis );
    s += sprintf( s, " mixed_ref=%d", p->analyse.b_mixed_references );
    s += sprintf( s, " me_range=%d", p->analyse.i_me_range );
    s += sprintf( s, " chroma_me=%d", p->analyse.b_chroma_me );
    s += sprintf( s, " trellis=%d", p->analyse.i_trellis );
    s += sprintf( s, " 8x8dct=%d", p->analyse.b_transform_8x8 );
    s += sprintf( s, " cqm=%d", p->i_cqm_preset );
    s += sprintf( s, " deadzone=%d,%d", p->analyse.i_luma_deadzone[0], p->analyse.i_luma_deadzone[1] );
    s += sprintf( s, " fast_pskip=%d", p->analyse.b_fast_pskip );
    s += sprintf( s, " chroma_qp_offset=%d", p->analyse.i_chroma_qp_offset );
    s += sprintf( s, " threads=%d", p->i_threads );
    s += sprintf( s, " lookahead_threads=%d", p->i_lookahead_threads );
    s += sprintf( s, " sliced_threads=%d", p->b_sliced_threads );
    if( p->i_slice_count )
        s += sprintf( s, " slices=%d", p->i_slice_count );
    if( p->i_slice_max_size )
        s += sprintf( s, " slice_max_size=%d", p->i_slice_max_size );
    if( p->i_slice_max_mbs )
        s += sprintf( s, " slice_max_mbs=%d", p->i_slice_max_mbs );
    s += sprintf( s, " nr=%d", p->analyse.i_noise_reduction );
    s += sprintf( s, " decimate=%d", p->analyse.b_dct_decimate );
    s += sprintf( s, " interlaced=%s", p->b_interlaced ? p->b_tff ? "tff" : "bff" : p->b_fake_interlaced ? "fake" : "0" );
    s += sprintf( s, " bluray_compat=%d", p->b_bluray_compat );

    s += sprintf( s, " constrained_intra=%d", p->b_constrained_intra );

    s += sprintf( s, " bframes=%d", p->i_bframe );
    if( p->i_bframe )
    {
        s += sprintf( s, " b_pyramid=%d b_adapt=%d b_bias=%d direct=%d weightb=%d open_gop=%d",
                      p->i_bframe_pyramid, p->i_bframe_adaptive, p->i_bframe_bias,
                      p->analyse.i_direct_mv_pred, p->analyse.b_weighted_bipred, p->b_open_gop );
    }
    s += sprintf( s, " weightp=%d", p->analyse.i_weighted_pred > 0 ? p->analyse.i_weighted_pred : 0 );

    if( p->i_keyint_max == X264_KEYINT_MAX_INFINITE )
        s += sprintf( s, " keyint=infinite" );
    else
        s += sprintf( s, " keyint=%d", p->i_keyint_max );
    s += sprintf( s, " keyint_min=%d scenecut=%d intra_refresh=%d",
                  p->i_keyint_min, p->i_scenecut_threshold, p->b_intra_refresh );

    if( p->rc.b_mb_tree || p->rc.i_vbv_buffer_size )
        s += sprintf( s, " rc_lookahead=%d", p->rc.i_lookahead );

    s += sprintf( s, " rc=%s mbtree=%d", p->rc.i_rc_method == X264_RC_ABR ?
                               ( p->rc.b_stat_read ? "2pass" : p->rc.i_vbv_max_bitrate == p->rc.i_bitrate ? "cbr" : "abr" )
                               : p->rc.i_rc_method == X264_RC_CRF ? "crf" : "cqp", p->rc.b_mb_tree );
    if( p->rc.i_rc_method == X264_RC_ABR || p->rc.i_rc_method == X264_RC_CRF )
    {
        if( p->rc.i_rc_method == X264_RC_CRF )
            s += sprintf( s, " crf=%.1f", p->rc.f_rf_constant );
        else
            s += sprintf( s, " bitrate=%d ratetol=%.1f",
                          p->rc.i_bitrate, p->rc.f_rate_tolerance );
        s += sprintf( s, " qcomp=%.2f qpmin=%d qpmax=%d qpstep=%d",
                      p->rc.f_qcompress, p->rc.i_qp_min, p->rc.i_qp_max, p->rc.i_qp_step );
        if( p->rc.b_stat_read )
            s += sprintf( s, " cplxblur=%.1f qblur=%.1f",
                          p->rc.f_complexity_blur, p->rc.f_qblur );
        if( p->rc.i_vbv_buffer_size )
        {
            s += sprintf( s, " vbv_maxrate=%d vbv_bufsize=%d",
                          p->rc.i_vbv_max_bitrate, p->rc.i_vbv_buffer_size );
            if( p->rc.i_rc_method == X264_RC_CRF )
                s += sprintf( s, " crf_max=%.1f", p->rc.f_rf_constant_max );
        }
    }
    else if( p->rc.i_rc_method == X264_RC_CQP )
        s += sprintf( s, " qp=%d", p->rc.i_qp_constant );

    if( p->rc.i_vbv_buffer_size )
    	s += sprintf( s, " nal_hrd=%d", p->i_nal_hrd );
    if( p->crop_rect.i_left | p->crop_rect.i_top | p->crop_rect.i_right | p->crop_rect.i_bottom )
        s += sprintf( s, " crop_rect=%u,%u,%u,%u", p->crop_rect.i_left, p->crop_rect.i_top,
                                                   p->crop_rect.i_right, p->crop_rect.i_bottom );
    if( p->i_frame_packing >= 0 )
        s += sprintf( s, " frame-packing=%d", p->i_frame_packing );

    if( !(p->rc.i_rc_method == X264_RC_CQP && p->rc.i_qp_constant == 0) )
    {
        s += sprintf( s, " ip_ratio=%.2f", p->rc.f_ip_factor );
        if( p->i_bframe && !p->rc.b_mb_tree )
            s += sprintf( s, " pb_ratio=%.2f", p->rc.f_pb_factor );
        s += sprintf( s, " aq=%d", p->rc.i_aq_mode );
        if( p->rc.i_aq_mode )
            s += sprintf( s, ":%.2f", p->rc.f_aq_strength );
    }

    return buf;
}
