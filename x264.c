#include <stdio.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include "../libx264/common/x264.h"
#include "input.h"
#include "output.h"

/* In microseconds */
#define UPDATE_INTERVAL 1000000

typedef struct {
    int b_progress; /* whether print encoding progress to console */
    int i_seek;     /* seek position of start encoding */
    void * hin;     /* input handle */
    void * hout;    /* output handle */
} cli_opt_t;

/* logging and printing for within the cli system */
static int cli_log_level;
void x264_cli_log( const char *name, int i_level, const char *fmt, ... )
{
	char *s_level;
	va_list arg;
    if( i_level > cli_log_level )
        return;
    switch( i_level )
    {
        case X264_LOG_ERROR:
            s_level = "error";
            break;
        case X264_LOG_WARNING:
            s_level = "warning";
            break;
        case X264_LOG_INFO:
            s_level = "info";
            break;
        case X264_LOG_DEBUG:
            s_level = "debug";
            break;
        default:
            s_level = "unknown";
            break;
    }
    printf( "%s [%s]: ", name, s_level );
    va_start( arg, fmt );
    vprintf( fmt, arg );
    va_end( arg );
}

#define FAIL_IF_ERROR( cond, ... )\
if( cond )\
{\
	x264_cli_log( "x264", X264_LOG_ERROR, __VA_ARGS__ );\
	return -1;\
}

#define FAIL_IF_ERROR2( cond, ... )\
if( cond )\
{\
    x264_cli_log( "x264", X264_LOG_ERROR, __VA_ARGS__ );\
    retval = -1;\
    goto fail;\
}

static int  parse( int argc, char **argv, x264_param_t *param, cli_opt_t *opt );
static int  encode( x264_param_t *param, cli_opt_t *opt );

int main( int argc, char **argv )
{
    x264_param_t param;
    cli_opt_t opt = {0};
    int ret = 0;

    /* Parse command line */
    if( parse( argc, argv, &param, &opt ) < 0 )
        ret = -1;

    if( !ret )
        ret = encode( &param, &opt );

    /* clean up handles */
    if( opt.hin )
        cli_input.close_file( opt.hin );
    if( opt.hout )
        cli_output.close_file( opt.hout, 0, 0 );

    return ret;
}

static int parse( int argc, char **argv, x264_param_t *param, cli_opt_t *opt )
{
    char *input_filename = "704x576.yuv";
    char *output_filename = "out.264";
	video_info_t info = {0};

	opt->b_progress = 1;
	if ( argc > 1 )
		input_filename = argv[1];
    if ( argc > 2 )
    	output_filename = argv[2];

    x264_param_default( param );
    cli_log_level = param->i_log_level;

    /* set info flags to be overwritten by demuxer as necessary. */
    info.csp        = param->i_csp;
    info.fps_num    = param->i_fps_num;
    info.fps_den    = param->i_fps_den;
    info.fullrange  = param->vui.b_fullrange;
    info.interlaced = param->b_interlaced;
    info.sar_width  = param->vui.i_sar_width;
    info.sar_height = param->vui.i_sar_height;
    info.tff        = param->b_tff;
    info.vfr        = param->b_vfr_input;

    FAIL_IF_ERROR( cli_output.open_file( output_filename, &opt->hout ),
    		"could not open output file `%s'\n", output_filename )

    FAIL_IF_ERROR( cli_input.open_file( input_filename, &opt->hin, &info ),
    		"could not open input file `%s'\n", input_filename )

    x264_cli_log( "input", X264_LOG_INFO, "%dx%d%c %u:%u @ %u/%u fps (%cfr)\n", info.width,
                  info.height, info.interlaced ? 'i' : 'p', info.sar_width, info.sar_height,
                  info.fps_num, info.fps_den, info.vfr ? 'v' : 'c' );

    /* force end result resolution */
    if( !param->i_width && !param->i_height )
    {
        param->i_height = info.height;
        param->i_width  = info.width;
    }

    info.num_frames = ((info.num_frames - opt->i_seek) > 0) ? (info.num_frames - opt->i_seek) : 0;
    if( (!info.num_frames || param->i_frame_total < info.num_frames) && param->i_frame_total > 0 )
        info.num_frames = param->i_frame_total;
    param->i_frame_total = info.num_frames;

    return 0;
}

static int encode_frame( x264_t *h, void * hout, x264_picture_t *pic, int64_t *last_dts )
{
    x264_picture_t pic_out;
    x264_nal_t *nal;
    int i_nal;
    int i_frame_size = 0;

    i_frame_size = x264_encoder_encode( h, &nal, &i_nal, pic, &pic_out );

    FAIL_IF_ERROR( i_frame_size < 0, "x264_encoder_encode failed\n" );

    if( i_frame_size )
    {
        i_frame_size = cli_output.write_frame( hout, nal[0].p_payload, i_frame_size, &pic_out );
        *last_dts = pic_out.i_dts;
    }

    return i_frame_size;
}

static int64_t print_status( int64_t i_start, int64_t i_previous, int i_frame, int i_frame_total, int64_t i_file, x264_param_t *param, int64_t last_ts )
{
    int64_t i_time = time(NULL) * 1000000;
	int64_t i_elapsed;
	double fps, bitrate;

    if( i_previous && i_time - i_previous < UPDATE_INTERVAL )
        return i_previous;

    i_elapsed = i_time - i_start;
    fps = i_elapsed > 0 ? i_frame * 1000000. / i_elapsed : 0;
    if( last_ts )
        bitrate = (double) i_file * 8 / ( (double) last_ts * 1000 * param->i_timebase_num / param->i_timebase_den );
    else
        bitrate = (double) i_file * 8 / ( (double) 1000 * param->i_fps_den / param->i_fps_num );
    if( i_frame_total )
    {
        int eta = i_elapsed * (i_frame_total - i_frame) / ((int64_t)i_frame * 1000000);
        printf( "x264 [%.1f%%] %d/%d frames, %.2f fps, %.2f kb/s, eta %d:%02d:%02d\n",
                 100. * i_frame / i_frame_total, i_frame, i_frame_total, fps, bitrate,
                 eta/3600, (eta/60)%60, eta%60 );
    }
    else
    {
        printf( "x264 %d frames: %.2f fps, %.2f kb/s\n", i_frame, fps, bitrate );
    }

    return i_time;
}

static int encode( x264_param_t *param, cli_opt_t *opt )
{
    x264_t *h = NULL;
    x264_picture_t pic;
    cli_pic_t cli_pic;

    int     i_frame = 0;
    int     i_frame_output = 0;
    int64_t i_end, i_previous = 0, i_start = 0;
    int64_t i_file = 0;
    int     i_frame_size;
    int64_t last_dts = 0;
    int64_t prev_dts = 0;
    int64_t first_dts = 0;
#   define  MAX_PTS_WARNING 3 /* arbitrary */
    int     pts_warning_cnt = 0;
    int64_t largest_pts = -1;
    int64_t second_largest_pts = -1;
    int64_t ticks_per_frame;
    double  duration;
    int     retval = 0;

    opt->b_progress &= param->i_log_level < X264_LOG_DEBUG;

    h = x264_encoder_open( param );
    FAIL_IF_ERROR2( !h, "x264_encoder_open failed\n" );

    x264_encoder_parameters( h, param );

    i_start = time(NULL) * 1000000;

    /* ticks/frame = ticks/second / frames/second */
    ticks_per_frame = (int64_t)param->i_timebase_den * param->i_fps_den / param->i_timebase_num / param->i_fps_num;
    FAIL_IF_ERROR2( ticks_per_frame < 1 && !param->b_vfr_input, "ticks_per_frame invalid: %lld\n", ticks_per_frame )
    ticks_per_frame = (ticks_per_frame > 1) ? ticks_per_frame : 1;

    if( !param->b_repeat_headers )
    {
        // Write SPS/PPS/SEI
        x264_nal_t *headers;
        int i_nal;

        FAIL_IF_ERROR2( x264_encoder_headers( h, &headers, &i_nal ) < 0, "x264_encoder_headers failed\n" )
        FAIL_IF_ERROR2( (i_file = cli_output.write_headers( opt->hout, headers )) < 0, "error writing headers to output file\n" );
    }

	/* Alloc picture for encoding frame */
    FAIL_IF_ERROR2( cli_input.picture_alloc( &cli_pic, param->i_csp, param->i_width, param->i_height ), "can't alloc picture\n" );

    /* Encode frames */
    for( ; (i_frame < param->i_frame_total || !param->i_frame_total); i_frame++ )
    {
    	if ( cli_input.read_frame( &cli_pic, opt->hin, i_frame + opt->i_seek ) )
    		break;

        x264_picture_init( &pic );
        memcpy( pic.img.i_stride, cli_pic.img.stride, sizeof(cli_pic.img.stride) );
        memcpy( pic.img.plane, cli_pic.img.plane, sizeof(cli_pic.img.plane) );
        pic.img.i_plane = cli_pic.img.planes;
        pic.img.i_csp = cli_pic.img.csp;
        pic.i_pts = cli_pic.pts;

        if( !param->b_vfr_input )
            pic.i_pts = i_frame;

        if( pic.i_pts <= largest_pts )
        {
            if( cli_log_level >= X264_LOG_DEBUG || pts_warning_cnt < MAX_PTS_WARNING )
                x264_cli_log( "x264", X264_LOG_WARNING, "non-strictly-monotonic pts at frame %d (%lld <= %lld)\n", i_frame, pic.i_pts, largest_pts );
            else if( pts_warning_cnt == MAX_PTS_WARNING )
                x264_cli_log( "x264", X264_LOG_WARNING, "too many non-monotonic pts warnings, suppressing further ones\n" );
            pts_warning_cnt++;
            pic.i_pts = largest_pts + ticks_per_frame;
        }

        second_largest_pts = largest_pts;
        largest_pts = pic.i_pts;

        prev_dts = last_dts;
        i_frame_size = encode_frame( h, opt->hout, &pic, &last_dts );
        if( i_frame_size < 0 )
        {
            retval = -1;
        }
        else if( i_frame_size )
        {
            i_file += i_frame_size;
            i_frame_output++;
            if( i_frame_output == 1 )
                first_dts = prev_dts = last_dts;
        }

        if( cli_input.release_frame && cli_input.release_frame(&cli_pic, opt->hin ) )
        	break;

        /* update status line (up to 1000 times per input file) */
        if( opt->b_progress && i_frame_output )
            i_previous = print_status( i_start, i_previous, i_frame_output, param->i_frame_total, i_file, param, 2 * last_dts - prev_dts - first_dts );
    }

    cli_input.picture_clean(&cli_pic);

fail:
    if( pts_warning_cnt >= MAX_PTS_WARNING && cli_log_level < X264_LOG_DEBUG )
        x264_cli_log( "x264", X264_LOG_WARNING, "%d suppressed non-monotonic pts warnings\n", pts_warning_cnt-MAX_PTS_WARNING );

    /* duration algorithm fails when only 1 frame is output */
    if( i_frame_output == 1 )
        duration = (double)param->i_fps_den / param->i_fps_num;
    else
        duration = (double)(2 * largest_pts - second_largest_pts) * param->i_timebase_num / param->i_timebase_den;

    i_end = time(NULL) * 1000000;
    if( h )
        x264_encoder_close( h );

    cli_output.close_file( opt->hout, largest_pts, second_largest_pts );
    opt->hout = NULL;

    if( i_frame_output > 0 )
    {
        double fps = (double)i_frame_output * (double)1000000 / (double)( i_end - i_start );
        printf("encoded %d frames, %.2f fps, %.2f kb/s\n", i_frame_output, fps, (double) i_file * 8 / ( 1000 * duration ) );
    }

    return retval;
}
