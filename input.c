/*****************************************************************************
 * input.c: encoder input modules
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "../libx264/common/x264.h"
#include "input.h"

typedef struct
{
    const char *name;
    int planes;
    float width[4];
    float height[4];
    int mod_width;
    int mod_height;
} x264_cli_csp_t;

static const x264_cli_csp_t x264_cli_csps[] = {
    [X264_CSP_I420] = { "i420", 3, { 1, .5, .5 }, { 1, .5, .5 }, 2, 2 },
    [X264_CSP_I422] = { "i422", 3, { 1, .5, .5 }, { 1,  1,  1 }, 2, 1 },
    [X264_CSP_I444] = { "i444", 3, { 1,  1,  1 }, { 1,  1,  1 }, 1, 1 },
    [X264_CSP_YV12] = { "yv12", 3, { 1, .5, .5 }, { 1, .5, .5 }, 2, 2 },
    [X264_CSP_YV16] = { "yv16", 3, { 1, .5, .5 }, { 1,  1,  1 }, 2, 1 },
    [X264_CSP_YV24] = { "yv24", 3, { 1,  1,  1 }, { 1,  1,  1 }, 1, 1 },
    [X264_CSP_NV12] = { "nv12", 2, { 1,  1 },     { 1, .5 },     2, 2 },
    [X264_CSP_NV16] = { "nv16", 2, { 1,  1 },     { 1,  1 },     2, 1 },
    [X264_CSP_BGR]  = { "bgr",  1, { 3 },         { 1 },         1, 1 },
    [X264_CSP_BGRA] = { "bgra", 1, { 4 },         { 1 },         1, 1 },
    [X264_CSP_RGB]  = { "rgb",  1, { 3 },         { 1 },         1, 1 },
};

static int x264_cli_csp_is_invalid( int csp )
{
    int csp_mask = csp & X264_CSP_MASK;
    return csp_mask <= X264_CSP_NONE || csp_mask >= X264_CSP_MAX;
}

static int x264_cli_csp_depth_factor( int csp )
{
    if( x264_cli_csp_is_invalid( csp ) )
        return 0;
    /* the csp has a depth of 8 or 16 bits per pixel component */
    return (csp & X264_CSP_HIGH_DEPTH) ? 2 : 1;
}

static uint64_t x264_cli_pic_plane_size( int csp, int width, int height, int plane )
{
	uint64_t size = (uint64_t)width * height;
    int csp_mask = csp & X264_CSP_MASK;
    if( x264_cli_csp_is_invalid( csp ) || plane < 0 || plane >= x264_cli_csps[csp_mask].planes )
        return 0;
    size *= x264_cli_csps[csp_mask].width[plane] * x264_cli_csps[csp_mask].height[plane];
    size *= x264_cli_csp_depth_factor( csp );
    return size;
}

static int x264_cli_pic_alloc( cli_pic_t *pic, int csp, int width, int height )
{
	int csp_mask, i;
    memset( pic, 0, sizeof(cli_pic_t) );
    csp_mask = csp & X264_CSP_MASK;
    if( x264_cli_csp_is_invalid( csp ) )
        pic->img.planes = 0;
    else
        pic->img.planes = x264_cli_csps[csp_mask].planes;
    pic->img.csp    = csp;
    pic->img.width  = width;
    pic->img.height = height;
    for( i = 0; i < pic->img.planes; i++ )
    {
         pic->img.plane[i] = malloc( x264_cli_pic_plane_size( csp, width, height, i ) );
         if( !pic->img.plane[i] )
             return -1;
         pic->img.stride[i] = width * x264_cli_csps[csp_mask].width[i] * x264_cli_csp_depth_factor( csp );
    }

    return 0;
}

static void x264_cli_pic_clean( cli_pic_t *pic )
{
	int i;
    for( i = 0; i < pic->img.planes; i++ )
        free( pic->img.plane[i] );
    memset( pic, 0, sizeof(cli_pic_t) );
}

static const x264_cli_csp_t *x264_cli_get_csp( int csp )
{
    if( x264_cli_csp_is_invalid( csp ) )
        return NULL;
    return x264_cli_csps + (csp&X264_CSP_MASK);
}

typedef struct
{
    FILE *fh;               /* input file handle */
    int next_frame;         /* next frame number */
    uint64_t plane_size[4]; /* plane size in pixels */
    uint64_t frame_size;    /* frame size in bytes*/
} input_hnd_t;

static int open_file( char *psz_filename, void **p_handle, video_info_t *info )
{
	int i;
	char *p;
	const x264_cli_csp_t *csp;
    input_hnd_t *h = calloc( 1, sizeof(input_hnd_t) );
    if( !h )
        return -1;

	/* try to parse the file name to retrieve width and height */
	for( p = psz_filename; *p; p++ )
		if( *p >= '0' && *p <= '9' && sscanf( p, "%dx%d", &info->width, &info->height ) == 2 )
			break;
	if (!info->width || !info->height)
		return -1;

	/* default color-space: I420 without high bit-depth */
	info->csp = X264_CSP_I420;
    info->num_frames  = 0;

    if( !strcmp( psz_filename, "-" ) )
        h->fh = stdin;
    else
        h->fh = fopen( psz_filename, "rb" );
    if( h->fh == NULL )
        return -1;

    csp = x264_cli_get_csp( info->csp );
    for( i = 0; i < csp->planes; i++ )
    {
        h->plane_size[i] = x264_cli_pic_plane_size( info->csp, info->width, info->height, i );
        h->frame_size += h->plane_size[i];
        /* x264_cli_pic_plane_size returns the size in bytes, we need the value in pixels from here on */
        h->plane_size[i] /= x264_cli_csp_depth_factor( info->csp );
    }

    if( h->frame_size > 0 )
    {
		uint64_t i_size;
        fseek( h->fh, 0, SEEK_END );
        i_size = ftell( h->fh );
        fseek( h->fh, 0, SEEK_SET );
        info->num_frames = i_size / h->frame_size;
    }

    *p_handle = h;
    return 0;
}

static int read_frame_internal( cli_pic_t *pic, input_hnd_t *h )
{
    int error = 0;
	int i;
    int pixel_depth = x264_cli_csp_depth_factor( pic->img.csp );
    for( i = 0; i < pic->img.planes && !error; i++ )
    {
        error |= fread( pic->img.plane[i], pixel_depth, h->plane_size[i], h->fh ) != h->plane_size[i];
    }
    return error;
}

static int read_frame( cli_pic_t *pic, void * handle, int i_frame )
{
    input_hnd_t *h = handle;

    if( i_frame > h->next_frame )
    	fseek( h->fh, i_frame * h->frame_size, SEEK_SET );

    if( read_frame_internal( pic, h ) )
        return -1;

    h->next_frame = i_frame+1;
    return 0;
}

static int close_file( void * handle )
{
    input_hnd_t *h = handle;
    if( !h || !h->fh )
        return 0;
    fclose( h->fh );
    free( h );
    return 0;
}

const cli_input_t cli_input = { open_file, x264_cli_pic_alloc, read_frame, NULL, x264_cli_pic_clean, close_file };
