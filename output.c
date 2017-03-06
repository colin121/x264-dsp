/*****************************************************************************
 * output.c: encoder output modules
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "../libx264/common/x264.h"
#include "output.h"

static int open_file( char *psz_filename, void **p_handle )
{
    if( !strcmp( psz_filename, "-" ) )
        *p_handle = stdout;
    else if( !(*p_handle = fopen( psz_filename, "w+b" )) )
        return -1;

    return 0;
}

static int write_headers( void * handle, x264_nal_t *p_nal )
{
    int size = p_nal[0].i_payload + p_nal[1].i_payload + p_nal[2].i_payload;

    if( fwrite( p_nal[0].p_payload, size, 1, (FILE*)handle ) )
        return size;
    return -1;
}

static int write_frame( void * handle, uint8_t *p_nalu, int i_size, x264_picture_t *p_picture )
{
    if( fwrite( p_nalu, i_size, 1, (FILE*)handle ) )
        return i_size;
    return -1;
}

static int close_file( void * handle, int64_t largest_pts, int64_t second_largest_pts )
{
    if( !handle || handle == stdout )
        return 0;

    return fclose( (FILE*)handle );
}

const cli_output_t cli_output = { open_file, write_headers, write_frame, close_file };
