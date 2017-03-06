/*****************************************************************************
 * output.h: encoder output modules
 *****************************************************************************/

#ifndef X264_OUTPUT_H
#define X264_OUTPUT_H

typedef struct
{
    int (*open_file)( char *psz_filename, void **p_handle );
    int (*write_headers)( void * handle, x264_nal_t *p_nal );
    int (*write_frame)( void * handle, uint8_t *p_nal, int i_size, x264_picture_t *p_picture );
    int (*close_file)( void * handle, int64_t largest_pts, int64_t second_largest_pts );
} cli_output_t;

extern const cli_output_t cli_output;

#endif
