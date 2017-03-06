/*****************************************************************************
 * input.h: encoder input modules
 *****************************************************************************/

#ifndef X264_INPUT_H
#define X264_INPUT_H

/* properties of the source given by the demuxer */
typedef struct
{
    int csp;          /* color space of the input. default: I420 */
    uint32_t fps_num; /* fps numerator. default: 25 */
    uint32_t fps_den; /* fps denominator. default: 1 */
    int fullrange;    /* has 2^bit_depth-1 instead of 219*2^(bit_depth-8) ranges (YUV only) */
    int width;        /* widht in pixels */
    int height;       /* height in pixels */
    int interlaced;   /* interlaced or progressive */
    int num_frames;   /* total frame number */
    uint32_t sar_width;    /* sar width. default: 0 */
    uint32_t sar_height;   /* sar height. default: 0 */
    int tff;               /*  */
    uint32_t timebase_num; /* timebase numerator */
    uint32_t timebase_den; /* timebase denominator */
    int vfr;               /*  */
} video_info_t;

/* image data type used by x264cli */
typedef struct
{
    int     csp;       /* color space. default: I420 */
    int     width;     /* width in pixels of the picture */
    int     height;    /* height in pixels of the picture */
    int     planes;    /* number of planes */
    uint8_t *plane[4]; /* pointers for each plane */
    int     stride[4]; /* strides in bytes for each plane */
} cli_image_t;

typedef struct
{
    cli_image_t img;
    int64_t pts;       /* input pts. default: 0 */
    int64_t duration;  /* frame duration - used for vfr */
    void    *opaque;   /* opaque handle */
} cli_pic_t;

typedef struct
{
    int (*open_file)( char *psz_filename, void **p_handle, video_info_t *info );
    int (*picture_alloc)( cli_pic_t *pic, int csp, int width, int height );
    int (*read_frame)( cli_pic_t *pic, void * handle, int i_frame );
    int (*release_frame)( cli_pic_t *pic, void * handle );
    void (*picture_clean)( cli_pic_t *pic );
    int (*close_file)( void * handle );
} cli_input_t;

extern const cli_input_t cli_input;

#endif
