/*****************************************************************************
 * frame.h: frame handling
 *****************************************************************************/

#ifndef X264_FRAME_H
#define X264_FRAME_H

/* number of pixels past the edge of the frame, for motion estimation/compensation */
#define PADH 32 /* padding of horizontal */
#define PADV 32 /* padding of vertical */

typedef struct x264_frame
{
    int     i_poc;       /* picture order count */
    int     i_delta_poc[2];
    int     i_type;      /* current frame type. IDR/I/P/B */
    int     i_qpplus1;   /* current frame QP plus one */
    int64_t i_pts;       /* present timestamp */
    int64_t i_dts;       /* decode timestamp */
    int64_t i_reordered_pts;
    int64_t i_duration;  /* in SPS time_scale units (i.e 2 * timebase units) used for vfr */
    float   f_duration;  /* in seconds */
    int64_t i_cpb_duration;
    int64_t i_cpb_delay; /* in SPS time_scale units (i.e 2 * timebase units) */
    int64_t i_dpb_output_delay;
    x264_param_t *param;

    int     i_frame;       /* Presentation frame number */
    int     i_coded;       /* Coded frame number */
    int64_t i_field_cnt;   /* Presentation field count */
    int     i_frame_num;   /* 7.4.3 frame_num */
    int     b_kept_as_ref; /* Kept as reference flag. Set 1 if (i_nal_ref_idc != NAL_PRIORITY_DISPOSABLE && h->param.i_keyint_max > 1) */
    int     i_pic_struct;
    int     b_keyframe;    /* key frame flag */
    uint8_t b_fdec;        /* whether uses as a decoded frame or as a encoding frame */
    uint8_t b_last_minigop_bframe; /* this frame is the last b in a sequence of bframes */
    uint8_t i_bframes;     /* number of bframes following this nonb in coded order */
    float   f_qp_avg_rc;   /* QPs as decided by ratecontrol */
    float   f_qp_avg_aq;   /* QPs as decided by AQ in addition to ratecontrol */
    float   f_crf_avg;     /* Average effective CRF for this frame */
    int     i_poc_l0ref0;  /* poc of first refframe in L0, used to check if direct temporal is possible */

    /* YUV buffer */
    int     i_csp;         /* color space. default: i420 */
    /********************************************************
     * plane count.                                         *
     * for I420/I422, set i_plane = 2.                      *
     * one for luma,  the other for chroma (uv interleaved) *
     * for I444, set i_plane = 3.                           *
     ********************************************************/
    int     i_plane;
    /* stride of luma/chroma in pixel. luma = chroma = width + 2 * PADH
     * e.g. (1280 + 2 * 32) = 1344 for 720P */
    int     i_stride[3];
    /* width of luma/chroma in pixel.
     * e.g. [0]:1280, [1]:640 for 720P */
    int     i_width[3];
    /* lines of luma/chroma in pixel.
     * e.g. [0]:720, [1]:360 for 720P */
    int     i_lines[3];
    /* stride for low resolution (half-size). width / 2 + 2 * PADH
     * e.g. (1280 / 2 + 2 * 32) = 704 for 720P */
    int     i_stride_lowres;
    /* width for low resolution (half-size).
     * e.g. 1280 / 2 = 640 for 720P */
    int     i_width_lowres;
    /* lines for low resolution (half-size).
     * e.g. 720 / 2 = 360 for 720P */
    int     i_lines_lowres;

    pixel *plane[3];         /* plane[0]: luma, plane[1]: chroma (uv interleaved) */
    pixel *filtered[3][4];   /* plane[0], H, V, HV  of all luma plane */
    pixel *lowres[4];        /* half-size copy of input frame: Orig, H, V, HV */
    /*uint16_t *integral;*/  /* integral plane data. used only in ESA */

    /**********************************************************************************************
     * for unrestricted mv we allocate more data than needed allocated data are stored in buffer. *
     *                                                                                            *
     *                   luma (full and 1/2 pixel)                chroma (uv interleaved)         *
     *     buffer[0] =>  **************************  buffer[1] => **************************      *
     *                   *          32            *               *          16            *      *
     *       plane[0] => * => YYYYYYYYYYYYYYYY    *   plane[1] => * => UVUVUVUVUVUVUVUV    *      *
     * filtered[0][0]    *    YYYYYYYYYYYYYYYY    *               * 32 UVUVUVUVUVUVUVUV 32 *      *
     *                   * 32 YYYYYYYYYYYYYYYY 32 *               *          16            *      *
     *                   *    YYYYYYYYYYYYYYYY    *               **************************      *
     *                   *          32            *                                               *
     *                   **************************                                               *
     *                   **************************                                               *
     *                   *          32            *                                               *
     * filtered[0][1] => * => YYYYYYYYYYYYYYYY    *                                               *
     *                   *    YYYYYYYYYYYYYYYY    *                                               *
     *    (1/2 H)        * 32 YYYYYYYYYYYYYYYY 32 *                                               *
     *                   *    YYYYYYYYYYYYYYYY    *                                               *
     *                   *          32            *                                               *
     *                   **************************                                               *
     *                   **************************                                               *
     *                   *          32            *                                               *
     * filtered[0][2] => * => YYYYYYYYYYYYYYYY    *                                               *
     *                   *    YYYYYYYYYYYYYYYY    *                                               *
     *    (1/2 V)        * 32 YYYYYYYYYYYYYYYY 32 *                                               *
     *                   *    YYYYYYYYYYYYYYYY    *                                               *
     *                   *          32            *                                               *
     *                   **************************                                               *
     *                   **************************                                               *
     *                   *          32            *                                               *
     * filtered[0][3] => * => YYYYYYYYYYYYYYYY    *                                               *
     *                   *    YYYYYYYYYYYYYYYY    *                                               *
     *    (1/2 HV)       * 32 YYYYYYYYYYYYYYYY 32 *                                               *
     *                   *    YYYYYYYYYYYYYYYY    *                                               *
     *                   *          32            *                                               *
     *                   **************************                                               *
     *                                                                                            *
     **********************************************************************************************/
    pixel *buffer[4];
    /* buffer for half-resolution */
    pixel *buffer_lowres[4];

    /* weighted frame */
    x264_weight_t weight[X264_REF_MAX][3]; /* [ref_index][plane] */
    pixel *weighted[X264_REF_MAX]; /* plane[0] weighted of the reference frames */
    int b_duplicate;               /* whether frame is a blank copy of real frame (including pointers) */
    struct x264_frame *orig;

    /* motion data, used only for reconstruction frame (fdec) */
    int8_t  *mb_type;      /* mb type table for all mbs of current frame */
    uint8_t *mb_partition; /* mb partition table for all mbs of current frame */
    int16_t (*mv[2])[2];   /* motion vectors for all mbs of current frame */
    int16_t (*mv16x16)[2];
    int16_t (*lowres_mvs[2][X264_BFRAME_MAX+1])[2];
    uint8_t *field;
    uint8_t *effective_qp; /* effective qp table for all mbs of current frame. used only when x264_param_t.analyse.b_mb_info enabled */

    /* Stored as (lists_used << LOWRES_COST_SHIFT) + (cost).
     * Doesn't need special addressing for intra cost because
     * lists_used is guaranteed to be zero in that cast. */
    /*
    uint16_t (*lowres_costs[X264_BFRAME_MAX+2][X264_BFRAME_MAX+2]);
    #define LOWRES_COST_MASK ((1<<14)-1)
    #define LOWRES_COST_SHIFT 14
    */

    int     *lowres_mv_costs[2][X264_BFRAME_MAX+1];
    int8_t  *ref[2];  /* reference frames of list 0/1 for all mbs of current frame */
    int     i_ref[2];
    int     ref_poc[2][X264_REF_MAX];
    int16_t inv_ref_poc[2]; // inverse values of ref0 poc to avoid divisions in temporal MV prediction

    /* for adaptive B-frame decision.
     * contains the SATD cost of the lowres frame encoded in various modes.
     */
    int     i_cost_est[X264_BFRAME_MAX+2][X264_BFRAME_MAX+2];
    int     i_cost_est_aq[X264_BFRAME_MAX+2][X264_BFRAME_MAX+2];
    int     i_satd;          /* the i_cost_est of the selected frame type */
    int     i_intra_mbs[X264_BFRAME_MAX+2];
    int     *i_row_satds[X264_BFRAME_MAX+2][X264_BFRAME_MAX+2];
    int     *i_row_satd;
    int     *i_row_bits;     /* accumulated encoded mb bits for each row */
    float   *f_row_qp;
    float   *f_row_qscale;
    float   *f_qp_offset;    /* qp offset for each mb. values are depending on ac energy of each mb and AQ strength. */
    float   *f_qp_offset_aq; /* qp offset for each mb. values are depending on ac energy of each mb and AQ strength. */
    int     b_intra_calculated;
    uint32_t i_pixel_sum[3]; /* sum of pixel value for y/u/v */
    uint64_t i_pixel_ssd[3]; /* sum of squared pixel value for y/u/v */

    /* hrd */
    x264_hrd_t hrd_timing;

    /* vbv */
    uint8_t i_planned_type[X264_LOOKAHEAD_MAX+1];
    int i_planned_satd[X264_LOOKAHEAD_MAX+1];
    double f_planned_cpb_duration[X264_LOOKAHEAD_MAX+1];
    int64_t i_coded_fields_lookahead;
    int64_t i_cpb_delay_lookahead;

    /* threading */
    int     i_lines_completed; /* in pixels */
    int     i_lines_weighted;  /* FIXME: this only supports weighting of one reference frame */
    int     i_reference_count; /* number of threads using this frame (not necessarily the number of pointers) */

    /* periodic intra refresh */
    float   f_pir_position;
    int     i_pir_start_col;
    int     i_pir_end_col;
    int     i_frames_since_pir;

    /* interactive encoder control */
    int     b_corrupt;

    /* user data */
    void *opaque;

    /* user frame properties. see comment of x264_image_properties_t from x264.h */
    uint8_t *mb_info; /* copy from x264_picture_t.prop.mb_info */
    void (*mb_info_free)( void* ); /* copy from x264_picture_t.prop.mb_info_free */
} x264_frame_t;

/* synchronized frame list */
typedef struct
{
   x264_frame_t **list;
   int i_max_size;
   int i_size;
} x264_sync_frame_list_t;

typedef void (*x264_deblock_inter_t)( pixel *pix, intptr_t stride, int alpha, int beta, int8_t *tc0 );
typedef void (*x264_deblock_intra_t)( pixel *pix, intptr_t stride, int alpha, int beta );
typedef struct
{
    x264_deblock_inter_t deblock_luma[2];
    x264_deblock_inter_t deblock_chroma[2];
    x264_deblock_intra_t deblock_luma_intra[2];
    x264_deblock_intra_t deblock_chroma_intra[2];
    void (*deblock_strength) ( uint8_t nnz[X264_SCAN8_SIZE], int8_t ref[2][X264_SCAN8_LUMA_SIZE],
                               int16_t mv[2][X264_SCAN8_LUMA_SIZE][2], uint8_t bs[2][8][4] );
} x264_deblock_function_t;

void          x264_frame_delete( x264_frame_t *frame );

int           x264_frame_copy_picture( x264_t *h, x264_frame_t *dst, x264_picture_t *src );

void          x264_frame_expand_border( x264_t *h, x264_frame_t *frame, int mb_y );
void          x264_frame_expand_border_filtered( x264_t *h, x264_frame_t *frame, int mb_y, int b_end );
void          x264_frame_expand_border_lowres( x264_frame_t *frame );
void          x264_frame_expand_border_mod16( x264_t *h, x264_frame_t *frame );

void          x264_frame_deblock_row( x264_t *h, int mb_y );

void          x264_frame_filter( x264_t *h, x264_frame_t *frame, int mb_y, int b_end );
void          x264_frame_init_lowres( x264_t *h, x264_frame_t *frame );

void          x264_deblock_init( int cpu, x264_deblock_function_t *pf );

void          x264_frame_push( x264_frame_t **list, x264_frame_t *frame );
x264_frame_t *x264_frame_pop( x264_frame_t **list );
void          x264_frame_unshift( x264_frame_t **list, x264_frame_t *frame );
x264_frame_t *x264_frame_shift( x264_frame_t **list );
void          x264_frame_push_unused( x264_t *h, x264_frame_t *frame );
void          x264_frame_push_blank_unused( x264_t *h, x264_frame_t *frame );
x264_frame_t *x264_frame_pop_blank_unused( x264_t *h );
x264_frame_t *x264_frame_pop_unused( x264_t *h, int b_fdec );
void          x264_frame_delete_list( x264_frame_t **list );

int           x264_sync_frame_list_init( x264_sync_frame_list_t *slist, int nelem );
void          x264_sync_frame_list_delete( x264_sync_frame_list_t *slist );
void          x264_sync_frame_list_push( x264_sync_frame_list_t *slist, x264_frame_t *frame );
x264_frame_t *x264_sync_frame_list_pop( x264_sync_frame_list_t *slist );

#endif
