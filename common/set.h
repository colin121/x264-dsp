/*****************************************************************************
 * set.h: quantization init
 *****************************************************************************/

#ifndef X264_SET_H
#define X264_SET_H

enum profile_e
{
	/* Baseline Profile (BP)
	 * x264 implements as Constrained Baseline Profile (CBP), which corresponds to
	 * the subset of features that are in common between the Baseline, Main, and High Profiles.
	 * CBP was defined in 2009, these two profiles share the same profile identifier code value.
	 */
    PROFILE_BASELINE = 66,
    /* Main Profile (MP)
     * This profile is used for standard-definition digital TV broadcasts that use the MPEG-4
     * format as defined in the DVB standard. It is not, however, used for high-definition
     * television broadcasts, as the importance of this profile faded when the High Profile
     * was developed in 2004 for that application.
     */
    PROFILE_MAIN     = 77,
    /* High Profile (HiP)
     * The primary profile for broadcast and disc storage applications, particularly for high-definition
     * television applications (for example, this is the profile adopted by the Blu-ray Disc storage
     * format and the DVB HDTV broadcast service).
     */
    PROFILE_HIGH    = 100,
    /* High 10 Profile (Hi10P)
     * this profile builds on top of the High Profile,
     * adding support for up to 10 bits per sample of decoded picture precision.
     */
    PROFILE_HIGH10  = 110,
    /* High 4:2:2 Profile (Hi422P)
     * Primarily targeting professional applications that use interlaced video, this profile builds on top of
     * the High 10 Profile, adding support for the 4:2:2 chroma subsampling format while using up to 10 bits
     * per sample of decoded picture precision.
     */
    PROFILE_HIGH422 = 122,
    /* High 4:4:4 Predictive Profile (Hi444PP)
     * This profile builds on top of the High 4:2:2 Profile, supporting up to 4:4:4 chroma sampling, up to 14 bits
     * per sample, and additionally supporting efficient lossless region coding and the coding of each picture as
     * three separate color planes.
     * For camcorders, editing, and professional applications, the standard contains four additional Intra-frame-only
     * profiles, which are defined as simple subsets of other corresponding profiles. These are mostly for professional
     * (e.g., camera and editing system) applications.
     */
    PROFILE_HIGH444_PREDICTIVE = 244,
};

enum chroma_format_e
{
    CHROMA_400 = 0,
    CHROMA_420 = 1,
    CHROMA_422 = 2,
    CHROMA_444 = 3,
};

enum cqm4_e
{
    CQM_4IY = 0, /* Intra_4x4_LUMA */
    CQM_4PY = 1, /* Inter_4x4_LUMA */
    CQM_4IC = 2, /* Intra_4x4_CHROMA */
    CQM_4PC = 3  /* Inter_4x4_CHROMA */
};
enum cqm8_e
{
    CQM_8IY = 0, /* Intra_8x8_LUMA */
    CQM_8PY = 1, /* Inter_8x8_LUMA */
    CQM_8IC = 2, /* Intra_8x8_CHROMA */
    CQM_8PC = 3, /* Inter_8x8_CHROMA */
};

typedef struct
{
	/* SPS id, range: [0, 31] */
    int i_id;

    /* profile indication */
    int i_profile_idc;
    /* level indication */
    int i_level_idc;

    /* profile compatibility */
    int b_constraint_set0;
    int b_constraint_set1;
    int b_constraint_set2;
    int b_constraint_set3;

    /* maximum frame number */
    int i_log2_max_frame_num;

    /* picture-order-count type */
    int i_poc_type;
    /* poc 0 */
    int i_log2_max_poc_lsb;

    /* maximum reference frames */
    int i_num_ref_frames;
    int b_gaps_in_frame_num_value_allowed;
    /* macro-blocks in width */
    int i_mb_width;
    /* macro-blocks in height */
    int i_mb_height;
    /* frame/filed encoding mode flag */
    int b_frame_mbs_only;
    int b_mb_adaptive_frame_field;
    /* predict mode of direct or skip in B-slice */
    int b_direct8x8_inference;

    /* crop flag for video output */
    int b_crop;
    struct
    {
        int i_left;
        int i_right;
        int i_top;
        int i_bottom;
    } crop;

    /* video usability information flag */
    int b_vui;
    struct
    {
        int b_aspect_ratio_info_present;
        int i_sar_width;
        int i_sar_height;

        int b_overscan_info_present;
        int b_overscan_info;

        int b_signal_type_present;
        int i_vidformat;
        int b_fullrange;
        int b_color_description_present;
        int i_colorprim;
        int i_transfer;
        int i_colmatrix;

        int b_chroma_loc_info_present;
        int i_chroma_loc_top;
        int i_chroma_loc_bottom;

        int b_timing_info_present;    /* whether present time base in sps. default: on */
        uint32_t i_num_units_in_tick;
        uint32_t i_time_scale;
        int b_fixed_frame_rate;

        int b_nal_hrd_parameters_present; /* we don't use NAL HRD unless user indicated (param->i_nal_hrd). default: off */
        int b_vcl_hrd_parameters_present; /* we don't support VCL HRD. default: off */

        struct
        {
            int i_cpb_cnt;
            int i_bit_rate_scale;
            int i_cpb_size_scale;
            int i_bit_rate_value;
            int i_cpb_size_value;
            int i_bit_rate_unscaled;
            int i_cpb_size_unscaled;
            int b_cbr_hrd; /* CBR HRD. Set 1 only if (h->param.i_nal_hrd == X264_NAL_HRD_CBR). default: off */

            int i_initial_cpb_removal_delay_length;
            int i_cpb_removal_delay_length;
            int i_dpb_output_delay_length;
            int i_time_offset_length;
        } hrd;

        int b_pic_struct_present; /* we don't present picture struct unless user indicated (param->b_pic_struct). default: off  */
        int b_bitstream_restriction;
        int b_motion_vectors_over_pic_boundaries;
        int i_max_bytes_per_pic_denom;
        int i_max_bits_per_mb_denom;
        int i_log2_max_mv_length_horizontal;
        int i_log2_max_mv_length_vertical;
        int i_num_reorder_frames;
        int i_max_dec_frame_buffering;

        /* FIXME to complete */
    } vui;

    /* Set 1 in lossless of High profile. Ignored in Baseline/Main profile. */
    int b_qpprime_y_zero_transform_bypass;
    /* Chroma format indication, default: 1 (4:2:0 chroma format). Ignored in Baseline/Main profile. */
    int i_chroma_format_idc;

} x264_sps_t;

typedef struct
{
	/* PPS id, range: [0-255] */
    int i_id;
    /* SPS id, range: [0-31] */
    int i_sps_id;

    /* entropy coding mode flag, 0: CAVLC, 1: CABAC */
    int b_cabac;

    /* picture order present flag */
    int b_pic_order;
    /* number of slice groups */
    int i_num_slice_groups;

    /* number of reference frames in list 0 */
    int i_num_ref_idx_l0_default_active;
    /* number of reference frames in list 1 */
    int i_num_ref_idx_l1_default_active;

    /* weighted predict flag for P-slice */
    int b_weighted_pred;
    /* weighted predict flag for B-slice */
    int b_weighted_bipred;

    /* initial value of quantization parameters of luma */
    int i_pic_init_qp;
    /* initial value of quantization parameters of luma for SP/SI-slice */
    int i_pic_init_qs;

    /* offset of quantization parameters of chroma. default: 0 */
    int i_chroma_qp_index_offset;

    /* deblocking-filter control flag */
    int b_deblocking_filter_control;
    /* constrained-intra-predict flag */
    int b_constrained_intra_pred;
    /* redundant-picture-present flag */
    int b_redundant_pic_cnt;

    /* 8x8 transform mode flag. depends on x264_param_t.analyse.b_transform_8x8 */
    int b_transform_8x8_mode;

    int i_cqm_preset;
    /********************************************************
     * quantization matrices                                *
     * low  4 matrices for 4x4 quant: 4IY, 4PY, 4IC, 4PC    *
     * high 4 matrices for 8x8 quant: 8IY, 8PY, 8IC, 8PC    *
     * could be 12, but we don't allow separate Cb/Cr lists *
     * default value: x264_cqm_flat16                       *
     ********************************************************/
    const uint8_t *scaling_list[8];

} x264_pps_t;

/* default quant matrices of jvt mode */
/****************************************************************************
 * Page 94 of <ITU-T-REC-H.264-201201.pdf>                                  *
 * Table 7-3 ¨C Specification of default scaling lists                       *
 * Default_4x4_Intra and Default_4x4_Inter                                  *
 * idx                     0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15   *
 * Default_4x4_Intra[idx]  6 13 13 20 20 20 28 28 28 28 32 32 32 37 37 42   *
 * Default_4x4_Inter[idx] 10 14 14 20 20 20 24 24 24 24 27 27 27 30 30 34   *
 ****************************************************************************/
static const uint8_t x264_cqm_jvt4i[16] =
{
      6,13,20,28,
     13,20,28,32,
     20,28,32,37,
     28,32,37,42
};
static const uint8_t x264_cqm_jvt4p[16] =
{
    10,14,20,24,
    14,20,24,27,
    20,24,27,30,
    24,27,30,34
};

/****************************************************************************
 * Page 95 of <ITU-T-REC-H.264-201201.pdf>                                  *
 * Table 7-4 ¨C Specification of default scaling lists                       *
 * Default_8x8_Intra and Default_8x8_Inter                                  *
 * idx                     0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15   *
 * Default_8x8_Intra[idx]  6 10 10 13 11 13 16 16 16 16 18 18 18 18 18 23   *
 * Default_8x8_Inter[idx]  9 13 13 15 13 15 17 17 17 17 19 19 19 19 19 21   *
 *                                                                          *
 * idx                    16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31   *
 * Default_8x8_Intra[idx] 23 23 23 23 23 25 25 25 25 25 25 25 27 27 27 27   *
 * Default_8x8_Inter[idx] 21 21 21 21 21 22 22 22 22 22 22 22 24 24 24 24   *
 *                                                                          *
 * idx                    32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47   *
 * Default_8x8_Intra[idx] 27 27 27 27 29 29 29 29 29 29 29 31 31 31 31 31   *
 * Default_8x8_Inter[idx] 24 24 24 24 25 25 25 25 25 25 25 27 27 27 27 27   *
 *                                                                          *
 * idx                    48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63   *
 * Default_8x8_Intra[idx] 31 33 33 33 33 33 36 36 36 36 38 38 38 40 40 42   *
 * Default_8x8_Inter[idx] 27 28 28 28 28 28 30 30 30 30 32 32 32 33 33 35   *
 ****************************************************************************/
static const uint8_t x264_cqm_jvt8i[64] =
{
     6,10,13,16,18,23,25,27,
    10,11,16,18,23,25,27,29,
    13,16,18,23,25,27,29,31,
    16,18,23,25,27,29,31,33,
    18,23,25,27,29,31,33,36,
    23,25,27,29,31,33,36,38,
    25,27,29,31,33,36,38,40,
    27,29,31,33,36,38,40,42
};
static const uint8_t x264_cqm_jvt8p[64] =
{
     9,13,15,17,19,21,22,24,
    13,13,17,19,21,22,24,25,
    15,17,19,21,22,24,25,27,
    17,19,21,22,24,25,27,28,
    19,21,22,24,25,27,28,30,
    21,22,24,25,27,28,30,32,
    22,24,25,27,28,30,32,33,
    24,25,27,28,30,32,33,35
};
/* custom-quantization-matrix of flat mode */
static const uint8_t x264_cqm_flat16[64] =
{
    16,16,16,16,16,16,16,16,
    16,16,16,16,16,16,16,16,
    16,16,16,16,16,16,16,16,
    16,16,16,16,16,16,16,16,
    16,16,16,16,16,16,16,16,
    16,16,16,16,16,16,16,16,
    16,16,16,16,16,16,16,16,
    16,16,16,16,16,16,16,16
};
/* custom-quantization-matrix of jvt mode */
static const uint8_t * const x264_cqm_jvt[8] =
{
    x264_cqm_jvt4i, x264_cqm_jvt4p,
    x264_cqm_jvt4i, x264_cqm_jvt4p,
    x264_cqm_jvt8i, x264_cqm_jvt8p,
    x264_cqm_jvt8i, x264_cqm_jvt8p
};

int  x264_cqm_init( x264_t *h );
void x264_cqm_delete( x264_t *h );

#endif
