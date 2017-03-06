/*****************************************************************************
 * x264.h: x264 public header
 *****************************************************************************/

#ifndef X264_X264_H
#define X264_X264_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdarg.h>

#define X264_BUILD 128

/* x264_t:
 *      opaque handler for encoder */
typedef struct x264_t x264_t;

/****************************************************************************
 * NAL structure and functions
 ****************************************************************************/

enum nal_unit_type_e
{
    NAL_UNKNOWN     = 0,
    NAL_SLICE       = 1,    /* I/P/B slice */
    NAL_SLICE_DPA   = 2,    /* data partition a, only for Extended profile */
    NAL_SLICE_DPB   = 3,    /* data partition b, only for Extended profile */
    NAL_SLICE_DPC   = 4,    /* data partition c, only for Extended profile */
    NAL_SLICE_IDR   = 5,    /* ref_idc != 0 */
    NAL_SEI         = 6,    /* ref_idc == 0 */
    NAL_SPS         = 7,    /* sequence parameter set */
    NAL_PPS         = 8,    /* picture parameter set */
    NAL_AUD         = 9,    /* access unit delimiter, only for specail scenario */
    NAL_FILLER      = 12    /* filler, only for constant bitrate */
    /* ref_idc == 0 for 6,9,10,11,12 */
};
enum nal_priority_e
{
    NAL_PRIORITY_DISPOSABLE = 0,
    NAL_PRIORITY_LOW        = 1,
    NAL_PRIORITY_HIGH       = 2,
    NAL_PRIORITY_HIGHEST    = 3
};

/* The data within the payload is already NAL-encapsulated; the ref_idc and type
 * are merely in the struct for easy access by the calling application.
 * All data returned in an x264_nal_t, including the data in p_payload, is no longer
 * valid after the next call to x264_encoder_encode.  Thus it must be used or copied
 * before calling x264_encoder_encode or x264_encoder_headers again. */
typedef struct
{
    int i_ref_idc;  /* nal_priority_e */
    int i_type;     /* nal_unit_type_e */
    int b_long_startcode; /* long start-code flag. used only in annexb mode. */
    int i_first_mb; /* If this NAL is a slice, the index of the first MB in the slice. */
    int i_last_mb;  /* If this NAL is a slice, the index of the last MB in the slice. */

    /* Size of payload in bytes. */
    int     i_payload;
    /* If param->b_annexb is set, Annex-B bytestream with startcode.
     * Otherwise, startcode is replaced with a 4-byte size.
     * This size is the size used in mp4/similar muxing; it is equal to i_payload-4 */
    uint8_t *p_payload;
} x264_nal_t;

/****************************************************************************
 * Encoder parameters
 ****************************************************************************/
/* CPU flags
 */
#define X264_CPU_CACHELINE_32    0x0000001  /* avoid memory loads that span the border between two cachelines */
#define X264_CPU_CACHELINE_64    0x0000002  /* 32/64 is the size of a cacheline in bytes */
#define X264_CPU_ALTIVEC         0x0000004
#define X264_CPU_MMX             0x0000008
#define X264_CPU_MMX2            0x0000010  /* MMX2 aka MMXEXT aka ISSE */
#define X264_CPU_MMXEXT          X264_CPU_MMX2
#define X264_CPU_SSE             0x0000020
#define X264_CPU_SSE2            0x0000040
#define X264_CPU_SSE2_IS_SLOW    0x0000080  /* avoid most SSE2 functions on Athlon64 */
#define X264_CPU_SSE2_IS_FAST    0x0000100  /* a few functions are only faster on Core2 and Phenom */
#define X264_CPU_SSE3            0x0000200
#define X264_CPU_SSSE3           0x0000400
#define X264_CPU_SHUFFLE_IS_FAST 0x0000800  /* Penryn, Nehalem, and Phenom have fast shuffle units */
#define X264_CPU_STACK_MOD4      0x0001000  /* if stack is only mod4 and not mod16 */
#define X264_CPU_SSE4            0x0002000  /* SSE4.1 */
#define X264_CPU_SSE42           0x0004000  /* SSE4.2 */
#define X264_CPU_SSE_MISALIGN    0x0008000  /* Phenom support for misaligned SSE instruction arguments */
#define X264_CPU_LZCNT           0x0010000  /* Phenom support for "leading zero count" instruction. */
#define X264_CPU_ARMV6           0x0020000
#define X264_CPU_NEON            0x0040000  /* ARM NEON */
#define X264_CPU_FAST_NEON_MRC   0x0080000  /* Transfer from NEON to ARM register is fast (Cortex-A9) */
#define X264_CPU_SLOW_CTZ        0x0100000  /* BSR/BSF x86 instructions are really slow on some CPUs */
#define X264_CPU_SLOW_ATOM       0x0200000  /* The Atom just sucks */
#define X264_CPU_AVX             0x0400000  /* AVX support: requires OS support even if YMM registers
                                             * aren't used. */
#define X264_CPU_XOP             0x0800000  /* AMD XOP */
#define X264_CPU_FMA4            0x1000000  /* AMD FMA4 */
#define X264_CPU_AVX2            0x2000000  /* AVX2 */
#define X264_CPU_FMA3            0x4000000  /* Intel FMA3 */
#define X264_CPU_BMI1            0x8000000  /* BMI1 */
#define X264_CPU_BMI2           0x10000000  /* BMI2 */
#define X264_CPU_TBM            0x20000000  /* AMD TBM */

/* Analyse flags
 */
#define X264_ANALYSE_I4x4       0x0001  /* Analyse i4x4 */
#define X264_ANALYSE_I8x8       0x0002  /* Analyse i8x8 (requires 8x8 transform) */
#define X264_ANALYSE_PSUB16x16  0x0010  /* Analyse p16x8, p8x16 and p8x8 */
#define X264_ANALYSE_PSUB8x8    0x0020  /* Analyse p8x4, p4x8, p4x4 */
#define X264_ANALYSE_BSUB16x16  0x0100  /* Analyse b16x8, b8x16 and b8x8 */
#define X264_DIRECT_PRED_NONE        0  /* B-frame direct predict mode: none (turn off) */
#define X264_DIRECT_PRED_SPATIAL     1  /* B-frame direct predict mode: spatial */
#define X264_DIRECT_PRED_TEMPORAL    2  /* B-frame direct predict mode: temporal */
#define X264_DIRECT_PRED_AUTO        3  /* B-frame direct predict mode: auto (spatial or temporal) */
#define X264_ME_DIA                  0  /* Motion estimation mode: DIA */
#define X264_ME_HEX                  1  /* Motion estimation mode: HEX. default */
#define X264_ME_UMH                  2  /* Motion estimation mode: UMH */
#define X264_ME_ESA                  3  /* Motion estimation mode: ESA */
#define X264_ME_TESA                 4  /* Motion estimation mode: TESA */
#define X264_CQM_FLAT                0  /* CQM mode: Flat. default */
#define X264_CQM_JVT                 1  /* CQM mode: JVT definition */
#define X264_CQM_CUSTOM              2  /* CQM mode: customized by caller */
#define X264_RC_CQP                  0  /* Rate control mode: Constant QP */
#define X264_RC_CRF                  1  /* Rate control mode: Constant Rate Factor (or Constant Quality). default */
#define X264_RC_ABR                  2  /* Rate control mode: Average Bit Rate */
#define X264_QP_AUTO                 0
#define X264_AQ_NONE                 0  /* Adaptive quantization mode: none */
#define X264_AQ_VARIANCE             1  /* Adaptive quantization mode: variance. default */
#define X264_AQ_AUTOVARIANCE         2  /* Adaptive quantization mode: auto-variance */
#define X264_B_ADAPT_NONE            0  /* B-frame adaptive mode: none */
#define X264_B_ADAPT_FAST            1  /* B-frame adaptive mode: fast. default */
#define X264_B_ADAPT_TRELLIS         2  /* B-frame adaptive mode: trellis */
#define X264_WEIGHTP_NONE            0  /* Weighted P-frame predict mode: none */
#define X264_WEIGHTP_SIMPLE          1  /* Weighted P-frame predict mode: simple. */
#define X264_WEIGHTP_SMART           2  /* Weighted P-frame predict mode: smart. default */
#define X264_B_PYRAMID_NONE          0  /* B-frame pyramid mode: none */
#define X264_B_PYRAMID_STRICT        1  /* B-frame pyramid mode: strict */
#define X264_B_PYRAMID_NORMAL        2  /* B-frame pyramid mode: normal. default */
#define X264_KEYINT_MIN_AUTO         0
#define X264_KEYINT_MAX_INFINITE     (1<<30)

/* Colorspace type */
#define X264_CSP_MASK           0x00ff  /* */
#define X264_CSP_NONE           0x0000  /* Invalid mode     */
#define X264_CSP_I420           0x0001  /* yuv 4:2:0 planar. default */
#define X264_CSP_YV12           0x0002  /* yvu 4:2:0 planar */
#define X264_CSP_NV12           0x0003  /* yuv 4:2:0, with one y plane and one packed u+v */
#define X264_CSP_I422           0x0004  /* yuv 4:2:2 planar. supported only in High422 profile */
#define X264_CSP_YV16           0x0005  /* yvu 4:2:2 planar */
#define X264_CSP_NV16           0x0006  /* yuv 4:2:2, with one y plane and one packed u+v */
#define X264_CSP_I444           0x0007  /* yuv 4:4:4 planar. supported only in High444 profile */
#define X264_CSP_YV24           0x0008  /* yvu 4:4:4 planar */
#define X264_CSP_BGR            0x0009  /* packed bgr 24bits   */
#define X264_CSP_BGRA           0x000a  /* packed bgr 32bits   */
#define X264_CSP_RGB            0x000b  /* packed rgb 24bits   */
#define X264_CSP_MAX            0x000c  /* end of list */
#define X264_CSP_VFLIP          0x1000  /* the csp is vertically flipped */
#define X264_CSP_HIGH_DEPTH     0x2000  /* the csp has a depth of 16 bits per pixel component. supported only in High10 profile */

/* Slice type */
#define X264_TYPE_AUTO          0x0000  /* Let x264 choose the right type */
#define X264_TYPE_IDR           0x0001
#define X264_TYPE_I             0x0002
#define X264_TYPE_P             0x0003
#define X264_TYPE_BREF          0x0004  /* Non-disposable B-frame */
#define X264_TYPE_B             0x0005
#define X264_TYPE_KEYFRAME      0x0006  /* IDR or I depending on b_open_gop option */
#define IS_X264_TYPE_I(x) ((x)==X264_TYPE_I || (x)==X264_TYPE_IDR)
#define IS_X264_TYPE_B(x) ((x)==X264_TYPE_B || (x)==X264_TYPE_BREF)

/* Log level */
#define X264_LOG_NONE          (-1)
#define X264_LOG_ERROR          0
#define X264_LOG_WARNING        1
#define X264_LOG_INFO           2
#define X264_LOG_DEBUG          3

/* Threading */
#define X264_THREADS_AUTO 0 /* Automatically select optimal number of threads */
#define X264_SYNC_LOOKAHEAD_AUTO (-1) /* Automatically select optimal lookahead thread buffer size */

/* HRD */
#define X264_NAL_HRD_NONE            0
#define X264_NAL_HRD_VBR             1
#define X264_NAL_HRD_CBR             2

typedef struct x264_param_t
{
    /* CPU flags */
    unsigned int cpu;                /* cpu feature set. default: 0 */
    int         i_threads;           /* encode multiple frames in parallel. Set 1 if no thread support */
    int         i_lookahead_threads; /* multiple threads for lookahead analysis. Set 1 if no thread support */
    int         b_sliced_threads;    /* Whether to use slice-based threading. default: off */
    int         b_deterministic;     /* whether to allow non-deterministic optimizations when threaded. Set 0 if no thread support */
    int         b_cpu_independent;   /* force canonical behavior rather than cpu-dependent optimal algorithms. default: off */
    int         i_sync_lookahead;    /* threaded lookahead buffer. Set 0 if no thread support */

    /* Video Properties */
    int         i_width;       /* width in pixels */
    int         i_height;      /* height in pixels */
    int         i_csp;         /* CSP of encoded bitstream */
    int         i_level_idc;   /* H.264 level indication */
    int         i_frame_total; /* number of frames to encode if known, else 0 */

    /* NAL HRD
     * Uses Buffering and Picture Timing SEIs to signal HRD
     * The HRD in H.264 was not designed with VFR in mind.
     * It is therefore not recommended to use NAL HRD with VFR.
     * Furthermore, reconfiguring the VBV (via x264_encoder_reconfig)
     * will currently generate invalid HRD.
     *
     * Default: 0
     * Signal HRD information. Required for Blue-ray streams, television broadcast and a few other specialist areas.
     * Acceptable values are:
     * 0: none. Specify no HRD information
     * 1: vbr. Specify HRD information
     * 2: cbr. Specify HRD information and pack the bitstream to the bitrate specified by -bitrate. Requires bitrate mode ratecontrol.
     * Recommendation: none, unless you need to signal this information.
     */
    int         i_nal_hrd;

    struct
    {
        /* they will be reduced to be 0 < x <= 65535 and prime */
        int         i_sar_height;
        int         i_sar_width;

        int         i_overscan;    /* 0=undef, 1=no overscan, 2=overscan */

        /* see h264 annex E for the values of the following */
        int         i_vidformat;
        int         b_fullrange;
        int         i_colorprim;
        int         i_transfer;
        int         i_colmatrix;
        int         i_chroma_loc;    /* both top & bottom */
    } vui;

    /* Bitstream parameters */
    int         i_frame_reference;    /* Maximum number of reference frames */
    int         i_dpb_size;           /* Force a DPB size larger than that implied by B-frames and reference frames.
                                       * Useful in combination with interactive error resilience. */
    int         i_keyint_max;         /* Force an IDR keyframe at this interval. default: 50 */
    int         i_keyint_min;         /* Scenecuts closer together than this are coded as I, not IDR. default: 5 */
    int         i_scenecut_threshold; /* how aggressively to insert extra I frames */
    int         b_intra_refresh;      /* Whether or not to use periodic intra refresh instead of IDR frames. default: off */

    int         i_bframe;             /* how many b-frame between 2 references pictures */
    /*************************************************************************************************
     * Set the adaptive B-frame placement decision algorithm. This setting controls                  *
     * how x264 decides between placing a P- or B-frame.                                             *
     * range: [0, 2], default: 1                                                                     *
     *                                                                                               *
     * 0. Disabled. Pick B-frames always. This is the same as what the older no-b-adapt setting did. *
     * 1. 'Fast' algorithm, faster, speed slightly increases with higher --b-frames setting.         *
     * When using this mode, you basically always want to use --bframes 16.                          *
     * 2. 'Optimal' algorithm, slower, speed significantly decreases with higher --b-frames setting. *
     *************************************************************************************************/
    int         i_bframe_adaptive;
    /*******************************************************************************
     * Controls the likelihood of B-frames being used instead of P-frames.         *
     * range: [-100, 100], default: 0                                              *
     * Values greater than 0 increase the weighting towards B-frames,              *
     * while values less than 0 do the opposite. This number is an arbitrary       *
     * metric. The range is from -100 to 100. A value of 100/-100 does not         *
     * guarantee every/no P-frame will be converted (use --b-adapt 0 for that).    *
     * Only use this if you think you make better ratecontrol decisions than x264. *
     *******************************************************************************/
    int         i_bframe_bias;
    /***********************************************************************************
     * Allow the use of B-frames as references for other frames. Without this setting, *
     * frames can only reference I- or P-frames. Although I/P-frames are more valued   *
     * as references because of their higher quality, B-frames can also be useful.     *
     * B-frames designated as references will get a quantizer halfway between P-frames *
     * and normal B-frames. Need to use at least two B-frames before B-pyramid work.   *
     *                                                                                 *
     * range: [0, 2], default: 2 (normal)                                              *
     *                                                                                 *
     * - 0 (none): do not allow B-frames to be used as references.                     *
     * - 1 (strict): allow one B-frame per minigop to be used as reference;            *
     * - 2 (normal): allow numerous B-frames per minigop to be used as references.     *
     ***********************************************************************************/
    int         i_bframe_pyramid;
    int         b_open_gop;           /* Whether enables Open-GOP. default: off */
    /*************************************************************
     * Whether enables compatible with Blu-Ray players.         *
     * This feature enables a bunch of 'Blu-Ray hacks':         *
     * - min-CR + level 4.1 hack                                 *
     * - Special b-pyramid SEI                                   *
     * - B-frames cannot reference frames outside their minigop  *
     * - Open-GOP keyframe interval hack                         *
     *************************************************************/
    int         b_bluray_compat;

    int         b_deblocking_filter;            /* deblocking filter. default: on */
    int         i_deblocking_filter_alphac0;    /* [-6, 6] -6 light filter, 6 strong. default: 0 */
    int         i_deblocking_filter_beta;       /* [-6, 6]  idem. default: 0 */

    int         b_cabac;            /* CABAC entropy coding flag. default: on */
    /* The index for determining the initialization table used in
     * the initialization process for context variables.
     * range: [0-2], default: 0
     */
    int         i_cabac_init_idc;

    int         b_interlaced;        /* interlaced mode. default: off */
    int         b_constrained_intra; /* Enable constrained intra prediction, which is required for the base layer of SVC encodes. default: off */

    int         i_cqm_preset;       /* X264_CQM_*, value: FLAT/JVT/CUSTOM, default: FLAT */
    /*char        *psz_cqm_file;*/      /* JM format */
    /* used only if i_cqm_preset == X264_CQM_CUSTOM */
    /*uint8_t     cqm_4iy[16];*/        /* Intra_4x4_LUMA */
    /*uint8_t     cqm_4py[16];*/        /* Inter_4x4_LUMA */
    /*uint8_t     cqm_4ic[16];*/        /* Intra_4x4_CHROMA */
    /*uint8_t     cqm_4pc[16];*/        /* Inter_4x4_CHROMA */
    /*uint8_t     cqm_8iy[64];*/        /* Intra_8x8_LUMA */
    /*uint8_t     cqm_8py[64];*/        /* Inter_8x8_LUMA */
    /*uint8_t     cqm_8ic[64];*/        /* Intra_8x8_CHROMA */
    /*uint8_t     cqm_8pc[64];*/        /* Inter_8x8_CHROMA */

    /* Log */
    void        (*pf_log)( void *, int i_level, const char *psz, va_list );
    void        *p_log_private;
    int         i_log_level;
    /*int         b_full_recon;*/   /* fully reconstruct frames, even when not necessary for encoding. default: off */
    /*char        *psz_dump_yuv;*/  /* filename for reconstructed frames. default: NULL */

    /* Encoder analyser parameters */
    struct
    {
        unsigned int intra;     /* intra partitions. default: I4x4|I8x8 */
        unsigned int inter;     /* inter partitions. default: I4x4|I8x8|PSUB16x16|BSUB16*16 */

        int          b_transform_8x8; /* 8x8 transform mode flag. default: on */
        /*************************************************************************
         * weighting for P-frames                                                *
         * Enables use of explicit weighted prediction to improve compression    *
         * in P-frames. Also improves quality in fades. Higher modes are slower. *
         *                                                                       *
         * NOTE: When encoding for Adobe Flash set this to 1 - its decoder       *
         * generates artifacts otherwise. Flash 10.1 fixes this bug.             *
         *                                                                       *
         * 0. Disabled.                                                          *
         * 1. Simple: fade analysis, but no reference duplication.               *
         * 2. Smart: fade analysis and reference duplication. default            *
         *************************************************************************/
        int          i_weighted_pred;
        int          b_weighted_bipred; /* implicit weighting for B-frames */
        int          i_direct_mv_pred; /* spatial vs temporal mv prediction */
        /*********************************************************************
         * Add an offset to the quantizer of chroma planes when encoding.    *
         * The offset can be negative. default: 0.                           *
         *                                                                   *
         * When using psy options are enabled (psy-rd, psy-trellis),         *
         * x264 automatically subtracts 2 from this value to compensate      *
         * for these optimisations overly favouring luma detail by default.  *
         *                                                                   *
         * Note: x264 only encodes the luma and chroma planes at the same    *
         * quantizer up to quantizer 29. After this, chroma is progressively *
         * quantized by a lower amount than luma until you end with luma at  *
         * q51 and chroma at q39. This is required by the H.264 standard.    *
         *********************************************************************/
        int          i_chroma_qp_offset;

        /***********************************************************************************
         * motion estimation algorithm to use (X264_ME_*). default: HEX                    *
         *                                                                                 *
         * dia (diamond) is the simplest search, consisting of starting at the best        *
         * predictor, checking the motion vectors at one pixel upwards, left, down,        *
         * and to the right, picking the best, and repeating the process until it          *
         * no longer finds any better motion vector.                                       *
         *                                                                                 *
         * hex (hexagon) consists of a similar strategy, except it uses a range-2          *
         * search of 6 surrounding points, thus the name. It is considerably more          *
         * efficient than dia and hardly any slower, and therefore makes a good            *
         * choice for general-use encoding.                                                *
         *                                                                                 *
         * umh (uneven multi-hex) is considerably slower than hex, but searches a          *
         * complex multi-hexagon pattern in order to avoid missing harder-to-find          *
         * motion vectors. Unlike hex and dia, the merange parameter directly controls     *
         * umh's search radius, allowing one to increase or decrease the size of the       *
         * wide search.                                                                    *
         *                                                                                 *
         * esa (exhaustive) is a highly optimized intelligent search of the entire motion  *
         * search space within merange of the best predictor. It is mathematically         *
         * equivalent to the bruteforce method of searching every single motion vector in  *
         * that area, though faster. However, it is still considerably slower than UMH,    *
         * with not too much benefit, so is not particularly useful for everyday encoding. *
         ***********************************************************************************/
        int          i_me_method;
        /*********************************************************************************
         * integer pixel motion estimation search range (from predicted mv). default: 16 *
         * merange controls the max range of the motion search in pixels.                *
         * For hex and dia, the range is clamped to 4-16, with a default of 16.          *
         * For umh and esa, it can be increased beyond the default 16 to allow for a     *
         * wider-range motion search, which is useful on HD footage and for high-motion  *
         * footage. Note that for umh, esa, and tesa, increasing merange will            *
         * significantly slow down encoding.                                             *
         *********************************************************************************/
        int          i_me_range; /*  */
        /************************************************************************
         * maximum length of mv (pixels). default: -1(auto), range:[32-512].    *
         * if mv_range is not setted by user, use value defined by H.264 level. *
         * level > 30, mv_range: 512                                            *
         * level > 20, mv_range: 256                                            *
         * level > 10, mv_range: 128                                            *
         * level <= 10, mv_range: 64                                            *
         ************************************************************************/
        int          i_mv_range;
        int          i_mv_range_thread; /* minimum space between threads. -1 = auto, based on number of threads. */
        /**************************************************************************************
         * sub-pixel motion estimation quality, range: [0-11], default: 7                     *
         * Set the subpixel estimation complexity. Higher numbers are better. Levels 1-5      *
         * simply control the subpixel refinement strength. Level 6 enables RDO for mode      *
         * decision, and level 8 enables RDO for motion vectors and intra prediction modes.   *
         * Level 10 enables QPRD (use rate distortion optimization for qp selection).         *
         *                                                                                    *
         * Using a value less than 2 will enable a faster, and lower quality lookahead mode,  *
         * as well as cause poorer --scenecut decisions to be made, and thus not recommended. *
         *                                                                                    *
         * Possible Values:                                                                   *
         * 0. fullpel only                                                                    *
         * 1. QPel SAD 1 iteration                                                            *
         * 2. QPel SATD 2 iterations                                                          *
         * 3. HPel on MB then QPel                                                            *
         * 4. Always QPel                                                                     *
         * 5. Multi QPel + bi-directional motion estimation                                   *
         * 6. RD on I/P frames                                                                *
         * 7. RD on all frames. [default]                                                     *
         * 8. RD refinement on I/P frames                                                     *
         * 9. RD refinement on all frames                                                     *
         * 10. QP-RD (requires --trellis=2, --aq-mode > 0)                                    *
         * 11. Full RD                                                                        *
         **************************************************************************************/
        int          i_subpel_refine;
        int          b_chroma_me; /* chroma ME for subpel and mode decision in P-frames. default: on */
        int          b_mixed_references; /* allow each mb partition to have its own reference frame. default: on (need i_frame_reference > 1) */
        /****************************************************************************
         * Trellis RD quantization. range: [0-2], default: 1.                       *
         * - 0: disabled.                                                           *
         * - 1: enabled only on the final encode of a MB. [default]                 *
         * - 2: enabled on all mode decisions of a MB.                              *
         *                                                                          *
         * Trellis quantization is an algorithm that can improve data compression   *
         * in DCT-based encoding methods. It is used to optimize residual DCT       *
         * coefficients after motion estimation in lossy video compression.         *
         *                                                                          *
         * Trellis quantization reduces the size of some DCT coefficients while     *
         * recovering others to take their place. This process can increase quality *
         * because coefficients chosen by Trellis have the lowest rate-distortion   *
         * ratio. Trellis quantization effectively finds the optimal quantization   *
         * for each block to maximize the PSNR relative to bitrate.                 *
         *                                                                          *
         * Note that Trellis is still considered 'experimental', and almost         *
         * certainly is a Bad Thing for at least cartoons.                          *
         ****************************************************************************/
        int          i_trellis;
        int          b_fast_pskip; /* early SKIP detection on P-frames. default: on */
        /*****************************************************
         * DCT Decimation. default: on                       *
         * transform coefficient thresholding on P-frames.   *
         *                                                   *
         * This will drop DCT blocks it deems "unnecessary", *
         * and improve coding efficiency, with a usually     *
         * negligible loss in quality.                       *
         *****************************************************/
        int          b_dct_decimate;
        /********************************************************
         * adaptive pseudo-deadzone. default: 0                 *
         * Performs fast noise reduction. Estimates film noise  *
         * based on this value and attempts to remove it by     *
         * dropping small details before quantization. This may *
         * not match the quality of a good external noise       *
         * reduction filter, but it performs very fast.         *
         ********************************************************/
        int          i_noise_reduction;

        /********************************************************************************
         * Psy RD: Psychovisually optimized rate-distortion optimization                *
         * How it works (simply): the human eye doesn't just want the image to look     *
         * similar to the original, it wants the image to have similar complexity.      *
         * Therefore, we would rather see a somewhat distorted but still detailed block *
         * than a non-distorted but completely blurred block. The result is a bias      *
         * towards a detailed and/or grainy output image, a bit like xvid except that   *
         * its actual detail rather than ugly blocking.                                 *
         ********************************************************************************/
        float        f_psy_rd; /* Psy RD strength. (requires subme>=6 to activate) */
        float        f_psy_trellis; /* Psy trellis strength. (requires trellis>=1 to activate) */
        int          b_psy; /* Toggle all psy optimizations. default: on */

        int          b_mb_info; /* Use input mb_info data in x264_picture_t. see x264_image_properties_t.mb_info. default: no */
        int          b_mb_info_update; /* Update the values in mb_info according to the results of encoding. default: no */
        /******************************************************************
         *  Set the deadzone size that will be used in luma quantization. *
         *  The deadzone value sets the level of fine detail that x264    *
         *  will arbitrarily drop without attempting to preserve. Very    *
         *  fine detail is both hard to see and expensive to encode,      *
         *  dropping this detail without attempting to preserve it stops  *
         *  wasting bits on such a low-return section of the video.       *
         *  Deadzone is incompatible with Trellis.                        *
         *  default: [0]: 21 (inter), [1]: 11 (intra), range: [0 - 32]    *
         ******************************************************************/
        int          i_luma_deadzone[2];

        int          b_psnr;    /* compute and print PSNR stats. default: off */
        int          b_ssim;    /* compute and print SSIM stats. default: off */
    } analyse;

    /* Rate control parameters */
    struct
    {
        int         i_rc_method;    /* X264_RC_*, value: CQP/CRF/ABR, default: CRF */

        int         i_qp_constant;  /* 0 to (51 + 6*(x264_bit_depth-8)). 0=lossless. default: 23 */
        int         i_qp_min;       /* min allowed QP value */
        int         i_qp_max;       /* max allowed QP value */
        int         i_qp_step;      /* max QP step between frames. default: 4 */

        int         i_bitrate;      /* In ABR mode, average bitrate (kbps) */
        float       f_rf_constant;  /* 1pass VBR, nominal QP. default: 23 */
        float       f_rf_constant_max;  /* In CRF mode, maximum CRF as caused by VBV */
        /*********************************************************************************************
         * rate tolerance. default: 1.0                                                              *
         *                                                                                           *
         * This is a dual purpose parameter:                                                         *
         * In 1-pass bitrate encodes, this settings controls the percentage that x264 can miss the   *
         * target average bitrate by. You can set this to 'inf' to disable this overflow detection   *
         * completely. The lowest you can set this is to 0.01. The higher you set this to the better *
         * x264 can react to complex scenes near the end of the movie. The unit of measure for this  *
         * purpose is percent (eg, 1.0 = 1% bitrate deviation allowed).                              *
         * Many movies (any action movie, for instance) are most complex at the climatic finale. As  *
         * a 1pass encode doesn't know this, the number of bits required for the end is usually      *
         * underestimated. A ratetol of inf can mitigate this by allowing the encode to function     *
         * more like a --crf encode, but the filesize will blow out.                                 *
         *                                                                                           *
         * When VBV is activated (ie, you're specified --vbv-* options), this setting also affects   *
         * VBV aggressiveness. Setting this higher allows VBV to fluctuate more at the risk of       *
         * possibly violating the VBV settings. For this purpose, the unit of measure is arbitrary.  *
         *********************************************************************************************/
        float       f_rate_tolerance;
        /******************************************************************************************
         * The VBV (Video Buffer Verifier) system in x264 is used to constrain the output bitrate *
         * so it is suitable for streaming in a bandwidth constrained environment. The h.264 VBV  *
         * model is based around the idea of a "VBV buffer" on the decoder side. As the h.264     *
         * stream is downloaded by the client it's stored in the VBV buffer. A frame must be      *
         * fully downloaded into the VBV buffer before it can be decoded.                         *
         *                                                                                        *
         * x264 has three options that control the VBV buffer:                                    *
         * 1. The buffer's size is specified in kbit with --vbv-bufsize,                          *
         * 2. The rate at which the buffer fills is specified in kbit/sec by --vbv-maxrate,       *
         * 3. The proportion of the buffer that must be filled before playback can start is       *
         *    specified by --vbv-init.                                                            *
         *                                                                                        *
         * When using VBV, you always need to set the first two options and should never need to  *
         * set the last one.                                                                      *
         ******************************************************************************************/
        int         i_vbv_max_bitrate; /* setted by --vbv-maxrate. default: 0 */
        int         i_vbv_buffer_size; /* setted by --vbv-bufsize. default: 0 */
        float       f_vbv_buffer_init; /* <=1: fraction of buffer_size. >1: kbit. setted by --vbv-init. default: 0.9 */
        float       f_ip_factor;    /* qp ratio of I-frames / P-frames. default: 1.4 */
        float       f_pb_factor;    /* qp ratio of P-frames / B-frames. default: 1.3 */

        /********************************************************************************************
         * Adaptive Quantization Mode                                                               *
         *                                                                                          *
         * Without AQ, x264 tends to underallocate bits to less-detailed sections.                  *
         * AQ is used to better distribute the available bits between all macroblocks in the video. *
         * This setting changes what scope AQ re-arranges bits in:                                  *
         * 0: Do not use AQ at all.                                                                 *
         * 1: Allow AQ to redistribute bits across the whole video and within frames. default.      *
         * 2: Auto-variance AQ (experimental) which attempts to adapt strength per-frame.           *
         ********************************************************************************************/
        int         i_aq_mode;      /* psy adaptive QP. (X264_AQ_*) */
        float       f_aq_strength;  /* strength of AQ bias towards low detail ('flat') macroblocks. range: [0.0 - 2.0], default: 1.0 */
        /*********************************************************************************
         * Macroblock-tree ratecontrol. default: on                                      *
         * Using macroblock tree ratecontrol overall improves the compression by keeping *
         * track of temporal propagation across frames and weighting accordingly.        *
         *********************************************************************************/
        int         b_mb_tree;
        int         i_lookahead;    /* lookahead value. default: 40. Set 0 if no thread support */

        /* 2pass mode only */
        int         b_stat_write;   /* Enable stat writing in psz_stat_out. default: off */
        char        *psz_stat_out;  /* stat out file name. default: null */
        int         b_stat_read;    /* Read stat from psz_stat_in and use it. default: off */
        char        *psz_stat_in;   /* stat in file name. default: null */

        /* 2pass params (same as ffmpeg ones) */
        float       f_qcompress;    /* 0.0 => cbr, 1.0 => constant qp. default: 0.6 */
        float       f_qblur;        /* temporally blur quants */
        float       f_complexity_blur; /* temporally blur complexity */
    } rc;

    /* Cropping Rectangle parameters: added to those implicitly defined by
       non-mod16 video resolutions. */
    struct
    {
        unsigned int i_left;
        unsigned int i_top;
        unsigned int i_right;
        unsigned int i_bottom;
    } crop_rect;

    /**************************************************************
     * For stereoscopic (3D) videos define frame arrangement.     *
     * range: [-1 - 5], default: -1 (not set)                     *
     * - 0: checkerboard - pixels are alternatively from L and R  *
     * - 1: column alternation - L and R are interlaced by column *
     * - 2: row alternation - L and R are interlaced by row       *
     * - 3: side by side - L is on the left, R on the right       *
     * - 4: top bottom - L is on top, R on bottom                 *
     * - 5: frame alternation - one view per frame                *
     **************************************************************/
    int i_frame_packing;

    /* Muxing parameters */
    int b_aud;                  /* generate access unit delimiters. default: off */
    int b_repeat_headers;       /* put SPS/PPS before each keyframe. default: on. set 0 if output is mp4/mkv/flv */
    int b_annexb;               /* if set, place start codes (4 bytes) before NAL units,
                                 * otherwise place size (4 bytes) before NAL units. default: on. set 0 if output is mp4/mkv/flv */
    int i_sps_id;               /* SPS and PPS id number. range:[0-31], default: 0 */
    int b_vfr_input;            /* VFR input.  If 1, use timebase and timestamps for ratecontrol purposes.
                                 * If 0, use fps only. */
    int b_pulldown;             /* use explicity set timebase for CFR */
    uint32_t i_fps_num;         /* FPS numerator */
    uint32_t i_fps_den;         /* FPS denominator */
    uint32_t i_timebase_num;    /* Timebase numerator */
    uint32_t i_timebase_den;    /* Timebase denominator */

    /* Top field first flag, used only in interlaced mode.
     * 1: top field followed by bottom
     * 0: bottom field followed by top
     */
    int b_tff;

    /* Pulldown:
     * The correct pic_struct must be passed with each input frame.
     * The input timebase should be the timebase corresponding to the output framerate. This should be constant.
     * e.g. for 3:2 pulldown timebase should be 1001/30000
     * The PTS passed with each frame must be the PTS of the frame after pulldown is applied.
     * Frame doubling and tripling require b_vfr_input set to zero (see H.264 Table D-1)
     *
     * Pulldown changes are not clearly defined in H.264. Therefore, it is the calling app's responsibility to manage this.
     */

    int b_pic_struct;

    /* Fake Interlaced.
     *
     * Used only when b_interlaced=0. Setting this flag makes it possible to flag the stream as PAFF interlaced yet
     * encode all frames progessively. It is useful for encoding 25p and 30p Blu-Ray streams.
     */

    int b_fake_interlaced;

    /* Slicing parameters */
    int i_slice_max_size;    /* Max size per slice in bytes; includes estimated NAL overhead. default: 0 (unlimited) */
    int i_slice_max_mbs;     /* Max number of MBs per slice; overrides i_slice_count. default: 0 (unlimited) */
    int i_slice_count;       /* Number of slices per frame: forces rectangular slices. default: 0 */

    /* Optional callback for freeing this x264_param_t when it is done being used.
     * Only used when the x264_param_t sits in memory for an indefinite period of time,
     * i.e. when an x264_param_t is passed to x264_t in an x264_picture_t or in zones.
     * Not used when x264_encoder_reconfig is called directly. */
    void (*param_free)( void* );

    /* Optional low-level callback for low-latency encoding.  Called for each output NAL unit
     * immediately after the NAL unit is finished encoding.  This allows the calling application
     * to begin processing video data (e.g. by sending packets over a network) before the frame
     * is done encoding.
     *
     * This callback MUST do the following in order to work correctly:
     * 1) Have available an output buffer of at least size nal->i_payload*3/2 + 5 + 16.
     * 2) Call x264_nal_encode( h, dst, nal ), where dst is the output buffer.
     * After these steps, the content of nal is valid and can be used in the same way as if
     * the NAL unit were output by x264_encoder_encode.
     *
     * This does not need to be synchronous with the encoding process: the data pointed to
     * by nal (both before and after x264_nal_encode) will remain valid until the next
     * x264_encoder_encode call.  The callback must be re-entrant.
     *
     * This callback does not work with frame-based threads; threads must be disabled
     * or sliced-threads enabled.  This callback also does not work as one would expect
     * with HRD -- since the buffering period SEI cannot be calculated until the frame
     * is finished encoding, it will not be sent via this callback.
     *
     * Note also that the NALs are not necessarily returned in order when sliced threads is
     * enabled.  Accordingly, the variable i_first_mb and i_last_mb are available in
     * x264_nal_t to help the calling application reorder the slices if necessary.
     *
     * When this callback is enabled, x264_encoder_encode does not return valid NALs;
     * the calling application is expected to acquire all output NALs through the callback.
     *
     * It is generally sensible to combine this callback with a use of slice-max-mbs or
     * slice-max-size.
     *
     * The opaque pointer is the opaque pointer from the input frame associated with this
     * NAL unit. This helps distinguish between nalu_process calls from different sources,
     * e.g. if doing multiple encodes in one process.
     */
    void (*nalu_process) ( x264_t *h, x264_nal_t *nal, void *opaque );
} x264_param_t;

void x264_nal_encode( x264_t *h, uint8_t *dst, x264_nal_t *nal );

/****************************************************************************
 * H.264 level restriction information
 ****************************************************************************/

typedef struct
{
    int level_idc;   /* level indication */
    int mbps;        /* max macroblock processing rate (macroblocks/sec) */
    int frame_size;  /* max frame size (macroblocks) */
    int dpb;         /* max decoded picture buffer (bytes) */
    int bitrate;     /* max bitrate (kbit/sec) */
    int cpb;         /* max vbv buffer (kbit) */
    int mv_range;    /* max vertical mv component range (pixels) */
    int mvs_per_2mb; /* max mvs per 2 consecutive mbs. */
    int slice_rate;  /* ?? */
    int mincr;       /* min compression ratio */
    int bipred8x8;   /* limit bipred to >=8x8 */
    int direct8x8;   /* limit b_direct to >=8x8 */
    int frame_only;  /* forbid interlacing */
} x264_level_t;

/* all of the levels defined in the standard, terminated by .level_idc=0 */
extern const x264_level_t x264_levels[];

/****************************************************************************
 * Basic parameter handling functions
 ****************************************************************************/

/* x264_param_default:
 *      fill x264_param_t with default values and do CPU detection */
void    x264_param_default( x264_param_t * );

/****************************************************************************
 * Picture structures and functions
 ****************************************************************************/

/* x264_bit_depth:
 *      Specifies the number of bits per pixel that x264 uses. This is also the
 *      bit depth that x264 encodes in. If this value is > 8, x264 will read
 *      two bytes of input data for each pixel sample, and expect the upper
 *      (16-x264_bit_depth) bits to be zero.
 *      Note: The flag X264_CSP_HIGH_DEPTH must be used to specify the
 *      colorspace depth as well. */
extern const int x264_bit_depth;

/* x264_chroma_format:
 *      Specifies the chroma formats that x264 supports encoding. When this
 *      value is non-zero, then it represents a X264_CSP_* that is the only
 *      chroma format that x264 supports encoding. If the value is 0 then
 *      there are no restrictions. */
extern const int x264_chroma_format;

enum pic_struct_e
{
    PIC_STRUCT_AUTO              = 0, // automatically decide (default)
    PIC_STRUCT_PROGRESSIVE       = 1, // progressive frame
    // "TOP" and "BOTTOM" are not supported in x264 (PAFF only)
    PIC_STRUCT_TOP_BOTTOM        = 4, // top field followed by bottom
    PIC_STRUCT_BOTTOM_TOP        = 5, // bottom field followed by top
    PIC_STRUCT_TOP_BOTTOM_TOP    = 6, // top field, bottom field, top field repeated
    PIC_STRUCT_BOTTOM_TOP_BOTTOM = 7, // bottom field, top field, bottom field repeated
    PIC_STRUCT_DOUBLE            = 8, // double frame
    PIC_STRUCT_TRIPLE            = 9  // triple frame
};

typedef struct
{
    double cpb_initial_arrival_time;
    double cpb_final_arrival_time;
    double cpb_removal_time;

    double dpb_output_time;
} x264_hrd_t;

typedef struct
{
    int     i_csp;       /* Colorspace */
    int     i_plane;     /* Number of image planes */
    int     i_stride[4]; /* Strides for each plane */
    uint8_t *plane[4];   /* Pointers to each plane */
} x264_image_t;

typedef struct
{
    /* All arrays of data here are ordered as follows:
     * each array contains one offset per macroblock, in raster scan order.  In interlaced
     * mode, top-field MBs and bottom-field MBs are interleaved at the row level.
     * Macroblocks are 16x16 blocks of pixels (with respect to the luma plane).  For the
     * purposes of calculating the number of macroblocks, width and height are rounded up to
     * the nearest 16.  If in interlaced mode, height is rounded up to the nearest 32 instead. */

    /* In: an array of quantizer offsets to be applied to this image during encoding.
     *     These are added on top of the decisions made by x264.
     *     Offsets can be fractional; they are added before QPs are rounded to integer.
     *     Adaptive quantization must be enabled to use this feature.  Behavior if quant
     *     offsets differ between encoding passes is undefined. */
    float *quant_offsets;
    /* In: optional callback to free quant_offsets when used.
     *     Useful if one wants to use a different quant_offset array for each frame. */
    void (*quant_offsets_free)( void* );

    /* In: optional array of flags for each macroblock.
     *     Allows specifying additional information for the encoder such as which macroblocks
     *     remain unchanged.  Usable flags are listed below.
     *     x264_param_t.analyse.b_mb_info must be set to use this, since x264 needs to track
     *     extra data internally to make full use of this information.
     *
     * Out: if b_mb_info_update is set, x264 will update this array as a result of encoding.
     *
     *      For "MBINFO_CONSTANT", it will remove this flag on any macroblock whose decoded
     *      pixels have changed.  This can be useful for e.g. noting which areas of the
     *      frame need to actually be blitted. Note: this intentionally ignores the effects
     *      of deblocking for the current frame, which should be fine unless one needs exact
     *      pixel-perfect accuracy.
     *
     *      Results for MBINFO_CONSTANT are currently only set for P-frames, and are not
     *      guaranteed to enumerate all blocks which haven't changed.  (There may be false
     *      negatives, but no false positives.)
     */
    uint8_t *mb_info;
    /* In: optional callback to free mb_info when used. */
    void (*mb_info_free)( void* );

    /* The macroblock is constant and remains unchanged from the previous frame. */
    #define X264_MBINFO_CONSTANT   (1<<0)
    /* More flags may be added in the future. */

    /* Out: SSIM of the the frame luma (if x264_param_t.b_ssim is set) */
    double f_ssim;
    /* Out: Average PSNR of the frame (if x264_param_t.b_psnr is set) */
    double f_psnr_avg;
    /* Out: PSNR of Y, U, and V (if x264_param_t.b_psnr is set) */
    double f_psnr[3];

    /* Out: Average effective CRF of the encoded frame */
    double f_crf_avg;
} x264_image_properties_t;

typedef struct
{
    /* In: force picture type (if not auto)
     *     If x264 encoding parameters are violated in the forcing of picture types,
     *     x264 will correct the input picture type and log a warning.
     *     The quality of frametype decisions may suffer if a great deal of fine-grained
     *     mixing of auto and forced frametypes is done.
     * Out: type of the picture encoded */
    int     i_type;
    /* In: force quantizer for != X264_QP_AUTO */
    int     i_qpplus1;
    /* In: pic_struct, for pulldown/doubling/etc...used only if b_pic_struct=1.
     *     use pic_struct_e for pic_struct inputs
     * Out: pic_struct element associated with frame */
    int     i_pic_struct;
    /* Out: whether this frame is a keyframe.  Important when using modes that result in
     * SEI recovery points being used instead of IDR frames. */
    int     b_keyframe;
    /* In: user pts, Out: pts of encoded picture (user)*/
    int64_t i_pts;
    /* Out: frame dts. When the pts of the first frame is close to zero,
     *      initial frames may have a negative dts which must be dealt with by any muxer */
    int64_t i_dts;
    /* In: custom encoding parameters to be set from this frame forwards
           (in coded order, not display order). If NULL, continue using
           parameters from the previous frame.  Some parameters, such as
           aspect ratio, can only be changed per-GOP due to the limitations
           of H.264 itself; in this case, the caller must force an IDR frame
           if it needs the changed parameter to apply immediately. */
    x264_param_t *param;
    /* In: raw image data */
    /* Out: reconstructed image data.  x264 may skip part of the reconstruction process,
            e.g. deblocking, in frames where it isn't necessary.  To force complete
            reconstruction, at a small speed cost, set b_full_recon. */
    x264_image_t img;
    /* In: optional information to modify encoder decisions for this frame
     * Out: information about the encoded frame */
    x264_image_properties_t prop;
    /* Out: HRD timing information. Output only when i_nal_hrd is set. */
    x264_hrd_t hrd_timing;
    /* private user data. copied from input to output frames. */
    void *opaque;
} x264_picture_t;

/* x264_picture_init:
 *  initialize an x264_picture_t.  Needs to be done if the calling application
 *  allocates its own x264_picture_t as opposed to using x264_picture_alloc. */
void x264_picture_init( x264_picture_t *pic );

/* x264_picture_alloc:
 *  alloc data for a picture. You must call x264_picture_clean on it.
 *  returns 0 on success, or -1 on malloc failure or invalid colorspace. */
int x264_picture_alloc( x264_picture_t *pic, int i_csp, int i_width, int i_height );

/* x264_picture_clean:
 *  free associated resource for a x264_picture_t allocated with
 *  x264_picture_alloc ONLY */
void x264_picture_clean( x264_picture_t *pic );

/****************************************************************************
 * Encoder functions
 ****************************************************************************/

/* x264_encoder_open:
 *      create a new encoder handler, all parameters from x264_param_t are copied */
x264_t *x264_encoder_open( x264_param_t * );
/* x264_encoder_parameters:
 *      copies the current internal set of parameters to the pointer provided
 *      by the caller.  useful when the calling application needs to know
 *      how x264_encoder_open has changed the parameters, or the current state
 *      of the encoder after multiple x264_encoder_reconfig calls.
 *      note that the data accessible through pointers in the returned param struct
 *      (e.g. filenames) should not be modified by the calling application. */
void    x264_encoder_parameters( x264_t *, x264_param_t * );
/* x264_encoder_headers:
 *      return the SPS and PPS that will be used for the whole stream.
 *      *pi_nal is the number of NAL units outputted in pp_nal.
 *      returns negative on error.
 *      the payloads of all output NALs are guaranteed to be sequential in memory. */
int     x264_encoder_headers( x264_t *, x264_nal_t **pp_nal, int *pi_nal );
/* x264_encoder_encode:
 *      encode one picture.
 *      *pi_nal is the number of NAL units outputted in pp_nal.
 *      returns negative on error, zero if no NAL units returned.
 *      the payloads of all output NALs are guaranteed to be sequential in memory. */
int     x264_encoder_encode( x264_t *, x264_nal_t **pp_nal, int *pi_nal, x264_picture_t *pic_in, x264_picture_t *pic_out );
/* x264_encoder_close:
 *      close an encoder handler */
void    x264_encoder_close  ( x264_t * );

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
