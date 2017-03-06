/*****************************************************************************
 * analyse.c: macroblock analysis
 *****************************************************************************/

#include "common/common.h"
#include "macroblock.h"
#include "me.h"
#include "ratecontrol.h"
#include "analyse.h"

typedef struct
{
    /* 16x16 */
    x264_me_t me16x16; /* me of b16x16, setted by x264_mb_analyse_inter_p16x16 */

    /* 8x8 */
    int       i_cost8x8; /* sum of 4 b8x8 costs */
    /* [ref][0] is 16x16 mv, [ref][1..4] are 8x8 mv from partition [0..3] */
    ALIGNED_4( int16_t mvc[32][5][2] );
    x264_me_t me8x8[4]; /* me of 4 b8x8, setted by x264_mb_analyse_inter_p8x8 */

    /* 16x8 */
    int       i_cost16x8;
    x264_me_t me16x8[2]; /* me of 2 b16x8, setted by x264_mb_analyse_inter_p16x8 */

    /* 8x16 */
    int       i_cost8x16;
    x264_me_t me8x16[2]; /* me of 2 b8x16, setted by x264_mb_analyse_inter_p8x16 */

} x264_mb_analysis_list_t;

typedef struct
{
    /* conduct the analysis using this lamda and QP */
    int i_lambda;  /* x264_lambda_tab[i_qp] */
    int i_qp;      /* current mb luma qp */
    uint16_t *p_cost_mv; /* an array of lambda*nbits for all possible mvs. pointer to x264_t.cost_mv[i_qp] */
    uint16_t *p_cost_ref[2];
    /******************************************************************************
     * RD mode for macroblock analysis                                            *
     *                                                                            *
     * i_mbrd = (subme>=6) + (subme>=8) + (subme>=10)                             *
     *                                                                            *
     * mbrd == 0 -> RD disabled (default)                                         *
     * mbrd == 1 -> RD mode decision                                              *
     * mbrd == 2 -> RD refinement                                                 *
     * mbrd == 3 -> QPRD. (Slowest, best quality)                                 *
     * When QPRD is enabled, use rate distortion optimization for qp selection.   *
     ******************************************************************************/
    int i_mbrd;

    /* I: Intra part */
    /* Take some shortcuts in intra search if intra is deemed unlikely. */
    /* Unlikely detection is performed in x264_mb_analyse_init. */
    int b_fast_intra;
    int b_force_intra; /* For Periodic Intra Refresh.  Only supported in P-frames. default: off */
    int b_avoid_topright; /* For Periodic Intra Refresh: don't predict from top-right pixels. */
    int b_try_skip; /* Whether try probe skip in p16x16/b16x16 */

    /* Luma part */
    int i_satd_i16x16;    /* i16x16 satd */
    int i_satd_i16x16_dir[7];
    int i_predict16x16;   /* predict mode for 1 b16 */

    int i_satd_i4x4;      /* i4x4 satd */
    int i_predict4x4[16]; /* predict mode for 16 b4 */

    /* Chroma part */
    int i_satd_chroma;    /* Intra_Chroma satd for chroma mode decision */
    int i_satd_chroma_dir[7];
    int i_predict8x8chroma; /* predict mode for chroma */

    /* II: Inter part P/B frame */
    x264_mb_analysis_list_t l0;
    x264_mb_analysis_list_t l1;

    int i_satd8x8[3][4]; /* [L0,L1,BI][8x8 0..3] SATD only */
    int i_cost_est16x8[2]; /* Per-partition estimated cost */
    int i_cost_est8x16[2];

    int i_mb_partition16x8[2]; /* mb_partition_e */
    int i_mb_partition8x16[2];
    int i_mb_type16x8; /* mb_class_e */
    int i_mb_type8x16;

    int b_early_terminate;  /* fast decision flag. disabled only when full rd (subme == 11). default: on */

} x264_mb_analysis_t;

/********************************************************************
 * lambda = pow(2, qp / 6 - 2)                                      *
 * e.g.                                                             *
 * lambda[39] = pow(2, 39 / 6 - 2) = pow(2, 4.5) = (int)22.63 => 23 *
 * lambda[51] = pow(2, 51 / 6 - 2) = pow(2, 6.5) = (int)90.51 => 91 *
 *                                                                  *
 * usually we only use qp range of luma: [0, 51], chroma: [0, 39]   *
 ********************************************************************/
const uint16_t x264_lambda_tab[QP_MAX_MAX+1] =
{
   1,   1,   1,   1,   1,   1,   1,   1, /*  0- 7 */
   1,   1,   1,   1,   1,   1,   1,   1, /*  8-15 */
   2,   2,   2,   2,   3,   3,   3,   4, /* 16-23 */
   4,   4,   5,   6,   6,   7,   8,   9, /* 24-31 */
  10,  11,  13,  14,  16,  18,  20,  23, /* 32-39 */
  25,  29,  32,  36,  40,  45,  51,  57, /* 40-47 */
  64,  72,  81,  91, 102, 114, 128, 144, /* 48-55 */
 161, 181, 203, 228, 256, 287, 323, 362, /* 56-63 */
 406, 456, 512, 575, 645, 724, 813, 912, /* 64-71 */
1024,1149,1290,1448,1625,1825,2048,2299, /* 72-79 */
2048,2299,                               /* 80-81 */
};

/****************************************
 * lambda2 = pow(lambda, 2) x 0.9 x 256 *
 * Capped to avoid overflow             *
 ****************************************/
const int x264_lambda2_tab[QP_MAX_MAX+1] =
{
       14,       18,       22,       28,       36,       45,      57,      72, /*  0- 7 */
       91,      115,      145,      182,      230,      290,     365,     460, /*  8-15 */
      580,      731,      921,     1161,     1462,     1843,    2322,    2925, /* 16-23 */
     3686,     4644,     5851,     7372,     9289,    11703,   14745,   18578, /* 24-31 */
    23407,    29491,    37156,    46814,    58982,    74313,   93628,  117964, /* 32-39 */
   148626,   187257,   235929,   297252,   374514,   471859,  594505,  749029, /* 40-47 */
   943718,  1189010,  1498059,  1887436,  2378021,  2996119, 3774873, 4756042, /* 48-55 */
  5992238,  7549747,  9512085, 11984476, 15099494, 19024170,23968953,30198988, /* 56-63 */
 38048341, 47937906, 60397977, 76096683, 95875813,120795955,                   /* 64-69 */
134217727,134217727,134217727,134217727,134217727,134217727,                   /* 70-75 */
134217727,134217727,134217727,134217727,134217727,134217727,                   /* 76-81 */
};

/* lookup table for x264_log2 function */
const float x264_log2_lut[128] =
{
    0.00000, 0.01123, 0.02237, 0.03342, 0.04439, 0.05528, 0.06609, 0.07682,
    0.08746, 0.09803, 0.10852, 0.11894, 0.12928, 0.13955, 0.14975, 0.15987,
    0.16993, 0.17991, 0.18982, 0.19967, 0.20945, 0.21917, 0.22882, 0.23840,
    0.24793, 0.25739, 0.26679, 0.27612, 0.28540, 0.29462, 0.30378, 0.31288,
    0.32193, 0.33092, 0.33985, 0.34873, 0.35755, 0.36632, 0.37504, 0.38370,
    0.39232, 0.40088, 0.40939, 0.41785, 0.42626, 0.43463, 0.44294, 0.45121,
    0.45943, 0.46761, 0.47573, 0.48382, 0.49185, 0.49985, 0.50779, 0.51570,
    0.52356, 0.53138, 0.53916, 0.54689, 0.55459, 0.56224, 0.56986, 0.57743,
    0.58496, 0.59246, 0.59991, 0.60733, 0.61471, 0.62205, 0.62936, 0.63662,
    0.64386, 0.65105, 0.65821, 0.66534, 0.67243, 0.67948, 0.68650, 0.69349,
    0.70044, 0.70736, 0.71425, 0.72110, 0.72792, 0.73471, 0.74147, 0.74819,
    0.75489, 0.76155, 0.76818, 0.77479, 0.78136, 0.78790, 0.79442, 0.80090,
    0.80735, 0.81378, 0.82018, 0.82655, 0.83289, 0.83920, 0.84549, 0.85175,
    0.85798, 0.86419, 0.87036, 0.87652, 0.88264, 0.88874, 0.89482, 0.90087,
    0.90689, 0.91289, 0.91886, 0.92481, 0.93074, 0.93664, 0.94251, 0.94837,
    0.95420, 0.96000, 0.96578, 0.97154, 0.97728, 0.98299, 0.98868, 0.99435,
};

/* Avoid an int/float conversion. */
const float x264_log2_lz_lut[32] =
{
    31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0
};

static void x264_analyse_update_cache( x264_t *h, x264_mb_analysis_t *a );
static uint16_t x264_cost_ref[QP_MAX+1][3][33];     /* reference cost for all qp. list: [0, 1, 2], ref: [-16, 16] */

/*****************************************************
 * mv_bits = (int)(log2f(mv + 1) * 2 + 1.718f + .5f) *
 * e.g.                                              *
 * mv_bits: 10, mv_range: [14, 14 + 6)               *
 * 10 = (int)(log2f(14 + 1) * 2 + 1.718f + .5f)      *
 *                                                   *
 * mv_bits: 15, mv_range: [83, 83 + 35)              *
 * 15 = (int)(log2f(83 + 1) * 2 + 1.718f + .5f)      *
 *****************************************************/
static const uint16_t x264_mv_bits_tab[23][3] =
{
	/* mv_bits,  mv_range */
	{     4,     1,     1 },
	{     5,     2,     1 },
	{     6,     3,     2 },
	{     7,     5,     2 },
	{     8,     7,     3 },
	{     9,    10,     4 },
	{    10,    14,     6 },
	{    11,    20,     9 },
	{    12,    29,    12 },
	{    13,    41,    18 },
	{    14,    59,    24 },
	{    15,    83,    35 },
	{    16,   118,    49 },
	{    17,   167,    70 },
	{    18,   237,    98 },
	{    19,   335,   139 },
	{    20,   474,   197 },
	{    21,   671,   278 },
	{    22,   949,   393 },
	{    23,  1342,   556 },
	{    24,  1898,   787 },
	{    25,  2685,  1112 },
	{    26,  3797,   300 },
	/* truncate last range to 4096 */
	/*
	{    26,  3797,  1574 },
	{    27,  5371,  2224 },
	{    28,  7595,  3147 },
	{    29, 10742,  4449 },
	{    30, 15191,  6293 },
	{    31, 21484,  8899 },
	*/
};

static inline void x264_memset_uint16( uint16_t * dst, uint16_t d, int count )
{
	int i;
	int div8 = count >> 3;
	int mod8 = count - (div8 << 3);
	uint16_t * cur = dst;

	if( div8 > 0 )
	{
		uint32_t d4 = _pack2(  d,  d );
		int64_t  d8 = _itoll( d4, d4 );

		for( i = 0; i < div8; i++ )
		{
			_mem8( cur ) = d8;
			_mem8( cur + 4 ) = d8;
			cur += 8;
		}
	}

	if( mod8 > 0 )
	{
		for( i = 0; i < mod8; i++ )
			cur[i] = d;
	}
}

/**********************************************************
 * x264_analyse_init_costs is called by x264_encoder_open *
 * for initialization of cost_mv table.                   *
 * this may occupy space about: 16KB * (52 - 23) = 464KB  *
 *                                                        *
 * this function costs 150,0000 cpu cycles,               *
 * including 140,0000 cycles of x264_memset_uint16 call.  *
 **********************************************************/
int x264_analyse_init_costs( x264_t *h )
{
	int i, qp;
	uint16_t lambda, mv_cost[23], ref_bits;

	for( qp = 0; qp <= QP_MAX_SPEC; qp++ )
	{
		/* if lambda is equal, then mv_cost and ref_cost should be same.
		 * by copy them directly, we save 23 qp calculations here.
		 */
		if( qp > 0 && x264_lambda_tab[qp] == x264_lambda_tab[qp - 1] )
		{
			h->cost_mv[qp] = h->cost_mv[qp - 1];
			memcpy(x264_cost_ref[qp], x264_cost_ref[qp - 1], 3 * 33 * sizeof(uint16_t));
		}
		else
		{
			lambda = x264_lambda_tab[qp];

			/* factor of 4 from qpel, 2 from sign, and 2 because mv can be opposite from mvp */
			CHECKED_MALLOC( h->cost_mv[qp], (4*4*512 + 1) * sizeof(uint16_t) ); /* 16KB */
			h->cost_mv[qp] += 2*4*512;
			h->cost_mv[qp][0] = lambda;

#pragma MUST_ITERATE(23, 23, 23)
			for( i = 0; i < 23; i++ )
			{
				/***************************************************************
				 * mv_cost = lambda * mv_bits                                  *
				 *                                                             *
				 * 1. lambda =  2 ^ (qp / 6 - 2)                               *
				 *           => (uint_16)(2 ^ (qp / 6 - 2))                    *
				 *    pre-defined as const uint_16 by x264_lambda_tab          *
				 *                                                             *
				 * 2. mv_bits =  log2f(mv + 1) * 2 + 1.718f                    *
				 *            => (uint_16)(log2f(mv + 1) * 2 + 1.718f + .5f)   *
				 *    pre-defined as const uint_16 by x264_mv_bits_tab         *
				 *                                                             *
				 *    This is the continuous probability distribution to which *
				 *    exp-golomb is a discrete approximation. We just took     *
				 *    cavlc's mv cost and removed the rounding to whole bits.  *
				 *                                                             *
				 * h->cost_mv[qp][i] = h->cost_mv[qp][-i] = lambda * mv_bits   *
				 *  =  (2^(qp/6-2)) * (log2f(i+1)*2 + 1.718f) + .5f            *
				 *  => (uint_16)(2 ^ (qp / 6 - 2)) *                           *
				 *     (uint_16)(log2f(mv + 1) * 2 + 1.718f + .5f)             *
				 *                                                             *
				 ***************************************************************/
				mv_cost[i] = lambda * x264_mv_bits_tab[i][0]; /* 16bit x 16bit multiple */
			}

			/* FIXME: memory assignment too slow, cost: 140,0000 cpu cycles for 29 qp. */
			for( i = 22; i >= 0; i-- )
				x264_memset_uint16( h->cost_mv[qp] - (x264_mv_bits_tab[i][1] + x264_mv_bits_tab[i][2] - 1), mv_cost[i], x264_mv_bits_tab[i][2] );
			for( i = 0; i < 23; i++ )
				x264_memset_uint16( h->cost_mv[qp] + x264_mv_bits_tab[i][1], mv_cost[i], x264_mv_bits_tab[i][2] );

			/* init reference cost: lambda * ref_bits */
			x264_memset_uint16( x264_cost_ref[qp][0],      0, 33 );
			x264_memset_uint16( x264_cost_ref[qp][1], lambda, 33 );
#pragma MUST_ITERATE(33, 33, 33)
			for( i = 0; i < 33; i++ )
			{
				ref_bits = (uint16_t)bs_size_te( 2, i );
				x264_cost_ref[qp][2][i] = lambda * ref_bits; /* 16bit x 16bit multiple */
			}
		}
	}

    return 0;
fail:
    return -1;
}

void x264_analyse_free_costs( x264_t *h )
{
	int qp;
    for( qp = 0; qp <= QP_MAX_SPEC; qp++ )
    {
        if( h->cost_mv[qp] && ( qp == 0 || x264_lambda_tab[qp] != x264_lambda_tab[qp - 1] ) )
            x264_free( h->cost_mv[qp] - 2*4*512 );
    }
}

static void x264_mb_analyse_init( x264_t *h, x264_mb_analysis_t *a, int qp )
{
    a->i_mbrd = (h->param.analyse.i_subpel_refine >= 6) + (h->param.analyse.i_subpel_refine >= 8) + (h->param.analyse.i_subpel_refine >= 10);
    h->mb.b_deblock_rdo = h->param.analyse.i_subpel_refine >= 9 && h->sh.i_disable_deblocking_filter_idc != 1;
    a->b_early_terminate = h->param.analyse.i_subpel_refine < 11;

    /* init qp and lambda values */
    a->i_lambda = x264_lambda_tab[qp];
    h->mb.b_trellis = h->param.analyse.i_trellis > 1 && a->i_mbrd;

    /* init noise reduction */
    if( qp > QP_MAX_SPEC )
    {
    	/****** FIXME: Emergency mode denoising was removed by chenmin on 2013-2-4 ******/
        /*h->nr_offset = h->nr_offset_emergency[qp-QP_MAX_SPEC-1];*/
        h->nr_residual_sum = h->nr_residual_sum_buf[1];
        h->nr_count = h->nr_count_buf[1];
        h->mb.b_noise_reduction = 1;
        qp = QP_MAX_SPEC; /* Out-of-spec QPs are just used for calculating lambda values. */
    }
    else
    {
        h->nr_offset = h->nr_offset_denoise;
        h->nr_residual_sum = h->nr_residual_sum_buf[0];
        h->nr_count = h->nr_count_buf[0];
        h->mb.b_noise_reduction = 0;
    }

    a->i_qp = h->mb.i_qp = qp;
    h->mb.i_chroma_qp = h->chroma_qp_table[qp];
    h->mb.b_transform_8x8 = 0;

    /* I: Intra part */
    a->i_satd_i16x16 =
    a->i_satd_i4x4   =
    a->i_satd_chroma = COST_MAX;

    a->b_fast_intra = 0;
    a->b_avoid_topright = 0;
    h->mb.i_skip_intra = a->i_mbrd ? 2 : !h->param.analyse.i_trellis && !h->param.analyse.i_noise_reduction;

    /* II: Inter part P/B frame */
    if( h->sh.i_type != SLICE_TYPE_I )
    {
        int i_fmv_range = h->param.analyse.i_mv_range<<2;
        // limit motion search to a slightly smaller range than the theoretical limit,
        // since the search may go a few iterations past its given range
        int i_fpel_border = 6; // umh: 1 for diamond, 2 for octagon, 2 for hpel

        /* Calculate max allowed MV range */
        h->mb.mv_min[0] = (-(h->mb.i_mb_x<<4) - 24)<<2;
        h->mb.mv_max[0] = (((h->mb.i_mb_width - h->mb.i_mb_x - 1)<<4) + 24)<<2;
        /* limit mv_spel to 4 * mv_range */
        h->mb.mv_min_spel[0] = x264_clip3( h->mb.mv_min[0], -i_fmv_range, i_fmv_range-1 );
        h->mb.mv_max_spel[0] = x264_clip3( h->mb.mv_max[0], -i_fmv_range, i_fmv_range-1 );
        /* mv_fpel = mv_spel / 4 */
        h->mb.mv_min_fpel[0] = (h->mb.mv_min_spel[0]>>2) + i_fpel_border;
        h->mb.mv_max_fpel[0] = (h->mb.mv_max_spel[0]>>2) - i_fpel_border;
        if( h->mb.i_mb_x == 0 )
        {
			h->mb.mv_min[1] = (-(h->mb.i_mb_y<<4) - 24)<<2;
			h->mb.mv_max[1] = (((h->mb.i_mb_height - h->mb.i_mb_y - 1)<<4) + 24)<<2;
			h->mb.mv_min_spel[1] = x264_clip3( h->mb.mv_min[1], -i_fmv_range, i_fmv_range );
			h->mb.mv_max_spel[1] = x264_clip3( h->mb.mv_max[1], -i_fmv_range, i_fmv_range-1 );
			h->mb.mv_min_fpel[1] = (h->mb.mv_min_spel[1]>>2) + i_fpel_border;
			h->mb.mv_max_fpel[1] = (h->mb.mv_max_spel[1]>>2) - i_fpel_border;
        }

        a->l0.me16x16.cost =
        a->l0.i_cost8x8    =
        a->l0.i_cost16x8   =
        a->l0.i_cost8x16   = COST_MAX;

        /******************************************************************
         * Fast intra decision for P/B frame:                             *
         * 1. Always run in fast-intra mode for subme < 3                 *
         * 2. Or if any neighbour(left/top/topleft/topright) mb is intra, *
         *    then intra is very likely, don't run in fast-intra mode.    *
         ******************************************************************/
        if( a->b_early_terminate && h->mb.i_mb_xy - h->sh.i_first_mb > 4 )
        {
            if( h->mb.i_subpel_refine > 2 &&
              ( IS_INTRA( h->mb.i_mb_type_left[0] ) ||
                IS_INTRA( h->mb.i_mb_type_top ) ||
                IS_INTRA( h->mb.i_mb_type_topleft ) ||
                IS_INTRA( h->mb.i_mb_type_topright ) ))
            { /* intra is likely */ }
            else
            {
                a->b_fast_intra = 1;
            }
        }
        h->mb.b_skip_mc = 0;
        a->b_force_intra = 0;
    }
}

/* Prediction modes allowed for various combinations of neighbors. */
/* Terminated by a -1. */
/* In order, no neighbors, left, top, top/left, top/left/topleft */

/****************************************************
 * I_16x16 available modes depending on neighbors   *
 * 1. no neighbors exist: only DC_128 available     *
 * 2. left neighbors exist: DC_LEFT and H           *
 * 3. top neighbors exist: DC_TOP and V             *
 * 4. top/left neighbors exist: V, H, DC            *
 * 5. top/left/topleft neighbors exist: V, H, DC, P *
 ****************************************************/
static const int8_t i16x16_mode_available[5][5] =
{
    {I_PRED_16x16_DC_128, -1, -1, -1, -1},
    {I_PRED_16x16_DC_LEFT, I_PRED_16x16_H, -1, -1, -1},
    {I_PRED_16x16_DC_TOP, I_PRED_16x16_V, -1, -1, -1},
    {I_PRED_16x16_V, I_PRED_16x16_H, I_PRED_16x16_DC, -1, -1},
    {I_PRED_16x16_V, I_PRED_16x16_H, I_PRED_16x16_DC, I_PRED_16x16_P, -1},
};

/****************************************************
 * I_8x8c available modes depending on neighbors    *
 * 1. no neighbors exist: only DC_128 available     *
 * 2. left neighbors exist: DC_LEFT and H           *
 * 3. top neighbors exist: DC_TOP and V             *
 * 4. top/left neighbors exist: V, H, DC            *
 * 5. top/left/topleft neighbors exist: V, H, DC, P *
 ****************************************************/
static const int8_t chroma_mode_available[5][5] =
{
    {I_PRED_CHROMA_DC_128, -1, -1, -1, -1},
    {I_PRED_CHROMA_DC_LEFT, I_PRED_CHROMA_H, -1, -1, -1},
    {I_PRED_CHROMA_DC_TOP, I_PRED_CHROMA_V, -1, -1, -1},
    {I_PRED_CHROMA_V, I_PRED_CHROMA_H, I_PRED_CHROMA_DC, -1, -1},
    {I_PRED_CHROMA_V, I_PRED_CHROMA_H, I_PRED_CHROMA_DC, I_PRED_CHROMA_P, -1},
};

/***************************************************************************
 * I_4x4/I_8x8 available modes depending on neighbors                      *
 * 1. no neighbors exist: only DC_128 available                            *
 * 2. left neighbors exist: DC_LEFT, H, HU                                 *
 * 3. top neighbors exist: DC_TOP, V, DDL, VL                              *
 * 4. top/left neighbors exist: DC, H, V, DDL, VL, HU                      *
 * 5. top/left/topleft neighbors exist: DC, H, V, DDL, DDR, VR, HD, VL, HU *
 ***************************************************************************/
static const int8_t i4x4_mode_available[2][5][10] =
{
    {   /* I_4x4 mode that involves top-right neighbors */
        {I_PRED_4x4_DC_128, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {I_PRED_4x4_DC_LEFT, I_PRED_4x4_H, I_PRED_4x4_HU, -1, -1, -1, -1, -1, -1, -1},
        {I_PRED_4x4_DC_TOP, I_PRED_4x4_V, I_PRED_4x4_DDL, I_PRED_4x4_VL, -1, -1, -1, -1, -1, -1},
        {I_PRED_4x4_DC, I_PRED_4x4_H, I_PRED_4x4_V, I_PRED_4x4_DDL, I_PRED_4x4_VL, I_PRED_4x4_HU, -1, -1, -1, -1},
        {I_PRED_4x4_DC, I_PRED_4x4_H, I_PRED_4x4_V, I_PRED_4x4_DDL, I_PRED_4x4_DDR, I_PRED_4x4_VR, I_PRED_4x4_HD, I_PRED_4x4_VL, I_PRED_4x4_HU, -1},
    },
    {   /* I_4x4 mode that avoid top-right neighbors */
        {I_PRED_4x4_DC_128, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {I_PRED_4x4_DC_LEFT, I_PRED_4x4_H, I_PRED_4x4_HU, -1, -1, -1, -1, -1, -1, -1},
        {I_PRED_4x4_DC_TOP, I_PRED_4x4_V, -1, -1, -1, -1, -1, -1, -1, -1},
        {I_PRED_4x4_DC, I_PRED_4x4_H, I_PRED_4x4_V, I_PRED_4x4_HU, -1, -1, -1, -1, -1, -1},
        {I_PRED_4x4_DC, I_PRED_4x4_H, I_PRED_4x4_V, I_PRED_4x4_DDR, I_PRED_4x4_VR, I_PRED_4x4_HD, I_PRED_4x4_HU, -1},
    }
};

static ALWAYS_INLINE const int8_t *predict_16x16_mode_available( int i_neighbour )
{
    int idx = i_neighbour & (MB_TOP|MB_LEFT|MB_TOPLEFT);
    idx = (idx == (MB_TOP|MB_LEFT|MB_TOPLEFT)) ? 4 : idx & (MB_TOP|MB_LEFT);
    return i16x16_mode_available[idx];
}

static ALWAYS_INLINE const int8_t *predict_chroma_mode_available( int i_neighbour )
{
    int idx = i_neighbour & (MB_TOP|MB_LEFT|MB_TOPLEFT);
    idx = (idx == (MB_TOP|MB_LEFT|MB_TOPLEFT)) ? 4 : idx & (MB_TOP|MB_LEFT);
    return chroma_mode_available[idx];
}

static ALWAYS_INLINE const int8_t *predict_4x4_mode_available( int i_neighbour )
{
    int idx = i_neighbour & (MB_TOP|MB_LEFT|MB_TOPLEFT);
    idx = (idx == (MB_TOP|MB_LEFT|MB_TOPLEFT)) ? 4 : idx & (MB_TOP|MB_LEFT);
    return i4x4_mode_available[0][idx];
}

static void x264_mb_analyse_intra_chroma( x264_t *h, x264_mb_analysis_t *a )
{
	const int8_t *predict_mode;
	int i_mode, i_satd;

    if( a->i_satd_chroma < COST_MAX )
        return;

    predict_mode = predict_chroma_mode_available( h->mb.i_neighbour_intra );

    /* Prediction selection for chroma */
    if( predict_mode[3] >= 0 )
    {
    	/* all 4 I_8x8c predict modes are available. */
        int satdu[4], satdv[4];
        /* calculate the cost of 3 mode: H, V, DC */
        h->pixf.intra_mbcmp_x3_chroma( h->mb.pic.p_fenc[1], h->mb.pic.p_fdec[1], satdu );
        h->pixf.intra_mbcmp_x3_chroma( h->mb.pic.p_fenc[2], h->mb.pic.p_fdec[2], satdv );
        /* do the pridiction of I_PRED_CHROMA_P and calculate the cost */
        h->predict_chroma[I_PRED_CHROMA_P]( h->mb.pic.p_fdec[1] );
        h->predict_chroma[I_PRED_CHROMA_P]( h->mb.pic.p_fdec[2] );
        satdu[I_PRED_CHROMA_P] = h->pixf.mbcmp[PIXEL_8x8]( h->mb.pic.p_fdec[1], FDEC_STRIDE, h->mb.pic.p_fenc[1], FENC_STRIDE );
        satdv[I_PRED_CHROMA_P] = h->pixf.mbcmp[PIXEL_8x8]( h->mb.pic.p_fdec[2], FDEC_STRIDE, h->mb.pic.p_fenc[2], FENC_STRIDE );

        /* compare costs (sum of SATD of U and V + lambda * R) of 4 mode and choose the best */
        a->i_satd_chroma_dir[2] = satdu[2] + satdv[2] + a->i_lambda * 3; /* bs_size_ue(2) */
        a->i_satd_chroma_dir[1] = satdu[1] + satdv[1] + a->i_lambda * 3; /* bs_size_ue(1) */
        a->i_satd_chroma_dir[0] = satdu[0] + satdv[0] + a->i_lambda * 1; /* bs_size_ue(0) */
        a->i_satd_chroma_dir[3] = satdu[3] + satdv[3] + a->i_lambda * 5; /* bs_size_ue(3) */
        COPY2_IF_LT( a->i_satd_chroma, a->i_satd_chroma_dir[2], a->i_predict8x8chroma, 2 );
        COPY2_IF_LT( a->i_satd_chroma, a->i_satd_chroma_dir[1], a->i_predict8x8chroma, 1 );
        COPY2_IF_LT( a->i_satd_chroma, a->i_satd_chroma_dir[0], a->i_predict8x8chroma, 0 );
        COPY2_IF_LT( a->i_satd_chroma, a->i_satd_chroma_dir[3], a->i_predict8x8chroma, 3 );
    }
    else
    {
    	/* not all 4 I_8x8c predict modes are available */
        for( ; *predict_mode >= 0; predict_mode++ )
        {
            i_mode = *predict_mode;
            /* we do the prediction */
            h->predict_chroma[i_mode]( h->mb.pic.p_fdec[1] ); /* U */
            h->predict_chroma[i_mode]( h->mb.pic.p_fdec[2] ); /* V */
            /* we calculate the cost: sum of SATD of U and V + lambda * R  */
            i_satd = h->pixf.mbcmp[PIXEL_8x8]( h->mb.pic.p_fdec[1], FDEC_STRIDE, h->mb.pic.p_fenc[1], FENC_STRIDE ) +
                     h->pixf.mbcmp[PIXEL_8x8]( h->mb.pic.p_fdec[2], FDEC_STRIDE, h->mb.pic.p_fenc[2], FENC_STRIDE ) +
                     a->i_lambda * bs_size_ue( x264_mb_chroma_pred_mode_fix[i_mode] );
            a->i_satd_chroma_dir[i_mode] = i_satd;
            COPY2_IF_LT( a->i_satd_chroma, i_satd, a->i_predict8x8chroma, i_mode );
        }
    }

    h->mb.i_chroma_pred_mode = a->i_predict8x8chroma;
}

static void x264_mb_analyse_intra( x264_t *h, x264_mb_analysis_t *a, int i_satd_inter )
{
	/* flag of mb Intra predict for Intra slice or Inter slice. */
    const unsigned int flags = h->sh.i_type == SLICE_TYPE_I ? h->param.analyse.intra : h->param.analyse.inter;
    pixel *p_src = h->mb.pic.p_fenc[0];
    pixel *p_dst = h->mb.pic.p_fdec[0];
    static const int8_t intra_analysis_shortcut[2][2][2][5] =
    {
    	/* avoid top-right: no */
        {{{I_PRED_4x4_HU, -1, -1, -1, -1},                                      /* favor horizontal */
          {I_PRED_4x4_DDL, I_PRED_4x4_VL, -1, -1, -1}},                         /* favor vertical   */
         {{I_PRED_4x4_DDR, I_PRED_4x4_HD, I_PRED_4x4_HU, -1, -1},               /* favor horizontal */
          {I_PRED_4x4_DDL, I_PRED_4x4_DDR, I_PRED_4x4_VR, I_PRED_4x4_VL, -1}}}, /* favor vertical   */
        /* avoid top-right: yes */
        {{{I_PRED_4x4_HU, -1, -1, -1, -1},                                      /* favor horizontal */
          {-1, -1, -1, -1, -1}},                                                /* favor vertical   */
         {{I_PRED_4x4_DDR, I_PRED_4x4_HD, I_PRED_4x4_HU, -1, -1},               /* favor horizontal */
          {I_PRED_4x4_DDR, I_PRED_4x4_VR, -1, -1, -1}}},                        /* favor vertical   */
    };

    int idx;
    int lambda = a->i_lambda;
    int i_mode, i_satd;

    /*---------------- Try all mode and calculate their score ---------------*/

    /* 16x16 prediction selection */
    const int8_t *predict_mode = predict_16x16_mode_available( h->mb.i_neighbour_intra );
    int i16x16_thresh = a->b_fast_intra ? i_satd_inter : COST_MAX;

    if( predict_mode[3] >= 0 )
    {
    	/* all 4 I_16x16 predict modes are available. */
        h->pixf.intra_mbcmp_x3_16x16( p_src, p_dst, a->i_satd_i16x16_dir );
        a->i_satd_i16x16_dir[0] += lambda * 1; /* bs_size_ue(0) */
        a->i_satd_i16x16_dir[1] += lambda * 3; /* bs_size_ue(1) */
        a->i_satd_i16x16_dir[2] += lambda * 3; /* bs_size_ue(2) */
        COPY2_IF_LT( a->i_satd_i16x16, a->i_satd_i16x16_dir[0], a->i_predict16x16, 0 );
        COPY2_IF_LT( a->i_satd_i16x16, a->i_satd_i16x16_dir[1], a->i_predict16x16, 1 );
        COPY2_IF_LT( a->i_satd_i16x16, a->i_satd_i16x16_dir[2], a->i_predict16x16, 2 );

        /* Plane is expensive, so don't check it unless one of the previous modes was useful. */
        if( a->i_satd_i16x16 <= i16x16_thresh )
        {
            h->predict_16x16[I_PRED_16x16_P]( p_dst );
            a->i_satd_i16x16_dir[I_PRED_16x16_P] = h->pixf.mbcmp[PIXEL_16x16]( p_dst, FDEC_STRIDE, p_src, FENC_STRIDE );
            a->i_satd_i16x16_dir[I_PRED_16x16_P] += lambda * 5; /* bs_size_ue(3) */
            COPY2_IF_LT( a->i_satd_i16x16, a->i_satd_i16x16_dir[I_PRED_16x16_P], a->i_predict16x16, 3 );
        }
    }
    else
    {
    	/* not all 4 I_16x16 predict modes are available. */
        for( ; *predict_mode >= 0; predict_mode++ )
        {
            i_mode = *predict_mode;
            h->predict_16x16[i_mode]( p_dst );
            i_satd = h->pixf.mbcmp[PIXEL_16x16]( p_dst, FDEC_STRIDE, p_src, FENC_STRIDE ) +
                     lambda * bs_size_ue( x264_mb_pred_mode16x16_fix[i_mode] );
            COPY2_IF_LT( a->i_satd_i16x16, i_satd, a->i_predict16x16, i_mode );
            a->i_satd_i16x16_dir[i_mode] = i_satd;
        }
    }

    /* Reaching thresh indicates Intra predict is unlikely better than Inter predict, return now. */
    if( a->i_satd_i16x16 > i16x16_thresh )
        return;

    /* 4x4 prediction selection */
    if( flags & X264_ANALYSE_I4x4 )
    {
        int i_cost = lambda * 40; /* 24 from JVT (SATD0), 16 from base predmode costs */
        int i_4x4_mode_cost = lambda * 3;
        int i_satd_thresh = a->b_early_terminate ? X264_MIN( i_satd_inter, a->i_satd_i16x16 ) : COST_MAX;
        h->mb.i_cbp_luma = 0;

        /* loop at most 16 times for each block 4x4 */
        for( idx = 0;; idx++ )
        {
            pixel *p_src_by = p_src + block_idx_xy_fenc[idx];
            pixel *p_dst_by = p_dst + block_idx_xy_fdec[idx];
            int i_best = COST_MAX;
            int i_pred_mode = x264_mb_predict_intra4x4_mode( h, idx );

            predict_mode = predict_4x4_mode_available( h->mb.i_neighbour4[idx] );

            /******************************************
             *  emulate missing topright samples      *
             *  block idx: [3, 7, 11, 13, 15]         *
             *                                        *
             *  block 4x4 process order:              *
             *  0  1  | 4  5                          *
             *  2  3  | 6  7                          *
             *  -------------                         *
             *  8  9  | 12 13                         *
             *  10 11 | 14 15                         *
             ******************************************/
            if( (h->mb.i_neighbour4[idx] & (MB_TOPRIGHT|MB_TOP)) == MB_TOP )
                MPIXEL_X4( &p_dst_by[4 - FDEC_STRIDE] ) = PIXEL_SPLAT_X4( p_dst_by[3 - FDEC_STRIDE] );

			/**********************************************************************************
			 * intra_mbcmp_x9_4x4 is not implemented, so take shortcuts for fast compute.     *
			 * If DC/H/V is all available, calculate them and compare the cost of H and V.    *
			 * If H(horizontal) is better, DDL/DDR/HU/HD is analysed later, VL/VR is ignored. *
			 * If V(vertical) is better, DDL/DDR/VL/VR is analysed later, HU/HD is ignored.   *
			 **********************************************************************************/
			if( predict_mode[5] >= 0 )
			{
				int satd[9];
				int favor_vertical;
				/* calculate cost of V/H/DC modes. */
				h->pixf.intra_mbcmp_x3_4x4( p_src_by, p_dst_by, satd );
				favor_vertical = satd[I_PRED_4x4_H] > satd[I_PRED_4x4_V];
				/* calculate cost of DDL/DDR/VR/VL or DDL/DDR/HD/HU  */
				if( predict_mode[8] >= 0 )
				{
					if( favor_vertical )
						h->pixf.intra_mbcmp_x4_4x4_v( p_src_by, p_dst_by, satd );
					else
						h->pixf.intra_mbcmp_x4_4x4_h( p_src_by, p_dst_by, satd );
				}
				satd[i_pred_mode] -= i_4x4_mode_cost;
				i_best = satd[I_PRED_4x4_DC]; a->i_predict4x4[idx] = I_PRED_4x4_DC;
				COPY2_IF_LT( i_best, satd[I_PRED_4x4_H], a->i_predict4x4[idx], I_PRED_4x4_H );
				COPY2_IF_LT( i_best, satd[I_PRED_4x4_V], a->i_predict4x4[idx], I_PRED_4x4_V );
				if( predict_mode[8] >= 0 )
				{
					COPY2_IF_LT( i_best, satd[I_PRED_4x4_DDL], a->i_predict4x4[idx], I_PRED_4x4_DDL );
					COPY2_IF_LT( i_best, satd[I_PRED_4x4_DDR], a->i_predict4x4[idx], I_PRED_4x4_DDR );
					if( favor_vertical )
					{
						COPY2_IF_LT( i_best, satd[I_PRED_4x4_VR], a->i_predict4x4[idx], I_PRED_4x4_VR );
						COPY2_IF_LT( i_best, satd[I_PRED_4x4_VL], a->i_predict4x4[idx], I_PRED_4x4_VL );
					}
					else
					{
						COPY2_IF_LT( i_best, satd[I_PRED_4x4_HD], a->i_predict4x4[idx], I_PRED_4x4_HD );
						COPY2_IF_LT( i_best, satd[I_PRED_4x4_HU], a->i_predict4x4[idx], I_PRED_4x4_HU );
					}
					/*At this point, we have analysed all possible modes. */
					predict_mode = NULL;
				}
				else
				{
					/* Take analysis shortcuts: don't analyse modes that are too far away direction-wise from the favored mode. */
					predict_mode = intra_analysis_shortcut[a->b_avoid_topright][0][favor_vertical];
				}
			}

			if( predict_mode && i_best > 0 )
			{
				for( ; *predict_mode >= 0; predict_mode++ )
				{
					i_mode = *predict_mode;
					h->predict_4x4[i_mode]( p_dst_by );
					i_satd = h->pixf.mbcmp[PIXEL_4x4]( p_dst_by, FDEC_STRIDE, p_src_by, FENC_STRIDE );
					if( i_pred_mode == x264_mb_pred_mode4x4_fix(i_mode) )
					{
						i_satd -= i_4x4_mode_cost;
						if( i_satd <= 0 )
						{
							i_best = i_satd;
							a->i_predict4x4[idx] = i_mode;
							break;
						}
					}

					COPY2_IF_LT( i_best, i_satd, a->i_predict4x4[idx], i_mode );
				}
			}

			i_cost += i_best + i_4x4_mode_cost;
			/* Reaching thresh indicates Intra predict is unlikely better than Inter predict, return now. */
			if( i_cost > i_satd_thresh || idx == 15 )
				break;

			/* do predict here by chosen mode to prepare for later encoding */
			h->predict_4x4[a->i_predict4x4[idx]]( p_dst_by );
			h->mb.cache.intra4x4_pred_mode[x264_scan8[idx]] = a->i_predict4x4[idx];

            /* we need to encode this 4x4 block now (for next ones) */
            x264_mb_encode_i4x4( h, idx, a->i_qp, a->i_predict4x4[idx], 0 );
        }
        if( idx == 15 )
        {
            a->i_satd_i4x4 = i_cost;
            if( h->mb.i_skip_intra )
            {
                h->mc.copy[PIXEL_16x16]( h->mb.pic.i4x4_fdec_buf, 16, p_dst, FDEC_STRIDE, 16 );
                h->mb.pic.i4x4_nnz_buf[0] = M32( &h->mb.cache.non_zero_count[x264_scan8[ 0]] );
                h->mb.pic.i4x4_nnz_buf[1] = M32( &h->mb.cache.non_zero_count[x264_scan8[ 2]] );
                h->mb.pic.i4x4_nnz_buf[2] = M32( &h->mb.cache.non_zero_count[x264_scan8[ 8]] );
                h->mb.pic.i4x4_nnz_buf[3] = M32( &h->mb.cache.non_zero_count[x264_scan8[10]] );
                h->mb.pic.i4x4_cbp = h->mb.i_cbp_luma;
            }
        }
        else
            a->i_satd_i4x4 = COST_MAX;
    }
}

#define LOAD_FENC(m, src, xoff, yoff) \
{ \
    (m)->p_cost_mv = a->p_cost_mv; \
    (m)->i_stride[0] = h->mb.pic.i_stride[0]; \
    (m)->i_stride[1] = h->mb.pic.i_stride[1]; \
    (m)->i_stride[2] = h->mb.pic.i_stride[2]; \
    (m)->p_fenc[0] = &(src)[0][(xoff)+(yoff)*FENC_STRIDE]; \
    (m)->p_fenc[1] = &(src)[1][((xoff)>>CHROMA_H_SHIFT)+((yoff)>>CHROMA_V_SHIFT)*FENC_STRIDE]; \
    (m)->p_fenc[2] = &(src)[2][((xoff)>>CHROMA_H_SHIFT)+((yoff)>>CHROMA_V_SHIFT)*FENC_STRIDE]; \
}

#define LOAD_HPELS(m, src, list, ref, xoff, yoff) \
{ \
    (m)->p_fref_w = (m)->p_fref[0] = &(src)[0][(xoff)+(yoff)*(m)->i_stride[0]]; \
    (m)->p_fref[1] = &(src)[1][(xoff)+(yoff)*(m)->i_stride[0]]; \
    (m)->p_fref[2] = &(src)[2][(xoff)+(yoff)*(m)->i_stride[0]]; \
    (m)->p_fref[3] = &(src)[3][(xoff)+(yoff)*(m)->i_stride[0]]; \
    (m)->p_fref[4] = &(src)[4][(xoff)+((yoff)>>CHROMA_V_SHIFT)*(m)->i_stride[1]]; \
    (m)->weight = x264_weight_none; \
    (m)->i_ref = ref; \
}

static void x264_mb_analyse_inter_p16x16( x264_t *h, x264_mb_analysis_t *a )
{
    x264_me_t m;
    int i_ref, i_mvc;
    ALIGNED_4( int16_t mvc[8][2] );
    int i_halfpel_thresh = INT_MAX;
    int *p_halfpel_thresh = (a->b_early_terminate && h->mb.pic.i_fref[0]>1) ? &i_halfpel_thresh : NULL;

    /* 16x16 Search on all ref frame */
    m.i_pixel = PIXEL_16x16;
    LOAD_FENC( &m, h->mb.pic.p_fenc, 0, 0 );

    a->l0.me16x16.cost = INT_MAX;
    /* try all reference frames of l0, from nearest to farest */
    for( i_ref = 0; i_ref < h->mb.pic.i_fref[0]; i_ref++ )
    {
        m.i_ref_cost = a->p_cost_ref[0][i_ref];
        i_halfpel_thresh -= m.i_ref_cost;

        /* search with ref */
        LOAD_HPELS( &m, h->mb.pic.p_fref[0][i_ref], 0, i_ref, 0, 0 );

        /* motion vector predict only for 16x16 */
        x264_mb_predict_mv_16x16( h, 0, i_ref, m.mvp );
        if( h->mb.ref_blind_dupe == i_ref )
        {
            CP32( m.mv, a->l0.mvc[0][0] );
            x264_me_refine_qpel_refdupe( h, &m, p_halfpel_thresh );
        }
        else
        {
        	/* find more mv candidates and save it to mvc array */
            x264_mb_predict_mv_ref16x16( h, 0, i_ref, mvc, &i_mvc );
            x264_me_search_ref( h, &m, mvc, i_mvc, p_halfpel_thresh );
        }

        /* save mv for predicting neighbors */
        CP32( h->mb.mvr[0][i_ref][h->mb.i_mb_xy], m.mv );
        CP32( a->l0.mvc[i_ref][0], m.mv );

        /*********************************************************************
         * Early termination conditions are used to speed up mode decision.  *
         *                                                                   *
         * We search the 16x16 partition for the immediate previous frame    *
         * (P16x16ref0). If P16x16ref0's MV is equal to P-Skip's MV, and the *
         * distortion caused by leaving it with no residual is sufficiently  *
         * small (see JVT-??? for the threshold), then this macroblock is    *
         * assigned to be P-Skip and any further analysis is avoided.        *
         *                                                                   *
         * SSD threshold would probably be better than SATD                  *
         *********************************************************************/
        if( i_ref == 0 && a->b_try_skip
            && m.cost-m.cost_mv < 300*a->i_lambda
            &&  abs(m.mv[0]-h->mb.cache.pskip_mv[0])
              + abs(m.mv[1]-h->mb.cache.pskip_mv[1]) <= 1
            && x264_macroblock_probe_pskip( h ) )
        {
            h->mb.i_type = P_SKIP;
            x264_analyse_update_cache( h, a );
            /*assert( h->mb.cache.pskip_mv[1] <= h->mb.mv_max_spel[1] );*/
            return;
        }

        m.cost += m.i_ref_cost;
        i_halfpel_thresh += m.i_ref_cost;

        /* if cost of current me is lower, save it to a->l0.me16x16 */
        if( m.cost < a->l0.me16x16.cost )
            h->mc.memcpy_aligned( &a->l0.me16x16, &m, sizeof(x264_me_t) );
    }

    x264_macroblock_cache_ref( h, 0, 0, 4, 4, 0, a->l0.me16x16.i_ref );
    /*assert( a->l0.me16x16.mv[1] <= h->mb.mv_max_spel[1] );*/

    h->mb.i_type = P_L0;
}

static void x264_mb_analyse_inter_p8x8( x264_t *h, x264_mb_analysis_t *a )
{
    /* Duplicate refs are rarely useful in p8x8 due to the high cost of the
     * reference frame flags.  Thus, if we're not doing mixedrefs, just
     * don't bother analysing the dupes. */
    const int i_ref = h->mb.ref_blind_dupe == a->l0.me16x16.i_ref ? 0 : a->l0.me16x16.i_ref;
    const int i_ref_cost = h->param.b_cabac || i_ref ? a->p_cost_ref[0][i_ref] : 0;
    pixel **p_fenc = h->mb.pic.p_fenc;
    int i_mvc;
    int16_t (*mvc)[2] = a->l0.mvc[i_ref];
	int i;

    /* XXX Needed for x264_mb_predict_mv */
    h->mb.i_partition = D_8x8;

    i_mvc = 1;
    CP32( mvc[0], a->l0.me16x16.mv );

    /* loop for each block 8x8 of current mb */
    for( i = 0; i < 4; i++ )
    {
        x264_me_t *m = &a->l0.me8x8[i];
        int x8 = i&1;
        int y8 = i>>1;

        m->i_pixel = PIXEL_8x8;
        m->i_ref_cost = i_ref_cost;

        LOAD_FENC( m, p_fenc, x8<<3, y8<<3 );
        LOAD_HPELS( m, h->mb.pic.p_fref[0][i_ref], 0, i_ref, x8<<3, y8<<3 );

        /* do the mv predict here */
        x264_mb_predict_mv( h, 0, i<<2, 2, m->mvp );
        x264_me_search( h, m, mvc, i_mvc );

        x264_macroblock_cache_mv_ptr( h, x8<<1, y8<<1, 2, 2, 0, m->mv );

        CP32( mvc[i_mvc++], m->mv );

        a->i_satd8x8[0][i] = m->cost - m->cost_mv;

        /* mb type cost */
        m->cost += i_ref_cost;
    }

    /* accumulate 4 b8x8 costs */
    a->l0.i_cost8x8 = a->l0.me8x8[0].cost + a->l0.me8x8[1].cost +
                      a->l0.me8x8[2].cost + a->l0.me8x8[3].cost;
    /* theoretically this should include 4*ref_cost,
     * but 3 seems a better approximation of cabac. */
    if( h->param.b_cabac )
        a->l0.i_cost8x8 -= i_ref_cost;
    h->mb.i_sub_partition[0] = h->mb.i_sub_partition[1] =
    h->mb.i_sub_partition[2] = h->mb.i_sub_partition[3] = D_L0_8x8;
}

static void x264_mb_analyse_inter_p16x8( x264_t *h, x264_mb_analysis_t *a, int i_best_satd )
{
    x264_me_t m;
    pixel **p_fenc = h->mb.pic.p_fenc;
    ALIGNED_4( int16_t mvc[3][2] );
	int i, j;

    /* XXX Needed for x264_mb_predict_mv */
    h->mb.i_partition = D_16x8;

    /* loop for each block 16x8 of current mb */
    for( i = 0; i < 2; i++ )
    {
        x264_me_t *l0m = &a->l0.me16x8[i];
        const int minref = X264_MIN( a->l0.me8x8[i<<1].i_ref, a->l0.me8x8[(i<<1)+1].i_ref );
        const int maxref = X264_MAX( a->l0.me8x8[i<<1].i_ref, a->l0.me8x8[(i<<1)+1].i_ref );
        const int ref8[2] = { minref, maxref };
        const int i_ref8s = ( ref8[0] == ref8[1] ) ? 1 : 2;

        m.i_pixel = PIXEL_16x8;

        LOAD_FENC( &m, p_fenc, 0, i<<3 );
        l0m->cost = INT_MAX;
        for( j = 0; j < i_ref8s; j++ )
        {
            const int i_ref = ref8[j];
            m.i_ref_cost = a->p_cost_ref[0][i_ref];

            /* if we skipped the 16x16 predictor, we wouldn't have to copy anything... */
            CP32( mvc[0], a->l0.mvc[i_ref][0] );
            CP32( mvc[1], a->l0.mvc[i_ref][(i<<1)+1] );
            CP32( mvc[2], a->l0.mvc[i_ref][(i<<1)+2] );

            LOAD_HPELS( &m, h->mb.pic.p_fref[0][i_ref], 0, i_ref, 0, i<<3 );

            x264_macroblock_cache_ref( h, 0, i<<1, 4, 2, 0, i_ref );
            /* do the mv predict here */
            x264_mb_predict_mv( h, 0, i<<3, 4, m.mvp );
            /* We can only take this shortcut if the first search was performed on ref0. */
            if( h->mb.ref_blind_dupe == i_ref && !ref8[0] )
            {
                /* We can just leave the MV from the previous ref search. */
                x264_me_refine_qpel_refdupe( h, &m, NULL );
            }
            else
                x264_me_search( h, &m, mvc, 3 );

            m.cost += m.i_ref_cost;

            if( m.cost < l0m->cost )
                h->mc.memcpy_aligned( l0m, &m, sizeof(x264_me_t) );
        }

        /* Early termination based on the current SATD score of partition[0]
           plus the estimated SATD score of partition[1] */
        if( a->b_early_terminate && (!i && l0m->cost + a->i_cost_est16x8[1] > i_best_satd) )
        {
            a->l0.i_cost16x8 = COST_MAX;
            return;
        }

        x264_macroblock_cache_mv_ptr( h, 0, i<<1, 4, 2, 0, l0m->mv );
        x264_macroblock_cache_ref( h, 0, i<<1, 4, 2, 0, l0m->i_ref );
    }

    /* accumulate 2 b16x8 costs */
    a->l0.i_cost16x8 = a->l0.me16x8[0].cost + a->l0.me16x8[1].cost;
}

static void x264_mb_analyse_inter_p8x16( x264_t *h, x264_mb_analysis_t *a, int i_best_satd )
{
    x264_me_t m;
    pixel **p_fenc = h->mb.pic.p_fenc;
    ALIGNED_4( int16_t mvc[3][2] );
	int i, j;

    /* XXX Needed for x264_mb_predict_mv */
    h->mb.i_partition = D_8x16;

    /* loop for each block 8x16 of current mb */
    for( i = 0; i < 2; i++ )
    {
        x264_me_t *l0m = &a->l0.me8x16[i];
        const int minref = X264_MIN( a->l0.me8x8[i].i_ref, a->l0.me8x8[i+2].i_ref );
        const int maxref = X264_MAX( a->l0.me8x8[i].i_ref, a->l0.me8x8[i+2].i_ref );
        const int ref8[2] = { minref, maxref };
        const int i_ref8s = ( ref8[0] == ref8[1] ) ? 1 : 2;

        m.i_pixel = PIXEL_8x16;

        LOAD_FENC( &m, p_fenc, i<<3, 0 );
        l0m->cost = INT_MAX;
        for( j = 0; j < i_ref8s; j++ )
        {
            const int i_ref = ref8[j];
            m.i_ref_cost = a->p_cost_ref[0][i_ref];

            CP32( mvc[0], a->l0.mvc[i_ref][0] );
            CP32( mvc[1], a->l0.mvc[i_ref][i+1] );
            CP32( mvc[2], a->l0.mvc[i_ref][i+3] );

            LOAD_HPELS( &m, h->mb.pic.p_fref[0][i_ref], 0, i_ref, i<<3, 0 );

            x264_macroblock_cache_ref( h, i<<1, 0, 2, 4, 0, i_ref );
            x264_mb_predict_mv( h, 0, i<<2, 2, m.mvp );
            /* We can only take this shortcut if the first search was performed on ref0. */
            if( h->mb.ref_blind_dupe == i_ref && !ref8[0] )
            {
                /* We can just leave the MV from the previous ref search. */
                x264_me_refine_qpel_refdupe( h, &m, NULL );
            }
            else
                x264_me_search( h, &m, mvc, 3 );

            m.cost += m.i_ref_cost;

            if( m.cost < l0m->cost )
                h->mc.memcpy_aligned( l0m, &m, sizeof(x264_me_t) );
        }

        /* Early termination based on the current SATD score of partition[0]
           plus the estimated SATD score of partition[1] */
        if( a->b_early_terminate && (!i && l0m->cost + a->i_cost_est8x16[1] > i_best_satd) )
        {
            a->l0.i_cost8x16 = COST_MAX;
            return;
        }

        x264_macroblock_cache_mv_ptr( h, i<<1, 0, 2, 4, 0, l0m->mv );
        x264_macroblock_cache_ref( h, i<<1, 0, 2, 4, 0, l0m->i_ref );
    }

    /* accumulate 2 b8x16 costs */
    a->l0.i_cost8x16 = a->l0.me8x16[0].cost + a->l0.me8x16[1].cost;
}

/*****************************************************************************
 * x264_macroblock_analyse:
 *****************************************************************************/
void x264_macroblock_analyse( x264_t *h )
{
    x264_mb_analysis_t analysis;
    int i_cost = COST_MAX;
	int i;

#if 0
    h->mb.i_qp = x264_ratecontrol_mb_qp( h );
    /* If the QP of this MB is within 1 of the previous MB, code the same QP as the previous MB,
     * to lower the bit cost of the qp_delta.  Don't do this if QPRD is enabled. */
    if( h->param.rc.i_aq_mode && h->param.analyse.i_subpel_refine < 10 && abs(h->mb.i_qp - h->mb.i_last_qp) == 1 )
        h->mb.i_qp = h->mb.i_last_qp;

    if( h->param.analyse.b_mb_info )
        h->fdec->effective_qp[h->mb.i_mb_xy] = h->mb.i_qp; /* Store the real analysis QP. */
#endif

    /* init analyse for current mb, using qp of current frame */
    x264_mb_analyse_init( h, &analysis, h->sh.i_qp );

    /*--------------------------- Do the analysis ---------------------------*/
    if( h->sh.i_type == SLICE_TYPE_I )
    {
        /* analyse intra roughly by sad/satd */
        x264_mb_analyse_intra( h, &analysis, COST_MAX );

        /* compare and choose the lowest cost between I_16x16 and I_4x4 */
        i_cost = analysis.i_satd_i16x16; h->mb.i_type = I_16x16;
        COPY2_IF_LT( i_cost, analysis.i_satd_i4x4, h->mb.i_type, I_4x4 );
    }
    else if( h->sh.i_type == SLICE_TYPE_P )
    {
        int b_skip = 0;

        h->mc.prefetch_ref( h->mb.pic.p_fref[0][0][h->mb.i_mb_x&3], h->mb.pic.i_stride[0], 0 );

		/* Fast P_SKIP detection */
        analysis.b_try_skip = 0;
		if( h->param.analyse.b_fast_pskip )
		{
			if( h->param.analyse.i_subpel_refine >= 3 ) /* try probe skip later in p16x16 */
				analysis.b_try_skip = 1;
			else if( h->mb.i_mb_type_left[0] == P_SKIP || /* try probe skip if any neighbor mode is skip */
					 h->mb.i_mb_type_top == P_SKIP ||
					 h->mb.i_mb_type_topleft == P_SKIP ||
					 h->mb.i_mb_type_topright == P_SKIP )
				b_skip = x264_macroblock_probe_pskip( h );
		}

        h->mc.prefetch_ref( h->mb.pic.p_fref[0][0][h->mb.i_mb_x&3], h->mb.pic.i_stride[0], 1 );

        if( b_skip )
        {
            h->mb.i_type = P_SKIP;
            h->mb.i_partition = D_16x16;

            /* Set up MVs for future predictors */
            for( i = 0; i < h->mb.pic.i_fref[0]; i++ )
                M32( h->mb.mvr[0][i][h->mb.i_mb_xy] ) = 0;
        }
        else
        {
            const unsigned int flags = h->param.analyse.inter;
            int i_type;
            int i_partition;
            int i_thresh16x8;

            /* initialize an array of lambda*nbits for all possible mvs */
            analysis.p_cost_mv = h->cost_mv[analysis.i_qp];
            analysis.p_cost_ref[0] = x264_cost_ref[analysis.i_qp][x264_clip3(h->sh.i_num_ref_idx_l0_active-1,0,2)];
            analysis.p_cost_ref[1] = x264_cost_ref[analysis.i_qp][x264_clip3(h->sh.i_num_ref_idx_l1_active-1,0,2)];

            /* do the analysis of P16x16 */
            x264_mb_analyse_inter_p16x16( h, &analysis );

            if( h->mb.i_type == P_SKIP )
            {
                for( i = 1; i < h->mb.pic.i_fref[0]; i++ )
                    M32( h->mb.mvr[0][i][h->mb.i_mb_xy] ) = 0;
                return;
            }

            /* do the analysis of P8x8 */
            if( flags & X264_ANALYSE_PSUB16x16 )
                x264_mb_analyse_inter_p8x8( h, &analysis );

            /* Select best inter mode */
            i_type = P_L0;
            i_partition = D_16x16;
            i_cost = analysis.l0.me16x16.cost;

            if( ( flags & X264_ANALYSE_PSUB16x16 ) && (!analysis.b_early_terminate ||
                analysis.l0.i_cost8x8 < analysis.l0.me16x16.cost) )
            {
                i_type = P_8x8;
                i_partition = D_8x8;
                i_cost = analysis.l0.i_cost8x8;
            }

            /****************************************************************
             * The cost of P16x8 and P8x16 are estimated as being equal to  *
             * (SATD of P8x8) + 0.5*(bit cost of P8x8). If this estimate is *
             * worse than the known SATD0 cost of P16x16, then skip P16x8   *
             * and P8x16. Otherwise search them using only the reference    *
             * frames chosen for the two co-located 8x8 partitions.         *
             ****************************************************************/
            i_thresh16x8 = analysis.l0.me8x8[1].cost_mv + analysis.l0.me8x8[2].cost_mv;
            if( ( flags & X264_ANALYSE_PSUB16x16 ) && (!analysis.b_early_terminate ||
                analysis.l0.i_cost8x8 < analysis.l0.me16x16.cost + i_thresh16x8) )
            {
                int i_avg_mv_ref_cost = (analysis.l0.me8x8[2].cost_mv + analysis.l0.me8x8[2].i_ref_cost
                                      + analysis.l0.me8x8[3].cost_mv + analysis.l0.me8x8[3].i_ref_cost + 1) >> 1;
                analysis.i_cost_est16x8[1] = analysis.i_satd8x8[0][2] + analysis.i_satd8x8[0][3] + i_avg_mv_ref_cost;

                x264_mb_analyse_inter_p16x8( h, &analysis, i_cost );
                COPY3_IF_LT( i_cost, analysis.l0.i_cost16x8, i_type, P_L0, i_partition, D_16x8 );

                i_avg_mv_ref_cost = (analysis.l0.me8x8[1].cost_mv + analysis.l0.me8x8[1].i_ref_cost
                                  + analysis.l0.me8x8[3].cost_mv + analysis.l0.me8x8[3].i_ref_cost + 1) >> 1;
                analysis.i_cost_est8x16[1] = analysis.i_satd8x8[0][1] + analysis.i_satd8x8[0][3] + i_avg_mv_ref_cost;

                x264_mb_analyse_inter_p8x16( h, &analysis, i_cost );
                COPY3_IF_LT( i_cost, analysis.l0.i_cost8x16, i_type, P_L0, i_partition, D_8x16 );
            }

            h->mb.i_partition = i_partition;

            /* refine qpel */
            if( i_partition == D_16x16 )
            {
                x264_me_refine_qpel( h, &analysis.l0.me16x16 );
                i_cost = analysis.l0.me16x16.cost;
            }
            else if( i_partition == D_16x8 )
            {
                x264_me_refine_qpel( h, &analysis.l0.me16x8[0] );
                x264_me_refine_qpel( h, &analysis.l0.me16x8[1] );
                i_cost = analysis.l0.me16x8[0].cost + analysis.l0.me16x8[1].cost;
            }
            else if( i_partition == D_8x16 )
            {
                x264_me_refine_qpel( h, &analysis.l0.me8x16[0] );
                x264_me_refine_qpel( h, &analysis.l0.me8x16[1] );
                i_cost = analysis.l0.me8x16[0].cost + analysis.l0.me8x16[1].cost;
            }
            else if( i_partition == D_8x8 )
            {
            	x264_me_refine_qpel( h, &analysis.l0.me8x8[0] );
            	x264_me_refine_qpel( h, &analysis.l0.me8x8[1] );
            	x264_me_refine_qpel( h, &analysis.l0.me8x8[2] );
            	x264_me_refine_qpel( h, &analysis.l0.me8x8[3] );
            	i_cost = analysis.l0.me8x8[0].cost + analysis.l0.me8x8[1].cost
            			+ analysis.l0.me8x8[2].cost + analysis.l0.me8x8[3].cost;
            }

            /* Intra predict */
#if 0
            x264_mb_analyse_intra( h, &analysis, i_cost );
            COPY2_IF_LT( i_cost, analysis.i_satd_i16x16, i_type, I_16x16 );
            COPY2_IF_LT( i_cost, analysis.i_satd_i4x4, i_type, I_4x4 );
#endif

            h->mb.i_type = i_type;
        }
    }

    /* update mb predict mode decisions to x264_t.mb.cache */
    x264_analyse_update_cache( h, &analysis );

    h->mb.b_trellis = h->param.analyse.i_trellis;
    h->mb.b_noise_reduction = h->mb.b_noise_reduction || (!!h->param.analyse.i_noise_reduction && !IS_INTRA( h->mb.i_type ));
    if( h->mb.b_trellis == 1 || h->mb.b_noise_reduction )
        h->mb.i_skip_intra = 0;
}

/*-------------------- Update MB from the analysis ----------------------*/
static void x264_analyse_update_cache( x264_t *h, x264_mb_analysis_t *a  )
{
    switch( h->mb.i_type )
    {
        case I_4x4:
            /* h->mb.cache.intra4x4_pred_mode[x264_scan8[i]] = a->i_predict4x4[i]; */
        	h->mb.cache.intra4x4_pred_mode[12] = a->i_predict4x4[0];
        	h->mb.cache.intra4x4_pred_mode[13] = a->i_predict4x4[1];
        	h->mb.cache.intra4x4_pred_mode[20] = a->i_predict4x4[2];
        	h->mb.cache.intra4x4_pred_mode[21] = a->i_predict4x4[3];
        	h->mb.cache.intra4x4_pred_mode[14] = a->i_predict4x4[4];
        	h->mb.cache.intra4x4_pred_mode[15] = a->i_predict4x4[5];
        	h->mb.cache.intra4x4_pred_mode[22] = a->i_predict4x4[6];
        	h->mb.cache.intra4x4_pred_mode[23] = a->i_predict4x4[7];
        	h->mb.cache.intra4x4_pred_mode[28] = a->i_predict4x4[8];
        	h->mb.cache.intra4x4_pred_mode[29] = a->i_predict4x4[9];
        	h->mb.cache.intra4x4_pred_mode[36] = a->i_predict4x4[10];
        	h->mb.cache.intra4x4_pred_mode[37] = a->i_predict4x4[11];
        	h->mb.cache.intra4x4_pred_mode[30] = a->i_predict4x4[12];
        	h->mb.cache.intra4x4_pred_mode[31] = a->i_predict4x4[13];
        	h->mb.cache.intra4x4_pred_mode[38] = a->i_predict4x4[14];
        	h->mb.cache.intra4x4_pred_mode[39] = a->i_predict4x4[15];
            x264_mb_analyse_intra_chroma( h, a );
            break;
        case I_16x16:
            h->mb.i_intra16x16_pred_mode = a->i_predict16x16;
            x264_mb_analyse_intra_chroma( h, a );
            break;
        case P_L0:
            switch( h->mb.i_partition )
            {
                case D_16x16:
                    x264_macroblock_cache_ref( h, 0, 0, 4, 4, 0, a->l0.me16x16.i_ref );
                    x264_macroblock_cache_mv_ptr( h, 0, 0, 4, 4, 0, a->l0.me16x16.mv );
                    break;
                case D_16x8:
                    x264_macroblock_cache_ref( h, 0, 0, 4, 2, 0, a->l0.me16x8[0].i_ref );
                    x264_macroblock_cache_ref( h, 0, 2, 4, 2, 0, a->l0.me16x8[1].i_ref );
                    x264_macroblock_cache_mv_ptr( h, 0, 0, 4, 2, 0, a->l0.me16x8[0].mv );
                    x264_macroblock_cache_mv_ptr( h, 0, 2, 4, 2, 0, a->l0.me16x8[1].mv );
                    break;
                case D_8x16:
                    x264_macroblock_cache_ref( h, 0, 0, 2, 4, 0, a->l0.me8x16[0].i_ref );
                    x264_macroblock_cache_ref( h, 2, 0, 2, 4, 0, a->l0.me8x16[1].i_ref );
                    x264_macroblock_cache_mv_ptr( h, 0, 0, 2, 4, 0, a->l0.me8x16[0].mv );
                    x264_macroblock_cache_mv_ptr( h, 2, 0, 2, 4, 0, a->l0.me8x16[1].mv );
                    break;
                default:
                    x264_log( h, X264_LOG_ERROR, "internal error P_L0 and partition=%d\n", h->mb.i_partition );
                    break;
            }
            break;
        case P_8x8:
            x264_macroblock_cache_ref( h, 0, 0, 2, 2, 0, a->l0.me8x8[0].i_ref );
            x264_macroblock_cache_ref( h, 2, 0, 2, 2, 0, a->l0.me8x8[1].i_ref );
            x264_macroblock_cache_ref( h, 0, 2, 2, 2, 0, a->l0.me8x8[2].i_ref );
            x264_macroblock_cache_ref( h, 2, 2, 2, 2, 0, a->l0.me8x8[3].i_ref );
            x264_macroblock_cache_mv_ptr( h, 0, 0, 2, 2, 0, a->l0.me8x8[0].mv );
            x264_macroblock_cache_mv_ptr( h, 2, 0, 2, 2, 0, a->l0.me8x8[1].mv );
            x264_macroblock_cache_mv_ptr( h, 0, 2, 2, 2, 0, a->l0.me8x8[2].mv );
            x264_macroblock_cache_mv_ptr( h, 2, 2, 2, 2, 0, a->l0.me8x8[3].mv );
            break;
        case P_SKIP:
            h->mb.i_partition = D_16x16;
            x264_macroblock_cache_ref( h, 0, 0, 4, 4, 0, 0 );
            x264_macroblock_cache_mv_ptr( h, 0, 0, 4, 4, 0, h->mb.cache.pskip_mv );
            break;
    }
}
