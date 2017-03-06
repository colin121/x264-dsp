/*****************************************************************************
 * quant.c: quantization and level-run
 *****************************************************************************/

#include "common.h"

#ifdef __TI_COMPILER_VERSION__
int quant_4x4_ti ( dctcoef dct[16], udctcoef mf[16], udctcoef bias[16] );
int quant_4x4_dc_ti ( dctcoef dct[16], int mf, int bias );
int quant_2x2_dc_ti ( dctcoef dct[4], int mf, int bias );
#endif

const int x264_dequant_div_lut[QP_MAX+1][2] = /* { qp/6, qp%6 } */
{
	{ 0, 0}, { 0, 1}, { 0, 2}, { 0, 3}, { 0, 4}, { 0, 5}, /* QP: [ 0 ~  5] */
	{ 1, 0}, { 1, 1}, { 1, 2}, { 1, 3}, { 1, 4}, { 1, 5}, /* QP: [ 6 ~ 11] */
	{ 2, 0}, { 2, 1}, { 2, 2}, { 2, 3}, { 2, 4}, { 2, 5}, /* QP: [12 ~ 17] */
	{ 3, 0}, { 3, 1}, { 3, 2}, { 3, 3}, { 3, 4}, { 3, 5}, /* QP: [18 ~ 23] */
	{ 4, 0}, { 4, 1}, { 4, 2}, { 4, 3}, { 4, 4}, { 4, 5}, /* QP: [24 ~ 29] */
	{ 5, 0}, { 5, 1}, { 5, 2}, { 5, 3}, { 5, 4}, { 5, 5}, /* QP: [30 ~ 35] */
	{ 6, 0}, { 6, 1}, { 6, 2}, { 6, 3}, { 6, 4}, { 6, 5}, /* QP: [36 ~ 41] */
	{ 7, 0}, { 7, 1}, { 7, 2}, { 7, 3}, { 7, 4}, { 7, 5}, /* QP: [42 ~ 47] */
	{ 8, 0}, { 8, 1}, { 8, 2}, { 8, 3}, { 8, 4}, { 8, 5}, /* QP: [48 ~ 53] */
	{ 9, 0}, { 9, 1}, { 9, 2}, { 9, 3}, { 9, 4}, { 9, 5}, /* QP: [54 ~ 59] */
	{10, 0}, {10, 1}, {10, 2}, {10, 3}, {10, 4}, {10, 5}, /* QP: [60 ~ 65] */
	{11, 0}, {11, 1}, {11, 2}, {11, 3}                    /* QP: [66 ~ 69] */
};

#define QUANT_ONE( coef, mf, f ) \
{ \
    if( (coef) > 0 ) \
        (coef) = (f + (coef)) * (mf) >> 16; \
    else \
        (coef) = - ((f - (coef)) * (mf) >> 16); \
    nz |= (coef); \
}

static int quant_4x4( dctcoef dct[16], udctcoef mf[16], udctcoef bias[16] )
{
    int i, nz = 0;
    for( i = 0; i < 16; i++ )
        QUANT_ONE( dct[i], mf[i], bias[i] );
    return !!nz;
}

static int quant_4x4_dc( dctcoef dct[16], int mf, int bias )
{
    int i, nz = 0;
    for( i = 0; i < 16; i++ )
        QUANT_ONE( dct[i], mf, bias );
    return !!nz;
}

static int quant_2x2_dc( dctcoef dct[4], int mf, int bias )
{
    int nz = 0;
    QUANT_ONE( dct[0], mf, bias );
    QUANT_ONE( dct[1], mf, bias );
    QUANT_ONE( dct[2], mf, bias );
    QUANT_ONE( dct[3], mf, bias );
    return !!nz;
}

static void dequant_4x4( dctcoef dct[16], int dequant_mf[6][16], int i_qp )
{
    const int i_mf = i_qp%6;
    const int i_qbits = i_qp/6 - 4;
	int i;

    if( i_qbits >= 0 )
    {
        for( i = 0; i < 16; i++ )
        	dct[i] = ( dct[i] * dequant_mf[i_mf][i] ) << i_qbits;
    }
    else
    {
        const int f = 1 << (-i_qbits-1);
        for( i = 0; i < 16; i++ )
        	dct[i] = ( dct[i] * dequant_mf[i_mf][i] + f ) >> (-i_qbits);
    }
}

static void dequant_4x4_dc( dctcoef dct[16], int dequant_mf[6][16], int i_qp )
{
    const int i_qbits = i_qp/6 - 6;
	int i;

    if( i_qbits >= 0 )
    {
        const int i_dmf = dequant_mf[i_qp%6][0] << i_qbits;
        for( i = 0; i < 16; i++ )
            dct[i] *= i_dmf;
    }
    else
    {
        const int i_dmf = dequant_mf[i_qp%6][0];
        const int f = 1 << (-i_qbits-1);
        for( i = 0; i < 16; i++ )
            dct[i] = ( dct[i] * i_dmf + f ) >> (-i_qbits);
    }
}

#ifdef __TI_COMPILER_VERSION__
void dequant_4x4_shl_ti( dctcoef dct[16], int dequant_mf[16], int qbits );
void dequant_4x4_shr_ti( dctcoef dct[16], int dequant_mf[16], int qbits, int bias );

static void dequant_4x4_ti( dctcoef dct[16], int dequant_mf[6][16], int i_qp )
{
    const int i_mf    = x264_dequant_div_lut[i_qp][1];     /* i_qp%6 */
    const int i_qbits = x264_dequant_div_lut[i_qp][0] - 4; /* i_qp/6 - 4 */

    if( i_qbits >= 0 )
    	dequant_4x4_shl_ti( dct, dequant_mf[i_mf], i_qbits );
    else
    	dequant_4x4_shr_ti( dct, dequant_mf[i_mf], -i_qbits, 1 << (-i_qbits-1) );
}

void dequant_4x4_dc_shl_ti( dctcoef dct[16], int dequant_mf, int qbits );
void dequant_4x4_dc_shr_ti( dctcoef dct[16], int dequant_mf, int qbits, int bias );

static void dequant_4x4_dc_ti( dctcoef dct[16], int dequant_mf[6][16], int i_qp )
{
	const int i_mf    = x264_dequant_div_lut[i_qp][1];     /* i_qp%6 */
    const int i_qbits = x264_dequant_div_lut[i_qp][0] - 6; /* i_qp/6 - 6 */

    if( i_qbits >= 0 )
    	dequant_4x4_dc_shl_ti( dct, dequant_mf[i_mf][0], i_qbits );
    else
    	dequant_4x4_dc_shr_ti( dct, dequant_mf[i_mf][0], -i_qbits, 1 << (-i_qbits-1) );
}
#endif

static ALWAYS_INLINE void optimize_chroma_idct_dequant_2x2( dctcoef out[4], dctcoef dct[4], int dmf )
{
    int d0 = dct[0] + dct[1];
    int d1 = dct[2] + dct[3];
    int d2 = dct[0] - dct[1];
    int d3 = dct[2] - dct[3];
    out[0] = ((d0 + d1) * dmf >> 5) + 32;
    out[1] = ((d0 - d1) * dmf >> 5) + 32;
    out[2] = ((d2 + d3) * dmf >> 5) + 32;
    out[3] = ((d2 - d3) * dmf >> 5) + 32;
}

static ALWAYS_INLINE int optimize_chroma_round( dctcoef *ref, dctcoef *dct, int dequant_mf )
{
    dctcoef out[8];
	int sum = 0, i;

    optimize_chroma_idct_dequant_2x2( out, dct, dequant_mf );

    for( i = 0; i < 4; i++ )
        sum |= ref[i] ^ out[i];
    return sum >> 6;
}

static int optimize_chroma_2x2_dc( dctcoef dct[4], int dequant_mf )
{
    /* dequant_mf = h->dequant4_mf[CQM_4IC + b_inter][i_qp%6][0] << i_qp/6, max 32*64 */
    dctcoef dct_orig[8];
    int coeff, nz, i;
	int sum = 0;

    optimize_chroma_idct_dequant_2x2( dct_orig, dct, dequant_mf );

    /* If the DC coefficients already round to zero, terminate early. */
    for( i = 0; i < 4; i++ )
        sum |= dct_orig[i];
    if( !(sum >> 6) )
        return 0;

    /* Start with the highest frequency coefficient... is this the best option? */
    for( nz = 0, coeff = 3; coeff >= 0; coeff-- )
    {
        int level = dct[coeff];
        int sign = level>>31 | 1; /* dct[coeff] < 0 ? -1 : 1 */

        while( level )
        {
            dct[coeff] = level - sign;
            if( optimize_chroma_round( dct_orig, dct, dequant_mf ) )
            {
                nz = 1;
                dct[coeff] = level;
                break;
            }
            level -= sign;
        }
    }

    return nz;
}

static void x264_denoise_dct( dctcoef *dct, uint32_t *sum, udctcoef *offset, int size )
{
	int i;
    for( i = 0; i < size; i++ )
    {
        int level = dct[i];
        int sign = level>>31;
        level = (level+sign)^sign;
        sum[i] += level;
        level -= offset[i];
        dct[i] = level<0 ? 0 : (level^sign)-sign;
    }
}

#ifdef __TI_COMPILER_VERSION__
int x264_decimate_score16_ti( dctcoef *dct );
#endif

/* (ref: JVT-B118)
 * x264_mb_decimate_score: given dct coeffs it returns a score to see if we could empty this dct coeffs
 * to 0 (low score means set it to null)
 * Used in inter macroblock (luma and chroma)
 *  luma: for a 8x8 block: if score < 4 -> null
 *        for the complete mb: if score < 6 -> null
 *  chroma: for the complete mb: if score < 7 -> null
 */

const uint8_t x264_decimate_table4[16] =
{
    3,2,2,1,1,1,0,0,0,0,0,0,0,0,0,0
};

static int ALWAYS_INLINE x264_decimate_score_internal( dctcoef *dct, int i_max )
{
    const uint8_t *ds_table = x264_decimate_table4;
    int i_score = 0;
    int idx = i_max - 1;

    while( idx >= 0 && dct[idx] == 0 ) /* skip consecutive zeros */
        idx--;
    while( idx >= 0 )
    {
        int i_run;

        /* if current non-zero coeff is neither 1 nor -1, return 9 directly */
        if( (unsigned)(dct[idx--] + 1) > 2 )
            return 9;

        i_run = 0;
        while( idx >= 0 && dct[idx] == 0 ) /* skip consecutive zeros */
        {
            idx--;
            i_run++;
        }
        i_score += ds_table[i_run];
    }

    return i_score;
}

static int x264_decimate_score15( dctcoef *dct )
{
    return x264_decimate_score_internal( dct+1, 15 ); /* excludes dc */
}
static int x264_decimate_score16( dctcoef *dct )
{
    return x264_decimate_score_internal( dct, 16 ); /* includes dc */
}

#define last(num)\
static int x264_coeff_last##num( dctcoef *l )\
{\
    int i_last = num-1;\
    while( i_last >= 0 && l[i_last] == 0 )\
        i_last--;\
    return i_last;\
}

last(4)
last(8)
last(15)
last(16)
last(64)

#ifdef __TI_COMPILER_VERSION__
int x264_coeff_last16_ti( dctcoef *dct );
#endif

#define level_run(num)\
static int x264_coeff_level_run##num( dctcoef *dct, x264_run_level_t *runlevel )\
{\
    int i_last = runlevel->last = x264_coeff_last##num(dct);\
    int i_total = 0;\
    int mask = 0;\
    do\
    {\
        runlevel->level[i_total++] = dct[i_last];\
        mask |= 1 << (i_last);\
        while( --i_last >= 0 && dct[i_last] == 0 );\
    } while( i_last >= 0 );\
    runlevel->mask = mask;\
    return i_total;\
}

level_run(4)
level_run(8)
level_run(15)
level_run(16)

void x264_quant_init( x264_t *h, int cpu, x264_quant_function_t *pf )
{
    pf->quant_4x4 = quant_4x4;
    pf->quant_4x4_dc = quant_4x4_dc;
    pf->quant_2x2_dc = quant_2x2_dc;

    pf->dequant_4x4 = dequant_4x4;
    pf->dequant_4x4_dc = dequant_4x4_dc;

    pf->optimize_chroma_2x2_dc = optimize_chroma_2x2_dc;

    pf->denoise_dct = x264_denoise_dct;
    pf->decimate_score15 = x264_decimate_score15;
    pf->decimate_score16 = x264_decimate_score16;

    pf->coeff_last4 = x264_coeff_last4;
    pf->coeff_last8 = x264_coeff_last8;
    pf->coeff_last[  DCT_LUMA_AC] = x264_coeff_last15;
    pf->coeff_last[ DCT_LUMA_4x4] = x264_coeff_last16;
    pf->coeff_last[ DCT_LUMA_8x8] = x264_coeff_last64;
    pf->coeff_level_run4 = x264_coeff_level_run4;
    pf->coeff_level_run8 = x264_coeff_level_run8;
    pf->coeff_level_run[  DCT_LUMA_AC] = x264_coeff_level_run15;
    pf->coeff_level_run[ DCT_LUMA_4x4] = x264_coeff_level_run16;

    pf->coeff_last[DCT_LUMA_DC]     = pf->coeff_last[DCT_CHROMAU_DC]  = pf->coeff_last[DCT_CHROMAV_DC] =
    pf->coeff_last[DCT_CHROMAU_4x4] = pf->coeff_last[DCT_CHROMAV_4x4] = pf->coeff_last[DCT_LUMA_4x4];
    pf->coeff_last[DCT_CHROMA_AC]   = pf->coeff_last[DCT_CHROMAU_AC]  =
    pf->coeff_last[DCT_CHROMAV_AC]  = pf->coeff_last[DCT_LUMA_AC];
    pf->coeff_last[DCT_CHROMAU_8x8] = pf->coeff_last[DCT_CHROMAV_8x8] = pf->coeff_last[DCT_LUMA_8x8];

    pf->coeff_level_run[DCT_LUMA_DC]     = pf->coeff_level_run[DCT_CHROMAU_DC]  = pf->coeff_level_run[DCT_CHROMAV_DC] =
    pf->coeff_level_run[DCT_CHROMAU_4x4] = pf->coeff_level_run[DCT_CHROMAV_4x4] = pf->coeff_level_run[DCT_LUMA_4x4];
    pf->coeff_level_run[DCT_CHROMA_AC]   = pf->coeff_level_run[DCT_CHROMAU_AC]  =
    pf->coeff_level_run[DCT_CHROMAV_AC]  = pf->coeff_level_run[DCT_LUMA_AC];

#ifdef __TI_COMPILER_VERSION__
    pf->quant_4x4 = quant_4x4_ti;
    pf->quant_4x4_dc = quant_4x4_dc_ti;
    pf->quant_2x2_dc = quant_2x2_dc_ti;

    pf->dequant_4x4 = dequant_4x4_ti;
    pf->dequant_4x4_dc = dequant_4x4_dc_ti;

    pf->decimate_score16 = x264_decimate_score16_ti;
    pf->coeff_last[ DCT_LUMA_4x4] = x264_coeff_last16_ti;

    pf->coeff_last[DCT_LUMA_DC]     = pf->coeff_last[DCT_CHROMAU_DC]  = pf->coeff_last[DCT_CHROMAV_DC] =
    pf->coeff_last[DCT_CHROMAU_4x4] = pf->coeff_last[DCT_CHROMAV_4x4] = pf->coeff_last[DCT_LUMA_4x4];
#endif
}
