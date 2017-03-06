/*****************************************************************************
 * deblock.c: deblocking
 *****************************************************************************/

#include "common.h"

#ifdef __TI_COMPILER_VERSION__
void deblock_v_luma_ti   ( pixel *pix, intptr_t stride, int alpha, int beta, int8_t *tc0 );
void deblock_h_luma_ti   ( pixel *pix, intptr_t stride, int alpha, int beta, int8_t *tc0 );
void deblock_v_chroma_ti ( pixel *pix, intptr_t stride, int alpha, int beta, int8_t *tc0 );
void deblock_h_chroma_ti ( pixel *pix, intptr_t stride, int alpha, int beta, int8_t *tc0 );

void deblock_v_luma_intra_ti   ( pixel *pix, intptr_t stride, int alpha, int beta );
void deblock_h_luma_intra_ti   ( pixel *pix, intptr_t stride, int alpha, int beta );
void deblock_v_chroma_intra_ti ( pixel *pix, intptr_t stride, int alpha, int beta );
void deblock_h_chroma_intra_ti ( pixel *pix, intptr_t stride, int alpha, int beta );

void deblock_strength_ti ( uint8_t nnz[X264_SCAN8_SIZE], int8_t ref[2][X264_SCAN8_LUMA_SIZE], int16_t mv[2][X264_SCAN8_LUMA_SIZE][2], uint8_t bs[2][8][4] );
#endif

/* Deblocking filter */

/* Table 8-16 Page 225 <ITU-T-REC-H.264-201201.pdf>
 * Derivation of offset dependent threshold variables ¦Á¡ä and ¦Â¡ä from indexA (QP + offsetA) and indexB (QP + offsetB)
 */
static const uint8_t i_alpha_table[52+12*3] =
{
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
     /* alpha values from indexA range: [0 - 51] */
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,         /* QP:  0 ~  9 */
     0,  0,  0,  0,  0,  0,  4,  4,  5,  6,         /* QP: 10 ~ 19 */
     7,  8,  9, 10, 12, 13, 15, 17, 20, 22,         /* QP: 20 ~ 29 */
    25, 28, 32, 36, 40, 45, 50, 56, 63, 71,         /* QP: 30 ~ 39 */
    80, 90,101,113,127,144,162,182,203,226,         /* QP: 40 ~ 49 */
   255,255,                                         /* QP: 50 ~ 51 */
   255,255,255,255,255,255,255,255,255,255,255,255,
};
static const uint8_t i_beta_table[52+12*3] =
{
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
     /* beta values from indexB range: [0 - 51] */
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,         /* QP:  0 ~  9 */
     0,  0,  0,  0,  0,  0,  2,  2,  2,  3,         /* QP: 10 ~ 19 */
     3,  3,  3,  4,  4,  4,  6,  6,  7,  7,         /* QP: 20 ~ 29 */
     8,  8,  9,  9, 10, 10, 11, 11, 12, 12,         /* QP: 30 ~ 39 */
    13, 13, 14, 14, 15, 15, 16, 16, 17, 17,         /* QP: 40 ~ 49 */
    18, 18,                                         /* QP: 50 ~ 51 */
    18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18,
};

/* Table 8-17 Page 225 <ITU-T-REC-H.264-201201.pdf>
 * Value of variable t¡äC0 as a function of indexA and bS
 */
static const int8_t i_tc0_table[52+12*3][4] =
{
    {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 },
    {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 },
    {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 },
    {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 },
    /* tc0 values from indexA range: [0 - 51], bS range: [0 - 3] */
    {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, /* QP:  0 ~  5 */
    {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, /* QP:  6 ~ 11 */
    {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 0 }, {-1, 0, 0, 1 }, /* QP: 12 ~ 17 */
    {-1, 0, 0, 1 }, {-1, 0, 0, 1 }, {-1, 0, 0, 1 }, {-1, 0, 1, 1 }, {-1, 0, 1, 1 }, {-1, 1, 1, 1 }, /* QP: 18 ~ 23 */
    {-1, 1, 1, 1 }, {-1, 1, 1, 1 }, {-1, 1, 1, 1 }, {-1, 1, 1, 2 }, {-1, 1, 1, 2 }, {-1, 1, 1, 2 }, /* QP: 24 ~ 29 */
    {-1, 1, 1, 2 }, {-1, 1, 2, 3 }, {-1, 1, 2, 3 }, {-1, 2, 2, 3 }, {-1, 2, 2, 4 }, {-1, 2, 3, 4 }, /* QP: 30 ~ 35 */
    {-1, 2, 3, 4 }, {-1, 3, 3, 5 }, {-1, 3, 4, 6 }, {-1, 3, 4, 6 }, {-1, 4, 5, 7 }, {-1, 4, 5, 8 }, /* QP: 36 ~ 41 */
    {-1, 4, 6, 9 }, {-1, 5, 7,10 }, {-1, 6, 8,11 }, {-1, 6, 8,13 }, {-1, 7,10,14 }, {-1, 8,11,16 }, /* QP: 42 ~ 47 */
    {-1, 9,12,18 }, {-1,10,13,20 }, {-1,11,15,23 }, {-1,13,17,25 },                                 /* QP: 48 ~ 51 */
    {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 },
    {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 }, {-1,13,17,25 },
};
#define alpha_table(x) i_alpha_table[(x)+24]
#define beta_table(x)  i_beta_table[(x)+24]
#define tc0_table(x)   i_tc0_table[(x)+24]

/* From ffmpeg */
static ALWAYS_INLINE void deblock_edge_luma_c( pixel *pix, intptr_t xstride, int alpha, int beta, int8_t tc0 )
{
    int p2 = pix[-3*xstride];
    int p1 = pix[-2*xstride];
    int p0 = pix[-1*xstride];
    int q0 = pix[ 0*xstride];
    int q1 = pix[ 1*xstride];
    int q2 = pix[ 2*xstride];

    /* enable deblock of edge only if all following conditions are satisfied:
     * |p0 - q0| < alpha
     * |p1 - p0| < beta
     * |q1 - q0| < beta
     */
    if( abs( p0 - q0 ) < alpha && abs( p1 - p0 ) < beta && abs( q1 - q0 ) < beta )
    {
        int tc = tc0;
        int delta;
        /* deblock for internal pixel: p1 */
        /* delta = (p2 + 0.5 * (p0 + q0 + 1) - 2p1) / 2 */
        if( abs( p2 - p0 ) < beta )
        {
            if( tc0 )
                pix[-2*xstride] = p1 + x264_clip3( (( p2 + ((p0 + q0 + 1) >> 1)) >> 1) - p1, -tc0, tc0 ); /* p1' */
            tc++;
        }
        /* deblock for internal pixel: q1 */
        /* delta = (q2 + 0.5 * (p0 + q0 + 1) - 2q1) / 2 */
        if( abs( q2 - q0 ) < beta )
        {
            if( tc0 )
                pix[ 1*xstride] = q1 + x264_clip3( (( q2 + ((p0 + q0 + 1) >> 1)) >> 1) - q1, -tc0, tc0 ); /* q1' */
            tc++;
        }
        /* deblock for edge pixel: p0 and q0 */
        /* delta = (4 * (q0 - p0) + (p1 - q1) + 4) / 8 */
        delta = x264_clip3( (((q0 - p0 ) << 2) + (p1 - q1) + 4) >> 3, -tc, tc );
        pix[-1*xstride] = x264_clip_pixel( p0 + delta );    /* p0' */
        pix[ 0*xstride] = x264_clip_pixel( q0 - delta );    /* q0' */
    }
}
static inline void deblock_luma_c( pixel *pix, intptr_t xstride, intptr_t ystride, int alpha, int beta, int8_t *tc0 )
{
	int i;
    for( i = 0; i < 4; i++ )
    {
		int d;
        if( tc0[i] < 0 )
        {
            pix += 4*ystride;
            continue;
        }
        for( d = 0; d < 4; d++, pix += ystride )
            deblock_edge_luma_c( pix, xstride, alpha, beta, tc0[i] );
    }
}
/* deblock of luma for one vertical line (16 pixel) */
static void deblock_v_luma_c( pixel *pix, intptr_t stride, int alpha, int beta, int8_t *tc0 )
{
    deblock_luma_c( pix, stride, 1, alpha, beta, tc0 );
}
/* deblock of luma for one horizontal line (16 pixel) */
static void deblock_h_luma_c( pixel *pix, intptr_t stride, int alpha, int beta, int8_t *tc0 )
{
    deblock_luma_c( pix, 1, stride, alpha, beta, tc0 );
}

static ALWAYS_INLINE void deblock_edge_chroma_c( pixel *pix, intptr_t xstride, int alpha, int beta, int8_t tc )
{
    int p1 = pix[-2*xstride];
    int p0 = pix[-1*xstride];
    int q0 = pix[ 0*xstride];
    int q1 = pix[ 1*xstride];

    /* enable deblock of edge only if all following conditions are satisfied:
     * |p0 - q0| < alpha
     * |p1 - p0| < beta
     * |q1 - q0| < beta
     */
    if( abs( p0 - q0 ) < alpha && abs( p1 - p0 ) < beta && abs( q1 - q0 ) < beta )
    {
        /* deblock for edge pixel: p0 and q0 */
        /* delta = (4 * (q0 - p0) + (p1 - q1) + 4) / 8 */
        int delta = x264_clip3( (((q0 - p0 ) << 2) + (p1 - q1) + 4) >> 3, -tc, tc );
        pix[-1*xstride] = x264_clip_pixel( p0 + delta );    /* p0' */
        pix[ 0*xstride] = x264_clip_pixel( q0 - delta );    /* q0' */
    }
}
static ALWAYS_INLINE void deblock_chroma_c( pixel *pix, intptr_t xstride, intptr_t ystride, int alpha, int beta, int8_t *tc0 )
{
	int i;
    for( i = 0; i < 4; i++ )
    {
        int tc = tc0[i];
		int d, e;
        if( tc <= 0 )
        {
            pix += 2*ystride;
            continue;
        }
        for( d = 0; d < 2; d++, pix += ystride-2 )
            for( e = 0; e < 2; e++, pix++ )
                deblock_edge_chroma_c( pix, xstride, alpha, beta, tc0[i] );
    }
}
/* deblock of chroma for one vertical line */
static void deblock_v_chroma_c( pixel *pix, intptr_t stride, int alpha, int beta, int8_t *tc0 )
{
    deblock_chroma_c( pix, stride, 2, alpha, beta, tc0 );
}
/* deblock of chroma for one horizontal line */
static void deblock_h_chroma_c( pixel *pix, intptr_t stride, int alpha, int beta, int8_t *tc0 )
{
    deblock_chroma_c( pix, 2, stride, alpha, beta, tc0 );
}

static ALWAYS_INLINE void deblock_edge_luma_intra_c( pixel *pix, intptr_t xstride, int alpha, int beta )
{
    int p2 = pix[-3*xstride];
    int p1 = pix[-2*xstride];
    int p0 = pix[-1*xstride];
    int q0 = pix[ 0*xstride];
    int q1 = pix[ 1*xstride];
    int q2 = pix[ 2*xstride];

    /* enable deblock of edge only if all following conditions are satisfied:
     * |p0 - q0| < alpha
     * |p1 - p0| < beta
     * |q1 - q0| < beta
     */
    if( abs( p0 - q0 ) < alpha && abs( p1 - p0 ) < beta && abs( q1 - q0 ) < beta )
    {
    	/* Constraint condition: |p0 - q0| < (alpha >> 2) + 2
    	 * If true, use 4-tap or 5-tap strong filter to fix p2', p1', p0', q0', q1', q2'.
    	 * otherwise, use 3-tap weak filter to fix p0', q0'.
    	 */
        if( abs( p0 - q0 ) < ((alpha >> 2) + 2) )
        {
            if( abs( p2 - p0 ) < beta )
            {
                const int p3 = pix[-4*xstride];
                pix[-1*xstride] = ( p2 + 2*p1 + 2*p0 + 2*q0 + q1 + 4 ) >> 3; /* p0' */
                pix[-2*xstride] = ( p2 + p1 + p0 + q0 + 2 ) >> 2;            /* p1' */
                pix[-3*xstride] = ( 2*p3 + 3*p2 + p1 + p0 + q0 + 4 ) >> 3;   /* p2' */
            }
            else
                pix[-1*xstride] = ( 2*p1 + p0 + q1 + 2 ) >> 2;               /* p0' */
            if( abs( q2 - q0 ) < beta )
            {
                const int q3 = pix[3*xstride];
                pix[0*xstride] = ( p1 + 2*p0 + 2*q0 + 2*q1 + q2 + 4 ) >> 3;  /* q0' */
                pix[1*xstride] = ( p0 + q0 + q1 + q2 + 2 ) >> 2;             /* q1' */
                pix[2*xstride] = ( 2*q3 + 3*q2 + q1 + q0 + p0 + 4 ) >> 3;    /* q2' */
            }
            else
                pix[0*xstride] = ( 2*q1 + q0 + p1 + 2 ) >> 2;                /* q0' */
        }
        else
        {
            pix[-1*xstride] = ( 2*p1 + p0 + q1 + 2 ) >> 2;                   /* p0' */
            pix[ 0*xstride] = ( 2*q1 + q0 + p1 + 2 ) >> 2;                   /* q0' */
        }
    }
}
static inline void deblock_luma_intra_c( pixel *pix, intptr_t xstride, intptr_t ystride, int alpha, int beta )
{
	int d;
    for( d = 0; d < 16; d++, pix += ystride )
        deblock_edge_luma_intra_c( pix, xstride, alpha, beta );
}
/* deblock (BS == 4) of luma for one vertical line (16 pixel) */
static void deblock_v_luma_intra_c( pixel *pix, intptr_t stride, int alpha, int beta )
{
    deblock_luma_intra_c( pix, stride, 1, alpha, beta );
}
/* deblock (BS == 4) of luma for one horizontal line (16 pixel) */
static void deblock_h_luma_intra_c( pixel *pix, intptr_t stride, int alpha, int beta )
{
    deblock_luma_intra_c( pix, 1, stride, alpha, beta );
}

static ALWAYS_INLINE void deblock_edge_chroma_intra_c( pixel *pix, intptr_t xstride, int alpha, int beta )
{
    int p1 = pix[-2*xstride];
    int p0 = pix[-1*xstride];
    int q0 = pix[ 0*xstride];
    int q1 = pix[ 1*xstride];

    /* enable deblock of edge only if all following conditions are satisfied:
     * |p0 - q0| < alpha
     * |p1 - p0| < beta
     * |q1 - q0| < beta
     */
    if( abs( p0 - q0 ) < alpha && abs( p1 - p0 ) < beta && abs( q1 - q0 ) < beta )
    {
        pix[-1*xstride] = (2*p1 + p0 + q1 + 2) >> 2;   /* p0' */
        pix[ 0*xstride] = (2*q1 + q0 + p1 + 2) >> 2;   /* q0' */
    }
}
static ALWAYS_INLINE void deblock_chroma_intra_c( pixel *pix, int width, int height, intptr_t xstride, intptr_t ystride, int alpha, int beta )
{
	int d, e;
    for( d = 0; d < height; d++, pix += ystride-2 )
        for( e = 0; e < width; e++, pix++ )
            deblock_edge_chroma_intra_c( pix, xstride, alpha, beta );
}
/* deblock (BS == 4) of chroma for one vertical line */
static void deblock_v_chroma_intra_c( pixel *pix, intptr_t stride, int alpha, int beta )
{
    deblock_chroma_intra_c( pix, 1, 16, stride, 2, alpha, beta );
}
/* deblock (BS == 4) of chroma for one horizontal line */
static void deblock_h_chroma_intra_c( pixel *pix, intptr_t stride, int alpha, int beta )
{
    deblock_chroma_intra_c( pix, 2, 8, 2, stride, alpha, beta );
}

static void deblock_strength_c( uint8_t nnz[X264_SCAN8_SIZE], int8_t ref[2][X264_SCAN8_LUMA_SIZE], int16_t mv[2][X264_SCAN8_LUMA_SIZE][2], uint8_t bs[2][8][4] )
{
	int dir;
    for( dir = 0; dir < 2; dir++ )
    {
        int s1 = dir ? 1 : 8;
        int s2 = dir ? 8 : 1;
		int edge, i, loc;
        for( edge = 0; edge < 4; edge++ )
            for( i = 0, loc = X264_SCAN8_0+edge*s2; i < 4; i++, loc += s1 )
            {
                int locn = loc - s2;
                /* If at least one block residual is coded (nnz > 0), set BS = 2 */
                if( nnz[loc] || nnz[locn] )
                    bs[dir][edge][i] = 2;
                /* If reference frame is different, or mv diff >= 4, set BS = 1 */
                else if( ref[0][loc] != ref[0][locn] ||
                         abs( mv[0][loc][0] - mv[0][locn][0] ) >= 4 ||
                         abs( mv[0][loc][1] - mv[0][locn][1] ) >= 4 )
                {
                    bs[dir][edge][i] = 1;
                }
                else
                    bs[dir][edge][i] = 0;
            }
    }
}

static ALWAYS_INLINE void deblock_edge( x264_t *h, pixel *pix, intptr_t i_stride, uint8_t bS[4], int index_a,
                                        int alpha, int beta, int b_chroma, x264_deblock_inter_t pf_inter )
{
    int8_t tc[4];

    if( !M32(bS) || !alpha || !beta )
        return;

    tc[0] = tc0_table(index_a)[bS[0]] + b_chroma;
    tc[1] = tc0_table(index_a)[bS[1]] + b_chroma;
    tc[2] = tc0_table(index_a)[bS[2]] + b_chroma;
    tc[3] = tc0_table(index_a)[bS[3]] + b_chroma;

    pf_inter( pix, i_stride, alpha, beta, tc );
}

void x264_frame_deblock_row( x264_t *h, int mb_y )
{
    int a   = h->sh.i_alpha_c0_offset;    /* OffsetA that will affect alpha and c0 value. default: 0 */
    int b   = h->sh.i_beta_offset;        /* OffsetB that will affect beta value. default: 0 */
    int qp  = h->sh.i_qp;                 /* luma qp of current slice  */
    int qpc = h->chroma_qp_table[qp];     /* chroma qp by mapping table */
    int index_a  = qp + a;                /* luma IndexA */
    int index_b  = qp + b;                /* luma IndexB */
    int index_ac = qpc + a;               /* chroma IndexA */
    int index_bc = qpc + b;               /* chroma IndexB */
    int alpha    = alpha_table(index_a);  /* luma alpha */
    int beta     = beta_table(index_b);   /* luma beta */
    int alphac   = alpha_table(index_ac); /* chroma alpha */
    int betac    = beta_table(index_bc);  /* chroma beta */
    int stridey  = h->fdec->i_stride[0];  /* luma stride */
    int strideuv = h->fdec->i_stride[1];  /* chroma stride */
    int stridey_4x  = stridey<<2;         /* luma stride x 4 */
    int stridey_8x  = stridey<<3;         /* luma stride x 8 */
    int stridey_12x = stridey*12;         /* luma stride x 12 */
    int strideuv_4x = strideuv<<2;        /* chroma stride x 4 */
	pixel *pixy  = h->fdec->plane[0] + ((mb_y*stridey )<<4);
	pixel *pixuv = h->fdec->plane[1] + ((mb_y*strideuv)<<3);
	int mb_xy    = mb_y*h->mb.i_mb_stride;
	int mb_x, intra_cur, first_edge_only;
	uint8_t (*bs)[8][4];

    for( mb_x = 0; mb_x < h->mb.i_mb_width; mb_x += 1 )
    {
    	/* FIXME: prefecth encode frame is unnecessary ? */
        /* x264_prefetch_fenc( h, h->fdec, mb_x, mb_y ); */

        /* deblock strength is calculated by x264_macroblock_deblock_strength */
        bs = h->deblock_strength[mb_y&1][mb_x];
        intra_cur = IS_INTRA( h->mb.type[mb_xy] );
        first_edge_only = h->mb.partition[mb_xy] == D_16x16 && !h->mb.cbp[mb_xy] && !intra_cur;

        /* horizontal deblocking of 4 luma lines and 2 chroma lines  */
        if( mb_x > 0 )
        {
			/* use intra deblock (BS = 4) if any block use intra predict. */
			if( intra_cur || IS_INTRA( h->mb.type[mb_xy-1] ) )
			{
				h->loopf.deblock_luma_intra[0]  ( pixy,  stridey,  alpha,  beta );
				h->loopf.deblock_chroma_intra[0]( pixuv, strideuv, alphac, betac );
			}
			else
			{
				deblock_edge( h, pixy,  stridey,  bs[0][0], index_a,  alpha,  beta,  0, h->loopf.deblock_luma[0] );
				deblock_edge( h, pixuv, strideuv, bs[0][0], index_ac, alphac, betac, 1, h->loopf.deblock_chroma[0] );
			}
        }
        if( !first_edge_only )
        {
        	deblock_edge( h, pixy + 4,  stridey,  bs[0][1], index_a,  alpha,  beta,  0, h->loopf.deblock_luma[0] );
        	deblock_edge( h, pixy + 8,  stridey,  bs[0][2], index_a,  alpha,  beta,  0, h->loopf.deblock_luma[0] );
        	deblock_edge( h, pixy + 12, stridey,  bs[0][3], index_a,  alpha,  beta,  0, h->loopf.deblock_luma[0] );
        	deblock_edge( h, pixuv + 8, strideuv, bs[0][2], index_ac, alphac, betac, 1, h->loopf.deblock_chroma[0] );
        }

        /* vertical deblocking of 4 luma lines and 2 chroma lines */
        if( mb_y > 0 )
        {
			/* use intra deblock (BS = 4) if any block use intra predict. */
			if( intra_cur || IS_INTRA( h->mb.type[mb_xy - h->mb.i_mb_stride] ) )
			{
				h->loopf.deblock_luma_intra[1]  ( pixy,  stridey,  alpha,  beta );
				h->loopf.deblock_chroma_intra[1]( pixuv, strideuv, alphac, betac );
			}
			else
			{
				deblock_edge( h, pixy,  stridey,  bs[1][0], index_a,  alpha,  beta,  0, h->loopf.deblock_luma[1] );
				deblock_edge( h, pixuv, strideuv, bs[1][0], index_ac, alphac, betac, 1, h->loopf.deblock_chroma[1] );
			}
        }
        if( !first_edge_only )
        {
        	deblock_edge( h, pixy + stridey_4x,   stridey,  bs[1][1], index_a,  alpha,  beta,  0, h->loopf.deblock_luma[1] );
			deblock_edge( h, pixy + stridey_8x,   stridey,  bs[1][2], index_a,  alpha,  beta,  0, h->loopf.deblock_luma[1] );
			deblock_edge( h, pixy + stridey_12x,  stridey,  bs[1][3], index_a,  alpha,  beta,  0, h->loopf.deblock_luma[1] );
			deblock_edge( h, pixuv + strideuv_4x, strideuv, bs[1][2], index_ac, alphac, betac, 1, h->loopf.deblock_chroma[1] );
        }

        mb_xy += 1;
        pixy += 16;
        pixuv += 16;
    }
}

void x264_deblock_init( int cpu, x264_deblock_function_t *pf )
{
	pf->deblock_luma[0] = deblock_h_luma_c;
    pf->deblock_luma[1] = deblock_v_luma_c;
    pf->deblock_chroma[0] = deblock_h_chroma_c;
    pf->deblock_chroma[1] = deblock_v_chroma_c;

    pf->deblock_luma_intra[0] = deblock_h_luma_intra_c;
    pf->deblock_luma_intra[1] = deblock_v_luma_intra_c;
    pf->deblock_chroma_intra[0] = deblock_h_chroma_intra_c;
    pf->deblock_chroma_intra[1] = deblock_v_chroma_intra_c;

    pf->deblock_strength = deblock_strength_c;

#ifdef __TI_COMPILER_VERSION__
    pf->deblock_luma[0] = deblock_h_luma_ti;
    pf->deblock_luma[1] = deblock_v_luma_ti;
    pf->deblock_chroma[0] = deblock_h_chroma_ti;
    pf->deblock_chroma[1] = deblock_v_chroma_ti;

    pf->deblock_luma_intra[0] = deblock_h_luma_intra_ti;
    pf->deblock_luma_intra[1] = deblock_v_luma_intra_ti;
    pf->deblock_chroma_intra[0] = deblock_h_chroma_intra_ti;
    pf->deblock_chroma_intra[1] = deblock_v_chroma_intra_ti;

    pf->deblock_strength = deblock_strength_ti;
#endif
}
