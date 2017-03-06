/*****************************************************************************
 * me.c: motion estimation
 *****************************************************************************/

#include "common/common.h"
#include "macroblock.h"
#include "me.h"

/******************************************************************************************
 * presets selected from good points on the speed-vs-quality curve of several test videos *
 * subpel_iters[i_subpel_refine] = { refine_hpel, refine_qpel, me_hpel, me_qpel }         *
 * where me_* are the number of EPZS(Enhanced Predictive Zonal Search) iterations run on  *
 * all candidate block types, and refine_* are run only on the winner.                    *
 *                                                                                        *
 * the subme=8,9 values are much higher because any amount of satd search makes           *
 * up its time by reducing the number of qpel-rd iterations.                              *
 ******************************************************************************************/
static const uint8_t subpel_iterations[][4] =
{
	{ 0, 0, 0, 0 },   /* subme = 0  (fullpel only) */
	{ 1, 1, 0, 0 },   /* subme = 1  (QPel SAD 1 iteration) */
	{ 0, 1, 1, 0 },   /* subme = 2  (QPel SATD 2 iterations) */
	{ 0, 2, 1, 0 },   /* subme = 3  (HPel on MB then QPel) */
	{ 0, 2, 1, 1 },   /* subme = 4  (Always QPel) */
	{ 0, 2, 1, 2 },   /* subme = 5  (Multi QPel + bi-directional me) */
	{ 0, 0, 2, 2 },   /* subme = 6  (RD on I/P frames) */
	{ 0, 0, 2, 2 },   /* subme = 7  (RD on all frames) */
	{ 0, 0, 4, 10 },  /* subme = 8  (RD refinement on I/P frames) */
	{ 0, 0, 4, 10 },  /* subme = 9  (RD refinement on all frames) */
	{ 0, 0, 4, 10 },  /* subme = 10 (QP-RD) */
	{ 0, 0, 4, 10 }   /* subme = 11 (Full RD) */
};

/* (x-1)%6 */
static const uint8_t mod6m1[8] = {5,0,1,2,3,4,5,0};
/* radius 2 hexagon. repeated entries are to avoid having to compute mod6 every time. */
static const int8_t hex2[8][2] = {{-1,-2}, {-2,0}, {-1,2}, {1,2}, {2,0}, {1,-2}, {-1,-2}, {-2,0}};
static const int8_t square1[9][2] = {{0,0}, {0,-1}, {0,1}, {-1,0}, {1,0}, {-1,-1}, {-1,1}, {1,-1}, {1,1}};

static void refine_subpel( x264_t *h, x264_me_t *m, int hpel_iters, int qpel_iters, int *p_halfpel_thresh, int b_refine_qpel );

#define BITS_MVD( mx, my ) (p_cost_mvx[(mx)<<2] + p_cost_mvy[(my)<<2])

/***********************************************************************
 * compute cost (sad + mvd_bits) of mv(mx, my) and compare with bcost. *
 * if cost is lower, set bcost = cost, bmx = mx, bmy = my.             *
 ***********************************************************************/
#define COST_MV( mx, my )\
{\
    int cost = h->pixf.fpelcmp[i_pixel]( p_fenc, FENC_STRIDE, &p_fref_w[(my)*stride+(mx)], stride ) + BITS_MVD(mx, my);\
    COPY3_IF_LT( bcost, cost, bmx, mx, bmy, my );\
}

/*********************************************************************************
 * compute cost (sad + mvd_bits) of half-mv(mx, my) and compare with bpred_cost. *
 * if cost is lower, set bpred_cost = cost, bpred_mx = mx, bpred_my = my.        *
 *********************************************************************************/
#define COST_MV_HPEL( mx, my ) \
{ \
    intptr_t stride2 = 16; \
    pixel *src = h->mc.get_ref( pix, &stride2, m->p_fref, stride, mx, my, bw, bh, &m->weight[0] ); \
    int cost = h->pixf.fpelcmp[i_pixel]( p_fenc, FENC_STRIDE, src, stride2 ) + p_cost_mvx[mx] + p_cost_mvy[my]; \
    COPY3_IF_LT( bpred_cost, cost, bpred_mx, mx, bpred_my, my ); \
}

/*****************************************************************
 * compute cost (sad + mvd_bits) of 3 mvs using sad_x3 for speed *
 * mv base: bmx, mv delta: (m0x, m0y), (m1x, m1y), (m2x, m2y)    *
 * all costs are saved to array for later comparison.            *
 *****************************************************************/
#define COST_MV_X3_DIR( m0x, m0y, m1x, m1y, m2x, m2y, costs )\
{\
    pixel *pix_base = p_fref_w + bmx + bmy*stride;\
    h->pixf.fpelcmp_x3[i_pixel]( p_fenc,\
        pix_base + (m0x) + (m0y)*stride,\
        pix_base + (m1x) + (m1y)*stride,\
        pix_base + (m2x) + (m2y)*stride,\
        stride, costs );\
    (costs)[0] += BITS_MVD( bmx+(m0x), bmy+(m0y) );\
    (costs)[1] += BITS_MVD( bmx+(m1x), bmy+(m1y) );\
    (costs)[2] += BITS_MVD( bmx+(m2x), bmy+(m2y) );\
}

/**************************************************************************
 * compute cost (sad + mvd_bits) of 4 mvs using sad_x4 for speed          *
 * mv base: bmx, mv delta: (m0x, m0y), (m1x, m1y), (m2x, m2y), (m3x, m3y) *
 * all costs are saved to array for later comparison.                     *
 **************************************************************************/
#define COST_MV_X4_DIR( m0x, m0y, m1x, m1y, m2x, m2y, m3x, m3y, costs )\
{\
    pixel *pix_base = p_fref_w + bmx + bmy*stride;\
    h->pixf.fpelcmp_x4[i_pixel]( p_fenc,\
        pix_base + (m0x) + (m0y)*stride,\
        pix_base + (m1x) + (m1y)*stride,\
        pix_base + (m2x) + (m2y)*stride,\
        pix_base + (m3x) + (m3y)*stride,\
        stride, costs );\
    (costs)[0] += BITS_MVD( bmx+(m0x), bmy+(m0y) );\
    (costs)[1] += BITS_MVD( bmx+(m1x), bmy+(m1y) );\
    (costs)[2] += BITS_MVD( bmx+(m2x), bmy+(m2y) );\
    (costs)[3] += BITS_MVD( bmx+(m3x), bmy+(m3y) );\
}

/**************************************************************************
 * compute cost (sad + mvd_bits) of 4 mvs using sad_x4 for speed          *
 * mv base: bmx, mv delta: (m0x, m0y), (m1x, m1y), (m2x, m2y), (m3x, m3y) *
 * lowest cost is saved to bcost, relative mv is also saved to bmx/bmy    *
 * bcost = costs[i], bmx = omx+(mix), bmy = omy+(miy)                     *
 **************************************************************************/
#define COST_MV_X4( m0x, m0y, m1x, m1y, m2x, m2y, m3x, m3y )\
{\
    pixel *pix_base = p_fref_w + omx + omy*stride;\
    h->pixf.fpelcmp_x4[i_pixel]( p_fenc,\
        pix_base + (m0x) + (m0y)*stride,\
        pix_base + (m1x) + (m1y)*stride,\
        pix_base + (m2x) + (m2y)*stride,\
        pix_base + (m3x) + (m3y)*stride,\
        stride, costs );\
    costs[0] += BITS_MVD( omx+(m0x), omy+(m0y) );\
    costs[1] += BITS_MVD( omx+(m1x), omy+(m1y) );\
    costs[2] += BITS_MVD( omx+(m2x), omy+(m2y) );\
    costs[3] += BITS_MVD( omx+(m3x), omy+(m3y) );\
    COPY3_IF_LT( bcost, costs[0], bmx, omx+(m0x), bmy, omy+(m0y) );\
    COPY3_IF_LT( bcost, costs[1], bmx, omx+(m1x), bmy, omy+(m1y) );\
    COPY3_IF_LT( bcost, costs[2], bmx, omx+(m2x), bmy, omy+(m2y) );\
    COPY3_IF_LT( bcost, costs[3], bmx, omx+(m3x), bmy, omy+(m3y) );\
}

void x264_me_search_ref( x264_t *h, x264_me_t *m, int16_t (*mvc)[2], int i_mvc, int *p_halfpel_thresh )
{
    const int bw = x264_pixel_size[m->i_pixel].w; /* block width */
    const int bh = x264_pixel_size[m->i_pixel].h; /* block height */
    const int i_pixel = m->i_pixel;
    const int stride = m->i_stride[0];
    int i_me_range = h->param.analyse.i_me_range; /* me range, should not be greater than 16 in DIA/HEX */
    int bmx, bmy, bcost;
    int bpred_mx = 0, bpred_my = 0, bpred_cost = COST_MAX;
    int pmx, pmy;
    pixel *p_fenc = m->p_fenc[0];
    pixel *p_fref_w = m->p_fref_w;
    ALIGNED_ARRAY_16( pixel, pix, [256] );

    int i;
	int costs[16];

    int mv_x_min = h->mb.mv_min_fpel[0];
    int mv_y_min = h->mb.mv_min_fpel[1];
    int mv_x_max = h->mb.mv_max_fpel[0];
    int mv_y_max = h->mb.mv_max_fpel[1];
    /* qpel = 4 * fpel */
    int mv_x_min_qpel = mv_x_min << 2;
    int mv_y_min_qpel = mv_y_min << 2;
    int mv_x_max_qpel = mv_x_max << 2;
    int mv_y_max_qpel = mv_y_max << 2;
/* Special version of pack to allow shortcuts in CHECK_MVRANGE */
#define pack16to32_mask2(mx, my) ((mx<<16)|(my&0x7FFF))
    uint32_t mv_min = pack16to32_mask2( -mv_x_min, -mv_y_min );
    uint32_t mv_max = pack16to32_mask2( mv_x_max, mv_y_max )|0x8000;

#define CHECK_MVRANGE(mx, my) (!(((pack16to32_mask2(mx,my) + mv_min) | (mv_max - pack16to32_mask2(mx,my))) & 0x80004000))

    const uint16_t *p_cost_mvx = m->p_cost_mv - m->mvp[0];
    const uint16_t *p_cost_mvy = m->p_cost_mv - m->mvp[1];

    uint32_t pmv;
    /* set mvp as start point of motion estimation */
    bmx = x264_clip3( m->mvp[0], mv_x_min_qpel, mv_x_max_qpel );
    bmy = x264_clip3( m->mvp[1], mv_y_min_qpel, mv_y_max_qpel );
    pmx = ( bmx + 2 ) >> 2;
    pmy = ( bmy + 2 ) >> 2;
    /* set COST_MAX as intial value of best cost */
    bcost = COST_MAX;

    /* try extra predictors if provided */
    /* find the best base point from mv candidates array: mvc */
    if( h->mb.i_subpel_refine >= 3 )
    {
        pmv = pack16to32_mask(bmx, bmy);
        if( i_mvc )
            COST_MV_HPEL( bmx, bmy );
        for( i = 0; i < i_mvc; i++ )
        {
            if( M32( mvc[i] ) && (pmv != M32( mvc[i] )) )
            {
                int mx = x264_clip3( mvc[i][0], mv_x_min_qpel, mv_x_max_qpel );
                int my = x264_clip3( mvc[i][1], mv_y_min_qpel, mv_y_max_qpel );
                COST_MV_HPEL( mx, my );
            }
        }
        bmx = ( bpred_mx + 2 ) >> 2;
        bmy = ( bpred_my + 2 ) >> 2;
        COST_MV( bmx, bmy );
    }
    else
    {
        /* check the MVP */
        bmx = pmx;
        bmy = pmy;
        /* Because we are rounding the predicted motion vector to fullpel, there will be
         * an extra MV cost in 15 out of 16 cases.  However, when the predicted MV is
         * chosen as the best predictor, it is often the case that the subpel search will
         * result in a vector at or next to the predicted motion vector. Therefore, it is
         * sensible to omit the cost of the MV from the rounded MVP to avoid unfairly
         * biasing against use of the predicted motion vector. */
        bcost = h->pixf.fpelcmp[i_pixel]( p_fenc, FENC_STRIDE, &p_fref_w[bmy*stride+bmx], stride );
        pmv = pack16to32_mask( bmx, bmy );
        if( i_mvc > 0 )
        {
        	/* round clip all mv candidates to full-pixel */
            ALIGNED_ARRAY_8( int16_t, mvc_fpel, [16], [2] );
            x264_predictor_roundclip( mvc_fpel+2, mvc, i_mvc, mv_x_min, mv_x_max, mv_y_min, mv_y_max );
            M32( mvc_fpel[1] ) = pmv;
            bcost <<= 4;
            for( i = 1; i <= i_mvc; i++ )
            {
                if( M32( mvc_fpel[i+1] ) && (pmv != M32( mvc_fpel[i+1] )) )
                {
                    int mx = mvc_fpel[i+1][0];
                    int my = mvc_fpel[i+1][1];
                    int cost = h->pixf.fpelcmp[i_pixel]( p_fenc, FENC_STRIDE, &p_fref_w[my*stride+mx], stride ) + BITS_MVD( mx, my );
                    cost = (cost << 4) + i;
                    COPY1_IF_LT( bcost, cost );
                }
            }
            bmx = mvc_fpel[(bcost&15)+1][0];
            bmy = mvc_fpel[(bcost&15)+1][1];
            bcost >>= 4;
        }
    }

    /* calculate cost of mv(0,0) as initial bcost */
    if( pmv )
        COST_MV( 0, 0 );

    switch( h->mb.i_me_method )
    {
        case X264_ME_DIA:
        {
            /****************************************
             * diamond search, radius 1             *
             *                                      *
             *    A      A: (0, -1)                 *
             *  /   \                               *
             * C  O  D   C: (-1, 0) D: (1,  0)      *
             *  \   /                               *
             *    B      B: (0, 1)                  *
             *                                      *
             ****************************************/
            int i = i_me_range;
            /* make sure the 4 low bits of bcost is zero. */
            /* 4 low bits are used to indicate indication. */
			bcost <<= 4;
            do
            {
            	/* compute cost (sad + mvd_bits) of 4 pixels relative to current (bmx, bmy) and save to costs. */
            	/* Diamond radius: 1, topology: top(0, -1), bottom(0, 1), left(-1, 0), right(1, 0) */
                COST_MV_X4_DIR( 0, -1, 0, 1, -1, 0, 1, 0, costs );
                /* compare costs with bcost and save the lowest to bcost */
                COPY1_IF_LT( bcost, (costs[0]<<4)+1 );  /* 0001 => dx:00, dy:01(+1) */
                COPY1_IF_LT( bcost, (costs[1]<<4)+3 );  /* 0011 => dx:00, dy:11(-1) */
                COPY1_IF_LT( bcost, (costs[2]<<4)+4 );  /* 0100 => dx:01(+1), dy:00 */
                COPY1_IF_LT( bcost, (costs[3]<<4)+12 ); /* 1100 => dx:11(-1), dy:00 */
                /* check if this round has changed bcost. if not, stop search. */
                if( !(bcost&15) )
                    break;

                /* move base point to the lowest cost position */
                bmx -= (bcost<<28)>>30; /* extract 2 bits (28,29) of bcost to indicate dx */
                bmy -= (bcost<<30)>>30; /* extract 2 bits (30,31) of bcost to indicate dy */
                bcost &= ~15;           /* reset 4 low bits of bcost */
            } while( --i && CHECK_MVRANGE(bmx, bmy) );
            bcost >>= 4;
            break;
        }

        case X264_ME_HEX:
        {
            /****************************************
             * hexagon search, radius 2             *
             *                                      *
             *   F --- E     F: (-1, -2) E: (1, -2) *
             *  /       \                           *
             * A    O    D   A: (-2,  0) D: (2,  0) *
             *  \       /                           *
             *   B --- C     B: (-1,  2) C: (1,  2) *
             *                                      *
             ****************************************/
			int dir;
#if 0
            for( int i = 0; i < (i_me_range>>1); i++ )
            {
                omx = bmx; omy = bmy;
                COST_MV( omx-2, omy   ); /* A */
                COST_MV( omx-1, omy+2 ); /* B */
                COST_MV( omx+1, omy+2 ); /* C */
                COST_MV( omx+2, omy   ); /* D */
                COST_MV( omx+1, omy-2 ); /* E */
                COST_MV( omx-1, omy-2 ); /* F */
                if( bmx == omx && bmy == omy )
                    break;
                if( !CHECK_MVRANGE(bmx, bmy) )
                    break;
            }
#else
            /* equivalent to the above, but eliminates duplicate candidates */

            /* first time hexagon search */
            COST_MV_X3_DIR( -2, 0, -1,  2,  1,  2, costs   ); /* A, B, C */
            COST_MV_X3_DIR(  2, 0,  1, -2, -1, -2, costs+3 ); /* D, E, F */
            /* make sure the 3 low bits of bcost is zero. */
            /* 3 low bits are used to indicate indication. */
            bcost <<= 3;
            COPY1_IF_LT( bcost, (costs[0]<<3)+2 );
            COPY1_IF_LT( bcost, (costs[1]<<3)+3 );
            COPY1_IF_LT( bcost, (costs[2]<<3)+4 );
            COPY1_IF_LT( bcost, (costs[3]<<3)+5 );
            COPY1_IF_LT( bcost, (costs[4]<<3)+6 );
            COPY1_IF_LT( bcost, (costs[5]<<3)+7 );

            /* check if this round has changed bcost. if not, stop search. */
            if( bcost&7 )
            {
            	/* move base point to the lowest cost position */
                int dir = (bcost&7)-2;
                bmx += hex2[dir+1][0];
                bmy += hex2[dir+1][1];

                /********************************************************
                 * half hexagon, not overlapping the previous iteration *
                 * 1. last hexagon: ABCDEF, suppose lowest cost is D.   *
                 * 2. now base point: D, new hexagon: OCGHIE.           *
                 * 3. since O,C,E is overlapped, only cosider G, H, I.  *
                 *                                                      *
                 *   F --- E --- I                                      *
                 *  /    /  \     \                                     *
                 * A    O    D     H                                    *
                 *  \    \  /     /                                     *
                 *   B --- C --- G                                      *
                 *                                                      *
                 ********************************************************/
                for( i = (i_me_range>>1) - 1; i > 0 && CHECK_MVRANGE(bmx, bmy); i-- )
                {
                    COST_MV_X3_DIR( hex2[dir+0][0], hex2[dir+0][1], /* G */
                                    hex2[dir+1][0], hex2[dir+1][1], /* H */
                                    hex2[dir+2][0], hex2[dir+2][1], /* I */
                                    costs );
                    bcost &= ~7;
                    COPY1_IF_LT( bcost, (costs[0]<<3)+1 );
                    COPY1_IF_LT( bcost, (costs[1]<<3)+2 );
                    COPY1_IF_LT( bcost, (costs[2]<<3)+3 );
                    if( !(bcost&7) )
                        break;
                    dir += (bcost&7)-2;
                    dir = mod6m1[dir+1];
                    bmx += hex2[dir+1][0];
                    bmy += hex2[dir+1][1];
                }
            }
            bcost >>= 3;
#endif
            /* final square refinement */
            dir = 0;
            /****************************************
             *    A      A: (0, -1)                 *
             *  /   \                               *
             * C  O  D   C: (-1, 0) D: (1,  0)      *
             *  \   /                               *
             *    B      B: (0, 1)                  *
             ****************************************/
            COST_MV_X4_DIR( 0, -1, 0, 1, -1, 0, 1, 0, costs );
            COPY2_IF_LT( bcost, costs[0], dir, 1 );
            COPY2_IF_LT( bcost, costs[1], dir, 2 );
            COPY2_IF_LT( bcost, costs[2], dir, 3 );
            COPY2_IF_LT( bcost, costs[3], dir, 4 );
            /****************************************
             * A --- C   A: (-1, -1) C: (1, -1)     *
             * |  O  |                              *
             * B --- D   B: (-1,  1) D: (1,  1)     *
             ****************************************/
            COST_MV_X4_DIR( -1, -1, -1, 1, 1, -1, 1, 1, costs );
            COPY2_IF_LT( bcost, costs[0], dir, 5 );
            COPY2_IF_LT( bcost, costs[1], dir, 6 );
            COPY2_IF_LT( bcost, costs[2], dir, 7 );
            COPY2_IF_LT( bcost, costs[3], dir, 8 );
            bmx += square1[dir][0];
            bmy += square1[dir][1];
            break;
        }
        /*
        case X264_ME_UMH:
        case X264_ME_ESA:
        case X264_ME_TESA:
        break;
        */
    }

    /* compare bcost with bpred_cost and save qpel mv */
    if( bpred_cost < bcost )
    {
        m->mv[0] = bpred_mx;
        m->mv[1] = bpred_my;
        m->cost = bpred_cost;
    }
    else
    {
        m->mv[0] = bmx << 2; /* save mvx */
        m->mv[1] = bmy << 2; /* save mvy */
        m->cost = bcost;     /* save cost */
    }

    /* compute the real cost */
    m->cost_mv = p_cost_mvx[ m->mv[0] ] + p_cost_mvy[ m->mv[1] ];
    if( bmx == pmx && bmy == pmy && h->mb.i_subpel_refine < 3 )
        m->cost += m->cost_mv;

    /* subpel refine */
    if( h->mb.i_subpel_refine >= 2 )
    {
        int hpel = subpel_iterations[h->mb.i_subpel_refine][2]; /* iters for half-pixel */
        int qpel = subpel_iterations[h->mb.i_subpel_refine][3]; /* iters for quat-pixel */
        refine_subpel( h, m, hpel, qpel, p_halfpel_thresh, 0 );
    }
}
#undef COST_MV

void x264_me_refine_qpel( x264_t *h, x264_me_t *m )
{
    int hpel = subpel_iterations[h->mb.i_subpel_refine][0]; /* iters for half-pixel */
    int qpel = subpel_iterations[h->mb.i_subpel_refine][1]; /* iters for quat-pixel */

    if( m->i_pixel <= PIXEL_8x8 )
        m->cost -= m->i_ref_cost;

    refine_subpel( h, m, hpel, qpel, NULL, 1 );
}

void x264_me_refine_qpel_refdupe( x264_t *h, x264_me_t *m, int *p_halfpel_thresh )
{
    refine_subpel( h, m, 0, X264_MIN( 2, subpel_iterations[h->mb.i_subpel_refine][3] ), p_halfpel_thresh, 0 );
}

#define COST_MV_SAD( mx, my ) \
{ \
    intptr_t stride = 16; \
    pixel *src = h->mc.get_ref( pix, &stride, m->p_fref, m->i_stride[0], mx, my, bw, bh, &m->weight[0] ); \
    int cost = h->pixf.fpelcmp[i_pixel]( m->p_fenc[0], FENC_STRIDE, src, stride ) + p_cost_mvx[mx] + p_cost_mvy[my]; \
    COPY3_IF_LT( bcost, cost, bmx, mx, bmy, my ); \
}

#define COST_MV_SATD( mx, my, dir ) \
if( b_refine_qpel || (dir^1) != odir ) \
{ \
    intptr_t stride = 16; \
    pixel *src = h->mc.get_ref( pix, &stride, m->p_fref, m->i_stride[0], mx, my, bw, bh, &m->weight[0] ); \
    int cost = h->pixf.mbcmp_unaligned[i_pixel]( m->p_fenc[0], FENC_STRIDE, src, stride ) + p_cost_mvx[mx] + p_cost_mvy[my]; \
    COPY4_IF_LT( bcost, cost, bmx, mx, bmy, my, bdir, dir ); \
}

/*************************************************************
 * refine_subpel: sub-pixel refinement                       *
 * this function refines in 1/2 and 1/4 sub-pixel precision. *
 * @ hpel_iters: half-pixel refine iters                     *
 * @ qpel_iters: quat-pixel refine iters                     *
 * @ p_halfpel_thresh: half-pixel thresh                     *
 *************************************************************/
static void refine_subpel( x264_t *h, x264_me_t *m, int hpel_iters, int qpel_iters, int *p_halfpel_thresh, int b_refine_qpel )
{
    const int bw = x264_pixel_size[m->i_pixel].w;
    const int bh = x264_pixel_size[m->i_pixel].h;
    const uint16_t *p_cost_mvx = m->p_cost_mv - m->mvp[0];
    const uint16_t *p_cost_mvy = m->p_cost_mv - m->mvp[1];
    const int i_pixel = m->i_pixel;
    /* chrome me should only be used in large block (16x16,16x8,8x16,8x8) */
    /* const int b_chroma_me = h->mb.b_chroma_me && (i_pixel <= PIXEL_8x8); */
    ALIGNED_ARRAY_16( pixel, pix, [1152] ); // really 17x17x2, but round up to 64x18 for alignment
	int i;

    int bmx = m->mv[0];
    int bmy = m->mv[1];
    int bcost = m->cost;
    int odir = -1, bdir;

    /* try the subpel component of the predicted mv */
    if( hpel_iters && h->mb.i_subpel_refine < 3 )
    {
        int mx = x264_clip3( m->mvp[0], h->mb.mv_min_spel[0]+2, h->mb.mv_max_spel[0]-2 );
        int my = x264_clip3( m->mvp[1], h->mb.mv_min_spel[1]+2, h->mb.mv_max_spel[1]-2 );
        if( (mx-bmx)|(my-bmy) )
            COST_MV_SAD( mx, my );
    }

    /* half-pixel diamond search */
    for( i = hpel_iters; i > 0; i-- )
    {
        int omx = bmx, omy = bmy;
        int costs[4];
        intptr_t stride = 64; // candidates are either all hpel or all qpel, so one stride is enough
        pixel *src0, *src1, *src2, *src3;

        /****************************************************
         * extract 4 half-pixel pointers around O(omx, omy) *
         *       src0                                       *
         * src2   O    src3                                 *
         *       src1                                       *
         ****************************************************/
        src0 = h->mc.get_ref( pix,    &stride, m->p_fref, m->i_stride[0], omx, omy-2, bw, bh+1, &m->weight[0] );
        src2 = h->mc.get_ref( pix+32, &stride, m->p_fref, m->i_stride[0], omx-2, omy, bw+4, bh, &m->weight[0] );
        src1 = src0 + stride;
        src3 = src2 + 1;
        h->pixf.fpelcmp_x4[i_pixel]( m->p_fenc[0], src0, src1, src2, src3, stride, costs );
        COPY2_IF_LT( bcost, costs[0] + p_cost_mvx[omx  ] + p_cost_mvy[omy-2], bmy, omy-2 );
        COPY2_IF_LT( bcost, costs[1] + p_cost_mvx[omx  ] + p_cost_mvy[omy+2], bmy, omy+2 );
        COPY3_IF_LT( bcost, costs[2] + p_cost_mvx[omx-2] + p_cost_mvy[omy  ], bmx, omx-2, bmy, omy );
        COPY3_IF_LT( bcost, costs[3] + p_cost_mvx[omx+2] + p_cost_mvy[omy  ], bmx, omx+2, bmy, omy );
        if( (bmx == omx) & (bmy == omy) )
            break;
    }

    /* following is only used for me refinement of all candidate blocks, not the last refinement of winner */
    if( !b_refine_qpel && h->pixf.mbcmp_unaligned[0] != h->pixf.fpelcmp[0] )
    {
        bcost = COST_MAX;
        COST_MV_SATD( bmx, bmy, -1 );
    }

    /* early termination when examining multiple reference frames in P/B_16x16 analyse */
    if( p_halfpel_thresh )
    {
        if( (bcost*7)>>3 > *p_halfpel_thresh )
        {
            m->cost = bcost;
            m->mv[0] = bmx;
            m->mv[1] = bmy;
            // don't need cost_mv
            return;
        }
        else if( bcost < *p_halfpel_thresh )
            *p_halfpel_thresh = bcost;
    }

    /* quat-pixel diamond search */
    if( h->mb.i_subpel_refine != 1 )
    {
        bdir = -1;
        for( i = qpel_iters; i > 0; i-- )
        {
			int omx, omy;
            if( bmy <= h->mb.mv_min_spel[1] || bmy >= h->mb.mv_max_spel[1] ||
            	bmx <= h->mb.mv_min_spel[0] || bmx >= h->mb.mv_max_spel[0] )
                break;

            /* save best dir, mx/my of last iter */
            odir = bdir;
            omx = bmx;
			omy = bmy;
			/* skip already done block using condition: (dir^1) != odir */
            COST_MV_SATD( omx, omy - 1, 0 );
            COST_MV_SATD( omx, omy + 1, 1 );
            COST_MV_SATD( omx - 1, omy, 2 );
            COST_MV_SATD( omx + 1, omy, 3 );
            if( (bmx == omx) & (bmy == omy) )
                break;
        }
    }
    else if( bmy > h->mb.mv_min_spel[1] && bmy < h->mb.mv_max_spel[1] &&
    		 bmx > h->mb.mv_min_spel[0] && bmx < h->mb.mv_max_spel[0] )
    {
    	/* Special simplified case for subme=1 */
        int costs[4];
        int omx = bmx, omy = bmy;
        /* We have to use mc_luma because all strides must be the same to use fpelcmp_x4 */
        h->mc.mc_luma( pix   , 64, m->p_fref, m->i_stride[0], omx, omy-1, bw, bh, &m->weight[0] );
        h->mc.mc_luma( pix+16, 64, m->p_fref, m->i_stride[0], omx, omy+1, bw, bh, &m->weight[0] );
        h->mc.mc_luma( pix+32, 64, m->p_fref, m->i_stride[0], omx-1, omy, bw, bh, &m->weight[0] );
        h->mc.mc_luma( pix+48, 64, m->p_fref, m->i_stride[0], omx+1, omy, bw, bh, &m->weight[0] );
        h->pixf.fpelcmp_x4[i_pixel]( m->p_fenc[0], pix, pix+16, pix+32, pix+48, 64, costs );
        COPY2_IF_LT( bcost, costs[0] + p_cost_mvx[omx  ] + p_cost_mvy[omy-1], bmy, omy-1 );
        COPY2_IF_LT( bcost, costs[1] + p_cost_mvx[omx  ] + p_cost_mvy[omy+1], bmy, omy+1 );
        COPY3_IF_LT( bcost, costs[2] + p_cost_mvx[omx-1] + p_cost_mvy[omy  ], bmx, omx-1, bmy, omy );
        COPY3_IF_LT( bcost, costs[3] + p_cost_mvx[omx+1] + p_cost_mvy[omy  ], bmx, omx+1, bmy, omy );
    }

    m->cost = bcost;
    m->mv[0] = bmx;
    m->mv[1] = bmy;
    m->cost_mv = p_cost_mvx[bmx] + p_cost_mvy[bmy];
}
