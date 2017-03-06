/*****************************************************************************
 * dct.c: transform and zigzag
 *****************************************************************************/

#include "common.h"

#ifdef __TI_COMPILER_VERSION__
void sub4x4_dct_ti     ( dctcoef dct[16], pixel *pix1, pixel *pix2 );
void add4x4_idct_ti    ( pixel *p_dst, dctcoef dct[16] );

void sub8x8_dct_ti     ( dctcoef dct[4][16], pixel *pix1, pixel *pix2 );
void sub8x8_dct_dc_ti  ( dctcoef dct[4], pixel *pix1, pixel *pix2 );
void add8x8_idct_ti    ( pixel *p_dst, dctcoef dct[4][16] );
void add8x8_idct_dc_ti ( pixel *p_dst, dctcoef dct[4] );

static void add16x16_idct_ti( pixel *p_dst, dctcoef dct[16][16] )
{
    add8x8_idct_ti( &p_dst[  0], &dct[ 0] );
    add8x8_idct_ti( &p_dst[  8], &dct[ 4] );
    add8x8_idct_ti( &p_dst[256], &dct[ 8] ); /* p_dst[8*FDEC_STRIDE+0] */
    add8x8_idct_ti( &p_dst[264], &dct[12] ); /* p_dst[8*FDEC_STRIDE+8] */
}
static void sub16x16_dct_ti( dctcoef dct[16][16], pixel *pix1, pixel *pix2 )
{
    sub8x8_dct_ti( &dct[ 0], &pix1[  0], &pix2[  0] );
    sub8x8_dct_ti( &dct[ 4], &pix1[  8], &pix2[  8] );
    sub8x8_dct_ti( &dct[ 8], &pix1[128], &pix2[256] ); /* pix1[8*FENC_STRIDE+0], pix2[8*FDEC_STRIDE+0] */
    sub8x8_dct_ti( &dct[12], &pix1[136], &pix2[264] ); /* pix1[8*FENC_STRIDE+8], pix2[8*FDEC_STRIDE+8] */
}
void add16x16_idct_dc_ti ( pixel *p_dst, dctcoef dct[16] );

void dct4x4dc_ti   ( dctcoef d[16] );
void idct4x4dc_ti  ( dctcoef d[16] );
#endif

static void dct4x4dc( dctcoef d[16] )
{
    dctcoef tmp[16];
	int i;

    /* two-pass butterfly operation for fast hadamard */
    /* first pass by columns, second pass by rows */
    for( i = 0; i < 4; i++ )
    {
        int s01 = d[i*4+0] + d[i*4+1]; /* x0 + x1 */
        int d01 = d[i*4+0] - d[i*4+1]; /* x0 - x1 */
        int s23 = d[i*4+2] + d[i*4+3]; /* x2 + x3 */
        int d23 = d[i*4+2] - d[i*4+3]; /* x2 - x3 */

        tmp[0*4+i] = s01 + s23;        /* X0 = (x0 + x1) + (x2 + x3) */
        tmp[1*4+i] = s01 - s23;        /* X1 = (x0 + x1) - (x2 + x3) */
        tmp[2*4+i] = d01 - d23;        /* X2 = (x0 - x1) - (x2 - x3) */
        tmp[3*4+i] = d01 + d23;        /* X3 = (x0 - x1) + (x2 - x3) */
    }

    for( i = 0; i < 4; i++ )
    {
        int s01 = tmp[i*4+0] + tmp[i*4+1];
        int d01 = tmp[i*4+0] - tmp[i*4+1];
        int s23 = tmp[i*4+2] + tmp[i*4+3];
        int d23 = tmp[i*4+2] - tmp[i*4+3];

        d[i*4+0] = ( s01 + s23 + 1 ) >> 1;
        d[i*4+1] = ( s01 - s23 + 1 ) >> 1;
        d[i*4+2] = ( d01 - d23 + 1 ) >> 1;
        d[i*4+3] = ( d01 + d23 + 1 ) >> 1;
    }
}

static void idct4x4dc( dctcoef d[16] )
{
    dctcoef tmp[16];
	int i;

    for( i = 0; i < 4; i++ )
    {
        int s01 = d[i*4+0] + d[i*4+1];
        int d01 = d[i*4+0] - d[i*4+1];
        int s23 = d[i*4+2] + d[i*4+3];
        int d23 = d[i*4+2] - d[i*4+3];

        tmp[0*4+i] = s01 + s23;
        tmp[1*4+i] = s01 - s23;
        tmp[2*4+i] = d01 - d23;
        tmp[3*4+i] = d01 + d23;
    }

    for( i = 0; i < 4; i++ )
    {
        int s01 = tmp[i*4+0] + tmp[i*4+1];
        int d01 = tmp[i*4+0] - tmp[i*4+1];
        int s23 = tmp[i*4+2] + tmp[i*4+3];
        int d23 = tmp[i*4+2] - tmp[i*4+3];

        d[i*4+0] = s01 + s23;
        d[i*4+1] = s01 - s23;
        d[i*4+2] = d01 - d23;
        d[i*4+3] = d01 + d23;
    }
}

static inline void pixel_sub_wxh( dctcoef *diff, int i_size,
                                  pixel *pix1, int i_pix1, pixel *pix2, int i_pix2 )
{
	int y, x;
    for( y = 0; y < i_size; y++ )
    {
        for( x = 0; x < i_size; x++ )
            diff[x + y*i_size] = pix1[x] - pix2[x];
        pix1 += i_pix1;
        pix2 += i_pix2;
    }
}

static void sub4x4_dct( dctcoef dct[16], pixel *pix1, pixel *pix2 )
{
    dctcoef d[16];
    dctcoef tmp[16];
	int i;

    pixel_sub_wxh( d, 4, pix1, FENC_STRIDE, pix2, FDEC_STRIDE );

    /* two-pass butterfly operation for fast dct */
    /* first pass by columns, second pass by rows */
    for( i = 0; i < 4; i++ )
    {
        int s03 = d[i*4+0] + d[i*4+3]; /* x0 + x3 */
        int s12 = d[i*4+1] + d[i*4+2]; /* x1 + x2 */
        int d03 = d[i*4+0] - d[i*4+3]; /* x0 - x3 */
        int d12 = d[i*4+1] - d[i*4+2]; /* x1 - x2 */

        tmp[0*4+i] =   s03 +   s12;    /* X0 = 1 * (x0 + x3) + 1 * (x1 + x2) */
        tmp[1*4+i] = 2*d03 +   d12;    /* X1 = 2 * (x0 - x3) + 1 * (x1 - x2) */
        tmp[2*4+i] =   s03 -   s12;    /* X2 = 1 * (x0 + x3) - 1 * (x1 + x2) */
        tmp[3*4+i] =   d03 - 2*d12;    /* X3 = 1 * (x0 - x3) - 2 * (x1 - x2) */
    }

    for( i = 0; i < 4; i++ )
    {
        int s03 = tmp[i*4+0] + tmp[i*4+3];
        int s12 = tmp[i*4+1] + tmp[i*4+2];
        int d03 = tmp[i*4+0] - tmp[i*4+3];
        int d12 = tmp[i*4+1] - tmp[i*4+2];

        dct[i*4+0] =   s03 +   s12;
        dct[i*4+1] = 2*d03 +   d12;
        dct[i*4+2] =   s03 -   s12;
        dct[i*4+3] =   d03 - 2*d12;
    }
}

static void sub8x8_dct( dctcoef dct[4][16], pixel *pix1, pixel *pix2 )
{
    sub4x4_dct( dct[0], &pix1[0], &pix2[0] );
    sub4x4_dct( dct[1], &pix1[4], &pix2[4] );
    sub4x4_dct( dct[2], &pix1[4*FENC_STRIDE+0], &pix2[4*FDEC_STRIDE+0] );
    sub4x4_dct( dct[3], &pix1[4*FENC_STRIDE+4], &pix2[4*FDEC_STRIDE+4] );
}

static void sub16x16_dct( dctcoef dct[16][16], pixel *pix1, pixel *pix2 )
{
    sub8x8_dct( &dct[ 0], &pix1[0], &pix2[0] );
    sub8x8_dct( &dct[ 4], &pix1[8], &pix2[8] );
    sub8x8_dct( &dct[ 8], &pix1[8*FENC_STRIDE+0], &pix2[8*FDEC_STRIDE+0] );
    sub8x8_dct( &dct[12], &pix1[8*FENC_STRIDE+8], &pix2[8*FDEC_STRIDE+8] );
}

static int sub4x4_dct_dc( pixel *pix1, pixel *pix2 )
{
    int sum = 0;
	int i;
    for( i=0; i<4; i++, pix1 += FENC_STRIDE, pix2 += FDEC_STRIDE )
        sum += pix1[0] + pix1[1] + pix1[2] + pix1[3]
             - pix2[0] - pix2[1] - pix2[2] - pix2[3];
    return sum;
}

static void sub8x8_dct_dc( dctcoef dct[4], pixel *pix1, pixel *pix2 )
{
	int d0, d1, d2, d3;
    dct[0] = sub4x4_dct_dc( &pix1[0], &pix2[0] );
    dct[1] = sub4x4_dct_dc( &pix1[4], &pix2[4] );
    dct[2] = sub4x4_dct_dc( &pix1[4*FENC_STRIDE+0], &pix2[4*FDEC_STRIDE+0] );
    dct[3] = sub4x4_dct_dc( &pix1[4*FENC_STRIDE+4], &pix2[4*FDEC_STRIDE+4] );

    /* 2x2 DC transform */
    d0 = dct[0] + dct[1];
    d1 = dct[2] + dct[3];
    d2 = dct[0] - dct[1];
    d3 = dct[2] - dct[3];
    dct[0] = d0 + d1;
    dct[1] = d0 - d1;
    dct[2] = d2 + d3;
    dct[3] = d2 - d3;
}

static void add4x4_idct( pixel *p_dst, dctcoef dct[16] )
{
    dctcoef d[16];
    dctcoef tmp[16];
	int x, y, i;

    for( i = 0; i < 4; i++ )
    {
        int s02 =  dct[0*4+i]     +  dct[2*4+i];
        int d02 =  dct[0*4+i]     -  dct[2*4+i];
        int s13 =  dct[1*4+i]     + (dct[3*4+i]>>1);
        int d13 = (dct[1*4+i]>>1) -  dct[3*4+i];

        tmp[i*4+0] = s02 + s13;
        tmp[i*4+1] = d02 + d13;
        tmp[i*4+2] = d02 - d13;
        tmp[i*4+3] = s02 - s13;
    }

    for( i = 0; i < 4; i++ )
    {
        int s02 =  tmp[0*4+i]     +  tmp[2*4+i];
        int d02 =  tmp[0*4+i]     -  tmp[2*4+i];
        int s13 =  tmp[1*4+i]     + (tmp[3*4+i]>>1);
        int d13 = (tmp[1*4+i]>>1) -  tmp[3*4+i];

        d[0*4+i] = ( s02 + s13 + 32 ) >> 6;
        d[1*4+i] = ( d02 + d13 + 32 ) >> 6;
        d[2*4+i] = ( d02 - d13 + 32 ) >> 6;
        d[3*4+i] = ( s02 - s13 + 32 ) >> 6;
    }

    for( y = 0; y < 4; y++ )
    {
        for( x = 0; x < 4; x++ )
            p_dst[x] = x264_clip_pixel( p_dst[x] + d[y*4+x] );
        p_dst += FDEC_STRIDE;
    }
}

static void add8x8_idct( pixel *p_dst, dctcoef dct[4][16] )
{
    add4x4_idct( &p_dst[0],               dct[0] );
    add4x4_idct( &p_dst[4],               dct[1] );
    add4x4_idct( &p_dst[4*FDEC_STRIDE+0], dct[2] );
    add4x4_idct( &p_dst[4*FDEC_STRIDE+4], dct[3] );
}

static void add16x16_idct( pixel *p_dst, dctcoef dct[16][16] )
{
    add8x8_idct( &p_dst[0],               &dct[0] );
    add8x8_idct( &p_dst[8],               &dct[4] );
    add8x8_idct( &p_dst[8*FDEC_STRIDE+0], &dct[8] );
    add8x8_idct( &p_dst[8*FDEC_STRIDE+8], &dct[12] );
}

static void inline add4x4_idct_dc( pixel *p_dst, dctcoef dc )
{
	int i;
    dc = (dc + 32) >> 6;
    for( i = 0; i < 4; i++, p_dst += FDEC_STRIDE )
    {
        p_dst[0] = x264_clip_pixel( p_dst[0] + dc );
        p_dst[1] = x264_clip_pixel( p_dst[1] + dc );
        p_dst[2] = x264_clip_pixel( p_dst[2] + dc );
        p_dst[3] = x264_clip_pixel( p_dst[3] + dc );
    }
}

static void add8x8_idct_dc( pixel *p_dst, dctcoef dct[4] )
{
    add4x4_idct_dc( &p_dst[0],               dct[0] );
    add4x4_idct_dc( &p_dst[4],               dct[1] );
    add4x4_idct_dc( &p_dst[4*FDEC_STRIDE+0], dct[2] );
    add4x4_idct_dc( &p_dst[4*FDEC_STRIDE+4], dct[3] );
}

static void add16x16_idct_dc( pixel *p_dst, dctcoef dct[16] )
{
	int i;
    for( i = 0; i < 4; i++, dct += 4, p_dst += 4*FDEC_STRIDE )
    {
        add4x4_idct_dc( &p_dst[ 0], dct[0] );
        add4x4_idct_dc( &p_dst[ 4], dct[1] );
        add4x4_idct_dc( &p_dst[ 8], dct[2] );
        add4x4_idct_dc( &p_dst[12], dct[3] );
    }
}


/****************************************************************************
 * x264_dct_init:
 ****************************************************************************/
void x264_dct_init( int cpu, x264_dct_function_t *dctf )
{
    dctf->sub4x4_dct    = sub4x4_dct;
    dctf->add4x4_idct   = add4x4_idct;

    dctf->sub8x8_dct    = sub8x8_dct;
    dctf->sub8x8_dct_dc = sub8x8_dct_dc;
    dctf->add8x8_idct   = add8x8_idct;
    dctf->add8x8_idct_dc = add8x8_idct_dc;

    dctf->sub16x16_dct  = sub16x16_dct;
    dctf->add16x16_idct = add16x16_idct;
    dctf->add16x16_idct_dc = add16x16_idct_dc;

    dctf->dct4x4dc  = dct4x4dc;
    dctf->idct4x4dc = idct4x4dc;

#ifdef __TI_COMPILER_VERSION__
    dctf->sub4x4_dct  = sub4x4_dct_ti;
    dctf->add4x4_idct = add4x4_idct_ti;

    dctf->sub8x8_dct    = sub8x8_dct_ti;
    dctf->sub8x8_dct_dc = sub8x8_dct_dc_ti;
    dctf->add8x8_idct   = add8x8_idct_ti;
    dctf->add8x8_idct_dc = add8x8_idct_dc_ti;

    dctf->sub16x16_dct  = sub16x16_dct_ti;
    dctf->add16x16_idct = add16x16_idct_ti;
    dctf->add16x16_idct_dc = add16x16_idct_dc_ti;

    dctf->dct4x4dc  = dct4x4dc_ti;
    dctf->idct4x4dc = idct4x4dc_ti;
#endif
}

#ifdef __TI_COMPILER_VERSION__
void zigzag_scan_4x4_frame_ti( dctcoef level[16], dctcoef dct[16] );
#endif

static void zigzag_scan_4x4_frame( dctcoef level[16], dctcoef dct[16] )
{
    level[0] = dct[0*4+0];
    level[1] = dct[1*4+0];
    level[2] = dct[0*4+1];
    level[3] = dct[0*4+2];
    level[4] = dct[1*4+1];
    level[5] = dct[2*4+0];
    level[6] = dct[3*4+0];
    level[7] = dct[2*4+1];
    level[8] = dct[1*4+2];
    level[9] = dct[0*4+3];
    level[10] = dct[1*4+3];
    level[11] = dct[2*4+2];
    level[12] = dct[3*4+1];
    level[13] = dct[3*4+2];
    level[14] = dct[2*4+3];
    level[15] = dct[3*4+3];
}

void x264_zigzag_init( int cpu, x264_zigzag_function_t *zigzagf )
{
    zigzagf->scan_4x4  = zigzag_scan_4x4_frame;
#ifdef __TI_COMPILER_VERSION__
    zigzagf->scan_4x4  = zigzag_scan_4x4_frame_ti;
#endif
}
