/*****************************************************************************
 * dct.h: transform and zigzag
 *****************************************************************************/

#ifndef X264_DCT_H
#define X264_DCT_H

typedef struct
{
    // pix1  stride = FENC_STRIDE
    // pix2  stride = FDEC_STRIDE
    // p_dst stride = FDEC_STRIDE
    void (*sub4x4_dct)   ( dctcoef dct[16], pixel *pix1, pixel *pix2 );     /* 4x4 dct  */
    void (*add4x4_idct)  ( pixel *p_dst, dctcoef dct[16] );                 /* 4x4 idct */

    void (*sub8x8_dct)   ( dctcoef dct[4][16], pixel *pix1, pixel *pix2 );  /* 8x8 dct */
    void (*sub8x8_dct_dc)( dctcoef dct[4], pixel *pix1, pixel *pix2 );      /* 8x8 dct dc (hadamard) */
    void (*add8x8_idct)  ( pixel *p_dst, dctcoef dct[4][16] );              /* 8x8 idct */
    void (*add8x8_idct_dc) ( pixel *p_dst, dctcoef dct[4] );                /* 8x8 idct dc (hadamard) */

    void (*sub16x16_dct) ( dctcoef dct[16][16], pixel *pix1, pixel *pix2 ); /* 16x16 dct */
    void (*add16x16_idct)( pixel *p_dst, dctcoef dct[16][16] );             /* 16x16 idct */
    void (*add16x16_idct_dc) ( pixel *p_dst, dctcoef dct[16] );             /* 16x16 idct dc (hadamard) */

    /* dct2x2dc is defined in macroblock.c as inline function */
    void (*dct4x4dc) ( dctcoef d[16] );                                     /* 4x4 hadamard transform */
    void (*idct4x4dc)( dctcoef d[16] );                                     /* 4x4 inverse hadamard transform */
} x264_dct_function_t;

typedef struct
{
    void (*scan_4x4)( dctcoef level[16], dctcoef dct[16] );
} x264_zigzag_function_t;

void x264_dct_init( int cpu, x264_dct_function_t *dctf );
void x264_zigzag_init( int cpu, x264_zigzag_function_t *zigzagf );

#endif
