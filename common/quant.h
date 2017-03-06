/*****************************************************************************
 * quant.h: quantization and level-run
 *****************************************************************************/

#ifndef X264_QUANT_H
#define X264_QUANT_H

typedef struct
{
    int (*quant_4x4)( dctcoef dct[16], udctcoef mf[16], udctcoef bias[16] );
    int (*quant_4x4_dc)( dctcoef dct[16], int mf, int bias );
    int (*quant_2x2_dc)( dctcoef dct[4], int mf, int bias );

    void (*dequant_4x4)( dctcoef dct[16], int dequant_mf[6][16], int i_qp );
    void (*dequant_4x4_dc)( dctcoef dct[16], int dequant_mf[6][16], int i_qp );

    int (*optimize_chroma_2x2_dc)( dctcoef dct[4], int dequant_mf );

    void (*denoise_dct)( dctcoef *dct, uint32_t *sum, udctcoef *offset, int size );

    int (*decimate_score15)( dctcoef *dct );
    int (*decimate_score16)( dctcoef *dct );

    int (*coeff_last[14])( dctcoef *dct );
    int (*coeff_last4)( dctcoef *dct );
    int (*coeff_last8)( dctcoef *dct );
    int (*coeff_level_run[13])( dctcoef *dct, x264_run_level_t *runlevel );
    int (*coeff_level_run4)( dctcoef *dct, x264_run_level_t *runlevel );
    int (*coeff_level_run8)( dctcoef *dct, x264_run_level_t *runlevel );

} x264_quant_function_t;

void x264_quant_init( x264_t *h, int cpu, x264_quant_function_t *pf );

#endif
