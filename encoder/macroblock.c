/*****************************************************************************
 * macroblock.c: macroblock encoding
 *****************************************************************************/

#include "common/common.h"
#include "macroblock.h"

/* These chroma DC functions don't have assembly versions and are only used here. */
static inline void zigzag_scan_2x2_dc( dctcoef level[4], dctcoef dct[4] )
{
	level[0] = dct[0];
	level[1] = dct[2];
	level[2] = dct[1];
	level[3] = dct[3];
}

static inline void idct_dequant_2x2_dc( dctcoef dct[4], dctcoef dct4x4[4][16], int dequant_mf[6][16], int i_qp )
{
	/* (dequant_mf[i_qp%6][0] << i_qp/6) >> 5 */
	int dmf = (dequant_mf[x264_dequant_div_lut[i_qp][1]][0] << x264_dequant_div_lut[i_qp][0]) >> 5;
	int d0 = dct[0] + dct[1];
	int d1 = dct[2] + dct[3];
	int d2 = dct[0] - dct[1];
	int d3 = dct[2] - dct[3];
    dct4x4[0][0] = (d0 + d1) * dmf;
    dct4x4[1][0] = (d0 - d1) * dmf;
    dct4x4[2][0] = (d2 + d3) * dmf;
    dct4x4[3][0] = (d2 - d3) * dmf;
}

static inline void idct_dequant_2x2_dconly( dctcoef dct[4], int dequant_mf[6][16], int i_qp )
{
	/* (dequant_mf[i_qp%6][0] << i_qp/6) >> 5 */
	int dmf = (dequant_mf[x264_dequant_div_lut[i_qp][1]][0] << x264_dequant_div_lut[i_qp][0]) >> 5;
    int d0 = dct[0] + dct[1];
    int d1 = dct[2] + dct[3];
    int d2 = dct[0] - dct[1];
    int d3 = dct[2] - dct[3];
    dct[0] = (d0 + d1) * dmf;
    dct[1] = (d0 - d1) * dmf;
    dct[2] = (d2 + d3) * dmf;
    dct[3] = (d2 - d3) * dmf;
}

static inline void dct2x2dc( dctcoef d[4], dctcoef dct4x4[4][16] )
{
    int d0 = dct4x4[0][0] + dct4x4[1][0];
    int d1 = dct4x4[2][0] + dct4x4[3][0];
    int d2 = dct4x4[0][0] - dct4x4[1][0];
    int d3 = dct4x4[2][0] - dct4x4[3][0];
    d[0] = d0 + d1;
    d[2] = d2 + d3;
    d[1] = d0 - d1;
    d[3] = d2 - d3;
    dct4x4[0][0] = 0;
    dct4x4[1][0] = 0;
    dct4x4[2][0] = 0;
    dct4x4[3][0] = 0;
}

/* All encoding functions must output the correct CBP and NNZ values.
 * The entropy coding functions will check CBP first, then NNZ, before
 * actually reading the DCT coefficients.  NNZ still must be correct even
 * if CBP is zero because of the use of NNZ values for context selection.
 * "NNZ" need only be 0 or 1 rather than the exact coefficient count because
 * that is only needed in CAVLC, and will be calculated by CAVLC's residual
 * coding and stored as necessary. */

/* This means that decimation can be done merely by adjusting the CBP and NNZ
 * rather than memsetting the coefficients. */

static void x264_mb_encode_i16x16( x264_t *h, int i_qp )
{
    pixel *p_src = h->mb.pic.p_fenc[0];
    pixel *p_dst = h->mb.pic.p_fdec[0];

    ALIGNED_ARRAY_16( dctcoef, dct4x4, [16], [16] ); /* coefficients of dct 16x16 */
    ALIGNED_ARRAY_16( dctcoef, dct_dc4x4, [16] );    /* coefficients of dct dc4x4 */

    int i, nz, block_cbp = 0;
    int decimate_score = h->mb.b_dct_decimate ? 0 : 9; /* only use decimate for inter frame */
    int i_mode = h->mb.i_intra16x16_pred_mode;

    h->predict_16x16[i_mode]( h->mb.pic.p_fdec[0] );
    h->dctf.sub16x16_dct( dct4x4, p_src, p_dst );

    /* dct coeff process for each 4x4 block */
    for( i = 0; i < 16; i++ )
    {
        /* copy dc coeff */
        if( h->mb.b_noise_reduction )
            h->quantf.denoise_dct( dct4x4[i], h->nr_residual_sum[0], h->nr_offset[0], 16 );
        dct_dc4x4[block_idx_xy_1d[i]] = dct4x4[i][0];
        dct4x4[i][0] = 0;

        /* quant/scan/dequant */
        nz = h->quantf.quant_4x4( dct4x4[i], h->quant4_mf[CQM_4IY][i_qp], h->quant4_bias[CQM_4IY][i_qp] );
        h->mb.cache.non_zero_count[x264_scan8[i]] = nz;
        if( nz )
        {
            h->zigzagf.scan_4x4( h->dct.luma4x4[i], dct4x4[i] );
            h->quantf.dequant_4x4( dct4x4[i], h->dequant4_mf[CQM_4IY], i_qp );
            if( decimate_score < 6 ) decimate_score += h->quantf.decimate_score15( h->dct.luma4x4[i] );
            block_cbp = 0xf;
        }
    }

    /* Writing the 16 CBFs in an i16x16 block is quite costly, so decimation can save many bits. */
    /* More useful with CAVLC, but still useful with CABAC. */
    if( decimate_score < 6 )
    {
        block_cbp = 0;
        M32( &h->mb.cache.non_zero_count[x264_scan8[0]] ) = 0;
        M32( &h->mb.cache.non_zero_count[x264_scan8[2]] ) = 0;
        M32( &h->mb.cache.non_zero_count[x264_scan8[8]] ) = 0;
        M32( &h->mb.cache.non_zero_count[x264_scan8[10]] ) = 0;
    }
    else
        h->mb.i_cbp_luma |= block_cbp;

    /* dc4x4 coeff process */
    h->dctf.dct4x4dc( dct_dc4x4 );
    nz = h->quantf.quant_4x4_dc( dct_dc4x4, h->quant4_mf[CQM_4IY][i_qp][0]>>1, h->quant4_bias[CQM_4IY][i_qp][0]<<1 );
    h->mb.cache.non_zero_count[x264_scan8[LUMA_DC]] = nz;
    if( nz )
    {
        h->zigzagf.scan_4x4( h->dct.luma16x16_dc[0], dct_dc4x4 );

        /* output samples to fdec */
        h->dctf.idct4x4dc( dct_dc4x4 );
        h->quantf.dequant_4x4_dc( dct_dc4x4, h->dequant4_mf[CQM_4IY], i_qp );  /* XXX not inversed */

        /* copy transformed dc coeff back to dct4x4 */
        if( block_cbp )
        {
            // dct4x4[i][0] = dct_dc4x4[block_idx_xy_1d[i]]
            // block_idx_xy_1d: {0, 1, 4, 5, 2, 3, 6, 7, 8, 9, 12, 13, 10, 11, 14, 15}
            dct4x4[0][0]  = dct_dc4x4[0];
            dct4x4[1][0]  = dct_dc4x4[1];
            dct4x4[2][0]  = dct_dc4x4[4];
            dct4x4[3][0]  = dct_dc4x4[5];
            dct4x4[4][0]  = dct_dc4x4[2];
            dct4x4[5][0]  = dct_dc4x4[3];
            dct4x4[6][0]  = dct_dc4x4[6];
            dct4x4[7][0]  = dct_dc4x4[7];
            dct4x4[8][0]  = dct_dc4x4[8];
            dct4x4[9][0]  = dct_dc4x4[9];
            dct4x4[10][0] = dct_dc4x4[12];
            dct4x4[11][0] = dct_dc4x4[13];
            dct4x4[12][0] = dct_dc4x4[10];
            dct4x4[13][0] = dct_dc4x4[11];
            dct4x4[14][0] = dct_dc4x4[14];
            dct4x4[15][0] = dct_dc4x4[15];
        }
    }

    /* put pixels to fdec */
    if( block_cbp )
        h->dctf.add16x16_idct( p_dst, dct4x4 );
    else if( nz )
        h->dctf.add16x16_idct_dc( p_dst, dct_dc4x4 );
}

/* Round down coefficients losslessly in DC-only chroma blocks.
 * Unlike luma blocks, this can't be done with a lookup table or
 * other shortcut technique because of the interdependencies
 * between the coefficients due to the chroma DC transform. */
static ALWAYS_INLINE int x264_mb_optimize_chroma_dc( x264_t *h, dctcoef *dct_dc, int dequant_mf[6][16], int i_qp )
{
	/* dequant_mf[i_qp%6][0] << i_qp/6 */
	int dmf = dequant_mf[x264_dequant_div_lut[i_qp][1]][0] << x264_dequant_div_lut[i_qp][0];
	return h->quantf.optimize_chroma_2x2_dc( dct_dc, dmf );
}

static void x264_mb_encode_chroma( x264_t *h, int b_inter, int i_qp )
{
	int i, ch, nz, nz_dc;
    int b_decimate = h->mb.b_dct_decimate;
    int (*dequant_mf)[16] = h->dequant4_mf[CQM_4IC + b_inter];
    ALIGNED_ARRAY_16( dctcoef, dct_dc, [4] );

    h->mb.i_cbp_chroma = 0;
    h->nr_count[2] += h->mb.b_noise_reduction << 2;

    /* Early termination: check variance of chroma residual before encoding.
     * Don't bother trying early termination at low QPs.
     * Values are experimentally derived. */
    if( b_decimate && i_qp >= 18 && !h->mb.b_noise_reduction )
    {
        int thresh = (x264_lambda2_tab[i_qp] + 32) >> 6;
        int ssd[2];

        int score  = h->pixf.var2[PIXEL_8x8]( h->mb.pic.p_fenc[1], FENC_STRIDE, h->mb.pic.p_fdec[1], FDEC_STRIDE, &ssd[0] );
        if( score < (thresh<<2) )
            score += h->pixf.var2[PIXEL_8x8]( h->mb.pic.p_fenc[2], FENC_STRIDE, h->mb.pic.p_fdec[2], FDEC_STRIDE, &ssd[1] );
        if( score < (thresh<<2) )
        {
        	/* set chroma ac coeff to zero */
            M16( &h->mb.cache.non_zero_count[x264_scan8[16]] ) = 0;
            M16( &h->mb.cache.non_zero_count[x264_scan8[18]] ) = 0;
            M16( &h->mb.cache.non_zero_count[x264_scan8[32]] ) = 0;
            M16( &h->mb.cache.non_zero_count[x264_scan8[34]] ) = 0;
            h->mb.cache.non_zero_count[x264_scan8[CHROMA_DC+0]] = 0;
            h->mb.cache.non_zero_count[x264_scan8[CHROMA_DC+1]] = 0;

            /* chroma dc coeff process */
            for( ch = 0; ch < 2; ch++ )
            {
                if( ssd[ch] > thresh )
                {
                    pixel *p_src = h->mb.pic.p_fenc[1+ch];
                    pixel *p_dst = h->mb.pic.p_fdec[1+ch];
                    h->dctf.sub8x8_dct_dc( dct_dc, p_src, p_dst );
                    nz_dc = h->quantf.quant_2x2_dc( dct_dc, h->quant4_mf[CQM_4IC+b_inter][i_qp][0] >> 1, h->quant4_bias[CQM_4IC+b_inter][i_qp][0] << 1 );
                    if( nz_dc )
                    {
                    	/* If the QP is too high (QP > 22 or dmf > 2048), there's no benefit to rounding optimization. */
                        if( (i_qp <= 22) && !x264_mb_optimize_chroma_dc( h, dct_dc, dequant_mf, i_qp ) )
                            continue;

                        /* DC-only chroma encoding */
                        h->mb.cache.non_zero_count[x264_scan8[CHROMA_DC+ch]] = 1;
                        zigzag_scan_2x2_dc( h->dct.chroma_dc[ch], dct_dc );
                        idct_dequant_2x2_dconly( dct_dc, dequant_mf, i_qp );
                        h->dctf.add8x8_idct_dc( p_dst, dct_dc );
                        h->mb.i_cbp_chroma = 1;
                    }
                }
            }
            return;
        }
    }

    for( ch = 0; ch < 2; ch++ )
    {
        pixel *p_src = h->mb.pic.p_fenc[1+ch];
        pixel *p_dst = h->mb.pic.p_fdec[1+ch];
        int i_decimate_score = 0;
        int nz_ac = 0;

        ALIGNED_ARRAY_16( dctcoef, dct4x4, [4], [16] );

        /* get chroma u/v diff of block 8x8 */
        h->dctf.sub8x8_dct( dct4x4, p_src, p_dst );
        if( h->mb.b_noise_reduction )
            for( i = 0; i < 4; i++ )
                h->quantf.denoise_dct( dct4x4[i], h->nr_residual_sum[2], h->nr_offset[2], 16 );
        dct2x2dc( dct_dc, dct4x4 );

        /* calculate dct coeffs */
        for( i = 0; i < 4; i++ )
        {
            nz = h->quantf.quant_4x4( dct4x4[i], h->quant4_mf[CQM_4IC+b_inter][i_qp], h->quant4_bias[CQM_4IC+b_inter][i_qp] );
            h->mb.cache.non_zero_count[x264_scan8[16+i+(ch<<4)]] = nz;
            if( nz )
            {
                nz_ac = 1;
                h->zigzagf.scan_4x4( h->dct.luma4x4[16+i+(ch<<4)], dct4x4[i] );
                h->quantf.dequant_4x4( dct4x4[i], dequant_mf, i_qp );
                if( b_decimate )
                    i_decimate_score += h->quantf.decimate_score15( h->dct.luma4x4[16+i+(ch<<4)] );
            }
        }
        /* quantization of chroma dc */
        nz_dc = h->quantf.quant_2x2_dc( dct_dc, h->quant4_mf[CQM_4IC+b_inter][i_qp][0] >> 1, h->quant4_bias[CQM_4IC+b_inter][i_qp][0] << 1 );
        h->mb.cache.non_zero_count[x264_scan8[CHROMA_DC+ch]] = nz_dc;

        /* if decimate score < 7 or nz_ac == 0, skip chroma ac encoding */
        if( (b_decimate && i_decimate_score < 7) || !nz_ac )
        {
            /* Decimate the block */
            M16( &h->mb.cache.non_zero_count[x264_scan8[16+(ch<<4)]] ) = 0;
            M16( &h->mb.cache.non_zero_count[x264_scan8[18+(ch<<4)]] ) = 0;

            /* Whole block is empty if nz_dc is zero */
            if( !nz_dc )
                continue;

            /* If the QP is too high (QP > 22 or dmf > 2048), there's no benefit to rounding optimization. */
            if( (i_qp <= 22) && !x264_mb_optimize_chroma_dc( h, dct_dc, dequant_mf, i_qp ) )
            {
                h->mb.cache.non_zero_count[x264_scan8[CHROMA_DC+ch]] = 0;
                continue;
            }

            /* DC-only chroma encoding */
            zigzag_scan_2x2_dc( h->dct.chroma_dc[ch], dct_dc );
            idct_dequant_2x2_dconly( dct_dc, dequant_mf, i_qp );
            h->dctf.add8x8_idct_dc( p_dst, dct_dc );
        }
        else
        {
            h->mb.i_cbp_chroma = 1;
            if( nz_dc )
            {
            	zigzag_scan_2x2_dc( h->dct.chroma_dc[ch], dct_dc );
            	idct_dequant_2x2_dc( dct_dc, dct4x4, dequant_mf, i_qp );
            }
            h->dctf.add8x8_idct( p_dst, dct4x4 );
        }
    }

    /* chroma cbp: 0 = none, 1 = DC only, 2 = DC+AC */
    h->mb.i_cbp_chroma += (h->mb.cache.non_zero_count[x264_scan8[CHROMA_DC+0]] | h->mb.cache.non_zero_count[x264_scan8[CHROMA_DC+1]] | h->mb.i_cbp_chroma);
}

/*****************************************************************************
 * x264_macroblock_encode:
 *****************************************************************************/
void x264_macroblock_encode( x264_t *h )
{
    int i_qp = h->mb.i_qp;
    int b_decimate = h->mb.b_dct_decimate;
    int nz, i, cbp;

    /* encode luma block by mb type */
    h->mb.i_cbp_luma = 0;
    h->mb.cache.non_zero_count[x264_scan8[LUMA_DC]] = 0;
    if( h->mb.i_type == P_SKIP )
    {
        /* don't do pskip motion compensation if it was already done in macroblock_analyse */
        if( !h->mb.b_skip_mc )
        {
            int mvx = x264_clip3( h->mb.cache.mv[0][x264_scan8[0]][0], h->mb.mv_min[0], h->mb.mv_max[0] );
            int mvy = x264_clip3( h->mb.cache.mv[0][x264_scan8[0]][1], h->mb.mv_min[1], h->mb.mv_max[1] );
            h->mc.mc_luma( h->mb.pic.p_fdec[0], FDEC_STRIDE, &h->mb.pic.p_fref[0][0][0], h->mb.pic.i_stride[0], mvx, mvy, 16, 16, &h->sh.weight[0][0] );

			/* Special case for mv0, which is (of course) very common in P-skip mode. */
			if( mvx | mvy )
				h->mc.mc_chroma( h->mb.pic.p_fdec[1], h->mb.pic.p_fdec[2], FDEC_STRIDE, h->mb.pic.p_fref[0][0][4], h->mb.pic.i_stride[1], mvx, mvy, 8, 8 );
			else
				h->mc.load_deinterleave_chroma_fdec( h->mb.pic.p_fdec[1], h->mb.pic.p_fref[0][0][4], h->mb.pic.i_stride[1], 8 );
        }

        /* encode pskip mb */
        M32( &h->mb.cache.non_zero_count[x264_scan8[ 0]] ) = 0;
        M32( &h->mb.cache.non_zero_count[x264_scan8[ 2]] ) = 0;
        M32( &h->mb.cache.non_zero_count[x264_scan8[ 8]] ) = 0;
        M32( &h->mb.cache.non_zero_count[x264_scan8[10]] ) = 0;
        M32( &h->mb.cache.non_zero_count[x264_scan8[16+ 0]] ) = 0;
        M32( &h->mb.cache.non_zero_count[x264_scan8[16+ 2]] ) = 0;
        M32( &h->mb.cache.non_zero_count[x264_scan8[32+ 0]] ) = 0;
        M32( &h->mb.cache.non_zero_count[x264_scan8[32+ 2]] ) = 0;
        h->mb.i_cbp_luma = 0;
        h->mb.i_cbp_chroma = 0;
        h->mb.cbp[h->mb.i_mb_xy] = 0;
        return;
    }
    else if( h->mb.i_type == I_16x16 )
    {
        h->mb.b_transform_8x8 = 0;
        x264_mb_encode_i16x16( h, i_qp );
    }
    else if( h->mb.i_type == I_4x4 )
    {
        h->mb.b_transform_8x8 = 0;
        /* If we already encoded 15 of the 16 i4x4 blocks, we don't have to do them again. */
        if( h->mb.i_skip_intra )
        {
            h->mc.copy[PIXEL_16x16]( h->mb.pic.p_fdec[0], FDEC_STRIDE, h->mb.pic.i4x4_fdec_buf, 16, 16 );
            M32( &h->mb.cache.non_zero_count[x264_scan8[ 0]] ) = h->mb.pic.i4x4_nnz_buf[0];
            M32( &h->mb.cache.non_zero_count[x264_scan8[ 2]] ) = h->mb.pic.i4x4_nnz_buf[1];
            M32( &h->mb.cache.non_zero_count[x264_scan8[ 8]] ) = h->mb.pic.i4x4_nnz_buf[2];
            M32( &h->mb.cache.non_zero_count[x264_scan8[10]] ) = h->mb.pic.i4x4_nnz_buf[3];
            h->mb.i_cbp_luma = h->mb.pic.i4x4_cbp;
        }
		for( i = (h->mb.i_skip_intra) ? 15 : 0; i < 16; i++ )
		{
			pixel *p_dst = &h->mb.pic.p_fdec[0][block_idx_xy_fdec[i]];
			int i_mode = h->mb.cache.intra4x4_pred_mode[x264_scan8[i]];

			/* emulate missing topright samples */
			if( (h->mb.i_neighbour4[i] & (MB_TOPRIGHT|MB_TOP)) == MB_TOP )
				MPIXEL_X4( &p_dst[4-FDEC_STRIDE] ) = PIXEL_SPLAT_X4( p_dst[3-FDEC_STRIDE] );

			x264_mb_encode_i4x4( h, i, i_qp, i_mode, 1 );
		}
    }
    else    /* Inter MB */
    {
		ALIGNED_ARRAY_16( dctcoef, dct4x4,[16],[16] );
		int plane_cbp = 0;
		int i8x8, i4x4;
        int i_decimate_mb = 0;
        int i_decimate_8x8;

        /* Don't repeat motion compensation if it was already done in non-RD transform analysis */
        if( !h->mb.b_skip_mc )
            x264_mb_mc( h );

        /* get luma diff of block 16x16 */
		h->dctf.sub16x16_dct( dct4x4, h->mb.pic.p_fenc[0], h->mb.pic.p_fdec[0] );
		h->nr_count[0] += h->mb.b_noise_reduction << 4;
		for( i8x8 = 0; i8x8 < 4; i8x8++ )
		{
			cbp = 0;
			i_decimate_8x8 = 0;

			/* encode one 4x4 block */
			for( i4x4 = 0; i4x4 < 4; i4x4++ )
			{
				int idx = (i8x8<<2) + i4x4;
				nz = x264_quant_4x4( h, dct4x4[idx], i_qp, DCT_LUMA_4x4, 0, idx );
				h->mb.cache.non_zero_count[x264_scan8[idx]] = nz;
				if( nz )
				{
					h->zigzagf.scan_4x4( h->dct.luma4x4[idx], dct4x4[idx] );
					h->quantf.dequant_4x4( dct4x4[idx], h->dequant4_mf[CQM_4PY], i_qp );
					if( b_decimate && i_decimate_8x8 < 6 )
						i_decimate_8x8 += h->quantf.decimate_score16( h->dct.luma4x4[idx] );
					cbp = 1;
				}
			}

			/* decimate this 8x8 block */
			if( b_decimate )
			{
				i_decimate_mb += i_decimate_8x8;
				/* for a single 8x8 block, if score < 4, set decimation */
				if( i_decimate_8x8 < 4 )
				{
					M16( &h->mb.cache.non_zero_count[x264_scan8[i8x8<<2]+0] ) = 0;
					M16( &h->mb.cache.non_zero_count[x264_scan8[i8x8<<2]+8] ) = 0;
				}
				else
					plane_cbp |= 1<<i8x8;
			}
			else if( cbp )
			{
				h->dctf.add8x8_idct( &h->mb.pic.p_fdec[0][((i8x8&1) + (i8x8>>1)*FDEC_STRIDE)<<3], &dct4x4[i8x8<<2] );
				plane_cbp |= 1<<i8x8;
			}
		}

		if( b_decimate )
		{
			/* for a complete macroblock, if score < 6, set decimation */
			if( i_decimate_mb < 6 )
			{
				plane_cbp = 0;
			    M32( &h->mb.cache.non_zero_count[x264_scan8[0]] ) = 0;
			    M32( &h->mb.cache.non_zero_count[x264_scan8[2]] ) = 0;
			    M32( &h->mb.cache.non_zero_count[x264_scan8[8]] ) = 0;
			    M32( &h->mb.cache.non_zero_count[x264_scan8[10]] ) = 0;
			}
			else
			{
				for( i8x8 = 0; i8x8 < 4; i8x8++ )
					if( plane_cbp&(1<<i8x8) )
						h->dctf.add8x8_idct( &h->mb.pic.p_fdec[0][((i8x8&1) + (i8x8>>1)*FDEC_STRIDE)<<3], &dct4x4[i8x8<<2] );
			}
		}
		h->mb.i_cbp_luma |= plane_cbp;
    }

    /* encode chroma block 8x8 */
	if( IS_INTRA( h->mb.i_type ) )
	{
		int i_mode = h->mb.i_chroma_pred_mode;
		h->predict_chroma[i_mode]( h->mb.pic.p_fdec[1] );
		h->predict_chroma[i_mode]( h->mb.pic.p_fdec[2] );
	}
	x264_mb_encode_chroma( h, !IS_INTRA( h->mb.i_type ), h->mb.i_chroma_qp );

    /* store cbp. 4 high bits for chroma, 4 low bits for luma */
    cbp = h->mb.i_cbp_chroma << 4 | h->mb.i_cbp_luma;
    if( h->param.b_cabac )
        cbp |= h->mb.cache.non_zero_count[x264_scan8[LUMA_DC    ]] << 8
            |  h->mb.cache.non_zero_count[x264_scan8[CHROMA_DC+0]] << 9
            |  h->mb.cache.non_zero_count[x264_scan8[CHROMA_DC+1]] << 10;
    h->mb.cbp[h->mb.i_mb_xy] = cbp;

    /*****************************************
     * Force P_SKIP if following conditions: *
     * 1. P_16x16                            *
     * 2. cbp_luma and cbp_chroma are 0      *
     * 3. mv == pskip_mv and ref == 0        *
     *****************************************/
	if( h->mb.i_type == P_L0 && h->mb.i_partition == D_16x16 &&
		!(h->mb.i_cbp_luma | h->mb.i_cbp_chroma) &&
		M32( h->mb.cache.mv[0][x264_scan8[0]] ) == M32( h->mb.cache.pskip_mv )
		&& h->mb.cache.ref[0][x264_scan8[0]] == 0 )
	{
		h->mb.i_type = P_SKIP;
	}
}

/*****************************************************************************
 * x264_macroblock_probe_pskip:
 *  Check if the current MB could be encoded as a P_SKIP
 *****************************************************************************/
int x264_macroblock_probe_pskip( x264_t *h )
{
    ALIGNED_ARRAY_16( dctcoef, dct4x4, [4], [16] );
    ALIGNED_ARRAY_16( dctcoef, dct_dc, [4] );
    ALIGNED_ARRAY_16( dctcoef, dctscan, [16] );
    ALIGNED_4( int16_t mvp[2] );
    int i_qp = h->mb.i_qp;
    int i_decimate_mb = 0;
	int i8x8, i4x4, ch;
	int thresh, ssd;

	/* Get the MV */
	mvp[0] = x264_clip3( h->mb.cache.pskip_mv[0], h->mb.mv_min[0], h->mb.mv_max[0] );
	mvp[1] = x264_clip3( h->mb.cache.pskip_mv[1], h->mb.mv_min[1], h->mb.mv_max[1] );
	/* Motion compensation */
	h->mc.mc_luma( h->mb.pic.p_fdec[0], FDEC_STRIDE, &h->mb.pic.p_fref[0][0][0], h->mb.pic.i_stride[0], mvp[0], mvp[1], 16, 16, &h->sh.weight[0][0] );

	/* try to encode 4 block 8x8 and detect decimate score */
	for( i8x8 = 0; i8x8 < 4; i8x8++ )
	{
		int fenc_offset = ((i8x8&1) + (i8x8>>1) * FENC_STRIDE)<<3;
		int fdec_offset = ((i8x8&1) + (i8x8>>1) * FDEC_STRIDE)<<3;
		/* get luma diff of block 8x8 */
		h->dctf.sub8x8_dct( dct4x4, h->mb.pic.p_fenc[0] + fenc_offset, h->mb.pic.p_fdec[0] + fdec_offset );

		/* encode each 4x4 block of current 8x8 block */
		for( i4x4 = 0; i4x4 < 4; i4x4++ )
		{
			if( h->mb.b_noise_reduction )
				h->quantf.denoise_dct( dct4x4[i4x4], h->nr_residual_sum[0], h->nr_offset[0], 16 );
			/* quantization of block 4x4. if coefs are all zero, skip decimate. */
			if( !h->quantf.quant_4x4( dct4x4[i4x4], h->quant4_mf[CQM_4PY][i_qp], h->quant4_bias[CQM_4PY][i_qp] ) )
				continue;
			/* zigzag scan of block 4x4 */
			h->zigzagf.scan_4x4( dctscan, dct4x4[i4x4] );
			/* accumulate decimate_mb value */
			i_decimate_mb += h->quantf.decimate_score16( dctscan );
			/* if decimate_mb >= 6, skip mode is improper. stop probe now */
			if( i_decimate_mb >= 6 )
				return 0;
		}
	}

    /* check chroma planes */
	i_qp = h->mb.i_chroma_qp;
	thresh = (x264_lambda2_tab[i_qp] + 32) >> 6;

	/* Special case for mv0, which is (of course) very common in P-skip mode. */
	if( M32( mvp ) )
		h->mc.mc_chroma( h->mb.pic.p_fdec[1], h->mb.pic.p_fdec[2], FDEC_STRIDE, h->mb.pic.p_fref[0][0][4], h->mb.pic.i_stride[1], mvp[0], mvp[1], 8, 8 );
	else
		h->mc.load_deinterleave_chroma_fdec( h->mb.pic.p_fdec[1], h->mb.pic.p_fref[0][0][4], h->mb.pic.i_stride[1], 8 );

	for( ch = 0; ch < 2; ch++ )
	{
		pixel *p_src = h->mb.pic.p_fenc[1+ch];
		pixel *p_dst = h->mb.pic.p_fdec[1+ch];

		/* there is almost never a termination during chroma, but we can't avoid the check entirely */
		/* so instead we check SSD and skip the actual check if the score is low enough. */
		ssd = h->pixf.ssd[PIXEL_8x8]( p_dst, FDEC_STRIDE, p_src, FENC_STRIDE );
		if( ssd < thresh )
			continue;

		/* The vast majority of chroma checks will terminate during the DC check or the higher
		 * threshold check, so we can save time by doing a DC-only DCT. */
		if( h->mb.b_noise_reduction )
		{
			h->dctf.sub8x8_dct( dct4x4, p_src, p_dst );
			for( i4x4 = 0; i4x4 < 4; i4x4++ )
			{
				h->quantf.denoise_dct( dct4x4[i4x4], h->nr_residual_sum[2], h->nr_offset[2], 16 );
				dct_dc[i4x4] = dct4x4[i4x4][0];
			}
		}
		else
		{
			h->dctf.sub8x8_dct_dc( dct_dc, p_src, p_dst );
		}
		if( h->quantf.quant_2x2_dc( dct_dc, h->quant4_mf[CQM_4PC][i_qp][0] >> 1, h->quant4_bias[CQM_4PC][i_qp][0] << 1 ) )
			return 0;

		/* If there wasn't a termination in DC, we can check against a much higher threshold. */
		if( ssd < (thresh<<2) )
			continue;

        if( !h->mb.b_noise_reduction )
        	h->dctf.sub8x8_dct( dct4x4, p_src, p_dst );

		/* calculate dct coeffs */
		for( i4x4 = 0, i_decimate_mb = 0; i4x4 < 4; i4x4++ )
		{
			dct4x4[i4x4][0] = 0;
			if( h->mb.b_noise_reduction )
				h->quantf.denoise_dct( dct4x4[i4x4], h->nr_residual_sum[2], h->nr_offset[2], 16 );
			/* quantization of block 4x4. if coefs are all zero, skip decimate. */
			if( !h->quantf.quant_4x4( dct4x4[i4x4], h->quant4_mf[CQM_4PC][i_qp], h->quant4_bias[CQM_4PC][i_qp] ) )
				continue;
			/* zigzag scan of block 4x4 */
			h->zigzagf.scan_4x4( dctscan, dct4x4[i4x4] );
			/* accumulate decimate_mb value */
			i_decimate_mb += h->quantf.decimate_score15( dctscan );
			/* if decimate_mb >= 7, skip mode is improper. stop probe now */
			if( i_decimate_mb >= 7 )
				return 0;
		}
	}

    /* mark we have done motion compensation */
	h->mb.b_skip_mc = 1;
	/* return 1 indicates skip mode is available */
    return 1;
}
