/*****************************************************************************
 * mc.c: motion compensation
 *****************************************************************************/

#include "common.h"

#ifdef __TI_COMPILER_VERSION__
void mc_copy_w16_ti ( pixel *dst, intptr_t dst_stride, pixel *src, intptr_t src_stride, int i_height );
void mc_copy_w8_ti  ( pixel *dst, intptr_t dst_stride, pixel *src, intptr_t src_stride, int i_height );
void mc_copy_w4_ti  ( pixel *dst, intptr_t dst_stride, pixel *src, intptr_t src_stride, int i_height );

void x264_plane_copy_ti              ( pixel *dst, intptr_t i_dst, pixel *src, intptr_t i_src, int w, int h );
void x264_plane_copy_interleave_ti   ( pixel *dst,  intptr_t i_dst, pixel *srcu, intptr_t i_srcu, pixel *srcv, intptr_t i_srcv, int w, int h );
void x264_plane_copy_deinterleave_ti ( pixel *dstu, intptr_t i_dstu, pixel *dstv, intptr_t i_dstv, pixel *src,  intptr_t i_src, int w, int h );
void x264_plane_copy_deinterlace_ti  (
		pixel *srcy, intptr_t i_srcy, pixel *dsty, intptr_t i_dsty,
		pixel *srcc, intptr_t i_srcc, pixel *dstc, intptr_t i_dstc,
		int i_width, int i_height );
void x264_plane_deinterlace_ti       ( pixel *pixy, intptr_t i_pixy, pixel *pixc, intptr_t i_pixc, int i_width, int i_height );

void store_interleave_chroma_ti       ( pixel *dst, intptr_t i_dst, pixel *srcu, pixel *srcv, int height );
void load_deinterleave_chroma_fenc_ti ( pixel *dst, pixel *src, intptr_t i_src, int height );
void load_deinterleave_chroma_fdec_ti ( pixel *dst, pixel *src, intptr_t i_src, int height );

void frame_init_lowres_core_ti ( pixel *src0, pixel *dst0, pixel *dsth, pixel *dstv, pixel *dstc, intptr_t src_stride, intptr_t dst_stride, int width, int height );
#endif

const x264_weight_t x264_weight_none[3] = { {{0}} };

/***************************************************
 * pixel_avg: copy avg of src1 and src2 to dst.    *
 * copy area: i_width * i_height, pixels of one    *
 * line were given by i_***_stride.                *
 *                                                 *
 * pixel_avg are called by mc_luma and get_ref     *
 * with wxh, wx(h+1) and (w+4)xh                   *
 * possible width x height:                        *
 * 16x16, 16x17, 20x16                             *
 * 16x8,  16x9,  20x8                              *
 * 8x16,  8x17,  12x16                             *
 * 8x8,   8x9,   12x8                              *
 *                                                 *
 * NOTe: src1 and src2 are not 8-bytes aligned,    *
 * but dst is 8-bytes aligned                      *
 ***************************************************/
#ifdef __TI_COMPILER_VERSION__

void pixel_avg_w8e_ti ( pixel *,  intptr_t, pixel *, intptr_t, pixel *, intptr_t, int, int );
void pixel_avg_w8o_ti ( pixel *,  intptr_t, pixel *, intptr_t, pixel *, intptr_t, int, int );
void pixel_avg_w12_ti ( pixel *,  intptr_t, pixel *, intptr_t, pixel *, intptr_t, int, int );
void pixel_avg_w16_ti ( pixel *,  intptr_t, pixel *, intptr_t, pixel *, intptr_t, int, int );
void pixel_avg_w20_ti ( pixel *,  intptr_t, pixel *, intptr_t, pixel *, intptr_t, int, int );

static inline void pixel_avg( pixel *restrict dst,  intptr_t i_dst_stride,
                              pixel *restrict src1, intptr_t i_src1_stride,
                              pixel *restrict src2, intptr_t i_src2_stride, int i_width, int i_height )
{
	switch( i_width )
	{
	case 20: pixel_avg_w20_ti( dst, i_dst_stride, src1, i_src1_stride, src2, i_src2_stride, i_width, i_height ); break;
	case 16: pixel_avg_w16_ti( dst, i_dst_stride, src1, i_src1_stride, src2, i_src2_stride, i_width, i_height ); break;
	case 12: pixel_avg_w12_ti( dst, i_dst_stride, src1, i_src1_stride, src2, i_src2_stride, i_width, i_height ); break;
	case 8:
	{
		if( i_height == 8 || i_height == 16 )
			pixel_avg_w8e_ti( dst, i_dst_stride, src1, i_src1_stride, src2, i_src2_stride, i_width, i_height );
		else
			pixel_avg_w8o_ti( dst, i_dst_stride, src1, i_src1_stride, src2, i_src2_stride, i_width, i_height );
		break;
	}
	}
}
#else
static inline void pixel_avg( pixel *dst,  intptr_t i_dst_stride,
                              pixel *src1, intptr_t i_src1_stride,
                              pixel *src2, intptr_t i_src2_stride, int i_width, int i_height )
{
	int x, y;
    for( y = 0; y < i_height; y++ )
    {
        for( x = 0; x < i_width; x++ )
            dst[x] = ( src1[x] + src2[x] + 1 ) >> 1;
        dst  += i_dst_stride;
        src1 += i_src1_stride;
        src2 += i_src2_stride;
    }
}
#endif

/* copy pixel value of src to dst space.
 * copy area: i_width * i_height, pixels of one line were given by i_***_stride.
 */
static void mc_copy( pixel *src, intptr_t i_src_stride, pixel *dst, intptr_t i_dst_stride, int i_width, int i_height )
{
	int y;
    for( y = 0; y < i_height; y++ )
    {
        memcpy( dst, src, i_width * sizeof(pixel) );

        src += i_src_stride;
        dst += i_dst_stride;
    }
}

/*********************************************************************
 * hpel_filter: half-pixel filter                                    *
 * The luma prediction values at half sample positions are derived   *
 * by applying a 6-tap filter with tap values(1, -5, 20, 20, -5, 1). *
 *                                                                   *
 * @ # @ # @ # @ # @                                                 *
 * $ ^ $ ^ $ ^ $ ^ $                                                 *
 * @ # @ # @ # @ # @                                                 *
 * $ ^ $ ^ $ ^ $ ^ $                                                 *
 * @ # @ # @ # @ # @                                                 *
 *                                                                   *
 * @: full-pixel, load from src                                      *
 * #: horizontal half-pixel, save to dsth                            *
 * $: vertical half-pixel, save to dstv                              *
 * ^: center half-pixel, save to dstc                                *
 *********************************************************************/
#ifdef __TI_COMPILER_VERSION__

void hpel_filter_v_ti( pixel *dst, pixel *src, intptr_t stride, int width, int height );
void hpel_filter_h_ti( pixel *dst, pixel *src, intptr_t stride, int width, int height );

/*****************************************************
 * hpel_filter is called by x264_frame_filter        *
 * for each mb row of encoding frame.                *
 *                                                   *
 * e.g. CIF (352x288), calls: 288 / 16 = 18.         *
 * stride: 352 + 2 * 32 = 416                        *
 * width: 352 + 16 = 368                             *
 * height: 16 for all mb lines except last line (32) *
 *****************************************************/
static void hpel_filter_ti( pixel *restrict dsth, pixel *restrict dstv, pixel *restrict dstc, pixel *restrict src,
                         intptr_t stride, int width, int height, int16_t *buf )
{
	hpel_filter_v_ti( dstv, src, stride, width, height );
	hpel_filter_h_ti( dsth, src, stride, width, height );
	hpel_filter_h_ti( dstc, dstv, stride, width, height );
}
#endif

#define TAPFILTER(pix, d) ((pix)[x-2*d] + (pix)[x+3*d] - 5*((pix)[x-d] + (pix)[x+2*d]) + 20*((pix)[x] + (pix)[x+d]))
static void hpel_filter( pixel *dsth, pixel *dstv, pixel *dstc, pixel *src,
                         intptr_t stride, int width, int height, int16_t *buf )
{
	int x, y;
    for( y = 0; y < height; y++ )
    {
        for( x = -2; x < width+3; x++ )
        {
            int v = TAPFILTER(src,stride);
            dstv[x] = x264_clip_pixel( (v + 16) >> 5 );
            /* transform v for storage in a 16-bit integer */
            buf[x+2] = v;
        }
        for( x = 0; x < width; x++ )
            dstc[x] = x264_clip_pixel( (TAPFILTER(buf+2,1) + 512) >> 10 );
        for( x = 0; x < width; x++ )
            dsth[x] = x264_clip_pixel( (TAPFILTER(src,1) + 16) >> 5 );
        dsth += stride;
        dstv += stride;
        dstc += stride;
        src += stride;
    }
}

/******************************************
 * half-pixel reference index.            *
 * 0: full-pixel value                    *
 * 1: horizontal half-pixel value         *
 *    (right of full-pixel)               *
 * 2: vertical half-pixel value           *
 *    (down of full-pixel)                *
 * 3: hv half-pixel value                 *
 *    (diagonal down right of full-pixel) *
 *                                        *
 * hpel_ref0 represents position of src1. *
 * 0 1 1 1                                *
 * 0 1 1 1                                *
 * 2 3 3 3                                *
 * 0 1 1 1                                *
 *                                        *
 * hpel_ref1 represents position of src2. *
 * only used for quat-pixel interpolation *
 * 0 0 0 0                                *
 * 2 2 3 2                                *
 * 2 2 3 2                                *
 * 2 2 3 2                                *
 ******************************************/
static const uint8_t hpel_ref0[16] = {0,1,1,1,0,1,1,1,2,3,3,3,0,1,1,1};
static const uint8_t hpel_ref1[16] = {0,0,0,0,2,2,3,2,2,2,3,2,2,2,3,2};

/******************************************************
 * mc_luma: motion compensation of luma               *
 *                                                    *
 * this function retrieves pixel values based on 1/4  *
 * sub-pixel precision, with consideration of weight. *
 *                                                    *
 * @ dst: output pixel values                         *
 * @ i_dst_stride: pixels of one line in dst plane    *
 * @ src: input pixel values, half-pixel filtered.    *
 *      [0] yN: full-pixel value of luma              *
 *      [1] yH: horizontal half-pixel value of luma   *
 *              (right of full-pixel)                 *
 *      [2] yV: vertical half-pixel value of luma     *
 *              (down of full-pixel)                  *
 *      [3] yHV: hv half-pixel value of luma          *
 *              (diagonal down right of full-pixel)   *
 * @ i_src_stride: pixels of one line in src plane    *
 * @ mvx, mvy: mv of x/y, based on 1/4 sub-pixel      *
 * @ i_width, i_height: width * height to process     *
 * @ weight: x264_weight_t struct                     *
 ******************************************************/
static void mc_luma( pixel *dst,    intptr_t i_dst_stride,
                     pixel *src[4], intptr_t i_src_stride,
                     int mvx, int mvy,
                     int i_width, int i_height, const x264_weight_t *weight )
{
	/* qpel_idx indicates sub-pixel part of mvx, mvy */
    int qpel_idx = ((mvy&3)<<2) + (mvx&3);
    /* offset indicates full-pixel position of mvx, mvy */
    int offset = (mvy>>2)*i_src_stride + (mvx>>2);
    /* src1, src2 indicates half-pixel values */
    pixel *src1 = src[hpel_ref0[qpel_idx]] + offset + ((mvy&3) == 3) * i_src_stride;

    /* since half-pixel values are computed and stored by x264_frame_filter in the init of encoding,
     * here we do quat-pixel interpolation only if mvx/mvy indicates to a 1/4 pixel position.
     */
    if( qpel_idx & 5 )
    {
    	/* qpel interpolation: average of two near 1/2 pixels */
        pixel *src2 = src[hpel_ref1[qpel_idx]] + offset + ((mvx&3) == 3);
        pixel_avg( dst, i_dst_stride, src1, i_src_stride, src2, i_src_stride, i_width, i_height );
    }
    else
        mc_copy( src1, i_src_stride, dst, i_dst_stride, i_width, i_height );
}

static pixel *get_ref( pixel *dst,   intptr_t *i_dst_stride,
                       pixel *src[4], intptr_t i_src_stride,
                       int mvx, int mvy,
                       int i_width, int i_height, const x264_weight_t *weight )
{
    int qpel_idx = ((mvy&3)<<2) + (mvx&3);
    int offset = (mvy>>2)*i_src_stride + (mvx>>2);
    pixel *src1 = src[hpel_ref0[qpel_idx]] + offset + ((mvy&3) == 3) * i_src_stride;

    /* since half-pixel values are computed and stored by x264_frame_filter in the init of encoding,
     * here we do quat-pixel interpolation only if mvx/mvy indicates to a 1/4 pixel position.
     */
    if( qpel_idx & 5 ) /* qpel interpolation needed */
    {
        pixel *src2 = src[hpel_ref1[qpel_idx]] + offset + ((mvx&3) == 3);
        pixel_avg( dst, *i_dst_stride, src1, i_src_stride, src2, i_src_stride, i_width, i_height );
        return dst;
    }
    else
    {
        *i_dst_stride = i_src_stride;
        return src1;
    }
}

/***************************************************
 * mc_chroma: full chroma mc (ie until 1/8 pixel). *
 *                                                 *
 * mc_chroma are called by x264_mb_mc_xywh         *
 * possible width x height:                        *
 * 8x8, 8x4, 4x8, 4x4                              *
 ***************************************************/
#ifdef __TI_COMPILER_VERSION__

void mc_chroma_w8_ti ( pixel *dstu, pixel *dstv, intptr_t i_dst_stride, pixel *src, intptr_t i_src_stride, int mvx, int mvy, int i_width, int i_height );
void mc_chroma_w4_ti ( pixel *dstu, pixel *dstv, intptr_t i_dst_stride, pixel *src, intptr_t i_src_stride, int mvx, int mvy, int i_width, int i_height );

static void mc_chroma_ti( pixel *restrict dstu, pixel *restrict dstv, intptr_t i_dst_stride,
                       pixel *restrict src, intptr_t i_src_stride,
                       int mvx, int mvy,
                       int i_width, int i_height )
{
	if ( i_width == 8 )
		mc_chroma_w8_ti( dstu, dstv, i_dst_stride, src, i_src_stride, mvx, mvy, i_width, i_height );
	else
		mc_chroma_w4_ti( dstu, dstv, i_dst_stride, src, i_src_stride, mvx, mvy, i_width, i_height );
}
#endif

static void mc_chroma( pixel *dstu, pixel *dstv, intptr_t i_dst_stride,
                       pixel *src, intptr_t i_src_stride,
                       int mvx, int mvy,
                       int i_width, int i_height )
{
    pixel *srcp;
	int x, y;

    /* dst[x] = ((8-dx)*(8-dy)*src[A] + dx*(8-dy)*src[B] + (8-dx)*dy*src[C] + dx*dy*src[D])/64 */
	int d8x = mvx&0x07;
    int d8y = mvy&0x07;
    int cA = (8-d8x)*(8-d8y);
    int cB = d8x    *(8-d8y);
    int cC = (8-d8x)*d8y;
    int cD = d8x    *d8y;

    src += (mvy >> 3) * i_src_stride + (mvx >> 3)*2;
    srcp = &src[i_src_stride];

    for( y = 0; y < i_height; y++ )
    {
        for( x = 0; x < i_width; x++ )
        {
            dstu[x] = ( cA*src[2*x]  + cB*src[2*x+2] +
                        cC*srcp[2*x] + cD*srcp[2*x+2] + 32 ) >> 6;
            dstv[x] = ( cA*src[2*x+1]  + cB*src[2*x+3] +
                        cC*srcp[2*x+1] + cD*srcp[2*x+3] + 32 ) >> 6;
        }
        dstu += i_dst_stride;
        dstv += i_dst_stride;
        src   = srcp;
        srcp += i_src_stride;
    }
}

#define MC_COPY(W) \
static void mc_copy_w##W( pixel *dst, intptr_t i_dst, pixel *src, intptr_t i_src, int i_height ) \
{ \
    mc_copy( src, i_src, dst, i_dst, W, i_height ); \
}
MC_COPY( 16 )
MC_COPY( 8 )
MC_COPY( 4 )

void x264_plane_copy_c( pixel *dst, intptr_t i_dst,
                        pixel *src, intptr_t i_src, int w, int h )
{
    while( h-- )
    {
        memcpy( dst, src, w * sizeof(pixel) );
        dst += i_dst;
        src += i_src;
    }
}

void x264_plane_copy_interleave_c( pixel *dst,  intptr_t i_dst,
                                   pixel *srcu, intptr_t i_srcu,
                                   pixel *srcv, intptr_t i_srcv, int w, int h )
{
	int x, y;
    for( y=0; y<h; y++, dst+=i_dst, srcu+=i_srcu, srcv+=i_srcv )
        for( x=0; x<w; x++ )
        {
            dst[2*x]   = srcu[x];
            dst[2*x+1] = srcv[x];
        }
}

static void x264_plane_copy_deinterleave_c( pixel *dstu, intptr_t i_dstu,
                                            pixel *dstv, intptr_t i_dstv,
                                            pixel *src,  intptr_t i_src, int w, int h )
{
	int x, y;
    for( y=0; y<h; y++, dstu+=i_dstu, dstv+=i_dstv, src+=i_src )
        for( x=0; x<w; x++ )
        {
            dstu[x] = src[2*x];
            dstv[x] = src[2*x+1];
        }
}

static void store_interleave_chroma( pixel *dst, intptr_t i_dst, pixel *srcu, pixel *srcv, int height )
{
	int x, y;
    for( y=0; y<height; y++, dst+=i_dst, srcu+=FDEC_STRIDE, srcv+=FDEC_STRIDE )
        for( x=0; x<8; x++ )
        {
            dst[2*x]   = srcu[x];
            dst[2*x+1] = srcv[x];
        }
}

static void load_deinterleave_chroma_fenc( pixel *dst, pixel *src, intptr_t i_src, int height )
{
    x264_plane_copy_deinterleave_c( dst, FENC_STRIDE, dst+FENC_STRIDE/2, FENC_STRIDE, src, i_src, 8, height );
}

static void load_deinterleave_chroma_fdec( pixel *dst, pixel *src, intptr_t i_src, int height )
{
    x264_plane_copy_deinterleave_c( dst, FDEC_STRIDE, dst+FDEC_STRIDE/2, FDEC_STRIDE, src, i_src, 8, height );
}

static void prefetch_fenc_null( pixel *pix_y,  intptr_t stride_y,
                                pixel *pix_uv, intptr_t stride_uv, int mb_x )
{}

static void prefetch_ref_null( pixel *pix, intptr_t stride, int parity )
{}

static void memzero_aligned( void * dst, size_t n )
{
    memset( dst, 0, n );
}

void x264_frame_init_lowres( x264_t *h, x264_frame_t *frame )
{
    pixel *src = frame->plane[0];
    int i_stride = frame->i_stride[0];
    int i_height = frame->i_lines[0];
    int i_width  = frame->i_width[0];
	int x, y;

    // duplicate last row and column so that their interpolation doesn't have to be special-cased
    for( y = 0; y < i_height; y++ )
        src[i_width+y*i_stride] = src[i_width-1+y*i_stride];
    memcpy( src+i_stride*i_height, src+i_stride*(i_height-1), (i_width+1) * sizeof(pixel) );

    h->mc.frame_init_lowres_core( src, frame->lowres[0], frame->lowres[1], frame->lowres[2], frame->lowres[3],
                                  i_stride, frame->i_stride_lowres, frame->i_width_lowres, frame->i_lines_lowres );
    x264_frame_expand_border_lowres( frame );

    memset( frame->i_cost_est, -1, sizeof(frame->i_cost_est) );

    for( y = 0; y < h->param.i_bframe + 2; y++ )
        for( x = 0; x < h->param.i_bframe + 2; x++ )
            frame->i_row_satds[y][x][0] = -1;

    for( y = 0; y <= !!h->param.i_bframe; y++ )
        for( x = 0; x <= h->param.i_bframe; x++ )
            frame->lowres_mvs[y][x][0][0] = 0x7FFF;
}

static void frame_init_lowres_core( pixel *src0, pixel *dst0, pixel *dsth, pixel *dstv, pixel *dstc,
                                    intptr_t src_stride, intptr_t dst_stride, int width, int height )
{
	int x, y;
    for( y = 0; y < height; y++ )
    {
        pixel *src1 = src0+src_stride;
        pixel *src2 = src1+src_stride;
        for( x = 0; x<width; x++ )
        {
            // slower than naive bilinear, but matches asm
#define FILTER(a,b,c,d) ((((a+b+1)>>1)+((c+d+1)>>1)+1)>>1)
            dst0[x] = FILTER(src0[2*x  ], src1[2*x  ], src0[2*x+1], src1[2*x+1]);
            dsth[x] = FILTER(src0[2*x+1], src1[2*x+1], src0[2*x+2], src1[2*x+2]);
            dstv[x] = FILTER(src1[2*x  ], src2[2*x  ], src1[2*x+1], src2[2*x+1]);
            dstc[x] = FILTER(src1[2*x+1], src2[2*x+1], src1[2*x+2], src2[2*x+2]);
#undef FILTER
        }
        src0 += src_stride*2;
        dst0 += dst_stride;
        dsth += dst_stride;
        dstv += dst_stride;
        dstc += dst_stride;
    }
}

void x264_mc_init( int cpu, x264_mc_functions_t *pf )
{
    pf->mc_luma   = mc_luma;
    pf->get_ref   = get_ref;
    pf->mc_chroma = mc_chroma;

    pf->copy[PIXEL_16x16] = mc_copy_w16;
    pf->copy[PIXEL_8x8]   = mc_copy_w8;
    pf->copy[PIXEL_4x4]   = mc_copy_w4;

    pf->store_interleave_chroma       = store_interleave_chroma;
    pf->load_deinterleave_chroma_fenc = load_deinterleave_chroma_fenc;
    pf->load_deinterleave_chroma_fdec = load_deinterleave_chroma_fdec;

    pf->plane_copy = x264_plane_copy_c;
    pf->plane_copy_interleave = x264_plane_copy_interleave_c;
    pf->plane_copy_deinterleave = x264_plane_copy_deinterleave_c;

    pf->hpel_filter = hpel_filter;

    pf->prefetch_fenc_420 = prefetch_fenc_null;
    pf->prefetch_ref  = prefetch_ref_null;
    pf->memcpy_aligned = memcpy;
    pf->memzero_aligned = memzero_aligned;
    pf->frame_init_lowres_core = frame_init_lowres_core;

#ifdef __TI_COMPILER_VERSION__
    pf->mc_chroma = mc_chroma_ti;

    pf->copy[PIXEL_16x16] = mc_copy_w16_ti;
    pf->copy[PIXEL_8x8]   = mc_copy_w8_ti;
    pf->copy[PIXEL_4x4]   = mc_copy_w4_ti;

    pf->store_interleave_chroma       = store_interleave_chroma_ti;
    pf->load_deinterleave_chroma_fenc = load_deinterleave_chroma_fenc_ti;
    pf->load_deinterleave_chroma_fdec = load_deinterleave_chroma_fdec_ti;

    pf->plane_copy_interleave = x264_plane_copy_interleave_ti;
    pf->plane_copy_deinterleave = x264_plane_copy_deinterleave_ti;
    pf->plane_copy_deinterlace = x264_plane_copy_deinterlace_ti;
    pf->plane_deinterlace = x264_plane_deinterlace_ti;

    pf->hpel_filter = hpel_filter_ti;

    pf->frame_init_lowres_core = frame_init_lowres_core_ti;
#endif
}

void x264_frame_filter( x264_t *h, x264_frame_t *frame, int mb_y, int b_end )
{
    /* during filtering, 8 extra pixels were filtered on each edge,
     * but up to 3 of the horizontal ones may be wrong.
     * x264_frame_expand_border_filtered is called after this
       to expand border from the last filtered pixel */

    int start = (mb_y<<4) - 8; // buffer = 4 for deblock + 3 for 6tap, rounded to 8
    int height = (b_end ? frame->i_lines[0] : (mb_y<<4)) + 8;
    int stride = frame->i_stride[0];
    const int width = frame->i_width[0];
    int offs = start*stride - 8; // buffer = 3 for 6tap, aligned to 8 for simd

	/* computes luma half-pixel values of frame for later encoding */
    h->mc.hpel_filter(
		frame->filtered[0][1] + offs, /* dst of h */
		frame->filtered[0][2] + offs, /* dst of v */
		frame->filtered[0][3] + offs, /* dst of hv */
		frame->plane[0] + offs,       /* src of luma */
		stride,                       /* stride */
		width + 16,                   /* width */
		height - start,               /* height */
		h->scratch_buffer );          /* buffer */

    /* generate integral image: (needed only for esa)
     * frame->integral contains 2 planes. in the upper plane, each element is
     * the sum of an 8x8 pixel region with top-left corner on that point.
     * in the lower plane, 4x4 sums (needed only with --partitions p4x4). */
    /* if( frame->integral ) */
}
