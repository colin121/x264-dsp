/*****************************************************************************
 * me.h: motion estimation
 *****************************************************************************/

#ifndef X264_ME_H
#define X264_ME_H

#define COST_MAX (1<<28)

typedef struct
{
    /* aligning the first member is a gcc hack to force the struct to be
     * 16 byte aligned, as well as force sizeof(struct) to be a multiple of 16 */
    /* input */
    ALIGNED_16( int i_pixel );   /* PIXEL_WxH */
    uint16_t *p_cost_mv; /* lambda * nbits for each possible mv */
    int      i_ref_cost; /* cost of reference number */
    int      i_ref;      /* reference number */
    const x264_weight_t *weight;

    pixel *p_fref[12]; /* refer to x264_t.mb.pic.p_fref */
    pixel *p_fref_w;   /* refer to x264_t.mb.pic.p_fref_w */
    pixel *p_fenc[3];
    int      i_stride[3];

    ALIGNED_4( int16_t mvp[2] ); /* mv predict x/y */

    /* output */
    int cost_mv;        /* lambda * nbits for the chosen mv */
    int cost;           /* satd + lambda * nbits */
    ALIGNED_4( int16_t mv[2] ); /* chosen mv x/y */
} ALIGNED_16( x264_me_t );

typedef struct
{
    int sad;
    int16_t mv[2];
} mvsad_t;

void x264_me_search_ref( x264_t *h, x264_me_t *m, int16_t (*mvc)[2], int i_mvc, int *p_fullpel_thresh );
#define x264_me_search( h, m, mvc, i_mvc ) x264_me_search_ref( h, m, mvc, i_mvc, NULL )

void x264_me_refine_qpel( x264_t *h, x264_me_t *m );
void x264_me_refine_qpel_refdupe( x264_t *h, x264_me_t *m, int *p_halfpel_thresh );

#define COPY1_IF_LT(x,y)\
if((y)<(x))\
    (x)=(y);

#define COPY2_IF_LT(x,y,a,b)\
if((y)<(x))\
{\
    (x)=(y);\
    (a)=(b);\
}

#define COPY3_IF_LT(x,y,a,b,c,d)\
if((y)<(x))\
{\
    (x)=(y);\
    (a)=(b);\
    (c)=(d);\
}

#define COPY4_IF_LT(x,y,a,b,c,d,e,f)\
if((y)<(x))\
{\
    (x)=(y);\
    (a)=(b);\
    (c)=(d);\
    (e)=(f);\
}

#define COPY2_IF_GT(x,y,a,b)\
if((y)>(x))\
{\
    (x)=(y);\
    (a)=(b);\
}

#endif
