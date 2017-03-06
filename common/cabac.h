/*****************************************************************************
 * cabac.h: arithmetic coder
 *****************************************************************************/

#ifndef X264_CABAC_H
#define X264_CABAC_H

typedef struct
{
    /* state */
    int i_low;
    int i_range;

    /* bit stream */
    int i_queue; //stored with an offset of -8 for faster asm
    int i_bytes_outstanding;

    uint8_t *p_start; /* start position of bitstream. refer to h->out.bs.p_start */
    uint8_t *p;       /* current position of bitstream. refer to h->out.bs.p */
    uint8_t *p_end;   /* end position of bitstream. refer to h->out.bs.p_end */

    /* aligned for memcpy_aligned starting here */
    ALIGNED_16( int f8_bits_encoded ); // only if using x264_cabac_size_decision()

    /************************************************************************
     * States of all contexts are keeped in following state array.          *
     * The initial values are based on current slice type and qp.           *
     * States include following two parts, value range: [2~127]             *
     * The high 7 bit represents probability index of LPS. value: [1~63]    *
     * The lowest bit represents MPS(Most Probability Symbol). value: [0~1] *
     * Odd means MPS is 1, even means MPS is 0.                             *
     ************************************************************************/
    uint8_t state[276];

    /* for 16-byte alignment */
    uint8_t padding[12];
} x264_cabac_t;

extern const uint8_t x264_cabac_transition[128][2];
extern const uint16_t x264_cabac_entropy[128];

/* init the contexts given i_slice_type, the quantif and the model */
void x264_cabac_context_init( x264_t *h, x264_cabac_t *cb, int i_slice_type, int i_qp, int i_model );
void x264_cabac_encode_init ( x264_cabac_t *cb, uint8_t *p_data, uint8_t *p_end );
void x264_cabac_encode_decision_c( x264_cabac_t *cb, int i_ctx, int b );
void x264_cabac_encode_bypass_c( x264_cabac_t *cb, int b );
void x264_cabac_encode_terminal_c( x264_cabac_t *cb );
void x264_cabac_encode_ue_bypass( x264_cabac_t *cb, int exp_bits, int val );
void x264_cabac_encode_flush( x264_t *h, x264_cabac_t *cb );

#define x264_cabac_encode_decision x264_cabac_encode_decision_c
#define x264_cabac_encode_bypass x264_cabac_encode_bypass_c
#define x264_cabac_encode_terminal x264_cabac_encode_terminal_c
#define x264_cabac_encode_decision_noup x264_cabac_encode_decision

static ALWAYS_INLINE int x264_cabac_pos( x264_cabac_t *cb )
{
    return ((cb->p - cb->p_start + cb->i_bytes_outstanding) << 3) + cb->i_queue;
}

/* internal only. these don't write the bitstream, just calculate bit cost: */

static ALWAYS_INLINE void x264_cabac_size_decision( x264_cabac_t *cb, long i_ctx, long b )
{
    int i_state = cb->state[i_ctx];
    cb->state[i_ctx] = x264_cabac_transition[i_state][b];
    cb->f8_bits_encoded += x264_cabac_entropy[i_state^b];
}

static ALWAYS_INLINE int x264_cabac_size_decision2( uint8_t *state, long b )
{
    int i_state = *state;
    *state = x264_cabac_transition[i_state][b];
    return x264_cabac_entropy[i_state^b];
}

static ALWAYS_INLINE void x264_cabac_size_decision_noup( x264_cabac_t *cb, long i_ctx, long b )
{
    int i_state = cb->state[i_ctx];
    cb->f8_bits_encoded += x264_cabac_entropy[i_state^b];
}

static ALWAYS_INLINE int x264_cabac_size_decision_noup2( uint8_t *state, long b )
{
    return x264_cabac_entropy[*state^b];
}

#endif
