/*****************************************************************************
 * bitstream.c: bitstream writing
 *****************************************************************************/

#include "common.h"

#ifdef __TI_COMPILER_VERSION__
uint8_t *x264_nal_escape_ti( uint8_t *dst, uint8_t *src, uint8_t *end );
#endif

static uint8_t *x264_nal_escape_c( uint8_t *dst, uint8_t *src, uint8_t *end )
{
    if( src < end ) *dst++ = *src++;
    if( src < end ) *dst++ = *src++;
    while( src < end )
    {
        if( src[0] <= 0x03 && !dst[-2] && !dst[-1] )
            *dst++ = 0x03;
        *dst++ = *src++;
    }
    return dst;
}

/****************************************************************************
 * x264_nal_encode:
 ****************************************************************************/
void x264_nal_encode( x264_t *h, uint8_t *dst, x264_nal_t *nal )
{
    uint8_t *src = nal->p_payload;
    uint8_t *end = nal->p_payload + nal->i_payload;
    uint8_t *orig_dst = dst;
	int size;

    if( h->param.b_annexb )
    {
        if( nal->b_long_startcode )
            *dst++ = 0x00;
        *dst++ = 0x00;
        *dst++ = 0x00;
        *dst++ = 0x01;
    }
    else /* save room for size later */
        dst += 4;

    /* nal header: type + ref_idc */
    *dst++ = ( 0x00 << 7 ) | ( nal->i_ref_idc << 5 ) | nal->i_type;

    dst = h->bsf.nal_escape( dst, src, end );
    size = (dst - orig_dst) - 4;

    /* Write the size header for mp4/etc */
    if( !h->param.b_annexb )
    {
        /* Size doesn't include the size of the header we're writing now. */
        orig_dst[0] = size>>24;
        orig_dst[1] = size>>16;
        orig_dst[2] = size>> 8;
        orig_dst[3] = size>> 0;
    }

    nal->i_payload = size+4;
    nal->p_payload = orig_dst;
}

void x264_bitstream_init( int cpu, x264_bitstream_function_t *pf )
{
    pf->nal_escape = x264_nal_escape_c;
#ifdef __TI_COMPILER_VERSION__
    pf->nal_escape = x264_nal_escape_ti;
#endif
}
