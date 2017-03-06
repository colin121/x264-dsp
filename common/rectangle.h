/*****************************************************************************
 * rectangle.h: rectangle filling
 *****************************************************************************/

#ifdef __TI_COMPILER_VERSION__
#define x264_macroblock_cache_mv_ptr( a, x, y, w, h, l, mv ) x264_macroblock_cache_mv( a, x, y, w, h, l, M32( mv ) )
/* x264_macroblock_cache_mvd updates mv of (x,y) to x264_t.mb.cache.mv */
static ALWAYS_INLINE void x264_macroblock_cache_mv( x264_t *h, int x, int y, int width, int height, int i_list, uint32_t mv )
{
    uint8_t *mv_cache = (uint8_t *)(&h->mb.cache.mv[i_list][X264_SCAN8_0+x+(y<<3)]);
    uint64_t v8 = _itoll(mv, mv);

    if( width == 4 && height == 4 )
    {
    	_mem8((uint64_t *)(mv_cache +   0)) = v8;
    	_mem8((uint64_t *)(mv_cache +   8)) = v8;
    	_mem8((uint64_t *)(mv_cache +  32)) = v8;
    	_mem8((uint64_t *)(mv_cache +  40)) = v8;
    	_mem8((uint64_t *)(mv_cache +  64)) = v8;
    	_mem8((uint64_t *)(mv_cache +  72)) = v8;
    	_mem8((uint64_t *)(mv_cache +  96)) = v8;
    	_mem8((uint64_t *)(mv_cache + 104)) = v8;
    }
    else if( width == 4 && height == 2 )
    {
    	_mem8((uint64_t *)(mv_cache +   0)) = v8;
    	_mem8((uint64_t *)(mv_cache +   8)) = v8;
    	_mem8((uint64_t *)(mv_cache +  32)) = v8;
    	_mem8((uint64_t *)(mv_cache +  40)) = v8;
    }
    else if( width == 2 && height == 4 )
    {
    	_mem8((uint64_t *)(mv_cache +   0)) = v8;
    	_mem8((uint64_t *)(mv_cache +  32)) = v8;
    	_mem8((uint64_t *)(mv_cache +  64)) = v8;
    	_mem8((uint64_t *)(mv_cache +  96)) = v8;
    }
    else /* width == 2 && height == 2 */
    {
    	_mem8((uint64_t *)(mv_cache +   0)) = v8;
    	_mem8((uint64_t *)(mv_cache +  32)) = v8;
    }
}

/* x264_macroblock_cache_mvd updates mv delta of (x,y) to x264_t.mb.cache.mvd */
static ALWAYS_INLINE void x264_macroblock_cache_mvd( x264_t *h, int x, int y, int width, int height, int i_list, uint16_t mvd )
{
    uint8_t *mvd_cache = (uint8_t *)(&h->mb.cache.mvd[i_list][X264_SCAN8_0+x+(y<<3)]);
    uint32_t v4 = _pack2(mvd, mvd);
    uint64_t v8 = _itoll(v4, v4);

    if( width == 4 && height == 4 )
    {
    	_mem8((uint64_t *)(mvd_cache +  0)) = v8;
    	_mem8((uint64_t *)(mvd_cache + 16)) = v8;
    	_mem8((uint64_t *)(mvd_cache + 32)) = v8;
    	_mem8((uint64_t *)(mvd_cache + 48)) = v8;
    }
    else if( width == 4 && height == 2 )
    {
    	_mem8((uint64_t *)(mvd_cache +  0)) = v8;
    	_mem8((uint64_t *)(mvd_cache + 16)) = v8;
    }
    else if( width == 2 && height == 4 )
    {
    	_mem4((uint32_t *)(mvd_cache +  0)) = v4;
    	_mem4((uint32_t *)(mvd_cache + 16)) = v4;
    	_mem4((uint32_t *)(mvd_cache + 32)) = v4;
    	_mem4((uint32_t *)(mvd_cache + 48)) = v4;
    }
    else /* width == 2 && height == 2 */
    {
    	_mem4((uint32_t *)(mvd_cache +  0)) = v4;
    	_mem4((uint32_t *)(mvd_cache + 16)) = v4;
    }
}

/* x264_macroblock_cache_ref updates reference frame index of (x,y) to x264_t.mb.cache.ref */
static ALWAYS_INLINE void x264_macroblock_cache_ref( x264_t *h, int x, int y, int width, int height, int i_list, uint8_t ref )
{
    uint8_t *ref_cache = (uint8_t *)(&h->mb.cache.ref[i_list][X264_SCAN8_0+x+(y<<3)]);
    uint16_t v2 = _pack2(ref, ref);
    uint32_t v4 = _packl4(v2, v2);

    if( width == 4 && height == 4 )
    {
   		_mem4((uint32_t *)(ref_cache +  0)) = v4;
   		_mem4((uint32_t *)(ref_cache +  8)) = v4;
   		_mem4((uint32_t *)(ref_cache + 16)) = v4;
   		_mem4((uint32_t *)(ref_cache + 24)) = v4;
    }
    else if( width == 4 && height == 2 )
    {
   		_mem4((uint32_t *)(ref_cache +  0)) = v4;
   		_mem4((uint32_t *)(ref_cache +  8)) = v4;
    }
    else if( width == 2 && height == 4 )
    {
    	_mem2((uint16_t *)(ref_cache +  0)) = v2;
    	_mem2((uint16_t *)(ref_cache +  8)) = v2;
    	_mem2((uint16_t *)(ref_cache + 16)) = v2;
    	_mem2((uint16_t *)(ref_cache + 24)) = v2;
    }
    else /* width == 2 && height == 2 */
    {
    	_mem2((uint16_t *)(ref_cache +  0)) = v2;
    	_mem2((uint16_t *)(ref_cache +  8)) = v2;
    }
}
#else
/* This function should only be called with constant w / h / s arguments! */
/* x264_macroblock_cache_rect copy v to dst. w: width, h: height, s: size, v: value */
static ALWAYS_INLINE void x264_macroblock_cache_rect( void *dst, int w, int h, int s, uint32_t v )
{
    uint8_t *d = dst;
    uint16_t v2 = s == 2 ? v : v * 0x101;
    uint32_t v4 = s == 4 ? v : s == 2 ? v * 0x10001 : v * 0x1010101;
    uint64_t v8 = v4 + ((uint64_t)v4 << 32);
    s *= 8;

    if( w == 2 )
    {
        M16( d+s*0 ) = v2;
        if( h == 1 ) return;
        M16( d+s*1 ) = v2;
        if( h == 2 ) return;
        M16( d+s*2 ) = v2;
        M16( d+s*3 ) = v2;
    }
    else if( w == 4 )
    {
        M32( d+s*0 ) = v4;
        if( h == 1 ) return;
        M32( d+s*1 ) = v4;
        if( h == 2 ) return;
        M32( d+s*2 ) = v4;
        M32( d+s*3 ) = v4;
    }
    else if( w == 8 )
    {
        if( WORD_SIZE == 8 )
        {
            M64( d+s*0 ) = v8;
            if( h == 1 ) return;
            M64( d+s*1 ) = v8;
            if( h == 2 ) return;
            M64( d+s*2 ) = v8;
            M64( d+s*3 ) = v8;
        }
        else
        {
            M32( d+s*0+0 ) = v4;
            M32( d+s*0+4 ) = v4;
            if( h == 1 ) return;
            M32( d+s*1+0 ) = v4;
            M32( d+s*1+4 ) = v4;
            if( h == 2 ) return;
            M32( d+s*2+0 ) = v4;
            M32( d+s*2+4 ) = v4;
            M32( d+s*3+0 ) = v4;
            M32( d+s*3+4 ) = v4;
        }
    }
    else if( w == 16 )
    {
        /* height 1, width 16 doesn't occur */
        /* assert( h != 1 ); */
        if( WORD_SIZE == 8 )
        {
            do
            {
                M64( d+s*0+0 ) = v8;
                M64( d+s*0+8 ) = v8;
                M64( d+s*1+0 ) = v8;
                M64( d+s*1+8 ) = v8;
                h -= 2;
                d += s*2;
            } while( h );
        }
        else
        {
            do
            {
                M32( d+ 0 ) = v4;
                M32( d+ 4 ) = v4;
                M32( d+ 8 ) = v4;
                M32( d+12 ) = v4;
                d += s;
            } while( --h );
        }
    }
    /* else assert(0); */
}

#define x264_macroblock_cache_mv_ptr( a, x, y, w, h, l, mv ) x264_macroblock_cache_mv( a, x, y, w, h, l, M32( mv ) )
/* x264_macroblock_cache_mvd updates mv of (x,y) to x264_t.mb.cache.mv */
static ALWAYS_INLINE void x264_macroblock_cache_mv( x264_t *h, int x, int y, int width, int height, int i_list, uint32_t mv )
{
    void *mv_cache = &h->mb.cache.mv[i_list][X264_SCAN8_0+x+(y<<3)];
    x264_macroblock_cache_rect( mv_cache, width<<2, height, 4, mv );
}

/* x264_macroblock_cache_mvd updates mv delta of (x,y) to x264_t.mb.cache.mvd */
static ALWAYS_INLINE void x264_macroblock_cache_mvd( x264_t *h, int x, int y, int width, int height, int i_list, uint16_t mvd )
{
    void *mvd_cache = &h->mb.cache.mvd[i_list][X264_SCAN8_0+x+(y<<3)];
    x264_macroblock_cache_rect( mvd_cache, width<<1, height, 2, mvd );
}

/* x264_macroblock_cache_ref updates reference frame index of (x,y) to x264_t.mb.cache.ref */
static ALWAYS_INLINE void x264_macroblock_cache_ref( x264_t *h, int x, int y, int width, int height, int i_list, uint8_t ref )
{
    void *ref_cache = &h->mb.cache.ref[i_list][X264_SCAN8_0+x+(y<<3)];
    x264_macroblock_cache_rect( ref_cache, width, height, 1, ref );
}
#endif
