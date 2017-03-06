/*****************************************************************************
 * osdep.h: platform-specific code
 *****************************************************************************/

#ifndef X264_OSDEP_H
#define X264_OSDEP_H

#include <stdio.h>
#include <inttypes.h>
#include <math.h>

#ifdef __TI_COMPILER_VERSION__
#define HAVE_MEM_ALIGN     1  /* TI compiler defines memalign in stdlib.h, there is no memory.h */
#define HAVE_LOG2F         1  /* TI compiler defines log2f in mathf.h */
#define HAVE_THREAD        0  /* disable threads */
#define HAVE_INTERLACED    0  /* disable interlaced */
/********************************************************
 * X264_CHROMA_FORMAT specifies the chroma formats that *
 * x264 supports encoding. When this value is non-zero, *
 * then it represents a X264_CSP_* that is the only     *
 * chroma format that x264 supports encoding. If the    *
 * value is 0 then there are no restrictions.           *
 ********************************************************/
#define X264_CHROMA_FORMAT 0  /* default: 0 */
#define X264_BIT_DEPTH     8  /* bit-depth: 8 */
#define HIGH_BIT_DEPTH     0  /* disable high bit-depth */
#endif

#define DECLARE_ALIGNED( var, n ) var __attribute__((aligned(n)))
#define ALIGNED_16( var ) DECLARE_ALIGNED( var, 16 )
#define ALIGNED_8( var )  DECLARE_ALIGNED( var, 8 )
#define ALIGNED_4( var )  DECLARE_ALIGNED( var, 4 )

// ARM compiliers don't reliably align stack variables
// - EABI requires only 8 byte stack alignment to be maintained
// - gcc can't align stack variables to more even if the stack were to be correctly aligned outside the function
// - armcc can't either, but is nice enough to actually tell you so
// - Apple gcc only maintains 4 byte alignment
// - llvm can align the stack, but only in svn and (unrelated) it exposes bugs in all released GNU binutils...

#define ALIGNED_ARRAY_EMU( mask, type, name, sub1, ... )\
    uint8_t name##_u [sizeof(type sub1 __VA_ARGS__) + mask]; \
    type (*name) __VA_ARGS__ = (void*)((intptr_t)(name##_u+mask) & ~mask)

#define ALIGNED_ARRAY_8( type, name, sub1, ... )\
    ALIGNED_8( type name sub1 __VA_ARGS__ )
#define ALIGNED_ARRAY_16( type, name, sub1, ... )\
    ALIGNED_16( type name sub1 __VA_ARGS__ )
#define ALIGNED_ARRAY_32( ... ) ALIGNED_ARRAY_EMU( 31, __VA_ARGS__ )
#define ALIGNED_ARRAY_64( ... ) ALIGNED_ARRAY_EMU( 63, __VA_ARGS__ )

/* GCC specific features support */
#if defined(__GNUC__) && (__GNUC__ > 3 || __GNUC__ == 3 && __GNUC_MINOR__ > 0)
#define UNUSED __attribute__((unused))
#define ALWAYS_INLINE __attribute__((always_inline)) inline
#define NOINLINE __attribute__((noinline))
#define MAY_ALIAS __attribute__((may_alias))
#define x264_constant_p(x) __builtin_constant_p(x)
#define x264_nonconstant_p(x) (!__builtin_constant_p(x))
#elif defined(__TI_COMPILER_VERSION__)
/* FIXME: #elif defined(__TI_GNU_ATTRIBUTE_SUPPORT__) */ /* Defined if GCC extensions are enabled (-gcc) */
#define UNUSED __attribute__((unused))
#define ALWAYS_INLINE __attribute__((always_inline)) inline
#define NOINLINE __attribute__((noinline))
#define MAY_ALIAS /* Not supported according to Section 6.15.2 of <TMS320C6000 Optimizing Compiler v7.4 User's Guide> */
#define x264_constant_p(x) __builtin_constant_p(x)
#define x264_nonconstant_p(x) (!__builtin_constant_p(x))
#else
#define UNUSED
#define ALWAYS_INLINE inline
#define NOINLINE
#define MAY_ALIAS
#define x264_constant_p(x) 0
#define x264_nonconstant_p(x) 0
#endif

#if !HAVE_LOG2F
#define log2f(x) (logf(x)*1.4426950408889634f)
#define log2(x) (log(x)*1.4426950408889634)
#endif

/* threads */
#if HAVE_POSIXTHREAD
#include <pthread.h>
#define x264_pthread_t               pthread_t
#define x264_pthread_create          pthread_create
#define x264_pthread_join            pthread_join
#define x264_pthread_mutex_t         pthread_mutex_t
#define x264_pthread_mutex_init      pthread_mutex_init
#define x264_pthread_mutex_destroy   pthread_mutex_destroy
#define x264_pthread_mutex_lock      pthread_mutex_lock
#define x264_pthread_mutex_unlock    pthread_mutex_unlock
#define x264_pthread_cond_t          pthread_cond_t
#define x264_pthread_cond_init       pthread_cond_init
#define x264_pthread_cond_destroy    pthread_cond_destroy
#define x264_pthread_cond_broadcast  pthread_cond_broadcast
#define x264_pthread_cond_wait       pthread_cond_wait
#define x264_pthread_attr_t          pthread_attr_t
#define x264_pthread_attr_init       pthread_attr_init
#define x264_pthread_attr_destroy    pthread_attr_destroy
#define X264_PTHREAD_MUTEX_INITIALIZER PTHREAD_MUTEX_INITIALIZER
#else
#define x264_pthread_t               int
#define x264_pthread_create(t,u,f,d) 0
#define x264_pthread_join(t,s)
#define x264_pthread_mutex_t         int
#define x264_pthread_mutex_init(m,f) 0
#define x264_pthread_mutex_destroy(m)
#define x264_pthread_mutex_lock(m)
#define x264_pthread_mutex_unlock(m)
#define x264_pthread_cond_t          int
#define x264_pthread_cond_init(c,f)  0
#define x264_pthread_cond_destroy(c)
#define x264_pthread_cond_broadcast(c)
#define x264_pthread_cond_wait(c,m)
#define x264_pthread_attr_t          int
#define x264_pthread_attr_init(a)    0
#define x264_pthread_attr_destroy(a)
#define X264_PTHREAD_MUTEX_INITIALIZER 0
#endif //HAVE_*THREAD

#define WORD_SIZE sizeof(void*)

#if WORDS_BIGENDIAN
#define endian_fix(x) (x)
#define endian_fix64(x) (x)
#define endian_fix32(x) (x)
#define endian_fix16(x) (x)
#else
static ALWAYS_INLINE uint32_t endian_fix32( uint32_t x )
{
    return (x<<24) + ((x<<8)&0xff0000) + ((x>>8)&0xff00) + (x>>24);
}
static ALWAYS_INLINE uint64_t endian_fix64( uint64_t x )
{
    return endian_fix32(x>>32) + ((uint64_t)endian_fix32(x)<<32);
}
static ALWAYS_INLINE intptr_t endian_fix( intptr_t x )
{
    return WORD_SIZE == 8 ? endian_fix64(x) : endian_fix32(x);
}
#ifndef endian_fix16
static ALWAYS_INLINE uint16_t endian_fix16( uint16_t x )
{
    return (x<<8)|(x>>8);
}
#endif
#endif

#if defined(__GNUC__) && (__GNUC__ > 3 || __GNUC__ == 3 && __GNUC_MINOR__ > 3)
#define x264_clz(x) __builtin_clz(x)
#define x264_ctz(x) __builtin_ctz(x)
#else
static int ALWAYS_INLINE x264_clz( uint32_t x )
{
    static uint8_t lut[16] = {4,3,2,2,1,1,1,1,0,0,0,0,0,0,0,0};
    int y, z = (((x >> 16) - 1) >> 27) & 16;
    x >>= z^16;
    z += y = ((x - 0x100) >> 28) & 8;
    x >>= y^8;
    z += y = ((x - 0x10) >> 29) & 4;
    x >>= y^4;
    return z + lut[x];
}

static int ALWAYS_INLINE x264_ctz( uint32_t x )
{
    static uint8_t lut[16] = {4,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0};
    int y, z = (((x & 0xffff) - 1) >> 27) & 16;
    x >>= z;
    z += y = (((x & 0xff) - 1) >> 28) & 8;
    x >>= y;
    z += y = (((x & 0xf) - 1) >> 29) & 4;
    x >>= y;
    return z + lut[x&0xf];
}
#endif

/* We require that prefetch not fault on invalid reads, so we only enable it on
 * known architectures. */
#if defined(__GNUC__) && (__GNUC__ > 3 || __GNUC__ == 3 && __GNUC_MINOR__ > 1) &&\
      (ARCH_X86 || ARCH_X86_64 || ARCH_ARM || ARCH_PPC)
#define x264_prefetch(x) __builtin_prefetch(x)
#else
#define x264_prefetch(x) /* Not supported according to Section 6.15.5 of <TMS320C6000 Optimizing Compiler v7.4 User's Guide> */
#endif

#define x264_stack_align(func,...) func(__VA_ARGS__)

#endif /* X264_OSDEP_H */
