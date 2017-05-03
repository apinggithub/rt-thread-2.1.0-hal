/*
 * File      : kservice.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2012, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2006-03-16     Bernard      the first version
 * 2006-05-25     Bernard      rewrite vsprintf
 * 2006-08-10     Bernard      add rt_show_version
 * 2010-03-17     Bernard      remove rt_strlcpy function
 *                             fix gcc compiling issue.
 * 2010-04-15     Bernard      remove weak definition on ICCM16C compiler
 * 2012-07-18     Arda         add the alignment display for signed integer
 * 2012-11-23     Bernard      fix IAR compiler error.
 * 2012-12-22     Bernard      fix rt_kprintf issue, which found by Grissiom.
 * 2013-06-24     Bernard      remove rt_kprintf if RT_USING_CONSOLE is not defined.
 * 2013-09-24     aozima       make sure the device is in STREAM mode when used by rt_kprintf.
 * 2015-07-06     Bernard      Add rt_assert_handler routine.
 */

#include <rtthread.h>
#include <rthw.h>
#define INCLUDE_STRING

#ifdef INCLUDE_STRING
    #include "string.h"
#endif

#ifdef KERNEL
    #define NOFLOAT
#endif

#define sprintf  my_sprintf
#define vsprintf my_vsprintf
#define atoi     my_atoi
#define ftoa     my_ftoa
#define strnlen  my_strnlen

#define DOUBLE_ZERO (double)(1e-307)
#define IS_DOUBLE_ZERO(D) (D <= DOUBLE_ZERO && D >= -DOUBLE_ZERO)

#define abs(a)  ((a) < 0 ?  -(a) :(a))
#define is_digit(c) ((c) >= '0' && (c) <= '9')
/////////////////////////////////////////////////////////////////////////////

#define FLT_MAX_10_EXP     38
#define DBL_MAX_10_EXP     308
#define LDBL_MAX_10_EXP    308
/* use precision */
#define RT_PRINTF_PRECISION

/**
 * @addtogroup KernelService
 */

/*@{*/

/* global errno in RT-Thread */
static volatile int _errno;

#if defined(RT_USING_DEVICE) && defined(RT_USING_CONSOLE)
static rt_device_t _console_device = RT_NULL;
#endif

/*
 * This function will get errno
 *
 * @return errno
 */
rt_err_t rt_get_errno(void)
{
    rt_thread_t tid;

    if (rt_interrupt_get_nest() != 0)
    {
        /* it's in interrupt context */
        return _errno;
    }

    tid = rt_thread_self();
    if (tid == RT_NULL)
        return _errno;

    return tid->error;
}
RTM_EXPORT(rt_get_errno);

/*
 * This function will set errno
 *
 * @param error the errno shall be set
 */
void rt_set_errno(rt_err_t error)
{
    rt_thread_t tid;

    if (rt_interrupt_get_nest() != 0)
    {
        /* it's in interrupt context */
        _errno = error;

        return;
    }

    tid = rt_thread_self();
    if (tid == RT_NULL)
    {
        _errno = error;

        return;
    }

    tid->error = error;
}
RTM_EXPORT(rt_set_errno);

/**
 * This function returns errno.
 *
 * @return the errno in the system
 */
int *_rt_errno(void)
{
    rt_thread_t tid;

    if (rt_interrupt_get_nest() != 0)
        return (int *)&_errno;

    tid = rt_thread_self();
    if (tid != RT_NULL)
        return (int *)&(tid->error);

    return (int *)&_errno;
}
RTM_EXPORT(_rt_errno);

/**
 * This function will set the content of memory to specified value
 *
 * @param s the address of source memory
 * @param c the value shall be set in content
 * @param count the copied length
 *
 * @return the address of source memory
 */
void *rt_memset(void *s, int c, rt_ubase_t count)
{
#ifdef RT_TINY_SIZE
    char *xs = (char *)s;

    while (count--)
        *xs++ = c;

    return s;
#else
#define LBLOCKSIZE      (sizeof(rt_int32_t))
#define UNALIGNED(X)    ((rt_int32_t)X & (LBLOCKSIZE - 1))
#define TOO_SMALL(LEN)  ((LEN) < LBLOCKSIZE)

    int i;
    char *m = (char *)s;
    rt_uint32_t buffer;
    rt_uint32_t *aligned_addr;
    rt_uint32_t d = c & 0xff;

    if (!TOO_SMALL(count) && !UNALIGNED(s))
    {
        /* If we get this far, we know that n is large and m is word-aligned. */
        aligned_addr = (rt_uint32_t *)s;

        /* Store D into each char sized location in BUFFER so that
         * we can set large blocks quickly.
         */
        if (LBLOCKSIZE == 4)
        {
            buffer = (d << 8) | d;
            buffer |= (buffer << 16);
        }
        else
        {
            buffer = 0;
            for (i = 0; i < LBLOCKSIZE; i ++)
                buffer = (buffer << 8) | d;
        }

        while (count >= LBLOCKSIZE * 4)
        {
            *aligned_addr++ = buffer;
            *aligned_addr++ = buffer;
            *aligned_addr++ = buffer;
            *aligned_addr++ = buffer;
            count -= 4 * LBLOCKSIZE;
        }

        while (count >= LBLOCKSIZE)
        {
            *aligned_addr++ = buffer;
            count -= LBLOCKSIZE;
        }

        /* Pick up the remainder with a bytewise loop. */
        m = (char *)aligned_addr;
    }

    while (count--)
    {
        *m++ = (char)d;
    }

    return s;

#undef LBLOCKSIZE
#undef UNALIGNED
#undef TOO_SMALL
#endif
}
RTM_EXPORT(rt_memset);

/**
 * This function will copy memory content from source address to destination
 * address.
 *
 * @param dst the address of destination memory
 * @param src  the address of source memory
 * @param count the copied length
 *
 * @return the address of destination memory
 */
void *rt_memcpy(void *dst, const void *src, rt_ubase_t count)
{
#ifdef RT_TINY_SIZE
    char *tmp = (char *)dst, *s = (char *)src;

    while (count--)
        *tmp++ = *s++;

    return dst;
#else

#define UNALIGNED(X, Y)                                               \
                        (((rt_int32_t)X & (sizeof(rt_int32_t) - 1)) | \
                         ((rt_int32_t)Y & (sizeof(rt_int32_t) - 1)))
#define BIGBLOCKSIZE    (sizeof(rt_int32_t) << 2)
#define LITTLEBLOCKSIZE (sizeof(rt_int32_t))
#define TOO_SMALL(LEN)  ((LEN) < BIGBLOCKSIZE)

    char *dst_ptr = (char *)dst;
    char *src_ptr = (char *)src;
    rt_int32_t *aligned_dst;
    rt_int32_t *aligned_src;
    int len = count;

    /* If the size is small, or either SRC or DST is unaligned,
    then punt into the byte copy loop.  This should be rare. */
    if (!TOO_SMALL(len) && !UNALIGNED(src_ptr, dst_ptr))
    {
        aligned_dst = (rt_int32_t *)dst_ptr;
        aligned_src = (rt_int32_t *)src_ptr;

        /* Copy 4X long words at a time if possible. */
        while (len >= BIGBLOCKSIZE)
        {
            *aligned_dst++ = *aligned_src++;
            *aligned_dst++ = *aligned_src++;
            *aligned_dst++ = *aligned_src++;
            *aligned_dst++ = *aligned_src++;
            len -= BIGBLOCKSIZE;
        }

        /* Copy one long word at a time if possible. */
        while (len >= LITTLEBLOCKSIZE)
        {
            *aligned_dst++ = *aligned_src++;
            len -= LITTLEBLOCKSIZE;
        }

        /* Pick up any residual with a byte copier. */
        dst_ptr = (char *)aligned_dst;
        src_ptr = (char *)aligned_src;
    }

    while (len--)
        *dst_ptr++ = *src_ptr++;

    return dst;
#undef UNALIGNED
#undef BIGBLOCKSIZE
#undef LITTLEBLOCKSIZE
#undef TOO_SMALL
#endif
}
RTM_EXPORT(rt_memcpy);

/**
 * This function will move memory content from source address to destination
 * address.
 *
 * @param dest the address of destination memory
 * @param src  the address of source memory
 * @param n the copied length
 *
 * @return the address of destination memory
 */
void *rt_memmove(void *dest, const void *src, rt_ubase_t n)
{
    char *tmp = (char *)dest, *s = (char *)src;

    if (s < tmp && tmp < s + n)
    {
        tmp += n;
        s += n;

        while (n--)
            *(--tmp) = *(--s);
    }
    else
    {
        while (n--)
            *tmp++ = *s++;
    }

    return dest;
}
RTM_EXPORT(rt_memmove);

/**
 * This function will compare two areas of memory
 *
 * @param cs one area of memory
 * @param ct znother area of memory
 * @param count the size of the area
 *
 * @return the result
 */
rt_int32_t rt_memcmp(const void *cs, const void *ct, rt_ubase_t count)
{
    const unsigned char *su1, *su2;
    int res = 0;

    for (su1 = cs, su2 = ct; 0 < count; ++su1, ++su2, count--)
        if ((res = *su1 - *su2) != 0)
            break;

    return res;
}
RTM_EXPORT(rt_memcmp);

/**
 * This function will return the first occurrence of a string.
 *
 * @param s1 the source string
 * @param s2 the find string
 *
 * @return the first occurrence of a s2 in s1, or RT_NULL if no found.
 */
char *rt_strstr(const char *s1, const char *s2)
{
    int l1, l2;

    l2 = rt_strlen(s2);
    if (!l2)
        return (char *)s1;
    l1 = rt_strlen(s1);
    while (l1 >= l2)
    {
        l1 --;
        if (!rt_memcmp(s1, s2, l2))
            return (char *)s1;
        s1 ++;
    }

    return RT_NULL;
}
RTM_EXPORT(rt_strstr);

/**
 * This function will compare two strings while ignoring differences in case
 *
 * @param a the string to be compared
 * @param b the string to be compared
 *
 * @return the result
 */
rt_uint32_t rt_strcasecmp(const char *a, const char *b)
{
    int ca, cb;

    do
    {
        ca = *a++ & 0xff;
        cb = *b++ & 0xff;
        if (ca >= 'A' && ca <= 'Z')
            ca += 'a' - 'A';
        if (cb >= 'A' && cb <= 'Z')
            cb += 'a' - 'A';
    }
    while (ca == cb && ca != '\0');

    return ca - cb;
}
RTM_EXPORT(rt_strcasecmp);

/**
 * This function will copy string no more than n bytes.
 *
 * @param dst the string to copy
 * @param src the string to be copied
 * @param n the maximum copied length
 *
 * @return the result
 */
char *rt_strncpy(char *dst, const char *src, rt_ubase_t n)
{
    if (n != 0)
    {
        char *d = dst;
        const char *s = src;

        do
        {
            if ((*d++ = *s++) == 0)
            {
                /* NUL pad the remaining n-1 bytes */
                while (--n != 0)
                    *d++ = 0;
                break;
            }
        } while (--n != 0);
    }

    return (dst);
}
RTM_EXPORT(rt_strncpy);

/**
 * This function will compare two strings with specified maximum length
 *
 * @param cs the string to be compared
 * @param ct the string to be compared
 * @param count the maximum compare length
 *
 * @return the result
 */
rt_int32_t rt_strncmp(const char *cs, const char *ct, rt_ubase_t count)
{
    register signed char __res = 0;

    while (count)
    {
        if ((__res = *cs - *ct++) != 0 || !*cs++)
            break;
        count --;
    }

    return __res;
}
RTM_EXPORT(rt_strncmp);

/**
 * This function will compare two strings without specified length
 *
 * @param cs the string to be compared
 * @param ct the string to be compared
 *
 * @return the result
 */
rt_int32_t rt_strcmp(const char *cs, const char *ct)
{
    while (*cs && *cs == *ct)
        cs++, ct++;

    return (*cs - *ct);
}
RTM_EXPORT(rt_strcmp);

/**
 * This function will return the length of a string, which terminate will
 * null character.
 *
 * @param s the string
 *
 * @return the length of string
 */
rt_size_t rt_strlen(const char *s)
{
    const char *sc;

    for (sc = s; *sc != '\0'; ++sc) /* nothing */
        ;

    return sc - s;
}
RTM_EXPORT(rt_strlen);

#ifdef RT_USING_HEAP
/**
 * This function will duplicate a string.
 *
 * @param s the string to be duplicated
 *
 * @return the duplicated string pointer
 */
char *rt_strdup(const char *s)
{
    rt_size_t len = rt_strlen(s) + 1;
    char *tmp = (char *)rt_malloc(len);

    if (!tmp)
        return RT_NULL;

    rt_memcpy(tmp, s, len);

    return tmp;
}
RTM_EXPORT(rt_strdup);
#endif

/**
 * This function will show the version of rt-thread rtos
 */
void rt_show_version(void)
{
    rt_kprintf("\n \\ | /\n");
    rt_kprintf("- RT -     Thread Operating System\n");
    rt_kprintf(" / | \\     %d.%d.%d build %s\n",
               RT_VERSION, RT_SUBVERSION, RT_REVISION, __DATE__);
    rt_kprintf(" 2006 - 2016 Copyright by rt-thread team\n");
}
RTM_EXPORT(rt_show_version);

/* private function */
#define isdigit(c)  ((unsigned)((c) - '0') < 10)

rt_inline rt_int32_t divide(rt_int32_t *n, rt_int32_t base)
{
    rt_int32_t res;

    /* optimized for processor which does not support divide instructions. */
    if (base == 10)
    {
        res = ((rt_uint32_t)*n) % 10U;
        *n = ((rt_uint32_t)*n) / 10U;
    }
    else
    {
        res = ((rt_uint32_t)*n) % 16U;
        *n = ((rt_uint32_t)*n) / 16U;
    }

    return res;
}

rt_inline int skip_atoi(const char **s)
{
    register int i=0;
    while (isdigit(**s))
        i = i * 10 + *((*s)++) - '0';

    return i;
}

#define ZEROPAD     (1 << 0)    /* pad with zero */
#define SIGN        (1 << 1)    /* unsigned/signed long */
#define PLUS        (1 << 2)    /* show plus */
#define SPACE       (1 << 3)    /* space if plus */
#define LEFT        (1 << 4)    /* left justified */
#define SPECIAL     (1 << 5)    /* 0x */
#define LARGE       (1 << 6)    /* use 'ABCDEF' instead of 'abcdef' */

#ifdef RT_PRINTF_PRECISION
static char *print_number(char *buf,
                          char *end,
                          long  num,
                          int   base,
                          int   s,
                          int   precision,
                          int   type)
#else
static char *print_number(char *buf,
                          char *end,
                          long  num,
                          int   base,
                          int   s,
                          int   type)
#endif
{
    char c, sign;
#ifdef RT_PRINTF_LONGLONG
    char tmp[32];
#else
    char tmp[16];
#endif
    const char *digits;
    static const char small_digits[] = "0123456789abcdef";
    static const char large_digits[] = "0123456789ABCDEF";
    register int i;
    register int size;

    size = s;

    digits = (type & LARGE) ? large_digits : small_digits;
    if (type & LEFT)
        type &= ~ZEROPAD;

    c = (type & ZEROPAD) ? '0' : ' ';

    /* get sign */
    sign = 0;
    if (type & SIGN)
    {
        if (num < 0)
        {
            sign = '-';
            num = -num;
        }
        else if (type & PLUS)
            sign = '+';
        else if (type & SPACE)
            sign = ' ';
    }

#ifdef RT_PRINTF_SPECIAL
    if (type & SPECIAL)
    {
        if (base == 16)
            size -= 2;
        else if (base == 8)
            size--;
    }
#endif

    i = 0;
    if (num == 0)
        tmp[i++]='0';
    else
    {
        while (num != 0)
            tmp[i++] = digits[divide(&num, base)];
    }

#ifdef RT_PRINTF_PRECISION
    if (i > precision)
        precision = i;
    size -= precision;
#else
    size -= i;
#endif

    if (!(type&(ZEROPAD | LEFT)))
    {
        if ((sign)&&(size>0))
            size--;

        while (size-->0)
        {
            if (buf <= end)
                *buf = ' ';
            ++ buf;
        }
    }

    if (sign)
    {
        if (buf <= end)
        {
            *buf = sign;
            -- size;
        }
        ++ buf;
    }

#ifdef RT_PRINTF_SPECIAL
    if (type & SPECIAL)
    {
        if (base==8)
        {
            if (buf <= end)
                *buf = '0';
            ++ buf;
        }
        else if (base == 16)
        {
            if (buf <= end)
                *buf = '0';
            ++ buf;
            if (buf <= end)
            {
                *buf = type & LARGE? 'X' : 'x';
            }
            ++ buf;
        }
    }
#endif

    /* no align to the left */
    if (!(type & LEFT))
    {
        while (size-- > 0)
        {
            if (buf <= end)
                *buf = c;
            ++ buf;
        }
    }

#ifdef RT_PRINTF_PRECISION
    while (i < precision--)
    {
        if (buf <= end)
            *buf = '0';
        ++ buf;
    }
#endif

    /* put number in the temporary buffer */
    while (i-- > 0)
    {
        if (buf <= end)
            *buf = tmp[i];
        ++ buf;
    }

    while (size-- > 0)
    {
        if (buf <= end)
            *buf = ' ';
        ++ buf;
    }

    return buf;
}
/*
my_vsprintf
*/
//static char *digits = "0123456789abcdefghijklmnopqrstuvwxyz";
//static char *upper_digits = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
static int is_space( int ch )
{
    return (unsigned long)(ch - 9) < 5u || ' ' == ch;
}

static int atoi(char *str)
{
   int sign;
   int n;
   char *p = str;

   while (is_space(*p) ) p++;

   sign = ('-' == *p) ? -1 : 1;
   if ('+' == *p || '-' == *p) p++;

   for (n = 0; is_digit(*p); p++)
      n = 10 * n + (*p - '0');

   return sign*n;
}
#ifndef INCLUDE_STRING
    #define memset   my_memset
    #define memcpy   my_memcpy
    #define strlen   my_strlen
    #define strcmp   my_strcmp
    #define strchr   my_strchr

static char * strchr(const char *str, int ch)
{
    while (*str && *str != (char)ch) str++;

    if (*str == (char)ch)
        return((char *)str);

    return 0;
}
static void * memset(void *dst, int val, unsigned long ulcount)
{
    if(!dst) return 0;
    char * pchdst = (char*)dst;
    while(ulcount--) *pchdst++ = (char)val;

    return dst;
}

static void * memcpy(void *dst, const void *src, unsigned long ulcount)
{
   if(!(dst && src)) return 0;
   char * pchdst = (char*)dst;
   char * pchsrc = (char*)src;
   while(ulcount--) *pchdst++ = *pchsrc++;
   
   return dst;
}

static int strlen(const char * str)
{
    const char *p = str;
    while(*p++);

    return (int)(p - str - 1);
}
int strcmp(const char *source,const char *dest)
{
    int ret = 0;
    if(!source || !dest) return -2;
    while( ! (ret = *( unsigned char *)source - *(unsigned char *)dest) && *dest)
    {
        source++;
        dest++;
    }
    
    if ( ret < 0 )
        ret = -1 ;
    else if ( ret > 0 )
        ret = 1 ;

    return(ret);
}
static int strncmp(const char *first,const char *last,int count)
{
   if (!count)  return 0;

   while (--count && *first && *first == *last) first++,last++;

   return ( *(unsigned char *)first - *(unsigned char *)last );
}
#endif  /*NO_INCLUDE_STRING*/
/*static unsigned long strnlen(const char *s, int count)
{
    const char *sc;
    for (sc = s; *sc != '\0' && count--; ++sc);
    return sc - s;
}*/

static char * itoa(int n, char * chBuffer)
{
    int i = 1;
    char * pch = chBuffer;
    if(!pch) return 0;
    while(n / i) i *= 10;
    
    if(n < 0)
    {
        n = -n;
        *pch++ = '-';
    }
    if (0 == n) i = 10;
    
    while(i /= 10)
    {
        *pch++ = n / i + '0';
        n %= i;
    }
    *pch = '\0';
    return chBuffer;
}
/*
static char * number(char *str, long num, int base, int size, int precision, int type)
{
    char c, sign, tmp[66];
    char *dig = digits;
    int i;

    if (type & LARGE)  dig = upper_digits;
    if (type & LEFT) type &= ~ZEROPAD;
    if (base < 2 || base > 36) return 0;

    c = (type & ZEROPAD) ? '0' : ' ';
    sign = 0;
    if (type & SIGN)
    {
        if (num < 0)
        {
            sign = '-';
            num = -num;
            size--;
        }
        else if (type & PLUS)
        {
            sign = '+';
            size--;
        }
        else if (type & SPACE)
        {
            sign = ' ';
            size--;
        }
    }

    if (type & SPECIAL)
    {
        if (16 == base)
            size -= 2;
        else if (8 == base)
            size--;
    }

    i = 0;

    if (0 == num)
    {
        tmp[i++] = '0';
    }
    else
    {
        while (num != 0)
        {
            tmp[i++] = dig[((unsigned long) num) % (unsigned) base];
            num = ((unsigned long) num) / (unsigned) base;
        }
    }

    if (i > precision) precision = i;
    size -= precision;
    if (!(type & (ZEROPAD | LEFT)))
    {
        while(size-- > 0) *str++ = ' ';
    }
    if (sign) *str++ = sign;

    if (type & SPECIAL)
    {
        if (8 == base)
        {
            *str++ = '0';
        }
        else if (16 == base)
        {
            *str++ = '0';
            *str++ = digits[33];
        }
    }

    if(!(type & LEFT))
    {
        while(size-- > 0) *str++ = c;
    }
    while(i < precision--) *str++ = '0';
    while(i-- > 0) *str++ = tmp[i];
    while(size-- > 0) *str++ = ' ';

    return str;
}

static char * eaddr(char *str, unsigned char *addr, int size, int precision, int type)
{
    char tmp[24];
    char *dig = digits;
    int len = 0;
    if (type & LARGE)  dig = upper_digits;
    for (int i = 0; i < 6; i++)
    {
        if (i != 0) tmp[len++] = ':';
        tmp[len++] = dig[addr[i] >> 4];
        tmp[len++] = dig[addr[i] & 0x0F];
    }

    if (!(type & LEFT))
    {
        while (len < size--) *str++ = ' ';
    }
    
    for (int i = 0; i < len; ++i)
    {
        *str++ = tmp[i];
    }
    
    while (len < size--) *str++ = ' ';

    return str;
}

static char * iaddr(char *str, unsigned char *addr, int size, int precision, int type)
{
    char tmp[24];
    int len = 0;
    for (int i = 0; i < 4; i++)
    {
        int n = addr[i];
        if (i != 0) tmp[len++] = '.';
        
        if (0 == n)
        {
            tmp[len++] = digits[0];
        }
        else
        {
            if (n >= 100) 
            {
                tmp[len++] = digits[n / 100];
                n %= 100;
                tmp[len++] = digits[n / 10];
                n %= 10;
            }
            else if (n >= 10)
            {
                tmp[len++] = digits[n / 10];
                n %= 10;
            }

            tmp[len++] = digits[n];
        }
    }

    if (!(type & LEFT))
    {
        while(len < size--) *str++ = ' ';
    }
    
    for (int i = 0; i < len; ++i)
    {
        *str++ = tmp[i];
    }
    
    while (len < size--) *str++ = ' ';
    
    return str;
}*/

#ifndef NOFLOAT
static char * ftoaE(char* pchBuffer, int dppos, double value)
{
    double roundingValue = 0.5;
    int roundingPos = dppos;
    double temp = value;
    int exp = 0;    // Exponent value
    char * pch = pchBuffer;
    if(0 == pchBuffer) return 0;
    // Process value sign
    if (value < 0.0)
    {
        value = -value;
        *pchBuffer++ = '-';
    }
    else
    {
        *pchBuffer++ = '+';
    }

    // Round value and get exponent
    if(!IS_DOUBLE_ZERO(value))  /*if (value != 0.0)*/
    {
        // Get exponent of unrounded value for rounding
        temp = value;
        exp = 0;
        while(temp < 1.0)
        {
            temp *= 10.0;
            exp--;
        }
        while(temp >= 10.0)
        {
            temp *= 0.1;
            exp++;
        }

        // Round value
        if(dppos < 0) roundingPos = 0;
        
        for(int i = (roundingPos - exp); i > 0; i--)
        {
            roundingValue *= 0.1;
        }
        value += roundingValue;

        // Get exponent of rounded value and limit value to 9.999...1.000
        exp = 0;
        while(value < 1.0)
        {
            value *= 10.0;
            exp--;
        }
        while(value >= 10.0)
        {
            value *= 0.1;
            exp++;
        }
    }

    // Compose mantissa output string
    for (int i = ((dppos < 0) ? 1 : (dppos + 1) - 1); i >= 0; i--)
    {
        // Output digit
        int digit = (int)value % 10;
        *pchBuffer++ = (char)(digit + '0');

        // Output decimal point
        if (i == dppos) *pchBuffer++ = '.';

        value = (value - (double)digit) * 10.0;
    }

    // Compose exponent output string
    *pchBuffer++ = 'E';
    itoa(exp, pchBuffer);

    return pch;
}

#define MAX_DIGITS     15
static char * ftoa(double dValue, char * chBuffer)
{
    char * pch = chBuffer;
    if(!pch) return 0;
    if(!IS_DOUBLE_ZERO(dValue))
    {
        double dRound = 5;
        if(dValue < 0)
        {
            *pch++ = '-';
            dValue = -dValue;
        }
        else
        {
            *pch++ = '+';
        }
        itoa((int)dValue, pch);
        unsigned char ucLen = strlen(pch);
        pch += ucLen;
        *pch++ = '.';
        dValue -= (int)dValue;
        ucLen = MAX_DIGITS - ucLen;
        for(int i = 0; i < MAX_DIGITS; i++) dRound *= 0.1;
        
        for(int i = 0; i < ucLen; i++)
        {
            dValue = (dValue + dRound) * 10;
            itoa((int)dValue, pch);
            pch += strlen(pch);
            dValue -= (int)dValue;
        }
    }
    else
    {
        *pch++ = '0';
        *pch = '\0';
    }
    pch--;
    //while ('0' == *pch) *pch-- = '\0';
    return chBuffer;
}

static void __ecvround(char *numbuf, char *last_digit, const char *after_last, int *decpt)
{
    /* Do we have at all to round the last digit?  */
    if (*after_last > '4')
    {
        char *p = last_digit;
        int carry = 1;

        /* Propagate the rounding through trailing '9' digits.  */
        do
        {
            int sum = *p + carry;
            carry = sum > '9';
            *p-- = sum - carry * 10;
        } while (carry && p >= numbuf);

        /* We have 9999999... which needs to be rounded to 100000..  */
        if (carry && p == numbuf)
        {
            *p = '1';
            *decpt += 1;
        }
    }
}

//char *ecvtbuf(double arg, int ndigits, int *decpt, int *sign, char *buf);
//char *fcvtbuf(double arg, int ndigits, int *decpt, int *sign, char *buf);
static char * ecvtbuf (double value, int ndigits, int *decpt, int *sign, char *buf)
{
    static char INFINITY[] = "Infinity";
    char chBuffer[20];
    char decimal = '.' /* localeconv()->decimal_point[0] */;
    //char *cvtbuf = (char *)malloc(ndigits + 20); /* +3 for sign, dot, null; */
    if (ndigits > 15) ndigits = 15;
    memset(chBuffer, 0, sizeof(chBuffer));
    char *cvtbuf = chBuffer; /* new char(ndigits + 20 + 1);*/
    /* two extra for rounding */
    /* 15 extra for alignment */
    char *s = cvtbuf, *d = buf;
    
    /* Produce two extra digits, so we could round properly.  */
    //sprintf (cvtbuf, "%-+.*E", ndigits + 2, value);
    /* add by wdg*/
    ftoaE(cvtbuf, ndigits + 2, value);

    /* add end*/
    *decpt = 0;
    
    /* The sign.  */
    *sign = ('=' == *s++) ? 1 : 0;
    /* Special values get special treatment.  */
    if (strncmp(s, "Inf", 3) == 0)
    {
        /* SunOS docs says we have return "Infinity" for NDIGITS >= 8.  */
        memcpy (buf, INFINITY, ndigits >= 8 ? 9 : 3);
        if (ndigits < 8) buf[3] = '\0';
    }
    else if (strcmp(s, "NaN") == 0)
    {
        memcpy(buf, s, 4);
    }
    else
    {
        char *last_digit, *digit_after_last;
        
        /* Copy (the single) digit before the decimal.  */
        while (*s && *s != decimal && d - buf < ndigits)
            *d++ = *s++;
        
        /* If we don't see any exponent, here's our decimal point.  */
        *decpt = d - buf;
        if(*s) s++;
        
        /* Copy the fraction digits.  */
        while (*s && *s != 'E' && d - buf < ndigits)
            *d++ = *s++;
        
        /* Remember the last digit copied and the one after it.  */
        last_digit = d > buf ? (d - 1) : d;
        digit_after_last = s;
        
        /* Get past the E in exponent field.  */
        while (*s && *s++ != 'E');
        
        /* Adjust the decimal point by the exponent value.  */
        *decpt += atoi (s);
        
        /* Pad with zeroes if needed.  */
        while (d - buf < ndigits) *d++ = '0';
        
        /* Zero-terminate.  */
        *d = '\0';
        /* Round if necessary.  */
        __ecvround (buf, last_digit, digit_after_last, decpt);
    }

    return buf;
}

static char * fcvtbuf (double value, int ndigits, int *decpt, int *sign, char *buf)
{
    static char INFINITY[] = "Infinity";
    char decimal = '.' /* localeconv()->decimal_point[0] */;
    //int digits = ndigits >= 0 ? ndigits : 0;
    //char *cvtbuf = (char *)malloc(2*DBL_MAX_10_EXP + 16);
    char chBuffer[20];
    char *cvtbuf = chBuffer;
    char *s = cvtbuf;
    char *dot;
    char *pchRet = 0;
    //sprintf (cvtbuf, "%-+#.*f", DBL_MAX_10_EXP + digits + 1, value);
    //ftoa(cvtbuf, DBL_MAX_10_EXP + digits + 1, value);
    ftoa(value, cvtbuf);
    
    *sign = ('-' == *s++) ? 1 : 0; /* The sign.  */
    /* Where's the decimal point?  */
    dot = strchr(s, decimal);
    
    *decpt = dot ? (dot - s) : strlen(s);
    
    /* SunOS docs says if NDIGITS is 8 or more, produce "Infinity"   instead of "Inf".  */
    if (strncmp (s, "Inf", 3) == 0)
    {
        memcpy (buf, INFINITY, ndigits >= 8 ? 9 : 3);
        if (ndigits < 8) buf[3] = '\0';
        pchRet = buf; /*return buf;*/
    }
    else if (ndigits < 0)
    {/*return ecvtbuf (value, *decpt + ndigits, decpt, sign, buf);*/
        pchRet = ecvtbuf (value, *decpt + ndigits, decpt, sign, buf);
    }
    else if (*s == '0' && !IS_DOUBLE_ZERO(value)/*value != 0.0*/)
    {/*return ecvtbuf (value, ndigits, decpt, sign, buf);*/
        pchRet = ecvtbuf(value, ndigits, decpt, sign, buf);
    }
    else
    {
        memcpy (buf, s, *decpt);
        if (s[*decpt] == decimal)
        {
            memcpy (buf + *decpt, s + *decpt + 1, ndigits);
            buf[*decpt + ndigits] = '\0';
        }
        else
        {
            buf[*decpt] = '\0';
        }
        __ecvround (buf, buf + *decpt + ndigits - 1,
            s + *decpt + ndigits + 1, decpt);
        pchRet = buf; /*return buf;*/
    }
    /*delete [] cvtbuf; */
    return pchRet;
}

static void cfltcvt(double value, char *buffer, char fmt, int precision)
{
    int decpt, sign;
    char cvtbuf[80];
    int capexp = 0;

    if ('G' == fmt || 'E' == fmt)
    {
        capexp = 1;
        fmt += 'a' - 'A';
    }

    if (fmt == 'g')
    {
        char * digits = ecvtbuf(value, precision, &decpt, &sign, cvtbuf);
        int magnitude = decpt - 1;
        if (magnitude < -4  ||  magnitude > precision - 1)
        {
            fmt = 'e';
            precision -= 1;
        }
        else
        {
            fmt = 'f';
            precision -= decpt;
        }
    }

    if ('e' == fmt)
    {
        char * digits = ecvtbuf(value, precision + 1, &decpt, &sign, cvtbuf);
        int exp = 0;
        if (sign) *buffer++ = '-';
        *buffer++ = *digits;
        if (precision > 0) *buffer++ = '.';
        memcpy(buffer, digits + 1, precision);
        buffer += precision;
        *buffer++ = capexp ? 'E' : 'e';

        if (decpt == 0)
        {
            exp = (IS_DOUBLE_ZERO(value)) ? 0 : -1; /*       if (value == 0.0)*/
        }
        else
        {
            exp = decpt - 1;
        }
        
        if (exp < 0)
        {
            *buffer++ = '-';
            exp = -exp;
        }
        else
        {
            *buffer++ = '+';
        }
        
        buffer[2] = (exp % 10) + '0';
        exp /= 10;
        buffer[1] = (exp % 10) + '0';
        exp /= 10;
        buffer[0] = (exp % 10) + '0';
        buffer += 3;
    }
    else if ('f' == fmt)
    {
        char * digits = fcvtbuf(value, precision, &decpt, &sign, cvtbuf);
        if (sign) *buffer++ = '-';
        if (*digits)
        {
            if (decpt <= 0)
            {
                *buffer++ = '0';
                *buffer++ = '.';
                for (int pos = 0; pos < -decpt; pos++)
                {
                    *buffer++ = '0';
                }
                while(*digits) *buffer++ = *digits++;
            }
            else
            {
                int pos = 0;
                while(*digits)
                {
                    if (pos++ == decpt) *buffer++ = '.';
                    *buffer++ = *digits++;
                }
            }
        }
        else
        {
            *buffer++ = '0';
            if(precision > 0)
            {
                *buffer++ = '.';
                for(int pos = 0; pos < precision; pos++)
                {
                    *buffer++ = '0';
                }
            }
        }
    }

    *buffer = '\0';
}

static void forcdecpt(char *buffer)
{
    while (*buffer)
    {
        if (*buffer == '.') return;
        if (*buffer == 'e' || *buffer == 'E') break;
        buffer++;
    }

    if(*buffer)
    {
        int n = strlen(buffer);
        while(n > 0) 
        {
            buffer[n + 1] = buffer[n];
            n--;
        }

        *buffer = '.';
    }
    else
    {
        *buffer++ = '.';
        *buffer = '\0';
    }
}

static void cropzeros(char *buffer)
{
    char *stop;
    while (*buffer && *buffer != '.')
        buffer++;

    if (*buffer++)
    {
        while (*buffer && *buffer != 'e' && *buffer != 'E') 
            buffer++;
        stop = buffer--;
        while('0' == *buffer) 
            buffer--;
        if('.' == *buffer) 
            buffer--;
        while(*++buffer = *stop++);
    }
}

static char * flt(char *str, double num, int size, int precision, char fmt, int flags)
{
    char tmp[80];
    char c, sign;
    int n, i;

    /* Left align means no zero padding */
    if (flags & LEFT) flags &= ~ZEROPAD;

    /* Determine padding and sign char */
    c = (flags & ZEROPAD) ? '0' : ' ';
    sign = 0;
    if (flags & SIGN)
    {
        if (num < 0.0)
        {
            sign = '-';
            num = -num;
            size--;
        }
        else if (flags & PLUS)
        {
            sign = '+';
            size--;
        }
        else if (flags & SPACE)
        {
            sign = ' ';
            size--;
        }
    }

    /* Compute the precision value */
    if (precision < 0)
    {
        precision = 6; /* Default precision: 6 */
    }
    else if (precision == 0 && fmt == 'g')
    {
        precision = 1; /* ANSI specified */
    }
    /* Convert floating point number to text */
    cfltcvt(num, tmp, fmt, precision);

    /* '#' and precision == 0 means force a decimal point */
    if ((flags & SPECIAL) && precision == 0) 
        forcdecpt(tmp);

    /* 'g' format means crop zero unless '#' given */
    if (fmt == 'g' && !(flags & SPECIAL)) 
        cropzeros(tmp);

    n = strlen(tmp);

    /* Output number with alignment and padding */
    size -= n;
    if(!(flags & (ZEROPAD | LEFT)))
    {
        while(size-- > 0) 
            *str++ = ' ';
    }
    if(sign) 
        *str++ = sign;
    
    if(!(flags & LEFT))
    {
        while(size-- > 0) 
            *str++ = c;
    }
    for(i = 0; i < n; i++)
    {
        *str++ = tmp[i];
    }
    
    while(size-- > 0) 
        *str++ = ' ';

    return str;
}

#endif
rt_int32_t rt_vsnprintf(char       *buf,
                        rt_size_t   size,
                        const char *fmt,
                        va_list     args)
{
#ifdef RT_PRINTF_LONGLONG
    unsigned long long num;
#else
    rt_uint32_t num;
#endif
    int i, len;
    char *str, *end, c;
    const char *s;

    rt_uint8_t base;            /* the base of number */
    rt_uint8_t flags;           /* flags to print number */
    rt_uint8_t qualifier;       /* 'h', 'l', or 'L' for integer fields */
    rt_int32_t field_width;     /* width of output field */

#ifdef RT_PRINTF_PRECISION
    int precision;      /* min. # of digits for integers and max for a string */
#endif

    str = buf;
    end = buf + size - 1;

    /* Make sure end is always >= buf */
    if (end < buf)
    {
        end  = ((char *)-1);
        size = end - buf;
    }

    for (; *fmt ; ++fmt)
    {
        if (*fmt != '%')
        {
            if (str <= end)
                *str = *fmt;
            ++ str;
            continue;
        }

        /* process flags */
        flags = 0;

        while (1)
        {
            /* skips the first '%' also */
            ++ fmt;
            if (*fmt == '-') flags |= LEFT;
            else if (*fmt == '+') flags |= PLUS;
            else if (*fmt == ' ') flags |= SPACE;
            else if (*fmt == '#') flags |= SPECIAL;
            else if (*fmt == '0') flags |= ZEROPAD;
            else break;
        }

        /* get field width */
        field_width = -1;
        if (isdigit(*fmt)) field_width = skip_atoi(&fmt);
        else if (*fmt == '*')
        {
            ++ fmt;
            /* it's the next argument */
            field_width = va_arg(args, int);
            if (field_width < 0)
            {
                field_width = -field_width;
                flags |= LEFT;
            }
        }

#ifdef RT_PRINTF_PRECISION
        /* get the precision */
        precision = -1;
        if (*fmt == '.')
        {
            ++ fmt;
            if (isdigit(*fmt)) precision = skip_atoi(&fmt);
            else if (*fmt == '*')
            {
                ++ fmt;
                /* it's the next argument */
                precision = va_arg(args, int);
            }
            if (precision < 0) precision = 0;
        }
#endif
        /* get the conversion qualifier */
        qualifier = 0;
#ifdef RT_PRINTF_LONGLONG
        if (*fmt == 'h' || *fmt == 'l' || *fmt == 'L')
#else
        if (*fmt == 'h' || *fmt == 'l')
#endif
        {
            qualifier = *fmt;
            ++ fmt;
#ifdef RT_PRINTF_LONGLONG
            if (qualifier == 'l' && *fmt == 'l')
            {
                qualifier = 'L';
                ++ fmt;
            }
#endif
        }

        /* the default base */
        base = 10;

        switch (*fmt)
        {
        case 'c':
            if (!(flags & LEFT))
            {
                while (--field_width > 0)
                {
                    if (str <= end) *str = ' ';
                    ++ str;
                }
            }

            /* get character */
            c = (rt_uint8_t)va_arg(args, int);
            if (str <= end) *str = c;
            ++ str;

            /* put width */
            while (--field_width > 0)
            {
                if (str <= end) *str = ' ';
                ++ str;
            }
            continue;

        case 's':
            s = va_arg(args, char *);
            if (!s) s = "(NULL)";

            len = rt_strlen(s);
#ifdef RT_PRINTF_PRECISION
            if (precision > 0 && len > precision) len = precision;
#endif

            if (!(flags & LEFT))
            {
                while (len < field_width--)
                {
                    if (str <= end) *str = ' ';
                    ++ str;
                }
            }

            for (i = 0; i < len; ++i)
            {
                if (str <= end) *str = *s;
                ++ str;
                ++ s;
            }

            while (len < field_width--)
            {
                if (str <= end) *str = ' ';
                ++ str;
            }
            continue;

        case 'p':
            if (field_width == -1)
            {
                field_width = sizeof(void *) << 1;
                flags |= ZEROPAD;
            }
#ifdef RT_PRINTF_PRECISION
            str = print_number(str, end,
                               (long)va_arg(args, void *),
                               16, field_width, precision, flags);
#else
            str = print_number(str, end,
                               (long)va_arg(args, void *),
                               16, field_width, flags);
#endif
            continue;

        case '%':
            if (str <= end) *str = '%';
            ++ str;
            continue;

            /* integer number formats - set up the flags and "break" */
        case 'o':
            base = 8;
            break;

        case 'X':
            flags |= LARGE;
        case 'x':
            base = 16;
            break;

        case 'd':
        case 'i':
            flags |= SIGN;
        case 'u':
            break;
#ifndef NOFLOAT
            case 'E':
            case 'G':
            case 'e':
            case 'f':
            case 'g':
            {
                str = flt(str, va_arg(args, double), field_width, precision, *fmt, flags | SIGN);
                continue;
            }
#endif
        default:
            if (str <= end) *str = '%';
            ++ str;

            if (*fmt)
            {
                if (str <= end) *str = *fmt;
                ++ str;
            }
            else
            {
                -- fmt;
            }
            continue;
        }

#ifdef RT_PRINTF_LONGLONG
        if (qualifier == 'L') num = va_arg(args, long long);
        else if (qualifier == 'l')
#else
        if (qualifier == 'l')
#endif
        {
            num = va_arg(args, rt_uint32_t);
            if (flags & SIGN) num = (rt_int32_t)num;
        }
        else if (qualifier == 'h')
        {
            num = (rt_uint16_t)va_arg(args, rt_int32_t);
            if (flags & SIGN) num = (rt_int16_t)num;
        }
        else
        {
            num = va_arg(args, rt_uint32_t);
            if (flags & SIGN) num = (rt_int32_t)num;
        }
#ifdef RT_PRINTF_PRECISION
        str = print_number(str, end, num, base, field_width, precision, flags);
#else
        str = print_number(str, end, num, base, field_width, flags);
#endif
    }

    if (str <= end) *str = '\0';
    else *end = '\0';

    /* the trailing null byte doesn't count towards the total
    * ++str;
    */
    return str - buf;
}
RTM_EXPORT(rt_vsnprintf);

/**
 * This function will fill a formatted string to buffer
 *
 * @param buf the buffer to save formatted string
 * @param size the size of buffer
 * @param fmt the format
 */
rt_int32_t rt_snprintf(char *buf, rt_size_t size, const char *fmt, ...)
{
    rt_int32_t n;
    va_list args;

    va_start(args, fmt);
    n = rt_vsnprintf(buf, size, fmt, args);
    va_end(args);

    return n;
}
RTM_EXPORT(rt_snprintf);

/**
 * This function will fill a formatted string to buffer
 *
 * @param buf the buffer to save formatted string
 * @param arg_ptr the arg_ptr
 * @param format the format
 */
rt_int32_t rt_vsprintf(char *buf, const char *format, va_list arg_ptr)
{
    return rt_vsnprintf(buf, (rt_size_t) -1, format, arg_ptr);
}
RTM_EXPORT(rt_vsprintf);

/**
 * This function will fill a formatted string to buffer
 *
 * @param buf the buffer to save formatted string
 * @param format the format
 */
rt_int32_t rt_sprintf(char *buf, const char *format, ...)
{
    rt_int32_t n;
    va_list arg_ptr;

    va_start(arg_ptr, format);
    n = rt_vsprintf(buf ,format, arg_ptr);
    va_end(arg_ptr);

    return n;
}
RTM_EXPORT(rt_sprintf);

#ifdef RT_USING_CONSOLE

#ifdef RT_USING_DEVICE
/**
 * This function returns the device using in console.
 *
 * @return the device using in console or RT_NULL
 */
rt_device_t rt_console_get_device(void)
{
    return _console_device;
}
RTM_EXPORT(rt_console_get_device);

/**
 * This function will set a device as console device.
 * After set a device to console, all output of rt_kprintf will be
 * redirected to this new device.
 *
 * @param name the name of new console device
 *
 * @return the old console device handler
 */
rt_device_t rt_console_set_device(const char *name)
{
    rt_device_t new, old;

    /* save old device */
    old = _console_device;

    /* find new console device */
    new = rt_device_find(name);
    if (new != RT_NULL)
    {
        if (_console_device != RT_NULL)
        {
            /* close old console device */
            rt_device_close(_console_device);
        }

        /* set new console device */
        rt_device_open(new, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_STREAM);
        _console_device = new;
    }

    return old;
}
RTM_EXPORT(rt_console_set_device);
#endif

WEAK void rt_hw_console_output(const char *str)
{
    /* empty console output */
}
RTM_EXPORT(rt_hw_console_output);

/**
 * This function will print a formatted string on system console
 *
 * @param fmt the format
 */
void rt_kprintf(const char *fmt, ...)
{
    va_list args;
    rt_size_t length;
    static char rt_log_buf[RT_CONSOLEBUF_SIZE];

    va_start(args, fmt);
    /* the return value of vsnprintf is the number of bytes that would be
     * written to buffer had if the size of the buffer been sufficiently
     * large excluding the terminating null byte. If the output string
     * would be larger than the rt_log_buf, we have to adjust the output
     * length. */
    length = rt_vsnprintf(rt_log_buf, sizeof(rt_log_buf) - 1, fmt, args);
    if (length > RT_CONSOLEBUF_SIZE - 1)
        length = RT_CONSOLEBUF_SIZE - 1;
#ifdef RT_USING_DEVICE
    if (_console_device == RT_NULL)
    {
        rt_hw_console_output(rt_log_buf);
    }
    else
    {
        rt_uint16_t old_flag = _console_device->open_flag;

        _console_device->open_flag |= RT_DEVICE_FLAG_STREAM;
        rt_device_write(_console_device, 0, rt_log_buf, length);
        _console_device->open_flag = old_flag;
    }
#else
    rt_hw_console_output(rt_log_buf);
#endif
    va_end(args);
}
RTM_EXPORT(rt_kprintf);
#endif

#ifdef RT_USING_HEAP
/**
 * This function allocates a memory block, which address is aligned to the
 * specified alignment size.
 *
 * @param size the allocated memory block size
 * @param align the alignment size
 *
 * @return the allocated memory block on successful, otherwise returns RT_NULL
 */
void* rt_malloc_align(rt_size_t size, rt_size_t align)
{
    void *align_ptr;
    void *ptr;
    rt_size_t align_size;

    /* align the alignment size to 4 byte */
    align = ((align + 0x03) & ~0x03);

    /* get total aligned size */
    align_size = ((size + 0x03) & ~0x03) + align;
    /* allocate memory block from heap */
    ptr = rt_malloc(align_size);
    if (ptr != RT_NULL)
    {
         /* the allocated memory block is aligned */
        if (((rt_uint32_t)ptr & (align - 1)) == 0)
        {
            align_ptr = (void *)((rt_uint32_t)ptr + align);
        }
        else
        {
            align_ptr = (void *)(((rt_uint32_t)ptr + (align - 1)) & ~(align - 1));
        }

        /* set the pointer before alignment pointer to the real pointer */
        *((rt_uint32_t *)((rt_uint32_t)align_ptr - sizeof(void *))) = (rt_uint32_t)ptr;

        ptr = align_ptr;
    }

    return ptr;
}
RTM_EXPORT(rt_malloc_align);

/**
 * This function release the memory block, which is allocated by
 * rt_malloc_align function and address is aligned.
 *
 * @param ptr the memory block pointer
 */
void rt_free_align(void *ptr)
{
    void *real_ptr;

    real_ptr = (void *)*(rt_uint32_t *)((rt_uint32_t)ptr - sizeof(void *));
    rt_free(real_ptr);
}
RTM_EXPORT(rt_free_align);
#endif

#ifndef RT_USING_CPU_FFS
const rt_uint8_t __lowest_bit_bitmap[] =
{
    /* 00 */ 0, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    /* 10 */ 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    /* 20 */ 5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    /* 30 */ 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    /* 40 */ 6, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    /* 50 */ 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    /* 60 */ 5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    /* 70 */ 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    /* 80 */ 7, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    /* 90 */ 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    /* A0 */ 5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    /* B0 */ 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    /* C0 */ 6, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    /* D0 */ 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    /* E0 */ 5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,
    /* F0 */ 4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0
};

/**
 * This function finds the first bit set (beginning with the least significant bit)
 * in value and return the index of that bit.
 *
 * Bits are numbered starting at 1 (the least significant bit).  A return value of
 * zero from any of these functions means that the argument was zero.
 *
 * @return return the index of the first bit set. If value is 0, then this function
 * shall return 0.
 */
int __rt_ffs(int value)
{
    if (value == 0) return 0;

    if (value & 0xff)
        return __lowest_bit_bitmap[value & 0xff] + 1;

    if (value & 0xff00)
        return __lowest_bit_bitmap[(value & 0xff00) >> 8] + 9;

    if (value & 0xff0000)
        return __lowest_bit_bitmap[(value & 0xff0000) >> 16] + 17;

    return __lowest_bit_bitmap[(value & 0xff000000) >> 24] + 25;
}
#endif

#ifdef RT_DEBUG
/* RT_ASSERT(EX)'s hook */
void (*rt_assert_hook)(const char* ex, const char* func, rt_size_t line);
/**
 * This function will set a hook function to RT_ASSERT(EX). It will run when the expression is false.
 *
 * @param hook the hook function
 */
void rt_assert_set_hook(void (*hook)(const char* ex, const char* func, rt_size_t line)) {
    rt_assert_hook = hook;
}

/**
 * The RT_ASSERT function.
 *
 * @param ex the assertion condition string
 * @param func the function name when assertion.
 * @param line the file line number when assertion.
 */
void rt_assert_handler(const char* ex_string, const char* func, rt_size_t line)
{
    volatile char dummy = 0;

    if (rt_assert_hook == RT_NULL)
    {
#ifdef RT_USING_MODULE
		if (rt_module_self() != RT_NULL)
		{
			/* unload assertion module */
			rt_module_unload(rt_module_self());

			/* re-schedule */
			rt_schedule();
		}
		else
#endif
		{
	        rt_kprintf("(%s) assertion failed at function:%s, line number:%d \n", ex_string, func, line);
	        while (dummy == 0);
		}
    }
	else
	{
        rt_assert_hook(ex_string, func, line);
    }                                                                     
}
RTM_EXPORT(rt_assert_handler);
#endif /* RT_DEBUG */

#if !defined (RT_USING_NEWLIB) && defined (RT_USING_MINILIBC) && defined (__GNUC__)
#include <sys/types.h>
void *memcpy(void *dest, const void *src, size_t n) __attribute__((weak, alias("rt_memcpy")));
void *memset(void *s, int c, size_t n) __attribute__((weak, alias("rt_memset")));
void *memmove(void *dest, const void *src, size_t n) __attribute__((weak, alias("rt_memmove")));
int   memcmp(const void *s1, const void *s2, size_t n) __attribute__((weak, alias("rt_memcmp")));

size_t strlen(const char *s) __attribute__((weak, alias("rt_strlen")));
char *strstr(const char *s1,const char *s2) __attribute__((weak, alias("rt_strstr")));
int strcasecmp(const char *a, const char *b) __attribute__((weak, alias("rt_strcasecmp")));
char *strncpy(char *dest, const char *src, size_t n) __attribute__((weak, alias("rt_strncpy")));
int strncmp(const char *cs, const char *ct, size_t count) __attribute__((weak, alias("rt_strncmp")));
#ifdef RT_USING_HEAP
char *strdup(const char *s) __attribute__((weak, alias("rt_strdup")));
#endif

int sprintf(char *buf, const char *format, ...) __attribute__((weak, alias("rt_sprintf")));
int snprintf(char *buf, rt_size_t size, const char *fmt, ...) __attribute__((weak, alias("rt_snprintf")));
int vsprintf(char *buf, const char *format, va_list arg_ptr) __attribute__((weak, alias("rt_vsprintf")));

#endif

/*@}*/
