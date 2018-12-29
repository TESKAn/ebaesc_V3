/*******************************************************************************
*
* Copyright 2007-2015 Freescale Semiconductor, Inc.
*
* This software is owned or controlled by Freescale Semiconductor.
* Use of this software is governed by the Freescale License
* distributed with this Material.
* See the LICENSE file distributed for more details.
* 
*
****************************************************************************//*!
*
* @brief  Basic types for FSLESL
* 
*******************************************************************************/
#ifndef _MLIB_TYPES_H_
#define _MLIB_TYPES_H_

#pragma slld on

#if defined(__cplusplus) 
extern "C" { 
#endif 

#define MLIB_CONST const 

/* Boolean data types */
typedef unsigned short bool_t;
#ifndef FALSE
#define FALSE    ((bool_t)0)                       
#endif

#ifndef TRUE
#define TRUE     ((bool_t)1)                        
#endif

/* Unsigned integer data types */
typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long uint32_t;
typedef unsigned long long uint64_t;

/* Signed integer data types */
typedef signed char int8_t;
typedef signed short int16_t;
typedef signed long int32_t;
typedef signed long long int64_t;

/* Fractional data types */
typedef signed long long frac64_t;
typedef signed long frac32_t;
typedef signed short frac16_t;

/* Accumulator data types */
typedef signed long long acc64_t;
typedef signed long acc32_t;
typedef signed short acc16_t;

/* Fractional conversion macros */
#if !defined(FRAC16)
#define FRAC16(x) ((frac16_t)((x) < 0.999969482421875 ? ((x) >= -1 ? (x)*0x8000 : 0x8000) : 0x7FFF))
#endif //!FRAC16

#if !defined(FRAC32)
#define FRAC32(x) ((frac32_t)((x) < 1 ? ((x) >= -1 ? (x)*0x80000000 : 0x80000000) : 0x7FFFFFFF))
#endif //!FRAC32

#if !defined(FRAC64)
#define FRAC64(x) ((frac64_t)((x) < 1 ? ((x) >= -1 ? (x)*0x8000000000000000 : 0x8000000000000000) : 0x7FFFFFFFFFFFFFFF))
#endif //!FRAC64

/* Accumulator conversion macros */
#if !defined(ACC16)
#define ACC16(x) ((acc16_t)((x) < 255.9921875 ? ((x) >= -256 ? (x)*0x80 : 0x8000) : 0x7FFF))
#endif //!ACC16

#if !defined(ACC32)
#define ACC32(x) ((acc32_t)((x) < 65535.999969482421875 ? ((x) >= -65536 ? (x)*0x8000 : 0x80000000) : 0x7FFFFFFF))
#endif //!ACC32

#if !defined(ACC64)
#define ACC64(x) ((acc64_t)((x) < 4294967295.9999999995343387126923 ? ((x) >= -4294967296 ? (x)*0x80000000 : 0x8000000000000000) : 0x7FFFFFFFFFFFFFFF))
#endif //!ACC64

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_TYPES_H_ */
