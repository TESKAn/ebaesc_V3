/******************************************************************************
* 
* Copyright (c) 2008 Freescale Semiconductor;
* All Rights Reserved                       
*
******************************************************************************* 
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR 
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
* THE POSSIBILITY OF SUCH DAMAGE.
*
***************************************************************************//*!
*
* @file      56800E_types.h
*
* @author    R61928
* 
* @version   1.0.3.0
* 
* @date      Apr-24-2013
* 
* @brief     Basic types for GFLIB on 56800E platform.
*
*******************************************************************************
*
* Basic types for GFLIB on 56800E platform.
*
******************************************************************************/
#ifndef _56800E_TYPES_H_
#define _56800E_TYPES_H_

#ifndef OPTION_CORE_V3
#define OTPION_CORE_V3 0
#endif

#include <intrinsics_56800E.h>

#if OPTION_CORE_V3 == 1
#include <intrinsics_56800Ex.h>
#endif

#if (!defined(__TYPES_H) && !defined(__PE_Types_H))
#define __TYPES_H

/* Generic word types for ITU compatibility */
typedef char           Word8;
typedef unsigned char  UWord8;
typedef short          Word16;
typedef unsigned short UWord16;
typedef long           Word32;
typedef unsigned long  UWord32;

typedef char           Int8;
typedef unsigned char  UInt8;
typedef short          Int16;
typedef unsigned int   UInt16;
typedef long           Int32;
typedef unsigned long  UInt32;

/* Fractional data types for portability */
typedef short          Frac16;
typedef long           Frac32;

#ifndef NULL
#define NULL ((void *)0)  /* or for C++ #define NULL 0 */
#endif

/* Miscellaneous types */
#ifndef COMPILER_HAS_BOOL
typedef int            bool;
#endif

#ifndef COMPILER_HAS_BOOL
#define true  1
#define false 0
#endif

#endif /* !__TYPES_H && !__PE_Types_H */

#if !defined(FRAC16)
#define FRAC16(x) ((Frac16)((x) < 0.999969482421875 ? ((x) >= -1 ? (x)*0x8000 : 0x8000) : 0x7FFF))

#endif //!FRAC16

#if !defined(FRAC32)
#define FRAC32(x) ((Frac32)((x) < 1 ? ((x) >= -1 ? (x)*0x80000000 : 0x80000000) : 0x7FFFFFFF))

#endif //!FRAC32

#endif /* _56800E_TYPES_H_ */
