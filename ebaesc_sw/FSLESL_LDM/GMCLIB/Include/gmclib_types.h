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
* @brief  Types related to GMCLIB functions
* 
*******************************************************************************/
#ifndef _GMCLIB_TYPES_H_
#define _GMCLIB_TYPES_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 

#include "mlib_types.h"

#define GMCLIB_CONST const 

/* Data type defintion(s) */
typedef struct
{
    frac16_t f16A;
    frac16_t f16B;
    frac16_t f16C;
} GMCLIB_3COOR_T_F16;

typedef struct
{
    frac16_t f16A;
    frac16_t f16B;
} GMCLIB_2COOR_T_F16;

typedef struct
{
    frac16_t f16Alpha;
    frac16_t f16Beta;
} GMCLIB_2COOR_ALBE_T_F16;

typedef struct
{
    frac16_t f16D;
    frac16_t f16Q;
} GMCLIB_2COOR_DQ_T_F16;

typedef struct
{
    frac16_t f16Sin;
    frac16_t f16Cos;
} GMCLIB_2COOR_SINCOS_T_F16;

typedef struct
{
    frac32_t f32Alpha;
    frac32_t f32Beta;
} GMCLIB_2COOR_ALBE_T_F32;

typedef struct
{
    frac32_t f32D;
    frac32_t f32Q;
} GMCLIB_2COOR_DQ_T_F32;

#if defined(__cplusplus) 
} 
#endif 

#endif /* _GMCLIB_TYPES_H_ */
