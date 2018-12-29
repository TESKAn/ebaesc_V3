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
* @brief Arcus tangent function based on the provided x,y coordinates as arguments
* 		 using division and piece-wise polynomial approximation 
* 
*******************************************************************************/
#ifndef _GFLIB_ATANYX_F32_ASM_H_
#define _GFLIB_ATANYX_F32_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib.h"
#include "gflib_types.h"
#include "GFLIB_Atan_F32_Asm.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/

/******************************************************************************
* Types
******************************************************************************/

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/

extern asm frac16_t GFLIB_AtanYX_F16_FAsm(frac16_t f16Y,
							 	   	   	  frac16_t f16X,
							 	   	   	  const GFLIB_ATAN_T_F32 *psParam,
							 	   	   	  bool_t *pbErrFlag);

/******************************************************************************
* Inline functions
******************************************************************************/
#if defined(__cplusplus) 
} 
#endif 

#endif /* _GFLIB_ATANYX_F32_ASM_H_ */
