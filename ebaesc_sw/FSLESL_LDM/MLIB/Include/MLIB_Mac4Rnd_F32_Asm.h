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
* @brief  Sum of two products functions with 32-bit rounded fractional output 
* 		  in assembler
* 
*******************************************************************************/
#ifndef _MLIB_MAC4RND_F32_ASM_H_
#define _MLIB_MAC4RND_F32_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib_types.h"
#include "MLIB_Mac4Rnd_F32_Asmi.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define MLIB_Mac4Rnd_F32_Asmi(f32Add1Mult1, f32Add1Mult2, f32Add2Mult1, f32Add2Mult2) \
	MLIB_Mac4Rnd_F32_FAsmi(f32Add1Mult1, f32Add1Mult2, f32Add2Mult1, f32Add2Mult2)
#define MLIB_Mac4RndSat_F32_Asmi(f32Add1Mult1, f32Add1Mult2, f32Add2Mult1, f32Add2Mult2) \
	MLIB_Mac4RndSat_F32_FAsmi(f32Add1Mult1, f32Add1Mult2, f32Add2Mult1, f32Add2Mult2)

/******************************************************************************
* Types
******************************************************************************/

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/

/******************************************************************************
* Inline functions
******************************************************************************/
#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_MAC4RND_F32_ASM_H_ */
