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
* @brief  Subtraction of 4 values functions with 32-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_SUB4_F32_ASM_H_
#define _MLIB_SUB4_F32_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib_types.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define MLIB_Sub4_F32_Asmi(f32Min, f32Sub1, f32Sub2, f32Sub3) MLIB_Sub4_F32_FAsmi(f32Min, f32Sub1, f32Sub2, f32Sub3)
#define MLIB_Sub4Sat_F32_Asmi(f32Min, f32Sub1, f32Sub2, f32Sub3) MLIB_Sub4Sat_F32_FAsmi(f32Min, f32Sub1, f32Sub2, f32Sub3)

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

/***************************************************************************//*!
*
* @brief  32-bit subtraction function of four arguments
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Min
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Sub1
*                         - Argument in [-1;1] in frac32_t
*                       frac32_t f32Sub2
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Sub3
*                         - Argument in [-1;1] in frac32_t
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the subtraction of four inputs. 
* 			The output of the function is defined by the following equation:
* 						f32Min - f32Sub1 - f32Sub2 - f32Sub3
* 			The function does not saturate the output if the saturation mode is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline frac32_t MLIB_Sub4_F32_FAsmi(register frac32_t f32Min, register frac32_t f32Sub1, register frac32_t f32Sub2, register frac32_t f32Sub3)
{
	asm(.optimize_iasm on);
	
	asm(sub f32Sub1,f32Min);
	
	asm(sub f32Sub2,f32Min);
	
	asm(sub f32Sub3,f32Min);
	
	asm(.optimize_iasm off);
	
	return f32Min;
}

/***************************************************************************//*!
*
* @brief  32-bit subtraction saturated function of four arguments
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Min
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Sub1
*                         - Argument in [-1;1] in frac32_t
*                       frac32_t f32Sub2
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Sub3
*                         - Argument in [-1;1] in frac32_t
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the subtraction of four inputs. 
*  			The output of the function is defined by the following equation:
* 					f32Min - f32Sub1 - f32Sub2 - f32Sub3
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac32_t MLIB_Sub4Sat_F32_FAsmi(register frac32_t f32Min, register frac32_t f32Sub1, register frac32_t f32Sub2, register frac32_t f32Sub3)
{
	asm(.optimize_iasm on);
	
	asm(sub f32Sub1,f32Min);
	
	asm(sub f32Sub2,f32Min);
	
	asm(sub f32Sub3,f32Min);
	
	asm(sat f32Min);
	
	asm(.optimize_iasm off);
	
	return f32Min;		
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_SUB4_F32_ASM_H_ */
