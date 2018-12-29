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
* @brief  Subtraction of two products functions with 32-bit rounded fractional 
* 		  output in assembler for V3
* 
*******************************************************************************/
#ifndef _MLIB_MSU4RND_F32_ASMI_H_
#define _MLIB_MSU4RND_F32_ASMI_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib_types.h"
#include "MLIB_Mul_F64_Asm.h"

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

/******************************************************************************
* Inline functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  	32-bit inputs 32-output vector multiply and subtract function
* 			with result rounding
*
* @param  	ptr			
* 
* @param  	in    		frac32_t f32MinMult1
*                         - Argument in [-1;1] in frac32_t
*                       frac32_t f32MinMult2
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32SubMult1
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32SubMult2
*                         - Argument in [-1;1] in frac32_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the vector multiply and subtract of input values. 
* 			The input values as well as output value is considered as 32-bit fractional values. 
* 			The output saturation is not implemented in this function. 
* 			The output of the function is defined by the following equation:
* 			f32MinMult1 * f32MinMult2 - f32SubMult1 * f32SubMult2
*
*			SATURATION must be turned off!
*
****************************************************************************/
extern inline frac32_t MLIB_Msu4Rnd_F32_FAsmi(register frac32_t f32MinMult1, register frac32_t f32MinMult2, register frac32_t f32SubMult1, register frac32_t f32SubMult2)
{
	register frac32_t f32Val1, f32Val2;
	
	asm(.optimize_iasm on);

	asm(mpy64 f32MinMult1,f32MinMult2,f32Val1);	/* A:Y = f32MinMult1 * f32MinMult2 */
	
	asm(tfr	Y,f32Val2);							/* f32Val2 = Y */
	
	asm(mpy64 f32SubMult1,f32SubMult2,B);		/* B:Y = - f32SubMult1 * f32SubMult2 */
	
	asm(sub B,f32Val1);							/* A = A - B */
	
	asm(tfr f32Val2, A);						/* f32Val1 = A */
	
	asm(tfr Y,B);
	
	asm(move.w #0,B2);
	
	asm(move.w #0,A2);
	
	asm(sub B, A);								/* f32MinMult1 = f32MinMult1 - B */
	
	asm(asr16 A);
	
	asm(rnd A);									/* Rounding */
	
	asm(asr16 A);
	
	asm(add	A, f32Val1);						/* f32Val1 = f32Val1 + Rounding Bit */		

	asm(.optimize_iasm off);

	return f32Val1;	
}

/***************************************************************************//*!
*
* @brief  	32-bit inputs 32-output vector multiply and subtract saturated function
* 			with result rounding
*
* @param  	ptr			
* 
* @param  	in    		frac32_t f32MinMult1
*                         - Argument in [-1;1] in frac32_t
*                       frac32_t f32MinMult2
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32SubMult1
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32SubMult2
*                         - Argument in [-1;1] in frac32_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the vector multiply and subtract of input values. 
* 			The input values as well as output value is considered as 32-bit fractional values. 
* 			The function saturates the output. 
* 			The output of the function is defined by the following equation:
* 			f32MinMult1 * f32MinMult2 - f32SubMult1 * f32SubMult2
*
*			SATURATION must be turned off!
*
****************************************************************************/
extern inline frac32_t MLIB_Msu4RndSat_F32_FAsmi(register frac32_t f32MinMult1, register frac32_t f32MinMult2, register frac32_t f32SubMult1, register frac32_t f32SubMult2)
{
	register frac32_t f32Val1, f32Val2;
	
	asm(.optimize_iasm on);

	asm(mpy64 f32MinMult1,f32MinMult2,f32Val1);	/* A:Y = f32MinMult1 * f32MinMult2 */
	
	asm(tfr	Y,f32Val2);							/* f32Val2 = Y */
	
	asm(mpy64 f32SubMult1,f32SubMult2,B);		/* B:Y = - f32SubMult1 * f32SubMult2 */
	
	asm(sub B,f32Val1);							/* A = A - B */
	
	asm(tfr f32Val2, f32SubMult1);				/* f32Val1 = A */
	
	asm(tfr Y,B);
	
	asm(move.w #0,B2);
	
	asm(move.w #0,A2);
	
	asm(sub B, A);								/* f32MinMult1 = f32MinMult1 - B */
	
	asm(asr16 A);
	
	asm(rnd A);									/* Rounding */
	
	asm(asr16 A);
	
	asm(add	f32Val1,A);							/* f32Val1 = f32Val1 + Rounding Bit */		
	
	asm(sat A, f32SubMult1);					/* Saturation */

	asm(.optimize_iasm off);

	return f32SubMult1;	
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_MSU4RND_F32_ASMI_H_ */
