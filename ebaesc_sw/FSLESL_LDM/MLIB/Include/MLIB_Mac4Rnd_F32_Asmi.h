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
* 		  in assembler for V3
* 
*******************************************************************************/
#ifndef _MLIB_MAC4RND_F32_ASMI_H_
#define _MLIB_MAC4RND_F32_ASMI_H_

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
* @brief  	32-bit inputs 32-output vector multiply accumulate function
* 			with result rounding
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Add1Mult1
*                         - Argument in [-1;1] in frac32_t
*                       frac32_t f32Add1Mult2
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Add2Mult1
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Add2Mult2
*                         - Argument in [-1;1] in frac32_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the vector multiply accumulate of input values. 
* 			The input values as well as output value is considered as 32-bit fractional values. 
* 			The output saturation is not implemented in this function. 
* 			The output of the function is defined by the following equation:
* 			f32Add1Mult1 * f32Add1Mult2 + f32Add2Mult1 * f32Add2Mult2
*
*			SATURATION must be turned off!
*
****************************************************************************/
extern inline frac32_t MLIB_Mac4Rnd_F32_FAsmi(register frac32_t f32Add1Mult1, register frac32_t f32Add1Mult2, register frac32_t f32Add2Mult1, register frac32_t f32Add2Mult2)
{
	register frac32_t f32Val1, f32Val2;
	
	asm(.optimize_iasm on);

	asm(mpy64 f32Add1Mult1,f32Add1Mult2,f32Val1);/* f32Val1:Y = f32Add1Mult1 * f32Add1Mult2 */
	
	asm(tfr	Y,f32Add1Mult1);					/* f32Add1Mult1 = Y */

	asm(mpy64 f32Add2Mult1,f32Add2Mult2,B);		/* B:Y = f32Add2Mult1 * f32Add2Mult2 */
	
	asm(add B,f32Val1);							/* B = B + f32Val1 */
	
	asm(add	f32Add1Mult1,Y);					/* Y = Y + f32Add1Mult1 */
	
	asm(nop);
	
	asm(lsrr.l #31,Y);							/* Y = highest bit of result lower 32 bits; for rounding */
	
	asm(nop);
	
	asm(adc	Y,f32Val1);							/* f32Val1 = f32Val1 + Rounding Bit + Carry Bit */								

	asm(.optimize_iasm off);

	return f32Val1;	
}

/***************************************************************************//*!
*
* @brief  	32-bit inputs 32-output vector multiply accumulate saturated function
* 			with result rounding
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Add1Mult1
*                         - Argument in [-1;1] in frac32_t
*                       frac32_t f32Add1Mult2
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Add2Mult1
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Add2Mult2
*                         - Argument in [-1;1] in frac32_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the vector multiply accumulate of input values. 
* 			The input values as well as output value is considered as 32-bit fractional values. 
* 			The function saturates the output. 
* 			The output of the function is defined by the following equation:
* 			f32Add1Mult1 * f32Add1Mult2 + f32Add2Mult1 * f32Add2Mult2
*
*			SATURATION must be turned off!
*
****************************************************************************/
extern inline frac32_t MLIB_Mac4RndSat_F32_FAsmi(register frac32_t f32Add1Mult1, register frac32_t f32Add1Mult2, register frac32_t f32Add2Mult1, register frac32_t f32Add2Mult2)
{
	register frac32_t f32Val1, f32Val2;
	
	asm(.optimize_iasm on);

	asm(mpy64 f32Add1Mult1,f32Add1Mult2,A);		/* A:Y = f32Add1Mult1 * f32Add1Mult2 */
	
	asm(tfr	Y,f32Add1Mult1);					/* f32Add1Mult1 = Y */

	asm(mpy64 f32Add2Mult1,f32Add2Mult2,B);		/* B:Y = f32Add2Mult1 * f32Add2Mult2 */
	
	asm(add B,A);								/* B = B + A */
	
	asm(add	f32Add1Mult1,Y);					/* Y = Y + f32Add1Mult1 */
	
	asm(nop);
	
	asm(lsrr.l #31,Y);							/* Y = highest bit of result lower 32 bits; for rounding */
	
	asm(nop);
	
	asm(adc	Y,A);								/* A = A + Rounding Bit + Carry Bit */
	
	asm(sat A,f32Val1);							/* Saturation */

	asm(.optimize_iasm off);

	return f32Val1;	
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_MAC4RND_F32_ASMI_H_ */
