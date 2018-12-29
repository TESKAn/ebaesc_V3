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
* @brief  Reciprocal functions with 32-bit accumulator output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_RCP_A32_ASM_H_
#define _MLIB_RCP_A32_ASM_H_

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
/* Reciprocal */
#define MLIB_Rcp1_A32s_Asmi(f16Denom) MLIB_Rcp1_A32s_FAsmi(f16Denom)
#define MLIB_Rcp_A32s_Asmi(f16Denom) MLIB_Rcp_A32s_FAsmi(f16Denom)

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
* @brief  16-bit input 32-output 16-bit precision quadrant reciprocal function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Denom
*                         - Denominator in [-1;1] in frac16_t
*                       
*
* @return This function returns
*     - acc32_t value [-65536.0;65536.0 - (2^-15)]
*		
* @remarks 	This function calculates the multiplicative inverse value of
* 			the fractional input:
* 			result = FRAC16(1) / f16Denom. The function calculates
* 			the result with 16-bit division precision.
* 			The function normalizes the inputs to get higher precision of
* 			division.
* 			If the denominator is 0, the output is 0x7FFF FFFF.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline acc32_t MLIB_Rcp1_A32s_FAsmi(register frac16_t f16Denom)
{
	register frac32_t f32Result;
	register int16_t w16ClbDenom;
		
	asm(.optimize_iasm on);
		
	asm(clb f16Denom,w16ClbDenom);		/* w16ClbDenom = number of leading bits of f16Denom */
	asm(move.w #0x4000,A);				/* A = FRAC16(0.5) */
	asm(asll.l w16ClbDenom,f16Denom);	/* normalization of f16Denom to 0.5 to 1.0 */
	asm(sub.w #15,w16ClbDenom);			/* Add 1 because the numerator is 0.5 (not 1) */ 
	
	asm(tst a);							/* Clears the C flag */
	asm(rep 8);							/* Repeat 8 times */
	asm(div f16Denom,A);				/* A = A / f16Denom */
	asm(rep 8);							/* Repeat 8 times */
	asm(div f16Denom,A);				/* A = A / f16Denom */
		
	asm(move.w A0,A);					/* f32Result = f32Result << 16 */
	asm(tfr A,f32Result);				/* Copy of the result */			

	asm(neg	A);							/* Negates the result */
	asm(tst.w f16Denom);				/* Checks the sign of the input */
	asm(tlt	A,f32Result);				/* A = -A if different signs */
	asm(tfr f32Result,A);				/* Copy of the result */

	asm(asll.l w16ClbDenom,f32Result);	/* f32Result = f32Result << w16ClbDenom (arithmetically) */

	asm(bfchg #0x8000,A1);				/* changes the MSB of the result */		

	asm(cmp.w #15,w16ClbDenom);			/* Leading bits comparison */

	asm(tgt A,f32Result);				/* In case of result's overflow, uses the maximum output */
		
	asm(sat f32Result);					/* Saturation */
		
	asm(.optimize_iasm off);
		
	return f32Result;
}

/***************************************************************************//*!
*
* @brief  16-bit input 32-output 32-bit precision single quadrant reciprocal function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Denom
*                         - Denominator in [-1;1] in frac16_t
*                       
*
* @return This function returns
*     - acc32_t value [-65536;65536.0 - (2^-15)]
*		
* @remarks 	This function calculates the multiplicative inverse value of
* 			the fractional input:
* 			result = FRAC16(1) / f16Denom. The function calculates
* 			the result with 32-bit division precision.
* 			The function normalizes the inputs to get higher precision of
* 			division.
* 			If the denominator is 0, the output is 0x7FFF FFFF.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline acc32_t MLIB_Rcp_A32s_FAsmi(register frac16_t f16Denom)
{
	register frac32_t f32Result;
	register int16_t w16ClbDenom;
		
	asm(.optimize_iasm on);
		
	asm(clb f16Denom,w16ClbDenom);		/* w16ClbDenom = number of leading bits of f16Denom */
	asm(move.w #0x4000,A);				/* A = FRAC16(0.5) */
	asm(asll.l w16ClbDenom,f16Denom);	/* normalization of f16Denom to 0.5 to 1.0 */
	asm(sub.w #15,w16ClbDenom);			/* Sub 15 because the numerator is 1 << 15 */ 
	
	asm(tst a);							/* Clears the C flag */
	asm(rep 8);							/* Repeat 8 times */
	asm(div f16Denom,A);				/* A = A / f16Denom */
	asm(rep 8);							/* Repeat 8 times */
	asm(div f16Denom,A);				/* A = A / f16Denom */

	asm(move.w A0,B);					/* Upper 16 bits of the result */ 

	asm(rep 8);							/* Repeat 8 times */
	asm(div f16Denom,A);				/* A = A / f16Denom */
	asm(rep 8);							/* Repeat 8 times */
	asm(div f16Denom,A);				/* A = A / f16Denom */

	asm(move.w A0,B0);					/* Lower 16 bits of the result */
		
	asm(tfr B,f32Result);				/* Copy of the result */

	asm(neg	B);							/* Negates the result */
	asm(tst.w f16Denom);				/* Checks the sign of the input */
	asm(tlt	B,f32Result);				/* B = -B if different signs */
	asm(tfr f32Result,B);				/* Copy of the result */
		
	asm(asll.l w16ClbDenom,f32Result);	/* f32Result = f32Result << w16ClbDenom (arithmetically) */
	
	asm(bfchg #0x8000,B1);				/* changes the MSB of the result */		

	asm(cmp.w #15,w16ClbDenom);			/* Leading bits comparison */

	asm(tgt B,f32Result);				/* In case of result's overflow, uses the maximum output */

	asm(sat f32Result);					/* Saturation */
		
	asm(.optimize_iasm off);
		
	return f32Result;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_RCP_A32_ASM_H_ */
