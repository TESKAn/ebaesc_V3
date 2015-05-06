/******************************************************************************
* 
* Copyright (c) 2013 Freescale Semiconductor;
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
* @file      MLIB_DivAsm.h
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Aug-12-2013
* 
* @brief     Division functions in assembler
*
*******************************************************************************
*
* Division functions in assembler.
*
******************************************************************************/
#ifndef _MLIB_DIVASM_H_
#define _MLIB_DIVASM_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "56800E_types.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define MLIB_Div1Q16SSAsmi(f16Num, f16Denom) MLIB_Div1Q16SSFAsmi(f16Num, f16Denom)
#define MLIB_Div4Q16SSAsmi(f16Num, f16Denom) MLIB_Div4Q16SSFAsmi(f16Num, f16Denom)

#define MLIB_Div1Q16LSAsmi(f32Num, f16Denom) MLIB_Div1Q16LSFAsmi(f32Num, f16Denom)
#define MLIB_Div4Q16LSAsmi(f32Num, f16Denom) MLIB_Div4Q16LSFAsmi(f32Num, f16Denom)

#define MLIB_Div1Q32LSAsmi(f32Num, f16Denom) MLIB_Div1Q32LSFAsmi(f32Num, f16Denom)
#define MLIB_Div4Q32LSAsmi(f32Num, f16Denom) MLIB_Div4Q32LSFAsmi(f32Num, f16Denom)

#define MLIB_Rcp161QAsmi(f16Denom) MLIB_Rcp161QFAsmi(f16Denom)
#define MLIB_Rcp164QAsmi(f16Denom) MLIB_Rcp164QFAsmi(f16Denom)

#define MLIB_Rcp321QAsmi(f16Denom) MLIB_Rcp321QFAsmi(f16Denom)
#define MLIB_Rcp324QAsmi(f16Denom) MLIB_Rcp324QFAsmi(f16Denom)

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
* @brief  16-bit inputs 16-output single quadrant division function
*
* @param  ptr			
* 
* @param  in    		Frac16 f16Num
*                         - Numerator in [0;1] in Frac16
*						Frac16 f16Denom
*                         - Denominator in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function divides two non-negative fractional inputs:
* 			result = f16Num / f16Denom.
* 			The function normalizes the inputs to get higher precision of
* 			division.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 MLIB_Div1Q16SSFAsmi(register Frac16 f16Num, register Frac16 f16Denom)
{
		register Word16 w16ClbNum, w16ClbDenom, w16ClbResult;
		register Frac16 f16Out;
		
		asm(.optimize_iasm on);
	
		asm(.iasm_sideeffects off;	.iasm_reg2regsetcopyflag off;
			move.w f16Num,A;	
			.iasm_sideeffects on;	.iasm_reg2regsetcopyflag on);
		
		asm(clb A,w16ClbNum);				/* w16ClbNum = number of leading bits of f16Num */
		
		asm(sub.w #1,w16ClbNum);			/* w16ClbNum = w16ClbNum - 1 because we need to have f16Num 0.25 to 0.5 */
	
		asm(clb f16Denom,w16ClbDenom);		/* w16ClbDenom = number of leading bits of f16Denom */
		
		asm(asll.l w16ClbNum,A);			/* normalization of f16Num to the range 0.25 to 0.5 */	
		
		asm(asll.l w16ClbDenom,f16Denom);	/* normalization of f16Denom to 0.5 to 1.0 */
	
		asm(sub w16ClbNum,w16ClbDenom);		/* w16ClbDenom = shifts of f16Denom - shifts of w16ClbNum */
	
		asm(tst a);							/* Clears the C flag */
		asm(rep 8);							/* Repeat 8 times */
		asm(div f16Denom,A);				/* f32Num = A / f16Denom */
		asm(rep 8);							/* Repeat 8 times */
		asm(div f16Denom,A);				/* f32Num = A / f16Denom */
		
		asm(move.w A0,A);					/* A = A << 16 */	
		asm(tfr A,B);						/* Copy of the result */			

		asm(clb A,w16ClbResult);			/* Leading bits of the result */

		asm(asll.l w16ClbDenom,B);			/* B = B << w16ClbDenom (arithmetically) */
	
		asm(bfchg #0x8000,A1);				/* changes the MSB of the result */		

		asm(cmp.w w16ClbDenom,w16ClbResult);/* Leading bits comparison */
		
		asm(tlt A,B);						/* In case of result's overflow, uses the maximum output */
		
		asm(move.w B,f16Out);				/* saturation */
		
		asm(.optimize_iasm off);
		
		return f16Out;
}

/***************************************************************************//*!
*
* @brief  16-bit inputs inputs 16-output 4-quadrant division function
*
* @param  ptr			
* 
* @param  in    		Frac16 f16Num
*                         - Numerator in [0;1] in Frac16
*						Frac16 f16Denom
*                         - Denominator in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function divides two fractional inputs:
* 			result = f16Num / f16Denom.
* 			The function normalizes the inputs to get higher precision of
* 			division.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 MLIB_Div4Q16SSFAsmi(register Frac16 f16Num, register Frac16 f16Denom)
{
		register Word16 w16ClbNum, w16ClbDenom, w16ClbResult;
		register Frac16 f16Out;
		
		asm(.optimize_iasm on);
	
		asm(.iasm_sideeffects off;	.iasm_reg2regsetcopyflag off;
			move.w f16Num,A;	
			.iasm_sideeffects on;	.iasm_reg2regsetcopyflag on);
	
		asm(clb A,w16ClbNum);				/* w16ClbNum = number of leading bits of f16Num */
		asm(sub.w #1,w16ClbNum);			/* w16ClbNum = w16ClbNum - 1 because we need to have f16Num 0.25 to 0.5 */
		asm(asll.l w16ClbNum,A);			/* normalization of f16Num to the range 0.25 to 0.5 */	
		asm(clb f16Denom,w16ClbDenom);		/* w16ClbDenom = number of leading bits of f16Denom */
	
		asm(abs A);							/* Absolute value of the numerator */
		
		asm(asll.l w16ClbDenom,f16Denom);	/* normalization of f16Denom to 0.5 to 1.0 */
		asm(sub w16ClbNum,w16ClbDenom);		/* w16ClbDenom = shifts of f16Denom - shifts of w16ClbNum */
	
		asm(tst a);							/* Clears the C flag */
		asm(rep 8);							/* Repeat 8 times */
		asm(div f16Denom,A);				/* A = A / f16Denom */
		asm(rep 8);							/* Repeat 8 times */
		asm(div f16Denom,A);				/* A = A / f16Denom */
	
		asm(move.w A0,A);					/* A = A << 16 */	
		asm(tfr A,B);						/* Copy of the result */			

		asm(neg	A);							/* Negates the result */
		asm(eor.w f16Denom,f16Num);			/* Save sign bit of quotient in N bit of SR */
		asm(nop);
		asm(tlt	A,B);						/* B = -B if different signs */

		asm(clb B,w16ClbResult);			/* Leading bits of the result */
		asm(tfr B,A);						/* Copy of the result */
		
		asm(asll.l w16ClbDenom,B);			/* B = B << w16ClbDenom (arithmetically) */

		asm(bfchg #0x8000,A1);				/* changes the MSB of the result */		

		asm(cmp.w w16ClbDenom,w16ClbResult);/* Leading bits comparison */

		asm(tlt A,B);						/* In case of result's overflow, uses the maximum output */
		
		asm(move.w B,f16Out);				/* saturation */
		
		asm(.optimize_iasm off);
		
		return f16Out;
}

/***************************************************************************//*!
*
* @brief  32-bit numerator, 16-bit denominator inputs 16-output single quadrant
* 		  division function
*
* @param  ptr			
* 
* @param  in    		Frac32 f32Num
*                         - Numerator in [0;1] in Frac32
*						Frac16 f16Denom
*                         - Denominator in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function divides two  non-negative fractional inputs:
* 			result = f32Num / f16Denom.
* 			The function normalizes the inputs to get higher precision of
* 			division.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 MLIB_Div1Q16LSFAsmi(register Frac32 f32Num, register Frac16 f16Denom)
{
		register Word16 w16ClbNum, w16ClbDenom, w16ClbResult;
		register Frac16 f16Out;
		
		asm(.optimize_iasm on);

		asm(tfr f32Num,A);
		
		asm(clb A,w16ClbNum);				/* w16ClbNum = number of leading bits of f32Num */
		
		asm(sub.w #1,w16ClbNum);			/* w16ClbNum = w16ClbNum - 1 because we need to have f16Num 0.25 to 0.5 */
	
		asm(clb f16Denom,w16ClbDenom);		/* w16ClbDenom = number of leading bits of f16Denom */
		
		asm(asll.l w16ClbNum,A);			/* normalization of f16Num to the range 0.25 to 0.5 */	
		
		asm(asll.l w16ClbDenom,f16Denom);	/* normalization of f16Denom to 0.5 to 1.0 */
	
		asm(sub w16ClbNum,w16ClbDenom);		/* w16ClbDenom = shifts of f16Denom - shifts of w16ClbNum */
	
		asm(tst a);							/* Clears the C flag */
		asm(rep 8);							/* Repeat 8 times */
		asm(div f16Denom,A);				/* f32Num = A / f16Denom */
		asm(rep 8);							/* Repeat 8 times */
		asm(div f16Denom,A);				/* f32Num = A / f16Denom */
		
		asm(move.w A0,A);					/* A = A << 16 */	
		asm(tfr A,B);						/* Copy of the result */			

		asm(clb A,w16ClbResult);			/* Leading bits of the result */

		asm(asll.l w16ClbDenom,B);			/* V = V << w16ClbDenom (arithmetically) */
	
		asm(bfchg #0x8000,A1);				/* changes the MSB of the result */		

		asm(cmp.w w16ClbDenom,w16ClbResult);/* Leading bits comparison */
		
		asm(tlt A,B);						/* In case of result's overflow, uses the maximum output */
		
		asm(move.w B,f16Out);				/* saturation */
		
		asm(.optimize_iasm off);
		
		return f16Out;
}



/***************************************************************************//*!
*
* @brief  32-bit numerator, 16-bit denominator inputs 16-output 4-quadrant
* 		  division function
*
* @param  ptr			
* 
* @param  in    		Frac32 f32Num
*                         - Numerator in [0;1] in Frac32
*						Frac16 f16Denom
*                         - Denominator in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function divides two fractional inputs:
* 			result = f32Num / f16Denom.
* 			The function normalizes the inputs to get higher precision of
* 			division.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 MLIB_Div4Q16LSFAsmi(register Frac32 f32Num, register Frac16 f16Denom)
{
		register Word16 w16ClbNum, w16ClbDenom, w16ClbResult;
		register Frac16 f16Out;
		
		asm(.optimize_iasm on);

		asm(tfr f32Num,A);

		asm(clb A,w16ClbNum);				/* w16ClbNum = number of leading bits of f32Num */
		asm(sub.w #1,w16ClbNum);			/* w16ClbNum = w16ClbNum - 1 because we need to have f16Num 0.25 to 0.5 */
		asm(asll.l w16ClbNum,A);			/* normalization of f16Num to the range 0.25 to 0.5 */	
		asm(clb f16Denom,w16ClbDenom);		/* w16ClbDenom = number of leading bits of f16Denom */
	
		asm(abs A);							/* Absolute value of the numerator */
		
		asm(asll.l w16ClbDenom,f16Denom);	/* normalization of f16Denom to 0.5 to 1.0 */
		asm(sub w16ClbNum,w16ClbDenom);		/* w16ClbDenom = shifts of f16Denom - shifts of w16ClbNum */
	
		asm(tst a);							/* Clears the C flag */
		asm(rep 8);							/* Repeat 8 times */
		asm(div f16Denom,A);				/* A = A / f16Denom */
		asm(rep 8);							/* Repeat 8 times */
		asm(div f16Denom,A);				/* A = A / f16Denom */
	
		asm(move.w A0,A);					/* A = A << 16 */	
		asm(tfr A,B);						/* Copy of the result */			

		asm(neg	A);							/* Negates the result */
		asm(eor.w f16Denom,f32Num);			/* Save sign bit of quotient in N bit of SR */
		asm(nop);
		asm(tlt	A,B);						/* B = -B if different signs */

		asm(clb B,w16ClbResult);			/* Leading bits of the result */
		asm(tfr B,A);						/* Copy of the result */
		
		asm(asll.l w16ClbDenom,B);			/* B = B << w16ClbDenom (arithmetically) */

		asm(bfchg #0x8000,A1);				/* changes the MSB of the result */		

		asm(cmp.w w16ClbDenom,w16ClbResult);/* Leading bits comparison */

		asm(tlt A,B);						/* In case of result's overflow, uses the maximum output */
		
		asm(move.w B,f16Out);				/* saturation */
		
		asm(.optimize_iasm off);
		
		return f16Out;
}

/***************************************************************************//*!
*
* @brief  32-bit numerator, 16-bit denominator inputs 32-output single quadrant
* 		  division function
*
* @param  ptr			
* 
* @param  in    		Frac32 f32Num
*                         - Numerator in [0;1] in Frac32
*						Frac16 f16Denom
*                         - Denominator in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function divides two  non-negative fractional inputs:
* 			result = f32Num / f16Denom.
* 			The function normalizes the inputs to get higher precision of
* 			division.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 MLIB_Div1Q32LSFAsmi(register Frac32 f32Num, register Frac16 f16Denom)
{
		register Frac32 f32Result;
		register Word16 w16ClbNum, w16ClbDenom, w16ClbResult;
		register Frac16 f16Out;
		
		asm(.optimize_iasm on);

		asm(tfr f32Num,A);
		
		asm(clb A,w16ClbNum);				/* w16ClbNum = number of leading bits of f32Num */
		
		asm(sub.w #1,w16ClbNum);			/* w16ClbNum = w16ClbNum - 1 because we need to have f16Num 0.25 to 0.5 */

		asm(clb f16Denom,w16ClbDenom);		/* w16ClbDenom = number of leading bits of f16Denom */
		
		asm(asll.l w16ClbNum,A);			/* normalization of f16Num to the range 0.25 to 0.5 */	
		
		asm(asll.l w16ClbDenom,f16Denom);	/* normalization of f16Denom to 0.5 to 1.0 */

		asm(sub w16ClbNum,w16ClbDenom);		/* w16ClbDenom = shifts of f16Denom - shifts of w16ClbNum */

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
		
		asm(clb B,w16ClbResult);			/* Leading bits of the result */

		asm(tfr B,A);						/* Copy of the result */

		asm(asll.l w16ClbDenom,A);			/* A = A << w16ClbDenom (arithmetically) */
		
		asm(bfchg #0x8000,B1);				/* changes the MSB of the result */
		
		asm(cmp.w w16ClbDenom,w16ClbResult);/* Leading bits comparison */
		
		asm(tlt B,A);						/* In case of result's overflow, uses the maximum output */
		
		asm(sat A,f32Result);				/* saturation */
		
		asm(.optimize_iasm off);
		
		return f32Result;
}

/***************************************************************************//*!
*
* @brief  32-bit numerator, 16-bit denominator inputs 32-output four quadrant
* 		  division function
*
* @param  ptr			
* 
* @param  in    		Frac32 f32Num
*                         - Numerator in [0;1] in Frac32
*						Frac16 f16Denom
*                         - Denominator in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function divides two fractional inputs:
* 			result = f32Num / f16Denom.
* 			The function normalizes the inputs to get higher precision of
* 			division.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 MLIB_Div4Q32LSFAsmi(register Frac32 f32Num, register Frac16 f16Denom)
{
		register Frac32 f32Result;
		register Word16 w16ClbNum, w16ClbDenom, w16ClbResult;
		register Frac16 f16Out;
		
		asm(.optimize_iasm on);

		asm(tfr f32Num,A);
		
		asm(clb A,w16ClbNum);				/* w16ClbNum = number of leading bits of f16Num */
		
		asm(sub.w #1,w16ClbNum);			/* w16ClbNum = w16ClbNum - 1 because we need to have f16Num 0.25 to 0.5 */
		asm(asll.l w16ClbNum,A);			/* normalization of f16Num to the range 0.25 to 0.5 */	

		asm(clb f16Denom,w16ClbDenom);		/* w16ClbDenom = number of leading bits of f16Denom */
		
		asm(abs A);							/* Absolute value of the numerator */
		
		asm(asll.l w16ClbDenom,f16Denom);	/* normalization of f16Denom to 0.5 to 1.0 */

		asm(sub w16ClbNum,w16ClbDenom);		/* w16ClbDenom = shifts of f16Denom - shifts of w16ClbNum */

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

		asm(tfr B,A);						/* Copy of the result */			

		asm(neg	A);							/* Negates the result */
		asm(eor.w f16Denom,f32Num);			/* Save sign bit of quotient in N bit of SR */
		asm(nop);
		asm(tlt	A,B);						/* B = -B if different signs */
		
		asm(clb B,w16ClbResult);			/* Leading bits of the result */

		asm(tfr B,A);						/* Copy of the result */

		asm(asll.l w16ClbDenom,A);			/* A = A << w16ClbDenom (arithmetically) */
		
		asm(bfchg #0x8000,B1);				/* changes the MSB of the result */
		
		asm(cmp.w w16ClbDenom,w16ClbResult);/* Leading bits comparison */
		
		asm(tlt B,A);						/* In case of result's overflow, uses the maximum output */
		
		asm(sat A,f32Result);				/* saturation */
		
		asm(.optimize_iasm off);
		
		return f32Result;
}


/***************************************************************************//*!
*
* @brief  16-bit input 32-output 16-bit precision single quadrant reciprocal function
*
* @param  ptr			
* 
* @param  in    		Frac16 f16Denom
*                         - Denominator in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function calculates the multiplicative inverse value of
* 			the non-negative fractional input:
* 			result = FRAC32(1) / f16Denom. The function calculates
* 			the result with 16-bit division precision.
* 			The function normalize the inputs to get higher precision of
* 			division.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 MLIB_Rcp161QFAsmi(register Frac16 f16Denom)
{
		register Frac32 f32Result;
		register Word16 w16ClbDenom;
		
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

		asm(asll.l w16ClbDenom,f32Result);	/* f32Result = f32Result << w16ClbDenom (arithmetically) */

		asm(bfchg #0x8000,A1);				/* changes the MSB of the result */		

		asm(cmp.w #15,w16ClbDenom);			/* Leading bits comparison */

		asm(tgt A,f32Result);				/* In case of result's overflow, uses the maximum output */
		
		asm(sat f32Result);					/* saturation */
		
		asm(.optimize_iasm off);
		
		return f32Result;
}

/***************************************************************************//*!
*
* @brief  16-bit input 32-output 16-bit precision single quadrant reciprocal function
*
* @param  ptr			
* 
* @param  in    		Frac16 f16Denom
*                         - Denominator in [-1;1] in Frac16
*                       
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function calculates the multiplicative inverse value of
* 			the fractional input:
* 			result = FRAC32(1) / f16Denom. The function calculates
* 			the result with 16-bit division precision.
* 			The function normalizes the inputs to get higher precision of
* 			division.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 MLIB_Rcp164QFAsmi(register Frac16 f16Denom)
{
		register Frac32 f32Result;
		register Word16 w16ClbDenom;
		
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
* @param  in    		Frac16 f16Denom
*                         - Denominator in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function calculates the multiplicative inverse value of
* 			the non-negative fractional input:
* 			result = FRAC32(1) / f16Denom. The function calculates
* 			the result with 32-bit division precision.
* 			The function normalizes the inputs to get higher precision of
* 			division.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 MLIB_Rcp321QFAsmi(register Frac16 f16Denom)
{
		register Frac32 f32Result;
		register Word16 w16ClbDenom;
		
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

		asm(asll.l w16ClbDenom,f32Result);	/* f32Result = f32Result << w16ClbDenom (arithmetically) */
	
		asm(bfchg #0x8000,B1);				/* changes the MSB of the result */		

		asm(cmp.w #15,w16ClbDenom);			/* Leading bits comparison */

		asm(tgt B,f32Result);				/* In case of result's overflow, uses the maximum output */

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
* @param  in    		Frac16 f16Denom
*                         - Denominator in [-1;1] in Frac16
*                       
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function calculates the multiplicative inverse value of
* 			the fractional input:
* 			result = FRAC32(1) / f16Denom. The function calculates
* 			the result with 32-bit division precision.
* 			The function normalizes the inputs to get higher precision of
* 			division.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 MLIB_Rcp324QFAsmi(register Frac16 f16Denom)
{
		register Frac32 f32Result;
		register Word16 w16ClbDenom;
		
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

#endif /* _MLIB_DIVASM_H_ */

