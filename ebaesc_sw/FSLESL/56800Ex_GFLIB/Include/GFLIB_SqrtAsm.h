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
* @file      GFLIB_SqrtAsm.h
*
* @author    r59400
* 
* @version   1.0.2.0
* 
* @date      Aug-15-2013
* 
* @brief     Square root calculation in assembler
*
*******************************************************************************
*
* Square root calculation in assembler.
*
******************************************************************************/
#ifndef _GFLIB_SQRTASM_H_
#define _GFLIB_SQRTASM_H_

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
#define GFLIB_SqrtShortAsm(f32Arg) GFLIB_SqrtShortFAsm(f32Arg)
#define GFLIB_SqrtFastAsm(f32Arg) GFLIB_SqrtFastFAsm(f32Arg)
#define GFLIB_SqrtIterAsm(f32Arg) GFLIB_SqrtIterFAsm(f32Arg)

/******************************************************************************
* Types
******************************************************************************/

/* Polynom table line */
typedef struct
{
    Frac16 f16Dummy;
    Frac16 f16XkOffset;
    Frac16 f16PolyCoef[5];
    Frac16 f16NYScl;
    Frac32 f32YkOffset;
} GFLIB_SQRT_POLY_ROW_T;

/* Polynom table line pointer */
typedef struct
{
	GFLIB_SQRT_POLY_ROW_T *pudtLine;
} GFLIB_SQRT_POLY_OFFSET_T;

/* Polynom table */
typedef struct
{
	GFLIB_SQRT_POLY_OFFSET_T udtLine1;
	GFLIB_SQRT_POLY_OFFSET_T udtLine2;
	GFLIB_SQRT_POLY_OFFSET_T udtLine3;
	GFLIB_SQRT_POLY_ROW_T udtInterval1;
	GFLIB_SQRT_POLY_ROW_T udtInterval2;
	GFLIB_SQRT_POLY_ROW_T udtInterval3;
} GFLIB_SQRT_POLY_TABLE_T;

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  Calculates the square root of the argument.
*
* @param  ptr
* @param  in    		Frac16 f32Arg
*                         - Argument in [0;1] in Frac32
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function calculates square root. The argument must be in the
*			range [0;1]. The algorithm adopts the principle of half interval with
*			14 iterations. The implementation is made by assembler.
*			This function uses a loop to be as short as possible from the memory
*			point of view.
*
****************************************************************************/
extern asm Frac16 GFLIB_SqrtShortFAsm(Frac32 f32Arg);

/***************************************************************************//*!
*
* @brief  Calculates the square root of the argument.
*
* @param  ptr
* @param  in    		Frac16 f32Arg
*                         - Argument in [0;1] in Frac32
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function calculates square root. The argument must be in the
*			range [0;1]. The algorithm adopts the principle of half interval with
*			14 iterations. The implementation is made by assembler.
*			This doesn't have loops so it's faster but less effecient from the
*			memory point of view.
*
****************************************************************************/
extern asm Frac16 GFLIB_SqrtFastFAsm(Frac32 f32Arg);

/***************************************************************************//*!
*
* @brief  Calculates the square root of the argument.
*
* @param  ptr			GFLIB_SQRT_POLY_TABLE *pPolyTable
*						  - Pointer to the polynom table 
* @param  in    		Frac16 f32Arg
*                         - Argument in [0;1] in Frac32
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function calculates square root. The argument must be in the
*			range [0;1]. The algorithm calculates the raw result using a polynom
*			of the 4th order using 3 intervals. Then the raw result is iterated
*			in 3 steps to get a more precise result.
*			If the value is negative, it is negated and then calculated as it was
*			positive. But the result keeps positive.
*
*			SATURATION INDEPENDENT!
* 
****************************************************************************/
extern asm Frac16 GFLIB_SqrtPolyFAsm(Frac32 f32Arg, const GFLIB_SQRT_POLY_TABLE_T *pudtPolyTable);

/***************************************************************************//*!
*
* @brief  Calculation of the square root.
*
* @param  in    	Frac32 f32Arg
*                   - Argument in range <0; 1) in Frac32
*
* @return This function returns
*     				- Frac16 value <0; 1)
*		
* @remarks 	
*	This function calculates Sqrt(x) using iterations:
*
*	1. The argument is normalized to 32-bit signed number where the number of
*	   shift is saved and the number of shifts is saved.
*
*	2. The are 4 iterations with the following equation:
*
*				  	 	 3      2 X
*	   Yk+1 = 2 * Yk * (--- - Yk ---)
*					 	 4        4
*
*	   where the initial Y0 = 0.5.
*
*	3. The final value of square root: Y = 2 * X * Y4
*
*	4. If the number was shifted to the left by the number of
*	   shifts before the sqrt calculation, after the calculatation the
*	   result must be shifted to the right by the half shifts.
*
*	   If the number of shifts is odd, the 0.5 shift is equal to:
*
*	   2 ^ 0.5 = 1 / sqrt(2) = 0.70711
*
*	   In the case of odd number the number is multiplied by 0.70711
*	   and then shifted by the number of n / 2 shifts. In case of even number,
*	   the result is only shifted by the number of n / 2 shifts.
*
*	   THE FUNCTION USES THE HARDARE DO LOOP.
*
*      SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm Frac16 GFLIB_SqrtIterFAsm(Frac32 f32Arg);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _GFLIB_SQRTASM_H_ */

