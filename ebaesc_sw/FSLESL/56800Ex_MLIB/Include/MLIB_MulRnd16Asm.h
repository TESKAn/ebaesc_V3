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
* @file      MLIB_MulRnd16Asm.h
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Aug-12-2013
* 
* @brief     Multiply and round functions in assembler
*
*******************************************************************************
*
* Multiply and round functions in assembler.
*
******************************************************************************/
#ifndef _MLIB_MULRNDASM_H_
#define _MLIB_MULRNDASM_H_

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
#define MLIB_MulRnd16SSAsmi(f16In1, f16In2) MLIB_MulRnd16SSFAsmi(f16In1, f16In2)
#define MLIB_MulRnd16SSSatAsmi(f16In1, f16In2) MLIB_MulRnd16SSSatFAsmi(f16In1, f16In2)

#define MLIB_MulRnd32SSAsmi(f16In1, f16In2) MLIB_MulRnd32SSFAsmi(f16In1, f16In2)
#define MLIB_MulRnd32SSSatAsmi(f16In1, f16In2) MLIB_MulRnd32SSSatFAsmi(f16In1, f16In2)

#define MLIB_MulNegRnd16SSAsmi(f16In1, f16In2) MLIB_MulNegRnd16SSFAsmi(f16In1, f16In2)
#define MLIB_MulNegRnd16SSSatAsmi(f16In1, f16In2) MLIB_MulNegRnd16SSSatFAsmi(f16In1, f16In2)

#define MLIB_MulNegRnd32SSAsmi(f16In1, f16In2) MLIB_MulNegRnd32SSFAsmi(f16In1, f16In2)
#define MLIB_MulNegRnd32SSSatAsmi(f16In1, f16In2) MLIB_MulNegRnd32SSSatFAsmi(f16In1, f16In2)

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
* @brief  16-bit inputs 16-output multiply function with result rounding
*
* @param  ptr			
* 
* @param  in    		Frac16 f16In1
*                         - Argument in [0;1] in Frac16
*						Frac16 f16In2
*                         - Argument in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs:
* 			result = f16In1 * f16In2.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline Frac16 MLIB_MulRnd16SSFAsmi(register Frac16 f16In1, register Frac16 f16In2)
{
		register Frac32 f32Value;
		register Frac16 f16Out;
		
		asm(.optimize_iasm on);
		
		asm(mpyr f16In1,f16In2,f32Value);

		asm(nop);
		
		asm(move.w f32Value.1,f16Out);			
		
		asm(.optimize_iasm off);
		
		return f16Out;		
}

/***************************************************************************//*!
*
* @brief  16-bit inputs 16-output multiply saturated function with result rounding
*
* @param  ptr			
* 
* @param  in    		Frac16 f16In1
*                         - Argument in [0;1] in Frac16
*						Frac16 f16In2
*                         - Argument in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs and rounds to the upper 16 bits:
* 			result = f16In1 * f16In2.
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 MLIB_MulRnd16SSSatFAsmi(register Frac16 f16In1, register Frac16 f16In2)
{
		register Frac32 f32Value;
		register Frac16 f16Out;
	
		asm(.optimize_iasm on);
		
		asm(mpyr f16In1,f16In2,f32Value);
		
		asm(nop);
		
		asm(sat f32Value,f16Out);
		
		asm(.optimize_iasm off);
		
		return f16Out;
}

/***************************************************************************//*!
*
* @brief  16-bit inputs 32-output multiply function with result rounding
*
* @param  ptr			
* 
* @param  in    		Frac16 f16In1
*                         - Argument in [0;1] in Frac16
*						Frac16 f16In2
*                         - Argument in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs:
* 			result = f16In1 * f16In2.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline Frac32 MLIB_MulRnd32SSFAsmi(register Frac16 f16In1, register Frac16 f16In2)
{
		register Frac32 f32Value;
		
		asm(.optimize_iasm on);
		
		asm(mpyr f16In1,f16In2,f32Value);

		asm(nop);
		
		asm(.optimize_iasm off);
		
		return f32Value;
}

/***************************************************************************//*!
*
* @brief  16-bit inputs 32-output multiply saturated function with result rounding
*
* @param  ptr			
* 
* @param  in    		Frac16 f16In1
*                         - Argument in [0;1] in Frac16
*						Frac16 f16In2
*                         - Argument in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs and rounds to the upper 16 bits:
* 			result = f16In1 * f16In2.
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 MLIB_MulRnd32SSSatFAsmi(register Frac16 f16In1, register Frac16 f16In2)
{
		register Frac32 f32Value;
	
		asm(.optimize_iasm on);
		
		asm(mpyr f16In1,f16In2,f32Value);

		asm(nop);
		
		asm(sat f32Value);
		
		asm(.optimize_iasm off);
		
		return f32Value;
}



/***************************************************************************//*!
*
* @brief  16-bit inputs 16-output multiply negate function with result rounding
*
* @param  ptr			
* 
* @param  in    		Frac16 f16In1
*                         - Argument in [0;1] in Frac16
*						Frac16 f16In2
*                         - Argument in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs and negates the result
*  			and rounds to the upper 16 bits:
* 			result = -f16In1 * f16In2.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*			
****************************************************************************/
extern inline Frac16 MLIB_MulNegRnd16SSFAsmi(register Frac16 f16In1, register Frac16 f16In2)
{
		register Frac32 f32Value;
		register Frac16 f16Out;
		
		asm(.optimize_iasm on);
		
		asm(mpyr -f16In1,f16In2,f32Value);
	
		asm(nop);
		
		asm(move.w f32Value.1,f16Out);	
		
		asm(.optimize_iasm off);
		
		return f16Out;
}

/***************************************************************************//*!
*
* @brief  16-bit inputs 16-output multiply negate function with result rounding
* 		  with saturation
*
* @param  ptr			
* 
* @param  in    		Frac16 f16In1
*                         - Argument in [0;1] in Frac16
*						Frac16 f16In2
*                         - Argument in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs and negates the result
*  			and rounds to the upper 16 bits:
* 			result = -f16In1 * f16In2.
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 MLIB_MulNegRnd16SSSatFAsmi(register Frac16 f16In1, register Frac16 f16In2)
{
		register Frac32 f32Value;
		register Frac16 f16Out;	
		
		asm(.optimize_iasm on);
		
		asm(mpyr -f16In1,f16In2,f32Value);
	
		asm(nop);
		
		asm(sat f32Value,f16Out);
		
		asm(.optimize_iasm off);
		
		return f16Out;
}

/***************************************************************************//*!
*
* @brief  16-bit inputs 32-output multiply negate function with result rounding
*
* @param  ptr			
* 
* @param  in    		Frac16 f16In1
*                         - Argument in [0;1] in Frac16
*						Frac16 f16In2
*                         - Argument in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs and negates the result
*  			and rounds to the upper 16 bits:
* 			result = -f16In1 * f16In2.
*			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline Frac32 MLIB_MulNegRnd32SSFAsmi(register Frac16 f16In1, register Frac16 f16In2)
{
		register Frac32 f32Value;
		
		asm(.optimize_iasm on);
		
		asm(mpyr -f16In1,f16In2,f32Value);
	
		asm(nop);
		
		asm(.optimize_iasm off);
		
		return f32Value;
}

/***************************************************************************//*!
*
* @brief  16-bit inputs 32-output multiply negate function with result rounding
*
* @param  ptr			
* 
* @param  in    		Frac16 f16In1
*                         - Argument in [0;1] in Frac16
*						Frac16 f16In2
*                         - Argument in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs and negates the result
*  			and rounds to the upper 16 bits:
* 			result = -f16In1 * f16In2.
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 MLIB_MulNegRnd32SSSatFAsmi(register Frac16 f16In1, register Frac16 f16In2)
{
		register Frac32 f32Value;
		
		asm(.optimize_iasm on);
		
		asm(mpyr -f16In1,f16In2,f32Value);
	
		asm(nop);
	
		asm(sat f32Value);
		
		asm(.optimize_iasm off);
		
		return f32Value;
}

#endif /* _MLIB_MULRND16ASM_H_ */

