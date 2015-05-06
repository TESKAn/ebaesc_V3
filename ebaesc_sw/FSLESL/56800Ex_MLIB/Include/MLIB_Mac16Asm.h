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
* @file      MLIB_Mac16Asm.h
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Aug-12-2013
* 
* @brief     Multiply accumulate functions in assembler
*
*******************************************************************************
*
* Multiply accumulate functions in assembler.
*
******************************************************************************/
#ifndef _MLIB_MAC16ASM_H_
#define _MLIB_MAC16ASM_H_

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
#define MLIB_Mac16SSSAsmi(f16Acc, f16In1, f16In2) MLIB_Mac16SSSFAsmi(f16Acc, f16In1, f16In2)
#define MLIB_Mac16SSSSatAsmi(f16Acc, f16In1, f16In2) MLIB_Mac16SSSSatFAsmi(f16Acc, f16In1, f16In2)

#define MLIB_Msu16SSSAsmi(f16Acc, f16In1, f16In2) MLIB_Msu16SSSFAsmi(f16Acc, f16In1, f16In2)
#define MLIB_Msu16SSSSatAsmi(f16Acc, f16In1, f16In2) MLIB_Msu16SSSSatFAsmi(f16Acc, f16In1, f16In2)

#define MLIB_Mac32LSSAsmi(f32Acc, f16In1, f16In2) MLIB_Mac32LSSFAsmi(f32Acc, f16In1, f16In2)
#define MLIB_Mac32LSSSatAsmi(f32Acc, f16In1, f16In2) MLIB_Mac32LSSSatFAsmi(f32Acc, f16In1, f16In2)

#define MLIB_Msu32LSSAsmi(f32Acc, f16In1, f16In2) MLIB_Msu32LSSFAsmi(f32Acc, f16In1, f16In2)
#define MLIB_Msu32LSSSatAsmi(f32Acc, f16In1, f16In2) MLIB_Msu32LSSSatFAsmi(f32Acc, f16In1, f16In2)

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
* @brief  16-bit inputs 16-output multiply accumulate function
*
* @param  ptr			
* 
* @param  in    		Frac16 f16Acc
*                         - Argument in [0;1] in Frac16, accumulator
*                       Frac16 f16In1
*                         - Argument in [0;1] in Frac16
*						Frac16 f16In2
*                         - Argument in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs and adds them to
* 			the accumulator:
* 			result = f16Acc + f16In1 * f16In2.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline Frac16 MLIB_Mac16SSSFAsmi(register Frac16 f16Acc, register Frac16 f16In1, register Frac16 f16In2)
{
		register Frac32 f32Acc;
		register Frac16 f16Out;
		
		asm(.optimize_iasm on);
		
		asm(move.w f16Acc,f32Acc);
				
		asm(mac f16In1,f16In2,f32Acc);
		
		asm(nop);
		
		asm(move.w f32Acc.1,f16Out);
		
		asm(.optimize_iasm off);
		
		return f16Out;
}

/***************************************************************************//*!
*
* @brief  16-bit inputs 16-output multiply accumulate saturated function
*
* @param  ptr			
* 
* @param  in    		Frac16 f16Acc
*                         - Argument in [0;1] in Frac16, accumulator
*                       Frac16 f16In1
*                         - Argument in [0;1] in Frac16
*						Frac16 f16In2
*                         - Argument in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs and adds them to
* 			the accumulator:
* 			result = f16Acc + f16In1 * f16In2.
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 MLIB_Mac16SSSSatFAsmi(register Frac16 f16Acc, register Frac16 f16In1, register Frac16 f16In2)
{
		register Frac32 f32Acc;
		register Frac16 f16Out;
	
		asm(.optimize_iasm on);
		
		asm(move.w f16Acc,f32Acc);	
		
		asm(mac f16In1,f16In2,f32Acc);

		asm(nop);
		
		asm(sat f32Acc,f16Out);
		
		asm(.optimize_iasm off);
		
		return f16Out;
}

/***************************************************************************//*!
*
* @brief  16-bit inputs 16-output multiply subtract function
*
* @param  ptr			
* 
* @param  in    		Frac16 f16Acc
*                         - Argument in [0;1] in Frac16, accumulator
*                       Frac16 f16In1
*                         - Argument in [0;1] in Frac16
*						Frac16 f16In2
*                         - Argument in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs and subtracts them from
* 			the accumulator:
* 			result = f16Acc - f16In1 * f16In2.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline Frac16 MLIB_Msu16SSSFAsmi(register Frac16 f16Acc, register Frac16 f16In1, register Frac16 f16In2)
{
		register Frac32 f32Acc;
		register Frac16 f16Out;
		
		asm(.optimize_iasm on);
		
		asm(move.w f16Acc,f32Acc);
				
		asm(mac -f16In1,f16In2,f32Acc);

		asm(nop);		
		
		asm(move.w f32Acc.1,f16Out);		
		
		asm(.optimize_iasm off);
		
		return f16Out;
}

/***************************************************************************//*!
*
* @brief  16-bit inputs 16-output multiply subtract saturated function
*
* @param  ptr			
* 
* @param  in    		Frac16 f16Acc
*                         - Argument in [0;1] in Frac16, accumulator
*                       Frac16 f16In1
*                         - Argument in [0;1] in Frac16
*						Frac16 f16In2
*                         - Argument in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs and subtracts them from
* 			the accumulator:
* 			result = f16Acc - f16In1 * f16In2.
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 MLIB_Msu16SSSSatFAsmi(register Frac16 f16Acc, register Frac16 f16In1, register Frac16 f16In2)
{
		register Frac32 f32Acc;
		register Frac16 f16Out;
	
		asm(.optimize_iasm on);
		
		asm(move.w f16Acc,f32Acc);	
		
		asm(mac -f16In1,f16In2,f32Acc);

		asm(nop);		
		
		asm(sat f32Acc,f16Out);
		
		asm(.optimize_iasm off);
		
		return f16Out;
}

/***************************************************************************//*!
*
* @brief  32-bit accumulator input, 16-bit multipliers input, 32-output
* 		  multiply accumulate function
*
* @param  ptr			
* 
* @param  in    		Frac32 f32Acc
*                         - Argument in [0;1] in Frac32, accumulator
*                       Frac16 f16In1
*                         - Argument in [0;1] in Frac16
*						Frac16 f16In2
*                         - Argument in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs and adds them to
* 			the accumulator:
* 			result = f32Acc + f16In1 * f16In2.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline Frac32 MLIB_Mac32LSSFAsmi(register Frac32 f32Acc, register Frac16 f16In1, register Frac16 f16In2)
{
		asm(.optimize_iasm on);
		
		asm(mac f16In1,f16In2,f32Acc);
		
		asm(nop);

		asm(.optimize_iasm off);
		
		return f32Acc;
}

/***************************************************************************//*!
*
* @brief  32-bit accumulator input, 16-bit multipliers input, 32-output
* 		  multiply accumulate saturated function
*
* @param  ptr			
* 
* @param  in    		Frac32 f32Acc
*                         - Argument in [0;1] in Frac32, accumulator
*                       Frac16 f16In1
*                         - Argument in [0;1] in Frac16
*						Frac16 f16In2
*                         - Argument in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs and adds them to
* 			the accumulator:
* 			result = f32Acc + f16In1 * f16In2.
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 MLIB_Mac32LSSSatFAsmi(register Frac32 f32Acc, register Frac16 f16In1, register Frac16 f16In2)
{
		asm(.optimize_iasm on);
		
		asm(mac f16In1,f16In2,f32Acc);

		asm(nop);
		
		asm(sat f32Acc);
		
		asm(.optimize_iasm off);
		
		return f32Acc;
}

/***************************************************************************//*!
*
* @brief  32-bit accumulator input, 16-bit multipliers input,
* 		  32-output multiply subtract function
*
* @param  ptr			
* 
* @param  in    		Frac32 f32Acc
*                         - Argument in [0;1] in Frac32, accumulator
*                       Frac16 f16In1
*                         - Argument in [0;1] in Frac16
*						Frac16 f16In2
*                         - Argument in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs and subtracts them from
* 			the accumulator:
* 			result = f32Acc - f16In1 * f16In2.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline Frac32 MLIB_Msu32LSSFAsmi(register Frac32 f32Acc, register Frac16 f16In1, register Frac16 f16In2)
{
		asm(.optimize_iasm on);
		
		asm(mac -f16In1,f16In2,f32Acc);

		asm(nop);
		
		asm(.optimize_iasm off);
		
		return f32Acc;
}

/***************************************************************************//*!
*
* @brief  32-bit accumulator input, 16-bit multipliers input,
* 		  32-output multiply subtract saturated function
*
* @param  ptr			
* 
* @param  in    		Frac32 f32Acc
*                         - Argument in [0;1] in Frac32, accumulator
*                       Frac16 f16In1
*                         - Argument in [0;1] in Frac16
*						Frac16 f16In2
*                         - Argument in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs and subtracts them from
* 			the accumulator:
* 			result = f32Acc - f16In1 * f16In2.
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 MLIB_Msu32LSSSatFAsmi(register Frac32 f32Acc, register Frac16 f16In1, register Frac16 f16In2)
{
		asm(.optimize_iasm on);
		
		asm(mac -f16In1,f16In2,f32Acc);

		asm(nop);
		
		asm(sat f32Acc);
		
		asm(.optimize_iasm off);
		
		return f32Acc;
}

#endif /* _MLIB_MAC16ASM_H_ */

