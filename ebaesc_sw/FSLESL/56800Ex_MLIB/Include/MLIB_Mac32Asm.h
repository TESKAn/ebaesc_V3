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
* @file      MLIB_Mac32Asm.h
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Aug-12-2013
* 
* @brief     32x32-bit multiply accumulate functions in assembler
*
*******************************************************************************
*
* 32x32-bit multiply accumulate functions in assembler.
*
******************************************************************************/
#ifndef _MLIB_MAC32ASM_H_
#define _MLIB_MAC32ASM_H_

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
#define MLIB_Mac32LLLAsmi(f32Acc, f32In1, f32In2) MLIB_Mac32LLLFAsmi(f32Acc, f32In1, f32In2)
#define MLIB_Mac32LLLSatAsmi(f32Acc, f32In1, f32In2) MLIB_Mac32LLLSatFAsmi(f32Acc, f32In1, f32In2)

#define MLIB_Msu32LLLAsmi(f32Acc, f32In1, f32In2) MLIB_Msu32LLLFAsmi(f32Acc, f32In1, f32In2)
#define MLIB_Msu32LLLSatAsmi(f32Acc, f32In1, f32In2) MLIB_Msu32LLLSatFAsmi(f32Acc, f32In1, f32In2)

/* V3 core instruction functions */
#define MLIB_V3Mac32LLLAsmi(f32Acc, f32In1, f32In2) MLIB_V3Mac32LLLFAsmi(f32Acc, f32In1, f32In2)
#define MLIB_V3Mac32LLLSatAsmi(f32Acc, f32In1, f32In2) MLIB_V3Mac32LLLSatFAsmi(f32Acc, f32In1, f32In2)

#define MLIB_V3Msu32LLLAsmi(f32Acc, f32In1, f32In2) MLIB_V3Msu32LLLFAsmi(f32Acc, f32In1, f32In2)
#define MLIB_V3Msu32LLLSatAsmi(f32Acc, f32In1, f32In2) MLIB_V3Msu32LLLSatFAsmi(f32Acc, f32In1, f32In2)

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
* @brief  32-bit accumulator input, 32x32-bit multipliers input, 32-output
* 		  multiply accumulate function
*
* @param  ptr			
* 
* @param  in    		Frac32 f32Acc
*                         - Argument in [0;1] in Frac32, accumulator
*                       Frac32 f32In1
*                         - Argument in [0;1] in Frac32
*						Frac16 f32In2
*                         - Argument in [0;1] in Frac32
*                       
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs (32x32) and adds them to
* 			the accumulator:
* 			result = f32Acc + f32In1 * f32In2.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline Frac32 MLIB_Mac32LLLFAsmi(register Frac32 f32Acc, register Frac32 f32In1, register Frac32 f32In2)
{
		register Frac32 f32Result;
		
		asm(.optimize_iasm on);
	
		asm(tfr f32In1,y);
		
		asm(tfr f32In2,y);
		asm(move.w f32In1,x0);
		asm(mpysu x0,y0,f32Result);
		asm(tfr f32In1,y);
		asm(move.w f32In2,x0);
		asm(macsu x0,y0,f32Result);
		asm(asr16 f32Result);
		asm(add f32Acc,f32Result);
		asm(mac y1,f32In2.1,f32Result);
		
		asm(nop);
		
		asm(.optimize_iasm off);
		
		return f32Result;
}


/***************************************************************************//*!
*
* @brief  32-bit accumulator input, 32x32-bit multipliers input, 32-output
* 		  multiply accumulate saturated function
*
* @param  ptr			
* 
* @param  in    		Frac32 f32Acc
*                         - Argument in [0;1] in Frac32, accumulator
*                       Frac32 f32In1
*                         - Argument in [0;1] in Frac32
*						Frac32 f32In2
*                         - Argument in [0;1] in Frac32
*                       
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs (32x32) and adds them to
* 			the accumulator:
* 			result = f32Acc + f32In1 * f32In2.
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 MLIB_Mac32LLLSatFAsmi(register Frac32 f32Acc, register Frac32 f32In1, register Frac32 f32In2)
{
		register Frac32 f32Result;
		
		asm(.optimize_iasm on);
		
		asm(tfr f32In2,y);
		asm(move.w f32In1,x0);
		asm(mpysu x0,y0,f32Result);
		asm(tfr f32In1,y);
		asm(move.w f32In2,x0);
		asm(macsu x0,y0,f32Result);
		asm(asr16 f32Result);
		asm(add f32Acc,f32Result);
		asm(mac y1,f32In2.1,f32Result);

		asm(nop);
		
		asm(sat f32Result);
		
		asm(.optimize_iasm off);
		
		return f32Result;
}

/***************************************************************************//*!
*
* @brief  32-bit accumulator input, 32x32-bit multipliers input, 32-output
* 		  multiply subtract function
*
* @param  ptr			
* 
* @param  in    		Frac32 f32Acc
*                         - Argument in [0;1] in Frac32, accumulator
*                       Frac32 f32In1
*                         - Argument in [0;1] in Frac32
*						Frac16 f32In2
*                         - Argument in [0;1] in Frac32
*                       
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs (32x32) and subtracts them to
* 			the accumulator:
* 			result = f32Acc - f32In1 * f32In2.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline Frac32 MLIB_Msu32LLLFAsmi(register Frac32 f32Acc, register Frac32 f32In1, register Frac32 f32In2)
{
		register Frac32 f32Result;
		
		asm(.optimize_iasm on);
	
		asm(tfr f32In2,y);
		asm(move.w f32In1,x0);
		asm(mpysu x0,y0,f32Result);
		asm(tfr f32In1,y);
		asm(move.w f32In2,x0);
		asm(macsu x0,y0,f32Result);
		asm(asr16 f32Result);
		asm(sub f32Result,f32Acc);
		asm(mac -y1,f32In2.1,f32Acc);

		asm(nop);
		
		asm(.optimize_iasm off);
		
		return f32Acc;
}


/***************************************************************************//*!
*
* @brief  32-bit accumulator input, 32x32-bit multipliers input, 32-output
* 		  multiply subtract function
*
* @param  ptr			
* 
* @param  in    		Frac32 f32Acc
*                         - Argument in [0;1] in Frac32, accumulator
*                       Frac32 f32In1
*                         - Argument in [0;1] in Frac32
*						Frac32 f16In2
*                         - Argument in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs (32x32) and subtracts them to
* 			the accumulator:
* 			result = f32Acc - f32In1 * f16In2.
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 MLIB_Msu32LLLSatFAsmi(register Frac32 f32Acc, register Frac32 f32In1, register Frac32 f32In2)
{
		register Frac32 f32Result;
		
		asm(.optimize_iasm on);
		
		asm(tfr f32In2,y);
		asm(move.w f32In1,x0);
		asm(mpysu x0,y0,f32Result);
		asm(tfr f32In1,y);
		asm(move.w f32In2,x0);
		asm(macsu x0,y0,f32Result);
		asm(asr16 f32Result);
		asm(sub f32Result,f32Acc);
		asm(mac -y1,f32In2.1,f32Acc);
		
		asm(nop);
		
		asm(sat f32Acc);
		
		asm(.optimize_iasm off);
		
		return f32Acc;
}

/***************************************************************************//*!
*
* @brief  32-bit accumulator input, 32x32-bit multipliers input, 32-output
* 		  multiply accumulate function
*
* @param  ptr			
* 
* @param  in    		Frac32 f32Acc
*                         - Argument in [0;1] in Frac32, accumulator
*                       Frac32 f32In1
*                         - Argument in [0;1] in Frac32
*						Frac32 f32In2
*                         - Argument in [0;1] in Frac32
*                       
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs (32x32) and adds them to
* 			the accumulator:
* 			result = f32Acc + f32In1 * f32In2.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*		    The V3 core instructions used!			
*
****************************************************************************/
extern inline Frac32 MLIB_V3Mac32LLLFAsmi(register Frac32 f32Acc, register Frac32 f32In1, register Frac32 f32In2)
{
		asm(.optimize_iasm on);
		
		asm(mac32 f32In1,f32In2,f32Acc);
		
		asm(nop);
		
		asm(.optimize_iasm off);
		
		return f32Acc;
}


/***************************************************************************//*!
*
* @brief  32-bit accumulator input, 32x16-bit multipliers input, 32-output
* 		  multiply accumulate saturated function
*
* @param  ptr			
* 
* @param  in    		Frac32 f32Acc
*                         - Argument in [0;1] in Frac32, accumulator
*                       Frac32 f32In1
*                         - Argument in [0;1] in Frac32
*						Frac32 f32In2
*                         - Argument in [0;1] in Frac32
*                       
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs (32x32) and adds them to
* 			the accumulator:
* 			result = f32Acc + f32In1 * f32In2.
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*		    The V3 core instructions used!			
*
****************************************************************************/
extern inline Frac32 MLIB_V3Mac32LLLSatFAsmi(register Frac32 f32Acc, register Frac32 f32In1, register Frac32 f32In2)
{
		asm(.optimize_iasm on);
		
		asm(mac32 f32In1,f32In2,f32Acc);
		
		asm(nop);
		
		asm(sat f32Acc);
		
		asm(.optimize_iasm off);
		
		return f32Acc;
}

/***************************************************************************//*!
*
* @brief  32-bit accumulator input, 32x32-bit multipliers input, 32-output
* 		  multiply subtract function
*
* @param  ptr			
* 
* @param  in    		Frac32 f32Acc
*                         - Argument in [0;1] in Frac32, accumulator
*                       Frac32 f32In1
*                         - Argument in [0;1] in Frac32
*						Frac16 f32In2
*                         - Argument in [0;1] in Frac32
*                       
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs (32x16) and subtracts them to
* 			the accumulator:
* 			result = f32Acc - f32In1 * f32In2.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*		    The V3 core instructions used!
*
****************************************************************************/
extern inline Frac32 MLIB_V3Msu32LLLFAsmi(register Frac32 f32Acc, register Frac32 f32In1, register Frac32 f32In2)
{
		asm(.optimize_iasm on);
	
		asm(mac32 -f32In1,f32In2,f32Acc);
		
		asm(nop);
		
		asm(.optimize_iasm off);
		
		return f32Acc;
}


/***************************************************************************//*!
*
* @brief  32-bit accumulator input, 32x32-bit multipliers input, 32-output
* 		  multiply subtract function
*
* @param  ptr			
* 
* @param  in    		Frac32 f32Acc
*                         - Argument in [0;1] in Frac32, accumulator
*                       Frac32 f32In1
*                         - Argument in [0;1] in Frac32
*						Frac32 f16In2
*                         - Argument in [0;1] in Frac16
*                       
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function multiplies two fractional inputs (32x16) and subtracts them to
* 			the accumulator:
* 			result = f32Acc - f32In1 * f16In2.
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*		    The V3 core instructions used!
*
****************************************************************************/
extern inline Frac32 MLIB_V3Msu32LLLSatFAsmi(register Frac32 f32Acc, register Frac32 f32In1, register Frac32 f32In2)
{
		asm(.optimize_iasm on);
		
		asm(mac32 -f32In1,f32In2,f32Acc);
		
		asm(nop);
		
		asm(sat f32Acc);
		
		asm(.optimize_iasm off);
		
		return f32Acc;
}

#endif /* _MLIB_MAC32ASM_H_ */

