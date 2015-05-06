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
* @file      MLIB_NegAsm.h
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Aug-12-2013
* 
* @brief     Negative value functions in assembler
*
*******************************************************************************
*
* Negate functions in assembler.
*
******************************************************************************/
#ifndef _MLIB_NEGASM_H_
#define _MLIB_NEGASM_H_

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
#define MLIB_Neg16Asmi(f16In) MLIB_Neg16FAsmi(f16In)
#define MLIB_Neg16SatAsmi(f16In) MLIB_Neg16SatFAsmi(f16In)
#define MLIB_Neg32Asmi(f32In) MLIB_Neg32FAsmi(f32In)
#define MLIB_Neg32SatAsmi(f32In) MLIB_Neg32SatFAsmi(f32In)

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
* @brief  16-bit negative value function
*
* @param  ptr			
* 
* @param  in    		Frac16 f16In
*                         - Argument in [0;1] in Frac16
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function returns the value value of the input. The function
* 			does not saturate the output, i.e. if input is -32768 the output
* 			will be -32768 too if the saturation mode is turned off.
*
*			SATURATION required for correct functionality!
*
****************************************************************************/
extern inline Frac16 MLIB_Neg16FAsmi(register Frac16 f16In)
{
		asm(.optimize_iasm on);
		
		asm(neg f16In);
		
		asm(.optimize_iasm off);
		
		return f16In;
}

/***************************************************************************//*!
*
* @brief  16-bit negative value saturated function
*
* @param  ptr			
* 
* @param  in    		Frac16 f16In
*                         - Argument in [0;1] in Frac16
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function returns the negative value of the input. The function
* 			saturates the output to 32767 if input is -32768.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 MLIB_Neg16SatFAsmi(register Frac16 f16In)
{
		register Frac32	f32Value;
		register Frac16 f16Out;
	
		asm(.optimize_iasm on);
		
		asm(move.w	f16In,f32Value);	
		
		asm(neg f32Value);

		asm(sat f32Value,f16Out);
		
		asm(.optimize_iasm off);
		
		return f16Out;
}

/***************************************************************************//*!
*
* @brief  32-bit negative value function
*
* @param  ptr			
* 
* @param  in    		Frac16 f32In
*                         - Argument in [0;1] in Frac32
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function returns the negative value of the input. The function
* 			does not saturate the output, i.e. if input is 0x8000 0000 the output
* 			will be 0x8000 0000 too if the saturation mode is turned off.
*
*			SATURATION required for correct functionality!
*
****************************************************************************/
extern inline Frac32 MLIB_Neg32FAsmi(register Frac32 f32In)
{
		asm(.optimize_iasm on);
		
		asm(neg f32In);
		
		asm(.optimize_iasm off);
		
		return f32In;
}

/***************************************************************************//*!
*
* @brief  32-bit negative value saturated function
*
* @param  ptr			
* 
* @param  in    		Frac16 f32In
*                         - Argument in [0;1] in Frac16
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function returns the negative value of the input. The function
* 			saturates the output to 0x7FFF FFFF if input is 0x8000 0000.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 MLIB_Neg32SatFAsmi(register Frac32 f32In)
{
		asm(.optimize_iasm on);
		
		asm(neg f32In);

		asm(sat f32In);
		
		asm(.optimize_iasm off);
		
		return f32In;
}

#endif /* _MLIB_NEGASM_H_ */

