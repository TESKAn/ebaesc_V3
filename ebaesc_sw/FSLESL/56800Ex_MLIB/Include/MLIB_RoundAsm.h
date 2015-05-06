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
* @file      MLIB_RoundAsm.h
*
* @author    R61928
* 
* @version   1.0.2.0
* 
* @date      Jan-8-2014
* 
* @brief     Absolute value functions in assembler
*
*******************************************************************************
*
* Round functions in assembler.
*
******************************************************************************/
#ifndef _MLIB_ROUNDASM_H_
#define _MLIB_ROUNDASM_H_

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
#define MLIB_Rnd16Asmi(f32In) MLIB_Rnd16FAsmi(f32In)
#define MLIB_Rnd16SatAsmi(f32In) MLIB_Rnd16SatFAsmi(f32In)
#define MLIB_Rnd32Asmi(f32In) MLIB_Rnd32FAsmi(f32In)
#define MLIB_Rnd32SatAsmi(f32In) MLIB_Rnd32SatFAsmi(f32In)

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
* @brief  16-bit round function
*
* @param  ptr			
* 
* @param  in    		Frac32 f32In
*                         - Argument in [0;1] in Frac32
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function rounds the lower 16-bits of the 32-bit input and
* 			returns the upper 16-bit. The function
* 			does not saturate the output if the saturation mode is turned off.
*
*			SATURATION required for correct functionality!
*
****************************************************************************/
extern inline Frac16 MLIB_Rnd16FAsmi(register Frac32 f32In)
{
		register Frac16 f16Out;
	
		asm(.optimize_iasm on);
		
		asm(rnd f32In);
		
		asm(move.w f32In.1,f16Out);
		
		asm(.optimize_iasm off);
		
		return f16Out;
}

/***************************************************************************//*!
*
* @brief  16-bit round function with saturation
*
* @param  ptr			
* 
* @param  in    		Frac32 f32In
*                         - Argument in [0;1] in Frac32
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function rounds the lower 16-bits of the 32-bit input and
* 			returns the upper 16-bit. The function saturates the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 MLIB_Rnd16SatFAsmi(register Frac32 f32In)
{
		register Frac16 f16Out;
	
		asm(.optimize_iasm on);
		
		asm(rnd f32In);
		
		asm(sat f32In,f16Out);
		
		asm(.optimize_iasm off);
		
		return f16Out;
}

/***************************************************************************//*!
*
* @brief  32-bit round function
*
* @param  ptr			
* 
* @param  in    		Frac32 f32In
*                         - Argument in [0;1] in Frac32
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function rounds the lower 16-bits of the 32-bit input. The function
* 			does not saturate the output if the saturation mode is turned off.
*
*			SATURATION required for correct functionality!
*
****************************************************************************/
extern inline Frac32 MLIB_Rnd32FAsmi(register Frac32 f32In)
{
		asm(.optimize_iasm on);
		
		asm(rnd f32In);
		
		asm(.optimize_iasm off);
		
		return f32In;
}

/***************************************************************************//*!
*
* @brief  32-bit round function
*
* @param  ptr			
* 
* @param  in    		Frac32 f32In
*                         - Argument in [0;1] in Frac32
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function rounds the lower 16-bits of the 32-bit input. The function
* 			saturates the output, in case the input is > x7fff8000, the output is
* 			0x7fff ffff.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 MLIB_Rnd32SatFAsmi(register Frac32 f32In)
{
		asm(.optimize_iasm on);
		
		asm(rnd f32In);
		
		asm(sat f32In);
		
		asm(.optimize_iasm off);
		
		return f32In;
}


#endif /* _MLIB_ROUNDASM_H_ */

