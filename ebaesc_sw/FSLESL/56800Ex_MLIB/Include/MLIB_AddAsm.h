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
* @file      MLIB_AddAsm.h
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Aug-12-2013
* 
* @brief     Addition functions in assembler
*
*******************************************************************************
*
* Addition functions in assembler.
*
******************************************************************************/
#ifndef _MLIB_ADDASM_H_
#define _MLIB_ADDASM_H_

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
#define MLIB_Add16Asmi(f16In1, f16In2) MLIB_Add16FAsmi(f16In1, f16In2)
#define MLIB_Add16SatAsmi(f16In1, f16In2) MLIB_Add16SatFAsmi(f16In1, f16In2)

#define MLIB_Add32Asmi(f32In1, f32In2) MLIB_Add32FAsmi(f32In1, f32In2)
#define MLIB_Add32SatAsmi(f32In1, f32In2) MLIB_Add32SatFAsmi(f32In1, f32In2)

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
* @brief  16-bit addition function
*
* @param  ptr			
* 
* @param  in    		Frac16 f16In1
*                         - Argument in [0;1] in Frac16
*						Frac16 f16In2
*                         - Argument in [0;1] in Frac16                         
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function returns the addition of two inputs. The function
* 			does not saturate the output if the saturation mode is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline Frac16 MLIB_Add16FAsmi(register Frac16 f16In1, register Frac16 f16In2)
{
		asm(.optimize_iasm on);
		
		asm(add f16In2,f16In1);
		
		asm(.optimize_iasm off);
		
		return f16In1;
}

/***************************************************************************//*!
*
* @brief  16-bit addition saturated function
*
* @param  ptr			
* 
* @param  in    		Frac16 f16In1
*                         - Argument in [0;1] in Frac16
*						Frac16 f16In2
*                         - Argument in [0;1] in Frac16                         
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function returns the addition of two inputs. The function
* 			saturates the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 MLIB_Add16SatFAsmi(register Frac16 f16In1, register Frac16 f16In2)
{
		register Frac32	f32Value;
		register Frac16 f16Out;
	
		asm(.optimize_iasm on);
		
		asm(move.w	f16In1,f32Value);	
		
		asm(add f16In2,f32Value);

		asm(sat f32Value,f16Out);
		
		asm(.optimize_iasm off);
		
		return f16Out;
}

/***************************************************************************//*!
*
* @brief  32-bit addition function
*
* @param  ptr			
* 
* @param  in    		Frac16 f32In1
*                         - Argument in [0;1] in Frac32
*						Frac16 f32In2
*                         - Argument in [0;1] in Frac32
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function returns the addition of two inputs. The function
* 			does not saturate the output if the saturation mode is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline Frac32 MLIB_Add32FAsmi(register Frac32 f32In1, register Frac32 f32In2)
{
		asm(.optimize_iasm on);
		
		asm(add f32In2,f32In1);
		
		asm(.optimize_iasm off);
		
		return f32In1;
}

/***************************************************************************//*!
*
* @brief  32-bit addition saturated function
*
* @param  ptr			
* 
* @param  in    		Frac16 f32In1
*                         - Argument in [0;1] in Frac32
*						Frac16 f32In2
*                         - Argument in [0;1] in Frac32
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function returns the addition of two inputs. The function
* 			saturates the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 MLIB_Add32SatFAsmi(register Frac32 f32In1, register Frac32 f32In2)
{
		asm(.optimize_iasm on);
		
		asm(add f32In2,f32In1);

		asm(sat f32In1);
		
		asm(.optimize_iasm off);
		
		return f32In1;
}

#endif /* _MLIB_ADDASM_H_ */

