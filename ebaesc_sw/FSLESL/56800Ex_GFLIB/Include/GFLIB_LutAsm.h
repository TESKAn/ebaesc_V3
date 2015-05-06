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
* @file      GFLIB_LutAsm.h
*
* @author    R61928
* 
* @version   1.0.3.0
* 
* @date      Aug-15-2013
* 
* @brief     Look-up table algorithm implemented in assembler
*
*******************************************************************************
*
* Look-up table algorithm implemented in assembler.
*
******************************************************************************/
#ifndef _GFLIB_LUTASM_H_
#define _GFLIB_LUTASM_H_

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
#define GFLIB_LutAsm(f16Arg, pf16Table, uw16TableSize) GFLIB_LutFAsm(f16Arg, pf16Table, uw16TableSize)

/* The V3 core instructions */
#define GFLIB_V3LutAsm(f16Arg, pf16Table, uw16TableSize) GFLIB_V3LutFAsm(f16Arg, pf16Table, uw16TableSize)

/******************************************************************************
* Types
******************************************************************************/


/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  Look-up table algorithm using linear interpolation
*
* @param  ptr   		Frac16 *pf16Table
*						  - Pointer to the table values
* @param  in    		Frac16 f16Arg
*                         - Argument in [-1;1] in Frac16
*						UWord16 uw16TableSize
*						  - Size of the look-up table in bit shifts, 3 for 8 values
*
* @return This function returns
*     - Frac16 value [-1;1]
*		
* @remarks 	This function calculates the number from the table using interpolation
*			of two values in the table.
*			The table size must be based on 2^x, i.e. 256 values contains
*			the size 8.
*
*			SATURATION MUST BE TURNED OFF!
*
****************************************************************************/
extern asm Frac16 GFLIB_LutFAsm(Frac16 f16Arg, const Frac16 *pf16Table, UWord16 uw16TableSize);

/***************************************************************************//*!
*
* @brief  Look-up table algorithm using linear interpolation
*
* @param  ptr   		Frac16 *pf16Table
*						  - Pointer to the table values
* @param  in    		Frac16 f16Arg
*                         - Argument in [-1;1] in Frac16
*						UWord16 uw16TableSize
*						  - Size of the look-up table in bit shifts, 3 for 8 values
*
* @return This function returns
*     - Frac16 value [-1;1]
*		
* @remarks 	This function calculates the number from the table using interpolation
*			of two values in the table.
*			The table size must be based on 2^x, i.e. 256 values contains
*			the size 8.
*
*			SATURATION MUST BE TURNED OFF!
*		    The V3 core instructions used! 	
*
****************************************************************************/
extern asm Frac16 GFLIB_V3LutFAsm(Frac16 f16Arg, const Frac16 *pf16Table, UWord16 uw16TableSize);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _GFLIB_LUTASM_H_ */

