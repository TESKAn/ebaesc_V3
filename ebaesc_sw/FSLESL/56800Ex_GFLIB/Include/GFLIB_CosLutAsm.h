/******************************************************************************
* 
* Copyright (c) 2008 Freescale Semiconductor;
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
* @file      GFLIB_CosLutAsm.h
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Feb-26-2013
* 
* @brief     Cosine algorithm using the look-up table, implemented in assembler
*
*******************************************************************************
*
* Cosine algorithm using the look-up table, implemented in assembler.
*
******************************************************************************/
#ifndef _GFLIB_COSLUTASM_H_
#define _GFLIB_COSLUTASM_H_

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
* @brief  Calculates the cosine of the given argument using a look-up table
*
* @param  ptr   		Frac16 *pf16SinTable
*						  - Pointer to the 1q sine look-up table
* @param  in    		const Frac16 fwX
*                         - Agument in [-1;1] in Frac16 corresponding to [-pi;pi]
*						UWord16 uw16TableSize
*						  - Size of the look-up table in bit shifts
*
* @return This function returns
*     - Frac16 value [-1;1]
*		
* @remarks 	This function calculates cos(pi * x) using the look-up table. The
*			argument must be in the range [-1;1] that corresponds [-pi;pi].
*			The look-up table is accessed using the pointer.
*			The table size must be based on 2^x, i.e. 256 values contains
*			the size 8. The look-up table is the sine and the function
*			uses the fact that cos = six(x + pi/2).
*
* 			SATURATION MUST BE TURNED ON!
*
****************************************************************************/
extern asm Frac16 GFLIB_CosLutFAsm(Frac16 f16Arg,
								   const Frac16 *pf16SinTable,
								   UWord16 uw16TableSize);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _GFLIB_COSLUTASM_H_ */