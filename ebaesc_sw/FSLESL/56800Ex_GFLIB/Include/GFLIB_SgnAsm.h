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
* @file      GFLIB_SgnAsm.h
*
* @author    R61928
* 
* @version   1.0.4.0
* 
* @date      Aug-27-2013
* 
* @brief     Signum function in assembler
*
*******************************************************************************
*
* Signum function in assembler.
*
******************************************************************************/
#ifndef _GFLIB_SGNASM_H_
#define _GFLIB_SGNASM_H_

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
#define GFLIB_SgnAsm(f16Arg) GFLIB_SgnFAsmi(f16Arg)

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
* @brief  Signum function
*
* @param  ptr
* @param  in    		Frac16 f16Arg
*                         - Argument in [0;1] in Frac16
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function returns the sign of the argument:
*			f16Arg = 0: returns 0
*			f16Arg > 0: returns 32767 (FRAC16(1.0))
*			f16Arg < 0: returns -32768 (FRAC16(-1.0))
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm Frac16 GFLIB_SgnFAsm(Frac16 f16Arg);

/******************************************************************************
* Inline functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  Signum function
*
* @param  ptr
* @param  in    		Frac16 f16Arg
*                         - Argument in [0;1] in Frac16
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function returns the sign of the argument:
*			f16Arg = 0: returns 0
*			f16Arg > 0: returns 32767 (FRAC16(1.0))
*			f16Arg < 0: returns -32768 (FRAC16(-1.0))
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 GFLIB_SgnFAsmi(register Frac16 f16Arg)
{
	register Frac32 f32Out;
	register Frac16 f16Pos;
	register Frac16 f16Neg;
	register Frac16 f16Out;	
	
	asm(.optimize_iasm on);

	asm(clr		f32Out);			/* Clears f32Out */
	asm(move.w	#$7fff,f16Pos);		/* FRAC16(1.0) to f16Pos */
	asm(move.w	#$8000,f16Neg);		/* FRAC16(-1.0) to f16Neg */
	asm(tst.w	f16Arg);			/* Compares f16Arg to 0 */
	asm(tgt		f16Pos,f32Out);		/* FRAC16(1.0) to A if f16Arg > 0 */
	asm(tlt		f16Neg,f32Out);		/* FRAC16(-1.0) to A if f16Arg < 0 */
	asm(move.w	f32Out.1,f16Out);	/* Copies the result */
	
	asm(.optimize_iasm off);
	
	return f16Out;
}

#endif /* _GFLIB_SGNASM_H_ */

