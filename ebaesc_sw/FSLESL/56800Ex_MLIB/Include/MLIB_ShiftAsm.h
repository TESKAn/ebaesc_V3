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
* @file      MLIB_ShiftAsm.h
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Aug-12-2013
* 
* @brief     Shift functions in assembler
*
*******************************************************************************
*
* Shift functions in assembler.
*
******************************************************************************/
#ifndef _MLIB_SHIFTASM_H_
#define _MLIB_SHIFTASM_H_

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
#define MLIB_Sh1L16Asmi(f16In) MLIB_Sh1L16FAsmi(f16In)
#define MLIB_Sh1L16SatAsmi(f16In) MLIB_Sh1L16SatFAsmi(f16In)
#define MLIB_Sh1R16Asmi(f16In) MLIB_Sh1R16FAsmi(f16In)

#define MLIB_Sh1L32Asmi(f32In) MLIB_Sh1L32FAsmi(f32In)
#define MLIB_Sh1L32SatAsmi(f32In) MLIB_Sh1L32SatFAsmi(f32In)
#define MLIB_Sh1R32Asmi(f32In) MLIB_Sh1R32FAsmi(f32In)

#define MLIB_ShL16Asmi(f16In, w16N) MLIB_ShL16FAsmi(f16In, w16N)
#define MLIB_ShL16SatAsmi(f16In, w16N) MLIB_ShL16SatFAsmi(f16In, w16N)
#define MLIB_ShR16Asmi(f16In, w16N) MLIB_ShR16FAsmi(f16In, w16N)
#define MLIB_ShR16SatAsmi(f16In, w16N) MLIB_ShR16SatFAsmi(f16In, w16N)

#define MLIB_ShL32Asmi(f32In, w16N) MLIB_ShL32FAsmi(f32In, w16N)
#define MLIB_ShL32SatAsmi(f32In, w16N) MLIB_ShL32SatFAsmi(f32In, w16N)
#define MLIB_ShR32Asmi(f32In, w16N) MLIB_ShR32FAsmi(f32In, w16N)
#define MLIB_ShR32SatAsmi(f32In, w16N) MLIB_ShR32SatFAsmi(f32In, w16N)

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
* @brief  16-bit one arithmetic shift left function
*
* @param  ptr			
* 
* @param  in    		Frac16 f16In
*                         - Argument in [0;1] in Frac16
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function returns the input shifted one bit to the left. The function
* 			does not saturate the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 MLIB_Sh1L16FAsmi(register Frac16 f16In)
{
		asm(.optimize_iasm on);
		
		asm(asl.w f16In);
		
		asm(.optimize_iasm off);
		
		return f16In;
}

/***************************************************************************//*!
*
* @brief  16-bit one arithmetic shift left function with saturation
*
* @param  ptr			
* 
* @param  in    		Frac16 f16In
*                         - Argument in [0;1] in Frac16
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function returns the input shifted one bit to the left and
* 			saturates if necessary.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 MLIB_Sh1L16SatFAsmi(register Frac16 f16In)
{
		register Frac32	f32Value;
		register Frac16 f16Out;
	
		asm(.optimize_iasm on);

		asm(move.w	f16In,f32Value);	
		
		asm(asl f32Value);
		asm(sat f32Value,f16Out);
		
		asm(.optimize_iasm off);
		
		return f16Out;
}

/***************************************************************************//*!
*
* @brief  16-bit one arithmetic shift right function
*
* @param  ptr			
* 
* @param  in    		Frac16 f16In
*                         - Argument in [0;1] in Frac16
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function returns the input shifted one bit to the right. The function
* 			does not saturate the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 MLIB_Sh1R16FAsmi(register Frac16 f16In)
{
		register Frac32	f32Value;
		register Frac16 f16Out;
		
		asm(.optimize_iasm on);
		
		asm(move.w	f16In,f32Value);	
		asm(asr f32Value);
		
		asm(move.w f32Value.1,f16Out);		
		
		asm(.optimize_iasm off);
		
		return f16Out;		
}

/***************************************************************************//*!
*
* @brief  32-bit one arithmetic shift left function
*
* @param  ptr			
* 
* @param  in    		Frac32 f32In
*                         - Argument in [0;1] in Frac32
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function returns the input shifted one bit to the left. The function
* 			does not saturate the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 MLIB_Sh1L32FAsmi(register Frac32 f32In)
{
		asm(.optimize_iasm on);
		
		asm(asl f32In);
		
		asm(.optimize_iasm off);
		
		return f32In;
}

/***************************************************************************//*!
*
* @brief  32-bit one arithmetic shift left function with saturation
*
* @param  ptr			
* 
* @param  in    		Frac32 f32In
*                         - Argument in [0;1] in Frac32
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function returns the input shifted one bit to the left and
* 			saturates the result if necessary.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 MLIB_Sh1L32SatFAsmi(register Frac32 f32In)
{
		asm(.optimize_iasm on);
		
		asm(asl f32In);
		asm(sat f32In);
		
		asm(.optimize_iasm off);
		
		return f32In;
}

/***************************************************************************//*!
*
* @brief  32-bit one arithmetic shift right function
*
* @param  ptr			
* 
* @param  in    		Frac32 f32In
*                         - Argument in [0;1] in Frac32
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function returns the input shifted one bit to the right. The function
* 			does not saturate the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 MLIB_Sh1R32FAsmi(register Frac32 f32In)
{
		asm(.optimize_iasm on);
		
		asm(asr f32In);
		
		asm(.optimize_iasm off);
		
		return f32In;
}

/***************************************************************************//*!
*
* @brief  16-bit arithmetic multi-bit shift left function
*
* @param  ptr			
* 
* @param  in    		Frac16 f16In
*                         - Argument in [0;1] in Frac16
*                       Word16 w16N
*                         - Number of shifts to the left; negative: shifts to the right  
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function returns the f16In input shifted by the number of w16N to the left. 
* 			If the w16N is negative, the input is shifted to the right. The function
* 			does not saturate the output. 
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 MLIB_ShL16FAsmi(register Frac16 f16In, register Word16 w16N)
{
		register Frac32	f32Value;
		register Frac16 f16Out;
		
		asm(.optimize_iasm on);

		asm(move.w f16In,f32Value);	/* converts it to 32-bit, because 16-bit shift is uni-directional */

		asm(asll.l w16N,f32Value);
		
		asm(nop);
		
		asm(move.w f32Value.1,f16Out);	
		
		asm(.optimize_iasm off);
		
		return f16Out;		
}

/***************************************************************************//*!
*
* @brief  16-bit arithmetic multi-bit shift left function with saturation
*
* @param  ptr			
* 
* @param  in    		Frac16 f16In
*                         - Argument in [0;1] in Frac16
*                       Word16 w16N
*                         - Number of shifts to the left; negative: shifts to the right  
*                         
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks This function returns the f16In input shifted by the number of w16N to the left. 
* 			If the w16N is negative, the input is shifted to the right. The function
* 			saturates if necessary.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 MLIB_ShL16SatFAsmi(register Frac16 f16In, register Word16 w16N)
{
		register Frac32	f32Value0, f32Value1;
		register Word16 w16Clb;
		register Frac16 f16Out;
	
		asm(.optimize_iasm on);

		asm(move.w	f16In,f32Value0);	/* converts it to 32-bit, because 16-bit shift is uni-directional */
		asm(tfr f32Value0,f32Value1);	/* makes a copy */

	    asm(clb f32Value0,w16Clb);    	/* acquire number of leading ones/zeros minus one of f32Value0 */ 

		asm(asll.l w16N,f32Value0);		/* Shifts the input to the left */

		asm(bfchg #0x8000,f32Value1.1);	/* changes the MSB of f32Value1 */

	    asm(cmp w16N,w16Clb);         	/* w16Clb - w16N -> comparing the number of leading ones of f32Value and w16N  */
		
		asm(tlt	f32Value1,f32Value0);	/* if no space to shift, uses the limit value */

		asm(sat f32Value0,f16Out);		/* saturates the result */
		
		asm(.optimize_iasm off);
		
		return f16Out;
}

/***************************************************************************//*!
*
* @brief  16-bit arithmetic multi-bit shift right function
*
* @param  ptr			
* 
* @param  in    		Frac16 f16In
*                         - Argument in [0;1] in Frac16
*                       Word16 w16N
*                         - Number of shifts to the left; negative: shifts to the left  
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function returns the f16In input shifted by the number of w16N to the right. 
* 			If the w16N is negative, the input is shifted to the left. The function
* 			does not saturate the output. 
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 MLIB_ShR16FAsmi(register Frac16 f16In, register Word16 w16N)
{
		register Frac32	f32Value;
		register Frac16 f16Out;
		
		asm(.optimize_iasm on);
		
		asm(move.w f16In,f32Value);	/* converts it to 32-bit, because 16-bit shift is uni-directional */

		asm(asrr.l w16N,f32Value);

		asm(nop);		
		
		asm(move.w f32Value.1,f16Out);
		
		asm(.optimize_iasm off);
		
		return f16Out;		
}

/***************************************************************************//*!
*
* @brief  16-bit arithmetic multi-bit shift right function with saturation
*
* @param  ptr			
* 
* @param  in    		Frac16 f16In
*                         - Argument in [0;1] in Frac16
*                       Word16 w16N
*                         - Number of shifts to the right; negative: shifts to the left  
*                         
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks This function returns the f16In input shifted by the number of w16N to the right. 
* 			If the w16N is negative, the input is shifted to the right. The function
* 			saturates if necessary.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 MLIB_ShR16SatFAsmi(register Frac16 f16In, register Word16 w16N)
{
		register Frac32	f32Value0, f32Value1;
		register Word16 w16Clb;
		register Frac16 f16Out;
	
		asm(.optimize_iasm on);

		asm(move.w	f16In,f32Value0);	/* converts it to 32-bit, because 16-bit shift is uni-directional */
		asm(tfr f32Value0,f32Value1);	/* makes a copy */

	    asm(clb f32Value0,w16Clb);    	/* acquire number of leading ones/zeros minus one of f32Value0 */ 

		asm(asrr.l w16N,f32Value0);		/* Shifts the input to the right */

		asm(bfchg #0x8000,f32Value1.1);	/* changes the MSB of f32Value */

	    asm(add w16N,w16Clb);         	/* w16Clb + w16N -> comparing the number of leading ones of f32Value and w16N  */
		
		asm(tlt	f32Value1,f32Value0);	/* if no space to shift, uses the limit value */

		asm(sat f32Value0,f16Out);		/* saturates the result */
		
		asm(.optimize_iasm off);
		
		return f16Out;
}

/***************************************************************************//*!
*
* @brief  32-bit arithmetic multi-bit shift left function
*
* @param  ptr			
* 
* @param  in    		Frac32 f16In
*                         - Argument in [0;1] in Frac32
*                       Word16 w16N
*                         - Number of shifts to the left; negative: shifts to the right  
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function returns the f32In input shifted by the number of w16N to the left. 
* 			If the w16N is negative, the input is shifted to the right. The function
* 			does not saturate the output. 
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 MLIB_ShL32FAsmi(register Frac32 f32In, register Word16 w16N)
{
		asm(.optimize_iasm on);
		
		asm(asll.l w16N,f32In);
		
		asm(nop);
		
		asm(.optimize_iasm off);
		
		return f32In;
}

/***************************************************************************//*!
*
* @brief  32-bit arithmetic multi-bit shift left function with saturation
*
* @param  ptr			
* 
* @param  in    		Frac32 f32In
*                         - Argument in [0;1] in Frac32
*                       Word16 w16N
*                         - Number of shifts to the left; negative: shifts to the right  
*                         
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks This function returns the 32In input shifted by the number of w16N to the left. 
* 			If the w16N is negative, the input is shifted to the right. The function
* 			saturates if necessary.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 MLIB_ShL32SatFAsmi(register Frac32 f32In, register Word16 w16N)
{
		register Frac32 f32Value1;
		register Word16 w16Clb;
		register Frac16 f16Out;
	
		asm(.optimize_iasm on);

		asm(tfr f32In,f32Value1);	/* makes a copy */

	    asm(clb f32In,w16Clb);    	/* acquire number of leading ones/zeros minus one of f32Value0 */ 

		asm(asll.l w16N,f32In);		/* Shifts the input to the left */

		asm(bfchg #0x8000,f32Value1.1);	/* changes the MSB of f32Value1 */

	    asm(cmp w16N,w16Clb);       /* w16Clb - w16N -> comparing the number of leading ones of f32Value and w16N  */
		
		asm(tlt	f32Value1,f32In);	/* if no space to shift, uses the limit value */

		asm(sat f32In);				/* saturates the result */
		
		asm(.optimize_iasm off);
		
		return f32In;
}

/***************************************************************************//*!
*
* @brief  32-bit arithmetic multi-bit shift right function
*
* @param  ptr			
* 
* @param  in    		Frac32 f16In
*                         - Argument in [0;1] in Frac32
*                       Word16 w16N
*                         - Number of shifts to the left; negative: shifts to the left  
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function returns the f32In input shifted by the number of w16N to the right. 
* 			If the w16N is negative, the input is shifted to the left. The function
* 			does not saturate the output. 
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 MLIB_ShR32FAsmi(register Frac32 f32In, register Word16 w16N)
{
		asm(.optimize_iasm on);
		
		asm(asrr.l w16N,f32In);
		
		asm(nop);
		
		asm(.optimize_iasm off);
		
		return f32In;
}

/***************************************************************************//*!
*
* @brief  32-bit arithmetic multi-bit shift right function with saturation
*
* @param  ptr			
* 
* @param  in    		Frac32 f32In
*                         - Argument in [0;1] in Frac32
*                       Word16 w16N
*                         - Number of shifts to the right; negative: shifts to the left  
*                         
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks This function returns the f32In input shifted by the number of w16N to the right. 
* 			If the w16N is negative, the input is shifted to the right. The function
* 			saturates if necessary.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 MLIB_ShR32SatFAsmi(register Frac32 f32In, register Word16 w16N)
{
		register Frac32 f32Value1;
		register Word16 w16Clb;
		register Frac16 f16Out;
	
		asm(.optimize_iasm on);

		asm(tfr f32In,f32Value1);	/* makes a copy */

	    asm(clb f32In,w16Clb);    	/* acquire number of leading ones/zeros minus one of f32Value0 */ 

		asm(asrr.l w16N,f32In);		/* Shifts the input to the right */

		asm(bfchg #0x8000,f32Value1.1);	/* changes the MSB of f32Value */

	    asm(add w16N,w16Clb);       /* w16Clb + w16N -> comparing the number of leading ones of f32Value and w16N  */
		
		asm(tlt	f32Value1,f32In);	/* if no space to shift, uses the limit value */

		asm(sat f32In);				/* saturates the result */
		
		asm(.optimize_iasm off);
		
		return f32In;
}

#endif /* _MLIB_SHIFTASM_H_ */

