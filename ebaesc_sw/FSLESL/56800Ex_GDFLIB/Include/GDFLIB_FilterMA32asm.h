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
* @file      GDFLIB_FilterMA32asm.h
*
* @author    r61928
* 
* @version   1.0.3.0
* 
* @date      Aug-15-2013
* 
* @brief     Moving Average Filter (FIR) assembler + inline asm implementations
*
*******************************************************************************
*
* Moving Average Filter (FIR) assembler + inline asm implementations.
*
******************************************************************************/

#ifndef _GDFLIB_FILTERMA32ASM_H_
#define _GDFLIB_FILTERMA32ASM_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "56800E_types.h"

/******************************************************************************
* Macros 
******************************************************************************/
#define GDFLIB_FilterMA32Asm(f16In, pudtFilter) GDFLIB_FilterMA32FAsm(f16In, pudtFilter)
#define GDFLIB_FilterMA32InitC(pudtFilter) GDFLIB_FilterMA32InitFC(pudtFilter)
#define GDFLIB_FilterMA32InitValAsm(f16InitVal, pudtFilter) GDFLIB_FilterMA32InitValFAsm(f16InitVal, pudtFilter)

#define GDFLIB_FilterMA32Asmi(f16In, pudtFilter) GDFLIB_FilterMA32FAsmi(f16In, pudtFilter)

#define GDFLIB_FILTER_MA32_DEFAULT {0, 0}

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
	Frac32	f32Acc;
	Word16	w16N;
} GDFLIB_FILTER_MA32_T;

/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  The function initializes recursive form moving average filter
*
* @param  ptr   		GDFLIB_FILTER_MA32_T *pudtFilter
*						  	- Pointer to filter structure
*						OUT - f32Acc - accumulator of the filter
*
* @param  in    		
*
* @return N/A
*		
* @remarks 
*
****************************************************************************/
extern void GDFLIB_FilterMA32InitFC(GDFLIB_FILTER_MA32_T * const pudtFilter);

/***************************************************************************//*!
*
* @brief  The function initializes the moving average filter with the predefined value.
*
* @param  ptr   		GDFLIB_FILTER_MA32_T *pudtFilter
*						  	- Pointer to filter structure
*
* @param  in    		Frac16 f16InitVal
* 							- Initial value
*
* @return N/A
*		
* @remarks 
*
****************************************************************************/
extern asm void GDFLIB_FilterMA32InitValFAsm(Frac16 f16InitVal, GDFLIB_FILTER_MA32_T * const pudtFilter);

/***************************************************************************//*!
*
* @brief  The function calculates more accurate recursive form moving average filter
*
* @param  ptr   		GDFLIB_FILTER_MA32_T *pudtFilter
*						  	- Pointer to filter structure
*						IN/OUT - f32Acc - accumulator of the filter
*						IN	- w16N - 2 ^ w16N is the number os values
*
* @param  in    		f16In - input signal
*
* @return This function returns the filtered signal
*     - Frac16 value [-1;1]
*		
* @remarks 
*
* Filter Equation:
* 
* sum(k) 	= sum(k-1) + input
* out(k)	= sum(k)/N
* sum(k-1)	= sum(k) - out(k)
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm Frac16 GDFLIB_FilterMA32FAsm(Frac16 f16In, GDFLIB_FILTER_MA32_T * const pudtFilter);

/******************************************************************************
* Inline functions
******************************************************************************/
/***************************************************************************//*!
*
* @brief  The function calculates more accurate recursive form moving average filter
*
* @param  ptr   		GDFLIB_FILTER_MA32_T *pudtFilter
*						  	- Pointer to filter structure
*						OUT/IN - f32Acc - accumulator of the filter
*						IN	- w16N - 2 ^ w16N is the number os values
*
* @param  in    		f16In - input signal
*
* @return This function returns
*     - Frac16 value [-1;1]
*		
* @remarks 
*
* Filter Equation:
* 
* sum(k) 	= sum(k-1) + input
* out(k)	= sum(k)/N
* sum(k-1)	= sum(k) - out(k)
*
*			THE SATURATION MUST BE TURNED ON.
*
****************************************************************************/
inline Frac16 GDFLIB_FilterMA32FAsmi(register Frac16 f16In, 
									 register GDFLIB_FILTER_MA32_T *pudtFilter)
{
	register Frac16 f16Out;

	asm(.optimize_iasm on);
	asm(tfra	pudtFilter,R2);
	asm(move.l  X:(R2)+,B);
	asm(asr16   f16In,A);
	asm(add     B,A				X:(R2)+,Y0);
	asm(tfr		A,B);
	asm(asrr.l	Y0,A);
	asm(asl16   A,f16Out);
	asm(sub		A,B);
	asm(move.l	B10,X:(pudtFilter));
	asm(.optimize_iasm off);
	return(f16Out);
}
 
#endif /* _GDFLIB_FILTERMA32ASM_H_ */
