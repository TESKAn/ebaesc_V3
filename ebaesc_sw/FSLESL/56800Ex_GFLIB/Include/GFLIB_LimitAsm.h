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
* @file      GFLIB_LimitAsm.h
*
* @author    R61928
* 
* @version   1.0.3.0
* 
* @date      Aug-15-2013
* 
* @brief     Limit functions in assembler
*
*******************************************************************************
*
* Limit functions in assembler.
*
******************************************************************************/
#ifndef _GFLIB_LIMITASM_H_
#define _GFLIB_LIMITASM_H_

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
#define GFLIB_Limit16Asm(f16Arg, pudtLimit) GFLIB_Limit16FAsm(f16Arg, pudtLimit)
#define GFLIB_Limit32Asm(f32Arg, pudtLimit) GFLIB_Limit32FAsm(f32Arg, pudtLimit)
#define GFLIB_UpperLimit16Asm(f16Arg, f16UpperLimit) GFLIB_UpperLimit16FAsm(f16Arg, f16UpperLimit)
#define GFLIB_UpperLimit32Asm(f32Arg, f32UpperLimit) GFLIB_UpperLimit32FAsm(f32Arg, f32UpperLimit)
#define GFLIB_LowerLimit16Asm(f16Arg, f16LowerLimit) GFLIB_LowerLimit16FAsm(f16Arg, f16LowerLimit)
#define GFLIB_LowerLimit32Asm(f32Arg, f32LowerLimit) GFLIB_LowerLimit32FAsm(f32Arg, f32LowerLimit)

#define GFLIB_Limit16Asmi(f16Arg, pudtLimit) GFLIB_Limit16FAsmi(f16Arg, pudtLimit)
#define GFLIB_Limit32Asmi(f32Arg, pudtLimit) GFLIB_Limit32FAsmi(f32Arg, pudtLimit)
#define GFLIB_UpperLimit16Asmi(f16Arg, f16UpperLimit) GFLIB_UpperLimit16FAsmi(f16Arg, f16UpperLimit)
#define GFLIB_UpperLimit32Asmi(f32Arg, f32UpperLimit) GFLIB_UpperLimit32FAsmi(f32Arg, f32UpperLimit)
#define GFLIB_LowerLimit16Asmi(f16Arg, f16LowerLimit) GFLIB_LowerLimit16FAsmi(f16Arg, f16LowerLimit)
#define GFLIB_LowerLimit32Asmi(f32Arg, f32LowerLimit) GFLIB_LowerLimit32FAsmi(f32Arg, f32LowerLimit)


/******************************************************************************
* Types
******************************************************************************/
/* Limit structure */
typedef struct
{
    Frac16 f16UpperLimit;
    Frac16 f16LowerLimit;
} GFLIB_LIMIT16_T;

/* Limit structure */
typedef struct
{
    Frac32 f32UpperLimit;
    Frac32 f32LowerLimit;
} GFLIB_LIMIT32_T;

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  Limit function
*
* @param  ptr			GFLIB_LIMIT16_T *pudtLimit
*						  - upperLimit: max output trim
*						  - lowerLimit: min output trim
* @param  in    		Frac16 f16Arg
*                         - Argument in [0;1] in Frac16
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function trims the argument according to the upper and lower
*			limits in the pLimit structure. The upper limit must >= lower limit.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm Frac16 GFLIB_Limit16FAsm(Frac16 f16Arg, const GFLIB_LIMIT16_T *pudtLimit);

/***************************************************************************//*!
*
* @brief  Limit function 32-bit version
*
* @param  ptr			GFLIB_LIMIT32_T *pudtLimit
*						  - upperLimet: max output trim
*						  - lowerLimit: min output trim
* @param  in    		Frac32 f32Arg
*                         - Argument in [0;1] in Frac32
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function trims the argument according to the upper and lower
*			limits in the pLimit structure. The upper limit must >= lower limit.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm Frac32 GFLIB_Limit32FAsm(Frac32 f32Arg, const GFLIB_LIMIT32_T *pudtLimit);

/***************************************************************************//*!
*
* @brief  Upper limit function
*
* @param  ptr			
* @param  in    		Frac16 f16Arg
*                         - Argument in [0;1] in Frac16
*						Frac16 f16UpperLimit
*						  - Max output trim
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function trims the argument according to the upper 
*			limit in the f16UpperLimit variable.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm Frac16 GFLIB_UpperLimit16FAsm(Frac16 f16Arg, Frac16 f16UpperLimit);

/***************************************************************************//*!
*
* @brief  Uppwer limit function 32-bit version
*
* @param  ptr			
* @param  in    		Frac32 f32Arg
*                         - Argument in [0;1] in Frac32
*						Frac32 f32UpperLimit
*						  - Max output trim
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function trims the argument according to the upper 
*			limit in the f32UpperLimit variable.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm Frac32 GFLIB_UpperLimit32FAsm(Frac32 f32Arg, Frac32 f32UpperLimit);

/***************************************************************************//*!
*
* @brief  Lower limit function
*
* @param  ptr			
* @param  in    		Frac16 f16Arg
*                         - Argument in [0;1] in Frac16
*						Frac16 fwLowerLimit
*						  - Min output trim
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function trims the argument according to the lower 
*			limit in the fwLowerLimit variable.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm Frac16 GFLIB_LowerLimit16FAsm(Frac16 f16Arg, Frac16 f16LowerLimit);

/***************************************************************************//*!
*
* @brief  Lower limit function 32-bit version
*
* @param  ptr			
* @param  in    		Frac32 f32Arg
*                         - Argument in [0;1] in Frac32
*						Frac32 f32LowerLimit
*						  - Max output trim
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function trims the argument according to the lower 
*			limits in the f32LowerLimit variable.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm Frac32 GFLIB_LowerLimit32FAsm(Frac32 f32Arg, Frac32 f32LowerLimit);

/******************************************************************************
* Inline functions
******************************************************************************/
/***************************************************************************//*!
*
* @brief  Limit function
*
* @param  ptr			GFLIB_LIMIT16_T *pudtLimit
*						  - upperLimit: max output trim
*						  - lowerLimit: min output trim
* @param  in    		Frac16 f16Arg
*                         - Argument in [0;1] in Frac16
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function trims the argument according to the upper and lower
*			limits in the pLimit structure. The upper limit must >= lower limit.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 GFLIB_Limit16FAsmi(register Frac16 f16Arg, const register GFLIB_LIMIT16_T *pudtLimit)
{
	register Frac32 f32Out;
	register Frac16 f16Limit;
	register Frac16 f16Out;
	
	asm(.optimize_iasm on);
	
	asm(move.w	f16Arg,f32Out);				/* Copies argument to f32Out */
	asm(move.w	X:(pudtLimit)+,f16Limit);	/* Copies upper limit to f16Limit */
	asm(cmp.w	f16Limit,f32Out);			/* Compares f16Arg to upper limit */
	asm(tgt		f16Limit,f32Out);			/* Upper limit to f32Out if f16Arg > upper limit */
	asm(move.w	X:(pudtLimit),f16Limit);	/* Copies lower limit to f16Limit */
	asm(cmp.w	f16Limit,f32Out);			/* Compares new f16Arg to lower limit */
	asm(tlt		f16Limit,f32Out);			/* Lower limit to f32Out if f16Arg < lower limit */
	asm(move.w 	f32Out.1,f16Out);

	asm(.optimize_iasm off);
	
	return f16Out;
}

/***************************************************************************//*!
*
* @brief  Limit function 32-bit version
*
* @param  ptr			GFLIB_LIMIT32_T *pudtLimit
*						  - upperLimet: max output trim
*						  - lowerLimit: min output trim
* @param  in    		Frac32 f32Arg
*                         - Argument in [0;1] in Frac32
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function trims the argument according to the upper and lower
*			limits in the pLimit structure. The upper limit must >= lower limit.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 GFLIB_Limit32FAsmi(register Frac32 f32Arg, const register GFLIB_LIMIT32_T *pudtLimit)
{
	register Frac32 f32Limit;
	
	asm(.optimize_iasm on);
	
	asm(move.l	X:(pudtLimit)+,f32Limit);		/* Copies upper limit to f32Limit */
	asm(cmp.l	f32Limit,f32Arg);				/* Compares f32Arg to upper limit */
	asm(tgt		f32Limit,f32Arg);				/* Upper limit to f32Arg if f32Arg > upper limit */
	asm(move.l	X:(pudtLimit),f32Limit);		/* Copies lower limit to f32Limit */
	asm(cmp.l	f32Limit,f32Arg);				/* Compares f32Arg to upper limit */
	asm(tlt		f32Limit,f32Arg);				/* Lower limit to f32Arg if f32Arg < lower limit */
	
	asm(.optimize_iasm off);
	
	return f32Arg;
}

/***************************************************************************//*!
*
* @brief  Upper limit function
*
* @param  ptr			
* @param  in    		Frac16 f16Arg
*                         - Argument in [0;1] in Frac16
*						Frac16 f16UpperLimit
*						  - Max output trim
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function trims the argument according to the upper 
*			limit in the f16UpperLimit variable.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 GFLIB_UpperLimit16FAsmi(register Frac16 f16Arg, register Frac16 f16UpperLimit)
{
	register Frac32 f32Out;
	register Frac16 f16Out;

	asm(.optimize_iasm on);
	
	asm(move.w	f16Arg,f32Out);			/* Copies argument to f32Out */
	asm(cmp.w	f16UpperLimit,f16Arg);	/* Compares f16Arg to upper limit */
	asm(tgt		f16UpperLimit,f32Out);	/* Upper limit to f32Out if f16Arg > upper limit */
	asm(move.w	f32Out.1,f16Out);		/* Copies the result */
	
	asm(.optimize_iasm off);
	
	return f16Out;
}

/***************************************************************************//*!
*
* @brief  Upper limit function 32-bit version
*
* @param  ptr			
* @param  in    		Frac32 f32Arg
*                         - Argument in [0;1] in Frac32
*						Frac32 f32UpperLimit
*						  - Max output trim
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function trims the argument according to the upper 
*			limit in the f32UpperLimit variable.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 GFLIB_UpperLimit32FAsmi(register Frac32 f32Arg, register Frac32 f32UpperLimit)
{

	asm(.optimize_iasm on);
	
	asm(cmp.l 	f32UpperLimit,f32Arg);	/* Compares f32Arg to upper limit */
	asm(tgt 	f32UpperLimit,f32Arg);	/* Upper limit to A if f32Arg > upper limit */

	asm(.optimize_iasm off);
	
	return f32Arg;
}


/***************************************************************************//*!
*
* @brief  Lower limit function
*
* @param  ptr			
* @param  in    		Frac16 f16Arg
*                         - Argument in [0;1] in Frac16
*						Frac16 fwLowerLimit
*						  - Min output trim
*
* @return This function returns
*     - Frac16 value [0;1]
*		
* @remarks 	This function trims the argument according to the lower 
*			limit in the fwLowerLimit variable.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 GFLIB_LowerLimit16FAsmi(register Frac16 f16Arg, register Frac16 f16LowerLimit)
{
	register Frac32 f32Out;
	register Frac16 f16Out;

	asm(.optimize_iasm on);
	
	asm(move.w	f16Arg,f32Out);			/* Copies argument to f32Out */
	asm(cmp.w	f16LowerLimit,f16Arg);	/* Compares f16Arg to upper limit */
	asm(tlt		f16LowerLimit,f32Out);	/* Lower limit to f16Arg if f16Arg < lower limit */
	asm(move.w	f32Out.1,f16Out);		/* Copies the result */
	
	asm(.optimize_iasm off);
	
	return f16Out;
}

/***************************************************************************//*!
*
* @brief  Lower limit function 32-bit version
*
* @param  ptr			
* @param  in    		Frac32 f32Arg
*                         - Argument in [0;1] in Frac32
*						Frac32 f32LowerLimit
*						  - Max output trim
*
* @return This function returns
*     - Frac32 value [0;1]
*		
* @remarks 	This function trims the argument according to the lower 
*			limits in the f32LowerLimit variable.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac32 GFLIB_LowerLimit32FAsmi(register Frac32 f32Arg, register Frac32 f32LowerLimit)
{
		/*
			f32Arg - A
			f32LowerLimit - B
			return - A
		*/
	asm(.optimize_iasm on);

	asm(cmp.l	f32LowerLimit,f32Arg);		/* Compares f32Arg to upper limit */
	asm(tlt		f32LowerLimit,f32Arg);		/* Lower limit to f32Arg if f32Arg < lower limit */

	asm(.optimize_iasm off);
	
	return f32Arg;
}

#endif /* _GFLIB_LIMITASM_H_ */

