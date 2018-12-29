/*******************************************************************************
*
* Copyright 2007-2015 Freescale Semiconductor, Inc.
*
* This software is owned or controlled by Freescale Semiconductor.
* Use of this software is governed by the Freescale License
* distributed with this Material.
* See the LICENSE file distributed for more details.
* 
*
****************************************************************************//*!
*
* @brief  Binary logarithm function in assembler
* 
*******************************************************************************/
#ifndef MLIB_LOG2ASM_H_
#define MLIB_LOG2ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib_types.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define MLIB_Log2_U16_Asmi(u16Val) MLIB_Log2_U16_FAsmi(u16Val)

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
* @brief  Binary logarithm of 16-bit input
*
* @param  ptr			
* 
* @param  in    		uint16_t u16Val
*                         - Argument in uint16_t
*
* @return This function returns
*     - uint16_t value
*		
* @remarks 	This function returns the 16-bit integer part of binary logarithm of the input. 
* 			Returns 0 for input u16Val <= 0.  
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline uint16_t MLIB_Log2_U16_FAsmi(register uint16_t u16Val)
{
	asm(.optimize_iasm on);
		
	asm(move.w #-1,Y1);		
		
	asm(move.w #14,X0);
		
	asm(clb u16Val,A);			/* Number of leading bits of ui16In => A */
				
	asm(tst.w u16Val);			/* Compares ui16In to 0 */

	asm(tle	X0,A);				/* If (u16Val <= 0), then 14 => A */
		
	asm(move.w #14,Y1);			/* 14 => Y1 */
		
	asm(sub A,Y1);				/* Y1 = 14 - A */
				
	asm(move.w	Y1, u16Val);			
		
	asm(.optimize_iasm off);
		
	return u16Val;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* MLIB_LOG2ASM_H_ */
