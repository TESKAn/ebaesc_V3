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
* @brief  Digital IIR Filter, 3rd order assembler implementation
* 
*******************************************************************************/
#ifndef _GDFLIB_FILTERIIR3_F32_ASM_H_
#define _GDFLIB_FILTERIIR3_F32_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define GDFLIB_FilterIIR3Init_F16_Asmi(psParam) GDFLIB_FilterIIR3Init_F16_FAsmi(psParam)
#define GDFLIB_FilterIIR3_F16_Asm(f16InX, psParam) GDFLIB_FilterIIR3_F16_FAsm(f16InX, psParam) 

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
	frac32_t			f32B0;
	frac32_t			f32B1;
	frac32_t			f32B2;
	frac32_t			f32B3;
	frac32_t			f32A1;
	frac32_t			f32A2;
	frac32_t			f32A3;
} GDFLIB_FILTER_IIR3_COEFF_T_F32;

typedef struct 
{
    GDFLIB_FILTER_IIR3_COEFF_T_F32	sFltCoeff;
    frac32_t 						f32FltBfrY[3];
    frac16_t 						f16FltBfrX[3];
} GDFLIB_FILTER_IIR3_T_F32;

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/
extern asm frac16_t GDFLIB_FilterIIR3_F16_FAsm(frac16_t f16In, GDFLIB_FILTER_IIR3_T_F32 *psParam);

/******************************************************************************
* Inline functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  The function clears the buffer of 4th order IIR filter
*
* @param  ptr			GDFLIB_FILTER_IIR3_T *psParam
* 						- pointer to the structure of the filter
*
*                         GDFLIB_FILTER_IIR3_T_F32 data structure:
*                           GDFLIB_FILTER_IIR3_COEFF_T_F32	sFltCoeff   
*                               - filter coefficients in frac32_t divided by 4
*                                 in the following range
*                                 -1.0 <= f16Gain < 1.0
*                               - B coefficients:
*                               	frac32_t			f32B0
*									frac32_t			f32B1
*									frac32_t			f32B2
*									frac32_t			f32B3
*								- -A coefficient, the sign is negated
*									frac32_t			f32A1;
*									frac32_t			f32A2;
*									frac32_t			f32A3;
*                           frac32_t f32FltBfrY[3];
*                               - y history buffer in frac32_t
*							frac16_t f16FltBfrX[3];
*								- x history buffer in frac16_t
*
* @param  in    		
*
* @return N/A
*		
* @remarks 
*
****************************************************************************/
extern inline void GDFLIB_FilterIIR3Init_F16_FAsmi(register GDFLIB_FILTER_IIR3_T_F32 *psParam)
{
	register frac32_t f32Temp = 0;
	register frac32_t *pf32Temp = psParam->f32FltBfrY;
	
	asm(move.l f32Temp,X:(pf32Temp)+);
	asm(move.l f32Temp,X:(pf32Temp)+);
	asm(move.l f32Temp,X:(pf32Temp)+);
	asm(move.w f32Temp,X:(pf32Temp)+);
	asm(move.w f32Temp,X:(pf32Temp)+);
	asm(move.w f32Temp,X:(pf32Temp));
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _GDFLIB_FILTERIIR3_F32_ASM_H_ */
