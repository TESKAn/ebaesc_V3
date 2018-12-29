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
* @brief  GDFLIB include file
* 
*******************************************************************************/
#ifndef _GDFLIB_H_
#define _GDFLIB_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 

#include "mlib.h"
#include "GDFLIB_FilterIIR1_F32_Asm.h"
#include "GDFLIB_FilterIIR2_F32_Asm.h"
#include "GDFLIB_FilterIIR3_F32_Asm.h"
#include "GDFLIB_FilterIIR4_F32_Asm.h"
#include "GDFLIB_FilterMA_A32_Asm.h"

/* Redeclaration of GDFLIB function */
#define GDFLIB_FilterIIR1Init_F16(psParam) GDFLIB_FilterIIR1Init_F16_Asmi(psParam)
#define GDFLIB_FilterIIR1_F16(f16InX, psParam) GDFLIB_FilterIIR1_F16_Asmi(f16InX, psParam) 

#define GDFLIB_FilterIIR2Init_F16(psParam) GDFLIB_FilterIIR2Init_F16_Asmi(psParam)
#define GDFLIB_FilterIIR2_F16(f16InX, psParam) GDFLIB_FilterIIR2_F16_Asm(f16InX, psParam) 

#define GDFLIB_FilterIIR3Init_F16(psParam) GDFLIB_FilterIIR3Init_F16_Asmi(psParam)
#define GDFLIB_FilterIIR3_F16(f16InX, psParam) GDFLIB_FilterIIR3_F16_Asm(f16InX, psParam) 

#define GDFLIB_FilterIIR4Init_F16(psParam) GDFLIB_FilterIIR4Init_F16_Asmi(psParam)
#define GDFLIB_FilterIIR4_F16(f16InX, psParam) GDFLIB_FilterIIR4_F16_Asm(f16InX, psParam) 

#define GDFLIB_FilterMAInit_F16(f16InitVal, psParam) GDFLIB_FilterMAInit_F16_Asmi(f16InitVal, psParam)
#define GDFLIB_FilterMA_F16(f16InX, psParam) GDFLIB_FilterMA_F16_Asmi(f16InX, psParam)

#if defined(__cplusplus) 
} 
#endif 

#endif

