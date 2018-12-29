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
* @brief  GMCLIB include file
* 
*******************************************************************************/
#ifndef _GMCLIB_H_
#define _GMCLIB_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 

#include "gflib.h"
#include "gmclib_types.h"

#include "GMCLIB_Clark_F16_Asm.h"
#include "GMCLIB_Park_F16_Asm.h"
#include "GMCLIB_Decoupling_A32_Asm.h"
#include "GMCLIB_ElimDcBusRip_F16_Asm.h"
#include "GMCLIB_Svm_F16_Asm.h"

/* Redeclaration of GMCLIB function */
#define	GMCLIB_SvmStd_F16(psIn, psOut) GMCLIB_SvmStd_F16_Asm(psIn, psOut)
#define	GMCLIB_SvmIct_F16(psIn, psOut) GMCLIB_SvmIct_F16_Asm(psIn, psOut)
#define	GMCLIB_SvmU0n_F16(psIn, psOut) GMCLIB_SvmU0n_F16_Asm(psIn, psOut)
#define	GMCLIB_SvmU7n_F16(psIn, psOut) GMCLIB_SvmU7n_F16_Asm(psIn, psOut)


#define	GMCLIB_Clark_F16(psIn, psOut) GMCLIB_Clark_F16_Asmi(psIn, psOut)
#define GMCLIB_ClarkInv_F16(psIn, psOut) GMCLIB_ClarkInv_F16_Asmi(psIn, psOut)

#define GMCLIB_Park_F16(psIn, psAnglePos, psOut) GMCLIB_Park_F16_Asmi(psIn, psAnglePos, psOut)
#define GMCLIB_ParkInv_F16(psIn, psAnglePos, psOut) GMCLIB_ParkInv_F16_Asmi(psIn, psAnglePos, psOut)

#define GMCLIB_DecouplingPMSM_F16(psUDQ, psIDQ, f16SpeedEl, psParam, psUDQDec) \
			GMCLIB_DecouplingPMSM_F16_Asm(psUDQ, psIDQ, f16SpeedEl, psParam, psUDQDec)

#define GMCLIB_ElimDcBusRip_F16sas(f16UDCBus, a32IdxMod, psUAlBe, psUAlBeComp) \
			GMCLIB_ElimDcBusRip_F16sas_Asm(f16UDCBus, a32IdxMod, psUAlBe, psUAlBeComp)

#define GMCLIB_ElimDcBusRipFOC_F16(f16UDCBus, psUAlBe, psUAlBeComp) \
			GMCLIB_ElimDcBusRipFOC_F16_Asm(f16UDCBus, psUAlBe, psUAlBeComp)

#if defined(__cplusplus) 
} 
#endif 

#endif

