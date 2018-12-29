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
* @brief  AMCLIB include file
* 
*******************************************************************************/
#ifndef _AMCLIB_H_
#define _AMCLIB_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 

#include "mlib.h"
#include "gmclib.h"

#include "AMCLIB_AngleTrackObsrv_F32_Asm.h"
#include "AMCLIB_TrackObsrv_F32_Asm.h"
#include "AMCLIB_PMSMBemfObsrvDQ_A32_Asm.h"


/* Redeclaration of AMCLIB function */
#define AMCLIB_AngleTrackObsrvInit_F16(f16ThetaInit, psCtrl) AMCLIB_AngleTrackObsrvInit_F16_Asm(f16ThetaInit, psCtrl)
#define AMCLIB_AngleTrackObsrv_F16(psSinCos, psCtrl) AMCLIB_AngleTrackObsrv_F16_Asm(psSinCos, psCtrl)
#define AMCLIB_TrackObsrvInit_F16(f16ThetaInit, psCtrl) AMCLIB_TrackObsrvInit_F16_Asmi(f16ThetaInit, psCtrl)
#define AMCLIB_TrackObsrv_F16(f16Error, psCtrl) AMCLIB_TrackObsrv_F16_Asm(f16Error, psCtrl)
#define AMCLIB_PMSMBemfObsrvDQInit_F16(psCtrl) AMCLIB_PMSMBemfObsrvDQInit_F16_Asmi(psCtrl)
#define AMCLIB_PMSMBemfObsrvDQ_F16(psIDQ, psUDQ, f16Speed, psCtrl) AMCLIB_PMSMBemfObsrvDQ_F16_Asm(psIDQ, psUDQ, f16Speed, psCtrl)

#if defined(__cplusplus) 
} 
#endif 

#endif /* _AMCLIB_H_ */
