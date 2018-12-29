/*******************************************************************************
*
* Copyright 2007-2014 Freescale Semiconductor, Inc.
*
* This software is owned or controlled by Freescale Semiconductor.
* Use of this software is governed by the Freescale License
* distributed with this Material.
* See the LICENSE file distributed for more details.
* 
*
****************************************************************************/
/*!
*
* @file      pclib.h
*
* @author    B37570
* 
* @version   2.0.00.0
* 
* @date      November-26-2014
* 
* @brief     Master header file for PCLIB
*
*******************************************************************************
*
* Functions that are included in PCLIB.
*
******************************************************************************/
#ifndef _PCLIB_H_
#define _PCLIB_H_

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
* Macros
*******************************************************************************/
#define PCLIB_Ctrl2P2ZInit_F16(psParam) PCLIB_Ctrl2P2ZInit_F16_Asmi(psParam)
#define PCLIB_Ctrl2P2Z_F16(f16InErr, psParam) PCLIB_Ctrl2P2Z_F16_Asm(f16InErr, psParam)
#define PCLIB_Ctrl3P3ZInit_F16(psParam) PCLIB_Ctrl3P3ZInit_F16_Asmi(psParam)
#define PCLIB_Ctrl3P3Z_F16(f16InErr, psParam) PCLIB_Ctrl3P3Z_F16_Asm(f16InErr, psParam)
#define PCLIB_CtrlPIInit_F16(psParam) PCLIB_CtrlPIInit_F16_Asmi(psParam)
#define PCLIB_CtrlPI_F16(f16InErr, psParam) PCLIB_CtrlPI_F16_Asm(f16InErr, psParam)
#define PCLIB_CtrlPIandLPInit_F16(psParam) PCLIB_CtrlPIandLPInit_F16_Asmi(psParam)
#define PCLIB_CtrlPIandLP_F16(f16InErr, psParam) PCLIB_CtrlPIandLP_F16_Asm(f16InErr, psParam)
#define PCLIB_CtrlPIDInit_F16(psParam) PCLIB_CtrlPIDInit_F16_Asmi(psParam)
#define PCLIB_CtrlPID_F16(f16InErr, psParam) PCLIB_CtrlPID_F16_Asm(f16InErr, psParam)

/******************************************************************************
* Includes
******************************************************************************/
#include "PCLIB_CtrlPI_F16_Asm.h"
#include "PCLIB_CtrlPIandLPFilter_F16_Asm.h"
#include "PCLIB_CtrlPID_F16_Asm.h"
#include "PCLIB_Ctrl2P2Z_F16_Asm.h"
#include "PCLIB_Ctrl3P3Z_F16_Asm.h"

#if defined(__cplusplus)
}
#endif  
  
#endif /* _PCLIB_H_ */
