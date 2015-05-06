/******************************************************************************
* 
* Copyright (c) 2008 Freescale Semiconductor;
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
* @file      gflib.h
*
* @author    r61928
* 
* @version   1.0.4.0
* 
* @date      Aug-12-2013
* 
* @brief     GFLIB include file
*
*******************************************************************************
*
* Functions that are included in GFLIB.
*
******************************************************************************/
#ifndef _GFLIB_H_
#define _GFLIB_H_

#include "56800E_types.h"

#include "GFLIB_SinCosDefAsm.h"
#include "GFLIB_SinLutAsm.h"
#include "GFLIB_CosLutAsm.h"

#include "GFLIB_SinCosTlrDefAsm.h"
#include "GFLIB_SinCosTlrAsm.h"

#include "GFLIB_TanAsmDef.h"
#include "GFLIB_TanAsm.h"

#include "GFLIB_AsinAcosAsm.h"
#include "GFLIB_AsinAcosAsmDef.h"

#include "GFLIB_AtanAsm.h"
#include "GFLIB_AtanAsmDef.h"

#include "GFLIB_AtanYXAsm.h"
#include "GFLIB_AtanYXAsmDef.h"

#include "GFLIB_AtanYXShiftedAsm.h"

#include "GFLIB_SgnAsm.h"

#include "GFLIB_Sgn2Asm.h"

#include "GFLIB_SqrtAsm.h"
#include "GFLIB_SqrtDefAsm.h"

#include "GFLIB_HystAsm.h"

#include "GFLIB_LutAsm.h"

#include "GFLIB_RampAsm.h"

#include "GFLIB_DynRampAsm.h"

#include "GFLIB_LimitAsm.h"

#include "GFLIB_ControllerPIRecurrentAsm.h"

#include "GFLIB_ControllerPIRecurrentLIMAsm.h"

#include "GFLIB_ControllerPIpAsm.h"

#include "GFLIB_ControllerPIDpAsm.h"

#include "GFLIB_ControllerPIDRecurrentAsm.h"

#if OPTION_CORE_V3 == 1 /* V3 core instructions used */

/* redeclaration of the functions */
#define GFLIB_SinLut(f16Arg) GFLIB_SinLutAsm(f16Arg)
#define GFLIB_CosLut(f16Arg) GFLIB_CosLutAsm(f16Arg)
#define GFLIB_SinLut2(f16Arg, udtSinTable) GFLIB_SinLut2Asm(f16Arg, udtSinTable)
#define GFLIB_CosLut2(f16Arg, udtSinTable) GFLIB_CosLut2Asm(f16Arg, udtSinTable)

#define GFLIB_SinTlr(f16In) GFLIB_V3SinTlrAsm(f16In)
#define GFLIB_CosTlr(f16In) GFLIB_V3CosTlrAsm(f16In)

#define GFLIB_Sin12Tlr(f16In) GFLIB_Sin12TlrAsm(f16In)
#define GFLIB_Cos12Tlr(f16In) GFLIB_Cos12TlrAsm(f16In)


#define GFLIB_Tan(f16Arg) GFLIB_TanAsm(f16Arg)

#define GFLIB_Asin(f16Arg)   GFLIB_AsinAsm(f16Arg)
#define GFLIB_Acos(f16Arg)   GFLIB_AcosAsm(f16Arg)

#define GFLIB_Atan(f16Arg)   GFLIB_AtanAsm(f16Arg)

#define GFLIB_AtanYX(f16ValY, f16ValX, pi16ErrFlag) GFLIB_AtanYXAsm(f16ValY, f16ValX, pi16ErrFlag)

#define GFLIB_AtanYXShifted(f16ValY, f16ValX, pudtAtanYXCoeff) GFLIB_AtanYXShiftedAsm(f16ValY, f16ValX, pudtAtanYXCoeff)

#define GFLIB_Sgn(f16Arg) GFLIB_SgnAsm(f16Arg)

#define GFLIB_Sgn2(f16Arg) GFLIB_Sgn2Asm(f16Arg)

#define GFLIB_SqrtPoly(f32Arg) GFLIB_SqrtPolyAsm(f32Arg)
#define GFLIB_SqrtIter(f32Arg) GFLIB_SqrtIterAsm(f32Arg)
#define GFLIB_SqrtShort(f32Arg) GFLIB_SqrtShortAsm(f32Arg)
#define GFLIB_SqrtFast(f32Arg) GFLIB_SqrtFastAsm(f32Arg)

#define GFLIB_Hyst(pudtHystVar) GFLIB_HystAsm(pudtHystVar)

#define GFLIB_Lut(f16Arg, pf16Table, uw16TableSize) GFLIB_V3LutAsm(f16Arg, pf16Table, uw16TableSize)

#define GFLIB_Ramp16(f16Desired, f16Actual, pudtParam) GFLIB_Ramp16Asm(f16Desired, f16Actual, pudtParam)
#define GFLIB_Ramp32(f32Desired, f32Actual, pudtParam) GFLIB_Ramp32Asm(f32Desired, f32Actual, pudtParam)

#define GFLIB_DynRamp16InitVal(f16InitVal, pudtParam) GFLIB_DynRamp16InitValAsm(f16InitVal, pudtParam);
#define GFLIB_DynRamp16(f16Desired, f16Instant, uw16SatFlag, pudtParam) GFLIB_DynRamp16Asm(f16Desired, f16Instant, uw16SatFlag, pudtParam)

#define GFLIB_DynRamp32InitVal(f32InitVal, pudtParam) GFLIB_DynRamp32InitValAsm(f32InitVal, pudtParam);
#define GFLIB_DynRamp32(f32Desired, f32Instant, uw16SatFlag, pudtParam) GFLIB_DynRamp32Asm(f32Desired, f32Instant, uw16SatFlag, pudtParam)

#define GFLIB_Limit16(f16Arg, pudtLimit) GFLIB_Limit16Asmi(f16Arg, pudtLimit)
#define GFLIB_Limit32(f32Arg, pudtLimit) GFLIB_Limit32Asmi(f32Arg, pudtLimit)
#define GFLIB_UpperLimit16(f16Arg, f16UpperLimit) GFLIB_UpperLimit16Asmi(f16Arg, f16UpperLimit)
#define GFLIB_UpperLimit32(f32Arg, f32UpperLimit) GFLIB_UpperLimit32Asmi(f32Arg, f32UpperLimit)
#define GFLIB_LowerLimit16(f16Arg, f16LowerLimit) GFLIB_LowerLimit16Asmi(f16Arg, f16LowerLimit)
#define GFLIB_LowerLimit32(f32Arg, f32LowerLimit) GFLIB_LowerLimit32Asmi(f32Arg, f32LowerLimit)

#define GFLIB_ControllerPIr(f16Error, pudtCtrl) GFLIB_ControllerPIRecurrentAsm(f16Error, pudtCtrl)

#define GFLIB_ControllerPIrLim(f16Error, pudtCtrl) GFLIB_ControllerPIRecurrentLimAsm(f16Error, pudtCtrl)

#define GFLIB_ControllerPIpInitVal(f16InitVal, pudtPidParams) GFLIB_ControllerPIpInitValAsm(f16InitVal, pudtPidParams)
#define GFLIB_ControllerPIp(f16InputErrorK, pPiParams, pi16SatFlag) GFLIB_ControllerPIpAsm(f16InputErrorK, pPiParams, pi16SatFlag)

#define GFLIB_ControllerPIDpInitVal(f16InitVal, pudtPidParams) GFLIB_ControllerPIDpInitValAsm(f16InitVal, pudtPidParams)
#define GFLIB_ControllerPIDp(f16InputErrorK, f16InputDErrorK, pPiParams, pSatFlag, pf16InputDErrorK_1) GFLIB_ControllerPIDpAsm(f16InputErrorK, f16InputDErrorK, pPiParams, pSatFlag, pf16InputDErrorK_1)

#define GFLIB_ControllerPIDr(f16Error, pudtCtrl) GFLIB_ControllerPIDRecurrentAsm(f16Error, pudtCtrl)

#else /* V3 core instructions not used */

/* redeclaration of the functions */
#define GFLIB_SinLut(f16Arg) GFLIB_SinLutAsm(f16Arg)
#define GFLIB_CosLut(f16Arg) GFLIB_CosLutAsm(f16Arg)
#define GFLIB_SinLut2(f16Arg, udtSinTable) GFLIB_SinLut2Asm(f16Arg, udtSinTable)
#define GFLIB_CosLut2(f16Arg, udtSinTable) GFLIB_CosLut2Asm(f16Arg, udtSinTable)

#define GFLIB_SinTlr(f16In) GFLIB_SinTlrAsm(f16In)
#define GFLIB_CosTlr(f16In) GFLIB_CosTlrAsm(f16In)

#define GFLIB_Sin12Tlr(f16In) GFLIB_Sin12TlrAsm(f16In)
#define GFLIB_Cos12Tlr(f16In) GFLIB_Cos12TlrAsm(f16In)


#define GFLIB_Tan(f16Arg) GFLIB_TanAsm(f16Arg)

#define GFLIB_Asin(f16Arg)   GFLIB_AsinAsm(f16Arg)
#define GFLIB_Acos(f16Arg)   GFLIB_AcosAsm(f16Arg)

#define GFLIB_Atan(f16Arg)   GFLIB_AtanAsm(f16Arg)

#define GFLIB_AtanYX(f16ValY, f16ValX, pi16ErrFlag) GFLIB_AtanYXAsm(f16ValY, f16ValX, pi16ErrFlag)

#define GFLIB_AtanYXShifted(f16ValY, f16ValX, pudtAtanYXCoeff) GFLIB_AtanYXShiftedAsm(f16ValY, f16ValX, pudtAtanYXCoeff)

#define GFLIB_Sgn(f16Arg) GFLIB_SgnAsm(f16Arg)

#define GFLIB_Sgn2(f16Arg) GFLIB_Sgn2Asm(f16Arg)

#define GFLIB_SqrtPoly(f32Arg) GFLIB_SqrtPolyAsm(f32Arg)
#define GFLIB_SqrtIter(f32Arg) GFLIB_SqrtIterAsm(f32Arg)
#define GFLIB_SqrtShort(f32Arg) GFLIB_SqrtShortAsm(f32Arg)
#define GFLIB_SqrtFast(f32Arg) GFLIB_SqrtFastAsm(f32Arg)

#define GFLIB_Hyst(pudtHystVar) GFLIB_HystAsm(pudtHystVar)

#define GFLIB_Lut(f16Arg, pf16Table, uw16TableSize) GFLIB_LutAsm(f16Arg, pf16Table, uw16TableSize)

#define GFLIB_Ramp16(f16Desired, f16Actual, pudtParam) GFLIB_Ramp16Asm(f16Desired, f16Actual, pudtParam)
#define GFLIB_Ramp32(f32Desired, f32Actual, pudtParam) GFLIB_Ramp32Asm(f32Desired, f32Actual, pudtParam)

#define GFLIB_DynRamp16InitVal(f16InitVal, pudtParam) GFLIB_DynRamp16InitValAsm(f16InitVal, pudtParam)
#define GFLIB_DynRamp16(f16Desired, f16Instant, uw16SatFlag, pudtParam) GFLIB_DynRamp16Asm(f16Desired, f16Instant, uw16SatFlag, pudtParam)

#define GFLIB_DynRamp32InitVal(f32InitVal, pudtParam) GFLIB_DynRamp32InitValAsm(f32InitVal, pudtParam)
#define GFLIB_DynRamp32(f32Desired, f32Instant, uw16SatFlag, pudtParam) GFLIB_DynRamp32Asm(f32Desired, f32Instant, uw16SatFlag, pudtParam)

#define GFLIB_Limit16(f16Arg, pudtLimit) GFLIB_Limit16Asmi(f16Arg, pudtLimit)
#define GFLIB_Limit32(f32Arg, pudtLimit) GFLIB_Limit32Asmi(f32Arg, pudtLimit)
#define GFLIB_UpperLimit16(f16Arg, f16UpperLimit) GFLIB_UpperLimit16Asmi(f16Arg, f16UpperLimit)
#define GFLIB_UpperLimit32(f32Arg, f32UpperLimit) GFLIB_UpperLimit32Asmi(f32Arg, f32UpperLimit)
#define GFLIB_LowerLimit16(f16Arg, f16LowerLimit) GFLIB_LowerLimit16Asmi(f16Arg, f16LowerLimit)
#define GFLIB_LowerLimit32(f32Arg, f32LowerLimit) GFLIB_LowerLimit32Asmi(f32Arg, f32LowerLimit)

#define GFLIB_ControllerPIr(f16Error, pudtCtrl) GFLIB_ControllerPIRecurrentAsm(f16Error, pudtCtrl)

#define GFLIB_ControllerPIrLim(f16Error, pudtCtrl) GFLIB_ControllerPIRecurrentLimAsm(f16Error, pudtCtrl)

#define GFLIB_ControllerPIpInitVal(f16InitVal, pudtPidParams) GFLIB_ControllerPIpInitValAsm(f16InitVal, pudtPidParams)
#define GFLIB_ControllerPIp(f16InputErrorK, pPiParams, pi16SatFlag) GFLIB_ControllerPIpAsm(f16InputErrorK, pPiParams, pi16SatFlag)

#define GFLIB_ControllerPIDpInitVal(f16InitVal, pudtPidParams) GFLIB_ControllerPIDpInitValAsm(f16InitVal, pudtPidParams)
#define GFLIB_ControllerPIDp(f16InputErrorK, f16InputDErrorK, pPiParams, pSatFlag, pf16InputDErrorK_1) GFLIB_ControllerPIDpAsm(f16InputErrorK, f16InputDErrorK, pPiParams, pSatFlag, pf16InputDErrorK_1)

#define GFLIB_ControllerPIDr(f16Error, pudtCtrl) GFLIB_ControllerPIDRecurrentAsm(f16Error, pudtCtrl)

#endif

#endif /* _GFLIB_H_ */

