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
* @file      mlib.h
*
* @author    r61928
* 
* @version   1.0.2.0
* 
* @date      Jan-8-2014
* 
* @brief     MLIB include file
*
*******************************************************************************
*
* Functions that are included in MLIB.
*
******************************************************************************/
#ifndef _MLIB_H_
#define _MLIB_H_

#include "56800E_types.h"

#include "MLIB_AbsAsm.h"
#include "MLIB_NegAsm.h"
#include "MLIB_ShiftAsm.h"
#include "MLIB_AddAsm.h"
#include "MLIB_SubAsm.h"
#include "MLIB_Mul16Asm.h"
#include "MLIB_MulRnd16Asm.h"
#include "MLIB_Mul24Asm.h"
#include "MLIB_Mul32Asm.h"
#include "MLIB_Mac16Asm.h"
#include "MLIB_MacRnd16Asm.h"
#include "MLIB_Mac24Asm.h"
#include "MLIB_Mac32Asm.h"
#include "MLIB_DivAsm.h"
#include "MLIB_RoundAsm.h"

#if OPTION_CORE_V3 == 1 /* V3 core instructions used */

/* Redeclaration of MLIB function */

#define MLIB_Abs16(f16In) MLIB_Abs16Asmi(f16In)
#define MLIB_Abs16Sat(f16In) MLIB_Abs16SatAsmi(f16In)
#define MLIB_Abs32(f32In) MLIB_Abs32Asmi(f32In)
#define MLIB_Abs32Sat(f32In) MLIB_Abs32SatAsmi(f32In)

#define MLIB_Neg16(f16In) MLIB_Neg16Asmi(f16In)
#define MLIB_Neg16Sat(f16In) MLIB_Neg16SatAsmi(f16In)
#define MLIB_Neg32(f32In) MLIB_Neg32Asmi(f32In)
#define MLIB_Neg32Sat(f32In) MLIB_Neg32SatAsmi(f32In)

#define MLIB_Sh1L16(f16In) MLIB_Sh1L16Asmi(f16In)
#define MLIB_Sh1L16Sat(f16In) MLIB_Sh1L16SatAsmi(f16In)
#define MLIB_Sh1R16(f16In) MLIB_Sh1R16Asmi(f16In)
#define MLIB_Sh1L32(f32In) MLIB_Sh1L32Asmi(f32In)
#define MLIB_Sh1L32Sat(f32In) MLIB_Sh1L32SatAsmi(f32In)
#define MLIB_Sh1R32(f32In) MLIB_Sh1R32Asmi(f32In)
#define MLIB_ShL16(f16In, w16N) MLIB_ShL16Asmi(f16In, w16N)
#define MLIB_ShL16Sat(f16In, w16N) MLIB_ShL16SatAsmi(f16In, w16N)
#define MLIB_ShR16(f16In, w16N) MLIB_ShR16Asmi(f16In, w16N)
#define MLIB_ShR16Sat(f16In, w16N) MLIB_ShR16SatAsmi(f16In, w16N)
#define MLIB_ShL32(f32In, w16N) MLIB_ShL32Asmi(f32In, w16N)
#define MLIB_ShL32Sat(f32In, w16N) MLIB_ShL32SatAsmi(f32In, w16N)
#define MLIB_ShR32(f32In, w16N) MLIB_ShR32Asmi(f32In, w16N)
#define MLIB_ShR32Sat(f32In, w16N) MLIB_ShR32SatAsmi(f32In, w16N)

#define MLIB_Add16(f16In1, f16In2) MLIB_Add16Asmi(f16In1, f16In2)
#define MLIB_Add16Sat(f16In1, f16In2) MLIB_Add16SatAsmi(f16In1, f16In2)
#define MLIB_Add32(f32In1, f32In2) MLIB_Add32Asmi(f32In1, f32In2)
#define MLIB_Add32Sat(f32In1, f32In2) MLIB_Add32SatAsmi(f32In1, f32In2)

#define MLIB_Sub16(f16In1, f16In2) MLIB_Sub16Asmi(f16In1, f16In2)
#define MLIB_Sub16Sat(f16In1, f16In2) MLIB_Sub16SatAsmi(f16In1, f16In2)
#define MLIB_Sub32(f32In1, f32In2) MLIB_Sub32Asmi(f32In1, f32In2)
#define MLIB_Sub32Sat(f32In1, f32In2) MLIB_Sub32SatAsmi(f32In1, f32In2)

#define MLIB_Mul16SS(f16In1, f16In2) MLIB_Mul16SSAsmi(f16In1, f16In2)
#define MLIB_Mul16SSSat(f16In1, f16In2) MLIB_Mul16SSSatAsmi(f16In1, f16In2)
#define MLIB_Mul32SS(f16In1, f16In2) MLIB_Mul32SSAsmi(f16In1, f16In2)
#define MLIB_Mul32SSSat(f16In1, f16In2) MLIB_Mul32SSSatAsmi(f16In1, f16In2)
#define MLIB_MulNeg16SS(f16In1, f16In2) MLIB_MulNeg16SSAsmi(f16In1, f16In2)
#define MLIB_MulNeg32SS(f16In1, f16In2) MLIB_MulNeg32SSAsmi(f16In1, f16In2)

#define MLIB_MulRnd16SS(f16In1, f16In2) MLIB_MulRnd16SSAsmi(f16In1, f16In2)
#define MLIB_MulRnd16SSSat(f16In1, f16In2) MLIB_MulRnd16SSSatAsmi(f16In1, f16In2)
#define MLIB_MulRnd32SS(f16In1, f16In2) MLIB_MulRnd32SSAsmi(f16In1, f16In2)
#define MLIB_MulRnd32SSSat(f16In1, f16In2) MLIB_MulRnd32SSSatAsmi(f16In1, f16In2)
#define MLIB_MulNegRnd16SS(f16In1, f16In2) MLIB_MulNegRnd16SSAsmi(f16In1, f16In2)
#define MLIB_MulNegRnd16SSSat(f16In1, f16In2) MLIB_MulNegRnd16SSSatAsmi(f16In1, f16In2)
#define MLIB_MulNegRnd32SS(f16In1, f16In2) MLIB_MulNegRnd32SSAsmi(f16In1, f16In2)
#define MLIB_MulNegRnd32SSSat(f16In1, f16In2) MLIB_MulNegRnd32SSSatAsmi(f16In1, f16In2)

#define MLIB_Mul32LS(f32In1, f16In2) MLIB_V3Mul32LSAsmi(f32In1, f16In2)
#define MLIB_Mul32LSSat(f32In1, f16In2) MLIB_V3Mul32LSSatAsmi(f32In1, f16In2)
#define MLIB_MulNeg32LS(f32In1, f16In2) MLIB_V3MulNeg32LSAsmi(f32In1, f16In2)

#define MLIB_Mul32LL(f32In1, f32In2) MLIB_V3Mul32LLAsmi(f32In1, f32In2)
#define MLIB_Mul32LLSat(f32In1, f32In2) MLIB_V3Mul32LLSatAsmi(f32In1, f32In2)
#define MLIB_MulNeg32LL(f32In1, f32In2) MLIB_V3MulNeg32LLAsmi(f32In1, f32In2)

#define MLIB_Mac16SSS(f16Acc, f16In1, f16In2) MLIB_Mac16SSSAsmi(f16Acc, f16In1, f16In2)
#define MLIB_Mac16SSSSat(f16Acc, f16In1, f16In2) MLIB_Mac16SSSSatAsmi(f16Acc, f16In1, f16In2)
#define MLIB_Mac32LSS(f32Acc, f16In1, f16In2) MLIB_Mac32LSSAsmi(f32Acc, f16In1, f16In2)
#define MLIB_Mac32LSSSat(f32Acc, f16In1, f16In2) MLIB_Mac32LSSSatAsmi(f32Acc, f16In1, f16In2)
#define MLIB_Msu16SSS(f16Acc, f16In1, f16In2) MLIB_Msu16SSSAsmi(f16Acc, f16In1, f16In2)
#define MLIB_Msu16SSSSat(f16Acc, f16In1, f16In2) MLIB_Msu16SSSSatAsmi(f16Acc, f16In1, f16In2)
#define MLIB_Msu32LSS(f32Acc, f16In1, f16In2) MLIB_Msu32LSSAsmi(f32Acc, f16In1, f16In2)
#define MLIB_Msu32LSSSat(f32Acc, f16In1, f16In2) MLIB_Msu32LSSSatAsmi(f32Acc, f16In1, f16In2)

#define MLIB_MacRnd16SSS(f16Acc, f16In1, f16In2) MLIB_MacRnd16SSSAsmi(f16Acc, f16In1, f16In2)
#define MLIB_MacRnd16SSSSat(f16Acc, f16In1, f16In2) MLIB_MacRnd16SSSSatAsmi(f16Acc, f16In1, f16In2)
#define MLIB_MacRnd32LSS(f32Acc, f16In1, f16In2) MLIB_MacRnd32LSSAsmi(f32Acc, f16In1, f16In2)
#define MLIB_MacRnd32LSSSat(f32Acc, f16In1, f16In2) MLIB_MacRnd32LSSSatAsmi(f32Acc, f16In1, f16In2)
#define MLIB_MsuRnd16SSS(f16Acc, f16In1, f16In2) MLIB_MsuRnd16SSSAsmi(f16Acc, f16In1, f16In2)
#define MLIB_MsuRnd16SSSSat(f16Acc, f16In1, f16In2) MLIB_MsuRnd16SSSSatAsmi(f16Acc, f16In1, f16In2)
#define MLIB_MsuRnd32LSS(f32Acc, f16In1, f16In2) MLIB_MsuRnd32LSSAsmi(f32Acc, f16In1, f16In2)
#define MLIB_MsuRnd32LSSSat(f32Acc, f16In1, f16In2) MLIB_MsuRnd32LSSSatAsmi(f32Acc, f16In1, f16In2)

#define MLIB_Mac32LLS(f32Acc, f32In1, f16In2) MLIB_V3Mac32LLSAsmi(f32Acc, f32In1, f16In2)
#define MLIB_Mac32LLSSat(f32Acc, f32In1, f16In2) MLIB_V3Mac32LLSSatAsmi(f32Acc, f32In1, f16In2)
#define MLIB_Msu32LLS(f32Acc, f32In1, f16In2) MLIB_V3Msu32LLSAsmi(f32Acc, f32In1, f16In2)
#define MLIB_Msu32LLSSat(f32Acc, f32In1, f16In2) MLIB_V3Msu32LLSSatAsmi(f32Acc, f32In1, f16In2)

#define MLIB_Mac32LLL(f32Acc, f32In1, f32In2) MLIB_V3Mac32LLLAsmi(f32Acc, f32In1, f32In2)
#define MLIB_Mac32LLLSat(f32Acc, f32In1, f32In2) MLIB_V3Mac32LLLSatAsmi(f32Acc, f32In1, f32In2)
#define MLIB_Msu32LLL(f32Acc, f32In1, f32In2) MLIB_V3Msu32LLLAsmi(f32Acc, f32In1, f32In2)
#define MLIB_Msu32LLLSat(f32Acc, f32In1, f32In2) MLIB_V3Msu32LLLSatAsmi(f32Acc, f32In1, f32In2)

#define MLIB_Div1Q16SS(f16Num, f16Denom) MLIB_Div1Q16SSAsmi(f16Num, f16Denom)
#define MLIB_Div4Q16SS(f16Num, f16Denom) MLIB_Div4Q16SSAsmi(f16Num, f16Denom)
#define MLIB_Div1Q16LS(f32Num, f16Denom) MLIB_Div1Q16LSAsmi(f32Num, f16Denom)
#define MLIB_Div4Q16LS(f32Num, f16Denom) MLIB_Div4Q16LSAsmi(f32Num, f16Denom)
#define MLIB_Div1Q32LS(f32Num, f16Denom) MLIB_Div1Q32LSAsmi(f32Num, f16Denom)
#define MLIB_Div4Q32LS(f32Num, f16Denom) MLIB_Div4Q32LSAsmi(f32Num, f16Denom)
#define MLIB_Div1Q32LL(f32Num, f32Denom) MLIB_Div1Q32LLAsmi(f32Num, f32Denom)
#define MLIB_Div4Q32LL(f32Num, f32Denom) MLIB_Div4Q32LLAsmi(f32Num, f32Denom)

#define MLIB_Rcp161Q(f16Denom) MLIB_Rcp161QAsmi(f16Denom)
#define MLIB_Rcp164Q(f16Denom) MLIB_Rcp164QAsmi(f16Denom)
#define MLIB_Rcp321Q(f16Denom) MLIB_Rcp321QAsmi(f16Denom)
#define MLIB_Rcp324Q(f16Denom) MLIB_Rcp324QAsmi(f16Denom)

#define MLIB_Rnd16(f32In) MLIB_Rnd16Asmi(f32In)
#define MLIB_Rnd16Sat(f32In) MLIB_Rnd16SatAsmi(f32In)
#define MLIB_Rnd32(f32In) MLIB_Rnd32Asmi(f32In)
#define MLIB_Rnd32Sat(f32In) MLIB_Rnd32SatAsmi(f32In)

#else /* V3 core instructions not used */

/* Redeclaration of MLIB function */

#define MLIB_Abs16(f16In) MLIB_Abs16Asmi(f16In)
#define MLIB_Abs16Sat(f16In) MLIB_Abs16SatAsmi(f16In)
#define MLIB_Abs32(f32In) MLIB_Abs32Asmi(f32In)
#define MLIB_Abs32Sat(f32In) MLIB_Abs32SatAsmi(f32In)

#define MLIB_Neg16(f16In) MLIB_Neg16Asmi(f16In)
#define MLIB_Neg16Sat(f16In) MLIB_Neg16SatAsmi(f16In)
#define MLIB_Neg32(f32In) MLIB_Neg32Asmi(f32In)
#define MLIB_Neg32Sat(f32In) MLIB_Neg32SatAsmi(f32In)

#define MLIB_Sh1L16(f16In) MLIB_Sh1L16Asmi(f16In)
#define MLIB_Sh1L16Sat(f16In) MLIB_Sh1L16SatAsmi(f16In)
#define MLIB_Sh1R16(f16In) MLIB_Sh1R16Asmi(f16In)
#define MLIB_Sh1L32(f32In) MLIB_Sh1L32Asmi(f32In)
#define MLIB_Sh1L32Sat(f32In) MLIB_Sh1L32SatAsmi(f32In)
#define MLIB_Sh1R32(f32In) MLIB_Sh1R32Asmi(f32In)
#define MLIB_ShL16(f16In, w16N) MLIB_ShL16Asmi(f16In, w16N)
#define MLIB_ShL16Sat(f16In, w16N) MLIB_ShL16SatAsmi(f16In, w16N)
#define MLIB_ShR16(f16In, w16N) MLIB_ShR16Asmi(f16In, w16N)
#define MLIB_ShR16Sat(f16In, w16N) MLIB_ShR16SatAsmi(f16In, w16N)
#define MLIB_ShL32(f32In, w16N) MLIB_ShL32Asmi(f32In, w16N)
#define MLIB_ShL32Sat(f32In, w16N) MLIB_ShL32SatAsmi(f32In, w16N)
#define MLIB_ShR32(f32In, w16N) MLIB_ShR32Asmi(f32In, w16N)
#define MLIB_ShR32Sat(f32In, w16N) MLIB_ShR32SatAsmi(f32In, w16N)

#define MLIB_Add16(f16In1, f16In2) MLIB_Add16Asmi(f16In1, f16In2)
#define MLIB_Add16Sat(f16In1, f16In2) MLIB_Add16SatAsmi(f16In1, f16In2)
#define MLIB_Add32(f32In1, f32In2) MLIB_Add32Asmi(f32In1, f32In2)
#define MLIB_Add32Sat(f32In1, f32In2) MLIB_Add32SatAsmi(f32In1, f32In2)

#define MLIB_Sub16(f16In1, f16In2) MLIB_Sub16Asmi(f16In1, f16In2)
#define MLIB_Sub16Sat(f16In1, f16In2) MLIB_Sub16SatAsmi(f16In1, f16In2)
#define MLIB_Sub32(f32In1, f32In2) MLIB_Sub32Asmi(f32In1, f32In2)
#define MLIB_Sub32Sat(f32In1, f32In2) MLIB_Sub32SatAsmi(f32In1, f32In2)

#define MLIB_Mul16SS(f16In1, f16In2) MLIB_Mul16SSAsmi(f16In1, f16In2)
#define MLIB_Mul16SSSat(f16In1, f16In2) MLIB_Mul16SSSatAsmi(f16In1, f16In2)
#define MLIB_Mul32SS(f16In1, f16In2) MLIB_Mul32SSAsmi(f16In1, f16In2)
#define MLIB_Mul32SSSat(f16In1, f16In2) MLIB_Mul32SSSatAsmi(f16In1, f16In2)
#define MLIB_MulNeg16SS(f16In1, f16In2) MLIB_MulNeg16SSAsmi(f16In1, f16In2)
#define MLIB_MulNeg32SS(f16In1, f16In2) MLIB_MulNeg32SSAsmi(f16In1, f16In2)

#define MLIB_MulRnd16SS(f16In1, f16In2) MLIB_MulRnd16SSAsmi(f16In1, f16In2)
#define MLIB_MulRnd16SSSat(f16In1, f16In2) MLIB_MulRnd16SSSatAsmi(f16In1, f16In2)
#define MLIB_MulRnd32SS(f16In1, f16In2) MLIB_MulRnd32SSAsmi(f16In1, f16In2)
#define MLIB_MulRnd32SSSat(f16In1, f16In2) MLIB_MulRnd32SSSatAsmi(f16In1, f16In2)
#define MLIB_MulNegRnd16SS(f16In1, f16In2) MLIB_MulNegRnd16SSAsmi(f16In1, f16In2)
#define MLIB_MulNegRnd16SSSat(f16In1, f16In2) MLIB_MulNegRnd16SSSatAsmi(f16In1, f16In2)
#define MLIB_MulNegRnd32SS(f16In1, f16In2) MLIB_MulNegRnd32SSAsmi(f16In1, f16In2)
#define MLIB_MulNegRnd32SSSat(f16In1, f16In2) MLIB_MulNegRnd32SSSatAsmi(f16In1, f16In2)

#define MLIB_Mul32LS(f32In1, f16In2) MLIB_Mul32LSAsmi(f32In1, f16In2)
#define MLIB_Mul32LSSat(f32In1, f16In2) MLIB_Mul32LSSatAsmi(f32In1, f16In2)
#define MLIB_MulNeg32LS(f32In1, f16In2) MLIB_MulNeg32LSAsmi(f32In1, f16In2)

#define MLIB_Mul32LL(f32In1, f32In2) MLIB_Mul32LLAsmi(f32In1, f32In2)
#define MLIB_Mul32LLSat(f32In1, f32In2) MLIB_Mul32LLSatAsmi(f32In1, f32In2)
#define MLIB_MulNeg32LL(f32In1, f32In2) MLIB_MulNeg32LLAsmi(f32In1, f32In2)

#define MLIB_Mac16SSS(f16Acc, f16In1, f16In2) MLIB_Mac16SSSAsmi(f16Acc, f16In1, f16In2)
#define MLIB_Mac16SSSSat(f16Acc, f16In1, f16In2) MLIB_Mac16SSSSatAsmi(f16Acc, f16In1, f16In2)
#define MLIB_Mac32LSS(f32Acc, f16In1, f16In2) MLIB_Mac32LSSAsmi(f32Acc, f16In1, f16In2)
#define MLIB_Mac32LSSSat(f32Acc, f16In1, f16In2) MLIB_Mac32LSSSatAsmi(f32Acc, f16In1, f16In2)
#define MLIB_Msu16SSS(f16Acc, f16In1, f16In2) MLIB_Msu16SSSAsmi(f16Acc, f16In1, f16In2)
#define MLIB_Msu16SSSSat(f16Acc, f16In1, f16In2) MLIB_Msu16SSSSatAsmi(f16Acc, f16In1, f16In2)
#define MLIB_Msu32LSS(f32Acc, f16In1, f16In2) MLIB_Msu32LSSAsmi(f32Acc, f16In1, f16In2)
#define MLIB_Msu32LSSSat(f32Acc, f16In1, f16In2) MLIB_Msu32LSSSatAsmi(f32Acc, f16In1, f16In2)

#define MLIB_MacRnd16SSS(f16Acc, f16In1, f16In2) MLIB_MacRnd16SSSAsmi(f16Acc, f16In1, f16In2)
#define MLIB_MacRnd16SSSSat(f16Acc, f16In1, f16In2) MLIB_MacRnd16SSSSatAsmi(f16Acc, f16In1, f16In2)
#define MLIB_MacRnd32LSS(f32Acc, f16In1, f16In2) MLIB_MacRnd32LSSAsmi(f32Acc, f16In1, f16In2)
#define MLIB_MacRnd32LSSSat(f32Acc, f16In1, f16In2) MLIB_MacRnd32LSSSatAsmi(f32Acc, f16In1, f16In2)
#define MLIB_MsuRnd16SSS(f16Acc, f16In1, f16In2) MLIB_MsuRnd16SSSAsmi(f16Acc, f16In1, f16In2)
#define MLIB_MsuRnd16SSSSat(f16Acc, f16In1, f16In2) MLIB_MsuRnd16SSSSatAsmi(f16Acc, f16In1, f16In2)
#define MLIB_MsuRnd32LSS(f32Acc, f16In1, f16In2) MLIB_MsuRnd32LSSAsmi(f32Acc, f16In1, f16In2)
#define MLIB_MsuRnd32LSSSat(f32Acc, f16In1, f16In2) MLIB_MsuRnd32LSSSatAsmi(f32Acc, f16In1, f16In2)

#define MLIB_Mac32LLS(f32Acc, f32In1, f16In2) MLIB_Mac32LLSAsmi(f32Acc, f32In1, f16In2)
#define MLIB_Mac32LLSSat(f32Acc, f32In1, f16In2) MLIB_Mac32LLSSatAsmi(f32Acc, f32In1, f16In2)
#define MLIB_Msu32LLS(f32Acc, f32In1, f16In2) MLIB_Msu32LLSAsmi(f32Acc, f32In1, f16In2)
#define MLIB_Msu32LLSSat(f32Acc, f32In1, f16In2) MLIB_Msu32LLSSatAsmi(f32Acc, f32In1, f16In2)

#define MLIB_Mac32LLL(f32Acc, f32In1, f32In2) MLIB_Mac32LLLAsmi(f32Acc, f32In1, f32In2)
#define MLIB_Mac32LLLSat(f32Acc, f32In1, f32In2) MLIB_Mac32LLLSatAsmi(f32Acc, f32In1, f32In2)
#define MLIB_Msu32LLL(f32Acc, f32In1, f32In2) MLIB_Msu32LLLAsmi(f32Acc, f32In1, f32In2)
#define MLIB_Msu32LLLSat(f32Acc, f32In1, f32In2) MLIB_Msu32LLLSatAsmi(f32Acc, f32In1, f32In2)

#define MLIB_Div1Q16SS(f16Num, f16Denom) MLIB_Div1Q16SSAsmi(f16Num, f16Denom)
#define MLIB_Div4Q16SS(f16Num, f16Denom) MLIB_Div4Q16SSAsmi(f16Num, f16Denom)
#define MLIB_Div1Q16LS(f32Num, f16Denom) MLIB_Div1Q16LSAsmi(f32Num, f16Denom)
#define MLIB_Div4Q16LS(f32Num, f16Denom) MLIB_Div4Q16LSAsmi(f32Num, f16Denom)
#define MLIB_Div1Q32LS(f32Num, f16Denom) MLIB_Div1Q32LSAsmi(f32Num, f16Denom)
#define MLIB_Div4Q32LS(f32Num, f16Denom) MLIB_Div4Q32LSAsmi(f32Num, f16Denom)
#define MLIB_Div1Q32LL(f32Num, f32Denom) MLIB_Div1Q32LLAsmi(f32Num, f32Denom)
#define MLIB_Div4Q32LL(f32Num, f32Denom) MLIB_Div4Q32LLAsmi(f32Num, f32Denom)

#define MLIB_Rcp161Q(f16Denom) MLIB_Rcp161QAsmi(f16Denom)
#define MLIB_Rcp164Q(f16Denom) MLIB_Rcp164QAsmi(f16Denom)
#define MLIB_Rcp321Q(f16Denom) MLIB_Rcp321QAsmi(f16Denom)
#define MLIB_Rcp324Q(f16Denom) MLIB_Rcp324QAsmi(f16Denom)

#define MLIB_Rnd16(f32In) MLIB_Rnd16Asmi(f32In)
#define MLIB_Rnd16Sat(f32In) MLIB_Rnd16SatAsmi(f32In)
#define MLIB_Rnd32(f32In) MLIB_Rnd32Asmi(f32In)
#define MLIB_Rnd32Sat(f32In) MLIB_Rnd32SatAsmi(f32In)

#endif



#endif

