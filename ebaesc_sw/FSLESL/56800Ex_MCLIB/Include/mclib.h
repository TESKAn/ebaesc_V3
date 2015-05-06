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
* @file      mclib.h
*
* @author    r61928
* 
* @version   1.0.2.0
* 
* @date      Apr-23-2013
* 
* @brief     MCLIB include file
*
*******************************************************************************
*
* Functions that are included in MCLIB.
*
******************************************************************************/
#ifndef _MCLIB_H_
#define _MCLIB_H_

#include "56800E_types.h"
#include "MCLIB_types.h"

#include "MCLIB_CPTrfAsm.h"
#include "MCLIB_DecouplingAsm.h"
#include "MCLIB_ElimDcBusRipAsm.h"
#include "MCLIB_SvmAsm.h"
#include "MCLIB_VectorLimitAsm.h"

#if OPTION_CORE_V3 == 1 /* V3 core instructions used */

/* Redeclaration of MCLIB function */
#define	MCLIB_SvmStd(pudtAlphaBeta, pudtAbc)		MCLIB_SvmStdAsm(pudtAlphaBeta, pudtAbc)
#define	MCLIB_SvmU0n(pudtAlphaBeta, pudtAbc)		MCLIB_SvmU0nAsm(pudtAlphaBeta, pudtAbc)
#define	MCLIB_SvmU7n(pudtAlphaBeta, pudtAbc)		MCLIB_SvmU7nAsm(pudtAlphaBeta, pudtAbc)
#define	MCLIB_SvmAlt(pudtAlphaBeta, pudtAbc)		MCLIB_SvmAltAsm(pudtAlphaBeta, pudtAbc)
#define	MCLIB_PwmIct(pudtAlphaBeta, pudtAbc)		MCLIB_PwmIctAsm(pudtAlphaBeta, pudtAbc)
#define	MCLIB_SvmSci(pudtAlphaBeta, pudtAbc)		MCLIB_SvmSciAsm(pudtAlphaBeta, pudtAbc)

#define	MCLIB_ClarkTrf(pudtAlphaBeta, pudtAbc) MCLIB_ClarkTrfAsm(pudtAlphaBeta, pudtAbc)
#define MCLIB_ClarkTrfInv(pudtAbc, pudtAlphaBeta) MCLIB_ClarkTrfInvAsm(pudtAbc, pudtAlphaBeta)
#define MCLIB_ParkTrf(pudtDQ, pudtAlphaBeta, pudtSinCos) MCLIB_ParkTrfAsm(pudtDQ, pudtAlphaBeta, pudtSinCos)
#define MCLIB_ParkTrfInv(pudtAlphaBeta, pudtDQ, pudtSinCos) MCLIB_ParkTrfInvAsm(pudtAlphaBeta, pudtDQ, pudtSinCos)

#define MCLIB_DecouplingPMSM(pudtUs, pudtIs, f16AngularVelocity, pudtDecParam, pudtUsDec) \
			MCLIB_V3DecouplingPMSMAsm(pudtUs, pudtIs, f16AngularVelocity, pudtDecParam, pudtUsDec)

#define MCLIB_ElimDcBusRip(f16InvModIndex, f16DcBusMsr, pudtInAlphaBeta, pudtOutAlphaBeta) \
			MCLIB_ElimDcBusRipAsm(f16InvModIndex, f16DcBusMsr, pudtInAlphaBeta, pudtOutAlphaBeta)

#define MCLIB_ElimDcBusRipGen(f16DcBusMsr, pudtInAlphaBeta, pudtOutAlphaBeta) \
			MCLIB_ElimDcBusRipGenAsm(f16DcBusMsr, pudtInAlphaBeta, pudtOutAlphaBeta)


#define	MCLIB_VectorLimit(pudtInVector, pudtLimVector, pudtParams) \
			MCLIB_VectorLimitAsm(pudtInVector, pudtLimVector, pudtParams)

/* Faster algorithms using 12-bit precision functions inside (Sin, Sqrt...)*/
#define	MCLIB_VectorLimit12(pudtInVector, pudtLimVector, pudtParams) \
			MCLIB_VectorLimit12Asm(pudtInVector, pudtLimVector, pudtParams)

#else /* V3 core instructions not used */

/* Redeclaration of MCLIB function */
#define	MCLIB_SvmStd(pudtAlphaBeta, pudtAbc)		MCLIB_SvmStdAsm(pudtAlphaBeta, pudtAbc)
#define	MCLIB_SvmU0n(pudtAlphaBeta, pudtAbc)		MCLIB_SvmU0nAsm(pudtAlphaBeta, pudtAbc)
#define	MCLIB_SvmU7n(pudtAlphaBeta, pudtAbc)		MCLIB_SvmU7nAsm(pudtAlphaBeta, pudtAbc)
#define	MCLIB_SvmAlt(pudtAlphaBeta, pudtAbc)		MCLIB_SvmAltAsm(pudtAlphaBeta, pudtAbc)
#define	MCLIB_PwmIct(pudtAlphaBeta, pudtAbc)		MCLIB_PwmIctAsm(pudtAlphaBeta, pudtAbc)
#define	MCLIB_SvmSci(pudtAlphaBeta, pudtAbc)		MCLIB_SvmSciAsm(pudtAlphaBeta, pudtAbc)

#define	MCLIB_ClarkTrf(pudtAlphaBeta, pudtAbc) MCLIB_ClarkTrfAsm(pudtAlphaBeta, pudtAbc)
#define MCLIB_ClarkTrfInv(pudtAbc, pudtAlphaBeta) MCLIB_ClarkTrfInvAsm(pudtAbc, pudtAlphaBeta)
#define MCLIB_ParkTrf(pudtDQ, pudtAlphaBeta, pudtSinCos) MCLIB_ParkTrfAsm(pudtDQ, pudtAlphaBeta, pudtSinCos)
#define MCLIB_ParkTrfInv(pudtAlphaBeta, pudtDQ, pudtSinCos) MCLIB_ParkTrfInvAsm(pudtAlphaBeta, pudtDQ, pudtSinCos)

#define MCLIB_DecouplingPMSM(pudtUs, pudtIs, f16AngularVelocity, pudtDecParam, pudtUsDec) \
			MCLIB_DecouplingPMSMAsm(pudtUs, pudtIs, f16AngularVelocity, pudtDecParam, pudtUsDec)

#define MCLIB_ElimDcBusRip(f16InvModIndex, f16DcBusMsr, pudtInAlphaBeta, pudtOutAlphaBeta) \
			MCLIB_ElimDcBusRipAsm(f16InvModIndex, f16DcBusMsr, pudtInAlphaBeta, pudtOutAlphaBeta)

#define MCLIB_ElimDcBusRipGen(f16DcBusMsr, pudtInAlphaBeta, pudtOutAlphaBeta) \
			MCLIB_ElimDcBusRipGenAsm(f16DcBusMsr, pudtInAlphaBeta, pudtOutAlphaBeta)


#define	MCLIB_VectorLimit(pudtInVector, pudtLimVector, pudtParams) \
			MCLIB_VectorLimitAsm(pudtInVector, pudtLimVector, pudtParams)

/* Faster algorithms using 12-bit precision functions inside (Sin, Sqrt...)*/
#define	MCLIB_VectorLimit12(pudtInVector, pudtLimVector, pudtParams) \
			MCLIB_VectorLimit12Asm(pudtInVector, pudtLimVector, pudtParams)

#endif



#endif

