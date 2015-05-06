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
* @file      aclib.h
*
* @author    r59400
* 
* @version   1.0.2.0
* 
* @date      Apr-23-2013
* 
* @brief     ACLIB include file
*
*******************************************************************************
*
* Functions that are included in ACLIB.
*
******************************************************************************/
#ifndef _ACLIB_H_
#define _ACLIB_H_

#include "56800E_types.h"

#include "ACLIB_AngleTrackObsrvAsm.h"
#include "ACLIB_TrackObsrvAsm.h"
#include "ACLIB_PMSMBemfObsrvABAsm.h"
#include "ACLIB_PMSMBemfObsrvDQAsm.h"
#include "ACLIB_IntegratorAsm.h"

#if OPTION_CORE_V3 == 1 /* V3 core instructions used */

/* Redeclaration of ACLIB function */
#define ACLIB_AngleTrackObsrv(pudtSinCos, pudtCtrl) 	ACLIB_V3AngleTrackObsrvAsm(pudtSinCos, pudtCtrl)
#define ACLIB_TrackObsrv(f16Error, pudtCtrl) 			ACLIB_V3TrackObsrvAsm(f16Error, pudtCtrl)
#define ACLIB_PMSMBemfObsrvAB(pudtCurrentAlphaBeta, pudtVoltageAlphaBeta, f16Speed, pudtCtrl) 	ACLIB_V3PMSMBemfObsrvABAsm(pudtCurrentAlphaBeta, pudtVoltageAlphaBeta, f16Speed, pudtCtrl)
#define ACLIB_PMSMBemfObsrvDQ(pudtCurrentDQ, pudtVoltageDQ, f16Speed, pudtCtrl) 	ACLIB_V3PMSMBemfObsrvDQAsm(pudtCurrentDQ, pudtVoltageDQ, f16Speed, pudtCtrl)

#define ACLIB_IntegratorInitVal(f16InitVal, pudtIntg) ACLIB_IntegratorInitValAsm(f16InitVal, pudtIntg)
#define ACLIB_Integrator(f16Xinp, pudtIntg) 			ACLIB_IntegratorAsm(f16Xinp, pudtIntg)

/* Reduced precision but faster algorithms */
#define ACLIB_AngleTrackObsrv12(pudtSinCos, pudtCtrl) 	ACLIB_V3AngleTrackObsrv12Asm(pudtSinCos, pudtCtrl)
#define ACLIB_PMSMBemfObsrv12AB(pudtCurrentAlphaBeta, pudtVoltageAlphaBeta, f16Speed, pudtCtrl) 	ACLIB_V3PMSMBemfObsrv12ABAsm(pudtCurrentAlphaBeta, pudtVoltageAlphaBeta, f16Speed, pudtCtrl)

#else /* V3 core instructions not used */

/* Redeclaration of ACLIB function */
#define ACLIB_AngleTrackObsrv(pudtSinCos, pudtCtrl) 	ACLIB_AngleTrackObsrvAsm(pudtSinCos, pudtCtrl)
#define ACLIB_TrackObsrv(f16Error, pudtCtrl) 			ACLIB_TrackObsrvAsm(f16Error, pudtCtrl)
#define ACLIB_PMSMBemfObsrvAB(pudtCurrentAlphaBeta, pudtVoltageAlphaBeta, f16Speed, pudtCtrl) 	ACLIB_PMSMBemfObsrvABAsm(pudtCurrentAlphaBeta, pudtVoltageAlphaBeta, f16Speed, pudtCtrl)
#define ACLIB_PMSMBemfObsrvDQ(pudtCurrentDQ, pudtVoltageDQ, f16Speed, pudtCtrl) 	ACLIB_PMSMBemfObsrvDQAsm(pudtCurrentDQ, pudtVoltageDQ, f16Speed, pudtCtrl)

#define ACLIB_IntegratorInitVal(f16InitVal, pudtIntg) ACLIB_IntegratorInitValAsm(f16InitVal, pudtIntg)
#define ACLIB_Integrator(f16Xinp, pudtIntg) 			ACLIB_IntegratorAsm(f16Xinp, pudtIntg)

/* Reduced precision but faster algorithms */
#define ACLIB_AngleTrackObsrv12(pudtSinCos, pudtCtrl) 	ACLIB_AngleTrackObsrv12Asm(pudtSinCos, pudtCtrl)
#define ACLIB_PMSMBemfObsrv12AB(pudtCurrentAlphaBeta, pudtVoltageAlphaBeta, f16Speed, pudtCtrl) 	ACLIB_PMSMBemfObsrv12ABAsm(pudtCurrentAlphaBeta, pudtVoltageAlphaBeta, f16Speed, pudtCtrl)

#endif

#endif /* _ACLIB_H_ */
