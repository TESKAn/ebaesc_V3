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
* @file      gdflib.h
*
* @author    R61928
* 
* @version   1.0.2.0
* 
* @date      Apr-23-2013
* 
* @brief     GDFLIB include file
*
*******************************************************************************
*
* Functions that are included in GDFLIB.
*
******************************************************************************/
#ifndef _GDFLIB_H_
#define _GDFLIB_H_

#include "56800E_types.h"
#include "GDFLIB_FilterIIRasm.h"								
#include "GDFLIB_FilterMA32Asm.h"

#if OPTION_CORE_V3 == 1 /* V3 core instructions used */

/* Redeclaration of the functions */
#define GDFLIB_FilterIIR1Init(pudtFilter) GDFLIB_FilterIIR1InitC(pudtFilter)
#define GDFLIB_FilterIIR2Init(pudtFilter) GDFLIB_FilterIIR2InitC(pudtFilter)
#define GDFLIB_FilterIIR3Init(pudtFilter) GDFLIB_FilterIIR3InitC(pudtFilter)
#define GDFLIB_FilterIIR4Init(pudtFilter) GDFLIB_FilterIIR4InitC(pudtFilter)

#define GDFLIB_FilterIIR1(f16In, pudtFilter) GDFLIB_V3FilterIIR1Asm(f16In, pudtFilter)
#define GDFLIB_FilterIIR2(f16In, pudtFilter) GDFLIB_V3FilterIIR2Asm(f16In, pudtFilter) 
#define GDFLIB_FilterIIR3(f16In, pudtFilter) GDFLIB_V3FilterIIR3Asm(f16In, pudtFilter) 
#define GDFLIB_FilterIIR4(f16In, pudtFilter) GDFLIB_V3FilterIIR4Asm(f16In, pudtFilter) 

#define GDFLIB_FilterMA32Init(pudtFilter) GDFLIB_FilterMA32InitC(pudtFilter)
#define GDFLIB_FilterMA32InitVal(f16InitVal, pudtFilter) GDFLIB_FilterMA32InitValAsm(f16InitVal, pudtFilter)
#define GDFLIB_FilterMA32(f16In, pudtFilter) GDFLIB_FilterMA32Asm(f16In, pudtFilter)
#define GDFLIB_FilterMA32i(f16In, pudtFilter) GDFLIB_FilterMA32Asmi(f16In, pudtFilter)

#else /* V3 core instructions not used */

/* Redeclaration of the functions */
#define GDFLIB_FilterIIR1Init(pudtFilter) GDFLIB_FilterIIR1InitC(pudtFilter)
#define GDFLIB_FilterIIR2Init(pudtFilter) GDFLIB_FilterIIR2InitC(pudtFilter)
#define GDFLIB_FilterIIR3Init(pudtFilter) GDFLIB_FilterIIR3InitC(pudtFilter)
#define GDFLIB_FilterIIR4Init(pudtFilter) GDFLIB_FilterIIR4InitC(pudtFilter)

#define GDFLIB_FilterIIR1(f16In, pudtFilter) GDFLIB_FilterIIR1Asm(f16In, pudtFilter)
#define GDFLIB_FilterIIR2(f16In, pudtFilter) GDFLIB_FilterIIR2Asm(f16In, pudtFilter) 
#define GDFLIB_FilterIIR3(f16In, pudtFilter) GDFLIB_FilterIIR3Asm(f16In, pudtFilter) 
#define GDFLIB_FilterIIR4(f16In, pudtFilter) GDFLIB_FilterIIR4Asm(f16In, pudtFilter) 

#define GDFLIB_FilterMA32Init(pudtFilter) GDFLIB_FilterMA32InitC(pudtFilter)
#define GDFLIB_FilterMA32InitVal(f16InitVal, pudtFilter) GDFLIB_FilterMA32InitValAsm(f16InitVal, pudtFilter)
#define GDFLIB_FilterMA32(f16In, pudtFilter) GDFLIB_FilterMA32Asm(f16In, pudtFilter)
#define GDFLIB_FilterMA32i(f16In, pudtFilter) GDFLIB_FilterMA32Asmi(f16In, pudtFilter)

#endif

#endif /* _GDFLIB_H_ */

