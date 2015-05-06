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
* @file      GFLIB_AsinAcosAsmDef.h
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Feb-26-2013
* 
* @brief     Definition file for the functions Asin(x) and Acos(x)
*
*******************************************************************************
*
* Definition file for the functions Asin(x) and Acos(x).
*
******************************************************************************/
#ifndef _GFLIB_ASINACOSASMDEF_H_
#define _GFLIB_ASINACOSASMDEF_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "GFLIB_AsinAcosAsm.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
//#define GFLIB_AsinAsm(f16Arg)   GFLIB_AsinFAsm(f16Arg, &gudtSqrtPolyTable, &gudtAsinAcosAddrTab)
//#define GFLIB_AcosAsm(f16Arg)   GFLIB_AcosFAsm(f16Arg, &gudtSqrtPolyTable, &gudtAsinAcosAddrTab)
#define GFLIB_AsinAsm(f16Arg)   GFLIB_AsinFAsm(f16Arg, &gudtAsinAcosAddrTab)
#define GFLIB_AcosAsm(f16Arg)   GFLIB_AcosFAsm(f16Arg, &gudtAsinAcosAddrTab)

/******************************************************************************
* Types
******************************************************************************/

/******************************************************************************
* Global variables
******************************************************************************/
extern GFLIB_ASINACOS_COEFFICIENTS_ADDR_T gudtAsinAcosAddrTab;
extern GFLIB_SQRT_POLY_TABLE_T gudtSqrtPolyTable;

/*****************************************************************************
* Global functions
******************************************************************************/

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _GFLIB_ASINACOSASMDEF_H_ */

