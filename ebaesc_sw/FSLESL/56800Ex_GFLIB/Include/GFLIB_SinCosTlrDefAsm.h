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
* @file      GFLIB_SinCosTlrDefAsm.h
*
* @author    R61928
* 
* @version   1.0.2.0
* 
* @date      Aug-15-2013
* 
* @brief     Taylor Sin/Cos table and macros
*
*******************************************************************************
*
* Contains a table of constants that are used for the Sin/Cos polynomial calculation.
*
******************************************************************************/
#ifndef _GFLIB_SINCOSTLRDEFASM_H_
#define _GFLIB_SINCOSTLRDEFASM_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "GFLIB_SinCosTlrAsm.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define GFLIB_SinTlrAsm(f16In) GFLIB_SinTlrFAsm(f16In, &gudtSinCoefTable)
#define GFLIB_CosTlrAsm(f16In) GFLIB_CosTlrFAsmi(f16In, &gudtSinCoefTable)
#define GFLIB_Sin12TlrAsm(f16In) GFLIB_Sin12TlrFAsm(f16In, &gudtSin12CoefTable)
#define GFLIB_Cos12TlrAsm(f16In) GFLIB_Cos12TlrFAsmi(f16In, &gudtSin12CoefTable)

/* The V3 core instructions */
#define GFLIB_V3SinTlrAsm(f16In) GFLIB_V3SinTlrFAsm(f16In, &gudtSinCoefTable)
#define GFLIB_V3CosTlrAsm(f16In) GFLIB_V3CosTlrFAsmi(f16In, &gudtSinCoefTable)

/******************************************************************************
* Types
******************************************************************************/

/******************************************************************************
* Global constants
******************************************************************************/

/******************************************************************************
* Global variables
******************************************************************************/
extern GFLIB_SIN_TAYLOR_COEF_T gudtSinCoefTable;
extern GFLIB_SIN12_TAYLOR_COEF_T gudtSin12CoefTable;

/******************************************************************************
* Global functions
******************************************************************************/

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _GFLIB_SINCOSTLRDEFASM_H_ */

