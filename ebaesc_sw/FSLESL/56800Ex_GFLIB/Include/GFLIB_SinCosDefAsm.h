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
* @file      GFLIB_SinCosDefAsm.h
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Feb-26-2013
* 
* @brief     Sine/cosine look-up table and macros.
*
*******************************************************************************
*
* This file contains macros to redefine the 3-arguments sine and cosine to
* 2- and 1-argument sine and cosine. The 1q sine look-up table is defined in
* this module.
*
******************************************************************************/
#ifndef _GFLIB_SINCOSDEFASM_H_
#define _GFLIB_SINCOSDEFASM_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "GFLIB_SinLutAsm.h"
#include "GFLIB_CosLutAsm.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define GFLIB_SinLut2Asm(f16Arg, udtSinTable) GFLIB_SinLutFAsm(f16Arg, udtSinTable.f16TableValues, udtSinTable.uw16TableSize)
#define GFLIB_SinLutAsm(f16Arg) 	GFLIB_SinLutFAsm(f16Arg, gudtSinLut.f16TableValues, gudtSinLut.uw16TableSize)

#define GFLIB_CosLut2Asm(f16Arg, udtSinTable) GFLIB_CosLutFAsm(f16Arg, udtSinTable.f16TableValues, udtSinTable.uw16TableSize)
#define GFLIB_CosLutAsm(f16Arg) 	GFLIB_CosLutFAsm(f16Arg, gudtSinLut.f16TableValues, gudtSinLut.uw16TableSize)

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
	UWord16 uw16TableSize;
	Frac16 f16TableValues[];
} GFLIB_SINCOSLUT_T;

/******************************************************************************
* Global constants
******************************************************************************/
extern const GFLIB_SINCOSLUT_T gudtSinLut;

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _GFLIB_SINCOSDEFASM_H_ */

