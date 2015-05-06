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
* @file      GFLIB_RampAsm.h
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Feb-26-2013
* 
* @brief     Ramp functions in assembler
*
*******************************************************************************
*
* Ramp functions in assembler.
*
******************************************************************************/
#ifndef _GFLIB_RAMPASM_H_
#define _GFLIB_RAMPASM_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "56800E_types.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define GFLIB_Ramp16Asm(f16Desired, f16Actual, pudtParam) GFLIB_Ramp16FAsm(f16Desired, f16Actual, pudtParam)
#define GFLIB_Ramp32Asm(f32Desired, f32Actual, pudtParam) GFLIB_Ramp32FAsm(f32Desired, f32Actual, pudtParam)

/******************************************************************************
* Types
******************************************************************************/
/* Ramp structure */
typedef struct
{
    Frac16 f16RampUp;
    Frac16 f16RampDown;
} GFLIB_RAMP16_T;

/* Ramp structure */
typedef struct
{
    Frac32 f32RampUp;
    Frac32 f32RampDown;
} GFLIB_RAMP32_T;

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  Ramp function
*
* @param  ptr			GFLIB_RAMP16_T *pudtParam
*						  - rampUp: Ramp-up increment
*						  - rampDown: Ramp-down increment
* @param  in    		Frac16 f16Desired
*                         - Desired value in [-1;1] in Frac16
*						Frac16 f16Actual
*						  - Actual value in [-1;1] in Frac16
*
* @return This function returns
*     - Frac16 value [-1;1]
*		
* @remarks 	This function ramps the value from the f16Actual value up/down to
*			the f16Desired value using the up/down increments defined in
*			the pParam structure.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm Frac16 GFLIB_Ramp16FAsm(Frac16 f16Desired, Frac16 f16Actual, const GFLIB_RAMP16_T *pudtParam);

/***************************************************************************//*!
*
* @brief  Ramp function 32-bit version
*
* @param  ptr			GFLIB_RAMP32_T *pudtParam
*						  - rampUp: Ramp-up increment
*						  - rampDown: Ramp-down increment
* @param  in    		Frac32 f32Desired
*                         - Desired value in [-1;1] in Frac32
*						Frac32 f32Actual
*						  - Actual value in [-1;1] in Frac32
*
* @return This function returns
*     - Frac32 value [-1;1]
*		
* @remarks 	This function ramps the value from the f32Actual value up/down to
*			the f32Desired value using the up/down increments defined in
*			the pParam structure.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm Frac32 GFLIB_Ramp32FAsm(Frac32 f32Desired, Frac32 f32Actual, const GFLIB_RAMP32_T *pudtParam);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _GFLIB_RAMPASM_H_ */

