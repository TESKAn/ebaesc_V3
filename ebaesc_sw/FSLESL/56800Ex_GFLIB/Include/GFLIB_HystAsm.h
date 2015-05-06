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
* @file      GFLIB_HystAsm.h
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Feb-26-2013
* 
* @brief     Hysteresis
*
*           Special Issues: 
*                           The function requires the saturation mode to be set.
*
*******************************************************************************
*
* Hysteresis.
*
******************************************************************************/
#ifndef _GFLIB_HYSTASM_H_
#define _GFLIB_HYSTASM_H_

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
#define GFLIB_HYST_DEFAULT {0,0,-16384,-32768,16384,32767}

#define GFLIB_HystAsm(pudtHystVar) GFLIB_HystFAsm(pudtHystVar)

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
	Frac16 f16In;
    Frac16 f16Out;
    Frac16 f16HystOff;
    Frac16 f16OutOff;  
    Frac16 f16HystOn;
    Frac16 f16OutOn;
}GFLIB_HYST_T;

/******************************************************************************
* Global variables
******************************************************************************/
   
/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  Hysteresis function
*
* @param  ptr			GFLIB_HYST_T *pudtHystVar
*						  - f16In: Input to the function
*						  - f16Out: Output of the function
*						  - f16HystOff: Lower threshold
*						  - f16OutOff: Output when input is below f16HystOff
*						  - f16HystOn: Upper threshold
*						  - f16OutOn: Output when input is above f16HystOn
*
* @return N/A
*		
* @remarks 	The function represents a hysteresis (or relay) function. The
* 			function switches output between the two predefined values. When
*			the input is higher than upper threshold f16HystOn, the output
*			is high; when the input is below another (lower) threshold
*			f16HystOff, the output is low; when the input is between the two,
*			the output retains its value.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm void GFLIB_HystFAsm(GFLIB_HYST_T *pudtHystVar);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _GFLIB_HYSTASM_H_ */

