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
* @file      GFLIB_TanAsm.h
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Feb-26-2013
* 
* @brief     Tan algorithm using piece-wise polynomial approximation
*
* Special Issues: The function requires the saturation mode to be set.
*
*******************************************************************************
*
* Tan algorithm using piece-wise polynomial approximation.
*
******************************************************************************/
#ifndef _GFLIB_TANASM_H_
#define _GFLIB_TANASM_H_

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


/******************************************************************************
* Types
******************************************************************************/
typedef struct 
{
    UWord16 uw16Dummy;
    Frac16  f16OffsetX;
    Frac16  f16A[7];
    Frac16  f16YScl;
    Frac32  f32OffsetY;
}GFLIB_TAN_PARAM_T;

typedef struct  
{
	GFLIB_TAN_PARAM_T udtTanCoeff[3];
}GFLIB_TAN_COEFFICIENTS_T;

typedef struct
{
    GFLIB_TAN_PARAM_T *pudtInterval0;
    GFLIB_TAN_PARAM_T *pudtInterval1;
    GFLIB_TAN_PARAM_T *pudtInterval2;
}GFLIB_TAN_COEFFICIENTS_ADDR_T;

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief    The MCLIB_Tan function computes the tan(pi*x) using    
*		    piece-wise polynomial approximation.
*
* @param  ptr   		*pudtTanPoly
*                           Pointer to the table
*
* @param  in    		f16Arg
*                           The input data value is in the range of [-1,1), which
*                           corresponds to the angle in the range of [-pi,pi).   
*                       
*
* @return The function returns tan(pi*x).
*		
* @remarks 	The MCLIB_Tan function computes the tan(pi*x) using     
*		  	piece-wise polynomial approximation. All tangent values
*			falling beyond [-1, 1), are truncated to -1 and 1      
*			respectively.calculates tan of the input fractional
*			argument                                               
*
*           x = (f16Arg - nOffsetX)<<(4)
*
*           Tan(x) = A1*x^6 + A2*x^5 + A3*x^4 + A4*x^3 + A5*x^2 + A6*x + A7
*
*           |Tan(fwArd)| = 2 * ((Tan(x)>>(nYScl)) + lOffsetY)
*           
*           Tan(fwArd) = 2 * (sign * |Tan(x)|)
*
*           f(f16Arg) = sign     |f(f16Arg)  = -0.5
*                               |
*                               |f(-f16Arg) =  0.5
*                               |
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm Frac16 GFLIB_TanFAsm(Frac16 f16Arg, const GFLIB_TAN_COEFFICIENTS_ADDR_T *pudtTanPoly);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _GFLIB_TANASM_H_ */

