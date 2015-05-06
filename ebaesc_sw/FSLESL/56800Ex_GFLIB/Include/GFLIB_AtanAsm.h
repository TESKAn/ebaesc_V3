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
* @file      GFLIB_AtanAsm.h
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Feb-26-2013
* 
* @brief     Atan algorithm using piece-wise polynomial approximation
*
* Special Issues:  The function requires the saturation mode to be set.
*
*******************************************************************************
*
* Atan algorithm using piece-wise polynomial approximation.
*
******************************************************************************/
#ifndef _GFLIB_ATANASM_H_
#define _GFLIB_ATANASM_H_

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
    UWord16  uw16Dummy;
    Frac16  f16OffsetX;
    Frac16  f16A[5];
    Frac16  f16YScl;
    Frac32  f32OffsetY;
}GFLIB_ATAN_PARAM_T;

typedef struct  
{
	GFLIB_ATAN_PARAM_T udtAtanCoeff[3];
}GFLIB_ATAN_COEFFICIENTS_T;

typedef struct
{
    GFLIB_ATAN_PARAM_T *pudtInterval0;
    GFLIB_ATAN_PARAM_T *pudtInterval1 ;
    GFLIB_ATAN_PARAM_T *pudtInterval2;
}GFLIB_ATAN_COEFFICIENTS_ADDR_T;


/******************************************************************************
* Global variables
******************************************************************************/


/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief    The MCLIB_Atan function computes the Atan(fwArd)/(pi/4) 
*		    using piece-wise polynomial approximation.          
*
* @param  ptr   		*pudtAtanPoly
*                           Pointer to the table
*
* @param  in    		f16Arg
*                           The input data value is in the range of [-1,1).   
*                       
*
* @return   The function returns atan(fwArd)/(pi/4).The output data value 
*     	    is in range [-0.25, 0.25), which corresponds to the angle 
*     	    in the range of [-pi/4,pi/4).                             
*		
* @remarks 	The MCLIB_Atan function computes the atan(x)/(pi/4) 
*		  	using piece-wise polynomial approximation.          
*			
*
*           x = (f16Arg - nOffsetX)<<(2)
*
*           Atan(x) =  A1*x^4 + A2*x^3 + A3*x^2 + A4*x + A5
*
*           |Atan(fwArd)| = 2 * ((Atan(x)>>(nYScl)) + lOffsetY)
*           
*           Atan(fwArd) = 2 * (sing * |Atan(x)|)
*
*           f(f16Arg) = sing     |f(f16Arg)  = -0.5
*                               |
*                               |f(-f16Arg) =  0.5
*                               |
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm Frac16 GFLIB_AtanFAsm(Frac16 f16Arg, GFLIB_ATAN_COEFFICIENTS_ADDR_T *pudtAtanPoly);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _GFLIB_ATANASM_H_ */

