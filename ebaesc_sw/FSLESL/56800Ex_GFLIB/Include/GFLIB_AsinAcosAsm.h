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
* @file      GFLIB_AsinAcosAsm.h
*
* @author    R61928
* 
* @version   1.0.2.0
* 
* @date      Aug-15-2013
* 
* @brief     Functions Asin(x) and Acos(x)
*
*******************************************************************************
*
* Functions Asin(x) and Acos(x).
*
******************************************************************************/
#ifndef _GFLIB_ASINACOSASM_H_
#define _GFLIB_ASINACOSASM_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "56800E_types.h"
#include "GFLIB_SqrtDefAsm.h"

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
    Frac16  f16OffsetX;
    Frac16  f16A[4];
    Frac16  f16YScl;
    Frac32  f32OffsetY;
}GFLIB_ASINACOS_PARAM_T;

typedef struct  
{
	GFLIB_ASINACOS_PARAM_T udtAsinAcosCoeff[3];
}GFLIB_ASINACOS_COEFFICIENTS_T;

typedef struct
{
    GFLIB_ASINACOS_PARAM_T *pudtInterval0;
    GFLIB_ASINACOS_PARAM_T *pudtInterval1 ;
    GFLIB_ASINACOS_PARAM_T *pudtInterval2;
}GFLIB_ASINACOS_COEFFICIENTS_ADDR_T;

/******************************************************************************
* Global variables
******************************************************************************/


/*****************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief    The MCLIB_Asin function computes the Asin(pi*x) using    
*		    piece-wise polynomial approximation.
*
* @param  ptr   		*pudtAsinPoly
*                           Pointer to the table
*						*pudtPolyTable
*						    Pointer to the SQRT function table
*
* @param  in    		f16Arg
*                           The input data value is in the range of [-1,1), which
*                           corresponds to the angle in the range of [-pi,pi).   
*                       
*
* @return The function returns Asin(pi*x).
*     
*		
* @remarks 	The MCLIB_Asin function computes the Asin(pi*x) using     
*		  	piece-wise polynomial approximation. All Asin values
*			falling beyond [-1, 1), are truncated to -1 and 1      
*			respectively.calculates Asin of the input fractional
*			argument                                               
*
*           f(f16Arg)   =   | f(-0,5 > f16Arg > 0.5) = f16Arg 
*                           |
*                           | f(-0,5 <= f16Arg <= 0.5) = Sqrt(f16Arg) 
*                           |
*
*           x = (f16Arg - nOffsetX)<<(3)
*
*           Asin(x) = A1*x^3 + A2*x^2 + A3*x + A4 
*
*           |Asin(f16Arg)| = 2 * ((Asin(x)>>(nYScl)) + lOffsetY)
*           
*           Asin(f16Arg) = 2 * (nCUneven * |Asin(x)|)
*
*           f(f16Arg) = nCUneven  |f(f16Arg)  = -0.5
*                                 |
*                                 |f(-f16Arg) =  0.5
*                                 |
*			
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm Frac16 GFLIB_AsinFAsm(Frac16 f16Arg, GFLIB_ASINACOS_COEFFICIENTS_ADDR_T *pudtAsinPoly);

/***************************************************************************//*!
*
* @brief    The MCLIB_Acos function computes the Acos(pi*x) using    
*		    piece-wise polynomial approximation.
*
* @param  ptr   		*pudtAcosPoly
*                           Pointer to the table
*						*pudtPolyTable
*							Pointer to the SQRT function table
*
* @param  in    		f16Arg
*                           The input data value is in the range of [-1,1), which
*                           corresponds to the angle in the range of [-pi,pi).   
*
* @return The function returns Acos(pi*x).
*     
*		
* @remarks 	The MCLIB_Acos function computes the Acos(pi*x) using     
*		  	Asin(pi*x) piece-wise polynomial approximation. All Asin values
*			falling beyond [-1, 1), are truncated to -1 and 1      
*			respectively.calculates Asin of the input fractional
*			argument                                               
*
*
*           Acos(f16Arg) = -1 * (- 0.5 + Asin(f16Arg) ) 
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm Frac16 GFLIB_AcosFAsm(Frac16 f16Arg, GFLIB_ASINACOS_COEFFICIENTS_ADDR_T *pudtAcosPoly);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _GFLIB_ASINACOSASM_H_ */

