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
* @file      GFLIB_AtanYXAsm.h
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Feb-26-2013
* 
* @brief     AtanYX algorithm using piece-wise polynomial approximation
*
* Special Issues: The function requires the saturation mode to be set.
*
*******************************************************************************
*
* AtanYX algorithm using piece-wise polynomial approximation.
*
******************************************************************************/
#ifndef _GFLIB_ATANYXASM_H_
#define _GFLIB_ATANYXASM_H_

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
    Frac16  f16A[5];
    Frac16  f16YScl;
    Frac32  f32OffsetY;
} GFLIB_ATANYX_PARAM_T;

typedef struct  
{
	GFLIB_ATANYX_PARAM_T udtAtanYXCoeff[3];
} GFLIB_ATANYX_COEFFICIENTS_T;

typedef struct
{
    GFLIB_ATANYX_PARAM_T *pudtInterval0;
    GFLIB_ATANYX_PARAM_T *pudtInterval1 ;
    GFLIB_ATANYX_PARAM_T *pudtInterval2;
} GFLIB_ATANYX_COEFFICIENTS_ADDR_T;


/******************************************************************************
* Global variables
******************************************************************************/


/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief    The MCLIB_AtanYX function computes the AtanYX(f16ValY,f16ValX) 
*		    using piece-wise polynomial approximation.          
*
* @param  ptr   		*pudtAtanPoly
*                           Pointer to the table
*
*						*pi16ErrFlag
*							The pointed variable is 1, if the inputs were 0, 0
*							otherwise it is 0.
*
* @param  in    		f16ValY, f16ValX
*                           The input data value is in the range of [-1,1).   
*                       
*
* @return   The function returns AtanYX(f16ValY,f16ValX).The output data value 
*     	    is in range [-0.25, 0.25), which corresponds to the angle 
*     	    in the range of [-pi/4,pi/4).                             
*		
* @remarks 	The MCLIB_Atan function computes the atan(x)/(pi/4) 
*		  	using piece-wise polynomial approximation.          
*			
*
*           f16Arg = f16ValY / f16ValX           
*
*           x = (f16Arg - nOffsetX)<<(2)
*
*           Atan(x) =  A1*x^4 + A2*x^3 + A3*x^2 + A4*x + A5
*
*           |Atan(f16Arg)| = 2 * ((Atan(x)>>(nYScl)) + lOffsetY)
*           
*           Atan(f16Arg) = 2 * (sing * |Atan(x)|)
*
*           | 16bit Xsign | 15bit Ysign |  Result Interval  |
*           |===============================================|
*           |      0      |      0      |       [0;0,5]     | 
*           |      0      |      1      |       (0,5;1)     |
*			|      1      |      0      |       [-0,5;0)    |
*			|      1      |      1      |       [-1;-0.5)   |
*			|_____________|_____________|___________________|
*           "sign register"
*
*			In case of the inputs 0, 0 the error flag is set to 1 and the output
*			of the function is 0. 
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm Frac16 GFLIB_AtanYXFAsm(register Frac16 f16ValY,
							 	   register Frac16 f16ValX,
							 	   const Int16 *pi16ErrFlag,
							 	   GFLIB_ATANYX_COEFFICIENTS_ADDR_T *pudtAtanYXPoly);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _GFLIB_ATANYXASM_H_ */

