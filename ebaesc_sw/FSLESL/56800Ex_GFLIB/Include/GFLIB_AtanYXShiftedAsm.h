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
* @file      GFLIB_AtanYXShiftedAsm.h
*
* @author    r59400
* 
* @version   1.0.2.0
* 
* @date      Mar-21-2013
* 
* @brief     Function AtanYXShifted
*
*           Special Issues: 
*                           The function requires the saturation mode to be set.
*
*******************************************************************************
*
* Function AtanYXShifted.
*
******************************************************************************/
#ifndef _GFLIB_ATANYXSHIFTEDASM_H_
#define _GFLIB_ATANYXSHIFTEDASM_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "56800E_types.h"
#include "GFLIB_AtanYXAsmDef.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define GFLIB_ATANYXSHIFTED_DEFAULT {32767, 32767, 0, 0, 0}

#define GFLIB_AtanYXShiftedAsm(f16ArgY, f16ArgX, pudtAtanYXCoeff)\
           GFLIB_AtanYXShiftedFAsm(f16ArgY, f16ArgX, pudtAtanYXCoeff)

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
   Frac16       f16Ky;      
   Frac16       f16Kx;      
   Int16        i16Ny;      
   Int16        i16Nx;      
   Frac16       f16ThetaAdj;
}GFLIB_ATANYXSHIFTED_T;

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/
/***************************************************************************//*!
*
* @brief    The GFLIB_AtanYXShifted function computes the angle using    
*		    two sine waves. The signals should be scaled to be in 
*			the range [-1,1) (fractional range).                  
*
* @param  ptr   *pudtAtanYXCoeff
*				     Pointer to the table of the AtanYX function
*
* @param  in    nValY - the pInterval1 signal assumed to be equal to sin(theta) where 
*                        theta is the shaft angular position, the signal should be    
*				       scaled to the fractional range [-1, 1)                   
*				       ([0x8000, 0x7FFF) hex).                                  
*				                                                                
*				nValX - the pInterval2 signal from position sensor assumed to be     
*					       equal to sin(theta + dtheta) where theta is the shaft    
*					       angular and dtheta the phase shift to the pInterval1 position 
*					       sensor signal. The signal should be scaled to the        
*					       fractional range [-1, 1) ([0x8000, 0x7FFF) hex)          
*
*           f16Ky   - multiplication coefficient of y signal
*			f16Kx   - multiplication coefficient of x signal
*			i16Ny   - scaling coefficient of y signal       
*			i16Nx   - scaling coefficient of x signal       
*			f16ThetaAdj - adjusting angle                   
*           
*           This coefficients initializes the internal   
*		  	variables and should be	initializes before the call to this function.
*
*           
* @return   The function returns an angle of two sine waves shifted in 
*     	    in phase. The output data value is in the range [-1,1)    
*     	    which corresponds to [-pi,pi) range of angle.             
*		
* @remarks 	The AtanYXShifted function computes the angle using     
*		  	two sine waves. The signals should be scaled to be in   
*			the range [-1,1) (fractional range).                    
*			                                                        
*           The function assumes that the provided arguments        
*			correspond to the following signals:                    
*                                                                   
*			- nValY (pInterval1 argument) = sin(theta)                    
*           - nValX (pInterval2 argument) = sin(theta + dtheta)          
*                                                                   
*			where:                                                  
*             theta     the actual angle                            
*			  dtheta    the shift betwen signals, can be set in the 
*			            header file                                 
*			                                                        
*			The returned angle is equal:                            
*			                                                        
*			  thetac = theta + thetaoffset                          
*			                                                        
*			where:                                                  
*			  thetac       computed angle                           
*			  dtheta       as above                                 
*			  thetaoffset  angle offset, can be set in header file  
*			                                                        
*			The signal values (sin(theta) and sin(theta + dtheta))  
*			are provided to the function as argument.                 
*			                                                        
*			The function computes angle with accuracy depending on  
*			signals' phase difference. The function is most         
*			accurate when dtheta does not differ from pi/2 or -pi2. 
*			On the other hand the function error raises to infinity 
*			with dtheta approaching 0 or -pi. The detailed error    
*			analysis can be found in the accompanying documentation 
*
*       Special Issues: The function requires GFLIB_AtanYX function to be called
*		                                                                   		                                                                   
*
*		SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm Frac16 GFLIB_AtanYXShiftedFAsm
(
	register Frac16 f16ValY,
	register Frac16 f16ValX,
	GFLIB_ATANYXSHIFTED_T *pudtAtanYXCoeff
);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _GFLIB_ATANYXSHIFTEDASM_H_ */

