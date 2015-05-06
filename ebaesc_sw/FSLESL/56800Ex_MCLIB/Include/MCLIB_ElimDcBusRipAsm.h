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
* @file      MCLIB_ElimDcBusRipAsm.h
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Feb-26-2013
* 
* @brief     Function is used for elimination of the DC-Bus voltage ripple
*
*******************************************************************************
*
* Function is used for elimination of the DC-Bus voltage ripple.
*
******************************************************************************/
#ifndef _MCLIB_ELIMDCBUSRIPASM_H_
#define _MCLIB_ELIMDCBUSRIPASM_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "56800E_types.h"
#include "MCLIB_types.h"

/******************************************************************************
* Constants
******************************************************************************/


/******************************************************************************
* Macros 
******************************************************************************/
#define MCLIB_ElimDcBusRipAsm(f16InvModIndex, f16DcBusMsr, pudtInAlphaBeta, pudtOutAlphaBeta) \
			MCLIB_ElimDcBusRipFAsm(f16InvModIndex, f16DcBusMsr, pudtInAlphaBeta, pudtOutAlphaBeta)

#define MCLIB_ElimDcBusRipGenAsm(f16DcBusMsr, pudtInAlphaBeta, pudtOutAlphaBeta) \
			MCLIB_ElimDcBusRipGenFAsm(f16DcBusMsr, pudtInAlphaBeta, pudtOutAlphaBeta)

/******************************************************************************
* Types
******************************************************************************/

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  The function is used for elimination of the DC-Bus voltage 
*		  ripple.                                                
*
*
*
* @param  ptr   	- pudtInAlphaBeta - Direct(alpha) and quadrat.(beta)
*					                    component of the stator voltage 
*					                    vector in the stationary        
*					                    reference frame.                
*					                    Format Q15, range 8000-7FFF     
*
*
*                   - pudtOutAlphaBeta - Direct(alpha) and quadrat.(beta)
*					                     component of the stator voltage 
*					                     vector in the stationary        
*					                     reference frame after DC-Bus    
*					                     ripple elimination.             
*					                     Format Q15, range 8000-7FFF     
*
* @param  in                                                     
*                   - f16InvModIndex - Inverse Modulation Index is       
*                                      dependent on the type modulation  
*				                       technique being used:            
*				                       ---------------+-----------------
*				                       Mod. Technique I   invModIndex   
*				                       ---------------+-----------------
*				                       svmStd         I FRAC16(0.866025)
*				                       svmU0n         I FRAC16(0.866025)
*				                       svmU7n         I FRAC16(0.866025)
*				                       svmAlt         I FRAC16(0.866025)
*				                       svmSci         I FRAC16(0.866025)
*				                       svmPwmIct      I FRAC16(1.0)     
*				                       ---------------+-----------------
*				                       Format Q15, range 0-7FFF         
*				                                                        
*				    - f16DcBusMsr - Actual effective value of the    
*				                    DC-Bus voltage.                  
*				                    Format Q15, range 0-7FFF         
*				                                                        
* @return Function reads, recalculate and fills variables alpha,
*		  beta in the data structure MCLIB_2_COOR_SYST_ALPHA_BETA_T.                
*		
* @remarks The input direct(alpha) and quadrature(beta) 
*		   components form stator voltage vector of its
*		   magnitude has to reside in the unit circle.
*
*		   The scaling equation:
*
*
*				   Uph_max        Uph		  Uph	    Uph_max      mod_idx
*		   Upwm = --------- * ----------- = ------- * ----------- * ---------
*				   Udc_max		  Udc		  Udc       Udc_max			2
*							   ---------	 -----
*								mod_idx		   2
*
*		   Where:
*					Uph_max - max. phase voltage (alpha, beta) scale
*					Udc_max - max. dc bus voltage scale
*					Uph - applied phase voltage (alpha, beta)
*					Udc - measured dc bus voltage
*					mod_idx - modulation index; sqrt(3) for SVM with 3rd harmonic inj., 2 for PWM application
*					
*				   Uph
*		   Upwm = ----- * inv_mod_idx
*				   Udc
*
*		   Where:				   Uph_max	   mod_idx
*					inv_mod_idx = --------- * ---------
*								   Udc_max        2
*
*
*		   In general cases of Uph_max = Udc_max we can state:
*
*		   if mod_idx = sqrt(3) then inv_mod_idx = sqrt(3) / 2 = 1.732050 / 2 = 0.866025
*		   if mod_idx = 2 then inv_mod_idx = 2 / 2 = 1.0
*			
*
*
*										    alphaIn
*		   alphaOut =  f16InvModIndex * ---------------
*										  f16DcBusMsr
*										 -------------
*											   2
*
*										   betaIn
*		   betaOut =  f16InvModIndex * ---------------
*										 f16DcBusMsr
*										-------------
*										     2  
*
*		   SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm void MCLIB_ElimDcBusRipFAsm(Frac16 f16InvModIndex,
							    	   Frac16 f16DcBusMsr, 
                        			   MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtInAlphaBeta,
                        			   MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtOutAlphaBeta);

/***************************************************************************//*!
*
* @brief  The function is used for elimination of the DC-Bus voltage 
*		  ripple in general cases where the phase voltage is scale as
*	      measured dc bus voltage divided by the mod. index		  .                                                
*
*
*
* @param  ptr   	- pudtInAlphaBeta - Direct(alpha) and quadrat.(beta)
*					                    component of the stator voltage 
*					                    vector in the stationary        
*					                    reference frame.                
*					                    Format Q15, range 8000-7FFF     
*
*
*                   - pudtOutAlphaBeta - Direct(alpha) and quadrat.(beta)
*					                     component of the stator voltage 
*					                     vector in the stationary        
*					                     reference frame after DC-Bus    
*					                     ripple elimination.             
*					                     Format Q15, range 8000-7FFF     
*
* @param  in        - f16DcBusMsr - Actual effective value of the    
*				                    DC-Bus voltage.                  
*				                    Format Q15, range 0-7FFF         
*				                                                        
* @return Function reads, recalculate and fills variables alpha,
*		  beta in the data structure MCLIB_2_COOR_SYST_ALPHA_BETA_T.                
*		
* @remarks The input direct(alpha) and quadrature(beta) 
*		   components form stator voltage vector of its
*		   magnitude has to reside in the unit circle.
*
*		   The scaling equation:
*
*
*				   Uph_max        Uph		  Uph	    Uph_max      
*		   Upwm = --------- * ----------- = ------- * ----------- * mod_idx
*				   Udc_max		  Udc		  Udc       Udc_max			
*							   ---------	 
*								mod_idx		 
*
*		   Where:
*					Uph_max - max. phase voltage (alpha, beta) scale
*					Udc_max - max. dc bus voltage scale
*					Uph - applied phase voltage (alpha, beta)
*					Udc - measured dc bus voltage
*					mod_idx - modulation index; sqrt(3) for SVM with 3rd harmonic inj., 2 for PWM application
*					
*		   In case where Uph_max = Udc_max / mod_idx then:
*
*
*					Udc_max
*				   ---------
*				   	mod_idx         Uph		    Uph	      Udc_max      mod_idx	   Uph
*		   Upwm = ----------- * ----------- = ------- * ----------- * --------- = -----
*				    Udc_max		    Udc		    Udc       Udc_max	   mod_idx	   Udc
*							     ---------	   
*								  mod_idx		
*
*		   So to be able to use this algorithm, the following conditions must be valid:
*
*		   Uph_max = Udc_max / mod_idx
*
*		   For mod_idx = sqrt(3) then the Uph_max = Udc_max / sqrt(3)
*		   For mod_idx = 2 then the Uph_max = Udc_max / 2
*
*
*					     alphaIn
*		   alphaOut = -------------
*					   f16DcBusMsr
*
*					 	betaIn
*		   betaOut = -------------
*					  f16DcBusMsr
*
*
*		   SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm void MCLIB_ElimDcBusRipGenFAsm(Frac16 f16DcBusMsr, 
                        		   	      MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtInAlphaBeta,
                        	 	          MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtOutAlphaBeta);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _MCLIB_ELIMDCBUSRIPASM_H_ */

