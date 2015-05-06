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
* @file      GFLIB_ControllerPIpAsm.h
*
* @author    R61928
* 
* @version   1.0.3.0
* 
* @date      Aug-15-2013
* 
* @brief     PI controller algorithms implemented in assembler
*
*******************************************************************************
*
* PI controller algorithms implemented in assembler.
*
******************************************************************************/
#ifndef _GFLIB_CONTROLLERPIPASM_H_
#define _GFLIB_CONTROLLERPIPASM_H_

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
#define GFLIB_ControllerPIpInitValAsm(f16InitVal, pudtPidParams) GFLIB_ControllerPIpInitValFAsm(f16InitVal, pudtPidParams)

#define GFLIB_ControllerPIpAsm(f16InputErrorK, pudtPiParams, pi16SatFlag) 	GFLIB_ControllerPIpFAsm(f16InputErrorK, pudtPiParams, pi16SatFlag)

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
   Frac16 f16PropGain;
   Frac16 f16IntegGain;
   Int16  i16PropGainShift;
   Int16  i16IntegGainShift;
   Frac32 f32IntegPartK_1;
   Frac16 f16UpperLimit;  
   Frac16 f16LowerLimit;    
   Int16  i16LimitFlag;
} GFLIB_CONTROLLER_PI_P_PARAMS_T;

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  The function initializes the integral part of the PI controller.
*
* @param  ptr   		GFLIB_CONTROLLER_PI_P_PARAMS_T *pudtPidParams
*						  	- Pointer to PI controller structure
*
* @param  in    		Frac16 f16InitVal
* 							- Initial value
*
* @return N/A
*		
* @remarks 
*
****************************************************************************/
extern asm void GFLIB_ControllerPIpInitValFAsm(Frac16 f16InitVal, GFLIB_CONTROLLER_PI_P_PARAMS_T *pudtPiParams);

/***************************************************************************//*!
*
* @brief The function calculates the Proportional-Integral
*        (PI) algorithm according to equations below. The controller output
*        is limited and limit values (f16UpperLimit and f16LowerLimit) are
*        defined by user. The PI controller algorithm also returns
*        limitation flag. This flag named "i16LimitFlag" is the member of
*        the structure of the PI controller parameters (GFLIB_CONTROLLER_PI_P_PARAMS_T).
*        If the PI controller output reaches the upper or lower limit
*        then i16LimitFlag = 1 otherwise i16LimitFlag = 0.
*        Anti-windup strategy is implemented by limiting the integral part.
*        There are two ways of limiting the integral part: 
*        - The integral state is limited by the controller limits,
*          in the same way as controller output does.
*        - When the variable satFlag set by user software outside of
*          the PI controller function and passed into the function as
*          the pointer pSatFlag is not zero, then the integral part is frozen.
*
*
*
* @param  ptr   		GFLIB_CONTROLLER_PI_P_PARAMS_T *pudtPiParams
*						  - Pointer to the structure of PI controller parameters
*
*                           GFLIB_CONTROLLER_PI_P_PARAMS_T data structure:
*                           Frac16 f16PropGain    
*                               - proportional gain; input parameter
*                                 in the following range
*                                 0 <= f16PropGain < 1  
*                           Frac16 f16IntegGain
*                               - integral gain; input parameter
*                                 in the following range
*                                 0 <= f16IntegGain < 1  
*                           Int16  i16PropGainShift
*                               - proportional gain shift; input parameter
*                                 in the following range:
*                                 0 <= i16PropGainShift < 14  
*                           Int16  i16IntegGainShift
*                               - integral gain shift; input parameter
*                                 in the following range:
*                                 0 <= i16IntegGainShift < 14  
*                           Frac32 f32IntegPartK_1
*                               - state variable; integral part at step k-1;
*                                 can be modified outside of the function;
*                                 input/output parameter
*                           Frac16 f16UpperLimit
*                               - upper limit of the controller output;
*                                 input parameter;
*                                 f16UpperLimit > f16LowerLimit
*                           Frac16 f16LowerLimit
*                               - lower limit of the controller output;
*                                 input parameter;
*                                 f16UpperLimit > f16LowerLimit
*                           Int16 i16LimitFlag
*                               - limitation flag; if set to 1, the controller
*                                 output reached either f16UpperLimit or f16LowerLimit,
*                                 otherwise i16LimitFlag = 0;
*                                 i16LimitFlag is managed by the function  
*
*                       const Int16 *pi16SatFlag
*                         - Pointer to a 16-bit integer variable; if the integer
*                           variable passed into the function as a pointer is set
*                           to 0, then the integral part is limited only by
*                           the PI controller limits.
*                           If the integer variable is not zero, then
*                           the integral part is frozen immediately
* 
* @param  in    		Frac16 f16InputErrorK
*                         - Input error of the controller at step K processed
*                           by P and I terms of the PI algorithm
*                           fractional value in the range <-1;1)
*
* @return  The function GFLIB_ControllerPI1Asm returns a fractional value
*          as a result of the PI algorithm. The value returned by the algorithm
*          is in the following range:
*          f16LowerLimit <= PIresult <= f16UpperLimit
*		
* @remarks 	    PI controller algorithm:                                           
*                                                                       
*                        -----------                 
*                 e(k)  |           |  u(k)           
*               --------|    PI     |---------        
*                       |           |                 
*                        -----------                  
* 		                                                                
*       e(t) - input error in continuous time domain                    
*       u(t) - controller output in continuous time domain              
*                                                                       
*       e(k) - input error at step k - discrete time domain             
*       u(k) - controller output at step k - discrete time domain          
*                                                                       
*       e_max - max range of e(k)                                              
*       u_max - max range of u(k)                                          
*                                                                       
*       The PI controller algorithm in continuous time domain:          
*                              /t                                       
*       u(t) = K[e(t) + 1/Ti * | e(t)*dt]                            
*                              /0                                             
*       K  - controller gain                                            
*       Ti - integral time constant                                     
*                                                                       
*       PI controller expressed in fractional arithmetic:               
*                                                                      
*       u_f(k) = K_sc * e_f(k) + ui_f(k - 1) + Ki_sc * e_f(k)        
*                                                                           
*       e_f(k) = e(k)/e_max                                                
*       u_f(k) = u(k)/u_max                                                 
*                                                                       
*       K_sc =  K * e_max/u_max                                       
*       Ki_sc = K * T/Ti * e_max/u_max                               
*                                                                       
*       T - sampling time/period                                               
*                                                                      
*       f16PropGain = K_sc * 2^(-i16PropGainShift)
* 
*       f16IntegGain = Ki_sc * 2^(-i16IntegGainShift)
*
*        0 <= i16PropGainShift < 14
*
*        0 <= i16IntegGainShift < 14
*
*        0 <= f16PropGain < 1   
*
*        0 <= f16IntegGain < 1   
*        
*       Example:
*        
*       Assumption: K_sc = 1.8
*        In this case K_sc cannot be directly interpreted as a fractional
*        value since the range of fractional values is <-1,1). It is
*        necessary to scale the K_sc using i16PropGainShift to fit
*        parameter f16PropGain into the range <-1,1)
*        
*       Solution:
*        The most precise scaling is to scale down the parameter K_sc
*        to have the f16PropGain between the following range
*        0.5 <= f16PropGain < 1 and to calculate the corresponding  
*        i16PropGainShift parameter.
*        
*        [log(K_sc)-log(0.5)]/log(2) >= i16PropGainShift  
*        [log(1.8)-log(0.5)]/log(2) >= i16PropGainShift  
*        i16PropGainShift <= 1.8
*        
*        
*        [log(K_sc)-log(1)]/log(2) < i16PropGainShift  
*        [log(1.8)-log(1)]/log(2) < i16PropGainShift  
*        i16PropGainShift > 0.8
*        
*        0.8 < i16PropGainShift <= 1.8  => i16PropGainShift = 1
*        because the i16PropGainShift is integer value
*        
*        Then
*        f16PropGain = K_sc * 2^(- i16PropGainShift)
*        f16PropGain = 1.8 * 2^(-1) = 0.9
*        
*       Result:
*        f16PropGain      = 0.9
*        i16PropGainShift = 1
*
*		SATURATION INDEPENDENT!
*
****************************************************************************/
asm Frac16 GFLIB_ControllerPIpFAsm(Frac16 f16InputErrorK,
								   GFLIB_CONTROLLER_PI_P_PARAMS_T *pudtPiParams,
                                   const Int16 *pi16SatFlag);
                                
/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _GFLIB_CONTROLLERPIPASM_H_ */

