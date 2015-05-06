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
* @file      GFLIB_ControllerPIDpAsm.h
*
* @author    R61928
* 
* @version   1.0.4.0
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
#ifndef _GFLIB_CONTROLLERPIDPASM_H_
#define _GFLIB_CONTROLLERPIDPASM_H_

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
#define GFLIB_ControllerPIDpInitValAsm(f16InitVal, pudtPidParams) \
		GFLIB_ControllerPIDpInitValFAsm(f16InitVal, pudtPidParams)

#define GFLIB_ControllerPIDpAsm(f16InputErrorK, f16InputDErrorK, pPiParams, pSatFlag, pf16InputDErrorK_1) \
		GFLIB_ControllerPIDpFAsm(f16InputErrorK, f16InputDErrorK, pPiParams, pSatFlag, pf16InputDErrorK_1)

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
   Frac16 f16PropGain;
   Frac16 f16IntegGain;
   Frac16 f16DerGain;
   Int16  i16PropGainShift;
   Int16  i16IntegGainShift;
   Int16  i16DerGainShift;
   Frac32 f32IntegPartK_1;
   Frac16 f16UpperLimit;
   Frac16 f16LowerLimit;
   Int16  i16LimitFlag;
} GFLIB_CONTROLLER_PID_P_PARAMS_T;



/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  The function initializes the integral part of the PID controller.
*
* @param  ptr   		GFLIB_CONTROLLER_PID_P_PARAMS_T *pudtPidParams
*						  	- Pointer to PID controller structure
*
* @param  in    		Frac16 f16InitVal
* 							- Initial value
*
* @return N/A
*		
* @remarks 
*
****************************************************************************/
extern asm void GFLIB_ControllerPIDpInitValFAsm(Frac16 f16InitVal, GFLIB_CONTROLLER_PID_P_PARAMS_T *pudtPidParams);

/***************************************************************************//*!
*
* @brief The function calculates
*        the Proportional-Integral-Derivative (PID) algorithm according to
*        equations below. The controller output is limited and limit values
*        (f16UpperLimit and f16LowerLimit) are defined by user. The PID controller
*        algorithm also returns limitation flag. This flag named "i16LimitFlag"
*        is the member of the structure of the PID controller parameters
*        (GFLIB_PID_CONTROLLER_PARAMS). If the PID controller output reaches
*        the upper or lower limit then i16LimitFlag = 1 otherwise i16LimitFlag = 0.
*        Anti-windup strategy is implemented by limiting the integral part.
*        There are two ways of limiting the integral part: 
*        - The integral state is limited by the controller limits, in the same way
*          as controller output does.
*        - When the variable satFlag set by user software outside of
*          the PID controller function and passed into the function as
*          the pointer pSatFlag is not zero, then the integral part is frozen. 
*
* @param  ptr   		GFLIB_CONTROLLER_PID_P_PARAMS_T *pudtPidParams
*                         - Pointer to a structure of PID controller parameters;
*                           the GFLIB_PID_CONTROLLER_PARAMS data type is defined
*                           in the header file GFLIB_ControllerPIDAsm.h
*
*                           GFLIB_CONTROLLER_PID_P_PARAMS_T data structure:
*                           Frac16 f16PropGain    
*                               - proportional gain; input parameter
*                                 in the following range
*                                 0 <= f16PropGain < 1  
*                           Frac16 f16IntegGain
*                               - integral gain; input parameter
*                                 in the following range
*                                 0 <= f16IntegGain < 1  
*                           Frac16 f16DerGain
*                               - derivative gain; input parameter
*                                 in the following range:
*                                 0 <= f16DerGain < 1  
*                           Int16  i16PropGainShift
*                               - proportional gain shift; input parameter
*                                 in the following range:
*                                 0 <= i16PropGainShift < 14  
*                           Int16  i16IntegGainShift
*                               - integral gain shift; input parameter
*                                 in the following range:
*                                 0 <= i16IntegGainShift < 14  
*                           Int16  i16DerGainShift
*                               - derivative gain shift; input parameter
*                                 in the following range:
*                                 0 <= i16DerGainShift < 14  
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
*                                 otherwise i16LimitFlag = 0  
*
*                       const Int16 *pi16SatFlag
*                         - Pointer to a 16-bit integer variable;
*                           if the integer variable passed into the function
*                           as a pointer is set to 0, then the integral part
*                           is limited by the PID controller limits.
*                           If the integer variable is not zero,
*                           then the integral part is frozen
*
*                       Frac16 *pf16InputDErrorK_1
*                         - Pointer to a 16-bit fractional variable;
*                           input error at step K-1 processed only by
*                           the derivative term of the PID algorithm. It is
*                           the state variable modified by the function.
*                           The variable can also be modified outside of the function.
*                           The purpose can be the initialization of the variable
* 
* @param  in    		Frac16 f16InputErrorK
*                         - Input error of the controller at step K
*                           processed by P and I terms of the PID algorithm.
*                           Fractional value in the range <-1;1)
*
*                       Frac16 f16InputDErrorK
*                         - Input error of the controller at step K processed
*                           by the D term of the PID algorithm
*                           Fractional value in the range <-1;1)
*
*
*
*
* @return The function GFLIB_ControllerPIDAsm returns a fractional value as a result
*         of the PID algorithm. The value returned by the algorithm is
*         in the following range:
*         f16LowerLimit <= PIDresult <= f16UpperLimit
*		
* @remarks 	    PID controller algorithm:                                           
*                                                                       
*                        -----------                 
*                 e(k)  |           |  u(k)           
*               --------|    PID    |---------        
*               --------|           |                 
*                ed(k)  |           |
*                        -----------                  
* 		                                                                
*       e(t) - input error in continuous time domain, processed by P and I components of PID algorithm                    
*       ed(t) - input error in continuous time domain,processed by D component of PID algorithm
*       u(t) - controller output in continuous time domain              
*                                                                       
*       e(k) - input error at step k - discrete time domain, processed by P and I components             
*       ed(k) - input error at step k - discrete time domain, processed by D component             
*       u(k) - controller output at step k - discrete time domain          
*                                                                       
*       e_max - max range of e(k)                                              
*       u_max - max range of u(k)                                          
*       ed_max = e_max       
*                                                                
*       The PID controller algorithm in continuous time domain:          
*                              /t                                       
*       u(t) = K[e(t) + 1/Ti * | e(t)*dt + Td(ded/dt]                            
*                              /0                                             
*       K  - controller gain                                            
*       Ti - integral time constant
*       Td - derivative time constant                                     
*                                                                       
*       PID controller expressed in fractional arithmetic:               
*                                                                      
*       u_f(k) = K_sc * e_f(k) + ui_f(k - 1) + Ki_sc * e_f(k) + Kd_sc(e_d_f(k) - e_d_f(k_1))        
*                                                                           
*       e_f(k) = e(k)/e_max                                                
*       e_d_f(k) = e_d_f(k)/ed_max
*       u_f(k) = u(k)/u_max                                                 
*                                                                       
*       K_sc =  K * e_max/u_max                                       
*       Ki_sc = K * T/Ti * e_max/u_max
*       Kd_sc = K * Td/T * ed_max/u_max                               
*                                                                       
*       T - sampling time/period                                               
*                                                                      
*       f16PropGain = K_sc * 2^(- i16PropGainShift)
* 
*       f16IntegGain = Ki_sc * 2^(- i16IntegGainShift)
*
*       f16DerGain = Kd_sc * 2^(- i16DerGainShift)
*
*        0 < i16PropGainShift < 14
*
*        0 < i16IntegGainShift < 14
*
*        0 < i16DerGainShift < 14
*
*        0 <= f16PropGain < 1   
*
*        0 <= f16IntegGain < 1   
*
*        0 <= f16DerGain < 1   
*
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
*       Note:
*                Ti>4*Td
*
*		SATURATION INDEPENDENT!
*
****************************************************************************/
asm Frac16 GFLIB_ControllerPIDpFAsm(Frac16 f16InputErrorK, 
								    Frac16 f16InputDErrorK,
                                    GFLIB_CONTROLLER_PID_P_PARAMS_T *pudtPidParams,
                                    const Int16 *pi16SatFlag,
                                    Frac16 *pf16InputDErrorK_1);
                   


/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _GFLIB_CONTROLLERPIDPASM_H_ */

