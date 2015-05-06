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
* @file      GFLIB_ControllerPIDRecurrentAsm.h
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Feb-26-2013
* 
* @brief     PID controller calculated by recurrent expression
*
*           Special Issues: 
*                           The function requires the saturation mode to be set.
*
*******************************************************************************
*
* PID controller calculated by recurrent expression.
*
******************************************************************************/
#ifndef _GFLIB_CONTROLLER_PID_RECURRENT_ASM_H_
#define _GFLIB_CONTROLLER_PID_RECURRENT_ASM_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "56800E_types.h"

/******************************************************************************
* Macros 
******************************************************************************/
#define GFLIB_ControllerPIDRecurrentAsm(f16Error, pudtCtrl) 	GFLIB_ControllerPIDRecurrentFAsm(f16Error, pudtCtrl)

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
    Frac32  f32Acc;
    Frac16  f16ErrorK_2;
	Frac16  f16CC3Sc;
	Frac16  f16CC1Sc;
	Frac16  f16CC2Sc;
    Frac16  f16ErrorK_1;
    UInt16  ui16NShift;
} GFLIB_CONTROLLER_PID_RECURRENT_T;
   
/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief    PID controller implemented by recurrent expression
*           with no limitation, and no anti wind-up (AW).
*           
* @param    f16Error
*           defined as f16Error = f16Required - f16Actual
*
* @param    const pudtCtrl
*           const pointer to recurrent PI controller data structure
*           1)  32-bit accumulator of delayed action value u(k-1)
*           2)  16-bit error delayed value error(k-1)
*           3)  16-bit controller parameter CC2Sc related to error(k-1)
*           4)  16-bit controller parameter CC1Sc related to error(k)
*           5)  16-bit error delayed value error(k-2)
*           6)  16-bit controller parameter CC3Sc related to error(k-2)
*           7)  16-bit unsigned shift value due to scaling
*
* @return   Y0 - u(k) 
*           1) action value u(k) in range <-1,1)
*		
* @remarks 	
*           Difference equation:
*           u(k) = u(k-1) + CC3*error(k-2) + CC2*error(k-1) + CC1*error(k) ;
*
*           u(k)        <-> controller output - action value
*           u(k-1)      <-> delayed action value u(k-1) (stored in 32-bit)
*           error(k)    <-> controller input (error)
*           error(k-1)  <-> controller input (error in k-1 step)
*           error(k-2)  <-> controller input (error in k-2 step)
*           CC3         <-> 3rd controller coefficient related to error(k-2)
*           CC2         <-> 2dn controller coefficient related to error(k-1)
*           CC1         <-> 1st controller coefficient related to error(k)
*
*           Transition from the continuous to the discrete time domain (Discretization)
*
*           +-------------------+-------------------------------+-------------------------------+
*           | Controller Coeff. | Integration w. Backward Rect. | Integration w. Bilinear       |
*           |                   | Derivation  w. Backward Rect. | Derivation w. Backward Rect.  |
*           +-------------------+-------------------------------+-------------------------------+ 
*           |       CC1         |  Kp+Ki*Ts+Kd/Ts               |  Kp+Ki*Ts/2+Kd/Ts             |
*           |       CC2         | -Kp-2*Kd/Ts                   | -Kp+Ki*Ts/2-2*Kd/Ts           |
*           |       CC3         |  Kd/Ts                        |  Kd/Ts                        |
*           +-------------------+-------------------------------+-------------------------------+
*
*           Kp - proportional gain
*           Ki - integral gain
*           Kd - derivation gain
*           Ts - sampling period
*
*           Difference equation by fractional arithmetic:
*           u(k)    <-> UMAX - maximal value of controller output
*           e(k)    <-> EMAX - maximal value of controller input (error)
*           N       <-> NShift
*   
*           f32u(k)*(2^-N)  = f32u(k-1)*(2^-N)+CC1*(2^-N)*f16Err(k)+CC2*(2^-N)*f16Err(k-1)+CC3*(2^-N)*f16Err(k-2);
*           f32u(k)*(2^-N)  = f32u(k-1)*(2^-N)+CC1Sc*f16Err(k)+CC2Sc*f16Err(k-1)+CC3Sc*f16Err(k-2);
*       
*           f16u(k)         <-> fractional representation <-1,1) of controller output
*           f16Err(k)       <-> fractional representation <-1,1) of controller input (error)
*           CC1Sc           <-> (CC1*EMAX/UMAX)*2^-N is fractional representation in range <-1,1)
*           CC2Sc           <-> (CC2*EMAX/UMAX)*2^-N is fractional representation in range <-1,1)
*           CC3Sc           <-> (CC3*EMAX/UMAX)*2^-N is fractional representation in range <-1,1)
*           uiNShift        <-> chosen such that both coefficients are in the range <-1,1)
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/  
extern asm Frac16 GFLIB_ControllerPIDRecurrentFAsm
(
	register Frac16 f16Error,
	GFLIB_CONTROLLER_PID_RECURRENT_T * const pudtCtrl
);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _GFLIB_CONTROLLER_PID_RECURRENT_ASM_H_ */
