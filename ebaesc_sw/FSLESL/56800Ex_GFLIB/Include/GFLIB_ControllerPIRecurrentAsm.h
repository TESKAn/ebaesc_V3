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
* @file      GFLIB_ControllerPIRecurrentAsm.h
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Feb-26-2013
* 
* @brief     PI controller calculated by recurrent expression
*
*           Special Issues: 
*                           The function requires the saturation mode to be set.
*
*******************************************************************************
*
* PI controller calculated by recurrent expression.
*
******************************************************************************/
#ifndef _GFLIB_CONTROLLER_PI_RECURRENT_ASM_H_
#define _GFLIB_CONTROLLER_PI_RECURRENT_ASM_H_

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
#define GFLIB_ControllerPIRecurrentAsm(f16Error, pudtCtrl) 	GFLIB_ControllerPIRecurrentFAsm(f16Error, pudtCtrl)

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
    Frac32  f32Acc;
    Frac16  f16ErrorK_1;
	Frac16  f16CC2Sc;
	Frac16  f16CC1Sc;
    UInt16  ui16NShift;
} GFLIB_CONTROLLER_PI_RECURRENT_T;


/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief    PI controller implemented by recurrent expression
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
*           5)  16-bit unsigned shift value due to scaling
*
* @return   Y0 - u(k) 
*           1) action value u(k) in range <-1,1)
*		
* @remarks 	
*           Difference equation:
*           u(k) = u(k-1) + CC2*error(k-1) + CC1*error(k) ;
*
*           u(k)        <-> controller output - action value
*           u(k-1)      <-> delayed action value u(k-1) (stored in 32-bit)
*           error(k)    <-> controller input (error)
*           CC2         <-> 2dn controller coefficient related to error(k-1)
*           CC1         <-> 1st controller coefficient related to error(k)
*
*           Transition from the continuous to the discrete time domain (Discretization)
*
*           +-------------------+----------------+-------------------+-------------------+
*           | Controller Coeff. |  Bilinear      |   Backward Rect.  |   Forward Rect.   |
*           +-------------------+----------------+-------------------+-------------------+ 
*           |       CC1         |  Kp+Ki*Ts/2    |   Kp+Ki*Ts        |   Kp              |
*           |       CC2         | -Kp+Ki*Ts/2    |  -Kp              |  -Kp+Ki*Ts        |
*           +-------------------+----------------+-------------------+-------------------+
*
*           Kp - proportional gain
*           Ki - integral gain
*           Ts - sampling period
*
*           Difference equation by fractional arithmetic:
*           u(k)    <-> UMAX - maximal value of controller output
*           e(k)    <-> EMAX - maximal value of controller input (error)
*           N       <-> NShift
*   
*           f32u(k)*(2^-N)  = f32u(k-1)*(2^-N) + CC1*(2^-N)*f16Err(k) + CC2*(2^-N)*f16Err(k-1);
*           f32u(k)*(2^-N)  = f32u(k-1)*(2^-N) + CC1Sc*f16Err(k) + CC2Sc*f16Err(k-1);
*       
*           f16u(k)         <-> fractional representation <-1,1) of controller output
*           error(k)        <-> fractional representation <-1,1) of controller input (error)
*           CC1Sc           <-> (CC1*EMAX/UMAX)*2^-N is fractional representation in range <-1,1)
*           CC2Sc           <-> (CC2*EMAX/UMAX)*2^-N is fractional representation in range <-1,1)
*           NShift          <-> chosen such that both coefficients are in the range <-1,1)
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/ 
extern asm Frac16 GFLIB_ControllerPIRecurrentFAsm
(
	register Frac16 f16Error,
	GFLIB_CONTROLLER_PI_RECURRENT_T * const pudtCtrl
);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _GFLIB_CONTROLLER_PI_RECURRENT_ASM_H_ */
