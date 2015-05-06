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
* @file      ACLIB_IntegratorAsm.h
*
* @author    r61928
* 
* @version   1.0.4.0
* 
* @date      Jan-8-2014
* 
* @brief     Pure integrator without any limitation in assembler
*
*******************************************************************************
*
* Pure integrator without any limitation in assembler.
*
******************************************************************************/
#ifndef _ACLIB_INTEGRATOR_ASM_H_
#define _ACLIB_INTEGRATOR_ASM_H_

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
#define ACLIB_IntegratorInitValAsm(f16InitVal, pudtIntg) ACLIB_IntegratorInitValFAsm(f16InitVal, pudtIntg)
#define ACLIB_IntegratorAsm(f16Xinp, pudtIntg) 	ACLIB_IntegratorFAsm(f16Xinp,pudtIntg)

/******************************************************************************
* Types
******************************************************************************/

typedef struct
{
	Frac32	f32I_1;
	Frac16  f16IntegGain;
	Int16	i16IntegGainShift;
	
} ACLIB_INTEGRATOR_T;

/******************************************************************************
* Global variables
******************************************************************************/
   
/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  The function initializes the integral part of integrator.
*
* @param  ptr   		ACLIB_INTEGRATOR_T *pudtIntg
*						  	- Pointer to integrator structure
*
* @param  in    		Frac16 f16InitVal
* 							- Initial value
*
* @return N/A
*		
* @remarks 
*
****************************************************************************/
extern asm void ACLIB_IntegratorInitValFAsm(Frac16 f16InitVal, ACLIB_INTEGRATOR_T *pudtIntg);


/***************************************************************************//*!
*
* @brief  Integrated input value according to equation:	
*		   
*		  Output = (Input x Integ_scale)<<(Integ_scale_shift) + Output_K-1
*
* @param  ptr   		ACLIB_INTEGRATOR_T *pudtIntg
*						  - pointer to integrator parameters 
* @param  in    		Frac16 f16Xinp
*                         - input value
*
* @return out			Frac16 - function output		
*		
* @remarks
*
* THE SATURATION MUST BE TURNED OFF!
*
****************************************************************************/
extern asm Frac16 ACLIB_IntegratorFAsm(Frac16 f16Xinp, ACLIB_INTEGRATOR_T *pudtIntg);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _ACLIB_INTEGRATOR_ASM_H */
