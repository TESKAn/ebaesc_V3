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
* @file      MCLIB_DecouplingAsm.h
*
* @author    R61928
* 
* @version   1.0.3.0
* 
* @date      Jan-8-2014
* 
* @brief     Decoupling calculation algorithm
*
*******************************************************************************
*
* Decoupling calculation algorithm.
*
******************************************************************************/
#ifndef _MCLIB_DECOUPLINGASM_H_
#define _MCLIB_DECOUPLINGASM_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "56800E_types.h"
#include "mclib_types.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define MCLIB_DecouplingPMSMAsm(pudtUs, pudtIs, f16AngularVelocity, pudtDecParam, pudtUsDec) \
			MCLIB_DecouplingPMSMFAsm(pudtUs, pudtIs, f16AngularVelocity, pudtDecParam, pudtUsDec)

/* The V3 core instructions */
#define MCLIB_V3DecouplingPMSMAsm(pudtUs, pudtIs, f16AngularVelocity, pudtDecParam, pudtUsDec) \
			MCLIB_V3DecouplingPMSMFAsm(pudtUs, pudtIs, f16AngularVelocity, pudtDecParam, pudtUsDec)

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
    Frac16  f16KdGain;
    Int16   i16KdGainShift;
    Frac16  f16KqGain;
    Int16   i16KqGainShift;
} MCLIB_DECOUPLING_PMSM_PARAM_T;

/******************************************************************************
* Global variables
******************************************************************************/
   
/******************************************************************************
* Global functions
******************************************************************************/

/*******************************************************************************
* 
* @brief	This function calculates decoupled d-axis and q-axis voltage commands
*			of PMSM. The d,q axis decoupling terms are added to controller outputs.
*
* @param  ptr			MCLIB_2_COOR_SYST_D_Q_T *pudtUs
*						IN  - pointer to the structure of d-axis
*                           and q-axis output voltages from d-axis
*                           and q-axis controllers
*						MCLIB_2_COOR_SYST_D_Q_T *pudtIs
*						IN  - pointer to the structure d-axis
*                           and q-axis current feedbacks
*
*						MCLIB_DECOUPLING_PMSM_PARAM_T *pudtDecParam
*						IN  - pointer to the structure of motor
*                           parameters needed for decoupling scaled
*                           to 16-bit fractional range
*                           The members of the structure are:
*                       	- f16KdGain - coefficient scaled to 16-bit
*                                     	  fractional range 
*                           			- f16KdGain = Kd	     		see equ(10)
*
*                           - i16KdGainShift - scaling coefficient
*                            				 - i16KdGainShift = d_sc 	see equ(12)                               
*
*                           - f16KqGain - coefficient scaled to 16-bit
*                                     	  fractional range
*                           			- f16KqGain = Kq	     		see equ(9)
*
*                           - i16KqGainShift - scaling coefficient                                 
*                                   	 	 - i16KqScale = q_sc     	see equ(11)
*
*						MCLIB_2_COOR_SYST_D_Q_T *pudtUsDec
*						OUT - pointer to the structure of decoupled
*                           d-axis and q-axis voltage commands
*
* @remarks
*  
*   The decoupling algorithm is calculated according to the following equations:
*           ud_dec = ud - Lq * f16AngularVelocity * iq        (1)        
*           uq_dec = uq + Ld * f16AngularVelocity * id        (2)
*   
*   The physical equations (1) and (2) expressed in fractional arithmetic:
*           ud_dec_s = ud_s - f16AngularVelocity * iq * (Lq * omega_max * i_max / u_max)   (3)        
*           uq_dec_s = uq_s + f16AngularVelocity * id * (Ld * omega_max * i_max / u_max)   (4)
*   where
*       Kq = Lq * omega_max * i_max / u_max      (5)        
*       Kd = Ld * omega_max * i_max / u_max      (6)
*
*   The Kq and Kd parameters have to be scaled to 16-bit fractional range.
*       0.5 <= Kq * 2 ^ (-q_sc) < 1   (7)
*       0.5 <= Kd * 2 ^ (-d_sc) < 1   (8)
*
*   Then
*       Kq_sc = Kq * 2 ^ (-q_sc)   (9)        
*       Kd_sc = Kd * 2 ^ (-d_sc)   (10)        
*
*   The scaling parameters are then
*               q_sc <= (logKq - log0.5) / log2 and q_sc > logKq / log2   (11)
*               d_sc <= (logKd - log0.5) / log2 and d_sc > logKd / log2   (12)
*   q_sc and d_sc range
*               -15 <= q_sc <= 15   (13)
*               -15 <= d_sc <= 15   (14)
*
*	THE FUNCTION IS SATURATION INDEPENDENT!
*
*******************************************************************************/
extern asm void MCLIB_DecouplingPMSMFAsm(MCLIB_2_COOR_SYST_D_Q_T *pudtUs, \
							 			 MCLIB_2_COOR_SYST_D_Q_T *pudtIs, \
							 	  		 Frac16 f16AngularVelocity, \
							 	  		 MCLIB_DECOUPLING_PMSM_PARAM_T *pudtDecParam, \
							 	  		 MCLIB_2_COOR_SYST_D_Q_T *pudtUsDec); 

/*******************************************************************************
* 
* @brief	This function calculates decoupled d-axis and q-axis voltage commands
*			of PMSM. The d,q axis decoupling terms are added to controller outputs.
*
* @param  ptr			MCLIB_2_COOR_SYST_D_Q_T *pudtUs
*						IN  - pointer to the structure of d-axis
*                           and q-axis output voltages from d-axis
*                           and q-axis controllers
*						MCLIB_2_COOR_SYST_D_Q_T *pudtIs
*						IN  - pointer to the structure d-axis
*                           and q-axis current feedbacks
*
*						MCLIB_DECOUPLING_PMSM_PARAM_T *pudtDecParam
*						IN  - pointer to the structure of motor
*                           parameters needed for decoupling scaled
*                           to 16-bit fractional range
*                           The members of the structure are:
*                       	- f16KdGain - coefficient scaled to 16-bit
*                                     	  fractional range 
*                           			- f16KdGain = Kd	     		see equ(10)
*
*                           - i16KdGainShift - scaling coefficient
*                            				 - i16KdGainShift = d_sc 	see equ(12)                               
*
*                           - f16KqGain - coefficient scaled to 16-bit
*                                     	  fractional range
*                           			- f16KqGain = Kq	     		see equ(9)
*
*                           - i16KqGainShift - scaling coefficient                                 
*                                   	 	 - i16KqScale = q_sc     	see equ(11)
*
*						MCLIB_2_COOR_SYST_D_Q_T *pudtUsDec
*						OUT - pointer to the structure of decoupled
*                           d-axis and q-axis voltage commands
*
* @remarks
*  
*   The decoupling algorithm is calculated according to the following equations:
*           ud_dec = ud - Lq * f16AngularVelocity * iq        (1)        
*           uq_dec = uq + Ld * f16AngularVelocity * id        (2)
*   
*   The physical equations (1) and (2) expressed in fractional arithmetic:
*           ud_dec_s = ud_s - f16AngularVelocity * iq * (Lq * omega_max * i_max / u_max)   (3)        
*           uq_dec_s = uq_s + f16AngularVelocity * id * (Ld * omega_max * i_max / u_max)   (4)
*   where
*       Kq = Lq * omega_max * i_max / u_max      (5)        
*       Kd = Ld * omega_max * i_max / u_max      (6)
*
*   The Kq and Kd parameters have to be scaled to 16-bit fractional range.
*       0.5 <= Kq * 2 ^ (-q_sc) < 1   (7)
*       0.5 <= Kd * 2 ^ (-d_sc) < 1   (8)
*
*   Then
*       Kq_sc = Kq * 2 ^ (-q_sc)   (9)        
*       Kd_sc = Kd * 2 ^ (-d_sc)   (10)        
*
*   The scaling parameters are then
*               q_sc <= (logKq - log0.5) / log2 and q_sc > logKq / log2   (11)
*               d_sc <= (logKd - log0.5) / log2 and d_sc > logKd / log2   (12)
*   q_sc and d_sc range
*               -15 <= q_sc <= 15   (13)
*               -15 <= d_sc <= 15   (14)
*
*	THE FUNCTION IS SATURATION INDEPENDENT!
*	The V3 core instructions used! 	
*
*******************************************************************************/
extern asm void MCLIB_V3DecouplingPMSMFAsm(MCLIB_2_COOR_SYST_D_Q_T *pudtUs, \
							 			 MCLIB_2_COOR_SYST_D_Q_T *pudtIs, \
							 	  		 Frac16 f16AngularVelocity, \
							 	  		 MCLIB_DECOUPLING_PMSM_PARAM_T *pudtDecParam, \
							 	  		 MCLIB_2_COOR_SYST_D_Q_T *pudtUsDec); 
/******************************************************************************
* Inline functions
******************************************************************************/


#endif /* _MCLIB_DECOUPLINGASM_H_ */

