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
* @file      MCLIB_CPTrfAsm.h
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Feb-26-2013
* 
* @brief     Clarke, Clarke Inverse, Park, Park Inverse Transformations
*
*******************************************************************************
*
* Clarke, Clarke Inverse, Park, Park Inverse Transformations.
*
******************************************************************************/
#ifndef _MCLIB_CPTRFM_H_
#define _MCLIB_CPTRFM_H_

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
#define MCLIB_ONE_DIV_SQRT3		18919
#define MCLIB_SQRT3_DIV_2		28378

#define	MCLIB_ClarkTrfAsm(pudtAlphaBeta, pudtAbc) MCLIB_ClarkTrfFAsm(pudtAlphaBeta, pudtAbc)
#define MCLIB_ClarkTrfInvAsm(pudtAbc, pudtAlphaBeta) MCLIB_ClarkTrfInvFAsm(pudtAbc, pudtAlphaBeta)
#define MCLIB_ParkTrfAsm(pudtDQ, pudtAlphaBeta, pudtSinCos) MCLIB_ParkTrfFAsm(pudtDQ, pudtAlphaBeta, pudtSinCos)
#define MCLIB_ParkTrfInvAsm(pudtAlphaBeta, pudtDQ, pudtSinCos) MCLIB_ParkTrfInvFAsm(pudtAlphaBeta, pudtDQ, pudtSinCos)

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
* @brief  The function calculates Clarke Transformation which is used 
*         for transforming values (current, voltage, flux) from the 
*         three phase stationary coordination system  to alpha-beta 
*         stationary orthogonal coordination system.
*
* @param  ptr			MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtAlphaBeta
*                       OUT - pointer to structure containing data of two phase
*                           stationary orthogonal system
*                       MCLIB_3_COOR_SYST_T *pudtAbc
*						IN  - pointer to structure containing data of three phase
*                           stationary system
*
* @remarks 	Modifies the structure pointed by pudtAlphaBeta pointer 
*           according to the folloving equations:
*           alpha = a
*           beta  = 1 / sqrt(3) * a + 2 / sqrt(3) * b
*
*			THE FUNCTION IS SATURATION INDEPENDENT!
* 
****************************************************************************/
extern asm void MCLIB_ClarkTrfFAsm(MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtAlphaBeta,
						    	   MCLIB_3_COOR_SYST_T *pudtAbc);

/***************************************************************************//*!
*
* @brief  	The function calculates Inverse Clarke Transformation which is used
*           for transforming values (current, voltage, flux) from alpha-beta 
*           stationary orthogonal coordination system to three phase 
*           stationary coordination system
*
* @param  ptr			MCLIB_3_COOR_SYST_T *pudtAbc
*						OUT - pointer to structure containing data of three phase
*                           stationary system
*                       MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtAlphaBeta
*                       IN  - pointer to structure containing data of two phase
*                           stationary orthogonal system
*
* @remarks 	Modifies the structure pointed by p_abc pointer 
*           according to the folloving equations:
*           a = alpha
*           b = -0.5 * alpha + sgrt(3) / 2 * beta
*           c = -0.5 * alpha - sgrt(3) / 2 * beta
*
*			THE FUNCTION IS SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm void MCLIB_ClarkTrfInvFAsm(MCLIB_3_COOR_SYST_T *pudtAbc,
							   		  MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtAlphaBeta);

/***************************************************************************//*!
*
* @brief  The function calculates Park Transformation which is used for 
*         transforming values (current, voltage, flux) from 
*         alpha-beta stationery orthogonal coordination system 
*         to d-q rotating orthogonal coordination system
*
* @param  ptr			MCLIB_2_COOR_SYST_D_Q_T *pudtDQ
*                       OUT - pointer to structure containing data of 
*                           DQ coordinate two-phase stationary 
*                           orthogonal syste
*                       MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtAlphaBeta
*                       IN  - pointer to structure containing data of two phase
*                           stationary orthogonal system
*                       MCLIB_ANGLE_T *pudtSinCos
*                       IN  - pointer to structure where the values 
*                           of sine and cosine are stored
*
* @remarks  Modifies the structure pointed by pDQ pointer
*           according to the folloving equations
*           d = alpha * cos(theta) + beta * sin(theta)
*           q = beta * cos(theta) - alpha * sin(theta)
*
*			THE FUNCTION IS SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm void MCLIB_ParkTrfFAsm(MCLIB_2_COOR_SYST_D_Q_T *pudtDQ,
						          MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtAlphaBeta,
						   		  MCLIB_ANGLE_T *pudtSinCos);

/***************************************************************************//*!
*
* @brief  The function calculates Inverse Park Transformation which is used 
*         for transforming values (current, voltage, flux) from 
*         d-q rotating orthogonal coordination system to alpha-beta 
*         stationery orthogonal coordination system.
*
* @param  ptr			MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtAlphaBeta
*                       OUT - pointer to structure containing data of two phase
*                           stationary orthogonal system
*                       MCLIB_2_COOR_SYST_D_Q_T *pudtDQ
*                       IN  - pointer to structure containing data of 
*                           DQ coordinate two-phase stationary 
*                           orthogonal syste
*                       MCLIB_ANGLE_T *pudtSinCos
*                       IN  - pointer to structure where the values 
*                           of sine and cosine are stored
*
* @remarks  Modifies the structure pointed by pAlphaBeta pointer 
*           according folloving equations:
*           alpha = d * cos(theta) - q * sin(theta)
*           beta  = d * sin(theta) + q * cos(theta)
*
*			THE FUNCTION IS SATURATION INDEPENDENT!
*
*******************************************************************************/
extern asm void MCLIB_ParkTrfInvFAsm(MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtAlphaBeta,
							  	  	 MCLIB_2_COOR_SYST_D_Q_T *pudtDQ,
							  		 MCLIB_ANGLE_T *pudtSinCos);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _MCLIB_CPTRFM_H_ */

