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
* @file      MCLIB_SvmAsm.h
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Feb-26-2013
* 
* @brief     Space vector modulation functions in assembler.
*
*******************************************************************************
*
* Space vector modulation functions in assembler.
*
******************************************************************************/
#ifndef _MCLIB_SVMASM_H_
#define _MCLIB_SVMASM_H_

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
#define	MCLIB_SvmStdAsm(pudtAlphaBeta, pudtAbc)		MCLIB_SvmStdFAsm(pudtAlphaBeta, pudtAbc)
#define	MCLIB_SvmU0nAsm(pudtAlphaBeta, pudtAbc)		MCLIB_SvmU0nFAsm(pudtAlphaBeta, pudtAbc)
#define	MCLIB_SvmU7nAsm(pudtAlphaBeta, pudtAbc)		MCLIB_SvmU7nFAsm(pudtAlphaBeta, pudtAbc)
#define	MCLIB_SvmAltAsm(pudtAlphaBeta, pudtAbc)		MCLIB_SvmAltFAsm(pudtAlphaBeta, pudtAbc)
#define	MCLIB_PwmIctAsm(pudtAlphaBeta, pudtAbc)		MCLIB_PwmIctFAsm(pudtAlphaBeta, pudtAbc)
#define	MCLIB_SvmSciAsm(pudtAlphaBeta, pudtAbc)		MCLIB_SvmSciFAsm(pudtAlphaBeta, pudtAbc)

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
* @brief  Standard SVM function
*
* @param  ptr			pudtAlphaBeta
*						  - IN: Direct(alpha) and quadrature(beta)
*                           component of the stator voltage     
*                           vector in the stationary reference frame.                              
*                           Format Frac16, range 8000-7FFF         
*						pudtAbc
*						  - OUT: Pointer to duty cycle of the
*                           A, B and C phases.
*                           Format Frac16, range 8000-7FFF 
*
* @return This function returns
*     - sector number of the stator voltage vector resides in.
*		
* @remarks 	The function calculates the appropriate duty cycles needed  
*           to generate a given stator reference voltage using the  
*           Space Vector Modulation with a duty cycle of the null   
*           switching state from states O000 and O111 in each sector
*           of the hexagon.
*
*			The function fills variables a, b and c 
*           in the data structure pudtAbc and returns the
*           sector number of the stator voltage vector resides in.
*
*			THE SATURATION MUST BE SET OFF. 
****************************************************************************/
extern asm UWord16 MCLIB_SvmStdFAsm(MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtAlphaBeta, MCLIB_3_COOR_SYST_T *pudtAbc);
    
/***************************************************************************//*!
*
* @brief  O000 state null vector SVM function
*
* @param  ptr			pudtAlphaBeta
*						  - IN: Direct(alpha) and quadrature(beta)
*                           component of the stator voltage     
*                           vector in the stationary reference frame.                              
*                           Format Frac16, range 8000-7FFF         
*						pudtAbc
*						  - OUT: Pointer to duty cycle of the
*                           A, B and C phases.
*                           Format Frac16, range 8000-7FFF 
*
* @return This function returns
*     - sector number of the stator voltage vector resides in.
*		
* @remarks 	The function calculates the appropriate duty cycles needed
*          	to generate a given stator reference voltage using the
*          	Space Vector Modulation with a duty cycle of the null
*          	switching state combined only from states O000 in each
*          	sector of the hexagon.
*
*		   	The function fills variables a, b and c 
*          	in the data structure pudtAbc and returns the
*          	sector number of the stator voltage vector resides in.
*
*			THE SATURATION MUST BE SET OFF. 
****************************************************************************/
extern asm UWord16 MCLIB_SvmU0nFAsm(MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtAlphaBeta, MCLIB_3_COOR_SYST_T *pudtAbc);
    
/***************************************************************************//*!
*
* @brief  O111 state null vector SVM function
*
* @param  ptr			pudtAlphaBeta
*						  - IN: Direct(alpha) and quadrature(beta)
*                           component of the stator voltage     
*                           vector in the stationary reference frame.                              
*                           Format Frac16, range 8000-7FFF         
*						pudtAbc
*						  - OUT: Pointer to duty cycle of the
*                           A, B and C phases.
*                           Format Frac16, range 8000-7FFF 
*
* @return This function returns
*     - sector number of the stator voltage vector resides in.
*		
* @remarks 	The function calculates the appropriate duty cycles needed
*           to generate a given stator reference voltage using the
*           Space Vector Modulation with a duty cycle of the null
*           switching state combined only from states O111 in each
*           sector of the hexagon.
*
*		   	The function fills variables a, b and c 
*          	in the data structure pudtAbc and returns the
*          	sector number of the stator voltage vector resides in.
*
*			THE SATURATION MUST BE SET OFF. 
****************************************************************************/
extern asm UWord16 MCLIB_SvmU7nFAsm(MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtAlphaBeta, MCLIB_3_COOR_SYST_T *pudtAbc);
	
/***************************************************************************//*!
*
* @brief  Alternating state null vector SVM function
*
* @param  ptr			pudtAlphaBeta
*						  - IN: Direct(alpha) and quadrature(beta)
*                           component of the stator voltage     
*                           vector in the stationary reference frame.                              
*                           Format Frac16, range 8000-7FFF         
*						pudtAbc
*						  - OUT: Pointer to duty cycle of the
*                           A, B and C phases.
*                           Format Frac16, range 8000-7FFF 
*
* @return This function returns
*     - sector number of the stator voltage vector resides in.
*		
* @remarks 	The function calculates the appropriate duty cycles needed
*           to generate a given stator reference voltage using the
*           Space Vector Modulation with a duty cycle of the null
*           switching state combined from state O000 and O111 in the
*           even (2,4,6) and odd (1,3,5) sectors of the hexagon,
*           respectively.
*
*		   	The function fills variables a, b and c 
*          	in the data structure pudtAbc and returns the
*          	sector number of the stator voltage vector resides in.
*
*			THE SATURATION MUST BE SET OFF. 
****************************************************************************/
extern asm UWord16 MCLIB_SvmAltFAsm(MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtAlphaBeta, MCLIB_3_COOR_SYST_T *pudtAbc);
                           
/***************************************************************************//*!
*
* @brief  General sinusoidal modulation function
*
* @param  ptr			pudtAlphaBeta
*						  - IN: Direct(alpha) and quadrature(beta)
*                           component of the stator voltage     
*                           vector in the stationary reference frame.                              
*                           Format Frac16, range 8000-7FFF         
*						pudtAbc
*						  - OUT: Pointer to duty cycle of the
*                           A, B and C phases.
*                           Format Frac16, range 8000-7FFF 
*
* @return This function returns
*     - sector number of the stator voltage vector resides in.
*		
* @remarks	The function calculates the appropriate duty cycles needed
*           to generate a given stator reference voltage using the
*           General Sinusoidal Modulation technique.
*
*		   	The function fills variables a, b and c 
*          	in the data structure pudtAbc and returns the
*          	sector number of the stator voltage vector resides in.
*
*			THE SATURATION MUST BE SET OFF. 
****************************************************************************/
extern asm UWord16 MCLIB_PwmIctFAsm(MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtAlphaBeta, MCLIB_3_COOR_SYST_T *pudtAbc);                            

/***************************************************************************//*!
*
* @brief  General Sinusoidal Modulation with 3rd harmonic injection
*
* @param  ptr			pudtAlphaBeta
*						  - IN: Direct(alpha) and quadrature(beta)
*                           component of the stator voltage     
*                           vector in the stationary reference frame.                              
*                           Format Frac16, range 8000-7FFF         
*						pudtAbc
*						  - OUT: Pointer to duty cycle of the
*                           A, B and C phases.
*                           Format Frac16, range 8000-7FFF 
*
* @return This function returns
*     - sector number of the stator voltage vector resides in.
*		
* @remarks  The function calculates the appropriate duty cycles needed
*           to generate a given stator reference voltage using the
*           General Sinusoidal Modulation with an injection of the
*           third harmonic.
* 
*		   	The function fills variables a, b and c 
*          	in the data structure pudtAbc and returns the
*          	sector number of the stator voltage vector resides in.
*
*			THE SATURATION MUST BE SET OFF. 
****************************************************************************/
extern asm UWord16 MCLIB_SvmSciFAsm(MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtAlphaBeta, MCLIB_3_COOR_SYST_T *pudtAbc);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _MCLIB_SVMASM_H_ */

