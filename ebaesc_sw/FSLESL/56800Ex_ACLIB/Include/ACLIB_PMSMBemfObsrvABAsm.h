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
* @file      ACLIB_PMSMBemfObsrvABAsm.h
*
* @author    r61928
* 
* @version   1.0.3.0
* 
* @date      Jan-8-2014
* 
* @brief     template
*
*******************************************************************************
*
* PMSM Back-emf observer.
*
******************************************************************************/
#ifndef _ACLIB_PMSM_BEMF_OBSRV_AB_ASM_H_
#define _ACLIB_PMSM_BEMF_OBSRV_AB_ASM_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "56800E_types.h"
#include "gflib.h"
#include "mclib.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define ACLIB_PMSMBemfObsrvABAsm(pudtCurrentAlphaBeta, pudtVoltageAlphaBeta, f16Speed, pudtCtrl) \
	ACLIB_PMSMBemfObsrvABFAsm(pudtCurrentAlphaBeta, pudtVoltageAlphaBeta, f16Speed, pudtCtrl)

#define ACLIB_PMSMBemfObsrv12ABAsm(pudtCurrentAlphaBeta, pudtVoltageAlphaBeta, f16Speed, pudtCtrl) \
	ACLIB_PMSMBemfObsrv12ABFAsm(pudtCurrentAlphaBeta, pudtVoltageAlphaBeta, f16Speed, pudtCtrl)

/* The V3 core instructions */
#define ACLIB_V3PMSMBemfObsrvABAsm(pudtCurrentAlphaBeta, pudtVoltageAlphaBeta, f16Speed, pudtCtrl) \
	ACLIB_V3PMSMBemfObsrvABFAsm(pudtCurrentAlphaBeta, pudtVoltageAlphaBeta, f16Speed, pudtCtrl)

#define ACLIB_V3PMSMBemfObsrv12ABAsm(pudtCurrentAlphaBeta, pudtVoltageAlphaBeta, f16Speed, pudtCtrl) \
	ACLIB_V3PMSMBemfObsrv12ABFAsm(pudtCurrentAlphaBeta, pudtVoltageAlphaBeta, f16Speed, pudtCtrl)


/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
	//Extended BEMF – alpha/beta
	struct
	{
		Frac32 f32Alpha;                //0
		Frac32 f32Beta;	                //2
	} udtEObsrv;

	//Accumulators – alpha/beta
	struct
	{
		Frac32 f32Alpha;                //4
		Frac32 f32Beta;	                //6
	} udtIObsrv;
	
	//Observer parameters for controllers
	struct
	{
		Frac32	f32IAlpha_1; //alpha-accumulator	//8
		Frac32	f32IBeta_1;  //beta-accumulator		//10
		Frac16  f16PropGain;	          			//12
		Int16	i16PropGainShift;           		//13
		Frac16  f16IntegGain;	        			//14
		Int16	i16IntegGainShift;          		//15
	} udtCtrl;
	
	//Unity vector - structure taken from MCLIB
	MCLIB_ANGLE_T mcUnityVctr;         //16
			
	//Configuration parameters 
	Frac16	f16IGain;		//current   	//18
	Frac16	f16UGain;     	//voltage   	//19
	Frac16	f16WIGain;    	//decoupling	//20
	Frac16	f16EGain;     	//extended BEMF //21
	
} ACLIB_BEMF_OBSRV_AB_T;


/******************************************************************************
* Global variables
******************************************************************************/
   
/******************************************************************************
* Global functions
******************************************************************************/


/***************************************************************************//*!
*
* @brief  BEMF observer using alpha, beta components to estimate the angle's
*         sin and cosine.
*
* @param  ptr   		MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtCurrentAlphaBeta
*                         - pointer to the alpha, beta current
*                       
*                       MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtVoltageAlphaBeta
*                         - pointer to the alpha, beta voltage
*
*                       ACLIB_BEMF_OBSRV_AB_T *pudtCtrl
*                         - pointer to the parameters of the observer
*
* @param  in    		Frac16 f16Speed
*                         - actual speed
*
* @return None
*		
* @remarks SATURATION IS RECOMMENDED TO BE TURNED OFF FOR BETTER PRECISION,
*		   NEVERTHELESS THE FUNCTION ALSO WORKS WITH THE SATURATION TURNED ON!
*
****************************************************************************/
extern asm void ACLIB_PMSMBemfObsrvABFAsm
(
    MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtCurrentAlphaBeta,
    MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtVoltageAlphaBeta,
    Frac16 f16Speed,
    ACLIB_BEMF_OBSRV_AB_T * const pudtCtrl
);


/***************************************************************************//*!
*
* @brief  BEMF observer using alpha, beta components to estimate the angle's
*         sin and cosine. This version uses the 12-bit precision SQRT algorithm
*		  so it is a bit faster but with a bit lower precision.
*
* @param  ptr   		MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtCurrentAlphaBeta
*                         - pointer to the alpha, beta current
*                       
*                       MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtVoltageAlphaBeta
*                         - pointer to the alpha, beta voltage
*
*                       ACLIB_BEMF_OBSRV_AB_T *pudtCtrl
*                         - pointer to the parameters of the observer
*
* @param  in    		Frac16 f16Speed
*                         - actual speed
*
* @return None
*		
* @remarks SATURATION IS RECOMMENDED TO BE TURNED OFF FOR BETTER PRECISION,
*		   NEVERTHELESS THE FUNCTION ALSO WORKS WITH THE SATURATION TURNED ON!
*
****************************************************************************/
extern asm void ACLIB_PMSMBemfObsrv12ABFAsm
(
    MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtCurrentAlphaBeta,
    MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtVoltageAlphaBeta,
    Frac16 f16Speed,
    ACLIB_BEMF_OBSRV_AB_T * const pudtCtrl
);

/***************************************************************************//*!
*
* @brief  BEMF observer using alpha, beta components to estimate the angle's
*         sin and cosine.
*
* @param  ptr   		MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtCurrentAlphaBeta
*                         - pointer to the alpha, beta current
*                       
*                       MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtVoltageAlphaBeta
*                         - pointer to the alpha, beta voltage
*
*                       ACLIB_BEMF_OBSRV_AB_T *pudtCtrl
*                         - pointer to the parameters of the observer
*
* @param  in    		Frac16 f16Speed
*                         - actual speed
*
* @return None
*		
* @remarks SATURATION IS RECOMMENDED TO BE TURNED OFF FOR BETTER PRECISION,
*		   NEVERTHELESS THE FUNCTION ALSO WORKS WITH THE SATURATION TURNED ON!
*		   The V3 core instructions used! 	
*
****************************************************************************/
extern asm void ACLIB_V3PMSMBemfObsrvABFAsm
(
    MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtCurrentAlphaBeta,
    MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtVoltageAlphaBeta,
    Frac16 f16Speed,
    ACLIB_BEMF_OBSRV_AB_T * const pudtCtrl
);

/***************************************************************************//*!
*
* @brief  BEMF observer using alpha, beta components to estimate the angle's
*         sin and cosine. This version uses the 12-bit precision SQRT algorithm
*		  so it is a bit faster but with a bit lower precision.
*
* @param  ptr   		MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtCurrentAlphaBeta
*                         - pointer to the alpha, beta current
*                       
*                       MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtVoltageAlphaBeta
*                         - pointer to the alpha, beta voltage
*
*                       ACLIB_BEMF_OBSRV_AB_T *pudtCtrl
*                         - pointer to the parameters of the observer
*
* @param  in    		Frac16 f16Speed
*                         - actual speed
*
* @return None
*		
* @remarks SATURATION IS RECOMMENDED TO BE TURNED OFF FOR BETTER PRECISION,
*		   NEVERTHELESS THE FUNCTION ALSO WORKS WITH THE SATURATION TURNED ON!
*		   The V3 core instructions used! 	
*
****************************************************************************/
extern asm void ACLIB_V3PMSMBemfObsrv12ABFAsm
(
    MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtCurrentAlphaBeta,
    MCLIB_2_COOR_SYST_ALPHA_BETA_T *pudtVoltageAlphaBeta,
    Frac16 f16Speed,
    ACLIB_BEMF_OBSRV_AB_T * const pudtCtrl
);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _ACLIB_PMSM_BEMF_OBSRV_AB_ASM_H_ */	
