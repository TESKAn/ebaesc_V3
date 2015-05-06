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
* @file      ACLIB_PMSMBemfObsrvDQAsm.h
*
* @author    r61928
* 
* @version   1.0.3.0
* 
* @date      Jan-8-2014
* 
* @brief     PMSM Back-emf observer.
*
*******************************************************************************
*
* PMSM Back-emf observer.
*
******************************************************************************/
#ifndef _ACLIB_PMSM_BEMF_OBSRV_DQ_ASM_H_
#define _ACLIB_PMSM_BEMF_OBSRV_DQ_ASM_H_

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
#define ACLIB_PMSMBemfObsrvDQAsm(pudtCurrentAlphaBeta, pudtVoltageAlphaBeta, f16Speed, pudtCtrl) \
	ACLIB_PMSMBemfObsrvDQFAsm(pudtCurrentAlphaBeta, pudtVoltageAlphaBeta, f16Speed, pudtCtrl)

/* The V3 core instructions */
#define ACLIB_V3PMSMBemfObsrvDQAsm(pudtCurrentAlphaBeta, pudtVoltageAlphaBeta, f16Speed, pudtCtrl) \
	ACLIB_V3PMSMBemfObsrvDQFAsm(pudtCurrentAlphaBeta, pudtVoltageAlphaBeta, f16Speed, pudtCtrl)

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
	//Extended BEMF - d/q
	struct
	{
		Frac32 f32D;	//0
		Frac32 f32Q;	//2
	} udtEObsrv;

	//Accumulators - d/q
	struct
	{
		Frac32 f32D;	//4
		Frac32 f32Q;	//6
	} udtIObsrv;
	
	//Observer parameters for controllers
	struct
	{
		Frac32		f32ID_1;	//d-accumulator		//8
		Frac32		f32IQ_1;	//q-accumulator		//10
		Frac16  	f16PropGain;					//12
		Int16		i16PropGainShift;				//13
		Frac16  	f16IntegGain;					//14
		Int16		i16IntegGainShift;				//15
	} udtCtrl;

	//misalignment error of reference frame
	Frac16	f16Error;								//16

	//Configuration parameters 
	Frac16	f16IGain; 	//current				//17
	Frac16	f16UGain; 	//voltage				//18
	Frac16	f16WIGain;	//decoupling			//19
	Frac16	f16EGain;	//extended BEMF			//20
	
} ACLIB_BEMF_OBSRV_DQ_T;


/******************************************************************************
* Global variables
******************************************************************************/
   
/******************************************************************************
* Global functions
******************************************************************************/


/***************************************************************************//*!
*
* @brief  BEMF observer using D, Q components to calculate error
*		  BEMF gamma/delta.
*
* @param  ptr   		MCLIB_2_COOR_SYST_D_Q_T *pudtCurrentDQ
*                         - pointer to the D, Q current
*                       
*                       MCLIB_2_COOR_SYST_D_Q_T *pudtVoltageDQ
*                         - pointer to the D, Q voltage
*
*                       ACLIB_BEMF_OBSRV_DQ_T *pudtCtrl
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
extern asm void ACLIB_PMSMBemfObsrvDQFAsm
(
    MCLIB_2_COOR_SYST_D_Q_T *pudtCurrentDQ,
    MCLIB_2_COOR_SYST_D_Q_T *pudtVoltageDQ,
    Frac16 f16Speed,
    ACLIB_BEMF_OBSRV_DQ_T * pudtCtrl
);

/***************************************************************************//*!
*
* @brief  BEMF observer using D, Q components to calculate error
*		  BEMF gamma/delta.
*
* @param  ptr   		MCLIB_2_COOR_SYST_D_Q_T *pudtCurrentDQ
*                         - pointer to the D, Q current
*                       
*                       MCLIB_2_COOR_SYST_D_Q_T *pudtVoltageDQ
*                         - pointer to the D, Q voltage
*
*                       ACLIB_BEMF_OBSRV_DQ_T *pudtCtrl
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
extern asm void ACLIB_V3PMSMBemfObsrvDQFAsm
(
    MCLIB_2_COOR_SYST_D_Q_T *pudtCurrentDQ,
    MCLIB_2_COOR_SYST_D_Q_T *pudtVoltageDQ,
    Frac16 f16Speed,
    ACLIB_BEMF_OBSRV_DQ_T * pudtCtrl
);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _ACLIB_PMSM_BEMF_OBSRV_DQ_ASM_H_ */	
