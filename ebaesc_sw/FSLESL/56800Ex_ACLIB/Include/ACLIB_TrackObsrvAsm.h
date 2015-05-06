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
* @file      ACLIB_TrackObsrvAsm.h
*
* @author    r61928
* 
* @version   1.0.3.0
* 
* @date      Jan-8-2014
* 
* @brief     Header for controller tracking observer in assembler
*
*******************************************************************************
*
* Header for controller tracking observer in assembler.
*
******************************************************************************/
#ifndef _ACLIB_TRACK_OBSRV_ASM_H
#define _ACLIB_TRACK_OBSRV_ASM_H

/******************************************************************************
* Includes
******************************************************************************/
#include "56800E_types.h"
#include "mclib.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define ACLIB_TrackObsrvAsm(f16Error, pudtCtrl) 	ACLIB_TrackObsrvFAsm(f16Error, pudtCtrl)

/* The V3 core instructions */
#define ACLIB_V3TrackObsrvAsm(f16Error, pudtCtrl) 	ACLIB_V3TrackObsrvFAsm(f16Error, pudtCtrl)

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{

	Frac32	f32Theta;
	Frac32	f32Speed;
	Frac32	f32I_1;
	Frac16	f16IntegGain;
	Frac16	i16IntegGainShift;
	Frac16	f16PropGain;
	Frac16	i16PropGainShift;
	Frac16	f16ThGain;
	Frac16	i16ThGainShift;
	
} ACLIB_TRACK_OBSRV_T;

/******************************************************************************
* Global variables
******************************************************************************/
   
/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  			Tracking observer using angle error calculates 
*					estimated position.
*
* @param  ptr   	ACLIB_TRACK_OBSRV_T *pudtCtrl
*                         - pointer to control structure
*
*
* @param  in    	Frac16 f16Error
*                         - error of angle
*
* @return 			f16Theta
*                         - estimated position
*		
* @remarks 			THE SATURATION MUST BE TURNED OFF!
*
****************************************************************************/
extern asm Frac16 ACLIB_TrackObsrvFAsm
(
    Frac16 f16ThetaErr, 
    ACLIB_TRACK_OBSRV_T * const pudtCtrl
);

/***************************************************************************//*!
*
* @brief  			Tracking observer using angle error calculates 
*					estimated position.
*
* @param  ptr   	ACLIB_TRACK_OBSRV_T *pudtCtrl
*                         - pointer to control structure
*
*
* @param  in    	Frac16 f16Error
*                         - error of angle
*
* @return 			f16Theta
*                         - estimated position
*		
* @remarks			THE SATURATION MUST BE TURNED OFF!
*		   			The V3 core instructions used! 	
****************************************************************************/
extern asm Frac16 ACLIB_V3TrackObsrvFAsm
(
    Frac16 f16ThetaErr, 
    ACLIB_TRACK_OBSRV_T * const pudtCtrl
);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _ACLIB_ANGLE_TRACK_OBSRV_ASM_H */
