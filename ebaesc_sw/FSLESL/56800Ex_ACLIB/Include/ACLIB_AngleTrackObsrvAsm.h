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
* @file      ACLIB_AngleTrackObsrvAsm.h
*
* @author    r61928
* 
* @version   1.0.3.0
* 
* @date      Jan-8-2014
* 
* @brief     Angle tracking observer
*
*******************************************************************************
*
* Angle tracking observer.
*
******************************************************************************/
#ifndef _ACLIB_ANGLE_TRACK_OBSRV_ASM_H_
#define _ACLIB_ANGLE_TRACK_OBSRV_ASM_H_

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
#define ACLIB_AngleTrackObsrvAsm(pudtSinCos, pudtCtrl) 	ACLIB_AngleTrackObsrvFAsm(pudtSinCos, pudtCtrl)
#define ACLIB_AngleTrackObsrv12Asm(pudtSinCos, pudtCtrl) 	ACLIB_AngleTrackObsrv12FAsm(pudtSinCos, pudtCtrl)

/* V3 core instructions */
#define ACLIB_V3AngleTrackObsrvAsm(pudtSinCos, pudtCtrl) 	ACLIB_V3AngleTrackObsrvFAsm(pudtSinCos, pudtCtrl)
#define ACLIB_V3AngleTrackObsrv12Asm(pudtSinCos, pudtCtrl) 	ACLIB_V3AngleTrackObsrv12FAsm(pudtSinCos, pudtCtrl)

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
	Frac32	f32Speed;       	//0
	Frac32	f32A2;          	//2
    Frac16  f16Theta;       	//4
	Frac16	f16SinEstim;    	//5
	Frac16	f16CosEstim;    	//6
	Frac16	f16K1Gain;    		//7
	Int16	i16K1GainShift;     //8
	Frac16	f16K2Gain;    		//9
	Int16	i16K2GainShift;     //10
	Frac16	f16A2Gain;    		//11
	Int16	i16A2GainShift;     //12
		
} ACLIB_ANGLE_TRACK_OBSRV_T;

/******************************************************************************
* Global variables
******************************************************************************/
   
/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  Tracks the angle of the system and calculates the speed.
*
* @param  ptr   		MCLIB_ANGLE_T *pudtSinCos
*                         - angle's sine and cosine components
*
*                       ACLIB_ANGLE_TRACK_OBSRV_T * const pudtCtrl
*                         - parameters of the observer
*
* @param  in    		None
*
* @return This function returns the calculated angle
*		
* @remarks SATURATION MUST BE TURNED OFF! 	
*
****************************************************************************/
extern asm Frac16 ACLIB_AngleTrackObsrvFAsm
(
    MCLIB_ANGLE_T *pudtSinCos,
    ACLIB_ANGLE_TRACK_OBSRV_T * const pudtCtrl
);

/***************************************************************************//*!
*
* @brief  Tracks the angle of the system and calculates the speed. Reduced
*		  precision of the algorithm by the use of faster 12-bit precision
*		  Sin/Cos algorithms.
*
* @param  ptr   		MCLIB_ANGLE_T *pudtSinCos
*                         - angle's sine and cosine components
*
*                       ACLIB_ANGLE_TRACK_OBSRV_T * const pudtCtrl
*                         - parameters of the observer
*
* @param  in    		None
*
* @return This function returns the calculated angle
*		
* @remarks SATURATION MUST BE TURNED OFF! 	
*
****************************************************************************/
extern asm Frac16 ACLIB_AngleTrackObsrv12FAsm
(
    MCLIB_ANGLE_T *pudtSinCos,
    ACLIB_ANGLE_TRACK_OBSRV_T * const pudtCtrl
);


/***************************************************************************//*!
*
* @brief  Tracks the angle of the system and calculates the speed.
*
* @param  ptr   		MCLIB_ANGLE_T *pudtSinCos
*                         - angle's sine and cosine components
*
*                       ACLIB_ANGLE_TRACK_OBSRV_T * const pudtCtrl
*                         - parameters of the observer
*
* @param  in    		None
*
* @return This function returns the calculated angle
*		
* @remarks SATURATION MUST BE TURNED OFF!
* 		   The V3 core instructions used! 	
*
****************************************************************************/
extern asm Frac16 ACLIB_V3AngleTrackObsrvFAsm
(
    MCLIB_ANGLE_T *pudtSinCos,
    ACLIB_ANGLE_TRACK_OBSRV_T * const pudtCtrl
);

/***************************************************************************//*!
*
* @brief  Tracks the angle of the system and calculates the speed. Reduced
*		  precision of the algorithm by the use of faster 12-bit precision
*		  Sin/Cos algorithms.
*
* @param  ptr   		MCLIB_ANGLE_T *pudtSinCos
*                         - angle's sine and cosine components
*
*                       ACLIB_ANGLE_TRACK_OBSRV_T * const pudtCtrl
*                         - parameters of the observer
*
* @param  in    		None
*
* @return This function returns the calculated angle
*		
* @remarks SATURATION MUST BE TURNED OFF!
*		   The V3 core instructions used! 		
*
****************************************************************************/
extern asm Frac16 ACLIB_V3AngleTrackObsrv12FAsm
(
    MCLIB_ANGLE_T *pudtSinCos,
    ACLIB_ANGLE_TRACK_OBSRV_T * const pudtCtrl
);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _ACLIB_ANGLE_TRACK_OBSRV_ASM_H_ */
