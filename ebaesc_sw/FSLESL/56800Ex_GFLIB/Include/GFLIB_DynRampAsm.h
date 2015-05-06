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
* @file      GFLIB_DynRampAsm.h
*
* @author    R61928
* 
* @version   1.0.3.0
* 
* @date      Aug-15-2013
* 
* @brief     Dynamic ramp functions in assembler
*
*******************************************************************************
*
* Dynamic ramp functions in assembler.
*
******************************************************************************/
#ifndef _GFLIB_DYNRAMPASM_H_
#define _GFLIB_DYNRAMPASM_H_

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
#define GFLIB_DynRamp16InitValAsm(f16InitVal, pudtParam) \
			GFLIB_DynRamp16InitValFAsm(f16InitVal, pudtParam);

#define GFLIB_DynRamp32InitValAsm(f32InitVal, pudtParam) \
			GFLIB_DynRamp32InitValFAsm(f32InitVal, pudtParam);

#define GFLIB_DynRamp16Asm(f16Desired, f16Instant, uw16SatFlag, pudtParam) \
			GFLIB_DynRamp16FAsm(f16Desired, f16Instant, uw16SatFlag, pudtParam)

#define GFLIB_DynRamp32Asm(f32Desired, f32Instant, uw16SatFlag, pudtParam) \
			GFLIB_DynRamp32FAsm(f32Desired, f32Instant, uw16SatFlag, pudtParam)
						   			  
/******************************************************************************
* Types
******************************************************************************/
/* Ramp structure */
typedef struct
{
    Frac16 f16RampUp;
    Frac16 f16RampDown;
	Frac16 f16RampUpSat;
	Frac16 f16RampDownSat;
	Frac16 f16Actual;
} GFLIB_DYNRAMP16_T;

/* Ramp structure */
typedef struct
{
    Frac32 f32RampUp;
    Frac32 f32RampDown;
	Frac32 f32RampUpSat;
	Frac32 f32RampDownSat;
	Frac32 f32Actual;
} GFLIB_DYNRAMP32_T;

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  The function initializes the actual value of DynRamp16.
*
* @param  ptr   		GFLIB_RAMP16 *pudtParam
*						  - f16RampUp: Ramp-up increment
*						  - f16RampDown: Ramp-down increment
*						  - f16RampUpSat: Ramp-up increment used in case of saturation
*						  - f16RampDownSat: Ramp-down increment used in case of saturation
*						  - f16Actual: Previous ramp value
*
* @param  in    		Frac16 f16InitVal
* 							- Initial value
*
* @return N/A
*		
* @remarks 
*
****************************************************************************/
extern asm void GFLIB_DynRamp16InitValFAsm(Frac16 f16InitVal, GFLIB_DYNRAMP16_T *pudtParam);

/***************************************************************************//*!
*
* @brief  The function initializes the actual value of DynRamp32.
*
* @param  ptr   		GFLIB_RAMP32 *pudtParam
*						  - f32RampUp: Ramp-up increment
*						  - f32RampDown: Ramp-down increment
*						  - f32RampUpSat: Ramp-up increment used in case of saturation
*						  - f32RampDownSat: Ramp-down increment used in case of saturation
*						  - f32Actaul: Previous ramp value
*
* @param  in    		Frac16 f16InitVal
* 							- Initial value
*
* @return N/A
*		
* @remarks 
*
****************************************************************************/
extern asm void GFLIB_DynRamp32InitValFAsm(Frac32 f32InitVal, GFLIB_DYNRAMP32_T *pudtParam);

/***************************************************************************//*!
*
* @brief  Ramp function
*
* @param  ptr			GFLIB_RAMP16 *pudtParam
*						  - f16RampUp: Ramp-up increment
*						  - f16RampDown: Ramp-down increment
*						  - f16RampUpSat: Ramp-up increment used in case of saturation
*						  - f16RampDownSat: Ramp-down increment used in case of saturation
*						  - f16Actual: Previous ramp value
* @param  in    		Frac16 f16Desired
*                         - Desired value in [-1;1] in Frac16
*						Frac16 f16Instant
*						  - Instant value in [-1;1] in Frac16
*						UWord16 uw16SatFlag
*						  - determines the saturation mode: 0 non-saturation
*
* @return This function returns
*     - Frac16 value [-1;1]
*		
* @remarks 	This function ramps the value from the actual value up/down to
*			the f16Desired value using the up/down increments defined in
*			the pParam structure. In case of saturation (uw16SatFlag != 0)
*			the function uses the saturation up/down increments and ramps
*			the value toward the f16Instant value.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm Frac16 GFLIB_DynRamp16FAsm(Frac16 f16Desired,
									  Frac16 f16Instant,
								   	  UWord16 uw16SatFlag,
								   	  GFLIB_DYNRAMP16_T *pudtParam);


/***************************************************************************//*!
*
* @brief  Ramp function 32-bit version
*
* @param  ptr			GFLIB_RAMP32 *pudtParam
*						  - f32RampUp: Ramp-up increment
*						  - f32RampDown: Ramp-down increment
*						  - f32RampUpSat: Ramp-up increment used in case of saturation
*						  - f32RampDownSat: Ramp-down increment used in case of saturation
*						  - f32Actaul: Previous ramp value
* @param  in    		Frac32 f32Desired
*                         - Desired value in [-1;1] in Frac32
*						Frac32 f32Instant
*						  - Instant value in [-1;1] in Frac32
*						UWord16 uw16SatFlag
*						  - determines the saturation mode: 0 non-saturation
*
* @return This function returns
*     - Frac32 value [-1;1]
*		
* @remarks 	This function ramps the value from the actual value up/down to
*			the f16Desired value using the up/down increments defined in
*			the pParam structure. In case of saturation (uw16SatFlag != 0)
*			the function uses the saturation up/down increments and ramps
*			the value toward the f16Instant value.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm Frac32 GFLIB_DynRamp32FAsm(Frac32 f32Desired,
									  Frac32 f32Instant,
						   			  UWord16 uw16SatFlag,
						   			  GFLIB_DYNRAMP32_T *pudtParam);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _GFLIB_DYNRAMPASM_H_ */

