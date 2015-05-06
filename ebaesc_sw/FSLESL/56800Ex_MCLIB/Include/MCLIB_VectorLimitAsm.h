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
* @file      MCLIB_VectorLimitAsm.h
*
* @author    r61928
* 
* @version   1.0.1.0
* 
* @date      Feb-26-2013
* 
* @brief     Vector limitation calculation
*
*******************************************************************************
*
* Vector limitation calculation.
*
******************************************************************************/
#ifndef _MCLIB_VECTORLIMITASM_H_
#define _MCLIB_VECTORLIMITASM_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "56800E_types.h"
#include "MCLIB_types.h"
#include "gflib.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define	MCLIB_VectorLimitAsm(pudtInVector, pudtLimVector, pudtParams) \
			MCLIB_VectorLimitFAsm(pudtInVector, pudtLimVector, pudtParams)

#define	MCLIB_VectorLimit12Asm(pudtInVector, pudtLimVector, pudtParams) \
			MCLIB_VectorLimit12FAsm(pudtInVector, pudtLimVector, pudtParams)

#define MCLIB_VECTOR_LIMIT_PARAMS_DEFAULT {32767,0}

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
   Frac16 	f16Lim;
   bool 	blnLimFlag;
} MCLIB_VECTOR_LIMIT_PARAMS_T;


/******************************************************************************
* Global variables
******************************************************************************/


/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  Vector Limit function
*
* @param  ptr			MCLIB_2_COOR_SYST_T *pudtInVector    
*						  - f16A: IN d component to be limited
*						  - f16B: IN q component to be limited
*
*           			MCLIB_2_COOR_SYST_T *pudtLimVector    
*						  - f16A: OUT d component limited
*						  - f16B: OUT q component limited
*
* 						MCLIB_VECTOR_LIMIT_PARAMS_T *pudtParams    
*						  - f16Lim: IN limit parameter
*						  - blnLimFlag: OUT 0 - not limited, 1 - limited
*
* @param  in N/A
*
* @return N/A
*		
* @remarks 	The function limits the amplitude of the input vector. The input
*           vector dq components pudtInVector.f16A and pudtInVector.f16B are passed into the function
*           as the input arguments. The resulting limited vector is transformed
*           back into the dq components pudtLimVector.f16A, pudtLimVector.f16B. The function uses
*           GFLIB_Sqrt to calculate the modulus of the input
*           vector.
*
*			THE SATURATION MUST BE TURNED OFF!
*
******************************************************************************/
extern asm void MCLIB_VectorLimitFAsm(MCLIB_2_COOR_SYST_T *pudtInVector,
							  		  MCLIB_2_COOR_SYST_T *pudtLimVector,
							   		  MCLIB_VECTOR_LIMIT_PARAMS_T *pudtParams);

/***************************************************************************//*!
*
* @brief  Vector Limit function with lower resolution but faster calculation of
*		  SQRT calculation
*		
*
* @param  ptr			MCLIB_2_COOR_SYST_T *pudtInVector    
*						  - f16A: IN d component to be limited
*						  - f16B: IN q component to be limited
*
*           			MCLIB_2_COOR_SYST_T *pudtLimVector    
*						  - f16A: OUT d component limited
*						  - f16B: OUT q component limited
*
* 						MCLIB_VECTOR_LIMIT_PARAMS_T *pudtParams    
*						  - f16Lim: IN limit parameter
*						  - blnLimFlag: OUT 0 - not limited, 1 - limited
*
* @param  in N/A
*
* @return N/A
*		
* @remarks 	The function limits the amplitude of the input vector. The input
*           vector dq components pudtInVector.f16A and pudtInVector.f16B are passed into the function
*           as the input arguments. The resulting limited vector is transformed
*           back into the dq components pudtLimVector.f16A, pudtLimVector.f16B. The function uses
*           GFLIB_Sqrt to calculate the modulus of the input
*           vector.
*
*			THE SATURATION MUST BE TURNED OFF!
*
******************************************************************************/
extern asm void MCLIB_VectorLimit12FAsm(MCLIB_2_COOR_SYST_T *pudtInVector,
	          					        MCLIB_2_COOR_SYST_T *pudtLimVector,
							     		MCLIB_VECTOR_LIMIT_PARAMS_T *pudtParams);

#endif /* _MCLIB_VECTOR_LIMIT_H_ */
