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
* @file      GFLIB_SqrtDefAsm.h
*
* @author    R61928
* 
* @version   1.0.2.0
* 
* @date      Aug-15-2013
* 
* @brief     Sqrt table and macros.
*
*******************************************************************************
*
* Contains a table of constants that are used for the Sqrt polynom calculation.
*
******************************************************************************/
#ifndef _GFLIB_SQRTDEFASM_H_
#define _GFLIB_SQRTDEFASM_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "GFLIB_SqrtAsm.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define GFLIB_SqrtPolyAsm(f32Arg) GFLIB_SqrtPolyFAsm(f32Arg, &gudtSqrtPolyTable)

/******************************************************************************
* Types
******************************************************************************/

/******************************************************************************
* Global constants
******************************************************************************/

/******************************************************************************
* Global variables
******************************************************************************/
extern GFLIB_SQRT_POLY_TABLE_T gudtSqrtPolyTable;

/******************************************************************************
* Global functions
******************************************************************************/

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _GFLIB_SQRTDEFASM_H_ */

