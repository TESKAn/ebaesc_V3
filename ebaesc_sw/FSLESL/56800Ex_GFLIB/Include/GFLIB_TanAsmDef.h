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
* @file      GFLIB_TanAsmDef.h
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Feb-26-2013
* 
* @brief     Tables for the Tan algorithm using piece-wise polynomial approximation
*
* Special Issues: The function requires the saturation mode to be set.
*
*******************************************************************************
*
* Tables for the Tan algorithm using piece-wise polynomial approximation.
*
******************************************************************************/
#ifndef _GFLIB_TANASMDEF_H_
#define _GFLIB_TANASMDEF_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "GFLIB_TanAsm.h"

/******************************************************************************
* Constants
******************************************************************************/


/******************************************************************************
* Macros 
******************************************************************************/
#define GFLIB_TanAsm(f16Arg)   GFLIB_TanFAsm(f16Arg, &gudtTanTabAddr)

/******************************************************************************
* Types
******************************************************************************/

/******************************************************************************
* Global variables
******************************************************************************/
extern GFLIB_TAN_COEFFICIENTS_ADDR_T gudtTanTabAddr;

/******************************************************************************
* Global functions
******************************************************************************/

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _GFLIB_TANASMDEF_H_ */

