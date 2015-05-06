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
* @file      GFLIB_AtanYXShiftedAsmDef.h
*
* @author    R61928
* 
* @version   1.0.2.0
* 
* @date      Mar-21-2013
* 
* @brief     Function AtanYXShifted
*
*
*******************************************************************************
*
* Function AtanYXShifted.
*
******************************************************************************/
#ifndef _GFLIB_ATANYXSHIFTEDASMDEF_H_
#define _GFLIB_ATANYXSHIFTEDASMDEF_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "GFLIB_AtanYXShiftedAsm.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/


/******************************************************************************
* Types
******************************************************************************/


/******************************************************************************
* Global variables
******************************************************************************/
extern AtanYXPolyCoefficientAddr ATANYXTABADDR;

/******************************************************************************
* Global functions
******************************************************************************/
extern Frac16 GFLIB_AtanYXShiftedAsm(register Frac16 nValY, register Frac16 nValX,
									 AtanYXPolyCoefficientAddr *pAtanYXPoly);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _GFLIB_ATANYXSHIFTEDASMDEF_H_ */

