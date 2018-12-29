/*******************************************************************************
*
* Copyright 2007-2015 Freescale Semiconductor, Inc.
*
* This software is owned or controlled by Freescale Semiconductor.
* Use of this software is governed by the Freescale License
* distributed with this Material.
* See the LICENSE file distributed for more details.
* 
*
****************************************************************************//*!
*
* @brief  Space vector modulation functions in assembler
* 
*******************************************************************************/
#ifndef _GMCLIB_SVM_F16_ASM_H_
#define _GMCLIB_SVM_F16_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib.h"
#include "gmclib_types.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define	GMCLIB_SvmStd_F16_Asm(psIn, psOut) GMCLIB_SvmStd_F16_FAsm(psIn, psOut)
#define	GMCLIB_SvmIct_F16_Asm(psIn, psOut) GMCLIB_SvmIct_F16_FAsm(psIn, psOut)
#define	GMCLIB_SvmU0n_F16_Asm(psIn, psOut) GMCLIB_SvmU0n_F16_FAsm(psIn, psOut)
#define	GMCLIB_SvmU7n_F16_Asm(psIn, psOut) GMCLIB_SvmU7n_F16_FAsm(psIn, psOut)

/******************************************************************************
* Types
******************************************************************************/

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/

extern asm uint16_t GMCLIB_SvmStd_F16_FAsm(const GMCLIB_2COOR_ALBE_T_F16 *psIn, GMCLIB_3COOR_T_F16 *psOut);
    
extern asm uint16_t GMCLIB_SvmIct_F16_FAsm(const GMCLIB_2COOR_ALBE_T_F16 *psIn, GMCLIB_3COOR_T_F16 *psOut);                            

extern asm uint16_t GMCLIB_SvmU0n_F16_FAsm(const GMCLIB_2COOR_ALBE_T_F16 *psIn, GMCLIB_3COOR_T_F16 *psOut);                            

extern asm uint16_t GMCLIB_SvmU7n_F16_FAsm(const GMCLIB_2COOR_ALBE_T_F16 *psIn, GMCLIB_3COOR_T_F16 *psOut);                            


/******************************************************************************
* Inline functions
******************************************************************************/
#if defined(__cplusplus) 
} 
#endif 

#endif /* _GMCLIB_SVM_F16_ASM_H_ */

