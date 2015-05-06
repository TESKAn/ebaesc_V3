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
* @file      mclib_types.h
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Feb-26-2013
* 
* @brief     Types related to MCLIB funcitons.
*
*******************************************************************************
*
* Types related to MCLIB funcitons.
*
******************************************************************************/
#ifndef _MCLIB_TYPES_H_
#define _MCLIB_TYPES_H_

#include "56800E_types.h"

/* Data type defintion(s)                                           */
typedef struct
{
    Frac16 f16A;
    Frac16 f16B;
    Frac16 f16C;
} MCLIB_3_COOR_SYST_T;

typedef struct
{
    Frac16 f16A;
    Frac16 f16B;
} MCLIB_2_COOR_SYST_T;

typedef struct
{
    Frac16 f16Alpha;
    Frac16 f16Beta;
} MCLIB_2_COOR_SYST_ALPHA_BETA_T;

typedef struct
{
    Frac16 f16D;
    Frac16 f16Q;
} MCLIB_2_COOR_SYST_D_Q_T;

typedef struct
{
    Frac16 f16Sin;
    Frac16 f16Cos;
} MCLIB_ANGLE_T;

#endif /* _MCLIB_TYPES_H_ */
