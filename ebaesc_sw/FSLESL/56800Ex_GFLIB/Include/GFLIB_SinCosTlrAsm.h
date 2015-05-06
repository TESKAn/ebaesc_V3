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
* @file      GFLIB_SinCosTlrAsm.h
*
* @author    R61928
* 
* @version   1.0.1.0
* 
* @date      Feb-26-2013
* 
* @brief     Sine & cosine algorithm using Taylor 9th order polynomial
*			 - Optimized for speed using parallel moves
*
*******************************************************************************
*
* Sine & cosine algorithm using Taylor 9th order polynomial. Optimized
* for speed using parallel moves.
*
******************************************************************************/
#ifndef _GFLIB_SINCOSTLRASM_H_
#define _GFLIB_SINCOSTLRASM_H_

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

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
	Frac32  f32A[5];
} GFLIB_SIN_TAYLOR_COEF_T;

typedef struct
{
	Frac16  f16A[5];
} GFLIB_SIN12_TAYLOR_COEF_T;

/******************************************************************************
* Global variables
******************************************************************************/
   
/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  Calculates the sine of the given argument using Taylor 9th order
*         polynomial.
*
* @param  ptr		Frac16 *pudtSinData
*					- Pointer to the Taylor polynomial coefficients
* @param  in    	Frac16 f16In
*                   - Argument in range [-1; 1) in Frac16 corresponding
*					  to [-pi; pi)
*
* @return This function returns
*     				- Frac16 value [-1; 1)
*		
* @remarks 	
*	This function calculates sin(pi * x) using Taylor 9th order polynomial:
*	x=[-pi; pi]:
*		sine(x) = x - x^3/3! + x^5/5! - x^7/7! + x^9/9!
*		sine(x) = x*(1 - x^2/6*(1 - x^2/20*(1 - x^2/42*(1 - x^2/72))))
*		sine(x) = x(d1 + x^2(d3 + x^2(d5 + x^2(d7 + x^2*d9))))
*
*		where:
*		d1 =  1
*		d3 = -1 / 3!
*		d5 =  1 / 5!
*		d7 = -1 / 7!
*		d9 =  1 / 9!
*
*	because the x value is in range [-1; 1) that corresponds to [-pi; pi],
*	the coefficients 'd' needs to be scaled:
*
*		x=[-1; 1):
*		sin(pi * x) = x(c1 + x^2(c3 + x^2(c5 + x^2(c7 + x^2*c9))))
*
*		c1 = d1 * pi^1 =  pi
*		c3 = d3 * pi^3 = -pi^3 / 3!
*		c5 = d5 * pi^5 =  pi^5 / 5!
*		c7 = d7 * pi^7 = -pi^7 / 7!
*		c9 = d9 * pi^9 =  pi^9 / 9!
*
*	The 9th order polynomial approximation of the sine function
*	has very good accuracy in range [-pi/2; pi/2] of argument, but in wider
*	range the calculation error is quickly growing up. Therefore the sine
*	function is	approximated only in range [-pi/2; pi/2] and it is used the
*	symmetry of the sine function [ sin(alpha) = sin(pi - alpha) ].
*	Therefore the alpha argument is in range [-pi/2; pi/2) that corresponds
*	to x [-0.5; 0.5).
*	To make calculations more precise (because we need to use the value
*	of x^2 rounded to 16-bit fractional number in our calculations),
*	the given argument value x (that is in range [-0.5; 0.5)) is shifted
*	by 1 bit to the left (multiplied by 2), then the calculated x^2 value
*	is in range [-1; 1) instead of [-0.25; 0.25).
*	Then the polynomial coefficients 'c' needs to be scaled
*	(shifted to the right):
*
*		b1 = c1 / 2^1 =  pi / 2
*		b3 = c3 / 2^3 = -pi^3 / 3! / 2^3
*		b5 = c5 / 2^5 =  pi^5 / 5! / 2^5
*		b7 = c7 / 2^7 = -pi^7 / 7! / 2^7
*		b9 = c9 / 2^9 =  pi^9 / 9! / 2^9
*
*	To avoid the saturation error during the polynomial calculation
*	the coefficients 'b' are divided by 2. After the polynomial calculation
*	the result is multiplied by 2 (shifted by 1 bit to the left)
*	to take the right result ( in range [-1; 1) ).
*
*		a1 = b1 / 2 =  pi / 2^2         =  0.785398163
*		a3 = b3 / 2 = -pi^3 / 3! / 2^4  = -0.322982049
*		a5 = b5 / 2 =  pi^5 / 5! / 2^6  =  0.039846313
*		a7 = b7 / 2 = -pi^7 / 7! / 2^8  = -0.002340877
*		a9 = b9 / 2 =  pi^9 / 9! / 2^10 =  0.000080220
*
*		In 32-bit signed fractional representation:
*		a1 = 0x6487ED51
*		a3 = 0xD6A88634
*		a5 = 0x0519AF1A
*		a7 = 0xFFB34B4D
*		a9 = 0x0002A0F0
*
*	sin(pi * x) = (x<<1)*(a1 + (x<<1)^2(a3 + (x<<1)^2(a5 + (x<<1)^2(a7 \
*				  + (x<<1)^2*a9)))) << 1
*
*	For better accuracy the (x<<1)^2*(a#...) calculations are performed
*	as the following:
*		(x<<1)^2 as 16-bit fractional value
*			  a# as 32-bit fractional value
*		[ (x<<1)^2 ] * [ (a#... ] = [16-bit]*[32-bit] = 32-bit number
*
* 			SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm Frac16 GFLIB_SinTlrFAsm(Frac16 f16In, const GFLIB_SIN_TAYLOR_COEF_T *pudtSinData);

/***************************************************************************//*!
*
* @brief  Calculates the cosine of the given argument using Taylor 9th order
*         polynomial.
*
* @param  ptr		Frac16 *pudtSinData
*					- Pointer to the Taylor polynomial coefficients
* @param  in    	Frac16 f16In
*                   - Argument in range [-1; 1) in Frac16 corresponding
*					  to [-pi; pi)
*
* @return This function returns
*     				- Frac16 value [-1; 1)
*		
* @remarks 	
*	This function calculates cos(pi * x) using the Taylor 9th order
*	polynomial approximation of sine function.
*	At the beginning the input argument of cosine function
*	is transferred to argument of sine function:
*
*	sine_argument = pi/2 - ABS(cos_argument)
*
*	then the cosine function is calculated using the sine function.
*	The constant pi/2 can not be directly added to the cos_argument,
*	because the saturation is enabled, therefore the saturation
*	can occur during the addition.
*
*	The function calculates sin(pi * x) using the Taylor 9th order	polynomial:
*	x=[-pi; pi]:
*		sine(x) = x - x^3/3! + x^5/5! - x^7/7! + x^9/9!
*		sine(x) = x*(1 - x^2/6*(1 - x^2/20*(1 - x^2/42*(1 - x^2/72))))
*		sine(x) = x(d1 + x^2(d3 + x^2(d5 + x^2(d7 + x^2*d9))))
*
*		where:
*		d1 =  1
*		d3 = -1 / 3!
*		d5 =  1 / 5!
*		d7 = -1 / 7!
*		d9 =  1 / 9!
*
*	because the x value is in range [-1; 1) that corresponds to [-pi; pi],
*	the coefficients 'd' needs to be scaled:
*
*		x=[-1; 1):
*		sin(pi * x) = x(c1 + x^2(c3 + x^2(c5 + x^2(c7 + x^2*c9))))
*
*		c1 = d1 * pi^1 =  pi
*		c3 = d3 * pi^3 = -pi^3 / 3!
*		c5 = d5 * pi^5 =  pi^5 / 5!
*		c7 = d7 * pi^7 = -pi^7 / 7!
*		c9 = d9 * pi^9 =  pi^9 / 9!
*
*	The 9th order polynomial approximation of the sine function
*	has very good accuracy in range [-pi/2; pi/2] of argument, but in wider
*	range the calculation error is quickly growing up. Therefore the sine
*	function is	approximated only in range [-pi/2; pi/2] and it is used the
*	symmetry of the sine function [ sin(alpha) = sin(pi - alpha) ].
*	Therefore the alpha argument is in range [-pi/2; pi/2) that corresponds
*	to x [-0.5; 0.5).
*	To make calculations more precise (because we need to use the value
*	of x^2 rounded to 16-bit fractional number in our calculations),
*	the given argument value x (that is in range [-0.5; 0.5)) is shifted
*	by 1 bit to the left (multiplied by 2), then the calculated x^2 value
*	is in range [-1; 1) instead of [-0.25; 0.25).
*	Then the polynomial coefficients 'c' needs to be scaled
*	(shifted to the right):
*
*		b1 = c1 / 2^1 =  pi / 2
*		b3 = c3 / 2^3 = -pi^3 / 3! / 2^3
*		b5 = c5 / 2^5 =  pi^5 / 5! / 2^5
*		b7 = c7 / 2^7 = -pi^7 / 7! / 2^7
*		b9 = c9 / 2^9 =  pi^9 / 9! / 2^9
*
*	To avoid the saturation error during the polynomial calculation
*	the coefficients 'b' are divided by 2. After the polynomial calculation
*	the result is multiplied by 2 (shifted by 1 bit to the left)
*	to take the right result ( in range [-1; 1) ).
*
*		a1 = b1 / 2 =  pi / 2^2         =  0.785398163
*		a3 = b3 / 2 = -pi^3 / 3! / 2^4  = -0.322982049
*		a5 = b5 / 2 =  pi^5 / 5! / 2^6  =  0.039846313
*		a7 = b7 / 2 = -pi^7 / 7! / 2^8  = -0.002340877
*		a9 = b9 / 2 =  pi^9 / 9! / 2^10 =  0.000080220
*
*		In 32-bit signed fractional representation:
*		a1 = 0x6487ED51
*		a3 = 0xD6A88634
*		a5 = 0x0519AF1A
*		a7 = 0xFFB34B4D
*		a9 = 0x0002A0F0
*
*	sin(pi * x) = (x<<1)*(a1 + (x<<1)^2(a3 + (x<<1)^2(a5 + (x<<1)^2(a7 \
*				  + (x<<1)^2*a9)))) << 1
*
*	For better accuracy the (x<<1)^2*(a#...) calculations are performed
*	as the following:
*		(x<<1)^2 as 16-bit fractional value
*			  a# as 32-bit fractional value
*		[ (x<<1)^2 ] * [ (a#... ] = [16-bit]*[32-bit] = 32-bit number
*
* 			SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm Frac16 GFLIB_CosTlrFAsm(Frac16 f16In, const GFLIB_SIN_TAYLOR_COEF_T *pudtSinData);

/***************************************************************************//*!
*
* @brief  Calculates the cosine of the given argument using Taylor 9th order
*         polynomial. Inline function which increments the argument by 0.5 and
*		  calls the Sin calculation.
*
* @param  ptr		Frac16 *pudtSinData
*					- Pointer to the Taylor polynomial coefficients
* @param  in    	Frac16 f16In
*                   - Argument in range [-1; 1) in Frac16 corresponding
*					  to [-pi; pi)
*
* @return This function returns
*     				- Frac16 value [-1; 1)
*		
* @remarks 	
*	This function calculates cos(pi * x) using the Taylor 9th order
*	polynomial approximation of sine function.
*	At the beginning the input argument of cosine function
*	is transferred to argument of sine function:
*
*	sine_argument = pi/2 - ABS(cos_argument)
*
*	then the cosine function is calculated using the sine function.
*	The constant pi/2 can not be directly added to the cos_argument,
*	because the saturation is enabled, therefore the saturation
*	can occur during the addition.
*
*	The function calculates sin(pi * x) using the Taylor 9th order	polynomial:
*	x=[-pi; pi]:
*		sine(x) = x - x^3/3! + x^5/5! - x^7/7! + x^9/9!
*		sine(x) = x*(1 - x^2/6*(1 - x^2/20*(1 - x^2/42*(1 - x^2/72))))
*		sine(x) = x(d1 + x^2(d3 + x^2(d5 + x^2(d7 + x^2*d9))))
*
*		where:
*		d1 =  1
*		d3 = -1 / 3!
*		d5 =  1 / 5!
*		d7 = -1 / 7!
*		d9 =  1 / 9!
*
*	because the x value is in range [-1; 1) that corresponds to [-pi; pi],
*	the coefficients 'd' needs to be scaled:
*
*		x=[-1; 1):
*		sin(pi * x) = x(c1 + x^2(c3 + x^2(c5 + x^2(c7 + x^2*c9))))
*
*		c1 = d1 * pi^1 =  pi
*		c3 = d3 * pi^3 = -pi^3 / 3!
*		c5 = d5 * pi^5 =  pi^5 / 5!
*		c7 = d7 * pi^7 = -pi^7 / 7!
*		c9 = d9 * pi^9 =  pi^9 / 9!
*
*	The 9th order polynomial approximation of the sine function
*	has very good accuracy in range [-pi/2; pi/2] of argument, but in wider
*	range the calculation error is quickly growing up. Therefore the sine
*	function is	approximated only in range [-pi/2; pi/2] and it is used the
*	symmetry of the sine function [ sin(alpha) = sin(pi - alpha) ].
*	Therefore the alpha argument is in range [-pi/2; pi/2) that corresponds
*	to x [-0.5; 0.5).
*	To make calculations more precise (because we need to use the value
*	of x^2 rounded to 16-bit fractional number in our calculations),
*	the given argument value x (that is in range [-0.5; 0.5)) is shifted
*	by 1 bit to the left (multiplied by 2), then the calculated x^2 value
*	is in range [-1; 1) instead of [-0.25; 0.25).
*	Then the polynomial coefficients 'c' needs to be scaled
*	(shifted to the right):
*
*		b1 = c1 / 2^1 =  pi / 2
*		b3 = c3 / 2^3 = -pi^3 / 3! / 2^3
*		b5 = c5 / 2^5 =  pi^5 / 5! / 2^5
*		b7 = c7 / 2^7 = -pi^7 / 7! / 2^7
*		b9 = c9 / 2^9 =  pi^9 / 9! / 2^9
*
*	To avoid the saturation error during the polynomial calculation
*	the coefficients 'b' are divided by 2. After the polynomial calculation
*	the result is multiplied by 2 (shifted by 1 bit to the left)
*	to take the right result ( in range [-1; 1) ).
*
*		a1 = b1 / 2 =  pi / 2^2         =  0.785398163
*		a3 = b3 / 2 = -pi^3 / 3! / 2^4  = -0.322982049
*		a5 = b5 / 2 =  pi^5 / 5! / 2^6  =  0.039846313
*		a7 = b7 / 2 = -pi^7 / 7! / 2^8  = -0.002340877
*		a9 = b9 / 2 =  pi^9 / 9! / 2^10 =  0.000080220
*
*		In 32-bit signed fractional representation:
*		a1 = 0x6487ED51
*		a3 = 0xD6A88634
*		a5 = 0x0519AF1A
*		a7 = 0xFFB34B4D
*		a9 = 0x0002A0F0
*
*	sin(pi * x) = (x<<1)*(a1 + (x<<1)^2(a3 + (x<<1)^2(a5 + (x<<1)^2(a7 \
*				  + (x<<1)^2*a9)))) << 1
*
*	For better accuracy the (x<<1)^2*(a#...) calculations are performed
*	as the following:
*		(x<<1)^2 as 16-bit fractional value
*			  a# as 32-bit fractional value
*		[ (x<<1)^2 ] * [ (a#... ] = [16-bit]*[32-bit] = 32-bit number
*
* 			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 GFLIB_CosTlrFAsmi(register Frac16 f16In, const GFLIB_SIN_TAYLOR_COEF_T *pudtSinData)
{

	asm(.optimize_iasm on);
	// change argument of cosine function to sin(x+0.5pi); uses long because of saturation independence
	asm(move.w f16In,Y0);
	asm(clr.w Y1);
	asm(add.l #16384,Y);
	asm(move.w Y0,f16In);
	return GFLIB_SinTlrFAsm(f16In, pudtSinData);
	asm(.optimize_iasm off);
}

/***************************************************************************//*!
*
* @brief  Calculates the sine of the given argument using Taylor 9th order
*         polynomial  with the result precision of 12 bits..
*
* @param  ptr		Frac16 *pudtSinData
*					- Pointer to the Taylor polynomial coefficients
* @param  in    	Frac16 f16In
*                   - Argument in range [-1; 1) in Frac16 corresponding
*					  to [-pi; pi)
*
* @return This function returns
*     				- Frac16 value [-1; 1)
*		
* @remarks 	
*	This function calculates sin(pi * x) using Taylor 9th order polynomial:
*	x=[-pi; pi]:
*		sine(x) = x - x^3/3! + x^5/5! - x^7/7! + x^9/9!
*		sine(x) = x*(1 - x^2/6*(1 - x^2/20*(1 - x^2/42*(1 - x^2/72))))
*		sine(x) = x(d1 + x^2(d3 + x^2(d5 + x^2(d7 + x^2*d9))))
*
*		where:
*		d1 =  1
*		d3 = -1 / 3!
*		d5 =  1 / 5!
*		d7 = -1 / 7!
*		d9 =  1 / 9!
*
*	because the x value is in range [-1; 1) that corresponds to [-pi; pi],
*	the coefficients 'd' needs to be scaled:
*
*		x=[-1; 1):
*		sin(pi * x) = x(c1 + x^2(c3 + x^2(c5 + x^2(c7 + x^2*c9))))
*
*		c1 = d1 * pi^1 =  pi
*		c3 = d3 * pi^3 = -pi^3 / 3!
*		c5 = d5 * pi^5 =  pi^5 / 5!
*		c7 = d7 * pi^7 = -pi^7 / 7!
*		c9 = d9 * pi^9 =  pi^9 / 9!
*
*	The 9th order polynomial approximation of the sine function
*	has very good accuracy in range [-pi/2; pi/2] of argument, but in wider
*	range the calculation error is quickly growing up. Therefore the sine
*	function is	approximated only in range [-pi/2; pi/2] and it is used the
*	symmetry of the sine function [ sin(alpha) = sin(pi - alpha) ].
*	Therefore the alpha argument is in range [-pi/2; pi/2) that corresponds
*	to x [-0.5; 0.5).
*	To make calculations more precise (because we need to use the value
*	of x^2 rounded to 16-bit fractional number in our calculations),
*	the given argument value x (that is in range [-0.5; 0.5)) is shifted
*	by 1 bit to the left (multiplied by 2), then the calculated x^2 value
*	is in range [-1; 1) instead of [-0.25; 0.25).
*	Then the polynomial coefficients 'c' needs to be scaled
*	(shifted to the right):
*
*		b1 = c1 / 2^1 =  pi / 2
*		b3 = c3 / 2^3 = -pi^3 / 3! / 2^3
*		b5 = c5 / 2^5 =  pi^5 / 5! / 2^5
*		b7 = c7 / 2^7 = -pi^7 / 7! / 2^7
*		b9 = c9 / 2^9 =  pi^9 / 9! / 2^9
*
*	To avoid the saturation error during the polynomial calculation
*	the coefficients 'b' are divided by 2. After the polynomial calculation
*	the result is multiplied by 2 (shifted by 1 bit to the left)
*	to take the right result ( in range [-1; 1) ).
*
*		a1 = b1 / 2 =  pi / 2^2         =  0.785398163
*		a3 = b3 / 2 = -pi^3 / 3! / 2^4  = -0.322982049
*		a5 = b5 / 2 =  pi^5 / 5! / 2^6  =  0.039846313
*		a7 = b7 / 2 = -pi^7 / 7! / 2^8  = -0.002340877
*		a9 = b9 / 2 =  pi^9 / 9! / 2^10 =  0.000080220
*
*		In 32-bit signed fractional representation:
*		a1 = 0x6488
*		a3 = 0xD6A9
*		a5 = 0x051A
*		a7 = 0xFFB3
*		a9 = 0x0003
*
*	sin(pi * x) = (x<<1)*(a1 + (x<<1)^2(a3 + (x<<1)^2(a5 + (x<<1)^2(a7 \
*				  + (x<<1)^2*a9)))) << 1
*
*	For better accuracy the (x<<1)^2*(a#...) calculations are performed
*	as the following:
*		(x<<1)^2 as 16-bit fractional value
*			  a# as 16-bit fractional value
*		[ (x<<1)^2 ] * [ (a#... ] = [16-bit]*[16-bit] = 32-bit number
*
* 			SATURATION INDEPENDENT!
*
****************************************************************************/
extern asm Frac16 GFLIB_Sin12TlrFAsm(Frac16 f16In, const GFLIB_SIN12_TAYLOR_COEF_T *pudtSinData);

/***************************************************************************//*!
*
* @brief  Calculates the cosine of the given argument using Taylor 9th order
*         polynomial with the result precision of 12 bits. Inline function
*		  which increments the argument by 0.5 and calls the Sin calculation.
*
* @param  ptr		Frac16 *pudtSinData
*					- Pointer to the Taylor polynomial coefficients
*
* @param  in    	Frac16 f16In
*                   - Argument in range [-1; 1) in Frac16 corresponding
*					  to [-pi; pi)
*
* @return This function returns
*     				- Frac16 value [-1; 1)
*		
* @remarks 	
*	This function adds 90 degrees to the argument and then uses the
*	GFLIB_Sin12TlrFAsm function.
*
* 			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline Frac16 GFLIB_Cos12TlrFAsmi(register Frac16 f16In, const GFLIB_SIN12_TAYLOR_COEF_T *pudtSinData)
{

	asm(.optimize_iasm on);
	// change argument of cosine function to sin(x+0.5pi); uses long because of saturation independence
	asm(move.w f16In,Y0);
	asm(clr.w Y1);
	asm(add.l #16384,Y);
	asm(move.w Y0,f16In);
	return GFLIB_Sin12TlrFAsm(f16In, pudtSinData);
	asm(.optimize_iasm off);
}

/***************************************************************************//*!
*
* @brief  Calculates the sine of the given argument using Taylor 9th order
*         polynomial.
*
* @param  ptr		Frac16 *pudtSinData
*					- Pointer to the Taylor polynomial coefficients
* @param  in    	Frac16 f16In
*                   - Argument in range [-1; 1) in Frac16 corresponding
*					  to [-pi; pi)
*
* @return This function returns
*     				- Frac16 value [-1; 1)
*		
* @remarks 	
*	This function calculates sin(pi * x) using Taylor 9th order polynomial:
*	x=[-pi; pi]:
*		sine(x) = x - x^3/3! + x^5/5! - x^7/7! + x^9/9!
*		sine(x) = x*(1 - x^2/6*(1 - x^2/20*(1 - x^2/42*(1 - x^2/72))))
*		sine(x) = x(d1 + x^2(d3 + x^2(d5 + x^2(d7 + x^2*d9))))
*
*		where:
*		d1 =  1
*		d3 = -1 / 3!
*		d5 =  1 / 5!
*		d7 = -1 / 7!
*		d9 =  1 / 9!
*
*	because the x value is in range [-1; 1) that corresponds to [-pi; pi],
*	the coefficients 'd' needs to be scaled:
*
*		x=[-1; 1):
*		sin(pi * x) = x(c1 + x^2(c3 + x^2(c5 + x^2(c7 + x^2*c9))))
*
*		c1 = d1 * pi^1 =  pi
*		c3 = d3 * pi^3 = -pi^3 / 3!
*		c5 = d5 * pi^5 =  pi^5 / 5!
*		c7 = d7 * pi^7 = -pi^7 / 7!
*		c9 = d9 * pi^9 =  pi^9 / 9!
*
*	The 9th order polynomial approximation of the sine function
*	has very good accuracy in range [-pi/2; pi/2] of argument, but in wider
*	range the calculation error is quickly growing up. Therefore the sine
*	function is	approximated only in range [-pi/2; pi/2] and it is used the
*	symmetry of the sine function [ sin(alpha) = sin(pi - alpha) ].
*	Therefore the alpha argument is in range [-pi/2; pi/2) that corresponds
*	to x [-0.5; 0.5).
*	To make calculations more precise (because we need to use the value
*	of x^2 rounded to 16-bit fractional number in our calculations),
*	the given argument value x (that is in range [-0.5; 0.5)) is shifted
*	by 1 bit to the left (multiplied by 2), then the calculated x^2 value
*	is in range [-1; 1) instead of [-0.25; 0.25).
*	Then the polynomial coefficients 'c' needs to be scaled
*	(shifted to the right):
*
*		b1 = c1 / 2^1 =  pi / 2
*		b3 = c3 / 2^3 = -pi^3 / 3! / 2^3
*		b5 = c5 / 2^5 =  pi^5 / 5! / 2^5
*		b7 = c7 / 2^7 = -pi^7 / 7! / 2^7
*		b9 = c9 / 2^9 =  pi^9 / 9! / 2^9
*
*	To avoid the saturation error during the polynomial calculation
*	the coefficients 'b' are divided by 2. After the polynomial calculation
*	the result is multiplied by 2 (shifted by 1 bit to the left)
*	to take the right result ( in range [-1; 1) ).
*
*		a1 = b1 / 2 =  pi / 2^2         =  0.785398163
*		a3 = b3 / 2 = -pi^3 / 3! / 2^4  = -0.322982049
*		a5 = b5 / 2 =  pi^5 / 5! / 2^6  =  0.039846313
*		a7 = b7 / 2 = -pi^7 / 7! / 2^8  = -0.002340877
*		a9 = b9 / 2 =  pi^9 / 9! / 2^10 =  0.000080220
*
*		In 32-bit signed fractional representation:
*		a1 = 0x6487ED51
*		a3 = 0xD6A88634
*		a5 = 0x0519AF1A
*		a7 = 0xFFB34B4D
*		a9 = 0x0002A0F0
*
*	sin(pi * x) = (x<<1)*(a1 + (x<<1)^2(a3 + (x<<1)^2(a5 + (x<<1)^2(a7 \
*				  + (x<<1)^2*a9)))) << 1
*
*	For better accuracy the (x<<1)^2*(a#...) calculations are performed
*	as the following:
*		(x<<1)^2 as 16-bit fractional value
*			  a# as 32-bit fractional value
*		[ (x<<1)^2 ] * [ (a#... ] = [16-bit]*[32-bit] = 32-bit number
*
* 			SATURATION INDEPENDENT!
*		    The V3 core instructions used! 	
*
****************************************************************************/
extern asm Frac16 GFLIB_V3SinTlrFAsm(Frac16 f16In, const GFLIB_SIN_TAYLOR_COEF_T *pudtSinData);

/***************************************************************************//*!
*
* @brief  Calculates the cosine of the given argument using Taylor 9th order
*         polynomial. Inline function which increments the argument by 0.5 and
*		  calls the Sin calculation.
*
* @param  ptr		Frac16 *pudtSinData
*					- Pointer to the Taylor polynomial coefficients
* @param  in    	Frac16 f16In
*                   - Argument in range [-1; 1) in Frac16 corresponding
*					  to [-pi; pi)
*
* @return This function returns
*     				- Frac16 value [-1; 1)
*		
* @remarks 	
*	This function calculates cos(pi * x) using the Taylor 9th order
*	polynomial approximation of sine function.
*	At the beginning the input argument of cosine function
*	is transferred to argument of sine function:
*
*	sine_argument = pi/2 - ABS(cos_argument)
*
*	then the cosine function is calculated using the sine function.
*	The constant pi/2 can not be directly added to the cos_argument,
*	because the saturation is enabled, therefore the saturation
*	can occur during the addition.
*
*	The function calculates sin(pi * x) using the Taylor 9th order	polynomial:
*	x=[-pi; pi]:
*		sine(x) = x - x^3/3! + x^5/5! - x^7/7! + x^9/9!
*		sine(x) = x*(1 - x^2/6*(1 - x^2/20*(1 - x^2/42*(1 - x^2/72))))
*		sine(x) = x(d1 + x^2(d3 + x^2(d5 + x^2(d7 + x^2*d9))))
*
*		where:
*		d1 =  1
*		d3 = -1 / 3!
*		d5 =  1 / 5!
*		d7 = -1 / 7!
*		d9 =  1 / 9!
*
*	because the x value is in range [-1; 1) that corresponds to [-pi; pi],
*	the coefficients 'd' needs to be scaled:
*
*		x=[-1; 1):
*		sin(pi * x) = x(c1 + x^2(c3 + x^2(c5 + x^2(c7 + x^2*c9))))
*
*		c1 = d1 * pi^1 =  pi
*		c3 = d3 * pi^3 = -pi^3 / 3!
*		c5 = d5 * pi^5 =  pi^5 / 5!
*		c7 = d7 * pi^7 = -pi^7 / 7!
*		c9 = d9 * pi^9 =  pi^9 / 9!
*
*	The 9th order polynomial approximation of the sine function
*	has very good accuracy in range [-pi/2; pi/2] of argument, but in wider
*	range the calculation error is quickly growing up. Therefore the sine
*	function is	approximated only in range [-pi/2; pi/2] and it is used the
*	symmetry of the sine function [ sin(alpha) = sin(pi - alpha) ].
*	Therefore the alpha argument is in range [-pi/2; pi/2) that corresponds
*	to x [-0.5; 0.5).
*	To make calculations more precise (because we need to use the value
*	of x^2 rounded to 16-bit fractional number in our calculations),
*	the given argument value x (that is in range [-0.5; 0.5)) is shifted
*	by 1 bit to the left (multiplied by 2), then the calculated x^2 value
*	is in range [-1; 1) instead of [-0.25; 0.25).
*	Then the polynomial coefficients 'c' needs to be scaled
*	(shifted to the right):
*
*		b1 = c1 / 2^1 =  pi / 2
*		b3 = c3 / 2^3 = -pi^3 / 3! / 2^3
*		b5 = c5 / 2^5 =  pi^5 / 5! / 2^5
*		b7 = c7 / 2^7 = -pi^7 / 7! / 2^7
*		b9 = c9 / 2^9 =  pi^9 / 9! / 2^9
*
*	To avoid the saturation error during the polynomial calculation
*	the coefficients 'b' are divided by 2. After the polynomial calculation
*	the result is multiplied by 2 (shifted by 1 bit to the left)
*	to take the right result ( in range [-1; 1) ).
*
*		a1 = b1 / 2 =  pi / 2^2         =  0.785398163
*		a3 = b3 / 2 = -pi^3 / 3! / 2^4  = -0.322982049
*		a5 = b5 / 2 =  pi^5 / 5! / 2^6  =  0.039846313
*		a7 = b7 / 2 = -pi^7 / 7! / 2^8  = -0.002340877
*		a9 = b9 / 2 =  pi^9 / 9! / 2^10 =  0.000080220
*
*		In 32-bit signed fractional representation:
*		a1 = 0x6487ED51
*		a3 = 0xD6A88634
*		a5 = 0x0519AF1A
*		a7 = 0xFFB34B4D
*		a9 = 0x0002A0F0
*
*	sin(pi * x) = (x<<1)*(a1 + (x<<1)^2(a3 + (x<<1)^2(a5 + (x<<1)^2(a7 \
*				  + (x<<1)^2*a9)))) << 1
*
*	For better accuracy the (x<<1)^2*(a#...) calculations are performed
*	as the following:
*		(x<<1)^2 as 16-bit fractional value
*			  a# as 32-bit fractional value
*		[ (x<<1)^2 ] * [ (a#... ] = [16-bit]*[32-bit] = 32-bit number
*
* 			SATURATION INDEPENDENT!
*		    The V3 core instructions used! 	
*
****************************************************************************/
extern inline Frac16 GFLIB_V3CosTlrFAsmi(register Frac16 f16In, const GFLIB_SIN_TAYLOR_COEF_T *pudtSinData)
{

	asm(.optimize_iasm on);
	// change argument of cosine function to sin(x+0.5pi); uses long because of saturation independence
	asm(move.w f16In,Y0);
	asm(clr.w Y1);
	asm(add.l #16384,Y);
	asm(move.w Y0,f16In);
	return GFLIB_V3SinTlrFAsm(f16In, pudtSinData);
	asm(.optimize_iasm off);
}

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _GFLIB_SINCOSTLRASM_H_ */

