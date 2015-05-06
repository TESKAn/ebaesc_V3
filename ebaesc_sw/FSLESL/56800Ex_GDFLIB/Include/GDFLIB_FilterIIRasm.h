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
* @file      GDFLIB_FilterIIRasm.h
*
* @author    r59400
* 
* @version   1.0.3.0
* 
* @date      Aug-15-2013
* 
* @brief     Digital IIR Filter, 1st & 2nd order assembler implementation
*
*******************************************************************************
*
* Digital IIR Filter, 1st, 2nd, 3rd and 4th order assembler implementation.
*
******************************************************************************/
#ifndef _GDFLIB_IIRFILTERASM_H_
#define _GDFLIB_IIRFILTERASM_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "56800E_types.h"

/******************************************************************************
* Macros 
******************************************************************************/
#define GDFLIB_FilterIIR1InitC(pudtFilter) GDFLIB_FilterIIR1InitFC(pudtFilter)
#define GDFLIB_FilterIIR2InitC(pudtFilter) GDFLIB_FilterIIR2InitFC(pudtFilter)
#define GDFLIB_FilterIIR3InitC(pudtFilter) GDFLIB_FilterIIR3InitFC(pudtFilter)
#define GDFLIB_FilterIIR4InitC(pudtFilter) GDFLIB_FilterIIR4InitFC(pudtFilter)

#define GDFLIB_FilterIIR1Asm(f16In, pudtFilter) GDFLIB_FilterIIR1FAsm(f16In, pudtFilter)
#define GDFLIB_FilterIIR2Asm(f16In, pudtFilter) GDFLIB_FilterIIR2FAsm(f16In, pudtFilter) 
#define GDFLIB_FilterIIR3Asm(f16In, pudtFilter) GDFLIB_FilterIIR3FAsm(f16In, pudtFilter) 
#define GDFLIB_FilterIIR4Asm(f16In, pudtFilter) GDFLIB_FilterIIR4FAsm(f16In, pudtFilter) 

/* The V3 core instructions */
#define GDFLIB_V3FilterIIR1Asm(f16In, pudtFilter) GDFLIB_V3FilterIIR1FAsm(f16In, pudtFilter)
#define GDFLIB_V3FilterIIR2Asm(f16In, pudtFilter) GDFLIB_V3FilterIIR2FAsm(f16In, pudtFilter)
#define GDFLIB_V3FilterIIR3Asm(f16In, pudtFilter) GDFLIB_V3FilterIIR3FAsm(f16In, pudtFilter) 
#define GDFLIB_V3FilterIIR4Asm(f16In, pudtFilter) GDFLIB_V3FilterIIR4FAsm(f16In, pudtFilter) 

#define GDFLIB_FILTER_IIR1_DEFAULT {\
							{0},\
							{0},\
							{0,0,0}}

#define GDFLIB_FILTER_IIR2_DEFAULT {\
							{0,0},\
							{0,0},\
							{0,0,0,0,0}}

#define GDFLIB_FILTER_IIR3_DEFAULT {\
							{0,0,0},\
							{0,0,0},\
							{0,0,0,0,0,0,0}}

#define GDFLIB_FILTER_IIR4_DEFAULT {\
							{0,0,0,0},\
							{0,0,0,0},\
							{0,0,0,0,0,0,0,0,0}}


/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
	Frac16			f16B1;
	Frac16			f16B2;
	Frac16			f16A2;
} GDFLIB_FILTER_IIR_COEFF1_T;

typedef struct
{
	Frac16			f16B1;
	Frac16			f16B2;
	Frac16			f16A2;
	Frac16			f16B3;
	Frac16			f16A3;
} GDFLIB_FILTER_IIR_COEFF2_T;

typedef struct
{
	Frac16			f16B1;
	Frac16			f16B2;
	Frac16			f16A2;
	Frac16			f16B3;
	Frac16			f16A3;
	Frac16			f16B4;
	Frac16			f16A4;
} GDFLIB_FILTER_IIR_COEFF3_T;

typedef struct
{
	Frac16			f16B1;
	Frac16			f16B2;
	Frac16			f16A2;
	Frac16			f16B3;
	Frac16			f16A3;
	Frac16			f16B4;
	Frac16			f16A4;
	Frac16			f16B5;
	Frac16			f16A5;
} GDFLIB_FILTER_IIR_COEFF4_T;

typedef struct 
{
    Frac32 						f32FiltBufferY[1];
    Frac16 						f16FiltBufferX[1];
	GDFLIB_FILTER_IIR_COEFF1_T	udtFiltCoeff;
} GDFLIB_FILTER_IIR1_T;

typedef struct 
{
    Frac32 						f32FiltBufferY[2];    
    Frac16 						f16FiltBufferX[2];
    GDFLIB_FILTER_IIR_COEFF2_T	udtFiltCoeff;
} GDFLIB_FILTER_IIR2_T;

typedef struct 
{
    Frac32 						f32FiltBufferY[3];
    Frac16 						f16FiltBufferX[3];
    GDFLIB_FILTER_IIR_COEFF3_T	udtFiltCoeff;
} GDFLIB_FILTER_IIR3_T;

typedef struct 
{
    Frac32 						f32FiltBufferY[4];
    Frac16 						f16FiltBufferX[4];
    GDFLIB_FILTER_IIR_COEFF4_T	udtFiltCoeff;
} GDFLIB_FILTER_IIR4_T;

/******************************************************************************
* Global functions
******************************************************************************/


/***************************************************************************//*!
*
* @brief  The function clears buffer of 1st order IIR filter pointed to 
*		  by argument
*
* @param  ptr   		GDFLIB_FILTER_IIR1_T *pudtFilter
*						  	- Pointer to filter structure
*
* @param  in    		
*
* @return N/A
*		
* @remarks 
*
****************************************************************************/
extern void	GDFLIB_FilterIIR1InitFC(GDFLIB_FILTER_IIR1_T * const pudtFilter);


/***************************************************************************//*!
*
* @brief  The function clears buffer of 2nd order IIR filter pointed to 
*		  by argument
*
* @param  ptr   		GDFLIB_FILTER_IIR2_T *pudtFilter
*						  	- Pointer to filter structure
*
* @param  in    		
*
* @return N/A
*		
* @remarks 
*
****************************************************************************/
extern void	GDFLIB_FilterIIR2InitFC(GDFLIB_FILTER_IIR2_T * const pudtFilter);

/***************************************************************************//*!
*
* @brief  The function clears buffer of 3rd order IIR filter pointed to 
*		  by argument
*
* @param  ptr   		GDFLIB_FILTER_IIR3_T *pudtFilter
*						  	- Pointer to filter structure
*
* @param  in    		
*
* @return N/A
*		
* @remarks 
*
****************************************************************************/
extern void GDFLIB_FilterIIR3InitFC(GDFLIB_FILTER_IIR3_T * const pudtFilter);

/***************************************************************************//*!
*
* @brief  The function clears buffer of 4th order IIR filter pointed to 
*		  by argument
*
* @param  ptr   		GDFLIB_FILTER_IIR4_T *pudtFilter
*						  	- Pointer to filter structure
*
* @param  in    		
*
* @return N/A
*		
* @remarks 
*
****************************************************************************/
extern void GDFLIB_FilterIIR4InitFC(GDFLIB_FILTER_IIR4_T * const pudtFilter);

/***************************************************************************//*!
*
* @brief  The function calculates 1st order IIR filter pointed to by argument
*
* @param  ptr   		GDFLIB_FILTER_IIR1_T *pudtFilter
*						  	- Pointer to filter structure
*
* @param  in    	 	f16In - input signal	
*
* @return This function returns
*     - Frac16 value [-1;1]
*		
* @remarks 
*
*
*	Filter Equation:
*	y(n) = b1*x(n) + b2*x(n-1) + (-a2)*y(n-1)
*	
*	--------- function argument passing ----------	
*	 Y0	filter input
*	 R2	pointer to filter structure
*		    Frac32 f32FiltBufferY[1];  
*		    Frac16 f16FiltBufferX[1];
*		    Frac16 udtFiltCoeff;
*	-----------------------------------------------
*
*	
*	Pointers Convention Table:
*	
*	R2 					pointer to parameters y
*	R0	= R2 + #2		pointer to parameters x
*	
*	note: y is 32-bit variable e.g. 2 words
*		
*	|offset	|	0	|
*	|-------|-------|
*	|R2	->	| y(n-1)|
*
*	|offset	|	2	|
*	|-------|-------|
*	|R0	->	| x(n-1)|
*
*	|offset	|	3	|	4	|	5	|
*	|-------|-------|-------|-------|
*	|R0	->	|	b1	|	b2	|	a2	|
*	
*	
*	SATURATION MUST BE TURNED OFF!
*
****************************************************************************/
extern asm Frac16 GDFLIB_FilterIIR1FAsm(Frac16 f16In, GDFLIB_FILTER_IIR1_T * const pudtFilter);

/***************************************************************************//*!
*
* @brief  Function calculates 2nd order IIR filter pointed to by argument
*
* @param  ptr   		GDFLIB_FILTER_IIR2_T *pudtFilter
*						  	- Pointer to filter structure
*
* @param  in    	 	f16In - input signal	
*
* @return This function returns the filtered signal
*     - Frac16 value [-1;1]
*		
* @remarks 
*
*
*	Filter equation:
*	y(n) = b1*x(n) + b2*x(n-1) + b3*x(n-2) + (-a2)*y(n-1) + (-a3)*y(n-2)
*	
*	--------- function argument passing ----------	
*	 Y0	filter input
*	 R2	pointer to filter structure
*		    Frac32 f32FiltBufferY[2];  
*		    Frac16 f16FiltBufferX[2];
*		    Frac16 udtFiltCoeff;
*	-----------------------------------------------
*
*
*	Pointers Convention Table:
*	
*	R3	= R2 + 6		pointer to filter coefficients
*	R0	= R2 + 4		pointer to parameters x
*	R1	= R2 + 0		pointer to parameters y
*	
*	note: y is 32-bit variable e.g. 2 words
*		
*	|offset	|	0	|	2	|
*	|-------|-------|-------|
*	|R1	->	| y(n-1)| y(n-2)|
*
*	|offset	|	4	|	5	|
*	|-------|-------|-------|
*	|R0	->	| x(n-1)| x(n-2)|
*	
*	|offset	|	6	|	7	|	8	| 	9	|	a	|
*	|-------|-------|-------|-------|-------|-------|
*	|R3	->	|	b1	|	b2	|	a2	| 	b3	|	a3	|
*
*	
*	THE SATURATION MUST BE TURNED OFF!
*
****************************************************************************/
extern asm Frac16 GDFLIB_FilterIIR2FAsm(Frac16 f16In, GDFLIB_FILTER_IIR2_T * const pudtFilter);

/***************************************************************************//*!
*
* @brief  Function calculates 3nd order IIR filter pointed to by argument
*
* @param  ptr   		GDFLIB_FILTER_IIR3_T *pudtFilter
*						  	- Pointer to filter structure
*
* @param  in    	 	f16In - input signal	
*
* @return This function returns the filtered signal
*     - Frac16 value [-1;1]
*		
* @remarks 
*
*	Filter equation:
*	y(n) = b1*x(n) + b2*x(n-1) + b3*x(n-2) + b4*x(n-3) + (-a2)*y(n-1) + (-a3)*y(n-2) + (-a4)*y(n-3)
*	
*	--------- function argument passing ----------	
*	 Y0	filter input
*	 R2	pointer to filter structure
*		    Frac32 f32FiltBufferY[3];  
*		    Frac16 f16FiltBufferX[3];
*		    Frac16 udtFiltCoeff;
*	-----------------------------------------------
*
*
*	Pointers Convention Table:
*	
*	R3	= R2 + 9		pointer to filter coefficients
*	R0	= R2 + 6		pointer to parameters x
*	R1	= R2 + 0		pointer to parameters y
*	
*	note: y is 32-bit variable e.g. 2 words
*		
*	|offset	|	0	|	2	|	4	|
*	|-------|-------|-------|-------|
*	|R1	->	| y(n-1)| y(n-2)| y(n-3)|
*	
*	|offset	|	6	|	7	|   8   |
*	|-------|-------|-------|-------|
*	|R0	->	| x(n-1)| x(n-2)| x(n-3)|
*	
*	|offset	|	9	|	a	|	b	| 	c	|	d	| 	e	|	f	|
*	|-------|-------|-------|-------|-------|-------|-------|-------|
*	|R3	->	|	b1	|	b2	|	a2	| 	b3	|	a3	|   b4  |   a4  |
*
*	THE SATURATION MUST BE TURNED ON!
*
****************************************************************************/
extern asm Frac16 GDFLIB_FilterIIR3FAsm(Frac16 f16In, GDFLIB_FILTER_IIR3_T * const pudtFilter);

/***************************************************************************//*!
*
* @brief  Function calculates 4th order IIR filter pointed to by argument
*
* @param  ptr   		GDFLIB_FILTER_IIR4_T *pudtFilter
*						  	- Pointer to filter structure
*
* @param  in    	 	f16In - input signal	
*
* @return This function returns the filtered signal
*     - Frac16 value [-1;1]
*		
* @remarks 
*
*	Filter equation:
*	y(n) = b1*x(n) + b2*x(n-1) + b3*x(n-2) + b4*x(n-3) + b5*x(n-4) + (-a2)*y(n-1) + (-a3)*y(n-2) + (-a4)*y(n-3) + (-a5)*y(n-4)
*	
*	--------- function argument passing ----------	
*	 Y0	filter input
*	 R2	pointer to filter structure
*		    Frac32 f32FiltBufferY[4];  
*		    Frac16 f16FiltBufferX[4];
*		    Frac16 udtFiltCoeff;
*	-----------------------------------------------
*
*
*	Pointers Convention Table:
*	
*	R3	= R2 + 0xC		pointer to filter coefficients
*	R0	= R2 + 8		pointer to parameters x
*	R1	= R2 + 0		pointer to parameters y
*	
*	note: y is 32-bit variable e.g. 2 words
*		
*	|offset	|	0	|	2	|	4	|	6	|
*	|-------|-------|-------|-------|-------|
*	|R1	->	| y(n-1)| y(n-2)| y(n-3)| y(n-4)|
*
*	|offset	|	8	|	9	|   a   |   b   |
*	|-------|-------|-------|-------|-------|
*	|R0	->	| x(n-1)| x(n-2)| x(n-3)| x(n-4)|
*
*	|offset	|	c	|	d	|	e	| 	f	|	10	| 	11	|	12	| 	13	|	14	|
*	|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
*	|R3	->	|	b1	|	b2	|	a2	| 	b3	|	a3	|   b4  |   a4  |   b5  |   a5  |
*
*	
*	
*	THE SATURATION MUST BE TURNED OFF!
*
****************************************************************************/
extern asm Frac16 GDFLIB_FilterIIR4FAsm(Frac16 f16In, GDFLIB_FILTER_IIR4_T * const pudtFilter);


/***************************************************************************//*!
*
* @brief  The function calculates 1st order IIR filter pointed to by argument
*
* @param  ptr   		GDFLIB_FILTER_IIR1_T *pudtFilter
*						  	- Pointer to filter structure
*
* @param  in    	 	f16In - input signal	
*
* @return This function returns
*     - Frac16 value [-1;1]
*		
* @remarks 
*
*
*	Filter Equation:
*	y(n) = b1*x(n) + b2*x(n-1) + (-a2)*y(n-1)
*	
*	--------- function argument passing ----------	
*	 Y0	filter input
*	 R2	pointer to filter structure
*		    Frac32 f32FiltBufferY[1];  
*		    Frac16 f16FiltBufferX[1];
*		    Frac16 udtFiltCoeff;
*	-----------------------------------------------
*
*	
*	Pointers Convention Table:
*	
*	R2 					pointer to parameters y
*	R0	= R2 + #2		pointer to parameters x
*	
*	note: y is 32-bit variable e.g. 2 words
*		
*	|offset	|	0	|
*	|-------|-------|
*	|R2	->	| y(n-1)|
*
*	|offset	|	2	|
*	|-------|-------|
*	|R0	->	| x(n-1)|
*
*	|offset	|	3	|	4	|	5	|
*	|-------|-------|-------|-------|
*	|R0	->	|	b1	|	b2	|	a2	|
*	
*	
*	SATURATION MUST BE TURNED OFF!
*	The V3 core instructions used! 
*
****************************************************************************/
extern asm Frac16 GDFLIB_V3FilterIIR1FAsm(Frac16 f16In, GDFLIB_FILTER_IIR1_T * const pudtFilter);

/***************************************************************************//*!
*
* @brief  Function calculates 2nd order IIR filter pointed to by argument
*
* @param  ptr   		GDFLIB_FILTER_IIR2_T *pudtFilter
*						  	- Pointer to filter structure
*
* @param  in    	 	f16In - input signal	
*
* @return This function returns the filtered signal
*     - Frac16 value [-1;1]
*		
* @remarks 
*
*
*	Filter equation:
*	y(n) = b1*x(n) + b2*x(n-1) + b3*x(n-2) + (-a2)*y(n-1) + (-a3)*y(n-2)
*	
*	--------- function argument passing ----------	
*	 Y0	filter input
*	 R2	pointer to filter structure
*		    Frac32 f32FiltBufferY[2];  
*		    Frac16 f16FiltBufferX[2];
*		    Frac16 udtFiltCoeff;
*	-----------------------------------------------
*
*
*	Pointers Convention Table:
*	
*	R3	= R2 + 6		pointer to filter coefficients
*	R0	= R2 + 4		pointer to parameters x
*	R1	= R2 + 0		pointer to parameters y
*	
*	note: y is 32-bit variable e.g. 2 words
*		
*	|offset	|	0	|	2	|
*	|-------|-------|-------|
*	|R1	->	| y(n-1)| y(n-2)|
*
*	|offset	|	4	|	5	|
*	|-------|-------|-------|
*	|R0	->	| x(n-1)| x(n-2)|
*	
*	|offset	|	6	|	7	|	8	| 	9	|	a	|
*	|-------|-------|-------|-------|-------|-------|
*	|R3	->	|	b1	|	b2	|	a2	| 	b3	|	a3	|
*
*	
*	THE SATURATION MUST BE TURNED OFF!
*	The V3 core instructions used! 
*
****************************************************************************/
extern asm Frac16 GDFLIB_V3FilterIIR2FAsm(Frac16 f16In, GDFLIB_FILTER_IIR2_T * const pudtFilter);

/***************************************************************************//*!
*
* @brief  Function calculates 3nd order IIR filter pointed to by argument
*
* @param  ptr   		GDFLIB_FILTER_IIR3_T *pudtFilter
*						  	- Pointer to filter structure
*
* @param  in    	 	f16In - input signal	
*
* @return This function returns the filtered signal
*     - Frac16 value [-1;1]
*		
* @remarks 
*
* Benchmarking:	routine execution duration 49 cycles (without jumping into
*				the function 'jsr')
*
*	Filter equation:
*	y(n) = b1*x(n) + b2*x(n-1) + b3*x(n-2) + b4*x(n-3) + (-a2)*y(n-1) + (-a3)*y(n-2) + (-a4)*y(n-3)
*	
*	--------- function argument passing ----------	
*	 Y0	filter input
*	 R2	pointer to filter structure
*		    Frac32 f32FiltBufferY[3];  
*		    Frac16 f16FiltBufferX[3];
*		    Frac16 udtFiltCoeff;
*	-----------------------------------------------
*
*
*	Pointers Convention Table:
*	
*	R3	= R2 + 9		pointer to filter coefficients
*	R0	= R2 + 6		pointer to parameters x
*	R1	= R2 + 0		pointer to parameters y
*	
*	note: y is 32-bit variable e.g. 2 words
*		
*	|offset	|	0	|	2	|	4	|
*	|-------|-------|-------|-------|
*	|R1	->	| y(n-1)| y(n-2)| y(n-3)|
*	
*	|offset	|	6	|	7	|   8   |
*	|-------|-------|-------|-------|
*	|R0	->	| x(n-1)| x(n-2)| x(n-3)|
*	
*	|offset	|	9	|	a	|	b	| 	c	|	d	| 	e	|	f	|
*	|-------|-------|-------|-------|-------|-------|-------|-------|
*	|R3	->	|	b1	|	b2	|	a2	| 	b3	|	a3	|   b4  |   a4  |
*
*	THE SATURATION MUST BE TURNED ON!
*	The V3 core instructions used! 
*
****************************************************************************/
extern asm Frac16 GDFLIB_V3FilterIIR3FAsm(Frac16 f16In, GDFLIB_FILTER_IIR3_T * const pudtFilter);

/***************************************************************************//*!
*
* @brief  Function calculates 4th order IIR filter pointed to by argument
*
* @param  ptr   		GDFLIB_FILTER_IIR4_T *pudtFilter
*						  	- Pointer to filter structure
*
* @param  in    	 	f16In - input signal	
*
* @return This function returns the filtered signal
*     - Frac16 value [-1;1]
*		
* @remarks 
*
*	Filter equation:
*	y(n) = b1*x(n) + b2*x(n-1) + b3*x(n-2) + b4*x(n-3) + b5*x(n-4) + (-a2)*y(n-1) + (-a3)*y(n-2) + (-a4)*y(n-3) + (-a5)*y(n-4)
*	
*	--------- function argument passing ----------	
*	 Y0	filter input
*	 R2	pointer to filter structure
*		    Frac32 f32FiltBufferY[4];  
*		    Frac16 f16FiltBufferX[4];
*		    Frac16 udtFiltCoeff;
*	-----------------------------------------------
*
*
*	Pointers Convention Table:
*	
*	R3	= R2 + 0xC		pointer to filter coefficients
*	R0	= R2 + 8		pointer to parameters x
*	R1	= R2 + 0		pointer to parameters y
*	
*	note: y is 32-bit variable e.g. 2 words
*		
*	|offset	|	0	|	2	|	4	|	6	|
*	|-------|-------|-------|-------|-------|
*	|R1	->	| y(n-1)| y(n-2)| y(n-3)| y(n-4)|
*
*	|offset	|	8	|	9	|   a   |   b   |
*	|-------|-------|-------|-------|-------|
*	|R0	->	| x(n-1)| x(n-2)| x(n-3)| x(n-4)|
*
*	|offset	|	c	|	d	|	e	| 	f	|	10	| 	11	|	12	| 	13	|	14	|
*	|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
*	|R3	->	|	b1	|	b2	|	a2	| 	b3	|	a3	|   b4  |   a4  |   b5  |   a5  |
*
*	
*	
*	THE SATURATION MUST BE TURNED OFF!
*	The V3 core instructions used! 
*
****************************************************************************/
extern asm Frac16 GDFLIB_V3FilterIIR4FAsm(Frac16 f16In, GDFLIB_FILTER_IIR4_T * const pudtFilter);

#endif /* _GDFLIB_IIRFILTERASM_H_ */
