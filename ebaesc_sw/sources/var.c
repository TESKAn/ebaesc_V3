/*
 * var.c
 *
 *  Created on: Mar 29, 2015
 *      Author: Jure
 */

#include "allincludes.h"

// Some flags
FLAGBITS flag0;
FLAGBITS flag1;

Int16 i16CurrentCalArrayIndex = 4096;
Frac16 f16CurrentCalArrayData = FRAC16(0.0);

// Variable that holds all system data
SYSTEMVARIABLES SYSTEM;

// Variable for MOSFET driver
DRV8301DATA DRV8301;



PARAMCONVERSION pConv;

