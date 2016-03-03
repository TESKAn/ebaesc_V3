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

// NTC temperature
Int16 i16TemperatureTable[32] = {-38,-26,-19,-13,-8,-3, 1, 4, 8, 11, 14,18,21,24,27,
		30,33,37,40,44,47,52,56,61,66,73,80,89,102,120,153,305};

Int16 i16CurrentCalArrayIndex = 4096;
Frac16 f16CurrentCalArrayData = FRAC16(0.0);

// RS485 temporary vars
UInt8 ui8RS485RXVal = 0;

// Variable that holds all system data
SYSTEMVARIABLES SYSTEM;

// Variable for MOSFET driver
DRV8301DATA DRV8301;

Int16 i16RecorderTrigger = 0;

UInt8 ui8SerialBuffer[16];
UInt8 RS485Address = 1;
UInt8 ui8RS485RcvdByte = 0;

// Transmit ring buffer
RING_BUFFER SCI0RXBuff;
UInt8 SCI0RXBuffer[256];

RS485MOTOR RS485DataStruct;

Int32 i32Var = 0;

PARAMCONVERSION pConv;

