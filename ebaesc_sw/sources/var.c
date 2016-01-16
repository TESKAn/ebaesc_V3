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
UInt8 SCI0RXBuffer[128];

RS485MOTOR RS485DataStruct;

PARAMCONVERSION pConv;

