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
Int16 i16TemperatureTable[32] = {-37,-26,-18,-12,-6,0,4,7,12,16,19,23,26,29,33,36,39,42,46,50,54,59,64,67,72,80,86,97,109,125,150,254};


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

UInt16 ui16CANTestCounter = 0;

