/*
 * var.c
 *
 *  Created on: Mar 29, 2015
 *      Author: Jure
 */

#include "allincludes.h"

// EEPROM test
Int8 i8EEPROMOp = 0;
Int16 i16EEPROMReadErrors = 0;
UWord32 uw32EEPROMAddress = 0;
UWord32 uw32ErrorIndex = 0;
UWord16 uw16EEPROMData = 0;
Int16 i16EEPROMCRCOK = 0;
// End eeprom test

// CAN test
Int8 i8CANTest = 0;
Int16 i16CANBufOverflowCount = 0;

// Parameters test 
Int8 i8ParamTest = 0;


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

COMMDATA COMMDataStruct;

Int32 i32Var = 0;

PARAMCONVERSION pConv;

UInt16 ui16CANTestCounter = 0;

unsigned int uiCRCVar1 = 0;

