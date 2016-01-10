/*
 * functions.h
 *
 *  Created on: Mar 29, 2015
 *      Author: Jure
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

Int16 LogError(UInt8 ui8Error);
Int16 OneMsEvent(void);
Int16 CalculateSIValues(void);
Int16 CalculateCalibrationData(void);
Int16 CalculateFloat(void);
Int16 CalculateShiftGain(float K);
void calculateFactors(void);
void calculateFloats(void);
void StopMotor(void);
void interruptDelay(unsigned int count);
void delay(unsigned int count);
UInt16 DRV8301_SPI_Read(UInt16 uiAddress);
UInt16 DRV8301_SPI_Write(UInt16 uiAddress, UInt16 uiData);
Int16 InitDRV8301(Int16 wReset, Int16 wCurrLimit, Int16 wOC_MODE);
Int16 RB_full(RING_BUFFER* rb);
Int16 RB_Init(RING_BUFFER* rb, UInt8 *buf, Int16 size);
Int16 RB_push(RING_BUFFER* rb, UInt8 data);
UInt8 RB_pop(RING_BUFFER* rb);
Int16 RB_flush(RING_BUFFER* rb);

#endif /* FUNCTIONS_H_ */
