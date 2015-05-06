/*
 * events.c
 *
 *  Created on: Mar 29, 2015
 *      Author: Jure
 */

#include "allincludes.h"

#pragma interrupt saveall
void ADC_1_EOS_ISR(void)
{
	Int16 i16Temp = 0;
	Frac16 f16Temp = FRAC16(0.0);
	
	// Clear EOSI flag
	ioctl(ADC_1, ADC_CLEAR_STATUS_EOSI, NULL);
	// Call freemaster recorder
	FMSTR_Recorder();
	
	// Read data
	// Phase currents
	SYSTEM.ADC.m3IphUVW.f16A = ioctl(ADC_1, ADC_READ_SAMPLE, 0);
	SYSTEM.ADC.m3IphUVW.f16C = ioctl(ADC_1, ADC_READ_SAMPLE, 8);
	// Calculate third
	SYSTEM.ADC.m3IphUVW.f16B = -SYSTEM.ADC.m3IphUVW.f16A - SYSTEM.ADC.m3IphUVW.f16C;
	
	// Multiply with gain
	SYSTEM.ADC.m3IphUVW.f16A = mult(DRV8301_GAIN_FACTOR, SYSTEM.ADC.m3IphUVW.f16A);
	SYSTEM.ADC.m3IphUVW.f16B = mult(DRV8301_GAIN_FACTOR, SYSTEM.ADC.m3IphUVW.f16B);
	SYSTEM.ADC.m3IphUVW.f16C = mult(DRV8301_GAIN_FACTOR, SYSTEM.ADC.m3IphUVW.f16C);
	
	// Measure DC link voltage
	SYSTEM.ADC.f16DCLinkVoltage = ioctl(ADC_1, ADC_READ_SAMPLE, 2);
	// Filter	
	SYSTEM.ADC.f16DCLinkVoltageFiltered = GDFLIB_FilterMA32(SYSTEM.ADC.f16DCLinkVoltage, &SYSTEM.ADC.FilterMA32DCLink);	
	
	// Measure temperature
	SYSTEM.ADC.f16Temperature = ioctl(ADC_1, ADC_READ_SAMPLE, 10);
	// Filter	
	SYSTEM.ADC.f16TemperatureFiltered = GDFLIB_FilterMA32(SYSTEM.ADC.f16Temperature, &SYSTEM.ADC.FilterMA32Temperature);	
	
	// Measure sensor value
	SYSTEM.ADC.f16SensorValueA = ioctl(ADC_1, ADC_READ_SAMPLE, 1);
	SYSTEM.ADC.f16SensorValueB = ioctl(ADC_1, ADC_READ_SAMPLE, 9);
	// Filter
	SYSTEM.ADC.f16SensorValueAFiltered = GDFLIB_FilterMA32(SYSTEM.ADC.f16SensorValueA, &SYSTEM.ADC.FilterMA32SensorA);
	SYSTEM.ADC.f16SensorValueBFiltered = GDFLIB_FilterMA32(SYSTEM.ADC.f16SensorValueB, &SYSTEM.ADC.FilterMA32SensorB);
	
	// Get measured angle
	// Get current position index
	SYSTEM.POSITION.i16SensorIndex = (Int16)(SYSTEM.ADC.f16SensorValueB >> 5);
	// Get filtered current position
	SYSTEM.POSITION.i16SensorIndexFiltered = (Int16)(SYSTEM.ADC.f16SensorValueBFiltered >> 5);
	// Add offset to index
	SYSTEM.POSITION.i16SensorIndex += SYSTEM.POSITION.i16SensorIndexOffset;
	// Add phase delay
	i16Temp = mult(SYSTEM.POSITION.i16SensorIndexPhaseDelay, SYSTEM.POSITION.f16FilteredSpeed);
	SYSTEM.POSITION.i16SensorIndex = i16Temp + SYSTEM.POSITION.i16SensorIndex;
	// Wrap
    /*
     * TODO: Check wrap for negative speeds 
     *
     */
	SYSTEM.POSITION.i16SensorIndex = SYSTEM.POSITION.i16SensorIndex & 1023;
	// Get measured angle from previous iteration
	f16Temp = SYSTEM.CALIBRATION.f16CalibrationArray[SYSTEM.POSITION.i16SensorIndex_m];
	// Calculate phase error
	// SYSTEM.POSITION.f16RotorAngle = calculated angle from previous iteration
	// f16Temp = measured angle from previous iteration
	SYSTEM.POSITION.f16AnglePhaseError = f16Temp - SYSTEM.POSITION.f16RotorAngle; 
	// Store to previous angle
	SYSTEM.POSITION.f16RotorAngle_m = SYSTEM.POSITION.f16RotorAngle;
	// Calculate observer
	SYSTEM.POSITION.f16RotorAngle = ACLIB_TrackObsrv(SYSTEM.POSITION.f16AnglePhaseError, &SYSTEM.POSITION.acToPos);
	
	
	// Load new PWM values	
	ioctl(EFPWMA, EFPWM_CENTER_ALIGN_UPDATE_VALUE_REGS_COMPL_012, &SYSTEM.PWMValues);

}

#pragma interrupt saveall
void QT_B3_ISR(void)
{
	ioctl(QTIMER_B3, QT_CLEAR_COMPARE_FLAG, QT_COMPARE1_FLAG|QT_COMPARE2_FLAG);
}

#pragma interrupt saveall
void PWM_A0_CompareISR(void)
{
	int i = 0;
	i++;
}

#pragma interrupt saveall
void RX1_Full_ISR(void)
{
	unsigned int data;

	data = ioctl(SCI_1, SCI_GET_STATUS_REG, NULL);		// Clear RDRF flag
	data = ioctl(SCI_1, SCI_READ_DATA, NULL);			// Read data
}

#pragma interrupt saveall
void SPI_0_RX_FULL_ISR(void)
{
	
}

#pragma interrupt saveall
void PIT_0_ISR(void)
{
	ioctl(PIT_0, PIT_CLEAR_ROLLOVER_INT, NULL);
	checkSystemStates();
}
