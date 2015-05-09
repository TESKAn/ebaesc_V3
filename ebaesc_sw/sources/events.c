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
	Int16 i16Temp1 = 0;
	Int16 i16Temp2 = 0;
	Frac16 f16Temp = FRAC16(0.0);
	Frac16 mf16ErrorK;
	Int16 mi16SatFlag = 0;
	Int16 mi16SatFlagD = 0;
	Int16 mi16SatFlagQ = 0;
	
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
	
	// Get phase voltage values
	i16Temp = ioctl(EFPWMA_SUB0, EFPWMS_READ_CAPTURE_VAL0, NULL);
	i16Temp1 = ioctl(EFPWMA_SUB0, EFPWMS_READ_CAPTURE_VAL1, NULL);
	i16Temp2 = i16Temp1 - i16Temp;
	SYSTEM.MCTRL.m3U_X_UVW.f16A = (Frac16)i16Temp2;
	
	// Get measured angle
	// Get current position index
	SYSTEM.POSITION.i16SensorIndex = (Int16)(SYSTEM.ADC.f16SensorValueB >> 3);
	// Get filtered current position
	SYSTEM.POSITION.i16SensorIndexFiltered = (Int16)(SYSTEM.ADC.f16SensorValueBFiltered >> 3);
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
	SYSTEM.POSITION.i16SensorIndex = SYSTEM.POSITION.i16SensorIndex & 4095;
	// Get measured angle from previous iteration
	f16Temp = SYSTEM.CALIBRATION.f16CalibrationArray[SYSTEM.POSITION.i16SensorIndex_m];
	// Calculate phase error
	// SYSTEM.POSITION.f16RotorAngle = calculated angle from previous iteration
	// f16Temp = measured angle from previous iteration
	SYSTEM.POSITION.f16AnglePhaseError = f16Temp - SYSTEM.POSITION.f16RotorAngle; 
	// Store to previous angle
	SYSTEM.POSITION.f16RotorAngle_m = SYSTEM.POSITION.f16RotorAngle;
	
	// Clark transform to get Ia, Ib
	MCLIB_ClarkTrf(&SYSTEM.MCTRL.m2IAlphaBeta, &SYSTEM.ADC.m3IphUVW);
	// Calculate park transform to get Id, Iq
	// Out m2IDQ
	MCLIB_ParkTrf(&SYSTEM.MCTRL.m2IDQ, &SYSTEM.MCTRL.m2IAlphaBeta, &SYSTEM.POSITION.mSinCosAngle);
	
	
	
	
	
	
	
	// Here are differences according to how we are running the motor
	// Run BEMF observer to get BEMF error
	// Merge(?) errors from measured and observed angle
	// Calculate angle tracking observer or use forced angle
	SYSTEM.POSITION.f16RotorAngle = ACLIB_TrackObsrv(SYSTEM.POSITION.f16AnglePhaseError, &SYSTEM.POSITION.acToPos);
	
	
	
	// Calculate new sin/cos
	SYSTEM.POSITION.mSinCosAngle.f16Sin = GFLIB_SinTlr(SYSTEM.POSITION.f16RotorAngle);
	SYSTEM.POSITION.mSinCosAngle.f16Cos = GFLIB_CosTlr(SYSTEM.POSITION.f16RotorAngle);
	
	/*
	 * TODO: Speed controller to set Id
	 */
	
	
	// Do PI regulation for D, Q current to get D, Q voltages
	// Store result in systemVariables.MOTOR.mudtDQInv
	// D
	// Error calculation
	mf16ErrorK = SYSTEM.REGULATORS.m2IDQReq.f16D - SYSTEM.MCTRL.m2IDQ.f16D;
	// Controller calculation
	SYSTEM.MCTRL.m2UDQ.f16D = GFLIB_ControllerPIp(mf16ErrorK, &SYSTEM.REGULATORS.mudtControllerParamId, &mi16SatFlagD);
	// Q
	// Calculate error
	mf16ErrorK = SYSTEM.REGULATORS.m2IDQReq.f16Q - SYSTEM.MCTRL.m2IDQ.f16Q;
	/*
	 * TODO: Limit voltages to available 
	 */  
	// Controller calculation
	SYSTEM.MCTRL.m2UDQ.f16Q = GFLIB_ControllerPIp(mf16ErrorK, &SYSTEM.REGULATORS.mudtControllerParamIq, &mi16SatFlagQ);
	// We have Ud, Uq
	// Do inverse park
	MCLIB_ParkTrfInv(&SYSTEM.MCTRL.m2UAlphaBeta, &SYSTEM.MCTRL.m2UDQ, &SYSTEM.POSITION.mSinCosAngle);
	// Eliminate DC bus ripple
	//MCLIB_ElimDcBusRip(systemVariables.MOTOR.f16InvModeIndex, systemVariables.MOTOR.f16DCBusVoltage, &systemVariables.MOTOR.m2UAlphaBeta, &systemVariables.MOTOR.m2UAlphaBetaRippleElim);
	// Do SVM
	//systemVariables.MOTOR.SVMVoltageSector = MCLIB_SvmStd(&systemVariables.MOTOR.m2UAlphaBetaRippleElim, &systemVariables.MOTOR.m3U_UVW);
	SYSTEM.MCTRL.SVMVoltageSector = MCLIB_SvmStd(&SYSTEM.MCTRL.m2UAlphaBeta, &SYSTEM.MCTRL.m3U_UVW);
	// Store PWMs
	SYSTEM.PWMValues.pwmSub_0_Channel_23_Value = SYSTEM.MCTRL.m3U_UVW.f16A;
	SYSTEM.PWMValues.pwmSub_1_Channel_23_Value = SYSTEM.MCTRL.m3U_UVW.f16B;
	SYSTEM.PWMValues.pwmSub_2_Channel_23_Value = SYSTEM.MCTRL.m3U_UVW.f16C;
	
	// Load new PWM values	
	//ioctl(EFPWMA, EFPWM_CENTER_ALIGN_UPDATE_VALUE_REGS_COMPL_012, &SYSTEM.PWMValues);

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
