/*
 * functions.c
 *
 *  Created on: Mar 29, 2015
 *      Author: Jure
 */

#include "allincludes.h"

#pragma interrupt called
Int16 LogError(UInt8 ui8Error)
{
	// Store error to log
	SYSTEM.i8ErrorLog[SYSTEM.i16ErrorIndex] = ui8Error;
	SYSTEM.i16ErrorIndex++;
	SYSTEM.i16ErrorIndex = SYSTEM.i16ErrorIndex & 0x0F;
	
	return 0;
}


#pragma interrupt called
Int16 OneMsEvent(void)
{

	// Check that DRV is alive
	if(1 == DRV_DATA_READ)
	{
		// Reset to 0
		DRV_DATA_READ = 0;
		// Initiate new read
		DRV_POLL = 1;
		// Check and store if there are errors
		if(DRV8301.StatReg1.FAULT)
		{
			LogError(ERROR_DRV8301_STAT1);
		}
	}
	else
	{
		// Data not read in 1 ms, mark error
		LogError(ERROR_DRV8301_READ);
	}
		
	return 0;
}

// Function that calculates SI values from frac values
#pragma interrupt called
Int16 CalculateSIValues(void)
{
	float fTemp = 0.0;
	float fTemp1 = 0.0;
	
	// UIn
	fTemp = (float)SYSTEM.ADC.f16DCLinkVoltageFiltered;
	fTemp = fTemp * SI_UIN_FACTOR;
	SYSTEM.SIVALUES.fUIn = fTemp / 32768;
	
	// Pin
	fTemp = (float)SYSTEM.MCTRL.m2IDQ.f16Q;		// Iq
	fTemp = fTemp * SI_IIN_FACTOR;
	fTemp = fTemp / 32768;
	fTemp1 = (float)SYSTEM.MCTRL.m2UDQ.f16Q;	// Uq
	fTemp1 = fTemp1 * SI_UIN_FACTOR;
	fTemp1 = fTemp1 / 65535;//32768;
	SYSTEM.SIVALUES.fPIn = fTemp * fTemp1;		// P = Iq * Uq
	
	fTemp = (float)SYSTEM.MCTRL.m2IDQ.f16D;		// Id
	fTemp = fTemp * SI_IIN_FACTOR;
	fTemp = fTemp / 32768;
	fTemp1 = (float)SYSTEM.MCTRL.m2UDQ.f16D;	// Ud
	fTemp1 = fTemp1 * SI_UIN_FACTOR;
	fTemp1 = fTemp1 / 65535;//32768;
	SYSTEM.SIVALUES.fPIn += fTemp * fTemp1;		// P = Iq * Uq

	// IIn
	SYSTEM.SIVALUES.fIIn = SYSTEM.SIVALUES.fPIn / SYSTEM.SIVALUES.fUIn;
	// Filter I in
	SYSTEM.SIVALUES.fIInAcc += SYSTEM.SIVALUES.fIIn;
	SYSTEM.SIVALUES.fIInFilt = SYSTEM.SIVALUES.fIInAcc * SYSTEM.SIVALUES.fIInFiltDiv; 
	SYSTEM.SIVALUES.fIInAcc -= SYSTEM.SIVALUES.fIInFilt; 
	
	// RPM
	fTemp = (float)SYSTEM.POSITION.f16SpeedFiltered;
	fTemp = fTemp * SI_RPM_FACTOR;
	SYSTEM.SIVALUES.fRPM = fTemp / 32768;
	
	// Calculate angle error in deg, scaled to frac16
	fTemp = SYSTEM.SIVALUES.fRPM * SYSTEM.POSITION.fOffsetCalcFactor;
	// Store 	
	SYSTEM.POSITION.f16AngleOffset = (Frac16)fTemp;
	
	// Calculate how much voltage we can have for Q regulator and set limits
	// Ud squared
	fTemp = (float)SYSTEM.MCTRL.m2UDQ.f16D;
	fTemp = fTemp / 32768;
	fTemp = fTemp * fTemp;
	fTemp = 0.9f - fTemp;
	fTemp = sqrtf(fTemp);
	fTemp = fTemp * 32768;
	SYSTEM.REGULATORS.f16UqRemaining = (Frac16)fTemp;
	
	// Calculate phase current
	// Id^2
	fTemp = (float)SYSTEM.MCTRL.m2IDQ.f16D;
	fTemp = fTemp / 32768;
	fTemp = fTemp * fTemp;
	// Iq^2
	fTemp1 = (float)SYSTEM.MCTRL.m2IDQ.f16Q;
	fTemp1 = fTemp1 / 32768;
	fTemp1 = fTemp1 * fTemp1;
	// Add
	fTemp += fTemp1;
	// Sqrt
	fTemp = sqrtf(fTemp);
	// Calculate FRAC16 value
	fTemp = fTemp * 32768;
	SYSTEM.MCTRL.f16IPh = (Frac16)fTemp;	

	
	return 0;
}


Int16 CalculateCalibrationData(void)
{
	Int16 i16Temp0 = 0;
	Int16 i16Temp1 = 0;
	Int16 i16Temp2 = 0;
	Int16 i16Temp3 = 0;
	Int16 i32Temp0 = 0;
	Frac16 f16Temp0 = FRAC16(0.0);
	// Zero array
	for(i16Temp0 = 0; i16Temp0 < 1024; i16Temp0 ++)
	{
		SYSTEM.CALIBRATION.f16CalibrationArray[i16Temp0] = FRAC16(0.0);
	}
	// Interpolate values between poles
	// i16Temp0,1,2
	// Use temp0 for counting pole pairs 											
	for(i16Temp0 = 0; i16Temp0 < SYSTEM.CALIBRATION.i16MotorPolePairs; i16Temp0 ++)
	{
		// Temp1 stores current index in calibration array
		i16Temp1 = SYSTEM.CALIBRATION.i16PolePairArray[i16Temp0];	
		// Temp2 stores number of samples for current pole pair
		if((i16Temp0 + 1) == SYSTEM.CALIBRATION.i16MotorPolePairs)	
		{
			// If last pole pair
			i16Temp2 = SYSTEM.CALIBRATION.i16MaxSensorIndex - SYSTEM.CALIBRATION.i16PolePairArray[i16Temp0] + SYSTEM.CALIBRATION.i16PolePairArray[0];
		}
		else
		{
			i16Temp2 = SYSTEM.CALIBRATION.i16PolePairArray[i16Temp0 + 1] - SYSTEM.CALIBRATION.i16PolePairArray[i16Temp0];
		}						
		// f16Temp0 stores angle delta in current pole
		i32Temp0 = 65536 / i16Temp2;
		f16Temp0 = (Frac16)i32Temp0;		
		// Set first value
		SYSTEM.CALIBRATION.f16CalibrationArray[i16Temp1] = FRAC16(-1.0);
		// Calculate the rest
		for(i16Temp3 = 0; i16Temp3 < i16Temp2; i16Temp3++)
		{
			// Move to next index
			i16Temp1 ++;
			// Wrap to 0
			i16Temp1 = i16Temp1 & 4095;
			// Add difference
			if(0 == i16Temp1)
			{
				SYSTEM.CALIBRATION.f16CalibrationArray[i16Temp1] = SYSTEM.CALIBRATION.f16CalibrationArray[4095] + f16Temp0;
			}
			else
			{
				SYSTEM.CALIBRATION.f16CalibrationArray[i16Temp1] = SYSTEM.CALIBRATION.f16CalibrationArray[i16Temp1 - 1] + f16Temp0;
			}							
		}						
	}
	// Recalculate last pole value
	//i16MaxSensorIndex
	//i16MinSensorIndex
	// Store index for zero angle
	i16Temp1 = SYSTEM.CALIBRATION.i16MaxSensorIndex - SYSTEM.CALIBRATION.i16PolePairArray[SYSTEM.CALIBRATION.i16MotorPolePairs - 1];
	// Get increment
	i16Temp2 = SYSTEM.CALIBRATION.i16PolePairArray[0] - SYSTEM.CALIBRATION.i16MinSensorIndex;
	i16Temp2 = i16Temp2 + i16Temp1;
	// f16Temp0 stores angle delta in current pole
	i32Temp0 = 65536 / i16Temp2;
	f16Temp0 = (Frac16)i32Temp0;
	// Set first index
	i16Temp1 = SYSTEM.CALIBRATION.i16PolePairArray[SYSTEM.CALIBRATION.i16MotorPolePairs - 1];
	// Set first value
	SYSTEM.CALIBRATION.f16CalibrationArray[i16Temp1] = FRAC16(-1.0);
	// Fill until last
	for(i16Temp0 = i16Temp1 + 1; i16Temp0 < SYSTEM.CALIBRATION.i16MaxSensorIndex; i16Temp0 ++)	
	{
		SYSTEM.CALIBRATION.f16CalibrationArray[i16Temp0] = SYSTEM.CALIBRATION.f16CalibrationArray[i16Temp0 - 1] + f16Temp0;
	}
	// Fill until last					
	for(i16Temp0 = SYSTEM.CALIBRATION.i16MaxSensorIndex; i16Temp0 < 4096; i16Temp0 ++)	
	{
		SYSTEM.CALIBRATION.f16CalibrationArray[i16Temp0] = SYSTEM.CALIBRATION.f16CalibrationArray[i16Temp0 - 1];
	}				
	// Now for lower part of values
	// Fill first value
	SYSTEM.CALIBRATION.f16CalibrationArray[0] = SYSTEM.CALIBRATION.f16CalibrationArray[4095];
	// Fill first part
	for(i16Temp0 = 1; i16Temp0 < SYSTEM.CALIBRATION.i16MinSensorIndex; i16Temp0 ++)	
	{
		SYSTEM.CALIBRATION.f16CalibrationArray[i16Temp0] = SYSTEM.CALIBRATION.f16CalibrationArray[i16Temp0 - 1];
	}
	// Fill the rest
	for(i16Temp0 = SYSTEM.CALIBRATION.i16MinSensorIndex; i16Temp0 < SYSTEM.CALIBRATION.i16PolePairArray[0]; i16Temp0 ++)	
	{
		SYSTEM.CALIBRATION.f16CalibrationArray[i16Temp0] = SYSTEM.CALIBRATION.f16CalibrationArray[i16Temp0 - 1] + f16Temp0;
	}		
	// Set last value
	SYSTEM.CALIBRATION.f16CalibrationArray[SYSTEM.CALIBRATION.i16PolePairArray[0]] = FRAC16(1.0);

	return 0;
}


// Function calculates float value from shift/gain
Int16 CalculateFloat(void)
{
	float fTemp1 = 0.0;
	float fTemp2 = 0.0;
	float fTemp3 = 0.0;
	int power = -pConv.shift;
	fTemp1 = (float)pConv.gain;
	fTemp1 = fTemp1 / 32768;
	// Calculate factor
	fTemp2 = 1.0f;
	if(0 < power)
	{
		while(0 != power)
		{
			power --;
			fTemp2 = fTemp2 * 2;
		}
	}
	else if(0 > power)
	{
		while(0 != power)
		{
			power ++;
			fTemp2 = fTemp2 / 2;
		}
	}
	fTemp3 = fTemp1 / fTemp2;
	pConv.value = fTemp3;	
	return 0;
}

// Function calculates frac gain and int shift from float input
Int16 CalculateShiftGain(float K)
{
	Int16 shift1;
	Int16 shift2;
	Int16 shift;
	Frac16 gain1;
	float x;
	float logK;
	float log05 = log(0.5);
	float log2 = log(2.0);
	
	logK = log(K);
	
	x = logK - log05;
	x = x / log2;
	shift1 = (Int16)x;
	
	x = logK / log2;
	shift2 = (Int16)x;	
	
	// Select the right shift
	// Larger of the two numbers (abs value)
	// Sign determined by logK
	if(0 <= logK)
	{
		// Both positive
		if(shift1 > shift2)
		{
			shift = shift1;
		}
		else
		{
			shift = shift2;
		}		
	}
	else
	{
		// Both negative
		if(shift1 > shift2)
		{
			shift = shift2;
		}
		else
		{
			shift = shift1;
		}
	}	
	
	// Calculate shift
	pConv.shift = shift;
	
	// Calculate gain
	x = K;
	while(shift != 0)
	{
		if(0 < shift)
		{
			x = x/2;
			shift--;
		}
		else
		{
			x = x*2;
			shift++;
		}
	}
	
	// Convert to fractional
	x = x * 32768;
	pConv.gain = (Frac16)x;

	return 0;
}

// Function calculates frac factors from float values
void calculateFactors(void)
{
	
	// Id regulator
	// D regulator proportional term
	CalculateShiftGain(SYSTEM.REGULATORS.Kpd);
	SYSTEM.REGULATORS.mudtControllerParamId.f16PropGain = pConv.gain;
	SYSTEM.REGULATORS.mudtControllerParamId.i16PropGainShift = pConv.shift;
	// D regulator integral term
	CalculateShiftGain(SYSTEM.REGULATORS.Kid);
	SYSTEM.REGULATORS.mudtControllerParamId.f16IntegGain = pConv.gain;
	SYSTEM.REGULATORS.mudtControllerParamId.i16IntegGainShift = pConv.shift;
	
	// Iq regulator
	// Q regulator proportional term
	CalculateShiftGain(SYSTEM.REGULATORS.Kpq);
	SYSTEM.REGULATORS.mudtControllerParamIq.f16PropGain = pConv.gain;
	SYSTEM.REGULATORS.mudtControllerParamIq.i16PropGainShift = pConv.shift;
	// Q regulator integral term
	CalculateShiftGain(SYSTEM.REGULATORS.Kiq);
	SYSTEM.REGULATORS.mudtControllerParamIq.f16IntegGain = pConv.gain;
	SYSTEM.REGULATORS.mudtControllerParamIq.i16IntegGainShift = pConv.shift;	
	
	// W regulator
	// W regulator proportional term
	CalculateShiftGain(SYSTEM.REGULATORS.Kpw);
	SYSTEM.REGULATORS.mudtControllerParamW.f16PropGain = pConv.gain;
	SYSTEM.REGULATORS.mudtControllerParamW.i16PropGainShift = pConv.shift;
	// W regulator integral term
	CalculateShiftGain(SYSTEM.REGULATORS.Kiw);
	SYSTEM.REGULATORS.mudtControllerParamW.f16IntegGain = pConv.gain;
	SYSTEM.REGULATORS.mudtControllerParamW.i16IntegGainShift = pConv.shift;
	
	// AcToPos
	// Kp
	CalculateShiftGain(SYSTEM.POSITION.Kpactopos);
	SYSTEM.POSITION.acToPos.f16PropGain = pConv.gain;
	SYSTEM.POSITION.acToPos.i16PropGainShift = pConv.shift;
	// Ki
	CalculateShiftGain(SYSTEM.POSITION.Kiactopos);
	SYSTEM.POSITION.acToPos.f16IntegGain = pConv.gain;
	SYSTEM.POSITION.acToPos.i16IntegGainShift = pConv.shift;
	
}

// Function calculates float values from frac factors
void calculateFloats(void)
{
	
	// Calculate regulator gains
	// Kpd
	pConv.gain = SYSTEM.REGULATORS.mudtControllerParamId.f16PropGain;
	pConv.shift = SYSTEM.REGULATORS.mudtControllerParamId.i16PropGainShift;
	CalculateFloat();
	SYSTEM.REGULATORS.Kpd = pConv.value;
	// Kid
	pConv.gain = SYSTEM.REGULATORS.mudtControllerParamId.f16IntegGain;
	pConv.shift = SYSTEM.REGULATORS.mudtControllerParamId.i16IntegGainShift;
	CalculateFloat();
	SYSTEM.REGULATORS.Kid = pConv.value;
	
	// Kpq
	pConv.gain = SYSTEM.REGULATORS.mudtControllerParamIq.f16PropGain;
	pConv.shift = SYSTEM.REGULATORS.mudtControllerParamIq.i16PropGainShift;
	CalculateFloat();
	SYSTEM.REGULATORS.Kpq = pConv.value;
	// Kiq
	pConv.gain = SYSTEM.REGULATORS.mudtControllerParamIq.f16IntegGain;
	pConv.shift = SYSTEM.REGULATORS.mudtControllerParamIq.i16IntegGainShift;
	CalculateFloat();
	SYSTEM.REGULATORS.Kiq = pConv.value;	
	
	// Kpw
	pConv.gain = SYSTEM.REGULATORS.mudtControllerParamW.f16PropGain;
	pConv.shift = SYSTEM.REGULATORS.mudtControllerParamW.i16PropGainShift;
	CalculateFloat();
	SYSTEM.REGULATORS.Kpw = pConv.value;
	// Kiw
	pConv.gain = SYSTEM.REGULATORS.mudtControllerParamW.f16IntegGain;
	pConv.shift = SYSTEM.REGULATORS.mudtControllerParamW.i16IntegGainShift;
	CalculateFloat();
	SYSTEM.REGULATORS.Kiw = pConv.value;
	
	// Kpactopos	
	pConv.gain = SYSTEM.POSITION.acToPos.f16PropGain;
	pConv.shift = SYSTEM.POSITION.acToPos.i16PropGainShift;
	CalculateFloat();
	SYSTEM.POSITION.Kpactopos = pConv.value;
	
	// Kiactopos	
	pConv.gain = SYSTEM.POSITION.acToPos.f16IntegGain;
	pConv.shift = SYSTEM.POSITION.acToPos.i16IntegGainShift;
	CalculateFloat();
	SYSTEM.POSITION.Kiactopos = pConv.value;
	
}

// Stop motor - set all variables to required value
#pragma interrupt called
void StopMotor(void)
{
	// Turn OFF PWM
	ioctl(EFPWMA, EFPWM_SET_OUTPUTS_DISABLE, EFPWM_SUB0_PWM_A|EFPWM_SUB0_PWM_B|EFPWM_SUB1_PWM_A|EFPWM_SUB1_PWM_B|EFPWM_SUB2_PWM_A|EFPWM_SUB2_PWM_B);
	// Reinitialize variables
	InitSysVars(0);
	
	// Disable regulators
	PWM_ENABLED = 0;
	// Set current source
	SYSTEM.REGULATORS.i16CurrentSource = CURRENT_SOURCE_NONE;
	// Set position source
	SYSTEM.POSITION.i16PositionSource = POSITION_SOURCE_NONE;
	// Set no control
	CONTROL_TORQUE = 0;
	CONTROL_SPEED = 0;
	CONTROL_MANUAL = 0;
	
	// Set regulator outputs to 0
	SYSTEM.MCTRL.m2UDQ.f16D = FRAC16(0.0);
	SYSTEM.MCTRL.m2UDQ.f16Q = FRAC16(0.0);
	SYSTEM.MCTRL.m2UDQ_m.f16D = FRAC16(0.0);
	SYSTEM.MCTRL.m2UDQ_m.f16Q = FRAC16(0.0);
	SYSTEM.MCTRL.m3U_UVW.f16A = FRAC16(0.5);
	SYSTEM.MCTRL.m3U_UVW.f16B = FRAC16(0.5);
	SYSTEM.MCTRL.m3U_UVW.f16C = FRAC16(0.5);
	// Store PWMs
	SYSTEM.PWMValues.pwmSub_0_Channel_23_Value = FRAC16(0.5);
	SYSTEM.PWMValues.pwmSub_1_Channel_23_Value = FRAC16(0.5);
	SYSTEM.PWMValues.pwmSub_2_Channel_23_Value = FRAC16(0.5);
	
	// Load new PWM values	
	ioctl(EFPWMA, EFPWM_CENTER_ALIGN_UPDATE_VALUE_REGS_COMPL_012, &SYSTEM.PWMValues);	
	
	/*
	// Disable PWMs
	// Mark dont align rotor
	ALIGN = 0;
	// Do not check merge
	MERGE_CHECK=0;
	// Observer is inactive
	OBSERVER_ACTIVE = 0;
	// System not running from BEMF
	RUNNING_FROM_BEMF = 0;		
	// Set to speed mode
	MOTOR_MODE_TORQUE = 0;	
	// Disable algo
	RUN_ADC_EOS_ALG = 0;
	SYSTEM_GOTO_ACTIVE = 0;
	SYSTEM_GOTO_RUN = 0;
	
	RUN_WSENSOR = 0;
	RUN_FROMSENSOR = 0;
	SENSOR_PREDICTION_ZEROED = 0;
	// Stop calibrating
	if(SYSTEM_MAN_ROTATE)
	{
		SYSTEM_MAN_ROTATE = 0;
		SYSTEM_CALIBRATED = 0;
		SYSTEM_SPINNING = 0;
		systemVariables.ui16CalibrationState = CALIBRATE_INIT;
	}*/
}

#pragma interrupt called
void interruptDelay(unsigned int count)
{
	//delay approx. count * 1 uSec
	int i = 0;
	int j = 0;
	for(i = 0; i < count; i++)
	{
		/* feed the watchdog periodically */
    	 	ioctl(COP, COP_CLEAR_COUNTER, NULL);
		i++;
		for(j=0; j<12; j++)
		{
			asm(nop);
		}
	}
}

void delay(unsigned int count)
{
	//delay approx. count * 1 uSec
	int i = 0;
	int j = 0;
	for(i = 0; i < count; i++)
	{
		/* feed the watchdog periodically */
    	 	ioctl(COP, COP_CLEAR_COUNTER, NULL);
		i++;
		for(j=0; j<12; j++)
		{
			asm(nop);
		}
	}
}

// Read from a DRV8301 Register
UInt16 DRV8301_SPI_Read(UInt16 uiAddress)
{
	volatile UInt16 wDummy,wDummyCnt;
	wDummyCnt=0;
	
	DRV8301.RegReq.RW = 1;					//we are initiating a read
	DRV8301.RegReq.ADDR = uiAddress;		//load the address
	DRV8301.RegReq.DATA = 0;				//dummy data;
					
	// Send data
	ioctl(SPI_0, SPI_WRITE_DATA, DRV8301.RegReq.reg);
	
	
	// Wait for finish
	wDummyCnt=0;
	while((ioctl(SPI_0, SPI_CAN_READ_DATA, null) == 0)||(wDummyCnt<100))
	{
		wDummyCnt++;		
	}
	
	wDummy = ioctl(SPI_0, SPI_READ_DATA, null);					//dummy read to clear the INT_FLAG bit
	
	DRV8301.RegReq.RW = 1;					//we are initiating a read
	DRV8301.RegReq.ADDR = uiAddress;		//load the address
	DRV8301.RegReq.DATA = 0;				//dummy data;
	
	ioctl(SPI_0, SPI_WRITE_DATA, DRV8301.RegReq.reg);
	
	// Wait for finish
	wDummyCnt=0;
	while((ioctl(SPI_0, SPI_CAN_READ_DATA, null) == 0)||(wDummyCnt<100))
	{
		wDummyCnt++;		
	}
	
	wDummy = ioctl(SPI_0, SPI_READ_DATA, null);					//dummy read to clear the INT_FLAG bit

	return(wDummy);
}

// Write to a DRV8301 Register
// SPI writes always clock out the data in Status Register 1.
// Since it's available we'll return the status from this function
UInt16 DRV8301_SPI_Write(UInt16 uiAddress, UInt16 uiData)
{
	volatile UInt16 stat_reg1,wDummyCnt;
	
	DRV8301.RegReq.RW = 0;					//we are initiating a write
	DRV8301.RegReq.ADDR = uiAddress;		//load the address
	DRV8301.RegReq.DATA = uiData;			//data to be written;
	
	ioctl(SPI_0, SPI_WRITE_DATA, DRV8301.RegReq.reg);
	
	// Wait for finish
	wDummyCnt=0;
	while((ioctl(SPI_0, SPI_CAN_READ_DATA, null) == 0)||(wDummyCnt<100))
	{
		wDummyCnt++;		
	}
	
	stat_reg1 = ioctl(SPI_0, SPI_READ_DATA, null);					// read returned value of Status Register 1 and clear the INT_FLAG bit

	return(stat_reg1);	
}

// Initialize Driver
Int16 InitDRV8301(Int16 wReset, Int16 wCurrLimit, Int16 wOC_MODE)
{
	Int16 result = 0;
	Int16 wTestRead;
	Int16 wDelayspi;
	Int16 wDummyCnt;	
	wDelayspi=0;
	wTestRead=0;

			
			
	DRV8301.CtrlReg1.GATE_CURRENT = 0;		// full current 1.7A
//	DRV8301.CtrlReg1.GATE_CURRENT = 1;		// med current 0.7A
//	DRV8301.CtrlReg1.GATE_CURRENT = 2;		// min current 0.25A

	DRV8301.CtrlReg1.GATE_RESET = 0;			// Normal Mode
			
	DRV8301.CtrlReg1.PWM_MODE = 0;			// six independant PWMs
			
	DRV8301.CtrlReg1.OC_MODE = 2;			// No shutdown
	//DRV8301.CtrlReg1.OC_MODE = 1;	// latched OC shutdown
			
	DRV8301.CtrlReg1.OC_ADJ_SET = 31;//wCurrLimit;	// OC @ Vds=0.25V
	DRV8301.CtrlReg1.Reserved = 0;
			
	DRV8301.CtrlReg2.OCTW_SET = 0;			// report OT and OC
	//DRV8301.CtrlReg2.OCTW_SET = 1;			// report OT only


	DRV8301.CtrlReg2.GAIN = DRV8301_GAIN_AMP;
	DRV8301.CtrlReg2.DC_CAL_CH1 = 0;			// not in CS calibrate mode
	DRV8301.CtrlReg2.DC_CAL_CH2 = 0;			// not in CS calibrate mode
	DRV8301.CtrlReg2.OC_TOFF = 0;				// normal mode
	DRV8301.CtrlReg2.Reserved = 0;
			
	//write to DRV8301 control register 1, returns status register 1 
	DRV8301.StatReg1.reg = DRV8301_SPI_Write(DRV8301_CNTRL_REG_1_ADDR, DRV8301.CtrlReg1.reg);
			
	//write to DRV8301 control register 2, returns status register 1
	DRV8301.StatReg1.reg = DRV8301_SPI_Write(DRV8301_CNTRL_REG_2_ADDR, DRV8301.CtrlReg2.reg);
			
	wTestRead=DRV8301_SPI_Read(DRV8301_CNTRL_REG_1_ADDR);
		
	wTestRead&=0b0000011111111111; //mask only bits 10-0
	if(wTestRead!=DRV8301.CtrlReg1.reg)
	{
		result = -1;
	}
	else
	{
		result = 0;
	}
	
	wTestRead=DRV8301_SPI_Read(DRV8301_CNTRL_REG_2_ADDR);
	wTestRead&=0b0000011111111111; //mask only bits 10-0
	if(wTestRead!=DRV8301.CtrlReg2.reg)
	{
		result = -1;
	}
	else
	{
		result = 0;
	}
	
	return result;
}

// Ring buffer functions

Int16 RB_full(RING_BUFFER* rb)
{
    if(rb->count == rb->size) return 0;
    else return -1;
}

Int16 RB_Init(RING_BUFFER* rb, UInt8 *buf, Int16 size)
{
	rb->buffer = buf;

    rb->buffer_end = rb->buffer + size;
    rb->size = size;
    rb->data_start = rb->buffer;
    rb->data_end = rb->buffer;
    rb->count = 0;

	return 0;
}

Int16 RB_push(RING_BUFFER* rb, UInt8 data)
{
    *rb->data_end = data;
    rb->data_end++;
    if (rb->data_end == rb->buffer_end)
        rb->data_end = rb->buffer;

    if (0 == RB_full(rb))
    {
        if ((rb->data_start + 1) == rb->buffer_end)
        {
            rb->data_start = rb->buffer;
        }
        else
        {
            rb->data_start++;
        }
    }
    else
    {
        rb->count++;
    }
	return 0;
}

UInt8 RB_pop(RING_BUFFER* rb)
{
    UInt8 data = *rb->data_start;
    rb->data_start++;
    if (rb->data_start == rb->buffer_end)
    {
        rb->data_start = rb->buffer;
    }
    rb->count--;

    return data;
}

Int16 RB_flush(RING_BUFFER* rb)
{	
    rb->data_start = rb->buffer;
    rb->data_end = rb->buffer;
    rb->count = 0;
    return 0;
}
