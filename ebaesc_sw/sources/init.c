/*
 * init.c
 *
 *  Created on: Mar 29, 2015
 *      Author: Jure
 */

#include "allincludes.h"

/*
 * If initial != 0, init all variables. Else only reset.
 */
void InitSysVars(Int16 initial)
{	
	float fTemp = 0.0f;
	//******************************************
	// Motor ID
	//******************************************	
	COMMDataStruct.REGS.ui8ID = COMM_ID;
	//******************************************
	// Calibration
	//******************************************
	if(0 != initial)
	{
		SYSTEM.CALIBRATION.i16MotorPolePairs = M_POLEPAIRS;
		SYSTEM.CALIBRATION.i16PolePairArray[0] = M_POLEARRAY_0;
		SYSTEM.CALIBRATION.i16PolePairArray[1] = M_POLEARRAY_1;
		SYSTEM.CALIBRATION.i16PolePairArray[2] = M_POLEARRAY_2;
		SYSTEM.CALIBRATION.i16PolePairArray[3] = M_POLEARRAY_3;
		SYSTEM.CALIBRATION.i16PolePairArray[4] = M_POLEARRAY_4;
		SYSTEM.CALIBRATION.i16PolePairArray[5] = M_POLEARRAY_5;
		SYSTEM.CALIBRATION.i16PolePairArray[6] = M_POLEARRAY_6;
		SYSTEM.CALIBRATION.i16PolePairArray[7] = M_POLEARRAY_7;
		SYSTEM.CALIBRATION.i16MaxSensorIndex = M_MAXSENSORINDEX;
		SYSTEM.CALIBRATION.i16MinSensorIndex = M_MINSENSORINDEX;
		SYSTEM.CALIBRATION.i16CalibrationState = CALIBRATE_INIT;
	}

		
	//******************************************
	// Regulators
	//******************************************
	// D, Q regulators
	// D
	if(0 != initial)
	{	
		SYSTEM.REGULATORS.mudtControllerParamId.a32PGain = D_KP_ACC32;
		SYSTEM.REGULATORS.mudtControllerParamId.a32IGain = D_KI_ACC32;
		//SYSTEM.REGULATORS.mudtControllerParamId.a32PGain = 0;
		//SYSTEM.REGULATORS.mudtControllerParamId.a32IGain = 0;		
		SYSTEM.REGULATORS.mudtControllerParamId.f16UpperLim = FRAC16(0.95);
		SYSTEM.REGULATORS.mudtControllerParamId.f16LowerLim = FRAC16(-0.95);
		
		// Q
		SYSTEM.REGULATORS.mudtControllerParamIq.a32PGain = Q_KP_ACC32;
		SYSTEM.REGULATORS.mudtControllerParamIq.a32IGain = Q_KI_ACC32;;
		SYSTEM.REGULATORS.mudtControllerParamIq.f16UpperLim = FRAC16(0.95);
		SYSTEM.REGULATORS.mudtControllerParamIq.f16LowerLim = FRAC16(-0.95);
		
		// W regulator
		SYSTEM.REGULATORS.mudtControllerParamW.a32PGain = SPEED_PI_PROP_ACC32;
		SYSTEM.REGULATORS.mudtControllerParamW.a32IGain = SPEED_PI_INTEG_ACC32;
		SYSTEM.REGULATORS.mudtControllerParamW.f16UpperLim = SPEED_LOOP_HIGH_LIMIT;
		SYSTEM.REGULATORS.mudtControllerParamW.f16LowerLim = SPEED_LOOP_LOW_LIMIT;	
		
		SYSTEM.REGULATORS.ui16SpeedRegInterval = SPEED_LOOP_CNTR;
	}
	GFLIB_CtrlPIpAWInit_F16(FRAC16(0), &SYSTEM.REGULATORS.mudtControllerParamId);
	GFLIB_CtrlPIpAWInit_F16(FRAC16(0), &SYSTEM.REGULATORS.mudtControllerParamIq);
	GFLIB_CtrlPIpAWInit_F16(FRAC16(0), &SYSTEM.REGULATORS.mudtControllerParamW);
	
	
	// Set req currents to 0
	SYSTEM.REGULATORS.m2IDQReq.f16D = FRAC16(0.0);
	SYSTEM.REGULATORS.m2IDQReq.f16Q = FRAC16(0.0);

	// Set counter interval
	SYSTEM.REGULATORS.ui16SpeedRegCounter = 0;
	// D,Q,W saturation flags
	SYSTEM.REGULATORS.i16SatFlagD = 0;
	SYSTEM.REGULATORS.i16SatFlagQ = 0;
	SYSTEM.REGULATORS.i16SatFlagW = 0;
	
	SYSTEM.REGULATORS.i16CurrentSource = CURRENT_SOURCE_NONE;
	
	//******************************************
	// Position
	//******************************************
	// Init for BEMF observer
	
	if(0 != initial)
	{
		SYSTEM.POSITION.acBemfObsrvDQ.sCtrl.a32PGain = BEMF_DQ_KP;
		SYSTEM.POSITION.acBemfObsrvDQ.sCtrl.a32IGain = BEMF_DQ_KI;
		SYSTEM.POSITION.acBemfObsrvDQ.a32IGain = I_SCALE;
		SYSTEM.POSITION.acBemfObsrvDQ.a32UGain = U_SCALE;
		SYSTEM.POSITION.acBemfObsrvDQ.a32WIGain= WI_SCALE;
		SYSTEM.POSITION.acBemfObsrvDQ.a32EGain = E_SCALE;
	}	
	AMCLIB_PMSMBemfObsrvDQInit_F16(&SYSTEM.POSITION.acBemfObsrvDQ);
	// Tracking observer
	if(0 != initial)
	{
		SYSTEM.POSITION.acToPos.f16PGain= TO_KP_GAIN;
		SYSTEM.POSITION.acToPos.i16PGainSh= TO_KP_SHIFT;
		SYSTEM.POSITION.acToPos.f16IGain= TO_KI_GAIN;
		SYSTEM.POSITION.acToPos.i16IGainSh= TO_KI_SHIFT;
		SYSTEM.POSITION.acToPos.f16ThGain= TO_THETA_GAIN;
		SYSTEM.POSITION.acToPos.i16ThGainSh = TO_THETA_SHIFT;	
	}
	
	AMCLIB_TrackObsrvInit_F16(FRAC16(0.0), &SYSTEM.POSITION.acToPos);
	
	// Other init - once
	if(0 != initial)
	{
		// Position offset
		SYSTEM.POSITION.i16SensorIndexOffset = 0;
		// Phase delay from speed
		SYSTEM.POSITION.i16SensorIndexPhaseDelay = 0;	
		// Speed MA filter
		SYSTEM.POSITION.FilterMA32Speed.u16Sh = 2;
		GDFLIB_FilterMAInit_F16(FRAC16(0.0), &SYSTEM.POSITION.FilterMA32Speed);
		SYSTEM.POSITION.f16ManualAngleIncrease = MANUAL_ANGLE_INCREASE;			
		SYSTEM.POSITION.f16AddedAngleOffset = FRAC16(0.0);		
		SYSTEM.POSITION.fOffsetCalcFactor = AOFFSET_FACTOR;
	}
	
	SYSTEM.POSITION.acToPos.f32Theta = FRAC32(0.0);
	SYSTEM.POSITION.acToPos.f32Speed = FRAC32(0.0);
	SYSTEM.POSITION.acToPos.f32I_1 = FRAC32(0.0);
	
	SYSTEM.POSITION.f16RotorAngle = FRAC16(0.0);
	SYSTEM.POSITION.f16RotorAngle_m = FRAC16(0.0);

	// Initial speed
	SYSTEM.POSITION.f16Speed = FRAC16(0.0);

	// Filtered speed
	SYSTEM.POSITION.f16SpeedFiltered = FRAC16(0.0);
	
	SYSTEM.POSITION.i16PositionSource = POSITION_SOURCE_NONE;
	
	SYSTEM.POSITION.f16AngleOffset = FRAC16(0.0);
	
	SENSORLESS_BEMF_ON = 0;
	
	//******************************************
	// Sensorless
	//******************************************
	if(0 != initial)
	{
		SYSTEM.SENSORLESS.f16MinSpeed = SENSORLESS_MIN_SPEED;
		SYSTEM.SENSORLESS.f16MaxObserverError = SENSORLESS_MAX_ERROR;
		SYSTEM.SENSORLESS.f16AlignCurrent = SENSORLESS_ALIGN_CURRENT;
		SYSTEM.SENSORLESS.f16StartCurrent = SENSORLESS_START_CURRENT;
		SYSTEM.SENSORLESS.i16AlignTime = OL_ALIGN_TIME;
		SYSTEM.SENSORLESS.f16StartSpeed = SENSORLESS_START_SPEED;
		SYSTEM.SENSORLESS.f16StartTorque = SENSORLESS_START_TORQUE;
		SYSTEM.SENSORLESS.f16AngleManualError = SENSORLESS_ANGLE_MAN_ERROR;
		SYSTEM.SENSORLESS.f16BEMFErrorPart = FRAC16(0.0);		
		SYSTEM.SENSORLESS.ui8MaxBemfObserverErrorCount = MAX_BEMF_ERROR_COUNT;
	}
	SYSTEM.SENSORLESS.f16MinSpeedHysteresis = mult(SYSTEM.SENSORLESS.f16MinSpeed, FRAC16(0.1));
	SYSTEM.SENSORLESS.i16Counter = 0;

	
	//******************************************
	// Ramps
	//******************************************
	if(0 != initial)
	{
		SYSTEM.RAMPS.Ramp16_AlignCurrent.f16RampUp = OL_I_RAMP_UP;
		SYSTEM.RAMPS.Ramp16_AlignCurrent.f16RampDown = OL_I_RAMP_DOWN;	
				
		SYSTEM.RAMPS.Ramp16_Speed.f16RampUp = SPEED_RAMP_UP;
		SYSTEM.RAMPS.Ramp16_Speed.f16RampDown = SPEED_RAMP_DOWN;	
		
		SYSTEM.RAMPS.Ramp16_Torque.f16RampUp = TORQUE_RAMP_UP;
		SYSTEM.RAMPS.Ramp16_Torque.f16RampDown = TORQUE_RAMP_DOWN;	
		
		
	}
	GFLIB_RampInit_F16(FRAC16(0.0), &SYSTEM.RAMPS.Ramp16_AlignCurrent);
	GFLIB_RampInit_F16(FRAC16(0.0), &SYSTEM.RAMPS.Ramp16_Speed);
	GFLIB_RampInit_F16(FRAC16(0.0), &SYSTEM.RAMPS.Ramp16_Torque);
	
	// Startup current ramp
	SYSTEM.RAMPS.f16AlignCurrentActualValue = FRAC16(0.0);
	SYSTEM.RAMPS.f16AlignCurrentDesiredValue = FRAC16(0.0);	
	
	// Speed ramp
	SYSTEM.RAMPS.f16SpeedRampDesiredValue = FRAC16(0.0);
	SYSTEM.RAMPS.f16SpeedRampActualValue = FRAC16(0.0);
	
	// Torque ramp
	SYSTEM.RAMPS.f16TorqueRampDesiredValue = FRAC16(0.0);
	SYSTEM.RAMPS.f16TorqueRampActualValue = FRAC16(0.0);

	//******************************************
	// ADC
	//******************************************
	if(0 != initial)
	{
		// Angle filter
		SYSTEM.ADC.FilterMA32SensorA.u16Sh = 2;
		GDFLIB_FilterMAInit_F16(FRAC16(0.0), &SYSTEM.ADC.FilterMA32SensorA);
		SYSTEM.ADC.FilterMA32SensorB.u16Sh = 2;
		GDFLIB_FilterMAInit_F16(FRAC16(0.0), &SYSTEM.ADC.FilterMA32SensorB);
		
		// DC link MA filter
		SYSTEM.ADC.FilterMA32DCLink.u16Sh = 6;
		GDFLIB_FilterMAInit_F16(FRAC16(0.0), &SYSTEM.ADC.FilterMA32DCLink);

		// Temperature MA filter
		SYSTEM.ADC.FilterMA32Temperature.u16Sh = 6;
		GDFLIB_FilterMAInit_F16(FRAC16(0.0), &SYSTEM.ADC.FilterMA32Temperature);
		
		SYSTEM.ADC.f16OffsetU = FRAC16(0.0);
		SYSTEM.ADC.f16OffsetW = FRAC16(0.0);
		
		SYSTEM.ADC.f16CurrentGainFactor = DRV8301_GAIN_FACTOR;

	}

	// Measured currents to 0
	SYSTEM.ADC.m3IphUVW.f16A = FRAC16(0.0);
	SYSTEM.ADC.m3IphUVW.f16B = FRAC16(0.0);
	SYSTEM.ADC.m3IphUVW.f16C = FRAC16(0.0);
	

	//******************************************
	// MCTRL
	//******************************************
	if(0 != initial)
	{
		SYSTEM.MCTRL.f16TorqueFactor = TORQUE_FACTOR;
	}
	
	// D,Q voltage out
	SYSTEM.MCTRL.m2UDQ.f16D = FRAC16(0.0);
	SYSTEM.MCTRL.m2UDQ.f16Q = FRAC16(0.0);	
	

	//******************************************
	// PWM
	//******************************************
	if(0 != initial)
	{
		// PWM input variables
		SYSTEM.PWMIN.i16PWMfullThrottle = PWM_IN_HIGH_VAL_REF;
		SYSTEM.PWMIN.i16PWMoffThrottle = PWM_IN_LOW_VAL_REF + PWM_IN_OFF_ZONE;
		SYSTEM.PWMIN.i16PWMinThrottle = PWM_IN_LOW_VAL_REF;
		SYSTEM.PWMIN.i16PWMThrottleDifference = PWM_IN_HIGH_VAL_REF - PWM_IN_LOW_VAL_REF;
		
		// PWM input parameters
		SYSTEM.PWMIN.i16PWMInMiddleValue = PWM_IN_MIDDLE_VALUE;
		SYSTEM.PWMIN.i16PWMInHighValRef = PWM_IN_HIGH_VAL_REF;
		SYSTEM.PWMIN.i16PWMInLowValRef = PWM_IN_LOW_VAL_REF;
		SYSTEM.PWMIN.i16PWMInMeasureTime = PWM_IN_MEASURE_TIME;
		SYSTEM.PWMIN.i16PWMInOffZone = PWM_IN_OFF_ZONE;		
		
		SYSTEM.PWMIN.i16PWMFilterSize = PWM_MEAS_FILTER_SAMPLES;
		
	}

	// States for measuring PWM input parameters
	SYSTEM.PWMIN.i16PWMMeasureStates = 0;
	// PWM measure state timer
	SYSTEM.PWMIN.i16PWMMeasureTimer = 0; 
	// PWM timeout parameter
	SYSTEM.PWMIN.i16PWMTimeout = 0;
	
	
	//******************************************
	// SI values
	//******************************************	
	if(0 != initial)
	{
		SYSTEM.SIVALUES.fIInFiltDiv = 0.25f;
	}
	SYSTEM.SIVALUES.fIInAcc = 0.0f;
	
	//******************************************
	// CAN
	//******************************************
	
	//******************************************
	// Measure params
	//******************************************
	SYSTEM.MEASUREPARAMS.f16SetpointMulti = FRAC16(0.7485);		// 1.5 Tau
	SYSTEM.MEASUREPARAMS.f16LphaIset = FRAC16(0.05);			// ~16.48 A
	SYSTEM.MEASUREPARAMS.i16TotalMeasurements = 8;
	
	// Error log init
	for(SYSTEM.i16ErrorIndex = 0; SYSTEM.i16ErrorIndex < 16; SYSTEM.i16ErrorIndex++)
	{
		SYSTEM.i8ErrorLog[SYSTEM.i16ErrorIndex] = 0;
	}
	SYSTEM.i16ErrorIndex = 0;
	
	//******************************************
	// CAN
	//******************************************
	SYSTEM.CAN.ui16CANInfoInterval = CAN_INFO_INTERVAL;
	SYSTEM.CAN.ui16CANInfoTimer = 0;
	
	SYSTEM.CAN.ui16CANStatusInterval = CAN_STATUS_INTERVAL;
	SYSTEM.CAN.ui16CANStatusTimer = 0;
	
	SYSTEM.DRIVERSTATE.i8DriverFaultCount = 0;
	SYSTEM.DRIVERSTATE.i8DriverFault = 0;
	
	SYSTEM.SENSORLESS.ui8BemfObserverErrorCount = 0;
	
	//******************************************
	// Other
	//******************************************
	if(0 != initial)
	{
		// Mark transition to idle state
		SYSTEM.i16StateTransition = SYSTEM_WAKEUP;

		SYSTEM.systemState = SYSTEM_WAKEUP;
		
		// Set driver restart state
		SYSTEM.i16DriverRestartState = SYSTEM_RESTART_INIT;
		
		// If there is calibration data
		if(0 != SYSTEM.CALIBRATION.i16PolePairArray[0])
		{
			// Calculate calibration values
			CalculateCalibrationData();	
			// Mark calibrated
			SYSTEM_CALIBRATED = 1;	
		}
	}	
	SYSTEM.ui32SystemTime = 0;
	
	if(0 != initial)
	{
		// Comm data struct
		// Unit registers
		COMMDataStruct.REGS.ui8ID = COMM_ID;
		COMMDataStruct.REGS.ui16ModelNumber = 0x001c;
		COMMDataStruct.REGS.ui8FirmwareVersion = 1;
		COMMDataStruct.REGS.ui8BaudRate = 3;
		COMMDataStruct.REGS.ui16Errors = 0;
		COMMDataStruct.REGS.ui16State = SYSTEM.systemState;
		COMMDataStruct.REGS.ui8Armed = 0;
		COMMDataStruct.REGS.ui8Park = 0;
		COMMDataStruct.REGS.ui8ReturnDelayTime = 1;
		// Park position
		COMMDataStruct.REGS.i16ParkPosition = M_PARK_POSITION;
		
		COMMDataStruct.ui16RegsBytes = 64;
		COMMDataStruct.errStatus = 0;
		COMMDataStruct.ui16RXCommTimeout = 100;
		COMMDataStruct.ui16RXTimeoutCounter = 0;
		COMMDataStruct.ui16TXCommTimeout = 100;
		COMMDataStruct.ui16TXTimeoutCounter = 0;	
		COMMDataStruct.ui16ReadOnlyLow = 0;
		COMMDataStruct.ui16ReadOnlyHigh = 2;
		
		COMMDataStruct.REGS.i16PWMMax = 2000;
		COMMDataStruct.REGS.i16PWMMin = 1000;
		
		COMMDataStruct.REGS.i16MaxRPM = 18000;
		COMMDataStruct.REGS.i16MinRPM = 2000;
		
		COMMDataStruct.REGS.i16CurrentPWM = 1000;
		COMMDataStruct.REGS.i16ParkPosition = 2048;
		
		COMMDataStruct.REGS.i16ZeroSpeedPWM = 50;
		
		COMMDataStruct.REGS.i16SetRPM = 0;
	}
	
}

// Store vars to EEPROM
UWord32 EEPROMStorei8(UWord32 uw32CurrIndex, Int8 i8Data)
{
	// Var for converting
	t16BitVars t16Vars;
	
	t16Vars.ints.i8[0] = i8Data;
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t16Vars.uw16);
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, (UInt32)t16Vars.uw16);
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMStoreui8(UWord32 uw32CurrIndex, UInt8 ui8Data)
{
	// Var for converting
	t16BitVars t16Vars;
	
	t16Vars.bytes.ui8[0] = ui8Data;
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t16Vars.uw16);
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, (UInt32)t16Vars.uw16);
	
	return uw32CurrIndex + 1;
}

// Store 16 bit signed integer to address and return address + 1
UWord32 EEPROMStorei16(UWord32 uw32CurrIndex, Int16 i16Data)
{
	// Var for converting
	t16BitVars t16Vars;
	
	t16Vars.i16 = i16Data;
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t16Vars.uw16);
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, (UInt32)t16Vars.uw16);
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMStoref16(UWord32 uw32CurrIndex, Frac16 f16Data)
{
	// Var for converting
	t16BitVars t16Vars;
	
	t16Vars.f16 = f16Data;
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t16Vars.uw16);
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, (UInt32)t16Vars.uw16);
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMStoreui16(UWord32 uw32CurrIndex, UInt16 ui16Data)
{
	// Var for converting
	t16BitVars t16Vars;
	
	t16Vars.ui16 = ui16Data;
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t16Vars.uw16);
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, (UInt32)t16Vars.uw16);
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMStorew16(UWord32 uw32CurrIndex, Word16 w16Data)
{
	// Var for converting
	t16BitVars t16Vars;
	
	t16Vars.w16 = w16Data;
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t16Vars.uw16);
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, (UInt32)t16Vars.uw16);
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMStoref(UWord32 uw32CurrIndex, float fData)
{
	// Var for converting
	t32BitVars t32Vars;
	
	t32Vars.f = fData;	
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t32Vars.uwords.uw16[0]);
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32Vars.uwords.uw16[0]);
	
	uw32CurrIndex++;
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t32Vars.uwords.uw16[1]);
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32Vars.uwords.uw16[1]);
	
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMStoreACC32(UWord32 uw32CurrIndex, acc32_t a32Data)
{
	// Var for converting
	t32BitVars t32Vars;
	
	t32Vars.a32 = a32Data;	
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t32Vars.uwords.uw16[0]);
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32Vars.uwords.uw16[0]);
	
	uw32CurrIndex++;
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t32Vars.uwords.uw16[1]);
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32Vars.uwords.uw16[1]);
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMStoreUW32(UWord32 uw32CurrIndex, UWord32 uw32Data)
{
	// Var for converting
	t32BitVars t32Vars;
	
	t32Vars.uw32 = uw32Data;	
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t32Vars.uwords.uw16[0]);
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32Vars.uwords.uw16[0]);
	
	uw32CurrIndex++;
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t32Vars.uwords.uw16[1]);
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32Vars.uwords.uw16[1]);
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMReadi8(UWord32 uw32CurrIndex, Int8 *i8Data)
{
	// Var for converting
	t16BitVars t16Vars;
	
	EepromReadWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, &t16Vars.uw16);
	
	*i8Data = t16Vars.ints.i8[0];
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMReadui8(UWord32 uw32CurrIndex, UInt8 *ui8Data)
{
	// Var for converting
	t16BitVars t16Vars;
	
	EepromReadWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, &t16Vars.uw16);
	
	*ui8Data = t16Vars.bytes.ui8[0];
	
	return uw32CurrIndex + 1;
}

// Load 16 bit signed integer to address and return address + 1
UWord32 EEPROMReadi16(UWord32 uw32CurrIndex, Int16 *i16Data)
{
	// Var for converting
	t16BitVars t16Vars;
	
	EepromReadWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, &t16Vars.uw16);
	
	*i16Data = t16Vars.i16;
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMReadsi16(UWord32 uw32CurrIndex, int16_t *i16Data)
{
	// Var for converting
	t16BitVars t16Vars;
	
	EepromReadWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, &t16Vars.uw16);
	
	*i16Data = t16Vars.i16;
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMReadf16(UWord32 uw32CurrIndex, Frac16 *f16Data)
{
	// Var for converting
	t16BitVars t16Vars;
	
	EepromReadWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, &t16Vars.uw16);
	
	*f16Data = t16Vars.f16;
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMReadui16(UWord32 uw32CurrIndex, UInt16 *ui16Data)
{
	// Var for converting
	t16BitVars t16Vars;
	
	EepromReadWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, &t16Vars.uw16);
	
	*ui16Data = t16Vars.ui16;
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMReadw16(UWord32 uw32CurrIndex, Word16 *w16Data)
{
	// Var for converting
	t16BitVars t16Vars;
	
	EepromReadWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, &t16Vars.uw16);
	
	*w16Data = t16Vars.w16;
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMReadf(UWord32 uw32CurrIndex, float *fData)
{
	// Var for converting
	t32BitVars t32Vars;
	
	EepromReadWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, &t32Vars.uwords.uw16[0]);
	uw32CurrIndex++;
	EepromReadWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, &t32Vars.uwords.uw16[1]);
	
	*fData = t32Vars.f;	
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMReadACC32(UWord32 uw32CurrIndex, acc32_t *a32Data)
{
	// Var for converting
	t32BitVars t32Vars;
	
	EepromReadWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, &t32Vars.uwords.uw16[0]);
	uw32CurrIndex++;
	EepromReadWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, &t32Vars.uwords.uw16[1]);
	
	*a32Data = t32Vars.a32;	
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMReadUW32(UWord32 uw32CurrIndex, UWord32 *uw32Data)
{
	// Var for converting
	t32BitVars t32Vars;
	
	EepromReadWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, &t32Vars.uwords.uw16[0]);
	uw32CurrIndex++;
	EepromReadWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, &t32Vars.uwords.uw16[1]);
	
	*uw32Data = t32Vars.uw32;	
	
	return uw32CurrIndex + 1;
}

Int16 checkEEPROMCRC()
{
	UInt16 ui16Index = 0;
	UInt16 ui16Data = 0;
	UWord32 uw32Index = 0;
	UWord32 uw32CRC = 0;
	UWord32 i = 0;
	
	// Setup CRC
	ioctl(CRC, CRC_SET_WRITE_AS_SEED, CRC_ENABLE);	
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, 0xffffffff);	
	ioctl(CRC, CRC_SET_WRITE_AS_SEED, CRC_DISABLE);
	
	EEPROMReadUW32(1, &uw32CRC);
	EEPROMReadui16(3, &ui16Index);
	
	for(i=4; i<ui16Index; i++)
	{
		uw32Index = (UWord32)i;
		uw32Index = EEPROMReadui16(uw32Index, &ui16Data);
		ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, ui16Data);	
		if(ioctl(CRC, CRC_READ_CRC_REG, NULL) == uw32CRC)
		{
			return 0;
		}	
		
	}


	{
		return -1;
	}
}

Int16 StoreEEPROM()
{
	// Index
	UWord32 uw32Index = 0;
	UWord32 uw32CRC = 0;
	// Setup CRC
	ioctl(CRC, CRC_SET_WRITE_AS_SEED, CRC_ENABLE);	
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, 0xffffffff);	
	ioctl(CRC, CRC_SET_WRITE_AS_SEED, CRC_DISABLE);
	
	
	
	// Store - EEPROM written
	uw32Index = EEPROMStorei16(uw32Index, 1);
	
	// Leave two spaces for CRC, one for max index. Start index = 4
	uw32Index += 3;
	//******************************************
	// Motor ID
	//******************************************	
	uw32Index = EEPROMStoreui8(uw32Index, COMMDataStruct.REGS.ui8ID); //1
	//******************************************
	// Calibration
	//******************************************
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16MotorPolePairs); //2
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16PolePairArray[0]); //3
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16PolePairArray[1]); //4
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16PolePairArray[2]); //5
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16PolePairArray[3]); //6
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16PolePairArray[4]); //7
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16PolePairArray[5]); //8
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16PolePairArray[6]); //9
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16PolePairArray[7]); //10
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16MaxSensorIndex); //11
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16MinSensorIndex); //12
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16CalibrationState); //13
	//******************************************
	// Regulators
	//******************************************
	// D, Q regulators
	// D
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.REGULATORS.mudtControllerParamId.a32PGain); //14
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.REGULATORS.mudtControllerParamId.a32IGain); //15
	//SYSTEM.REGULATORS.mudtControllerParamId.a32PGain = 0;
	//SYSTEM.REGULATORS.mudtControllerParamId.a32IGain = 0;		
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamId.f16UpperLim); //16
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamId.f16LowerLim); //17
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.REGULATORS.mudtControllerParamIq.a32PGain); //18
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.REGULATORS.mudtControllerParamIq.a32IGain); //19
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamIq.f16UpperLim); //20
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamIq.f16LowerLim); //21
	// W regulator
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.REGULATORS.mudtControllerParamW.a32PGain); //22
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.REGULATORS.mudtControllerParamW.a32IGain); //23
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamW.f16UpperLim); //24
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamW.f16LowerLim); //25
	uw32Index = EEPROMStoreui16(uw32Index, SYSTEM.REGULATORS.ui16SpeedRegInterval); //26
	//******************************************
	// Position
	//******************************************
	// Init for BEMF observer
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.POSITION.acBemfObsrvDQ.sCtrl.a32PGain); //27
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.POSITION.acBemfObsrvDQ.sCtrl.a32IGain); //28
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.POSITION.acBemfObsrvDQ.a32IGain); //29
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.POSITION.acBemfObsrvDQ.a32UGain); //30
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.POSITION.acBemfObsrvDQ.a32WIGain); //31
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.POSITION.acBemfObsrvDQ.a32EGain); //32
	// Tracking observer
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.POSITION.acToPos.f16PGain); //33
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.POSITION.acToPos.i16PGainSh); //34
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.POSITION.acToPos.f16IGain); //35
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.POSITION.acToPos.i16IGainSh); //36
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.POSITION.acToPos.f16ThGain); //37
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.POSITION.acToPos.i16ThGainSh); //38
	// Other init - once
	// Position offset
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.POSITION.i16SensorIndexOffset); //39
	// Phase delay from speed
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.POSITION.i16SensorIndexPhaseDelay); //40
	// Speed MA filter
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.POSITION.f16ManualAngleIncrease); //41
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.POSITION.f16AddedAngleOffset); //42
	uw32Index = EEPROMStoref(uw32Index, SYSTEM.POSITION.fOffsetCalcFactor); //43
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.POSITION.f16AngleOffset); //44
	//******************************************
	// Sensorless
	//******************************************
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.SENSORLESS.f16MinSpeed); //45
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.SENSORLESS.f16MaxObserverError); //46
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.SENSORLESS.f16AlignCurrent); //47
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.SENSORLESS.f16StartCurrent); //48
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.SENSORLESS.i16AlignTime); //49
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.SENSORLESS.f16StartSpeed); //50
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.SENSORLESS.f16StartTorque); //51
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.SENSORLESS.f16AngleManualError); //52
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.SENSORLESS.f16BEMFErrorPart); //53
	uw32Index = EEPROMStoreui8(uw32Index, SYSTEM.SENSORLESS.ui8MaxBemfObserverErrorCount); //54
	//******************************************
	// Ramps
	//******************************************
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.RAMPS.Ramp16_AlignCurrent.f16RampUp); //55
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.RAMPS.Ramp16_AlignCurrent.f16RampDown); //56
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.RAMPS.Ramp16_Speed.f16RampUp); //57
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.RAMPS.Ramp16_Speed.f16RampDown); //58
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.RAMPS.Ramp16_Torque.f16RampUp); //59
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.RAMPS.Ramp16_Torque.f16RampDown); //60
	//******************************************
	// ADC
	//******************************************
	// Angle filter
	// DC link MA filter
	// Temperature MA filter
	//******************************************
	// MCTRL
	//******************************************
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.MCTRL.f16TorqueFactor); //61
	//******************************************
	// PWM
	//******************************************
	// PWM input variables
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMfullThrottle); //62
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMoffThrottle); //63
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMinThrottle); //64
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMThrottleDifference); //65
	// PWM input parameters
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMInMiddleValue); //66
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMInHighValRef); //67
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMInLowValRef); //68
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMInMeasureTime); //69
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMInOffZone); //70
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMFilterSize); //71
	//******************************************
	// SI values
	//******************************************	
	uw32Index = EEPROMStoref(uw32Index, SYSTEM.SIVALUES.fIInFiltDiv); //72
	//******************************************
	// Measure params
	//******************************************
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.MEASUREPARAMS.f16SetpointMulti); //73
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.MEASUREPARAMS.f16LphaIset); //74
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.MEASUREPARAMS.i16TotalMeasurements); //75
	//******************************************
	// CAN
	//******************************************
	uw32Index = EEPROMStoreui16(uw32Index, SYSTEM.CAN.ui16CANInfoInterval); //76
	uw32Index = EEPROMStoreui16(uw32Index, SYSTEM.CAN.ui16CANStatusInterval); //77
	//******************************************
	// Other
	//******************************************
	// Comm data struct
	// Unit registers
	uw32Index = EEPROMStoreui8(uw32Index, COMMDataStruct.REGS.ui8ID); //78
	uw32Index = EEPROMStoreui16(uw32Index, COMMDataStruct.REGS.ui16ModelNumber); //79
	uw32Index = EEPROMStoreui8(uw32Index, COMMDataStruct.REGS.ui8FirmwareVersion); //80
	uw32Index = EEPROMStoreui8(uw32Index, COMMDataStruct.REGS.ui8BaudRate); //81
	uw32Index = EEPROMStoreui8(uw32Index, COMMDataStruct.REGS.ui8ReturnDelayTime); //82
	// Park position
	uw32Index = EEPROMStorei16(uw32Index, COMMDataStruct.REGS.i16ParkPosition); //83
	uw32Index = EEPROMStoreui16(uw32Index, COMMDataStruct.ui16RXCommTimeout); //84
	uw32Index = EEPROMStoreui16(uw32Index, COMMDataStruct.ui16RXTimeoutCounter); //85
	uw32Index = EEPROMStoreui16(uw32Index, COMMDataStruct.ui16TXCommTimeout); //86
	uw32Index = EEPROMStoreui16(uw32Index, COMMDataStruct.ui16TXTimeoutCounter); //87
	uw32Index = EEPROMStoreui16(uw32Index, COMMDataStruct.ui16ReadOnlyLow); //88
	uw32Index = EEPROMStoreui16(uw32Index, COMMDataStruct.ui16ReadOnlyHigh); //89
	uw32Index = EEPROMStorei16(uw32Index, COMMDataStruct.REGS.i16PWMMax); //90
	uw32Index = EEPROMStorei16(uw32Index, COMMDataStruct.REGS.i16PWMMin); //91
	uw32Index = EEPROMStorei16(uw32Index, COMMDataStruct.REGS.i16MaxRPM); //92
	uw32Index = EEPROMStorei16(uw32Index, COMMDataStruct.REGS.i16MinRPM); //93
	uw32Index = EEPROMStorei16(uw32Index, COMMDataStruct.REGS.i16CurrentPWM); //94
	uw32Index = EEPROMStorei16(uw32Index, COMMDataStruct.REGS.i16ZeroSpeedPWM); //95
	
	// Store CRC
	EEPROMStoreUW32(1, ioctl(CRC, CRC_READ_CRC_REG, NULL));
	
	// Store max index
	EEPROMStoreui16(3, (UInt16)uw32Index); //85
	
	return (Int16)uw32Index;
}

Int16 LoadEEPROM()
{
	// Index
	UWord32 uw32Index = 1;
	//******************************************
	// Motor ID
	//******************************************	
	uw32Index = EEPROMReadui8(uw32Index, &COMMDataStruct.REGS.ui8ID); //1
	//******************************************
	// Calibration
	//******************************************
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16MotorPolePairs); //2
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16PolePairArray[0]); //3
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16PolePairArray[1]); //4
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16PolePairArray[2]); //5
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16PolePairArray[3]); //6
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16PolePairArray[4]); //7
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16PolePairArray[5]); //8
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16PolePairArray[6]); //9
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16PolePairArray[7]); //10
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16MaxSensorIndex); //11
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16MinSensorIndex); //12
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16CalibrationState); //13
	//******************************************
	// Regulators
	//******************************************
	// D, Q regulators
	// D
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamId.a32PGain); //14
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamId.a32IGain); //15
	//SYSTEM.REGULATORS.mudtControllerParamId.a32PGain = 0;
	//SYSTEM.REGULATORS.mudtControllerParamId.a32IGain = 0;		
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamId.f16UpperLim); //16
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamId.f16LowerLim); //17
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamIq.a32PGain); //18
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamIq.a32IGain); //19
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamIq.f16UpperLim); //20
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamIq.f16LowerLim); //21
	// W regulator
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamW.a32PGain); //22
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamW.a32IGain); //23
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamW.f16UpperLim); //24
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamW.f16LowerLim); //25
	uw32Index = EEPROMReadui16(uw32Index, &SYSTEM.REGULATORS.ui16SpeedRegInterval); //26
	//******************************************
	// Position
	//******************************************
	// Init for BEMF observer
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.POSITION.acBemfObsrvDQ.sCtrl.a32PGain); //27
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.POSITION.acBemfObsrvDQ.sCtrl.a32IGain); //28
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.POSITION.acBemfObsrvDQ.a32IGain); //29
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.POSITION.acBemfObsrvDQ.a32UGain); //30
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.POSITION.acBemfObsrvDQ.a32WIGain); //31
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.POSITION.acBemfObsrvDQ.a32EGain); //32
	// Tracking observer
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.POSITION.acToPos.f16PGain); //33
	uw32Index = EEPROMReadsi16(uw32Index, &SYSTEM.POSITION.acToPos.i16PGainSh); //34
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.POSITION.acToPos.f16IGain); //35
	uw32Index = EEPROMReadsi16(uw32Index, &SYSTEM.POSITION.acToPos.i16IGainSh); //36
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.POSITION.acToPos.f16ThGain); //37
	uw32Index = EEPROMReadsi16(uw32Index, &SYSTEM.POSITION.acToPos.i16ThGainSh); //38
	// Other init - once
	// Position offset
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.POSITION.i16SensorIndexOffset); //39
	// Phase delay from speed
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.POSITION.i16SensorIndexPhaseDelay); //40
	// Speed MA filter
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.POSITION.f16ManualAngleIncrease); //41
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.POSITION.f16AddedAngleOffset); //42
	uw32Index = EEPROMReadf(uw32Index, &SYSTEM.POSITION.fOffsetCalcFactor); //43
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.POSITION.f16AngleOffset); //44
	//******************************************
	// Sensorless
	//******************************************
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.SENSORLESS.f16MinSpeed); //45
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.SENSORLESS.f16MaxObserverError); //46
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.SENSORLESS.f16AlignCurrent); //47
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.SENSORLESS.f16StartCurrent); //48
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.SENSORLESS.i16AlignTime); //49
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.SENSORLESS.f16StartSpeed); //50
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.SENSORLESS.f16StartTorque); //51
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.SENSORLESS.f16AngleManualError); //52
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.SENSORLESS.f16BEMFErrorPart); //53
	uw32Index = EEPROMReadui8(uw32Index, &SYSTEM.SENSORLESS.ui8MaxBemfObserverErrorCount); //54
	//******************************************
	// Ramps
	//******************************************
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.RAMPS.Ramp16_AlignCurrent.f16RampUp); //55
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.RAMPS.Ramp16_AlignCurrent.f16RampDown); //56
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.RAMPS.Ramp16_Speed.f16RampUp); //57
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.RAMPS.Ramp16_Speed.f16RampDown); //58
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.RAMPS.Ramp16_Torque.f16RampUp); //59
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.RAMPS.Ramp16_Torque.f16RampDown); //60
	//******************************************
	// ADC
	//******************************************
	// Angle filter
	// DC link MA filter
	// Temperature MA filter
	//******************************************
	// MCTRL
	//******************************************
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.MCTRL.f16TorqueFactor); //61
	//******************************************
	// PWM
	//******************************************
	// PWM input variables
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMfullThrottle); //62
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMoffThrottle); //63
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMinThrottle); //64
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMThrottleDifference); //65
	// PWM input parameters
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMInMiddleValue); //66
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMInHighValRef); //67
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMInLowValRef); //68
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMInMeasureTime); //69
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMInOffZone); //70
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMFilterSize); //71
	//******************************************
	// SI values
	//******************************************	
	uw32Index = EEPROMReadf(uw32Index, &SYSTEM.SIVALUES.fIInFiltDiv); //72
	//******************************************
	// Measure params
	//******************************************
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.MEASUREPARAMS.f16SetpointMulti); //73
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.MEASUREPARAMS.f16LphaIset); //74
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.MEASUREPARAMS.i16TotalMeasurements); //75
	//******************************************
	// CAN
	//******************************************
	uw32Index = EEPROMReadui16(uw32Index, &SYSTEM.CAN.ui16CANInfoInterval); //76
	uw32Index = EEPROMReadui16(uw32Index, &SYSTEM.CAN.ui16CANStatusInterval); //77
	//******************************************
	// Other
	//******************************************
	// Comm data struct
	// Unit registers
	uw32Index = EEPROMReadui8(uw32Index, &COMMDataStruct.REGS.ui8ID); //78
	uw32Index = EEPROMReadui16(uw32Index, &COMMDataStruct.REGS.ui16ModelNumber); //79
	uw32Index = EEPROMReadui8(uw32Index, &COMMDataStruct.REGS.ui8FirmwareVersion); //80
	uw32Index = EEPROMReadui8(uw32Index, &COMMDataStruct.REGS.ui8BaudRate); //81
	uw32Index = EEPROMReadui8(uw32Index, &COMMDataStruct.REGS.ui8ReturnDelayTime); //82
	// Park position
	uw32Index = EEPROMReadi16(uw32Index, &COMMDataStruct.REGS.i16ParkPosition); //83
	uw32Index = EEPROMReadui16(uw32Index, &COMMDataStruct.ui16RXCommTimeout); //84
	uw32Index = EEPROMReadui16(uw32Index, &COMMDataStruct.ui16RXTimeoutCounter); //85
	uw32Index = EEPROMReadui16(uw32Index, &COMMDataStruct.ui16TXCommTimeout); //86
	uw32Index = EEPROMReadui16(uw32Index, &COMMDataStruct.ui16TXTimeoutCounter); //87
	uw32Index = EEPROMReadui16(uw32Index, &COMMDataStruct.ui16ReadOnlyLow); //88
	uw32Index = EEPROMReadui16(uw32Index, &COMMDataStruct.ui16ReadOnlyHigh); //89
	uw32Index = EEPROMReadi16(uw32Index, &COMMDataStruct.REGS.i16PWMMax); //90
	uw32Index = EEPROMReadi16(uw32Index, &COMMDataStruct.REGS.i16PWMMin); //91
	uw32Index = EEPROMReadi16(uw32Index, &COMMDataStruct.REGS.i16MaxRPM); //92
	uw32Index = EEPROMReadi16(uw32Index, &COMMDataStruct.REGS.i16MinRPM); //93
	uw32Index = EEPROMReadi16(uw32Index, &COMMDataStruct.REGS.i16CurrentPWM); //94
	uw32Index = EEPROMReadi16(uw32Index, &COMMDataStruct.REGS.i16ZeroSpeedPWM); //95
	// If there is calibration data
	if(0 != SYSTEM.CALIBRATION.i16PolePairArray[0])
	{
		// Calculate calibration values
		CalculateCalibrationData();	
		// Mark calibrated
		SYSTEM_CALIBRATED = 1;	
	}	

	return (Int16)uw32Index;
}

Int16 CheckEEPROM()
{
	Int16 NOK = 0;
	Int8 i8Temp = 0;
	UInt8 ui8Temp = 0;
	Int16 i16Temp = 0;
	UInt16 ui16Temp = 0;
	Frac16 f16Temp = 0;
	Word16 w16Temp = 0;
	float fTemp = 0;
	acc32_t a32Temp = 0;
	
	
	// Index
	UWord32 uw32Index = 1;	//******************************************
	// Motor ID
	//******************************************	
	uw32Index = EEPROMReadui8(uw32Index, &ui8Temp);
	if(ui8Temp != COMMDataStruct.REGS.ui8ID) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //1
	//******************************************
	// Calibration
	//******************************************
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16MotorPolePairs) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //2
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16PolePairArray[0]) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //3
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16PolePairArray[1]) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //4
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16PolePairArray[2]) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //5
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16PolePairArray[3]) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //6
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16PolePairArray[4]) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //7
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16PolePairArray[5]) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //8
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16PolePairArray[6]) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //9
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16PolePairArray[7]) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //10
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16MaxSensorIndex) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //11
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16MinSensorIndex) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //12
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16CalibrationState) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //13
	//******************************************
	// Regulators
	//******************************************
	// D, Q regulators
	// D
	uw32Index = EEPROMReadACC32(uw32Index, &a32Temp);
	if(a32Temp != SYSTEM.REGULATORS.mudtControllerParamId.a32PGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //14
	uw32Index = EEPROMReadACC32(uw32Index, &a32Temp);
	if(a32Temp != SYSTEM.REGULATORS.mudtControllerParamId.a32IGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //15
	//SYSTEM.REGULATORS.mudtControllerParamId.a32PGain = 0;
	//SYSTEM.REGULATORS.mudtControllerParamId.a32IGain = 0;		
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamId.f16UpperLim) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //16
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamId.f16LowerLim) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //17
	uw32Index = EEPROMReadACC32(uw32Index, &a32Temp);
	if(a32Temp != SYSTEM.REGULATORS.mudtControllerParamIq.a32PGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //18
	uw32Index = EEPROMReadACC32(uw32Index, &a32Temp);
	if(a32Temp != SYSTEM.REGULATORS.mudtControllerParamIq.a32IGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //19
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamIq.f16UpperLim) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //20
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamIq.f16LowerLim) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //21
	// W regulator
	uw32Index = EEPROMReadACC32(uw32Index, &a32Temp);
	if(a32Temp != SYSTEM.REGULATORS.mudtControllerParamW.a32PGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //22
	uw32Index = EEPROMReadACC32(uw32Index, &a32Temp);
	if(a32Temp != SYSTEM.REGULATORS.mudtControllerParamW.a32IGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //23
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamW.f16UpperLim) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //24
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamW.f16LowerLim) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //25
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != SYSTEM.REGULATORS.ui16SpeedRegInterval) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //26
	//******************************************
	// Position
	//******************************************
	// Init for BEMF observer
	uw32Index = EEPROMReadACC32(uw32Index, &a32Temp);
	if(a32Temp != SYSTEM.POSITION.acBemfObsrvDQ.sCtrl.a32PGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //27
	uw32Index = EEPROMReadACC32(uw32Index, &a32Temp);
	if(a32Temp != SYSTEM.POSITION.acBemfObsrvDQ.sCtrl.a32IGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //28
	uw32Index = EEPROMReadACC32(uw32Index, &a32Temp);
	if(a32Temp != SYSTEM.POSITION.acBemfObsrvDQ.a32IGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //29
	uw32Index = EEPROMReadACC32(uw32Index, &a32Temp);
	if(a32Temp != SYSTEM.POSITION.acBemfObsrvDQ.a32UGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //30
	uw32Index = EEPROMReadACC32(uw32Index, &a32Temp);
	if(a32Temp != SYSTEM.POSITION.acBemfObsrvDQ.a32WIGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //31
	uw32Index = EEPROMReadACC32(uw32Index, &a32Temp);
	if(a32Temp != SYSTEM.POSITION.acBemfObsrvDQ.a32EGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //32
	// Tracking observer
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.POSITION.acToPos.f16PGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //33
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.POSITION.acToPos.i16PGainSh) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //34
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.POSITION.acToPos.f16IGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //35
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.POSITION.acToPos.i16IGainSh) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //36
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.POSITION.acToPos.f16ThGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //37
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.POSITION.acToPos.i16ThGainSh) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //38
	// Other init - once
	// Position offset
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.POSITION.i16SensorIndexOffset) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //39
	// Phase delay from speed
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.POSITION.i16SensorIndexPhaseDelay) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //40
	// Speed MA filter
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.POSITION.f16ManualAngleIncrease) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //41
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.POSITION.f16AddedAngleOffset) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //42
	uw32Index = EEPROMReadf(uw32Index, &fTemp);
	if(fTemp != SYSTEM.POSITION.fOffsetCalcFactor) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //43
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.POSITION.f16AngleOffset) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //44
	//******************************************
	// Sensorless
	//******************************************
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.SENSORLESS.f16MinSpeed) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //45
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.SENSORLESS.f16MaxObserverError) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //46
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.SENSORLESS.f16AlignCurrent) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //47
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.SENSORLESS.f16StartCurrent) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //48
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.SENSORLESS.i16AlignTime) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //49
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.SENSORLESS.f16StartSpeed) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //50
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.SENSORLESS.f16StartTorque) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //51
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.SENSORLESS.f16AngleManualError) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //52
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.SENSORLESS.f16BEMFErrorPart) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //53
	uw32Index = EEPROMReadui8(uw32Index, &ui8Temp);
	if(ui8Temp != SYSTEM.SENSORLESS.ui8MaxBemfObserverErrorCount) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //54
	//******************************************
	// Ramps
	//******************************************
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.RAMPS.Ramp16_AlignCurrent.f16RampUp) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //55
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.RAMPS.Ramp16_AlignCurrent.f16RampDown) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //56
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.RAMPS.Ramp16_Speed.f16RampUp) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //57
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.RAMPS.Ramp16_Speed.f16RampDown) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //58
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.RAMPS.Ramp16_Torque.f16RampUp) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //59
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.RAMPS.Ramp16_Torque.f16RampDown) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //60
	//******************************************
	// ADC
	//******************************************
	// Angle filter
	// DC link MA filter
	// Temperature MA filter
	//******************************************
	// MCTRL
	//******************************************
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.MCTRL.f16TorqueFactor) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //61
	//******************************************
	// PWM
	//******************************************
	// PWM input variables
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMfullThrottle) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //62
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMoffThrottle) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //63
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMinThrottle) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //64
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMThrottleDifference) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //65
	// PWM input parameters
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMInMiddleValue) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //66
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMInHighValRef) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //67
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMInLowValRef) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //68
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMInMeasureTime) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //69
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMInOffZone) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //70
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMFilterSize) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //71
	//******************************************
	// SI values
	//******************************************	
	uw32Index = EEPROMReadf(uw32Index, &fTemp);
	if(fTemp != SYSTEM.SIVALUES.fIInFiltDiv) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //72
	//******************************************
	// Measure params
	//******************************************
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.MEASUREPARAMS.f16SetpointMulti) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //73
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.MEASUREPARAMS.f16LphaIset) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //74
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.MEASUREPARAMS.i16TotalMeasurements) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //75
	//******************************************
	// CAN
	//******************************************
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != SYSTEM.CAN.ui16CANInfoInterval) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //76
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != SYSTEM.CAN.ui16CANStatusInterval) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //77
	//******************************************
	// Other
	//******************************************
	// Comm data struct
	// Unit registers
	uw32Index = EEPROMReadui8(uw32Index, &ui8Temp);
	if(ui8Temp != COMMDataStruct.REGS.ui8ID) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //78
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != COMMDataStruct.REGS.ui16ModelNumber) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //79
	uw32Index = EEPROMReadui8(uw32Index, &ui8Temp);
	if(ui8Temp != COMMDataStruct.REGS.ui8FirmwareVersion) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //80
	uw32Index = EEPROMReadui8(uw32Index, &ui8Temp);
	if(ui8Temp != COMMDataStruct.REGS.ui8BaudRate) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //81
	uw32Index = EEPROMReadui8(uw32Index, &ui8Temp);
	if(ui8Temp != COMMDataStruct.REGS.ui8ReturnDelayTime) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //82
	// Park position
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != COMMDataStruct.REGS.i16ParkPosition) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //83
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != COMMDataStruct.ui16RXCommTimeout) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //84
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != COMMDataStruct.ui16RXTimeoutCounter) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //85
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != COMMDataStruct.ui16TXCommTimeout) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //86
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != COMMDataStruct.ui16TXTimeoutCounter) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //87
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != COMMDataStruct.ui16ReadOnlyLow) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //88
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != COMMDataStruct.ui16ReadOnlyHigh) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //89
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != COMMDataStruct.REGS.i16PWMMax) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //90
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != COMMDataStruct.REGS.i16PWMMin) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //91
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != COMMDataStruct.REGS.i16MaxRPM) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //92
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != COMMDataStruct.REGS.i16MinRPM) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //93
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != COMMDataStruct.REGS.i16CurrentPWM) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //94
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != COMMDataStruct.REGS.i16ZeroSpeedPWM) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //95
	return NOK;
}

UWord32 CalculateDataCRC()
{
	// Setup var
	t32BitVars t32bit;
	// Setup CRC
	ioctl(CRC, CRC_SET_WRITE_AS_SEED, CRC_ENABLE);	
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, 0xffffffff);	
	ioctl(CRC, CRC_SET_WRITE_AS_SEED, CRC_DISABLE);
	
	// Feed CRC
	t32bit.uw32 = 0;
	t32bit.bytes.ui8[0] = COMMDataStruct.REGS.ui8ID;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.CALIBRATION.i16MotorPolePairs;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.CALIBRATION.i16PolePairArray[0];
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.CALIBRATION.i16PolePairArray[1];
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.CALIBRATION.i16PolePairArray[2];
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.CALIBRATION.i16PolePairArray[3];
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.CALIBRATION.i16PolePairArray[4];
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.CALIBRATION.i16PolePairArray[5];
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.CALIBRATION.i16PolePairArray[6];
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.CALIBRATION.i16PolePairArray[7];
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.CALIBRATION.i16MaxSensorIndex;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.CALIBRATION.i16MinSensorIndex;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.CALIBRATION.i16CalibrationState;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.a32 = SYSTEM.REGULATORS.mudtControllerParamId.a32PGain;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.a32 = SYSTEM.REGULATORS.mudtControllerParamId.a32IGain;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.REGULATORS.mudtControllerParamId.f16UpperLim;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.REGULATORS.mudtControllerParamId.f16LowerLim;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.a32 = SYSTEM.REGULATORS.mudtControllerParamIq.a32PGain;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.a32 = SYSTEM.REGULATORS.mudtControllerParamIq.a32IGain;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.REGULATORS.mudtControllerParamIq.f16UpperLim;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.REGULATORS.mudtControllerParamIq.f16LowerLim;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.a32 = SYSTEM.REGULATORS.mudtControllerParamW.a32PGain;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.a32 = SYSTEM.REGULATORS.mudtControllerParamW.a32IGain;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.REGULATORS.mudtControllerParamW.f16UpperLim;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.REGULATORS.mudtControllerParamW.f16LowerLim;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.uwords.uw16[0] = SYSTEM.REGULATORS.ui16SpeedRegInterval;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.a32 = SYSTEM.POSITION.acBemfObsrvDQ.sCtrl.a32PGain;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.a32 = SYSTEM.POSITION.acBemfObsrvDQ.sCtrl.a32IGain;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.a32 = SYSTEM.POSITION.acBemfObsrvDQ.a32IGain;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.a32 = SYSTEM.POSITION.acBemfObsrvDQ.a32UGain;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.a32 = SYSTEM.POSITION.acBemfObsrvDQ.a32WIGain;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.a32 = SYSTEM.POSITION.acBemfObsrvDQ.a32EGain;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.POSITION.acToPos.f16PGain;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.POSITION.acToPos.i16PGainSh;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.POSITION.acToPos.f16IGain;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.POSITION.acToPos.i16IGainSh;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.POSITION.acToPos.f16ThGain;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.POSITION.acToPos.i16ThGainSh;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.POSITION.i16SensorIndexOffset;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.POSITION.i16SensorIndexPhaseDelay;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.POSITION.f16ManualAngleIncrease;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.POSITION.f16AddedAngleOffset;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.f = SYSTEM.POSITION.fOffsetCalcFactor;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.POSITION.f16AngleOffset;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.SENSORLESS.f16MinSpeed;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.SENSORLESS.f16MaxObserverError;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.SENSORLESS.f16AlignCurrent;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.SENSORLESS.f16StartCurrent;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.SENSORLESS.i16AlignTime;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.SENSORLESS.f16StartSpeed;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.SENSORLESS.f16StartTorque;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.SENSORLESS.f16AngleManualError;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.SENSORLESS.f16BEMFErrorPart;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.bytes.ui8[0] = SYSTEM.SENSORLESS.ui8MaxBemfObserverErrorCount;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.RAMPS.Ramp16_AlignCurrent.f16RampUp;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.RAMPS.Ramp16_AlignCurrent.f16RampDown;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.RAMPS.Ramp16_Speed.f16RampUp;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.RAMPS.Ramp16_Speed.f16RampDown;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.RAMPS.Ramp16_Torque.f16RampUp;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.RAMPS.Ramp16_Torque.f16RampDown;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.MCTRL.f16TorqueFactor;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.PWMIN.i16PWMfullThrottle;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.PWMIN.i16PWMoffThrottle;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.PWMIN.i16PWMinThrottle;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.PWMIN.i16PWMThrottleDifference;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.PWMIN.i16PWMInMiddleValue;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.PWMIN.i16PWMInHighValRef;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.PWMIN.i16PWMInLowValRef;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.PWMIN.i16PWMInMeasureTime;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.PWMIN.i16PWMInOffZone;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.PWMIN.i16PWMFilterSize;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.f = SYSTEM.SIVALUES.fIInFiltDiv;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.MEASUREPARAMS.f16SetpointMulti;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.frac16.f16[0] = SYSTEM.MEASUREPARAMS.f16LphaIset;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = SYSTEM.MEASUREPARAMS.i16TotalMeasurements;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.uwords.uw16[0] = SYSTEM.CAN.ui16CANInfoInterval;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.uwords.uw16[0] = SYSTEM.CAN.ui16CANStatusInterval;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.bytes.ui8[0] = COMMDataStruct.REGS.ui8ID;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.uwords.uw16[0] = COMMDataStruct.REGS.ui16ModelNumber;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.bytes.ui8[0] = COMMDataStruct.REGS.ui8FirmwareVersion;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.bytes.ui8[0] = COMMDataStruct.REGS.ui8BaudRate;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.bytes.ui8[0] = COMMDataStruct.REGS.ui8ReturnDelayTime;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = COMMDataStruct.REGS.i16ParkPosition;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.uwords.uw16[0] = COMMDataStruct.ui16RXCommTimeout;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.uwords.uw16[0] = COMMDataStruct.ui16RXTimeoutCounter;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.uwords.uw16[0] = COMMDataStruct.ui16TXCommTimeout;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.uwords.uw16[0] = COMMDataStruct.ui16TXTimeoutCounter;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.uwords.uw16[0] = COMMDataStruct.ui16ReadOnlyLow;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.uwords.uw16[0] = COMMDataStruct.ui16ReadOnlyHigh;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = COMMDataStruct.REGS.i16PWMMax;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = COMMDataStruct.REGS.i16PWMMin;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = COMMDataStruct.REGS.i16MaxRPM;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = COMMDataStruct.REGS.i16MinRPM;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = COMMDataStruct.REGS.i16CurrentPWM;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	t32bit.uw32 = 0;
	t32bit.words.i16[0] = COMMDataStruct.REGS.i16ZeroSpeedPWM;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32bit.uw32);

	return ioctl(CRC, CRC_READ_CRC_REG, NULL);
}
