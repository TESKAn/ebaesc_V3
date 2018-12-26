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
	//******************************************
	// Motor ID
	//******************************************	
	COMMDataStruct.REGS.ui8ID = M_ID;
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
		SYSTEM.REGULATORS.mudtControllerParamId.f16PropGain = D_KP_GAIN;
		SYSTEM.REGULATORS.mudtControllerParamId.f16IntegGain = D_KI_GAIN;
		SYSTEM.REGULATORS.mudtControllerParamId.i16PropGainShift = D_KP_SHIFT;
		SYSTEM.REGULATORS.mudtControllerParamId.i16IntegGainShift = D_KI_SHIFT;
		SYSTEM.REGULATORS.mudtControllerParamId.f16UpperLimit = FRAC16(0.95);
		SYSTEM.REGULATORS.mudtControllerParamId.f16LowerLimit = FRAC16(-0.95);
		
		// Q
		SYSTEM.REGULATORS.mudtControllerParamIq.f16PropGain = Q_KP_GAIN;
		SYSTEM.REGULATORS.mudtControllerParamIq.f16IntegGain = Q_KI_GAIN;
		SYSTEM.REGULATORS.mudtControllerParamIq.i16PropGainShift = Q_KP_SHIFT;
		SYSTEM.REGULATORS.mudtControllerParamIq.i16IntegGainShift = Q_KI_SHIFT;
		SYSTEM.REGULATORS.mudtControllerParamIq.f16UpperLimit = FRAC16(0.95);
		SYSTEM.REGULATORS.mudtControllerParamIq.f16LowerLimit = FRAC16(-0.95);
		
		// W regulator
		SYSTEM.REGULATORS.mudtControllerParamW.f16PropGain = SPEED_PI_PROP_GAIN;
		SYSTEM.REGULATORS.mudtControllerParamW.f16IntegGain = SPEED_PI_INTEG_GAIN;
		SYSTEM.REGULATORS.mudtControllerParamW.i16PropGainShift = SPEED_PI_PROP_SHIFT;
		SYSTEM.REGULATORS.mudtControllerParamW.i16IntegGainShift = SPEED_PI_INTEG_SHIFT;
		SYSTEM.REGULATORS.mudtControllerParamW.f16UpperLimit = SPEED_LOOP_HIGH_LIMIT;
		SYSTEM.REGULATORS.mudtControllerParamW.f16LowerLimit = SPEED_LOOP_LOW_LIMIT;	
		
		SYSTEM.REGULATORS.ui16SpeedRegInterval = SPEED_LOOP_CNTR;
	}
	SYSTEM.REGULATORS.mudtControllerParamId.f32IntegPartK_1 = FRAC32(0.0);
	SYSTEM.REGULATORS.mudtControllerParamIq.f32IntegPartK_1 = FRAC32(0.0);
	SYSTEM.REGULATORS.mudtControllerParamW.f32IntegPartK_1 = FRAC32(0.0);
	
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
		SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.f16PropGain= BEMF_DQ_KP_GAIN;
		SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.i16PropGainShift= BEMF_DQ_KP_SHIFT;
		SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.f16IntegGain= BEMF_DQ_KI_GAIN;
		SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.i16IntegGainShift = BEMF_DQ_KI_SHIFT;
		
		SYSTEM.POSITION.acBemfObsrvDQ.f16IGain = I_SCALE;
		SYSTEM.POSITION.acBemfObsrvDQ.f16UGain = U_SCALE;
		SYSTEM.POSITION.acBemfObsrvDQ.f16EGain = E_SCALE;
		SYSTEM.POSITION.acBemfObsrvDQ.f16WIGain = WI_SCALE;		
	}	
	SYSTEM.POSITION.acBemfObsrvDQ.udtIObsrv.f32D = FRAC32(0.0);
	SYSTEM.POSITION.acBemfObsrvDQ.udtIObsrv.f32Q = FRAC32(0.0);
	SYSTEM.POSITION.acBemfObsrvDQ.udtEObsrv.f32D = FRAC32(0.0);
	SYSTEM.POSITION.acBemfObsrvDQ.udtEObsrv.f32Q = FRAC32(0.0);
	SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.f32ID_1= FRAC32(0.0);
	SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.f32IQ_1= FRAC32(0.0);
	
	// Tracking observer
	if(0 != initial)
	{
		SYSTEM.POSITION.acToPos.f16PropGain= TO_KP_GAIN;
		SYSTEM.POSITION.acToPos.i16PropGainShift= TO_KP_SHIFT;
		SYSTEM.POSITION.acToPos.f16IntegGain= TO_KI_GAIN;
		SYSTEM.POSITION.acToPos.i16IntegGainShift= TO_KI_SHIFT;
		SYSTEM.POSITION.acToPos.f16ThGain= TO_THETA_GAIN;
		SYSTEM.POSITION.acToPos.i16ThGainShift = TO_THETA_SHIFT;	
	}
	
	// Other init - once
	if(0 != initial)
	{
		// Position offset
		SYSTEM.POSITION.i16SensorIndexOffset = 0;
		// Phase delay from speed
		SYSTEM.POSITION.i16SensorIndexPhaseDelay = 0;	
		// Speed MA filter
		SYSTEM.POSITION.FilterMA32Speed.w16N = 2;
		GDFLIB_FilterMA32Init(&SYSTEM.POSITION.FilterMA32Speed);	
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
		SYSTEM.ADC.FilterMA32SensorA.w16N = 2;
		GDFLIB_FilterMA32Init(&SYSTEM.ADC.FilterMA32SensorA);
		SYSTEM.ADC.FilterMA32SensorB.w16N = 2;
		GDFLIB_FilterMA32Init(&SYSTEM.ADC.FilterMA32SensorB);	
		
		// DC link MA filter
		SYSTEM.ADC.FilterMA32DCLink.w16N = 6;
		GDFLIB_FilterMA32Init(&SYSTEM.ADC.FilterMA32DCLink);
		
		// Temperature MA filter
		SYSTEM.ADC.FilterMA32Temperature.w16N = 6;
		GDFLIB_FilterMA32Init(&SYSTEM.ADC.FilterMA32Temperature);	
		
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

// Store vars to EEPROM
UWord32 EEPROMStorei8(UWord32 uw32CurrIndex, Int8 i8Data)
{
	// Var for converting
	t16BitVars t16Vars;
	
	t16Vars.ints.i8[0] = i8Data;
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t16Vars.uw16);
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMStoreui8(UWord32 uw32CurrIndex, UInt8 ui8Data)
{
	// Var for converting
	t16BitVars t16Vars;
	
	t16Vars.bytes.ui8[0] = ui8Data;
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t16Vars.uw16);
	
	return uw32CurrIndex + 1;
}

// Store 16 bit signed integer to address and return address + 1
UWord32 EEPROMStorei16(UWord32 uw32CurrIndex, Int16 i16Data)
{
	// Var for converting
	t16BitVars t16Vars;
	
	t16Vars.i16 = i16Data;
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t16Vars.uw16);
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMStoref16(UWord32 uw32CurrIndex, Frac16 f16Data)
{
	// Var for converting
	t16BitVars t16Vars;
	
	t16Vars.f16 = f16Data;
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t16Vars.uw16);
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMStoreui16(UWord32 uw32CurrIndex, UInt16 ui16Data)
{
	// Var for converting
	t16BitVars t16Vars;
	
	t16Vars.ui16 = ui16Data;
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t16Vars.uw16);
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMStorew16(UWord32 uw32CurrIndex, Word16 w16Data)
{
	// Var for converting
	t16BitVars t16Vars;
	
	t16Vars.w16 = w16Data;
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t16Vars.uw16);
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMStoref(UWord32 uw32CurrIndex, float fData)
{
	// Var for converting
	t32BitVars t32Vars;
	
	t32Vars.f = fData;	
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t32Vars.uwords.uw16[0]);
	uw32CurrIndex++;
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t32Vars.uwords.uw16[1]);
	
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

Int16 LoadEEPROM()
{
	// Index
	UWord32 uw32Index = 1;
	//******************************************
	// Motor ID
	//******************************************	
	//******************************************
	// Calibration
	//******************************************
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16MotorPolePairs);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16PolePairArray[0]);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16PolePairArray[1]);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16PolePairArray[2]);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16PolePairArray[3]);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16PolePairArray[4]);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16PolePairArray[5]);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16PolePairArray[6]);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16PolePairArray[7]);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16MaxSensorIndex);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16MinSensorIndex);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.CALIBRATION.i16CalibrationState);
	//******************************************
	// Regulators
	//******************************************
	// D, Q regulators
	// D
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamId.f16PropGain);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamId.f16IntegGain);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamId.i16PropGainShift);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamId.i16IntegGainShift);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamId.f16UpperLimit);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamId.f16LowerLimit);
	// Q
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamIq.f16PropGain);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamIq.f16IntegGain);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamIq.i16PropGainShift);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamIq.i16IntegGainShift);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamIq.f16UpperLimit);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamIq.f16LowerLimit);
	// W regulator
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamW.f16PropGain);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamW.f16IntegGain);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamW.i16PropGainShift);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamW.i16IntegGainShift);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamW.f16UpperLimit);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamW.f16LowerLimit);
	uw32Index = EEPROMReadui16(uw32Index, &SYSTEM.REGULATORS.ui16SpeedRegInterval);
	//******************************************
	// Position
	//******************************************
	// Init for BEMF observer
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.f16PropGain);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.i16PropGainShift);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.f16IntegGain);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.i16IntegGainShift);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.POSITION.acBemfObsrvDQ.f16IGain);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.POSITION.acBemfObsrvDQ.f16UGain);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.POSITION.acBemfObsrvDQ.f16EGain);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.POSITION.acBemfObsrvDQ.f16WIGain);
	// Tracking observer
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.POSITION.acToPos.f16PropGain);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.POSITION.acToPos.i16PropGainShift);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.POSITION.acToPos.f16IntegGain);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.POSITION.acToPos.i16IntegGainShift);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.POSITION.acToPos.f16ThGain);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.POSITION.acToPos.i16ThGainShift);
	// Speed MA filter
	uw32Index = EEPROMReadw16(uw32Index, &SYSTEM.POSITION.FilterMA32Speed.w16N);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.POSITION.f16ManualAngleIncrease);
	uw32Index = EEPROMReadf(uw32Index, &SYSTEM.POSITION.fOffsetCalcFactor);
	//******************************************
	// Sensorless
	//******************************************
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.SENSORLESS.f16MinSpeed);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.SENSORLESS.f16MaxObserverError);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.SENSORLESS.f16AlignCurrent);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.SENSORLESS.f16StartCurrent);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.SENSORLESS.i16AlignTime);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.SENSORLESS.f16StartSpeed);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.SENSORLESS.f16StartTorque);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.SENSORLESS.f16AngleManualError);
	uw32Index = EEPROMReadui8(uw32Index, &SYSTEM.SENSORLESS.ui8MaxBemfObserverErrorCount);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.SENSORLESS.f16MinSpeedHysteresis);
	//******************************************
	// Ramps
	//******************************************
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.RAMPS.Ramp16_AlignCurrent.f16RampUp);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.RAMPS.Ramp16_AlignCurrent.f16RampDown);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.RAMPS.Ramp16_Speed.f16RampUp);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.RAMPS.Ramp16_Speed.f16RampDown);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.RAMPS.Ramp16_Torque.f16RampUp);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.RAMPS.Ramp16_Torque.f16RampDown);
	//******************************************
	// ADC
	//******************************************
	// Angle filter
	uw32Index = EEPROMReadw16(uw32Index, &SYSTEM.ADC.FilterMA32SensorA.w16N);
	uw32Index = EEPROMReadw16(uw32Index, &SYSTEM.ADC.FilterMA32SensorB.w16N);
	// DC link MA filter
	uw32Index = EEPROMReadw16(uw32Index, &SYSTEM.ADC.FilterMA32DCLink.w16N);
	// Temperature MA filter
	uw32Index = EEPROMReadw16(uw32Index, &SYSTEM.ADC.FilterMA32Temperature.w16N);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.ADC.f16CurrentGainFactor);
	//******************************************
	// MCTRL
	//******************************************
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.MCTRL.f16TorqueFactor);
	//******************************************
	// PWM
	//******************************************
	// PWM input variables
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMfullThrottle);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMoffThrottle);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMinThrottle);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMThrottleDifference);
	// PWM input parameters
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMInMiddleValue);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMInHighValRef);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMInLowValRef);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMInMeasureTime);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMInOffZone);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMFilterSize);
	//******************************************
	// SI values
	//******************************************	
	uw32Index = EEPROMReadf(uw32Index, &SYSTEM.SIVALUES.fIInFiltDiv);
	//******************************************
	// Measure params
	//******************************************
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.MEASUREPARAMS.f16SetpointMulti);
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.MEASUREPARAMS.f16LphaIset);
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.MEASUREPARAMS.i16TotalMeasurements);
	//******************************************
	// CAN
	//******************************************
	uw32Index = EEPROMReadui16(uw32Index, &SYSTEM.CAN.ui16CANInfoInterval);
	uw32Index = EEPROMReadui16(uw32Index, &SYSTEM.CAN.ui16CANStatusInterval);
	//******************************************
	// Other
	//******************************************
	// Comm data struct
	// Unit registers
	uw32Index = EEPROMReadui8(uw32Index, &COMMDataStruct.REGS.ui8ID);
	uw32Index = EEPROMReadui16(uw32Index, &COMMDataStruct.REGS.ui16ModelNumber);
	uw32Index = EEPROMReadui8(uw32Index, &COMMDataStruct.REGS.ui8FirmwareVersion);
	uw32Index = EEPROMReadui8(uw32Index, &COMMDataStruct.REGS.ui8BaudRate);
	uw32Index = EEPROMReadui8(uw32Index, &COMMDataStruct.REGS.ui8ReturnDelayTime);
	uw32Index = EEPROMReadi16(uw32Index, &COMMDataStruct.REGS.i16ParkPosition);
	uw32Index = EEPROMReadui16(uw32Index, &COMMDataStruct.ui16RegsBytes);
	uw32Index = EEPROMReadui16(uw32Index, &COMMDataStruct.ui16RXCommTimeout);
	uw32Index = EEPROMReadui16(uw32Index, &COMMDataStruct.ui16TXCommTimeout);
	uw32Index = EEPROMReadi16(uw32Index, &COMMDataStruct.REGS.i16PWMMax);
	uw32Index = EEPROMReadi16(uw32Index, &COMMDataStruct.REGS.i16PWMMin);
	uw32Index = EEPROMReadi16(uw32Index, &COMMDataStruct.REGS.i16MaxRPM);
	uw32Index = EEPROMReadi16(uw32Index, &COMMDataStruct.REGS.i16MinRPM);
	uw32Index = EEPROMReadi16(uw32Index, &COMMDataStruct.REGS.i16CurrentPWM);
	uw32Index = EEPROMReadi16(uw32Index, &COMMDataStruct.REGS.i16ParkPosition);
	uw32Index = EEPROMReadi16(uw32Index, &COMMDataStruct.REGS.i16ZeroSpeedPWM);
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

Int16 StoreEEPROM()
{
	// Index
	UWord32 uw32Index = 0;
	// Store - EEPROM written
	uw32Index = EEPROMStorei16(uw32Index, 1);
	//******************************************
	// Motor ID
	//******************************************	
	
	//******************************************
	// Calibration
	//******************************************
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16MotorPolePairs);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16PolePairArray[0]);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16PolePairArray[1]);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16PolePairArray[2]);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16PolePairArray[3]);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16PolePairArray[4]);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16PolePairArray[5]);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16PolePairArray[6]);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16PolePairArray[7]);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16MaxSensorIndex);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16MinSensorIndex);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.CALIBRATION.i16CalibrationState);
	//******************************************
	// Regulators
	//******************************************
	// D, Q regulators
	// D
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamId.f16PropGain);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamId.f16IntegGain);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamId.i16PropGainShift);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamId.i16IntegGainShift);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamId.f16UpperLimit);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamId.f16LowerLimit);
	// Q
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamIq.f16PropGain);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamIq.f16IntegGain);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamIq.i16PropGainShift);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamIq.i16IntegGainShift);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamIq.f16UpperLimit);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamIq.f16LowerLimit);
	// W regulator
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamW.f16PropGain);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamW.f16IntegGain);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamW.i16PropGainShift);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamW.i16IntegGainShift);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamW.f16UpperLimit);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamW.f16LowerLimit);
	uw32Index = EEPROMStoreui16(uw32Index, SYSTEM.REGULATORS.ui16SpeedRegInterval);
	//******************************************
	// Position
	//******************************************
	// Init for BEMF observer
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.f16PropGain);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.i16PropGainShift);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.f16IntegGain);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.i16IntegGainShift);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.POSITION.acBemfObsrvDQ.f16IGain);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.POSITION.acBemfObsrvDQ.f16UGain);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.POSITION.acBemfObsrvDQ.f16EGain);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.POSITION.acBemfObsrvDQ.f16WIGain);
	// Tracking observer
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.POSITION.acToPos.f16PropGain);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.POSITION.acToPos.i16PropGainShift);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.POSITION.acToPos.f16IntegGain);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.POSITION.acToPos.i16IntegGainShift);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.POSITION.acToPos.f16ThGain);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.POSITION.acToPos.i16ThGainShift);
	// Speed MA filter
	uw32Index = EEPROMStorew16(uw32Index, SYSTEM.POSITION.FilterMA32Speed.w16N);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.POSITION.f16ManualAngleIncrease);
	uw32Index = EEPROMStoref(uw32Index, SYSTEM.POSITION.fOffsetCalcFactor);
	//******************************************
	// Sensorless
	//******************************************
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.SENSORLESS.f16MinSpeed);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.SENSORLESS.f16MaxObserverError);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.SENSORLESS.f16AlignCurrent);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.SENSORLESS.f16StartCurrent);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.SENSORLESS.i16AlignTime);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.SENSORLESS.f16StartSpeed);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.SENSORLESS.f16StartTorque);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.SENSORLESS.f16AngleManualError);
	uw32Index = EEPROMStoreui8(uw32Index, SYSTEM.SENSORLESS.ui8MaxBemfObserverErrorCount);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.SENSORLESS.f16MinSpeedHysteresis);
	//******************************************
	// Ramps
	//******************************************
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.RAMPS.Ramp16_AlignCurrent.f16RampUp);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.RAMPS.Ramp16_AlignCurrent.f16RampDown);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.RAMPS.Ramp16_Speed.f16RampUp);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.RAMPS.Ramp16_Speed.f16RampDown);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.RAMPS.Ramp16_Torque.f16RampUp);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.RAMPS.Ramp16_Torque.f16RampDown);
	//******************************************
	// ADC
	//******************************************
	// Angle filter
	uw32Index = EEPROMStorew16(uw32Index, SYSTEM.ADC.FilterMA32SensorA.w16N);
	uw32Index = EEPROMStorew16(uw32Index, SYSTEM.ADC.FilterMA32SensorB.w16N);
	// DC link MA filter
	uw32Index = EEPROMStorew16(uw32Index, SYSTEM.ADC.FilterMA32DCLink.w16N);
	// Temperature MA filter
	uw32Index = EEPROMStorew16(uw32Index, SYSTEM.ADC.FilterMA32Temperature.w16N);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.ADC.f16CurrentGainFactor);
	//******************************************
	// MCTRL
	//******************************************
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.MCTRL.f16TorqueFactor);
	//******************************************
	// PWM
	//******************************************
	// PWM input variables
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMfullThrottle);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMoffThrottle);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMinThrottle);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMThrottleDifference);
	// PWM input parameters
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMInMiddleValue);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMInHighValRef);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMInLowValRef);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMInMeasureTime);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMInOffZone);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMFilterSize);
	//******************************************
	// SI values
	//******************************************	
	uw32Index = EEPROMStoref(uw32Index, SYSTEM.SIVALUES.fIInFiltDiv);
	//******************************************
	// Measure params
	//******************************************
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.MEASUREPARAMS.f16SetpointMulti);
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.MEASUREPARAMS.f16LphaIset);
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.MEASUREPARAMS.i16TotalMeasurements);
	//******************************************
	// CAN
	//******************************************
	uw32Index = EEPROMStoreui16(uw32Index, SYSTEM.CAN.ui16CANInfoInterval);
	uw32Index = EEPROMStoreui16(uw32Index, SYSTEM.CAN.ui16CANStatusInterval);
	//******************************************
	// Other
	//******************************************
	// Comm data struct
	// Unit registers
	uw32Index = EEPROMStoreui8(uw32Index, COMMDataStruct.REGS.ui8ID);
	uw32Index = EEPROMStoreui16(uw32Index, COMMDataStruct.REGS.ui16ModelNumber);
	uw32Index = EEPROMStoreui8(uw32Index, COMMDataStruct.REGS.ui8FirmwareVersion);
	uw32Index = EEPROMStoreui8(uw32Index, COMMDataStruct.REGS.ui8BaudRate);
	uw32Index = EEPROMStoreui8(uw32Index, COMMDataStruct.REGS.ui8ReturnDelayTime);
	uw32Index = EEPROMStorei16(uw32Index, COMMDataStruct.REGS.i16ParkPosition);
	uw32Index = EEPROMStoreui16(uw32Index, COMMDataStruct.ui16RegsBytes);
	uw32Index = EEPROMStoreui16(uw32Index, COMMDataStruct.ui16RXCommTimeout);
	uw32Index = EEPROMStoreui16(uw32Index, COMMDataStruct.ui16TXCommTimeout);
	uw32Index = EEPROMStorei16(uw32Index, COMMDataStruct.REGS.i16PWMMax);
	uw32Index = EEPROMStorei16(uw32Index, COMMDataStruct.REGS.i16PWMMin);
	uw32Index = EEPROMStorei16(uw32Index, COMMDataStruct.REGS.i16MaxRPM);
	uw32Index = EEPROMStorei16(uw32Index, COMMDataStruct.REGS.i16MinRPM);
	uw32Index = EEPROMStorei16(uw32Index, COMMDataStruct.REGS.i16CurrentPWM);
	uw32Index = EEPROMStorei16(uw32Index, COMMDataStruct.REGS.i16ParkPosition);
	uw32Index = EEPROMStorei16(uw32Index, COMMDataStruct.REGS.i16ZeroSpeedPWM);
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
	
	
	// Index
	UWord32 uw32Index = 1;	//******************************************

	//******************************************
	// Calibration
	//******************************************
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16MotorPolePairs) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16PolePairArray[0]) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16PolePairArray[1]) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16PolePairArray[2]) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16PolePairArray[3]) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16PolePairArray[4]) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16PolePairArray[5]) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16PolePairArray[6]) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16PolePairArray[7]) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16MaxSensorIndex) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16MinSensorIndex) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.CALIBRATION.i16CalibrationState) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	//******************************************
	// Regulators
	//******************************************
	// D, Q regulators
	// D
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamId.f16PropGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamId.f16IntegGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.REGULATORS.mudtControllerParamId.i16PropGainShift) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.REGULATORS.mudtControllerParamId.i16IntegGainShift) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamId.f16UpperLimit) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamId.f16LowerLimit) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	// Q
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamIq.f16PropGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamIq.f16IntegGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.REGULATORS.mudtControllerParamIq.i16PropGainShift) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.REGULATORS.mudtControllerParamIq.i16IntegGainShift) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamIq.f16UpperLimit) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamIq.f16LowerLimit) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	// W regulator
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamW.f16PropGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamW.f16IntegGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.REGULATORS.mudtControllerParamW.i16PropGainShift) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.REGULATORS.mudtControllerParamW.i16IntegGainShift) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamW.f16UpperLimit) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamW.f16LowerLimit) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != SYSTEM.REGULATORS.ui16SpeedRegInterval) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	//******************************************
	// Position
	//******************************************
	// Init for BEMF observer
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.f16PropGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.i16PropGainShift) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.f16IntegGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.i16IntegGainShift) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.POSITION.acBemfObsrvDQ.f16IGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.POSITION.acBemfObsrvDQ.f16UGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.POSITION.acBemfObsrvDQ.f16EGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.POSITION.acBemfObsrvDQ.f16WIGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	// Tracking observer
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.POSITION.acToPos.f16PropGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.POSITION.acToPos.i16PropGainShift) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.POSITION.acToPos.f16IntegGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.POSITION.acToPos.i16IntegGainShift) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.POSITION.acToPos.f16ThGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.POSITION.acToPos.i16ThGainShift) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	// Speed MA filter
	uw32Index = EEPROMReadw16(uw32Index, &w16Temp);
	if(w16Temp != SYSTEM.POSITION.FilterMA32Speed.w16N) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.POSITION.f16ManualAngleIncrease) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf(uw32Index, &fTemp);
	if(fTemp != SYSTEM.POSITION.fOffsetCalcFactor) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	//******************************************
	// Sensorless
	//******************************************
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.SENSORLESS.f16MinSpeed) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.SENSORLESS.f16MaxObserverError) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.SENSORLESS.f16AlignCurrent) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.SENSORLESS.f16StartCurrent) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.SENSORLESS.i16AlignTime) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.SENSORLESS.f16StartSpeed) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.SENSORLESS.f16StartTorque) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.SENSORLESS.f16AngleManualError) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadui8(uw32Index, &ui8Temp);
	if(ui8Temp != SYSTEM.SENSORLESS.ui8MaxBemfObserverErrorCount) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.SENSORLESS.f16MinSpeedHysteresis) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	//******************************************
	// Ramps
	//******************************************
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.RAMPS.Ramp16_AlignCurrent.f16RampUp) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.RAMPS.Ramp16_AlignCurrent.f16RampDown) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.RAMPS.Ramp16_Speed.f16RampUp) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.RAMPS.Ramp16_Speed.f16RampDown) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.RAMPS.Ramp16_Torque.f16RampUp) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.RAMPS.Ramp16_Torque.f16RampDown) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	//******************************************
	// ADC
	//******************************************
	// Angle filter
	uw32Index = EEPROMReadw16(uw32Index, &w16Temp);
	if(w16Temp != SYSTEM.ADC.FilterMA32SensorA.w16N) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadw16(uw32Index, &w16Temp);
	if(w16Temp != SYSTEM.ADC.FilterMA32SensorB.w16N) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	// DC link MA filter
	uw32Index = EEPROMReadw16(uw32Index, &w16Temp);
	if(w16Temp != SYSTEM.ADC.FilterMA32DCLink.w16N) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	// Temperature MA filter
	uw32Index = EEPROMReadw16(uw32Index, &w16Temp);
	if(w16Temp != SYSTEM.ADC.FilterMA32Temperature.w16N) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.ADC.f16CurrentGainFactor) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	//******************************************
	// MCTRL
	//******************************************
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.MCTRL.f16TorqueFactor) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	//******************************************
	// PWM
	//******************************************
	// PWM input variables
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMfullThrottle) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMoffThrottle) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMinThrottle) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMThrottleDifference) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	// PWM input parameters
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMInMiddleValue) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMInHighValRef) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMInLowValRef) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMInMeasureTime) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMInOffZone) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMFilterSize) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	//******************************************
	// SI values
	//******************************************	
	uw32Index = EEPROMReadf(uw32Index, &fTemp);
	if(fTemp != SYSTEM.SIVALUES.fIInFiltDiv) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	//******************************************
	// Measure params
	//******************************************
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.MEASUREPARAMS.f16SetpointMulti) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.MEASUREPARAMS.f16LphaIset) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.MEASUREPARAMS.i16TotalMeasurements) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	//******************************************
	// CAN
	//******************************************
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != SYSTEM.CAN.ui16CANInfoInterval) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != SYSTEM.CAN.ui16CANStatusInterval) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	//******************************************
	// Other
	//******************************************
	// Comm data struct
	// Unit registers
	uw32Index = EEPROMReadui8(uw32Index, &ui8Temp);
	if(ui8Temp != COMMDataStruct.REGS.ui8ID) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != COMMDataStruct.REGS.ui16ModelNumber) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadui8(uw32Index, &ui8Temp);
	if(ui8Temp != COMMDataStruct.REGS.ui8FirmwareVersion) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadui8(uw32Index, &ui8Temp);
	if(ui8Temp != COMMDataStruct.REGS.ui8BaudRate) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadui8(uw32Index, &ui8Temp);
	if(ui8Temp != COMMDataStruct.REGS.ui8ReturnDelayTime) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != COMMDataStruct.REGS.i16ParkPosition) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != COMMDataStruct.ui16RegsBytes) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != COMMDataStruct.ui16RXCommTimeout) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != COMMDataStruct.ui16TXCommTimeout) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != COMMDataStruct.REGS.i16PWMMax) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != COMMDataStruct.REGS.i16PWMMin) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != COMMDataStruct.REGS.i16MaxRPM) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != COMMDataStruct.REGS.i16MinRPM) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != COMMDataStruct.REGS.i16CurrentPWM) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != COMMDataStruct.REGS.i16ParkPosition) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != COMMDataStruct.REGS.i16ZeroSpeedPWM) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index; 
	}
	return NOK;
}
