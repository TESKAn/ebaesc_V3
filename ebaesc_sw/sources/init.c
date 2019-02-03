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
	}
	SYSTEM.CALIBRATION.i16CalibrationState = CALIBRATE_INIT;

		
	//******************************************
	// Regulators
	//******************************************
	// D, Q regulators
	// D
	if(0 != initial)
	{	
		SYSTEM.REGULATORS.mudtControllerParamId.a32PGain = D_KP_ACC32;
		SYSTEM.REGULATORS.mudtControllerParamId.a32IGain = D_KI_ACC32;
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
		SYSTEM.SENSORLESS.ui8MaxBemfObserverErrorCount = MAX_BEMF_ERROR_COUNT;
	}
	SYSTEM.SENSORLESS.f16BEMFErrorPart = FRAC16(0.0);
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
		SYSTEM.ADC.f16OffsetV = FRAC16(0.0);
		SYSTEM.ADC.f16OffsetW = FRAC16(0.0);
		
		SYSTEM.ADC.f16CurrentGainFactor = DRV8301_GAIN_FACTOR;
		
		SYSTEM.ADC.f16MaxCurrentLimit = MAX_PHASE_CURRENT;
		SYSTEM.ADC.i16MaxOvercurrentEvents = MAX_OVERCURRENT_EVENTS;

	}

	// Measured currents to 0
	SYSTEM.ADC.m3IphUVW.f16A = FRAC16(0.0);
	SYSTEM.ADC.m3IphUVW.f16B = FRAC16(0.0);
	SYSTEM.ADC.m3IphUVW.f16C = FRAC16(0.0);
	SYSTEM.ADC.i16MaxOvercurrentsPhA = 0;	
	SYSTEM.ADC.i16MaxOvercurrentsPhB = 0;
	SYSTEM.ADC.i16MaxOvercurrentsPhC = 0;

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
		
		COMMDataStruct.REGS.ui16State = SYSTEM.systemState;
		COMMDataStruct.REGS.ui8ReturnDelayTime = 1;
		// Park position
		COMMDataStruct.REGS.i16ParkPosition = M_PARK_POSITION;
		
		COMMDataStruct.ui16RegsBytes = 64;
		
		COMMDataStruct.ui16RXCommTimeout = 100;
		COMMDataStruct.ui16RXTimeoutCounter = 0;
		COMMDataStruct.ui16TXCommTimeout = 100;
		
		COMMDataStruct.ui16ReadOnlyLow = 0;
		COMMDataStruct.ui16ReadOnlyHigh = 2;
		
		COMMDataStruct.REGS.i16PWMMax = 2000;
		COMMDataStruct.REGS.i16PWMMin = 1000;
		
		COMMDataStruct.REGS.i16MaxRPM = 18000;
		COMMDataStruct.REGS.i16MinRPM = 4000;
		
		COMMDataStruct.REGS.i16CurrentPWM = 1000;
		COMMDataStruct.REGS.i16ParkPosition = 2048;
		
		COMMDataStruct.REGS.i16ZeroSpeedPWM = 50;
	}
	COMMDataStruct.REGS.ui8Armed = 0;
	COMMDataStruct.REGS.ui8Park = 0;
	COMMDataStruct.REGS.i16SetRPM = 0;
	COMMDataStruct.REGS.ui16Errors = 0;
	COMMDataStruct.ui16TXTimeoutCounter = 0;	
	COMMDataStruct.errStatus = 0;
}

// Store vars to EEPROM
UWord32 EEPROMStorei8(UWord32 uw32CurrIndex, Int8 i8Data)
{
	// Var for converting
	t32BitVars t32Vars;
	t32Vars.uw32 = 0;
	
	t32Vars.ints.i8[0] = i8Data;
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t32Vars.uwords.uw16[0]);
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32Vars.uw32);
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMStoreui8(UWord32 uw32CurrIndex, UInt8 ui8Data)
{
	// Var for converting
	t32BitVars t32Vars;
	t32Vars.uw32 = 0;
	
	t32Vars.bytes.ui8[0] = ui8Data;
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t32Vars.uwords.uw16[0]);
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32Vars.uw32);
	
	return uw32CurrIndex + 1;
}

// Store 16 bit signed integer to address and return address + 1
UWord32 EEPROMStorei16(UWord32 uw32CurrIndex, Int16 i16Data)
{
	// Var for converting
	t32BitVars t32Vars;
	t32Vars.uw32 = 0;
	
	t32Vars.words.i16[0] = i16Data;
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t32Vars.uwords.uw16[0]);
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32Vars.uw32);
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMStoref16(UWord32 uw32CurrIndex, Frac16 f16Data)
{
	// Var for converting
	t32BitVars t32Vars;
	t32Vars.uw32 = 0;
	
	t32Vars.frac16.f16[0] = f16Data;
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t32Vars.uwords.uw16[0]);
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32Vars.uw32);
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMStoreui16(UWord32 uw32CurrIndex, UInt16 ui16Data)
{
	// Var for converting
	t32BitVars t32Vars;
	t32Vars.uw32 = 0;
	
	t32Vars.uwords.uw16[0] = ui16Data;
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t32Vars.uwords.uw16[0]);
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32Vars.uw32);
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMStorew16(UWord32 uw32CurrIndex, Word16 w16Data)
{
	// Var for converting
	t32BitVars t32Vars;
	t32Vars.uw32 = 0;
	
	t32Vars.words.i16[0] = w16Data;
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t32Vars.uwords.uw16[0]);
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32Vars.uw32);
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMStoref(UWord32 uw32CurrIndex, float fData)
{
	// Var for converting
	t32BitVars t32Vars;
	t32Vars.uw32 = 0;
	
	t32Vars.f = fData;	
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t32Vars.uwords.uw16[0]);	
	uw32CurrIndex++;
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t32Vars.uwords.uw16[1]);
	
	t32Vars.uwords.uw16[1] = 0;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32Vars.uw32);	
	t32Vars.f = fData;	
	t32Vars.uwords.uw16[0] = t32Vars.uwords.uw16[1];
	t32Vars.uwords.uw16[1] = 0;	
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32Vars.uw32);
	
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMStoreACC32(UWord32 uw32CurrIndex, acc32_t a32Data)
{
	// Var for converting
	t32BitVars t32Vars;
	
	t32Vars.a32 = a32Data;	
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t32Vars.uwords.uw16[0]);
	uw32CurrIndex++;
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t32Vars.uwords.uw16[1]);
	
	t32Vars.uwords.uw16[1] = 0;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32Vars.uw32);	
	t32Vars.a32 = a32Data;	
	t32Vars.uwords.uw16[0] = t32Vars.uwords.uw16[1];
	t32Vars.uwords.uw16[1] = 0;	
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32Vars.uw32);
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMStoreUW32(UWord32 uw32CurrIndex, UWord32 uw32Data)
{
	// Var for converting
	t32BitVars t32Vars;
	
	t32Vars.uw32 = uw32Data;	
	
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t32Vars.uwords.uw16[0]);
	uw32CurrIndex++;
	EepromWriteWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, t32Vars.uwords.uw16[1]);
	
	t32Vars.uwords.uw16[1] = 0;
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32Vars.uw32);	
	t32Vars.uw32 = uw32Data;
	t32Vars.uwords.uw16[0] = t32Vars.uwords.uw16[1];
	t32Vars.uwords.uw16[1] = 0;	
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32Vars.uw32);
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMReadi8(UWord32 uw32CurrIndex, Int8 *i8Data)
{
	// Var for converting
	t32BitVars t32Vars;
	t32Vars.uw32 = 0;
	
	EepromReadWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, &t32Vars.uwords.uw16[0]);
	
	*i8Data = t32Vars.ints.i8[0];
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMReadui8(UWord32 uw32CurrIndex, UInt8 *ui8Data)
{
	// Var for converting
	t32BitVars t32Vars;
	t32Vars.uw32 = 0;
	
	EepromReadWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, &t32Vars.uwords.uw16[0]);
	
	*ui8Data = t32Vars.bytes.ui8[0];
	
	return uw32CurrIndex + 1;
}

// Load 16 bit signed integer to address and return address + 1
UWord32 EEPROMReadi16(UWord32 uw32CurrIndex, Int16 *i16Data)
{
	// Var for converting
	t32BitVars t32Vars;
	t32Vars.uw32 = 0;
	
	EepromReadWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, &t32Vars.uwords.uw16[0]);
	
	*i16Data = t32Vars.words.i16[0];
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMReadsi16(UWord32 uw32CurrIndex, int16_t *i16Data)
{
	// Var for converting
	t32BitVars t32Vars;
	t32Vars.uw32 = 0;
	
	EepromReadWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, &t32Vars.uwords.uw16[0]);
	
	*i16Data = t32Vars.words.i16[0];
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMReadf16(UWord32 uw32CurrIndex, Frac16 *f16Data)
{
	// Var for converting
	t32BitVars t32Vars;
	t32Vars.uw32 = 0;
	
	EepromReadWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, &t32Vars.uwords.uw16[0]);
	
	*f16Data = t32Vars.frac16.f16[0];
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMReadui16(UWord32 uw32CurrIndex, UInt16 *ui16Data)
{
	// Var for converting
	t32BitVars t32Vars;
	t32Vars.uw32 = 0;
	
	EepromReadWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, &t32Vars.uwords.uw16[0]);
	
	*ui16Data = t32Vars.uwords.uw16[0];
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMReadw16(UWord32 uw32CurrIndex, Word16 *w16Data)
{
	// Var for converting
	t32BitVars t32Vars;
	t32Vars.uw32 = 0;
	
	EepromReadWord(EEPROM_BASE_ADDR_WORD + uw32CurrIndex, &t32Vars.uwords.uw16[0]);
	
	*w16Data = t32Vars.words.i16[0];
	
	return uw32CurrIndex + 1;
}

UWord32 EEPROMReadf(UWord32 uw32CurrIndex, float *fData)
{
	// Var for converting
	t32BitVars t32Vars;
	t32Vars.uw32 = 0;
	
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
	t32Vars.uw32 = 0;
	
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
	t32Vars.uw32 = 0;
	
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
	UWord32 uw32CRC = 0;
	UWord32 i = 0;
	t32BitVars t32Vars;
	
	// Setup CRC
	ioctl(CRC, CRC_SET_WRITE_AS_SEED, CRC_ENABLE);	
	delay(1000);
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, 0xffffffff);
	delay(1000);
	ioctl(CRC, CRC_SET_WRITE_AS_SEED, CRC_DISABLE);
	
	EEPROMReadUW32(1, &uw32CRC);
	EEPROMReadui16(3, &ui16Index);
	
	if(1024 <= ui16Index)
	{
		// Error - index too high
		return -1;
	}
	
	for(i=4; i<ui16Index; i++)
	{
		EEPROMReadui16((UWord32)i, &ui16Data);
		
		t32Vars.uw32 = 0;
		t32Vars.uwords.uw16[0] = ui16Data;
		
		ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, t32Vars.uw32);	
		if(ioctl(CRC, CRC_READ_CRC_REG, NULL) == uw32CRC)
		{
			return 0;
		}			
	}
	return -1;
}

Int16 StoreEEPROM()
{
	// Index
	UWord32 uw32Index = 0;
	UWord32 uw32CRC = 0;
	
	// Store - EEPROM written
	uw32Index = EEPROMStorei16(uw32Index, 1);
	
	// Leave two spaces for CRC, one for max index. Start index = 4
	uw32Index = 4;
	
	// Setup CRC
	ioctl(CRC, CRC_SET_WRITE_AS_SEED, CRC_ENABLE);	
	ioctl(CRC, CRC_WRITE_32BIT_CRC_VALUE, 0xffffffff);	
	ioctl(CRC, CRC_SET_WRITE_AS_SEED, CRC_DISABLE);
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
	//******************************************
	// Regulators
	//******************************************
	// D, Q regulators
	// D
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.REGULATORS.mudtControllerParamId.a32PGain); //13
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.REGULATORS.mudtControllerParamId.a32IGain); //14
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamId.f16UpperLim); //15
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamId.f16LowerLim); //16
	// Q
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.REGULATORS.mudtControllerParamIq.a32PGain); //17
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.REGULATORS.mudtControllerParamIq.a32IGain); //18
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamIq.f16UpperLim); //19
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamIq.f16LowerLim); //20
	// W regulator
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.REGULATORS.mudtControllerParamW.a32PGain); //21
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.REGULATORS.mudtControllerParamW.a32IGain); //22
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamW.f16UpperLim); //23
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.REGULATORS.mudtControllerParamW.f16LowerLim); //24
	uw32Index = EEPROMStoreui16(uw32Index, SYSTEM.REGULATORS.ui16SpeedRegInterval); //25
	//******************************************
	// Position
	//******************************************
	// Init for BEMF observer
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.POSITION.acBemfObsrvDQ.sCtrl.a32PGain); //26
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.POSITION.acBemfObsrvDQ.sCtrl.a32IGain); //27
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.POSITION.acBemfObsrvDQ.a32IGain); //28
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.POSITION.acBemfObsrvDQ.a32UGain); //29
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.POSITION.acBemfObsrvDQ.a32WIGain); //30
	uw32Index = EEPROMStoreACC32(uw32Index, SYSTEM.POSITION.acBemfObsrvDQ.a32EGain); //31
	// Tracking observer
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.POSITION.acToPos.f16PGain); //32
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.POSITION.acToPos.i16PGainSh); //33
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.POSITION.acToPos.f16IGain); //34
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.POSITION.acToPos.i16IGainSh); //35
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.POSITION.acToPos.f16ThGain); //36
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.POSITION.acToPos.i16ThGainSh); //37
	// Other init - once
	// Position offset
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.POSITION.i16SensorIndexOffset); //38
	// Phase delay from speed
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.POSITION.i16SensorIndexPhaseDelay); //39
	// Speed MA filter
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.POSITION.f16ManualAngleIncrease); //40
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.POSITION.f16AddedAngleOffset); //41
	uw32Index = EEPROMStoref(uw32Index, SYSTEM.POSITION.fOffsetCalcFactor); //42
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.POSITION.f16AngleOffset); //43
	//******************************************
	// Sensorless
	//******************************************
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.SENSORLESS.f16MinSpeed); //44
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.SENSORLESS.f16MaxObserverError); //45
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.SENSORLESS.f16AlignCurrent); //46
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.SENSORLESS.f16StartCurrent); //47
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.SENSORLESS.i16AlignTime); //48
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.SENSORLESS.f16StartSpeed); //49
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.SENSORLESS.f16StartTorque); //50
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.SENSORLESS.f16AngleManualError); //51
	uw32Index = EEPROMStoreui8(uw32Index, SYSTEM.SENSORLESS.ui8MaxBemfObserverErrorCount); //52
	//******************************************
	// Ramps
	//******************************************
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.RAMPS.Ramp16_AlignCurrent.f16RampUp); //53
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.RAMPS.Ramp16_AlignCurrent.f16RampDown); //54
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.RAMPS.Ramp16_Speed.f16RampUp); //55
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.RAMPS.Ramp16_Speed.f16RampDown); //56
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.RAMPS.Ramp16_Torque.f16RampUp); //57
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.RAMPS.Ramp16_Torque.f16RampDown); //58
	//******************************************
	// ADC
	//******************************************
	// Angle filter
	// DC link MA filter
	// Temperature MA filter
	//******************************************
	// MCTRL
	//******************************************
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.MCTRL.f16TorqueFactor); //59
	//******************************************
	// PWM
	//******************************************
	// PWM input variables
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMfullThrottle); //60
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMoffThrottle); //61
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMinThrottle); //62
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMThrottleDifference); //63
	// PWM input parameters
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMInMiddleValue); //64
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMInHighValRef); //65
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMInLowValRef); //66
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMInMeasureTime); //67
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMInOffZone); //68
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.PWMIN.i16PWMFilterSize); //69
	//******************************************
	// SI values
	//******************************************	
	uw32Index = EEPROMStoref(uw32Index, SYSTEM.SIVALUES.fIInFiltDiv); //70
	//******************************************
	// Measure params
	//******************************************
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.MEASUREPARAMS.f16SetpointMulti); //71
	uw32Index = EEPROMStoref16(uw32Index, SYSTEM.MEASUREPARAMS.f16LphaIset); //72
	uw32Index = EEPROMStorei16(uw32Index, SYSTEM.MEASUREPARAMS.i16TotalMeasurements); //73
	//******************************************
	// CAN
	//******************************************
	uw32Index = EEPROMStoreui16(uw32Index, SYSTEM.CAN.ui16CANInfoInterval); //74
	uw32Index = EEPROMStoreui16(uw32Index, SYSTEM.CAN.ui16CANStatusInterval); //75
	//******************************************
	// Other
	//******************************************
	// Comm data struct
	// Unit registers
	uw32Index = EEPROMStoreui16(uw32Index, COMMDataStruct.REGS.ui16ModelNumber); //76
	uw32Index = EEPROMStoreui8(uw32Index, COMMDataStruct.REGS.ui8FirmwareVersion); //77
	uw32Index = EEPROMStoreui8(uw32Index, COMMDataStruct.REGS.ui8BaudRate); //78
	uw32Index = EEPROMStoreui8(uw32Index, COMMDataStruct.REGS.ui8ReturnDelayTime); //79
	// Park position
	uw32Index = EEPROMStorei16(uw32Index, COMMDataStruct.REGS.i16ParkPosition); //80
	uw32Index = EEPROMStoreui16(uw32Index, COMMDataStruct.ui16RXCommTimeout); //81
	uw32Index = EEPROMStoreui16(uw32Index, COMMDataStruct.ui16RXTimeoutCounter); //82
	uw32Index = EEPROMStoreui16(uw32Index, COMMDataStruct.ui16TXCommTimeout); //83
	uw32Index = EEPROMStoreui16(uw32Index, COMMDataStruct.ui16TXTimeoutCounter); //84
	uw32Index = EEPROMStoreui16(uw32Index, COMMDataStruct.ui16ReadOnlyLow); //85
	uw32Index = EEPROMStoreui16(uw32Index, COMMDataStruct.ui16ReadOnlyHigh); //86
	uw32Index = EEPROMStorei16(uw32Index, COMMDataStruct.REGS.i16PWMMax); //87
	uw32Index = EEPROMStorei16(uw32Index, COMMDataStruct.REGS.i16PWMMin); //88
	uw32Index = EEPROMStorei16(uw32Index, COMMDataStruct.REGS.i16MaxRPM); //89
	uw32Index = EEPROMStorei16(uw32Index, COMMDataStruct.REGS.i16MinRPM); //90
	uw32Index = EEPROMStorei16(uw32Index, COMMDataStruct.REGS.i16CurrentPWM); //91
	uw32Index = EEPROMStorei16(uw32Index, COMMDataStruct.REGS.i16ZeroSpeedPWM); //92
	// Store CRC
	EEPROMStoreUW32(1, ioctl(CRC, CRC_READ_CRC_REG, NULL));
	
	// Store max index
	EEPROMStoreui16(3, (UInt16)uw32Index); //85
	
	return (Int16)uw32Index;
}


Int16 LoadEEPROM()
{
	// Index
	UWord32 uw32Index = 4;
	// Check - is CRC OK?
	if(0 != checkEEPROMCRC())
	{
		// CRC not OK, return error
		return -1;
	}
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
	//******************************************
	// Regulators
	//******************************************
	// D, Q regulators
	// D
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamId.a32PGain); //13
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamId.a32IGain); //14
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamId.f16UpperLim); //15
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamId.f16LowerLim); //16
	// Q
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamIq.a32PGain); //17
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamIq.a32IGain); //18
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamIq.f16UpperLim); //19
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamIq.f16LowerLim); //20
	// W regulator
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamW.a32PGain); //21
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamW.a32IGain); //22
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamW.f16UpperLim); //23
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.REGULATORS.mudtControllerParamW.f16LowerLim); //24
	uw32Index = EEPROMReadui16(uw32Index, &SYSTEM.REGULATORS.ui16SpeedRegInterval); //25
	//******************************************
	// Position
	//******************************************
	// Init for BEMF observer
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.POSITION.acBemfObsrvDQ.sCtrl.a32PGain); //26
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.POSITION.acBemfObsrvDQ.sCtrl.a32IGain); //27
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.POSITION.acBemfObsrvDQ.a32IGain); //28
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.POSITION.acBemfObsrvDQ.a32UGain); //29
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.POSITION.acBemfObsrvDQ.a32WIGain); //30
	uw32Index = EEPROMReadACC32(uw32Index, &SYSTEM.POSITION.acBemfObsrvDQ.a32EGain); //31
	// Tracking observer
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.POSITION.acToPos.f16PGain); //32
	uw32Index = EEPROMReadsi16(uw32Index, &SYSTEM.POSITION.acToPos.i16PGainSh); //33
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.POSITION.acToPos.f16IGain); //34
	uw32Index = EEPROMReadsi16(uw32Index, &SYSTEM.POSITION.acToPos.i16IGainSh); //35
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.POSITION.acToPos.f16ThGain); //36
	uw32Index = EEPROMReadsi16(uw32Index, &SYSTEM.POSITION.acToPos.i16ThGainSh); //37
	// Other init - once
	// Position offset
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.POSITION.i16SensorIndexOffset); //38
	// Phase delay from speed
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.POSITION.i16SensorIndexPhaseDelay); //39
	// Speed MA filter
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.POSITION.f16ManualAngleIncrease); //40
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.POSITION.f16AddedAngleOffset); //41
	uw32Index = EEPROMReadf(uw32Index, &SYSTEM.POSITION.fOffsetCalcFactor); //42
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.POSITION.f16AngleOffset); //43
	//******************************************
	// Sensorless
	//******************************************
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.SENSORLESS.f16MinSpeed); //44
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.SENSORLESS.f16MaxObserverError); //45
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.SENSORLESS.f16AlignCurrent); //46
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.SENSORLESS.f16StartCurrent); //47
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.SENSORLESS.i16AlignTime); //48
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.SENSORLESS.f16StartSpeed); //49
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.SENSORLESS.f16StartTorque); //50
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.SENSORLESS.f16AngleManualError); //51
	uw32Index = EEPROMReadui8(uw32Index, &SYSTEM.SENSORLESS.ui8MaxBemfObserverErrorCount); //52
	//******************************************
	// Ramps
	//******************************************
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.RAMPS.Ramp16_AlignCurrent.f16RampUp); //53
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.RAMPS.Ramp16_AlignCurrent.f16RampDown); //54
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.RAMPS.Ramp16_Speed.f16RampUp); //55
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.RAMPS.Ramp16_Speed.f16RampDown); //56
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.RAMPS.Ramp16_Torque.f16RampUp); //57
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.RAMPS.Ramp16_Torque.f16RampDown); //58
	//******************************************
	// ADC
	//******************************************
	// Angle filter
	// DC link MA filter
	// Temperature MA filter
	//******************************************
	// MCTRL
	//******************************************
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.MCTRL.f16TorqueFactor); //59
	//******************************************
	// PWM
	//******************************************
	// PWM input variables
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMfullThrottle); //60
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMoffThrottle); //61
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMinThrottle); //62
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMThrottleDifference); //63
	// PWM input parameters
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMInMiddleValue); //64
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMInHighValRef); //65
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMInLowValRef); //66
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMInMeasureTime); //67
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMInOffZone); //68
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.PWMIN.i16PWMFilterSize); //69
	//******************************************
	// SI values
	//******************************************	
	uw32Index = EEPROMReadf(uw32Index, &SYSTEM.SIVALUES.fIInFiltDiv); //70
	//******************************************
	// Measure params
	//******************************************
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.MEASUREPARAMS.f16SetpointMulti); //71
	uw32Index = EEPROMReadf16(uw32Index, &SYSTEM.MEASUREPARAMS.f16LphaIset); //72
	uw32Index = EEPROMReadi16(uw32Index, &SYSTEM.MEASUREPARAMS.i16TotalMeasurements); //73
	//******************************************
	// CAN
	//******************************************
	uw32Index = EEPROMReadui16(uw32Index, &SYSTEM.CAN.ui16CANInfoInterval); //74
	uw32Index = EEPROMReadui16(uw32Index, &SYSTEM.CAN.ui16CANStatusInterval); //75
	//******************************************
	// Other
	//******************************************
	// Comm data struct
	// Unit registers
	uw32Index = EEPROMReadui16(uw32Index, &COMMDataStruct.REGS.ui16ModelNumber); //76
	uw32Index = EEPROMReadui8(uw32Index, &COMMDataStruct.REGS.ui8FirmwareVersion); //77
	uw32Index = EEPROMReadui8(uw32Index, &COMMDataStruct.REGS.ui8BaudRate); //78
	uw32Index = EEPROMReadui8(uw32Index, &COMMDataStruct.REGS.ui8ReturnDelayTime); //79
	// Park position
	uw32Index = EEPROMReadi16(uw32Index, &COMMDataStruct.REGS.i16ParkPosition); //80
	uw32Index = EEPROMReadui16(uw32Index, &COMMDataStruct.ui16RXCommTimeout); //81
	uw32Index = EEPROMReadui16(uw32Index, &COMMDataStruct.ui16RXTimeoutCounter); //82
	uw32Index = EEPROMReadui16(uw32Index, &COMMDataStruct.ui16TXCommTimeout); //83
	uw32Index = EEPROMReadui16(uw32Index, &COMMDataStruct.ui16TXTimeoutCounter); //84
	uw32Index = EEPROMReadui16(uw32Index, &COMMDataStruct.ui16ReadOnlyLow); //85
	uw32Index = EEPROMReadui16(uw32Index, &COMMDataStruct.ui16ReadOnlyHigh); //86
	uw32Index = EEPROMReadi16(uw32Index, &COMMDataStruct.REGS.i16PWMMax); //87
	uw32Index = EEPROMReadi16(uw32Index, &COMMDataStruct.REGS.i16PWMMin); //88
	uw32Index = EEPROMReadi16(uw32Index, &COMMDataStruct.REGS.i16MaxRPM); //89
	uw32Index = EEPROMReadi16(uw32Index, &COMMDataStruct.REGS.i16MinRPM); //90
	uw32Index = EEPROMReadi16(uw32Index, &COMMDataStruct.REGS.i16CurrentPWM); //91
	uw32Index = EEPROMReadi16(uw32Index, &COMMDataStruct.REGS.i16ZeroSpeedPWM); //92
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
	UWord32 uw32Index = 4;	//******************************************
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
	} //13
	uw32Index = EEPROMReadACC32(uw32Index, &a32Temp);
	if(a32Temp != SYSTEM.REGULATORS.mudtControllerParamId.a32IGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //14
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamId.f16UpperLim) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //15
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamId.f16LowerLim) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //16
	// Q
	uw32Index = EEPROMReadACC32(uw32Index, &a32Temp);
	if(a32Temp != SYSTEM.REGULATORS.mudtControllerParamIq.a32PGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //17
	uw32Index = EEPROMReadACC32(uw32Index, &a32Temp);
	if(a32Temp != SYSTEM.REGULATORS.mudtControllerParamIq.a32IGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //18
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamIq.f16UpperLim) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //19
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamIq.f16LowerLim) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //20
	// W regulator
	uw32Index = EEPROMReadACC32(uw32Index, &a32Temp);
	if(a32Temp != SYSTEM.REGULATORS.mudtControllerParamW.a32PGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //21
	uw32Index = EEPROMReadACC32(uw32Index, &a32Temp);
	if(a32Temp != SYSTEM.REGULATORS.mudtControllerParamW.a32IGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //22
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamW.f16UpperLim) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //23
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.REGULATORS.mudtControllerParamW.f16LowerLim) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //24
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != SYSTEM.REGULATORS.ui16SpeedRegInterval) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //25
	//******************************************
	// Position
	//******************************************
	// Init for BEMF observer
	uw32Index = EEPROMReadACC32(uw32Index, &a32Temp);
	if(a32Temp != SYSTEM.POSITION.acBemfObsrvDQ.sCtrl.a32PGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //26
	uw32Index = EEPROMReadACC32(uw32Index, &a32Temp);
	if(a32Temp != SYSTEM.POSITION.acBemfObsrvDQ.sCtrl.a32IGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //27
	uw32Index = EEPROMReadACC32(uw32Index, &a32Temp);
	if(a32Temp != SYSTEM.POSITION.acBemfObsrvDQ.a32IGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //28
	uw32Index = EEPROMReadACC32(uw32Index, &a32Temp);
	if(a32Temp != SYSTEM.POSITION.acBemfObsrvDQ.a32UGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //29
	uw32Index = EEPROMReadACC32(uw32Index, &a32Temp);
	if(a32Temp != SYSTEM.POSITION.acBemfObsrvDQ.a32WIGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //30
	uw32Index = EEPROMReadACC32(uw32Index, &a32Temp);
	if(a32Temp != SYSTEM.POSITION.acBemfObsrvDQ.a32EGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //31
	// Tracking observer
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.POSITION.acToPos.f16PGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //32
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.POSITION.acToPos.i16PGainSh) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //33
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.POSITION.acToPos.f16IGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //34
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.POSITION.acToPos.i16IGainSh) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //35
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.POSITION.acToPos.f16ThGain) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //36
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.POSITION.acToPos.i16ThGainSh) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //37
	// Other init - once
	// Position offset
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.POSITION.i16SensorIndexOffset) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //38
	// Phase delay from speed
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.POSITION.i16SensorIndexPhaseDelay) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //39
	// Speed MA filter
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.POSITION.f16ManualAngleIncrease) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //40
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.POSITION.f16AddedAngleOffset) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //41
	uw32Index = EEPROMReadf(uw32Index, &fTemp);
	if(fTemp != SYSTEM.POSITION.fOffsetCalcFactor) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //42
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.POSITION.f16AngleOffset) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //43
	//******************************************
	// Sensorless
	//******************************************
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.SENSORLESS.f16MinSpeed) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //44
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.SENSORLESS.f16MaxObserverError) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //45
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.SENSORLESS.f16AlignCurrent) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //46
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.SENSORLESS.f16StartCurrent) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //47
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.SENSORLESS.i16AlignTime) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //48
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.SENSORLESS.f16StartSpeed) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //49
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.SENSORLESS.f16StartTorque) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //50
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.SENSORLESS.f16AngleManualError) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //51
	uw32Index = EEPROMReadui8(uw32Index, &ui8Temp);
	if(ui8Temp != SYSTEM.SENSORLESS.ui8MaxBemfObserverErrorCount) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //52
	//******************************************
	// Ramps
	//******************************************
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.RAMPS.Ramp16_AlignCurrent.f16RampUp) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //53
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.RAMPS.Ramp16_AlignCurrent.f16RampDown) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //54
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.RAMPS.Ramp16_Speed.f16RampUp) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //55
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.RAMPS.Ramp16_Speed.f16RampDown) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //56
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.RAMPS.Ramp16_Torque.f16RampUp) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //57
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.RAMPS.Ramp16_Torque.f16RampDown) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //58
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
	} //59
	//******************************************
	// PWM
	//******************************************
	// PWM input variables
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMfullThrottle) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //60
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMoffThrottle) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //61
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMinThrottle) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //62
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMThrottleDifference) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //63
	// PWM input parameters
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMInMiddleValue) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //64
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMInHighValRef) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //65
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMInLowValRef) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //66
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMInMeasureTime) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //67
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMInOffZone) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //68
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.PWMIN.i16PWMFilterSize) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //69
	//******************************************
	// SI values
	//******************************************	
	uw32Index = EEPROMReadf(uw32Index, &fTemp);
	if(fTemp != SYSTEM.SIVALUES.fIInFiltDiv) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //70
	//******************************************
	// Measure params
	//******************************************
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.MEASUREPARAMS.f16SetpointMulti) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //71
	uw32Index = EEPROMReadf16(uw32Index, &f16Temp);
	if(f16Temp != SYSTEM.MEASUREPARAMS.f16LphaIset) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //72
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != SYSTEM.MEASUREPARAMS.i16TotalMeasurements) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //73
	//******************************************
	// CAN
	//******************************************
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != SYSTEM.CAN.ui16CANInfoInterval) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //74
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != SYSTEM.CAN.ui16CANStatusInterval) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //75
	//******************************************
	// Other
	//******************************************
	// Comm data struct
	// Unit registers
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != COMMDataStruct.REGS.ui16ModelNumber) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //76
	uw32Index = EEPROMReadui8(uw32Index, &ui8Temp);
	if(ui8Temp != COMMDataStruct.REGS.ui8FirmwareVersion) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //77
	uw32Index = EEPROMReadui8(uw32Index, &ui8Temp);
	if(ui8Temp != COMMDataStruct.REGS.ui8BaudRate) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //78
	uw32Index = EEPROMReadui8(uw32Index, &ui8Temp);
	if(ui8Temp != COMMDataStruct.REGS.ui8ReturnDelayTime) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //79
	// Park position
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != COMMDataStruct.REGS.i16ParkPosition) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //80
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != COMMDataStruct.ui16RXCommTimeout) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //81
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != COMMDataStruct.ui16RXTimeoutCounter) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //82
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != COMMDataStruct.ui16TXCommTimeout) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //83
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != COMMDataStruct.ui16TXTimeoutCounter) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //84
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != COMMDataStruct.ui16ReadOnlyLow) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //85
	uw32Index = EEPROMReadui16(uw32Index, &ui16Temp);
	if(ui16Temp != COMMDataStruct.ui16ReadOnlyHigh) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //86
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != COMMDataStruct.REGS.i16PWMMax) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //87
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != COMMDataStruct.REGS.i16PWMMin) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //88
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != COMMDataStruct.REGS.i16MaxRPM) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //89
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != COMMDataStruct.REGS.i16MinRPM) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //90
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != COMMDataStruct.REGS.i16CurrentPWM) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //91
	uw32Index = EEPROMReadi16(uw32Index, &i16Temp);
	if(i16Temp != COMMDataStruct.REGS.i16ZeroSpeedPWM) 
	{ 
		NOK++; 
		uw32ErrorIndex = uw32Index - 1; 
	} //92
	return NOK;
}
