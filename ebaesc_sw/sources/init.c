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
	SYSTEM.i16MotorID = M_ID;
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
		SYSTEM.REGULATORS.mudtControllerParamId.f16UpperLimit = FRAC16(0.9);
		SYSTEM.REGULATORS.mudtControllerParamId.f16LowerLimit = FRAC16(-0.9);
		
		// Q
		SYSTEM.REGULATORS.mudtControllerParamIq.f16PropGain = Q_KP_GAIN;
		SYSTEM.REGULATORS.mudtControllerParamIq.f16IntegGain = Q_KI_GAIN;
		SYSTEM.REGULATORS.mudtControllerParamIq.i16PropGainShift = Q_KP_SHIFT;
		SYSTEM.REGULATORS.mudtControllerParamIq.i16IntegGainShift = Q_KI_SHIFT;
		SYSTEM.REGULATORS.mudtControllerParamIq.f16UpperLimit = FRAC16(0.9);
		SYSTEM.REGULATORS.mudtControllerParamIq.f16LowerLimit = FRAC16(-0.9);
		
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
		SYSTEM.POSITION.acToPos.f16PropGain= FRAC16(0.767975);
		SYSTEM.POSITION.acToPos.i16PropGainShift= -5;
		SYSTEM.POSITION.acToPos.f16IntegGain= FRAC16(0.61438);
		SYSTEM.POSITION.acToPos.i16IntegGainShift= -10;
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
		SYSTEM.PWMIN.i16PWMFracMultiplier = 32768 / (PWM_IN_HIGH_VAL_REF - PWM_IN_LOW_VAL_REF);	
		
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
	// Other
	//******************************************
	if(0 != initial)
	{
		// Mark transition to idle state
		SYSTEM.i16StateTransition = SYSTEM_IDLE;
		
		SYSTEM.COMMVALUES.i16ParkPosition = 2048;
		
		// If there is calibration data
		if(0 != SYSTEM.CALIBRATION.i16PolePairArray[0])
		{
			// Calculate calibration values
			CalculateCalibrationData();	
			// Mark calibrated
			SYSTEM_CALIBRATED = 1;	
		}
	}	
	// Error log init
	for(SYSTEM.i16ErrorIndex = 0; SYSTEM.i16ErrorIndex < 16; SYSTEM.i16ErrorIndex++)
	{
		SYSTEM.i8ErrorLog[SYSTEM.i16ErrorIndex] = 0;
	}
	SYSTEM.i16ErrorIndex = 0;
}
