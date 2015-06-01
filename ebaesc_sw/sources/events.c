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
	Frac16 f16Temp1 = FRAC16(0.0);
	Frac16 mf16ErrorK;
	Frac16 f16SpeedErrorK = FRAC16(0.0);
	Int16 mi16SatFlag = 0;
	
	// Clear EOSI flag
	ioctl(ADC_1, ADC_CLEAR_STATUS_EOSI, NULL);
	// Call freemaster recorder
	FMSTR_Recorder();
	// Calibration data readout
	// Go through calibration data?
	if(4096 > i16CurrentCalArrayIndex)
	{
		// First increase index, then store value
		i16CurrentCalArrayIndex++;
		f16CurrentCalArrayData = SYSTEM.CALIBRATION.f16CalibrationArray[i16CurrentCalArrayIndex];					
	}	
	
	//******************************************
	// Read data
	//******************************************
	// Phase currents
	SYSTEM.ADC.m3IphUVW.f16A = ioctl(ADC_1, ADC_READ_SAMPLE, 0);
	SYSTEM.ADC.m3IphUVW.f16C = ioctl(ADC_1, ADC_READ_SAMPLE, 8);
	// Zero currents?
	if(SYS_ZERO_CURRENT)
	{
		SYS_ZERO_CURRENT = 0;
		SYSTEM.ADC.f16OffsetU = SYSTEM.ADC.m3IphUVW.f16A;
		SYSTEM.ADC.f16OffsetW = SYSTEM.ADC.m3IphUVW.f16C;
	}
	
	// Remove offsets
	SYSTEM.ADC.m3IphUVW.f16A = SYSTEM.ADC.m3IphUVW.f16A - SYSTEM.ADC.f16OffsetU;
	SYSTEM.ADC.m3IphUVW.f16C = SYSTEM.ADC.m3IphUVW.f16C - SYSTEM.ADC.f16OffsetW;
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
	SYSTEM.INPUTCAPTURE.Val0 = ioctl(EFPWMA_SUB0, EFPWMS_READ_CAPTURE_VAL0, NULL);
	SYSTEM.INPUTCAPTURE.Val1 = ioctl(EFPWMA_SUB0, EFPWMS_READ_CAPTURE_VAL1, NULL);
	SYSTEM.INPUTCAPTURE.Val2 = ioctl(EFPWMA_SUB0, EFPWMS_READ_CAPTURE_VAL2, NULL);
	SYSTEM.INPUTCAPTURE.Val3 = ioctl(EFPWMA_SUB0, EFPWMS_READ_CAPTURE_VAL3, NULL);
	SYSTEM.INPUTCAPTURE.Val4 = ioctl(EFPWMA_SUB0, EFPWMS_READ_CAPTURE_VAL4, NULL);
	SYSTEM.INPUTCAPTURE.Val5 = ioctl(EFPWMA_SUB0, EFPWMS_READ_CAPTURE_VAL5, NULL);
	
	// Enable capture
	ioctl(EFPWMA_SUB0, EFPWMS_ACTIVE_CAPTURE_X, NULL);
	
	
	
	
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
	i16Temp = mult(SYSTEM.POSITION.i16SensorIndexPhaseDelay, SYSTEM.POSITION.f16SpeedFiltered);
	SYSTEM.POSITION.i16SensorIndex = i16Temp + SYSTEM.POSITION.i16SensorIndex;
	// Wrap
    /*
     * TODO: Check wrap for negative speeds 
     *
     */
	SYSTEM.POSITION.i16SensorIndex = SYSTEM.POSITION.i16SensorIndex & 4095;
	// Get measured angle from previous iteration
	SYSTEM.POSITION.f16MeasuredRotorAngle = SYSTEM.CALIBRATION.f16CalibrationArray[SYSTEM.POSITION.i16SensorIndex];
	// Calculate phase error
	// SYSTEM.POSITION.f16RotorAngle = calculated angle from previous iteration
	// f16Temp = measured angle from previous iteration
	SYSTEM.POSITION.f16MeasuredAngleError = SYSTEM.POSITION.f16MeasuredRotorAngle - SYSTEM.POSITION.f16RotorAngle; 
	// Store to previous angle
	SYSTEM.POSITION.f16RotorAngle_m = SYSTEM.POSITION.f16RotorAngle;
	
	//******************************************
	// Transform currents to rotating frame
	//******************************************
	// Clark transform to get Ia, Ib
	MCLIB_ClarkTrf(&SYSTEM.MCTRL.m2IAlphaBeta, &SYSTEM.ADC.m3IphUVW);
	// Calculate park transform to get Id, Iq
	// Out m2IDQ
	MCLIB_ParkTrf(&SYSTEM.MCTRL.m2IDQ, &SYSTEM.MCTRL.m2IAlphaBeta, &SYSTEM.POSITION.mSinCosAngle);
	
	//******************************************
	// BEMF observer calculation
	//******************************************
	// Absolute speed value
	f16Temp = abs_s(SYSTEM.POSITION.f16SpeedFiltered);
	// If speed over BEMF min speed
	if(SYSTEM.SENSORLESS.f16MinSpeed < f16Temp)
	{
		// Calculate DQ observer
		ACLIB_PMSMBemfObsrvDQ(&SYSTEM.MCTRL.m2IDQ, &SYSTEM.MCTRL.m2UDQ_m, SYSTEM.POSITION.f16SpeedFiltered, &SYSTEM.POSITION.acBemfObsrvDQ);
		if(!SENSORLESS_BEMF_ON)
		{
			// Check that error is small enough before we enable use of observer
			// Are we using sensor?
			if(SYSTEM_CALIBRATED && SYSTEM_RUN_SENSORED)
			{
				// Check error
				f16Temp = abs_s(SYSTEM.POSITION.acBemfObsrvDQ.f16Error);
				if(f16Temp < SYSTEM.SENSORLESS.f16MaxObserverError)
				{
					SENSORLESS_BEMF_ON = 1;
					// Subtract hysteresis
					SYSTEM.SENSORLESS.f16MinSpeed -= SYSTEM.SENSORLESS.f16MinSpeedHysteresis;					
				}
			}
			else
			{
				// We are free - running - startup
				SENSORLESS_BEMF_ON = 1;
				// Subtract hysteresis
				SYSTEM.SENSORLESS.f16MinSpeed -= SYSTEM.SENSORLESS.f16MinSpeedHysteresis;	
			}
		}
	}
	// Else if observer was ON
	else if(SENSORLESS_BEMF_ON)
	{
		SENSORLESS_BEMF_ON = 0;
		// Add hysteresis
		SYSTEM.SENSORLESS.f16MinSpeed += SYSTEM.SENSORLESS.f16MinSpeedHysteresis;
	}
	
	
	//******************************************
	// Motor angle calculation and manual increase
	//******************************************
	// Here are differences according to how we are running the motor	
	switch(SYSTEM.POSITION.i16PositionSource)
	{
		case POSITION_SOURCE_NONE:
		{
			SYSTEM.POSITION.f16RotorAngle = FRAC16(0.0);
			SYSTEM.POSITION.f16Speed = FRAC16(0.0);
			SYSTEM.POSITION.f16SpeedFiltered = FRAC16(0.0);
			break;
		}
		case POSITION_SOURCE_MANUAL:
		{
			// Check CW/CCW.
			if(SYSTEM_RUN_MANUAL_CW)
			{
				// Increase angle
				SYSTEM.POSITION.f16RotorAngle += SYSTEM.POSITION.f16ManualAngleIncrease;
			}
			else if(SYSTEM_RUN_MANUAL_CCW)
			{
				// Decrease angle
				SYSTEM.POSITION.f16RotorAngle -= SYSTEM.POSITION.f16ManualAngleIncrease;
			}
			else if(SYS_CAL_GOTO_NEXT_POLE)
			{
				// Rotate until we get to next pole
				SYSTEM.POSITION.f16RotorAngle += 1;
				if(FRAC16(1.0) > SYSTEM.POSITION.f16RotorAngle)
				{
					if(FRAC16(0.95) > SYSTEM.POSITION.f16RotorAngle)
					{
						SYSTEM.POSITION.f16RotorAngle += 10;	
					}
					else
					{
						SYSTEM.POSITION.f16RotorAngle += 1;
					}
				}
				else
				{
					SYS_CAL_GOTO_NEXT_POLE = 0;
					SYSTEM.POSITION.f16RotorAngle = FRAC16(1.0);
				}
			}
			break;
		}
		case POSITION_SOURCE_SENSORLESS_ALIGN:
		{
			SYSTEM.POSITION.f16RotorAngle = FRAC16(1.0);
			SYSTEM.POSITION.f16Speed = FRAC16(0.0);
			SYSTEM.POSITION.f16SpeedFiltered = FRAC16(0.0);
			// Check align time
			if(0 == SYSTEM.SENSORLESS.i16Counter)
			{
				// Go to rotate
				SYSTEM.REGULATORS.m2IDQReq.f16Q = SYSTEM.REGULATORS.m2IDQReq.f16D;
				SYSTEM.REGULATORS.m2IDQReq.f16D = FRAC16(0.0);		
				// Set open loop ramp
				SYSTEM.RAMPS.f16OLSpeedRampActualValue = FRAC16(0.0);
				SYSTEM.POSITION.i16PositionSource = POSITION_SOURCE_SENSORLESS_ROTATE;
				SYSTEM.REGULATORS.i16CurrentSource = CURRENT_SOURCE_SENSORLESS_ROTATE;
			}
			break;
		}
		case POSITION_SOURCE_SENSORLESS_ROTATE:
		{
			// Manual angle increase for open loop startup
			SYSTEM.POSITION.f16RotorAngle += SYSTEM.RAMPS.f16OLSpeedRampActualValue;
			// Multiply angle increase with some factor to get speed
			SYSTEM.POSITION.f16Speed = SYSTEM.RAMPS.f16OLSpeedRampActualValue;
			// Filter speed
			SYSTEM.POSITION.f16SpeedFiltered = GDFLIB_FilterMA32(SYSTEM.POSITION.f16Speed, &SYSTEM.POSITION.FilterMA32Speed);
			if(SYSTEM.RAMPS.f16OLSpeedRampDesiredValue != SYSTEM.RAMPS.f16OLSpeedRampActualValue)
			{
				SYSTEM.RAMPS.f16OLSpeedRampActualValue = GFLIB_Ramp16(SYSTEM.RAMPS.f16OLSpeedRampDesiredValue, SYSTEM.RAMPS.f16OLSpeedRampActualValue, &SYSTEM.RAMPS.Ramp16_OLSpeedStartup);
			}			
			// If BEMF calculating
			if(SENSORLESS_BEMF_ON)
			{
				// Calculate tracking observer with observer error
				SYSTEM.POSITION.f16RotorAngle_OL = ACLIB_TrackObsrv(SYSTEM.POSITION.acBemfObsrvDQ.f16Error, &SYSTEM.POSITION.acToPos);
				
				// Check tracking observer and forced speed
				f16Temp = SYSTEM.POSITION.f16SpeedFiltered;
				f16Temp1 = extract_h(SYSTEM.POSITION.acToPos.f32Speed);
				f16Temp = f16Temp - f16Temp1;
				SYSTEM.SENSORLESS.f16SpeedDifference = abs_s(f16Temp);
				if(SYSTEM.SENSORLESS.f16MergeSpeedDifference > SYSTEM.SENSORLESS.f16SpeedDifference)
				{
					SYSTEM.SENSORLESS.i16MergeDifferenceCount++;
					if(SYSTEM.SENSORLESS.i16MergeDifferenceCount > SYSTEM.SENSORLESS.i16MergeDifferenceCountThreshold)
					{
						SYSTEM.SENSORLESS.f16MergeAngleOffset = SYSTEM.POSITION.f16RotorAngle - SYSTEM.POSITION.f16RotorAngle_OL;
						SYSTEM.POSITION.i16PositionSource = POSITION_SOURCE_SENSORLESS_MERGE;
					}
				}
			}			
			break;
		}
		case POSITION_SOURCE_SENSORLESS_MERGE:
		{
			// Keep updating forced angle
			//SYSTEM.POSITION.f16RotorAngle += SYSTEM.RAMPS.f16OLSpeedRampActualValue;
			// Calculate tracking observer with observer error
			SYSTEM.POSITION.f16RotorAngle = ACLIB_TrackObsrv(SYSTEM.POSITION.acBemfObsrvDQ.f16Error, &SYSTEM.POSITION.acToPos);
			
			SYSTEM.POSITION.f16RotorAngle += SYSTEM.SENSORLESS.f16MergeAngleOffset;
			
			// Merge forced angle to observer angle
			if(0 < SYSTEM.SENSORLESS.f16MergeAngleOffset)
			{
				SYSTEM.SENSORLESS.f16MergeAngleOffset --;
			}
			else if(0 > SYSTEM.SENSORLESS.f16MergeAngleOffset)
			{
				SYSTEM.SENSORLESS.f16MergeAngleOffset ++;
			}
			else
			{
				// Angles merged, switch
				SYSTEM.POSITION.i16PositionSource = POSITION_SOURCE_MULTIPLE;
				// Set current source to speed or torque
				if(CONTROL_SPEED)
				{
					SYSTEM.REGULATORS.i16CurrentSource = CURRENT_SOURCE_CONTROL_SPEED;
					// Set default values
					SYSTEM.REGULATORS.ui16SpeedRegCounter = 0;
					SYSTEM.RAMPS.f16SpeedRampDesiredValue = SYSTEM.SENSORLESS.f16StartSpeed;
					SYSTEM.RAMPS.f16SpeedRampActualValue = SYSTEM.POSITION.f16Speed;
				}
				else if(CONTROL_TORQUE)
				{
					SYSTEM.REGULATORS.i16CurrentSource = CURRENT_SOURCE_CONTROL_TORQUE;
					// Set default values
					SYSTEM.RAMPS.f16TorqueRampActualValue = SYSTEM.REGULATORS.m2IDQReq.f16Q /  SYSTEM.MCTRL.f16TorqueFactor;
					SYSTEM.RAMPS.f16TorqueRampDesiredValue = SYSTEM.SENSORLESS.f16StartTorque;
				}
				else
				{
					// No source, abort
					SYSTEM.POSITION.i16PositionSource = POSITION_SOURCE_NONE;
					SYSTEM.REGULATORS.i16CurrentSource = CURRENT_SOURCE_NONE;
				}
			}
			// Store & filter speed from tracking observer
			SYSTEM.POSITION.f16Speed = extract_h(SYSTEM.POSITION.acToPos.f32Speed);
			SYSTEM.POSITION.f16SpeedFiltered = GDFLIB_FilterMA32(SYSTEM.POSITION.f16Speed, &SYSTEM.POSITION.FilterMA32Speed);
			
			break;
		}
		case POSITION_SOURCE_MULTIPLE:
		{
			// Get rotor angle from tracking observer
			// Merge(?) errors from measured and observed angle
			// i16Temp counts error sources
			i16Temp = 0;
			mf16ErrorK = FRAC16(0.0);
			if(SYSTEM_CALIBRATED && SYSTEM_RUN_SENSORED)
			{
				// Sensor calibrated and use sensor
				i16Temp ++;
				// SYSTEM.POSITION.f16AnglePhaseError holds sensor error
				mf16ErrorK += SYSTEM.POSITION.f16MeasuredAngleError;
			}
			if(SENSORLESS_BEMF_ON)
			{
				// We have good BEMF signal, use it
				i16Temp ++;
				mf16ErrorK += SYSTEM.POSITION.acBemfObsrvDQ.f16Error;
			}
			// Divide by 2?
			if(1 < i16Temp)
			{
				mf16ErrorK = mult(mf16ErrorK, FRAC16(0.5));
			}
			
			SYSTEM.POSITION.f16AnglePhaseError = mf16ErrorK;
			
			// Calculate angle tracking observer or use forced angle
			SYSTEM.POSITION.f16RotorAngle = ACLIB_TrackObsrv(SYSTEM.POSITION.f16AnglePhaseError, &SYSTEM.POSITION.acToPos);
			
			// Store & filter speed
			SYSTEM.POSITION.f16Speed = extract_h(SYSTEM.POSITION.acToPos.f32Speed);
			SYSTEM.POSITION.f16SpeedFiltered = GDFLIB_FilterMA32(SYSTEM.POSITION.f16Speed, &SYSTEM.POSITION.FilterMA32Speed);	
			break;
		}
		default:
		{
			SYSTEM.POSITION.i16PositionSource = POSITION_SOURCE_NONE;
			break;
		}
	}
	
	// Calculate new sin/cos
	SYSTEM.POSITION.mSinCosAngle.f16Sin = GFLIB_SinTlr(SYSTEM.POSITION.f16RotorAngle);
	SYSTEM.POSITION.mSinCosAngle.f16Cos = GFLIB_CosTlr(SYSTEM.POSITION.f16RotorAngle);
	
	//******************************************
	// Set Id, Iq currents
	//******************************************
	switch(SYSTEM.REGULATORS.i16CurrentSource)
	{
		case CURRENT_SOURCE_NONE:
		{
			// Set currents to 0
			SYSTEM.REGULATORS.m2IDQReq.f16D = FRAC16(0.0);
			SYSTEM.REGULATORS.m2IDQReq.f16Q = FRAC16(0.0);
			break;
		}
		case CURRENT_SOURCE_CONTROL_TORQUE:
		{
			// Torque control, set Iq for some torque
			// Id = 0
			SYSTEM.REGULATORS.m2IDQReq.f16D = FRAC16(0.0);
			// Torque ramp
			if(SYSTEM.RAMPS.f16TorqueRampDesiredValue != SYSTEM.RAMPS.f16TorqueRampActualValue)
			{
				SYSTEM.RAMPS.f16TorqueRampActualValue = GFLIB_Ramp16(SYSTEM.RAMPS.f16TorqueRampDesiredValue, SYSTEM.RAMPS.f16TorqueRampActualValue, &SYSTEM.RAMPS.Ramp16_Torque);
			}
			// Scale with some factor
			SYSTEM.REGULATORS.m2IDQReq.f16Q = mult(SYSTEM.RAMPS.f16TorqueRampActualValue, SYSTEM.MCTRL.f16TorqueFactor); 
			break;
		}
		case CURRENT_SOURCE_CONTROL_SPEED:
		{
			// Run speed PI
			// Speed loop			
			SYSTEM.REGULATORS.ui16SpeedRegCounter++;
			if(SYSTEM.REGULATORS.ui16SpeedRegInterval < SYSTEM.REGULATORS.ui16SpeedRegCounter)
			{
				SYSTEM.REGULATORS.ui16SpeedRegCounter = 0;
				
				//******************************
				// TODO: If Id and/or Iq is on limit, stop integrator
				//******************************
				// Do we have to do ramp?
				if(SYSTEM.RAMPS.f16SpeedRampActualValue != SYSTEM.RAMPS.f16SpeedRampDesiredValue)
				{
					SYSTEM.RAMPS.f16SpeedRampActualValue = GFLIB_Ramp16(SYSTEM.RAMPS.f16SpeedRampDesiredValue, SYSTEM.RAMPS.f16SpeedRampActualValue, &SYSTEM.RAMPS.Ramp16_Speed);						
				}
				
				f16SpeedErrorK = SYSTEM.RAMPS.f16SpeedRampActualValue - SYSTEM.POSITION.f16SpeedFiltered;
				// Check Iq
				// If Iq > Iqmax, check speed error
				// If error leads to lower Iq, OK
				// Else skip regulator
				// mi16SatFlagQ -> Iq regulator is saturated
				
				if(0 != SYSTEM.REGULATORS.i16SatFlagQ)
				{
					// Iq is saturated, decrease Iq
					if(FRAC16(0.0) < SYSTEM.REGULATORS.m2IDQReq.f16Q)
					{
						SYSTEM.REGULATORS.m2IDQReq.f16Q--;
					}
					else
					{
						SYSTEM.REGULATORS.m2IDQReq.f16Q++;
					}
				}
				else
				{
					// Iq available, run regulation
					SYSTEM.REGULATORS.i16SatFlagW = 0;
					SYSTEM.REGULATORS.m2IDQReq.f16Q = GFLIB_ControllerPIp(f16SpeedErrorK, &SYSTEM.REGULATORS.mudtControllerParamW, &SYSTEM.REGULATORS.i16SatFlagW);
				}
				// Set some small D value
				//SYSTEM.REGULATORS.m2IDQReq.f16D = FRAC16(0.01);
			}		
			break;
		}
		case CURRENT_SOURCE_CONTROL_MANUAL:
		{
			// Calculate ramp
			if(SYSTEM.RAMPS.f16AlignCurrentActualValue != SYSTEM.RAMPS.f16AlignCurrentDesiredValue)
			{
				SYSTEM.RAMPS.f16AlignCurrentActualValue = GFLIB_Ramp16(SYSTEM.RAMPS.f16AlignCurrentDesiredValue, SYSTEM.RAMPS.f16AlignCurrentActualValue, &SYSTEM.RAMPS.Ramp16_AlignCurrent);						
			}		
			// If SYSTEM_RUN_MANUAL, set Id
			if(SYSTEM_RUN_MANUAL)
			{
				SYSTEM.REGULATORS.m2IDQReq.f16D = SYSTEM.RAMPS.f16AlignCurrentActualValue;
				SYSTEM.REGULATORS.m2IDQReq.f16Q = FRAC16(0.0);
			}
			// Else Iq if we are running from sensor
			else if(SYSTEM_CALIBRATED && SYSTEM_RUN_SENSORED)
			{
				SYSTEM.REGULATORS.m2IDQReq.f16D = FRAC16(0.0);
				SYSTEM.REGULATORS.m2IDQReq.f16Q = SYSTEM.RAMPS.f16AlignCurrentActualValue;
			}
			// Else set Id, Iq to 0
			else
			{
				SYSTEM.REGULATORS.m2IDQReq.f16D = FRAC16(0.0);
				SYSTEM.REGULATORS.m2IDQReq.f16Q = FRAC16(0.0);
			}
			break;
		}
		case CURRENT_SOURCE_SENSORLESS_ALIGN:
		{
			// Set Id to align current
			if(SYSTEM.REGULATORS.m2IDQReq.f16D != SYSTEM.SENSORLESS.f16AlignCurrent)
			{
				SYSTEM.REGULATORS.m2IDQReq.f16D = GFLIB_Ramp16(SYSTEM.SENSORLESS.f16AlignCurrent, SYSTEM.REGULATORS.m2IDQReq.f16D, &SYSTEM.RAMPS.Ramp16_AlignCurrent);						
			}
			SYSTEM.REGULATORS.m2IDQReq.f16Q = FRAC16(0.0);
			break;
		}
		case CURRENT_SOURCE_SENSORLESS_ROTATE:
		{
			// Set Id to 0, Iq to start current
			SYSTEM.REGULATORS.m2IDQReq.f16D = FRAC16(0.0);		
			if(SYSTEM.REGULATORS.m2IDQReq.f16Q != SYSTEM.SENSORLESS.f16StartCurrent)
			{
				SYSTEM.REGULATORS.m2IDQReq.f16Q = GFLIB_Ramp16(SYSTEM.SENSORLESS.f16StartCurrent, SYSTEM.REGULATORS.m2IDQReq.f16Q, &SYSTEM.RAMPS.Ramp16_AlignCurrent);						
			}
			break;
		}
		default:
		{
			SYSTEM.REGULATORS.i16CurrentSource = CURRENT_SOURCE_NONE;
			break;
		}
	}
	
	//******************************************
	// Regulation
	//******************************************
	// Do only when PWM is enabled
	if(PWM_ENABLED)
	{
		// First store current voltage values for next iteration
		SYSTEM.MCTRL.m2UDQ_m.f16D = SYSTEM.MCTRL.m2UDQ.f16D;
		SYSTEM.MCTRL.m2UDQ_m.f16Q = SYSTEM.MCTRL.m2UDQ.f16Q;
		// Do PI regulation for D, Q current to get D, Q voltages
		// Store result in systemVariables.MOTOR.mudtDQInv
		// D
		// Error calculation
		mf16ErrorK = SYSTEM.REGULATORS.m2IDQReq.f16D - SYSTEM.MCTRL.m2IDQ.f16D;
		// Controller calculation
		SYSTEM.MCTRL.m2UDQ.f16D = GFLIB_ControllerPIp(mf16ErrorK, &SYSTEM.REGULATORS.mudtControllerParamId, &SYSTEM.REGULATORS.i16SatFlagD);
		// Q
		// Calculate error
		mf16ErrorK = SYSTEM.REGULATORS.m2IDQReq.f16Q - SYSTEM.MCTRL.m2IDQ.f16Q;
		/*
		 * TODO: Limit voltages to available 
		 */  
		// Controller calculation
		SYSTEM.MCTRL.m2UDQ.f16Q = GFLIB_ControllerPIp(mf16ErrorK, &SYSTEM.REGULATORS.mudtControllerParamIq, &SYSTEM.REGULATORS.i16SatFlagQ);
		
		//******************************************
		// Transformation to stationary frame, SVM, PWM
		//******************************************
		// We have Ud, Uq
		// Do inverse park
		MCLIB_ParkTrfInv(&SYSTEM.MCTRL.m2UAlphaBeta, &SYSTEM.MCTRL.m2UDQ, &SYSTEM.POSITION.mSinCosAngle);
		// Eliminate DC bus ripple
		//MCLIB_ElimDcBusRip(systemVariables.MOTOR.f16InvModeIndex, systemVariables.MOTOR.f16DCBusVoltage, &systemVariables.MOTOR.m2UAlphaBeta, &systemVariables.MOTOR.m2UAlphaBetaRippleElim);
		// Do SVM
		SYSTEM.MCTRL.SVMVoltageSector = MCLIB_SvmStd(&SYSTEM.MCTRL.m2UAlphaBeta, &SYSTEM.MCTRL.m3U_UVW);
		// Store PWMs
		SYSTEM.PWMValues.pwmSub_0_Channel_23_Value = SYSTEM.MCTRL.m3U_UVW.f16A;
		SYSTEM.PWMValues.pwmSub_1_Channel_23_Value = SYSTEM.MCTRL.m3U_UVW.f16B;
		SYSTEM.PWMValues.pwmSub_2_Channel_23_Value = SYSTEM.MCTRL.m3U_UVW.f16C;
		
		// Load new PWM values	
		ioctl(EFPWMA, EFPWM_CENTER_ALIGN_UPDATE_VALUE_REGS_COMPL_012, &SYSTEM.PWMValues);		
	}
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

// RS485 interrupts
#pragma interrupt saveall
void RX0_Full_ISR(void)
{
	unsigned int data;

	data = ioctl(SCI_0, SCI_GET_STATUS_REG, NULL);		// Clear RDRF flag
	data = ioctl(SCI_0, SCI_READ_DATA, NULL);			// Read data
	// Call state machine
	RS485_States_slave((UInt8)data);
}

#pragma interrupt saveall
void TX0_Empty_ISR(void)
{
	RS485_writeByte();
}

#pragma interrupt saveall
void PIT_0_ISR(void)
{
	ioctl(PIT_0, PIT_CLEAR_ROLLOVER_INT, NULL);
	checkSystemStates();
	// Decrease counters
	if(0 < SYSTEM.SENSORLESS.i16Counter)
	{
		SYSTEM.SENSORLESS.i16Counter --;
	}
}
