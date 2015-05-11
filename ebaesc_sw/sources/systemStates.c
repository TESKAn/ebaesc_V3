/*
 * systemStates.c
 *
 *  Created on: Mar 29, 2015
 *      Author: Jure
 */

#include "allincludes.h"

#pragma interrupt called
Int16 checkSystemStates(void)
{
	Int16 result = -1;
	Int16 i16Temp0 = 0;
	Int16 i16Temp1 = 0;
	Int16 i16Temp2 = 0;
	Int16 i16Temp3 = 0;
	Int16 i32Temp0 = 0;
	Frac16 f16Temp0 = 0;
	switch(SYSTEM.systemState)
	{
		case SYSTEM_WAKEUP:
		{
			// System has woken up
			// If not in debug
			if(!SYS_DEBUG_MODE)
			{
				// Set pwm input check state
				//ui8PWMMeasureStates = PWM_MEAS_INIT;
				//systemVariables.systemState = SYSTEM_INIT;
			}
			// Check state transition
			if(SYSTEM_IDLE == SYSTEM.i16StateTransition)
			{
				// Transition to idle state through init state
				SYSTEM.systemState = SYSTEM_INIT;				
			}

			
			break;
		}
		case SYSTEM_INIT:
		{
			// Do initialization here - wait for input signal, measure input throttle value
			// If not in debug
			if(!SYS_DEBUG_MODE)
			{
				// Check PWM in value
				switch(SYSTEM.PWMIN.i16PWMMeasureStates)
				{
					
					case PWM_MEAS_INIT:
					{
						if(PWM_MEAS_INITIAL_SAMPLES < SYSTEM.PWMIN.ui32PWMSamplesReceived)
						{
							SYSTEM.PWMIN.i16PWMMeasureStates = PWM_MEAS_DECIDE;
						}
						break;
					}
					case PWM_MEAS_DECIDE:
					{
						if(SYSTEM.PWMIN.i16PWMInMiddleValue < SYSTEM.PWMIN.i16PWMFiltered)
						{
							// Throttle is over half value
							SYSTEM.PWMIN.i16PWMMeasureStates = PWM_MEAS_HIGH;
						}		
						else
						{
							// Throttle is under half value
							// Use default value for PWM in high
							SYSTEM.PWMIN.i16PWMfullThrottle = SYSTEM.PWMIN.i16PWMInHighValRef;
							// Go to measure low state
							SYSTEM.PWMIN.i16PWMMeasureStates = PWM_MEAS_LOW;
						}			
						// Measure for x ms
						// Throttle has to stay in this position for specified amount of time
						SYSTEM.PWMIN.i16PWMMeasureTimer = SYSTEM.PWMIN.i16PWMInMeasureTime;
						break;
					}
					case PWM_MEAS_HIGH:
					{	
						// If value over high ref, 
						if(SYSTEM.PWMIN.i16PWMInHighValRef < SYSTEM.PWMIN.i16PWMFiltered)
						{
							// Throttle is in max position
							SYSTEM.PWMIN.i16PWMfullThrottle = SYSTEM.PWMIN.i16PWMFiltered;
							if(0 < SYSTEM.PWMIN.i16PWMMeasureTimer)
							{
								SYSTEM.PWMIN.i16PWMMeasureTimer--;
							}
						}
						else
						{
							// Throttle is below high ref
							// Check timer
							if(0 == SYSTEM.PWMIN.i16PWMMeasureTimer)
							{
								// Measure high is OK, go to measure low
								// Go to measure low state
								SYSTEM.PWMIN.i16PWMMeasureStates = PWM_MEAS_LOW;
							}
							// Measure for x ms
							// Throttle has to stay in this position for specified amount of time
							// Throttle not in high position for specified time, reset measurement
							// OR set measurement time for low
							SYSTEM.PWMIN.i16PWMMeasureTimer = SYSTEM.PWMIN.i16PWMInMeasureTime;
						}
						
						break;
					}
					case PWM_MEAS_LOW:
					{
						// If PWM under half value
						if(SYSTEM.PWMIN.i16PWMInMiddleValue > SYSTEM.PWMIN.i16PWMFiltered)
						{
							// Throttle is in min position
							SYSTEM.PWMIN.i16PWMinThrottle = SYSTEM.PWMIN.i16PWMFiltered;
							if(0 < SYSTEM.PWMIN.i16PWMMeasureTimer)
							{
								SYSTEM.PWMIN.i16PWMMeasureTimer--;
								// Check if timer reached 0
								if(0 == SYSTEM.PWMIN.i16PWMMeasureTimer)
								{
									// It did, calculate ON time
									SYSTEM.PWMIN.i16PWMoffThrottle = SYSTEM.PWMIN.i16PWMinThrottle + SYSTEM.PWMIN.i16PWMInOffZone;
									// Calculate throttle difference
									SYSTEM.PWMIN.i16PWMThrottleDifference = SYSTEM.PWMIN.i16PWMfullThrottle - SYSTEM.PWMIN.i16PWMoffThrottle;
									// Calculate frac multiplier
									SYSTEM.PWMIN.i16PWMFracMultiplier = (Int16)(32768 / SYSTEM.PWMIN.i16PWMThrottleDifference);
									// Go to idle state
									SYSTEM.systemState = SYSTEM_IDLE;	
								}
							}
						}
						else
						{
							// Throttle is not on min value, reset timer
							SYSTEM.PWMIN.i16PWMMeasureTimer = SYSTEM.PWMIN.i16PWMInMeasureTime;
						}
						break;
					}
					default:
					{
						SYSTEM.PWMIN.i16PWMMeasureStates = PWM_MEAS_INIT;
						break;
					}
				}				
			}
			
			// Check state transition
			if(SYSTEM_IDLE == SYSTEM.i16StateTransition)
			{
				// Go to idle state
				SYSTEM.systemState = SYSTEM_IDLE;								
			}
			break;
		}
		case SYSTEM_IDLE:
		{
			// If not in debug
			if(!SYS_DEBUG_MODE)
			{
				// Check throttle
				if(SYSTEM.PWMIN.i16PWMoffThrottle < SYSTEM.PWMIN.i16PWMFiltered)
				{
					// Throttle over off value, start motor
					//SYSTEM_GOTO_ACTIVE = 1;
				}
				else
				{
					//SYSTEM_GOTO_ACTIVE = 0;
				}
			}
			switch(SYSTEM.i16StateTransition)
			{
				case SYSTEM_RUN:
				{	
					// Set PWM values
					SYSTEM.PWMValues.pwmSub_0_Channel_23_Value = FRAC16(0.5);
					SYSTEM.PWMValues.pwmSub_1_Channel_23_Value = FRAC16(0.5);
					SYSTEM.PWMValues.pwmSub_2_Channel_23_Value = FRAC16(0.5);
					ioctl(EFPWMA, EFPWM_CENTER_ALIGN_UPDATE_VALUE_REGS_COMPL_012, &SYSTEM.PWMValues);
					// Enable PWM outputs
					ioctl(EFPWMA, EFPWM_SET_OUTPUTS_ENABLE, EFPWM_SUB0_PWM_A|EFPWM_SUB0_PWM_B|EFPWM_SUB1_PWM_A|EFPWM_SUB1_PWM_B|EFPWM_SUB2_PWM_A|EFPWM_SUB2_PWM_B);

					// Check - run from sensor or sensorless?
					if(SYSTEM_CALIBRATED && SYSTEM_RUN_SENSORED)
					{
						// Run from sensor
						SYSTEM.POSITION.i16PositionSource = POSITION_SOURCE_MULTIPLE;
						if(CONTROL_SPEED)
						{
							SYSTEM.REGULATORS.i16CurrentSource = CURRENT_SOURCE_CONTROL_SPEED;
							// Set default values
							SYSTEM.RAMPS.f16SpeedRampDesiredValue = FRAC16(0.0);
							SYSTEM.RAMPS.f16SpeedRampActualValue = FRAC16(0.0);
						}
						else if(CONTROL_TORQUE)
						{
							SYSTEM.REGULATORS.i16CurrentSource = CURRENT_SOURCE_CONTROL_TORQUE;
							// Set default values
							SYSTEM.RAMPS.f16TorqueRampActualValue = FRAC16(0.0);
							SYSTEM.RAMPS.f16TorqueRampDesiredValue = FRAC16(0.0);
						}
						else
						{
							// No source, manual
							SYSTEM.POSITION.i16PositionSource = POSITION_SOURCE_MANUAL;
							SYSTEM.REGULATORS.i16CurrentSource = CURRENT_SOURCE_CONTROL_MANUAL;
							// Make sure that we will not rotate
							SYSTEM_RUN_MANUAL_CW = 0;
							SYSTEM_RUN_MANUAL_CCW = 0;
						}
						SYSTEM.systemState = SYSTEM_RUN;	
						// Enable regulators
						PWM_ENABLED = 1;
					}
					else if(CONTROL_SPEED || CONTROL_TORQUE)
					{
						// Run sensorless only for speed or torque modes
						// Start open loop mode
						SYSTEM.POSITION.i16PositionSource = POSITION_SOURCE_SENSORLESS_ALIGN;
						SYSTEM.REGULATORS.i16CurrentSource = CURRENT_SOURCE_SENSORLESS_ALIGN;
						// Set initial angle increase
						SYSTEM.RAMPS.f16OLSpeedRampActualValue = SYSTEM.SENSORLESS.f16InitialAngleIncrease;
						// Set desired angle increase
						SYSTEM.RAMPS.f16OLSpeedRampDesiredValue = SYSTEM.SENSORLESS.f16MaxAngleIncrease;
						// Set time for align
						SYSTEM.SENSORLESS.i16Counter = SYSTEM.SENSORLESS.i16AlignTime;
						// Mark no BEMF
						SENSORLESS_BEMF_ON = 0;
						// Go to run
						SYSTEM.systemState = SYSTEM_RUN;
						// Enable regulators
						PWM_ENABLED = 1;
					}
					else if(CONTROL_MANUAL)
					{
						// Manual control
						SYSTEM.POSITION.i16PositionSource = POSITION_SOURCE_MANUAL;
						SYSTEM.REGULATORS.i16CurrentSource = CURRENT_SOURCE_CONTROL_MANUAL;
						// Set D, Q currents to 0
						SYSTEM.RAMPS.f16AlignCurrentActualValue = FRAC16(0.0);
						SYSTEM.RAMPS.f16AlignCurrentDesiredValue = FRAC16(0.0);
						// Set Id, Iq to 0
						SYSTEM.REGULATORS.m2IDQReq.f16D = FRAC16(0.0);
						SYSTEM.REGULATORS.m2IDQReq.f16Q = FRAC16(0.0);
						SYSTEM_RUN_MANUAL = 1;
						SYSTEM_RUN_MANUAL_CW = 0;
						SYSTEM_RUN_MANUAL_CCW = 0;
						// Go to run
						SYSTEM.systemState = SYSTEM_RUN;
						// Enable regulators
						PWM_ENABLED = 1;
					}
					else
					{
						// No option, abort
						// Turn off PWMs
						ioctl(EFPWMA, EFPWM_SET_OUTPUTS_DISABLE, EFPWM_SUB0_PWM_A|EFPWM_SUB0_PWM_B|EFPWM_SUB1_PWM_A|EFPWM_SUB1_PWM_B|EFPWM_SUB2_PWM_A|EFPWM_SUB2_PWM_B);
						
					}
					break;
				}
				case SYSTEM_CALIBRATE:
				{						
					// Set position/current source
					SYSTEM.POSITION.i16PositionSource = POSITION_SOURCE_MANUAL;
					SYSTEM.REGULATORS.i16CurrentSource = CURRENT_SOURCE_CONTROL_MANUAL;
					// Set Id, Iq to 0
					SYSTEM.REGULATORS.m2IDQReq.f16D = FRAC16(0.0);
					SYSTEM.REGULATORS.m2IDQReq.f16Q = FRAC16(0.0);
					// Use align current
					SYSTEM.RAMPS.f16AlignCurrentDesiredValue = SYSTEM.SENSORLESS.f16AlignCurrent;
					SYSTEM.RAMPS.f16AlignCurrentActualValue = FRAC16(0.0);
					// Mark run manually
					SYSTEM_RUN_MANUAL = 1;
					// Mark system not calibrated
					SYSTEM_CALIBRATED = 0;
					// Do not use sensor
					SYSTEM_RUN_SENSORED = 0;
					// Do not move rotor
					SYSTEM_RUN_MANUAL_CW = 0;
					SYSTEM_RUN_MANUAL_CCW = 0;
					// Set PWM values
					SYSTEM.PWMValues.pwmSub_0_Channel_23_Value = FRAC16(0.5);
					SYSTEM.PWMValues.pwmSub_1_Channel_23_Value = FRAC16(0.5);
					SYSTEM.PWMValues.pwmSub_2_Channel_23_Value = FRAC16(0.5);
					ioctl(EFPWMA, EFPWM_CENTER_ALIGN_UPDATE_VALUE_REGS_COMPL_012, &SYSTEM.PWMValues);
					// Enable PWM outputs
					ioctl(EFPWMA, EFPWM_SET_OUTPUTS_ENABLE, EFPWM_SUB0_PWM_A|EFPWM_SUB0_PWM_B|EFPWM_SUB1_PWM_A|EFPWM_SUB1_PWM_B|EFPWM_SUB2_PWM_A|EFPWM_SUB2_PWM_B);
					// Enable regulators
					PWM_ENABLED = 1;
					SYSTEM.systemState = SYSTEM_CALIBRATE;		
					SYSTEM.CALIBRATION.i16CalibrationState = CALIBRATE_INIT;
					break;
				}
				case SYSTEM_PARKROTOR:
				{						
					// Set PWM values
					SYSTEM.PWMValues.pwmSub_0_Channel_23_Value = FRAC16(0.5);
					SYSTEM.PWMValues.pwmSub_1_Channel_23_Value = FRAC16(0.5);
					SYSTEM.PWMValues.pwmSub_2_Channel_23_Value = FRAC16(0.5);
					ioctl(EFPWMA, EFPWM_CENTER_ALIGN_UPDATE_VALUE_REGS_COMPL_012, &SYSTEM.PWMValues);
					// Enable PWM outputs
					ioctl(EFPWMA, EFPWM_SET_OUTPUTS_ENABLE, EFPWM_SUB0_PWM_A|EFPWM_SUB0_PWM_B|EFPWM_SUB1_PWM_A|EFPWM_SUB1_PWM_B|EFPWM_SUB2_PWM_A|EFPWM_SUB2_PWM_B);
					// Enable regulators
					PWM_ENABLED = 1;
					//systemVariables.ui16CalibrationState = CALIBRATE_INIT;
					//ui16ParkState = PARK_INIT;
					SYSTEM.systemState = SYSTEM_PARKROTOR;
					break;
				}
				case SYSTEM_SPINNINGROTOR:
				{						
					// Set PWM values
					SYSTEM.PWMValues.pwmSub_0_Channel_23_Value = FRAC16(0.5);
					SYSTEM.PWMValues.pwmSub_1_Channel_23_Value = FRAC16(0.5);
					SYSTEM.PWMValues.pwmSub_2_Channel_23_Value = FRAC16(0.5);
					ioctl(EFPWMA, EFPWM_CENTER_ALIGN_UPDATE_VALUE_REGS_COMPL_012, &SYSTEM.PWMValues);
					// Enable PWM outputs
					ioctl(EFPWMA, EFPWM_SET_OUTPUTS_ENABLE, EFPWM_SUB0_PWM_A|EFPWM_SUB0_PWM_B|EFPWM_SUB1_PWM_A|EFPWM_SUB1_PWM_B|EFPWM_SUB2_PWM_A|EFPWM_SUB2_PWM_B);		
					
					
					SYSTEM.systemState = SYSTEM_SPINNINGROTOR;
					
					//SYSTEM_MAN_ROTATE = 1;
	
					//systemVariables.f16IdReq = FRAC16(0.05);
					//systemVariables.f16IqReq = FRAC16(0.0);
					break;
				}
				case SYSTEM_RESET:
				{
					SYSTEM.systemState = SYSTEM_RESET;
					break;
				}
			}	
			break;
		}
		case SYSTEM_RUN:
		{
			switch(SYSTEM.i16StateTransition)
			{
				case SYSTEM_RESET:
				{
					SYSTEM.systemState = SYSTEM_RESET;
					break;
				}
			}
			break;
		}
		
		case SYSTEM_CALIBRATE:
		{
			// Mark calibrating
			switch(SYSTEM.CALIBRATION.i16CalibrationState)
			{
				case CALIBRATE_INIT:
				{
					SYSTEM.CALIBRATION.i16CalibrationState = CALIBRATE_START;
					SYSTEM.POSITION.f16RotorAngle = FRAC16(0.0);
					// Calculate sin/cos
					SYSTEM.POSITION.mSinCosAngle.f16Sin = GFLIB_SinTlr(SYSTEM.POSITION.f16RotorAngle);
					SYSTEM.POSITION.mSinCosAngle.f16Cos = GFLIB_CosTlr(SYSTEM.POSITION.f16RotorAngle);
					
					// Zero array
					for(i16Temp0 = 0; i16Temp0 < 4096; i16Temp0++)
					{
						SYSTEM.CALIBRATION.f16CalibrationArray[i16Temp0] = FRAC16(0.0);
					}
					// Mark waiting for zero cross
					SYS_CAL_ZERO_CROSSED = 0;
					
					// 1 ms interval, wait for 4 sec for initial alignment
					SYSTEM.CALIBRATION.i16Counter = 4000;
					break;
				}
				case CALIBRATE_START:
				{
					SYSTEM.CALIBRATION.i16Counter--;
					if(0 == SYSTEM.CALIBRATION.i16Counter)
					{
						SYSTEM.CALIBRATION.i16Counter = 5;
						//SYSTEM.CALIBRATION.i16CalibrationState = CALIBRATE_CW;
						SYSTEM.CALIBRATION.i16CalibrationState = CALIBRATE_FIND_ZERO_CROSS;
						//i16CurrentPolePair = systemVariables.POSITION.ui16FilteredPositionIndex;	
					}

					break;
				}
				case CALIBRATE_FIND_ZERO_CROSS:
				{
					if(!SYS_CAL_ZERO_CROSSED)
					{
						// Turn motor fast until we get to some high value
						if(3000 > SYSTEM.POSITION.i16SensorIndexFiltered)
						{
							SYSTEM.POSITION.f16RotorAngle += 40;
						}
						else
						{
							// Value is high enough
							// Move motor until we get to next pole pair
							SYS_CAL_GOTO_NEXT_POLE = 1;
							// Mark zero is crossed
							SYS_CAL_ZERO_CROSSED = 1;
						}
					}
					else
					{
						// Wait until motor moves to next pole
						if(!SYS_CAL_GOTO_NEXT_POLE)
						{
							// We are at next electrical pole
							// Check that position is below half mech angle
							// So that we are over zero cross
							if(1000 > SYSTEM.POSITION.i16SensorIndexFiltered)
							{
								// It is, this is our first pole. Verify it and set index
								SYSTEM.CALIBRATION.i16CurrentPolePair = 0;
								SYSTEM.CALIBRATION.i16CalibrationState = CALIBRATE_VERIFY_POLE;
								// Set settling time
								SYSTEM.CALIBRATION.i16Counter = 1000;
							}
							else
							{
								// Else go to next pole
								SYS_CAL_GOTO_NEXT_POLE = 1;
							}
						}
					}
					break;
				}
				case CALIBRATE_VERIFY_POLE:
				{
					// Wait for motor to settle to final position
					SYSTEM.CALIBRATION.i16Counter--;
					if(0 == SYSTEM.CALIBRATION.i16Counter)
					{
						// This is the spot
						// Store pole position
						SYSTEM.CALIBRATION.i16PolePairArray[SYSTEM.CALIBRATION.i16CurrentPolePair] = SYSTEM.POSITION.i16SensorIndexFiltered;
						// Pole over 0?
						if(0 < SYSTEM.CALIBRATION.i16CurrentPolePair)
						{
							// Check if it is the final pole
							if(SYSTEM.CALIBRATION.i16PolePairArray[SYSTEM.CALIBRATION.i16CurrentPolePair] < SYSTEM.CALIBRATION.i16PolePairArray[SYSTEM.CALIBRATION.i16CurrentPolePair - 1])	
							{
								// Current value is lower than previous - we went over mechanical pole so we are back at first pole
								// So go to final step for calibration
								//systemVariables.ui16CalibrationState = CALIBRATE_CALCULATE_VALUES;
								SYSTEM.CALIBRATION.i16CalibrationState = CALIBRATE_FIND_AD_IN_MAX;
								SYSTEM.CALIBRATION.i16MaxSensorIndex = 0;
								SYSTEM.CALIBRATION.i16MinSensorIndex = 4096;
								
								// And mark number of pole pairs
								SYSTEM.CALIBRATION.i16MotorPolePairs = (UInt16)SYSTEM.CALIBRATION.i16CurrentPolePair;
							}
							else
							{
								// Mark going to next pole
								SYS_CAL_GOTO_NEXT_POLE = 1;
								// Increase pole count
								SYSTEM.CALIBRATION.i16CurrentPolePair ++;	
								SYSTEM.CALIBRATION.i16CalibrationState = CALIBRATE_WAIT_NEXT_POLE;							
							}
						}
						else
						{
							// Else it is first pole
							// Mark going to next pole
							SYS_CAL_GOTO_NEXT_POLE = 1;
							// Increase pole count
							SYSTEM.CALIBRATION.i16CurrentPolePair ++;	
							SYSTEM.CALIBRATION.i16CalibrationState = CALIBRATE_WAIT_NEXT_POLE;
						}
					}
					
					break;
				}
				case CALIBRATE_WAIT_NEXT_POLE:
				{
					// Wait to move to next pole
					if(!SYS_CAL_GOTO_NEXT_POLE)
					{
						// We are at next pole, go to verify
						SYSTEM.CALIBRATION.i16CalibrationState = CALIBRATE_VERIFY_POLE;
						// Set settling time
						SYSTEM.CALIBRATION.i16Counter = 1000;
					}
					break;
				}
				case CALIBRATE_FIND_AD_IN_MAX:
				{
					// Spin back until we get max value
					if(3000 > SYSTEM.POSITION.i16SensorIndexFiltered)
					{
						// Rotate back
						if(SYSTEM.POSITION.i16SensorIndexFiltered > 20)
						{
							SYSTEM.POSITION.f16RotorAngle -= 5;
						}
						else
						{
							SYSTEM.POSITION.f16RotorAngle --;
						}
						// Store values
						if(SYSTEM.POSITION.i16SensorIndexFiltered > SYSTEM.CALIBRATION.i16MaxSensorIndex) SYSTEM.CALIBRATION.i16MaxSensorIndex = SYSTEM.POSITION.i16SensorIndexFiltered;
						if(SYSTEM.POSITION.i16SensorIndexFiltered < SYSTEM.CALIBRATION.i16MinSensorIndex) SYSTEM.CALIBRATION.i16MinSensorIndex = SYSTEM.POSITION.i16SensorIndexFiltered;
					}
					else
					{
						// Rotate little further back back
						SYSTEM.POSITION.f16RotorAngle -= 1000;
						SYSTEM.CALIBRATION.i16CalibrationState = CALIBRATE_FIND_AD_IN_MIN;
						SYSTEM.CALIBRATION.i16Counter = 1000;
					}
					break;
				}
				case CALIBRATE_FIND_AD_IN_MIN:
				{
					// Wait
					SYSTEM.CALIBRATION.i16Counter--;
					if(0 == SYSTEM.CALIBRATION.i16Counter)
					{
						SYSTEM.CALIBRATION.i16Counter++;
						// Spin forward until we get min value
						if(500 < SYSTEM.POSITION.i16SensorIndexFiltered)
						{
							// Rotate back
							SYSTEM.POSITION.f16RotorAngle ++;
							// Store values
							if(SYSTEM.POSITION.i16SensorIndexFiltered > SYSTEM.CALIBRATION.i16MaxSensorIndex) SYSTEM.CALIBRATION.i16MaxSensorIndex = SYSTEM.POSITION.i16SensorIndexFiltered;
							if(SYSTEM.POSITION.i16SensorIndexFiltered < SYSTEM.CALIBRATION.i16MinSensorIndex) SYSTEM.CALIBRATION.i16MinSensorIndex = SYSTEM.POSITION.i16SensorIndexFiltered;
						}
						else
						{
							// Go to recalculation
							SYSTEM.CALIBRATION.i16CalibrationState = CALIBRATE_CALCULATE_VALUES;
						}							
					}

					break;
				}
				case CALIBRATE_CALCULATE_VALUES:
				{
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
								SYSTEM.CALIBRATION.f16CalibrationArray[i16Temp1] = SYSTEM.CALIBRATION.f16CalibrationArray[1023] + f16Temp0;
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
					i16Temp1 = SYSTEM.CALIBRATION.i16PolePairArray[SYSTEM.CALIBRATION.i16MotorPolePairs - 1];
					// Get increment
					i16Temp2 = 4096 - i16Temp1 + SYSTEM.CALIBRATION.i16PolePairArray[0] - (4096 - SYSTEM.CALIBRATION.i16MaxSensorIndex) - SYSTEM.CALIBRATION.i16MinSensorIndex;
					// f16Temp0 stores angle delta in current pole
					i32Temp0 = 65536 / i16Temp2;
					f16Temp0 = (Frac16)i32Temp0;
					// Set first value
					SYSTEM.CALIBRATION.f16CalibrationArray[i16Temp1] = FRAC16(-1.0);
					// Fill until last
					for(i16Temp0 = i16Temp1 + 1; i16Temp0 < SYSTEM.CALIBRATION.i16MaxSensorIndex; i16Temp0 ++)	
					{
						SYSTEM.CALIBRATION.f16CalibrationArray[i16Temp0] = SYSTEM.CALIBRATION.f16CalibrationArray[i16Temp0] + f16Temp0;
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
					SYSTEM.CALIBRATION.i16CalibrationState = CALIBRATE_INIT;
					SYSTEM.systemState = SYSTEM_RESET;
					SYSTEM.REGULATORS.m2IDQReq.f16D = FRAC16(0.0);
					SYSTEM.REGULATORS.m2IDQReq.f16Q = FRAC16(0.0);
					SYSTEM_CALIBRATED = 1;
					SYSTEM.POSITION.i16PositionSource = POSITION_SOURCE_NONE;
					SYSTEM.REGULATORS.i16CurrentSource = CURRENT_SOURCE_NONE;
					break;
				}
				default:
				{
					SYSTEM.CALIBRATION.i16CalibrationState = CALIBRATE_INIT;
					break;
				}
			}

			break;
		}
		case SYSTEM_FAULT:
		{
			ioctl(EFPWMA, EFPWM_SET_OUTPUTS_DISABLE, EFPWM_SUB0_PWM_A|EFPWM_SUB0_PWM_B|EFPWM_SUB1_PWM_A|EFPWM_SUB1_PWM_B|EFPWM_SUB2_PWM_A|EFPWM_SUB2_PWM_B);
			break;
		}
		case SYSTEM_RESET:
		{
			// Stop motor
			StopMotor();
			// Change state
			SYSTEM.systemState = SYSTEM_WAKEUP;
			break;
		}
		
		/*
		case SYSTEM_ACTIVE:
		{
			// PWMs active
			// If goto run
			if(SYSTEM_GOTO_RUN)
			{
				// Check - run from sensor or sensorless?
				if(SYSTEM_CALIBRATED && RUN_WSENSOR)
				{
					RUN_FROMSENSOR = 1;
					RUN_ADC_EOS_ALG = 0;	
					// Set desired speed for sensor mode
					systemVariables.RAMPS.mf16SpeedRampDesiredValue = FRAC16(0.0);										
				}
				else
				{
					// Mark run
					RUN_FROMSENSOR = 0;
					RUN_ADC_EOS_ALG = 1;
					// Mark align rotor
					ALIGN = 1;			
					// Set desired speed for sensorless mode
					systemVariables.RAMPS.mf16SpeedRampDesiredValue = systemVariables.REFERENCES.f16StartupSpeed;		
				}
				// Do not check merge
				MERGE_CHECK=0;
				// Observer is inactive
				OBSERVER_ACTIVE = 0;
				// System not running from BEMF
				RUNNING_FROM_BEMF = 0;
				// Align time is set in InitMotorVars
				InitMotorVars();
			

				// Clear FOC lost flag
				RESTART_MOTOR_FOC_LOST = 0;
				systemVariables.systemState = SYSTEM_RUN;
				SYSTEM_GOTO_RUN = 0;				
			}
			break;
		}
		case SYSTEM_RUN:
		{
			// Check PWM input signal
			if(!SYS_DEBUG_MODE)
			{
				// Counter is reset in PWM input interrupt
				ui16PWMTimeout--;
				if(0 == ui16PWMTimeout)
				{
					// No PWM input for 1 sec, stop motor
					// Stop motor
					StopMotor();
					// Reset PWM in counter
					systemVariables.PWMTIMING.ui32PWMSamplesReceived = 0;
					// Go to fault state
					systemVariables.systemState = SYSTEM_PWM_IN_LOST;
				}
				else
				{
					// Set motor speed
					// Is PWM in under off throttle?
					if(i16PWMoffThrottle > systemVariables.PWMTIMING.i16PWMFiltered)
					{
						// Stop motor
						StopMotor();
						// Go to idle state
						systemVariables.systemState = SYSTEM_IDLE;
					}
					else
					{
						// Else calculate speed
						// Current PWM value
						i16Temp0 = systemVariables.PWMTIMING.i16PWMFiltered - i16PWMinThrottle;
						// Multiply to get frac representation
						i16Temp0 *= i16PWMFracMultiplier;
						// Store to req speed
						systemVariables.RAMPS.mf16SpeedRampDesiredValue = i16Temp0;
					}
				}
			}
				
			if(RUNNING_FROM_BEMF)
			{
				//runningMotor = 1;
			}
			// Check driver status register
			// Fault?
			if(DRV8301_stat_reg1.bit.FAULT)
			{
				// Fault occured
				// Disable PWMs
				//ioctl(PWM, PWM_OUTPUT_PAD, PWM_DISABLE);
				// Go to fault state
				//systemVariables.systemState = SYSTEM_FAULT_DRV8301;
			}
			// FOC sync lost?
			if(RESTART_MOTOR_FOC_LOST)
			{
				// Reconfigure driver
				if(0 != InitDRV8301(0,31,1))
				{
					// Driver did not initialize properly
					DRV8301_CONFIGURED = 0;
					systemVariables.systemState = SYSTEM_FAULT;
				}
				else
				{
					DRV8301_CONFIGURED = 1;
					RESTART_MOTOR_FOC_LOST = 0;
					systemVariables.systemState = SYSTEM_FOC_LOST_TIMEOUT;
					systemVariables.ui16FOCRestartTimeout = 250;					
				}	
			}
			break;
		}
		case SYSTEM_FAULT:
		{

			break;
		}			
		case SYSTEM_RESET:
		{
			// Stop motor
			StopMotor();			
			// Go to wakeup state
			systemVariables.systemState = SYSTEM_WAKEUP;//SYSTEM_IDLE;
			break;
		}
		case SYSTEM_RESTARTING:
		{
			// Check if lost sync is set and we have to restart the motor
			switch(systemVariables.ui16MotorRestartStates)
			{
				case RESTART_INIT:
				{
					// Stop motor algo
					StopMotor();
					// Mark check if motor is stopped - BEMF
					CHECK_MOTOR_STOPPED;
					systemVariables.ui16MotorRestartStates = RESTART_CHECKSTOPPED;		
					break;
				}
				case RESTART_CHECKSTOPPED:
				{
					// flag CHECK_IF_MOTOR_STOPPED set in macro CHECK_MOTOR_STOPPED - wait for it to be cleared
					if(!CHECK_IF_MOTOR_STOPPED)
					{
						// What is the state of motor
						if(MOTOR_STATIONARY)
						{
							// Motor is not rotating
							systemVariables.ui16MotorRestartStates = RESTART_RUN;
						}
						else
						{
							// Motor is rotating
							// Brake it
							if(BRAKE_MOTOR_SHORT)
							{
								systemVariables.ui16MotorRestartStates = RESTART_BRAKE_MOTOR_SHORT;
							}
							else if(BRAKE_MOTOR_FOC)
							{
								systemVariables.ui16MotorRestartStates = RESTART_BRAKE_MOTOR_SYNC;
							}
						}
					}				
					break;
				}
				case RESTART_BRAKE_MOTOR_SHORT:
				{
					switch(systemVariables.ui16BrakeWithShortState)
					{
						case BRAKE_SHORT_INIT:
						{
							// Set lower PWMs to 1
							// Switch to GPIO
							ioctl( GPIO_A, GPIO_CLEAR_PIN, BIT_0 | BIT_1 | BIT_2|BIT_3 | BIT_4 | BIT_5);
							ioctl( GPIO_A, GPIO_SETAS_GPIO, BIT_0 | BIT_1 | BIT_2|BIT_3 | BIT_4 | BIT_5);
							ioctl( GPIO_A, GPIO_CLEAR_PIN, BIT_0 | BIT_1 | BIT_2|BIT_3 | BIT_4 | BIT_5);
							// GPIO A0 to 1 (Low MOSFET phase W)
							ioctl( GPIO_A, GPIO_SET_PIN, BIT_0);
							systemVariables.ui16BrakeWithShortState = BRAKE_SHORT_BRAKING;
							// Mark check for phase I
							CHECK_MOTOR_STOPPED_I;
							break;
						}
						case BRAKE_SHORT_BRAKING:
						{	
							if(!CHECK_IF_MOTOR_STOPPED_I)
							{
								// Check motor status
								if(MOTOR_STATIONARY)
								{
									// Motor has stopped, stop braking and go to finish
									ioctl( GPIO_A, GPIO_CLEAR_PIN, BIT_0 | BIT_1 | BIT_2|BIT_3 | BIT_4 | BIT_5);
									systemVariables.ui16BrakeWithShortState = BRAKE_MOTOR_END;
								}
								else
								{
									// Mark check for phase I
									CHECK_MOTOR_STOPPED_I;								
								}
							}
							break;
						}
						case BRAKE_MOTOR_END:
						{
							// Motor is not rotating
							// Set pins for peripheral
							ioctl( GPIO_A, GPIO_SETAS_PERIPHERAL, BIT_0 | BIT_1 | BIT_2|BIT_3 | BIT_4 | BIT_5);
							// Go to restart run state
							systemVariables.ui16MotorRestartStates = RESTART_RUN;
							// Set restart brake init state
							systemVariables.ui16BrakeWithShortState = BRAKE_SHORT_INIT;
							break;
						}
						default:
						{
							systemVariables.ui16BrakeWithShortState = BRAKE_SHORT_INIT;
							break;
						}
					}
					break;
				}
				case RESTART_BRAKE_MOTOR_SYNC:
				{
					break;
				}
				case RESTART_RUN:
				{
					// Run motor
					// Go to idle system state
					systemVariables.systemState = SYSTEM_IDLE;
					// Mark go to active state
					SYSTEM_GOTO_ACTIVE = 1;
					break;
				}
				default:
				{
					systemVariables.ui16MotorRestartStates = RESTART_INIT;
					break;
				}
			}
			
			break;
		}
		case SYSTEM_FAULT_DRV8301:
		{
			// Check fault
			// Clear fault
			// Restart motor
			break;
		}
		case SYSTEM_FAULT_RESET:
		{
			// Disable PWMs
			ioctl(PWM, PWM_OUTPUT_PAD, PWM_DISABLE);
			// Mark dont rotor
			ALIGN = 0;
			// Do not check merge
			MERGE_CHECK=0;
			// Observer is inactive
			OBSERVER_ACTIVE = 0;
			// System not running from BEMF
			RUNNING_FROM_BEMF = 0;			
			// Disable algo
			RUN_ADC_EOS_ALG = 0;		
			// Go to fault state		
			systemVariables.systemState = SYSTEM_FAULT;		
			break;
		}		
		case SYSTEM_BLOCKEXEC:
		{
			ioctl(SYS, SYS_SOFTWARE_RESET, NULL);
			break;
		}
		case SYSTEM_FOC_LOST_TIMEOUT:
		{
			// Wait for some ms before attempting restart
			if(systemVariables.ui16FOCRestartTimeout > 0)
			{
				systemVariables.ui16FOCRestartTimeout--;
			}
			else
			{
				// Restart motor
				systemVariables.systemState = SYSTEM_RESTARTING;
				systemVariables.ui16MotorRestartStates = RESTART_INIT;
			}
			break;
		}
		case SYSTEM_PWM_IN_LOST:
		{
			// Check PWM in
			if(PWM_MEAS_INITIAL_SAMPLES < systemVariables.PWMTIMING.ui32PWMSamplesReceived)
			{
				systemVariables.systemState = SYSTEM_WAKEUP;
			}			
			break;
		}
		case SYSTEM_CALIBRATE:
		{
			// Mark calibrating
			SYSTEM_MAN_ROTATE = 1;
			switch(systemVariables.ui16CalibrationState)
			{
				case CALIBRATE_INIT:
				{
					systemVariables.ui16CalibrationState = CALIBRATE_START;
					systemVariables.f16IdReq = FRAC16(0.05);// systemVariables.REFERENCES.f16AlignCurrent;
					systemVariables.f16IqReq = FRAC16(0.0);
					systemVariables.MOTOR.f16PositionForced = FRAC16(0.0);
					
					// Zero array
					for(i16Temp0 = 0; i16Temp0 < 1024; i16Temp0++)
					{
						f16CalibrationArray[i16Temp0] = FRAC16(0.0);
					}
					// Mark waiting for zero cross
					SYS_CAL_ZERO_CROSSED = 0;
					
					// 1 ms interval, wait for 4 sec for initial alignment
					ui16CalibrationCount = 4000;
					break;
				}
				case CALIBRATE_START:
				{
					ui16CalibrationCount--;
					if(0 == ui16CalibrationCount)
					{
						ui16CalibrationCount = 5;
						ui16CalibrationState = 0;
						ui16DiscardCounter = 50;
						systemVariables.ui16CalibrationState = CALIBRATE_CW;
						systemVariables.ui16CalibrationState = CALIBRATE_FIND_ZERO_CROSS;
						//i16CurrentPolePair = systemVariables.POSITION.ui16FilteredPositionIndex;	
					}

					break;
				}
				case CALIBRATE_CW:
				{				
					switch(ui16CalibrationState)
					{
						case 0:
						{
							// Wait so we have some room from top and bottom
							if((800 > systemVariables.POSITION.ui16PositionIndex)&&(200 < systemVariables.POSITION.ui16PositionIndex))
							{
								ui16CalibrationState = 1;
								ui16SetIndex = systemVariables.POSITION.ui16PositionIndex;
								// Add some offset
								ui16SetIndex = ui16SetIndex + 50;
							}
							break;
						}
						case 1:
						{
							// Wait for index to go over offset
							if(ui16SetIndex < systemVariables.POSITION.ui16PositionIndex)
							{
								ui16SetIndex = ui16SetIndex - 50;
								ui16CalibrationState = 2;
							}	
							break;
						}
						case 2:
						{
							// Wait until current index is under stored index
							if(ui16SetIndex > systemVariables.POSITION.ui16PositionIndex)
							{
								ui16CalibrationState = 3;	
							}
							break;
						}
						case 3:
						{
							// Wait to go over initial reading
							if(ui16SetIndex < systemVariables.POSITION.ui16PositionIndex)
							{
								// Go to next stage
								ui16CalibrationState = 0;
								ui16DiscardCounter = 50;
								systemVariables.ui16CalibrationState = CALIBRATE_CCW;
							}
							break;
						}
					}
					
					break;
				}
				case CALIBRATE_CCW:
				{					
					switch(ui16CalibrationState)
					{
						case 0:
						{
							// Wait so we have some room from top and bottom
							if((800 > systemVariables.POSITION.ui16PositionIndex)&&(200 < systemVariables.POSITION.ui16PositionIndex))
							{
								ui16CalibrationState = 1;
								ui16SetIndex = systemVariables.POSITION.ui16PositionIndex;
								// Add some offset
								ui16SetIndex = ui16SetIndex - 50;								
							}
							break;
						}
						case 1:
						{
							// Wait for index to go over offset
							if(ui16SetIndex > systemVariables.POSITION.ui16PositionIndex)
							{
								ui16SetIndex = ui16SetIndex + 50;
								ui16CalibrationState = 2;
							}	
							break;
						}	
						case 2:
						{
							// Wait until current index is over stored index
							if(ui16SetIndex < systemVariables.POSITION.ui16PositionIndex)
							{

								ui16CalibrationState = 3;	
							}
							break;
						}
						case 3:
						{
							// Wait to go under initial reading
							if(ui16SetIndex > systemVariables.POSITION.ui16PositionIndex)
							{
								// Go to end
								ui16CalibrationState = 0;
								systemVariables.ui16CalibrationState = CALIBRATE_CALCULATE;
							}
							break;
						}
					}
					break;
				}
				
				case CALIBRATE_FIND_ZERO_CROSS:
				{
					if(!SYS_CAL_ZERO_CROSSED)
					{
						// Turn motor fast until we get to some high value
						if(930 > systemVariables.POSITION.ui16FilteredPositionIndex)
						{
							//i16CurrentPolePair = 
							systemVariables.MOTOR.f16PositionForced += 40;
						}
						else
						{
							// Value is high enough
							// Move motor until we get to next pole pair
							SYS_CAL_GOTO_NEXT_POLE = 1;
							// Mark zero is crossed
							SYS_CAL_ZERO_CROSSED = 1;
						}
					}
					else
					{
						// Wait until motor moves to next pole
						if(!SYS_CAL_GOTO_NEXT_POLE)
						{
							// We are at next electrical pole
							// Check that position is below half mech angle
							// So that we are over zero cross
							if(500 > systemVariables.POSITION.ui16FilteredPositionIndex)
							{
								// It is, this is our first pole. Verify it and set index
								i16CurrentPolePair = 0;
								systemVariables.ui16CalibrationState = CALIBRATE_VERIFY_POLE;
								// Set settling time
								ui16CalibrationCount = 1000;
							}
							else
							{
								// Else go to next pole
								SYS_CAL_GOTO_NEXT_POLE = 1;
							}
						}
					}
					break;
				}
				case CALIBRATE_VERIFY_POLE:
				{
					// Wait for motor to settle to final position
					ui16CalibrationCount--;
					if(0 == ui16CalibrationCount)
					{
						// This is the spot
						// Store pole position
						i16PolePairArray[i16CurrentPolePair] = (int16_t)systemVariables.POSITION.ui16FilteredPositionIndex;
						// Pole over 0?
						if(0 < i16CurrentPolePair)
						{
							// Check if it is the final pole
							if(i16PolePairArray[i16CurrentPolePair] < i16PolePairArray[i16CurrentPolePair - 1])	
							{
								// Current value is lower than previous - we went over mechanical pole so we are back at first pole
								// So go to final step for calibration
								//systemVariables.ui16CalibrationState = CALIBRATE_CALCULATE_VALUES;
								systemVariables.ui16CalibrationState = CALIBRATE_FIND_AD_IN_MAX;
								i16MaxSensorIndex = 0;
								i16MinSensorIndex = 1024;
								
								// And mark number of pole pairs
								ui16MotorPolePairs = (uint16_t)i16CurrentPolePair;
							}
							else
							{
								// Mark going to next pole
								SYS_CAL_GOTO_NEXT_POLE = 1;
								// Increase pole count
								i16CurrentPolePair ++;	
								systemVariables.ui16CalibrationState = CALIBRATE_WAIT_NEXT_POLE;							
							}
						}
						else
						{
							// Else it is first pole
							// Mark going to next pole
							SYS_CAL_GOTO_NEXT_POLE = 1;
							// Increase pole count
							i16CurrentPolePair ++;	
							systemVariables.ui16CalibrationState = CALIBRATE_WAIT_NEXT_POLE;
						}
					}
					
					break;
				}
				case CALIBRATE_WAIT_NEXT_POLE:
				{
					// Wait to move to next pole
					if(!SYS_CAL_GOTO_NEXT_POLE)
					{
						// We are at next pole, go to verify
						systemVariables.ui16CalibrationState = CALIBRATE_VERIFY_POLE;
						// Set settling time
						ui16CalibrationCount = 1000;
					}
					break;
				}
				case CALIBRATE_FIND_AD_IN_MAX:
				{
					// Spin back until we get max value
					if(950 > systemVariables.POSITION.ui16FilteredPositionIndex)
					{
						// Rotate back
						if(systemVariables.POSITION.ui16FilteredPositionIndex > 20)
						{
							systemVariables.MOTOR.f16PositionForced -= 5;
						}
						else
						{
							systemVariables.MOTOR.f16PositionForced --;
						}
						// Store values
						if((int16_t)systemVariables.POSITION.ui16FilteredPositionIndex > i16MaxSensorIndex) i16MaxSensorIndex = (int16_t)systemVariables.POSITION.ui16FilteredPositionIndex;
						if((int16_t)systemVariables.POSITION.ui16FilteredPositionIndex < i16MinSensorIndex) i16MinSensorIndex = (int16_t)systemVariables.POSITION.ui16FilteredPositionIndex;
					}
					else
					{
						// Rotate little further back back
						systemVariables.MOTOR.f16PositionForced -= 1000;
						systemVariables.ui16CalibrationState = CALIBRATE_FIND_AD_IN_MIN;
						ui16CalibrationCount = 1000;
					}
					break;
				}
				case CALIBRATE_FIND_AD_IN_MIN:
				{
					// Wait
					ui16CalibrationCount--;
					if(0 == ui16CalibrationCount)
					{
						ui16CalibrationCount++;
						// Spin forward until we get min value
						if(20 < systemVariables.POSITION.ui16FilteredPositionIndex)
						{
							// Rotate back
							systemVariables.MOTOR.f16PositionForced ++;
							// Store values
							if((int16_t)systemVariables.POSITION.ui16FilteredPositionIndex > i16MaxSensorIndex) i16MaxSensorIndex = (int16_t)systemVariables.POSITION.ui16FilteredPositionIndex;
							if((int16_t)systemVariables.POSITION.ui16FilteredPositionIndex < i16MinSensorIndex) i16MinSensorIndex = (int16_t)systemVariables.POSITION.ui16FilteredPositionIndex;
						}
						else
						{
							// Go to recalculation
							systemVariables.ui16CalibrationState = CALIBRATE_CALCULATE_VALUES;
						}							
					}

					break;
				}
				case CALIBRATE_CALCULATE_VALUES:
				{
					// Interpolate values between poles
					// i16Temp0,1,2
					// Use temp0 for counting pole pairs 											
					for(i16Temp0 = 0; i16Temp0 < (int16_t)ui16MotorPolePairs; i16Temp0 ++)
					{
						// Temp1 stores current index in calibration array
						i16Temp1 = i16PolePairArray[i16Temp0];	
						// Temp2 stores number of samples for current pole pair
						if((i16Temp0 + 1) == (int16_t)ui16MotorPolePairs)	
						{
							// If last pole pair
							i16Temp2 = 1024 - i16PolePairArray[i16Temp0] + i16PolePairArray[0];
						}
						else
						{
							i16Temp2 = i16PolePairArray[i16Temp0 + 1] - i16PolePairArray[i16Temp0];
						}						
						// f16Temp0 stores angle delta in current pole
						i32Temp0 = 65536 / i16Temp2;
						f16Temp0 = (Frac16)i32Temp0;		
						// Set first value
						f16CalibrationArray[i16Temp1] = FRAC16(-1.0);
						// Calculate the rest
						for(i16Temp3 = 0; i16Temp3 < i16Temp2; i16Temp3++)
						{
							// Move to next index
							i16Temp1 ++;
							// Wrap to 0
							i16Temp1 = i16Temp1 & 1023;
							// Add difference
							if(0 == i16Temp1)
							{
								f16CalibrationArray[i16Temp1] = f16CalibrationArray[1023] + f16Temp0;
							}
							else
							{
								f16CalibrationArray[i16Temp1] = f16CalibrationArray[i16Temp1 - 1] + f16Temp0;
							}							
						}						
					}
					// Recalculate last pole value
					//i16MaxSensorIndex
					//i16MinSensorIndex
					// Store index for zero angle
					i16Temp1 = i16PolePairArray[ui16MotorPolePairs - 1];
					// Get increment
					i16Temp2 = 1024 - i16Temp1 + i16PolePairArray[0] - (1024 - i16MaxSensorIndex) - i16MinSensorIndex;
					// f16Temp0 stores angle delta in current pole
					i32Temp0 = 65536 / i16Temp2;
					f16Temp0 = (Frac16)i32Temp0;
					// Set first value
					f16CalibrationArray[i16Temp1] = FRAC16(-1.0);
					// Fill until last
					for(i16Temp0 = i16Temp1 + 1; i16Temp0 < i16MaxSensorIndex; i16Temp0 ++)	
					{
						f16CalibrationArray[i16Temp0] = f16CalibrationArray[i16Temp0] + f16Temp0;
					}
					// Fill until last					
					for(i16Temp0 = i16MaxSensorIndex; i16Temp0 < 1024; i16Temp0 ++)	
					{
						f16CalibrationArray[i16Temp0] = f16CalibrationArray[i16Temp0 - 1];
					}				
					// Now for lower part of values
					// Fill first value
					f16CalibrationArray[0] = f16CalibrationArray[1023];
					// Fill first part
					for(i16Temp0 = 1; i16Temp0 < i16MinSensorIndex; i16Temp0 ++)	
					{
						f16CalibrationArray[i16Temp0] = f16CalibrationArray[i16Temp0 - 1];
					}
					// Fill the rest
					for(i16Temp0 = i16MinSensorIndex; i16Temp0 < i16PolePairArray[0]; i16Temp0 ++)	
					{
						f16CalibrationArray[i16Temp0] = f16CalibrationArray[i16Temp0 - 1] + f16Temp0;
					}								
					systemVariables.ui16CalibrationState = CALIBRATE_INIT;
					systemVariables.systemState = SYSTEM_RESET;
					systemVariables.f16IdReq = FRAC16(0.0);
					systemVariables.f16IqReq = FRAC16(0.0);
					SYSTEM_CALIBRATED = 1;
					SYSTEM_MAN_ROTATE = 0;
					break;
				}
				
				
				case CALIBRATE_CALCULATE:
				{
					// Get first non - zero value, counting from 0 up
					for(i16Temp0 = 0; i16Temp0 < 1024; i16Temp0++)
					{
						if(FRAC16(0.0) != f16CalibrationArray[i16Temp0])
						{
							f16Temp0 = f16CalibrationArray[i16Temp0];
							while(0 < i16Temp0)
							{
								i16Temp0--;
								f16CalibrationArray[i16Temp0] = f16Temp0;
							}
							f16CalibrationArray[0] = f16Temp0;
							i16Temp0 = 1024;
						}
					}		
					// Get last non - zero value
					for(i16Temp0 = 1023; i16Temp0 > 0; i16Temp0--)
					{
						if(FRAC16(0.0) != f16CalibrationArray[i16Temp0])
						{
							f16Temp0 = f16CalibrationArray[i16Temp0];
							while(1022 > i16Temp0)
							{
								i16Temp0++;
								f16CalibrationArray[i16Temp0] = f16Temp0;
							}
							f16CalibrationArray[1023] = f16Temp0;
							i16Temp0 = 0;
						}
					}		
					systemVariables.ui16CalibrationState = CALIBRATE_INIT;
					systemVariables.systemState = SYSTEM_RESET;
					systemVariables.f16IdReq = FRAC16(0.0);
					systemVariables.f16IqReq = FRAC16(0.0);
					SYSTEM_CALIBRATED = 1;
					SYSTEM_MAN_ROTATE = 0;
					break;
				}
				default:
				{
					systemVariables.ui16CalibrationState = CALIBRATE_INIT;
					break;
				}
			}
			break;
		}
		case SYSTEM_PARKROTOR:
		{
			switch(ui16ParkState)
			{
				case PARK_INIT:
				{
					SYSTEM_MAN_ROTATE = 1;
					systemVariables.ui16CalibrationState = CALIBRATE_START;
					systemVariables.f16IdReq = FRAC16(0.05);// systemVariables.REFERENCES.f16AlignCurrent;
					systemVariables.f16IqReq = FRAC16(0.0);
					systemVariables.MOTOR.f16PositionForced = f16CalibrationArray[systemVariables.POSITION.ui16PositionIndex];
					ui16ParkState = PARK_MOVE;
					break;
				}
				case PARK_MOVE:
				{
					// If position is not park position
					if(ui16ParkPosition != systemVariables.POSITION.ui16PositionIndex)
					{
						if(ui16ParkPosition > systemVariables.POSITION.ui16PositionIndex)
						{
							i16Temp0 = (int16_t)ui16ParkPosition - (int16_t)systemVariables.POSITION.ui16PositionIndex;
							if(i16Temp0 > 10) systemVariables.MOTOR.f16PositionForced -= 10;
							else systemVariables.MOTOR.f16PositionForced--;
						}
						else
						{
							i16Temp0 = (int16_t)systemVariables.POSITION.ui16PositionIndex - (int16_t)ui16ParkPosition;
							if(i16Temp0 > 10) systemVariables.MOTOR.f16PositionForced += 10;
							else systemVariables.MOTOR.f16PositionForced++;
						}
						
					}
					else
					{
						ui16ParkState = PARK_END;
					}
					break;
				}
				case PARK_END:
				{
					ui16ParkState = PARK_INIT;
					systemVariables.systemState = SYSTEM_RESET;
					systemVariables.f16IdReq = FRAC16(0.0);
					systemVariables.f16IqReq = FRAC16(0.0);
					SYSTEM_MAN_ROTATE = 0;
					break;
				}
				
				default:
				{
				
					ui16ParkState = PARK_INIT;
					break;
				}
			}
			break;
		}
		case SYSTEM_SPINNINGROTOR:
		{
			break;
		}*/
		default:
		{
			//systemVariables.systemState = SYSTEM_WAKEUP;
			break;
		}
	}
	return 0;
}
