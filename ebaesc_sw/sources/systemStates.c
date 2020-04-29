/*
 * systemStates.c
 *
 *  Created on: Mar 29, 2015
 *      Author: Jure
 */

#include "allincludes.h"

Int16 checkSystemStates(void)
{
	switch(SYSTEM.systemState)
	{
		case SYSTEM_WAKEUP:
		{
			SystemWakeupState();	
			break;
		}
		case SYSTEM_INIT:
		{
			switch(SYSTEM.PWMIN.i16PWMMeasureStates)
			{			
				case PWM_MEAS_INIT:
				{
					break;
				}
				case PWM_MEAS_DECIDE:
				{
					break;
				}
				case PWM_MEAS_HIGH:
				{
					break;
				}
				case PWM_MEAS_LOW:
				{
					break;
				}		
			}
			
			SystemInitState();
			break;
		}
		case SYSTEM_IDLE:
		{
			SystemIdleState();
			break;
		}
		case SYSTEM_RUN:
		{
			SystemRunState();
			break;
		}		
		case SYSTEM_FAULT:
		{
			SystemFaultState();
			break;
		}		
		case SYSTEM_RESET:
		{
			SystemResetState();
			break;
		}		
		case SYSTEM_RESTARTING:
		{
			SystemRestartingState();
			break;
		}		
		case SYSTEM_FAULT_DRV8301:
		{
			SystemFaultDRV83xxState();
			break;
		}		
		case SYSTEM_FAULT_RESET:
		{
			SystemFaultResetState();
			break;
		}		
		case SYSTEM_BLOCKEXEC:
		{
			SystemBlockExecState();
			break;
		}	
		case SYSTEM_FOC_LOST_TIMEOUT:
		{
			SystemFOCLostTimeoutState();
			break;
		}	
		case SYSTEM_PWM_IN_LOST:
		{
			SystemPWMInLostState();
			break;
		}		
		case SYSTEM_CALIBRATE:
		{
			SystemCalibrateState();
			break;
		}
		case SYSTEM_PARKROTOR:
		{
			SystemParkRotorState();
			break;
		}
		case SYSTEM_MEAS_RPHA:
		{
			SystemMeasureRPHAState();
			break;
		}
		case SYSTEM_MEAS_LPHA:
		{
			SystemMeasureLPHAState();
			break;
		}
		case SYSTEM_FAULT_OCEVENT:
		{
			SystemFaultOCEventState();
			break;
		}
		default:
		{
			SYSTEM.systemState = SYSTEM_WAKEUP;
			break;
		}
	}
	return 0;
}

Int16 SystemWakeupState()
{
	Int16 i16DriverState = 0;
	// Transit out of?
	if(SYSTEM.i16StateTransition != SYSTEM.systemState)
	{
		if(0 == SYSTEM.ui16StateTransitionTimeout)
		{
			SystemStateTransition();
		}
		else
		{
			SYSTEM.ui16StateTransitionTimeout--;
		}		
	}
	else
	{
		// Reset driver and wait for sequence to end
		i16DriverState = SystemResetDriver();
		if(0 == i16DriverState)
		{
			if(0 == SYSTEM.DRIVERSTATE.i8DriverFaultCount)
			{			
				SYSTEM.i16StateTransition = SYSTEM_INIT;
			}
			else
			{
				SYSTEM.i16StateTransition = SYSTEM_FAULT_DRV8301;
			}
		}	
	}
	return 0;
}

Int16 SystemInitState()
{
	float fTemp;
	// Transit out of?
	if(SYSTEM.i16StateTransition != SYSTEM.systemState)
	{
		if(0 == SYSTEM.ui16StateTransitionTimeout)
		{
			SystemStateTransition();
		}
		else
		{
			SYSTEM.ui16StateTransitionTimeout--;
		}	
	}
	else
	{
		// Do initialisation here - wait for input signal, measure input throttle value
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
					// Is throttle in high position
					if(SYSTEM.PWMIN.i16PWMInMiddleValue < SYSTEM.PWMIN.i16PWMFiltered)
					{
						// Throttle is over half value
						SYSTEM.PWMIN.i16PWMMeasureStates = PWM_MEAS_HIGH;
					}			
					// Measure for x ms
					// Throttle has to stay in this position for specified amount of time
					SYSTEM.PWMIN.i16PWMMeasureTimer = SYSTEM.PWMIN.i16PWMInMeasureTime;
					break;
				}
				case PWM_MEAS_HIGH:
				{				
					// If measuring less than measuring time
					if(0 < SYSTEM.PWMIN.i16PWMMeasureTimer)
					{
						SYSTEM.PWMIN.i16PWMMeasureTimer--;
						// If value over high ref, 
						if(SYSTEM.PWMIN.i16PWMInHighValRef < SYSTEM.PWMIN.i16PWMFiltered)
						{
							// Throttle is in max position
							SYSTEM.PWMIN.i16PWMfullThrottle = SYSTEM.PWMIN.i16PWMFiltered;
							COMMDataStruct.REGS.i16PWMMax = SYSTEM.PWMIN.i16PWMfullThrottle;	
						}
						else
						{
							// Wait with countdown until throttle is above high val ref
							SYSTEM.PWMIN.i16PWMMeasureTimer = SYSTEM.PWMIN.i16PWMInMeasureTime;						
						}					
					}				
					else
					{
						SYSTEM.PWMIN.i16PWMMeasureStates = PWM_MEAS_LOW;
						SYSTEM.PWMIN.i16PWMMeasureTimer = SYSTEM.PWMIN.i16PWMInMeasureTime;
					}
					break;
				}
				case PWM_MEAS_LOW:
				{
					// If measuring less than measuring time
					if(0 < SYSTEM.PWMIN.i16PWMMeasureTimer)
					{
						SYSTEM.PWMIN.i16PWMMeasureTimer--;
						if(SYSTEM.PWMIN.i16PWMInLowValRef > SYSTEM.PWMIN.i16PWMFiltered)
						{
							// Throttle is in min position
							SYSTEM.PWMIN.i16PWMinThrottle = SYSTEM.PWMIN.i16PWMFiltered;
	
							// Check if timer reached 0
							if(0 == SYSTEM.PWMIN.i16PWMMeasureTimer)
							{
								// It did, calculate ON value
								SYSTEM.PWMIN.i16PWMoffThrottle = SYSTEM.PWMIN.i16PWMinThrottle + SYSTEM.PWMIN.i16PWMInOffZone;
								// Calculate throttle difference
								SYSTEM.PWMIN.i16PWMThrottleDifference = SYSTEM.PWMIN.i16PWMfullThrottle - SYSTEM.PWMIN.i16PWMoffThrottle;
								// Calculate factors for RPM
								SYSTEM.PWMIN.fPWMFactor = (float)(COMMDataStruct.REGS.i16MaxRPM - COMMDataStruct.REGS.i16MinRPM);
								SYSTEM.PWMIN.fPWMFactor = SYSTEM.PWMIN.fPWMFactor / (float)(SYSTEM.PWMIN.i16PWMThrottleDifference);
								
								SYSTEM.PWMIN.fPWMOffset = (float)(SYSTEM.PWMIN.i16PWMoffThrottle);
								SYSTEM.PWMIN.fPWMOffset = SYSTEM.PWMIN.fPWMOffset * SYSTEM.PWMIN.fPWMFactor;
								SYSTEM.PWMIN.fPWMOffset = SYSTEM.PWMIN.fPWMOffset - (float)(COMMDataStruct.REGS.i16MinRPM);
	
								// Set right data
								COMMDataStruct.REGS.i16PWMMin = SYSTEM.PWMIN.i16PWMinThrottle;
								COMMDataStruct.REGS.i16ZeroSpeedPWM = SYSTEM.PWMIN.i16PWMInOffZone;
								// Go to idle state
								SYSTEM.i16StateTransition = SYSTEM_IDLE;	
								COMMDataStruct.REGS.ui8UsePWMIN = 1;
								SYSTEM.PWMIN.i16PWMMeasureStates = PWM_MEAS_INIT;
							}						
						}
						else
						{
							// Throttle is not on min value, reset timer
							SYSTEM.PWMIN.i16PWMMeasureTimer = SYSTEM.PWMIN.i16PWMInMeasureTime;						
						}
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
		else
		{
			SYSTEM.i16StateTransition = SYSTEM_IDLE;
		}
	}
	return 0;
}

Int16 SystemIdleState()
{
	// Transit out of?
	if(SYSTEM.i16StateTransition != SYSTEM.systemState)
	{
		if(0 == SYSTEM.ui16StateTransitionTimeout)
		{
			SystemStateTransition();
		}
		else
		{
			SYSTEM.ui16StateTransitionTimeout--;
		}	
	}
	else
	{
		// Use PWM?
		if(1 == COMMDataStruct.REGS.ui8UsePWMIN)
		{
			// If PWM over zero speed, go to run mode.
			if(SYSTEM.PWMIN.i16PWMFiltered > SYSTEM.PWMIN.i16PWMoffThrottle)
			{
				COMMDataStruct.REGS.ui8Armed = 1;
			}
		}	
		// Check comm regs
		if(1 == COMMDataStruct.REGS.ui8Armed)
		{
			// Control system speed
			CONTROL_SPEED = 1;
			SYSTEM.i16StateTransition = SYSTEM_RUN;
		}
	}
	return 0;
}

Int16 SystemRunState()
{
	//Int16 i16Temp0 = 0;
	UInt16 ui16Temp = 0;
	Frac16 f16Temp0 = 0;
	//Int32 i32Temp0 = 0;
	float fTemp;
	// Transit out of?
	if(SYSTEM.i16StateTransition != SYSTEM.systemState)
	{
		if(0 == SYSTEM.ui16StateTransitionTimeout)
		{
			// Stop if spinning
			if((FRAC16(0.01) < SYSTEM.POSITION.f16SpeedFiltered)||(FRAC16(-0.01) > SYSTEM.POSITION.f16SpeedFiltered))
			{
				SYSTEM.RAMPS.f16SpeedRampDesiredValue = FRAC16(0.0);
				SYSTEM.RAMPS.f16TorqueRampDesiredValue = FRAC16(0.0);			
				if(SYSTEM_FAULT_DRV8301 == SYSTEM.i16StateTransition)
				{
					StopMotor();
					SystemStateTransition();
				}		
				else if(2 == COMMDataStruct.REGS.ui8Armed)
				{
					COMMDataStruct.REGS.ui8Armed = 0;
					StopMotor();
					SystemStateTransition();
				}
				else if(3 == COMMDataStruct.REGS.ui8Armed)
				{
					// BEMF error - go to restart
					COMMDataStruct.REGS.ui8Armed = 1;
					SYSTEM.SENSORLESS.ui8BemfObserverErrorCount = 0;
					
					StopMotor();
					SystemStateTransition();
				}
				else if(!SYSTEM_CALIBRATED && !SYSTEM_RUN_SENSORED)
				{
					// Sensorless running - go out of transition
					StopMotor();
					SystemStateTransition();
				}
			}
			else
			{
				StopMotor();
				SystemStateTransition();
			}
		}
		else
		{
			SYSTEM.ui16StateTransitionTimeout--;
		}	
	}
	else
	{
		// Running?
		if(1 == COMMDataStruct.REGS.ui8Armed)
		{
			// Use PWM for speed?
			if(1 == COMMDataStruct.REGS.ui8UsePWMIN)
			{
				// Current PWM over minimum (=OFF) PWM?
				if(SYSTEM.PWMIN.i16PWMFiltered > SYSTEM.PWMIN.i16PWMoffThrottle)
				{
					// Do only if startup is complete
					if(POSITION_SOURCE_MULTIPLE == SYSTEM.POSITION.i16PositionSource)
					{				
						// Calculate speed based on input PWM value
						fTemp = (float)SYSTEM.PWMIN.i16PWMFiltered;
						fTemp = fTemp * SYSTEM.PWMIN.fPWMFactor;
						fTemp = fTemp - SYSTEM.PWMIN.fPWMOffset;
						// fTemp = physical RPM that we want
						// Calculate electrical RPM that we want
						fTemp *= (float)SYSTEM.CALIBRATION.i16MotorPolePairs;
						// Calculate frac value
						// Divide by max el RPM for value 1, multiply with 32768 -> 32768/480000
						fTemp *= 0.06826666666666666666666666666667f;				
						// Check sign
						if(0 == COMMDataStruct.REGS.ui8ReverseRotation)
						{
							SYSTEM.RAMPS.f16SpeedRampDesiredValue = (Frac16)fTemp;
						}
						else
						{
							SYSTEM.RAMPS.f16SpeedRampDesiredValue = -(Frac16)fTemp;
						}	
					}
				}
				else
				{
					// Go out of run mode
					SYSTEM.RAMPS.f16SpeedRampDesiredValue = FRAC16(0.0);
					COMMDataStruct.REGS.ui8Armed = 0;
				}
			}
			else
			{
				// Limit
				if(COMMDataStruct.REGS.i16SetRPM > COMMDataStruct.REGS.i16MaxRPM)
				{
					COMMDataStruct.REGS.i16SetRPM = COMMDataStruct.REGS.i16MaxRPM;
				}
				else if(COMMDataStruct.REGS.i16SetRPM < COMMDataStruct.REGS.i16MinRPM)
				{
					COMMDataStruct.REGS.i16SetRPM = COMMDataStruct.REGS.i16MinRPM;
				}
				fTemp = (float)COMMDataStruct.REGS.i16SetRPM;			
				fTemp *= (float)SYSTEM.CALIBRATION.i16MotorPolePairs;
				
				// Calculate frac value
				// Divide by max el RPM for value 1, multiply with 32768 -> 32768/480000
				fTemp *= 0.06826666666666666666666666666667f;				
				// Check sign
				if(0 == COMMDataStruct.REGS.ui8ReverseRotation)
				{
					SYSTEM.RAMPS.f16SpeedRampDesiredValue = (Frac16)fTemp;
				}
				else
				{
					SYSTEM.RAMPS.f16SpeedRampDesiredValue = -(Frac16)fTemp;
				}	
			}
		
			// Park rotor?
			if(1 == COMMDataStruct.REGS.ui8Park)
			{
				f16Temp0 = abs_s(SYSTEM.POSITION.f16SpeedFiltered);
				// If speed below limit, park
				if(FRAC16(0.01) > f16Temp0)
				{
					SYSTEM.i16StateTransition = SYSTEM_PARKROTOR;
				}
				// Else slow down
				else
				{
					SYSTEM.RAMPS.f16SpeedRampDesiredValue = FRAC16(0.0);
				}
			}
		}
		else if(CONTROL_MANUAL)
		{
			
		}
		else
		{
			// Go out of run mode
			SYSTEM.i16StateTransition = SYSTEM_RESET;
			SYSTEM.RAMPS.f16SpeedRampDesiredValue = FRAC16(0.0);
		}
		// BEMF error, restart?
		if(SYSTEM.SENSORLESS.ui8MaxBemfObserverErrorCount < SYSTEM.SENSORLESS.ui8BemfObserverErrorCount)
		{
			// Go out of run mode
			SYSTEM.i16StateTransition = SYSTEM_FAULT;
			SYSTEM.RAMPS.f16SpeedRampDesiredValue = FRAC16(0.0);
			COMMDataStruct.REGS.ui8Armed = 3;
			COMMDataStruct.REGS.i16SetRPM = COMMDataStruct.REGS.i16MinRPM; 
			// Timeout 250 ms
			SYSTEM.ui16StateTransitionTimeout = 250;
		}
		// BEMF ON error, restart?
		if(SYSTEM.SENSORLESS.ui16MaxBEMFONErrors < SYSTEM.SENSORLESS.ui16BEMFONError)
		{
			// Go out of run mode
			SYSTEM.i16StateTransition = SYSTEM_FAULT;
			SYSTEM.RAMPS.f16SpeedRampDesiredValue = FRAC16(0.0);
			COMMDataStruct.REGS.ui8Armed = 3;
			COMMDataStruct.REGS.i16SetRPM = COMMDataStruct.REGS.i16MinRPM; 		
			// Timeout 250 ms
			SYSTEM.ui16StateTransitionTimeout = 250;
		}	
		// DQ merge error, restart?
		if(ERROR_DQ_MERGE)
		{
			// Go out of run mode
			SYSTEM.i16StateTransition = SYSTEM_FAULT;
			SYSTEM.RAMPS.f16SpeedRampDesiredValue = FRAC16(0.0);
			COMMDataStruct.REGS.ui8Armed = 3;
			COMMDataStruct.REGS.i16SetRPM = COMMDataStruct.REGS.i16MinRPM; 		
			// Timeout 250 ms
			SYSTEM.ui16StateTransitionTimeout = 250;
		}
		
		// When in run mode, check driver status for errors
		if(0 != DRV8301.StatReg1.FAULT)
		{
			// Shut down PWMs, go out of run mode into fault mode
			SYSTEM.i16StateTransition = SYSTEM_FAULT_DRV8301;
			COMMDataStruct.REGS.ui8Armed = 0;
			/*
			// Mark fault
			COMMDataStruct.REGS.ui16Errors |= RS485ERROR_FAULT;
			// Check FETs
			if(0 != DRV8301.StatReg1.FETHA_OC) COMMDataStruct.REGS.ui16Errors |= RS485ERROR_FETHA;
			if(0 != DRV8301.StatReg1.FETLA_OC) COMMDataStruct.REGS.ui16Errors |= RS485ERROR_FETLA;
			if(0 != DRV8301.StatReg1.FETHB_OC) COMMDataStruct.REGS.ui16Errors |= RS485ERROR_FETHB;
			if(0 != DRV8301.StatReg1.FETLB_OC) COMMDataStruct.REGS.ui16Errors |= RS485ERROR_FETLB;
			if(0 != DRV8301.StatReg1.FETHC_OC) COMMDataStruct.REGS.ui16Errors |= RS485ERROR_FETHC;
			if(0 != DRV8301.StatReg1.FETLC_OC) COMMDataStruct.REGS.ui16Errors |= RS485ERROR_FETLC;
			*/
		}
		
		// Driver fault?
		if(0 != SYSTEM.DRIVERSTATE.i8DriverFault)
		{
			SYSTEM.i16StateTransition = SYSTEM_FAULT_DRV8301;
			COMMDataStruct.REGS.ui8Armed = 0;
		}
		
		// Check undervoltage
		fTemp = SYSTEM.SIVALUES.fUIn * 1000.0f;
		ui16Temp = (UInt16)fTemp;
		if(COMMDataStruct.REGS.ui16VoltageCutoff > ui16Temp)
		{
			SYSTEM.ADC.i16UnderVoltageEvents++;
			if(SYSTEM.ADC.i16MaxUnderVoltageEvents < SYSTEM.ADC.i16UnderVoltageEvents)
			{
				COMMDataStruct.REGS.ui8UVError = 1;
				SYSTEM.i16StateTransition = SYSTEM_FAULT;			
			}
		}
		else if(0 < SYSTEM.ADC.i16UnderVoltageEvents)
		{
			SYSTEM.ADC.i16UnderVoltageEvents--;
		}
	}	
	return 0;
}

Int16 SystemFaultState()
{
	ioctl(EFPWMA, EFPWM_SET_OUTPUTS_DISABLE, EFPWM_SUB0_PWM_A|EFPWM_SUB0_PWM_B|EFPWM_SUB1_PWM_A|EFPWM_SUB1_PWM_B|EFPWM_SUB2_PWM_A|EFPWM_SUB2_PWM_B);

	if(SYSTEM.i16StateTransition != SYSTEM.systemState)
	{
		// Transit out of?
		if(0 == SYSTEM.ui16StateTransitionTimeout)
		{
			StopMotor();
			SystemStateTransition();
		}
		else
		{
			SYSTEM.ui16StateTransitionTimeout--;
		}			
	}
	else
	{
		// Reset?
		if(1 == COMMDataStruct.REGS.ui8Reset)
		{
			COMMDataStruct.REGS.ui8Reset = 0;
			SYSTEM.i16StateTransition = SYSTEM_WAKEUP;
		}
	}
	return 0;
}

// Reset the system
Int16 SystemResetState()
{
	// Transit out of?
	if(SYSTEM.i16StateTransition != SYSTEM.systemState)
	{
		// Transit out of?
		if(0 == SYSTEM.ui16StateTransitionTimeout)
		{
			StopMotor();
			SystemStateTransition();
		}
		else
		{
			SYSTEM.ui16StateTransitionTimeout--;
		}	
	}
	else
	{
		SYSTEM.i16StateTransition = SYSTEM_WAKEUP;
	}
	return 0;
}

Int16 SystemRestartingState()
{
	// Transit out of?
	if(SYSTEM.i16StateTransition != SYSTEM.systemState)
	{
		// Transit out of?
		if(0 == SYSTEM.ui16StateTransitionTimeout)
		{
			StopMotor();
			SystemStateTransition();
		}
		else
		{
			SYSTEM.ui16StateTransitionTimeout--;
		}	
	}
	else
	{
		SYSTEM.i16StateTransition = SYSTEM_WAKEUP;
	}
	return 0;
}

Int16 SystemFaultDRV83xxState()
{
	Int16 i16Temp = 0;
	// Transit out of?
	if(SYSTEM.i16StateTransition != SYSTEM.systemState)
	{
		// Transit out of?
		if(0 == SYSTEM.ui16StateTransitionTimeout)
		{
			// Try restarting driver
			i16Temp = SystemResetDriver();
			if(0 == i16Temp)
			{
				// Change state
				SystemStateTransition();
			}	
		}
		else
		{
			SYSTEM.ui16StateTransitionTimeout--;
		}	
	}
	else
	{
		// Reset?
		if(1 == COMMDataStruct.REGS.ui8Reset)
		{
			COMMDataStruct.REGS.ui8Reset = 0;
			SYSTEM.i16StateTransition = SYSTEM_WAKEUP;
		}	
	}
	return 0;
}

Int16 SystemFaultResetState()
{
	// Transit out of?
	if(SYSTEM.i16StateTransition != SYSTEM.systemState)
	{
		// Transit out of?
		if(0 == SYSTEM.ui16StateTransitionTimeout)
		{
			StopMotor();
			SystemStateTransition();
		}
		else
		{
			SYSTEM.ui16StateTransitionTimeout--;
		}	
	}
	else
	{
		SYSTEM.i16StateTransition = SYSTEM_WAKEUP;
	}
	return 0;
}

Int16 SystemBlockExecState()
{
	// Transit out of?
	if(SYSTEM.i16StateTransition != SYSTEM.systemState)
	{
		if(0 == SYSTEM.ui16StateTransitionTimeout)
		{
			StopMotor();
			SystemStateTransition();
		}
		else
		{
			SYSTEM.ui16StateTransitionTimeout--;
		}	
	}
	else
	{
		SYSTEM.i16StateTransition = SYSTEM_WAKEUP;
	}
	return 0;
}

Int16 SystemFOCLostTimeoutState()
{
	// Transit out of?
	if(SYSTEM.i16StateTransition != SYSTEM.systemState)
	{
		if(0 == SYSTEM.ui16StateTransitionTimeout)
		{
			StopMotor();
			SystemStateTransition();
		}
		else
		{
			SYSTEM.ui16StateTransitionTimeout--;
		}	
	}
	else
	{
		SYSTEM.i16StateTransition = SYSTEM_WAKEUP;
	}
	return 0;
}

Int16 SystemPWMInLostState()
{
	// Transit out of?
	if(SYSTEM.i16StateTransition != SYSTEM.systemState)
	{
		if(0 == SYSTEM.ui16StateTransitionTimeout)
		{
			StopMotor();
			SystemStateTransition();
		}
		else
		{
			SYSTEM.ui16StateTransitionTimeout--;
		}	
	}
	else
	{
		SYSTEM.i16StateTransition = SYSTEM_WAKEUP;
	}
	return 0;
}

Int16 SystemCalibrateState()
{

	Int16 i16Temp0 = 0;
	Int16 i16Temp1 = 0;
	Int16 i16Temp2 = 0;
	Int16 i16Temp3 = 0;
	Frac16 f16Temp0 = 0;
	Int32 i32Temp0 = 0;
	// Transit out of?
	if(SYSTEM.i16StateTransition != SYSTEM.systemState)
	{
		if(0 == SYSTEM.ui16StateTransitionTimeout)
		{
			StopMotor();
			SystemStateTransition();
		}
		else
		{
			SYSTEM.ui16StateTransitionTimeout--;
		}	
	}
	else
	{
		// Mark calibrating
		switch(SYSTEM.CALIBRATION.i16CalibrationState)
		{
			case CALIBRATE_INIT:
			{
				SYSTEM.CALIBRATION.i16CalibrationState = CALIBRATE_START;
				SYSTEM.POSITION.f16RotorAngle = FRAC16(0.0);
				// Calculate sin/cos
				SYSTEM.POSITION.mSinCosAngle.f16Sin = GFLIB_Sin_F16(SYSTEM.POSITION.f16RotorAngle);
				SYSTEM.POSITION.mSinCosAngle.f16Cos = GFLIB_Cos_F16(SYSTEM.POSITION.f16RotorAngle);
				
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
				
				SYSTEM.CALIBRATION.i16CalibrationState = CALIBRATE_INIT;
				SYSTEM.systemState = SYSTEM_RESET;
				SYSTEM.i16StateTransition = SYSTEM_WAKEUP;
				
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
	}
	return 0;
}

Int16 SystemParkRotorState()
{
	Int16 i16Temp0 = 0;
	// Get how far away are we from final position
	// Transit out of?
	if(SYSTEM.i16StateTransition != SYSTEM.systemState)
	{
		if(0 == SYSTEM.ui16StateTransitionTimeout)
		{
			StopMotor();
			SystemStateTransition();
		}
		else
		{
			SYSTEM.ui16StateTransitionTimeout--;
		}	
	}
	else
	{
		SYSTEM_PARK_ROTOR = 1;
		SYSTEM_RUN_MANUAL_CW = 0;
		SYSTEM_RUN_MANUAL_CCW = 0;
		
		// Use PWM?
		if(1 == COMMDataStruct.REGS.ui8UsePWMIN)
		{
			// If PWM under zero speed, go to idle mode
			i16Temp0 = COMMDataStruct.REGS.i16CurrentPWM - COMMDataStruct.REGS.i16PWMMin;
	
			if(i16Temp0 < COMMDataStruct.REGS.i16ZeroSpeedPWM)
			{
				COMMDataStruct.REGS.ui8Armed = 0;
			}
		}
	
		// Go out of park?
		if(0 == COMMDataStruct.REGS.ui8Park)
		{
			// Control system speed
			CONTROL_SPEED = 1;
			SYSTEM.i16StateTransition = SYSTEM_RUN;
		}
		
		// Stop motor?
		if(0 == COMMDataStruct.REGS.ui8Armed)
		{
			// Go out of run mode
			SYSTEM.i16StateTransition = SYSTEM_RESET;
			SYSTEM.RAMPS.f16SpeedRampDesiredValue = FRAC16(0.0);
		}
	}
	return 0;	
}

Int16 SystemMeasureRPHAState()
{
	Frac16 f16Temp = 0;
	float fUph = 0.0f;
	float fRph = 0.0f;
	// Transit out of?
	if(SYSTEM.i16StateTransition != SYSTEM.systemState)
	{
		if(0 == SYSTEM.ui16StateTransitionTimeout)
		{
			StopMotor();
			SystemStateTransition();
		}
		else
		{
			SYSTEM.ui16StateTransitionTimeout--;
		}	
	}
	else
	{
		switch (SYSTEM.i16MotorRPhaMeasureState)
		{
			case SYSTEM_MEAS_RPHA_INIT:
			{
				// Set PWM values
				SYSTEM.PWMValues.pwmSub_0_Channel_23_Value = FRAC16(0.5);
				SYSTEM.PWMValues.pwmSub_1_Channel_23_Value = FRAC16(0.5);
				SYSTEM.PWMValues.pwmSub_2_Channel_23_Value = FRAC16(0.5);
				ioctl(EFPWMA, EFPWM_CENTER_ALIGN_UPDATE_VALUE_REGS_COMPL_012, &SYSTEM.PWMValues);
				// Enable PWM outputs
				ioctl(EFPWMA, EFPWM_SET_OUTPUTS_ENABLE, EFPWM_SUB0_PWM_A|EFPWM_SUB0_PWM_B|EFPWM_SUB1_PWM_A|EFPWM_SUB1_PWM_B|EFPWM_SUB2_PWM_A|EFPWM_SUB2_PWM_B);		
				// Set Id, Iq to 0
				SYSTEM.REGULATORS.m2IDQReq.f16D = FRAC16(0.0);
				SYSTEM.REGULATORS.m2IDQReq.f16Q = FRAC16(0.0);
				// Enable regulators
				PWM_ENABLED = 1;			
				// Set Id
				SYSTEM.POSITION.i16PositionSource = POSITION_SOURCE_STANDSTILL;
				SYSTEM.REGULATORS.i16CurrentSource = CURRENT_SOURCE_MEASURE_RPHA;
				// Set Id to ~6.592 A
				SYSTEM.MEASUREPARAMS.f16MeasureRPhaId = FRAC16(0.02);
				SYSTEM.i16MotorRPhaMeasureState = SYSTEM_MEAS_RPHA_SET1_IPHA;
				SYSTEM.MEASUREPARAMS.i16StabilizeCounter = 0;
				break;
			}
			case SYSTEM_MEAS_RPHA_SET1_IPHA:
			{
				// Wait set time
				SYSTEM.MEASUREPARAMS.i16StabilizeCounter++;
				if(1000 < SYSTEM.MEASUREPARAMS.i16StabilizeCounter)
				{
					SYSTEM.MEASUREPARAMS.i16StabilizeCounter = 1000;
					// Iphase at set value?
					f16Temp = SYSTEM.MCTRL.m2IDQ.f16D - SYSTEM.MEASUREPARAMS.f16MeasureRPhaId;
					SYSTEM.MEASUREPARAMS.f16TempDelta = f16Temp;
					if((FRAC16(0.001) > f16Temp)&&(FRAC16(-0.001) < f16Temp))
					{
						// Store U values
						SYSTEM.MEASUREPARAMS.f16U1 = SYSTEM.MCTRL.m2UDQ.f16D;
						// Set Id to ~9.888 A
						SYSTEM.MEASUREPARAMS.f16MeasureRPhaId = FRAC16(0.03);
						SYSTEM.i16MotorRPhaMeasureState = SYSTEM_MEAS_RPHA_SET2_IPHA;
						SYSTEM.MEASUREPARAMS.i16StabilizeCounter = 0;
					}
				}
				break;
			}
			case SYSTEM_MEAS_RPHA_SET2_IPHA:
			{
				// Wait set time
				SYSTEM.MEASUREPARAMS.i16StabilizeCounter++;
				if(1000 < SYSTEM.MEASUREPARAMS.i16StabilizeCounter)
				{
					SYSTEM.MEASUREPARAMS.i16StabilizeCounter = 1000;
					// Iphase at set value?
					f16Temp = SYSTEM.MCTRL.m2IDQ.f16D - SYSTEM.MEASUREPARAMS.f16MeasureRPhaId;
					SYSTEM.MEASUREPARAMS.f16TempDelta = f16Temp;
					if((FRAC16(0.001) > f16Temp)&&(FRAC16(-0.001) < f16Temp))
					{
						// Store U values
						SYSTEM.MEASUREPARAMS.f16U2 = SYSTEM.MCTRL.m2UDQ.f16D;
						// Set Id to 0 A
						SYSTEM.MEASUREPARAMS.f16MeasureRPhaId = FRAC16(0.0);
						// Calculate RPha
						f16Temp = SYSTEM.MEASUREPARAMS.f16U2 - SYSTEM.MEASUREPARAMS.f16U1;
						fUph = (float)f16Temp;
						fUph = fUph / 32768;
						fUph = fUph * SYSTEM.SIVALUES.fUIn;
						fRph = fUph / 3.296f; //9.888 - 6.592;
						
						SYSTEM.MEASUREPARAMS.fMeasuredRTotal = fRph;
						
						fRph = fRph - MOSFET_RDSON - (MOSFET_RDSON/2);
						
						fRph = fRph * 2.0f;
						fRph = fRph / 3.0f;
						
						SYSTEM.MEASUREPARAMS.fMeasuredRPha = fRph;
						
						SYSTEM.i16MotorRPhaMeasureState = SYSTEM_MEAS_RPHA_INIT;
						SYSTEM.i16StateTransition = SYSTEM_IDLE;
					}
				}
				break;
			}
			default:
			{
				break;
			}
		}
	}
	return 0;
}

Int16 SystemMeasureLPHAState()
{
	Frac16 f16Temp = 0;
	UWord16 uw16Temp = 0;
	float fLph = 0.0f;
	// Transit out of?
	if(SYSTEM.i16StateTransition != SYSTEM.systemState)
	{
		if(0 == SYSTEM.ui16StateTransitionTimeout)
		{
			StopMotor();
			SystemStateTransition();
		}
		else
		{
			SYSTEM.ui16StateTransitionTimeout--;
		}	
	}
	else
	{
		switch (SYSTEM.i16MotorLPhaMeasureState)
		{
			case SYSTEM_MEAS_LPHA_INIT:
			{
				// Set required current to get required PWM value
				// Set PWM values
				SYSTEM.PWMValues.pwmSub_0_Channel_23_Value = FRAC16(0.5);
				SYSTEM.PWMValues.pwmSub_1_Channel_23_Value = FRAC16(0.5);
				SYSTEM.PWMValues.pwmSub_2_Channel_23_Value = FRAC16(0.5);
				ioctl(EFPWMA, EFPWM_CENTER_ALIGN_UPDATE_VALUE_REGS_COMPL_012, &SYSTEM.PWMValues);
				// Enable PWM outputs
				ioctl(EFPWMA, EFPWM_SET_OUTPUTS_ENABLE, EFPWM_SUB0_PWM_A|EFPWM_SUB0_PWM_B|EFPWM_SUB1_PWM_A|EFPWM_SUB1_PWM_B|EFPWM_SUB2_PWM_A|EFPWM_SUB2_PWM_B);		
				// Set Id, Iq to 0
				SYSTEM.REGULATORS.m2IDQReq.f16D = FRAC16(0.0);
				SYSTEM.REGULATORS.m2IDQReq.f16Q = FRAC16(0.0);
				// Enable regulators
				PWM_ENABLED = 1;			
				// Set Id
				SYSTEM.POSITION.i16PositionSource = POSITION_SOURCE_STANDSTILL;
				SYSTEM.REGULATORS.i16CurrentSource = CURRENT_SOURCE_MEASURE_RPHA;
				// Set Id to ~6.592 A
				SYSTEM.MEASUREPARAMS.f16MeasureRPhaId = SYSTEM.MEASUREPARAMS.f16LphaIset;
				SYSTEM.i16MotorLPhaMeasureState = SYSTEM_MEAS_LPHA_GET_UREQ;
				SYSTEM.MEASUREPARAMS.i16StabilizeCounter = 0;
	
				break;
			}
			case SYSTEM_MEAS_LPHA_GET_UREQ:
			{
				// Store required PWM value and turn off PWM
				// Set PWM values
				// Wait set time
				SYSTEM.MEASUREPARAMS.i16StabilizeCounter++;
				if(250 < SYSTEM.MEASUREPARAMS.i16StabilizeCounter)
				{
					SYSTEM.MEASUREPARAMS.i16StabilizeCounter = 250;
					SYSTEM.MEASUREPARAMS.pwmSubReqPWMValues.pwmSub_0_Channel_23_Value = SYSTEM.MCTRL.m3U_UVW.f16A;
					SYSTEM.MEASUREPARAMS.pwmSubReqPWMValues.pwmSub_1_Channel_23_Value = SYSTEM.MCTRL.m3U_UVW.f16B;
					SYSTEM.MEASUREPARAMS.pwmSubReqPWMValues.pwmSub_2_Channel_23_Value = SYSTEM.MCTRL.m3U_UVW.f16C;
					
					SYSTEM.MEASUREPARAMS.f16IphAValue = SYSTEM.ADC.m3IphUVW.f16A;
					StopMotor();
					SYSTEM.MEASUREPARAMS.i16StabilizeCounter = 0;
					SYSTEM.MEASUREPARAMS.i16Measurements = 0;
					SYSTEM.MEASUREPARAMS.i16TotalTicks = 0;
					SYSTEM.i16MotorLPhaMeasureState = SYSTEM_MEAS_LPHA_DELAY;				
				}
				break;
			}
			case SYSTEM_MEAS_LPHA_DELAY:
			{
				// Wait to get current to 0 and set measurement
				// Wait set time
				SYSTEM.MEASUREPARAMS.i16StabilizeCounter++;
				if(250 < SYSTEM.MEASUREPARAMS.i16StabilizeCounter)
				{
					SYSTEM.MEASUREPARAMS.i16StabilizeCounter = 250;
					// Setup measurement
					
					// Get 63.2 % of value - tau
					// Get 74.85 % of value - 1.5 tau
					SYSTEM.MEASUREPARAMS.f16LphaITrig = mult(SYSTEM.MEASUREPARAMS.f16IphAValue, SYSTEM.MEASUREPARAMS.f16SetpointMulti);
	
					// Set PWMs to req value for I
					SYSTEM.PWMValues.pwmSub_0_Channel_23_Value = SYSTEM.MEASUREPARAMS.pwmSubReqPWMValues.pwmSub_0_Channel_23_Value;
					SYSTEM.PWMValues.pwmSub_1_Channel_23_Value = SYSTEM.MEASUREPARAMS.pwmSubReqPWMValues.pwmSub_1_Channel_23_Value;
					SYSTEM.PWMValues.pwmSub_2_Channel_23_Value = SYSTEM.MEASUREPARAMS.pwmSubReqPWMValues.pwmSub_2_Channel_23_Value;
					ioctl(EFPWMA, EFPWM_CENTER_ALIGN_UPDATE_VALUE_REGS_COMPL_012, &SYSTEM.PWMValues);
					// Enable PWM outputs
					ioctl(EFPWMA, EFPWM_SET_OUTPUTS_ENABLE, EFPWM_SUB0_PWM_A|EFPWM_SUB0_PWM_B|EFPWM_SUB1_PWM_A|EFPWM_SUB1_PWM_B|EFPWM_SUB2_PWM_A|EFPWM_SUB2_PWM_B);					
					
					SYSTEM.MEASUREPARAMS.i16ITauTicks = 0;
					AD_MEAS_LPHA = 1;
	
					SYSTEM.MEASUREPARAMS.i16TempValue = 1;
					
					SYSTEM.i16MotorLPhaMeasureState = SYSTEM_MEAS_LPHA_GETVAL;
					SYSTEM.MEASUREPARAMS.i16StabilizeCounter = 0;
				}
				break;
			}
			case SYSTEM_MEAS_LPHA_GETVAL:
			{
				if(0 == AD_MEAS_LPHA)
				{
					// Set PWMs to 50%
					SYSTEM.PWMValues.pwmSub_0_Channel_23_Value = FRAC16(0.5);
					SYSTEM.PWMValues.pwmSub_1_Channel_23_Value = FRAC16(0.5);
					SYSTEM.PWMValues.pwmSub_2_Channel_23_Value = FRAC16(0.5);
					ioctl(EFPWMA, EFPWM_CENTER_ALIGN_UPDATE_VALUE_REGS_COMPL_012, &SYSTEM.PWMValues);	
					
					SYSTEM.MEASUREPARAMS.i16Measurements++;
					SYSTEM.MEASUREPARAMS.i16TotalTicks += SYSTEM.MEASUREPARAMS.i16ITauTicks;
					if(SYSTEM.MEASUREPARAMS.i16Measurements < SYSTEM.MEASUREPARAMS.i16TotalMeasurements)
					{
						SYSTEM.i16MotorLPhaMeasureState = SYSTEM_MEAS_LPHA_DELAY;	
					}
					else
					{
						SYSTEM.i16MotorLPhaMeasureState = SYSTEM_MEAS_LPHA_END;
					}
				}
				break;
			}
			case SYSTEM_MEAS_LPHA_END:
			{
				// Calculate tau
				fLph = (float)SYSTEM.MEASUREPARAMS.i16TotalTicks;
				fLph *= 62.5f;
				fLph /= (float)SYSTEM.MEASUREPARAMS.i16Measurements;
				fLph /= 1.5f;
				// Tau is in usec
				
				fLph = fLph * SYSTEM.MEASUREPARAMS.fMeasuredRTotal;
				
				SYSTEM.MEASUREPARAMS.fMeasuredLTotal = fLph;
				
				fLph *= 2.0f;
				fLph /= 3.0f;
				
				// Convert to H
				fLph = fLph / 1000000;
				
				SYSTEM.MEASUREPARAMS.fMeasuredLPha = fLph;
				
				SYSTEM.i16MotorLPhaMeasureState = SYSTEM_MEAS_LPHA_INIT;
				SYSTEM.i16StateTransition = SYSTEM_IDLE;
				break;
			}
			default:
			{
				break;
			}
		}
	}
	return 0;
}

Int16 SystemFaultOCEventState()
{
	// Transit out of?
	if(SYSTEM.i16StateTransition != SYSTEM.systemState)
	{
		if(0 == SYSTEM.ui16StateTransitionTimeout)
		{
			StopMotor();
			SystemStateTransition();
		}
		else
		{
			SYSTEM.ui16StateTransitionTimeout--;
		}	
	}
	else
	{
		// Reset?
		if(1 == COMMDataStruct.REGS.ui8Reset)
		{
			COMMDataStruct.REGS.ui8Reset = 0;
			SYSTEM.i16StateTransition = SYSTEM_WAKEUP;
		}
	}
	return 0;
}

Int16 SystemResetDriver()
{
	Int16 i16RetVal = -1;
	// Try restarting driver
	switch(SYSTEM.i16DriverRestartState)
	{
		case SYSTEM_RESTART_INIT:
		{
			// Turn driver gate enable off
			EN_GATE_OFF;
			SYSTEM.i16DriverRestartState = SYSTEM_RESTART_WAIT_POWER_OFF;
			break;
		}
		case SYSTEM_RESTART_WAIT_POWER_OFF:
		{
			// Turn driver gate enable on
			EN_GATE_ON;
			SYSTEM.i16DriverRestartState = SYSTEM_RESTART_WAIT_POWER_ON;
			break;
		}
		case SYSTEM_RESTART_WAIT_POWER_ON:
		{
			/*
			// Set values
			REINIT_DRV8301 = 1;
			// Reset time counter
			SYSTEM.i16DriverRestartTimer = 0;
			SYSTEM.i16DriverRestartState = SYSTEM_RESTART_WAIT_REINIT;*/
			SYSTEM.i16DriverRestartState = SYSTEM_RESTART_GO;
			break;
		}
		case SYSTEM_RESTART_WAIT_REINIT:
		{
			// Increase timer
			SYSTEM.i16DriverRestartTimer++;
			// Check - initialised?
			if(DRV8301_CONFIGURED)
			{
				// Driver reconfiguration successful, read status
				// Read reg 1
				DRV8301.RegReq.RW = 1;								//we are initiating a read
				DRV8301.RegReq.ADDR = DRV8301_STAT_REG_1_ADDR;		//load the address
				DRV8301.RegReq.DATA = 0;							//dummy data;
				// Send data
				ioctl(SPI_0, SPI_WRITE_DATA, DRV8301.RegReq.reg);	
				
				SYSTEM.i16DriverRestartState = SYSTEM_RESTART_WAIT_READ_1;
			}
			else if(100 < SYSTEM.i16DriverRestartTimer)
			{
				// Something wrong, go to fault
				SYSTEM.systemState = SYSTEM_FAULT;
			}
			break;
		}
		case SYSTEM_RESTART_WAIT_READ_1:
		{
			// Data ready?
			if(ioctl(SPI_0, SPI_CAN_READ_DATA, null) == 0)
			{
				// Read to req reg
				DRV8301.RegReq.reg = ioctl(SPI_0, SPI_READ_DATA, null);
				// Read reg 2
				DRV8301.RegReq.RW = 1;								//we are initiating a read
				DRV8301.RegReq.ADDR = DRV8301_STAT_REG_2_ADDR;		//load the address
				DRV8301.RegReq.DATA = 0;							//dummy data;
				// Send data
				ioctl(SPI_0, SPI_WRITE_DATA, DRV8301.RegReq.reg);
				
				SYSTEM.i16DriverRestartState = SYSTEM_RESTART_WAIT_READ_2;
			}
			break;
		}
		case SYSTEM_RESTART_WAIT_READ_2:
		{
			// Data ready?
			if(ioctl(SPI_0, SPI_CAN_READ_DATA, null) == 0)
			{
				// Read to req reg
				DRV8301.RegReq.reg = ioctl(SPI_0, SPI_READ_DATA, null);
				// Store
				DRV8301.StatReg1.reg = DRV8301.RegReq.reg; 
				// Dummy read
				DRV8301.RegReq.RW = 1;								//we are initiating a read
				DRV8301.RegReq.ADDR = DRV8301_STAT_REG_2_ADDR;		//load the address
				DRV8301.RegReq.DATA = 0;							//dummy data;
				// Send data
				ioctl(SPI_0, SPI_WRITE_DATA, DRV8301.RegReq.reg);
				
				SYSTEM.i16DriverRestartState = SYSTEM_RESTART_WAIT_READ_3;
			}
			break;
		}
		case SYSTEM_RESTART_WAIT_READ_3:
		{
			// Data ready?
			if(ioctl(SPI_0, SPI_CAN_READ_DATA, null) == 0)
			{
				// Read to req reg
				DRV8301.RegReq.reg = ioctl(SPI_0, SPI_READ_DATA, null);
				// Store
				DRV8301.StatReg2.reg = DRV8301.RegReq.reg; 
				// Check data
				if(0 == DRV8301.StatReg1.FAULT)
				{
					// Go to init
					SYSTEM.systemState = SYSTEM_WAKEUP;
				}
				else
				{
					// Something wrong, go to fault
					SYSTEM.systemState = SYSTEM_FAULT;
				}
			}
			break;
		}
		case SYSTEM_RESTART_GO:
		{
			// Go to init
			SYSTEM.i16DriverRestartState = SYSTEM_RESTART_INIT;
			break;
		}
		default:
		{
			// Turn driver gate enable off
			EN_GATE_OFF;
			SYSTEM.i16DriverRestartState = SYSTEM_RESTART_WAIT_POWER_OFF;
			break;
		}
	}	
	
	return SYSTEM.i16DriverRestartState;
}

Int16 SystemStateTransition()
{
	float fTemp = 0.0f;
	// Check state transition	
	switch(SYSTEM.i16StateTransition)
	{
		case SYSTEM_WAKEUP:
		{
			SYSTEM.PWMValues.pwmSub_0_Channel_23_Value = FRAC16(0.5);
			SYSTEM.PWMValues.pwmSub_1_Channel_23_Value = FRAC16(0.5);
			SYSTEM.PWMValues.pwmSub_2_Channel_23_Value = FRAC16(0.5);
			// Load new PWM values	
			ioctl(EFPWMA, EFPWM_CENTER_ALIGN_UPDATE_VALUE_REGS_COMPL_012, &SYSTEM.PWMValues);	
			
			SYSTEM.systemState = SYSTEM_WAKEUP;
			break;
		}
		case SYSTEM_INIT:
		{
			SYSTEM.PWMValues.pwmSub_0_Channel_23_Value = FRAC16(0.5);
			SYSTEM.PWMValues.pwmSub_1_Channel_23_Value = FRAC16(0.5);
			SYSTEM.PWMValues.pwmSub_2_Channel_23_Value = FRAC16(0.5);
			// Load new PWM values	
			ioctl(EFPWMA, EFPWM_CENTER_ALIGN_UPDATE_VALUE_REGS_COMPL_012, &SYSTEM.PWMValues);	
			
			SYSTEM.systemState = SYSTEM_INIT;
			break;
		}
		case SYSTEM_IDLE:
		{
			SYSTEM.PWMValues.pwmSub_0_Channel_23_Value = FRAC16(0.5);
			SYSTEM.PWMValues.pwmSub_1_Channel_23_Value = FRAC16(0.5);
			SYSTEM.PWMValues.pwmSub_2_Channel_23_Value = FRAC16(0.5);
			// Load new PWM values	
			ioctl(EFPWMA, EFPWM_CENTER_ALIGN_UPDATE_VALUE_REGS_COMPL_012, &SYSTEM.PWMValues);	
			
			// Go to idle state
			SYSTEM.systemState = SYSTEM_IDLE;	
			break;
		}
		
		case SYSTEM_RUN:
		{	
			// Check undervoltage error
			if(0 != COMMDataStruct.REGS.ui8UVError)
			{
				// Battery low, abort
				SYSTEM.PWMValues.pwmSub_0_Channel_23_Value = FRAC16(0.5);
				SYSTEM.PWMValues.pwmSub_1_Channel_23_Value = FRAC16(0.5);
				SYSTEM.PWMValues.pwmSub_2_Channel_23_Value = FRAC16(0.5);
				// Load new PWM values	
				ioctl(EFPWMA, EFPWM_CENTER_ALIGN_UPDATE_VALUE_REGS_COMPL_012, &SYSTEM.PWMValues);	
				// Turn off PWMs
				ioctl(EFPWMA, EFPWM_SET_OUTPUTS_DISABLE, EFPWM_SUB0_PWM_A|EFPWM_SUB0_PWM_B|EFPWM_SUB1_PWM_A|EFPWM_SUB1_PWM_B|EFPWM_SUB2_PWM_A|EFPWM_SUB2_PWM_B);
				SYSTEM.i16StateTransition = SYSTEM_FAULT;
			}
			else
			{
				// Reset all errors
				SYSTEM.SENSORLESS.ui8BemfObserverErrorCount = 0;
				SYSTEM.SENSORLESS.ui16BEMFONError = 0;
				SYSTEM.DRIVERSTATE.i8DriverFaultCount = 0;
				SYSTEM.DRIVERSTATE.i8DriverFault = 0;
				COMMDataStruct.REGS.ui16Errors = 0;
				COMMDataStruct.REGS.ui8UVError = 0;
				COMMDataStruct.errStatus = 0;
				ERROR_DQ_MERGE = 0;
				
				// Set startup speed
				fTemp = (float)COMMDataStruct.REGS.i16MinRPM * 0.06826666666666666666666666666667f;
				fTemp = fTemp * (float)SYSTEM.CALIBRATION.i16MotorPolePairs;
				SYSTEM.SENSORLESS.f16StartSpeed = (Frac16)fTemp;
				
				// Set BEMF ON speed
				SYSTEM.SENSORLESS.f16MinSpeed = SYSTEM.SENSORLESS.f16StartSpeed / 2;
				
				// Set BEMF ON hysteresis
				SYSTEM.SENSORLESS.f16MinSpeedHysteresis = SYSTEM.SENSORLESS.f16MinSpeed / 2;
	
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
					// Check that minimum speed is set
					if(COMMDataStruct.REGS.i16MinRPM > COMMDataStruct.REGS.i16SetRPM)
					{
						COMMDataStruct.REGS.i16SetRPM = COMMDataStruct.REGS.i16MinRPM;
					}
					// Start open loop mode
					SYSTEM.POSITION.i16PositionSource = POSITION_SOURCE_SENSORLESS_ALIGN;
					SYSTEM.REGULATORS.i16CurrentSource = CURRENT_SOURCE_SENSORLESS_ALIGN;
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
					GFLIB_RampInit_F16(FRAC16(0.0), &SYSTEM.RAMPS.Ramp16_AlignCurrent);
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
					SYSTEM.PWMValues.pwmSub_0_Channel_23_Value = FRAC16(0.5);
					SYSTEM.PWMValues.pwmSub_1_Channel_23_Value = FRAC16(0.5);
					SYSTEM.PWMValues.pwmSub_2_Channel_23_Value = FRAC16(0.5);
					// Load new PWM values	
					ioctl(EFPWMA, EFPWM_CENTER_ALIGN_UPDATE_VALUE_REGS_COMPL_012, &SYSTEM.PWMValues);	
					// Turn off PWMs
					ioctl(EFPWMA, EFPWM_SET_OUTPUTS_DISABLE, EFPWM_SUB0_PWM_A|EFPWM_SUB0_PWM_B|EFPWM_SUB1_PWM_A|EFPWM_SUB1_PWM_B|EFPWM_SUB2_PWM_A|EFPWM_SUB2_PWM_B);
					SYSTEM.i16StateTransition = SYSTEM_RESET;
				}
			}
			break;
		}
		case SYSTEM_FAULT:
		{
			COMMDataStruct.REGS.ui8Armed = 0;
			SYSTEM.PWMValues.pwmSub_0_Channel_23_Value = FRAC16(0.5);
			SYSTEM.PWMValues.pwmSub_1_Channel_23_Value = FRAC16(0.5);
			SYSTEM.PWMValues.pwmSub_2_Channel_23_Value = FRAC16(0.5);
			// Load new PWM values	
			ioctl(EFPWMA, EFPWM_CENTER_ALIGN_UPDATE_VALUE_REGS_COMPL_012, &SYSTEM.PWMValues);	
			ioctl(EFPWMA, EFPWM_SET_OUTPUTS_DISABLE, EFPWM_SUB0_PWM_A|EFPWM_SUB0_PWM_B|EFPWM_SUB1_PWM_A|EFPWM_SUB1_PWM_B|EFPWM_SUB2_PWM_A|EFPWM_SUB2_PWM_B);
			SYSTEM.systemState = SYSTEM_FAULT;
			break;
		}
		case SYSTEM_RESET:
		{
			COMMDataStruct.REGS.ui8Armed = 0;
			SYSTEM.PWMValues.pwmSub_0_Channel_23_Value = FRAC16(0.5);
			SYSTEM.PWMValues.pwmSub_1_Channel_23_Value = FRAC16(0.5);
			SYSTEM.PWMValues.pwmSub_2_Channel_23_Value = FRAC16(0.5);
			// Load new PWM values	
			ioctl(EFPWMA, EFPWM_CENTER_ALIGN_UPDATE_VALUE_REGS_COMPL_012, &SYSTEM.PWMValues);	
			ioctl(EFPWMA, EFPWM_SET_OUTPUTS_DISABLE, EFPWM_SUB0_PWM_A|EFPWM_SUB0_PWM_B|EFPWM_SUB1_PWM_A|EFPWM_SUB1_PWM_B|EFPWM_SUB2_PWM_A|EFPWM_SUB2_PWM_B);
			SYSTEM.systemState = SYSTEM_RESET;
			break;
		}
		case SYSTEM_RESTARTING:
		{
			COMMDataStruct.REGS.ui8Armed = 0;
			SYSTEM.systemState = SYSTEM_RESTARTING;
			break;
		}
		case SYSTEM_FAULT_DRV8301:
		{
			COMMDataStruct.REGS.ui8Armed = 0;
			SYSTEM.systemState = SYSTEM_FAULT_DRV8301;
			SYSTEM.PWMValues.pwmSub_0_Channel_23_Value = FRAC16(0.5);
			SYSTEM.PWMValues.pwmSub_1_Channel_23_Value = FRAC16(0.5);
			SYSTEM.PWMValues.pwmSub_2_Channel_23_Value = FRAC16(0.5);
			// Load new PWM values	
			ioctl(EFPWMA, EFPWM_CENTER_ALIGN_UPDATE_VALUE_REGS_COMPL_012, &SYSTEM.PWMValues);	
			// PWMs OFF
			ioctl(EFPWMA, EFPWM_SET_OUTPUTS_DISABLE, EFPWM_SUB0_PWM_A|EFPWM_SUB0_PWM_B|EFPWM_SUB1_PWM_A|EFPWM_SUB1_PWM_B|EFPWM_SUB2_PWM_A|EFPWM_SUB2_PWM_B);
			// Reset driver
			SYSTEM.i16DriverRestartState = SYSTEM_RESTART_INIT;
			break;
		}
		case SYSTEM_FAULT_RESET:
		{
			COMMDataStruct.REGS.ui8Armed = 0;
			SYSTEM.systemState = SYSTEM_FAULT_RESET;
			break;
		}
		case SYSTEM_BLOCKEXEC:
		{
			COMMDataStruct.REGS.ui8Armed = 0;
			SYSTEM.systemState = SYSTEM_BLOCKEXEC;
			break;
		}
		case SYSTEM_FOC_LOST_TIMEOUT:
		{
			COMMDataStruct.REGS.ui8Armed = 0;
			SYSTEM.systemState = SYSTEM_FOC_LOST_TIMEOUT;
			break;
		}
		case SYSTEM_PWM_IN_LOST:
		{
			COMMDataStruct.REGS.ui8Armed = 0;
			SYSTEM.systemState = SYSTEM_PWM_IN_LOST;
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
			GFLIB_RampInit_F16(FRAC16(0.0), &SYSTEM.RAMPS.Ramp16_AlignCurrent);
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

			// Manual control
			SYSTEM.POSITION.i16PositionSource = POSITION_SOURCE_MANUAL;
			SYSTEM.REGULATORS.i16CurrentSource = CURRENT_SOURCE_CONTROL_MANUAL;
			// Set D, Q currents to 0
			SYSTEM.RAMPS.f16AlignCurrentActualValue = FRAC16(0.0);
			SYSTEM.RAMPS.f16AlignCurrentDesiredValue = FRAC16(0.01);
			GFLIB_RampInit_F16(FRAC16(0.0), &SYSTEM.RAMPS.Ramp16_AlignCurrent);
			// Set Id, Iq to 0
			SYSTEM.REGULATORS.m2IDQReq.f16D = FRAC16(0.01);
			SYSTEM.REGULATORS.m2IDQReq.f16Q = FRAC16(0.0);
			SYSTEM_RUN_MANUAL = 1;
			SYSTEM_RUN_MANUAL_CW = 0;
			SYSTEM_RUN_MANUAL_CCW = 0;
			// Enable regulators
			PWM_ENABLED = 1;

			SYSTEM.systemState = SYSTEM_PARKROTOR;
			break;
		}
		case SYSTEM_MEAS_RPHA:
		{			
			SYSTEM.i16MotorRPhaMeasureState = SYSTEM_MEAS_RPHA_INIT;
			SYSTEM.systemState = SYSTEM_MEAS_RPHA;
			break;
		}
		case SYSTEM_MEAS_LPHA:
		{
			SYSTEM.i16MotorLPhaMeasureState = SYSTEM_MEAS_LPHA_INIT;
			SYSTEM.systemState = SYSTEM_MEAS_LPHA;
			break;
		}
		case SYSTEM_FAULT_OCEVENT:
		{
			COMMDataStruct.REGS.ui8Armed = 0;
			SYSTEM.systemState = SYSTEM_FAULT_OCEVENT;
			break;
		}
		default:
		{
			SYSTEM.systemState = SYSTEM_WAKEUP;
			break;
		}
	}	
	return 0;
}
