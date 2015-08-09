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
	SYSTEM.ADC.m3IphUVWRaw.f16A = ioctl(ADC_1, ADC_READ_SAMPLE, 0);
	SYSTEM.ADC.m3IphUVWRaw.f16C = ioctl(ADC_1, ADC_READ_SAMPLE, 8);
	// Zero currents?
	if(SYS_ZERO_CURRENT)
	{
		SYS_ZERO_CURRENT = 0;
		SYSTEM.ADC.f16OffsetU = SYSTEM.ADC.m3IphUVWRaw.f16A;
		SYSTEM.ADC.f16OffsetW = SYSTEM.ADC.m3IphUVWRaw.f16C;
	}
	
	// Remove offsets
	SYSTEM.ADC.m3IphUVW.f16A = SYSTEM.ADC.m3IphUVWRaw.f16A - SYSTEM.ADC.f16OffsetU;
	SYSTEM.ADC.m3IphUVW.f16C = SYSTEM.ADC.m3IphUVWRaw.f16C - SYSTEM.ADC.f16OffsetW;
	// Calculate third
	SYSTEM.ADC.m3IphUVW.f16B = -SYSTEM.ADC.m3IphUVW.f16A - SYSTEM.ADC.m3IphUVW.f16C;
	
	// Multiply with gain
	SYSTEM.ADC.m3IphUVW.f16A = mult(SYSTEM.ADC.f16CurrentGainFactor, SYSTEM.ADC.m3IphUVW.f16A);
	SYSTEM.ADC.m3IphUVW.f16B = mult(SYSTEM.ADC.f16CurrentGainFactor, SYSTEM.ADC.m3IphUVW.f16B);
	SYSTEM.ADC.m3IphUVW.f16C = mult(SYSTEM.ADC.f16CurrentGainFactor, SYSTEM.ADC.m3IphUVW.f16C);
	
	// Measure phase voltages
	// U
	SYSTEM.ADC.m3UphUVW.f16A = ioctl(ADC_1, ADC_READ_SAMPLE, 3);
	// V
	SYSTEM.ADC.m3UphUVW.f16B = ioctl(ADC_1, ADC_READ_SAMPLE, 11);
	// W
	SYSTEM.ADC.m3UphUVW.f16C = ioctl(ADC_1, ADC_READ_SAMPLE, 4);
	
	
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
	
	// Change phase current amplification if necessary
	if(DRV8301_CONFIGURED)
	{
		switch(DRV8301.CtrlReg2.GAIN)
		{
			case DRV8301_GAIN_10:
			{
				// Check phase value
				if(DRV_A10_MIN > SYSTEM.MCTRL.f16IPh)
				{
					// Switch to x20
					DRV8301.CtrlReg2.GAIN = DRV8301_GAIN_20;
					SYSTEM.ADC.f16CurrentGainFactor = FRAC16(0.5);
					DRV_WRITE_GAIN = 1;
				}
				break;
			}
			case DRV8301_GAIN_20:
			{
				// Check phase value
				if(DRV_A20_MIN > SYSTEM.MCTRL.f16IPh)
				{
					// Switch to x40
					DRV8301.CtrlReg2.GAIN = DRV8301_GAIN_40;
					SYSTEM.ADC.f16CurrentGainFactor = FRAC16(0.25);
					DRV_WRITE_GAIN = 1;
					
				}
				else if(DRV_A20_MAX < SYSTEM.MCTRL.f16IPh)
				{
					// Switch to x10
					DRV8301.CtrlReg2.GAIN = DRV8301_GAIN_10;
					SYSTEM.ADC.f16CurrentGainFactor = FRAC16(1.0);
					DRV_WRITE_GAIN = 1;
				}
				break;
			}
			case DRV8301_GAIN_40:
			{
				// Check phase value
				if(DRV_A40_MIN > SYSTEM.MCTRL.f16IPh)
				{
					// Switch to x80
					DRV8301.CtrlReg2.GAIN = DRV8301_GAIN_80;
					SYSTEM.ADC.f16CurrentGainFactor = FRAC16(0.125);
					DRV_WRITE_GAIN = 1;
				}
				else if(DRV_A40_MAX < SYSTEM.MCTRL.f16IPh)
				{
					// Switch to x20
					DRV8301.CtrlReg2.GAIN = DRV8301_GAIN_20;
					SYSTEM.ADC.f16CurrentGainFactor = FRAC16(0.5);
					DRV_WRITE_GAIN = 1;
				}
				break;
			}
			case DRV8301_GAIN_80:
			{
				if(DRV_A40_MAX < SYSTEM.MCTRL.f16IPh)
				{
					// Switch to x40
					DRV8301.CtrlReg2.GAIN = DRV8301_GAIN_40;
					SYSTEM.ADC.f16CurrentGainFactor = FRAC16(0.25);
					DRV_WRITE_GAIN = 1;
				}
				break;
			}
		}
		
		if(DRV_WRITE_GAIN)
		{
			DRV_WRITE_GAIN = 0;
			DRV8301.RegReq.RW = 0;								//we are initiating a write
			DRV8301.RegReq.ADDR = DRV8301_CNTRL_REG_2_ADDR;		//load the address
			DRV8301.RegReq.DATA = DRV8301.CtrlReg2.reg;			//data to be written;
			
			ioctl(SPI_0, SPI_WRITE_DATA, DRV8301.RegReq.reg);
		}
		else
		{
			// Result ready in SPI buffer?
			if(ioctl(SPI_0, SPI_CAN_READ_DATA, null) == 0)
			{
				DRV_DATA_READ = 1;
				// Read to req reg
				DRV8301.RegReq.reg = ioctl(SPI_0, SPI_READ_DATA, null);
				// What is it?
				if(DRV8301_STAT_REG_1_ADDR == DRV8301.RegReq.ADDR)
				{
					// Store to stat reg 1
					DRV8301.StatReg1.reg = DRV8301.RegReq.reg; 
					DRV8301.ui16LastRegRead = DRV8301_STAT_REG_1_ADDR;
				}
				else if(DRV8301_STAT_REG_2_ADDR == DRV8301.RegReq.ADDR)
				{
					// Store to stat reg 2
					DRV8301.StatReg2.reg = DRV8301.RegReq.reg; 	
					DRV8301.ui16LastRegRead = DRV8301_STAT_REG_2_ADDR;
				}
			}
			else if(DRV_POLL)
			{
				DRV_POLL = 0;
				if(DRV8301_STAT_REG_1_ADDR == DRV8301.ui16LastRegRead)
				{
					// Read reg 2
					DRV8301.RegReq.RW = 1;								//we are initiating a read
					DRV8301.RegReq.ADDR = DRV8301_STAT_REG_2_ADDR;		//load the address
					DRV8301.RegReq.DATA = 0;							//dummy data;
					// Send data
					ioctl(SPI_0, SPI_WRITE_DATA, DRV8301.RegReq.reg);					
				}
				else if(DRV8301_STAT_REG_2_ADDR == DRV8301.ui16LastRegRead)
				{
					// Read reg 1
					DRV8301.RegReq.RW = 1;								//we are initiating a read
					DRV8301.RegReq.ADDR = DRV8301_STAT_REG_1_ADDR;		//load the address
					DRV8301.RegReq.DATA = 0;							//dummy data;
					// Send data
					ioctl(SPI_0, SPI_WRITE_DATA, DRV8301.RegReq.reg);					
				}
			}
		}
	}
		
	// Call freemaster recorder
	FMSTR_Recorder();
	
	// Recalculate input capture values
	// Values are read from timer modules in PWM reload interrupt
	SYSTEM.INPUTCAPTURE.m3UphUVW.f16A = (Frac16)(SYSTEM.INPUTCAPTURE.Val0 * 5);
	mac_r(SYSTEM.INPUTCAPTURE.m3UphUVW.f16A, SYSTEM.INPUTCAPTURE.Val0, FRAC16(0.24288));
	
	SYSTEM.INPUTCAPTURE.m3UphUVW.f16B = (Frac16)(SYSTEM.INPUTCAPTURE.Val1 * 5);
	mac_r(SYSTEM.INPUTCAPTURE.m3UphUVW.f16B, SYSTEM.INPUTCAPTURE.Val1, FRAC16(0.24288));
	
	SYSTEM.INPUTCAPTURE.m3UphUVW.f16C = (Frac16)(SYSTEM.INPUTCAPTURE.Val2 * 5);
	mac_r(SYSTEM.INPUTCAPTURE.m3UphUVW.f16C, SYSTEM.INPUTCAPTURE.Val2, FRAC16(0.24288));
	
	/*
	// Calculate clark park transform for voltages
	// Clark transform to get Ua, Ub
	MCLIB_ClarkTrf(&SYSTEM.INPUTCAPTURE.m2UAlphaBeta, &SYSTEM.INPUTCAPTURE.m3UphUVW);
	// Calculate park transform to get Ud, Uq
	// Out m2IDQ
	MCLIB_ParkTrf(&SYSTEM.INPUTCAPTURE.m2UDQ, &SYSTEM.INPUTCAPTURE.m2UAlphaBeta, &SYSTEM.POSITION.mSinCosAngle);	
	*/
	
	// Get measured angle
	// Get current position index
	SYSTEM.POSITION.i16SensorIndex = (Int16)(SYSTEM.ADC.f16SensorValueB >> 3);
	// Get filtered current position
	SYSTEM.POSITION.i16SensorIndexFiltered = (Int16)(SYSTEM.ADC.f16SensorValueBFiltered >> 3);
	// Add offset to index
	//SYSTEM.POSITION.i16SensorIndex += SYSTEM.POSITION.i16SensorIndexOffset;
	// Add phase delay
	//i16Temp = mult(SYSTEM.POSITION.i16SensorIndexPhaseDelay, SYSTEM.POSITION.f16SpeedFiltered);
	//SYSTEM.POSITION.i16SensorIndex = i16Temp + SYSTEM.POSITION.i16SensorIndex;
	// Wrap
    /*
     * TODO: Check wrap for negative speeds 
     *
     */	
	SYSTEM.POSITION.i16SensorIndex = SYSTEM.POSITION.i16SensorIndex & 4095;
	// Get measured angle from previous iteration
	SYSTEM.POSITION.f16MeasuredRotorAngle = SYSTEM.CALIBRATION.f16CalibrationArray[SYSTEM.POSITION.i16SensorIndex];
	// Add position offset from sensor delay
	SYSTEM.POSITION.f16MeasuredRotorAngle += SYSTEM.POSITION.f16AngleOffset;
	// Add fixed position offset
	SYSTEM.POSITION.f16MeasuredRotorAngle += SYSTEM.POSITION.f16AddedAngleOffset;
	// Calculate phase error
	// SYSTEM.POSITION.f16RotorAngle = calculated angle from previous iteration
	// SYSTEM.POSITION.f16MeasuredRotorAngle = current measured angle
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
			SYSTEM.POSITION.f16RotorAngle = FRAC16(0.0);
			SYSTEM.POSITION.f16Speed = FRAC16(0.0);
			SYSTEM.POSITION.f16SpeedFiltered = FRAC16(0.0);
			// Check align time
			if(0 == SYSTEM.SENSORLESS.i16Counter)
			{
				// Go to rotate
				SYSTEM.REGULATORS.m2IDQReq.f16Q = SYSTEM.REGULATORS.m2IDQReq.f16D;
				SYSTEM.REGULATORS.m2IDQReq.f16D = FRAC16(0.0);		
				
				SYSTEM.POSITION.i16PositionSource = POSITION_SOURCE_SENSORLESS_ROTATE;
				SYSTEM.REGULATORS.i16CurrentSource = CURRENT_SOURCE_SENSORLESS_ROTATE;
				// Reset bemf observer error part
				SYSTEM.SENSORLESS.f16BEMFErrorPart = FRAC16(0.0);
			}
			break;
		}
		case POSITION_SOURCE_SENSORLESS_ROTATE:
		{			
			// Use manual error
			i16RecorderTrigger = 1;
			
			if(SENSORLESS_BEMF_ON)
			{
				f16Temp = mult(SYSTEM.SENSORLESS.f16BEMFErrorPart, SYSTEM.POSITION.acBemfObsrvDQ.f16Error);
				if(FRAC16(1.0) > SYSTEM.SENSORLESS.f16BEMFErrorPart)
				{
					if(FRAC16(0.95) < SYSTEM.SENSORLESS.f16BEMFErrorPart)
					{
						SYSTEM.SENSORLESS.f16BEMFErrorPart = FRAC16(1.0);
					}
					else
					{
						SYSTEM.SENSORLESS.f16BEMFErrorPart += 20;
					}
					
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
						// Set current
						SYSTEM.REGULATORS.mudtControllerParamW.f32IntegPartK_1 = SYSTEM.REGULATORS.m2IDQReq.f16Q;
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
				SYSTEM.POSITION.f16RotorAngle = ACLIB_TrackObsrv(f16Temp, &SYSTEM.POSITION.acToPos);
				
			}
			else
			{
				SYSTEM.POSITION.f16RotorAngle = ACLIB_TrackObsrv(SYSTEM.SENSORLESS.f16AngleManualError, &SYSTEM.POSITION.acToPos);
			}
			
			// Store & filter speed
			SYSTEM.POSITION.f16Speed = extract_h(SYSTEM.POSITION.acToPos.f32Speed);
			SYSTEM.POSITION.f16SpeedFiltered = GDFLIB_FilterMA32(SYSTEM.POSITION.f16Speed, &SYSTEM.POSITION.FilterMA32Speed);	
			break;
		}
		case POSITION_SOURCE_SENSORLESS_MERGE:
		{
			// No source, abort
			SYSTEM.POSITION.i16PositionSource = POSITION_SOURCE_NONE;
			SYSTEM.REGULATORS.i16CurrentSource = CURRENT_SOURCE_NONE;
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
			// Use same counter as for speed control to slow down torque change
			SYSTEM.REGULATORS.ui16SpeedRegCounter++;
			if(SYSTEM.REGULATORS.ui16SpeedRegInterval < SYSTEM.REGULATORS.ui16SpeedRegCounter)
			{
				SYSTEM.REGULATORS.ui16SpeedRegCounter = 0;
				// Id = 0
				SYSTEM.REGULATORS.m2IDQReq.f16D = FRAC16(0.0);
				// Iq saturated?
				if(0 != SYSTEM.REGULATORS.mudtControllerParamIq.i16LimitFlag)
				{
					// Iq saturated, decrease it
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
					// Torque ramp
					if(SYSTEM.RAMPS.f16TorqueRampDesiredValue != SYSTEM.RAMPS.f16TorqueRampActualValue)
					{
						SYSTEM.RAMPS.f16TorqueRampActualValue = GFLIB_Ramp16(SYSTEM.RAMPS.f16TorqueRampDesiredValue, SYSTEM.RAMPS.f16TorqueRampActualValue, &SYSTEM.RAMPS.Ramp16_Torque);
					}
					// Scale with some factor
					SYSTEM.REGULATORS.m2IDQReq.f16Q = mult(SYSTEM.RAMPS.f16TorqueRampActualValue, SYSTEM.MCTRL.f16TorqueFactor);
				}
			}
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
				// SYSTEM.REGULATORS.mudtControllerParamIq.i16LimitFlag -> if 1, Iq regulator is saturated
				
				if(0 != SYSTEM.REGULATORS.mudtControllerParamIq.i16LimitFlag)
				{
					// If error will decrease required current, run regulation
					// Else do nothing					
					// Iq is saturated, run regulation, decrease Iq
					SYSTEM.REGULATORS.i16SatFlagW = 1;
					
					if(FRAC16(0.0) < SYSTEM.REGULATORS.m2IDQReq.f16Q)
					{
						if(f16SpeedErrorK <= FRAC16(0.0))
						{
							SYSTEM.REGULATORS.m2IDQReq.f16Q = GFLIB_ControllerPIp(f16SpeedErrorK, &SYSTEM.REGULATORS.mudtControllerParamW, &SYSTEM.REGULATORS.i16SatFlagW);
						}
					}
					else
					{
						if(f16SpeedErrorK >= FRAC16(0.0))
						{
							SYSTEM.REGULATORS.m2IDQReq.f16Q = GFLIB_ControllerPIp(f16SpeedErrorK, &SYSTEM.REGULATORS.mudtControllerParamW, &SYSTEM.REGULATORS.i16SatFlagW);
						}
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

		// Set Iq controller limits
		SYSTEM.REGULATORS.mudtControllerParamIq.f16UpperLimit = SYSTEM.REGULATORS.f16UqRemaining;
		SYSTEM.REGULATORS.mudtControllerParamIq.f16LowerLimit = -SYSTEM.REGULATORS.f16UqRemaining;
		
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
void PWM_A0_Reload_ISR(void)
{
	// Get phase voltage values
	SYSTEM.INPUTCAPTURE.Val0 = ioctl(QTIMER_B0, QT_READ_COUNTER_REG, NULL);
	SYSTEM.INPUTCAPTURE.Val1 = ioctl(QTIMER_B1, QT_READ_COUNTER_REG, NULL);
	SYSTEM.INPUTCAPTURE.Val2 = ioctl(QTIMER_B2, QT_READ_COUNTER_REG, NULL);
	// Reset timers
	ioctl(QTIMER_B0, QT_WRITE_COUNTER_REG, 0);
	ioctl(QTIMER_B1, QT_WRITE_COUNTER_REG, 0);
	ioctl(QTIMER_B2, QT_WRITE_COUNTER_REG, 0);
		
	// Clear reload flag
	ioctl(EFPWMA_SUB0, EFPWMS_CLEAR_SUBMODULE_FLAGS, EFPWM_RELOAD);
}

#pragma interrupt saveall
void GPIO_F_ISR(void)
{
	UInt16 ui16Temp = 0;
	// Read timer value
	ui16Temp = ioctl(QTIMER_B3, QT_READ_COUNTER_REG, NULL);
	// Clear counter
	ioctl(QTIMER_B3, QT_WRITE_COUNTER_REG, 0);
	// Check input value
	if(0 != (ioctl(GPIO_F, GPIO_READ_DATA, NULL) & BIT_7))
	{
		// Input value is 1
		// Read PWM high time
		SYSTEM.PWMIN.i16PWMHigh = ui16Temp;
		// Set detection polarity
		ioctl(GPIO_F, GPIO_INT_DETECTION_ACTIVE_LOW, BIT_7);
		
		// Filter PWM
		// Add to storage
		SYSTEM.PWMIN.i16PWMInFilterAcc += SYSTEM.PWMIN.i16PWMHigh;
		// Filter
		SYSTEM.PWMIN.i16PWMFiltered = SYSTEM.PWMIN.i16PWMInFilterAcc >> SYSTEM.PWMIN.i16PWMFilterSize;
		// Subtract from storage
		SYSTEM.PWMIN.i16PWMInFilterAcc -= SYSTEM.PWMIN.i16PWMFiltered;		
	}
	else
	{
		// Read PWM low time
		SYSTEM.PWMIN.i16PWMLow = ui16Temp;
		// Set detection polarity
		ioctl(GPIO_F, GPIO_INT_DETECTION_ACTIVE_HIGH, BIT_7);		
	}
	// Clear interrupt flag
	ioctl(GPIO_F, GPIO_CLEAR_INT_PENDING, BIT_7);
}

#pragma interrupt saveall
void QT_B3_ISR(void)
{
	// Clear input edge flag
	ioctl(QTIMER_B3, QT_CLEAR_FLAG, QT_INPUT_EDGE_FLAG);
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
	// Call master decode
	RS485_MasterecodeMessage(data);
	//ui8RS485RcvdByte = (UInt8)data;
	// Call state machine
	//RS485_States_slave((UInt8)data);
}

#pragma interrupt saveall
void TX0_Empty_ISR(void)
{
	//RS485_writeByte();
	RS485_MasterWriteByte();
}

#pragma interrupt saveall
void PIT_0_ISR(void)
{
	ioctl(PIT_0, PIT_CLEAR_ROLLOVER_INT, NULL);
	//RS485CommTest();
	// Check RS485 master
	RS485_States_Master();
	// Check system states
	checkSystemStates();
	// Recalculate SI values
	CalculateSIValues();
	// Call 1 ms event
	OneMsEvent();
	// Decrease counters
	if(0 < SYSTEM.SENSORLESS.i16Counter)
	{
		SYSTEM.SENSORLESS.i16Counter --;
	}
}
