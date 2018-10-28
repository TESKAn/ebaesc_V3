/*
 * functions.c
 *
 *  Created on: Mar 29, 2015
 *      Author: Jure
 */

#include "allincludes.h"

// Calculate NTC temperature
Int16 CalculateTemperature(Int16 valIndex)
{
	Int16 tempOut;
	Int16 i16Index_0;
	Int16 i16Index_1;
	Int16 i16Val0;
	Int16 i16Val1;
	
	i16Index_1 = valIndex  >> 10;
	i16Index_1 = i16Index_1 & 0x001F;
	tempOut = valIndex & 0x03FF;
	
	
	if(32 <= i16Index_1)
	{
		i16Index_1 = 31;
	}
	else if(0 >= i16Index_1)
	{
		i16Index_1 = 1;
	}
	i16Index_0 = i16Index_1 - 1;
	
	// i16Index_0 = 0...30
	// i16Index_1 = 1...31
	i16Val0 = i16TemperatureTable[i16Index_0];
	i16Val1 = i16TemperatureTable[i16Index_1];
	// dX = 1024
	// dY
	i16Val1 = i16Val1 - i16Val0;
	// dY * X
	tempOut = tempOut * i16Val1;
	// (dY*X)/dX
	tempOut = tempOut >> 10;
	// + Y0
	tempOut += i16Val0;
	
	return tempOut;
}

//#pragma interrupt called
Int16 LogError(UInt8 ui8Error)
{
	// Store error to log
	SYSTEM.i8ErrorLog[SYSTEM.i16ErrorIndex] = ui8Error;
	SYSTEM.i16ErrorIndex++;
	SYSTEM.i16ErrorIndex = SYSTEM.i16ErrorIndex & 0x0F;
	
	return 0;
}


//#pragma interrupt called
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

// Function to convert 32 bit float to 16 bit fixed point float
// Beware of byte order.
UInt16 Float32ToFloat16(float value)
{	
	t16BitVars t16Var1;
	t16BitVars t16Var2;
	

	const FP32 f32inf = { 255UL << 23U };
	const FP32 f16inf = { 31UL << 23U };
	const FP32 magic = { 15UL << 23U };
	const UInt32 sign_mask = 0x80000000UL;
	const UInt32 round_mask = ~0xFFFUL;

	FP32 in;
	UInt32 sign;
	    
	in.f = value;
	sign = in.u & sign_mask;
	in.u ^= sign;

	if (in.u >= f32inf.u)
	{
		t16Var1.ui16 = (in.u > f32inf.u) ? (UInt16)0x7FFFU : (UInt16)0x7C00U;
	}
	else
	{
	    in.u &= round_mask;
	    in.f *= magic.f;
	    in.u -= round_mask;
	    if (in.u > f16inf.u)
	    {
	        in.u = f16inf.u;
	    }
	    t16Var1.ui16 = (UInt16)(in.u >> 13U);
	}
	t16Var1.ui16 |= (UInt16)(sign >> 16U);
	
	t16Var2.bytes.ui8[0] = t16Var1.bytes.ui8[1]; 
	t16Var2.bytes.ui8[1] = t16Var1.bytes.ui8[0];

	return t16Var2.ui16;
}


// Function that calculates SI values from frac values
//#pragma interrupt called
Int16 CalculateSIValues(void)
{
	float fTemp = 0.0;
	float fTemp1 = 0.0;
	Frac32 f32Temp;
	Frac16 f16Temp;
	Frac16 f16Temp1;
	Int32 i32Temp = 0;
	Int32 i32Temp1 = 0;
	Int16 i16Temp = 0;
	
	// UIn
	fTemp = (float)SYSTEM.ADC.f16DCLinkVoltageFiltered;
	fTemp = fTemp * SI_UIN_FACTOR;
	SYSTEM.SIVALUES.fUIn = fTemp / 32768;
	
	// Pin
	fTemp = (float)SYSTEM.MCTRL.m2IDQ.f16Q;		// Iq
	fTemp = fTemp * SI_IIN_FACTOR;
	fTemp = fTemp / 32768;
	fTemp1 = (float)SYSTEM.MCTRL.m2UDQ.f16Q;	// Uq
	fTemp1 = fTemp1 * SYSTEM.SIVALUES.fUIn;
	fTemp1 = fTemp1 / 32768;
	SYSTEM.SIVALUES.fPIn = fTemp * fTemp1;		// P = Iq * Uq
	
	fTemp = (float)SYSTEM.MCTRL.m2IDQ.f16D;		// Id
	fTemp = fTemp * SI_IIN_FACTOR;
	fTemp = fTemp / 32768;
	fTemp1 = (float)SYSTEM.MCTRL.m2UDQ.f16D;	// Ud
	fTemp1 = fTemp1 * SYSTEM.SIVALUES.fUIn;
	fTemp1 = fTemp1 / 32768;
	SYSTEM.SIVALUES.fPIn += fTemp * fTemp1;		// P = Iq * Uq
	
	// Frac calc
	i32Temp = 0;
	
	f16Temp = mult(SYSTEM.MCTRL.m2IDQ.f16D, 1648);	// 0,1
	f16Temp1 = mult(SYSTEM.MCTRL.m2UDQ.f16D, 6087);	// 0,001
	
	L_mac(f16Temp, f16Temp1, i32Temp);	
	
	f16Temp = mult(SYSTEM.MCTRL.m2IDQ.f16Q, 1648);
	f16Temp1 = mult(SYSTEM.MCTRL.m2UDQ.f16Q, 6087);
	
	L_mac(f16Temp, f16Temp1, i32Temp); 
	
	// Calculate I
	// P is in 0,001 W scale
	// U voltage in 0,01 V scale
	// I is in 0,1 A scale
	i32Temp1 = i32Temp / (Int32)RS485DataStruct.REGS.i16UIn;
	RS485DataStruct.REGS.i16IIn = (Int16)i32Temp1;	
	// Scale is 1 mW, decrease to 1W
	i32Temp /= 1000;
	// Store to RS485 reg
	RS485DataStruct.REGS.i16PIn = (Int16)i32Temp;

	// IIn
	SYSTEM.SIVALUES.fIIn = SYSTEM.SIVALUES.fPIn / SYSTEM.SIVALUES.fUIn;
	// Filter I in
	SYSTEM.SIVALUES.fIInAcc += SYSTEM.SIVALUES.fIIn;
	SYSTEM.SIVALUES.fIInFilt = SYSTEM.SIVALUES.fIInAcc * SYSTEM.SIVALUES.fIInFiltDiv; 
	SYSTEM.SIVALUES.fIInAcc -= SYSTEM.SIVALUES.fIInFilt; 
	
	// RPM
	fTemp = (float)SYSTEM.POSITION.f16SpeedFiltered;
	fTemp = fTemp * SYSTEM.POSITION.Kthactopos;
	fTemp = fTemp * SI_WL_FACTOR;

	SYSTEM.SIVALUES.fRPM = fTemp / (float)SYSTEM.CALIBRATION.i16MotorPolePairs;
	
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
	
	
	i16Temp = CalculateTemperature(SYSTEM.ADC.f16TemperatureFiltered);
	SYSTEM.SIVALUES.fTempPCB = (float)i16Temp;
	
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
	int i;
	Int16 shift1;
	Int16 shift2;
	Int16 shift;
	Frac16 gain1;
	float x;
	float y;
	float z;
	float exp;
	float logK;
	float log05 = log10(0.5);
	float log2 = log10(2.0);
	
	logK = log10(K);
	
	x = logK - log05;
	x = x / log2;
	shift1 = (Int16)x;
	y = (float)shift1;
	if(y > x) shift1--;
	
	x = logK / log2;
	shift2 = (Int16)x;	
	y = (float)shift2;
	if(y < x) shift2++;
	
	x = (float)shift1;
	// Select the right shift
	for(i=shift2; i < shift1+1; i++)
	{
		z = K;
		shift = i;
		while(0 != shift)
		{
			if(0 < shift)
			{
				z = z/2;
				shift--;
			}
			else
			{
				z = z*2;
				shift++;
			}
		}
		if((0.5f <= z)&&(1.0f > z))
		{
			pConv.shift = i;
			z = z * 32768;
			pConv.gain = (Frac16)z;
			break;
		}
	}
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
	
	// Kthactopos	
	pConv.gain = SYSTEM.POSITION.acToPos.f16ThGain;
	pConv.shift = SYSTEM.POSITION.acToPos.i16ThGainShift;
	CalculateFloat();
	SYSTEM.POSITION.Kthactopos = pConv.value;
	
	// DQ Kp
	pConv.gain = SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.f16PropGain;
	pConv.shift = SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.i16PropGainShift;
	CalculateFloat();
	SYSTEM.POSITION.fKpBemfDQ = pConv.value;
	
	// DQ Ki
	pConv.gain = SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.f16IntegGain;
	pConv.shift = SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.i16IntegGainShift;
	CalculateFloat();
	SYSTEM.POSITION.fKiBemfDQ = pConv.value;
	
	// Scales
	SYSTEM.POSITION.f16Ifrac = SYSTEM.POSITION.acBemfObsrvDQ.f16IGain;
	SYSTEM.POSITION.f16Ufrac = SYSTEM.POSITION.acBemfObsrvDQ.f16UGain;
	SYSTEM.POSITION.f16Efrac = SYSTEM.POSITION.acBemfObsrvDQ.f16EGain;
	SYSTEM.POSITION.f16WIfrac = SYSTEM.POSITION.acBemfObsrvDQ.f16WIGain;	
}

// Stop motor - set all variables to required value
//#pragma interrupt called
void StopMotor(void)
{
	// Turn OFF PWM
	ioctl(EFPWMA, EFPWM_SET_OUTPUTS_DISABLE, EFPWM_SUB0_PWM_A|EFPWM_SUB0_PWM_B|EFPWM_SUB1_PWM_A|EFPWM_SUB1_PWM_B|EFPWM_SUB2_PWM_A|EFPWM_SUB2_PWM_B);
	// Reinitialise variables
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

//#pragma interrupt called
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

			
			
	DRV8301.CtrlReg1.GATE_CURRENT = 0;			// full current 1.7A
//	DRV8301.CtrlReg1.GATE_CURRENT = 1;			// med current 0.7A
//	DRV8301.CtrlReg1.GATE_CURRENT = 2;			// min current 0.25A

	DRV8301.CtrlReg1.GATE_RESET = 0;			// Normal Mode
			
	DRV8301.CtrlReg1.PWM_MODE = 0;				// six independant PWMs
			
	DRV8301.CtrlReg1.OC_MODE = 2;				// No shutdown
	//DRV8301.CtrlReg1.OC_MODE = 1;				// latched OC shutdown
			
	DRV8301.CtrlReg1.OC_ADJ_SET = 31;			//wCurrLimit;	// OC @ Vds=0.25V
	DRV8301.CtrlReg1.Reserved = 0;
			
	DRV8301.CtrlReg2.OCTW_SET = 0;				// report OT and OC
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
	if(rb->count < rb->size)
	{
		*rb->data_end = data;
		rb->data_end++;
		if (rb->data_end == rb->buffer_end)
		{
			rb->data_end = rb->buffer;
		}
		rb->count++;
	}
	else
	{
		// Return error
		return -1;
	}
	return 0;
}

UInt8 RB_pop(RING_BUFFER* rb)
{
	if(0 < rb->count)
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
	return -1;
}

Int16 RB_flush(RING_BUFFER* rb)
{	
    rb->data_start = rb->buffer;
    rb->data_end = rb->buffer;
    rb->count = 0;
    return 0;
}

Int16 MCAT_Calculate()
{
	float fTemp = 0;
	float fKe = 0;
	float fTemp1 = 0;
	float fOmegaMax = 0;
	float BemfDQ_Kps = 0;
	float BemfDQ_Kis = 0;
	
	// Calculate ke = V.sec/rad
	fKe = 0.1047f * SYSTEM.PMSMAPPCONFIG.fU_RPM;
	fKe = 1 / fKe;
	
	// Calculate omega max from max. E and fKe
	fOmegaMax = EMAX / fKe;
	
	// Calculate UFRAC
	fTemp = AD_SAMPLE_TIME * SYSTEM.MEASUREPARAMS.fRS;
	fTemp = fTemp + SYSTEM.MEASUREPARAMS.fLD;
	fTemp = fTemp * SI_IIN_FACTOR;
	fTemp1 = AD_SAMPLE_TIME * UMAX;
	fTemp1 = fTemp1 / fTemp;
	fTemp1 *= 32768;
	SYSTEM.POSITION.f16Ufrac = (Int16)fTemp1;
	
	// Calculate EFRAC
	fTemp = AD_SAMPLE_TIME * SYSTEM.MEASUREPARAMS.fRS;
	fTemp = fTemp + SYSTEM.MEASUREPARAMS.fLD;
	fTemp = fTemp * SI_IIN_FACTOR;
	fTemp1 = AD_SAMPLE_TIME * EMAX;
	fTemp1 = fTemp1 / fTemp;
	fTemp1 *= 32768;
	SYSTEM.POSITION.f16Efrac = (Int16)fTemp1;
	
	// WIfrac
	fTemp = AD_SAMPLE_TIME * SYSTEM.MEASUREPARAMS.fRS;
	fTemp = fTemp + SYSTEM.MEASUREPARAMS.fLD;
	fTemp1 = AD_SAMPLE_TIME * SYSTEM.MEASUREPARAMS.fLQ;
	fTemp1 = fTemp1 * fOmegaMax;
	fTemp1 = fTemp1 / fTemp;
	fTemp1 *= 32768;
	SYSTEM.POSITION.f16WIfrac = (Int16)fTemp1;
	
	// Ifrac
	fTemp = AD_SAMPLE_TIME * SYSTEM.MEASUREPARAMS.fRS;
	fTemp = fTemp + SYSTEM.MEASUREPARAMS.fLD;
	fTemp1 = SYSTEM.MEASUREPARAMS.fLD / fTemp;
	fTemp1 *= 32768;
	SYSTEM.POSITION.f16Ifrac = (Int16)fTemp1;
	
	// DQ KPs
	BemfDQ_Kps = 2*2*PI*(SYSTEM.MEASUREPARAMS.fLD - SYSTEM.MEASUREPARAMS.fRS);
	
	/*
    BEMF DQ observer constants 
    
    BemfDQ_Kps = 2*BEMF_DQ_att*2*Math.PI*BEMF_DQ_f0*Ld-Rs;
    BemfDQ_Kis = Ld*Math.pow(2*Math.PI*BEMF_DQ_f0,2);
    
    BemfDQ_Kpz = BemfDQ_Kps;
    BemfDQ_Kiz = BemfDQ_Kis*Ts;
    
    BemfDQ_Kpz_f = BemfDQ_Kpz*(Imax/Emax);
    BemfDQ_Kiz_f = BemfDQ_Kiz*(Imax/Emax);
    */
	
	return 0;
}

Int16 MCAT_Load()
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
	// Kth
	CalculateShiftGain(SYSTEM.POSITION.Kthactopos);
	SYSTEM.POSITION.acToPos.f16ThGain = pConv.gain;
	SYSTEM.POSITION.acToPos.i16ThGainShift = pConv.shift;
	
	// BEMF observer
	// DQ Kp
	CalculateShiftGain(SYSTEM.POSITION.fKpBemfDQ);
	SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.f16PropGain= pConv.gain;
	SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.i16PropGainShift= pConv.shift;	

	// DQ Ki
	CalculateShiftGain(SYSTEM.POSITION.fKiBemfDQ);
	SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.f16IntegGain= pConv.gain;
	SYSTEM.POSITION.acBemfObsrvDQ.udtCtrl.i16IntegGainShift= pConv.shift;	

	// Scales	
	SYSTEM.POSITION.acBemfObsrvDQ.f16IGain = SYSTEM.POSITION.f16Ifrac;
	SYSTEM.POSITION.acBemfObsrvDQ.f16UGain = SYSTEM.POSITION.f16Ufrac;
	SYSTEM.POSITION.acBemfObsrvDQ.f16EGain = SYSTEM.POSITION.f16Efrac;
	SYSTEM.POSITION.acBemfObsrvDQ.f16WIGain = SYSTEM.POSITION.f16WIfrac;	

	return 0;
}

Int16 CheckFaults()
{
	// Check driver fault
	if(0 == (ioctl(GPIO_C, GPIO_READ_DATA, NULL) & 0x0010))
	{
		SYSTEM.DRIVERSTATE.i8DriverFaultCount++;
		if(100 < SYSTEM.DRIVERSTATE.i8DriverFaultCount)
		{
			SYSTEM.DRIVERSTATE.i8DriverFaultCount = 100;
			SYSTEM.DRIVERSTATE.i8DriverFault = 1;
		}
	}
	else if(0 < SYSTEM.DRIVERSTATE.i8DriverFaultCount)
	{
		SYSTEM.DRIVERSTATE.i8DriverFaultCount--;
	}
		
	return 0;
}
