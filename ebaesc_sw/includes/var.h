/*
 * var.h
 *
 *  Created on: Mar 29, 2015
 *      Author: Jure
 */

#ifndef VAR_H_
#define VAR_H_

#include "allincludes.h"

typedef struct tagFLAGBITS 
{
  union 
  {
    UInt32 FLAG;
    struct 
    {
      unsigned BIT0:1;
      unsigned BIT1:1;
      unsigned BIT2:1;
      unsigned BIT3:1;
      unsigned BIT4:1;
      unsigned BIT5:1;
      unsigned BIT6:1;
      unsigned BIT7:1;
      unsigned BIT8:1;
      unsigned BIT9:1;
      unsigned BIT10:1;
      unsigned BIT11:1;
      unsigned BIT12:1;
      unsigned BIT13:1;
      unsigned BIT14:1;
      unsigned BIT15:1;
      unsigned BIT16:1;
      unsigned BIT17:1;
      unsigned BIT18:1;
      unsigned BIT19:1;
      unsigned BIT20:1;
      unsigned BIT21:1;
      unsigned BIT22:1;
      unsigned BIT23:1;
      unsigned BIT24:1;
      unsigned BIT25:1;
      unsigned BIT26:1;
      unsigned BIT27:1;
      unsigned BIT28:1;
      unsigned BIT29:1;
      unsigned BIT30:1;
      unsigned BIT31:1;
    };
  };
} FLAGBITS ;

typedef struct tagPARAMCONVERSION
{
	Int16 shift;
	Frac16 gain;
	float value;
}PARAMCONVERSION;


typedef struct tagSYSVARS
{
	// Motor control variables
	struct tagMCTRL
	{
		// alpha, beta currents after clark transform
		MCLIB_2_COOR_SYST_ALPHA_BETA_T m2IAlphaBeta;		
		// d, q currents after park transform
		MCLIB_2_COOR_SYST_D_Q_T m2IDQ;		
		// d, q voltages after regulation
		MCLIB_2_COOR_SYST_D_Q_T m2UDQ;
		// Saved d,q voltages
		MCLIB_2_COOR_SYST_D_Q_T m2UDQ_m;		
		// alpha, beta voltages after inverse park
		MCLIB_2_COOR_SYST_ALPHA_BETA_T m2UAlphaBeta;
		// alpha, beta voltages after ripple elimination
		MCLIB_2_COOR_SYST_ALPHA_BETA_T m2UAlphaBetaRippleElim;
		// U,V,W voltages after SVM
		MCLIB_3_COOR_SYST_T m3U_UVW;
		
	}MCTRL;
	
	// Position
	struct tagPOSITION
	{
		// Sin/Cos of angle
		MCLIB_ANGLE_T mSinCosAngle;		
		// BEMF observer - DQ
		ACLIB_BEMF_OBSRV_DQ_T acBemfObsrvDQ;
		// Angle tracking observer
		ACLIB_TRACK_OBSRV_T acToPos;	
		// Fractional value of speed
		Frac16 f16Speed;
		// Filtered speed fractional value
		Frac16 f16FilteredSpeed;
		// MA filter for speed
		GDFLIB_FILTER_MA32_T SpeedFilterMA32;
		// Value from position sensor
		Int16 i16SensorIndex;
		// Index in previous iteration
		Int16 i16SensorIndex_m;
		// Filtered index
		Int16 i16SensorIndexFiltered;
		// Position index offset
		Int16 i16SensorIndexOffset;
		// Phase delay from rotation speed
		Int16 i16SensorIndexPhaseDelay;
		// Angle values
		Frac16 f16RotorAngle;
		Frac16 f16RotorAngle_m;
		// Phase error
		Frac16 f16AnglePhaseError;
		// Kp, Ki - AcToPos
		float Kpactopos;
		float Kiactopos;
		
	}POSITION;
	
	// Calibration data
	struct tagCALIBRATION
	{
		Frac16 f16CalibrationArray[1024];
		Int16 i16MotorPolePairs;
		// Pole pair array
		Int16 i16PolePairArray[32];
		// Current pole pair
		Int16 i16CurrentPolePair;
		// Min/max values that we get from sensor
		Int16 i16MaxSensorIndex;
		Int16 i16MinSensorIndex;
		
	}CALIBRATION;
	

	
	// A/D variables
	struct tagADC
	{
		// Phase currents
		MCLIB_3_COOR_SYST_T m3IphUVW;	
		// Phase voltages
		MCLIB_3_COOR_SYST_T m3UphUVW;
		// Voltage
		Frac16 f16DCLinkVoltage;
		GDFLIB_FILTER_MA32_T FilterMA32DCLink;
		Frac16 f16DCLinkVoltageFiltered;
		// Temperature
		Frac16 f16Temperature;
		GDFLIB_FILTER_MA32_T FilterMA32Temperature;
		Frac16 f16TemperatureFiltered;
		// Raw angle value from sensor
		Frac16 f16SensorValueA;
		GDFLIB_FILTER_MA32_T FilterMA32SensorA;
		Frac16 f16SensorValueAFiltered;
		Frac16 f16SensorValueB;
		GDFLIB_FILTER_MA32_T FilterMA32SensorB;
		Frac16 f16SensorValueBFiltered;
	}ADC;
	
	// Regulators
	struct tagREGULATOR
	{
		// Regulators
		// D
		GFLIB_CONTROLLER_PI_P_PARAMS_T mudtControllerParamId;		
		// Q
		GFLIB_CONTROLLER_PI_P_PARAMS_T mudtControllerParamIq;		
		// W		
		GFLIB_CONTROLLER_PI_P_PARAMS_T mudtControllerParamW;
		// Required values for currents
		MCLIB_2_COOR_SYST_D_Q_T m2IDQReq;
		// Float values for parameters
		// Kp, Ki - D
		float Kpd;
		float Kid;
		// Kp, Ki - Q
		float Kpq;
		float Kiq;
		// Kp, Ki - W
		float Kpw;
		float Kiw;
	}REGULATORS;
	
	struct tagRAMPS
	{
		// Ramp for alignment current
		GFLIB_RAMP16_T Ramp16_Startup;
		Frac16 f16RampupRampDesiredValue;
		Frac16 f16RampupRampActualValue;	
		// Speed ramp
		GFLIB_RAMP16_T Ramp16_Speed;
		// Rampup vars
		Frac16 f16SpeedRampDesiredValue;
		Frac16 f16SpeedRampActualValue;
		// Torque ramp
		GFLIB_RAMP16_T Ramp16_Torque;
		// Rampup vars
		Frac16 f16TorqueRampDesiredValue;
		Frac16 f16TorqueRampActualValue;
	}RAMPS;
	
	struct tagPWMIN
	{
		// PWM input variables
		Int16 i16PWMfullThrottle;
		Int16 i16PWMoffThrottle;
		Int16 i16PWMinThrottle;
		Int16 i16PWMThrottleDifference;
		Int16 i16PWMFracMultiplier;
		// States for measuring PWM input parameters
		Int16 i16PWMMeasureStates;
		// PWM measure state timer
		Int16 i16PWMMeasureTimer; 
		// PWM timeout parameter
		Int16 i16PWMTimeout;
		// PWM input parameters
		Int16 i16PWMInMiddleValue;
		Int16 i16PWMInHighValRef;
		Int16 i16PWMInLowValRef;
		Int16 i16PWMInMeasureTime;
		Int16 i16PWMInOffZone;
		// Filtered PWM value
		Int16 i16PWMFiltered;
		// PWM samples received
		UInt32 ui32PWMSamplesReceived;
	}PWMIN;
	

	// Phase voltages
	MCLIB_3_COOR_SYST_T m3UphUVW;


	// PWM values
	pwm_sComplementaryValues PWMValues;
	
	// System state machine
	Int16 systemState;

	
}SYSTEMVARIABLES;

// Some flags
extern FLAGBITS flag0;
extern FLAGBITS flag1;

// Variable that holds all system data
extern SYSTEMVARIABLES SYSTEM;

// Variable for MOSFET driver
extern DRV8301DATA DRV8301;



extern PARAMCONVERSION pConv;



#endif /* VAR_H_ */
