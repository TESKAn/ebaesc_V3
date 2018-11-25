/*
 * var.h
 *
 *  Created on: Mar 29, 2015
 *      Author: Jure
 */

#ifndef VAR_H_
#define VAR_H_

#include "allincludes.h"


typedef union tag32BitVars
{
	union
	{
		struct
		{
			UInt8 ui8[4];
		}bytes;
		struct
		{
			UWord16 uw16[2];
		}uwords;
		struct
		{
			Int16 i16[2];
		}words;
		UWord32 uw32;
		UInt32 ui32;
		Int32 i32;
		Frac32 f32;
		Word32 w32;
		float f;
	};
}t32BitVars;

typedef union tagFP32
{
    UInt32 u;
    float f;
}FP32;

typedef union tag16BitVars
{
	union
	{
		struct
		{
			UInt8 ui8[2];
		}bytes;
		struct
		{
			Int8 i8[2];
		}ints;
		UWord16 uw16;
		UInt16 ui16;
		Int16 i16;
		Frac16 f16;
		Word16 w16;
	};
}t16BitVars;


typedef struct
{
	UInt8* buffer;
	UInt8* buffer_end;
	UInt8* data_start;
	UInt8* data_end;
     volatile Int32 count;
     Int32 size;
 }RING_BUFFER;

typedef struct tagFLAGBITS 
{
  union 
  {
    UInt16 FLAG;
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
    };
  };
}FLAGBITS;

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
		// Voltages from X inputs
		MCLIB_3_COOR_SYST_T m3U_X_UVW;
		// Sector where the phase voltage vector is
		UInt16 SVMVoltageSector;
		// Torque -> Iq factor
		Frac16 f16TorqueFactor;
		// Total phase current 
		Frac16 f16IPh;
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
		Frac16 f16SpeedFiltered;
		// MA filter for speed
		GDFLIB_FILTER_MA32_T FilterMA32Speed;
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
		// Previous angle 
		Frac16 f16RotorAngle_m;
		// Angle value from sensor
		Frac16 f16MeasuredRotorAngle;
		// Phase error from observer
		Frac16 f16ObserverAngleError;
		// Phase error from measurement
		Frac16 f16MeasuredAngleError;
		// Phase error
		Frac16 f16AnglePhaseError;
		// Kp, Ki, Kth - AcToPos
		float Kpactopos;
		float Kiactopos;
		float Kthactopos;
		// Manual angle increase
		Frac16 f16ManualAngleIncrease;
		// Position source
		Int16 i16PositionSource;
		// Offset from sensor delay
		Frac16 f16AngleOffset;
		// Add additional angle offset
		Frac16 f16AddedAngleOffset;
		// Factor for calculating offset
		float fOffsetCalcFactor;
		
		// Observer parameters
		float fKpBemfDQ;
		float fKiBemfDQ;
		Frac16 f16Ufrac;
		Frac16 f16Efrac;
		Frac16 f16WIfrac;
		Frac16 f16Ifrac;
		
	}POSITION;
	
	struct tagSENSORLESS
	{
		// When to start observer
		Frac16 f16MinSpeed;
		// Start speed hysteresis
		Frac16 f16MinSpeedHysteresis;
		// Max error for BEMF observer to use it
		Frac16 f16MaxObserverError;
		// Align current
		Frac16 f16AlignCurrent;
		// Start current
		Frac16 f16StartCurrent;
		// Time counter
		Int16 i16Counter;
		// Align time
		Int16 i16AlignTime;
		// Speed regulator start speed
		Frac16 f16StartSpeed;
		// Torque regulator start torque
		Frac16 f16StartTorque;
		// Angle increase on each iteration
		Frac16 f16AngleManualError;
		// How much BEMF error to use?
		Frac16 f16BEMFErrorPart;		
		// Detection count - observer error error
		UInt8 ui8BemfObserverErrorCount;
		// Max observer error count before restart
		UInt8 ui8MaxBemfObserverErrorCount;
		// Previous BEMF error
		Frac16 f16PrevBEMFObserverError;
		// Merge
	}SENSORLESS;
	
	// Calibration data
	struct tagCALIBRATION
	{
		Frac16 f16CalibrationArray[4096];
		Int16 i16MotorPolePairs;
		// Pole pair array
		Int16 i16PolePairArray[32];
		// Current pole pair
		Int16 i16CurrentPolePair;
		// Min/max values that we get from sensor
		Int16 i16MaxSensorIndex;
		Int16 i16MinSensorIndex;
		// Calibration states
		Int16 i16CalibrationState;
		// Time counter
		Int16 i16Counter;
		
	}CALIBRATION;
	

	
	// A/D variables
	struct tagADC
	{
		// Phase currents
		MCLIB_3_COOR_SYST_T m3IphUVW;	
		// Raw phase currents
		MCLIB_3_COOR_SYST_T m3IphUVWRaw;
		// Offsets
		Frac16 f16OffsetU;
		Frac16 f16OffsetV;
		Frac16 f16OffsetW;
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
		// Factor to account for different current amplifications
		Frac16 f16CurrentGainFactor;
		// Count number of cycles for offset measurement
		Int16 i16ADCOffsetMeasurements;
	}ADC;
	
	// Input capture PWM values
	struct tagINPUTCAPTURE
	{
		UInt16 Val0;
		UInt16 Val1;
		UInt16 Val2;
		// Fractional values of voltages
		MCLIB_3_COOR_SYST_T m3UphUVW;
		// Measured voltages after clark transform
		MCLIB_2_COOR_SYST_ALPHA_BETA_T m2UAlphaBeta;
		// Measured voltages after park transform
		MCLIB_2_COOR_SYST_D_Q_T m2UDQ;	
		
		// Values in PWM compare registers
		Word16 w16PWM0VAL2;
		Word16 w16PWM0VAL3;
		
	}INPUTCAPTURE;
	
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
		// Speed regulator interval time
		UInt16 ui16SpeedRegCounter;
		UInt16 ui16SpeedRegInterval;
		// Saturation flags
		Int16 i16SatFlagD;
		Int16 i16SatFlagQ;
		Int16 i16SatFlagW;
		
		// Current reference source
		Int16 i16CurrentSource;
		// Voltage remaining for Uq
		Frac16 f16UqRemaining;
		
	}REGULATORS;
	
	struct tagRAMPS
	{
		// Ramp for alignment current
		GFLIB_RAMP16_T Ramp16_AlignCurrent;
		// Rampup vars
		Frac16 f16AlignCurrentDesiredValue;
		Frac16 f16AlignCurrentActualValue;	

		// Speed ramp
		GFLIB_RAMP16_T Ramp16_Speed;
		// Rampup vars
		Frac16 f16SpeedRampDesiredValue;
		Frac16 f16SpeedRampActualValue;
		
		float fSRampUp;
		float fSRampDown;
		
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
		// Storage for filter
		Int16 i16PWMInFilterAcc;
		// Filter size
		Int16 i16PWMFilterSize;
		// Raw PWM input value
		Int16 i16PWMHigh;
		Int16 i16PWMLow;
		// PWM samples received
		UInt32 ui32PWMSamplesReceived;
		// Factors
		float fPWMFactor;
		float fPWMOffset;
		
		
	}PWMIN;
	
	struct tagSIVALUES
	{
		float fUIn;
		float fIIn;
		float fIInFilt;
		float fPIn;
		float fRPM;
		float fIInAcc;
		float fIInFiltDiv;
		float fTempPCB;
		float fTempExt;
	}SIVALUES;
	
	// CAN variables
	struct tagCAN
	{
		UInt16 ui16CANInfoTimer;
		UInt16 ui16CANInfoInterval;
	}CAN;
	
	// Measure motor parameters
	struct tagMEASUREPARAMS
	{
		Frac16 f16TempDelta;
		Frac16 f16MeasureRPhaId;
		Frac16 f16U1;
		Frac16 f16U2;
		Int16 i16ITauTicks;
		Int16 i16Measurements;
		Int16 i16TotalMeasurements;
		Int16 i16TotalTicks;
		UWord16 uw16SAR;
		UInt16 ui16LPhaTime;
		Int16 i16StabilizeCounter;
		UWord16 uw16Setpoint;
		Frac16 f16SetpointMulti;
		Frac16 f16LphaIset;
		Frac16 f16LphaITrig;
		pwm_sComplementaryValues pwmSubReqPWMValues;
		Frac16 f16IphAValue;
		Frac16 f1663PercentValue;
		Int16 i16TempValue;
		float fMeasuredRTotal;
		float fMeasuredLTotal;
		float fMeasuredRPha;
		float fMeasuredLPha;
		
		float fLD;
		float fLQ;
		float fRS;
		
	}MEASUREPARAMS;
	
	struct tagPMSMAppConfig
	{
		// D/Q current regulators
		float fCLoop_lim;
		
		float fID_KP;
		float fID_KI;
		float fID_KD;
		
		float fIQ_KP;
		float fIQ_KI;
		float fIQ_KD;
		
		// Speed loop		
		float fSLoop_HLim;
		float fSLoop_LLim;
		

		
		float fSLoop_KP;
		float fSLoop_KI;
		float fSLoop_KD;
		
		//Bemf DQ Observer       

		float fU_RPM;

	}PMSMAPPCONFIG;
	
	struct tagDRIVERSTATE
	{
		Int8 i8DriverFaultCount;
		Int8 i8DriverFault;
	}DRIVERSTATE;

	struct tagMEASUREWEL
	{
		Frac16 f16AngleDiff;
		Int16 i16ADCycleCounter;
		Int16 i16ADCycles;
		Int16 i16WEl;
		Int16 i16WMot;
	}MEASUREWEL;
	// Phase voltages
	MCLIB_3_COOR_SYST_T m3UphUVW;
	
	Int16 i16ADInterruptCycleTime;


	// PWM values
	pwm_sComplementaryValues PWMValues;
	
	// System state machine
	Int16 systemState;
	Int16 i16StateTransition;
	
	// Driver restart after fault
	Int16 i16DriverRestartState;
	Int16 i16DriverRestartTimer;
	
	// Measure Rpha state
	Int16 i16MotorRPhaMeasureState;

	// Measure Lpha state
	Int16 i16MotorLPhaMeasureState;

	// Motor ID
	Int16 i16MotorID;
	
	Int8 i8ErrorLog[16];
	Int16 i16ErrorIndex;
	
	Int16 i16EEPROMStoreDone;
	
	
	
}SYSTEMVARIABLES;

// EEPROM test
extern Int8 i8EEPROMOp;
extern UWord32 uw32EEPROMAddress;
extern UWord16 uw16EEPROMData;
// End eeprom test

// CAN test
extern Int8 i8CANTest;

// Parameters test 
extern Int8 i8ParamTest;

// LED test
extern Int8 i8LEDTest;
extern Int16 i16LEDToggleCount;
extern Int16 i16LEDToggles;

// Some flags
extern FLAGBITS flag0;
extern FLAGBITS flag1;

// NTC temperature
extern Int16 i16TemperatureTable[32];

extern Int16 i16CurrentCalArrayIndex;
extern Frac16 f16CurrentCalArrayData;

// RS485 temporary vars
extern UInt8 ui8RS485RXVal;

// Variable that holds all system data
extern SYSTEMVARIABLES SYSTEM;

// Variable for MOSFET driver
extern DRV8301DATA DRV8301;

extern Int16 i16RecorderTrigger;

extern PARAMCONVERSION pConv;

extern UInt8 ui8SerialBuffer[16];
extern UInt8 RS485Address;
extern UInt8 ui8RS485RcvdByte;

// Transmit ring buffer
extern RING_BUFFER SCI0RXBuff;
extern UInt8 SCI0RXBuffer[256];

extern RS485MOTOR RS485DataStruct;

extern Int32 i32Var;

extern UInt16 ui16CANTestCounter;

#endif /* VAR_H_ */
