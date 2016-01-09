/*
 * macro.h
 *
 *  Created on: Mar 29, 2015
 *      Author: Jure
 */

#ifndef MACRO_H_
#define MACRO_H_

// Calibration parameters
#define M_FR

#ifdef M_FR
#define M_ID				0x23
#define M_MAXSENSORINDEX	3707
#define M_MINSENSORINDEX	431
#define M_POLEPAIRS			7
#define M_POLEARRAY_0		432
#define M_POLEARRAY_1		906
#define M_POLEARRAY_2		1374
#define M_POLEARRAY_3		1839
#define M_POLEARRAY_4		2306
#define M_POLEARRAY_5		2770
#define M_POLEARRAY_6		3239
#define M_POLEARRAY_7		433
// This unit ID
#define RS485_ID			0x23
// Unit park position
#define M_PARK_POSITION		1000

#endif

#ifdef M_FL
#define M_ID				0x22
#define M_MAXSENSORINDEX	3736
#define M_MINSENSORINDEX	418
#define M_POLEPAIRS			7
#define M_POLEARRAY_0		819
#define M_POLEARRAY_1		1287
#define M_POLEARRAY_2		1764
#define M_POLEARRAY_3		2248
#define M_POLEARRAY_4		2702
#define M_POLEARRAY_5		3167
#define M_POLEARRAY_6		3652
#define M_POLEARRAY_7		819
// This unit ID
#define RS485_ID			0x22
// Unit park position
#define M_PARK_POSITION		1000

#endif

#ifdef M_R
#define M_ID				0x21
#define M_MAXSENSORINDEX	3714
#define M_MINSENSORINDEX	416
#define M_POLEPAIRS			7
#define M_POLEARRAY_0		513
#define M_POLEARRAY_1		980
#define M_POLEARRAY_2		1450
#define M_POLEARRAY_3		1922
#define M_POLEARRAY_4		2394
#define M_POLEARRAY_5		2860
#define M_POLEARRAY_6		3335
#define M_POLEARRAY_7		514
// This unit ID
#define RS485_ID			0x21
// Unit park position
#define M_PARK_POSITION		1000

#endif

// Error definitions
#define ERROR_DRV8301_READ		1
#define ERROR_DRV8301_STAT1		2


// Macros for calculating SI values
#define SI_UIN_FACTOR		60.87f		// 60.87V max
#define SI_IIN_FACTOR		164.8f		// 164.8A max
#define SI_RPM_FACTOR		17142.8571f	//120.000/n pole pairs

// Macros for correcting angle offset
#define AOFFSET_FACTOR		0.7340032f	//(.000096*6*(32768/180))*7

// Limits to switch current amplification
#define DRV_A10_MIN			FRAC16(0.21)
#define DRV_A20_MIN			FRAC16(0.09)
#define DRV_A20_MAX			FRAC16(0.22)
#define DRV_A40_MIN			FRAC16(0.04)
#define DRV_A40_MAX			FRAC16(0.1)
#define DRV_A80_MAX			FRAC16(0.05)

// System macros
#define MANUAL_ANGLE_INCREASE		FRAC16(0.0001)
#define MANUAL_SPEED				FRAC16(0.0)

#define TORQUE_RAMP_UP              FRAC16(0.0002)//FRAC16(0.000056818182)
#define TORQUE_RAMP_DOWN            FRAC16(0.0002)//FRAC16(0.000056818182)

#define TORQUE_FACTOR				FRAC16(1.0)

#define SENSORLESS_MIN_SPEED		FRAC16(0.01)	// When to calculate BEMF observer
#define SENSORLESS_MAX_ERROR		FRAC16(0.005)	// If error below this, we use observer result
#define SENSORLESS_ALIGN_CURRENT	FRAC16(0.025)	// Rotor align current
#define SENSORLESS_START_CURRENT	FRAC16(0.025)	// Rotor start current
#define SENSORLESS_START_SPEED		FRAC16(0.1)
#define SENSORLESS_START_TORQUE		FRAC16(0.005)
#define SENSORLESS_ANGLE_MAN_ERROR	FRAC16(0.01)	
#define OL_I_RAMP_UP                FRAC16(0.0001)	//FRAC16(0.000056818182)
#define OL_I_RAMP_DOWN              FRAC16(0.0001)	//FRAC16(0.000056818182)
#define OL_ALIGN_TIME				2000			// Time in ms for aligning

// Position sources states
#define POSITION_SOURCE_NONE					0
#define POSITION_SOURCE_MANUAL					1
#define POSITION_SOURCE_SENSORLESS_ALIGN		2
#define POSITION_SOURCE_SENSORLESS_ROTATE		3
#define POSITION_SOURCE_SENSORLESS_MERGE		4
#define POSITION_SOURCE_MULTIPLE				5

// Current source states
#define CURRENT_SOURCE_NONE						0
#define CURRENT_SOURCE_CONTROL_TORQUE			1
#define CURRENT_SOURCE_CONTROL_SPEED			2
#define CURRENT_SOURCE_CONTROL_MANUAL			3
#define CURRENT_SOURCE_SENSORLESS_ALIGN			4
#define CURRENT_SOURCE_SENSORLESS_ROTATE		5

// Calibration measure state
#define CALIBRATE_INIT							0
#define CALIBRATE_START							1
#define CALIBRATE_FIND_ZERO_CROSS				2
#define CALIBRATE_VERIFY_POLE					3
#define CALIBRATE_WAIT_NEXT_POLE				4
#define CALIBRATE_FIND_AD_IN_MAX				5
#define CALIBRATE_FIND_AD_IN_MIN				6
#define CALIBRATE_CALCULATE_VALUES				7


// Flag definitions
#define DRV8301_CONFIGURED 			flag0.BIT0
#define SYSTEM_CALIBRATED			flag0.BIT1
#define SYS_DEBUG_MODE				flag0.BIT2		// Enable/disable some stuff
#define SYSTEM_TEST_BIT				flag0.BIT3
#define SYSTEM_RUN_SENSORED			flag0.BIT4
#define CONTROL_TORQUE				flag0.BIT5		// Control motor torque
#define CONTROL_SPEED				flag0.BIT6		// Control motor speed
#define CONTROL_MANUAL				flag0.BIT7		// Control Id manually
#define SYSTEM_RUN_MANUAL			flag0.BIT8		// Manual angle increase - for calibration etc.
#define SYSTEM_RUN_MANUAL_CW		flag0.BIT9		// Manual run - CW
#define SYSTEM_RUN_MANUAL_CCW		flag0.BIT10		// Manual run - CCW
#define SENSORLESS_RUN				flag0.BIT11		// Use sensorless
#define SENSORLESS_ALIGN			flag0.BIT12		// Align rotor
#define SENSORLESS_ROTATE			flag0.BIT13		// Blind rotate until we get some speed
#define SENSORLESS_BEMF_ON			flag0.BIT14		// BEMF observer is running
#define SYS_CAL_ZERO_CROSSED		flag0.BIT15

#define SYS_CAL_GOTO_NEXT_POLE		flag1.BIT0
#define SYS_ZERO_CURRENT			flag1.BIT1
#define PWM_ENABLED					flag1.BIT2		// Only do regulation when PWMs are enabled
#define SYSTEM_RECALCULATE_FACTORS	flag1.BIT3		// Recalculate gain, shift factors from float values
#define SYSTEM_TEST_NEW_STARTUP		flag1.BIT4		// Flag to test new startup procedure
#define DRV_READ_STATUS_REG			flag1.BIT5		// Mark read DRV8301 status reg
#define DRV_DATA_READ				flag1.BIT6		// 0 - read reg 1, 1 - read reg 2
#define DRV_WRITE_GAIN				flag1.BIT7		// Mark write new gain to MOSFET driver
#define DRV_POLL					flag1.BIT8		// Mark poll DRV
#define SYSTEM_PARK_ROTOR			flag1.BIT9		// Mark park rotor





// Driver HW interface defs
#define EN_GATE_ON					ioctl(GPIO_C, GPIO_SET_PIN, BIT_13)
#define EN_GATE_OFF					ioctl(GPIO_C, GPIO_CLEAR_PIN, BIT_13)

// System states
#define SYSTEM_WAKEUP						0
#define SYSTEM_INIT							1
#define SYSTEM_IDLE							2
#define SYSTEM_RUN							3
#define SYSTEM_FAULT						4
#define SYSTEM_RESET						5
#define SYSTEM_RESTARTING					6		// System restart after motor stalled
#define SYSTEM_FAULT_DRV8301				7
#define SYSTEM_FAULT_RESET					8
#define SYSTEM_BLOCKEXEC					9
#define SYSTEM_FOC_LOST_TIMEOUT				10
#define SYSTEM_PWM_IN_LOST					11
#define SYSTEM_CALIBRATE					12
#define SYSTEM_PARKROTOR					13
#define SYSTEM_SPINNINGROTOR				14
#define SYSTEM_MEAS_RPHA					15
#define SYSTEM_MEAS_LPHA					16

// PWM input default values
#define PWM_IN_MIDDLE_VALUE					2343	// 1.5625 MHz, 0.64 us/tick, ~1.5 ms
#define PWM_IN_HIGH_VAL_REF					2967	// 1900 us
#define PWM_IN_LOW_VAL_REF					1719	// 1100 us
#define PWM_IN_MEASURE_TIME					1000	// Time (in ms) to measure throttle stick position
#define PWM_IN_OFF_ZONE						15		// Deadzone from PWM in low to throttle OFF
// Default PWM timeout
#define PWM_IN_DEFAULT_TIMEOUT				1000	// 1000 ms
// PWM measure states
#define PWM_MEAS_INIT						0
#define PWM_MEAS_DECIDE						1
#define PWM_MEAS_HIGH						2
#define PWM_MEAS_LOW						3
// PWM measurement filter - shift by how much
#define PWM_MEAS_FILTER_SAMPLES				2
// Wait for this much samples on power up
#define PWM_MEAS_INITIAL_SAMPLES			100	

// Position sensor err values
#define POS_SENS_LOW						200
#define POS_SENS_HIGH						3900



#endif /* MACRO_H_ */
