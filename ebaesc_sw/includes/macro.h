/*
 * macro.h
 *
 *  Created on: Mar 29, 2015
 *      Author: Jure
 */

#ifndef MACRO_H_
#define MACRO_H_

// System macros
#define MANUAL_ANGLE_INCREASE		FRAC16(0.0001)
#define MANUAL_SPEED				FRAC16(0.0)

#define TORQUE_RAMP_UP              FRAC16(0.0002)//FRAC16(0.000056818182)
#define TORQUE_RAMP_DOWN            FRAC16(0.0002)//FRAC16(0.000056818182)

#define TORQUE_FACTOR				FRAC16(1.0)

#define SENSORLESS_MIN_SPEED		FRAC16(0.003)	// When to calculate BEMF observer
#define SENSORLESS_MAX_ERROR		FRAC16(0.005)	// If error below this, we use observer result
#define SENSORLESS_ALIGN_CURRENT	FRAC16(0.025)	// Rotor align current
#define SENSORLESS_START_CURRENT	FRAC16(0.025)	// Rotor start current
#define SENSORLESS_MAX_RAMP_INC		FRAC16(0.004)	// Max speed for OL
#define SENSORLESS_START_SPEED		FRAC16(0.1)
#define SENSORLESS_START_TORQUE		FRAC16(0.005)
#define OL_I_RAMP_UP                FRAC16(0.0001)	//FRAC16(0.000056818182)
#define OL_I_RAMP_DOWN              FRAC16(0.0001)	//FRAC16(0.000056818182)
#define OL_ANGLE_TO_SPEED			FRAC16(0.5)
#define OL_ALIGN_TIME				2000			// Time in ms for aligning
#define OL_MERGE_SPEED_DIFFERENCE	FRAC16(0.01)
#define OL_MERGE_SPEED_COUNT		100

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
#define PWM_IN_MIDDLE_VALUE					2250	// 1.5 MHz, 0.6666 us/tick, ~1.5 ms
#define PWM_IN_HIGH_VAL_REF					2850	// 1900 us
#define PWM_IN_LOW_VAL_REF					1650	// 1100 us
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



#endif /* MACRO_H_ */
