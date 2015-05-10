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

// Flag definitions
#define RUNNING_FROM_BEMF 			flag0.BIT0
#define DRV8301_CONFIGURED 			flag0.BIT1
#define SYSTEM_CALIBRATED			flag0.BIT2
#define SYS_DEBUG_MODE				flag0.BIT3		// Enable/disable some stuff
#define SYSTEM_TEST_BIT				flag0.BIT4
#define SYSTEM_RUN_SENSORED			flag0.BIT5
#define CONTROL_TORQUE				flag0.BIT6		// Control motor torque
#define CONTROL_SPEED				flag0.BIT7		// Control motor speed
#define CONTROL_MANUAL				flag0.BIT8		// Control Id manually
#define SYSTEM_RUN_MANUAL			flag0.BIT9		// Manual angle increase - for calibration etc.
#define SYSTEM_RUN_MANUAL_CW		flag0.BIT10		// Manual run - CW
#define SYSTEM_RUN_MANUAL_CCW		flag0.BIT11		// Manual run - CCW
#define SENSORLESS_RUN				flag0.BIT12		// Use sensorless
#define SENSORLESS_ALIGN			flag0.BIT13		// Align rotor
#define SENSORLESS_ROTATE			flag0.BIT14		// Blind rotate until we get some speed
#define SENSORLESS_MERGE			flag0.BIT15		// Merge from free running to observer
#define SENSORLESS_RUN				flag0.BIT16		// Sensorless FOC locked

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
