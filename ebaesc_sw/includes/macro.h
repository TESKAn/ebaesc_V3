/*
 * macro.h
 *
 *  Created on: Mar 29, 2015
 *      Author: Jure
 */

#ifndef MACRO_H_
#define MACRO_H_

// Flag definitions
#define RUNNING_FROM_BEMF 			flag0.BIT0
#define DRV8301_CONFIGURED 			flag0.BIT1
#define SYSTEM_CALIBRATED			flag0.BIT2
#define SYS_DEBUG_MODE				flag0.BIT3		// Enable/disable some stuff

// Driver HW interface defs
#define EN_GATE_ON					ioctl(GPIO_C, GPIO_SET_PIN, BIT_13)
#define EN_GATE_OFF					ioctl(GPIO_C, GPIO_CLEAR_PIN, BIT_13)

// System states
#define SYSTEM_WAKEUP						0
#define SYSTEM_MEAS_RPHA					1
#define SYSTEM_MEAS_LPHA					2
#define SYSTEM_INIT							3
#define SYSTEM_IDLE							4
#define SYSTEM_ACTIVE						5
#define SYSTEM_RUN							6
#define SYSTEM_FAULT						7
#define SYSTEM_RESET						8
#define SYSTEM_RESTARTING					9		// System restart after motor stalled
#define SYSTEM_FAULT_DRV8301				10
#define SYSTEM_FAULT_RESET					11
#define SYSTEM_BLOCKEXEC					12
#define SYSTEM_FOC_LOST_TIMEOUT				13
#define SYSTEM_PWM_IN_LOST					14
#define SYSTEM_CALIBRATE					15
#define SYSTEM_PARKROTOR					16
#define SYSTEM_SPINNINGROTOR				17

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
