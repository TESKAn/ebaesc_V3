/*
 * systemStates.h
 *
 *  Created on: Mar 29, 2015
 *      Author: Jure
 */

#ifndef SYSTEMSTATES_H_
#define SYSTEMSTATES_H_

Int16 checkSystemStates(void);
Int16 SystemWakeupState();
Int16 SystemInitState();
Int16 SystemIdleState();
Int16 SystemRunState();
Int16 SystemCalibrateState();
Int16 SystemParkRotorState();
Int16 SystemFaultState();
Int16 SystemResetState();
Int16 SystemFaultDRV83xxState();
Int16 SystemMeasureRPHAState();
Int16 SystemMeasureLPHAState();
Int16 SystemFaultOCEventState();
Int16 SystemStateTransition();

#endif /* SYSTEMSTATES_H_ */
