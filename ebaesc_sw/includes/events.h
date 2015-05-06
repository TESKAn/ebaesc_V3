/*
 * events.h
 *
 *  Created on: Mar 29, 2015
 *      Author: Jure
 */

#ifndef EVENTS_H_
#define EVENTS_H_

void ADC_1_EOS_ISR(void);
void SPI_0_RX_FULL_ISR(void);
void PIT_0_ISR(void);

#endif /* EVENTS_H_ */
