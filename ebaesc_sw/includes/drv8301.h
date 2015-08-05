/*
 * drv8301.h
 *
 *  Created on: Mar 29, 2015
 *      Author: Jure
 */

#ifndef DRV8301_H_
#define DRV8301_H_

typedef struct tagDRV8301
{
	// Reg request
	union
	{
		UInt16 reg;
		struct
		{
			UInt16 DATA:11;	
			UInt16 ADDR:4;
			UInt16 RW:1;
		};	
	}RegReq;
	
	// Status reg 1
	union
	{
		UInt16 reg;
		struct
		{
			UInt16 FETLC_OC:1;					// 0		Phase C, low-side FET OC
			UInt16 FETHC_OC:1;					// 1		Phase C, high-side FET OC
			UInt16 FETLB_OC:1;					// 2		Phase B, low-side FET OC
			UInt16 FETHB_OC:1;					// 3		Phase B, high-side FET OC
			UInt16 FETLA_OC:1;					// 4		Phase A, low-side FET OC
			UInt16 FETHA_OC:1;					// 5		Phase A, high-side FET OC
			UInt16 OTW:1;	        			// 6		Over-temperature warning
			UInt16 OTSD:1;	          			// 7		Over-temperature shut down
			UInt16 PVDD_UV:1;					// 8		PVDD < 6V
			UInt16 GVDD_UV:1;					// 9		GVDD < 8V
			UInt16 FAULT:1;						// 10		FAULTn pin is asserted
			//UINT16 Reserved:5;					// 15:11 
			UInt16 ADDR:4;
			UInt16 RW:1;
		};	
	}StatReg1;
		
	// Status reg 2
	union
	{
		UInt16 reg;
		struct
		{
			UInt16 DEVICE_ID:4;					// 3:0		Device ID
			UInt16 Reserved1:3;					// 6:4
			UInt16 GVDD_OV:1;					// 7		GVDD > 16V
			//UINT16 Reserved2:8;				// 15:8 
			UInt16 Reserved2:3;
			UInt16 ADDR:4;
			UInt16 RW:1;		
		};	
	}StatReg2;
	
	// Control reg 1
	union
	{
		UInt16 reg;
		struct
		{
			UInt16 GATE_CURRENT:2;				// 1:0		Gate driver peak current, 1.7A (00b), 0.7A (01b), 0.25A (10b), Reserved (11b)
			UInt16 GATE_RESET:1;        		// 2		Reset all latched faults (1), Normal Mode (0)
			UInt16 PWM_MODE:1;          		// 3		Three (1) or six (0) pwm inputs
			UInt16 OC_MODE:2;					// 5:4		over-current mode, current limit (00b), latched shut down (01b), Report only (10b), OC disabled (11b)
			UInt16 OC_ADJ_SET:5;				// 10:6		Set Vds trip point for OC see the table in the datasheet
			UInt16 Reserved:5;					// 15:11 
		};	
	}CtrlReg1;
	// Control reg 2
	union
	{
		UInt16 reg;
		struct
		{
			UInt16 OCTW_SET:2;					// 1:0		Report OT and OC (00b), Report OT (01b), Report OC (10b), Reserved (11b)
			UInt16 GAIN:2;        				// 3:2		Gain of shunt amplifier, 10 (00b), 20 (01b), 40 (10b), 80 (11b)
			UInt16 DC_CAL_CH1:1;        		// 4		Shunt amplifier inputs shorted and disconnected from load (1) or shunt amplifier inputs connected to load (0)
			UInt16 DC_CAL_CH2:1;        		// 5		Shunt amplifier inputs shorted and disconnected from load (1) or shunt amplifier inputs connected to load (0)
			UInt16 OC_TOFF:1;					// 6		Normal CBC operation (0), off time control during OC (1)
			UInt16 Reserved:9;					// 15:7 
		};	
	}CtrlReg2;
}DRV8301DATA;

//DRV8301 Register Addresses
#define DRV8301_STAT_REG_1_ADDR		0x00
#define DRV8301_STAT_REG_2_ADDR		0x01
#define DRV8301_CNTRL_REG_1_ADDR	0x02
#define DRV8301_CNTRL_REG_2_ADDR	0x03

#define DRV8301_GAIN_10 0;// CS amplifier gain = 10, 164.8A max
#define DRV8301_GAIN_20 1;// CS amplifier gain = 20 *.5, 82,4A max
#define DRV8301_GAIN_40 2;// CS amplifier gain = 40 *.25, 41,2A max
#define DRV8301_GAIN_80 3;// CS amplifier gain = 80 *.125, 20,6A max

#define DRV8301_GAIN_AMP		DRV8301_GAIN_40
#define DRV8301_GAIN_FACTOR	FRAC16(0.25)


#endif /* DRV8301_H_ */
