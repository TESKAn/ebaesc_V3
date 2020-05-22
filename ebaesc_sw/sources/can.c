/*
 * can.s
 *
 *  Created on: Jan 20, 2017
 *      Author: Jure
 */

#include "allincludes.h"

Int16 CAN_Init()
{
	int i = 0;
	FCAN_MB *MB;
	int code = 0;
	UWord32 uw32IDValue = 0;
	

	if(ioctl(FCAN, FCAN_TEST_READY, null))
	{
		for(i = 0; i < 8; i++)
		{		
			MB = ioctl(FCAN, FCAN_GET_MB_MODULE, i);

			code = ioctl(MB, FCANMB_GET_CODE, null);
			ioctl(MB, FCANMB_SET_CODE, FCAN_MB_CODE_TXVOID);	
		}
		
		for(i=8;i<16;i++)
		{
			MB = ioctl(FCAN, FCAN_GET_MB_MODULE, i);			
			ioctl(MB, FCANMB_SET_ID, 0x00015500 | FCAN_ID_EXT);	
			ioctl(MB, FCANMB_SET_CODE, FCAN_MB_CODE_RXEMPTY);
		}
		// Set mailbox 8 to receive RPM set signal
		uw32IDValue = CAN_RXMID_SETRPM;; 
		uw32IDValue = uw32IDValue << 8;
		MB = ioctl(FCAN, FCAN_GET_MB_MODULE, 8);			
		ioctl(MB, FCANMB_SET_ID, uw32IDValue | FCAN_ID_EXT);	
		ioctl(MB, FCANMB_SET_CODE, FCAN_MB_CODE_RXEMPTY);
		
		// Set mailbox 9 to receive set reg signal
		uw32IDValue = CAN_RXMID_SET_REG; 
		uw32IDValue = uw32IDValue << 8;
		MB = ioctl(FCAN, FCAN_GET_MB_MODULE, 9);			
		ioctl(MB, FCANMB_SET_ID, uw32IDValue | FCAN_ID_EXT);	
		ioctl(MB, FCANMB_SET_CODE, FCAN_MB_CODE_RXEMPTY);
		
		ioctl(FCAN, FCAN_UNLOCK_ALL_MB, null);
		
	}
	return 0;
}



Int16 CAN_CRCAdd(FCAN_MB *MB)
{
	// NE DELA!!!
	UInt16 value = 0xFFFF;
	
	t32BitVars t32bit;
	
	int i = 0;
	

	t32bit.uw32 = MB->data[0];
	
	for(i=0;i<4;i++)
	{	
		value ^= ((UInt16)(t32bit.bytes.ui8[i]) << 8);
        if (value & 0x8000U)
        {
            value = (value << 1) ^ 0x1021U;
        }
        else
        {
            value = (value << 1);
        }
	}	
	
	t32bit.uw32 = MB->data[1];
	
	for(i=0;i<4;i++)
	{	
		value ^= ((UInt16)(t32bit.bytes.ui8[i]) << 8);
        if (value & 0x8000U)
        {
            value = (value << 1) ^ 0x1021U;
        }
        else
        {
            value = (value << 1);
        }
	}	
	
}

UInt32 CAN_GenerateID(UInt32 ui32PRIO, UInt32 ui32MID)
{
	UInt32 ui32Temp = 0;
	UInt32 ui32MsgID = 0;

	// Generate message ID
	ui32Temp = ui32PRIO;
	ui32Temp = ui32Temp << 24;
	ui32MsgID = ui32Temp;

	ui32Temp = ui32MID;
	ui32Temp = ui32Temp << 8;
	ui32MsgID = ui32MsgID | ui32Temp;

	ui32Temp = (UWord32)COMMDataStruct.REGS.ui8ID;
	ui32MsgID = ui32MsgID | ui32Temp;

	return ui32MsgID;
}

Int16 CAN_TXStatus()
{
	FCAN_MB *MB;
	int i = 0;
	int code = 0;
	UWord32 uw32MessageID = 0;
	UWord32 uw32Temp = 0;
	t32BitVars t32bit;
	t32BitVars t32bit1;
	
	t32bit.bytes.ui8[0] = 0;
	t32bit.bytes.ui8[1] = 0;
	t32bit.bytes.ui8[2] = 0;
	
	switch(SYSTEM.systemState)
	{
		case SYSTEM_WAKEUP:
		case SYSTEM_INIT:
		{
			t32bit.bytes.ui8[3] = 1;	// MODE = initialization
			break;
		}
		case SYSTEM_IDLE:
		{
			t32bit.bytes.ui8[3] = 2;	// MODE = initialization
			break;
		}
		case SYSTEM_RUN:
		{
			t32bit.bytes.ui8[3] = 3;
			break;
		}
		case SYSTEM_FAULT:
		{
			t32bit.bytes.ui8[3] = 4;
			break;
		}
		case SYSTEM_RESET:
		case SYSTEM_RESTARTING:
		{
			t32bit.bytes.ui8[3] = 5;
			break;
		}
		case SYSTEM_FAULT_DRV8301:
		case SYSTEM_FAULT_RESET:
		case SYSTEM_BLOCKEXEC:
		case SYSTEM_FOC_LOST_TIMEOUT:
		{
			t32bit.bytes.ui8[3] = 6;
			break;
		}
		case SYSTEM_PWM_IN_LOST:
		{
			t32bit.bytes.ui8[3] = 7;
			break;
		}
		case SYSTEM_CALIBRATE:
		case SYSTEM_PARKROTOR:
		case SYSTEM_MEAS_RPHA:
		case SYSTEM_MEAS_LPHA:
		{
			t32bit.bytes.ui8[3] = 8;
			break;
		}
		case SYSTEM_FAULT_OCEVENT:
		{
			t32bit.bytes.ui8[3] = 9;
			break;
		}
		default:
		{
			t32bit.bytes.ui8[3] = 10;
			break;
		}
	}
	
	// Generate message ID
	uw32MessageID = CAN_GenerateID(CAN_PRIO_STATUS, CAN_TXMID_STATUS);	
	// Get free MB
	for(i=0;i<8;i++)
	{
		MB = ioctl(FCAN, FCAN_GET_MB_MODULE, i);
		// Get code
		code = ioctl(MB, FCANMB_GET_CODE, null);
		if(code == 0b1000)
		{
			MB->data[0] = t32bit.uw32;				

			t32bit.uw32 = 0;
			t32bit.bytes.ui8[0] = 0xc0;
			MB->data[1] = t32bit.uw32;
			
			ioctl(MB, FCANMB_REORDER_BYTES, NULL);
			
			ioctl(MB, FCANMB_SET_ID, uw32MessageID | FCAN_ID_EXT);
			ioctl(MB, FCANMB_SET_LEN, 5);
			ioctl(MB, FCANMB_SET_CODE, FCAN_MB_CODE_TXONCE);		
			i = 15;
		}
	}		
	return 0;
}

Int16 CAN_TXVoltage()
{
	FCAN_MB *MB;
	int i = 0;
	int code = 0;
	UWord32 uw32MessageID = 0;
	UWord32 uw32Temp = 0;
	t32BitVars t32bit;
	
	UInt16 ui16fUIn = 0;
	UInt16 ui16fIIn = 0;
	UInt16 ui16fTemp = 0;
	ui16fUIn = Float32ToFloat16(SYSTEM.SIVALUES.fUIn);
	ui16fIIn = Float32ToFloat16(SYSTEM.SIVALUES.fIIn);
	ui16fTemp = Float32ToFloat16(SYSTEM.SIVALUES.fTempPCB);
	
	if(ioctl(FCAN, FCAN_TEST_READY, null))
	{
		// Generate message ID
		uw32MessageID = CAN_GenerateID(CAN_PRIO_UIN, CAN_TXMID_UIN);		
		// Get free MB
		for(i=0;i<8;i++)
		{
			MB = ioctl(FCAN, FCAN_GET_MB_MODULE, i);
			// Get code
			code = ioctl(MB, FCANMB_GET_CODE, null);
			if(code == 0b1000)
			{
				// Write ID
				//MB->id = 0xff4e2123;
				// Write message
				t32bit.words.i16[1] = COMMDataStruct.REGS.i16UIn;
				t32bit.words.i16[0] = ui16fIIn;
				MB->data[0] = t32bit.uw32;
				t32bit.words.i16[1] = ui16fTemp;
				t32bit.bytes.ui8[1] = 0xc0;
				MB->data[1] = t32bit.uw32;
				
				ioctl(MB, FCANMB_SET_ID, uw32MessageID | FCAN_ID_EXT);
				ioctl(MB, FCANMB_SET_LEN, 7);
				ioctl(MB, FCANMB_SET_CODE, FCAN_MB_CODE_TXONCE);		
				i = 15;
			}
		}		
	}		
	return 0;
}

Int16 CAN_TXRPMInfo()
{
	FCAN_MB *MB;
	int i = 0;
	int code = 0;
	UWord32 uw32MessageID = 0;
	UWord32 uw32Temp = 0;
	t32BitVars t32bit;
	
	
	if(ioctl(FCAN, FCAN_TEST_READY, null))
	{
		// Generate message ID
		uw32MessageID = CAN_GenerateID(CAN_PRIO_RPMINFO, CAN_TXMID_RPMINFO);		
		// Get free MB
		for(i=0;i<8;i++)
		{
			MB = ioctl(FCAN, FCAN_GET_MB_MODULE, i);
			// Get code
			code = ioctl(MB, FCANMB_GET_CODE, null);
			if(code == 0b1000)
			{
				
				t32bit.words.i16[1] = (Int16)SYSTEM.SIVALUES.fRPM;
				t32bit.words.i16[0] = COMMDataStruct.REGS.i16SetRPM;
				MB->data[0] = t32bit.uw32;
				t32bit.uw32 = 0;
				t32bit.bytes.ui8[0] = 0xc0;
				MB->data[1] = t32bit.uw32;
				
				ioctl(MB, FCANMB_REORDER_BYTES, NULL);
				
				ioctl(MB, FCANMB_SET_ID, uw32MessageID | FCAN_ID_EXT);
				ioctl(MB, FCANMB_SET_LEN, 5);
				ioctl(MB, FCANMB_SET_CODE, FCAN_MB_CODE_TXONCE);		
				i = 15;
			}
		}			
	}		
	return 0;
}





Int16 CAN_RXRPM(FCAN_MB *MB)
{
	t32BitVars t32bit;
	
	// Flip bytes
	ioctl(MB, FCANMB_REORDER_BYTES, NULL);
	
	t32bit.uw32 = MB->data[0];
	// Check that speed is within limits
	
	if(COMMDataStruct.REGS.i16MinRPM > t32bit.words.i16[0])
	{
		COMMDataStruct.REGS.i16SetRPM = COMMDataStruct.REGS.i16MinRPM;
	}
	else if(COMMDataStruct.REGS.i16MaxRPM < t32bit.words.i16[0])
	{
		COMMDataStruct.REGS.i16SetRPM = COMMDataStruct.REGS.i16MaxRPM;
	}
	else
	{
		COMMDataStruct.REGS.i16SetRPM = t32bit.words.i16[0];
	}
	return 0;
}

Int16 CAN_SetReg(FCAN_MB *MB)
{
	t32BitVars t32bit;
	t32BitVars t32bit1;
	
	// Flip bytes
	ioctl(MB, FCANMB_REORDER_BYTES, NULL);
	
	t32bit1.uw32 = MB->data[0];
	
	// Check reg to set
	t32bit.bytes.ui8[0] = t32bit1.bytes.ui8[1];
	t32bit.bytes.ui8[1] = t32bit1.bytes.ui8[0];
	
	
	switch(t32bit.uwords.uw16[0])
	{

		case CAN_RXSETREG_ARMED:
		{       
			COMMDataStruct.REGS.ui8Armed = t32bit1.bytes.ui8[3];
			break;
		}
		case CAN_RXSETREG_MINRPM:
		{
			t32bit.bytes.ui8[2] = t32bit1.bytes.ui8[3];
			t32bit.bytes.ui8[3] = t32bit1.bytes.ui8[2];
			
			t32bit1.uw32 = MB->data[1];
			                 
			t32bit.bytes.ui8[0] = t32bit1.bytes.ui8[1];
			t32bit.bytes.ui8[1] = t32bit1.bytes.ui8[0];
			
			COMMDataStruct.REGS.i16MinRPM = t32bit.words.i16[0];
			break;
		}
		case CAN_RXSETREG_MAXRPM:
		{
			t32bit.bytes.ui8[2] = t32bit1.bytes.ui8[3];
			t32bit.bytes.ui8[3] = t32bit1.bytes.ui8[2];
			
			t32bit1.uw32 = MB->data[1];
			                 
			t32bit.bytes.ui8[0] = t32bit1.bytes.ui8[1];
			t32bit.bytes.ui8[1] = t32bit1.bytes.ui8[0];
			
			COMMDataStruct.REGS.i16MaxRPM = t32bit.words.i16[0];
			break;
		}
		case CAN_RXSETREG_RESET:
		{
			SYSTEM.i16StateTransition = SYSTEM_RESET;
			break;
		}
		default:
		{
			break;
		}
	}
	
	return 0;
}
