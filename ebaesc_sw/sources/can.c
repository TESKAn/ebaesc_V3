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
		// Set mailbox 9 to receive RPM limits
		uw32IDValue = CAN_MID_SETRPMLIMIT; 
		uw32IDValue = uw32IDValue << 8;
		MB = ioctl(FCAN, FCAN_GET_MB_MODULE, 9);			
		ioctl(MB, FCANMB_SET_ID, uw32IDValue | FCAN_ID_EXT);	
		ioctl(MB, FCANMB_SET_CODE, FCAN_MB_CODE_RXEMPTY);
		
		// Set mailbox 10 to receive RPM setting limits
		switch(COMMDataStruct.REGS.ui8ID)
		{
			case CAN_MOTOR_FR_ID:
			{
				uw32IDValue = CAN_MID_SETRPM_FR; 
				break;
			}
			case CAN_MOTOR_FL_ID:
			{
				uw32IDValue = CAN_MID_SETRPM_FL; 
				break;
			}
			case CAN_MOTOR_RR_ID:
			{
				uw32IDValue = CAN_MID_SETRPM_RR; 
				break;
			}
			case CAN_MOTOR_RL_ID:
			{
				uw32IDValue = CAN_MID_SETRPM_RL; 
				break;
			}
		}
		
		uw32IDValue = uw32IDValue << 8;
		MB = ioctl(FCAN, FCAN_GET_MB_MODULE, 10);			
		ioctl(MB, FCANMB_SET_ID, uw32IDValue | FCAN_ID_EXT);	
		ioctl(MB, FCANMB_SET_CODE, FCAN_MB_CODE_RXEMPTY);
		
		// Set mailbox 11 to receive enable signal
		uw32IDValue = CAN_MID_ENABLE; 
		uw32IDValue = uw32IDValue << 8;
		MB = ioctl(FCAN, FCAN_GET_MB_MODULE, 11);			
		ioctl(MB, FCANMB_SET_ID, uw32IDValue | FCAN_ID_EXT);	
		ioctl(MB, FCANMB_SET_CODE, FCAN_MB_CODE_RXEMPTY);
		
		// Set mailbox 12 to receive reset signal
		uw32IDValue = CAN_MID_RESET_ESC; 
		uw32IDValue = uw32IDValue << 8;
		MB = ioctl(FCAN, FCAN_GET_MB_MODULE, 12);			
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
	
	
	UInt16 ui16fUIn = 0;
	UInt16 ui16fIIn = 0;
	UInt16 ui16fTemp = 0;
	ui16fUIn = Float32ToFloat16(SYSTEM.SIVALUES.fUIn);
	ui16fIIn = Float32ToFloat16(-53.10f);
	ui16fTemp = Float32ToFloat16(SYSTEM.SIVALUES.fTempPCB);
	
	if(ioctl(FCAN, FCAN_TEST_READY, null))
	{	
		// Get free MB
		for(i=0;i<8;i++)
		{
			MB = ioctl(FCAN, FCAN_GET_MB_MODULE, i);
			// Get code
			code = ioctl(MB, FCANMB_GET_CODE, null);
			if(code == 0b1000)
			{
				// Generate message ID				
				uw32MessageID = CAN_GenerateID(CAN_PRIO_STATUS, CAN_MID_STATUS);
				
				// Calculate time in seconds
				t32bit.uw32 = SYSTEM.ui32SystemTime / 1000;
				// Flip bytes
				t32bit1.bytes.ui8[0] = t32bit.bytes.ui8[3];
				t32bit1.bytes.ui8[1] = t32bit.bytes.ui8[2];
				t32bit1.bytes.ui8[2] = t32bit.bytes.ui8[1];
				t32bit1.bytes.ui8[3] = t32bit.bytes.ui8[0];
				// Store
				MB->data[0] = t32bit1.uw32;
				// 2 bits health
				// 3 bits mode
				// 3 bits submode
				switch(SYSTEM.systemState)
				{
					case SYSTEM_WAKEUP:
					case SYSTEM_INIT:
					{
						t32bit.bytes.ui8[3] = 8;	// MODE = initialization
						break;
					}
					case SYSTEM_IDLE:
					case SYSTEM_RUN:
					{
						t32bit.bytes.ui8[3] = 0;
						break;
					}
					case SYSTEM_FAULT:
					{
						t32bit.bytes.ui8[3] = 0xc0;
						break;
					}
					case SYSTEM_RESET:
					case SYSTEM_RESTARTING:
					{
						t32bit.bytes.ui8[3] = 0x40;
						break;
					}
					case SYSTEM_FAULT_DRV8301:
					case SYSTEM_FAULT_RESET:
					case SYSTEM_BLOCKEXEC:
					case SYSTEM_FOC_LOST_TIMEOUT:
					{
						t32bit.bytes.ui8[3] = 0xc0;
						break;
					}
					case SYSTEM_PWM_IN_LOST:
					{
						t32bit.bytes.ui8[3] = 0x40;
						break;
					}
					case SYSTEM_CALIBRATE:
					case SYSTEM_PARKROTOR:
					case SYSTEM_MEAS_RPHA:
					case SYSTEM_MEAS_LPHA:
					{
						t32bit.bytes.ui8[3] = 0;
						break;
					}
					case SYSTEM_FAULT_OCEVENT:
					{
						t32bit.bytes.ui8[3] = 0xc0;
						break;
					}
					default:
					{
						t32bit.bytes.ui8[3] = 0xc0;
						break;
					}
				}
				// 16 bits specific status code
				// Send system state
				t32bit1.words.i16[0] = SYSTEM.systemState;
				
				t32bit.bytes.ui8[2] = t32bit1.bytes.ui8[0];
				t32bit.bytes.ui8[1] = t32bit1.bytes.ui8[1];

				t32bit.bytes.ui8[0] = 0xc0;
				MB->data[1] = t32bit.uw32;
				
				ioctl(MB, FCANMB_SET_ID, uw32MessageID | FCAN_ID_EXT);
				ioctl(MB, FCANMB_SET_LEN, 8);
				ioctl(MB, FCANMB_SET_CODE, FCAN_MB_CODE_TXONCE);		
				i = 15;
			}
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
		uw32MessageID = CAN_GenerateID(CAN_PRIO_UIN, CAN_MID_UIN);		
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
				t32bit.words.i16[1] = ui16fUIn;
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
		uw32MessageID = CAN_GenerateID(CAN_PRIO_RPMINFO, CAN_MID_RPMINFO);		
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

Int16 CAN_RXRPMLimits(FCAN_MB *MB)
{
	t32BitVars t32bit;
	
	// Flip bytes
	ioctl(MB, FCANMB_REORDER_BYTES, NULL);
	
	
	t32bit.uw32 = MB->data[1];
	if((CAN_MOTOR_ALL_ID == t32bit.bytes.ui8[0])||(COMMDataStruct.REGS.ui8ID == t32bit.bytes.ui8[0]))
	{
		t32bit.uw32 = MB->data[0];
		
		COMMDataStruct.REGS.i16MinRPM = t32bit.words.i16[0];
		COMMDataStruct.REGS.i16MaxRPM = t32bit.words.i16[1];		
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
	/*
	t32bit.uw32 = MB->data[1];    
	if((CAN_MOTOR_ALL_ID == t32bit.bytes.ui8[0])||(COMMDataStruct.REGS.ui8ID == t32bit.bytes.ui8[0]))
	{
		t32bit.uw32 = MB->data[0];    
		// Check that speed is within limits
		if((COMMDataStruct.REGS.i16MinRPM <= t32bit.words.i16[0])&&(COMMDataStruct.REGS.i16MaxRPM >= t32bit.words.i16[0]))
		{
			COMMDataStruct.REGS.i16SetRPM = t32bit.words.i16[0];
		}
	}
	else if(COMMDataStruct.REGS.ui8ID == t32bit.bytes.ui8[1])
	{
		t32bit.uw32 = MB->data[0];  		
		// Check that speed is within limits
		if((COMMDataStruct.REGS.i16MinRPM <= t32bit.words.i16[1])&&(COMMDataStruct.REGS.i16MaxRPM >= t32bit.words.i16[1]))
		{
			COMMDataStruct.REGS.i16SetRPM = t32bit.words.i16[1];
		}
	}
	*/
	return 0;
}

Int16 CAN_RXENABLE(FCAN_MB *MB)
{
	t32BitVars t32bit;
	
	// Flip bytes
	ioctl(MB, FCANMB_REORDER_BYTES, NULL);
	
	t32bit.uw32 = MB->data[0];    
	if((CAN_MOTOR_ALL_ID == t32bit.bytes.ui8[1])||(COMMDataStruct.REGS.ui8ID == t32bit.bytes.ui8[1]))
	{
		if((COMMDataStruct.REGS.i16MinRPM <= COMMDataStruct.REGS.i16SetRPM)&&(1 == t32bit.bytes.ui8[0]))
		{
			COMMDataStruct.REGS.ui8Armed = 1;
		}
		else
		{
			COMMDataStruct.REGS.ui8Armed = 0;
		}
	}
	
	return 0;
}

Int16 CAN_RXRESET(FCAN_MB *MB)
{
	t32BitVars t32bit;
	
	// Flip bytes
	ioctl(MB, FCANMB_REORDER_BYTES, NULL);
	
	// Check ID
	t32bit.uw32 = MB->data[0];    
	if((CAN_MOTOR_ALL_ID == t32bit.bytes.ui8[1])||(COMMDataStruct.REGS.ui8ID == t32bit.bytes.ui8[1]))
	{
		COMMDataStruct.REGS.ui8Reset = 1;
	}
	
	return 0;
}
