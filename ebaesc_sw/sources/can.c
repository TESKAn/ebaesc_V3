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
				uw32Temp = CAN_PRIO_STATUS;
				uw32Temp = uw32Temp << 24;
				uw32MessageID = uw32Temp;
				
				uw32Temp = CAN_MID_STATUS;
				uw32Temp = uw32Temp << 8;
				uw32MessageID = uw32MessageID | uw32Temp;
				
				uw32Temp = (UWord32)COMMDataStruct.REGS.ui8ID;
				uw32MessageID = uw32MessageID | uw32Temp;
				// Calculate time in seconds
				t32bit.uw32 = SYSTEM.ui32SystemTime / 1000;
				// Flip bytes
				t32bit1.bytes.ui8[0] = t32bit.bytes.ui8[3];
				t32bit1.bytes.ui8[1] = t32bit.bytes.ui8[2];
				t32bit1.bytes.ui8[2] = t32bit.bytes.ui8[1];
				t32bit1.bytes.ui8[3] = t32bit.bytes.ui8[0];
				// Store
				MB->data[0] = t32bit1.uw32;
				
				t32bit.i32 = 0;
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
		uw32Temp = CAN_PRIO_UIN;
		uw32Temp = uw32Temp << 24;
		uw32MessageID = uw32Temp;
		uw32Temp = 21034;
		uw32Temp = uw32Temp << 8;
		uw32MessageID = uw32MessageID | uw32Temp;
		uw32Temp = (UWord32)COMMDataStruct.REGS.ui8ID;
		uw32MessageID = uw32MessageID | uw32Temp;
		
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
