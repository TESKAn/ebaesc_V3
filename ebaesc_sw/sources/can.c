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
	UWord32 uw32MessageID = 0;
	UWord32 uw32Temp = 0;
	FCAN_MB *MB;
	int code = 0;
	
	// Generate message ID
	uw32Temp = CAN_MID_UIN;
	uw32Temp = uw32Temp << 8;
	uw32MessageID = uw32MessageID | uw32Temp;
	uw32Temp = CAN_DEVID;
	uw32MessageID = uw32MessageID | uw32Temp;
	
	if(ioctl(FCAN, FCAN_TEST_READY, null))
	{
		for(i = 0; i < 8; i++)
		{		
			MB = ioctl(FCAN, FCAN_GET_MB_MODULE, i);

			code = ioctl(MB, FCANMB_GET_CODE, null);
			ioctl(MB, FCANMB_SET_CODE, FCAN_MB_CODE_TXVOID);	
		}
		/*
		for(i=8;i<16;i++)
		{
			MB = ioctl(FCAN, FCAN_GET_MB_MODULE, i);
			
			ioctl(MB, FCANMB_SET_ID, 0x4e2123 | FCAN_ID_EXT);	
			ioctl(MB, FCANMB_SET_CODE, FCAN_MB_CODE_RXEMPTY);
		}
		ioctl(FCAN, FCAN_UNLOCK_ALL_MB, null);
		*/
	}
	return 0;
}

Int16 CAN_TXStatus()
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
	ui16fIIn = Float32ToFloat16(-53.10f);
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
		uw32Temp = CAN_DEVID;
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

Int16 CAN_TXVoltage()
{
	//SYSTEM.SIVALUES.fUIn
	FCAN_MB *MB;
	int i = 0;
	int code = 0;
	UWord32 uw32MessageID = 0;
	UWord32 uw32Temp = 0;
	t32BitVars t32bit;
	
	if(ioctl(FCAN, FCAN_TEST_READY, null))
	{
		// Generate message ID
		uw32Temp = CAN_PRIO_UIN;
		uw32Temp = uw32Temp << 24;
		uw32MessageID = uw32Temp;
		uw32Temp = CAN_MID_UIN;
		uw32Temp = uw32Temp << 8;
		uw32MessageID = uw32MessageID | uw32Temp;
		uw32Temp = CAN_DEVID;
		uw32MessageID = uw32MessageID | uw32Temp;
		t32bit.f = SYSTEM.SIVALUES.fUIn;
		// Get free MB
		for(i=0;i<14;i++)
		{
			MB = ioctl(FCAN, FCAN_GET_MB_MODULE, i);
			// Get code
			code = ioctl(MB, FCANMB_GET_CODE, null);
			if(code == 0b1000)
			{
				// Write ID
				//MB->id = 0xff4e2123;
				// Write message
				MB->data[0] = t32bit.uw32;
				MB->data[1] = 0;
				ioctl(MB, FCANMB_SET_ID, uw32MessageID | FCAN_ID_EXT);
				ioctl(MB, FCANMB_SET_LEN, 4);
				ioctl(MB, FCANMB_SET_CODE, FCAN_MB_CODE_TXONCE);		
				i = 15;
			}
		}			
	}	
	return 0;
}
