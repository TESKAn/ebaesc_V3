/*
 * RS485comm.c
 *
 *  Created on: May 20, 2015
 *      Author: jmoc
 */

#include "Allincludes.h"

// Disable timeouts
//#define RS485_NOTIMEOUT

RS485MOTOR* RS485Data;

UInt16 ui16RS485Timer = 5000;


// Slave functions
Int16 RS485_initData(RS485MOTOR* dataStruct)
{
	// Store pointer to data structure
	RS485Data = dataStruct;
	// Unit registers
	RS485Data->REGS.ui8ID = RS485_ID;
	RS485Data->REGS.ui16ModelNumber = 0x001c;
	RS485Data->REGS.ui8FirmwareVersion = 1;
	RS485Data->REGS.ui8BaudRate = 3;
	RS485Data->REGS.ui16Errors = 0;
	//RS485Data->REGS.f32SetRPM = 0.0f;
	RS485Data->REGS.ui16State = SYSTEM.systemState;
	RS485Data->REGS.ui8Armed = 0;
	RS485Data->REGS.ui8Park = 0;
	RS485Data->REGS.ui8ReturnDelayTime = 1;
	// Park position
	RS485Data->REGS.i16ParkPosition = M_PARK_POSITION;
	//RS485Data->REGS.f32MinRPM = 1000;
	//RS485Data->REGS.f32MaxRPM = 9000;	
	
	RS485Data->ui16RegsBytes = 64;
	RS485Data->errStatus = 0;
	RS485Data->ui16RXCommTimeout = 100;
	RS485Data->ui16RXTimeoutCounter = 0;
	RS485Data->ui16TXCommTimeout = 100;
	RS485Data->ui16TXTimeoutCounter = 0;	
	RS485Data->ui16ReadOnlyLow = 0;
	RS485Data->ui16ReadOnlyHigh = 2;
	
	RS485Data->REGS.i16PWMMax = 2000;
	RS485Data->REGS.i16PWMMin = 1000;
	
	RS485Data->REGS.i16MaxRPM = 9000;
	RS485Data->REGS.i16MinRPM = 1000;
	RS485Data->REGS.i16CurrentPWM = 1000;
	RS485Data->REGS.i16ParkPosition = 2048;
	
	RS485Data->REGS.i16ZeroSpeedPWM = 50;
	
	
	RS485Data->ui8TXState = RS485_TX_IDLE;
	RS485Data->ui8RXState = RS485_TX_IDLE;
	

	// Set to receive
	RS485_ENABLE_RX;
	// Disable TX interrupt
	RS485_DISABLE_TX_INT;
}

//#pragma interrupt called
Int16 RS485_SyncToSystem()
{
	return 0;
}

//#pragma interrupt called
Int16 RS485_SyncToComm()
{
	float fTemp;

	
	// Set speed
	//fTemp = (float)SYSTEM.RAMPS.f16SpeedRampDesiredValue;
	//fTemp = fTemp * 60000;
	//fTemp = fTemp / (32768 * (float)SYSTEM.CALIBRATION.i16MotorPolePairs);
	//RS485Data->REGS.i16RPM = (Int16)fTemp;
	
	if(SYSTEM_IDLE != SYSTEM.systemState)
	{
		RS485Data->REGS.ui8Armed = 1;
	}
	else
	{
		RS485Data->REGS.ui8Armed = 0;
	}	
}

//#pragma interrupt called
void RS485_Timer()
{
#ifndef RS485_NOTIMEOUT
	if(RS485_RX_IDLE != RS485Data->ui8RXState)
	{
		RS485Data->ui16RXTimeoutCounter++;
		if(RS485Data->ui16RXTimeoutCounter > RS485Data->ui16RXCommTimeout)
		{
			RS485Data->ui16RXTimeoutCounter = RS485Data->ui16RXCommTimeout;
			// Reset RX process.
			RS485Data->ui8RXState = RS485_RX_IDLE;	
			RS485Data->ui8RXCounter = 0;
		}
	}
#endif
	if(RS485_TX_IDLE != RS485Data->ui8TXState)
	{		
		if(RS485_TX_DELAY == RS485Data->ui8TXState)
		{
			RS485Data->ui8TXState = RS485_TX_SENDING;
			RS485_writeByte();
		}
#ifndef RS485_NOTIMEOUT
		RS485Data->ui16TXTimeoutCounter++;
		
		if(RS485Data->ui16TXTimeoutCounter > RS485Data->ui16TXCommTimeout)
		{
			RS485Data->ui16TXTimeoutCounter = RS485Data->ui16TXCommTimeout;
			// Reset TX process.
			RS485Data->ui8TXState = RS485_TX_IDLE;
			RS485_ENABLE_RX;
			RS485_DISABLE_TX_INT;
			RS485_DISABLE_TX_IDLE_INT;
			// Reset TX buffer
			RS485Data->ui8TXIndex = 0;
			RS485Data->ui8TXBytesLeft = 0;
		}	
#endif
	}
	// Check return delay time
	if(0 != RS485Data->ui8ReturnDelay)
	{
		RS485Data->ui8ReturnDelay--;
		if(0 == RS485Data->ui8ReturnDelay)
		{
			RS485_writeByte();
		}
	}
}

//#pragma interrupt called
void RS485_writeByte()
{	
	RS485Data->ui16TXTimeoutCounter = 0;

	switch(RS485Data->ui8TXState)
	{
		case RS485_TX_IDLE:
		{
			// Data to send?
			if(0 != RS485Data->ui8TXBytesLeft)
			{
				// Init transmission
				RS485_ENABLE_TX;
				// Go to next state
				RS485Data->ui8TXState = RS485_TX_DELAY;
			}
			else
			{
				// Erroneous interrupt. Disable both.
				RS485_DISABLE_TX_INT;
				RS485_DISABLE_TX_IDLE_INT;
			}
			break;
		}
		case RS485_TX_DELAY:
		{
			break;
		}	
		case RS485_TX_SENDING:
		{
			// Data to send?
			if(0 != RS485Data->ui8TXBytesLeft)
			{
				while(4 != RS485_GET_TX_FIFO_CNT)
				{
					// Room in buffer?
					if(RS485_TEST_TX_EMPTY)
					{
						// Enable TX empty interrupt
						RS485_ENABLE_TX_INT;
						// Write one byte
						RS485_WRITE(RS485Data->RS485TXBuffer[RS485Data->ui8TXIndex]);
						RS485Data->ui8TXIndex++;
						RS485Data->ui8TXBytesLeft--;
					}
					// Check if buffer is empty
					if(0 == RS485Data->ui8TXBytesLeft)
					{
						// Last byte in buffer
						// Wait for end of transmission
						RS485Data->ui8TXState = RS485_TX_FINISHED;
						// Disable interrupt
						RS485_DISABLE_TX_INT;
						// Enable idle interrupt
						RS485_ENABLE_TX_IDLE_INT;
						break;
					}					
				}
			}
			else
			{
				// All data sent
				// Wait for end of transmission
				RS485Data->ui8TXState = RS485_TX_FINISHED;
				// Disable interrupt
				RS485_DISABLE_TX_INT;
				// Enable idle interrupt
				RS485_ENABLE_TX_IDLE_INT;
			}
			break;
		}
		case RS485_TX_FINISHED:
		{
			// Done with sending?
			if(RS485_TEST_TX_IDLE)
			{
				// No more TX bytes?
				if(0 == RS485_GET_TX_FIFO_CNT)
				{
					// Yes, enable receive
					RS485_ENABLE_RX;
					// Disable interrupt
					RS485_DISABLE_TX_IDLE_INT;
					// Go to idle state
					RS485Data->ui8TXState = RS485_TX_IDLE;
				}
			}
			else if(RS485_TEST_TX_EMPTY)
			{
				// This was TX empty interrupt. Disable it.
				RS485_DISABLE_TX_INT;
			}
			break;
		}
		default:
		{
			RS485_ENABLE_RX;
			RS485_DISABLE_TX_INT;
			RS485_DISABLE_TX_IDLE_INT;
			RS485Data->ui8TXState = RS485_TX_IDLE;
			RS485Data->ui8TXIndex = 0;
			RS485Data->ui8TXBytesLeft = 0;
			break;			
		}
	}
}

//#pragma interrupt called
void RS485_States_slave(UInt8 data)
{
	UInt16 ui16Temp = 0;
	RS485Data->ui16RXTimeoutCounter = 0;
	// Store
	RS485Data->RXDATA.ui8Parameters[RS485Data->RXDATA.ui8RS485RXIndex] = data;
	RS485Data->RXDATA.ui8RS485RXIndex++;
	if(128 <= RS485Data->RXDATA.ui8RS485RXIndex)
	{
		RS485Data->RXDATA.ui8RS485RXIndex = 0;
	}	
	switch(RS485Data->ui8RXState)
	{
		case RS485_RX_IDLE:
		{
			// Data == 0xFF?
			if(0xFF == data)
			{
				// Go to wait for signal
				RS485Data->ui8RXState = RS485_RX_WAIT_FOR_SIGNAL;
				RS485Data->RXDATA.HEADER.ui8Bytes[0] = data;
				RS485Data->ui8RXCounter = 1;
				RS485Data->RXDATA.ui8Parameters[0] = data;
				RS485Data->RXDATA.ui8RS485RXIndex = 1;
			}
			/*
			else
			{

			}
			*/
			break;
		}
		case RS485_RX_WAIT_FOR_SIGNAL:
		{
			if(3 > RS485Data->ui8RXCounter)
			{
				RS485Data->RXDATA.HEADER.ui8Bytes[RS485Data->ui8RXCounter] = data;
				RS485Data->ui8RXCounter ++;
			}
			else
			{
				// Check header
				if(0x00fdffff == RS485Data->RXDATA.HEADER.ui32Header)
				{
					RS485Data->ui8RXState = RS485_RX_WAIT_FOR_ID;
					RS485Data->ui8RXCounter = 0;
				}
				else
				{
					RS485Data->ui8RXState = RS485_RX_IDLE;
					RS485Data->ui8RXCounter = 0;
				}
			}
			break;
		}
		case RS485_RX_WAIT_FOR_ID:
		{
			// Data == ID or broadcast ID?
			if((data == RS485Data->REGS.ui8ID)||(RS485BROADCAST_ID == data))
			{
				// ID match, wait for data length
				RS485Data->ui8RXState = RS485_RX_WAIT_FOR_LENGTH;
				RS485Data->ui8RXCounter = 0;
				// Store ID
				RS485Data->RXDATA.ui8ID = data;				
			}
			else
			{
				// Id not matched, go to idle
				RS485Data->ui8RXState = RS485_RX_IDLE;
				// Reset RX buffer
				//RB_flush(&SCI0RXBuff);
			}
			break;
		}
		case RS485_RX_WAIT_FOR_LENGTH:
		{
			// Store byte
			RS485Data->RXDATA.LENGTH.ui8Bytes[RS485Data->ui8RXCounter] = data;
			RS485Data->ui8RXCounter++;
			if(2 == RS485Data->ui8RXCounter)
			{
				// Store number of bytes in parameters, is length + 5 bytes for total packet length - crc
				RS485Data->RXDATA.ui8ParamByteCount = RS485Data->RXDATA.LENGTH.ui16PacketLength + 5; 
				// Go to wait for instruction
				RS485Data->ui8RXState = RS485_RX_WAIT_FOR_INSTR_ERR;
				RS485Data->ui8RXCounter = 0;
			}
			break;
		}	
		case RS485_RX_WAIT_FOR_INSTR_ERR:
		{
			// Store instruction
			RS485Data->RXDATA.ui8Instruction = data;
			if(3 == RS485Data->RXDATA.LENGTH.ui16PacketLength)
			{
				// No parameters, only instruction + CRC
				// Wait for checksum
				RS485Data->ui8RXState = RS485_RX_WAIT_FOR_CHECKSUM;
				RS485Data->ui8RXCounter = 0;
			}
			else
			{
				// Waiting for parameters
				RS485Data->ui8RXState = RS485_RX_WAIT_FOR_PARAMETERS;				
			}
			break;
		}
		case RS485_RX_WAIT_FOR_PARAMETERS:
		{
			// Have all parameters?
			if(RS485Data->RXDATA.ui8ParamByteCount == RS485Data->RXDATA.ui8RS485RXIndex)
			{
				// Yes, go to receive checksum
				RS485Data->ui8RXState = RS485_RX_WAIT_FOR_CHECKSUM;
				RS485Data->ui8RXCounter = 0;
			}
			break;
		}
		case RS485_RX_WAIT_FOR_CHECKSUM:
		{
			RS485Data->RXDATA.CRCDATA.ui8Bytes[RS485Data->ui8RXCounter] = data;
			RS485Data->ui8RXCounter++;
			if(2 == RS485Data->ui8RXCounter)
			{
				
				// Calculate CRC
				ui16Temp = update_crc(0, RS485Data->RXDATA.ui8Parameters, 5 + RS485Data->RXDATA.LENGTH.ui16PacketLength);
				if(ui16Temp == RS485Data->RXDATA.CRCDATA.ui16CRC)
				{
					RS485_decodeMessage();
					// Reset RX buffer
					//RB_flush(&SCI0RXBuff);
				}
				RS485Data->ui8RXState = RS485_RX_IDLE;
				RS485Data->ui8RXCounter = 0;				
			}

			break;
		}		
		default:
		{
			RS485Data->ui8RXState = RS485_RX_IDLE;
		}
	}
}

//#pragma interrupt called
void RS485_decodeMessage()
{
	Int16 i = 0;
	UInt8 ui8WriteOK = 0;
	UInt16 ui16Temp = 0;
	UInt16 ui16RegAddress = 0;
	UInt16 ui16NumRegBytes = 0;
	union
	{
		UInt16 data;
		UInt8 bytes[2];
	}ui16Val;
	
	// Setup return packet
	// Signal
	RS485Data->RS485TXBuffer[0] = 0xff;
	RS485Data->RS485TXBuffer[1] = 0xff;
	RS485Data->RS485TXBuffer[2] = 0xfd;
	RS485Data->RS485TXBuffer[3] = 0x00;
	// ID
	RS485Data->RS485TXBuffer[4] = RS485Data->REGS.ui8ID;	
		
	switch(RS485Data->RXDATA.ui8Instruction)
	{
		case RS485_INSTR_PING:
		{
			// Len_l, len_h
			RS485Data->RS485TXBuffer[5] = 0x08; 
			RS485Data->RS485TXBuffer[6] = 0x00;
			// Instruction
			RS485Data->RS485TXBuffer[7] = 0x55;
			// Error
			RS485Data->RS485TXBuffer[8] = 0;
			// Parameters
			RS485Data->RS485TXBuffer[9] = RS485Data->errStatus;
			RS485Data->RS485TXBuffer[10] = RS485Data->ui8REGSData[0];
			RS485Data->RS485TXBuffer[11] = RS485Data->ui8REGSData[1];
			RS485Data->RS485TXBuffer[12] = RS485Data->ui8REGSData[2];
			ui16Temp = 13;
			break;
		}
		case RS485_INSTR_READ_DATA:
		{
			// Get address
			ui16Val.bytes[0] = RS485Data->RXDATA.ui8Parameters[8];
			ui16Val.bytes[1] = RS485Data->RXDATA.ui8Parameters[9];
			ui16RegAddress = ui16Val.data;
			// Get num bytes
			ui16Val.bytes[0] = RS485Data->RXDATA.ui8Parameters[10];
			ui16Val.bytes[1] = RS485Data->RXDATA.ui8Parameters[11];
			ui16NumRegBytes = ui16Val.data;
			// Check range
			ui16Temp = ui16RegAddress + ui16NumRegBytes - 1;
			if(ui16Temp < RS485Data->ui16RegsBytes)
			{
				// Sync data
				RS485_SyncToComm();
				// Setup data
				// Len_l, len_h
				ui16Val.data = ui16NumRegBytes + 5;
						
				RS485Data->RS485TXBuffer[5] = ui16Val.bytes[0]; 
				RS485Data->RS485TXBuffer[6] = ui16Val.bytes[1];
				// Instruction
				RS485Data->RS485TXBuffer[7] = 0x55;
				// Error
				RS485Data->RS485TXBuffer[8] = 0x00;
				// Parameters
				// Error reg
				RS485Data->RS485TXBuffer[9] = RS485Data->errStatus;
				// Data regs
				ui16Temp = 10;
				for(i = 0; i < ui16NumRegBytes; i++)
				{
					RS485Data->RS485TXBuffer[ui16Temp] = RS485Data->ui8REGSData[ui16RegAddress + i];
					ui16Temp++;
				}				
			}
			else
			{
				// Len_l, len_h						
				RS485Data->RS485TXBuffer[5] = 0x05; 
				RS485Data->RS485TXBuffer[6] = 0x00;
				// Instruction
				RS485Data->RS485TXBuffer[7] = 0x55;
				// Error
				RS485Data->RS485TXBuffer[8] = 0x04;
				// Parameter
				RS485Data->RS485TXBuffer[9] = RS485Data->RXDATA.ui8Instruction;
				ui16Temp = 10;
			}			
			
			break;
		}
		case RS485_INSTR_WRITE_DATA:
		{
			// Get address
			ui16Val.bytes[0] = RS485Data->RXDATA.ui8Parameters[8];
			ui16Val.bytes[1] = RS485Data->RXDATA.ui8Parameters[9];
			ui16RegAddress = ui16Val.data;
			// Get num bytes
			ui16NumRegBytes = RS485Data->RXDATA.LENGTH.ui16PacketLength - 5;
			// Check range
			ui8WriteOK = 0;
			if(2 < ui16RegAddress)
			{
				// Calculate last address
				ui16Temp = ui16RegAddress + ui16NumRegBytes - 1;
				if(RS485Data->ui16ReadOnlyLow > ui16Temp)
				{
					// OK to write
					ui8WriteOK = 1;
				}
				if(RS485Data->ui16ReadOnlyHigh < ui16RegAddress)
				{
					// Check total length
					if(ui16Temp < RS485Data->ui16RegsBytes)
					{
						// OK to write
						ui8WriteOK = 1;
					}
				}
			}
			if(1 == ui8WriteOK)
			{
				// Write data
				for(i = 0; i < ui16NumRegBytes; i++)
				{
					RS485Data->ui8REGSData[ui16RegAddress + i] = RS485Data->RXDATA.ui8Parameters[i + 10];
					ui16Temp++;
				}				
				// Sync
				RS485_SyncToSystem();
				// Setup data
				// Len_l, len_h
				ui16Val.data = 5;
						
				RS485Data->RS485TXBuffer[5] = ui16Val.bytes[0]; 
				RS485Data->RS485TXBuffer[6] = ui16Val.bytes[1];
				// Instruction
				RS485Data->RS485TXBuffer[7] = 0x55;
				// Error
				RS485Data->RS485TXBuffer[8] = 0x00;
				// Parameters
				// Error reg
				RS485Data->RS485TXBuffer[9] = RS485Data->errStatus;	
				ui16Temp = 10;
			}
			else
			{
				// Len_l, len_h						
				RS485Data->RS485TXBuffer[5] = 0x05; 
				RS485Data->RS485TXBuffer[6] = 0x00;
				// Instruction
				RS485Data->RS485TXBuffer[7] = 0x55;
				// Error
				RS485Data->RS485TXBuffer[8] = 0x07;
				// Parameter
				RS485Data->RS485TXBuffer[9] = RS485Data->RXDATA.ui8Instruction;
				ui16Temp = 10;
			}
			break;
		}
	}
	// CRC
	ui16Val.data = update_crc(0, RS485Data->RS485TXBuffer, ui16Temp);
	RS485Data->RS485TXBuffer[ui16Temp] = ui16Val.bytes[0];
	ui16Temp++;
	RS485Data->RS485TXBuffer[ui16Temp] = ui16Val.bytes[1];
	ui16Temp++;
	RS485Data->ui8TXBytesLeft = ui16Temp;
	RS485Data->ui8TXIndex = 0;		
	
	// If ID is unit ID, send data. Else no response.
	if(RS485Data->REGS.ui8ID == RS485Data->RXDATA.ui8ID)
	{
		// Load delay time for TX
		RS485Data->ui8ReturnDelay = RS485Data->REGS.ui8ReturnDelayTime;
	}	
}

UInt16 update_crc(UInt16 crc_accum, UInt8 *data_blk_ptr, UInt16 data_blk_size)
{
	UInt16 i, j;
	UInt16 crc_table[256] = {

        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,

        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,

        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,

        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,

        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,

        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,

        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,

        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,

        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,

        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,

        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,

        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,

        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,

        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,

        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,

        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,

        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,

        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,

        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,

        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,

        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,

        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,

        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,

        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,

        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,

        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,

        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,

        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,

        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,

        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,

        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,

        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202

    };
    for(j = 0; j < data_blk_size; j++)
    {
        i = ((UInt16)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    } 
    return crc_accum;
}
