/*
 * RS485comm.c
 *
 *  Created on: May 20, 2015
 *      Author: jmoc
 */

#include "Allincludes.h"


RS485MOTOR RS485Data;

UInt16 ui16RS485Timer = 5000;


// Slave functions
Int16 RS485_initData()
{
	// Unit registers
	RS485Data.REGS.ui8ID = RS485_ID;
	RS485Data.REGS.ui8BaudRate = 3;
	RS485Data.REGS.ui16Errors = 0;
	RS485Data.REGS.f32SetRPM = 0.0f;
	RS485Data.REGS.ui16State = SYSTEM.systemState;
	RS485Data.REGS.ui16Command = 0;
	RS485Data.REGS.ui8Armed = 0;
	RS485Data.REGS.ui8Park = 0;
	// Park position
	RS485Data.REGS.i16ParkPosition = M_PARK_POSITION;
	RS485Data.REGS.f32MinRPM = 1000;
	RS485Data.REGS.f32MaxRPM = 9000;	
	
	RS485Data.ui8TXState = RS485_TX_IDLE;
	RS485Data.ui8RXState = RS485_TX_IDLE;
	
	RS485Data.ui8RS485RXBufferIndex = 0;
	RS485Data.ui8RS485RXBytes = 0;
	RS485Data.ui8RS485RXChecksum = 0;
	RS485Data.ui8RS485RXInstrErr = 0;
	RS485Data.ui16TXCommTimeout = 1000;
	RS485Data.ui16TXTimeoutCounter = 0;
	RS485Data.ui16RXCommTimeout = 1000;
	RS485Data.ui16RXTimeoutCounter = 0;
	
	// Init ring buffer for tx
	RB_Init(&RS485Data.RS485TXBuff, RS485Data.RS485TXBuffer, 128);	
	
	// Set to receive
	RS485_ENABLE_RX;
	// Disable TX interrupt
	RS485_DISABLE_TX_INT;
}

#pragma interrupt called
Int16 RS485_SyncToSystem()
{
	// Sync com variables to system
	SYSTEM.i16StateTransition = RS485Data.REGS.ui16Command;
	SYSTEM.COMMVALUES.i16ParkPosition = RS485Data.REGS.i16ParkPosition;

	if(0 != RS485Data.REGS.ui8Armed)
	{
		switch(SYSTEM.systemState)
		{
			case SYSTEM_IDLE:
			{
				// Park rotor?
				if(0 != RS485Data.REGS.ui8Park)
				{
					SYSTEM.i16StateTransition = SYSTEM_PARKROTOR;
				}
				// Else run
				else
				{
					// Control system speed
					CONTROL_SPEED = 1;
					SYSTEM.i16StateTransition = SYSTEM_RUN;
				}
				break;
			}
		}
	}
	else
	{
		switch(SYSTEM.systemState)
		{
			case SYSTEM_PARKROTOR:
			{
				SYSTEM.i16StateTransition = SYSTEM_RESET;
				break;
			}
			case SYSTEM_RUN:
			{
				SYSTEM.i16StateTransition = SYSTEM_RESET;
				break;
			}
		}
	}
	return 0;
}

#pragma interrupt called
Int16 RS485_SyncToComm()
{
	RS485Data.REGS.f32IIn = SYSTEM.SIVALUES.fIInFilt;
	RS485Data.REGS.f32PIn = SYSTEM.SIVALUES.fPIn;
	RS485Data.REGS.f32RPM = SYSTEM.SIVALUES.fRPM;
	RS485Data.REGS.f32UIn = SYSTEM.SIVALUES.fUIn;
	RS485Data.REGS.ui16Command = SYSTEM.i16StateTransition; 
	RS485Data.REGS.i16ParkPosition = SYSTEM.COMMVALUES.i16ParkPosition;
	
	if(SYSTEM_IDLE != SYSTEM.systemState)
	{
		RS485Data.REGS.ui8Armed = 1;
	}
	else
	{
		RS485Data.REGS.ui8Armed = 0;
	}	
}

#pragma interrupt called
Int16 RS485_Timer()
{
	// Call each ms	
	if(RS485_RX_IDLE != RS485Data.ui8RXState)
	{
		RS485Data.ui16RXTimeoutCounter++;
		if(RS485Data.ui16RXTimeoutCounter > RS485Data.ui16RXCommTimeout)
		{
			RS485Data.ui16RXTimeoutCounter = RS485Data.ui16RXCommTimeout;
			// Reset RX process.
			RS485Data.ui8RXState = RS485_RX_IDLE;
			RS485Data.ui8RS485RXBufferIndex = 0;			
		}
	}
	if(RS485_TX_IDLE != RS485Data.ui8TXState)
	{		
		RS485Data.ui16TXTimeoutCounter++;
		
		if(RS485_TX_DELAY == RS485Data.ui8TXState)
		{
			RS485Data.ui8TXState = RS485_TX_SENDING;
			RS485_writeByte();
		}
		else if(RS485Data.ui16TXTimeoutCounter > RS485Data.ui16TXCommTimeout)
		{
			RS485Data.ui16TXTimeoutCounter = RS485Data.ui16TXCommTimeout;
			// Reset TX process.
			RS485Data.ui8TXState = RS485_TX_IDLE;
			RS485_ENABLE_RX;
			RS485_DISABLE_TX_IDLE_INT;
			// Reset TX buffer
			RB_flush(&RS485Data.RS485TXBuff);
		}
	}
}

#pragma interrupt called
Int16 RS485_writeByte(void)
{
	RS485Data.ui16TXTimeoutCounter = 0;
	switch(RS485Data.ui8TXState)
	{
		case RS485_TX_IDLE:
		{
			// Data to send?
			if(0 != RS485Data.RS485TXBuff.count)
			{
				// Init transmission
				RS485_ENABLE_TX;
				// Enable TX empty interrupt
				RS485_ENABLE_TX_INT;
				// Go to next state
				RS485Data.ui8TXState = RS485_TX_DELAY;
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
			if(0 != RS485Data.RS485TXBuff.count)
			{
				// Room in buffer?
				if(RS485_TEST_TX_EMPTY)
				{
					// Write one byte
					RS485_WRITE(RB_pop(&RS485Data.RS485TXBuff));
				}
			}
			else
			{
				// All data sent
				// Wait for end of transmission
				RS485Data.ui8TXState = RS485_TX_FINISHED;
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
				// Yes, enable receive
				RS485_ENABLE_RX;
				// Disable interrupt
				RS485_DISABLE_TX_IDLE_INT;
				// Go to idle state
				RS485Data.ui8TXState = RS485_TX_IDLE;
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
			RS485Data.ui8TXState = RS485_TX_IDLE;
			break;			
		}
	}
	
	return 0;
}

#pragma interrupt called
Int16 RS485_States_slave(UInt8 data)
{
	RS485Data.ui16RXTimeoutCounter = 0;
	switch(RS485Data.ui8RXState)
	{
		case RS485_RX_IDLE:
		{
			// Data == 0xFF?
			if(0xFF == data)
			{
				// Go to wait for signal
				RS485Data.ui8RXState = RS485_RX_WAIT_FOR_SIGNAL;
			}
			break;
		}
		case RS485_RX_WAIT_FOR_SIGNAL:
		{
			// Data == 0xFF?
			if(0xFF == data)
			{
				// Go to wait for ID
				RS485Data.ui8RXState = RS485_RX_WAIT_FOR_ID;
			}
			else
			{
				RS485Data.ui8RXState = RS485_RX_IDLE;
			}
			break;
		}
		case RS485_RX_WAIT_FOR_ID:
		{
			// Data == ID?
			if(data == RS485Data.REGS.ui8ID)
			{
				// ID match, wait for data length
				RS485Data.ui8RXState = RS485_RX_WAIT_FOR_LENGTH;
				// Set checksum to ID
				RS485Data.ui8RS485RXChecksum = data;
			}
			else
			{
				// Id not matched, go to idle
				RS485Data.ui8RXState = RS485_RX_IDLE;
			}
			break;
		}
		case RS485_RX_WAIT_FOR_LENGTH:
		{
			// Store how many bytes will follow
			RS485Data.ui8RS485RXBytes = data - 2;
			// Add to checksum
			RS485Data.ui8RS485RXChecksum += data;
			// Go to wait for instruction
			RS485Data.ui8RXState = RS485_RX_WAIT_FOR_INSTR_ERR;
			break;
		}
		case RS485_RX_WAIT_FOR_INSTR_ERR:
		{
			// Store instruction
			RS485Data.ui8RS485RXInstrErr = data;
			// Add to checksum
			RS485Data.ui8RS485RXChecksum += data;
			// Go to wait for parameters, if RS485Data.ui8RS485RXBytes > 0
			if(0 < RS485Data.ui8RS485RXBytes)
			{
				// Waiting for parameters
				RS485Data.ui8RXState = RS485_RX_WAIT_FOR_PARAMETERS;
				// Reset buffer index
				RS485Data.ui8RS485RXBufferIndex = 0;
			}
			else
			{
				// Wait for checksum
				RS485Data.ui8RXState = RS485_RX_WAIT_FOR_CHECKSUM;
			}		
			break;
		}
		case RS485_RX_WAIT_FOR_PARAMETERS:
		{
			// Store parameter
			RS485Data.RS485RXBuffer[RS485Data.ui8RS485RXBufferIndex] = data;
			RS485Data.ui8RS485RXBufferIndex ++;
			// Add to checksum
			RS485Data.ui8RS485RXChecksum += data;
			// Have all parameters?
			if(RS485Data.ui8RS485RXBytes == RS485Data.ui8RS485RXBufferIndex)
			{
				// Yes, go to receive checksum
				RS485Data.ui8RXState = RS485_RX_WAIT_FOR_CHECKSUM;
			}
			break;
		}
		case RS485_RX_WAIT_FOR_CHECKSUM:
		{
			//RS485Data.ui8Checksum = data;
			// Check checksum
			// Calculate
			RS485Data.ui8RS485RXChecksum = RS485Data.ui8RS485RXChecksum & 0XFF;
			RS485Data.ui8RS485RXChecksum = ~RS485Data.ui8RS485RXChecksum;
			// If checksum is OK, decode data
			if(RS485Data.ui8RS485RXChecksum == data)
			{
				// Decode received message
				RS485_decodeMessage();				
			}
			// Go to idle state
			RS485Data.ui8RXState = RS485_RX_IDLE;
			break;
		}		
		default:
		{
			RS485Data.ui8RXState = RS485_RX_IDLE;
		}
	
	}
	return 0;
}

#pragma interrupt called
Int16 RS485_decodeMessage(void)
{
	UInt8 ui8Temp = 0;
	
	switch(RS485Data.ui8RS485RXInstrErr)
	{
		case RS485_INSTR_PING:
		{
			// Return status packet
			// Setup data to be transmitted
			// Signal
			RB_push(&RS485Data.RS485TXBuff, 0xff);
			RB_push(&RS485Data.RS485TXBuff, 0xff);
			// ID
			RB_push(&RS485Data.RS485TXBuff, RS485Data.REGS.ui8ID);
			RS485Data.ui8TXChecksum = RS485Data.REGS.ui8ID;
			// Error
			RB_push(&RS485Data.RS485TXBuff, 0x00);
			RS485Data.ui8TXChecksum += 0x00;
			// Length = 2
			RB_push(&RS485Data.RS485TXBuff, 0x02);
			RS485Data.ui8TXChecksum += 0x02;
			// Calculate checksum
			RS485Data.ui8TXChecksum &= 0xff;
			RS485Data.ui8TXChecksum = ~RS485Data.ui8TXChecksum;
			RB_push(&RS485Data.RS485TXBuff, RS485Data.ui8TXChecksum);			
			break;
		}
		case RS485_INSTR_READ_DATA:
		{
			// Setup data to be transmitted
			// Sync
			RS485_SyncToComm();
			// Signal
			RB_push(&RS485Data.RS485TXBuff, 0xff);
			RB_push(&RS485Data.RS485TXBuff, 0xff);
			// ID
			RB_push(&RS485Data.RS485TXBuff, RS485Data.REGS.ui8ID);
			RS485Data.ui8TXChecksum = RS485Data.REGS.ui8ID;
			// Error
			RB_push(&RS485Data.RS485TXBuff, 0x00);
			RS485Data.ui8TXChecksum += 0x00;
			// Length = 2 + num tx bytes
			RB_push(&RS485Data.RS485TXBuff, 0x02 + RS485Data.RS485RXBuffer[1]);
			RS485Data.ui8TXChecksum += (0x02 + RS485Data.RS485RXBuffer[1]);
			// Data
			for(ui8Temp = 0; ui8Temp < RS485Data.RS485RXBuffer[1]; ui8Temp++)
			{
				// Store byte
				RB_push(&RS485Data.RS485TXBuff, RS485Data.ui8Data[RS485Data.RS485RXBuffer[0] + ui8Temp]);
				RS485Data.ui8TXChecksum += RS485Data.ui8Data[RS485Data.RS485RXBuffer[0] + ui8Temp];
			}
			// Calculate checksum
			RS485Data.ui8TXChecksum &= 0xff;
			RS485Data.ui8TXChecksum = ~RS485Data.ui8TXChecksum;
			RB_push(&RS485Data.RS485TXBuff, RS485Data.ui8TXChecksum);

			break;
		}
		case RS485_INSTR_WRITE_DATA:
		{
			// Sync status to comm
			RS485_SyncToComm();
			// Write data to regs and update
			for(ui8Temp = 0; ui8Temp < RS485Data.ui8RS485RXBytes - 1; ui8Temp++)
			{				
				RS485Data.ui8Data[RS485Data.RS485RXBuffer[0] + ui8Temp] = RS485Data.RS485RXBuffer[1 + ui8Temp];
			}
			// Return status packet
			// Setup data to be transmitted
			// Signal
			RB_push(&RS485Data.RS485TXBuff, 0xff);
			RB_push(&RS485Data.RS485TXBuff, 0xff);
			// ID
			RB_push(&RS485Data.RS485TXBuff, RS485Data.REGS.ui8ID);
			RS485Data.ui8TXChecksum = RS485Data.REGS.ui8ID;
			// Error
			RB_push(&RS485Data.RS485TXBuff, 0x00);
			RS485Data.ui8TXChecksum += 0x00;
			// Length = 2
			RB_push(&RS485Data.RS485TXBuff, 0x02);
			RS485Data.ui8TXChecksum += 0x02;
			// Calculate checksum
			RS485Data.ui8TXChecksum &= 0xff;
			RS485Data.ui8TXChecksum = ~RS485Data.ui8TXChecksum;
			RB_push(&RS485Data.RS485TXBuff, RS485Data.ui8TXChecksum);		
			// Sync back to system variables
			RS485_SyncToSystem();

			break;
		}		
	}
	// Start transmission
	RS485_writeByte();
	return 0;
}
