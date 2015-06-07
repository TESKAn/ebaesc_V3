/*
 * RS485comm.c
 *
 *  Created on: May 20, 2015
 *      Author: jmoc
 */

#include "Allincludes.h"


RS485SERVO RS485Data;

RS485SERVOMASTER RS485Master;

UInt16 ui16RS485Timer = 5000;
UInt8 ui8RS485TestSequence = 0;
UInt8 ui8Temp0;

void RS485CommTest(void)
{
	ui16RS485Timer--;
	if(1 > ui16RS485Timer)
	{
		ui16RS485Timer = 1000;
		ui8RS485TestSequence = 0;
		switch(ui8RS485TestSequence)
		{
			case 0:
			{
				// Set rcvd data to 0
				ui8RS485RcvdByte = 0;
				// Prepare data
				// Test serial comm - request
				// Setup data
				// FF FF 01 04 02 1E 04 D6
				ui8SerialBuffer[0] = 0xff;
				ui8SerialBuffer[1] = 0xff;
				// ID
				ui8SerialBuffer[2] = RS485Address;
				ui8SerialBuffer[3] = 0x02;
				ui8SerialBuffer[4] = 0x01;
				ui8SerialBuffer[5] = ~(RS485Address + 0x03);
				// Enable transmitter
				RS485_ENABLE_TX;
				// Send
				for(ui8Temp0 = 0; ui8Temp0 < 6; ui8Temp0++)
				{
					while(!RS485_TEST_TX_EMPTY)
					{
						asm(nop);
					}
					RS485_WRITE(ui8SerialBuffer[ui8Temp0]);
				}
				// Wait for end of transmission
				while(!RS485_TEST_TX_IDLE)
				{
					asm(nop);
				}
				// Enable receiver
				RS485_ENABLE_RX;   
				// Next address
				//RS485Address++;
				RS485Address = 1;
				if(255 < RS485Address) RS485Address = 0;
				break;
			}
		}
		
	}
}

// Master functions
UInt16 RS485_MasterInitData(void)
{
	Int16 i16Temp = 0;
	// Set slaves
	// ID's
	RS485Master.RS485Slaves[0].REGS.ui8ID = 1;
	// Common regs
	for(i16Temp = 0; i16Temp < RS485_NUMSLAVES; i16Temp++)
	{
		
	}
	// Set to receive
	RS485_ENABLE_RX;
	// Disable TX interrupt
	RS485_DISABLE_TX_INT;	
	// Set states
	RS485Master.ui8RcvState = RS485_M_IDLE;
	RS485Master.ui8TxState = RS485_M_TX_IDLE;
	return 0;
}

#pragma interrupt called
UInt16 RS485_MasterWriteByte(void)
{
	switch(RS485Master.ui8TxState)
	{
		case RS485_M_TX_IDLE:
		{
			// Data to send?
			if(RS485Master.ui8BytesToSend > RS485Master.ui8RcvBufferIndex)
			{
				// Init transmission
				RS485_ENABLE_TX;
				// Go to next state
				RS485Master.ui8TxState = RS485_M_TX_SENDING;
				// Reset index
				RS485Master.ui8RcvBufferIndex = 0;
			}
			else
			{
				// Erroreus interrupt. Disable both.
				RS485_DISABLE_TX_INT;
				RS485_DISABLE_TX_IDLE_INT;
			}
			break;
		}
		case RS485_M_TX_SENDING:
		{
			// Data to send?
			if(RS485Master.ui8BytesToSend > RS485Master.ui8RcvBufferIndex)
			{
				// Room in buffer?
				if(RS485_TEST_TX_EMPTY)
				{
					// Write one byte
					RS485_WRITE(RS485Master.ui8SendRcvBuffer[RS485Master.ui8RcvBufferIndex]);
					RS485Master.ui8RcvBufferIndex ++;
				}
			}
			else
			{
				// All data sent
				// Wait for end of transmission
				RS485Master.ui8TxState = RS485_M_TX_FINISHED;
				// Disable interrupt
				RS485_DISABLE_TX_INT;
				// Enable idle interrupt
				RS485_ENABLE_TX_IDLE_INT;
			}
			break;
		}
		case RS485_M_TX_FINISHED:
		{
			// Done with sending?
			if(RS485_TEST_TX_IDLE)
			{
				// Yes, enable receive
				RS485_ENABLE_RX;
				// Reset index
				RS485Master.ui8RcvBufferIndex = 0;
				// Disable interrupt
				RS485_DISABLE_TX_IDLE_INT;
				// Go to idle state
				RS485Master.ui8TxState = RS485_TX_IDLE;
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
			RS485Master.ui8TxState = RS485_M_TX_IDLE;
			break;			
		}
	}
	return 0;
}

// Special functions for servos - fill buffer with data
#pragma interrupt called
UInt16 RS485_WriteReg(UInt8 ID, UInt8 reg, UInt8 data)
{
	RS485Master.ui8SendRcvBuffer[0] = 0xff;
	RS485Master.ui8SendRcvBuffer[1] = 0xff;
	// ID
	RS485Master.ui8SendRcvBuffer[2] = ID;
	RS485Master.ui8SendRcvBuffer[3] = 0x02;
	RS485Master.ui8SendRcvBuffer[4] = RS485_COMMAND_WRITE;
	
	
	
	
	
	RS485Master.ui8SendRcvBuffer[5] = ~(RS485Master.ui8ReqSlaveAddress + 0x03);
	// Set data length
	RS485Master.ui8BytesToSend = 6;
	RS485Master.ui8RcvBufferIndex = 0;
}

#pragma interrupt called
UInt16 RS485_WriteAll(UInt8 ID)
{
	RS485Master.ui8SendRcvBuffer[0] = 0xff;
	RS485Master.ui8SendRcvBuffer[1] = 0xff;
	// ID
	RS485Master.ui8SendRcvBuffer[2] = ID;
	RS485Master.ui8SendRcvBuffer[3] = 0x02;	// Length
	RS485Master.ui8SendRcvBuffer[4] = RS485_COMMAND_WRITE;
	RS485Master.ui8SendRcvBuffer[5]
	
	
	
	
	
	
	
	RS485Master.ui8SendRcvBuffer[5] = ~(RS485Master.ui8ReqSlaveAddress + 0x03);
	// Set data length
	RS485Master.ui8BytesToSend = 6;
	RS485Master.ui8RcvBufferIndex = 0;
}

#pragma interrupt called
UInt16 RS485_States_Master(void)
{
	switch(RS485Master.ui8MasterState)
	{
		case RS485_M_STATE_IDLE:
		{
			// Check - command waiting?
			if(0 != RS485Master.ui8MasterRequest)
			{
				switch(RS485Master.ui8MasterRequest)
				{
					case RS485_COMMAND_PING:
					{
						// Send ping command
						// Prepare data
						RS485Master.ui8SendRcvBuffer[0] = 0xff;
						RS485Master.ui8SendRcvBuffer[1] = 0xff;
						// ID
						RS485Master.ui8SendRcvBuffer[2] = RS485Master.ui8ReqSlaveAddress;
						RS485Master.ui8SendRcvBuffer[3] = 0x02;
						RS485Master.ui8SendRcvBuffer[4] = 0x01;
						RS485Master.ui8SendRcvBuffer[5] = ~(RS485Master.ui8ReqSlaveAddress + 0x03);
						// Set data length
						RS485Master.ui8BytesToSend = 6;
						RS485Master.ui8RcvBufferIndex = 0;
						// Enable TX interrupts
						RS485_ENABLE_TX_INT;
						// Send first byte
						RS485_MasterWriteByte();
						// Set timeout
						RS485Master.ui16SlaveTimeout = RS485_SLAVE_TIMEOUT;
						// Reset request
						RS485Master.ui8MasterRequest = RS485_COMMAND_NONE;
						break;
					}
					case RS485_COMMAND_READ:
					{
						
						break;
					}
					case RS485_COMMAND_WRITE:
					{
						break;
					}
					default:
					{
						break;
					}
				}
			}
			break;
		}
		case RS485_M_STATE_REQUEST:
		{
			// Wait for response
			// Check timeout
			RS485Master.ui16SlaveTimeout--;
			// Timeout?
			if(0 == RS485Master.ui16SlaveTimeout)
			{
				// Disable interrupts
				RS485_DISABLE_TX_INT;
				RS485_DISABLE_TX_IDLE_INT;
				RS485Master.ui8BytesToSend = 0;
				RS485Master.ui8RcvBufferIndex = 0;
				RS485Master.ui8MasterState = RS485_M_STATE_IDLE;
				RS485Master.ui8MasterRequest = RS485_COMMAND_NONE;
			}
			break;
		}
		default:
		{
			// Disable interrupts
			RS485_DISABLE_TX_INT;
			RS485_DISABLE_TX_IDLE_INT;
			RS485Master.ui8BytesToSend = 0;
			RS485Master.ui8RcvBufferIndex = 0;
			RS485Master.ui8MasterState = RS485_M_STATE_IDLE;
			RS485Master.ui8MasterRequest = RS485_COMMAND_NONE;
			break;
		}
	}
	return 0;
}

#pragma interrupt called
UInt16 RS485_MasterecodeMessage(UInt8 data)
{/*
#define RS485_M_IDLE				0
#define RS485_M_WAIT_FOR_SIGNAL		1
#define RS485_M_WAIT_FOR_ID			2
#define RS485_M_WAIT_FOR_LENGTH		3
#define RS485_M_WAIT_FOR_INSTR_ERR	4
#define RS485_M_WAIT_FOR_PARAMETERS	5
#define RS485_M_WAIT_FOR_CHECKSUM	6*/
	// Just put stuff in a array
	RS485Master.ui8SendRcvBuffer[RS485Master.ui8RcvBufferIndex] = data;
	RS485Master.ui8RcvBufferIndex++;
	/*
	switch(RS485Master.ui8RcvState)
	{
		case RS485_M_IDLE:
		{
			// Data = signal?
			
			break;
		}
		default:
		{
			RS485Master.ui8RcvState = RS485_M_IDLE;
			break;
		}
	}*/
	
	return 0;
}

// Slave functions
UInt16 RS485_initData(void)
{
	RS485Data.REGS.ui8ID = RS485_ID;
	// Set to receive
	RS485_ENABLE_RX;
	// Disable TX interrupt
	RS485_DISABLE_TX_INT;
	// Set states
	RS485Data.ui8RcvState = RS485_IDLE;
	RS485Data.ui8TxState = RS485_TX_IDLE;
}

#pragma interrupt called
UInt16 RS485_writeByte(void)
{
	switch(RS485Data.ui8TxState)
	{
		case RS485_TX_IDLE:
		{
			// Data to send?
			if(RS485Data.ui8BytesToSend > RS485Data.ui8RcvBufferIndex)
			{
				// Init transmission
				RS485_ENABLE_TX;
				// Go to next state
				RS485Data.ui8TxState = RS485_TX_SENDING;
				// Reset index
				RS485Data.ui8RcvBufferIndex = 0;
			}
			else
			{
				// Erroreus interrupt. Disable both.
				RS485_DISABLE_TX_INT;
				RS485_DISABLE_TX_IDLE_INT;
			}
			break;
		}
		case RS485_TX_SENDING:
		{
			// Data to send?
			if(RS485Data.ui8BytesToSend > RS485Data.ui8RcvBufferIndex)
			{
				// Room in buffer?
				if(RS485_TEST_TX_EMPTY)
				{
					// Write one byte
					RS485_WRITE(RS485Data.ui8SendRcvBuffer[RS485Data.ui8RcvBufferIndex]);
					RS485Data.ui8RcvBufferIndex ++;
				}
			}
			else
			{
				// All data sent
				// Wait for end of transmission
				RS485Data.ui8TxState = RS485_TX_FINISHED;
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
				RS485Data.ui8TxState = RS485_TX_IDLE;
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
			RS485Data.ui8TxState = RS485_TX_IDLE;
			break;			
		}
	}
	
	return 0;
}

#pragma interrupt called
UInt16 RS485_States_slave(UInt8 data)
{
	switch(RS485Data.ui8RcvState)
	{
		case RS485_IDLE:
		{
			// Data == 0xFF?
			if(0xFF == data)
			{
				// Go to wait for signal
				RS485Data.ui8RcvState = RS485_WAIT_FOR_SIGNAL;
			}
			break;
		}
		case RS485_WAIT_FOR_SIGNAL:
		{
			// Data == 0xFF?
			if(0xFF == data)
			{
				// Go to wait for ID
				RS485Data.ui8RcvState = RS485_WAIT_FOR_ID;
			}
			else
			{
				RS485Data.ui8RcvState = RS485_IDLE;
			}
			break;
		}
		case RS485_WAIT_FOR_ID:
		{
			// Data == ID?
			if(data == RS485Data.REGS.ui8ID)
			{
				// ID match, wait for data length
				RS485Data.ui8RcvState = RS485_WAIT_FOR_LENGTH;
				// Set checksum to ID
				RS485Data.ui8Checksum = data;
			}
			else
			{
				// Id not mached, go to idle
				RS485Data.ui8RcvState = RS485_IDLE;
			}
			break;
		}
		case RS485_WAIT_FOR_LENGTH:
		{
			// Store how many bytes will follow
			// 
			RS485Data.ui8DataLength = data - 2;
			// Add to checksum
			RS485Data.ui8Checksum += data;
			// Go to wait for instruction
			RS485Data.ui8RcvState = RS485_WAIT_FOR_INSTR_ERR;
			break;
		}
		case RS485_WAIT_FOR_INSTR_ERR:
		{
			// Store instruction
			RS485Data.ui8InstrErr = data;
			// Add to checksum
			RS485Data.ui8Checksum += data;
			// Go to wait for parameters, if RS485Data.ui8DataLength > 0
			if(0 < RS485Data.ui8DataLength)
			{
				// Waiting for parameters
				RS485Data.ui8RcvState = RS485_WAIT_FOR_PARAMETERS;
				// Reset buffer index
				RS485Data.ui8RcvBufferIndex = 0;
			}
			else
			{
				// Wait for checksum
				RS485Data.ui8RcvState = RS485_WAIT_FOR_CHECKSUM;
			}		
			break;
		}
		case RS485_WAIT_FOR_PARAMETERS:
		{
			// Store parameter
			RS485Data.ui8SendRcvBuffer[RS485Data.ui8RcvBufferIndex] = data;
			RS485Data.ui8RcvBufferIndex ++;
			// Add to checksum
			RS485Data.ui8Checksum += data;
			// Have all parameters?
			if(RS485Data.ui8DataLength == RS485Data.ui8RcvBufferIndex)
			{
				// Yes, go to receive checksum
				RS485Data.ui8RcvState = RS485_WAIT_FOR_CHECKSUM;
			}
			break;
		}
		case RS485_WAIT_FOR_CHECKSUM:
		{
			//RS485Data.ui8Checksum = data;
			// Check checksum
			// Calculate
			RS485Data.ui8Checksum = RS485Data.ui8Checksum & 0XFF;
			RS485Data.ui8Checksum = ~RS485Data.ui8Checksum;
			// If checksum is OK, decode data
			if(RS485Data.ui8Checksum == data)
			{
				// Decode received message
				RS485_decodeMessage();				
			}
			// Go to idle state
			RS485Data.ui8RcvState = RS485_IDLE;
			break;
		}		
		default:
		{
			RS485Data.ui8RcvState = RS485_IDLE;
		}
	
	}
	return 0;
}

#pragma interrupt called
UInt16 RS485_decodeMessage(void)
{
	UInt8 ui8Temp = 0;
	switch(RS485Data.ui8InstrErr)
	{
		case RS485_INSTR_PING:
		{
			break;
		}
		case RS485_INSTR_READ_DATA:
		{
			// Setup data to be transmitted
			// Get address and length
			RS485Data.ui8RWAddress = RS485Data.ui8SendRcvBuffer[0];
			RS485Data.ui8DataLength = RS485Data.ui8SendRcvBuffer[1];
			// Signal
			RS485Data.ui8SendRcvBuffer[0] = 0xFF; 
			RS485Data.ui8SendRcvBuffer[1] = 0xFF;
			// ID
			RS485Data.ui8SendRcvBuffer[2] = RS485Data.REGS.ui8ID;
			// Length
			// Error
			RS485Data.ui8SendRcvBuffer[4] = RS485Data.REGS.ui8AlarmLED;
			RS485Data.ui8BytesToSend = 5; 
			// Set Length to 2
			RS485Data.ui8SendRcvBuffer[3] = 2;
			// Data
			for(ui8Temp = 0; ui8Temp < RS485Data.ui8DataLength; ui8Temp++)
			{
				// Store byte
				RS485Data.ui8SendRcvBuffer[RS485Data.ui8BytesToSend] = RS485Data.ui8Data[RS485Data.ui8RWAddress + ui8Temp];
				// Increase counter
				RS485Data.ui8BytesToSend ++;
				// Increase Length
				RS485Data.ui8SendRcvBuffer[3] ++;
			}
			// Calculate checksum
			RS485Data.ui8Checksum = 0;
			for(ui8Temp = 2; ui8Temp < RS485Data.ui8BytesToSend; ui8Temp++)
			{
				RS485Data.ui8Checksum += RS485Data.ui8SendRcvBuffer[ui8Temp];				
			}
			RS485Data.ui8Checksum = RS485Data.ui8Checksum & 0xFF;
			RS485Data.ui8Checksum = ~RS485Data.ui8Checksum;
			// Store checksum
			RS485Data.ui8SendRcvBuffer[RS485Data.ui8BytesToSend] = RS485Data.ui8Checksum;
			RS485Data.ui8BytesToSend ++;
			// Reset buffer index
			RS485Data.ui8RcvBufferIndex = 0;
			// Enable TX empty interrupt
			RS485_ENABLE_TX_INT;
			break;
		}
	}
	return 0;
}
