/*
 * RS485comm.h
 *
 *  Created on: May 20, 2015
 *      Author: jmoc
 */

#ifndef RS485COMM_H_
#define RS485COMM_H_



// Function declarations
Int16 RS485_initData();
Int16 RS485_SyncToSystem();
Int16 RS485_SyncToComm();
void RS485_Timer();
void RS485_writeByte();
void RS485_States_slave(UInt8 data);
void RS485_decodeMessage();
UInt16 update_crc(UInt16 crc_accum, UInt8 *data_blk_ptr, UInt16 data_blk_size);

// Hardware dependent macros
#define RS485_ENABLE_RX						ioctl(GPIO_C, GPIO_CLEAR_PIN, BIT_3)
#define RS485_ENABLE_TX						ioctl(GPIO_C, GPIO_SET_PIN, BIT_3)
#define RS485_ENABLE_TX_INT					ioctl(SCI_0, SCI_TX_EMPTY_INT, SCI_ENABLE)
#define RS485_DISABLE_TX_INT				ioctl(SCI_0, SCI_TX_EMPTY_INT, SCI_DISABLE)
#define RS485_ENABLE_TX_IDLE_INT			ioctl(SCI_0, SCI_TX_IDLE_INT, SCI_ENABLE)
#define RS485_DISABLE_TX_IDLE_INT			ioctl(SCI_0, SCI_TX_IDLE_INT, SCI_DISABLE)
#define RS485_WRITE(X)						ioctl(SCI_0, SCI_WRITE_DATA, X)
#define RS485_READ							ioctl(SCI_0, SCI_READ_DATA, NULL)
#define RS485_TEST_TX_EMPTY					ioctl(SCI_0, SCI_GET_TX_EMPTY, NULL)
#define RS485_TEST_TX_IDLE					ioctl(SCI_0, SCI_GET_TX_IDLE, NULL)
#define RS485_TEST_RX_FULL					ioctl(SCI_0, SCI_GET_RX_FULL, NULL)

// Receiver states
#define RS485_RX_IDLE					0
#define RS485_RX_WAIT_FOR_SIGNAL		1
#define RS485_RX_WAIT_FOR_ID			2
#define RS485_RX_WAIT_FOR_LENGTH		3
#define RS485_RX_WAIT_FOR_INSTR_ERR		4
#define RS485_RX_WAIT_FOR_PARAMETERS	5
#define RS485_RX_WAIT_FOR_CHECKSUM		6

// Transmitter states
#define RS485_TX_IDLE				0
#define RS485_TX_DELAY				1
#define RS485_TX_SENDING			2
#define RS485_TX_FINISHED			3

// Instructions
#define RS485_INSTR_PING			1
#define RS485_INSTR_READ_DATA		2
#define RS485_INSTR_WRITE_DATA		3
#define RS485_INSTR_REG WRITE		4
#define RS485_INSTR_ACTION			5
#define RS485_INSTR_RESET			6
#define RS485_INSTR_SYNC_WRITE		7

// Structure that holds all relevant data for motor
typedef struct
{
	UInt8 errStatus;
	UInt8 ui8TXState;
	UInt8 ui8RXState;
	
	// How many bytes in our structure
	UInt16 ui16RegsBytes;
	// Readonly limits
	UInt16 ui16ReadOnlyLow;
	UInt16 ui16ReadOnlyHigh;
	
	// Transmit buffer
	UInt8 RS485TXBuffer[128];	// Buffer
	UInt8 ui8TXBytesLeft;		// How many bytes in buffer
	UInt8 ui8TXIndex;			// Next byte to be transmitted
	
	UInt8 ui8RXCounter;			// Count bytes for RX
	
	// Timeouts
	UInt16 ui16RXTimeoutCounter;
	UInt16 ui16RXCommTimeout;

	UInt16 ui16TXTimeoutCounter;
	UInt16 ui16TXCommTimeout;
	
	struct
	{
		UInt8 ui8Parameters[128];	
		UInt8 ui8RS485RXIndex;			// Index of next place to write to
		UInt8 ui8ParamByteCount;		// How many bytes are in parameters		
		struct
		{
			union
			{
				UInt32 ui32Header;
				UInt8 ui8Bytes[4];
			}HEADER;
			UInt8 ui8ID;
			union
			{
				UInt16 ui16PacketLength;
				UInt8 ui8Bytes[2];
			}LENGTH;
			UInt8 ui8Instruction;	

			union
			{
				UInt16 ui16CRC;
				UInt8 ui8Bytes[2];
			}CRCDATA;	
		};
	}RXDATA;
	
	union
	{
		UInt8 ui8REGSData[69];				// Main data structure
		struct
		{
			// Some params
			UInt16 ui16ModelNumber;		// 0
			UInt8 ui8FirmwareVersion;	// 2
			UInt8 ui8ID;				// 3
			UInt8 ui8BaudRate;			// 4
			UInt8 ui8Empty;				// 5
			// Errors
			UInt16 ui16Errors;			// 6
			// Future expansion
			UInt8 uiEmpty1[24];			// 32 bytes total
			// Status of the motor
			float f32UIn;				// 32
			float f32IIn;				// 36
			float f32PIn;				// 40
			float f32RPM;				// 44
			float f32SetRPM;			// 48
			// Motor control
			// Motor state - idle, run, error
			UInt16 ui16State;			// 52
			// State transition command
			UInt16 ui16Command;			// 54
			// Arm
			UInt8 ui8Armed;				// 56
			// Park
			UInt8 ui8Park;				// 57
			// Reverse rotation
			UInt8 ui8ReverseRotation;	// 58
			// Park position
			Int16 i16ParkPosition;		// 59
			
			// Motor min/max RPM
			float f32MinRPM;			// 61
			float f32MaxRPM;			// 65	// +4 = total
		}REGS;
	};

}RS485MOTOR;




#endif /* RS485COMM_H_ */
