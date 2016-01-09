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
Int16 RS485_Timer();
Int16 RS485_writeByte(void);
Int16 RS485_States_slave(UInt8 data);
Int16 RS485_decodeMessage(void);

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
#define RS485_RX_WAIT_FOR_SIGNAL		2
#define RS485_RX_WAIT_FOR_ID			4
#define RS485_RX_WAIT_FOR_LENGTH		8
#define RS485_RX_WAIT_FOR_INSTR_ERR		16
#define RS485_RX_WAIT_FOR_PARAMETERS	32
#define RS485_RX_WAIT_FOR_CHECKSUM		64

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
	
	// Transmit ring buffer
	RING_BUFFER RS485TXBuff;
	UInt8 RS485TXBuffer[128];
	
	// Receive buffer
	UInt8 RS485RXBuffer[128];
	UInt8 ui8RS485RXBufferIndex;
	UInt8 ui8RS485RXChecksum;
	UInt8 ui8RS485RXBytes;
	UInt8 ui8RS485RXInstrErr;
	UInt8 ui8TXChecksum;
	
	// Timeouts
	UInt16 ui16RXTimeoutCounter;
	UInt16 ui16RXCommTimeout;

	UInt16 ui16TXTimeoutCounter;
	UInt16 ui16TXCommTimeout;
	
	union
	{
		UInt8 ui8Data[68];				// Main data structure
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
			UInt8 uiEmpty1[24];	// 32 bytes total
			// Status of the motor
			float f32UIn;				// 33
			float f32IIn;				// 37
			float f32PIn;				// 41
			float f32RPM;				// 45
			float f32SetRPM;			// 49
			// Motor control
			// Motor state - idle, run, error
			UInt16 ui16State;			// 53
			// State transition command
			UInt16 ui16Command;			// 55
			// Arm
			UInt8 ui8Armed;				// 57
			// Park
			UInt8 ui8Park;				// 58
			// Park position
			Int16 i16ParkPosition;		// 59
			
			// Motor min/max RPM
			float f32MinRPM;			// 61
			float f32MaxRPM;			// 65	// 36 bytes + 32 bytes = 68 bytes total total
		}REGS;
	};

}RS485MOTOR;




#endif /* RS485COMM_H_ */
