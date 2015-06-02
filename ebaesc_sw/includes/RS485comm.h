/*
 * RS485comm.h
 *
 *  Created on: May 20, 2015
 *      Author: jmoc
 */

#ifndef RS485COMM_H_
#define RS485COMM_H_

// This unit ID
#define RS485_ID		0x03

// Master mode macros
// Define number of slave devices
#define RS485_NUMSLAVES		3

// Function declarations
UInt16 RS485_MasterInitData(void);
UInt16 RS485_MasterWriteByte(void);
UInt16 RS485_States_Master(UInt8 data);
UInt16 RS485_MasterecodeMessage(void);

UInt16 RS485_initData(void);
UInt16 RS485_writeByte(void);
UInt16 RS485_States_slave(UInt8 data);
UInt16 RS485_decodeMessage(void);

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

// Master defines
// Receiver states
#define RS485_M_IDLE				0
#define RS485_M_WAIT_FOR_SIGNAL		1
#define RS485_M_WAIT_FOR_ID			2
#define RS485_M_WAIT_FOR_LENGTH		3
#define RS485_M_WAIT_FOR_INSTR_ERR	4
#define RS485_M_WAIT_FOR_PARAMETERS	5
#define RS485_M_WAIT_FOR_CHECKSUM	6

// Transmitter states
#define RS485_M_TX_IDLE				0
#define RS485_M_TX_SENDING			1
#define RS485_M_TX_FINISHED			2

// Slave defines
// Receiver states
#define RS485_IDLE					0
#define RS485_WAIT_FOR_SIGNAL		2
#define RS485_WAIT_FOR_ID			4
#define RS485_WAIT_FOR_LENGTH		8
#define RS485_WAIT_FOR_INSTR_ERR	16
#define RS485_WAIT_FOR_PARAMETERS	32
#define RS485_WAIT_FOR_CHECKSUM		64

// Transmitter states
#define RS485_TX_IDLE				0
#define RS485_TX_SENDING			1
#define RS485_TX_FINISHED			2

// Instructions
#define RS485_INSTR_PING			1
#define RS485_INSTR_READ_DATA		2
#define RS485_INSTR_WRITE_DATA		3
#define RS485_INSTR_REG WRITE		4
#define RS485_INSTR_ACTION			5
#define RS485_INSTR_RESET			6
#define RS485_INSTR_SYNC_WRITE		7

// Structure that holds all relevant data
typedef struct tagRS485SERVO
{
	union
	{
		struct
		{
			UInt8 ui8Data[49];				// Main data structure
			UInt8 ui8SendRcvBuffer[56];		// Buffer for transmitting/receiving data
			UInt8 ui8RcvBufferIndex;		// Index in buffer for receiving/transmitting
			UInt8 ui8BytesToSend;			// How many bytes to transmitt (including signal, ID, checksum)
			UInt8 ui8RcvState;				// Receiver state
			UInt8 ui8TxState;				// Transmitter state
			UInt8 ui8DataLength;			// How many parameter bytes to receive/read
			UInt8 ui8InstrErr;				// Instruction/error code of current message
			UInt8 ui8RWAddress;				// Address to start read/write
			UInt8 ui8BytesToRW;				// Bytes to read/write
			UInt8 ui8ParamsReceived;		
			UInt8 ui8Empty[1];		
			UInt8 ui8Checksum;				// Checksum of received data
			/*
			union 
			{
				UInt8 ui8Bytes[2];
				UInt16 ui16Word;
			}RcvdData;*/
		};
		struct
		{
			UInt16 ui16ModelNumber;
			UInt8 ui8FirmwareVersion;
			UInt8 ui8ID;
			UInt8 ui8BaudRate;
			UInt8 ui8ReturnDelayTime;
			UInt16 CWAngleLimit;
			UInt16 CCWAngleLimit;
			UInt8 ui8Empty1;
			UInt8 ui8InternalTempLimit;
			UInt8 ui8LowLimitVoltage;
			UInt8 ui8HighLimitVoltage;
			UInt16 ui16MaxTorque;
			UInt8 ui8StatusReturnLevel;
			UInt8 ui8AlarmLED;
			UInt8 ui8Empty2[5];
			UInt8 ui8AlarmShutdown;
			UInt8 ui8TorqueEnabled;
			UInt8 ui8LED;
			UInt8 ui8CWComplianceMargin;
			UInt8 ui8CCWComplianceMargin;
			UInt8 ui8CWComplianceSlope;
			UInt8 ui8CCWComplianceSlope;
			UInt16 ui16GoalPosition;
			UInt16 ui16MovingSpeed;
			UInt16 ui16TorqueLimit;
			UInt16 ui16PresentPosition;
			UInt16 ui16PresentSpeed;
			UInt16 ui16PresentLoad;
			UInt8 ui8PresentVoltage;
			UInt8 ui8PresentTemperature;
			UInt8 ui8Registered;
			UInt8 ui8Empty3;
			UInt8 ui8Moving;
			UInt8 ui8Lock;
			UInt16 ui16Punch;	
			UInt8 ui8SendRcvBuffer_[56];
			UInt8 ui8RcvBufferIndex_;
			UInt8 ui8BytesToSend_;
			UInt8 ui8RcvState_;
			UInt8 ui8TxState_;
			UInt8 ui8DataLength_;
			UInt8 ui8InstrErr_;
			UInt8 ui8RWAddress_;
			UInt8 ui8BytesToRW_;
			UInt8 ui8ParamsReceived_;
			UInt8 ui8Empty_[1];			
			UInt8 ui8Checksum_;
			//UInt8 ui8RcvdData[2];

		}REGS;
	};
	
}RS485SERVO;


typedef struct tagRS485SERVOSLAVE
{
	union
	{
		UInt8 ui8Data[49];				// Main data structure
		struct
		{
			UInt16 ui16ModelNumber;
			UInt8 ui8FirmwareVersion;
			UInt8 ui8ID;
			UInt8 ui8BaudRate;
			UInt8 ui8ReturnDelayTime;
			UInt16 CWAngleLimit;
			UInt16 CCWAngleLimit;
			UInt8 ui8Empty1;
			UInt8 ui8InternalTempLimit;
			UInt8 ui8LowLimitVoltage;
			UInt8 ui8HighLimitVoltage;
			UInt16 ui16MaxTorque;
			UInt8 ui8StatusReturnLevel;
			UInt8 ui8AlarmLED;
			UInt8 ui8Empty2[5];
			UInt8 ui8AlarmShutdown;
			UInt8 ui8TorqueEnabled;
			UInt8 ui8LED;
			UInt8 ui8CWComplianceMargin;
			UInt8 ui8CCWComplianceMargin;
			UInt8 ui8CWComplianceSlope;
			UInt8 ui8CCWComplianceSlope;
			UInt16 ui16GoalPosition;
			UInt16 ui16MovingSpeed;
			UInt16 ui16TorqueLimit;
			UInt16 ui16PresentPosition;
			UInt16 ui16PresentSpeed;
			UInt16 ui16PresentLoad;
			UInt8 ui8PresentVoltage;
			UInt8 ui8PresentTemperature;
			UInt8 ui8Registered;
			UInt8 ui8Empty3;
			UInt8 ui8Moving;
			UInt8 ui8Lock;
			UInt16 ui16Punch;	
			UInt8 ui8SendRcvBuffer_[56];
		}REGS;
	};
	
}RS485SERVOSLAVE;



typedef struct tagRS485SERVOMASTER
{
	// Structures that hold data for all servos
	RS485SERVOSLAVE RS485Slaves[RS485_NUMSLAVES];
	UInt8 ui8SendRcvBuffer[56];		// Buffer for transmitting/receiving data
	UInt8 ui8RcvBufferIndex;		// Index in buffer for receiving/transmitting
	UInt8 ui8BytesToSend;			// How many bytes to transmitt (including signal, ID, checksum)
	UInt8 ui8RcvState;				// Receiver state
	UInt8 ui8TxState;				// Transmitter state
	UInt8 ui8DataLength;			// How many parameter bytes to receive/read
	UInt8 ui8InstrErr;				// Instruction/error code of current message
	UInt8 ui8RWAddress;				// Address to start read/write
	UInt8 ui8BytesToRW;				// Bytes to read/write
	UInt8 ui8ParamsReceived;		
	UInt8 ui8Checksum;				// Checksum of received data
	union 
	{
		UInt8 ui8Bytes[2];
		UInt16 ui16Word;
	}RcvdData;
	
}RS485SERVOMASTER;

#endif /* RS485COMM_H_ */
