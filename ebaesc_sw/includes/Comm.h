/*
 * Commcomm.h
 *
 *  Created on: May 20, 2015
 *      Author: jmoc
 */

#ifndef COMM_H_
#define COMM_H_



// Function declarations
Int16 Comm_initData();
UInt16 update_crc(UInt16 crc_accum, UInt8 *data_blk_ptr, UInt16 data_blk_size);





// Structure that holds all relevant data for motor
typedef struct tagCOMMDATA
{
	UInt8 errStatus;
	UInt8 ui8TXState;
	UInt8 ui8RXState;
	UInt8 ui8ReturnDelay;
	
	// How many bytes in our structure
	UInt16 ui16RegsBytes;
	// Readonly limits
	UInt16 ui16ReadOnlyLow;
	UInt16 ui16ReadOnlyHigh;
	
	// Transmit buffer
	UInt8 CommTXBuffer[128];	// Buffer
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
		UInt8 ui8REGSData[64];				// Main data structure
		struct
		{
			// Some params
			// Errors
			UInt16 ui16Errors;			// 0
			UInt16 ui16ModelNumber;		// 2
			UInt8 ui8FirmwareVersion;	// 4
			UInt8 ui8ID;				// 5
			UInt8 ui8BaudRate;			// 6
			UInt8 ui8ReturnDelayTime;	// 7


			// Motor state - idle, run, error
			UInt16 ui16State;			// 8

			// Status of the motor
			Int16 i16UIn;				// 10
			Int16 i16IIn;				// 12
			Int16 i16PIn;				// 14
			Int16 i16RPM;				// 16
			Int16 i16Position;			// 18
			UInt8 ui8PresentTemperature;	//19
			// Future expansion
			UInt8 ui8Empty1[11];			// 32 bytes total

			// Motor control
			// Arm
			UInt8 ui8Armed;				// 32
			// Park
			UInt8 ui8Park;				// 33
			// Reverse rotation
			UInt8 ui8ReverseRotation;	// 34
			UInt8 uiEmpty2;
			// Park position
			Int16 i16ParkPosition;		// 36

			Int16 i16SetRPM;			// 38
			Int16 i16MaxRPM;			// 40
			Int16 i16MinRPM;			// 42

			// PWM input
			UInt8 ui8MeasurePWMMin;		// 44
			UInt8 ui8MeasurePWMMax;		// 45
			UInt8 ui8UsePWMIN;			// 46
			UInt8 uiEmpty3;				// 47
			Int16 i16PWMMin;			// 48
			Int16 i16PWMMax;			// 50
			Int16 i16ZeroSpeedPWM;		// 52
			Int16 i16CurrentPWM;		// 54	//55 bytes total

			UInt8 uiEmpty4[8];					// 64 bytes total

		}REGS;
	};

}COMMDATA;




#endif /* COMM_H_ */
