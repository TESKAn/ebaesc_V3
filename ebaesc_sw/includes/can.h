/*
 * can.h
 *
 *  Created on: Jan 20, 2017
 *      Author: Jure
 */

#ifndef CAN_H_
#define CAN_H_

/** 
  * @brief  CAN Tx message structure definition  
  */
typedef struct
{
  UInt32 StdId;  /*!< Specifies the standard identifier.
                        This parameter can be a value between 0 to 0x7FF. */

  UInt32 ExtId;  /*!< Specifies the extended identifier.
                        This parameter can be a value between 0 to 0x1FFFFFFF. */

  UInt8 IDE;     /*!< Specifies the type of identifier for the message that 
                        will be transmitted. This parameter can be a value 
                        of @ref CAN_identifier_type */

  UInt8 RTR;     /*!< Specifies the type of frame for the message that will 
                        be transmitted. This parameter can be a value of 
                        @ref CAN_remote_transmission_request */

  UInt8 DLC;     /*!< Specifies the length of the frame that will be 
                        transmitted. This parameter can be a value between 
                        0 to 8 */

  UInt8 Data[8]; /*!< Contains the data to be transmitted. It ranges from 0 
                        to 0xFF. */
} CanTxMsg;

/** 
  * @brief  CAN Rx message structure definition  
  */
typedef struct
{
  UInt32 StdId;  /*!< Specifies the standard identifier.
                        This parameter can be a value between 0 to 0x7FF. */

  UInt32 ExtId;  /*!< Specifies the extended identifier.
                        This parameter can be a value between 0 to 0x1FFFFFFF. */

  UInt8 IDE;     /*!< Specifies the type of identifier for the message that 
                        will be received. This parameter can be a value of 
                        @ref CAN_identifier_type */

  UInt8 RTR;     /*!< Specifies the type of frame for the received message.
                        This parameter can be a value of 
                        @ref CAN_remote_transmission_request */

  UInt8 DLC;     /*!< Specifies the length of the frame that will be received.
                        This parameter can be a value between 0 to 8 */

  UInt8 Data[8]; /*!< Contains the data to be received. It ranges from 0 to 
                        0xFF. */

  UInt8 FMI;     /*!< Specifies the index of the filter the message stored in 
                        the mailbox passes through. This parameter can be a 
                        value between 0 to 0xFF */
} CanRxMsg;

Int16 CAN_Init();
Int16 CAN_CRCAdd(FCAN_MB *MB);
Int16 CAN_TXStatus();
Int16 CAN_TXVoltage();

#endif /* CAN_H_ */
