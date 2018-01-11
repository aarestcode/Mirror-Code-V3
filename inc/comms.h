/***************************************************************************//**
 * @file	comms.h
 * @brief	Header file to manage communications
 *
 * This header file contains all the required definitions and function prototypes
 * through which to manage communications
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/
#ifndef COMMS_H_
#define COMMS_H_

#include <inttypes.h>

// PARAMETERS
#define MessageCommandN 1 //Length of command in byte
#define MessageDataN 4 // Length of data
#define MessageErrorN 1 // Length of data
#define MessageChecksumN 1 // length of checksum
#define MessageN (MessageCommandN + MessageDataN + MessageErrorN + MessageChecksumN) // Command(1) + Data(4) + Checksum(1)
unsigned char Message[MessageN]; //Vector of received bytes
unsigned char Feedback[MessageN]; //Vector of transmitted bytes

// ERROR ENUM
enum comms_error{	
	COMMS_READ_PORT = 81,
	COMMS_WRITE_PORT,
	COMMS_CHECK_CHECKSUM
};

enum comms_function{
	COMMS_INIT_Code = 81,
	IsCommandWaiting_Code,
	SendFeedback_Code,
	CheckMessage_Code,
	LoadData_Code,
};

// PORT
typedef enum{
	PORT_NONE,
	PORT_SPARE,
	PORT_XBEE
}port_t;

// FUNCTIONS
int COMMS_INIT(uint32_t timeout_ms);
port_t IsCommandWaiting(void);
int SendFeedback(port_t port, uint8_t address, int32_t data, uint8_t error);
int CheckMessage(uint8_t * command, int32_t * data, uint16_t * checksum);
int LoadData(port_t port, uint8_t * buffer, uint16_t numbytes);

#endif /* COMMS_H_ */