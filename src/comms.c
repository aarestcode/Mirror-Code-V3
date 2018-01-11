/***************************************************************************//**
 * @file	comms.c
 * @brief	Source file to manage comms
 *
 * This file contains all the implementations for the functions defined in:
 * inc/comms.h
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/

#include "uart.h"
#include "timer.h"
#include "memory.h"
#include "comms.h"
#include <avr/interrupt.h>
#include <string.h>

#ifndef OK
#define OK 0
#endif


/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Initialize comms
 *
 * @param [in] timeout_ms
 *	Timeout when receiving a message
 ******************************************************************************/
int COMMS_INIT(uint32_t timeout_ms){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + COMMS_INIT_Code;
	
	// Timeout to read a known length message
	REGISTER[memory_COMMUNICATION_TIMEOUT] = timeout_ms;
	
	REGISTER[memory_MESSAGE_COUNT_SPARE] = 0;
	REGISTER[memory_MESSAGE_COUNT_XBEE] = 0;
	REGISTER[memory_FEEDBACK_COUNT_SPARE] = 0;
	REGISTER[memory_FEEDBACK_COUNT_XBEE] = 0;
	
	
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Check if a telecommand is waiting in the buffer
 *
 ******************************************************************************/
port_t IsCommandWaiting(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + IsCommandWaiting_Code;
	
	cli();	
	
	if(UART0_buffer_index >= (MessageN)){ // USB (priority)
		memcpy(Message, &UART0_buffer[UART0_buffer_index - (MessageN)], MessageN);
		UART0_FLUSH();
		sei();
		REGISTER[memory_MESSAGE_COUNT_SPARE] ++;
		return PORT_SPARE;
	}
	if(UART1_buffer_index >= MessageN){ // XBee
		memcpy(Message, &UART1_buffer[UART1_buffer_index - (MessageN)], MessageN);
		UART1_FLUSH();
		sei();
		REGISTER[memory_MESSAGE_COUNT_XBEE] ++;
		return PORT_XBEE;
	}
	
	sei();
	
	return PORT_NONE;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Send message back to camera/spacecraft
 *
 * @param [in] port
 *	Port to send the message to
 * @param [in] address
 *	Command byte of the message
 * @param [in] data
 *	Data part of the message
 ******************************************************************************/
int SendFeedback(port_t port, uint8_t address, int32_t data, uint8_t error){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + SendFeedback_Code;
	
	int status;
	int II;
	int index = 0;
	uint32_t sum = 0;

	// Address
	for (II = 0; II < (MessageCommandN); II++){
		sum += (address >> 8*(MessageCommandN-II-1)) & 0xff;
		Feedback[index++] = (uint8_t)(address >> 8*(MessageCommandN-II-1));
	}
	
	// Data
	for (II = 0; II < (MessageDataN); II++){
		sum += (data >> 8*(MessageDataN-II-1)) & 0xff;
		Feedback[index++] = (uint8_t)(data >> 8*(MessageDataN-II-1));
	}
	
	// Error
	for (II = 0; II < (MessageErrorN); II++){
		sum += (error >> 8*(MessageErrorN-II-1)) & 0xff;
		Feedback[index++] = (uint8_t)(error >> 8*(MessageErrorN-II-1));
	}

	// Checksum
	if(MessageChecksumN){
		uint32_t mask = (1<<8*MessageChecksumN) - 1;
		uint32_t checksum = mask - (sum & mask);
		
		for (II = 0; II < MessageChecksumN; II++){
			Feedback[index++] = (uint8_t)(checksum >> 8*(MessageChecksumN-II-1));
		}
	}

	
	// Send
	if (port == PORT_SPARE){
		for(II = 0; II < (MessageN); II++){
			status = UART0_WRITE(Feedback[II]);
			if(status) return status;
			REGISTER[memory_FEEDBACK_COUNT_SPARE]++;
		}
	}
	else if (port == PORT_XBEE){
		for(II = 0; II < (MessageN); II++){
			status = UART1_WRITE(Feedback[II]);
			if(status) return status;
			REGISTER[memory_FEEDBACK_COUNT_XBEE]++;
		}
	}
	else return COMMS_WRITE_PORT;
	
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Check incoming message
 *
 * @param [out] command
 *	Command byte of the message
 * @param [out] data
 *	Data part of the message
 * @param [out] checksum
 *	Received Checksum
 ******************************************************************************/
int CheckMessage(uint8_t * command, int32_t * data, uint16_t * checksum){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + CheckMessage_Code;
	
	// Checksum
	if(MessageChecksumN)
	{
		uint32_t sum = 0;
		for(int II = 0; II < (MessageN)-(MessageChecksumN); II++)
		sum += Message[II];
		
		uint32_t check = 0;
		for (int II = 0; II < (MessageChecksumN); II++)
		check |= (Message[(MessageN)-(MessageChecksumN)+II] << 8*((MessageChecksumN)-II-1));
		
		sum += check;
		
		uint32_t mask = (1 << 8*(MessageChecksumN)) - 1;
		*checksum = sum & mask;
		
		if(*checksum != mask) return COMMS_CHECK_CHECKSUM;
	}
	
	// Command
	*command = 0;
	for (int II = 0; II < (MessageCommandN); II++)
	*command |= (Message[II] << 8*((MessageCommandN)-II-1));
	
	// Data
	*data = 0;
	for (int II = 0; II < (MessageDataN); II++)
	*data |= ((int32_t)Message[(MessageCommandN)+II] << 8*((MessageDataN)-II-1));
	
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Load large amount of bytes (!! deactivates interrupts !!)
 *
 * @param [in] port
 *	Port to read the data from
 * @param [out] buffer
 *	Pointer to the received data bytes
 * @param [in] numbytes
 *	Number of bytes to read
 ******************************************************************************/
int LoadData(port_t port, uint8_t * buffer, uint16_t numbytes){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + LoadData_Code;
	
	cli(); // Could not make use of interrupt for unknown reason. So I'm doing it the old way.
	
	if (port == PORT_SPARE){
		for (int II = 0; II < numbytes; II++) UART0_READ(&buffer[II]);
		UART0_FLUSH();
		sei();
	}
	else if (port == PORT_XBEE){
		for (int II = 0; II < numbytes; II++) UART1_READ(&buffer[II]);
		UART1_FLUSH();
		sei();
	}
	else {
		sei();
		return COMMS_READ_PORT;
	}
	
	return OK;
}
