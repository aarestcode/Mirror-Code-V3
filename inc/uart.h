/***************************************************************************//**
 * @file	uart.h
 * @brief	Header file to operate the UART interfaces
 *
 * This header file contains all the required definitions and function prototypes
 * through which to control the UART interfaces
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/
#ifndef UART_H_
#define UART_H_

#include <inttypes.h>

//--------------------------------------------------
//                 SERIAL INTERFACE 0
//--------------------------------------------------
// PINSET
// PORTD0 = RX
// PORTD1 = TX

// PARAMETERS
uint8_t UART0_buffer[256];
uint16_t UART0_buffer_index;

// ERROR ENUM
enum uart0_error{	
	UART0_INCORRECT_STOP = 21,
	UART0_FRAME_LOST,
	UART0_PARITY_CHECK
};

enum uart0_function{
	UART0_INIT_Code = 21,
	UART0_WRITE_Code,
	UART0_READ_Code,
	UART0_FLUSH_Code
};

// PROTOTYPES
int UART0_INIT(uint32_t USART_BAUDRATE);
int UART0_WRITE(uint8_t var);
int UART0_READ(uint8_t* var);
void UART0_FLUSH(void);

//--------------------------------------------------
//                 SERIAL INTERFACE 1
//--------------------------------------------------
// PINSET
// PORTD2 = RX
// PORTD3 = TX

// PARAMETERS
uint8_t UART1_buffer[256];
uint16_t UART1_buffer_index;

// ERROR ENUM
enum uart1_error{
	UART1_INCORRECT_STOP = 31,
	UART1_FRAME_LOST,
	UART1_PARITY_CHECK
};

enum uart1_function{
	UART1_INIT_Code = 21,
	UART1_WRITE_Code,
	UART1_READ_Code,
	UART1_FLUSH_Code
};

// PROTOTYPES
int UART1_INIT(uint32_t USART_BAUDRATE);
int UART1_WRITE(uint8_t var);
int UART1_READ(uint8_t* var);
void UART1_FLUSH(void);


#endif /* UART_H_ */