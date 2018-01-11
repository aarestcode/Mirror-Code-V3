/***************************************************************************//**
 * @file	uart.c
 * @brief	Source file to operate the UART interfaces
 *
 * This file contains all the implementations for the functions defined in:
 * inc/uart.h
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/

#ifndef F_CPU
#define F_CPU 8000000UL // CPU Frequency (IMPORTANT)
#endif

#include "memory.h"
#include "uart.h"
#include "timer.h"
#include <avr/io.h> //General I/O
#include <avr/interrupt.h> // Interrupt use to receive data from UART
#include <string.h>

#ifndef OK
#define OK 0
#endif

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Initialize UART
 *
 * @param [in] USART_BAUDRATE
 *	Baudrate of the UART in bits per seconds
 ******************************************************************************/
int UART0_INIT(uint32_t USART_BAUDRATE){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + UART0_INIT_Code;
	
	REGISTER[memory_UART0_BAUD] = USART_BAUDRATE;
	uint16_t UBRR_VALUE = (((F_CPU / (USART_BAUDRATE * 16UL))) - 1);

	// Set the baud rate
	UBRR0H = (uint8_t)(UBRR_VALUE>>8);
	UBRR0L = (uint8_t)(UBRR_VALUE);
	
	// Enable receiver, transmitter and RX complete interrupt
	UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);
	
	// Set frame format: 8data, 1stop bit, parity mode disabled
	UCSR0C = (1<<USBS0) | (3<<UCSZ00);
	
	// Flush the receive buffer
	UART0_FLUSH();
	
	return OK;
}
int UART1_INIT(uint32_t USART_BAUDRATE){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + UART1_INIT_Code;
	
	REGISTER[memory_UART1_BAUD] = USART_BAUDRATE;
	uint16_t UBRR_VALUE = (((F_CPU / (USART_BAUDRATE * 16UL))) - 1);

	// Set the baud rate
	UBRR1H = (uint8_t)(UBRR_VALUE>>8);
	UBRR1L = (uint8_t)(UBRR_VALUE);
	
	// Enable receiver, transmitter and RX complete interrupt
	UCSR1B = (1<<RXEN1) | (1<<TXEN1) | (1<<RXCIE1);
	
	// Set frame format: 8data, 1stop bit, parity mode disabled
	UCSR1C = (1<<USBS1) | (3<<UCSZ10);
	
	// Flush the receive buffer
	UART1_FLUSH();
	
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Write to the UART
 *
 * @param [in] var
 *	Character to write to the UART
 ******************************************************************************/
int UART0_WRITE(uint8_t var){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + UART0_WRITE_Code;
	
	cli();
	
	// Wait for empty transmit buffer
	while ( !(UCSR0A & (1<<UDRE0)))
	
	// Start transmission
	REGISTER[memory_UART0_TX] = (REGISTER[memory_UART0_TX]<<8) | var;
	UDR0 = var;
	
	sei();
	return OK;
}
int UART1_WRITE(uint8_t var){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + UART1_WRITE_Code;
	
	cli();
	
	// Wait for empty transmit buffer
	while ( !(UCSR1A & (1<<UDRE1)));
	
	// Start transmission
	REGISTER[memory_UART1_TX] = (REGISTER[memory_UART1_TX]<<8) | var;
	UDR1 = var;
	
	sei();
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Read character from UART
 *
 * @param [out] var
 *	Character read from UART
 ******************************************************************************/
int UART0_READ(uint8_t* var){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + UART0_READ_Code;
	
	// Wait for incoming data
	while ( !(UCSR0A & (1<<RXC0)) );
	
	// Incorrect stop error
	if(UCSR0A & (1<<FE0)) return UART0_INCORRECT_STOP;
	// Frame lost
	if(UCSR0A & (1<<DOR0)) return UART0_FRAME_LOST;
	// Parity check error
	if(UCSR0A & (1<<UPE0)) return UART0_PARITY_CHECK;
	
	*var = UDR0;
	REGISTER[memory_UART0_RX] = (REGISTER[memory_UART0_RX]<<8) | (*var);
	
	return OK;
}
int UART1_READ(uint8_t* var){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + UART1_READ_Code;
		
	// Wait for incoming data
	while ( !(UCSR1A & (1<<RXC1)));
	
	// Incorrect stop error
	if(UCSR1A & (1<<FE1)) return UART1_INCORRECT_STOP;
	// Frame lost
	if(UCSR1A & (1<<DOR1)) return UART1_FRAME_LOST;
	// Parity check error
	if(UCSR1A & (1<<UPE1)) return UART1_PARITY_CHECK;
	
	*var = UDR1;
	REGISTER[memory_UART1_RX] = (REGISTER[memory_UART1_RX]<<8) | (*var);
	
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Flush the UART
 *
 ******************************************************************************/
void UART0_FLUSH(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + UART0_FLUSH_Code;
			
	unsigned char dummy;
	while ( UCSR0A & (1<<RXC0) ) dummy = UDR0;
	dummy++; // Because I'm tired of the compiler warning me it's not used
	
	// Set buffer
	memset(UART0_buffer, 0, sizeof(UART0_buffer));
	UART0_buffer_index = 0;
}
void UART1_FLUSH(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + UART1_FLUSH_Code;
		
	unsigned char dummy;
	while ( UCSR1A & (1<<RXC1) ) dummy = UDR1;
	dummy++; // Because I'm tired of the compiler warning me it's not used
	
	// Set buffer
	memset(UART1_buffer, 0, sizeof(UART1_buffer));
	UART1_buffer_index = 0;
	
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Interrupt upon receiving a full byte
 *
 ******************************************************************************/
ISR(USART0_RX_vect){
	if (IsTimeout(TIMER_UART0)) UART0_buffer_index = 0;
	UART0_READ(&UART0_buffer[UART0_buffer_index++]);
	REGISTER[memory_UART0_INDEX] = UART0_buffer_index;
	StartTimer(TIMER_UART0, REGISTER[memory_COMMUNICATION_TIMEOUT]);
}
ISR(USART1_RX_vect){
	if (IsTimeout(TIMER_UART1)) UART1_buffer_index = 0;
	UART1_READ(&UART1_buffer[UART1_buffer_index++]);
	REGISTER[memory_UART1_INDEX] = UART1_buffer_index;
	StartTimer(TIMER_UART1, REGISTER[memory_COMMUNICATION_TIMEOUT]);
}