/***************************************************************************//**
 * @file	i2c.c
 * @brief	Source file to operate the I2C interface
 *
 * This file contains all the implementations for the functions defined in:
 * inc/i2c.h
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/

#ifndef F_CPU
#define F_CPU 8000000UL // CPU Frequency (IMPORTANT)
#endif

#include <avr/io.h> //General I/O
#include <util/twi.h> // For I2C interface
#include "i2c.h"
#include "memory.h"
#include "watchdog.h"
#include <avr/interrupt.h>
#include <util/delay.h> //Delay functions

#ifndef OK
#define OK 0
#endif

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Initialize I2C interface
 *
 * @param [in] F_I2C
 *	Frequency of I2C
 ******************************************************************************/
int I2C_INIT(uint32_t F_I2C){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + I2C_INIT_Code;
		
	REGISTER[memory_I2C_FREQ] = F_I2C;
	
	// Set frequency of I2C
	int16_t prescaler = ((F_CPU/F_I2C)-16)/2;
	if((prescaler < 0) || (prescaler > 255)) return I2C_CLOCK_OOB;
	TWBR = prescaler;
	TWSR &= ~((1<<TWPS1) | (1<<TWPS0));
	
	// Clear Control register
	TWCR = 0;
	
	// TODO: Delete (in register)
	REGISTER[memory_I2C_MAX_ITER] = 5;
	
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Write to I2C device
 *
 * @param [in] SLA
 *	Slave address
 * @param [in] data
 *	Array of bytes to be sent
 * @param [in] len
 *	Number of bytes to send
 ******************************************************************************/
int I2C_WRITE(uint8_t SLA, uint8_t * data, uint16_t len){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + I2C_WRITE_Code;
	
	resetWatchdogTimer();
	
	// Disable interrupts
	cli();
	
	//-------------------------------------------------------------------------------
	//                          Initialization
	//-------------------------------------------------------------------------------
	// Status to be returned
	int status = OK;

	// Number of tries
	REGISTER[memory_I2C_ITER] = 0;
	
	// Clear control register
	TWCR = 0;

	restart:
	if (++REGISTER[memory_I2C_ITER] > REGISTER[memory_I2C_MAX_ITER]) {goto quit;} //The device does not respond after MAX_ITER tries

	//-------------------------------------------------------------------------------
	//                                   Send START
	//-------------------------------------------------------------------------------
	// Send start
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

	// Wait for transmission
	uint32_t counter = 0;
	while (!(TWCR & (1<<TWINT)) && (counter < 100000)) {counter ++; _delay_us(1);}
	if (counter == 100000) {
		// Enable interrupts
		sei();
		
		return I2C_TIMEOUT;
	}

	// Check the status of the interface
	switch (TW_STATUS)
	{
		// Normal behavior
		case TW_REP_START:
		case TW_START:
		break;
		
		// Lost arbitration. Should never happen
		case TW_MT_ARB_LOST:
		status = I2C_START_ARB_LOST;
		goto restart;
		
		// Error. Should never happen. Do not send stop.
		default:
		// Enable interrupts
		sei();
		return I2C_START_CRITICAL;
	}
	status = 0;

	//-------------------------------------------------------------------------------
	//                                Send Device Address
	//-------------------------------------------------------------------------------
	// Save address in Register
	REGISTER[memory_I2C_SLA] = (uint32_t)(REGISTER[memory_I2C_SLA] << 8) | (SLA & 0xFE);
	
	// Load SLA+W into TWDR Register...
	TWDR = SLA & 0xFE;

	//...and send
	TWCR = (1<<TWINT) | (1<<TWEN);

	// Wait for transmission
	counter = 0;
	while (!(TWCR & (1<<TWINT)) && (counter < 100000)) {counter ++; _delay_us(1);}
	if (counter == 100000) {status = I2C_TIMEOUT; goto quit;};

	//4. Check the status of the interface
	switch (TW_STATUS)
	{
		// Normal behavior. Address acknowledged
		case TW_MT_SLA_ACK:
		break;
		
		// Not acknowledged. Device busy. Restart.
		case TW_MT_SLA_NACK:
		status = I2C_ADDR_NACK;
		goto restart;
		
		// Lost arbitration. Should never happen
		case TW_MT_ARB_LOST:
		status = I2C_ADDR_ARB_LOST;
		goto restart;
		
		// Error.
		default:
		status = I2C_ADDR_CRITICAL;
		goto quit;
	}
	status = 0;

	//-------------------------------------------------------------------------------
	//                                      Send Data
	//-------------------------------------------------------------------------------
	for (int II=0; II<len; II++){
		// Save data in Register
		REGISTER[memory_I2C_TX] = (REGISTER[memory_I2C_TX]<<8) | data[II];
		
		// Load data into TWDR Register... (and increment)
		TWDR = data[II];
		//...and send
		TWCR = (1<<TWINT) | (1<<TWEN);

		// Wait for transmission
		counter = 0;
		while (!(TWCR & (1<<TWINT)) && (counter < 100000)) {counter++; _delay_us(1);}
		if (counter == 100000) {status = I2C_TIMEOUT; goto quit;}

		// Check the status of the interface
		switch (TW_STATUS)
		{
			// Normal behavior. Data acknowledged
			case TW_MT_DATA_ACK:
			break;
			
			// Not acknowledged. Device busy. Restart.
			case TW_MT_DATA_NACK:
			status = I2C_DATA_NACK;
			goto restart;
			// Lost arbitration. Should never happen
			case TW_MT_ARB_LOST:
			status = I2C_DATA_ARB_LOST;
			goto restart;
			
			// Error.
			default:
			status = I2C_DATA_CRITICAL;
			goto quit;
		}
		status = 0;
	}

	//-------------------------------------------------------------------------------
	//                                       Quit
	//-------------------------------------------------------------------------------
	quit:
	//7. Transmit STOP condition
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	
	// Enable interrupts
	sei();

	return status;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Read from I2C interface
 *
 * @param [in] SLA
 *	Slave address
 * @param [in] data_write
 *	Array of bytes to be sent
 * @param [in] write_len
 *	Number of bytes to send
 * @param [out] data_read
 *	Pointer to array of received bytes
 * @param [in] read_len
 *	Number of bytes to read
 ******************************************************************************/
int I2C_READ(uint8_t SLA, uint8_t * data_write, uint16_t write_len, uint8_t * data_read, uint16_t read_len){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + I2C_READ_Code;
	
	resetWatchdogTimer();	
	// Disable interrupts
	cli();
	
	//-------------------------------------------------------------------------------
	//                                0. Initialization
	//-------------------------------------------------------------------------------
	// Status to be returned
	int status = OK;
	
	// Number of tries
	REGISTER[memory_I2C_ITER] = 0;
	
	// Clear control register
	TWCR = 0;

	restart:
	if (++REGISTER[memory_I2C_ITER] > REGISTER[memory_I2C_MAX_ITER]) {goto quit;} //The device does not respond after MAX_ITER tries
	
	//-------------------------------------------------------------------------------
	//                                  1. Send START
	//-------------------------------------------------------------------------------
	// Send start
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

	// Wait for transmission
	uint32_t counter = 0;
	while (!(TWCR & (1<<TWINT)) && (counter < 100000)) {counter ++; _delay_us(1);}
	if (counter == 100000) {
		// Enable interrupts
		sei();
		
		return I2C_TIMEOUT;
	}

	// Check the status of the interface
	switch (TW_STATUS)
	{
		// Normal behavior
		case TW_REP_START:
		case TW_START:
		break;
		
		// Lost arbitration. Should never happen
		case TW_MT_ARB_LOST:
		status = I2C_START_ARB_LOST;
		goto restart;
		
		// Error. Should never happen. Do not send stop.
		default:
		// Enable interrupts
		sei();
		return I2C_START_CRITICAL;
	}
	status = 0;

	//-------------------------------------------------------------------------------
	//                         2. Send Device Address + Write (0)
	//-------------------------------------------------------------------------------
	// Save address in Register
	REGISTER[memory_I2C_SLA] = (uint32_t)(REGISTER[memory_I2C_SLA] << 8) | (SLA & 0xFE);
	
	// Load SLA+W into TWDR Register...
	TWDR = SLA & 0xFE;

	//...and send
	TWCR = (1<<TWINT) | (1<<TWEN);

	// Wait for transmission
	counter = 0;
	while (!(TWCR & (1<<TWINT)) && (counter < 100000)) {counter ++; _delay_us(1);}
	if (counter == 100000) {status = I2C_TIMEOUT; goto quit;};

	//4. Check the status of the interface
	switch (TW_STATUS)
	{
		// Normal behavior. Address acknowledged
		case TW_MT_SLA_ACK:
		break;
		
		// Not acknowledged. Device busy. Restart.
		case TW_MT_SLA_NACK:
		status = I2C_ADDR_NACK;
		goto restart;
		// Lost arbitration. Should never happen
		case TW_MT_ARB_LOST:
		status = I2C_ADDR_ARB_LOST;
		goto restart;
		
		// Error.
		default:
		status = I2C_ADDR_CRITICAL;
		goto quit;
	}
	status = 0;

	//-------------------------------------------------------------------------------
	//                                3. Send Write Data
	//-------------------------------------------------------------------------------
	for (int II=0; II<write_len; II++){
		// Save data in Register
		REGISTER[memory_I2C_TX] = (REGISTER[memory_I2C_TX]<<8) | data_write[II];
		
		// Load data into TWDR Register... (and increment)
		TWDR = data_write[II];
		//...and send
		TWCR = (1<<TWINT) | (1<<TWEN);

		// Wait for transmission
		counter = 0;
		while (!(TWCR & (1<<TWINT)) && (counter < 100000)) {counter++; _delay_us(1);}
		if (counter == 100000) {status = I2C_TIMEOUT; goto quit;}

		// Check the status of the interface
		switch (TW_STATUS)
		{
			// Normal behavior. Data acknowledged
			case TW_MT_DATA_ACK:
			break;
			
			// Not acknowledged. Device busy. Restart.
			case TW_MT_DATA_NACK:
			status = I2C_DATA_NACK;
			goto restart;
			// Lost arbitration. Should never happen
			case TW_MT_ARB_LOST:
			status = I2C_DATA_ARB_LOST;
			goto restart;
			
			// Error.
			default:
			status = I2C_DATA_CRITICAL;
			goto quit;
		}
		status = 0;
	}
	

	//-------------------------------------------------------------------------------
	//                                 4. Send RESTART
	//-------------------------------------------------------------------------------
	// Send start
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

	// Wait for transmission
	counter = 0;
	while (!(TWCR & (1<<TWINT)) && (counter < 100000)) {counter ++; _delay_us(1);}
	if (counter == 100000) {status = I2C_TIMEOUT; goto quit;}

	// Check the status of the interface
	switch (TW_STATUS)
	{
		// Normal behavior
		case TW_REP_START:
		case TW_START:
		break;
		
		// Lost arbitration. Should never happen
		case TW_MR_ARB_LOST:
		status = I2C_RESTART_ARB_LOST;
		goto restart;
		
		// Error. Should never happen. Do not send stop.
		default:
		status = I2C_RESTART_CRITICAL;
		goto quit;
	}
	status = 0;

	//-------------------------------------------------------------------------------
	//                       5. Send Device Address + Read (1)
	//-------------------------------------------------------------------------------
	// Save address in Register
	REGISTER[memory_I2C_SLA] = (uint32_t)(REGISTER[memory_I2C_SLA] << 8) | (SLA | 0x01);
	
	// Load SLA+W into TWDR Register...
	TWDR = SLA | 0x01;
	
	//...and send
	TWCR = (1<<TWINT) | (1<<TWEN);

	// Wait for transmission
	counter = 0;
	while (!(TWCR & (1<<TWINT)) && (counter < 100000)) {counter ++; _delay_us(1);}
	if (counter == 100000) {status = I2C_TIMEOUT; goto quit;}

	//4. Check the status of the interface
	switch (TW_STATUS)
	{
		// Normal behavior. Address acknowledged
		case TW_MR_SLA_ACK:
		//n_ack++;
		break;
		
		// Not acknowledged. Device busy. Restart.
		case TW_MR_SLA_NACK:
		status = I2C_ADDR_NACK;
		goto restart;
		// Lost arbitration. Should never happen
		case TW_MR_ARB_LOST:
		status = I2C_ADDR_ARB_LOST;
		goto quit;
		
		// Error.
		default:
		status = I2C_ADDR_CRITICAL;
		goto quit;
	}
	status = 0;

	//-------------------------------------------------------------------------------
	//                                   6. Read
	//-------------------------------------------------------------------------------
	for (int II=0; II < read_len; II++)
	{
		if(II == read_len - 1) TWCR = (1<<TWINT) | (1<<TWEN); //Send NACK this time
		else TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
		
		// Wait for transmission
		long counter = 0;
		while (!(TWCR & (1<<TWINT)) && (counter < 100000)) {counter ++; _delay_us(1);}
		if (counter == 100000) {status = I2C_TIMEOUT; goto quit;}
		
		switch (TW_STATUS)
		{
			// Normal behavior. Data acknowledged
			case TW_MR_DATA_ACK:
			*data_read = TWDR; //Save data and then increment
			// Save data in Register
			REGISTER[memory_I2C_RX] = (REGISTER[memory_I2C_RX]<<8) | (*data_read++);
			break;
			
			case TW_MR_DATA_NACK:
			II = read_len; // Force end of loop
			*data_read = TWDR; //Save data and then increment
			// Save data in Register
			REGISTER[memory_I2C_RX] = (REGISTER[memory_I2C_RX]<<8) | (*data_read++);
			goto quit;
			
			// Error.
			default:
			status = I2C_READ_CRITICAL;
			goto quit;
		}
		status = 0;
	}
	//-------------------------------------------------------------------------------
	//                                   Q. Quit
	//-------------------------------------------------------------------------------
	quit:

	//7. Transmit STOP condition
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);

	// Enable interrupts
	sei();

	return status;
}
