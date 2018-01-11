/***************************************************************************//**
 * @file	spi.c
 * @brief	Source file to operate the SPI interface
 *
 * This file contains all the implementations for the functions defined in:
 * inc/spi.h
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/

#ifndef F_CPU
#define F_CPU 8000000UL // CPU Frequency (IMPORTANT)
#endif

#include "spi.h"
#include "memory.h"
#include <util/delay.h> //Delay functions
#include <avr/io.h> //General I/O
#include <avr/interrupt.h> // Interrupt use to receive data from UART

#ifndef OK
#define OK 0
#endif

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Initialize SPI interface
 *
 * @param [in] F_SPI
 *	Frequency of SPI
 ******************************************************************************/
int SPI_INIT(uint32_t F_SPI){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + SPI_INIT_Code;
	
	/* Set MOSI, SCK and SS output, all others input (MISO) */
	DDR_SPI |= (1<<MOSI_SPI)|(1<<SCK_SPI);
	DDR_SS_PICO |= (1<<SS_PICO);
	DDR_SS_HV |= (1<<SS_HV);
	DDR_SS_BIAS |= (1<<SS_BIAS);
	
	REGISTER[memory_SPI_FREQ] = F_SPI;
	
	/* Set clock */
	uint16_t prescaler = ceil(log(F_CPU/F_SPI)/log(2)); //ceil to unsure frequency less then F_SPI
	if(prescaler==1) {SPSR |= (1<<SPI2X); SPCR &= ~(1<<SPR1); SPCR &= ~(1<<SPR0);}
	else if(prescaler==2) {SPSR &= ~(1<<SPI2X); SPCR &= ~(1<<SPR1); SPCR &= ~(1<<SPR0);}
	else if(prescaler==3) {SPSR |= (1<<SPI2X); SPCR &= ~(1<<SPR1); SPCR |= (1<<SPR0);}
	else if(prescaler==4) {SPSR &= ~(1<<SPI2X); SPCR &= ~(1<<SPR1); SPCR |= (1<<SPR0);}
	else if(prescaler==5) {SPSR |= (1<<SPI2X); SPCR |= (1<<SPR1); SPCR &= ~(1<<SPR0);}
	else if(prescaler==6) {SPSR &= ~(1<<SPI2X); SPCR |= (1<<SPR1); SPCR &= ~(1<<SPR0);}
	else if(prescaler==7) {SPSR &= ~(1<<SPI2X); SPCR |= (1<<SPR1); SPCR |= (1<<SPR0);}
	else return SPI_CLOCK_OOB;
	
	/* Enable SPI, Master */ // For PICO: CPOL = 0, CPHA = 0 // For HV: CPOL=0, CPHA = 1;
	SPCR = (1<<SPE)|(1<<MSTR);;
	
	/*Put SS line high */
	PORT_SS_PICO |= (1<<SS_PICO);
	PORT_SS_HV |= (1<<SS_HV);
	PORT_SS_BIAS |= (1<<SS_BIAS);
	
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Write to SPI interface
 *
 * @param [in] select
 *	SPI lines select
 * @param [in] data
 *	Array of bytes to send
 * @param [in] nbytes
 *	Number f bytes to send
 ******************************************************************************/
int SPI_WRITE(SPI_select_t select, uint8_t * data, uint16_t nbytes){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + SPI_WRITE_Code;
	
	// Disable interrupts
	cli();
	
	// Begin the transmission. Adjust phase (CPHA=0 for PICO, CPHA=1 for HV). Put SS line low
	if(select==SELECT_PICO) {SPCR &= ~(1<<CPHA); PORT_SS_PICO &= ~(1<<SS_PICO);} //PICO
	if(select==SELECT_HV) {SPCR |= (1<<CPHA); PORT_SS_HV &= ~(1<<SS_HV);} //HV
	if(select==SELECT_BIAS) {SPCR |= (1<<CPHA); PORT_SS_BIAS &= ~(1<<SS_BIAS);} //BIAS
	
	for(int II =0; II < nbytes; II++)
	{
		// Save
		REGISTER[memory_SPI_TX] = (REGISTER[memory_SPI_TX] << 8) | data[II];
		// Transfer byte
		/* Start transmission */
		SPDR = data[II];
		/* Wait for transmission complete */
		uint32_t counter = 0;
		while((!(SPSR & (1<<SPIF)))  && (counter++ < 1000000)) _delay_us(1);
		if (counter == 1000000){
			// Enable interrupts
			sei();
			
			return SPI_TIMEOUT;
		}
	}

	// End the transmission. Put SS line high
	PORT_SS_PICO |= (1<<SS_PICO);
	PORT_SS_HV |= (1<<SS_HV);
	PORT_SS_BIAS |= (1<<SS_BIAS);

	// Enable interrupts
	sei();

	return OK;
}
