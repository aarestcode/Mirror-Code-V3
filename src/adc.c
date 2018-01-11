/***************************************************************************//**
 * @file	adc.c
 * @brief	Source file to operate the Analog to Digital Converter
 *
 * This file contains all the implementations for the functions defined in:
 * inc/adc.h
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/

#ifndef F_CPU
#define F_CPU 8000000UL // CPU Frequency (IMPORTANT)
#endif

#include "adc.h"
#include "memory.h"
#include <avr/interrupt.h>
#include <avr/io.h> //General I/O
#include <util/delay.h> //Delay functions

#ifndef OK
#define OK 0
#endif

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Initialize the ADC
 *
 * @param [in] F_ADC
 *	Frequency of the ADC
 ******************************************************************************/
int ADC_INIT(uint32_t F_ADC){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + ADC_INIT_Code;
		
	// CLOCK RATE (between 5kHz and 200kHz for good behavior)
	REGISTER[memory_ADC_FREQ] = F_ADC;
	if (F_ADC < 5000) return ADC_CLOCK_LOW;
	if (F_ADC > 200000) return ADC_CLOCK_HIGH;
	
	// Selection Register (AREF as voltage)
	ADMUX = 0;
	
	// Clock prescaler
	int prescaler = ceil(log(F_CPU/F_ADC)/log(2)); //ceil to unsure frequency less then F_ADC
	if(prescaler>7 || prescaler<0) return ADC_CLOCK_OOB;
	
	// Control and status register A - Enable ADC, No trigger, No Interrupt + set clock
	ADCSRA = (1<<ADEN) | prescaler;
	
	// Disable digital input (Reduce power consumption)
	DIDR0 = (1<<PICOMOTOR_VOLTAGE) | (1<<HV_GROUND) | (1<<HV_VOLTAGE) | (1<<BUS_CURR) | (1<<BUS_VOLT);
	
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Read a line from the ADC
 *
 * @param [in] line
 *	Line (pin of port A) to measure
 * @param [out] data
 *	output image of the read voltage (V = 3.3/1024*data)
 ******************************************************************************/
int ADC_READ(uint8_t line, uint16_t* data){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + ADC_READ_Code;
	
	// Disable interrupts
	cli();
	
	// Input channel selection
	ADMUX |= line;
	
	// Start conversion
	ADCSRA |= (1<<ADSC);
	
	// Wait for conversion to be done
	uint32_t counter = 0;
	while((ADCSRA & (1<<ADIF)) && (counter < 1000000)) {counter ++; _delay_us(1);}
	if (counter == 1000000){
		// Enable interrupts
		sei();
		
		return ADC_TIMEOUT;
	}
	
	// Extract data
	*data = (uint16_t)ADCL;
	*data |= (ADCH<<8);
	
	REGISTER[memory_ADC_RX] = (REGISTER[memory_ADC_RX] << 16) | (*data);
	
	// Reset channel selection
	ADMUX &= ~(line);
	
	// Enable interrupts
	sei();
	
	return OK;
}
