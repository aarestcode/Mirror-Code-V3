/***************************************************************************//**
 * @file	led.c
 * @brief	Source file to operate the code LED
 *
 * This file contains all the implementations for the functions defined in:
 * inc/led.h
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/

#include "led.h"
#include "memory.h"
#include <avr/io.h> //General I/O

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Initialize code LED
 *
 ******************************************************************************/
void LED_INIT(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + LED_INIT_Code;
	
	DDR_LED |= (1<<LED); // LED as output
	PORT_LED &= ~(1<<LED); // LED set to 0
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Set the state of the LED
 *
 * @param [in] state
 *	state of the LED: true = ON, false = OFF
 ******************************************************************************/
void SwitchLED(bool state){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + SwitchLED_Code;
	
	if (state) PORT_LED |= (1<<LED);
	else PORT_LED &= ~(1<<LED);
}