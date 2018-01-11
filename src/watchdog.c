/***************************************************************************//**
 * @file	watchdog.c
 * @brief	Source file to manage the temperature sensors
 *
 * This file contains all the implementations for the functions defined in:
 * inc/temperature.h
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/

#include "watchdog.h"
#include "avr/wdt.h"
#include "avr/interrupt.h"
#include "memory.h"

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Initialize watchdog timer
 *
 ******************************************************************************/
void WATCHDOG_INIT(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + WATCHDOG_INIT_Code;
	
	cli(); // Disable interrupts
	
	// Start timed sequence
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	
	// Set configuration to "Interrupt + System Reset", Timer to 8s
	WDTCSR = (1<<WDIE) | (1<<WDE) | (1<<WDP3) | (1<<WDP0);
	
	sei(); // Enable interrupts
}


void resetSystem(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + resetSystem_Code;
	
	// Change timeout
	cli(); // Disable interrupts
	
	// Start timed sequence
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	
	// Set configuration to "Interrupt + System Reset", Timer to 16ms
	WDTCSR = (1<<WDIE) | (1<<WDE);
	
	sei();
	
	// Infinite loop
	for(;;);
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Reset watchdog timer
 *
 ******************************************************************************/
void resetWatchdogTimer(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + resetWatchdogTimer_Code;
	
	wdt_reset();
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Stop watchdog timer
 *
 ******************************************************************************/
void DisableWatchdogTimer(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + DisableWatchdogTimer_Code;
	
	cli();
	wdt_reset();
	/* Clear WDRF in MCUSR */
	MCUSR &= ~(1<<WDRF);
	/* Write logical one to WDCE and WDE */
	/* Keep old prescaler setting to prevent unintentional time-out */
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	/* Turn off WDT */
	WDTCSR = 0x00;
	sei();
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Interrupt upon watchdog timer reset
 *
 ******************************************************************************/
ISR(WDT_vect){	
	REGISTER[memory_WATCHDOG_RESET_COUNT]++;
	SaveRegister(0);
	SaveRegister(1);
	
	for(;;);
}