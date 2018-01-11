/***************************************************************************//**
 * @file	timer.c
 * @brief	Source file to operate the Timer 1
 *
 * This file contains all the implementations for the functions defined in:
 * inc/timer.h
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/

#include "timer.h"
#include <avr/io.h> //General I/O
#include <avr/interrupt.h>
#include "memory.h"
#include <math.h>

#ifndef OK
#define OK 0
#endif

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Initialize Timer 1
 *
 ******************************************************************************/
void TIMER_INIT(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + TIMER_INIT_Code;
	
	REGISTER[memory_TIMER_SEPARATION] = 0;
	REGISTER[memory_TIMER_UART0] = 0;
	REGISTER[memory_TIMER_UART1] = 0;
	REGISTER[memory_TIMER_HEATH] = 0;
	REGISTER[memory_TIMER_REGISTER] = 0;
	
	TCCR1A = 0; // CTC mode
	TCCR1B = (1 << WGM12 ) | (1 << CS11 ); // CTC mode + 1 MHz clock
	TCCR1C = 0;
	
	OCR1A = (uint16_t)9999; // 10 ms interrupt
		
	TIMSK1 = (1 << OCIE1A); // Interrupt on OCR1A
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Start timer
 *
 * @param [in] timerID
 *	ID of timer to start
 * @param [in] timeout_ms
 *	Timeout of the timer in millisecond
 ******************************************************************************/
inline void StartTimer(timer_t timerID, uint32_t timeout_ms){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + StartTimer_Code;
	
	REGISTER[memory_TIMER_SEPARATION + timerID] = round(timeout_ms/10);
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Check if timer has overflown
 *
 * @param [in] timerID
 *	ID of timer to start
 ******************************************************************************/
inline bool IsTimeout(timer_t timerID){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + IsTimeout_Code;
	
	return (REGISTER[memory_TIMER_SEPARATION + timerID] == 0);
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Timer interrupt
 *
 ******************************************************************************/
ISR(TIMER1_COMPA_vect){
	if (REGISTER[memory_TIMER_SEPARATION] > 0) REGISTER[memory_TIMER_SEPARATION]--;
	if (REGISTER[memory_TIMER_UART0] > 0) REGISTER[memory_TIMER_UART0]--;
	if (REGISTER[memory_TIMER_UART1] > 0) REGISTER[memory_TIMER_UART1]--;
	if (REGISTER[memory_TIMER_HEATH] > 0) REGISTER[memory_TIMER_HEATH]--;
	if (REGISTER[memory_TIMER_REGISTER] > 0) REGISTER[memory_TIMER_REGISTER]--;
}