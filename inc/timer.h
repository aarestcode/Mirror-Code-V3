/***************************************************************************//**
 * @file	timer.h
 * @brief	Header file to operate the Timer 1
 *
 * This header file contains all the required definitions and function prototypes
 * through which to control the Timer 1
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/
#ifndef TIMER_H_
#define TIMER_H_

#include <inttypes.h>
#include <stdbool.h>

typedef enum timer_ID{
	TIMER_SEPARATION,
	TIMER_UART0,
	TIMER_UART1,
	TIMER_HEATH,
	TIMER_REGISTER,
} timer_t;

enum timer_function{
	TIMER_INIT_Code = 221,
	StartTimer_Code,
	IsTimeout_Code
};
	
void TIMER_INIT(void);
void StartTimer(timer_t timerID, uint32_t timeout_ms);
bool IsTimeout(timer_t timerID);

#endif /* TIMER_H_ */