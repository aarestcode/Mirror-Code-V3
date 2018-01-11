/***************************************************************************//**
 * @file	watchdog.h
 * @brief	Header file to manage the watchdog timer
 *
 * This header file contains all the required definitions and function prototypes
 * through which to manage the watchdog timer
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/
#ifndef WATCHDOG_H_
#define WATCHDOG_H_

enum watchdog_function{
	WATCHDOG_INIT_Code = 231,
	resetSystem_Code,
	resetWatchdogTimer_Code,
	DisableWatchdogTimer_Code
};

void WATCHDOG_INIT(void);
void resetSystem(void);
void resetWatchdogTimer(void);
void DisableWatchdogTimer(void);

#endif /* WATCHDOG_H_ */