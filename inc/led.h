/***************************************************************************//**
 * @file	led.h
 * @brief	Header file to operate the code LED
 *
 * This header file contains all the required definitions and function prototypes
 * through which to control the code LED
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/
#ifndef LED_H_
#define LED_H_

#include <stdbool.h>

#define DDR_LED DDRD
#define PORT_LED PORTD
#define LED PORTD7

enum led_function{
	LED_INIT_Code = 161,
	SwitchLED_Code
};

void LED_INIT(void);
void SwitchLED(bool state);

#endif /* LED_H_ */