/***************************************************************************//**
 * @file	seperation_device.h
 * @brief	Header file to manage the separation device
 *
 * This header file contains all the required definitions and function prototypes
 * through which to manage the separation device
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/
#ifndef SEPARATION_DEVICE_H_
#define SEPARATION_DEVICE_H_

#include <inttypes.h>
#include <stdbool.h>

// PINSET
#define DDR_SD_TRIG DDRB
#define PORT_SD_TRIG PORTB
#define SEP_DEV_TRIG PORTB3

#define DDR_SD_DET DDRC
#define PIN_SD_DET PINC
#define SEP_DEV_DET PORTC7

enum separation_device_function{
	SEPARATION_DEVICE_INIT_Code = 211,
	ActivateSeparationV_Code,
	DeactivateSeparationV_Code,
	StartRelease_Code,
	IsReleaseOver_Code,
	IsMirrorConstrained_Code
};

// FUNCTIONS
void SEPARATION_DEVICE_INIT(void);
void ActivateSeparationV(void);
void DeactivateSeparationV(void);
void StartRelease(uint32_t timeout_ms);
bool IsReleaseOver(void);
bool IsMirrorConstrained(void);

#endif /* SEPARATION_DEVICE_H_ */