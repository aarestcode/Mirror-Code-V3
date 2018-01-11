/***************************************************************************//**
 * @file	seperation_device.c
 * @brief	Source file to manage the separation device
 *
 * This file contains all the implementations for the functions defined in:
 * inc/seperation_device.h
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/

#include "separation_device.h"
#include "timer.h"
#include "memory.h"
#include <avr/io.h> //General I/O

#ifndef OK
#define OK 0
#endif

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Initialize the separation device
 *
 ******************************************************************************/
void SEPARATION_DEVICE_INIT(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + SEPARATION_DEVICE_INIT_Code;
		
	// Set pin as output
	DDR_SD_TRIG |= (1<<SEP_DEV_TRIG);
	
	// Set pin to 0
	PORT_SD_TRIG &= ~(1<<SEP_DEV_TRIG);
	
	// Set detection as input
	DDR_SD_DET &= ~(1<<SEP_DEV_DET);
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Activate supply to the separation device
 *
 ******************************************************************************/
void ActivateSeparationV(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + ActivateSeparationV_Code;
	
	PORT_SD_TRIG |= (1<<SEP_DEV_TRIG);
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Deactivate supply to the separation device
 *
 ******************************************************************************/
void DeactivateSeparationV(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + DeactivateSeparationV_Code;
	
	PORT_SD_TRIG &= ~(1<<SEP_DEV_TRIG);
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Start timer to cut the vectran cable
 *
 * @param [in] timeout_ms
 *	Timeout after which the function returns
 ******************************************************************************/
void StartRelease(uint32_t timeout_ms){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + StartRelease_Code;
	
	StartTimer(TIMER_SEPARATION, timeout_ms);
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Check if release is over (cable cut or timeout)
 *
 ******************************************************************************/
bool IsReleaseOver(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + IsReleaseOver_Code;
	
	return (!IsMirrorConstrained() || IsTimeout(TIMER_SEPARATION));
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Check the limit switch to know if the vectran cable is cut
 *
 ******************************************************************************/
bool IsMirrorConstrained(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + IsMirrorConstrained_Code;
	
	// TODO: IsMirrorConstrained
	return 1;// (PIN_SD_DET & (1<<SEP_DEV_DET));
}