/***************************************************************************//**
 * @file	board_management.c
 * @brief	Source file to manage the health of the boards
 *
 * This file contains all the implementations for the functions defined in:
 * inc/board_management.h
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/

#include "board_management.h"
#include "led.h"
#include "power.h"
#include "separation_device.h"
#include "picomotor_actuation.h"
#include "temperature.h"
#include "memory.h"
#include "timer.h"
#include <stdbool.h>

#ifndef OK
#define OK 0
#endif

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Initialize mode of operation
 *
 ******************************************************************************/
void BOARD_MANAGEMENT_INIT(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + BOARD_MANAGEMENT_INIT_Code;
	
	PreviousMode = MODE_NOMINAL;
	Mode = MODE_NOMINAL;
	REGISTER[memory_SAFE_MODE_TRIGGER_COUNTER] = 0;
	REGISTER[memory_SAFE_MODE_LAST_TRIGGERS] = 0;
	REGISTER[memory_HEALTH_ERROR] = 0;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Set Mode of operation
 *
 * @param [in] _Mode
 *	Mode to be set
 ******************************************************************************/
int SetMode(Mode_t NewMode){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + SetMode_Code;
		
	// If same as current mode, return
	if (NewMode == Mode) return OK;
	
	// Cannot change mode if currently in Safe Mode
	if (Mode == SAFE_MODE) return SET_FROM_SAFE_MODE;
	
	// If set to safe mode, add a count to the register
	if (NewMode == SAFE_MODE){
		SwitchLED(true);
		REGISTER[memory_SAFE_MODE_COUNT]++;
	}

	if (Mode == MODE_NOMINAL){
		if (NewMode == MODE_SEPARATION){
			ActivateSeparationV();
		}
		if (NewMode == MODE_PICOMOTOR){
			int error = ActivatePICOV();
			if (error) return error;
		}
		if (NewMode == MODE_ELECTRODE){
			int error = ActivateHV();
			if (error) return error;
		}
	}	
	
	
	if (Mode == MODE_PICOMOTOR){
		if (NewMode == MODE_SEPARATION) return SET_SEPARATION;
		if((!IsAllPicomotorUpdateDone()) && (NewMode != SAFE_MODE)) return SET_FROM_PICOMOTOR_RUNNING;
		
		DeactivatePICOV();
		
		if (NewMode == MODE_ELECTRODE){
			int error = ActivateHV();
			if (error) return error;
		}
	}
	if (Mode == MODE_ELECTRODE){
		if (NewMode == MODE_SEPARATION) return SET_SEPARATION;
		
		int error = DeactivateHV();
		if (error) return error;
		
		if (NewMode == MODE_PICOMOTOR){
			int error = ActivatePICOV();
			if (error) return error;
		}
	}
	
	if (Mode == MODE_SEPARATION){
		if((NewMode == MODE_ELECTRODE) || (NewMode == MODE_SEPARATION)) return SET_FROM_SEPARATION;
		if((!IsReleaseOver()) && (NewMode == MODE_NOMINAL)) return SET_FROM_SEPARATION_RUNNING;
		
		DeactivateSeparationV();
	}
	
	PreviousMode = Mode;
	Mode = NewMode;
	
	// Force Health Check after this function is called
	StartTimer(TIMER_HEATH, 0);
	
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Check Health of system
 *
 ******************************************************************************/
int CheckHealth(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + CheckHealth_Code;
	
	int16_t temperature;
	int error;
	bool outOfBound;
	bool allInBounds = true;
	
	// Check temperature sensors
	for (int II = 0; II < N_PCT2075; II++){
		// Check if sensor enabled
		if (IsPCT2075Active(II)){
			
			// Measure temperature
			error = GetTemperaturePCT2075(II, &temperature);
			if (error) {
				REGISTER[memory_HEALTH_ERROR] = (REGISTER[memory_HEALTH_ERROR] << 8) + error;
				EnablePCT2075(II, false);
			}
			
			// Compare to bounds
			outOfBound = false;
			if ((temperature < (int16_t)REGISTER[memory_TEMP_PCT2075_1_LOW+2*II]) || (temperature > (int16_t)REGISTER[memory_TEMP_PCT2075_1_HIGH+2*II])) outOfBound = true; //TODO: Range of temperature
			
			// If Not Safe Mode and out of bounds, increment
			if ((Mode != SAFE_MODE) && outOfBound){
				REGISTER[memory_SAFE_MODE_LAST_TRIGGERS] = (REGISTER[memory_SAFE_MODE_LAST_TRIGGERS] << 4) + PCT2075_1_OOB + II;
				REGISTER[memory_SAFE_MODE_TRIGGER_COUNTER]++;
			}
			
			// Increment boolean and all temperatures
			allInBounds &= !(outOfBound);
		}
	}
	
	for (int II = 0; II < N_TMP006; II++){
		// Check if sensor enabled
		if (IsTMP006Active(II)){
			
			// Measure temperature
			error = GetTemperatureTMP006(II, &temperature);
			if (error) {
				REGISTER[memory_HEALTH_ERROR] = (REGISTER[memory_HEALTH_ERROR] << 8) + error;
				EnableTMP006(II, false);
			}
			
			// Compare to bounds
			outOfBound = false;
			if ((temperature < (int16_t)REGISTER[memory_TEMP_TMP006_1_LOW+2*II]) || (temperature > (int16_t)REGISTER[memory_TEMP_TMP006_1_HIGH+2*II])) outOfBound = true; //TODO: Range of temperature
			
			// If Not Safe Mode and out of bounds, increment
			if ((Mode != SAFE_MODE) && outOfBound){
				REGISTER[memory_SAFE_MODE_LAST_TRIGGERS] = (REGISTER[memory_SAFE_MODE_LAST_TRIGGERS] << 4) + TMP006_1_OOB + II;
				REGISTER[memory_SAFE_MODE_TRIGGER_COUNTER]++;
			}
			
			// Increment boolean and all temperatures
			allInBounds &= !(outOfBound);
		}
	}	
	
	// If SAFE_MODE and all temperature in bounds, go back to nominal mode
	if (Mode == SAFE_MODE){
		if (allInBounds){
			REGISTER[memory_SAFE_MODE_TRIGGER_COUNTER] = 0;
			Mode = MODE_NOMINAL;
			SwitchLED(false);
		}
		return OK;
	}
	
	if (Mode == MODE_PICOMOTOR) {
		// Check feedback voltage
		/*if (IsFeedbackVOOB(FEEDBACK_VOLTAGE_PICOMOTOR_VOLTAGE, TWO_FIVE_V, 50)){
			REGISTER[memory_SAFE_MODE_LAST_TRIGGERS] = (REGISTER[memory_SAFE_MODE_LAST_TRIGGERS] << 4) + PICO_OOB;
			REGISTER[memory_SAFE_MODE_TRIGGER_COUNTER]++;
		}	
		
		// Check Current limiter
		if (IsCLFault(CURRENT_LIMITER_1)){
			REGISTER[memory_SAFE_MODE_LAST_TRIGGERS] = (REGISTER[memory_SAFE_MODE_LAST_TRIGGERS] << 4) + CL1_OOB;
			REGISTER[memory_SAFE_MODE_TRIGGER_COUNTER] = REGISTER[memory_SAFE_MODE_TRIGGER_OVERFLOW] + 1; // Force safe mode
		}*/
	}
	
	if (Mode == MODE_ELECTRODE) {
		// Check feedback voltage
		if (IsFeedbackVOOB(FEEDBACK_VOLTAGE_HV_VOLTAGE, TWO_FIVE_V, 50)){
			REGISTER[memory_SAFE_MODE_LAST_TRIGGERS] = (REGISTER[memory_SAFE_MODE_LAST_TRIGGERS] << 4) + VARIABLE_OOB;
			REGISTER[memory_SAFE_MODE_TRIGGER_COUNTER]++;
		}
		
		if (IsFeedbackVOOB(FEEDBACK_VOLTAGE_HV_GROUND, TWO_FIVE_V, 50)){
			REGISTER[memory_SAFE_MODE_LAST_TRIGGERS] = (REGISTER[memory_SAFE_MODE_LAST_TRIGGERS] << 4) + BIAS_OOB;
			REGISTER[memory_SAFE_MODE_TRIGGER_COUNTER]++;
		}
		
		// Check Current limiter
		if (IsCLFault(CURRENT_LIMITER_2)){
			REGISTER[memory_SAFE_MODE_LAST_TRIGGERS] = (REGISTER[memory_SAFE_MODE_LAST_TRIGGERS] << 4) + CL2_OOB;
			REGISTER[memory_SAFE_MODE_TRIGGER_COUNTER] = REGISTER[memory_SAFE_MODE_TRIGGER_OVERFLOW] + 1; // Force safe mode
		}
		
		if (IsCLFault(CURRENT_LIMITER_3)){
			REGISTER[memory_SAFE_MODE_LAST_TRIGGERS] = (REGISTER[memory_SAFE_MODE_LAST_TRIGGERS] << 4) + CL3_OOB;
			REGISTER[memory_SAFE_MODE_TRIGGER_COUNTER] = REGISTER[memory_SAFE_MODE_TRIGGER_OVERFLOW] + 1; // Force safe mode
		}
	}
	
	// Check safe mode counter
	if (REGISTER[memory_SAFE_MODE_TRIGGER_COUNTER] > REGISTER[memory_SAFE_MODE_TRIGGER_OVERFLOW]) {
		return SetMode(SAFE_MODE);
	}

	return OK;
	
}
