/***************************************************************************//**
 * @file	picomotor_actuation.c
 * @brief	Source file to manage the picomotors
 *
 * This file contains all the implementations for the functions defined in:
 * inc/picomotor_actuation.h
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/
#ifndef F_CPU
#define F_CPU 8000000UL // CPU Frequency (IMPORTANT)
#endif

#include "picomotor_actuation.h"
#include "i2c.h"
#include "spi.h"
#include "memory.h"
#include "timer.h"
#include <avr/io.h> //General I/O
#define __DELAY_BACKWARD_COMPATIBLE__ //To use variables in delay functions
#include <util/delay.h> //Delay functions
#include <stdlib.h>

#ifndef OK
#define OK 0
#endif

// MCP23S17
uint8_t IOEaddr = 0x40; // Address of the IO Expander (LSB = 0 for WRITE operations)
uint8_t IOEport[3] = {0x12,0x13,0x13}; // Address of the port to which each picomotor is connected (pico1, pico2, pico3)
uint8_t IOEpin[12] = {1,0,3,2,1,0,3,2,5,4,7,6}; // Pin of each port to which the switch is connected (pico1_FW_HIGH, pico1_FW_LOW, pico1_BW_HIGH, pico1_BW_LOW, ...)

// ADG715
uint8_t ENCODERSWITCHaddr = 0x90;
uint8_t ENCODERSWITCHport[3] = {0, 2, 4};
	
// Encoder select
picomotor_t encoder_select;

// Mode of picomotor (for Update function)
typedef enum{
	picomotor_actuation,
	picomotor_initialization,
	picomotor_calibration
} picomotor_mode_t;
picomotor_mode_t picomotorMode[3];

// For calibration
int32_t picomotorSumTicks[3];
uint32_t picomotorSumTicksSquared[3];
int32_t picomotorSumIntervals[3];

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Initialize the picomotor controllers
 *
 ******************************************************************************/
int PICOMOTOR_ACTUATION_INIT(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + PICOMOTOR_ACTUATION_INIT_Code;
	
	
	// TODO: Delete next line (set in register)
	REGISTER[memory_PICO_MAX_TICKS] = 1000;
	
	REGISTER[memory_PICO1_INTERVALS] = 0;
	REGISTER[memory_PICO2_INTERVALS] = 0;
	REGISTER[memory_PICO3_INTERVALS] = 0;
	REGISTER[memory_PICO1_TICKS] = 0;
	REGISTER[memory_PICO2_TICKS] = 0;
	REGISTER[memory_PICO3_TICKS] = 0;
	
	
	REGISTER[memory_PICO1_ERROR] = 0;
	REGISTER[memory_PICO2_ERROR] = 0;
	REGISTER[memory_PICO3_ERROR] = 0;
	
	picomotorMode[0] = picomotor_actuation;
	picomotorMode[1] = picomotor_actuation;
	picomotorMode[2] = picomotor_actuation;
	
	int error;
	
	// Set Encoders as inputs
	DDR_ENCODER &= ~((1<<ENCODERA) | (1<<ENCODERB));
	
	// Set CONFIGURATION bits to 0
	error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, 0x0A, 0x00},3);
	if (error) return error;
	
	// Set PortA pins as outputs (via IODIRA)
	error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, 0x00, 0x00},3);
	if (error) return error;
	
	// Set PortB pins as outputs (via IODIRB)
	error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, 0x01, 0x00},3);
	if (error) return error;
	
	// Deactivate PortA pull-up resistors
	error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, 0x0C, 0x00},3);
	if (error) return error;
	
	// Deactivate PortB pull-up resistors
	error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, 0x0D, 0x00},3);
	if (error) return error;
	
	// Set PortA pins to 0
	error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, 0x12, 0x00},3);
	if (error) return error;
	// Set PortB pins to 0
	error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, 0x13, 0x00},3);
	if (error) return error;
		
	// Encoder
	encoder_select = -1;
	error = I2C_WRITE(ENCODERSWITCHaddr, (uint8_t [1]){3 << ENCODERSWITCHport[0]}, 1);
	if (error) return error;
	encoder_select = 0;

	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Get the encoder state
 *
 * @param [in] index
 *	Index of the picomotor
 * @param [out] state
 *	State of the encoder
 ******************************************************************************/
int GetEncoderState(picomotor_t index, encoder_state_t* state){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + GetEncoderState_Code;
	
	if( encoder_select != index ){
		int error = I2C_WRITE(ENCODERSWITCHaddr, (uint8_t [1]){3 << ENCODERSWITCHport[index]}, 1);
		if (error) return error;
		encoder_select = index;
	}
	
	if ((PIN_ENCODER & (1<<ENCODERA)) && (PIN_ENCODER & (1<<ENCODERB))) {*state = state11; return OK;}
	if ((PIN_ENCODER & (1<<ENCODERA)) && !(PIN_ENCODER & (1<<ENCODERB))) {*state = state10; return OK;}
	if (!(PIN_ENCODER & (1<<ENCODERA)) && (PIN_ENCODER & (1<<ENCODERB))) {*state = state01; return OK;}
	if (!(PIN_ENCODER & (1<<ENCODERA)) && !(PIN_ENCODER & (1<<ENCODERB))) {*state = state00; return OK;}
	
	return ENCODER_STATE_CRITICAL;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Get the encoder state
 *
 * @param [in] current_state
 *	Current measured state of the encoder
 * @param [in] prev_state
 *	Previous measured state of the encoder (before actuation)
 * @param [out] update
 *	-1 (current state is one interval down) or 0 (no change of state) or 1 (current state is one interval up)
 ******************************************************************************/
int EncoderStateMonitor(encoder_state_t current_state, encoder_state_t prev_state, int8_t* update){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + EncoderStateMonitor_Code;
	
	//Check the state values
	if(!(current_state==state00) && !(current_state==state01) && !(current_state==state10) && !(current_state==state11)) return CURRENT_STATE_OOB;
	if(!(prev_state==state00) && !(prev_state==state01) && !(prev_state==state10) && !(prev_state==state11)) return PREVIOUS_STATE_OOB;
	
	//Check for incorrect state change (abs difference in state = 2)
	if (abs(current_state-prev_state)==2) return INCORRECT_STATE_CHANGE;
	
	//Return state change
	if(current_state==prev_state) {*update = 0; return OK;}
	if(((current_state-prev_state)==1) || ((current_state-prev_state)==-3)) {*update = 1; return OK;}
	if(((current_state-prev_state)==-1) || ((current_state-prev_state)==3)) {*update = -1; return OK;}
	
	return STATE_MONITOR_CRITICAL;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Get the encoder state
 *
 * @param [in] index
 *	Index of the picomotor
 * @param [in] ticks
 *	Number of ticks to move (signed)
 ******************************************************************************/
int MovePicomotorByTicks(picomotor_t index, int32_t ticks, int32_t* movedTicks){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + MovePicomotorByTicks_Code;
	
	// INFO: @4MHz SPI clock, full message takes 37us to be sent
	//ticks < 0 <=> BACKWARD
	//ticks > 0 <=> FORWARD
	int error = 0;
	int32_t II = 0;
	
	*movedTicks = 0;
	if(ticks>0)	{
		//MOVE FORWARD
		for (II=0; II<ticks; II++){			
			error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, IOEport[index], 1 << IOEpin[4*index]},3);
			if (error) goto end;
			_delay_us(326); // The delay was shorten by 2 full message durations. The next message is happening 37us earlier than Nicolas' code
			error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, IOEport[index], 0},3);
			if (error) goto end;
			_delay_us(10); // Unchanged. Message delay accounted in the previous pause.
			error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, IOEport[index], 1 << IOEpin[4*index+1]},3);
			if (error) goto end;
			_delay_us(63); // Accounted for message delay
			error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, IOEport[index], 0},3);
			if (error) goto end;			
			_delay_us(2463); // Accounted for message delay
			
			*movedTicks += 1;
		}
		
	}
	else if(ticks<0)
	{
		//BACKWARD
		for (II=0; II>ticks; II--){			
			error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, IOEport[index], 1 << IOEpin[4*index+2]},3);
			if (error) goto end;
			_delay_us(63); // Accounted for message delay
			error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, IOEport[index], 0},3);
			if (error) goto end;
			_delay_us(63); // Accounted for message delay
			error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, IOEport[index], 1 << IOEpin[4*index+3]},3);
			if (error) goto end;
			_delay_us(363); // Accounted for message delay
			error = SPI_WRITE(SELECT_PICO,(uint8_t [3]){IOEaddr, IOEport[index], 0},3);
			if (error) goto end;
			_delay_us(2463); // Accounted for message delay
			
			*movedTicks -= 1;
		}
	}
	
	end:
	return error;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Get the encoder state
 *
 * @param [in] index
 *	Index of the picomotor
 * @param [in] intervals
 *	Number of intervals to move (signed)
 * @param [out] movedIntervals
 *	Actual number of intervals moved (should be equal to intervals)
 * @param [out] movedTicks
 *	Number of ticks moved (signed)
 ******************************************************************************/
int MovePicomotorByIntervals(picomotor_t index, int32_t intervals, int32_t* movedIntervals, int32_t* movedTicks){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + MovePicomotorByIntervals_Code;
	
	
	int error;
	
	// Get the direction
	int8_t dir;
	if( intervals > 0 ) dir = 1;
	else if( intervals < 0 ) dir = -1;
	else return OK; //No need to move
	
	// Get current state of encoders
	uint8_t prev_state, current_state;
	error = GetEncoderState(index, &current_state);
	if(error) return error;
	
	// Loop
	*movedIntervals = 0;
	*movedTicks = 0;
	int32_t movedTicks_ = 0;
	int8_t update = 0;
	for (uint32_t II=0; II < abs(intervals); II++){
		// Update previous state for next interval
		prev_state = current_state;
		// Count the number of ticks
		uint32_t ticks_count=0;
		
		// Actuate while a new state is not reached
		do{
			// Move the picomotor one tick
			error = MovePicomotorByTicks(index,dir,&movedTicks_);
			if(error) return error;
			*movedTicks += movedTicks_;
			
			// Count the step
			if( ++ticks_count > REGISTER[memory_PICO_MAX_TICKS] ) return MAX_TICKS_COUNT;
			
			// Update current state
			error = GetEncoderState(index, &current_state);
			if(error) return error;
			
			// Update the the state monitor
			error = EncoderStateMonitor(current_state, prev_state, &update);
			if(error) return error;
			
			// Check the update (good direction)
			if(update==-dir) return MOVE_INTERVALS_WRONG_DIR;
			
		} while (!update);
		
		*movedIntervals+=dir;
	}
	
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Set number of ticks in the register for the update function
 *
 * @param [in] index
 *	Index of the picomotor
 * @param [in] ticks
 *	Number of ticks to move
 ******************************************************************************/
void SetTicks(picomotor_t index, int32_t ticks){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + SetTicks_Code;
	
	REGISTER[memory_PICO1_INTERVALS+index] = 0;
	REGISTER[memory_PICO1_TICKS+index] = (int32_t)(ticks);
	picomotorMode[index] = picomotor_actuation;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Set number of intervals in the register for the update function
 *
 * @param [in] index
 *	Index of the picomotor
 * @param [in] ticks
 *	Number of ticks to move
 ******************************************************************************/
void SetIntervals(picomotor_t index, int32_t intervals){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + SetIntervals_Code;
	
	REGISTER[memory_PICO1_INTERVALS+index] = (int32_t)(intervals);
	REGISTER[memory_PICO1_TICKS+index] = 0;
	picomotorMode[index] = picomotor_actuation;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Calculate the number of intervals and ticks to move
 *
 * @param [in] index
 *	Index of the picomotor
 * @param [in] location
 *	Desired location
 * @param [out] intervals
 *	Number of intervals to move
 * @param [out] ticks
 *	Number of ticks to move after the intervals + direction of approach
 ******************************************************************************/
void SetPreciseLocation(picomotor_t index, float location){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + SetPreciseLocation_Code;
	
	// Load current position
	float current_location = (float)(REGISTER[memory_PICO1_LOCATION+index]);
	
	// Calculate differential location
	float diff_location = location - floor(current_location);
	diff_location -= (diff_location < 0); // Due to floor function
	
	// No movement
	if (diff_location == 0){
		REGISTER[memory_PICO1_INTERVALS + index] = 0;
		REGISTER[memory_PICO1_TICKS + index] = 0;
	}
	
	// Calculate number of intervals
	int32_t intervals = round(diff_location);
	REGISTER[memory_PICO1_INTERVALS + index] = (int32_t)intervals; 
	
	// Calculate number of ticks
	diff_location += intervals; // Get remainder 
	int32_t ticks;
	if (diff_location > 0){
		float mean = (float)(REGISTER[memory_PICO1_MEANp+index]);
		float std = (float)(REGISTER[memory_PICO1_STDp+index]);
		float limit = -0.0123 + 0.8731*std + 1.03*mean + 3.493*pow(std,2) - 3.289*mean*std;
		ticks = 1 + ceil((diff_location-limit)/mean);
	}
	if (diff_location < 0){
		float mean = (float)(REGISTER[memory_PICO1_MEANn+index]);
		float std = (float)(REGISTER[memory_PICO1_STDn+index]);
		float limit = 0.0123 - 0.8731*std + 1.03*mean - 3.493*pow(std,2) - 3.289*mean*std;
		ticks = -1 - ceil((diff_location-limit)/mean);
	}
	else ticks = (intervals > 0) - (intervals < 0);
	REGISTER[memory_PICO1_TICKS + index] = (int32_t)ticks; 
		
	picomotorMode[index] = picomotor_actuation;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Initialize the picomotor position
 *
 * @param [in] index
 *	Index of the picomotor
 * @param [in] max_ticks
 *	Maximum number of ticks to get to the next interval
 ******************************************************************************/
void SetInitialization(picomotor_t index, uint32_t max_ticks){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + SetInitialization_Code;
	
	REGISTER[memory_PICO1_INTERVALS+index] = 0;
	REGISTER[memory_PICO1_TICKS+index] = max_ticks;
	picomotorMode[index] = picomotor_initialization;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Calibrate the picomotor
 *
 * @param [in] index
 *	Index of the picomotor
 * @param [in] max_ticks
 *	Maximum number of ticks to get to the next interval
 ******************************************************************************/
void SetCalibration(picomotor_t index, int16_t intervals){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + SetCalibration_Code;
	
	
	// To calculate the mean and std, we need the sum and the sum of squares
	picomotorSumTicks[index] = 0;
	picomotorSumTicksSquared[index] = 0;
	picomotorSumIntervals[index] = 0;
	
	REGISTER[memory_PICO1_INTERVALS + index] = (uint32_t)(intervals);
	REGISTER[memory_PICO1_TICKS+index] = 0;
	
	picomotorMode[index] = picomotor_calibration;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Update the picomotor position based on the register values
 *
 * @param [in] index
 *	Index of the picomotor
 ******************************************************************************/
int UpdatePicomotor(picomotor_t index){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + UpdatePicomotor_Code;
	
	int error;
	
	int32_t intervals = (int32_t)(REGISTER[memory_PICO1_INTERVALS + index]);
	int8_t dir_intervals = (intervals > 0) - (intervals < 0);
	
	int32_t ticks = (int32_t)(REGISTER[memory_PICO1_TICKS + index]);
	int8_t dir_ticks = (ticks > 0) - (ticks < 0);
	
	if ((ticks == 0) && (intervals == 0)) return OK;	
	
	//----------------------------------------------------------------------
	//                        PICOMOTOR INITIALIZATION
	//----------------------------------------------------------------------
	if (picomotorMode[index] == picomotor_initialization){
		uint32_t Nticks = REGISTER[memory_PICO1_TICKS + index];
		if (Nticks > REGISTER[memory_PICO_MAX_TICKS]) Nticks = REGISTER[memory_PICO_MAX_TICKS];
		
		// Get current state of encoders
		uint8_t prev_state, current_state;
		int error = GetEncoderState(index, &prev_state);
		if(error) return error;
		
		int8_t update = 0;
		uint32_t ticks_count=0;
		int32_t movedTicks = 0;
		do
		{
			// Move the picomotor one tick
			error = MovePicomotorByTicks(index,1,&movedTicks);
			if(error) return error;
			
			// Update current state
			error = GetEncoderState(index, &current_state);
			if(error) return error;
			
			// Update the the state monitor
			error = EncoderStateMonitor(current_state, prev_state, &update);
			if(error) return error;
			
		} while ((!update) && (++ticks_count < Nticks));
		
		if (update){
			// Stop actuation
			StopActuation(index);
			
			// Update location
			float mean = (float)(REGISTER[memory_PICO1_MEANp+index]);
			float std = (float)(REGISTER[memory_PICO1_STDp+index]);
			float pos = -0.003419 + 0.4494*std + 0.4731*mean + 0.7909*pow(std,2) - 0.942*mean*std;
			REGISTER[memory_PICO1_LOCATION + index] = (uint32_t)(pos);
		}
		
		return OK;
	}
	
	//----------------------------------------------------------------------
	//                        PICOMOTOR CALIBRATION
	//----------------------------------------------------------------------
	if (picomotorMode[index] == picomotor_calibration){
		// Move by one interval
		int32_t movedIntervals, movedTicks;
		error = MovePicomotorByIntervals(index, dir_intervals, &movedIntervals, &movedTicks);
		if (error){
			// Update position if ticks moved
			float pos;
			if (movedTicks > 0){
				float mean = (float)(REGISTER[memory_PICO1_MEANp+index]);
				pos = (float)(REGISTER[memory_PICO1_LOCATION + index]) + movedTicks*mean;
			}
			else{
				float mean = (float)(REGISTER[memory_PICO1_MEANn+index]);
				pos = (float)(REGISTER[memory_PICO1_LOCATION + index]) + movedTicks*mean;
			}
			REGISTER[memory_PICO1_LOCATION + index] = (uint32_t)(pos);
			
			return error;
		}
		
		// Update the number of intervals left
		intervals -= movedIntervals;
		REGISTER[memory_PICO1_INTERVALS + index] = (uint32_t)(intervals);
		
		picomotorSumTicks[index] += movedTicks;
		picomotorSumTicksSquared[index] += pow(movedTicks,2);
		picomotorSumIntervals[index] += movedIntervals;
		
		// If done with calibration, save calibration
		if (intervals == 0){
			// From sum_ticks and sum_ticks_squared, get the mean and std
			float mean = 1./picomotorSumTicks[index]*picomotorSumIntervals[index];
			float std = sqrt(picomotorSumTicksSquared[index]/abs(picomotorSumTicks[index])-(picomotorSumTicks[index]/picomotorSumIntervals[index]))*(mean);
			
			if (dir_intervals == 1) {
				REGISTER[memory_PICO1_MEANp + index] = (uint32_t)(mean);
				REGISTER[memory_PICO1_STDp + index] = (uint32_t)(std);
			}
			else {
				REGISTER[memory_PICO1_MEANn + index] = (uint32_t)(mean);
				REGISTER[memory_PICO1_STDn + index] = (uint32_t)(std);
			}
		}
			
		// Update the position of the picomotor
		float pos;
		if (dir_intervals == 1){
			float mean = (float)(REGISTER[memory_PICO1_MEANp+index]);
			float std = (float)(REGISTER[memory_PICO1_STDp+index]);
			pos = floor(REGISTER[memory_PICO1_LOCATION + index] + 1) -0.003419 + 0.4494*std + 0.4731*mean + 0.7909*pow(std,2) - 0.942*mean*std;
		}
		else{
			float mean = (float)(REGISTER[memory_PICO1_MEANn+index]);
			float std = (float)(REGISTER[memory_PICO1_STDn+index]);
			pos = floor(REGISTER[memory_PICO1_LOCATION + index] - 1) +0.003419 - 0.4494*std + 0.4731*mean - 0.7909*pow(std,2) - 0.942*mean*std;
		}
		REGISTER[memory_PICO1_LOCATION + index] = (uint32_t)(pos);
		
		return OK;
	}
		
	//----------------------------------------------------------------------
	//                          PICOMOTOR ACTUATION
	//----------------------------------------------------------------------
	// If we need to move intervals
	if(intervals){		
		// Move by one interval
		int32_t movedIntervals, movedTicks;		
		error = MovePicomotorByIntervals(index, dir_intervals, &movedIntervals, &movedTicks);
		if (error){			
			// Update position if ticks moved
			float pos;
			if (movedTicks > 0){
				float mean = (float)(REGISTER[memory_PICO1_MEANp+index]);
				pos = (float)(REGISTER[memory_PICO1_LOCATION + index]) + movedTicks*mean;
			}
			else{
				float mean = (float)(REGISTER[memory_PICO1_MEANn+index]);
				pos = (float)(REGISTER[memory_PICO1_LOCATION + index]) + movedTicks*mean;
			}
			REGISTER[memory_PICO1_LOCATION + index] = (uint32_t)(pos);
			
			return error;
		}
		
		intervals -= movedIntervals;
				
		// If precise location, last interval must be in the direction of the ticks	
		// If not precise location dir_ticks = 0 so the if-statement is false	
		if ( (intervals == 0) && (dir_ticks == -dir_intervals) ){
			// Add one interval
			intervals = dir_ticks;
			// First tick is discarded (just to be placed in the right interval)
			ticks -= dir_ticks;
		}
		
		// Update the number of intervals left
		REGISTER[memory_PICO1_INTERVALS + index] = (uint32_t)(intervals);
		
		// Update the position of the picomotor
		float pos;
		if (dir_intervals == 1){
			float mean = (float)(REGISTER[memory_PICO1_MEANp+index]);
			float std = (float)(REGISTER[memory_PICO1_STDp+index]);
			pos = floor(REGISTER[memory_PICO1_LOCATION + index] + 1) -0.003419 + 0.4494*std + 0.4731*mean + 0.7909*pow(std,2) - 0.942*mean*std;
		}
		else{
			float mean = (float)(REGISTER[memory_PICO1_MEANn+index]);
			float std = (float)(REGISTER[memory_PICO1_STDn+index]);
			pos = floor(REGISTER[memory_PICO1_LOCATION + index] - 1) +0.003419 - 0.4494*std + 0.4731*mean - 0.7909*pow(std,2) - 0.942*mean*std;
		}		
		REGISTER[memory_PICO1_LOCATION + index] = (uint32_t)(pos);
	}
	
	else if (ticks){
		
		int Nticks = ticks;
		if (abs(ticks) > REGISTER[memory_PICO_MAX_TICKS]) Nticks = dir_ticks*REGISTER[memory_PICO_MAX_TICKS];
		
		int32_t movedTicks;		
		error = MovePicomotorByTicks(index, Nticks, &movedTicks);
		
		// Update the position of the picomotor
		float pos;
		if (dir_ticks == 1){
			float mean = (float)(REGISTER[memory_PICO1_MEANp+index]);
			pos = (float)(REGISTER[memory_PICO1_LOCATION + index]) + movedTicks*mean;
		}
		else{
			float mean = (float)(REGISTER[memory_PICO1_MEANn+index]);
			pos = (float)(REGISTER[memory_PICO1_LOCATION + index]) + movedTicks*mean;
		}
		REGISTER[memory_PICO1_LOCATION + index] = (uint32_t)(pos);
		
		if (error) return error;
				
		ticks -= Nticks;
		
		// Update the number of ticks left
		REGISTER[memory_PICO1_TICKS + index] = (uint32_t)(ticks);
	}
	
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Stop picomotor actuation
 *
 * @param [in] index
 *	Index of the picomotor
 ******************************************************************************/
void StopActuation(picomotor_t index){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + StopActuation_Code;
	
	REGISTER[memory_PICO1_INTERVALS + index] = 0;
	REGISTER[memory_PICO1_TICKS + index] = 0;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Check if all picomotors are updated
 *
 * @param [in] index
 *	Index of the picomotor
 ******************************************************************************/
bool IsPicomotorUpdateDone(picomotor_t index){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + IsPicomotorUpdateDone_Code;
	
	if ((REGISTER[memory_PICO1_INTERVALS + index] == 0) && (REGISTER[memory_PICO1_TICKS + index] == 0)) return true;
	else return false;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Check if all picomotors are updated
 *
 * @param [in] index
 *	Index of the picomotor
 ******************************************************************************/
bool IsAllPicomotorUpdateDone(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + IsAllPicomotorUpdateDone_Code;
	
	if (IsPicomotorUpdateDone(picomotor1) && IsPicomotorUpdateDone(picomotor2) && IsPicomotorUpdateDone(picomotor3)) return true;
	else return false;
}