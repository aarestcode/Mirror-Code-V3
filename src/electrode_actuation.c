/***************************************************************************//**
 * @file	electrode actuation.c
 * @brief	Source file to manage the electrodes
 *
 * This file contains all the implementations for the functions defined in:
 * inc/electrode actuation.h
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/

#ifndef F_CPU
#define F_CPU 8000000UL // CPU Frequency (IMPORTANT)
#endif

#include "electrode_actuation.h"
#include "i2c.h"
#include "power.h"
#include "memory.h"
#include <math.h>
#define __DELAY_BACKWARD_COMPATIBLE__ //To use variables in delay functions
#include <util/delay.h> //Delay functions

#ifndef OK
#define OK 0
#endif


// ADRESSES OF I/O EXPANDERS
uint8_t MPaddr[2] = {0x98,0x84}; // Multiplexer I2C addresses (connected to SCL, SDA, V+). LSB is irrelevant (7-bit address in bit 7 to bit 1)

// ADRESSES OF SWITCHES
const bool MPIC[42] =   {   1,   1,   0,   0,   0,   0,   1,   0,   0,   1,   1,   0,   1,   1,   0,   1,   0,   0,   0,   0,   1,   0,   1,   0,   1,   0,   0,   0,   0,   1,   0,   1,   0,   0,   1,   0,   1,   0,   1,   0,   1,   1}; // I/O expander
uint8_t MPport[42]  =     {0x3F,0x3E,0x28,0x3E,0x39,0x38,0x34,0x2D,0x2F,0x39,0x3D,0x3C,0x30,0x2D,0x24,0x3A,0x2A,0x29,0x30,0x3D,0x2F,0x26,0x31,0x33,0x38,0x2E,0x2B,0x3A,0x25,0x33,0x3B,0x37,0x2C,0x31,0x3B,0x27,0x2E,0x3F,0x2C,0x32,0x3C,0x32};

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Initialize the electrodes
 *
 ******************************************************************************/
int ELECTRODE_ACTUATION_INIT(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + ELECTRODE_ACTUATION_INIT_Code;
	
	// TODO: Delete next 4 lines (set in register directly)
	REGISTER[memory_HV_TIMER] = 15; // Time for HV to stabilize [ms]
	REGISTER[memory_ELECTRODE_LIMIT_V] = 8088; // Limit (plus/minus) from bias
	REGISTER[memory_HV_STEP] = 337; // 10V steps
	REGISTER[memory_HV_BIAS] = 8191; // 8191 = +240V Bias
	
	REGISTER[memory_ELECTRODE_ERROR] = 0;
	
	
	// Integer to return
	int status;
	
	for (int i=0; i<2; i++){
		
		// Set mode to normal
		status = I2C_WRITE(MPaddr[i],(uint8_t [2]){0x04, 0x01},2); //Send message
		if(status) return status;
		
		// Set global current to 10.5mA for 0x06, 12mA for 0x07
		status = I2C_WRITE(MPaddr[i], (uint8_t [2]){0x02, 0x06},2); //Send message
		if(status) return status;
		
		// Set all pins to LED segment driver configuration (LED = switch)
		for (uint8_t cmd = 0x09; cmd <= 0x0F; cmd++){
			status = I2C_WRITE(MPaddr[i], (uint8_t [2]){cmd, 0},2); //Send message
			if(status) return status;
		}
		
		// Disable nonexistent ports on smaller multiplexer
		if (i%2==1)	{
			status = I2C_WRITE(MPaddr[i], (uint8_t [2]){0x09, 0x55},2); //Send message
			if(status) return status;
			status = I2C_WRITE(MPaddr[i], (uint8_t [2]){0x0A, 0x55},2); //Send message
			if(status) return status;
		}
		
		// Set all output values to 0
		for (uint8_t cmd = 0x44; cmd <= 0x5c; cmd += 8)	{
			status = I2C_WRITE(MPaddr[i], (uint8_t [2]){cmd, 0},2); //Send message
			if(status) return status;
		}
		
	}
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Connect electrode to HV power supply
 *
 * @param [in] ch
 *	Index of the electrode to connect
 ******************************************************************************/
int ChannelOn(uint8_t ch){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + ChannelOn_Code;
		
	if (ch > N_electrodes) return ELECTRODE_ID_OOB;
	int status = I2C_WRITE(MPaddr[MPIC[ch]], (uint8_t [2]){MPport[ch], 1},2); //Send message
	if(!status) REGISTER[memory_MUX_ACTIVE_CH] = ch;
	return status;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Disconnect electrode from HV power supply
 *
 * @param [in] ch
 *	Index of the electrode to connect
 ******************************************************************************/
int ChannelOff(uint8_t ch){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + ChannelOff_Code;
	
	if (ch > N_electrodes) return ELECTRODE_ID_OOB;
	int status = I2C_WRITE(MPaddr[MPIC[ch]], (uint8_t [2]){MPport[ch], 0},2); //Send message
	if(!status) REGISTER[memory_MUX_ACTIVE_CH] = -1;
	return status;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Start electrode actuation algorithm
 *
 ******************************************************************************/
int StartElectrodeActuation(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + StartElectrodeActuation_Code;
		
	int error;
	long volt;
	
	// Retrieve voltages of all electrodes from REGISTER
	
	// Initialize voltages for all electrodes (+ delays of 10ms)
	int N_increments = floor((double)(0x3fff - REGISTER[memory_HV_BIAS])/(double)REGISTER[memory_HV_STEP]);
	for(int II=0; II<N_increments; II++){
		
		volt = 0x3fff - II*REGISTER[memory_HV_STEP];
		
		error = SetBias(volt);
		if(error) return error;
		
		error = SetVoltage(volt);
		if(error) return error;
		
		_delay_ms(REGISTER[memory_HV_TIMER]);
		
		for (int ch=0; ch < N_electrodes; ch++){
			error = ChannelOn(ch);  // Start charging channel
			
			_delay_ms(10);
			
			error = ChannelOff(ch);  // Start charging channel
		}
		if(error) return error;
	}
	
	error = SetBias(REGISTER[memory_HV_BIAS]);
	if(error) return error;
	
	error = SetVoltage(REGISTER[memory_HV_BIAS]);
	if(error) return error;
	
	_delay_ms(REGISTER[memory_HV_TIMER]);
	
	for (int ch=0; ch < N_electrodes; ch++){
		error = ChannelOn(ch);  // Start charging channel
		
		_delay_ms(10);
		
		error = ChannelOff(ch);  // Start charging channel
	}
	if(error) return error;
	
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Actuate an electrode
 *
 * @param [in] channel
 *	Index of the electrode to connect
 ******************************************************************************/
int ActuateElectrode(int channel){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + ActuateElectrode_Code;
	
	if (channel > N_electrodes) return ELECTRODE_ID_OOB;
	
	unsigned int memory_address = memory_ELECTRODE1 + channel;
	if ( REGISTER[memory_address] == 0 ) return OK;
	
	int status;
	
	// 1. Check voltage
	uint16_t limit = (uint16_t)REGISTER[memory_ELECTRODE_LIMIT_V];
	uint16_t bias = (uint16_t)REGISTER[memory_HV_BIAS];
	uint16_t voltage = REGISTER[memory_address] & 0xffff;
	if(voltage > bias+limit) {
		voltage = bias+limit;
		REGISTER[memory_address] = (REGISTER[memory_address] & 0xffff0000) | voltage;
	}
	if(voltage < bias-limit) {
		voltage = bias-limit;
		REGISTER[memory_address] = (REGISTER[memory_address] & 0xffff0000) | voltage;
	}
	
	// 2. Set desired voltage
	if (voltage != REGISTER[memory_HV]){
		status = SetVoltage(voltage);  // Set DAC value
		if(status)	return status;
		_delay_ms(REGISTER[memory_HV_TIMER]);
	}
	
	// 3. Turn channel on
	status = ChannelOn(channel);  // Start charging channel
	if(status) return status;
	
	// 4. Charge electrode
	_delay_ms((REGISTER[memory_address] >> 24) & 0xff);
	
	// 5. Turn channel off
	status = ChannelOff(channel);
	if(status) return status;
	
	// 6. Update timer in electrode data
	REGISTER[memory_address] = ((REGISTER[memory_address] & 0xff0000) << 8) | (REGISTER[memory_address] & 0xffffff);
	
	return OK;
}
