/***************************************************************************//**
 * @file	temperature.c
 * @brief	Source file to manage the temperature sensors
 *
 * This file contains all the implementations for the functions defined in:
 * inc/temperature.h
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/

#include "temperature.h"
#include "memory.h"
#include "i2c.h"
#include <math.h>

#ifndef OK
#define OK 0
#endif

// ADDRESSES OF SENSORS
const uint8_t PCT2075addr[N_PCT2075] = {0x9E, 0x90};
const uint8_t TMP006addr[N_TMP006] = {0x80, 0x82, 0x8A};
	
bool TEMP_PCT2075_ACTIVE[N_PCT2075];
bool TEMP_TMP006_ACTIVE[N_TMP006];

// PARAMETERS
const double S0[3] = {0.00000000000006, 0.00000000000006, 0.00000000000006}; // TODO: Calibrate S0

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Initialize temperature sensors
 *
 ******************************************************************************/
int TEMP_SENSORS_INIT(int index){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + TEMP_SENSORS_INIT_Code;
	
	
	int status = OK;
	int error;
	
	if(index == 0){
		// PCT2075
		for(int II = 0; II < N_PCT2075; II++){
			error = I2C_WRITE(PCT2075addr[II], (uint8_t [2]){0x01, 0b00000000}, 2); // Normal reading mode
			if (error) status = error;
			error = I2C_WRITE(PCT2075addr[II], (uint8_t [2]){0x04, 0b00000001}, 2); // Period to measure temperature = 100 ms
			if (error) status = error;
			
			if (error) EnablePCT2075(II, false);
			else EnablePCT2075(II, true);
		}
		
		// TMP006
		for(int II = 0; II < N_TMP006; II++){
			error = I2C_WRITE(TMP006addr[II], (uint8_t [3]){0x02, 0x74, 0}, 3);
			if (error) status = error;
			
			if (error) EnableTMP006(II, false);
			else EnableTMP006(II, true);
		}
	}
	else if (index - 1 < N_PCT2075){
		index -= 1;
		
		error = I2C_WRITE(PCT2075addr[index], (uint8_t [2]){0x01, 0b00000000}, 2); // Normal reading mode
		if (error) status = error;
		error = I2C_WRITE(PCT2075addr[index], (uint8_t [2]){0x04, 0b00000001}, 2); // Period to measure temperature = 100 ms
		if (error) status = error;
		
		if (error) EnablePCT2075(index, false);
		else EnablePCT2075(index, true);
	}
	else if (index - 1 < N_PCT2075 + N_TMP006){
		index -= (1 + N_PCT2075);
		
		error = I2C_WRITE(TMP006addr[index], (uint8_t [3]){0x02, 0x74, 0}, 3);
		if (error) status = error;
		
		if (error) EnableTMP006(index, false);
		else EnableTMP006(index, true);
	}
	
	
	
	return status;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Check if PCT2075 sensor is active
 *
 * @param [in] sensor_index
 *	Index of the sensor
 ******************************************************************************/
bool IsPCT2075Active(int sensor_index){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + IsPCT2075Active_Code;
	
	// Check index
	if(sensor_index < N_PCT2075) return TEMP_PCT2075_ACTIVE[sensor_index];
	else return false;	
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Enable/Disable a PCT2075 sensor
 *
 * @param [in] sensor_index
 *	Index of the sensor
 * @param [in] state
 *	true = active, false = inactive
 ******************************************************************************/
void EnablePCT2075(int sensor_index, bool state){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + EnablePCT2075_Code;
	
	if(sensor_index < N_PCT2075) TEMP_PCT2075_ACTIVE[sensor_index] = state;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Measure temperature of PCT2075 sensors
 *
 * @param [in] sensor_index
 *	Index of the sensor
 * @param [out] temperature_128
 *	Measured temperature (T = temperature_128/128 [C])
 ******************************************************************************/
int GetTemperaturePCT2075(int sensor_index, int16_t * temperature_128){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + GetTemperaturePCT2075_Code;
	
	
	// Check index
	if(sensor_index >= N_PCT2075) return TEMP_SENSORS_INDEX_OOB;
	
	uint8_t read_data[2];
	
	int status = I2C_READ(PCT2075addr[sensor_index], (uint8_t [1]){0}, 1, read_data, 2);
	if(status) return status;
	
	int16_t temp_256 = (read_data[0] << 8) + read_data[1];
	
	*temperature_128 = temp_256/2;
	
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Check if TMP006 sensor is active
 *
 * @param [in] sensor_index
 *	Index of the sensor
 ******************************************************************************/
bool IsTMP006Active(int sensor_index){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + IsTMP006Active_Code;
	
	// Check index
	if(sensor_index < N_TMP006) return TEMP_TMP006_ACTIVE[sensor_index];
	else return false;	
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Enable/Disable a TMP006 sensor
 *
 * @param [in] sensor_index
 *	Index of the sensor
 * @param [in] state
 *	true = active, false = inactive
 ******************************************************************************/
void EnableTMP006(int sensor_index, bool state){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + EnableTMP006_Code;
	
	if(sensor_index < N_TMP006) TEMP_TMP006_ACTIVE[sensor_index] = state;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Measure temperature of TMP006 sensors
 *
 * @param [in] sensor_index
 *	Index of the sensor
 * @param [out] temperature_128
 *	Measured temperature (T = temperature_128/128 [C])
 ******************************************************************************/
int GetTemperatureTMP006(int sensor_index, int16_t * temperature_128){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + GetTemperatureTMP006_Code;
	
	
	// Check index
	if(sensor_index >= N_TMP006) return TEMP_SENSORS_INDEX_OOB;
	
	uint8_t read_data[2];
	
	// Extract T_DIE
	int status = I2C_READ(TMP006addr[sensor_index], (uint8_t [1]){0x01}, 1, read_data, 2);
	if(status) return status;
	
	int16_t data = (read_data[0]<<8) + read_data[1];
	double T_DIE = data / 128.0 + 273.15; // in Kelvin
	
	// EXTRACT V_SENSOR
	status = I2C_READ(TMP006addr[sensor_index], (uint8_t [1]){0x00}, 1, read_data, 2);
	if(status) return status;
	
	data = (read_data[0]<<8) + read_data[1];
	double V_SENSOR = data * 0.00000015625; // in Volt
	
	// Temperature calculation
	double T_REF = 298.15; // in Kelvin
	double S = S0[sensor_index]*( 1 + 0.00175*( T_DIE - T_REF ) - 0.00001678*pow(T_DIE - T_REF,2));
	double V_OS = -0.0000294 - 0.00000057*(T_DIE - T_REF) + 0.00000000463*pow(T_DIE - T_REF,2);
	double f = (V_SENSOR - V_OS) + 13.4*pow(V_SENSOR - V_OS,2);
	double T_OBJ = pow(pow(T_DIE,4) + (f/S),0.25) - 273.15; // in Celsius
	
	*temperature_128 = T_OBJ*128;
	
	return OK;
}