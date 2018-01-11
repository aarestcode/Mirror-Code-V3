/***************************************************************************//**
 * @file	temperature.h
 * @brief	Header file to manage the temperature sensors
 *
 * This header file contains all the required definitions and function prototypes
 * through which to manage the temperature sensors
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/
#ifndef TEMPERATURE_H_
#define TEMPERATURE_H_

#include <inttypes.h>
#include <stdbool.h>

#define N_PCT2075 2
#define N_TMP006 3


// ENUM
enum temp_sensors_error{
	TEMP_SENSORS_INDEX_OOB = 101,
};

enum temp_sensors_function{
	TEMP_SENSORS_INIT_Code = 101,
	IsPCT2075Active_Code,
	EnablePCT2075_Code,
	GetTemperaturePCT2075_Code,
	IsTMP006Active_Code,
	EnableTMP006_Code,
	GetTemperatureTMP006_Code
};

// FUNCTIONS
int TEMP_SENSORS_INIT(int index);

bool IsPCT2075Active(int sensor_index);
void EnablePCT2075(int sensor_index, bool state);
int GetTemperaturePCT2075(int sensor_index, int16_t * temperature_128);

bool IsTMP006Active(int sensor_index);
void EnableTMP006(int sensor_index, bool state);
int GetTemperatureTMP006(int sensor_index, int16_t * temperature_128);

#endif /* TEMPERATURE_H_ */