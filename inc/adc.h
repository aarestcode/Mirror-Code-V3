/***************************************************************************//**
 * @file	adc.h
 * @brief	Header file to operate the Analog to Digital Converter (ADC)
 *
 * This header file contains all the required definitions and function prototypes
 * through which to control the Analog to Digital Converter (ADC)
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/
#ifndef ADC_H_
#define ADC_H_

#include <inttypes.h>

// PINSET
#define DDR_SENSORS DDRA
#define PICOMOTOR_VOLTAGE PORTA0
#define HV_GROUND PORTA1
#define HV_VOLTAGE PORTA2
#define BUS_CURR PORTA3
#define BUS_VOLT PORTA4

// ERROR ENUM
enum adc_error{	
	ADC_CLOCK_LOW = 71,
	ADC_CLOCK_HIGH,
	ADC_CLOCK_OOB,
	ADC_TIMEOUT
};

enum adc_function{
	ADC_INIT_Code = 71,
	ADC_READ_Code
};

// FUNCTIONS
int ADC_INIT(uint32_t F_ADC);
int ADC_READ(uint8_t line, uint16_t* data);



#endif /* ADC_H_ */