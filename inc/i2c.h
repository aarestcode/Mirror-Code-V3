/***************************************************************************//**
 * @file	i2c.h
 * @brief	Header file to operate the I2C interface
 *
 * This header file contains all the required definitions and function prototypes
 * through which to control the I2C interface
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/
#ifndef I2C_H_
#define I2C_H_

#include <inttypes.h>

// PINSET
// PORTC0 = SCL
// PORTC1 = SDA

// ERROR ENUM
enum i2c_error{
	I2C_CLOCK_OOB = 51,
	I2C_TIMEOUT,
	I2C_START_ARB_LOST,
	I2C_START_CRITICAL,
	I2C_RESTART_ARB_LOST,
	I2C_RESTART_CRITICAL,
	I2C_ADDR_NACK,
	I2C_ADDR_ARB_LOST,
	I2C_ADDR_CRITICAL,
	I2C_DATA_NACK,
	I2C_DATA_ARB_LOST,
	I2C_DATA_CRITICAL,
	I2C_READ_CRITICAL
};

enum i2c_function{
	I2C_INIT_Code = 51,
	I2C_WRITE_Code,
	I2C_READ_Code
};

// FUNCTIONS
int I2C_INIT(uint32_t F_I2C);
int I2C_WRITE(uint8_t SLA, uint8_t * data, uint16_t len);
int I2C_READ(uint8_t SLA, uint8_t * data_write, uint16_t write_len, uint8_t * data_read, uint16_t read_len);

#endif /* I2C_H_ */