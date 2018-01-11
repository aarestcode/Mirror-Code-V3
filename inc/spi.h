/***************************************************************************//**
 * @file	spi.h
 * @brief	Header file to operate the SPI interface
 *
 * This header file contains all the required definitions and function prototypes
 * through which to control the SPI interface
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/
#ifndef SPI_H_
#define SPI_H_

#include <inttypes.h>

// PINSET
#define DDR_SPI DDRB
#define MOSI_SPI PORTB5
#define SCK_SPI PORTB7

#define DDR_SS_PICO DDRB
#define PORT_SS_PICO PORTB
#define SS_PICO PORTB4 // PICO I/O EXPANDERS

#define DDR_SS_HV DDRD
#define PORT_SS_HV PORTD
#define SS_HV PORTD5 // HV

#define DDR_SS_BIAS DDRD
#define PORT_SS_BIAS PORTD
#define SS_BIAS PORTD4 // HV BIAS

typedef enum {
	SELECT_PICO,
	SELECT_HV,
	SELECT_BIAS
} SPI_select_t;

// ERROR ENUM
enum spi_error{
	SPI_CLOCK_OOB = 41,
	SPI_TIMEOUT
};

enum spi_function{
	SPI_INIT_Code = 41,
	SPI_WRITE_Code
};

// FUNCTIONS
int SPI_INIT(uint32_t F_SPI);
int SPI_WRITE(SPI_select_t select, uint8_t * data, uint16_t nbytes);


#endif /* SPI_H_ */