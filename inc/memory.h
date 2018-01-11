/***************************************************************************//**
 * @file	memory.h
 * @brief	Header file to operate the internal EEPROM of the MCU
 *
 * This header file contains all the required definitions and function prototypes
 * through which to control the internal EEPROM of the MCU
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/
#ifndef MEMORY_H_
#define MEMORY_H_

#include <inttypes.h>

// ADDRESSES (Code for all the variables to save)
enum memory_enum
{
	memory_PRIVATE,
	
	/* ------------ BOOTLOADER INFO -------------- */
	memory_BOOT_SAFE,			  // W/R
	memory_EEPROM_CODE_ADDR,	  // W/R
	
	/* ------------------ UART ------------------- */
	memory_UART0_BAUD,           // R
	memory_UART0_TX,              // R
	memory_UART0_RX,             // R
	memory_UART0_INDEX,			 // R
	
	memory_UART1_BAUD,           // R
	memory_UART1_TX,             // R
	memory_UART1_RX,             // R
	memory_UART1_INDEX,			 // R
	
	/* ------------------ SPI -------------------- */
	memory_SPI_FREQ,              // R
	memory_SPI_TX,                // R
	
	/* ------------------ I2C -------------------- */
	memory_I2C_FREQ,              // R
	memory_I2C_MAX_ITER,          // W/R
	memory_I2C_ITER,              // R
	memory_I2C_SLA,               // R
	memory_I2C_TX,                // R
	memory_I2C_RX,                // R
	
	/* ------------------ ADC -------------------- */
	memory_ADC_FREQ,              // R
	memory_ADC_RX,				  // R
	
	/* ----------------- COMMS ------------------- */
	memory_MESSAGE_COUNT_SPARE,    // R
	memory_MESSAGE_COUNT_XBEE,     // R
	
	memory_FEEDBACK_COUNT_SPARE,   // R
	memory_FEEDBACK_COUNT_XBEE,    // R
	
	memory_COMMUNICATION_TIMEOUT,  // R
	
	/* ----------------- POWER ------------------- */
	memory_HV,                     // R
	memory_GND,                    // R
	memory_HV_TOL_V,               // W/R 
	memory_PICOMOTOR_V_FB,         // R    
	memory_HV_BIAS_FB,             // R    
	memory_HV_VOLT_FB,             // R 
	memory_BUS_CURR_FB,            // R   
	memory_BUS_VOLT_FB,            // R  
	
	/* -------------- TEMPERATURE ---------------- */	
	memory_TEMP_PCT2075_1_LOW,	  // W/R
	memory_TEMP_PCT2075_1_HIGH,   // W/R
	memory_TEMP_PCT2075_2_LOW,	  // W/R
	memory_TEMP_PCT2075_2_HIGH,   // W/R
	memory_TEMP_TMP006_1_LOW,     // W/R
	memory_TEMP_TMP006_1_HIGH,    // W/R
	memory_TEMP_TMP006_2_LOW,     // W/R
	memory_TEMP_TMP006_2_HIGH,    // W/R
	memory_TEMP_TMP006_3_LOW,     // W/R
	memory_TEMP_TMP006_3_HIGH,    // W/R
				
	/* ----------- PICOMOTOR ACTUATION ----------- */
	memory_PICO_MAX_TICKS,		  // W/R	
	
	memory_PICO1_LOCATION,        // R   
	memory_PICO2_LOCATION,        // R 
	memory_PICO3_LOCATION,        // R
	
	memory_PICO1_INTERVALS,		  // R 
	memory_PICO2_INTERVALS,       // R
	memory_PICO3_INTERVALS,       // R
	
	memory_PICO1_TICKS,			  // R
	memory_PICO2_TICKS,			  // R
	memory_PICO3_TICKS,			  // R
	
	memory_PICO1_MEANp,            // W/R
	memory_PICO2_MEANp,            // W/R
	memory_PICO3_MEANp,            // W/R
	memory_PICO1_STDp,             // W/R
	memory_PICO2_STDp,             // W/R
	memory_PICO3_STDp,             // W/R
	
	memory_PICO1_MEANn,            // W/R
	memory_PICO2_MEANn,            // W/R
	memory_PICO3_MEANn,            // W/R
	memory_PICO1_STDn,             // W/R
	memory_PICO2_STDn,             // W/R
	memory_PICO3_STDn,             // W/R
	
	memory_PICO1_ERROR,			  // W/R
	memory_PICO2_ERROR,			  // W/R
	memory_PICO3_ERROR,			  // W/R
		
	/* ----------- ELECTODE ACTUATION ------------ */
	memory_HV_TIMER,              // W/R
	memory_ELECTRODE_LIMIT_V,     // W/R
	memory_HV_STEP,               // W/R
	memory_HV_BIAS,               // W/R
	
	memory_MUX_ACTIVE_CH,         // R
	
	memory_ELECTRODE1,            // W/R
	memory_ELECTRODE2,            // W/R
	memory_ELECTRODE3,            // W/R
	memory_ELECTRODE4,            // W/R
	memory_ELECTRODE5,            // W/R
	memory_ELECTRODE6,            // W/R
	memory_ELECTRODE7,            // W/R
	memory_ELECTRODE8,            // W/R
	memory_ELECTRODE9,            // W/R
	memory_ELECTRODE10,           // W/R
	memory_ELECTRODE11,           // W/R
	memory_ELECTRODE12,           // W/R
	memory_ELECTRODE13,           // W/R
	memory_ELECTRODE14,           // W/R
	memory_ELECTRODE15,           // W/R
	memory_ELECTRODE16,           // W/R
	memory_ELECTRODE17,           // W/R
	memory_ELECTRODE18,           // W/R
	memory_ELECTRODE19,           // W/R
	memory_ELECTRODE20,           // W/R
	memory_ELECTRODE21,           // W/R
	memory_ELECTRODE22,           // W/R
	memory_ELECTRODE23,           // W/R
	memory_ELECTRODE24,           // W/R
	memory_ELECTRODE25,           // W/R
	memory_ELECTRODE26,           // W/R
	memory_ELECTRODE27,           // W/R
	memory_ELECTRODE28,           // W/R
	memory_ELECTRODE29,           // W/R
	memory_ELECTRODE30,           // W/R
	memory_ELECTRODE31,           // W/R
	memory_ELECTRODE32,           // W/R
	memory_ELECTRODE33,           // W/R
	memory_ELECTRODE34,           // W/R
	memory_ELECTRODE35,           // W/R
	memory_ELECTRODE36,           // W/R
	memory_ELECTRODE37,           // W/R
	memory_ELECTRODE38,           // W/R
	memory_ELECTRODE39,           // W/R
	memory_ELECTRODE40,           // W/R
	memory_ELECTRODE41,           // W/R
	
	memory_ELECTRODE_ERROR,		  // W/R
		
	/* --------------- BOARD MANAGEMENT ---------------- */
	memory_SAFE_MODE_TRIGGER_COUNTER,   // R
	memory_SAFE_MODE_TRIGGER_OVERFLOW,  // W/R
	memory_SAFE_MODE_COUNT,				// R
	memory_SAFE_MODE_LAST_TRIGGERS,		// W/R
	
	memory_HEALTH_TIMER,				// W/R
	memory_REGISTER_TIMER,				// W/R
	
	memory_HEALTH_ERROR,				// W/R
	
	/* -------------------- TIMERS --------------------- */
	memory_TIMER_SEPARATION,			// R
	memory_TIMER_UART0,					// R
	memory_TIMER_UART1,					// R
	memory_TIMER_HEATH,					// R
	memory_TIMER_REGISTER,				// R
		
	/* --------------------- CDH ----------------------- */
	memory_PARSE_ERROR,					// W/R
		
	/* ------------------- WATCHDOG -------------------- */
	memory_WATCHDOG_RESET_COUNT,		// R
	
	memory_CURRENT_FUNCTION,

	memoryCOUNT //To count the number of variables to memorize
};

// VECTORS DECLARATION
int32_t REGISTER[memoryCOUNT]; //Register vector in the RAM

// PARAMETERS
#define INT_EEPROM_MAX_ADDR 4096

// ENUM
enum int_eeprom_error{
	INT_EEPROM_OVERLOAD = 1,
};

enum int_eeprom_function{
	SaveRegister_Code = 1,
	SaveRegisterValue_Code,
	LoadRegister_Code,
	LoadRegisterValue_Code,
	ReadValueFromEEPROM_Code,
};

// FUNCTIONS
int SaveRegister(uint16_t eeprom_register);
int SaveRegisterValue(uint16_t eeprom_register, uint16_t memory_ID);
int LoadRegister(uint16_t eeprom_register);
int LoadRegisterValue(uint16_t eeprom_register, uint16_t memory_ID);
int ReadValueFromEEPROM(uint16_t eeprom_register, uint16_t memory_ID, uint32_t *value);

#endif /* MEMORY_H_ */