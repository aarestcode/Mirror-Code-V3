/***************************************************************************//**
 * @file	code_programming.c
 * @brief	Source file to manage and program the external EEPROM
 *
 * This file contains all the implementations for the functions defined in:
 * inc/code_programming.h
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/

#include "code_programming.h"
#include "memory.h"
#include "watchdog.h"
#include <avr/pgmspace.h>
#include "i2c.h"
#include <string.h>

#ifndef OK
#define OK 0
#endif

// ADDRESSES OF EEPROM
const char EXT_EEPROM_ADDR = 0xA0;

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Read DWord from the external EEPROM
 *
 * @param [in] CodeID
 *	Slot of the EEPROM to read
 * @param [in] addr
 *	Address of the DWord to read
 * @param [out] dword
 *	Read DWord
 ******************************************************************************/
int ReadDWordFromEEPROM(CodeID_t CodeID, uint16_t addr, uint32_t * dword){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + ReadDWordFromEEPROM_Code;

	uint8_t byte_addr[2] = {(addr >> 8) + 1, addr & 0xFF};
	uint8_t bytes[4] = {};
	int status = I2C_READ(EXT_EEPROM_ADDR + (CodeID<<1), byte_addr, 2, bytes, 4);
	if(status) return status;
	
	*dword = (((uint32_t)bytes[3]) << 24) + (((uint32_t)bytes[2]) << 16) + (((uint32_t)bytes[1]) << 8) + ((uint32_t)bytes[0]);
	
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Get the size of the code in the external EEPROM
 *
 * @param [in] CodeID
 *	Slot of the EEPROM to read
 * @param [out] Npages
 *	Read number of pages
 ******************************************************************************/
int GetSizeofCode(CodeID_t CodeID, uint8_t * Npages){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + GetSizeofCode_Code;
	
	uint8_t byte_addr[2] = {0,0};
	uint8_t read_bytes[1];
	int status = I2C_READ(EXT_EEPROM_ADDR + (CodeID<<1), byte_addr, 2, read_bytes, 1);
	if(status) return status;

	*Npages = read_bytes[0];
	
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Write code info (number of pages in external EEPROM
 *
 * @param [in] CodeID
 *	Slot of the EEPROM to read
 * @param [in] Npages
 *	Number of pages
 ******************************************************************************/
int WriteCodeInfoinEEPROM(CodeID_t CodeID, uint8_t Npages){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + WriteCodeInfoinEEPROM_Code;
	
	uint8_t data[2 + 1];
	data[0] = 0; // First page = code data
	data[1] = 0;
	data[2] = Npages;
	
	int status = I2C_WRITE(EXT_EEPROM_ADDR + (CodeID<<1), data, 2 + 1);
	if(status) return status;
	
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Write a full page (256 bytes) in external EEPROM
 *
 * @param [in] CodeID
 *	Slot of the EEPROM to read
 * @param [in] page
 *	Address of the page
 * @param [in] buffer
 *	Array of bytes containing the page
 ******************************************************************************/
int WritePageInEEPROM(CodeID_t CodeID, uint8_t page, uint8_t buffer[EXT_EEPROM_PAGESIZE]){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + WritePageInEEPROM_Code;
	
	uint8_t data[2 + EXT_EEPROM_PAGESIZE];
	data[0] = page + 1; // First page = code data
	data[1] = 0;
	memcpy(&(data[2]), buffer, EXT_EEPROM_PAGESIZE);
	int status = I2C_WRITE(EXT_EEPROM_ADDR + (CodeID<<1), data, 2 + EXT_EEPROM_PAGESIZE);
	if(status) return status;
	
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Write a full code from the camera to the external EEPROM
 *
 * @param [in] port
 *	Port to send messages to
 * @param [in] CodeID
 *	Slot of the EEPROM to read
 * @param [in] Npages
 *	Number of pages
 ******************************************************************************/
int SaveApplicationFromCameraToEEPROM(port_t port, CodeID_t CodeID, uint8_t Npages){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + SaveApplicationFromCameraToEEPROM_Code;

	uint8_t buffer[SPM_PAGESIZE];
	int status;
	
	// Save size of code
	status = WriteCodeInfoinEEPROM(CodeID, Npages);
	if(status) return status;
	
	for (int II = 0; II < Npages; II++){
		resetWatchdogTimer(); // Can be a long function
		
		SendFeedback(port,245,II,0); // Trigger the camera to send a page
		
		status = LoadData(port, buffer, SPM_PAGESIZE);
		if(status) return status;
		
		resetWatchdogTimer();
		
		status = WritePageInEEPROM(CodeID, II, buffer);
		if(status) return status;
	}
	
	return OK;
}