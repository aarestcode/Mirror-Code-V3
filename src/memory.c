/***************************************************************************//**
 * @file	memory.c
 * @brief	Source file to operate the internal EERPOM of the MCU
 *
 * This file contains all the implementations for the functions defined in:
 * inc/memory.h
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/

#include "memory.h"
#include <avr/eeprom.h> // To save variables to non-volatile memory

#ifndef OK
#define OK 0
#endif

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Save the Register from the RAM to the internal EEPPROM
 *
 * @param [in] eeprom_register
 *	Slot in the EEPROM to save the vector (1, 2, 3 or 4)
 ******************************************************************************/
int SaveRegister(uint16_t eeprom_register){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + SaveRegister_Code;
	
	if(eeprom_register*memoryCOUNT*4 + memoryCOUNT*4 - 1 > INT_EEPROM_MAX_ADDR) return INT_EEPROM_OVERLOAD;
	
	/* Update the EEPROM memory with the current RAM memory */
	eeprom_update_block((const void*)REGISTER, (void*)(eeprom_register*memoryCOUNT*4), memoryCOUNT*4); //*4 because the vectors are made of 32 bits int (4 bytes)
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Save a Register value from the RAM to the internal EEPPROM
 *
 * @param [in] eeprom_register
 *	Slot in the EEPROM to save the vector (1, 2, 3 or 4)
 * @param [in] memory_ID
 *	ID of the memory to save
 ******************************************************************************/
int SaveRegisterValue(uint16_t eeprom_register, uint16_t memory_ID){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + SaveRegisterValue_Code;
	

	if(eeprom_register*memoryCOUNT*4 + memory_ID*4 + 3 > INT_EEPROM_MAX_ADDR) return INT_EEPROM_OVERLOAD;
	
	/* Update the EEPROM memory with the current RAM memory */
	eeprom_update_block((const void*)&REGISTER[memory_ID], (void*)(eeprom_register*memoryCOUNT*4 + memory_ID*4), 4); //*4 because the vectors are made of 32 bits int (4 bytes)
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Load the Register from the internal EEPPROM to the RAM
 *
 * @param [in] eeprom_register
 *	Slot in the EEPROM to save the vector (1, 2, 3 or 4)
 ******************************************************************************/
int LoadRegister(uint16_t eeprom_register){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + LoadRegister_Code;
	

	if(eeprom_register*memoryCOUNT*4 + memoryCOUNT*4 - 1 > INT_EEPROM_MAX_ADDR) return INT_EEPROM_OVERLOAD;
	
	/* Load the EEPROM memory to the RAM memory */
	eeprom_read_block((void*)REGISTER, (const void*)(eeprom_register*memoryCOUNT*4), memoryCOUNT*4); //*4 because the vectors are made of 32 bits int (4 bytes)
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Load a Register value from the internal EEPPROM to the RAM
 *
 * @param [in] eeprom_register
 *	Slot in the EEPROM to save the vector (1, 2, 3 or 4)
 * @param [in] memory_ID
 *	ID of the memory to save
 ******************************************************************************/
int LoadRegisterValue(uint16_t eeprom_register, uint16_t memory_ID){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + LoadRegisterValue_Code;
	

	if(eeprom_register*memoryCOUNT*4 + memory_ID*4 + 3 > INT_EEPROM_MAX_ADDR) return INT_EEPROM_OVERLOAD;
	
	/* Load the EEPROM memory to the RAM memory */
	eeprom_read_block((void*)&REGISTER[memory_ID], (const void*)(eeprom_register*memoryCOUNT*4 + memory_ID*4), 4); //*4 because the vectors are made of 32 bits int (4 bytes)
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Read a value from the internal EEPPROM
 *
 * @param [in] eeprom_register
 *	Slot in the EEPROM to save the vector (1, 2, 3 or 4)
 * @param [in] memory_ID
 *	ID of the memory to save
 * @param [out] value
 *	Read value
 ******************************************************************************/
int ReadValueFromEEPROM(uint16_t eeprom_register, uint16_t memory_ID, uint32_t *value){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + ReadValueFromEEPROM_Code;
	
	if(eeprom_register*memoryCOUNT*4 + memory_ID*4 + 3 > INT_EEPROM_MAX_ADDR) return INT_EEPROM_OVERLOAD;
	
	/* Load the EEPROM memory to the RAM memory */
	eeprom_read_block((void*)value, (const void*)(eeprom_register*memoryCOUNT*4 + memory_ID*4), 4); //*4 because the vectors are made of 32 bits int (4 bytes)
	return OK;
}