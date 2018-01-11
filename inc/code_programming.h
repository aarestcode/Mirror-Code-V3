/***************************************************************************//**
 * @file	code_programming.h
 * @brief	Header file to manage and program the external EEPROM
 *
 * This header file contains all the required definitions and function prototypes
 * through which to manage and program the external EEPROM
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/
#ifndef CODE_PROGRAMMING_H_
#define CODE_PROGRAMMING_H_

#include "comms.h"
#include <inttypes.h>

#define EXT_EEPROM_PAGESIZE 256 //64 bytes per page

typedef enum {
	code0,
	code1,
	code2,
	code3
}CodeID_t;

enum code_programming_function{
	ReadDWordFromEEPROM_Code = 151,
	GetSizeofCode_Code,
	WriteCodeInfoinEEPROM_Code,
	WritePageInEEPROM_Code,
	SaveApplicationFromCameraToEEPROM_Code
};

int ReadDWordFromEEPROM(CodeID_t CodeID, uint16_t addr, uint32_t * dword);
int GetSizeofCode(CodeID_t CodeID, uint8_t * Npages);
int WriteCodeInfoinEEPROM(CodeID_t CodeID, uint8_t Npages);
int WritePageInEEPROM(CodeID_t CodeID, uint8_t page, uint8_t buffer[EXT_EEPROM_PAGESIZE]);
int SaveApplicationFromCameraToEEPROM(port_t port, CodeID_t CodeID, uint8_t Npages);


#endif /* CODE_PROGRAMMING_H_ */