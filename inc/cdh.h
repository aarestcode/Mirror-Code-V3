/***************************************************************************//**
 * @file	cdh.h
 * @brief	Header file to manage Command Data and Handling (CDH)
 *
 * This header file contains all the required definitions and function prototypes
 * through which to manage Command Data and Handling (CDH)
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/
#ifndef CDH_H_
#define CDH_H_

#include "comms.h"

enum cdh_function{
	CDH_INIT_Code = 141,
	ParseCommand_Code,
};

void CDH_INIT(void);
int ParseCommand(port_t port);

#endif /* CDH_H_ */