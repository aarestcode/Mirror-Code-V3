/***************************************************************************//**
 * @file	board_management.h
 * @brief	Header file to manage the health of the boards
 *
 * This header file contains all the required definitions and function prototypes
 * through which to manage the health of the boards
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/
#ifndef BOARD_MANAGEMENT_H_
#define BOARD_MANAGEMENT_H_

typedef enum{
	MODE_NOMINAL,
	MODE_ELECTRODE,
	MODE_PICOMOTOR,
	MODE_SEPARATION,
	SAFE_MODE,
} Mode_t;

Mode_t Mode;
Mode_t PreviousMode;

enum board_management_error {	
	SET_FROM_SAFE_MODE = 131,	
	SET_FROM_PICOMOTOR_RUNNING,
	SET_SEPARATION,
	SET_FROM_SEPARATION_RUNNING,
	SET_FROM_SEPARATION,
};

enum board_management_function{
	BOARD_MANAGEMENT_INIT_Code = 131,
	SetMode_Code,
	CheckHealth_Code,
};

enum safe_mode_triggers{
	PCT2075_1_OOB = 1,
	PCT2075_2_OOB,
	TMP006_1_OOB,
	TMP006_2_OOB,
	TMP006_3_OOB,
	SEPARATION_OOB,
	PICO_OOB,
	VARIABLE_OOB,
	BIAS_OOB,
	CL1_OOB,
	CL2_OOB,
	CL3_OOB
};

void BOARD_MANAGEMENT_INIT(void);
int SetMode(Mode_t _Mode);
int CheckHealth(void);

#endif /* BOARD_MANAGEMENT_H_ */