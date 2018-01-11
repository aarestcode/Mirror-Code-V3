/***************************************************************************//**
 * @file	electrode actuation.h
 * @brief	Header file to manage the electrodes
 *
 * This header file contains all the required definitions and function prototypes
 * through which to manage the electrodes
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/
#ifndef ELECTRODE_ACTUATION_H_
#define ELECTRODE_ACTUATION_H_

#include <inttypes.h>

#define N_electrodes 41 //Number of electrodes

enum electrode_error{
	ELECTRODE_ID_OOB = 121,
};

enum electrode_function{
	ELECTRODE_ACTUATION_INIT_Code = 121,
	ChannelOn_Code,
	ChannelOff_Code,
	StartElectrodeActuation_Code,
	ActuateElectrode_Code,
};

// FUNCTIONS
int ELECTRODE_ACTUATION_INIT(void);

int ChannelOn(uint8_t ch);
int ChannelOff(uint8_t ch);

int StartElectrodeActuation(void);
int ActuateElectrode(int channel);


#endif /* ELECTRODE_ACTUATION_H_ */