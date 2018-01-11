/***************************************************************************//**
 * @file	picomotor_actuation.h
 * @brief	Header file to manage the picomotors
 *
 * This header file contains all the required definitions and function prototypes
 * through which to manage the picomotors
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/
#ifndef PICOMOTOR_ACTUATION_H_
#define PICOMOTOR_ACTUATION_H_

#include <inttypes.h>
#include <stdbool.h>

// ENCODER PINSET
#define DDR_ENCODER DDRC
#define PIN_ENCODER PINC
#define ENCODERA PINC2
#define ENCODERB PINC3

// ENCODER STATES
typedef enum {
	state00,
	state10,
	state11,
	state01
} encoder_state_t;
	
// PICOMOTOR INDEX
typedef enum{
	picomotor1,
	picomotor2,
	picomotor3
} picomotor_t;

// ERROR ENUM
enum picomotor_error{
	ENCODER_STATE_CRITICAL = 111,
	CURRENT_STATE_OOB,
	PREVIOUS_STATE_OOB,
	INCORRECT_STATE_CHANGE,
	STATE_MONITOR_CRITICAL,
	MAX_TICKS_COUNT,
	MOVE_INTERVALS_WRONG_DIR,
};

enum picomotor_function{
	PICOMOTOR_ACTUATION_INIT_Code = 171,
	GetEncoderState_Code,
	EncoderStateMonitor_Code,
	MovePicomotorByTicks_Code,
	MovePicomotorByIntervals_Code,
	SetTicks_Code,
	SetIntervals_Code,
	SetPreciseLocation_Code,
	SetInitialization_Code,
	SetCalibration_Code,
	UpdatePicomotor_Code,
	StopActuation_Code,
	IsPicomotorUpdateDone_Code,
	IsAllPicomotorUpdateDone_Code
};

// FUCTIONS
int PICOMOTOR_ACTUATION_INIT(void);

int GetEncoderState(picomotor_t index, encoder_state_t* state);
int EncoderStateMonitor(encoder_state_t current_state, encoder_state_t prev_state, int8_t* update);

int MovePicomotorByTicks(picomotor_t index, int32_t ticks, int32_t* movedTicks);
int MovePicomotorByIntervals(picomotor_t index, int32_t intervals, int32_t* movedIntervals, int32_t* movedTicks);

void SetTicks(picomotor_t index, int32_t ticks);
void SetIntervals(picomotor_t index, int32_t intervals);
void SetPreciseLocation(picomotor_t index, float location);
void SetInitialization(picomotor_t index, uint32_t max_ticks);
void SetCalibration(picomotor_t index, int16_t intervals);

int UpdatePicomotor(picomotor_t index);
void StopActuation(picomotor_t index);
bool IsPicomotorUpdateDone(picomotor_t index);
bool IsAllPicomotorUpdateDone(void);

#endif /* PICOMOTOR_ACTUATION_H_ */