/***************************************************************************//**
 * @file	power.h
 * @brief	Header file to manage the power lines
 *
 * This header file contains all the required definitions and function prototypes
 * through which to manage the power lines
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/
#ifndef POWER_H_
#define POWER_H_

#include <inttypes.h>
#include <stdbool.h>

// AD5641
// PINSET
#define DDR_SV DDRB
#define PORT_SV PORTB
#define FIVE_V_E PORTB1
#define TWELVE_V_E PORTB0
#define TWO_EIGHT_V_E PORTB2

#define DDR_CL_E DDRC
#define PORT_CL_E PORTC
#define CL1_E PORTC4
#define CL2_E PORTC5
#define CL3_E PORTC6

#define DDR_CL_F1 DDRA
#define PIN_CL_F1 PINA
#define CL2_F PORTA6
#define CL3_F PORTA7

#define DDR_CL_F2 DDRD
#define PIN_CL_F2 PIND
#define CL1_F PORTD6

#define TWO_FIVE_V 775

typedef enum{
	SUPPLY_VOLTAGE_FIVE,
	SUPPLY_VOLTAGE_TWELVE,
	SUPPLY_VOLTAGE_TWO_EIGHT
} supply_voltage_t;

typedef enum{
	FEEDBACK_VOLTAGE_HV_VOLTAGE,
	FEEDBACK_VOLTAGE_HV_GROUND,
	FEEDBACK_VOLTAGE_PICOMOTOR_VOLTAGE,
	FEEDBACK_VOLTAGE_BUS_CURR,
	FEEDBACK_VOLTAGE_BUS_VOLT,
} feedback_voltage_t;

typedef enum{
	CURRENT_LIMITER_1,
	CURRENT_LIMITER_2,
	CURRENT_LIMITER_3
} current_limiter_t;

// ERROR ENUM
enum power_error{	
	VAR_FEEDBACK_OOB = 91,
	BIAS_FEEDBACK_OOB,
	PICOMOTOR_FEEDBACK_OOB,
	CL1_FAULT,
	CL2_FAULT,
	CL3_FAULT,
};

enum power_function{
	POWER_INIT_Code = 191,
	EnableSV_Code,
	MeasureV_Code,
	IsFeedbackVOOB_Code,
	EnableCL_Code,
	IsCLFault_Code,
	ActivateHV_Code,
	DeactivateHV_Code,
	SetVoltage_Code,
	SetBias_Code,
	ActivatePICOV_Code,
	DeactivatePICOV_Code
};

// FUNCTIONS
int POWER_INIT(void);

void EnableSV(supply_voltage_t SV_index, bool state);

int MeasureV(feedback_voltage_t FB_index, uint16_t* val);
bool IsFeedbackVOOB(feedback_voltage_t FB_index, uint16_t target, uint8_t max_tries);

void EnableCL(current_limiter_t CL_index, bool state);
bool IsCLFault(current_limiter_t CL_index);

int ActivateHV(void);
int DeactivateHV(void);
int SetVoltage(uint16_t voltage);
int SetBias(uint16_t voltage);

int ActivatePICOV(void);
void DeactivatePICOV(void);


#endif /* POWER_H_ */