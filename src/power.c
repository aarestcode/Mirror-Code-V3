/***************************************************************************//**
 * @file	power.c
 * @brief	Source file to manage  the power lines
 *
 * This file contains all the implementations for the functions defined in:
 * inc/power.h
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/

#ifndef F_CPU
#define F_CPU 8000000UL // CPU Frequency (IMPORTANT)
#endif

#include "power.h"
#include "memory.h"
#include "adc.h"
#include "spi.h"
#include <avr/io.h> //General I/O
#define __DELAY_BACKWARD_COMPATIBLE__ //To use variables in delay functions
#include <util/delay.h> //Delay functions
#include <stdlib.h>

#ifndef OK
#define OK 0
#endif

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Initialize power lines
 *
 ******************************************************************************/
int POWER_INIT(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + POWER_INIT_Code;
	
		
	//TODO: Delte (in register)
	REGISTER[memory_HV_TOL_V] = 50; // 0.15V tolerance over 3.3V max

	// Set Supply voltage enable as output
	DDR_SV |= ((1<<FIVE_V_E) | (1<<TWELVE_V_E) | (1<<TWO_EIGHT_V_E));
	
	// Disable Supply Voltages
	EnableSV(SUPPLY_VOLTAGE_TWELVE,false);
	EnableSV(SUPPLY_VOLTAGE_FIVE,false);
	EnableSV(SUPPLY_VOLTAGE_TWO_EIGHT,false);
	
	// Set Current Limiters Enable as output
	DDR_CL_E |= ((1<<CL1_E) | (1<<CL2_E) | (1<<CL3_E));
	
	// Disable Current Limiters
	EnableCL(CL1_E,false);
	EnableCL(CL2_E,false);
	EnableCL(CL3_E,false);
		
	// Set Current Limiters Fault as inputs
	DDR_CL_F1 &= ~((1<<CL2_F) | (1<<CL3_F));
	DDR_CL_F2 &= ~(1<<CL1_F);
	
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Enable supply voltage line
 *
 * @param [in] SV_index
 *	Index of the supply voltage
 * @param [in] state
 *	true = Enable, false = Disable
 ******************************************************************************/
void EnableSV(supply_voltage_t SV_index, bool state){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + EnableSV_Code;
	
	if(state){
		if(SV_index == SUPPLY_VOLTAGE_FIVE) PORT_SV |= (1<<FIVE_V_E);
		if(SV_index == SUPPLY_VOLTAGE_TWELVE) PORT_SV |= (1<<TWELVE_V_E);
		if(SV_index == SUPPLY_VOLTAGE_TWO_EIGHT) PORT_SV |= (1<<TWO_EIGHT_V_E);
	}
	else{
		if(SV_index == SUPPLY_VOLTAGE_FIVE)	 PORT_SV &= ~(1<<FIVE_V_E);
		if(SV_index == SUPPLY_VOLTAGE_TWELVE)	 PORT_SV &= ~(1<<TWELVE_V_E);
		if(SV_index == SUPPLY_VOLTAGE_TWO_EIGHT)	 PORT_SV &= ~(1<<TWO_EIGHT_V_E);
	}
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Enable supply voltage line
 *
 * @param [in] FB_index
 *	Index of the feedback voltage to measure
 * @param [out] val
 *	Pointer to the image of the measure voltage (V = 3.3/1024*val)
 ******************************************************************************/
int MeasureV(feedback_voltage_t FB_index, uint16_t* val){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + MeasureV_Code;
	
	if (FB_index == FEEDBACK_VOLTAGE_HV_VOLTAGE){
		int error = ADC_READ(HV_VOLTAGE,val);
		if(error) return error;
		
		REGISTER[memory_HV_VOLT_FB] = *val;
	}
	if (FB_index == FEEDBACK_VOLTAGE_HV_GROUND){
		int error = ADC_READ(HV_GROUND,val);
		if(error) return error;
		
		REGISTER[memory_HV_BIAS_FB] = *val;
	}
	if (FB_index == FEEDBACK_VOLTAGE_PICOMOTOR_VOLTAGE){
		int error = ADC_READ(PICOMOTOR_VOLTAGE,val);
		if(error) return error;
		
		REGISTER[memory_PICOMOTOR_V_FB] = *val;
	}
	if (FB_index == FEEDBACK_VOLTAGE_BUS_CURR){
		int error = ADC_READ(BUS_CURR,val);
		if(error) return error;
		
		REGISTER[memory_BUS_CURR_FB] = *val;
	}
	if (FB_index == FEEDBACK_VOLTAGE_BUS_VOLT){
		int error = ADC_READ(BUS_VOLT,val);
		if(error) return error;
		
		REGISTER[memory_BUS_VOLT_FB] = *val;
	}
	
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Enable supply voltage line
 *
 * @param [in] FB_index
 *	Index of the feedback voltage to measure
 * @param [out] target
 *	Target voltage
 * @param [out] max_tries
 *	Maximum number of tries to match measured voltage to target
 ******************************************************************************/
bool IsFeedbackVOOB(feedback_voltage_t FB_index, uint16_t target, uint8_t max_tries){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + IsFeedbackVOOB_Code;
	
	// Measure voltage
	uint16_t val;
	uint8_t counter = 0;
	
	do{
		MeasureV(FB_index,&val);		
	} while ((abs(val-target) > REGISTER[memory_HV_TOL_V])  && (counter++ < max_tries) );
	
	return (abs(val-target) > REGISTER[memory_HV_TOL_V]);
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Enable current limiter
 *
 * @param [in] CL_index
 *	Line of the current limiter
 * @param [in] state
 *	true = Enable, false = Disable
 ******************************************************************************/
void EnableCL(current_limiter_t CL_index, bool state){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + EnableCL_Code;
	
	if(state){
		if (CL_index == CURRENT_LIMITER_1) PORT_CL_E &= ~(1<<CL1_E);
		if (CL_index == CURRENT_LIMITER_2) PORT_CL_E &= ~(1<<CL2_E);
		if (CL_index == CURRENT_LIMITER_3) PORT_CL_E &= ~(1<<CL2_E);
	}
	else{
		if (CL_index == CURRENT_LIMITER_1) PORT_CL_E |= (1<<CL1_E);
		if (CL_index == CURRENT_LIMITER_2) PORT_CL_E |= (1<<CL2_E);
		if (CL_index == CURRENT_LIMITER_3) PORT_CL_E |= (1<<CL3_E);
	}
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Check if current limiter has faulted
 *
 * @param [in] CL_index
 *	Index of the current limiter
 ******************************************************************************/
bool IsCLFault(current_limiter_t CL_index){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + IsCLFault_Code;
	
	if (CL_index == CURRENT_LIMITER_1) return (PIN_CL_F2 & (1<<CL1_F));
	if (CL_index == CURRENT_LIMITER_2) return (PIN_CL_F1 & (1<<CL2_F));
	if (CL_index == CURRENT_LIMITER_3) return (PIN_CL_F1 & (1<<CL3_F));
	else return false;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Activate variable High voltage
 *
 ******************************************************************************/
int ActivateHV(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + ActivateHV_Code;
	
	
	int error;
	
	// 1) Enable 5V
	EnableSV(SUPPLY_VOLTAGE_FIVE,true);
	
	// 2) Command variable HV to 0V
	error = SetVoltage(0x3f00);
	if(error) return error;	
	
	// 3) Command ground to 0V
	error = SetBias(0x3f00);
	if(error) return error;
	
	// 4) Enable 12V
	EnableSV(SUPPLY_VOLTAGE_TWELVE,true);
	
	// 5) Enable CL3
	EnableCL(CURRENT_LIMITER_3,true);
	_delay_ms(1000);
	
	// 6) Check HV_VOLTAGE
	_delay_ms(1500);
	/*if ( IsFeedbackVOOB(FEEDBACK_VOLTAGE_HV_VOLTAGE, TWO_FIVE_V, 50) ){
		DeactivateHV();
		
		return VAR_FEEDBACK_OOB;
	}*/
		
	// 7) Check CL3
	/*if(IsCLFault(CURRENT_LIMITER_3)){
		DeactivateHV();
		
		return CL3_FAULT;
	}*/
	
	// 8) Enable CL2
	EnableCL(CURRENT_LIMITER_2,true);
	
	// 9) Check HV_GROUND at 2.5V
	_delay_ms(1500);
	/*if ( IsFeedbackVOOB(FEEDBACK_VOLTAGE_HV_GROUND, TWO_FIVE_V, 50) ){
		DeactivateHV();
		
		return BIAS_FEEDBACK_OOB;
	}*/
	
	// 10) Check CL2
	/*if(IsCLFault(CURRENT_LIMITER_2)){
		DeactivateHV();
		
		return CL2_FAULT;
	}*/
	
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Deactivate variable High voltage
 *
 ******************************************************************************/
int DeactivateHV(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + DeactivateHV_Code;
	

	// 1) Command variable HV to BIAS
	int error = SetVoltage(REGISTER[memory_HV_BIAS]);
	if(error) return error;
	_delay_ms(REGISTER[memory_HV_TIMER]);
	
	// 2) Disable 12V
	EnableSV(SUPPLY_VOLTAGE_TWELVE,false);
	
	// 3) Disable CL3
	EnableCL(CURRENT_LIMITER_3,false);
	
	// 4) Disable CL2
	EnableCL(CURRENT_LIMITER_2,false);
	
	// 5) Disable 5V
	EnableSV(SUPPLY_VOLTAGE_FIVE,false);
	
	return OK;
	
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Set voltage on variable HV supply
 *
 * @param [in] voltage
 *	Image of the voltage
 ******************************************************************************/
int SetVoltage(uint16_t voltage){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + SetVoltage_Code;
	
	int error = SPI_WRITE(SELECT_HV,(uint8_t [2]){voltage>>8, voltage},2);
	if(error) return error;
	
	REGISTER[memory_HV] = voltage;
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Set voltage on variable HV bias
 *
 * @param [in] voltage
 *	Image of the voltage
 ******************************************************************************/
int SetBias(uint16_t voltage){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + SetBias_Code;
	
	
	int error = SPI_WRITE(SELECT_BIAS,(uint8_t [2]){voltage>>8, voltage},2);
	if(error) return error;
	
	REGISTER[memory_GND] = voltage;
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Activate picomotor High voltage
 *
 ******************************************************************************/
int ActivatePICOV(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + ActivatePICOV_Code;
	
	// 1) Enable 12V
	EnableSV(SUPPLY_VOLTAGE_TWELVE,true);
	
	// 2) Enable CL1
	EnableCL(CURRENT_LIMITER_1,true);
	
	// 3) Check PICOMOTOR_VOLTAGE at 2.5V !!! TAKES ABOUT 1.5 sec TO STABILIZE !!!
	_delay_ms(1500);
	/*if ( IsFeedbackVOOB(FEEDBACK_VOLTAGE_PICOMOTOR_VOLTAGE, TWO_FIVE_V, 50) ){
		DeactivatePICOV();
		
		return PICOMOTOR_FEEDBACK_OOB;
	}
		
	// 4) Check CL1
	if(IsCLFault(CURRENT_LIMITER_1)){
		DeactivatePICOV();
		
		return CL1_FAULT;
	}*/

	// 5) Enable 2.8V
	EnableSV(SUPPLY_VOLTAGE_TWO_EIGHT,true);
		
	return OK;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Deactivate picomotor High voltage
 *
 ******************************************************************************/
void DeactivatePICOV(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + DeactivatePICOV_Code;
	
	_delay_ms(1000);
	
	// 1) Disable 12V
	EnableSV(SUPPLY_VOLTAGE_TWELVE,false);
	
	// 2) Disable CL1
	EnableCL(CURRENT_LIMITER_1,false);
	
	// 5) Disable 2.8V
	EnableSV(SUPPLY_VOLTAGE_TWO_EIGHT,false);
}

