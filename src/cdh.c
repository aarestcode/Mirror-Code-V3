/***************************************************************************//**
 * @file	cdh.c
 * @brief	Source file to manage Command Data and Handling (CDH)
 *
 * This file contains all the implementations for the functions defined in:
 * inc/cdh.h
 *
 * @author	Thibaud Talon
 * @date	12/03/2017
 *******************************************************************************/
#include "cdh.h" //
#include "watchdog.h" //
#include "uart.h" //
#include "spi.h" //
#include "i2c.h" //
#include "adc.h" //
#include "comms.h" //
#include "power.h" //
#include "separation_device.h" //
#include "picomotor_actuation.h" //
#include "electrode_actuation.h" //
#include "temperature.h" //
#include "memory.h" //
#include "code_programming.h"
#include "board_management.h"

#ifndef OK
#define OK 0
#endif

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Initialize CDH
 *
 ******************************************************************************/
void CDH_INIT(void){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + CDH_INIT_Code;
	
	REGISTER[memory_PARSE_ERROR] = 0;
}

/***************************************************************************//**
 * @author Thibaud Talon
 * @date   12/03/2017
 *
 * Parse command
 *
 ******************************************************************************/
int ParseCommand(port_t port){
	REGISTER[memory_CURRENT_FUNCTION] = (REGISTER[memory_CURRENT_FUNCTION]<<8) + ParseCommand_Code;
	int error;
	//--------------------------------------------------
    //                 MESSAGE CHECK
	//--------------------------------------------------
	uint8_t command =0;
	int32_t data = 0;
	uint16_t checksum;
	if ((error = CheckMessage(&command, &data, &checksum))){
		if ( port == 1 ) return SendFeedback(port,253,((uint32_t)REGISTER[memory_UART0_INDEX] << 16) + ((long)command<<8) + checksum, error);
		if ( port == 2 ) return SendFeedback(port,253,((uint32_t)REGISTER[memory_UART1_INDEX] << 16) + ((long)command<<8) + checksum, error);
	}	

	//--------------------------------------------------
    //                       PRIVATE
	//--------------------------------------------------
	// command = 0
	if (command	== 0){
		error = SendFeedback(port,0,0xAA12e57,OK); //Send back AAReST written in Hex
		if(error) return error;
	}

	//--------------------------------------------------
    //                   REGISTER WRITE
	//--------------------------------------------------
	// command = 1 to memoryCOUNT
	else if (command < memoryCOUNT)
	{	
		REGISTER[command] = data;
		error = SendFeedback(port,command,REGISTER[command],OK);
		if(error) return error;	
	}

	//--------------------------------------------------
    //                   REGISTER READ
	//--------------------------------------------------
	// command = 150
	else if(command == 150){
		error = SendFeedback(port,data,REGISTER[data], OK);
		if(error) return error;
	}
		
	//--------------------------------------------------
	//                        UART
	//--------------------------------------------------	
	// RE-INITIALIZE UART0
	else if (command==151){
		int status = UART0_INIT(data);
		error = SendFeedback(port,command,REGISTER[memory_UART0_BAUD],status);
		if(error) return error;
	}
	
	// RE-INITIALIZE UART1
	else if (command==152){
		int status = UART1_INIT(data);
		error = SendFeedback(port,command,REGISTER[memory_UART1_BAUD],status);
		if(error) return error;
	}
	
	//--------------------------------------------------
	//                        SPI
	//--------------------------------------------------
	// RE-INITIALIZE SPI
	else if (command==153){
		int status = SPI_INIT(data);
		error = SendFeedback(port,command,REGISTER[memory_SPI_FREQ],status);
		if(error) return error;
	}
	
	//--------------------------------------------------
	//                        I2C
	//--------------------------------------------------
	// RE-INITIALIZE I2C
	else if (command==154){
		int status = I2C_INIT(data);
		error = SendFeedback(port,command,REGISTER[memory_I2C_FREQ],status);
		if(error) return error;
	}
	
	// TEST I2C
	else if (command==155){
		int status;
		if (data & 0x01) status = I2C_READ(data, (uint8_t [0]){}, 0, (uint8_t [0]){}, 0); 
		else status = I2C_WRITE(data, (uint8_t [0]){}, 0); 
		error = SendFeedback(port,command,data & 0xFF,status);
		if(error) return error;
	}
	
	//--------------------------------------------------
	//                        ADC
	//--------------------------------------------------
	// RE-INITIALIZE ADC
	else if (command==156){
		int status = ADC_INIT(data);
		error = SendFeedback(port,command,REGISTER[memory_ADC_FREQ],status);
		if(error) return error;
	}
	
	//--------------------------------------------------
	//                        COMMS
	//--------------------------------------------------
	// RE-INITIALIZE COMMUNICATIONS
	else if (command==157){
		int status = COMMS_INIT(data);
		error = SendFeedback(port,command,REGISTER[memory_COMMUNICATION_TIMEOUT],status);
		if(error) return error;
	}
	
	//--------------------------------------------------
	//                        POWER
	//--------------------------------------------------
	// RE-INITIALIZE POWER
	else if (command==160){
		int status = POWER_INIT();
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	
	// ENABLE SUPPLY VOLTAGE
	else if (command==161){
		EnableSV(data,true);
		error = SendFeedback(port,command,0,OK);
		if(error) return error;
	}
	
	// DISABLE SUPPLY VOLTAGE
	else if (command==162){
		EnableSV(data,false);
		error = SendFeedback(port,command,0,OK);
		if(error) return error;
	}
	
	// MEASURE FB VOLTAGE
	else if (command==163){
		uint16_t val;
		int status = MeasureV(data,&val);
		error = SendFeedback(port,command, val, status);
		if(error) return error;
	}
	
	// CHECK IF FB VOLTAGE IS OUT OF BOUNDS
	else if (command==164){
		int status = IsFeedbackVOOB(data>>24, (data >> 8) & 0xffff, data & 0xff);
		error = SendFeedback(port,command,status,OK);
		if(error) return error;
	}
	
	// ENABLE CURRENT LIMITER
	else if (command==165){
		EnableCL(data,true);
		error = SendFeedback(port,command,0,OK);
		if(error) return error;
	}
	
	// DISABLE CURRENT LIMITER
	else if (command==166){
		EnableCL(data,false);
		error = SendFeedback(port,command,0,OK);
		if(error) return error;
	}
	
	// CURRENT LIMITER FAULT
	else if (command==167){
		int status = IsCLFault(data);
		error = SendFeedback(port,command,status,OK);
		if(error) return error;
	}
	
	// ACTIVATE ELECTRODE HV
	else if (command==168){
		int status = ActivateHV();
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	
	// DEACTIVATE ELECTRODE HV
	else if (command==169){
		int status = DeactivateHV();
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	
	// CHANGE VARIABLE HV
	else if (command==170){
		int status = SetVoltage(data);
		error = SendFeedback(port,command,data,status);
		if(error) return error;
	}
	
	// CHANGE BIAS HV
	else if (command==171){
		int status = SetBias(data);
		error = SendFeedback(port,command,data,status);
		if(error) return error;
	}
	
	// ACTIVATE PICOMOTOR HV
	else if (command==172){
		int status = ActivatePICOV();
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	
	// DEACTIVATE PICOMOTOR HV
	else if (command==173){
		DeactivatePICOV();
		error = SendFeedback(port,command,0,OK);
		if(error) return error;
	}
	
	//--------------------------------------------------
	//                 SEPARATION DEVICE
	//--------------------------------------------------
	// RE-INITIALIZE SEPERATION DEVICE
	else if (command==175){
		SEPARATION_DEVICE_INIT();
		error = SendFeedback(port,command,0,OK);
		if(error) return error;
	}
	
	// RELEASE SEPERATION DEVICE
	else if (command==176){
		int status = SetMode(MODE_SEPARATION);
		if (status == OK) StartRelease(data);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	
	// CHECK SEPERATION DEVICE OFF
	else if (command==177){
		int status = IsMirrorConstrained();
		error = SendFeedback(port,command,status,0);
		if(error) return error;
	}
	
	//--------------------------------------------------
	//               PICOMOTOR ACTUATION
	//--------------------------------------------------
	// RE-INITIALIZE PICOMOTORS
	else if (command==180){
		int status = PICOMOTOR_ACTUATION_INIT();
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	
	// LEFT PICOMOTOR
	else if(command==181){ // MOVE BY TICKS
		SetTicks(picomotor1, (int32_t)data);
		int status = SetMode(MODE_PICOMOTOR);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	else if(command==182){ // MOVE BY INTERVALS
		SetIntervals(picomotor1, (int32_t)data);
		int status = SetMode(MODE_PICOMOTOR);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	else if(command==183){ // PRECISE POSITIONING
		SetPreciseLocation(picomotor1, (float)data);
		int status = SetMode(MODE_PICOMOTOR);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	else if(command==184){ // INITIALIZE
		SetInitialization(picomotor1, (uint32_t)data);
		int status = SetMode(MODE_PICOMOTOR);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	else if(command==185){ // CALIBRATE
		SetCalibration(picomotor1, (int16_t)data);
		int status = SetMode(MODE_PICOMOTOR);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	else if(command==186){ // MEASURE ENCODER STATE
		encoder_state_t state;
		int status = GetEncoderState(picomotor1, &state);
		error = SendFeedback(port,command,state,status);
		if(error) return error;
	}
	else if(command==187){ // CHECK IF ACTUATION DONE
		int status = IsPicomotorUpdateDone(picomotor1);
		int error = SendFeedback(port,command,status,OK);
		if(error) return error;
	}
	else if(command==188){ // STOP ACTUATION
		StopActuation(picomotor1);
		int error = SendFeedback(port,command,0,OK);
		if(error) return error;
	}
	
	// RIGHT PICOMOTOR
	else if(command==191){ // MOVE BY TICKS
		SetTicks(picomotor2, (int32_t)data);
		int status = SetMode(MODE_PICOMOTOR);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	else if(command==192){ // MOVE BY INTERVALS
		SetIntervals(picomotor2, (int32_t)data);
		int status = SetMode(MODE_PICOMOTOR);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	else if(command==193){ // PRECISE POSITIONING
		SetPreciseLocation(picomotor2, (float)data);
		int status = SetMode(MODE_PICOMOTOR);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	else if(command==194){ // INITIALIZE
		SetInitialization(picomotor2, (uint32_t)data);
		int status = SetMode(MODE_PICOMOTOR);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	else if(command==195){ // CALIBRATE
		SetCalibration(picomotor2, (int16_t)data);
		int status = SetMode(MODE_PICOMOTOR);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	else if(command==196){ // MEASURE ENCODER STATE
		uint8_t state;
		int status = GetEncoderState(picomotor2, &state);
		error = SendFeedback(port,command, state,status);
		if(error) return error;
	}
	else if(command==197){ // CHECK IF ACTUATION DONE
		int status = IsPicomotorUpdateDone(picomotor2);
		int error = SendFeedback(port,command,status,OK);
		if(error) return error;
	}
	else if(command==198){ // STOP ACTUATION
		StopActuation(picomotor2);
		int error = SendFeedback(port,command,0,OK);
		if(error) return error;
	}
	
	// BOTTOM PICOMOTOR
	else if(command==201){ // MOVE BY TICKS
		SetTicks(picomotor3, (int32_t)data);
		int status = SetMode(MODE_PICOMOTOR);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	else if(command==202){ // MOVE BY INTERVALS
		SetIntervals(picomotor3, (int32_t)data);
		int status = SetMode(MODE_PICOMOTOR);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	else if(command==203){ // PRECISE POSITIONING
		SetPreciseLocation(picomotor3, (float)data);
		int status = SetMode(MODE_PICOMOTOR);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	else if(command==204){ // INITIALIZE
		SetInitialization(picomotor3, (uint32_t)data);
		int status = SetMode(MODE_PICOMOTOR);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	else if(command==205){ // CALIBRATE
		SetCalibration(picomotor3, (int16_t)data);
		int status = SetMode(MODE_PICOMOTOR);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	else if(command==206){ // MEASURE ENCODER STATE
		uint8_t state;
		int status = GetEncoderState(picomotor3, &state);
		int error = SendFeedback(port,command,state,status);
		if(error) return error;
	}
	else if(command==207){ // CHECK IF ACTUATION DONE
		int status = IsPicomotorUpdateDone(picomotor3);
		int error = SendFeedback(port,command,status,OK);
		if(error) return error;
	}
	else if(command==208){ // STOP ACTUATION
		StopActuation(picomotor3);
		int error = SendFeedback(port,command,0,OK);
		if(error) return error;
	}
	
	// CHECK IF PICOMOTOR ACTUATION DONE
	else if(command==209){ // CHECK IF ACTUATION DONE
		int status = IsAllPicomotorUpdateDone();
		int error = SendFeedback(port,command,status,OK);
		if(error) return error;
	}
	
	//--------------------------------------------------
	//               ELECTRODE ACTUATION
	//--------------------------------------------------
	// RE-INITIALIZE ELECTRODES
	else if (command==210){
		int status = ELECTRODE_ACTUATION_INIT();
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	
	// TURN CHANNEL ON
	else if (command==211){
		int status = ChannelOn(data);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	// TURN CHANNEL OFF
	else if (command==212){
		int status = ChannelOff(data);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	
	// START ELECTRODE ACTUATION
	else if (command==213){
		int status = SetMode(MODE_ELECTRODE);
		resetWatchdogTimer();
		if (status == 0) status = StartElectrodeActuation();
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	
	// STOP ELECTRODE ACTUATION
	else if (command==214){
		int status = OK;
		if (Mode == MODE_ELECTRODE) status = SetMode(MODE_NOMINAL);
		resetWatchdogTimer();
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	
	//--------------------------------------------------
	//                   TEMPERATURE
	//--------------------------------------------------
	// RE-INITIALIZE THERMO-SENSORS
	else if (command==215){
		int status = TEMP_SENSORS_INIT(data);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	
	// IS PCT2075 ACTIVE
	else if(command==216){
		int status = IsPCT2075Active(data);
		error = SendFeedback(port,command,status,OK);
		if(error) return error;
	}
	
	// ENABLE PCT2075 SENSOR
	else if(command==217){
		EnablePCT2075(data, true);
		error = SendFeedback(port,command,0,OK);
		if(error) return error;
	}
	
	// DISABLE PCT2075 SENSOR
	else if(command==218){
		EnablePCT2075(data, false);
		error = SendFeedback(port,command,0,OK);
		if(error) return error;
	}
	
	// MEASURE TEMP FROM PCT2075
	else if(command==219){
		int16_t temp;
		int status = GetTemperaturePCT2075(data, &temp);
		error = SendFeedback(port,command,temp,status);
		if(error) return error;
	}
	
	// IS TMP006 ACTIVE
	else if(command==220){
		int status = IsTMP006Active(data);
		error = SendFeedback(port,command,status,OK);
		if(error) return error;
	}
	
	// ENABLE TMP006 SENSOR
	else if(command==221){
		EnableTMP006(data, true);
		error = SendFeedback(port,command,0,OK);
		if(error) return error;
	}
	
	// DISABLE TMP006 SENSOR
	else if(command==222){
		EnableTMP006(data, false);
		error = SendFeedback(port,command,0,OK);
		if(error) return error;
	}
		
	// MEASURE TEMP FROM TMP006
	else if(command==223){
		int16_t temp;
		int status = GetTemperatureTMP006(data, &temp);
		error = SendFeedback(port,command,temp,status);
		if(error) return error;
	}
	
	//--------------------------------------------------
	//                     MEMORY
	//--------------------------------------------------
	// SAVE REGISTER
	else if(command==225){
		int status = SaveRegister(data);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	
	// SAVE REGISTER VALUE
	else if(command==226){
		int status = SaveRegisterValue(data >> 16, data & 0xffff);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}

	// LOAD MEMORY
	else if(command==227){
		int status = LoadRegister(data);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	
	// LOAD MEMORY VALUE
	else if(command==228){
		int status = LoadRegisterValue(data >> 16, data & 0xffff);
		error = SendFeedback(port,command,REGISTER[data & 0xffff],status);
		if(error) return error;
	}
	
	// READ MEMORY VALUE
	else if(command==229){
		uint32_t value;
		int status = ReadValueFromEEPROM(data >> 16, data & 0xffff, &value);
		error = SendFeedback(port,command,value,status);
		if(error) return error;
	}
	
	//--------------------------------------------------
	//                BOARD MANAGEMENT
	//--------------------------------------------------
	// SET MODE (NOT RECOMMENDED)
	else if(command==230){
		int status =  SetMode(data);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	// READ CURRENT MODE
	else if(command==231){
		error = SendFeedback(port,command,Mode,OK);
		if(error) return error;
	}
	// READ PREVIOUS MODE
	else if(command==232){
		error = SendFeedback(port,command,PreviousMode,OK);
		if(error) return error;
	}
	// TRIGGER HEALTH CHECK
	else if(command==233){
		int status = CheckHealth();
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
		
	//--------------------------------------------------
	//                CODE PROGRAMMING
	//--------------------------------------------------
	// WRITE CODE TO EEPROM
	else if(command==245){
		uint8_t Npages = data & 0xffff;
		CodeID_t CodeID = data >> 16;
		int status =  SaveApplicationFromCameraToEEPROM(port, CodeID, Npages);
		error = SendFeedback(port,command,0,status);
		if(error) return error;
	}
	
	// GET SIZE OF CODE IN EEPROM
	else if(command==246){
		uint8_t Npages;
		int status = GetSizeofCode(data, &Npages);
		error = SendFeedback(port,command,Npages,status);
		if(error) return error;
	}
	
	// READ DWORD OF CODE IN EEPROM
	else if(command==247){
		uint32_t dword;
		int status = ReadDWordFromEEPROM(data>>16, data & 0xffff, &dword);
		error = SendFeedback(port,command,dword,status);
		if(error) return error;
	}
	
	//--------------------------------------------------
	//         RESET, PING AND ERRONEOUS COMMAND
	//--------------------------------------------------
	// SAFE RESET
	else if(command==250){
		resetSystem();
	}
	
	//PING
	else if(command==255){
		error = SendFeedback(port,command,data,0);
		if(error) return error;
		REGISTER[memory_BOOT_SAFE] = 0;
		SaveRegisterValue(0, memory_BOOT_SAFE);
	}
	
	// WRONG COMMAND
	else{
		error = SendFeedback(port,254,command,0);
		if(error) return error;
	}
	
	return OK;
}
