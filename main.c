/*
 * FLIGHT_DM_V3.c
 *
 * Created: 12/3/2017 7:57:43 PM
 * Author : Thibaud
 */ 

#define F_CPU 8000000UL // CPU Frequency (IMPORTANT)

#include "memory.h"
#include "timer.h"
#include "uart.h"
#include "spi.h"
#include "i2c.h"
#include "led.h"
#include "adc.h"
#include "comms.h"
#include "power.h"
#include "separation_device.h"
#include "temperature.h"
#include "watchdog.h"
#include "electrode_actuation.h"
#include "picomotor_actuation.h"
#include "board_management.h"
#include "cdh.h"

int main(void)
{	
	WATCHDOG_INIT();
	
	LED_INIT();
	SwitchLED(true);
	resetWatchdogTimer();
	
	LoadRegister(0);
	resetWatchdogTimer();
	
	UART0_INIT(9600);
	resetWatchdogTimer();
	UART1_INIT(9600);
	resetWatchdogTimer();
	SPI_INIT(2000000);
	resetWatchdogTimer();
	I2C_INIT(200000);
	resetWatchdogTimer();
	ADC_INIT(100000);
	resetWatchdogTimer();
	
	COMMS_INIT(1000);
	resetWatchdogTimer();
	POWER_INIT();
	resetWatchdogTimer();
	PICOMOTOR_ACTUATION_INIT();
	resetWatchdogTimer();
	SEPARATION_DEVICE_INIT();
	resetWatchdogTimer();
	TEMP_SENSORS_INIT(0);
	resetWatchdogTimer();
	ELECTRODE_ACTUATION_INIT();
	resetWatchdogTimer();
	CDH_INIT();
	resetWatchdogTimer();
	
	BOARD_MANAGEMENT_INIT();
	resetWatchdogTimer();
	TIMER_INIT();
	resetWatchdogTimer();
	
	int ch = 0;
	port_t port;
	int error;
	
	SwitchLED(false);
	
    for(;;){
		
		resetWatchdogTimer();
		
		// Receive telecommand (if any)
		if ( (port = IsCommandWaiting()) ){
			error = ParseCommand(port);
			if (error) REGISTER[memory_PARSE_ERROR] = (REGISTER[memory_PARSE_ERROR] << 8) + error;
			
			resetWatchdogTimer();
		}
		
		if(IsTimeout(TIMER_HEATH)){
			error = CheckHealth();
			if (error) REGISTER[memory_HEALTH_ERROR] = (REGISTER[memory_HEALTH_ERROR] << 8) + error;
			StartTimer(TIMER_HEATH, REGISTER[memory_HEALTH_TIMER]);
		
			resetWatchdogTimer();
		}	
				
		//Actuation loop
		if ( Mode == MODE_ELECTRODE ){
			// Actuate the electrode
			error = ActuateElectrode(ch);
			if (error) REGISTER[memory_ELECTRODE_ERROR] = (REGISTER[memory_ELECTRODE_ERROR] << 8) + error;
			
			// Update electrode index
			if (++ch >= N_electrodes) ch = 0;
		}
				
		if ( Mode == MODE_PICOMOTOR ){
			// Update Picomotors
			error = UpdatePicomotor(picomotor1);
			if (error){
				StopActuation(picomotor1);
				REGISTER[memory_PICO1_ERROR] = (REGISTER[memory_PICO1_ERROR] << 8) + error;
			}
			resetWatchdogTimer();		
				
			UpdatePicomotor(picomotor2);
			if (error) {
				StopActuation(picomotor2);
				REGISTER[memory_PICO2_ERROR] = (REGISTER[memory_PICO2_ERROR] << 8) + error;
			}
			resetWatchdogTimer();
			
			UpdatePicomotor(picomotor3);
			if (error) {
				StopActuation(picomotor3);
				REGISTER[memory_PICO3_ERROR] = (REGISTER[memory_PICO3_ERROR] << 8) + error;
			}
			resetWatchdogTimer();
			
			if (IsAllPicomotorUpdateDone()) SetMode(PreviousMode);
		}
				
		if ( Mode == MODE_SEPARATION ){
			if (IsReleaseOver()) SetMode(MODE_NOMINAL);
		}
		
		resetWatchdogTimer();
				
		if(IsTimeout(TIMER_REGISTER)){
			SaveRegister(0);
			StartTimer(TIMER_REGISTER, REGISTER[memory_REGISTER_TIMER]);
		}
    }
}

