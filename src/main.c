#include "peripheral.h"
#include "sensor.h"
#include "serial.h"
#include "calibration.h"
extern uint8_t calibration_enabled;

int test_res = 0;

uint8_t connected;
uint8_t blinked;

uint8_t charging;
uint8_t discharged;
float charge;

// sensor variables
float accel[3], magnetom[3], gyro[3];
float aRes, mRes, gRes;
float uptime;

// timers
uint32_t timer_flag;

// button variables
uint8_t long_press, short_press, key_pressed;

int main (void)
{
	
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(RCC_Clocks.SYSCLK_Frequency/1000);  // setting system 1ms timer
	NVIC_SetPriority (SysTick_IRQn,0);

	// calculating sensor's resolution
	getAres();
	getMres();
	getGres();
		
	// I/O initialization
	GPIO_config();
	LEDG_ON;
	ADC1_Init();
	I2C1_config();
	
	// init sensors
#ifdef MPU_9250
	initMPU9250();
  initAK8963();
#else	// GY_85
	Accel_Init();
	Magn_Init();
	Gyro_Init();	
#endif

	Delay_ms(20);
	
	// Setting BT name of device	
	First_Startup();
	
	
	UART1_config(115200);
	
	// read lod calibration values
	read_EEPROM_calib();	
	
	SetTimer(led, LED_JOB_PERIOD);
	
	LEDG_OFF;
	
	//reset_sensor_fusion();
  SerialSetup();

#ifndef SOFT_TIMER
	Timer_init(SENSORS_INTERVAL);
#else
	SetTimer(get_sens,SENSORS_INTERVAL);
#endif	// SOFT_TIMER

	SetTimer(battery, BATTERY_PERIOD);	
	SetTimer(calib, CALIB_INTERVAL);	
	SetTimer(sens_reset, 5000);	
	
	
	while(1)
	{

		if (!(GPIOC->IDR & GPIO_Pin_15))
			connected = 0;		
		else
			connected = 1;

		if (GPIOA->IDR & GPIO_Pin_8)
			charging = 0;
		else
			charging = 1;

		// checking if calibration is started
		if (timer_flag & (1<<calib))
		{
			timer_flag &= ~(1<<calib);
			SetTimer(calib, CALIB_INTERVAL);
			
			calibration_loop();
		}
		
		// if time to get new sensor data
		if ((timer_flag & (1<<get_sens)) && !calibration_enabled)
		{
			timer_flag &= ~(1<<get_sens);
			SetTimer(get_sens,SENSORS_INTERVAL);				
			
			uptime += (float) SENSORS_INTERVAL/1000.0;
			
			// Update sensor readings
			read_sensors();
			
			if (connected)
				SerialSendPacket();		
		}

		

#ifdef CALIB_BUTTON		
		// button debounce check
		if (timer_flag & (1<<button_debounce))
		{
			timer_flag &= ~(1<<button_debounce);
			if (!(GPIOB->IDR & GPIO_Pin_9))
			{
				key_pressed = 1;
				SetTimer(button_press,BUTTON_LONG_PRESS);
			}
		}	
		// long press detecting timer
		if (timer_flag & (1<<button_press))
		{
			timer_flag &= ~(1<<button_press);
			if (key_pressed)
			{
				key_pressed = 0;
				long_press = 1;
			}
		}
#endif // CALIB_BUTTON
		
		// battery check timer
		if (timer_flag & (1<<battery))
		{
			timer_flag &= ~(1<<battery);
			SetTimer(battery, BATTERY_PERIOD);
			
			charge = Get_Battery();
			
			if (charge < BATTERY_ALARM_LEVEL)
				discharged = 1;
			else if (charge > BATTERY_ALARM_LEVEL + BATTERY_HYSTERESYS)
				discharged = 0;	
		}
#ifndef LEDS_OFF

		// main led timer
		if (timer_flag & (1<<led))
		{
			timer_flag &= ~(1<<led);
			
			if (discharged)
			{
				SetTimer(led, LED_DISCHARGE_PERIOD);
				LEDG_TOGGLE;
			}		
			else if (!connected && !calibration_enabled)				// stadndby mode
			{
				if (!blinked)
				{
					SetTimer(led, LED_STANDBY_OFF_PERIOD);
					LEDG_OFF;
				}
				else
				{
					SetTimer(led, LED_STANDBY_ON_PERIOD);
					LEDG_ON;
				}			
				blinked ^= 1;				
			}
			else if (connected && !calibration_enabled)		// active mode
			{
				SetTimer(led, LED_JOB_PERIOD);
				LEDG_TOGGLE;
			}
			else if (calibration_enabled)									// calibration enabled
			{
				SetTimer(led, LED_CALIB_PERIOD);
				LEDG_TOGGLE;
			}			
			
		}
#endif // LEDS_OFF		
	}
	return 1;
}

void TIM4_IRQHandler (void)
{
	TIM4->SR &= ~TIM_SR_UIF;
	
	// if time to get new sensor data
		if ( !calibration_enabled)
		{
			timer_flag &= ~(1<<get_sens);
			SetTimer(get_sens,SENSORS_INTERVAL);				
			
			uptime += (float) SENSORS_INTERVAL/1000.0;
			
			// Update sensor readings
			read_sensors();	
			
			if (connected)
				SerialSendPacket();

		}
}
