																																																
#ifndef PERIPHERAL_H 
#define PERIPHERAL_H 

#include "math.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"

//#include "RTE_Device.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_flash.h"

#include "sensor.h"

#define DEBUG_OUTPUT
//#define DEV_BOARD

//#define MPU_9250
#define HC05
//#define CALIB_BUTTON
//#define LEDS_OFF
//#define FILTER_MAGNETOMETER
//#define FILTER_ACCEL
//#define NO_DRIFT_CORRECTION
//#define SOFT_TIMER

#define NAME_FLAG_ADDR					0x8008C00

#define BATTERY_ALARM_LEVEL			3.3f		// volts
#define BATTERY_HYSTERESYS			0.1f
#define BATTERY_PERIOD					1000		// period of adc sample

#define ADC_SCALE								0.0008056640625f		// scaling factor for ADC

#define SENSORS_INTERVAL    		20.0			// sensors read period
#define CALIB_INTERVAL					1				// calibration loop period
#define SENSOR_RESET_INTERVAL		60000		// sensor reset interval

#define	BUTTON_LONG_PRESS				3000		// milliseconds of long press
#define BUTTON_DEBOUNCE					25			// debounce time

#define CAL_MAX_TIME						300000	// ttime to auto exit from calibration mode
#define LED_CALIB_PERIOD				50			// led period in calibration mode
#define LED_JOB_PERIOD					1000		// led period in active (connected) mode
#define LED_STANDBY_ON_PERIOD		100			// led ON period in standby mode
#define LED_STANDBY_OFF_PERIOD	2000		// led OFF period in standby mode
#define LED_DISCHARGE_PERIOD		300			// led period for discharged battery

#define I2C_TIMEOUT							10000		

#define Max_Num_Of_Timers				10

#define UART_RX_BUF_SIZE				512
#define UART_TX_BUF_SIZE				512
#define UART_CNT_MASK						UART_RX_BUF_SIZE-1

#define I2C_RX_BUF_SIZE					10

#ifndef MPU_9250
	#define GY_85
#endif

#ifndef HC05
	#define HC06
#endif

#ifndef DEV_BOARD

	#define LEDG_OFF							GPIOB->ODR |= GPIO_Pin_8
	#define LEDG_ON								GPIOB->ODR &= ~GPIO_Pin_8
	#define LEDG_TOGGLE						GPIOB->ODR ^= GPIO_Pin_8

#else // DEV_BOARD

	#define LEDG_OFF							GPIOC->ODR |= GPIO_Pin_13
	#define LEDG_ON								GPIOC->ODR &= ~GPIO_Pin_13
	#define LEDG_TOGGLE						GPIOC->ODR ^= GPIO_Pin_13

#endif // !DEV_BOARD

#define LEDR_OFF								GPIOB->ODR |= GPIO_Pin_4
#define LEDR_ON									GPIOB->ODR &= ~GPIO_Pin_4
#define LEDR_TOGGLE							GPIOB->ODR ^= GPIO_Pin_4

#define BT_MODE_UP							GPIOB->ODR |= GPIO_Pin_5
#define BT_MODE_DN							GPIOB->ODR &= ~GPIO_Pin_5

#define BT_RST_UP								GPIOB->ODR |= GPIO_Pin_13
#define BT_RST_DN								GPIOB->ODR &= ~GPIO_Pin_13



typedef enum 
{
	get_sens = 1,
	button_press,
	button_debounce,
	led,
	cal_max,
	battery,
	discharge,
	calib,
	sens_reset,
	
}timer_flags;

typedef struct 
{
	uint8_t rx_buf[UART_RX_BUF_SIZE];
	uint8_t rx_cnt;
}uart_t;

static __IO uint32_t TimingDelay;

void Delay_ms( __IO uint32_t nTime);
void SetTimer (unsigned char NewNumber, uint32_t NewTime);
void KillTimer (unsigned char Flag);
uint32_t GetTimer (unsigned char Flag);


void I2C1_config (void);
int I2C1_write(uint8_t HW_address, uint8_t sub, uint16_t bytes_num, uint8_t* data);
int I2C1_write_byte (uint8_t HW_address, uint8_t sub, uint8_t data);
int I2C1_read (uint8_t HW_address, uint8_t sub, uint16_t bytes_num, uint8_t* data);
uint8_t I2C1_read_byte (uint8_t HW_address, uint8_t sub);

void UART1_config (uint32_t baud);
void UART1_send_byte (uint8_t data);
void UART1_send_bytes (uint8_t* data, uint32_t length);
void UART1_send_str (char* str);

void GPIO_config (void);

void ADC1_Init (void);
float Get_Battery (void);

uint32_t FLASH_ReadWord (uint32_t addr);
void First_Startup (void);

void Timer_init (uint8_t ms);

#endif

