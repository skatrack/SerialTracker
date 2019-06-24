
#include "calibration.h"

extern float accel[3], magnetom[3], gyro[3];
extern float aRes, mRes, gRes;
extern uint32_t timer_flag;
extern uint8_t long_press, short_press, key_pressed;
extern uint8_t connected;

uint8_t host_calib;
uart_t uart1;

// Sensor calibration scale and offset values
// Accelerometer
float ACCEL_X_MIN = 32000;
float ACCEL_X_MAX = -31500;
float ACCEL_Y_MIN = 32000;
float ACCEL_Y_MAX = -31500;
float ACCEL_Z_MIN = 32000;
float ACCEL_Z_MAX = -31500;
float ACCEL_X_OFFSET = 0;
float ACCEL_Y_OFFSET = 0;
float ACCEL_Z_OFFSET = 0;
float ACCEL_X_SCALE = 0;
float ACCEL_Y_SCALE = 0;
float ACCEL_Z_SCALE = 0;
float ACCEL_X_AMPLITUDE = 0;
float ACCEL_Y_AMPLITUDE = 0;
float ACCEL_Z_AMPLITUDE = 0;


// Magnetometer
float MAGN_X_MIN = 32000;
float MAGN_X_MAX = -31500;
float MAGN_Y_MIN = 32000;
float MAGN_Y_MAX = -31500;
float MAGN_Z_MIN = 32000;
float MAGN_Z_MAX = -31500;
float MAGN_X_OFFSET = 0;
float MAGN_Y_OFFSET = 0;
float MAGN_Z_OFFSET = 0;
float MAGN_X_SCALE = 0;
float MAGN_Y_SCALE = 0;
float MAGN_Z_SCALE = 0;
float MAGN_X_AMPLITUDE = 0;
float MAGN_Y_AMPLITUDE = 0;
float MAGN_Z_AMPLITUDE = 0;

// Gyroscope
float GYRO_AVERAGE_OFFSET_X = 0;
float GYRO_AVERAGE_OFFSET_Y = 0;
float GYRO_AVERAGE_OFFSET_Z = 0;

float gyro_buffer[3];
int gyro_cnt = 0;

uint8_t calibration_enabled = 0;
int calib_sensor;



int led_cnt;

void USART1_IRQHandler (void)	
{
	if (USART1->SR & USART_FLAG_ORE)
	{
		USART1->DR;
		return;
	}
	if (USART1->SR & USART_FLAG_FE)
	{
		USART1->DR;
		return;
	}
	if (USART1->SR & USART_SR_RXNE)
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		USART1->SR;
		
		uart1.rx_buf[++uart1.rx_cnt] = USART1->DR;
		uart1.rx_cnt &= UART_CNT_MASK;
		
		switch (uart1.rx_buf[uart1.rx_cnt])
		{
			case 'C':
				host_calib = 1;
				calibration_start();
				
			break;
			
			case 'N':
				if (host_calib)
					calibration_next();
			break;
				
			case 'P':
				if (host_calib)
					calibration_prev();
			break;
			
			case 'E':
				if (host_calib)
				{		
					host_calib = 0;
					calibration_enabled = 0;
					
					write_EEPROM_calib();
				}

			break;
				
			case 'S':
				if (!connected)
						connected = 1;
				
			break;
				
			case 'Q':
				if (connected)
						connected = 0;
				
			break;
			
			default:
				
			break;
			
		}
			
	}
}

// button interrupt handler
void EXTI9_5_IRQHandler()
{
	 if (EXTI_GetITStatus(EXTI_Line9) != RESET)
	 {
			if (GPIOB->IDR & GPIO_Pin_9) // Rising
			{
				if (key_pressed)
				{
					key_pressed = 0;
					short_press = 1;
				}
				else
				{
					KillTimer(button_debounce);
					timer_flag &= ~(1<<button_debounce);
				}
			}
			if (!(GPIOB->IDR & GPIO_Pin_9)) // Falling
			{
        SetTimer(button_debounce,BUTTON_DEBOUNCE);
			}
			EXTI_ClearITPendingBit(EXTI_Line9);
	 }
}




void write_EEPROM_calib(void)
{
	
	FLASH_Unlock();
	FLASH_ErasePage(CALIB_ADDR);
	
	FLASH_ProgramWord(CALIB_ADDR + 0, *(uint32_t*) &ACCEL_X_MIN);
  FLASH_ProgramWord(CALIB_ADDR + 4, *(uint32_t*) &ACCEL_X_MAX);
  FLASH_ProgramWord(CALIB_ADDR + 8, *(uint32_t*) &ACCEL_Y_MIN);
  FLASH_ProgramWord(CALIB_ADDR + 12, *(uint32_t*) &ACCEL_Y_MAX);
  FLASH_ProgramWord(CALIB_ADDR + 16, *(uint32_t*) &ACCEL_Z_MIN);
  FLASH_ProgramWord(CALIB_ADDR + 20, *(uint32_t*) &ACCEL_Z_MAX);
      
  FLASH_ProgramWord(CALIB_ADDR + 24, *(uint32_t*) &MAGN_X_MIN);
  FLASH_ProgramWord(CALIB_ADDR + 28, *(uint32_t*) &MAGN_X_MAX);
  FLASH_ProgramWord(CALIB_ADDR + 32, *(uint32_t*) &MAGN_Y_MIN);
  FLASH_ProgramWord(CALIB_ADDR + 36, *(uint32_t*) &MAGN_Y_MAX);
  FLASH_ProgramWord(CALIB_ADDR + 40, *(uint32_t*) &MAGN_Z_MIN);
  FLASH_ProgramWord(CALIB_ADDR + 44, *(uint32_t*) &MAGN_Z_MAX);  
      
  FLASH_ProgramWord(CALIB_ADDR + 48, *(uint32_t*) &GYRO_AVERAGE_OFFSET_X);
  FLASH_ProgramWord(CALIB_ADDR + 52, *(uint32_t*) &GYRO_AVERAGE_OFFSET_Y);
  FLASH_ProgramWord(CALIB_ADDR + 56, *(uint32_t*) &GYRO_AVERAGE_OFFSET_Z);
	
	FLASH_Lock();
}
	

void read_EEPROM_calib (void)
{
	uint32_t tmp32;
	// Accelerometer
  // "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
   
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 0));
	ACCEL_X_MIN = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 4));
	ACCEL_X_MAX = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 8));
  ACCEL_Y_MIN = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 12));
  ACCEL_Y_MAX = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 16));
  ACCEL_Z_MIN = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 20));
  ACCEL_Z_MAX = *(float*)&tmp32;
  
  // Magnetometer (standard calibration mode)
  // "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
	
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 24));
  MAGN_X_MIN = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 28));
  MAGN_X_MAX = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 32));
  MAGN_Y_MIN = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 36));
  MAGN_Y_MAX = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 40));
  MAGN_Z_MIN = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 44));
  MAGN_Z_MAX = *(float*)&tmp32;
  
  // Gyroscope
  // "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
	
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 48));
  GYRO_AVERAGE_OFFSET_X = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 52));
  GYRO_AVERAGE_OFFSET_Y = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 56));
  GYRO_AVERAGE_OFFSET_Z = *(float*)&tmp32;
	
}

void read_EEPROM_accel (void)
{
	uint32_t tmp32;
	// Accelerometer
  // "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
   
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 0));
	ACCEL_X_MIN = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 4));
	ACCEL_X_MAX = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 8));
  ACCEL_Y_MIN = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 12));
  ACCEL_Y_MAX = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 16));
  ACCEL_Z_MIN = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 20));
  ACCEL_Z_MAX = *(float*)&tmp32;
	
}

void read_EEPROM_magn (void)
{
	uint32_t tmp32;

  // magneter (standard calibration mode)
  // "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
	
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 24));
  MAGN_X_MIN = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 28));
  MAGN_X_MAX = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 32));
  MAGN_Y_MIN = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 36));
  MAGN_Y_MAX = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 40));
  MAGN_Z_MIN = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 44));
  MAGN_Z_MAX = *(float*)&tmp32;
  
}

void read_EEPROM_gyro (void)
{
	uint32_t tmp32;

  // Gyroscope
  // "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
	
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 48));
  GYRO_AVERAGE_OFFSET_X = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 52));
  GYRO_AVERAGE_OFFSET_Y = *(float*)&tmp32;
	tmp32 = ( FLASH_ReadWord(CALIB_ADDR + 56));
  GYRO_AVERAGE_OFFSET_Z = *(float*)&tmp32;
	
}

void calculate_offset (void)
{
  ACCEL_X_OFFSET = ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0);
  ACCEL_Y_OFFSET = ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0);
  ACCEL_Z_OFFSET = ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0);
  ACCEL_X_SCALE = ((1/aRes)/(ACCEL_X_MAX - ACCEL_X_OFFSET));
  ACCEL_Y_SCALE = ((1/aRes)/(ACCEL_Y_MAX - ACCEL_Y_OFFSET));
  ACCEL_Z_SCALE = ((1/aRes)/(ACCEL_Z_MAX - ACCEL_Z_OFFSET));
	ACCEL_X_AMPLITUDE = ACCEL_X_MAX - ACCEL_X_MIN;
	ACCEL_Y_AMPLITUDE = ACCEL_Y_MAX - ACCEL_Y_MIN;
	ACCEL_Z_AMPLITUDE = ACCEL_Z_MAX - ACCEL_Z_MIN;
  
  MAGN_X_OFFSET = ((MAGN_X_MIN + MAGN_X_MAX) / 2.0);
  MAGN_Y_OFFSET = ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0);
  MAGN_Z_OFFSET = ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0);
  MAGN_X_SCALE = (1/mRes/(MAGN_X_MAX - MAGN_X_OFFSET));
  MAGN_Y_SCALE = (1/mRes/(MAGN_Y_MAX - MAGN_Y_OFFSET));
  MAGN_Z_SCALE = (1/mRes/(MAGN_Z_MAX - MAGN_Z_OFFSET));
	//MAGN_X_SCALE = (100/(MAGN_X_MAX - MAGN_X_OFFSET));
  //MAGN_Y_SCALE = (100/(MAGN_Y_MAX - MAGN_Y_OFFSET));
  //MAGN_Z_SCALE = (100/(MAGN_Z_MAX - MAGN_Z_OFFSET));
	
	MAGN_X_AMPLITUDE = MAGN_X_MAX - MAGN_X_MIN;
	MAGN_Y_AMPLITUDE = MAGN_Y_MAX - MAGN_Y_MIN;
	MAGN_Z_AMPLITUDE = MAGN_Z_MAX - MAGN_Z_MIN;
	
	if (ACCEL_X_AMPLITUDE > ACCEL_MAX_AMPLITUDE ||
			ACCEL_Y_AMPLITUDE > ACCEL_MAX_AMPLITUDE ||
			ACCEL_Z_AMPLITUDE > ACCEL_MAX_AMPLITUDE)
	{
		ACCEL_X_MIN = 3000;
		ACCEL_X_MAX = -2500;
		ACCEL_Y_MIN = 3000;
		ACCEL_Y_MAX = -2500;
		ACCEL_Z_MIN = 3000;
		ACCEL_Z_MAX = -2500;
	}
	
	if (MAGN_X_AMPLITUDE > MAGN_MAX_AMPLITUDE ||
			MAGN_Y_AMPLITUDE > MAGN_MAX_AMPLITUDE ||
			MAGN_Z_AMPLITUDE > MAGN_MAX_AMPLITUDE)
	{
		MAGN_X_MIN = 3000;
		MAGN_X_MAX = -2500;
		MAGN_Y_MIN = 3000;
		MAGN_Y_MAX = -2500;
		MAGN_Z_MIN = 3000;
		MAGN_Z_MAX = -2500;
	}
}

// Apply calibration to raw sensor readings
void compensate_sensor_errors(void) 
{
    // Compensate accelerometer error
    accel[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    accel[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    accel[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

    // Compensate magnetometer error
    magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
    magnetom[1] = (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
    magnetom[2] = (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;

    // Compensate gyroscope error
		gyro[0] -= GYRO_AVERAGE_OFFSET_X;
   	gyro[1] -= GYRO_AVERAGE_OFFSET_Y;
    gyro[2] -= GYRO_AVERAGE_OFFSET_Z;
//    gyro[0] = GYRO_SCALED_RAD(gyro[0]- GYRO_AVERAGE_OFFSET_X);
//   	gyro[1] = GYRO_SCALED_RAD(gyro[1]- GYRO_AVERAGE_OFFSET_Y);
//    gyro[2] = GYRO_SCALED_RAD(gyro[2]- GYRO_AVERAGE_OFFSET_Z);
}

void calibration_start (void) 
{
	
    calibration_enabled = 1;
        
    FLASH_Unlock();
		FLASH_ErasePage(CALIB_ADDR);
		
		FLASH_ProgramWord(CALIB_ADDR + 0,0x46FA0000);		// float 32000
		FLASH_ProgramWord(CALIB_ADDR + 4,0xC6F61800);		// float -31500
		FLASH_ProgramWord(CALIB_ADDR + 8,0x46FA0000);
		FLASH_ProgramWord(CALIB_ADDR + 12,0xC6F61800);
		FLASH_ProgramWord(CALIB_ADDR + 16,0x46FA0000);
		FLASH_ProgramWord(CALIB_ADDR + 20,0xC6F61800);
				
		FLASH_ProgramWord(CALIB_ADDR + 24,0x46FA0000);
		FLASH_ProgramWord(CALIB_ADDR + 28,0xC6F61800);
		FLASH_ProgramWord(CALIB_ADDR + 32,0x46FA0000);
		FLASH_ProgramWord(CALIB_ADDR + 36,0xC6F61800);
		FLASH_ProgramWord(CALIB_ADDR + 40,0x46FA0000);
		FLASH_ProgramWord(CALIB_ADDR + 44,0xC6F61800);  
				
		FLASH_ProgramWord(CALIB_ADDR + 48,0);
		FLASH_ProgramWord(CALIB_ADDR + 52,0);
		FLASH_ProgramWord(CALIB_ADDR + 56,0);
	
		FLASH_Lock();

		read_EEPROM_calib();
		
		gyro_cnt=0;
    gyro_buffer[0]=0;
    gyro_buffer[1]=0;
    gyro_buffer[2]=0;
				
    calib_sensor = 0;
    
		KillTimer(led);
		SetTimer(led, LED_CALIB_PERIOD);
		SetTimer(cal_max, CAL_MAX_TIME);
}

void calibration_next (void)
{
	if (calibration_enabled)
    {
			// reset timer
			KillTimer(cal_max);
			SetTimer(cal_max, CAL_MAX_TIME);
				
      calib_sensor++;
      if (calib_sensor >= 3) 
      { 
        calib_sensor = 0;
        calibration_enabled = 0;
				host_calib = 0;
				
				write_EEPROM_calib();
				
				KillTimer(cal_max);
        SetTimer(led, LED_JOB_PERIOD);
      }
			else
			{
				LEDG_ON;
				Delay_ms(500);
				LEDG_OFF;
			}
    }
    else calib_sensor = 0;

}

void calibration_prev (void)
{
	
	
	if (calibration_enabled)
    {
			// reset timer
			KillTimer(cal_max);
			SetTimer(cal_max, CAL_MAX_TIME);

			LEDG_ON;
      Delay_ms(500);
      LEDG_OFF;
			
      if (calib_sensor > 0) 
      { 
        calib_sensor--;
				
				if (calib_sensor == 0) 
					read_EEPROM_calib();
				else if (calib_sensor == 1) 
				{
					read_EEPROM_magn();
					read_EEPROM_gyro();
				}
				else if (calib_sensor == 2) 
				{
					read_EEPROM_gyro();
				}
				
				gyro_cnt=0;
        gyro_buffer[0]=0;
        gyro_buffer[1]=0;
        gyro_buffer[2]=0;
      }
    }
    else calib_sensor = 0;
}

void calibration_loop (void)
{
	
	if (timer_flag & (1<<cal_max))
	{
		timer_flag &= ~ (1<<cal_max);
		
		calibration_enabled = 0;
		host_calib = 0;
		
		write_EEPROM_calib();	
	}
	
	if (!calibration_enabled && short_press)
	{
		short_press = 0;
	} 
		
	else if (!calibration_enabled && long_press)
	{
		long_press = 0;
		short_press = 0;
		
		if (!host_calib)
			calibration_start();
	} 
	
	else if (calibration_enabled && short_press)
	{
		long_press = 0;
		short_press = 0;
		
		if (!host_calib)
			calibration_next();
	}
	else if (calibration_enabled && long_press)
	{		
		long_press = 0;
		short_press = 0;
		
		if (!host_calib)
		{
			LEDG_OFF;
			Delay_ms(1000);
			LEDG_ON;
					
			calibration_start();
		}
	}
		
	if (calibration_enabled) // calibration body
  {			
		
    read_sensors();
		calculate_offset();
		
     switch (calib_sensor)
     {
       case 0:   // accel
      
         if (accel[0]>ACCEL_X_MAX) ACCEL_X_MAX=accel[0];
         else if (accel[0]<ACCEL_X_MIN) ACCEL_X_MIN=accel[0];
         
         if (accel[1]>ACCEL_Y_MAX) ACCEL_Y_MAX=accel[1];
         else if (accel[1]<ACCEL_Y_MIN) ACCEL_Y_MIN=accel[1];
         
         if (accel[2]>ACCEL_Z_MAX) ACCEL_Z_MAX=accel[2];
         else if (accel[2]<ACCEL_Z_MIN) ACCEL_Z_MIN=accel[2]; 
  
       break;
       case 1:   // magnetometer
       
         if (magnetom[0]>MAGN_X_MAX) MAGN_X_MAX=magnetom[0];
         else if (magnetom[0]<MAGN_X_MIN) MAGN_X_MIN=magnetom[0];
         
         if (magnetom[1]>MAGN_Y_MAX) MAGN_Y_MAX=magnetom[1];
         else if (magnetom[1]<MAGN_Y_MIN) MAGN_Y_MIN=magnetom[1];
         
         if (magnetom[2]>MAGN_Z_MAX) MAGN_Z_MAX=magnetom[2];
         else if (magnetom[2]<MAGN_Z_MIN) MAGN_Z_MIN=magnetom[2];  
       break;
       case 2:   // gyroscope;
          //Serial.println(1);  //test output
          for (int i = 0; i < 3; i++)
          gyro_buffer[i] += gyro[i];
          gyro_cnt++;          
          GYRO_AVERAGE_OFFSET_X = gyro_buffer[0] / (float) gyro_cnt;
          GYRO_AVERAGE_OFFSET_Y = gyro_buffer[1] / (float) gyro_cnt;
          GYRO_AVERAGE_OFFSET_Z = gyro_buffer[2] / (float) gyro_cnt;        
       break;       
     }
     
#ifdef DEBUG_OUTPUT
			 
			 char tmp[20];
				
       UART1_send_str("ACCEL: ");
			 sprintf(tmp, "%.1f", ACCEL_X_MIN);
       UART1_send_str(tmp);
       UART1_send_str("/");
       sprintf(tmp, "%.1f", ACCEL_X_MAX);
		   UART1_send_str(tmp);
       UART1_send_str("; ");
       sprintf(tmp, "%.1f", ACCEL_Y_MIN);
       UART1_send_str(tmp);
			 UART1_send_str("/");
       sprintf(tmp, "%.1f", ACCEL_Y_MAX);
		   UART1_send_str(tmp);
       UART1_send_str("; ");
       sprintf(tmp, "%.1f", ACCEL_Z_MIN);
		   UART1_send_str(tmp);
       UART1_send_str("/");
       sprintf(tmp, "%.1f", ACCEL_Z_MAX);
			 UART1_send_str(tmp);
       UART1_send_str("  ");

       UART1_send_str("MAGN: ");
       sprintf(tmp, "%.1f", MAGN_X_MIN);
			 UART1_send_str(tmp);
       UART1_send_str("/");
       sprintf(tmp, "%.1f", MAGN_X_MAX);
			 UART1_send_str(tmp);
       UART1_send_str("; ");
       sprintf(tmp, "%.1f", MAGN_Y_MIN);
			 UART1_send_str(tmp);
       UART1_send_str("/");
       sprintf(tmp, "%.1f", MAGN_Y_MAX);
			 UART1_send_str(tmp);
       UART1_send_str("; ");
       sprintf(tmp, "%.1f", MAGN_Z_MIN);
			 UART1_send_str(tmp);
       UART1_send_str("/");
       sprintf(tmp, "%.1f", MAGN_Z_MAX);
			 UART1_send_str(tmp);
       UART1_send_str("  ");

UART1_send_str("GYRO: ");
       sprintf(tmp, "%.2f", GYRO_AVERAGE_OFFSET_X);
			 UART1_send_str(tmp);
       UART1_send_str("/");
       sprintf(tmp, "%.2f", GYRO_AVERAGE_OFFSET_Y);
			 UART1_send_str(tmp);
       UART1_send_str("/");
       sprintf(tmp, "%.2f", GYRO_AVERAGE_OFFSET_Z);
			 UART1_send_str(tmp);
       UART1_send_str("\r\n");
#endif       
  }
}



