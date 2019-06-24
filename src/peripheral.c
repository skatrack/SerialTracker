
#include "peripheral.h"

uint8_t uart3_transmitter_state = 0;
extern uint32_t timer_flag;

uint8_t I2C_rx_buf[I2C_RX_BUF_SIZE];

// UART settings
uint32_t bauds[3] =
{
  9600,
  38400,
  115200
};

struct {
	
	unsigned char Number;
	uint32_t Time;
}SoftTimer[Max_Num_Of_Timers];


void Delay_ms( uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}


void SetTimer (unsigned char NewNumber, uint32_t NewTime)			// new number это номер бита в флаговой переменной
{
	unsigned char i;
	for (i=0;i!=Max_Num_Of_Timers;i++)			// прочесываем очередь таймеров
	{
		if (SoftTimer[i].Number == NewNumber)	// если таймер с заданным номером уже есть
		{
			SoftTimer[i].Time = NewTime;				// то перезаписываем в нем время
			return;
		}
	}
	for (i=0;i!=Max_Num_Of_Timers;i++)			// иначе ищм пустой таймер 
	{																				// и записываем все в него
		if (SoftTimer[i].Number == 0)
		{
			SoftTimer[i].Number = NewNumber;	
			SoftTimer[i].Time = NewTime;
			return;			
		}
	}
}

void KillTimer (unsigned char Flag)
{
	int i;
	for (i=0;i<Max_Num_Of_Timers;i++)			// ищем заданный таймер
	{
		if (SoftTimer[i].Number == Flag)
		{
			SoftTimer[i].Number = 0;					// ставим в номер заглушку
			return;
		}
	}
}

uint32_t GetTimer (unsigned char Flag)
{
	int i;
	for (i=0;i<Max_Num_Of_Timers;i++)			// ищем заданный таймер
	{
		if (SoftTimer[i].Number == Flag)
		{
			return SoftTimer[i].Time;
		}
	}
	return 0;
}

void SysTick_Handler(void)
{
	unsigned char i;	
	if (TimingDelay != 0x00)										// реализация программной задержки
  {
    TimingDelay--;
  }
	
	for (i=0;i<Max_Num_Of_Timers;i++)						// реализация таймеров
	{
		if (SoftTimer[i].Number == 0) continue;
		if (SoftTimer[i].Time != 0) SoftTimer[i].Time--;
		else 
		{
			timer_flag	|=	(1<<SoftTimer[i].Number);
			SoftTimer[i].Number = 0;
		}
	}
}



void GPIO_config (void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_8;//  | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);	

#ifdef CALIB_BUTTON	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
#endif // CALIB_BUTTON
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
#ifdef DEV_BOARD
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);	
	
#endif

	EXTI_InitTypeDef EXTI_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;

	NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line9;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_Init(&EXTI_InitStruct);
	
	LEDG_OFF;
}

void I2C1_config (void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	
	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_OwnAddress1 = 0x0B;
	
	I2C_Init(I2C1,&I2C_InitStructure);
	I2C_Cmd(I2C1,ENABLE);
	
/*	DMA_InitTypeDef  DMA_InitStructure;
	 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&I2C1->DR; //=0x40005410 : address of data reading register of I2C1
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &I2C_rx_buf[0];
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //channel will be used for peripheral to memory transfer
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;	//setting normal mode (non circular)
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;	//medium priority
	DMA_InitStructure.DMA_BufferSize = I2C_RX_BUF_SIZE;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	//Location assigned to peripheral register will be source
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //automatic memory increment disable for peripheral
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//automatic memory increment enable for memory
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//source peripheral data size = 8bit
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	//destination memory data size = 8bit
	DMA_Init(DMA1_Channel7, &DMA_InitStructure);
*/	
}



int I2C1_write(uint8_t HW_address, uint8_t sub, uint16_t bytes_num, uint8_t* data)
{
	int ticks = I2C_TIMEOUT;			// number of flag checks before stop i2c transmition 
	
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)&&ticks) {ticks--;}
	if (ticks == 0) return -1;
	ticks = I2C_TIMEOUT;
	
	I2C_Send7bitAddress(I2C1, HW_address, I2C_Direction_Transmitter);
	while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && ticks) {ticks--;}
	if (ticks == 0) return -1;
	ticks = I2C_TIMEOUT;
	
	I2C_SendData(I2C1, sub);
	while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))&&ticks) {ticks--;}
	if (ticks == 0) return -1;
	ticks = I2C_TIMEOUT;
	
	for (int i=0; i<bytes_num; i++)
	{
		I2C_SendData(I2C1, data[i]);
		while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))&&ticks) {ticks--;}
		if (ticks == 0) return -1;
		ticks = I2C_TIMEOUT;
	}
	
	I2C_GenerateSTOP(I2C1, ENABLE);
	while((I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) && ticks) {ticks--;}
	if (ticks == 0) return -1;
}

int I2C1_write_byte (uint8_t HW_address, uint8_t sub, uint8_t data)
{	
	I2C1_write(HW_address, sub, 1, &data);
}

int I2C1_read (uint8_t HW_address, uint8_t sub, uint16_t bytes_num, uint8_t* data )
{
	int ticks = I2C_TIMEOUT;
	
	/* Disable DMA channel*/
//	DMA_Cmd(DMA1_Channel7, DISABLE);
//	DMA_SetCurrDataCounter(DMA1_Channel7, bytes_num);
	/* While the bus is busy */
	while((I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))&&ticks) {ticks--;}
	if (ticks == 0) return -1;
	ticks = I2C_TIMEOUT;
 
	/* Enable DMA NACK automatic generation */
	//I2C_DMALastTransferCmd(I2C1, ENABLE);					//Note this one, very important
	
	I2C_AcknowledgeConfig(I2C1,ENABLE);
	 
	/* Send START condition */
	I2C_GenerateSTART(I2C1, ENABLE);
	 
	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)&&ticks) {ticks--;}
	if (ticks == 0) return -1;
	ticks = I2C_TIMEOUT;
	 
	/* Send MPU6050 address for write */
	I2C_Send7bitAddress(I2C1, HW_address, I2C_Direction_Transmitter); 
	 
	/* Test on EV6 and clear it */
	while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && ticks) {ticks--;}
	if (ticks == 0) return -1;
	ticks = I2C_TIMEOUT;
	 
	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(I2C1, ENABLE);
	 
	/* Send the MPU6050's internal address to write to */
	I2C_SendData(I2C1, sub);
	 
	/* Test on EV8 and clear it */
	while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))&&ticks) {ticks--;}
	if (ticks == 0) return -1;
	ticks = I2C_TIMEOUT;
	 
	/* Send STRAT condition a second time */
	I2C_GenerateSTART(I2C1, ENABLE);
	 
	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)&&ticks) {ticks--;}
	if (ticks == 0) return -1;
	ticks = I2C_TIMEOUT;
	 
	/* Send MPU6050 address for read */
	I2C_Send7bitAddress(I2C1, HW_address, I2C_Direction_Receiver);
	 
	/* Test on EV6 and clear it */
	while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))&&ticks) {ticks--;}
	if (ticks == 0) return -1;
	ticks = I2C_TIMEOUT;
	
	for (int i=0; i<bytes_num; i++)
	{
		while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))&&ticks) {ticks--;}
		if (ticks == 0) return -1;
		ticks = I2C_TIMEOUT;
		
		data[i] = I2C1->DR;
		
	}
	I2C_AcknowledgeConfig(I2C1,DISABLE);
	
	I2C_GenerateSTOP(I2C1, ENABLE);
	while((I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))&&ticks) {ticks--;}
	if (ticks == 0) return -1;
	ticks = I2C_TIMEOUT;
/*	 
	// Start DMA to receive data from I2C 
	DMA_InitTypeDef  DMA_InitStructure;
	
	DMA_Cmd(DMA1_Channel7, ENABLE);
	I2C_DMACmd(I2C1, ENABLE);
	
	while(!(DMA_GetFlagStatus(DMA1_FLAG_TC7))&&ticks) {ticks--;}
	if (ticks == 0) return -1;
	ticks = I2C_TIMEOUT;
	
	DMA_ClearFlag(DMA1_FLAG_TC7);
	I2C_DMACmd(I2C1, DISABLE);
	
	// Send I2C1 STOP Condition 
	I2C_GenerateSTOP(I2C1, ENABLE);
	
	memcpy(data,I2C_rx_buf,bytes_num);
*/	
}


uint8_t I2C1_read_byte (uint8_t HW_address, uint8_t sub)
{
	uint8_t data;
	
	I2C1_read ( HW_address, sub, 1, &data);
	return data;
}

void UART1_config (uint32_t baud)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
		
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1,&USART_InitStructure);
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	//USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
	
	NVIC_SetPriority(USART1_IRQn,1);
	NVIC_EnableIRQ(USART1_IRQn);
	
	USART_Cmd(USART1,ENABLE);
}

void UART1_send_byte(uint8_t data)
{
	//while ((USART1->SR & (USART_SR_TXE))==0);
	while ((USART1->SR & (USART_SR_TC))==0);

	USART1->DR=(0xFF & data);
	
}
void UART1_send_bytes (uint8_t* data, uint32_t length)
{
	while (length--)
	{
		UART1_send_byte(*data++);
	}
}

void UART1_send_str (char* str)
{
	int i=0;
	while(str[i])
	{
		UART1_send_byte (str[i]);
		i++;
	}
}

void ADC1_Init (void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Cmd(ADC1, ENABLE);
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_6,1,ADC_SampleTime_55Cycles5);
	
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1));
	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

float Get_Battery (void)
{
	float level;
	unsigned short code = 0;
	for (int i=0;i<10;i++)
	{
		code += ADC_GetConversionValue(ADC1);	
	}
	code /= 10;
	
	level = (float) (code * ADC_SCALE * 2);
	
	return (level);			
}

uint32_t FLASH_ReadWord (uint32_t addr)
{
	return (*(__IO uint32_t*)addr);
}

void First_Startup (void)
{
	
	int num = 0;
	char str[30];
	int test_res = 0;
	
	num = (ADC_GetConversionValue(ADC1)) & 0x0F;
	Delay_ms(10);
	num |= ((ADC_GetConversionValue(ADC1)) & 0x0F)<<4;
	
	if (FLASH_ReadWord(NAME_FLAG_ADDR) != 0x01)
	{
		
		test_res = SelfTest();
	if (test_res != 0)
	{
		LEDG_OFF;
		Delay_ms(1500);
		LEDG_ON;
		while(1);
	}
			
#ifdef HC05
		
		sprintf(str,"AT+NAME=SkaTrack_%d\r\n",num);
		
		BT_MODE_UP;
		
		Delay_ms(100);
		
		for (int i=0; i<3; i++)
		{	
			UART1_config(bauds[i]);
			for (int k=0; k<3; k++)
			{
				UART1_send_str("AT\r\n");
				Delay_ms(200);
			}
			UART1_send_str("AT+UART=115200,0,0\r\n");
			Delay_ms(500);
			UART1_send_str(str);
			Delay_ms(500);
			USART_Cmd(USART1,DISABLE);
		}
		
		BT_MODE_DN;
#else		// HC06
		
		sprintf(str,"AT+NAMESkaTrack%d",num);

		for (int i=0; i<3; i++)
		{	
			UART1_config(bauds[i]);
			for (int k=0; k<3; k++)
			{
				UART1_send_str("AT\r\n");
				Delay_ms(200);
			}
			UART1_send_str("AT+BAUD8");
			Delay_ms(500);
			UART1_send_str(str);
			Delay_ms(500);
			USART_Cmd(USART1,DISABLE);
		}
#endif	// HC05
		FLASH_Unlock();
		FLASH_ProgramWord(NAME_FLAG_ADDR,0x01);
		FLASH_Lock();
	}
}

void Timer_init (uint8_t ms)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	
	TIM4->PSC       = RCC_Clocks.HCLK_Frequency/10000 - 1; //new clock = 1kHz
  TIM4->ARR       = (uint16_t) ms*10 - 1; //period 
  TIM4->CR1       |= TIM_CR1_DIR; //used as downcounter
  TIM4->CR1       &= ~TIM_CR1_OPM; // timer dont stops after update
  TIM4->DIER      |= TIM_DIER_UIE; //enable interrupt
	
	TIM4->CR1 			|= TIM_CR1_CEN;	// enable timer
	
	NVIC_EnableIRQ(TIM4_IRQn);
}
