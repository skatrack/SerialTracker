
#include "serial.h"

extern float accel[3], gyro[3], uptime;
data_t hat;


//att_t att;
void SerialSetup()
{
  hat.Begin=0xAAAA;
  hat.End=0x5555;
}


void SerialSendPacket()
{
	uint8_t buffer[32];
	uint8_t i=0;
	buffer[i++]=0xAA;
	buffer[i++]=0xAA;
	
	buffer[i++]= (*(uint32_t*) &gyro[0]);
	buffer[i++]= (*(uint32_t*) &gyro[0])>>8;
	buffer[i++]= (*(uint32_t*) &gyro[0])>>16;
  buffer[i++]= (*(uint32_t*) &gyro[0]) >> 24;

	buffer[i++]= (*(uint32_t*) &gyro[3]);
	buffer[i++]= (*(uint32_t*) &gyro[2])>>8;
	buffer[i++]= (*(uint32_t*) &gyro[1])>>16;
  buffer[i++]= (*(uint32_t*) &gyro[1])>>24;
	
	buffer[i++]= (*(uint32_t*) &gyro[2]);
	buffer[i++]= (*(uint32_t*) &gyro[2])>>8;
	buffer[i++]= (*(uint32_t*) &gyro[2])>>16;
  buffer[i++]= (*(uint32_t*) &gyro[2])>>24;
	
	buffer[i++]= (*(uint32_t*) &accel[0]);
	buffer[i++]= (*(uint32_t*) &accel[0])>>8;
	buffer[i++]= (*(uint32_t*) &accel[0])>>16;
	buffer[i++]= (*(uint32_t*) &accel[0])>>24;
	
	buffer[i++]= (*(uint32_t*) &accel[3]);
	buffer[i++]= (*(uint32_t*) &accel[2])>>8;
	buffer[i++]= (*(uint32_t*) &accel[1])>>16;
  buffer[i++]= (*(uint32_t*) &accel[1])>>24;
	
	buffer[i++]= (*(uint32_t*) &accel[2]);
	buffer[i++]= (*(uint32_t*) &accel[2])>>8;
	buffer[i++]= (*(uint32_t*) &accel[2])>>16;
  buffer[i++]= (*(uint32_t*) &accel[2])>>24;
	
	buffer[i++]= (*(uint32_t*) &uptime);
	buffer[i++]= (*(uint32_t*) &uptime)>>8;
	buffer[i++]= (*(uint32_t*) &uptime)>>16;
	buffer[i++]= (*(uint32_t*) &uptime)>>24;
	
	buffer[i++]=0x55;
	buffer[i++]=0x55;
	
  UART1_send_bytes( (uint8_t*) &buffer, sizeof(buffer));
}



