
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
  hat.gyro[0]=gyro[0];
  hat.gyro[1]=gyro[1];
  hat.gyro[2]=gyro[2];
	hat.accel[0]=accel[0];
  hat.accel[1]=accel[1];
  hat.accel[2]=accel[2];
	hat.timestamp = uptime;
	
  UART1_send_bytes( (uint8_t*) &hat,sizeof(hat));
}



