
#ifndef SERIAL_H
#define SERIAL_H

#include "peripheral.h"

typedef struct
{
  uint16_t  Begin;	   	// 2  	0xAAAA
  float     gyro[3];   	// 12  	gyro
  float     accel[3];   // 12 	accel
	float 	  timestamp;	// 4 		timestamp
  uint16_t  End;      	// 2  	0x5555
} 
data_t;

void SerialSetup(void);
void SerialSendPacket(void);



#endif
