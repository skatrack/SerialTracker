
#ifndef CALIBRATION_H
#define CALIBRATION_H


#include "peripheral.h"
#include "sensor.h"

#define CALIB_ADDR									0x08007C00
#define CALIB_PAGE									31


#ifdef MPU_9250
	#define ACCEL_MIN_AMPLITUDE				0	
	#define ACCEL_MAX_AMPLITUDE				5000
	#define MAGN_MIN_AMPLITUDE				0	
	#define MAGN_MAX_AMPLITUDE				10000
#else
	#define ACCEL_MIN_AMPLITUDE				0
	#define ACCEL_MAX_AMPLITUDE				700
	#define MAGN_MIN_AMPLITUDE				0
	#define MAGN_MAX_AMPLITUDE				8000
#endif

void write_EEPROM_calib(void);
void read_EEPROM_calib (void);

void calculate_offset (void);
void compensate_sensor_errors(void);

void calibration_start (void);
void calibration_next (void);
void calibration_prev(void);
void calibration_loop (void);





#endif


