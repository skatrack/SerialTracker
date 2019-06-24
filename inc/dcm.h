
#ifndef DCM_H
#define DCM_H

#include "peripheral.h"
#include "sensor.h"
#include "freetrack_math.h"


void Normalize(void);
void Drift_correction(void);
void Matrix_update(void);
void Euler_angles(void);
void reset_sensor_fusion(void);
void reset_yaw(void);

#endif



