
#ifndef TRACKER_MATH_H
#define TRACKER_MATH_H

#include "peripheral.h"

#define FILTER_WINDOW_SIZE			9			// dont forget to change the koeffs

enum
{
	accel_t = 0,
	magn_t,
	gyro_t,
};

void filter (float new_data, float* out, float* filter_buf);
float constrain (float x, float a, float b);
float ABS (float num);
float Vector_Dot_Product(const float v1[3], const float v2[3]);
void Vector_Cross_Product(float out[3], const float v1[3], const float v2[3]);
void Vector_Scale(float out[3], const float v[3], float scale);
void Vector_Add(float out[3], const float v1[3], const float v2[3]);
void Matrix_Multiply(const float a[3][3], const float b[3][3], float out[3][3]);
void Matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3]);
void init_rotation_matrix(float m[3][3], float yaw, float pitch, float roll);

float TO_RAD(float x);
float TO_DEG(float x);
float GYRO_SCALED_RAD (float x);

#endif

