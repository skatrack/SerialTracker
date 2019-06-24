
#include "tracker_math.h"



extern float gRes;

void filter (float new_data, float* out, float* filter_buf)
{

	float ret;

	filter_buf[FILTER_WINDOW_SIZE-1] = (0.2*new_data + 
																		0.2*filter_buf[FILTER_WINDOW_SIZE-2] + 0.1*filter_buf[FILTER_WINDOW_SIZE-3] +
																		0.1*filter_buf[FILTER_WINDOW_SIZE-4] + 0.1*filter_buf[FILTER_WINDOW_SIZE-5] +
																		0.05*filter_buf[FILTER_WINDOW_SIZE-6] + 0.05*filter_buf[FILTER_WINDOW_SIZE-7] +
																		0.05*filter_buf[FILTER_WINDOW_SIZE-8] + 0.05*filter_buf[FILTER_WINDOW_SIZE-9]);
	
	ret = filter_buf[FILTER_WINDOW_SIZE-1];

	for (int i=0; i<FILTER_WINDOW_SIZE; i++)
	{
		filter_buf[i-1] = filter_buf[i];
	}
	
	
		*out = ret;	
}

float ABS (float num)
{
	if (num < 0)
		return -num;
	else
		return num;
}

float constrain (float x, float a, float b)
{
	if (x >= a && x <= b) return x;
	else if (x < a) return a;
	else return b;
}

// Computes the dot product of two vectors
float Vector_Dot_Product(const float v1[3], const float v2[3])
{
  float result = 0;
  
  for(int c = 0; c < 3; c++)
  {
    result += v1[c] * v2[c];
  }
  
  return result; 
}

// Computes the cross product of two vectors
// out has to different from v1 and v2 (no in-place)!
void Vector_Cross_Product(float out[3], const float v1[3], const float v2[3])
{
  out[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
  out[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
  out[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

// Multiply the vector by a scalar
void Vector_Scale(float out[3], const float v[3], float scale)
{
  for(int c = 0; c < 3; c++)
  {
    out[c] = v[c] * scale; 
  }
}

// Adds two vectors
void Vector_Add(float out[3], const float v1[3], const float v2[3])
{
  for(int c = 0; c < 3; c++)
  {
    out[c] = v1[c] + v2[c];
  }
}

// Multiply two 3x3 matrices: out = a * b
// out has to different from a and b (no in-place)!
void Matrix_Multiply(const float a[3][3], const float b[3][3], float out[3][3])
{
  for(int x = 0; x < 3; x++)  // rows
  {
    for(int y = 0; y < 3; y++)  // columns
    {
      out[x][y] = a[x][0] * b[0][y] + a[x][1] * b[1][y] + a[x][2] * b[2][y];
    }
  }
}

// Multiply 3x3 matrix with vector: out = a * b
// out has to different from b (no in-place)!
void Matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3])
{
  for(int x = 0; x < 3; x++)
  {
    out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
  }
}

// Init rotation matrix using euler angles
void init_rotation_matrix(float m[3][3], float yaw, float pitch, float roll)
{
  float c1 =  cos(roll);
  float s1 =  sin(roll);
  float c2 = cos(pitch);
  float s2 = sin(pitch);
  float c3 = cos(yaw);
  float s3 = sin(yaw);
	
  // Euler angles, right-handed, intrinsic, XYZ convention
  // (which means: rotate around body axes Z, Y', X'') 
  m[0][0] = c2 * c3;
  m[0][1] = c3 * s1 * s2 - c1 * s3;
  m[0][2] = s1 * s3 + c1 * c3 * s2;

  m[1][0] = c2 * s3;
  m[1][1] = c1 * c3 + s1 * s2 * s3;
  m[1][2] = c1 * s2 * s3 - c3 * s1;

  m[2][0] = -s2;
  m[2][1] = c2 * s1;
  m[2][2] = c1 * c2;
}

float TO_RAD(float x) 
  {
    return (x * 0.01745329252); // *pi/180
  }
 
float TO_DEG(float x) 
{
  return (x * 57.2957795131);  // *180/pi
}

float  GYRO_SCALED_RAD (float x) 
{
  
	//return (x * TO_RAD( GYRO_GAIN ));
	return (x * TO_RAD( gRes ));
}

