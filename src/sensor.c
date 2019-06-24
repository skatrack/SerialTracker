
#include "sensor.h"

extern float accel[3], magnetom[3], gyro[3];
extern float aRes, mRes, gRes;

float accel0_filter[15];
float accel1_filter[15];
float accel2_filter[15];
float magn0_filter[15];
float magn1_filter[15];
float magn2_filter[15];

#ifdef MPU_9250

// Specify sensor full scale
uint8_t Gscale = GFS_2000DPS;
uint8_t Ascale = AFS_16G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read





void initMPU9250()
{
// wake up device
	I2C1_write_byte( MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Reset
	Delay_ms(10);
  I2C1_write_byte( MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
  Delay_ms(10); // Wait for all registers to reset 

 // get stable time source
  I2C1_write_byte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  Delay_ms(20);


 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
 // minimum Delay_ms time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  I2C1_write_byte(MPU9250_ADDRESS, CONFIG, 0x03);  

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  I2C1_write_byte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
                                    // determined inset in CONFIG above
 
 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = I2C1_read_byte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x02; // Clear Fchoice bits [1:0] 
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  I2C1_write_byte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

  // Set accelerometer full-scale range configuration
  c = I2C1_read_byte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer 
  I2C1_write_byte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = I2C1_read_byte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  I2C1_write_byte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  
  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled as master
  
   I2C1_write_byte(MPU9250_ADDRESS, INT_PIN_CFG, 0x32);    
   I2C1_write_byte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
	 I2C1_write_byte(MPU9250_ADDRESS, SIGNAL_PATH_RESET, 0x07);
 }

 void initAK8963()
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  I2C1_write_byte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  Delay_ms(10);
  I2C1_write_byte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  Delay_ms(10);
  I2C1_read(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  
  //destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
  //destination[1] =  (float)(rawData[1] - 128)/256. + 1.;  
  //destination[2] =  (float)(rawData[2] - 128)/256. + 1.; 
  
  I2C1_write_byte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  Delay_ms(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  I2C1_write_byte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
}

int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  I2C1_read(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

#else	// !MPU_9250

void Accel_Init(void)
{
	I2C1_write_byte(ACCEL_ADDR, 0x2D, 0x08);
  Delay_ms(5);
	I2C1_write_byte(ACCEL_ADDR, 0x31, 0x08);
  Delay_ms(5);
  
  // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwidth)
	I2C1_write_byte(ACCEL_ADDR, 0x2C, 0x09);
  Delay_ms(5);
}

void Magn_Init(void)
{
	I2C1_write_byte(MAGN_ADDR, 0x02, 0x00);
  Delay_ms(5);

	I2C1_write_byte(MAGN_ADDR, 0x00, 0x18);
  Delay_ms(5);
}

void Gyro_Init(void)
{
  // Power up reset defaults
	I2C1_write_byte(GYRO_ADDR, 0x3E, 0x80);
  Delay_ms(5);
  
  // Select full-scale range of the gyro sensors
  // Set LP filter bandwidth to 42Hz
	I2C1_write_byte(GYRO_ADDR, 0x16, 0x1B);
  Delay_ms(5);
  
  // Set sample rato to 50Hz
	I2C1_write_byte(GYRO_ADDR, 0x15, 0x0A);
  Delay_ms(5);

  // Set clock to PLL with z gyro reference
	I2C1_write_byte(GYRO_ADDR, 0x3E, 0x00);
  Delay_ms(5);
}


#endif	//MPU_9250

// Reads x, y and z accelerometer registers
void Read_Accel()
{
  	uint8_t rawData[6];  // x/y/z accel register data stored here

#ifdef MPU_9250
  
	I2C1_read(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  
  // No multiply by -1 for coordinate system transformation here, because of double negation:
  // We want the gravity vector, which is negated acceleration vector.
  
	accel[0] = (int16_t) (rawData[0] << 8) | rawData[1] ;  
  accel[1] = (int16_t) (rawData[2] << 8) | rawData[2] ;  
  accel[2] = (int16_t) (-1*(rawData[4] << 8) | rawData[5]) ;
	
#else		// GY_85
	
	I2C1_read(ACCEL_ADDR, 0x32, 6, &rawData[0]);
	
	accel[0] = (int16_t) (rawData[3] << 8) | rawData[2] ;  
  accel[1] = (int16_t) (rawData[1] << 8) | rawData[0] ;  
  accel[2] = (int16_t) (rawData[5] << 8) | rawData[4] ;

#endif 

#ifdef FILTER_ACCEL

	filter(accel[0],&accel[0],accel0_filter);
	filter(accel[1],&accel[1],accel1_filter);
	filter(accel[2],&accel[2],accel2_filter);
	
#endif
}

void Read_Magn()
{
   uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  
#ifdef MPU_9250
	
	if(I2C1_read_byte(AK8963_ADDRESS, AK8963_ST1) & 0x01) 
  { // wait for magnetometer data ready bit to be set
		I2C1_read(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
		uint8_t c = rawData[6]; // End data read by reading ST2 register
    
		if(!(c & 0x08)) 
		{ // Check if magnetic sensor overflow set, if not then report data

      // MSB byte first, then LSB; Y and Z reversed: X, Z, Y

			magnetom[0] = (int16_t) (-1 * ((rawData[3] << 8) | rawData[2])) ;  
			magnetom[1] = (int16_t) (-1 * ((rawData[1] << 8) | rawData[0])) ;  
			magnetom[2] = (int16_t) (-1 * ((rawData[5] << 8) | rawData[4])) ;			
    } 
  }
	
#else		// GY_85
	
	I2C1_read(MAGN_ADDR, 0x03, 6, &rawData[0]);
	
	magnetom[0] = (int16_t) (-1 * ((rawData[4] << 8) | rawData[5])) ;  
  magnetom[1] = (int16_t) (-1 * ((rawData[0] << 8) | rawData[1])) ;  
  magnetom[2] = (int16_t) (-1 * ((rawData[2] << 8) | rawData[3])) ;
	
#endif
	
#ifdef FILTER_MAGNETOMETER

	filter(magnetom[0],&magnetom[0],magn0_filter);
	filter(magnetom[1],&magnetom[1],magn1_filter);
	filter(magnetom[2],&magnetom[2],magn2_filter);
	
#endif
	
}


// Reads x, y and z gyroscope registers
void Read_Gyro()
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
	
#ifdef MPU_9250
	
  I2C1_read(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	
	gyro[0] = (int16_t) (-1 * ((rawData[2] << 8) | rawData[3])) ;  // Turn the MSB and LSB into a signed 16-bit value
  gyro[1] = (int16_t) (-1 * ((rawData[0] << 8) | rawData[1])) ;  
  gyro[2] = (int16_t) (-1 * ((rawData[4] << 8) | rawData[5])) ; 
	
#else		// GY_85
	
	I2C1_read(GYRO_ADDR, 0x1D, 6, &rawData[0]);
	
	gyro[0] = (int16_t) (-1 * ((rawData[2] << 8) | rawData[3])) ;  // Turn the MSB and LSB into a signed 16-bit value
  gyro[1] = (int16_t) (-1 * ((rawData[0] << 8) | rawData[1])) ;  
  gyro[2] = (int16_t) (-1 * ((rawData[4] << 8) | rawData[5])) ; 
	
#endif
}

void read_sensors (void)
{
	Read_Gyro(); // Read gyroscope
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
}

void getMres() 
{

#ifdef MPU_9250
  switch (Mscale)
  {
   // Possible magnetometer scales (and their register bit settings) are:
  // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          mRes = 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
  }
	
#else
	
	mRes = 1.0/100.0;
	
#endif
}

void getGres() 
{
#ifdef MPU_9250
  switch (Gscale)
  {
  // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
					
          break;
  }
	gRes = 0.06;
	
#else
	
	gRes = 0.06957;
	
#endif
	
}

void getAres() 
{
#ifdef MPU_9250
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;				// 0.00006103515625
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;				// 0.0001220703125
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;				// 0.000244140625
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;			// 0.00048828125
          break;
  }
#else
	
	aRes = 1.0/256.0;
	
#endif
	
}

int SelfTest (void)
{
	int ret = 0;
	int16_t accel_test[3], magnetom_test[3], gyro_test[3];
	
	read_sensors();
	
  for (int i=0; i<3; i++)
	{
		accel_test[i]=accel[i];
		magnetom_test[i]=magnetom[i];
		gyro_test[i]=gyro[i];
	}
	
	Delay_ms(1000);
	
	read_sensors();
	
	for (int i=0; i<3; i++)
	{
		if (accel_test[i]==accel[i])
			ret = 1;
		if (magnetom_test[i]==magnetom[i])
			ret = 2;
		if (gyro_test[i]==gyro[i])
			ret = 3;
	}
	return ret;
}

