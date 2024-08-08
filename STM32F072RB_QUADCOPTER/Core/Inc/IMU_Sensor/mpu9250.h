/*
 * mpu9250.h
 *
 *  Created on: Apr 10, 2024
 *      Author: Uzeyir Varli
 */

#ifndef INC_IMU_SENSOR_MPU9250_H_
#define INC_IMU_SENSOR_MPU9250_H_

#include <stdint.h>
#include <math.h>
#include "IMU_Sensor/mpu9250Config.h"
#define RAD2DEG 57.2957795131

typedef enum GyroRange_ {
	GYRO_RANGE_250DPS = 0,
	GYRO_RANGE_500DPS,
	GYRO_RANGE_1000DPS,
	GYRO_RANGE_2000DPS
} GyroRange;

typedef enum AccelRange_ {
	ACCEL_RANGE_2G = 0,
	ACCEL_RANGE_4G,
	ACCEL_RANGE_8G,
	ACCEL_RANGE_16G
} AccelRange;

typedef enum DLPFBandwidth_ {
	DLPF_BANDWIDTH_184HZ = 0,
	DLPF_BANDWIDTH_92HZ,
	DLPF_BANDWIDTH_41HZ,
	DLPF_BANDWIDTH_20HZ,
	DLPF_BANDWIDTH_10HZ,
	DLPF_BANDWIDTH_5HZ
} DLPFBandwidth;

typedef enum SampleRateDivider_ {
	LP_ACCEL_ODR_0_24HZ = 0,
	LP_ACCEL_ODR_0_49HZ,
	LP_ACCEL_ODR_0_98HZ,
	LP_ACCEL_ODR_1_95HZ,
	LP_ACCEL_ODR_3_91HZ,
	LP_ACCEL_ODR_7_81HZ,
	LP_ACCEL_ODR_15_63HZ,
	LP_ACCEL_ODR_31_25HZ,
	LP_ACCEL_ODR_62_50HZ,
	LP_ACCEL_ODR_125HZ,
	LP_ACCEL_ODR_250HZ,
	LP_ACCEL_ODR_500HZ
} SampleRateDivider;




typedef struct MPU9250_t{
float q[4];
float deltat;
float eInt[3];
uint32_t lastUpdate;
}MPU9250_t;



// parameters for 6 DoF sensor fusion calculations
#define beta 	0.9069                             // sqrt(0.75) * GyroMeasError and GyroMeasError = PI * (60.0f / 180.0f)
#define zeta 	0.015115                           // 0.866 * GyroMeasDrift, and GyroMeasDrift = PI * (1.0f / 180.0f)
// these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

#define Kp 2.0f * 5.0f

uint8_t MPU9250_Init();
/* read the data, each argiment should point to a array for x, y, and x */
//void MPU9250_GetData(int16_t* AccData, int16_t* MagData, int16_t* GyroData);

/* sets the sample rate divider to values other than default */
void MPU9250_SetSampleRateDivider(SampleRateDivider srd);
/* sets the DLPF bandwidth to values other than default */
void MPU9250_SetDLPFBandwidth(DLPFBandwidth bandwidth);
/* sets the gyro full scale range to values other than default */
void MPU9250_SetGyroRange(GyroRange range);
/* sets the accelerometer full scale range to values other than default */
void MPU9250_SetAccelRange(AccelRange range);
void MPU9250_GetAccData(int16_t* AccData);
void MPU9250_GetGyroData(int16_t* GyroData);
void MPU9250_GetMagData(int16_t* MagData);



void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float * q, float deltat);


// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
// measured ones.
//void MahonyQuaternionUpdate(MPU9250_t * dest)
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float * q, float deltat);

#endif /* INC_IMU_SENSOR_MPU9250_H_ */
