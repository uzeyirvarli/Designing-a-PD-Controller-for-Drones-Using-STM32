/*
 * acc_gyro_calc.h
 *
 *  Created on: Apr 10, 2024
 *      Author: Uzeyir Varli
 */

#ifndef INC_IMU_SENSOR_ACC_GYRO_CALC_H_
#define INC_IMU_SENSOR_ACC_GYRO_CALC_H_

#include "IMU_Sensor/MahonyAHRS.h"
#include "IMU_Sensor/mpu9250.h"
#include "stm32f0xx_it.h"

#define M_PI 3.1415926


float convert_AHRS_pitch(float AHRS_pitch);
float convert_AHRS_roll(float AHRS_roll);
float convert_AHRS_yaw(float AHRS_yaw);
get_yaw_pitch_roll(float* Yaw_Pitch_Roll,float* gyro_XYZ,int GyroSize);


#endif /* INC_IMU_SENSOR_ACC_GYRO_CALC_H_ */
