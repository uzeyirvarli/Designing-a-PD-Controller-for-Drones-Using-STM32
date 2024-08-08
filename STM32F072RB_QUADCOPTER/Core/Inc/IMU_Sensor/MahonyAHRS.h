/*
 * MahonyAHRS.h
 *
 *  Created on: Apr 10, 2024
 *      Author: Uzeyir Varli
 */

#ifndef INC_HARDWARE_MAHONYAHRS_H_
#define INC_HARDWARE_MAHONYAHRS_H_


#include <math.h>

void Mahony_begin(float sampleFrequency);
void Mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void Mahony_updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
float Mahony_getRoll();
float Mahony_getPitch();
float Mahony_getYaw();
float Mahony_getRollRadians();
float Mahony_getPitchRadians();
float Mahony_getYawRadians();

static float Mahony_invSqrt(float x);
void Mahony_computeAngles();
#endif /* INC_HARDWARE_MAHONYAHRS_H_ */
