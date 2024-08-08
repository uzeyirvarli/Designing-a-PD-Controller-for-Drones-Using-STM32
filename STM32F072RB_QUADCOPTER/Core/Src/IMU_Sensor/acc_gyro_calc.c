/*
 * acc_gyro_calc.c
 *
 *  Created on: Apr 10, 2024
 *      Author: Uzeyir Varli
 */



#include "IMU_Sensor/acc_gyro_calc.h"
#include "IMU_Sensor/mpu9250.h"

#define PI 3.1415926
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll,angle_yaw;
float acc_x = 0;//MPU9250_getAccelX_mss();
float acc_y = 0;//MPU9250_getAccelY_mss();
float acc_z = 0;//MPU9250_getAccelZ_mss();

float gyro_x = 0;//(-1)*MPU9250_getGyroX_rads();
float gyro_y = 0;//(-1)*MPU9250_getGyroY_rads();
float gyro_z = 0;//MPU9250_getGyroZ_rads();

float mag_x = 0;//MPU9250_getMagX_uT();
float mag_y = 0;//MPU9250_getMagY_uT();
float mag_z = 0;//MPU9250_getMagZ_uT();

float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values

int16_t AccData[3], GyroData[3], MagData[3];
float magCalibration[3], magbias[3],_mag_adjust[3];
float now,delta_t;
float lastUpdate;
static float yaw_angle=0;
float convert_AHRS_pitch(float AHRS_pitch)
{
	float pitch = AHRS_pitch*(-1);
	return AHRS_pitch;
}

float convert_AHRS_roll(float AHRS_roll)
{
	float roll = 0;
	if(AHRS_roll > 0)
		roll = AHRS_roll - 180;
	else if(AHRS_roll < 0)
		roll = AHRS_roll + 180;
	return AHRS_roll;
}

float convert_AHRS_yaw(float AHRS_yaw)
{
	float yaw = 0;
	if(AHRS_yaw > 180)
		yaw = 360 - AHRS_yaw;
	else if(AHRS_yaw < 180)
		yaw = AHRS_yaw*(-1);
	//return yaw;
	return AHRS_yaw;
}



// Get new IMU data
void get_acc_x_y_z()
{

}

get_yaw_pitch_roll(float* Yaw_Pitch_Roll,float* gyro_XYZ,int GyroSize)
{

	MPU9250_GetAccData(AccData);
	MPU9250_GetGyroData(GyroData);
	MPU9250_GetMagData(MagData);
	float aRes = 2.0f/32768.0f;
	float gRes = 250.0f/32768.0f;
	float mRes = 10.0*4912.0/32760.0; // Proper scale to return milliGauss
	magbias[0] = +491.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
	magbias[1] = -495.;  // User environmental x-axis correction in milliGauss
	magbias[2] = -1015.;
	readAK8963Registers(0x01,3,_mag_adjust);
	ax = (float)AccData[0]*aRes;
	ay = (float)AccData[1]*aRes;
	az = (float)AccData[2]*aRes;
	gx = (float)GyroData[0]*gRes*PI/180.0;  // get actual gyro value, this depends on scale being set
	gy = (float)GyroData[1]*gRes*PI/180.0;
	gz = (float)GyroData[2]*gRes*PI/180.0;

	acc_x = ax/9.81f;
	acc_y = ay/9.81f;
	acc_z = az/9.81f;
	gyro_x = gx*57.29578f;
	gyro_y = gy*57.29578f;
	gyro_z = gz*57.29578f;
	/*mag_x = (float)MagData[0]*mRes*_mag_adjust[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
	mag_y = (float)MagData[1]*mRes*_mag_adjust[1] - magbias[1];
	mag_z = (float)MagData[2]*mRes*_mag_adjust[2] - magbias[2];*/


	Mahony_update(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, mag_x, mag_y, mag_z);

	if ((gyro_z > -2.5 && gyro_z <2.5) ) gyro_z=0;
	if ((gyro_y > -2 && gyro_y <-1 )) gyro_y=0;
	if ((gyro_x > 0 && gyro_x < 1.5 )) gyro_x=0;


	if (yaw_angle>360) yaw_angle=yaw_angle-360;
	if (yaw_angle < -360) yaw_angle=yaw_angle+360.0;
	angle_pitch = convert_AHRS_pitch(Mahony_getPitch());
	angle_roll = convert_AHRS_roll(Mahony_getRoll());
	angle_yaw = convert_AHRS_yaw(Mahony_getYaw());

    now=HAL_GetTick();
    delta_t=lastUpdate-now;
    if (delta_t<0) delta_t=-delta_t;
    lastUpdate=now;
    if (gyro_z<0) gyro_z=gyro_z*0.95;
    if (gyro_z>0) gyro_z=gyro_z*1.07;
    yaw_angle=yaw_angle+gyro_z*(delta_t/1000);
	Yaw_Pitch_Roll[0]=yaw_angle ;
	Yaw_Pitch_Roll[1]=angle_pitch;
	Yaw_Pitch_Roll[2]=angle_roll ;
	gyro_XYZ[0]=gyro_x;
	gyro_XYZ[1]=gyro_y;
	gyro_XYZ[2]=gyro_z;

}








