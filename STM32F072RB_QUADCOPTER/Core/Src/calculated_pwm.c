/*
 * calculated_pwm.c
 *
 *  Created on: Feb 4, 2024
 *      Author: uzeyir
 */



#include "stm32f0xx_hal.h"
#include "main.h"
#include "Hardware/hardware.h"
#include "Hardware/adc.h"

float p0=1385;
float p1=2.870;
float p2=-29.42;
uint16_t read_adc_value1;
/*
float read_battery_voltaj1()
{
	float battery1;
	float R1 = 5.0; // resistance of R1 (100K) -see text!
	float R2 = 1.0;
	read_adc_value1=read_battery_voltajs();
	battery1=(((float)read_adc_value1)/4096)*3.3;
	battery1=battery1/(R2/(R1+R2))+0.1;
	return battery1;
}
uint16_t calculated_pwm(float desired_force, float read_voltaj)
{
	int pwm;
	read_voltaj=read_battery_voltaj1();
	pwm=p0+p1*desired_force+p2*read_voltaj;
	return pwm;

}*/
