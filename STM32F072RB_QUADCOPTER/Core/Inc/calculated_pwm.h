/*
 * calculated_pwm.h
 *
 *  Created on: Feb 4, 2024
 *      Author: uzeyir
 */

#ifndef INC_CALCULATED_PWM_H_
#define INC_CALCULATED_PWM_H_
#include "stm32f0xx_hal.h"
#include "adc.h"



float read_battery_voltaj1();
uint16_t calculated_pwm(float desired_force, float read_voltaj);
#endif /* INC_CALCULATED_PWM_H_ */
