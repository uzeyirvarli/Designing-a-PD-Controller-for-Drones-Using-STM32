/*
 * tim.h
 *
 *  Created on: Nov 23, 2023
 *      Author: uzeyir
 */

#ifndef HARDWARE_INC_TIM_H_
#define HARDWARE_INC_TIM_H_

#include "stm32f0xx_hal.h"
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/* Private function prototypes -----------------------------------------------*/
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
#endif /* HARDWARE_INC_TIM_H_ */
