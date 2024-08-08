/*
 * adc.h
 *
 *  Created on: Nov 23, 2023
 *      Author: uzeyir
 */

#ifndef HARDWARE_INC_ADC_H_
#define HARDWARE_INC_ADC_H_

#include "stm32f0xx_hal.h"
#include "adc.h"
extern ADC_HandleTypeDef hadc;

/* Private function prototypes -----------------------------------------------*/
void MX_ADC_Init(void);

#endif /* HARDWARE_INC_ADC_H_ */
