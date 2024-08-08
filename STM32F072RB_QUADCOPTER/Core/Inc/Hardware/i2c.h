/*
 * i2c.h
 *
 *  Created on: Nov 23, 2023
 *      Author: uzeyir
 */

#ifndef HARDWARE_INC_I2C_H_
#define HARDWARE_INC_I2C_H_

#include "stm32f0xx_hal.h"
extern I2C_HandleTypeDef hi2c1;


/* Private function prototypes -----------------------------------------------*/
;
void MX_I2C1_Init(void);
void MX_I2C2_Init(void);

#endif /* HARDWARE_INC_I2C_H_ */
