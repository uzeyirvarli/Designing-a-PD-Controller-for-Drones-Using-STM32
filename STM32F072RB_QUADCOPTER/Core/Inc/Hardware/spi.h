/*
 * spi.h
 *
 *  Created on: Nov 23, 2023
 *      Author: uzeyir
 */

#ifndef HARDWARE_INC_SPI_H_
#define HARDWARE_INC_SPI_H_

#include "stm32f0xx_hal.h"

extern SPI_HandleTypeDef hspi1;
/* Private function prototypes -----------------------------------------------*/

void MX_SPI1_Init(void);
#endif /* HARDWARE_INC_SPI_H_ */
