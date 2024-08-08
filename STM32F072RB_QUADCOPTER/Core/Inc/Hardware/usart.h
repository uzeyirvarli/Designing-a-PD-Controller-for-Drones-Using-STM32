/*
 * usart.h
 *
 *  Created on: Jan 3, 2024
 *      Author: uzeyir
 */

#ifndef INC_HARDWARE_USART_H_
#define INC_HARDWARE_USART_H_

#include "stm32f0xx_hal.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);


#endif /* INC_HARDWARE_USART_H_ */
