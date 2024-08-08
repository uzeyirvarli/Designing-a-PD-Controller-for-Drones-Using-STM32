/*
 * hardware.c
 *
 *  Created on: Nov 23, 2023
 *      Author: uzeyir
 */

#include "hardware.h"

void callling_inits()
{
  MX_GPIO_Init();
 // MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  //MX_USART1_UART_Init();
}
