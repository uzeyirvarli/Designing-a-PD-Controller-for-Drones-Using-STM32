/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f0xx_hal.h"
#include "usart.h"
#include <stdbool.h>
#include<stdio.h>
#include <string.h>
#include "stdlib.h"
#include "hardware.h"
#include "IMU_Sensor/MahonyAHRS.h"
#include "IMU_Sensor/acc_gyro_calc.h"
#include "math.h"
#include "FS_IA6B_ibus/FS-IA6B_ibus.h"
#include "calculated_pwm.h"
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Defines */
#define IBUS_USER_CHANNELS		6		// Use 6 channels
#define IBUS_LENGTH				0x20	// 32 bytes
#define IBUS_COMMAND40			0x40	// Command to set servo or motor speed is always 0x40
#define IBUS_MAX_CHANNLES		14
#define IBUS_USER_CHANNELS	6
#define GYRO_SIZE 3
/* Defines */



/* USER CODE END PV */
int initialize=0;
uint8_t rxBuf[32]; // fs-ia10b
uint16_t ibus_data_channels[IBUS_USER_CHANNELS];

float Yaw_Pitch_Roll[GYRO_SIZE];
float gyro_XYZ[GYRO_SIZE];
char str[16];
float initial_yaw,initial_pitch,initial_roll;
int start=0;
uint16_t motor[4];
int throttle;
float desired_Yaw_Pitch_Roll[3];
float roll_input;
float pitch_input;
float yaw_input;

int reset=0;

PIDController pidRoll, pidPitch, pidYaw;

float Proportional_Roll = 1.68;
float Integral_Roll = 0;
float  Derivative_Roll = 15;
float Proportional_Pitch = 1.68;
float Integral_Pitch = 0;
float  Derivative_Pitch = 15;
float Proportional_Yaw = 1.68;
float Integral_Yaw =0;//
float Derivative_Yaw = 15;
float input_yaw;
float input_pitch;
float input_roll;
float desired_yaw_pitch_roll_angle[3];
float gyro_XYZ[GYRO_SIZE];
float min_pitch=0.0;
float battery_voltaj;

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
float read_battery_voltaj();
void desired_yaw_pitch_roll(float* desired_yaw_pitch_roll);
float Mapping(float x,float in_min,float in_max,float out_min,float out_max);
float read_voltaj();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  HAL_UART_Receive_IT(&huart1, rxBuf, 32);


  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  PID_Init(&pidRoll, Proportional_Roll, Integral_Roll, Derivative_Roll);
  PID_Init(&pidPitch, Proportional_Pitch, Integral_Pitch, Derivative_Pitch);
  PID_Init(&pidYaw, Proportional_Yaw, Integral_Yaw, Derivative_Yaw);

  //HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	  if(start==0)
	  {
		  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
		  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
		  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
		  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);

		  for(int i=0;i<=3;i++)
		  {
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			  HAL_Delay(100);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			  HAL_Delay(100);
		  }

		  IbusData(ibus_data_channels,IBUS_USER_CHANNELS,rxBuf);
		  //if(ibus_data_channels[2]==0) NVIC_SystemReset();
		  if(ibus_data_channels[2]>=1000 && ibus_data_channels[2]<=2000) start=1;

	  }

	  if(rxBuf[14]==232 && initialize==0)
	  {
		  get_yaw_pitch_roll(Yaw_Pitch_Roll,gyro_XYZ, GYRO_SIZE);
		  initial_yaw=Yaw_Pitch_Roll[0];
		  initial_pitch=Yaw_Pitch_Roll[1];
		  initial_roll=Yaw_Pitch_Roll[2];
		  start=1;
	  }

	  else
	  {
		  IbusData(ibus_data_channels,IBUS_USER_CHANNELS,rxBuf);
		  throttle=ibus_data_channels[2];
		  get_yaw_pitch_roll(Yaw_Pitch_Roll,gyro_XYZ, GYRO_SIZE);

		  Yaw_Pitch_Roll[0]=Yaw_Pitch_Roll[0]-initial_yaw;
		  Yaw_Pitch_Roll[1]=Yaw_Pitch_Roll[1]-initial_pitch;
		  Yaw_Pitch_Roll[2]=Yaw_Pitch_Roll[2]-initial_roll;

		  desired_yaw_pitch_roll(desired_yaw_pitch_roll_angle);

		  pidRoll.setpoint=desired_yaw_pitch_roll_angle[2];
		  pidPitch.setpoint=desired_yaw_pitch_roll_angle[1];
		  pidYaw.setpoint=desired_yaw_pitch_roll_angle[0];

		  float rollOutput = PID_Compute(&pidRoll,pidRoll.setpoint,Yaw_Pitch_Roll[2]);
		  float pitchOutput = PID_Compute(&pidPitch, pidPitch.setpoint, Yaw_Pitch_Roll[1]);
		  float yawOutput = PID_Compute(&pidYaw,  pidYaw.setpoint, Yaw_Pitch_Roll[0]);


		  yaw_input=Yaw_Pitch_Roll[0];
		  pitch_input=Yaw_Pitch_Roll[1];
		  roll_input=Yaw_Pitch_Roll[2];
		  start=2;

	  }

	  if (throttle<=1300 && start==2)
	  {
		  motor[0] = throttle;
		  motor[1] = throttle;
		  motor[2] = throttle;
		  motor[3] = throttle;
	  }

	  else if (throttle>1300 && throttle<=2000 && start==2)
	  {
		  motor[0] = throttle - pidPitch.output + pidRoll.output - pidYaw.output; //Calculate the pulse for esc 1 (front-right - CCW)
		  motor[1] = throttle - pidPitch.output - pidRoll.output + pidYaw.output; //Calculate the pulse for esc 2 (rear-right - CW)
		  motor[2] = throttle + pidPitch.output - pidRoll.output - pidYaw.output; //Calculate the pulse for esc 3 (rear-left - CCW)
		  motor[3] = throttle + pidPitch.output + pidRoll.output + pidYaw.output; //Calculate the pulse for esc 4 (front-left - CW)



	  }

	  if (motor[0]<1000 || motor[0]>2200) motor[0]=1000;
	  if (motor[1]<1000 || motor[1]>2200) motor[1]=1000;
	  if (motor[2]<1000 || motor[2]>2200) motor[2]=1000;
	  if (motor[3]<1000 || motor[3]>2200) motor[3]=1000;

	  if ((motor[0]>2000) && (motor[0]<=2200)) motor[0]=2000;
	  if ((motor[1]>2000) && (motor[0]<=2200)) motor[1]=2000;
	  if ((motor[2]>2000) && (motor[0]<=2200)) motor[2]=2000;
	  if ((motor[3]>2000) && (motor[0]<=2200)) motor[3]=2000;

	  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,motor[0]);
	  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,motor[1]);
	  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,motor[2]);
	  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,motor[3]);
	  IbusData(ibus_data_channels,IBUS_USER_CHANNELS,rxBuf);
	  HAL_UART_Transmit(&huart2, (uint8_t *)str, sprintf(str, "%1.f\t",Yaw_Pitch_Roll[0] ), 0xFFFF);
	  HAL_UART_Transmit(&huart2, (uint8_t *)str, sprintf(str, "%1.f\t", Yaw_Pitch_Roll[1]), 0xFFFF);
	  HAL_UART_Transmit(&huart2, (uint8_t *)str, sprintf(str, "%1.f\t\n",Yaw_Pitch_Roll[2] ), 0xFFFF);
      battery_voltaj=read_voltaj();

	 HAL_Delay(10);


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */


/* USER CODE BEGIN 4 */

float read_voltaj()
{
	uint16_t adc_v;
	float R1=5.0;
	int R2=1.0;
	float battery;
	HAL_ADC_Start(&hadc); // ADC'yi start et pointer ile çagır.
	if (HAL_ADC_PollForConversion(&hadc,20)==HAL_OK)  // HAL_ADC_PollForConversion kullanman gerekecek
	{
		adc_v=HAL_ADC_GetValue(&hadc);
		/*HAL_UART_Transmit(&huart2, (uint8_t *)str, sprintf(str, "%d-", adc_v), 0xFFFF);
		HAL_UART_Transmit(&huart2, (uint8_t *)str, sprintf(str, "%d \n\r", adc_v/2), 0xFFFF);*/
	}

	HAL_ADC_Stop(&hadc); // ADC'yi start et pointer ile çagır.
	battery=(((float)adc_v)/4096)*3.3;
	battery=battery/(R2/(R1+R2))+0.1;
	return battery;
}



float Mapping(float x,float in_min,float in_max,float out_min,float out_max)
{
	float y;
	y=(x-in_min)*(out_max - out_min)/ (in_max - in_min) + out_min;
	return y;
}


void desired_yaw_pitch_roll(float* desired_yaw_pitch_roll)
{
	IbusData(ibus_data_channels,IBUS_USER_CHANNELS,rxBuf);
	  if (ibus_data_channels[0]<1500)
		  desired_yaw_pitch_roll[2]=Mapping(ibus_data_channels[0], 1000, 1500, -3, 0);
	  else if (ibus_data_channels[0]>=1500 && ibus_data_channels[0]<2000)
		  desired_yaw_pitch_roll[2]=Mapping(ibus_data_channels[0], 1501, 2000, 0, 3);

	  if (ibus_data_channels[1]<1500)
		  desired_yaw_pitch_roll[1]=Mapping(ibus_data_channels[1], 1000, 1500, -3, 0);
	  else if (ibus_data_channels[1]>=1500 && ibus_data_channels[1]<2000)
		  desired_yaw_pitch_roll[1]=Mapping(ibus_data_channels[1], 1500, 2000, 0, 3);

	  if (ibus_data_channels[3]<1500)
		  desired_yaw_pitch_roll[0]=Mapping(ibus_data_channels[3], 1000, 1500, -360, 0);
	  else if (ibus_data_channels[3]>=1500 && ibus_data_channels[3]<2000)
		  desired_yaw_pitch_roll[0]=Mapping(ibus_data_channels[3], 1500, 2000, 0, 360);
}

__weak void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */

  __HAL_TIM_SET_COUNTER(&htim2,0);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_IC_CaptureCallback could be implemented in the user file
   */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart1)
	{
		HAL_UART_Receive_IT(&huart1, rxBuf, 32);
	}

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
