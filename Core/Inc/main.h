/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KILL_SWITCH_Pin GPIO_PIN_2
#define KILL_SWITCH_GPIO_Port GPIOE
#define MOTOR_1_Pin GPIO_PIN_1
#define MOTOR_1_GPIO_Port GPIOA
#define MOTOR_2_Pin GPIO_PIN_2
#define MOTOR_2_GPIO_Port GPIOA
#define MOTOR_3_Pin GPIO_PIN_3
#define MOTOR_3_GPIO_Port GPIOA
#define MOTOR_0_Pin GPIO_PIN_5
#define MOTOR_0_GPIO_Port GPIOA
#define MPU9250_CS_Pin GPIO_PIN_12
#define MPU9250_CS_GPIO_Port GPIOB
#define DC_MOTOR_HALL_Pin GPIO_PIN_8
#define DC_MOTOR_HALL_GPIO_Port GPIOD
#define DC_MOTOR_AIN2_Pin GPIO_PIN_9
#define DC_MOTOR_AIN2_GPIO_Port GPIOD
#define DC_MOTOR_AIN1_Pin GPIO_PIN_10
#define DC_MOTOR_AIN1_GPIO_Port GPIOD
#define DC_MOTOR_STBY_Pin GPIO_PIN_11
#define DC_MOTOR_STBY_GPIO_Port GPIOD
#define DC_MOTOR_PWMA_Pin GPIO_PIN_12
#define DC_MOTOR_PWMA_GPIO_Port GPIOD
#define SERVO_1_Pin GPIO_PIN_13
#define SERVO_1_GPIO_Port GPIOD
#define SERVO_2_Pin GPIO_PIN_14
#define SERVO_2_GPIO_Port GPIOD
#define SERVO_3_Pin GPIO_PIN_15
#define SERVO_3_GPIO_Port GPIOD
#define MOTOR_4_Pin GPIO_PIN_6
#define MOTOR_4_GPIO_Port GPIOC
#define MOTOR_5_Pin GPIO_PIN_7
#define MOTOR_5_GPIO_Port GPIOC
#define MOTOR_6_Pin GPIO_PIN_8
#define MOTOR_6_GPIO_Port GPIOC
#define MOTOR_7_Pin GPIO_PIN_9
#define MOTOR_7_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
