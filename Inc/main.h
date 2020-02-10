/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32l4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MotA_CurrentSense_Pin GPIO_PIN_0
#define MotA_CurrentSense_GPIO_Port GPIOA
#define MotB_CurrentSense_Pin GPIO_PIN_1
#define MotB_CurrentSense_GPIO_Port GPIOA
#define MotB_Dir_Pin GPIO_PIN_5
#define MotB_Dir_GPIO_Port GPIOA
#define MotA_Dir_Pin GPIO_PIN_6
#define MotA_Dir_GPIO_Port GPIOA
#define MotB_PWM_Pin GPIO_PIN_7
#define MotB_PWM_GPIO_Port GPIOA
#define MotA_Brake_Pin GPIO_PIN_7
#define MotA_Brake_GPIO_Port GPIOC
#define MotB_Brake_Pin GPIO_PIN_10
#define MotB_Brake_GPIO_Port GPIOA
#define MotA_PWM_Pin GPIO_PIN_3
#define MotA_PWM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
