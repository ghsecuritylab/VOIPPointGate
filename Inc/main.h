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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define DI1_Pin GPIO_PIN_4
#define DI1_GPIO_Port GPIOA
#define DI2_Pin GPIO_PIN_5
#define DI2_GPIO_Port GPIOA
#define DI3_Pin GPIO_PIN_6
#define DI3_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_11
#define PWM1_GPIO_Port GPIOE
#define PWM2_Pin GPIO_PIN_13
#define PWM2_GPIO_Port GPIOE
#define CE_Pin GPIO_PIN_13
#define CE_GPIO_Port GPIOD
#define LED1_G_Pin GPIO_PIN_6
#define LED1_G_GPIO_Port GPIOC
#define LED1_R_Pin GPIO_PIN_7
#define LED1_R_GPIO_Port GPIOC
#define LED2_R_Pin GPIO_PIN_8
#define LED2_R_GPIO_Port GPIOC
#define LED2_G_Pin GPIO_PIN_9
#define LED2_G_GPIO_Port GPIOC
#define RS485_DIR_Pin GPIO_PIN_12
#define RS485_DIR_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
