/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define OSCIN_Pin GPIO_PIN_0
#define OSCIN_GPIO_Port GPIOD
#define OSCOUT_Pin GPIO_PIN_1
#define OSCOUT_GPIO_Port GPIOD
#define TCCS_Pin GPIO_PIN_4
#define TCCS_GPIO_Port GPIOA
#define TCSCK_Pin GPIO_PIN_5
#define TCSCK_GPIO_Port GPIOA
#define TCIN_Pin GPIO_PIN_6
#define TCIN_GPIO_Port GPIOA
#define LCDCS_Pin GPIO_PIN_12
#define LCDCS_GPIO_Port GPIOB
#define LCDCK_Pin GPIO_PIN_13
#define LCDCK_GPIO_Port GPIOB
#define LCDIN_Pin GPIO_PIN_14
#define LCDIN_GPIO_Port GPIOB
#define LCDOUT_Pin GPIO_PIN_15
#define LCDOUT_GPIO_Port GPIOB
#define USBD__Pin GPIO_PIN_11
#define USBD__GPIO_Port GPIOA
#define USBD_A12_Pin GPIO_PIN_12
#define USBD_A12_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define PWMOUT_Pin GPIO_PIN_8
#define PWMOUT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
