/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PIEZO_Pin GPIO_PIN_0
#define PIEZO_GPIO_Port GPIOA
#define SPARE_Pin GPIO_PIN_1
#define SPARE_GPIO_Port GPIOA
#define ATS_PWM_Pin GPIO_PIN_2
#define ATS_PWM_GPIO_Port GPIOA
#define FLASH_SS_Pin GPIO_PIN_3
#define FLASH_SS_GPIO_Port GPIOA
#define ALT_SS_Pin GPIO_PIN_4
#define ALT_SS_GPIO_Port GPIOA
#define SCLK_Pin GPIO_PIN_5
#define SCLK_GPIO_Port GPIOA
#define SDO_Pin GPIO_PIN_6
#define SDO_GPIO_Port GPIOA
#define SDI_Pin GPIO_PIN_7
#define SDI_GPIO_Port GPIOA
#define LORA_SS_Pin GPIO_PIN_4
#define LORA_SS_GPIO_Port GPIOC
#define LORA_BUSY_Pin GPIO_PIN_5
#define LORA_BUSY_GPIO_Port GPIOC
#define LORA_NRESET_Pin GPIO_PIN_0
#define LORA_NRESET_GPIO_Port GPIOB
#define DIO_1_Pin GPIO_PIN_1
#define DIO_1_GPIO_Port GPIOB
#define MCU_STATUS_SIG_Pin GPIO_PIN_4
#define MCU_STATUS_SIG_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_8
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_9
#define SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
