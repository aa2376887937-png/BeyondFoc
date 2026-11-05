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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* main.c 里定义的外设句柄。
 * 本工程没有按外设拆分 tim.h / adc.h / usart.h / gpio.h（CubeMX 的
 * "Generate peripheral initialization as a pair of .c/.h files" 选项关闭），
 * 所以统一在这里用 extern 声明，用户代码包含 main.h 即可。 */
extern ADC_HandleTypeDef    hadc1;
extern ADC_HandleTypeDef    hadc2;
extern CORDIC_HandleTypeDef hcordic;
extern SPI_HandleTypeDef    hspi1;
extern DMA_HandleTypeDef    hdma_spi1_rx;
extern DMA_HandleTypeDef    hdma_spi1_tx;
extern TIM_HandleTypeDef    htim1;
extern TIM_HandleTypeDef    htim2;
extern UART_HandleTypeDef   huart3;
extern DMA_HandleTypeDef    hdma_usart3_rx;
extern DMA_HandleTypeDef    hdma_usart3_tx;
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
#define KEY1_Pin GPIO_PIN_6
#define KEY1_GPIO_Port GPIOC
#define KEY2_Pin GPIO_PIN_7
#define KEY2_GPIO_Port GPIOC
#define KEY3_Pin GPIO_PIN_8
#define KEY3_GPIO_Port GPIOC
#define KEY4_Pin GPIO_PIN_9
#define KEY4_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOC
#define ROM_LOAD_Pin GPIO_PIN_7
#define ROM_LOAD_GPIO_Port GPIOA
#define ENC_CSN_Pin GPIO_PIN_2
#define ENC_CSN_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
