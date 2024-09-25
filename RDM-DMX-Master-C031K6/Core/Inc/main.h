/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32c0xx_hal.h"

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
#define USB_TX_Pin GPIO_PIN_2
#define USB_TX_GPIO_Port GPIOA
#define USB_RX_Pin GPIO_PIN_3
#define USB_RX_GPIO_Port GPIOA
#define LED_DMX_Received_Pin GPIO_PIN_5
#define LED_DMX_Received_GPIO_Port GPIOA
#define LED_DMX_Sent_Pin GPIO_PIN_6
#define LED_DMX_Sent_GPIO_Port GPIOA
#define Break_detection_Pin GPIO_PIN_0
#define Break_detection_GPIO_Port GPIOB
#define DMX_TX_Pin GPIO_PIN_9
#define DMX_TX_GPIO_Port GPIOA
#define DMX_RX_Pin GPIO_PIN_10
#define DMX_RX_GPIO_Port GPIOA
#define DMX_DE_Pin GPIO_PIN_12
#define DMX_DE_GPIO_Port GPIOA
#define Enter_button_Pin GPIO_PIN_15
#define Enter_button_GPIO_Port GPIOA
#define Up_button_Pin GPIO_PIN_3
#define Up_button_GPIO_Port GPIOB
#define Down_button_Pin GPIO_PIN_4
#define Down_button_GPIO_Port GPIOB
#define Back_button_Pin GPIO_PIN_5
#define Back_button_GPIO_Port GPIOB
#define Display_SCL_Pin GPIO_PIN_6
#define Display_SCL_GPIO_Port GPIOB
#define Display_SDA_Pin GPIO_PIN_7
#define Display_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
