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
#include "stm32h7xx_hal.h"

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
#define Yellow_Led_PIN_Pin GPIO_PIN_6
#define Yellow_Led_PIN_GPIO_Port GPIOE
#define Loadcell_PIN_Pin GPIO_PIN_9
#define Loadcell_PIN_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define Enable_PIN_Pin GPIO_PIN_2
#define Enable_PIN_GPIO_Port GPIOC
#define Direction_PIN_Pin GPIO_PIN_3
#define Direction_PIN_GPIO_Port GPIOC
#define Ethercat_SCK_Pin GPIO_PIN_5
#define Ethercat_SCK_GPIO_Port GPIOA
#define Ethercat_MISO_Pin GPIO_PIN_6
#define Ethercat_MISO_GPIO_Port GPIOA
#define Ethercat_MOSI_Pin GPIO_PIN_7
#define Ethercat_MOSI_GPIO_Port GPIOA
#define Red_Led_PIN_Pin GPIO_PIN_1
#define Red_Led_PIN_GPIO_Port GPIOG
#define Ethercat_SS_Pin GPIO_PIN_14
#define Ethercat_SS_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define Step_PIN_Pin GPIO_PIN_4
#define Step_PIN_GPIO_Port GPIOD
#define Alarm_PIN_Pin GPIO_PIN_5
#define Alarm_PIN_GPIO_Port GPIOD
#define Endstop_up_PIN_Pin GPIO_PIN_6
#define Endstop_up_PIN_GPIO_Port GPIOD
#define Endstop_up_PIN_EXTI_IRQn EXTI9_5_IRQn
#define Endstop_down_PIN_Pin GPIO_PIN_7
#define Endstop_down_PIN_GPIO_Port GPIOD
#define Endstop_down_PIN_EXTI_IRQn EXTI9_5_IRQn
#define Enc_A_PIN_Pin GPIO_PIN_10
#define Enc_A_PIN_GPIO_Port GPIOG
#define Enc_A_PIN_EXTI_IRQn EXTI15_10_IRQn
#define Enc_Z_PIN_Pin GPIO_PIN_11
#define Enc_Z_PIN_GPIO_Port GPIOG
#define Enc_Z_PIN_EXTI_IRQn EXTI15_10_IRQn
#define Enc_B_PIN_Pin GPIO_PIN_13
#define Enc_B_PIN_GPIO_Port GPIOG
#define Enc_B_PIN_EXTI_IRQn EXTI15_10_IRQn
#define Green_Led_PIN_Pin GPIO_PIN_15
#define Green_Led_PIN_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
