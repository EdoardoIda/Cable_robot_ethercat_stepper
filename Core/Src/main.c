/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "Led.h"
#include "Control_timer.h"
#include "Serial.h"
#include "Encoder_3C.h"
#include "StepperRT.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int32_t TestSinusoid(uint32_t ms_time) {
	double t = (double)ms_time/1000.0;
	return (int32_t)floor(4000.0*sin(M_PI*t/2.5));
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
control_timer_t control_timer;
#define CONTROL_PERIOD 1 // milliseconds
#define STEP_PER_REV 8000 // also set in hardware
serial_t serial;
led_t green_led,
	  yellow_led,
	  red_led;
enc3c_t pulley_enc;
stepperRT_t motor;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_SPI1_Init();
  MX_ADC3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  timer_init(&control_timer, CONTROL_PERIOD);
  led_init(&green_led, Green_Led_PIN_GPIO_Port, Green_Led_PIN_Pin, control_timer.now);
  led_init(&yellow_led, Yellow_Led_PIN_GPIO_Port, Yellow_Led_PIN_Pin, control_timer.now);
  led_init(&red_led, Red_Led_PIN_GPIO_Port, Red_Led_PIN_Pin, control_timer.now);
  serial_init(&serial, &huart6, SERIAL_INTERRUPT);
  encoder_init(&pulley_enc, Enc_A_PIN_GPIO_Port, Enc_A_PIN_Pin, Enc_B_PIN_GPIO_Port, Enc_B_PIN_Pin,
		  Enc_Z_PIN_GPIO_Port, Enc_Z_PIN_Pin, ENC3_FORWARD, 5000);
  stepper_init(&motor, &htim3, CONTROL_PERIOD,
		Endstop_up_PIN_GPIO_Port, Endstop_up_PIN_Pin,
		Endstop_down_PIN_GPIO_Port, Endstop_down_PIN_Pin,
		Enable_PIN_GPIO_Port, Enable_PIN_Pin,
		Step_PIN_GPIO_Port, Step_PIN_Pin,
		Direction_PIN_GPIO_Port, Direction_PIN_Pin,
		Alarm_PIN_GPIO_Port, Alarm_PIN_Pin,
		STEP_PER_REV, STP_CW);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  led_blink(&green_led,1000);
  led_blink(&red_led,500);
  led_blink(&yellow_led,250);
  stepper_enable(&motor);
//  serial_read(&serial);
//  serial_write(&serial,(uint8_t*)"wow che fico!\r");
  uint32_t ms_time_offset = *(control_timer.now);
  uint16_t counter = 0;
  uint16_t direction = 1;
  while (1)
  {
      if (timer_elapsed(&control_timer)) {
//    	  if (counter <= 16000 && direction) {
//    		  counter ++;
//    		  HAL_GPIO_WritePin(motor.stepper_par.direction_port, motor.stepper_par.direction_pin, GPIO_PIN_SET);
//    		  if (counter == 16000) direction = 0;
//    	  } else if (counter >= 0) {
//    		  counter --;
//    		  HAL_GPIO_WritePin(motor.stepper_par.direction_port, motor.stepper_par.direction_pin, GPIO_PIN_RESET);
//    		  if (counter == 0) direction = 1;
//    	  }
//    	  HAL_GPIO_TogglePin(motor.stepper_par.step_port, motor.stepper_par.step_pin);
    	  motor.stepper_var.target_position = TestSinusoid(*(control_timer.now)-ms_time_offset);
  //  	  motor.stepper_var.target_position ++;
    	  stepper_set_update_flag(&motor);
      }
	  led_update_blink(&green_led);
	  led_update_blink(&yellow_led);
	  led_update_blink(&red_led);
//	  if (elapsed(&control_timer)) {
//		  led_blink(&green_led,green_led.led_var.blink_period*4);
//		    led_blink(&red_led,red_led.led_var.blink_period*4);
//		    led_blink(&yellow_led,yellow_led.led_var.blink_period*4);
//	  }
//	s

//	  HAL_UART_Transmit(&huart6, &data[0], 7, 5);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6|RCC_PERIPHCLK_SPI1
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 19;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
