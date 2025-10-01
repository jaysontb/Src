/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "Emm_V5.h"
#include "MPU6050.h"
#include "inv_mpu.h"
#include "servo.h"
#include "oled.h"
#include <stdio.h>
#include "test.h"

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
  // float pitch = 0.0f;
  // float roll = 0.0f;
  // float yaw = 0.0f;
  // uint8_t mpu_ready = 0U;

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // mpu_ready = (mpu_dmp_init() == 0U);
  // if (mpu_ready)
  // {
  //   OLED_ShowString(1, 1, "MPU DMP READY ");
  //   OLED_ShowString(2, 1, "Pitch:");
  //   OLED_ShowString(3, 1, "Roll :");
  //   OLED_ShowString(4, 1, "Yaw  :");
  // }
  // else
  // {
  //   OLED_ShowString(1, 1, "MPU DMP ERROR");
  // }

	// __HAL_UART_CLEAR_IDLEFLAG(&huart1); 											
	// __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); 							
  // HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxCmd, CMD_LEN); 
  
  // HAL_Delay(1000);

  // Emm_V5_Pos_Control(1, 0, 100, 10, 3200, 0, 1);
  // HAL_Delay(100);

  // Emm_V5_Pos_Control(2, 0, 100, 10, 3200, 0, 1);
  // HAL_Delay(100);

  // Emm_V5_Pos_Control(3, 0, 100, 10, 3200, 0, 1);
  // HAL_Delay(100);

  // Emm_V5_Pos_Control(4, 0, 100, 10, 3200, 0, 1);
  // HAL_Delay(100);

  // Emm_V5_Synchronous_motion(0);
  // HAL_Delay(10);

	// while(rxCmd[1] != 0xFD || rxCmd[2] != 0x9F); rxFrameFlag = false;

  Test_2();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // if (mpu_ready)
    // {
    //   if (mpu_dmp_get_data(&pitch, &roll, &yaw) == 0U)
    //   {
    //     char value_buffer[12];
    //     OLED_ShowString(1, 1, "MPU DMP READY ");
    //     (void)snprintf(value_buffer, sizeof(value_buffer), "%7.2f", pitch);
    //     OLED_ShowString(2, 7, value_buffer);
    //     (void)snprintf(value_buffer, sizeof(value_buffer), "%7.2f", roll);
    //     OLED_ShowString(3, 7, value_buffer);
    //     (void)snprintf(value_buffer, sizeof(value_buffer), "%7.2f", yaw);
    //     OLED_ShowString(4, 7, value_buffer);
    //   }
    //   else
    //   {
    //     OLED_ShowString(1, 1, "MPU DATA FAIL ");
    //   }
    // }
    // HAL_Delay(50); 
  
  
  
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
	while(1);

  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
