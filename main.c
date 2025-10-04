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
#include "visual_comm.h"    // 添加视觉通信头文件
#include "visual_test.h"    // 添加视觉测试头文件
#include "motor.h"          // 添加电机控制头文件
#include "gripper.h"        // 添加机械爪控制头文件

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

// ==================== 电机测试函数声明 ====================
void Motor_Test_Basic(void);           // 基础测试: 前后左右旋转
void Motor_Test_Square(void);          // 正方形路径测试
void Motor_Test_Competition_Route(void); // 比赛路径模拟测试

// ==================== 机械爪测试函数声明 ====================
void Gripper_Test_Simple(void);        // 简化版升降测试

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
  
  //OLED_Init();  // 初始化OLED显示
  // ==================== 视觉通信初始化 ====================
  //Visual_Data_Init();          // 初始化视觉数据结构
  //Visual_UART_Start_Receive(); // 启动串口接收
  
  // 延时等待LubanCat启动(可选)
  //HAL_Delay(500);
  
  // ==================== 执行视觉测试 ====================
  // 取消注释以下任意一行进行测试:
  
  //Visual_Test_Basic();      // 基础测试(单次测试物料/凸台/二维码)
  // Visual_Test_FullTask();   // 完整任务流程测试
  
  // ==================== 原有测试代码 ====================
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

	__HAL_UART_CLEAR_IDLEFLAG(&huart1); 											
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); 							
  HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxCmd, CMD_LEN); 


  HAL_Delay(1000);

  Emm_V5_Pos_Control(6, 0, 100, 100, 16000, 0, 0);


  // Test_1();
  // Test_2();
  // Test_3();

  // 初始化电机模块
  // Motor_Init();
  
  
  // 测试1: 基础运动测试 (推荐首次测试)
  // 功能: 前进、后退、左移、右移、旋转各执行一次
  // 用时: 约30秒
  // 场地需求: 300×300mm
  //Motor_Test_Basic();
  
  // 测试2: 正方形路径测试 (推荐第二次测试)
  // 功能: 走一个200×200mm的正方形,检验定位精度
  // 用时: 约1分钟
  // 场地需求: 500×500mm
  // Motor_Test_Square();
  
  // 测试3: 比赛路径模拟 (完整流程测试)
  // 功能: 模拟启停区→物料台→测试区→返回的完整路径
  // 用时: 约2分钟
  // 场地需求: 1500×600mm
  // Motor_Test_Competition_Route();
  
  // ========== 手动测试代码示例 ==========
  // 如果想自定义测试,可以直接调用以下函数:
  
  // Motor_Move_Forward(100.0f, 0);   // 前进100mm,使用默认速度
  // HAL_Delay(500);
  //Motor_Move_Lateral(200.0f, 40);   // 右移200mm,速度40RPM
  //HAL_Delay(500);
  // Motor_Move_Rotate(90.0f, 30);    // 逆时针旋转90度,速度30RPM
  // HAL_Delay(500);
  
  // ========== 速度档位测试 ==========
  // Motor_Set_Speed_Profile(0);      // 精确模式 30RPM
  // Motor_Move_Forward(100.0f, 0);
  // HAL_Delay(1000);
  
  // Motor_Set_Speed_Profile(1);      // 正常模式 50RPM
  // Motor_Move_Forward(100.0f, 0);
  // HAL_Delay(1000);
  
  // Motor_Set_Speed_Profile(2);      // 快速模式 80RPM
  // Motor_Move_Forward(100.0f, 0);
  // HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // ==================== 视觉数据接收处理 ====================
    // if (visual_rx_complete_flag)  // 检测到视觉数据
    // {
    //     // 解析接收数据
    //     Visual_Data_Unpack(Visual_Get_RxBuffer());
        
    //     // 清除标志
    //     Visual_Clear_RxFlag(); 
        
    //     // TODO: 这里添加你的业务逻辑
    //     // 例如: 根据接收到的坐标控制电机运动
    // }
    
    // ==================== 循环测试(可选) ====================
    // 取消注释以下行进行连续测试:
    //Visual_Test_Loop();
    
    // ==================== MPU6050姿态显示(原有代码) ====================
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

// ==================== 电机测试函数实现 ====================

/**
 * @brief 基础运动测试
 * @details 测试前进、后退、左移、右移、旋转功能
 * @note 在空旷场地进行,确保300×300mm范围内无障碍物
 */
void Motor_Test_Basic(void)
{
	OLED_Clear();
	OLED_ShowString(1, 1, "Motor Test");
	OLED_ShowString(2, 1, "Basic Moves");
	
	// 1. 前进100mm
	OLED_ShowString(3, 1, "Forward 100mm");
	Motor_Move_Forward(200.0f, 0);  // 使用默认速度
	
	// 2. 后退100mm
	OLED_ShowString(3, 1, "Back 100mm   ");
	Motor_Move_Forward(-200.0f, 0);
	
	// 3. 右移50mm
	OLED_ShowString(3, 1, "Right 50mm   ");
	Motor_Move_Lateral(100.0f, 0);
	
	// 4. 左移50mm
	OLED_ShowString(3, 1, "Left 50mm    ");
	Motor_Move_Lateral(-100.0f, 0);
	
	// 5. 逆时针旋转90度
	OLED_ShowString(3, 1, "Rotate 90 CCW");
	Motor_Move_Rotate(90.0f, 0);
	
	// 6. 顺时针旋转90度
	OLED_ShowString(3, 1, "Rotate 90 CW ");
	Motor_Move_Rotate(-90.0f, 0);
	
	OLED_ShowString(3, 1, "Test Done!   ");
	OLED_ShowString(4, 1, "Check Position");
}

/**
 * @brief 正方形路径测试
 * @details 走一个200mm×200mm的正方形,检验定位精度
 * @note 理想情况下应该回到起点,观察误差大小
 */
void Motor_Test_Square(void)
{
	const float side_length = 200.0f;  // 正方形边长200mm
	
	OLED_Clear();
	OLED_ShowString(1, 1, "Square Test");
	OLED_ShowString(2, 1, "200x200mm");
	
	// 设置为正常速度模式
	Motor_Set_Speed_Profile(1);  // SPEED_NORMAL = 50 RPM
	
	for (uint8_t i = 0; i < 4; i++)
	{
		char step_str[16];
		snprintf(step_str, sizeof(step_str), "Side %d/4", i+1);
		OLED_ShowString(3, 1, step_str);
		
		// 前进一边
		Motor_Move_Forward(side_length, 0);
		HAL_Delay(200);
		
		// 逆时针转90度
		Motor_Move_Rotate(90.0f, 0);
		HAL_Delay(200);
	}
	
	OLED_ShowString(3, 1, "Square Done! ");
	OLED_ShowString(4, 1, "Check Return");
}

/**
 * @brief 比赛路径模拟测试
 * @details 模拟从启停区到物料台再到测试区的路径
 * @note 根据比赛场地实际尺寸调整距离参数
 * 
 * 假设场地布局(参考比赛文档):
 * - 启停区: (0, 0)
 * - 物料台: (600, 400) - 距离启停区约721mm
 * - 测试区: (1200, 400) - 距离物料台600mm
 */
void Motor_Test_Competition_Route(void)
{
	OLED_Clear();
	OLED_ShowString(1, 1, "Competition");
	OLED_ShowString(2, 1, "Route Test");
	
	// 阶段1: 从启停区到物料台
	OLED_ShowString(3, 1, "To Material");
	Motor_Set_Speed_Profile(2);  // SPEED_FAST = 80 RPM (空载)
	Motor_Move_Forward(600.0f, 0);   // 前进600mm
	HAL_Delay(200);
	Motor_Move_Lateral(400.0f, 0);   // 右移400mm
	HAL_Delay(500);
	
	// 阶段2: 在物料台取料(模拟)
	OLED_ShowString(3, 1, "Pick Material");
	Motor_Set_Speed_Profile(0);  // SPEED_PRECISE = 30 RPM (精确对位)
	// 这里应该调用机械臂抓取函数
	HAL_Delay(2000);  // 模拟抓取时间
	
	// 阶段3: 从物料台到测试区
	OLED_ShowString(3, 1, "To Test Zone");
	Motor_Set_Speed_Profile(1);  // SPEED_NORMAL = 50 RPM (载货)
	Motor_Move_Forward(600.0f, 0);   // 继续前进600mm
	HAL_Delay(500);
	
	// 阶段4: 在测试区放料(模拟)
	OLED_ShowString(3, 1, "Place Test  ");
	Motor_Set_Speed_Profile(0);  // SPEED_PRECISE = 30 RPM
	// 这里应该调用机械臂放置函数
	HAL_Delay(2000);  // 模拟放置时间
	
	// 阶段5: 返回启停区
	OLED_ShowString(3, 1, "Return Home ");
	Motor_Set_Speed_Profile(2);  // SPEED_FAST = 80 RPM (空载)
	Motor_Move_Rotate(180.0f, 0);    // 转向180度
	HAL_Delay(200);
	Motor_Move_Forward(1200.0f, 0);  // 直接返回
	HAL_Delay(200);
	Motor_Move_Lateral(-400.0f, 0);  // 左移回原位
	HAL_Delay(200);
	Motor_Move_Rotate(-180.0f, 0);   // 转回原方向
	
	OLED_ShowString(3, 1, "Route Done! ");
	OLED_ShowString(4, 1, "Check Pos   ");
}

// ==================== 机械爪测试函数实现 ====================

/**
 * @brief 简化版机械爪升降测试
 */
void Gripper_Test_Simple(void)
{
	// 测试1: 升到10mm (离地19cm)
	Gripper_Lift(10.0f);
	HAL_Delay(1000);
	
	// 测试2: 升到最高22mm (离地20.2cm)
	Gripper_Lift(22.0f);
	HAL_Delay(1000);
	
	// 测试3: 降到5mm (离地18.5cm)
	Gripper_Lift(5.0f);
	HAL_Delay(1000);
	
	// 测试4: 回到0mm (离地18cm)
	Gripper_Lift(0.0f);
}

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
