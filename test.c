#include "test.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "servo.h"
#include "gripper.h"  // 机械爪控制模块

// 定义云台舵机角度
#define PLATE1     185.0f//第一个物料盘位置
#define PLATE2     206.0f//第二个物料盘位置
#define PLATE3     224.0f//第三个物料盘位置
#define GRAB       0.0f  //机械臂抓取位置

#define SONGKAI    45.0f //机械爪松开
#define ZHUA      9.0f   //机械爪抓紧

void Test_1(void)
{   
    // 启动定时器PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    
    Servo_SetAngle1(PLATE1);
}
void Test_2(void)
{   
    // 启动定时器PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    
    Servo_SetAngle2(ZHUA);
}
void Test_3(void)
{   
    // 启动定时器PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    
    Servo_SetAngle3(0.0f);
}

/**
 * @brief Gripper综合测试: 抓取物块并放置到物料盘3
 * @details 完整流程:
 *   1. 初始位置: 云台在抓取位置(0°),爪子张开
 *   2. 降低到抓取高度,闭合爪子夹紧物块
 *   3. 升高到装配台L2高度(12mm,对应物料盘高度)
 *   4. 旋转云台到物料盘3位置(232°)
 *   5. 张开爪子释放物块
 *   6. 复位: 升高,旋转回抓取位置
 */
void Test_Gripper_PickAndPlace(void)
{
    // === 第1步: 初始化位置 ===
    Gripper_Pan_Rotate(PAN_ANGLE_GRAB);  // 确保云台在抓取位置(0°)
    HAL_Delay(500);  // 等待舵机到位
    
    Gripper_Claw_Open();  // 张开爪子
    HAL_Delay(1000);  // 等待舵机到位
    
    // === 第2步: 降低并抓取物块 ===
    Gripper_Lift(HEIGHT_PICKUP_LOWER);  // 降到抓取高度(0mm)
    
    Gripper_Claw_Close();  // 闭合爪子夹紧
    HAL_Delay(1000);  // 等待夹紧(稍长确保夹稳)
    
    // === 第3步: 升高到物料盘高度 ===
    Gripper_Lift(HEIGHT_TRANSPORT);  // 升到12mm(物料盘高度)
    
    // === 第4步: 旋转到物料盘3 ===
    Gripper_Pan_Rotate(PAN_ANGLE_PLATE1);  // 旋转到232°
    HAL_Delay(2000);  // 等待舵机到位
    
    // === 第5步: 释放物块 ===
    Gripper_Claw_Open();  // 张开爪子
    HAL_Delay(500);  // 等待舵机到位
    
    // === 第6步: 复位 ===
    
    Gripper_Pan_Rotate(PAN_ANGLE_GRAB);  // 旋转回抓取位置(0°)
    HAL_Delay(500);  // 等待舵机到位
    
    Gripper_Lift(HEIGHT_HOME);  // 降到初始高度(0mm)
}