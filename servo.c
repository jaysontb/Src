#include "servo.h"
#include "tim.h"

/**
  * @brief  设置云台舵机的角度
  * @param  angle: 0.0 ~ 180.0
  * @retval None
  */
void Servo_SetAngle1(float angle)
{
    // 将角度从0-180度映射到脉冲宽度500-2500
    uint16_t pulse = 500 + (angle / 180.0f) * 2000;
    
    // 设置TIM1通道1的CCR值来控制舵机
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}

/**
  * @brief  设置爪子舵机的角度
  * @param  angle: 0.0 ~ 180.0
  * @retval None
  */
void Servo_SetAngle2(float angle)
{
    // 将角度从0-180度映射到脉冲宽度500-2500
    uint16_t pulse = 500 + (angle / 180.0f) * 2000;
    
    // 设置TIM1通道2的CCR值来控制舵机
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse);
}

/**
  * @brief  设置物料盘舵机的角度
  * @param  angle: 0.0 ~ 180.0
  * @retval None
  */
void Servo_SetAngle3(float angle)
{
    // 将角度从0-180度映射到脉冲宽度500-2500
    uint16_t pulse = 500 + (angle / 180.0f) * 2000;
    
    // 设置TIM1通道3的CCR值来控制舵机
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse);
}
