#include "servo.h"
#include "tim.h"

static uint16_t Servo_AngleToPulse1(float angle)
{
    if (angle < 0.0f)
    {
        angle = 0.0f;
    }
    else if (angle > 270.0f)
    {
        angle = 270.0f;
    }
    return (uint16_t)(500.0f + (angle / 270.0f) * 2000.0f);
}

static uint16_t Servo_AngleToPulse2(float angle)
{
    if (angle < 0.0f)
    {
        angle = 0.0f;
    }
    else if (angle > 180.0f)
    {
        angle = 180.0f;
    }
    return (uint16_t)(500.0f + (angle / 180.0f) * 2000.0f);
}

/**
  * @brief  设置云台舵机的角度
  * @param  angle: 0.0 ~ 270.0
  * @retval None
  */
void Servo_SetAngle1(float angle)
{
    // 将角度从0-270度映射到脉冲宽度500-2500
    uint16_t pulse = Servo_AngleToPulse1(angle);
    
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
    uint16_t pulse = Servo_AngleToPulse2(angle);
    
    // 设置TIM1通道2的CCR值来控制舵机
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse);
}

/**
  * @brief  设置机械臂前后运动的距离
  * @param  angle: 0.0 ~ 180.0
  * @retval None
  */
void Servo_SetAngle3(float angle)
{
    // 将角度从0-180度映射到脉冲宽度500-2500
    uint16_t pulse = Servo_AngleToPulse2(angle);

    // 设置TIM1通道3的CCR值来控制舵机
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse);
}
