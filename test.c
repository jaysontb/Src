#include "test.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "servo.h"

// 定义云台舵机角度
#define PLATE1     177.0f
#define PLATE2     200.0f
#define PLATE3     218.0f

#define SONGKAI    45.0f
#define ZHUA      10.0f

void Test_1(void)
{   
    // 启动定时器PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    
    Servo_SetAngle1(PLATE3);
}
void Test_2(void)
{   
    // 启动定时器PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    
    Servo_SetAngle2(140.0f);
}
void Test_3(void)
{   
    // 启动定时器PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    
    Servo_SetAngle3(90.0f);
}