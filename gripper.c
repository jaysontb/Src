/**
  ******************************************************************************
  * @file    gripper.c
  * @brief   机械爪控制模块实现
  * @details 包含云台旋转、爪子开合、升降控制功能实现
  ******************************************************************************
  */

#include "gripper.h"
#include "Emm_V5.h"
#include "usart.h"
#include "servo.h"  // 云台和爪子舵机控制
#include "tim.h"    // 定时器PWM控制
#include <math.h>
#include <string.h>

/* ==================== 私有变量 ==================== */

// 当前机械爪高度
static float current_height_mm = 0.0f;

/* ==================== 私有函数声明 ==================== */

static void wait_motion_complete(uint32_t estimated_time_ms);

/* ==================== 公开函数实现 ==================== */

/**
 * @brief 控制机械爪升降到指定高度
 */
void Gripper_Lift(float height_mm)
{
    // 参数限制
    if (height_mm < 0.0f) height_mm = 0.0f;
    if (height_mm > GRIPPER_MAX_HEIGHT) height_mm = GRIPPER_MAX_HEIGHT;
    
    // 计算移动距离
    float delta_mm = height_mm - current_height_mm;
    
    // 距离太小则跳过
    if (fabs(delta_mm) < 0.1f) return;
    
    // 计算脉冲数和方向
    int32_t clk = (int32_t)(fabs(delta_mm) * GRIPPER_CLK_PER_MM);
    uint8_t dir = (delta_mm > 0) ? 0 : 1;  // 0=上升, 1=下降
    
    // 发送控制命令
    Emm_V5_Pos_Control(GRIPPER_LIFT_MOTOR_ADDR, dir, GRIPPER_DEFAULT_SPEED, 
                       GRIPPER_DEFAULT_ACC, clk, 0, 0);
    
    // 等待完成
    uint32_t time_ms = (uint32_t)(clk * 18.75f / GRIPPER_DEFAULT_SPEED);
    wait_motion_complete(time_ms);
    
    // 更新高度
    current_height_mm = height_mm;
}

/**
 * @brief 获取当前机械爪高度
 */
float Gripper_Get_Height(void)
{
    return current_height_mm;
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief 等待电机运动完成 (UART空闲中断优化版)
 * @details 使用中断标志+__WFI()低功耗等待,响应速度<1ms (原50ms)
 */
static void wait_motion_complete(uint32_t estimated_time_ms)
{
    extern volatile bool motor_arrived_flag;

    uint32_t timeout_ms = estimated_time_ms + 500;
    uint32_t start_tick = HAL_GetTick();
    uint32_t last_query_tick = start_tick;
    
    motor_arrived_flag = false;  // 清除到位标志
    rxFrameFlag = false;
    
    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        // 优化查询频率: 从50ms改为20ms,更快检测到位
        if ((HAL_GetTick() - last_query_tick) >= 20) {
            last_query_tick = HAL_GetTick();
            Emm_V5_Read_Sys_Params(GRIPPER_LIFT_MOTOR_ADDR, S_FLAG);
            
            // 等待UART接收完成或到位标志置位
            uint32_t wait_start = HAL_GetTick();
            while ((HAL_GetTick() - wait_start) < 20) {  // 最多等20ms
                if (motor_arrived_flag) {  // 中断已检测到到位
                    motor_arrived_flag = false;
                    return;
                }
                __WFI();  // 进入低功耗模式,等待任意中断唤醒
            }
            
            // 兼容旧方式: 如果中断未检测到,手动检查rxCmd
            if (rxFrameFlag && rxCmd[1] == 0xFD && rxCmd[2] == 0x3A) {
                if (rxCmd[4] & 0x01) {
                    rxFrameFlag = false;
                    return;
                }
                rxFrameFlag = false;
            }
        } else {
            __WFI();  // 查询间隔内休眠,降低CPU占用
        }
    }
    
    // 超时清除标志
    motor_arrived_flag = false;
    rxFrameFlag = false;
}

/* ==================== 云台旋转控制 ==================== */

/**
 * @brief 控制云台旋转到指定角度
 * @details 使用TIM1_CH1 PWM控制270°舵机
 */
void Gripper_Pan_Rotate(float angle_deg)
{
    // 参数限制
    if (angle_deg < GRIPPER_PAN_MIN_ANGLE) angle_deg = GRIPPER_PAN_MIN_ANGLE;
    if (angle_deg > GRIPPER_PAN_MAX_ANGLE) angle_deg = GRIPPER_PAN_MAX_ANGLE;
    
    // 启动PWM输出(首次调用时需要)
    static bool pwm_started = false;
    if (!pwm_started) {
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        pwm_started = true;
    }
    
    // 设置云台角度
    Servo_SetAngle1(angle_deg);
}

/* ==================== 爪子开合控制 ==================== */

/**
 * @brief 控制爪子张开
 * @details 使用TIM1_CH2 PWM控制180°舵机,张开到45°
 */
void Gripper_Claw_Open(void)
{
    // 启动PWM输出(首次调用时需要)
    static bool pwm_started = false;
    if (!pwm_started) {
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        pwm_started = true;
    }
    
    // 设置爪子张开
    Servo_SetAngle2(GRIPPER_CLAW_MAX_ANGLE);
}

/**
 * @brief 控制爪子闭合
 * @details 使用TIM1_CH2 PWM控制180°舵机,闭合到9°
 */
void Gripper_Claw_Close(void)
{
    // 启动PWM输出(首次调用时需要)
    static bool pwm_started = false;
    if (!pwm_started) {
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        pwm_started = true;
    }
    
    // 设置爪子闭合
    Servo_SetAngle2(GRIPPER_CLAW_MIN_ANGLE);
}
