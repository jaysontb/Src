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
#include <motor.h>

/* ==================== 私有变量 ==================== */

// 当前机械爪高度
static float current_height_mm = 0.0f;

// 抓取次数计数器
static uint8_t pickup_counter = 0;

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

/**
 * @brief 重置抓取计数器
 * @note 在新的抓取流程开始前调用,重置计数器为0
 */
void Gripper_Reset_PickupCounter(void)
{
    pickup_counter = 0;
}

/**
 * @brief 获取当前抓取次数
 * @return 当前已执行的抓取次数
 */
uint8_t Gripper_Get_PickupCounter(void)
{
    return pickup_counter;
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
    Servo_SetAngle2(GRIPPER_CLAW_MAX_ANGLE);
}

/**
 * @brief 控制爪子闭合
 * @details 使用TIM1_CH2 PWM控制180°舵机,闭合到9°
 */
void Gripper_Claw_Close(void)
{
    // 设置爪子闭合
    Servo_SetAngle2(GRIPPER_CLAW_MIN_ANGLE);
}

/**
 * @brief 抓取并放置物块
 */
void Gripper_PickAndPlace(int plate_number)
{
    // 增加调用次数计数
    pickup_counter++;
    
    // === 第1步: 确保0位 ===
    Gripper_Pan_Rotate(0.0f);  // 旋转回抓取位置(0°)
    HAL_Delay(500);  // 等待舵机到位
    Gripper_Claw_Open();  // 张开爪子
    HAL_Delay(1000);  // 等待舵机到位
    
    // === 第2步: 降低并抓取物块 ===
    Gripper_Lift(HEIGHT_PICKUP_LOWER);  // 降到抓取高度(0mm)
    
    Gripper_Claw_Close();  // 闭合爪子夹紧
    HAL_Delay(1000);  // 等待夹紧(稍长确保夹稳)
    
    // === 第3步: 升高到最高点 ===
    Gripper_Lift(HEIGHT_TRANSPORT);  //
    HAL_Delay(500);
    Motor_Move_Forward(-80.0f, 0); // 后退80mm，避免碰撞
    
    // === 第4步: 旋转到指定物料盘 ===
    float target_angle;
    switch (plate_number) {
        case 1:
            target_angle = PAN_ANGLE_PLATE1;
            break;
        case 2:
            target_angle = PAN_ANGLE_PLATE2;
            break;
        case 3:
            target_angle = PAN_ANGLE_PLATE3;
            break;
        default:
            target_angle = PAN_ANGLE_PLATE2;  // 默认使用第2个盘
            break;
    }
    Gripper_Pan_Rotate(180.0f);  // 再转到目标角度
    HAL_Delay(1800);
    Gripper_Pan_Rotate(target_angle);  // 再转到目标角度
    HAL_Delay(700);  // 增加延时到2秒,确保270°舵机有足够时间到位
    
    // === 第5步: 释放物块 ===
    Gripper_Lift(HEIGHT_ASSEMBLY_L2);
    Gripper_Claw_Open();  // 张开爪子
    HAL_Delay(1000);  // 等待舵机到位
    
    // === 第6步: 复位 (第3次调用时跳过回原位) ===
    Gripper_Pan_Rotate(PAN_ANGLE_GRAB);  // 旋转回抓取位置(0°)
    HAL_Delay(1000);  // 等待舵机到位
    
    Gripper_Lift(HEIGHT_TRANSPORT);  // 回到最高
    
    // 仅在前两次调用时执行回原位操作
    if (pickup_counter < 3) {
        Motor_Move_Forward(80.0f, 0); // 回到原位
    }
    // 第3次调用不执行回原位,直接结束
}
