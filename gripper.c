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
    if (fabs(delta_mm) < 0.5f) return;
    
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
 * @brief 等待电机运动完成
 */
static void wait_motion_complete(uint32_t estimated_time_ms)
{
    uint32_t timeout_ms = estimated_time_ms * 2 + 1000;
    uint32_t start_tick = HAL_GetTick();
    uint32_t last_query_tick = start_tick;
    
    rxFrameFlag = false;
    
    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        if ((HAL_GetTick() - last_query_tick) >= 50) {
            last_query_tick = HAL_GetTick();
            Emm_V5_Read_Sys_Params(GRIPPER_LIFT_MOTOR_ADDR, S_FLAG);
            HAL_Delay(10);
            
            if (rxFrameFlag && rxCmd[1] == 0xFD && rxCmd[2] == 0x3A) {
                if (rxCmd[4] & 0x01) {
                    rxFrameFlag = false;
                    return;
                }
                rxFrameFlag = false;
            }
        }
        HAL_Delay(5);
    }
}

/* ==================== 云台旋转控制 (待实现) ==================== */

// 后续添加云台旋转相关函数

/* ==================== 爪子开合控制 (待实现) ==================== */

// 后续添加爪子开合相关函数
