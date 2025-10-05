/**
  ******************************************************************************
  * @file    task_vision_gripper.c
  * @brief   视觉引导的机械爪任务流程
  * @details 集成视觉识别与机械爪控制,实现自动化抓取放置
  ******************************************************************************
  */

#include "task_vision_gripper.h"
#include "visual_comm.h"
#include "gripper.h"
#include "motor.h"
#include <string.h>
#include <stdio.h>

/* ==================== 私有变量 ==================== */

// 物料盘位置映射 (云台角度)
static const float PLATE_ANGLES[3] = {
    PAN_ANGLE_PLATE1,  // 185°
    PAN_ANGLE_PLATE2,  // 206°
    PAN_ANGLE_PLATE3   // 224°
};

/* ==================== 私有函数声明 ==================== */

static bool wait_visual_response(uint32_t timeout_ms);
static uint8_t map_color_to_plate_index(ColorTarget_t color);

/* ==================== 公开函数实现 ==================== */

/**
 * @brief 视觉引导抓取任务
 * @details 完整流程:
 *   1. 发送视觉识别请求
 *   2. 等待LubanCat返回物料坐标
 *   3. 根据坐标控制底盘移动到抓取位置
 *   4. 执行机械爪抓取动作
 * @param color 目标物料颜色(红/绿/蓝)
 * @return true=成功, false=失败
 */
bool Task_Vision_Pickup(ColorTarget_t color)
{
    // 步骤1: 请求视觉识别物料位置
    Visual_Send_Material_Request(color);
    
    // 步骤2: 等待视觉返回结果
    if (!wait_visual_response(3000)) {
        return false;  // 超时失败
    }
    
    // 步骤3: 获取物料坐标
    uint16_t target_x = 0, target_y = 0;
    switch (color) {
        case COLOR_RED:
            target_x = VIS_RX.r_block_x;
            target_y = VIS_RX.r_block_y;
            break;
        case COLOR_GREEN:
            target_x = VIS_RX.g_block_x;
            target_y = VIS_RX.g_block_y;
            break;
        case COLOR_BLUE:
            target_x = VIS_RX.b_block_x;
            target_y = VIS_RX.b_block_y;
            break;
        default:
            return false;
    }
    
    // 步骤4: 坐标转换为底盘移动距离 (像素→毫米)
    // TODO: 根据实际标定结果调整转换系数
    float move_x_mm = (target_x - 320) * 0.5f;  // 假设640×480分辨率,中心为(320,240)
    float move_y_mm = (240 - target_y) * 0.5f;  // Y轴向下为正,需要反转
    
    // 步骤5: 底盘移动到物料上方
    Motor_Move_Lateral(move_x_mm, 0);   // 左右对齐
    Motor_Move_Forward(move_y_mm, 0);   // 前后对齐
    
    // 步骤6: 执行抓取动作
    Gripper_Pan_Rotate(PAN_ANGLE_GRAB);  // 云台转到抓取位置
    HAL_Delay(300);
    
    Gripper_Claw_Open();  // 张开爪子
    HAL_Delay(500);
    
    Gripper_Lift(HEIGHT_PICKUP_LOWER);  // 降到抓取高度
    
    Gripper_Claw_Close();  // 夹紧物料
    HAL_Delay(800);
    
    Gripper_Lift(HEIGHT_TRANSPORT);  // 升到搬运高度
    
    return true;
}

/**
 * @brief 视觉引导放置任务
 * @details 将已抓取的物料放入指定物料盘
 * @param color 物料颜色,决定放入哪个物料盘
 * @return true=成功, false=失败
 */
bool Task_Vision_Place(ColorTarget_t color)
{
    // 步骤1: 根据颜色选择物料盘
    uint8_t plate_index = map_color_to_plate_index(color);
    if (plate_index >= 3) {
        return false;  // 无效颜色
    }
    
    // 步骤2: 旋转云台到目标物料盘
    Gripper_Pan_Rotate(PLATE_ANGLES[plate_index]);
    HAL_Delay(1000);  // 等待舵机到位
    
    // 步骤3: 释放物料
    Gripper_Claw_Open();
    HAL_Delay(500);
    
    // 步骤4: 复位
    Gripper_Pan_Rotate(PAN_ANGLE_GRAB);
    HAL_Delay(500);
    
    Gripper_Lift(HEIGHT_HOME);
    
    return true;
}

/**
 * @brief 完整的视觉引导抓取-放置流程
 * @details 自动化完成: 识别→移动→抓取→放置
 * @param color 目标物料颜色
 * @return true=成功, false=失败
 */
bool Task_Vision_PickAndPlace(ColorTarget_t color)
{
    // 执行抓取
    if (!Task_Vision_Pickup(color)) {
        return false;
    }
    
    HAL_Delay(200);  // 稳定间隔
    
    // 执行放置
    if (!Task_Vision_Place(color)) {
        return false;
    }
    
    return true;
}

/**
 * @brief 二维码识别任务
 * @details 读取二维码并返回6位数据
 * @param qr_result 输出6位二维码数据
 * @return true=成功, false=失败
 */
bool Task_Vision_ReadQRCode(uint8_t qr_result[6])
{
    // 步骤1: 请求二维码识别
    Visual_Send_QRCode_Request();
    
    // 步骤2: 等待视觉返回结果
    if (!wait_visual_response(5000)) {  // 二维码识别较慢,给5秒
        return false;
    }
    
    // 步骤3: 检查数据有效性
    if (!VIS_RX.qr_valid) {
        return false;
    }
    
    // 步骤4: 复制结果
    memcpy(qr_result, VIS_RX.qr_data, 6);
    
    return true;
}

/**
 * @brief 三色物料自动分拣任务
 * @details 按红→绿→蓝顺序依次抓取并放入对应物料盘
 * @return true=成功, false=失败
 */
bool Task_Auto_Sorting(void)
{
    const ColorTarget_t colors[3] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE};
    
    for (uint8_t i = 0; i < 3; i++) {
        if (!Task_Vision_PickAndPlace(colors[i])) {
            return false;  // 某一步失败则中止
        }
        HAL_Delay(500);  // 任务间隔
    }
    
    return true;
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief 等待视觉响应
 * @param timeout_ms 超时时间(毫秒)
 * @return true=收到数据, false=超时
 */
static bool wait_visual_response(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    
    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        if (visual_rx_complete_flag) {
            // 解析接收到的数据
            Visual_Data_Unpack(Visual_Get_RxBuffer());
            Visual_Clear_RxFlag();
            return true;
        }
        HAL_Delay(10);  // 降低轮询频率
    }
    
    return false;  // 超时
}

/**
 * @brief 颜色映射到物料盘索引
 * @param color 颜色枚举
 * @return 物料盘索引(0-2), 失败返回0xFF
 */
static uint8_t map_color_to_plate_index(ColorTarget_t color)
{
    switch (color) {
        case COLOR_RED:   return 0;  // 红色→物料盘1 (185°)
        case COLOR_GREEN: return 1;  // 绿色→物料盘2 (206°)
        case COLOR_BLUE:  return 2;  // 蓝色→物料盘3 (224°)
        default:          return 0xFF;
    }
}
