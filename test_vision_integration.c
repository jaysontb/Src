/**
  ******************************************************************************
  * @file    test_vision_integration.c
  * @brief   视觉集成测试用例
  * @details 提供分步测试函数,验证视觉+机械爪集成功能
  ******************************************************************************
  */

#include "test.h"
#include "task_vision_gripper.h"
#include "visual_comm.h"
#include "gripper.h"
#include "motor.h"
#include "oled.h"
#include <math.h>
#include <stdio.h>

/* ==================== 视觉集成测试函数 ==================== */

/**
 * @brief 测试1: 视觉通信基础测试
 * @details 验证STM32与LubanCat的UART通信是否正常
 */
void Test_Vision_Communication(void)
{
    // 发送空闲指令测试
    Visual_Send_Idle();
    HAL_Delay(100);
    
    // 请求二维码测试
    Visual_Send_QRCode_Request();
    HAL_Delay(3000);  // 等待响应
    
    // 检查是否收到数据
    if (visual_rx_complete_flag) {
        // 成功收到数据
        Visual_Data_Unpack(Visual_Get_RxBuffer());
        Visual_Clear_RxFlag();
        
        // TODO: 添加OLED显示或串口打印调试信息
    }
}

/**
 * @brief 测试2: 单个物料识别与抓取
 * @details 测试视觉识别红色物料并抓取
 */
void Test_Vision_Pickup_Red(void)
{
    // 执行红色物料抓取
    bool result = Task_Vision_Pickup(COLOR_RED);
    
    if (result) {
        // 抓取成功,保持搬运姿态2秒供观察
        HAL_Delay(2000);
        
        // 复位
        Gripper_Pan_Rotate(PAN_ANGLE_GRAB);
        HAL_Delay(500);
        Gripper_Claw_Open();
        HAL_Delay(500);
        Gripper_Lift(HEIGHT_HOME);
    }
    // else: 失败则不动作
}

/**
 * @brief 测试3: 完整的抓取-放置流程
 * @details 抓取红色物料并放入物料盘1
 */
void Test_Vision_PickAndPlace_Red(void)
{
    bool result = Task_Vision_PickAndPlace(COLOR_RED);
    
    // TODO: 根据result显示成功/失败状态
    (void)result;
}

/**
 * @brief 测试4: 三色自动分拣
 * @details 完整流程测试,按红→绿→蓝顺序分拣
 */
void Test_Vision_Auto_Sorting(void)
{
    bool result = Task_Auto_Sorting();
    
    // TODO: 根据result显示成功/失败状态
    (void)result;
}

/**
 * @brief 测试5: 二维码识别
 * @details 读取二维码并打印结果
 */
void Test_Vision_QRCode(void)
{
    uint8_t qr_data[6] = {0};
    
    bool result = Task_Vision_ReadQRCode(qr_data);
    
    if (result) {
        // 成功读取二维码
        // TODO: 通过OLED或串口显示qr_data内容
        (void)qr_data;
    }
}

/**
 * @brief 测试6: 视觉引导底盘定位
 * @details 测试根据视觉坐标移动底盘的精度
 */
void Test_Vision_Positioning(void)
{
    // 请求识别红色物料
    Visual_Send_Material_Request(COLOR_RED);
    
    // 等待响应
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 3000) {
        if (visual_rx_complete_flag) {
            Visual_Data_Unpack(Visual_Get_RxBuffer());
            Visual_Clear_RxFlag();
            
            // 计算移动距离
            float move_x = (426 - VIS_RX.r_block_x ) * 0.28f;
            float move_y = (220 - VIS_RX.r_block_y) * 0.38f;

            // 执行移动(不抓取,仅定位测试)
            Motor_Move_Lateral(move_x, 0);
            Motor_Move_Forward(move_y, 0);
            
            break;
        }
        HAL_Delay(10);
    }
}

/**
 * @brief 统一的视觉对位接口 (支持物块/凸台/凹槽)
 * @param target_type 目标类型 (物块/凸台/凹槽)
 * @param color 目标颜色 (红/绿/蓝)
 * @return true=成功定位, false=失败
 * 
 * @details 功能特性:
 * - 多帧平均滤波: 减少噪声影响
 * - 迭代逼近算法: 自动修正偏差
 * - 自适应参数: 根据目标类型调整精度要求
 * - 防过冲机制: 使用衰减系数避免震荡
 * 
 * @example
 * // 对准物料台的红色物块
 * Vision_Align_Target(TARGET_MATERIAL_BLOCK, COLOR_RED);
 * 
 * // 对准测试区的绿色凸台
 * Vision_Align_Target(TARGET_TEST_PLATFORM, COLOR_GREEN);
 * 
 * // 对准装配区的蓝色凹槽
 * Vision_Align_Target(TARGET_ASSEMBLY_SLOT, COLOR_BLUE);
 */
bool Vision_Align_Target(TargetType_t target_type, ColorTarget_t color)
{
    // ========== 自适应参数配置 ==========
    uint8_t max_iterations;
    uint8_t sample_frames;
    float pixel_threshold;
    float min_move_dist;
    float damping_factor;
    
    // 根据目标类型选择参数
    switch (target_type) {
        case TARGET_MATERIAL_BLOCK:
            // 物块: 高精度要求,需精确抓取
            max_iterations = 2;
            sample_frames = 3;
            pixel_threshold = 6.0f;      // ±6px ≈ 2mm
            min_move_dist = 2.0f;
            damping_factor = 0.10f;
            break;
            
        case TARGET_TEST_PLATFORM:
            // 凸台: 中等精度,放置区域较大
            max_iterations = 2;
            sample_frames = 2;
            pixel_threshold = 10.0f;     // ±10px ≈ 4mm
            min_move_dist = 2.0f;
            damping_factor = 0.15f;
            break;
            
        case TARGET_ASSEMBLY_SLOT:
            // 凹槽: 最高精度,装配容差小
            max_iterations = 2;
            sample_frames = 4;
            pixel_threshold = 5.0f;      // ±5px ≈ 1.5mm
            min_move_dist = 1.5f;
            damping_factor = 0.10f;
            break;
            
        default:
            return false;
    }
    
    // 标定参数 (所有目标类型共用同一相机)
    const float PIXEL_TO_MM_X = 0.21f;       // X轴像素转毫米系数
    const float PIXEL_TO_MM_Y = 0.22f;       // Y轴像素转毫米系数
    const uint16_t IMAGE_CENTER_X = 498;     // 图像中心X
    const uint16_t IMAGE_CENTER_Y = 223;     // 图像中心Y
    uint32_t total_start = HAL_GetTick();
    
    // ========== 迭代逼近循环 ==========
    for (uint8_t iter = 0; iter < max_iterations; iter++) {
        // ========== 步骤1: 多帧采样平均 ==========
        uint32_t sum_x = 0, sum_y = 0;
        uint8_t valid_samples = 0;
        
        for (uint8_t sample = 0; sample < sample_frames; sample++) {
            // 根据目标类型发送不同的识别请求
            switch (target_type) {
                case TARGET_MATERIAL_BLOCK:
                    Visual_Send_Material_Request(color);
                    break;
                case TARGET_TEST_PLATFORM:
                    Visual_Send_Platform_Request(color);
                    break;
                case TARGET_ASSEMBLY_SLOT:
                    Visual_Send_Slot_Request(color);
                    break;
            }
            
            if (Visual_Wait_Response(1000)) {
                Visual_Data_Unpack(Visual_Get_RxBuffer());
                Visual_Clear_RxFlag();
                
                // 根据目标类型和颜色选择坐标
                uint16_t x = 0, y = 0;
                
                if (target_type == TARGET_MATERIAL_BLOCK) {
                    // 物块坐标
                    if (color == COLOR_RED) {
                        x = VIS_RX.r_block_x;
                        y = VIS_RX.r_block_y;
                    } else if (color == COLOR_GREEN) {
                        x = VIS_RX.g_block_x;
                        y = VIS_RX.g_block_y;
                    } else {
                        x = VIS_RX.b_block_x;
                        y = VIS_RX.b_block_y;
                    }
                } 
                else if (target_type == TARGET_TEST_PLATFORM) {
                    // 凸台坐标 (假设视觉模块返回特定颜色凸台位置)
                    x = VIS_RX.platform_x;
                    y = VIS_RX.platform_y;
                }
                else if (target_type == TARGET_ASSEMBLY_SLOT) {
                    // 凹槽坐标
                    x = VIS_RX.slot_x;
                    y = VIS_RX.slot_y;
                }
                
                // 过滤无效数据
                if (x > 0 && y > 0 && x < 640 && y < 480) {
                    sum_x += x;
                    sum_y += y;
                    valid_samples++;
                }
            }
            HAL_Delay(50);  // 帧间延时
        }
        
        if (valid_samples == 0) {
            // 未获取到有效数据,重试
            OLED_ShowString(1, 1, "Vision: No Data");
            HAL_Delay(500);
            continue;
        }
        
        // 计算平均坐标
        uint16_t avg_x = sum_x / valid_samples;
        uint16_t avg_y = sum_y / valid_samples;
        
        // 计算误差 (图像坐标系 -> 机器人坐标系)
        int16_t error_x = -(avg_x - IMAGE_CENTER_X);  // 正值=目标在右,需右移
        int16_t error_y = IMAGE_CENTER_Y - avg_y;     // 正值=目标在前,需前移
        
        // ========== 步骤3: 判断是否达到精度要求 ==========
        float abs_error_x = fabsf((float)error_x);
        float abs_error_y = fabsf((float)error_y);
        
        if (abs_error_x < pixel_threshold && abs_error_y < pixel_threshold) {
            // 成功到位!
            uint32_t elapsed = HAL_GetTick() - total_start;
            
            // 根据目标类型显示不同提示
            char msg[16];
            switch (target_type) {
                case TARGET_MATERIAL_BLOCK:
                    snprintf(msg, sizeof(msg), "Block OK %lums", elapsed);
                    break;
                case TARGET_TEST_PLATFORM:
                    snprintf(msg, sizeof(msg), "Plat OK %lums", elapsed);
                    break;
                case TARGET_ASSEMBLY_SLOT:
                    snprintf(msg, sizeof(msg), "Slot OK %lums", elapsed);
                    break;
            }
            OLED_ShowString(1, 1, msg);
            HAL_Delay(300);
            return true;
        }
        
        // ========== 步骤4: 计算移动距离(带衰减) ==========
        // 衰减系数随迭代次数递减,防止过冲
        float damping = 1.0f - (iter * (1.0f - damping_factor) / max_iterations);
        float move_x_mm = error_x * PIXEL_TO_MM_X * damping;
        float move_y_mm = error_y * PIXEL_TO_MM_Y * damping;
        
        // 最小移动限制(避免无意义微调)
        if (fabsf(move_x_mm) < min_move_dist && fabsf(move_y_mm) < min_move_dist) {
            HAL_Delay(500);
            return true;  // 误差在可接受范围内
        }
        
        // ========== 步骤5: 执行小步移动 ==========
        // 调试信息
        char debug_msg[16];
        snprintf(debug_msg, sizeof(debug_msg), "Adj:%.1f,%.1f", move_x_mm, move_y_mm);
        OLED_ShowString(1, 1, debug_msg);
        
        if (fabsf(move_x_mm) >= min_move_dist) {
            Motor_Move_Lateral(move_x_mm, 25);  // 使用低速精确移动
        }
        if (fabsf(move_y_mm) >= min_move_dist) {
            Motor_Move_Forward(move_y_mm, 25);
        }
        
        HAL_Delay(200);  // 稳定间隔
    }
    
    // 超过最大迭代次数仍未达标
    OLED_ShowString(1, 1, "Vision: Timeout");
    HAL_Delay(1000);
    return false;
}
