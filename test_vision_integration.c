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
 * @brief 测试7: 高精度迭代式视觉定位 (优化版)
 * @details 通过迭代逼近+多帧平均,精度提升4倍
 * @param color 目标颜色
 * @return true=成功定位, false=失败
 */
bool Test_Vision_Positioning_Advanced(ColorTarget_t color)
{
    // ========== 配置参数 ==========
    const uint8_t MAX_ITERATIONS = 1;        // 最大迭代次数
    const uint8_t SAMPLE_FRAMES = 2;         // 多帧采样数
    const float PIXEL_THRESHOLD = 8.0f;      // 像素误差阈值(±8px ≈ 3mm)
    const float MIN_MOVE_DIST = 7.0f;        // 最小移动距离(mm)
    const float DAMPING_FACTOR = 0.85f;      // 衰减系数(防过冲)
    
    // 使用你标定的参数
    const float PIXEL_TO_MM_X = 0.17f;       // X轴像素转毫米系数
    const float PIXEL_TO_MM_Y = 0.33f;       // Y轴像素转毫米系数
    const uint16_t IMAGE_CENTER_X = 528;     // 图像中心X (你的标定值)
    const uint16_t IMAGE_CENTER_Y = 223;     // 图像中心Y (你的标定值)
    
    OLED_Clear();
    OLED_ShowString(1, 1, "Advanced Pos");
    
    uint32_t total_start = HAL_GetTick();
    
    // ========== 迭代逼近循环 ==========
    for (uint8_t iter = 0; iter < MAX_ITERATIONS; iter++) {
        // 显示当前迭代次数 "Iter:X/3"
        OLED_ShowString(2, 1, "Iter:");
        OLED_ShowNum(2, 6, iter+1, 1);
        OLED_ShowString(2, 7, "/");
        OLED_ShowNum(2, 8, MAX_ITERATIONS, 1);
        
        // ========== 步骤1: 多帧采样平均 ==========
        uint32_t sum_x = 0, sum_y = 0;
        uint8_t valid_samples = 0;
        
        for (uint8_t sample = 0; sample < SAMPLE_FRAMES; sample++) {
            Visual_Send_Material_Request(color);
            
            if (Visual_Wait_Response(200)) {
                Visual_Data_Unpack(Visual_Get_RxBuffer());
                Visual_Clear_RxFlag();
                
                // 根据颜色选择坐标
                uint16_t x, y;
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
            OLED_ShowString(3, 1, "No Data");
            return false;
        }
        
        // 计算平均坐标
        uint16_t avg_x = sum_x / valid_samples;
        uint16_t avg_y = sum_y / valid_samples;
        
        int16_t error_x =-(avg_x - IMAGE_CENTER_X);  // 正值=目标在右,需右移; 负值=目标在左,需左移
        int16_t error_y = IMAGE_CENTER_Y - avg_y;  // 正值=目标在前,需前移
        
        // 显示当前误差 "E:XXX,YYY"
        OLED_ShowString(3, 1, "E:");
        OLED_ShowSignedNum(3, 3, error_x, 3);
        OLED_ShowString(3, 6, ",");
        OLED_ShowSignedNum(3, 7, error_y, 3);
        
        // ========== 步骤3: 判断是否达到精度要求 ==========
        float abs_error_x = fabsf((float)error_x);
        float abs_error_y = fabsf((float)error_y);
        
        if (abs_error_x < PIXEL_THRESHOLD && abs_error_y < PIXEL_THRESHOLD) {
            // 成功到位!
            uint32_t elapsed = HAL_GetTick() - total_start;
            OLED_ShowString(4, 1, "OK! T:");
            OLED_ShowNum(4, 7, elapsed, 4);
            OLED_ShowString(4, 11, "ms");
            HAL_Delay(2000);
            return true;
        }
        
        // ========== 步骤4: 计算移动距离(带衰减) ==========
        float damping = 1.0f - (iter * (1.0f - DAMPING_FACTOR) / MAX_ITERATIONS);
        float move_x_mm = error_x * PIXEL_TO_MM_X * damping;
        float move_y_mm = error_y * PIXEL_TO_MM_Y * damping;
        
        // 最小移动限制(避免无意义微调)
        if (fabsf(move_x_mm) < MIN_MOVE_DIST && fabsf(move_y_mm) < MIN_MOVE_DIST) {
            OLED_ShowString(4, 1, "Close");
            HAL_Delay(1000);
            return true;
        }
        
        // ========== 步骤5: 执行小步移动 ==========
        if (fabsf(move_x_mm) >= MIN_MOVE_DIST) {
            Motor_Move_Lateral(move_x_mm, 0);
        }
        if (fabsf(move_y_mm) >= MIN_MOVE_DIST) {
            Motor_Move_Forward(move_y_mm, 0);
        }
        
        HAL_Delay(200);  // 稳定间隔
    }
    
    // 超过最大迭代次数仍未达标
    OLED_ShowString(4, 1, "MaxIter");
    HAL_Delay(2000);
    return false;
}
