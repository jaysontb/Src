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
            float move_x = (VIS_RX.r_block_x - 320) * 0.5f;
            float move_y = (240 - VIS_RX.r_block_y) * 0.5f;
            
            // 执行移动(不抓取,仅定位测试)
            Motor_Move_Lateral(move_x, 0);
            Motor_Move_Forward(move_y, 0);
            
            break;
        }
        HAL_Delay(10);
    }
}
