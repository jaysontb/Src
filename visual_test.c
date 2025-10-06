/**
  ******************************************************************************
  * @file    visual_test.c
  * @brief   视觉系统联调测试代码
  * @author  YOUR_NAME
  * @date    2025-10-03
  ******************************************************************************
  * @description
  * 本文件提供完整的视觉通信测试流程,包括:
  * 1. 基础通信测试(发送请求→接收响应)
  * 2. 物料识别测试
  * 3. 凸台/凹槽定位测试
  * 4. 二维码识别测试
  * 
  * 使用方法:
  * 在main.c中调用 Visual_Test_Basic() 进行基础测试
  ******************************************************************************
  */

#include "main.h"
#include "visual_comm.h"
#include "usart.h"
#include "oled.h"  // 添加OLED头文件
#include <stdio.h>
#include <string.h>

extern uint8_t visual_rx_complete_flag;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      在OLED上显示坐标
//  @return     void
//  @param      x, y: 坐标值
//  @param      line: OLED行号(1-4)
//  @note       在OLED上显示视觉识别的坐标
//-------------------------------------------------------------------------------------------------------------------
void Visual_Show_Coord_OLED(uint16_t x, uint16_t y, uint8_t line)
{
    char buffer[20];
    
    // 显示X坐标
    OLED_ShowString(line, 1, "X:");
    sprintf(buffer, "%d   ", x);  // 后面加空格清除旧数据
    OLED_ShowString(line, 3, buffer);
    
    // 显示Y坐标
    OLED_ShowString(line, 9, "Y:");
    sprintf(buffer, "%d   ", y);
    OLED_ShowString(line, 11, buffer);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      基础通信测试
//  @return     void
//  @param      void
//  @note       测试: 请求红色物料 → 接收坐标 → OLED显示结果
//-------------------------------------------------------------------------------------------------------------------
void Visual_Test_Basic(void)
{
    // 清空OLED并显示标题
    OLED_Clear();
    OLED_ShowString(1, 1, "Vision Test");
    HAL_Delay(500);
    
    // ========== Test 1: Material Recognition ==========
    OLED_ShowString(2, 1, "Test1:Material");
    
    // Send request: identify red material
    Visual_Send_Material_Request(COLOR_RED);
    
    // Wait for response (max 500ms)
    if (Visual_Wait_Response(1000))
    {
        // Parse received data
        Visual_Data_Unpack(Visual_Get_RxBuffer());
        Visual_Clear_RxFlag();
        
        // Display result on OLED
        Visual_Show_Coord_OLED(VIS_RX.r_block_x, VIS_RX.r_block_y, 3);
    }
    else
    {
        OLED_ShowString(3, 1, "ERROR:Timeout");
    }
    
    HAL_Delay(2000);
    
    // ========== Test 2: Platform Recognition ==========
    // OLED_Clear();
    // OLED_ShowString(1, 1, "Test2:Platform");
    
    // 发送前先清除标志
    // Visual_Clear_RxFlag();
    
    // Visual_Send_Platform_Request(COLOR_RED);
    
    // if (Visual_Wait_Response(1000))  // 增加超时时间到1秒
    // {
    //     Visual_Data_Unpack(Visual_Get_RxBuffer());
    //     Visual_Clear_RxFlag();
        
    //     // Display platform coordinates on OLED
    //     Visual_Show_Coord_OLED(VIS_RX.platform_x, VIS_RX.platform_y, 2);
    //     OLED_ShowString(3, 1, "Success!");
    // }
    // else
    // {
    //     OLED_ShowString(2, 1, "ERROR:Timeout");
    // }
    
    // HAL_Delay(2000);
    
    // ========== Test 3: QR Code Recognition ==========
    // sprintf(msg, "\r\n=== Test 3: QR Code Recognition ===\r\n");
    // HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    // Visual_Send_QRCode_Request();
    
    // if (Visual_Wait_Response(500))
    // {
    //     Visual_Data_Unpack(Visual_Get_RxBuffer());
    //     Visual_Clear_RxFlag();
        
    //     if (VIS_RX.qr_valid)
    //     {
    //         sprintf(msg, "QR Code Data: %c%c%c%c%c%c\r\n", 
    //                 VIS_RX.qr_data[0], VIS_RX.qr_data[1], VIS_RX.qr_data[2],
    //                 VIS_RX.qr_data[3], VIS_RX.qr_data[4], VIS_RX.qr_data[5]);
    //         HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    //     }
    //     else
    //     {
    //         sprintf(msg, "[WARN] QR Code data invalid\r\n");
    //         HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    //     }
    // }
    // else
    // {
    //     sprintf(msg, "[ERROR] 接收超时\r\n");
    //     HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    // }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      循环测试(持续请求并OLED显示坐标)
//  @return     void
//  @param      void
//  @note       用于验证通信稳定性,OLED实时显示坐标
//-------------------------------------------------------------------------------------------------------------------
void Visual_Test_Loop(void)
{
    static uint8_t test_count = 0;
    static uint8_t current_color = COLOR_RED;
    
    // Test different color materials in each loop
    uint8_t colors[] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE};
    static char color_names[3][3] = {"R:", "G:", "B:"};  // 改为非const
    
    current_color = colors[test_count % 3];
    
    // 显示当前测试信息
    OLED_Clear();
    OLED_ShowString(1, 1, "Loop Test");
    OLED_ShowString(1, 11, color_names[test_count % 3]);
    OLED_ShowNum(1, 14, test_count, 3);
    
    Visual_Send_Material_Request(current_color);
    
    if (Visual_Wait_Response(500))
    {
        Visual_Data_Unpack(Visual_Get_RxBuffer());
        Visual_Clear_RxFlag();
        
        // 显示对应颜色的坐标
        uint16_t x, y;
        if (current_color == COLOR_RED) {
            x = VIS_RX.r_block_x;
            y = VIS_RX.r_block_y;
        } else if (current_color == COLOR_GREEN) {
            x = VIS_RX.g_block_x;
            y = VIS_RX.g_block_y;
        } else {
            x = VIS_RX.b_block_x;
            y = VIS_RX.b_block_y;
        }
        
        Visual_Show_Coord_OLED(x, y, 2);
        OLED_ShowString(3, 1, "OK");
    }
    else
    {
        OLED_ShowString(2, 1, "Timeout");
    }
    
    test_count++;
    HAL_Delay(500);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      完整任务流程模拟
//  @return     void
//  @param      void
//  @note       模拟真实比赛流程: 读二维码 → 识别物料 → 定位凸台
//-------------------------------------------------------------------------------------------------------------------
void Visual_Test_FullTask(void)
{
    char msg[100];
    
    // Step 1: Read QR code to get task sequence
    sprintf(msg, "\r\n======== Start Task Flow ========\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    sprintf(msg, "[Step 1] Reading QR code...\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    Visual_Send_QRCode_Request();
    if (!Visual_Wait_Response(800))
    {
        sprintf(msg, "[ERROR] QR code read failed!\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
        return;
    }
    
    Visual_Data_Unpack(Visual_Get_RxBuffer());
    Visual_Clear_RxFlag();
    
    if (!VIS_RX.qr_valid)
    {
        sprintf(msg, "[ERROR] QR code data invalid!\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
        return;
    }
    
    sprintf(msg, "Task Sequence: %c%c%c%c%c%c\r\n", 
            VIS_RX.qr_data[0], VIS_RX.qr_data[1], VIS_RX.qr_data[2],
            VIS_RX.qr_data[3], VIS_RX.qr_data[4], VIS_RX.qr_data[5]);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    HAL_Delay(1000);
    
    // Step 2: Identify material according to first task
    sprintf(msg, "\r\n[Step 2] Identifying 1st material...\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    // Assume first character is color code (R/G/B)
    uint8_t target_color = COLOR_RED;  // Simplified, should parse QR data
    
    Visual_Send_Material_Request(target_color);
    if (!Visual_Wait_Response(500))
    {
        sprintf(msg, "[ERROR] Material recognition timeout!\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
        return;
    }
    
    Visual_Data_Unpack(Visual_Get_RxBuffer());
    Visual_Clear_RxFlag();
    
    sprintf(msg, "Material Coord: X=%d, Y=%d\r\n", VIS_RX.r_block_x, VIS_RX.r_block_y);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    // TODO: Should call motion control to move to material position
    sprintf(msg, "[TODO] Moving to material position and grabbing...\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    HAL_Delay(2000);  // Simulate motion time
    
    // Step 3: Locate platform
    sprintf(msg, "\r\n[Step 3] Locating red platform...\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    Visual_Send_Platform_Request(COLOR_RED);
    if (!Visual_Wait_Response(500))
    {
        sprintf(msg, "[ERROR] Platform recognition timeout!\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
        return;
    }
    
    Visual_Data_Unpack(Visual_Get_RxBuffer());
    Visual_Clear_RxFlag();
    
    sprintf(msg, "Platform Coord: X=%d, Y=%d\r\n", VIS_RX.platform_x, VIS_RX.platform_y);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    // TODO: 这里应该调用运动控制函数移动到凸台并放置
    sprintf(msg, "[TODO] 移动到凸台位置并放置物料...\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    HAL_Delay(2000);  // 模拟运动时间
    
    sprintf(msg, "\r\n======== 任务完成! ========\r\n\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      OLED实时显示完整视觉信息
//  @return     void
//  @param      void
//  @note       在OLED上显示所有视觉识别结果,适合实时监控
//-------------------------------------------------------------------------------------------------------------------
void Visual_Display_All_Info(void)
{
    OLED_Clear();
    
    // 第1行:标题
    OLED_ShowString(1, 1, "Vision Monitor");
    
    // 第2行:红色物料坐标
    OLED_ShowString(2, 1, "R:");
    if (VIS_RX.r_block_x > 0 && VIS_RX.r_block_y > 0) {
        Visual_Show_Coord_OLED(VIS_RX.r_block_x, VIS_RX.r_block_y, 2);
    } else {
        OLED_ShowString(2, 3, "---");
    }
    
    // 第3行:凸台坐标
    OLED_ShowString(3, 1, "P:");
    if (VIS_RX.platform_x > 0 && VIS_RX.platform_y > 0) {
        Visual_Show_Coord_OLED(VIS_RX.platform_x, VIS_RX.platform_y, 3);
    } else {
        OLED_ShowString(3, 3, "---");
    }
    
    // 第4行:凹槽坐标
    OLED_ShowString(4, 1, "S:");
    if (VIS_RX.slot_x > 0 && VIS_RX.slot_y > 0) {
        Visual_Show_Coord_OLED(VIS_RX.slot_x, VIS_RX.slot_y, 4);
    } else {
        OLED_ShowString(4, 3, "---");
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      单次快速测试(推荐用于运动控制集成)
//  @return     uint8_t: 1-成功, 0-失败
//  @param      color: 颜色码 (COLOR_RED/GREEN/BLUE)
//  @param      line: OLED显示行号
//  @note       发送请求→等待→显示,一气呵成,适合在主循环中调用
//-------------------------------------------------------------------------------------------------------------------
uint8_t Visual_Quick_Test(uint8_t color, uint8_t line)
{
    static char color_str[3][2] = {"R", "G", "B"};  // 改为非const
    
    // 显示请求状态
    OLED_ShowString(line, 1, color_str[color-1]);
    OLED_ShowString(line, 2, ":Wait..");
    
    // 发送请求
    Visual_Send_Material_Request(color);
    
    // 等待响应
    if (Visual_Wait_Response(500))
    {
        Visual_Data_Unpack(Visual_Get_RxBuffer());
        Visual_Clear_RxFlag();
        
        // 显示坐标
        uint16_t x, y;
        if (color == COLOR_RED) {
            x = VIS_RX.r_block_x; y = VIS_RX.r_block_y;
        } else if (color == COLOR_GREEN) {
            x = VIS_RX.g_block_x; y = VIS_RX.g_block_y;
        } else {
            x = VIS_RX.b_block_x; y = VIS_RX.b_block_y;
        }
        
        Visual_Show_Coord_OLED(x, y, line);
        return 1;  // 成功
    }
    else
    {
        OLED_ShowString(line, 2, ":Error");
        return 0;  // 失败
    }
}
