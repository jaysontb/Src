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
#include <stdio.h>
#include <string.h>

extern uint8_t visual_rx_complete_flag;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      等待视觉数据接收(带超时)
//  @return     uint8_t: 1-成功, 0-超时
//  @param      timeout_ms: 超时时间(毫秒)
//  @note       发送请求后调用,等待LubanCat返回数据
//-------------------------------------------------------------------------------------------------------------------
uint8_t Visual_Wait_Response(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    
    while ((HAL_GetTick() - start_tick) < timeout_ms)
    {
        if (visual_rx_complete_flag)
        {
            return 1;  // 接收成功
        }
        HAL_Delay(1);
    }
    
    return 0;  // 超时
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      打印坐标到串口(用于调试)
//  @return     void
//  @param      x, y: 坐标值
//  @note       通过USART1或printf输出到调试终端
//-------------------------------------------------------------------------------------------------------------------
void Visual_Print_Coord(uint16_t x, uint16_t y)
{
    char buffer[50];
    sprintf(buffer, "[VISUAL] X=%d, Y=%d\r\n", x, y);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 100);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      基础通信测试
//  @return     void
//  @param      void
//  @note       测试: 请求红色物料 → 接收坐标 → 打印结果
//-------------------------------------------------------------------------------------------------------------------
void Visual_Test_Basic(void)
{
    char msg[100];
    
    // ========== Test 1: Material Recognition ==========
    sprintf(msg, "\r\n=== Test 1: Red Material Recognition ===\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    // Send request: identify red material
    Visual_Send_Material_Request(COLOR_RED);
    
    // Wait for response (max 500ms)
    if (Visual_Wait_Response(500))
    {
        // Parse received data
        Visual_Data_Unpack(Visual_Get_RxBuffer());
        Visual_Clear_RxFlag();
        
        // Print result
        sprintf(msg, "Red Material Coord: X=%d, Y=%d\r\n", VIS_RX.r_block_x, VIS_RX.r_block_y);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    }
    else
    {
        sprintf(msg, "[ERROR] Receive timeout, check LubanCat connection\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    }
    
    HAL_Delay(1000);
    
    // ========== Test 2: Platform Recognition ==========
    sprintf(msg, "\r\n=== Test 2: Red Platform Recognition ===\r\n");
    
    // 发送前先清除标志
    Visual_Clear_RxFlag();
    
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    Visual_Send_Platform_Request(COLOR_RED);
    
    // 增加调试信息
    uint32_t start_time = HAL_GetTick();
    if (Visual_Wait_Response(1000))  // 增加超时时间到1秒
    {
        uint32_t elapsed = HAL_GetTick() - start_time;
        sprintf(msg, "[DEBUG] 接收成功,耗时%lums\r\n", elapsed);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
        
        // 打印接收到的原始数据
        uint8_t* rxbuf = Visual_Get_RxBuffer();
        sprintf(msg, "[DEBUG] RX Data: [0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X]\r\n",
                rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3], rxbuf[4], rxbuf[5], rxbuf[6]);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
        
        Visual_Data_Unpack(rxbuf);
        Visual_Clear_RxFlag();
        
        sprintf(msg, "Red Platform Coord: X=%d, Y=%d\r\n", VIS_RX.platform_x, VIS_RX.platform_y);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    }
    else
    {
        sprintf(msg, "[ERROR]\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
        
        // 检查标志位状态
        extern uint8_t visual_rx_complete_flag;
        sprintf(msg, "[DEBUG] rx_flag=%d\r\n", visual_rx_complete_flag);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    }
    
    HAL_Delay(1000);
    
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
//  @brief      循环测试(持续请求并打印坐标)
//  @return     void
//  @param      void
//  @note       用于验证通信稳定性
//-------------------------------------------------------------------------------------------------------------------
void Visual_Test_Loop(void)
{
    static uint8_t test_count = 0;
    char msg[100];
    
    // Test different color materials in each loop
    uint8_t colors[] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE};
    uint8_t color = colors[test_count % 3];
    
    Visual_Send_Material_Request(color);
    
    if (Visual_Wait_Response(300))
    {
        Visual_Data_Unpack(Visual_Get_RxBuffer());
        Visual_Clear_RxFlag();
        
        sprintf(msg, "[%d] Color%d: X=%d, Y=%d\r\n", test_count, color,
                (color == COLOR_RED) ? VIS_RX.r_block_x :
                (color == COLOR_GREEN) ? VIS_RX.g_block_x : VIS_RX.b_block_x,
                (color == COLOR_RED) ? VIS_RX.r_block_y :
                (color == COLOR_GREEN) ? VIS_RX.g_block_y : VIS_RX.b_block_y);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
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
