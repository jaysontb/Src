/**
  ******************************************************************************
  * @author  YOUR_NAME
  * @version V1.0
  * @date    2025-09-21
  * @brief   视觉通信中枢实现 - 适配LubanCat上位机
  ******************************************************************************
  */

#include "visual_comm.h"

/* 全局变量定义 */
VisualRxData_t VIS_RX;
uint8_t color_task[6] = {0};

//-------------------------------------------------------------------------------------------------------------------
//  @brief      视觉数据初始化
//  @return     void
//  @param      void
//  @note       初始化所有视觉接收数据为0
//-------------------------------------------------------------------------------------------------------------------
void Visual_Data_Init(void)
{
    // 物料位置数据清零
    VIS_RX.r_block_x = 0;
    VIS_RX.r_block_y = 0;
    VIS_RX.g_block_x = 0;
    VIS_RX.g_block_y = 0;
    VIS_RX.b_block_x = 0;
    VIS_RX.b_block_y = 0;
    
    // 凸台和凹槽数据清零
    VIS_RX.platform_x = 0;
    VIS_RX.platform_y = 0;
    VIS_RX.slot_x = 0;
    VIS_RX.slot_y = 0;
    
    // 色环和物料盘数据清零
    VIS_RX.rgb_circle_x = 0;
    VIS_RX.rgb_circle_y = 0;
    VIS_RX.rgb_circle_z = 0;
    VIS_RX.tray_center_x = 0;
    VIS_RX.tray_center_y = 0;
    
    // 控制指令清零
    VIS_RX.block_grab = 0;
    VIS_RX.block_order = 0;
    
    // 二维码数据清零
    for(uint8_t i = 0; i < 6; i++)
    {
        VIS_RX.qr_data[i] = 0;
        color_task[i] = 0;
    }
    VIS_RX.qr_valid = 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      发送空闲状态
//  @return     void
//  @param      void
//-------------------------------------------------------------------------------------------------------------------
void Visual_Send_Idle(void)
{
    uint8_t lubancat_data[4];
    
    lubancat_data[0] = 0x66;      // 帧头
    lubancat_data[1] = TASK_IDLE; // 任务码：空闲
    lubancat_data[2] = 0x00;      // 参数
    lubancat_data[3] = 0x00;      // 校验和
    
    HAL_UART_Transmit(&huart2, lubancat_data, 4, HAL_MAX_DELAY);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      请求识别二维码
//  @return     void
//  @param      void
//-------------------------------------------------------------------------------------------------------------------
void Visual_Send_QRCode_Request(void)
{
    uint8_t lubancat_data[4];
    
    lubancat_data[0] = 0x66;         // 帧头
    lubancat_data[1] = TASK_QR_CODE; // 任务码：二维码识别
    lubancat_data[2] = 0x00;         // 参数
    lubancat_data[3] = 0x00;         // 校验和
    
    HAL_UART_Transmit(&huart2, lubancat_data, 4, HAL_MAX_DELAY);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      请求识别物料盘
//  @return     void
//  @param      void
//-------------------------------------------------------------------------------------------------------------------
void Visual_Send_Tray_Request(void)
{
    uint8_t lubancat_data[4];
    
    lubancat_data[0] = 0x66;             // 帧头
    lubancat_data[1] = TASK_IDENTIFY_TRAY; // 任务码：物料盘识别
    lubancat_data[2] = 0x00;             // 参数
    lubancat_data[3] = lubancat_data[1] + lubancat_data[2]; // 校验和
    
    HAL_UART_Transmit(&huart2, lubancat_data, 4, HAL_MAX_DELAY);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      请求识别指定颜色物料
//  @return     void
//  @param      color: COLOR_RED/COLOR_GREEN/COLOR_BLUE
//-------------------------------------------------------------------------------------------------------------------
void Visual_Send_Material_Request(uint8_t color)
{
    uint8_t lubancat_data[4];
    
    lubancat_data[0] = 0x66;                  // 帧头
    lubancat_data[1] = TASK_IDENTIFY_MATERIAL; // 任务码：物料识别
    lubancat_data[2] = color;                 // 参数：颜色
    lubancat_data[3] = lubancat_data[1] + lubancat_data[2]; // 校验和
    
    HAL_UART_Transmit(&huart2, lubancat_data, 4, HAL_MAX_DELAY);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      请求识别指定颜色凸台
//  @return     void
//  @param      color: COLOR_RED/COLOR_GREEN/COLOR_BLUE
//-------------------------------------------------------------------------------------------------------------------
void Visual_Send_Platform_Request(uint8_t color)
{
    uint8_t lubancat_data[4];
    
    lubancat_data[0] = 0x66;           // 帧头
    lubancat_data[1] = TASK_PLATFORM;  // 任务码：凸台识别
    lubancat_data[2] = color;          // 参数：颜色
    lubancat_data[3] = lubancat_data[1] + lubancat_data[2]; // 校验和
    
    HAL_UART_Transmit(&huart2, lubancat_data, 4, HAL_MAX_DELAY);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      请求识别指定颜色凹槽
//  @return     void
//  @param      color: COLOR_RED/COLOR_GREEN/COLOR_BLUE
//-------------------------------------------------------------------------------------------------------------------
void Visual_Send_Slot_Request(uint8_t color)
{
    uint8_t lubancat_data[4];
    
    lubancat_data[0] = 0x66;        // 帧头
    lubancat_data[1] = TASK_SLOT;   // 任务码：凹槽识别
    lubancat_data[2] = color;       // 参数：颜色
    lubancat_data[3] = lubancat_data[1] + lubancat_data[2]; // 校验和
    
    HAL_UART_Transmit(&huart2, lubancat_data, 4, HAL_MAX_DELAY);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      请求识别色环位置
//  @return     void
//  @param      void
//-------------------------------------------------------------------------------------------------------------------
void Visual_Send_Rings_Request(void)
{
    uint8_t lubancat_data[4];
    
    lubancat_data[0] = 0x66;              // 帧头
    lubancat_data[1] = TASK_IDENTIFY_RINGS; // 任务码：色环识别
    lubancat_data[2] = 0x00;              // 参数
    lubancat_data[3] = lubancat_data[1] + lubancat_data[2]; // 校验和
    
    HAL_UART_Transmit(&huart2, lubancat_data, 4, HAL_MAX_DELAY);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      请求码垛定位
//  @return     void
//  @param      void
//-------------------------------------------------------------------------------------------------------------------
void Visual_Send_Stacking_Request(void)
{
    uint8_t lubancat_data[4];
    
    lubancat_data[0] = 0x66;            // 帧头
    lubancat_data[1] = TASK_STACKING_POS; // 任务码：码垛定位
    lubancat_data[2] = 0x00;            // 参数
    lubancat_data[3] = lubancat_data[1] + lubancat_data[2]; // 校验和
    
    HAL_UART_Transmit(&huart2, lubancat_data, 4, HAL_MAX_DELAY);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      发送调试指令
//  @return     void
//  @param      debug_cmd: 调试命令
//-------------------------------------------------------------------------------------------------------------------
void Visual_Send_Debug_Request(uint8_t debug_cmd)
{
    uint8_t lubancat_data[4];
    
    lubancat_data[0] = 0x66;        // 帧头
    lubancat_data[1] = TASK_DEBUG;  // 任务码：调试
    lubancat_data[2] = debug_cmd;   // 参数：调试命令
    lubancat_data[3] = lubancat_data[1] + lubancat_data[2]; // 校验和
    
    HAL_UART_Transmit(&huart2, lubancat_data, 4, HAL_MAX_DELAY);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      LubanCat数据解包
//  @return     void
//  @param      lubancat_data: 接收到的数据包
//  @note       解析LubanCat发送的各种识别结果
//-------------------------------------------------------------------------------------------------------------------
void Visual_Data_Unpack(uint8_t *lubancat_data)
{
    // 校验帧头
    if (lubancat_data[0] != 0x66)
        return;
    
    // 根据任务码解析数据
    switch(lubancat_data[1])
    {
        case TASK_IDENTIFY_TRAY:    // 物料盘位置
            if (lubancat_data[6] != 0x77)    // 校验帧尾
                return;
            VIS_RX.tray_center_x = (lubancat_data[2] << 8) | lubancat_data[3];
            VIS_RX.tray_center_y = (lubancat_data[4] << 8) | lubancat_data[5];
            break;
            
        case TASK_IDENTIFY_MATERIAL:    // 物料位置
            if (lubancat_data[7] != 0x77)    // 校验帧尾
                return;
            switch (lubancat_data[2])
            {
                case COLOR_RED:    // 红色物料
                    VIS_RX.r_block_x = (lubancat_data[3] << 8) | lubancat_data[4];
                    VIS_RX.r_block_y = (lubancat_data[5] << 8) | lubancat_data[6];
                    break;
                case COLOR_GREEN:  // 绿色物料
                    VIS_RX.g_block_x = (lubancat_data[3] << 8) | lubancat_data[4];
                    VIS_RX.g_block_y = (lubancat_data[5] << 8) | lubancat_data[6];
                    break;
                case COLOR_BLUE:   // 蓝色物料
                    VIS_RX.b_block_x = (lubancat_data[3] << 8) | lubancat_data[4];
                    VIS_RX.b_block_y = (lubancat_data[5] << 8) | lubancat_data[6];
                    break;
            }
            break;
            
        case TASK_PLATFORM:    // 凸台位置
            if (lubancat_data[6] != 0x77)    // 校验帧尾
                return;
            VIS_RX.platform_x = (lubancat_data[2] << 8) | lubancat_data[3];
            VIS_RX.platform_y = (lubancat_data[4] << 8) | lubancat_data[5];
            break;
            
        case TASK_SLOT:    // 凹槽位置
            if (lubancat_data[6] != 0x77)    // 校验帧尾
                return;
            VIS_RX.slot_x = (lubancat_data[2] << 8) | lubancat_data[3];
            VIS_RX.slot_y = (lubancat_data[4] << 8) | lubancat_data[5];
            break;
            
        case TASK_IDENTIFY_RINGS:    // 色环位置和Z轴偏差
            if (lubancat_data[8] != 0x77)    // 校验帧尾
                return;
            VIS_RX.rgb_circle_x = (lubancat_data[2] << 8) | lubancat_data[3];
            VIS_RX.rgb_circle_y = (lubancat_data[4] << 8) | lubancat_data[5];
            VIS_RX.rgb_circle_z = (int16_t)((lubancat_data[6] << 8) | lubancat_data[7]);
            break;
            
        case TASK_STACKING_POS:    // 码垛定位
            if (lubancat_data[8] != 0x77)    // 校验帧尾
                return;
            VIS_RX.rgb_circle_x = (lubancat_data[2] << 8) | lubancat_data[3];
            VIS_RX.rgb_circle_y = (lubancat_data[4] << 8) | lubancat_data[5];
            VIS_RX.rgb_circle_z = (int16_t)((lubancat_data[6] << 8) | lubancat_data[7]);
            break;
            
        case TASK_QR_CODE:    // 二维码数据
            if (lubancat_data[8] != 0x77)    // 校验帧尾
                return;
            
            // 和校验验证
            uint8_t checksum = 0;
            for(uint8_t i = 2; i < 8; i++)
            {
                checksum += lubancat_data[i];
            }
            
            if (checksum == 12)    // 校验通过
            {
                for (uint8_t i = 0; i < 6; i++)
                {
                    VIS_RX.qr_data[i] = lubancat_data[i + 2];
                    color_task[i] = lubancat_data[i + 2];
                }
                VIS_RX.qr_valid = 1;    // 设置数据有效标志
            }
            else
            {
                // 校验失败，清零数据
                for (uint8_t i = 0; i < 6; i++)
                {
                    VIS_RX.qr_data[i] = 0;
                    color_task[i] = 0;
                }
                VIS_RX.qr_valid = 0;
            }
            break;
            
        case TASK_DEBUG:    // 调试模式
            if (lubancat_data[3] != 0x77)    // 校验帧尾
                return;
            // 这里可以添加LED控制或其他调试功能
            break;
            
        default:
            break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      计算校验和
//  @return     uint8_t: 校验和值
//  @param      data: 数据指针
//  @param      len: 数据长度
//-------------------------------------------------------------------------------------------------------------------
uint8_t Visual_Calculate_Checksum(uint8_t *data, uint8_t len)
{
    uint8_t checksum = 0;
    for(uint8_t i = 0; i < len; i++)
    {
        checksum += data[i];
    }
    return checksum;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      验证数据帧
//  @return     uint8_t: 1-验证成功, 0-验证失败
//  @param      data: 数据指针
//  @param      expected_tail_pos: 期望的帧尾位置
//-------------------------------------------------------------------------------------------------------------------
uint8_t Visual_Verify_Frame(uint8_t *data, uint8_t expected_tail_pos)
{
    // 检查帧头
    if (data[0] != 0x66)
        return 0;
    
    // 检查帧尾
    if (data[expected_tail_pos] != 0x77)
        return 0;
    
    return 1;
}
