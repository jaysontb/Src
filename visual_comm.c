/**
  ******************************************************************************
  * @file           : visual_comm.c
  * @brief          : 视觉通信模块 - LubanCat视觉系统通信接口
  * @author         : Auto-generated
  * @version        : V1.0
  * @date           : 2025-01-03
  * @note           : 实现STM32与LubanCat2视觉系统的串口通信
  *                   协议格式: [0x66, unit, target, 0x77]
  ******************************************************************************
  */

#include "visual_comm.h"

/* 全局变量定义 */
VisualRxData_t VIS_RX;
uint8_t color_task[6] = {0};

/* ==================== 串口接收状态机相关变量 ==================== */
typedef enum
{
    RX_STATE_WAIT_HEADER,   // 等待帧头0x66
    RX_STATE_GET_UNIT,      // 接收unit(任务码)
    RX_STATE_GET_DATA,      // 接收数据部分
    RX_STATE_VERIFY_TAIL    // 验证帧尾0x77
} RxState_t;

static RxState_t rx_state = RX_STATE_WAIT_HEADER;
static uint8_t rx_buffer[10];        // 接收缓冲区(最长9字节)
static uint8_t rx_index = 0;         // 当前接收索引
static uint8_t rx_expected_len = 0;  // 期望数据长度
static uint8_t rx_byte;              // 单字节接收缓存
uint8_t visual_rx_complete_flag = 0; // 接收完成标志(供main.c使用)

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
    lubancat_data[1] = TASK_IDLE; // 任务码:空闲
    lubancat_data[2] = 0x00;      // 参数
    lubancat_data[3] = 0x77;      // 帧尾(固定0x77)
    
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
    lubancat_data[1] = TASK_QR_CODE; // 任务码:二维码识别
    lubancat_data[2] = 0x00;         // 参数
    lubancat_data[3] = 0x77;         // 帧尾(固定0x77)
    
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
    lubancat_data[1] = TASK_IDENTIFY_MATERIAL; // 任务码:物料识别
    lubancat_data[2] = color;                 // 参数:颜色
    lubancat_data[3] = 0x77;                  // 帧尾(固定0x77)
    
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
    lubancat_data[1] = TASK_PLATFORM;  // 任务码:凸台识别
    lubancat_data[2] = color;          // 参数:颜色
    lubancat_data[3] = 0x77;           // 帧尾(固定0x77)
    
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
    lubancat_data[1] = TASK_SLOT;   // 任务码:凹槽识别
    lubancat_data[2] = color;       // 参数:颜色
    lubancat_data[3] = 0x77;        // 帧尾(固定0x77)
    
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
            
        case TASK_QR_CODE:    // 二维码数据
        {
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
        }
            
        default:
            break;
    }
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

//-------------------------------------------------------------------------------------------------------------------
//  @brief      启动视觉串口接收
//  @return     void
//  @param      void
//  @note       在main.c初始化时调用,启动单字节中断接收
//-------------------------------------------------------------------------------------------------------------------
void Visual_UART_Start_Receive(void)
{
    rx_state = RX_STATE_WAIT_HEADER;
    rx_index = 0;
    visual_rx_complete_flag = 0;
    
    // 启动单字节中断接收
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      串口接收回调函数(状态机处理)
//  @return     void
//  @param      void
//  @note       在HAL_UART_RxCpltCallback中调用,单字节处理逻辑
//-------------------------------------------------------------------------------------------------------------------
void Visual_UART_RxCallback(void)
{
    switch (rx_state)
    {
        case RX_STATE_WAIT_HEADER:
            if (rx_byte == 0x66)  // 检测到帧头
            {
                rx_buffer[0] = 0x66;
                rx_index = 1;
                rx_state = RX_STATE_GET_UNIT;
            }
            break;
            
        case RX_STATE_GET_UNIT:
            rx_buffer[1] = rx_byte;  // 保存unit(任务码)
            
            // 根据unit确定后续数据长度
            switch (rx_byte)
            {
                case TASK_IDENTIFY_MATERIAL:  // 0x02: 物料识别
                    rx_expected_len = 5;      // color(1) + x(2) + y(2)
                    break;
                    
                case TASK_QR_CODE:            // 0x09: 二维码
                    rx_expected_len = 6;      // 6个ASCII字符
                    break;
                    
                case TASK_PLATFORM:           // 0x0A: 凸台
                case TASK_SLOT:               // 0x0B: 凹槽
                    rx_expected_len = 4;      // x(2) + y(2)
                    break;
                    
                default:  // 未知任务码,重置状态机
                    rx_state = RX_STATE_WAIT_HEADER;
                    rx_index = 0;
                    break;
            }
            
            if (rx_expected_len > 0)
            {
                rx_index = 2;
                rx_state = RX_STATE_GET_DATA;
            }
            break;
            
        case RX_STATE_GET_DATA:
            rx_buffer[rx_index++] = rx_byte;
            
            // 数据接收完毕,等待帧尾
            if (rx_index >= (2 + rx_expected_len))
            {
                rx_state = RX_STATE_VERIFY_TAIL;
            }
            break;
            
        case RX_STATE_VERIFY_TAIL:
            if (rx_byte == 0x77)  // 帧尾正确
            {
                rx_buffer[rx_index] = 0x77;
                visual_rx_complete_flag = 1;  // 置位接收完成标志
            }
            
            // 重置状态机,准备接收下一帧
            rx_state = RX_STATE_WAIT_HEADER;
            rx_index = 0;
            rx_expected_len = 0;
            break;
    }
    
    // 继续接收下一个字节
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取接收缓冲区指针
//  @return     uint8_t*: 接收缓冲区指针
//  @param      void
//  @note       供main.c调用,传递给Visual_Data_Unpack()解析
//-------------------------------------------------------------------------------------------------------------------
uint8_t* Visual_Get_RxBuffer(void)
{
    return rx_buffer;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      清除接收完成标志
//  @return     void
//  @param      void
//  @note       main.c处理完数据后调用
//-------------------------------------------------------------------------------------------------------------------
void Visual_Clear_RxFlag(void)
{
    visual_rx_complete_flag = 0;
}
