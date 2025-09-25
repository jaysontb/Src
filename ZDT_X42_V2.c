#include "ZDT_X42_V2.h"
#include <math.h>
#include "usart.h" // 包含 usart.h 以引用全局接收变量

// 假设在 main.c 中已经定义
extern UART_HandleTypeDef huart1;
// 移除对 rxBuffer 和 rxFrameCount 的 extern 声明，因为它们现在在 usart.h 中声明并由 main.c 定义
// extern uint8_t rxBuffer[256];
// extern volatile uint8_t rxFrameFlag; // rxFrameFlag 已经在 usart.h 中声明
// extern uint16_t rxFrameCount; // rxFrameCount 已经在 usart.h 中声明

// 私有函数：DMA发送命令
static void usart_SendCmd(uint8_t *cmd, uint8_t len)
{
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)cmd, len);
}

/**
 * @brief? ? 将当前位置清零
 * @param? ? addr? ：电机地址
 */
void ZDT_X42_V2_Reset_CurPos_To_Zero(uint8_t addr)
{
    uint8_t cmd[16] = {0};
    
    cmd[0] = addr;      // 地址
    cmd[1] = 0x0A;      // 功能码
    cmd[2] = 0x6D;      // 辅助码
    cmd[3] = 0x6B;      // 校验字节
    
    usart_SendCmd(cmd, 4);
}

/**
 * @brief? ? 解除堵转保护
 * @param? ? addr? ：电机地址
 */
void ZDT_X42_V2_Reset_Clog_Pro(uint8_t addr)
{
    uint8_t cmd[16] = {0};
    
    cmd[0] = addr;      // 地址
    cmd[1] = 0x0E;      // 功能码
    cmd[2] = 0x52;      // 辅助码
    cmd[3] = 0x6B;      // 校验字节
    
    usart_SendCmd(cmd, 4);
}

/**
 * @brief? ? 读取系统参数
 * @param? ? addr? ：电机地址
 * @param? ? s? ? ?：系统参数类型
 */
void ZDT_X42_V2_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
    uint8_t cmd[16] = {0};
    
    cmd[0] = addr; // 地址

    switch(s)
    {
        case S_VER  : cmd[1] = 0x1F; break;
        case S_RL   : cmd[1] = 0x20; break;
        case S_PID  : cmd[1] = 0x21; break;
        case S_ORG  : cmd[1] = 0x22; break;
        case S_VBUS : cmd[1] = 0x24; break;
        case S_CBUS : cmd[1] = 0x26; break;
        case S_CPHA : cmd[1] = 0x27; break;
        case S_ENC  : cmd[1] = 0x29; break;
        case S_CPUL : cmd[1] = 0x30; break;
        case S_ENCL : cmd[1] = 0x31; break;
        case S_TPUL : cmd[1] = 0x32; break;
        case S_TPOS : cmd[1] = 0x33; break;
        case S_OPOS : cmd[1] = 0x34; break;
        case S_VEL  : cmd[1] = 0x35; break;
        case S_CPOS : cmd[1] = 0x36; break;
        case S_PERR : cmd[1] = 0x37; break;
        case S_TEMP : cmd[1] = 0x39; break;
        case S_SFLAG: cmd[1] = 0x3A; break;
        case S_OFLAG: cmd[1] = 0x3B; break;
        case S_Conf : cmd[1] = 0x42; cmd[2] = 0x6C; break;
        case S_State: cmd[1] = 0x43; cmd[2] = 0x7A; break;
        default: return;
    }

    if(s >= S_Conf)
    {
        cmd[3] = 0x6B; usart_SendCmd(cmd, 4);
    }
    else
    {
        cmd[2] = 0x6B; usart_SendCmd(cmd, 3);
    }
}

/**
 * @brief? ? 修改开环/闭环控制模式
 */
void ZDT_X42_V2_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
    uint8_t cmd[16] = {0};
    
    cmd[0] = addr;          // 地址
    cmd[1] = 0x46;          // 功能码
    cmd[2] = 0x69;          // 辅助码
    cmd[3] = svF;           // 是否存储
    cmd[4] = ctrl_mode;     // 控制模式
    cmd[5] = 0x6B;          // 校验字节
    
    usart_SendCmd(cmd, 6);
}

/**
 * @brief? ? 使能信号控制
 */
void ZDT_X42_V2_En_Control(uint8_t addr, bool state, uint8_t snF)
{
    uint8_t cmd[16] = {0};
    
    cmd[0] = addr;      // 地址
    cmd[1] = 0xF3;      // 功能码
    cmd[2] = 0xAB;      // 辅助码
    cmd[3] = (uint8_t)state; // 使能状态
    cmd[4] = snF;       // 多机同步运动标志
    cmd[5] = 0x6B;      // 校验字节
    
    usart_SendCmd(cmd, 6);
}

/**
 * @brief? ? 力矩模式
 */
void ZDT_X42_V2_Torque_Control(uint8_t addr, uint8_t sign, uint16_t t_ramp, uint16_t torque, uint8_t snF)
{
    uint8_t cmd[16] = {0};
    
    cmd[0] = addr;          // 地址
    cmd[1] = 0xF5;          // 功能码
    cmd[2] = sign;          // 符号（方向）
    cmd[3] = (uint8_t)(t_ramp >> 8);
    cmd[4] = (uint8_t)(t_ramp >> 0);
    cmd[5] = (uint8_t)(torque >> 8);
    cmd[6] = (uint8_t)(torque >> 0);
    cmd[7] = snF;           // 多机同步运动标志
    cmd[8] = 0x6B;          // 校验字节
    
    usart_SendCmd(cmd, 9);
}

/**
 * @brief? ? 速度模式
 */
void ZDT_X42_V2_Velocity_Control(uint8_t addr, uint8_t dir, uint16_t v_ramp, float velocity, uint8_t snF)
{
    static uint8_t cmd[16] = {0};
    uint16_t vel = 0;

    vel = (uint16_t)ABS(velocity * 10.0f);

    cmd[0] = addr;          // 地址
    cmd[1] = 0xF6;          // 功能码
    cmd[2] = dir;           // 方向
    cmd[3] = (uint8_t)(v_ramp >> 8);
    cmd[4] = (uint8_t)(v_ramp >> 0);
    cmd[5] = (uint8_t)(vel >> 8);
    cmd[6] = (uint8_t)(vel >> 0);
    cmd[7] = snF;           // 多机同步运动标志
    cmd[8] = 0x6B;          // 校验字节
    
    usart_SendCmd(cmd, 9);
}

/**
 * @brief? ? 直通限速位置模式
 */
void ZDT_X42_V2_Bypass_Position_LV_Control(uint8_t addr, uint8_t dir, float velocity, float position, uint8_t raf, uint8_t snF)
{
    uint8_t cmd[16] = {0};
    uint16_t vel = 0;
    uint32_t pos = 0;

    vel = (uint16_t)ABS(velocity * 10.0f);
    pos = (uint32_t)ABS(position * 10.0f);

    cmd[0]  = addr;         // 地址
    cmd[1]  = 0xFB;         // 功能码
    cmd[2]  = dir;          // 符号（方向）
    cmd[3]  = (uint8_t)(vel >> 8);
    cmd[4]  = (uint8_t)(vel >> 0);
    cmd[5]  = (uint8_t)(pos >> 24);
    cmd[6]  = (uint8_t)(pos >> 16);
    cmd[7]  = (uint8_t)(pos >> 8);
    cmd[8]  = (uint8_t)(pos >> 0);
    cmd[9]  = raf;          // 相位位置/绝对位置标志
    cmd[10] = snF;          // 多机同步运动标志
    cmd[11] = 0x6B;         // 校验字节
    
    usart_SendCmd(cmd, 12);
}

/**
 * @brief? ? 梯形曲线位置模式
 */
void ZDT_X42_V2_Traj_Position_Control(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float velocity, float position, uint8_t raf, uint8_t snF)
{
    uint8_t cmd[32] = {0};
    uint16_t vel = 0;
    uint32_t pos = 0;

    vel = (uint16_t)ABS(velocity * 10.0f);
    pos = (uint32_t)ABS(position * 10.0f);

    cmd[0]  = addr;         // 地址
    cmd[1]  = 0xFD;         // 功能码
    cmd[2]  = dir;          // 符号（方向）
    cmd[3]  = (uint8_t)(acc >> 8);
    cmd[4]  = (uint8_t)(acc >> 0);
    cmd[5]  = (uint8_t)(dec >> 8);
    cmd[6]  = (uint8_t)(dec >> 0);
    cmd[7]  = (uint8_t)(vel >> 8);
    cmd[8]  = (uint8_t)(vel >> 0);
    cmd[9]  = (uint8_t)(pos >> 24);
    cmd[10] = (uint8_t)(pos >> 16);
    cmd[11] = (uint8_t)(pos >> 8);
    cmd[12] = (uint8_t)(pos >> 0);
    cmd[13] = raf;          // 相位位置/绝对位置标志
    cmd[14] = snF;          // 多机同步运动标志
    cmd[15] = 0x6B;         // 校验字节
    
    usart_SendCmd(cmd, 16);
}

/**
 * @brief? ? 立即停止
 */
void ZDT_X42_V2_Stop_Now(uint8_t addr, uint8_t snF)
{
    uint8_t cmd[16] = {0};
    
    cmd[0] = addr;      // 地址
    cmd[1] = 0xFE;      // 功能码
    cmd[2] = 0x98;      // 辅助码
    cmd[3] = snF;       // 多机同步运动标志
    cmd[4] = 0x6B;      // 校验字节
    
    usart_SendCmd(cmd, 5);
}

/**
 * @brief? ? 多机同步运动
 */
void ZDT_X42_V2_Synchronous_motion(uint8_t addr)
{
    uint8_t cmd[16] = {0};
    
    cmd[0] = addr;      // 地址
    cmd[1] = 0xFF;      // 功能码
    cmd[2] = 0x66;      // 辅助码
    cmd[3] = 0x6B;      // 校验字节
    
    usart_SendCmd(cmd, 4);
}

/**
 * @brief? ? 设置单圈回零的零点位置
 */
void ZDT_X42_V2_Origin_Set_O(uint8_t addr, bool svF)
{
    uint8_t cmd[16] = {0};
    
    cmd[0] = addr;      // 地址
    cmd[1] = 0x93;      // 功能码
    cmd[2] = 0x88;      // 辅助码
    cmd[3] = svF;       // 是否存储标志
    cmd[4] = 0x6B;      // 校验字节
    
    usart_SendCmd(cmd, 5);
}

/**
 * @brief? ? 修改回零参数
 */
void ZDT_X42_V2_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
    uint8_t cmd[32] = {0};
    
    cmd[0] = addr;      // 地址
    cmd[1] = 0x4C;      // 功能码
    cmd[2] = 0xAE;      // 辅助码
    cmd[3] = svF;       // 是否存储标志
    cmd[4] = o_mode;    // 回零模式
    cmd[5] = o_dir;     // 回零方向
    cmd[6]  = (uint8_t)(o_vel >> 8);
    cmd[7]  = (uint8_t)(o_vel >> 0);
    cmd[8]  = (uint8_t)(o_tm >> 24);
    cmd[9]  = (uint8_t)(o_tm >> 16);
    cmd[10] = (uint8_t)(o_tm >> 8);
    cmd[11] = (uint8_t)(o_tm >> 0);
    cmd[12] = (uint8_t)(sl_vel >> 8);
    cmd[13] = (uint8_t)(sl_vel >> 0);
    cmd[14] = (uint8_t)(sl_ma >> 8);
    cmd[15] = (uint8_t)(sl_ma >> 0);
    cmd[16] = (uint8_t)(sl_ms >> 8);
    cmd[17] = (uint8_t)(sl_ms >> 0);
    cmd[18] = potF;     // 上电自动触发回零
    cmd[19] = 0x6B;     // 校验字节
    
    usart_SendCmd(cmd, 20);
}

/**
 * @brief? ? 触发回零
 */
void ZDT_X42_V2_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
    uint8_t cmd[16] = {0};
    
    cmd[0] = addr;      // 地址
    cmd[1] = 0x9A;      // 功能码
    cmd[2] = o_mode;    // 回零模式
    cmd[3] = snF;       // 多机同步运动标志
    cmd[4] = 0x6B;      // 校验字节
    
    usart_SendCmd(cmd, 5);
}

/**
 * @brief? ? 强制中断并退出回零
 */
void ZDT_X42_V2_Origin_Interrupt(uint8_t addr)
{
    uint8_t cmd[16] = {0};
    
    cmd[0] = addr;      // 地址
    cmd[1] = 0x9C;      // 功能码
    cmd[2] = 0x48;      // 辅助码
    cmd[3] = 0x6B;      // 校验字节
    
    usart_SendCmd(cmd, 4);
}

/**
 * @brief? ? 接收数据 (HAL库非阻塞实现)
 * @param? ? p_rxCmd? ?: 接收到的数据缓存在该数组
 * @param? ? p_rxCount : 接收到的数据长度
 */
void ZDT_X42_V2_Receive_Data(uint8_t *p_rxCmd, uint8_t *p_rxCount)
{
    // 等待接收完成标志位被设置
    uint32_t startTick = HAL_GetTick();
    while (rxFrameFlag == 0)
    {
        // 设置超时，防止死锁
        if ((HAL_GetTick() - startTick) > 100)
        {
            *p_rxCount = 0;
            return;
        }
    }
    
    // 清除标志位
    rxFrameFlag = 0;
    
    // 获取接收到的数据长度
    *p_rxCount = rxCount; // 使用全局 rxCount

    // 拷贝数据
    for (int i = 0; i < rxCount; i++) // 使用全局 rxCount
    {
        p_rxCmd[i] = rxCmd[i]; // 从全局 rxCmd 拷贝到传入的缓冲区
    }
}
