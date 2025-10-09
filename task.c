/**
  ******************************************************************************
  * @file    task.c
  * @brief   比赛任务流程模块实现
  * @details 实现智能物流搬运比赛的完整任务流程
  ******************************************************************************
  */

#include "task.h"
#include "test.h"
#include "motor.h"
#include "gripper.h"
#include "visual_comm.h"
#include "visual_test.h"
#include "inv_mpu.h"
#include "oled.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* ==================== 私有变量 ==================== */
static float g_start_yaw = 0.0f;  /**< 起始航向角 */

/* ==================== 私有函数声明 ==================== */
static uint8_t read_initial_yaw(void);

/* ==================== 公开函数实现 ==================== */

/**
 * @brief 物料台抓取流程
 */
TaskStatus_t Task_Material_Pickup(void)
{
    // 重置抓取计数器(新流程开始)
    Gripper_Reset_PickupCounter();
    
    // 1. 读取起始航向
    if (read_initial_yaw() != 0) {
        return TASK_FAILED;
    }
    
    // 2. 驶出启停区并定位
    Motor_Move_Lateral(400.0f, 0);           // 驶出启停区
    Motor_Move_Forward(-90.0f, 0);           // 稍微后退
    
    // 3. 移动到第一个物块(蓝色)前
    Motor_Move_Lateral_WithYawHold(930.0f, g_start_yaw, 1.5f, 0, 400.0f);
    
    // 4. 抓取蓝色物块
    Gripper_Pan_Rotate(0.0f);  // 旋转回抓取位置(0°)
    HAL_Delay(2500);  // 等待舵机到位
    if (Vision_Align_Target(TARGET_MATERIAL_BLOCK, COLOR_BLUE) != 0) {
        return TASK_VISION_ERROR;
    }
    HAL_Delay(500);
    Gripper_PickAndPlace(3);  // 放入第3个物料盘
    
    // 抓取后航向矫正,防止累积偏航
    Motor_Correct_Yaw(g_start_yaw, 2.0f, 0);
    
    // 5. 移动到绿色物块
    Motor_Set_Speed_Profile(0);// SPEED_PRECISE = 30 RPM (精确对位)
    Motor_Move_Lateral(160.0f, 0);
    if (Vision_Align_Target(TARGET_MATERIAL_BLOCK, COLOR_GREEN) != 0) {
        return TASK_VISION_ERROR;
    }
    HAL_Delay(500);
    Gripper_PickAndPlace(2);  // 放入第2个物料盘
    
    // 6. 移动到红色物块
    Motor_Move_Lateral(160.0f, 0);
    if (Vision_Align_Target(TARGET_MATERIAL_BLOCK, COLOR_RED) != 0) {
        return TASK_VISION_ERROR;
    }
    HAL_Delay(1000);
    Gripper_PickAndPlace(1);  // 放入第1个物料盘
    
    // 最后一次航向矫正
    Motor_Correct_Yaw(g_start_yaw, 2.0f, 0);
    
    Motor_Set_Speed_Profile(1); // SPEED_FAST = 50 RPM (载货移动)
    Motor_Move_Lateral_WithYawHold(288.0f, g_start_yaw, 2.0f, 0, 200.0f);
    return TASK_SUCCESS;
}

/**
 * @brief 测试区放置(按红→绿→蓝顺序放置到测试凸台)
 * @details 从物料盘按蓝→绿→红抓取后,逆序放置到测试台
 *          放置顺序: 红(最后抓)→绿→蓝(最先抓)
 */
TaskStatus_t Task_Test_Zone_Place_All(void)
{
    // 已完成旋转90°,现在面向测试区三个凸台
    // 测试台布局(从左到右): 红色 - 绿色 - 蓝色
    if (read_initial_yaw() != 0) {
        return TASK_FAILED;
    }
    
    // 1. 先平移到红色测试凸台位置,放置红色物块(物料盘1号位)
    Motor_Move_Lateral_WithYawHold(1250.0f,g_start_yaw,1.5f,0, 400.0f); 
    Motor_Move_Forward(80,0);
    if (Vision_Align_Target(TARGET_TEST_PLATFORM, COLOR_RED) != 0) {
        return TASK_VISION_ERROR;
    }
    Gripper_Pan_Rotate(PAN_ANGLE_PLATE1);  // 转到1号物料盘位置
    HAL_Delay(1000);
    
    Gripper_Lift(HEIGHT_ASSEMBLY_L1);
    Gripper_Claw_Open();
    HAL_Delay(1000);
    
    Gripper_Claw_Close();
    HAL_Delay(1000);
    Gripper_Lift(HEIGHT_TRANSPORT);
    
    Gripper_Pan_Rotate(0);  // 云台转回前方
    HAL_Delay(2000);
    
    Gripper_Lift(HEIGHT_TEST_PLATFORM);  // 降到测试台高度
    Gripper_Claw_Open();  // 松开测试
    HAL_Delay(1000);  // 等待稳定判断
    
    Gripper_Lift(HEIGHT_TRANSPORT);
    
    // 2. 平移到绿色测试凸台,放置绿色物块(物料盘2号位)
    Motor_Move_Lateral(-150.0f, 0);  // 向右平移150mm到绿色凸台
    if (Vision_Align_Target(TARGET_TEST_PLATFORM, COLOR_GREEN) != 0) {
        return TASK_VISION_ERROR;
    }
    Motor_Move_Forward(-80.0f, 0);
    Gripper_Pan_Rotate(170);
    HAL_Delay(1000);
    Gripper_Pan_Rotate(PAN_ANGLE_PLATE2);
    HAL_Delay(1000);
    
    Gripper_Lift(HEIGHT_ASSEMBLY_L1);
    
    Gripper_Claw_Close();
    HAL_Delay(500);
    Gripper_Lift(HEIGHT_TRANSPORT);
    
    Gripper_Pan_Rotate(0);
    HAL_Delay(1000);
    Motor_Move_Forward(80.0f, 0);
    Gripper_Lift(HEIGHT_TEST_PLATFORM);
    Gripper_Claw_Open();
    HAL_Delay(1000);
    Gripper_Lift(HEIGHT_TRANSPORT);
    
    // 3. 平移到蓝色测试凸台,放置蓝色物块(物料盘3号位)
    Motor_Move_Lateral(-160.0f, 0);  // 再向右平移160mm到蓝色凸台
    if (Vision_Align_Target(TARGET_TEST_PLATFORM, COLOR_BLUE) != 0) {
        return TASK_VISION_ERROR;
    }
    Motor_Move_Forward(-80.0f, 0);
    Gripper_Pan_Rotate(180);
    HAL_Delay(1000);
    Gripper_Pan_Rotate(PAN_ANGLE_PLATE3);
    HAL_Delay(1000);
    
    Gripper_Lift(3.0f);
    
    Gripper_Claw_Close();
    HAL_Delay(500);
    Gripper_Lift(HEIGHT_TRANSPORT);
    
    Gripper_Pan_Rotate(0);
    HAL_Delay(1000);
    Motor_Move_Forward(80.0f, 0);
    Gripper_Lift(HEIGHT_TEST_PLATFORM);
    Gripper_Claw_Open();
    HAL_Delay(1000);

    Gripper_Lift(HEIGHT_TRANSPORT);
    
    return TASK_SUCCESS;
}

/**
 * @brief 将测试通过的物料按蓝→绿→红顺序放回物料盘
 */
TaskStatus_t Task_Return_To_Plate(void)
{
    // 当前位置在蓝色测试凸台,需要逆序取回并放到物料盘
    
    // 1. 从蓝色凸台取物料,放回物料盘3号位
    // 已经在蓝色位置,直接抓取凸台上的物料
    Gripper_Lift(HEIGHT_TEST_PLATFORM);
    Gripper_Claw_Close();
    HAL_Delay(1000);
    Gripper_Lift(HEIGHT_TRANSPORT);
    Motor_Move_Forward(-80.0f, 0); // 后退80mm，避免碰撞
    // 放回物料盘3号位
    Gripper_Pan_Rotate(180.0f);
    HAL_Delay(1000);
    Gripper_Pan_Rotate(PAN_ANGLE_PLATE3);
    HAL_Delay(1000);
    Gripper_Lift(HEIGHT_TEST_PLATFORM);
    Gripper_Claw_Open();
    HAL_Delay(1000);
    Gripper_Lift(HEIGHT_TRANSPORT);
    Gripper_Pan_Rotate(0);
    Motor_Move_Forward(80.0f, 0);
    
    // 2. 平移到绿色凸台,取物料放回2号位
    Motor_Move_Lateral(160.0f, 0);  // 向左平移160mm
    if (Vision_Align_Target(TARGET_TEST_PLATFORM, COLOR_GREEN) != 0) {
        return TASK_VISION_ERROR;
    }
    Gripper_Lift(HEIGHT_TEST_PLATFORM);
    Gripper_Claw_Close();
    HAL_Delay(1000);
    Gripper_Lift(HEIGHT_TRANSPORT);
    Motor_Move_Forward(-80.0f, 0); // 后退80mm，避免碰撞
    Gripper_Pan_Rotate(180.0f);
    HAL_Delay(1000);
    Gripper_Pan_Rotate(PAN_ANGLE_PLATE2);
    HAL_Delay(1000);

    Gripper_Lift(HEIGHT_TEST_PLATFORM);
    Gripper_Claw_Open();
    HAL_Delay(1000);
    Gripper_Lift(HEIGHT_TRANSPORT);
    Gripper_Pan_Rotate(0);
    Motor_Move_Forward(80.0f, 0);
    // 3. 平移到红色凸台,取物料放回1号位
    Motor_Move_Lateral(160.0f, 0);  // 再向左平移160mm
    if (Vision_Align_Target(TARGET_TEST_PLATFORM, COLOR_RED) != 0) {
        return TASK_VISION_ERROR;
    }
    Gripper_Lift(HEIGHT_ASSEMBLY_L1);
    Gripper_Claw_Close();
    HAL_Delay(1000);
    Gripper_Lift(HEIGHT_TRANSPORT);
    Motor_Move_Forward(-80.0f, 0); // 后退80mm，避免碰撞
    Gripper_Pan_Rotate(PAN_ANGLE_PLATE1);
    HAL_Delay(1000);
    Gripper_Lift(HEIGHT_ASSEMBLY_L1);
    Gripper_Claw_Open();
    HAL_Delay(1000);
    Gripper_Lift(HEIGHT_HOME);
    Gripper_Pan_Rotate(0);
    Motor_Move_Forward(80.0f, 0);
    return TASK_SUCCESS;
}

/**
 * @brief 装配码垛流程(第一层放置到凹槽)
 * @details 从物料盘按红→绿→蓝顺序放置到装配区凹槽
 *          流程: 移动到码垛区 → 旋转90° → 视觉定位 → 放置物块
 * @param assembly_mode 装配模式 (0=同色, 1=异色) - 预留给第二层使用
 */
TaskStatus_t Task_Assembly_Stacking(uint8_t assembly_mode)
{
    // 从测试区返回物料盘后,车辆位于启停区附近
    // 需要移动到装配码垛区
    
    if (read_initial_yaw() != 0) {
        return TASK_FAILED;
    }
    
    // 1. 从当前位置移动到装配码垛区前方
    // TODO: 根据实际场地尺寸调整距离
    Motor_Move_Lateral_WithYawHold(288.0f, g_start_yaw, 2.0f, 0, 200.0f);
    
    // 2. 逆时针旋转90°,面向装配区三个凹槽
    Motor_Rotate_90_DMP(0, 0);
    
    // 3. 读取旋转后的新航向角
    if (read_initial_yaw() != 0) {
        return TASK_FAILED;
    }
    
    // 装配区布局(从左到右): 红色凹槽 - 绿色凹槽 - 蓝色凹槽
    // 放置顺序: 红→绿→蓝 (从物料盘1/2/3号位取料)
    
    // 4. 平移到红色凹槽位置,放置红色物块(物料盘1号位)
    Motor_Move_Lateral_WithYawHold(1250.0f,g_start_yaw,1.5f,0, 400.0f);
    Motor_Move_Forward(80.0f, 0);  // 前进靠近凹槽
    
    if (Vision_Align_Target(TARGET_TEST_PLATFORM, COLOR_RED) != 0) {
        return TASK_VISION_ERROR;
    }
    
    // 云台转向物料盘1号位
    Gripper_Pan_Rotate(180.0f);  // 先转到后方
    HAL_Delay(1000);
    Gripper_Pan_Rotate(PAN_ANGLE_PLATE1);
    HAL_Delay(1000);
    
    // 从物料盘取红色物块
    Gripper_Lift(HEIGHT_ASSEMBLY_L1);
    Gripper_Claw_Close();
    HAL_Delay(1000);
    Gripper_Lift(HEIGHT_TRANSPORT);
    
    // 云台转回前方
    Gripper_Pan_Rotate(0);
    HAL_Delay(1000);
    
    // 放置到红色凹槽
    Gripper_Lift(HEIGHT_ASSEMBLY_L1);
    Gripper_Claw_Open();
    HAL_Delay(1000);
    Gripper_Lift(HEIGHT_TRANSPORT);
    
    // 5. 平移到绿色凹槽,放置绿色物块(物料盘2号位)
    Motor_Move_Lateral(-155.0f, 0);  // 向右平移155mm
    
    if (Vision_Align_Target(TARGET_TEST_PLATFORM, COLOR_GREEN) != 0) {
        return TASK_VISION_ERROR;
    }
    Motor_Move_Forward(-80.0f, 0);
    Gripper_Pan_Rotate(180.0f);
    HAL_Delay(1000);
    Gripper_Pan_Rotate(PAN_ANGLE_PLATE2);
    HAL_Delay(1000);
    
    Gripper_Lift(HEIGHT_ASSEMBLY_L1);
    Gripper_Claw_Close();
    HAL_Delay(500);
    Gripper_Lift(HEIGHT_TRANSPORT);
    
    Gripper_Pan_Rotate(0);
    HAL_Delay(1000);
    Motor_Move_Forward(80.0f, 0);
    Gripper_Lift(HEIGHT_ASSEMBLY_L1);
    Gripper_Claw_Open();
    HAL_Delay(500);
    Gripper_Lift(HEIGHT_TRANSPORT);
    
    // 6. 平移到蓝色凹槽,放置蓝色物块(物料盘3号位)
    Motor_Move_Lateral(-160.0f, 0);  // 再向右平移160mm
    
    if (Vision_Align_Target(TARGET_TEST_PLATFORM, COLOR_BLUE) != 0) {
        return TASK_VISION_ERROR;
    }
    Motor_Move_Forward(-80.0f, 0);
    Gripper_Pan_Rotate(180.0f);
    HAL_Delay(1000);
    Gripper_Pan_Rotate(PAN_ANGLE_PLATE3);
    HAL_Delay(1000);
    
    Gripper_Lift(HEIGHT_ASSEMBLY_L1);
    Gripper_Claw_Close();
    HAL_Delay(500);
    Gripper_Lift(HEIGHT_TRANSPORT);
    
    Gripper_Pan_Rotate(0);
    HAL_Delay(1000);
    Motor_Move_Forward(80.0f, 0);
    Gripper_Lift(HEIGHT_ASSEMBLY_L1);
    Gripper_Claw_Open();
    HAL_Delay(500);
    Gripper_Lift(HEIGHT_TRANSPORT);
    
    // TODO: 第二层码垛 - 根据assembly_mode进行同色/异色装配
    // if (assembly_mode == 0) {
    //     // 同色装配: 红上红, 绿上绿, 蓝上蓝
    // } else {
    //     // 异色装配: 根据任务码要求错配
    // }
    
    return TASK_SUCCESS;
}

/**
 * @brief 障碍穿越与回归流程 (预留接口)
 */
TaskStatus_t Task_Obstacle_Return(void)
{
    // TODO: 实现障碍穿越流程
    // 1. 从装配区抓取一组装配体
    // 2. 穿越障碍区(830x430mm亚克力板阵列孔)
    // 3. 回到启停区
    // 4. 关闭任务码显示
    Motor_Move_Lateral_WithYawHold(600.0f, g_start_yaw, 2.0f, 0, 200.0f);
    Motor_Move_Forward_WithYawHold(-2000.0f, g_start_yaw, 2.0f, 0);
    Motor_Move_Lateral_WithYawHold(300.0f, g_start_yaw, 2.0f, 0, 200.0f);
    return TASK_SUCCESS;
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief 读取起始航向角
 * @return 0=成功, 1=失败
 */
static uint8_t read_initial_yaw(void)
{
    float pitch, roll;
    
    for (uint8_t i = 0; i < 5; i++) {
        if (mpu_dmp_get_data(&pitch, &roll, &g_start_yaw) == 0) {
            return 0;  // 成功
        }
        HAL_Delay(10);
    }
    
    return 1;  // 失败
}

/**
 * @brief 二维码识别并显示任务码
 */
TaskStatus_t Task_QRCode_Read(uint8_t *assembly_mode)
{
    // 1. 发送二维码扫描指令到视觉模块
    uint8_t cmd[4] = {0x66, 0x09, 0x00, 0x77};  // unit=9: QR码识别
    HAL_UART_Transmit(&huart2, cmd, 4, 100);
    
    OLED_ShowString(1, 1, "QR Scanning...");
    
    // 2. 等待视觉模块返回二维码数据
    // 预期格式: [0x66, 0x09, mode, 0x77] 其中mode: 1=同色, 2=异色
    uint8_t timeout_count = 0;
    uint8_t rx_buffer[10];
    
    while (timeout_count < 50) {  // 5秒超时 (50 × 100ms)
        // 检查串口接收缓冲区
        if (Visual_GetLastResponse(rx_buffer, sizeof(rx_buffer)) == 0) {
            // 验证数据包
            if (rx_buffer[0] == 0x66 && rx_buffer[1] == 0x09) {
                uint8_t mode_code = rx_buffer[2];
                
                // 解析装配模式
                if (mode_code == 0x01) {
                    *assembly_mode = 0;  // 同色装配
                    Task_Display_Mode(1);
                    return TASK_SUCCESS;
                } 
                else if (mode_code == 0x02) {
                    *assembly_mode = 1;  // 异色错配
                    Task_Display_Mode(2);
                    return TASK_SUCCESS;
                }
            }
        }
        
        HAL_Delay(100);
        timeout_count++;
    }
    
    // 超时失败
    OLED_ShowString(1, 1, "QR Timeout!  ");
    return TASK_TIMEOUT;
}

/**
 * @brief 任务码显示控制
 * @note 使用24×24中文字库显示"同色"或"异色"
 *       字体高度8mm,满足比赛要求
 */
void Task_Display_Mode(uint8_t mode)
{
    OLED_Clear();
    uint8_t start_x = (OLED_WIDTH >= 48) ? (OLED_WIDTH - 48) / 2 : 0;
    uint8_t start_page = ((OLED_HEIGHT / 8) > 3) ? ((OLED_HEIGHT / 8) - 3) / 2 : 0;
    
    if (mode == 1) {
        // 同色装配 - 在屏幕中央显示"同色"两个大字
        // X坐标: 根据屏幕宽度动态居中, Y坐标: 居中显示
        OLED_ShowChinese24x24(start_x, start_page, OLED_GB2312_TONG);
        OLED_ShowChinese24x24(start_x + 24, start_page, OLED_GB2312_SE);
    } 
    else if (mode == 2) {
        // 异色错配 - 在屏幕中央显示"异色"两个大字
        OLED_ShowChinese24x24(start_x, start_page, OLED_GB2312_YI);
        OLED_ShowChinese24x24(start_x + 24, start_page, OLED_GB2312_SE);
    }
}
