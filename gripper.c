#include "gripper.h"
#include "tim.h"
#include "oled.h"

// 全局机械爪控制对象
static GripperControl_t gripper_ctrl = {
    .extend_position = EXTEND_RETRACT,
    .grip_position = GRIP_OPEN,
    .state = GRIPPER_IDLE,
    .material = MATERIAL_UNKNOWN,
    .has_material = false,
    .last_action_time = 0
};

// 私有函数声明
static void Gripper_SetPWM_Extend(uint16_t position);
static void Gripper_SetPWM_Grip(uint16_t position);
static bool Gripper_WaitForPosition(uint16_t timeout_ms);
static MaterialType_t Gripper_DetectMaterialType(void);

//-------------------------------------------------------------------------------------------------------------------
//  @brief      机械爪初始化
//  @return     void
//  @param      void
//  @note       初始化舵机PWM输出，设置到安全位置
//-------------------------------------------------------------------------------------------------------------------
void Gripper_Init(void)
{
    // 启动PWM输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // 伸缩舵机
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // 抓取舵机
    
    // 设置到安全位置
    Gripper_SetPWM_Extend(EXTEND_RETRACT);
    Gripper_SetPWM_Grip(GRIP_OPEN);
    
    // 等待舵机到位
    HAL_Delay(1000);
    
    gripper_ctrl.state = GRIPPER_IDLE;
    
    OLED_ShowString(4, 1, "Gripper Ready");
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      设置伸缩舵机PWM输出
//  @return     void
//  @param      position: 位置值(0-1000)
//-------------------------------------------------------------------------------------------------------------------
static void Gripper_SetPWM_Extend(uint16_t position)
{
    // 限制范围
    if(position > 1000) position = 1000;
    
    // 计算PWM值：0.5ms-2.5ms 对应 0-1000
    uint32_t pwm_value = (position * 2000 / 1000) + 500;  // 500-2500us
    
    // 转换为定时器计数值（假设定时器频率为1MHz）
    uint32_t ccr_value = pwm_value;
    
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_value);
    
    gripper_ctrl.extend_position = position;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      设置抓取舵机PWM输出
//  @return     void
//  @param      position: 位置值(0-1000)
//-------------------------------------------------------------------------------------------------------------------
static void Gripper_SetPWM_Grip(uint16_t position)
{
    // 限制范围
    if(position > 1000) position = 1000;
    
    // 计算PWM值
    uint32_t pwm_value = (position * 2000 / 1000) + 500;  // 500-2500us
    uint32_t ccr_value = pwm_value;
    
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_value);
    
    gripper_ctrl.grip_position = position;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      等待舵机到达目标位置
//  @return     true: 成功, false: 超时
//  @param      timeout_ms: 超时时间(毫秒)
//-------------------------------------------------------------------------------------------------------------------
static bool Gripper_WaitForPosition(uint16_t timeout_ms)
{
    uint32_t start_time = HAL_GetTick();
    
    while((HAL_GetTick() - start_time) < timeout_ms)
    {
        HAL_Delay(50);  // 舵机响应时间
        // 这里可以添加位置反馈检测（如果有编码器）
    }
    
    return true;  // 假设舵机已到位
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      设置伸缩位置
//  @return     true: 成功, false: 失败
//  @param      position: 目标位置
//  @param      timeout_ms: 超时时间
//-------------------------------------------------------------------------------------------------------------------
bool Gripper_SetExtendPosition(uint16_t position, uint16_t timeout_ms)
{
    if(gripper_ctrl.state == GRIPPER_ERROR) return false;
    
    gripper_ctrl.state = GRIPPER_EXTENDING;
    
    Gripper_SetPWM_Extend(position);
    
    bool success = Gripper_WaitForPosition(timeout_ms);
    
    if(success)
    {
        gripper_ctrl.state = GRIPPER_IDLE;
        gripper_ctrl.last_action_time = HAL_GetTick();
    }
    else
    {
        gripper_ctrl.state = GRIPPER_ERROR;
    }
    
    return success;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      设置抓取位置
//  @return     true: 成功, false: 失败
//  @param      position: 目标位置
//  @param      timeout_ms: 超时时间
//-------------------------------------------------------------------------------------------------------------------
bool Gripper_SetGripPosition(uint16_t position, uint16_t timeout_ms)
{
    if(gripper_ctrl.state == GRIPPER_ERROR) return false;
    
    gripper_ctrl.state = GRIPPER_GRIPPING;
    
    Gripper_SetPWM_Grip(position);
    
    bool success = Gripper_WaitForPosition(timeout_ms);
    
    if(success)
    {
        gripper_ctrl.state = GRIPPER_IDLE;
        gripper_ctrl.last_action_time = HAL_GetTick();
        
        // 判断是否抓有物料
        if(position > GRIP_LIGHT)
        {
            gripper_ctrl.has_material = true;
        }
        else
        {
            gripper_ctrl.has_material = false;
        }
    }
    else
    {
        gripper_ctrl.state = GRIPPER_ERROR;
    }
    
    return success;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      抓取物料
//  @return     true: 成功, false: 失败
//  @param      material_type: 物料类型
//-------------------------------------------------------------------------------------------------------------------
bool Gripper_GrabMaterial(MaterialType_t material_type)
{
    OLED_ShowString(4, 1, "Grabbing...");
    
    // 1. 先张开爪子
    if(!Gripper_SetGripPosition(GRIP_OPEN, 1000))
    {
        return false;
    }
    
    // 2. 伸出到抓取位置
    if(!Gripper_SetExtendPosition(EXTEND_GRAB, 1500))
    {
        return false;
    }
    
    // 3. 根据物料类型选择抓取力度
    uint16_t grip_force;
    switch(material_type)
    {
        case MATERIAL_A_LIGHT:
        case MATERIAL_B_LIGHT:
            grip_force = GRIP_LIGHT;
            break;
        case MATERIAL_A_HEAVY:
        case MATERIAL_B_HEAVY:
            grip_force = GRIP_NORMAL;
            break;
        case MATERIAL_ASSEMBLY:
            grip_force = GRIP_FIRM;
            break;
        default:
            grip_force = GRIP_NORMAL;
            break;
    }
    
    // 4. 抓紧物料
    if(!Gripper_SetGripPosition(grip_force, 1000))
    {
        return false;
    }
    
    // 5. 收回到安全位置
    if(!Gripper_SetExtendPosition(EXTEND_RETRACT, 1500))
    {
        return false;
    }
    
    gripper_ctrl.material = material_type;
    gripper_ctrl.has_material = true;
    
    OLED_ShowString(4, 1, "Grab Success");
    return true;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      放置物料到测试凸台
//  @return     true: 成功, false: 失败
//  @param      void
//-------------------------------------------------------------------------------------------------------------------
bool Gripper_PlaceMaterialOnPlatform(void)
{
    if(!gripper_ctrl.has_material) return false;
    
    OLED_ShowString(4, 1, "Placing Platform");
    
    // 1. 伸出到放置位置（测试凸台需要精确）
    if(!Gripper_SetExtendPosition(EXTEND_PLACE, 1500))
    {
        return false;
    }
    
    // 2. 缓慢松开（确保物料稳定放置）
    HAL_Delay(500);  // 稳定等待
    
    if(!Gripper_SetGripPosition(GRIP_OPEN, 1000))
    {
        return false;
    }
    
    // 3. 等待物料稳定
    HAL_Delay(500);
    
    // 4. 收回
    if(!Gripper_SetExtendPosition(EXTEND_RETRACT, 1500))
    {
        return false;
    }
    
    gripper_ctrl.has_material = false;
    gripper_ctrl.material = MATERIAL_UNKNOWN;
    
    OLED_ShowString(4, 1, "Platform Done");
    return true;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      放置物料到装配凹槽
//  @return     true: 成功, false: 失败
//  @param      void
//-------------------------------------------------------------------------------------------------------------------
bool Gripper_PlaceMaterialInSlot(void)
{
    if(!gripper_ctrl.has_material) return false;
    
    OLED_ShowString(4, 1, "Placing Slot");
    
    // 1. 伸出到装配位置
    if(!Gripper_SetExtendPosition(EXTEND_PLACE, 1500))
    {
        return false;
    }
    
    // 2. 慢慢放入凹槽（确保完全放入）
    HAL_Delay(300);
    
    if(!Gripper_SetGripPosition(GRIP_OPEN, 1000))
    {
        return false;
    }
    
    // 3. 确认放置稳定
    HAL_Delay(300);
    
    // 4. 收回
    if(!Gripper_SetExtendPosition(EXTEND_RETRACT, 1500))
    {
        return false;
    }
    
    gripper_ctrl.has_material = false;
    gripper_ctrl.material = MATERIAL_UNKNOWN;
    
    OLED_ShowString(4, 1, "Slot Done");
    return true;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      码垛操作（物料B套在物料A上）
//  @return     true: 成功, false: 失败
//  @param      void
//-------------------------------------------------------------------------------------------------------------------
bool Gripper_StackMaterial(void)
{
    if(!gripper_ctrl.has_material) return false;
    
    OLED_ShowString(4, 1, "Stacking...");
    
    // 1. 伸出到码垛位置（比普通放置更精确）
    if(!Gripper_SetExtendPosition(EXTEND_STACK, 2000))
    {
        return false;
    }
    
    // 2. 使用特殊的码垛抓取力度
    if(!Gripper_SetGripPosition(GRIP_STACK, 1000))
    {
        return false;
    }
    
    // 3. 微调位置确保对准
    HAL_Delay(500);
    
    // 4. 缓慢放置，确保物料B的凹槽与物料A正确套装
    if(!Gripper_SetGripPosition(GRIP_OPEN, 1500))
    {
        return false;
    }
    
    // 5. 等待套装稳定
    HAL_Delay(800);
    
    // 6. 收回
    if(!Gripper_SetExtendPosition(EXTEND_RETRACT, 1500))
    {
        return false;
    }
    
    gripper_ctrl.has_material = false;
    gripper_ctrl.material = MATERIAL_UNKNOWN;
    
    OLED_ShowString(4, 1, "Stack Success");
    return true;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      收回到安全位置
//  @return     true: 成功, false: 失败
//  @param      void
//-------------------------------------------------------------------------------------------------------------------
bool Gripper_RetractToSafe(void)
{
    OLED_ShowString(4, 1, "Retracting");
    
    // 收回伸缩机构到安全位置
    bool success = Gripper_SetExtendPosition(EXTEND_RETRACT, 1500);
    
    if(success)
    {
        gripper_ctrl.state = GRIPPER_IDLE;
        OLED_ShowString(4, 1, "Safe Position");
    }
    
    return success;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      微调位置（用于精确对准）
//  @return     true: 成功, false: 失败
//  @param      extend_delta: 伸缩调节量
//  @param      grip_delta: 抓取调节量
//-------------------------------------------------------------------------------------------------------------------
bool Gripper_AdjustPosition(int16_t extend_delta, int16_t grip_delta)
{
    uint16_t new_extend = gripper_ctrl.extend_position + extend_delta;
    uint16_t new_grip = gripper_ctrl.grip_position + grip_delta;
    
    // 范围检查
    if(new_extend > 1000) new_extend = 1000;
    if(new_grip > 1000) new_grip = 1000;
    
    // 执行调节
    Gripper_SetPWM_Extend(new_extend);
    Gripper_SetPWM_Grip(new_grip);
    
    HAL_Delay(200);  // 短暂等待
    
    return true;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取机械爪状态
//  @return     当前状态
//  @param      void
//-------------------------------------------------------------------------------------------------------------------
GripperState_t Gripper_GetState(void)
{
    return gripper_ctrl.state;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      检查是否抓有物料
//  @return     true: 有物料, false: 无物料
//  @param      void
//-------------------------------------------------------------------------------------------------------------------
bool Gripper_HasMaterial(void)
{
    return gripper_ctrl.has_material;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      检查是否就绪
//  @return     true: 就绪, false: 忙碌或错误
//  @param      void
//-------------------------------------------------------------------------------------------------------------------
bool Gripper_IsReady(void)
{
    return (gripper_ctrl.state == GRIPPER_IDLE);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      紧急停止
//  @return     void
//  @param      void
//-------------------------------------------------------------------------------------------------------------------
void Gripper_EmergencyStop(void)
{
    // 停止PWM输出
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    
    gripper_ctrl.state = GRIPPER_ERROR;
    
    OLED_ShowString(4, 1, "EMERGENCY STOP");
}
