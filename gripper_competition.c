#include "gripper.h"
#include "visual_comm.h"
#include "motor.h"
#include "oled.h"

//-------------------------------------------------------------------------------------------------------------------
//  @brief      比赛专用：抓取指定颜色物料
//  @return     true: 成功, false: 失败
//  @param      color: COLOR_RED/COLOR_GREEN/COLOR_BLUE
//-------------------------------------------------------------------------------------------------------------------
bool Competition_GrabMaterialByColor(uint8_t color)
{
    OLED_ShowString(1, 1, "Grab by Color");
    
    // 1. 发送视觉识别请求
    Visual_Send_Material_Request(color);
    
    // 2. 等待视觉返回物料位置
    uint32_t timeout = HAL_GetTick() + 3000; // 3秒超时
    while(!VIS_RX.material_valid && HAL_GetTick() < timeout)
    {
        HAL_Delay(50);
    }
    
    if(!VIS_RX.material_valid)
    {
        OLED_ShowString(4, 1, "Vision Timeout");
        return false;
    }
    
    // 3. 导航到物料位置
    float target_x = VIS_RX.material_x;
    float target_y = VIS_RX.material_y;
    
    // 像素坐标转换为实际距离（需要根据摄像头标定调整）
    float distance_mm = (640 - target_y) * 0.5f;  // 简化转换，需要实际标定
    float lateral_mm = (target_x - 320) * 0.3f;
    
    // 移动到物料附近
    if(lateral_mm > 10.0f)
    {
        Move_Lateral_Distance_Stable(lateral_mm, 0.0f);
    }
    
    if(distance_mm > 50.0f)
    {
        Move_Forward_Distance_Stable(distance_mm - 50.0f, 0.0f);
    }
    
    // 4. 根据物料估计重量选择类型
    MaterialType_t material_type = MATERIAL_A_LIGHT;
    if(VIS_RX.material_area > 5000)  // 大面积可能是重物料
    {
        material_type = MATERIAL_A_HEAVY;
    }
    
    // 5. 执行抓取
    bool success = Gripper_GrabMaterial(material_type);
    
    if(success)
    {
        OLED_ShowString(4, 1, "Material Grabbed");
    }
    
    return success;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      比赛专用：测试台操作
//  @return     true: 成功, false: 失败
//  @param      color: 凸台颜色
//-------------------------------------------------------------------------------------------------------------------
bool Competition_TestPlatform(uint8_t color)
{
    if(!Gripper_HasMaterial())
    {
        OLED_ShowString(4, 1, "No Material");
        return false;
    }
    
    OLED_ShowString(1, 1, "Test Platform");
    
    // 1. 发送凸台识别请求
    Visual_Send_Platform_Request(color);
    
    // 2. 等待视觉返回凸台位置
    uint32_t timeout = HAL_GetTick() + 3000;
    while(!VIS_RX.platform_valid && HAL_GetTick() < timeout)
    {
        HAL_Delay(50);
    }
    
    if(!VIS_RX.platform_valid)
    {
        OLED_ShowString(4, 1, "Platform Not Found");
        return false;
    }
    
    // 3. 精确导航到凸台
    float target_x = VIS_RX.platform_x;
    float target_y = VIS_RX.platform_y;
    
    // 转换坐标并导航
    float distance_mm = (640 - target_y) * 0.5f;
    float lateral_mm = (target_x - 320) * 0.3f;
    
    // 精确移动到凸台正上方
    if(fabsf(lateral_mm) > 5.0f)
    {
        Move_Lateral_Distance_Stable(lateral_mm, 0.0f);
    }
    
    if(distance_mm > 30.0f)
    {
        Move_Forward_Distance_Stable(distance_mm - 30.0f, 0.0f);
    }
    
    // 4. 执行测试放置
    bool success = Gripper_PlaceMaterialOnPlatform();
    
    if(success)
    {
        // 等待2秒观察物料是否掉落
        HAL_Delay(2000);
        
        // 重新抓取测试合格的物料
        success = Competition_RetrieveMaterialFromPlatform(color);
    }
    
    return success;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      比赛专用：从测试台取回合格物料
//  @return     true: 成功, false: 失败
//  @param      color: 凸台颜色
//-------------------------------------------------------------------------------------------------------------------
bool Competition_RetrieveMaterialFromPlatform(uint8_t color)
{
    OLED_ShowString(1, 1, "Retrieve Material");
    
    // 1. 重新识别凸台位置（物料可能有微小移动）
    Visual_Send_Platform_Request(color);
    
    uint32_t timeout = HAL_GetTick() + 2000;
    while(!VIS_RX.platform_valid && HAL_GetTick() < timeout)
    {
        HAL_Delay(50);
    }
    
    if(!VIS_RX.platform_valid)
    {
        return false;
    }
    
    // 2. 微调位置
    float target_x = VIS_RX.platform_x;
    float target_y = VIS_RX.platform_y;
    
    float lateral_adjust = (target_x - 320) * 0.3f;
    if(fabsf(lateral_adjust) > 3.0f)
    {
        Move_Lateral_Distance_Stable(lateral_adjust, 0.0f);
    }
    
    // 3. 重新抓取物料
    bool success = Gripper_GrabMaterial(MATERIAL_A_LIGHT);
    
    if(success)
    {
        OLED_ShowString(4, 1, "Retrieved OK");
    }
    
    return success;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      比赛专用：装配台操作
//  @return     true: 成功, false: 失败
//  @param      color: 凹槽颜色
//  @param      is_second_layer: 是否是第二层码垛
//-------------------------------------------------------------------------------------------------------------------
bool Competition_AssemblySlot(uint8_t color, bool is_second_layer)
{
    if(!Gripper_HasMaterial())
    {
        return false;
    }
    
    if(is_second_layer)
    {
        OLED_ShowString(1, 1, "Stack Layer 2");
    }
    else
    {
        OLED_ShowString(1, 1, "Assembly Layer 1");
    }
    
    // 1. 发送凹槽识别请求
    Visual_Send_Slot_Request(color);
    
    // 2. 等待视觉返回凹槽位置
    uint32_t timeout = HAL_GetTick() + 3000;
    while(!VIS_RX.slot_valid && HAL_GetTick() < timeout)
    {
        HAL_Delay(50);
    }
    
    if(!VIS_RX.slot_valid)
    {
        OLED_ShowString(4, 1, "Slot Not Found");
        return false;
    }
    
    // 3. 导航到凹槽位置
    float target_x = VIS_RX.slot_x;
    float target_y = VIS_RX.slot_y;
    
    float distance_mm = (640 - target_y) * 0.5f;
    float lateral_mm = (target_x - 320) * 0.3f;
    
    // 精确对准凹槽
    if(fabsf(lateral_mm) > 3.0f)
    {
        Move_Lateral_Distance_Stable(lateral_mm, 0.0f);
    }
    
    if(distance_mm > 25.0f)
    {
        Move_Forward_Distance_Stable(distance_mm - 25.0f, 0.0f);
    }
    
    // 4. 执行装配操作
    bool success;
    if(is_second_layer)
    {
        // 第二层：码垛操作
        success = Gripper_StackMaterial();
    }
    else
    {
        // 第一层：放入凹槽
        success = Gripper_PlaceMaterialInSlot();
    }
    
    return success;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      比赛专用：抓取装配体
//  @return     true: 成功, false: 失败
//  @param      color: 装配体颜色
//-------------------------------------------------------------------------------------------------------------------
bool Competition_GrabAssembly(uint8_t color)
{
    OLED_ShowString(1, 1, "Grab Assembly");
    
    // 1. 识别装配体位置
    Visual_Send_Slot_Request(color);  // 复用凹槽识别找到装配体
    
    uint32_t timeout = HAL_GetTick() + 3000;
    while(!VIS_RX.slot_valid && HAL_GetTick() < timeout)
    {
        HAL_Delay(50);
    }
    
    if(!VIS_RX.slot_valid)
    {
        return false;
    }
    
    // 2. 导航到装配体
    float target_x = VIS_RX.slot_x;
    float target_y = VIS_RX.slot_y;
    
    float distance_mm = (640 - target_y) * 0.5f;
    float lateral_mm = (target_x - 320) * 0.3f;
    
    if(fabsf(lateral_mm) > 5.0f)
    {
        Move_Lateral_Distance_Stable(lateral_mm, 0.0f);
    }
    
    if(distance_mm > 30.0f)
    {
        Move_Forward_Distance_Stable(distance_mm - 30.0f, 0.0f);
    }
    
    // 3. 抓取装配体（使用牢固抓取）
    bool success = Gripper_GrabMaterial(MATERIAL_ASSEMBLY);
    
    if(success)
    {
        OLED_ShowString(4, 1, "Assembly Grabbed");
    }
    
    return success;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      比赛专用：穿越障碍区
//  @return     true: 成功, false: 失败
//  @param      void
//-------------------------------------------------------------------------------------------------------------------
bool Competition_CrossObstacle(void)
{
    if(!Gripper_HasMaterial())
    {
        OLED_ShowString(4, 1, "No Assembly");
        return false;
    }
    
    OLED_ShowString(1, 1, "Cross Obstacle");
    
    // 1. 确保抓取牢固
    Gripper_SetGripPosition(GRIP_FIRM, 1000);
    
    // 2. 收回到最安全位置
    Gripper_SetExtendPosition(EXTEND_RETRACT, 1000);
    
    // 3. 缓慢穿越障碍区（降低速度避免振动）
    // 这里需要根据实际障碍区布局调整路径
    
    // 前进穿越障碍区
    bool success = Move_Forward_Distance_Stable(900.0f, 0.0f);  // 障碍区长度约900mm
    
    if(success)
    {
        OLED_ShowString(4, 1, "Obstacle Crossed");
        
        // 检查装配体是否还在
        if(Gripper_HasMaterial())
        {
            OLED_ShowString(4, 1, "Assembly Safe");
            return true;
        }
        else
        {
            OLED_ShowString(4, 1, "Assembly Lost");
            return false;
        }
    }
    
    return false;
}
