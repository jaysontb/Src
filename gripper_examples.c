// 比赛主流程中的机械爪使用示例

#include "gripper.h"
#include "visual_comm.h"
#include "motor.h"

// 比赛完整流程示例
void Competition_Main_Task(void)
{
    // 初始化
    Gripper_Init();
    
    // 第一阶段：测试台操作
    for(int round = 0; round < 2; round++)  // 两轮物料
    {
        // 1. 抓取红色物料
        if(Competition_GrabMaterialByColor(COLOR_RED))
        {
            // 2. 导航到测试台
            Navigate_To_Testing_Platform();
            
            // 3. 测试红色凸台
            if(Competition_TestPlatform(COLOR_RED))
            {
                // 4. 导航到装配台
                Navigate_To_Assembly_Platform();
                
                // 5. 放入红色凹槽（第一层）
                Competition_AssemblySlot(COLOR_RED, false);
            }
        }
        
        // 重复绿色和蓝色物料...
    }
    
    // 第二阶段：码垛操作
    for(int position = 0; position < 3; position++)
    {
        uint8_t colors[] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE};
        uint8_t color = colors[position];
        
        // 1. 抓取物料B
        if(Competition_GrabMaterialByColor(color))
        {
            // 2. 导航到装配台
            Navigate_To_Assembly_Platform();
            
            // 3. 码垛到第二层
            Competition_AssemblySlot(color, true);
        }
    }
    
    // 第三阶段：搬运装配体
    // 1. 抓取一个装配体
    if(Competition_GrabAssembly(COLOR_RED))  // 或根据策略选择颜色
    {
        // 2. 穿越障碍区
        if(Competition_CrossObstacle())
        {
            // 3. 回到启停区
            Navigate_To_Start_Area();
            
            // 4. 放下装配体（可选）
            Gripper_PlaceMaterial();
        }
    }
    
    // 比赛结束
    Gripper_RetractToSafe();
    OLED_ShowString(1, 1, "Competition Done");
}

// 视觉引导精确对位示例
void Visual_Guided_Precise_Placement(uint8_t target_color, bool is_platform)
{
    // 1. 粗略导航到目标区域
    Navigate_To_Target_Area();
    
    // 2. 启动视觉识别
    if(is_platform)
    {
        Visual_Send_Platform_Request(target_color);
    }
    else
    {
        Visual_Send_Slot_Request(target_color);
    }
    
    // 3. 等待视觉结果
    while(!VIS_RX.target_valid)
    {
        HAL_Delay(50);
    }
    
    // 4. 精确对位
    float error_x = VIS_RX.target_x - 320;  // 像素误差
    float error_y = VIS_RX.target_y - 240;
    
    // 转换为实际距离
    float lateral_adjust = error_x * 0.3f;   // mm
    float forward_adjust = error_y * 0.5f;   // mm
    
    // 微调位置
    if(fabsf(lateral_adjust) > 2.0f)
    {
        Move_Lateral_Distance_Stable(lateral_adjust, 0.0f);
    }
    
    if(fabsf(forward_adjust) > 2.0f)
    {
        Move_Forward_Distance_Stable(forward_adjust, 0.0f);
    }
    
    // 5. 执行精确操作
    if(is_platform)
    {
        Gripper_PlaceMaterialOnPlatform();
    }
    else
    {
        Gripper_PlaceMaterialInSlot();
    }
}

// 错误恢复策略
void Gripper_Error_Recovery(void)
{
    GripperState_t state = Gripper_GetState();
    
    if(state == GRIPPER_ERROR)
    {
        OLED_ShowString(4, 1, "Gripper Error");
        
        // 尝试重新初始化
        HAL_Delay(1000);
        Gripper_Init();
        
        // 检查是否恢复正常
        if(Gripper_IsReady())
        {
            OLED_ShowString(4, 1, "Recovery OK");
        }
        else
        {
            OLED_ShowString(4, 1, "Manual Check Needed");
            // 可能需要人工介入
        }
    }
}
