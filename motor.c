#include "motor.h"
#include "Emm_V5.h"
#include "inv_mpu.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#define WHEEL_RADIUS_MM     48.0f     // 轮半径

//电机地址
#define MOTOR_FL  1
#define MOTOR_FR  2
#define MOTOR_RL  3
#define MOTOR_RR  4

// 轨迹模式参数（需调）
#define ACC_RPM_S  800
#define DEC_RPM_S  800
#define MAX_RPM    200
#define EMM_DEFAULT_ACC   50U

// 累计角度（度，正数取绝对累加，方向单独用dir）
//static float acc_deg_FL = 0, acc_deg_FR = 0, acc_deg_RL = 0, acc_deg_RR = 0;

// 航向角控制器
static HeadingController_t heading_ctrl = {
    .target_yaw_deg = 0.0f,
    .current_yaw_deg = 0.0f,
    .yaw_error_deg = 0.0f,
    .kp_yaw = 2.0f,              // P控制器增益，需要调参
    .max_correction_rpm = 50.0f,  // 最大修正转速
    .enable_heading_hold = false
};

bool Motor_readangle(uint8_t motor_adr , float *angle)
{
    if (angle == NULL)
    {
        return false;
    }

    // 发送读取实时位置命令
    Emm_V5_Read_Sys_Params(motor_adr, S_CPOS);

    // 等待返回数据 (阻塞直到收到帧)
    while (rxFrameFlag == false)
    {
        // 简单延时，避免空转
        HAL_Delay(1);
    }
    rxFrameFlag = false;

    // 校验地址、功能码和长度
    if (rxCmd[0] == motor_adr && rxCmd[1] == 0x36 && rxCount == 8)
    {
        uint32_t pos;

        pos = ((uint32_t)rxCmd[3] << 24) |
              ((uint32_t)rxCmd[4] << 16) |
              ((uint32_t)rxCmd[5] << 8)  |
              ((uint32_t)rxCmd[6]);

        *angle = (float)pos * 0.1f;
        if (rxCmd[2] != 0U)
        {
            *angle = -*angle;
        }
        return true;
    }

    return false;
}

// 距离(mm) -> 电机角度(度)
// static float distance_mm_to_motor_deg(float mm)
// {
//     float n_rev_wheel = mm / (2.0f * 3.1415926f * WHEEL_RADIUS_MM);
//     float n_rev_motor = n_rev_wheel * 1.0f;
//     return n_rev_motor * 360.0f;
// }

// 电机角度(度) -> 距离(mm)
static float motor_deg_to_distance_mm(float deg)
{
    float n_rev_motor = deg / 360.0f;
    float n_rev_wheel = n_rev_motor * 1.0f;
    return n_rev_wheel * (2.0f * 3.1415926f * WHEEL_RADIUS_MM);
}

// 读取所有电机位置
static bool Motor_ReadAllPositions(float positions[4])
{
    return (Motor_readangle(MOTOR_FL, &positions[0]) &&
            Motor_readangle(MOTOR_FR, &positions[1]) &&
            Motor_readangle(MOTOR_RL, &positions[2]) &&
            Motor_readangle(MOTOR_RR, &positions[3]));
}

// 发送速度指令到所有电机
static void Motor_SetAllVelocities(float wheel_speeds[4])
{
    float speed_fl = wheel_speeds[0];
    float speed_fr = wheel_speeds[1];
    float speed_rl = wheel_speeds[2];
    float speed_rr = wheel_speeds[3];

    // 限制范围
    if (speed_fl > MAX_RPM) speed_fl = MAX_RPM;
    if (speed_fl < -MAX_RPM) speed_fl = -MAX_RPM;
    if (speed_fr > MAX_RPM) speed_fr = MAX_RPM;
    if (speed_fr < -MAX_RPM) speed_fr = -MAX_RPM;
    if (speed_rl > MAX_RPM) speed_rl = MAX_RPM;
    if (speed_rl < -MAX_RPM) speed_rl = -MAX_RPM;
    if (speed_rr > MAX_RPM) speed_rr = MAX_RPM;
    if (speed_rr < -MAX_RPM) speed_rr = -MAX_RPM;

    uint8_t dir_fl = (speed_fl >= 0.0f) ? 0 : 1;
    uint8_t dir_fr = (speed_fr >= 0.0f) ? 0 : 1;
    uint8_t dir_rl = (speed_rl >= 0.0f) ? 0 : 1;
    uint8_t dir_rr = (speed_rr >= 0.0f) ? 0 : 1;

    uint16_t rpm_fl = (uint16_t)(fabsf(speed_fl) + 0.5f);
    uint16_t rpm_fr = (uint16_t)(fabsf(speed_fr) + 0.5f);
    uint16_t rpm_rl = (uint16_t)(fabsf(speed_rl) + 0.5f);
    uint16_t rpm_rr = (uint16_t)(fabsf(speed_rr) + 0.5f);

    Emm_V5_Vel_Control(MOTOR_FL, dir_fl, rpm_fl, EMM_DEFAULT_ACC, false);
    Emm_V5_Vel_Control(MOTOR_FR, dir_fr, rpm_fr, EMM_DEFAULT_ACC, false);
    Emm_V5_Vel_Control(MOTOR_RL, dir_rl, rpm_rl, EMM_DEFAULT_ACC, false);
    Emm_V5_Vel_Control(MOTOR_RR, dir_rr, rpm_rr, EMM_DEFAULT_ACC, false);
}

// 停止所有电机
static void Motor_StopAll(void)
{
    Emm_V5_Stop_Now(MOTOR_FL, false);
    Emm_V5_Stop_Now(MOTOR_FR, false);
    Emm_V5_Stop_Now(MOTOR_RL, false);
    Emm_V5_Stop_Now(MOTOR_RR, false);
}

// /**
//  * @brief 兼容接口：前进/后退运动（使用当前航向角）
//  * @param d_mm 移动距离(mm)
//  */
// void Move_Forward_Distance(float d_mm)
// {
//     float roll, pitch, current_yaw = 0.0f;
//   
//     // 获取当前航向角
//     if(MPU6050_DMP_Get_Data(&pitch, &roll, &current_yaw) != 0)
//     {
//         current_yaw = 0.0f; // 如果读取失败，使用0度
//     }
//    
//     Move_Forward_Distance_Stable(d_mm, current_yaw);
// }
//
// /**
//  * @brief 兼容接口：左右平移运动（使用当前航向角）
//  * @param d_mm 移动距离(mm)
//  */
// void Move_Lateral_Distance(float d_mm)
// {
//     float roll, pitch, current_yaw = 0.0f;
//    
//     // 获取当前航向角
//     if(MPU6050_DMP_Get_Data(&pitch, &roll, &current_yaw) != 0)
//     {
//         current_yaw = 0.0f; // 如果读取失败，使用0度
//     }
//    
//     Move_Lateral_Distance_Stable(d_mm, current_yaw);
// }


bool Motor_Odom_Update(Odom2D_t *odom)
{
    if(!odom) return false;
    float pos[4];
    if(!Motor_ReadAllPositions(pos)) return false;

    float r,p,yaw=0.0f;
    bool imu_ok = (mpu_dmp_get_data(&p,&r,&yaw)==0);
    if(imu_ok) odom->yaw_deg = yaw; // 保留旧 yaw 如果读取失败

    if(!odom->initialized) {
        for(int i=0;i<4;i++) odom->last_deg[i] = pos[i];
        odom->initialized = true;
        return true;
    }

    // 计算每个轮的弧度增量
    float d_rad[4];
    for(int i=0;i<4;i++) {
        d_rad[i] = (pos[i] - odom->last_deg[i]) * (3.1415926f/180.0f);
        odom->last_deg[i] = pos[i];
    }

    float dsFL = WHEEL_RADIUS_MM * d_rad[0];
    float dsFR = WHEEL_RADIUS_MM * d_rad[1];
    float dsRL = WHEEL_RADIUS_MM * d_rad[2];
    float dsRR = WHEEL_RADIUS_MM * d_rad[3];

    float dX_body = 0.25f * (dsFL + dsFR + dsRL + dsRR);
    float dY_body = 0.25f * (-dsFL + dsFR + dsRL - dsRR);

    float yaw_rad = odom->yaw_deg * (3.1415926f/180.0f);
    float cy = cosf(yaw_rad), sy = sinf(yaw_rad);
    float dX_world =  cy * dX_body - sy * dY_body;
    float dY_world =  sy * dX_body + cy * dY_body;

    odom->x_mm += dX_world;
    odom->y_mm += dY_world;
    return true;
}

void Motor_Odom_Get(const Odom2D_t *odom, float *x_mm, float *y_mm, float *yaw_deg)
{
    if(!odom) return;
    if(x_mm) *x_mm = odom->x_mm;
    if(y_mm) *y_mm = odom->y_mm;
    if(yaw_deg) *yaw_deg = odom->yaw_deg;
}

// 航向角稳定控制函数实现

/**
 * @brief 设置目标航向角
 */
void Motor_SetHeadingTarget(float target_yaw_deg)
{
    heading_ctrl.target_yaw_deg = target_yaw_deg;
}

/**
 * @brief 启用/禁用航向保持
 */
void Motor_EnableHeadingHold(bool enable)
{
    heading_ctrl.enable_heading_hold = enable;
}

/**
 * @brief 角度标准化到[-180, 180]度范围
 */
static float normalize_angle(float angle_deg)
{
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

/**
 * @brief 更新航向角控制器
 * @param current_yaw_deg 当前航向角(度)
 * @return true: 控制器正常工作, false: 未启用或出错
 */
bool Motor_UpdateHeadingController(float current_yaw_deg)
{
    if (!heading_ctrl.enable_heading_hold) {
        return false;
    }
    
    heading_ctrl.current_yaw_deg = current_yaw_deg;
    
    // 计算航向角误差，考虑角度环绕
    heading_ctrl.yaw_error_deg = normalize_angle(
        heading_ctrl.target_yaw_deg - heading_ctrl.current_yaw_deg
    );
    
    return true;
}

/**
 * @brief 计算航向角修正速度
 * @return 修正转速(RPM), 正值表示向右修正，负值表示向左修正
 */
float calculate_yaw_correction_rpm(void)
{
    if (!heading_ctrl.enable_heading_hold) {
        return 0.0f;
    }
    
    // P控制器
    float correction_rpm = heading_ctrl.kp_yaw * heading_ctrl.yaw_error_deg;
    
    // 限制修正范围
    if (correction_rpm > heading_ctrl.max_correction_rpm) {
        correction_rpm = heading_ctrl.max_correction_rpm;
    } else if (correction_rpm < -heading_ctrl.max_correction_rpm) {
        correction_rpm = -heading_ctrl.max_correction_rpm;
    }
    
    return correction_rpm;
}

/**
 * @brief [周期调用] 根据当前航向修正前进速度
 * @param base_speed_rpm 基础前进速度(RPM), >0前进, <0后退
 * @param wheel_speeds   (输出) 计算出的四个轮子的目标速度数组
 */
void Motor_Yaw_Corrected_Forward(float base_speed_rpm, float wheel_speeds[4])
{
    float correction_rpm = 0.0f;
    float current_yaw = 0.0f;
    float roll, pitch;

    // 1. 获取当前航向角
    if (mpu_dmp_get_data(&pitch, &roll, &current_yaw) == 0) {
        // 2. 更新航向控制器，计算误差
        Motor_UpdateHeadingController(current_yaw);
        // 3. 根据误差计算修正速度
        correction_rpm = calculate_yaw_correction_rpm();
    }
    // 如果IMU读取失败，则不进行修正，correction_rpm为0

    // 4. 计算最终轮速 (前进时，左右轮差速实现转向)
    //    左轮需要减速(或反向加速)来实现右转, 右轮反之
    wheel_speeds[0] = base_speed_rpm - correction_rpm; // FL
    wheel_speeds[1] = base_speed_rpm + correction_rpm; // FR
    wheel_speeds[2] = base_speed_rpm - correction_rpm; // RL
    wheel_speeds[3] = base_speed_rpm + correction_rpm; // RR
}

/**
 * @brief [周期调用] 根据当前航向修正平移速度
 * @param base_speed_rpm 基础平移速度(RPM), >0向左, <0向右
 * @param wheel_speeds   (输出) 计算出的四个轮子的目标速度数组
 */
void Motor_Yaw_Corrected_Strafe(float base_speed_rpm, float wheel_speeds[4])
{
    float correction_rpm = 0.0f;
    float current_yaw = 0.0f;
    float roll, pitch;

    // 1. 获取当前航向角并计算修正量
    if (mpu_dmp_get_data(&pitch, &roll, &current_yaw) == 0) {
        Motor_UpdateHeadingController(current_yaw);
        correction_rpm = calculate_yaw_correction_rpm();
    }

    // 2. 定义基础轮速方向 (向左平移时: FL-, FR+, RL+, RR-)
    float base_fl = -base_speed_rpm;
    float base_fr =  base_speed_rpm;
    float base_rl =  base_speed_rpm;
    float base_rr = -base_speed_rpm;

    // 3. 计算最终轮速 (平移时，所有轮子同向增减速实现转向)
    //    为了向右转(correction_rpm > 0), 所有轮子都需要增加一个“右转”的速度分量
    //    右转时轮速: FL+, FR-, RL+, RR-
    wheel_speeds[0] = base_fl + correction_rpm; // FL
    wheel_speeds[1] = base_fr - correction_rpm; // FR
    wheel_speeds[2] = base_rl + correction_rpm; // RL
    wheel_speeds[3] = base_rr - correction_rpm; // RR
}

/**
 * @brief 带航向角稳定的前进/后退运动 (速度模式+位置反馈)
 * @param d_mm 移动距离(mm), >0前进, <0后退
 * @param target_yaw_deg 目标航向角(度)
 * @return true: 运动完成, false: 出错或被打断
 */
bool Move_Forward_Distance_Stable(float d_mm, float target_yaw_deg)
{
    float start_positions[4], current_positions[4];
    float target_distance = fabsf(d_mm);
    float base_speed_rpm = 120.0f; // 基础速度
    float wheel_speeds[4];
    int8_t direction = (d_mm >= 0) ? 1 : -1;
    
    // 1. 读取起始位置
    if(!Motor_ReadAllPositions(start_positions))
    {
        return false; // 读取失败
    }
    
    // 2. 设置目标航向角并启用航向保持
    Motor_SetHeadingTarget(target_yaw_deg);
    Motor_EnableHeadingHold(true);
    
    // 3. 主控制循环
    while(1)
    {
        // 读取当前位置
        if(!Motor_ReadAllPositions(current_positions))
        {
            Motor_StopAll();
            Motor_EnableHeadingHold(false);
            return false;
        }
        
        // 计算平均行驶距离
        float distances_moved[4];
        float total_distance = 0;
        
        for(int i = 0; i < 4; i++)
        {
            distances_moved[i] = motor_deg_to_distance_mm(
                fabsf(current_positions[i] - start_positions[i])
            );
            total_distance += distances_moved[i];
        }
        
        float avg_distance_moved = total_distance / 4.0f;
        
        // 检查是否到达目标
        if(avg_distance_moved >= target_distance)
        {
            Motor_StopAll();
            Motor_EnableHeadingHold(false);
            return true;
        }
        
        // 根据剩余距离调整速度（减速控制）
        float remaining_distance = target_distance - avg_distance_moved;
        float current_speed = base_speed_rpm;
        
        if(remaining_distance < 200.0f) // 距离目标200mm时开始减速
        {
            current_speed = base_speed_rpm * (remaining_distance / 200.0f);
            if(current_speed < 30.0f) current_speed = 30.0f; // 最小速度
        }
        
        // 应用方向
        current_speed *= direction;
        
        // 计算带航向修正的轮速
        Motor_Yaw_Corrected_Forward(current_speed, wheel_speeds);
        
        // 发送速度指令
        Motor_SetAllVelocities(wheel_speeds);
        
        HAL_Delay(50); // 50ms控制周期
    }
}

/**
 * @brief 带航向角稳定的左右平移运动 (速度模式+位置反馈)
 * @param d_mm 移动距离(mm), >0向左, <0向右
 * @param target_yaw_deg 目标航向角(度)
 * @return true: 运动完成, false: 出错或被打断
 */
bool Move_Lateral_Distance_Stable(float d_mm, float target_yaw_deg)
{
    float start_positions[4], current_positions[4];
    float target_distance = fabsf(d_mm);
    float base_speed_rpm = 100.0f; // 侧向运动稍慢
    float wheel_speeds[4];
    int8_t direction = (d_mm >= 0) ? 1 : -1;
    
    // 1. 读取起始位置
    if(!Motor_ReadAllPositions(start_positions))
    {
        return false;
    }
    
    // 2. 设置目标航向角并启用航向保持
    Motor_SetHeadingTarget(target_yaw_deg);
    Motor_EnableHeadingHold(true);
    
    // 3. 主控制循环
    while(1)
    {
        // 读取当前位置
        if(!Motor_ReadAllPositions(current_positions))
        {
            Motor_StopAll();
            Motor_EnableHeadingHold(false);
            return false;
        }
        
        // 计算侧向移动距离（基于轮子角度变化）
        // 对于麦克纳姆轮，侧向移动时每个轮子的角度变化相同
        float avg_deg_moved = 0;
        for(int i = 0; i < 4; i++)
        {
            avg_deg_moved += fabsf(current_positions[i] - start_positions[i]);
        }
        avg_deg_moved /= 4.0f;
        
        float distance_moved = motor_deg_to_distance_mm(avg_deg_moved);
        
        // 检查是否到达目标
        if(distance_moved >= target_distance)
        {
            Motor_StopAll();
            Motor_EnableHeadingHold(false);
            return true;
        }
        
        // 根据剩余距离调整速度
        float remaining_distance = target_distance - distance_moved;
        float current_speed = base_speed_rpm;
        
        if(remaining_distance < 150.0f) // 距离目标150mm时开始减速
        {
            current_speed = base_speed_rpm * (remaining_distance / 150.0f);
            if(current_speed < 25.0f) current_speed = 25.0f; // 最小速度
        }
        
        // 应用方向
        current_speed *= direction;
        
        // 计算带航向修正的轮速
        Motor_Yaw_Corrected_Strafe(current_speed, wheel_speeds);
        
        // 发送速度指令
        Motor_SetAllVelocities(wheel_speeds);
        
        HAL_Delay(50); // 50ms控制周期
    }
}

/**
 * @brief 原地旋转90度
 * @param clockwise true: 顺时针旋转90°, false: 逆时针旋转90°
 * @return true: 旋转完成, false: 出错或被打断
 */
bool Move_Rotate_90_Degrees(bool clockwise)
{
    float current_yaw = 0.0f;
    float roll, pitch;
    float target_yaw;
    float base_rotation_speed = 80.0f; // 旋转基础速度 (RPM)
    float wheel_speeds[4];
    uint32_t timeout_ms = 5000; // 5秒超时
    uint32_t start_time = HAL_GetTick();
    
    // 1. 获取当前航向角
    if (mpu_dmp_get_data(&pitch, &roll, &current_yaw) != 0)
    {
        return false; // IMU读取失败
    }
    
    // 2. 计算目标航向角
    if (clockwise)
    {
        target_yaw = current_yaw - 90.0f; // 顺时针减90度
    }
    else
    {
        target_yaw = current_yaw + 90.0f; // 逆时针加90度
    }
    
    // 标准化目标角度到[-180, 180]范围
    target_yaw = normalize_angle(target_yaw);
    
    // 3. 设置航向保持目标并启用
    Motor_SetHeadingTarget(target_yaw);
    Motor_EnableHeadingHold(true);
    
    // 4. 主旋转控制循环
    while(1)
    {
        // 超时检查
        if((HAL_GetTick() - start_time) > timeout_ms)
        {
            Motor_StopAll();
            Motor_EnableHeadingHold(false);
            return false; // 超时
        }
        
        // 获取当前航向角
        if (mpu_dmp_get_data(&pitch, &roll, &current_yaw) != 0)
        {
            Motor_StopAll();
            Motor_EnableHeadingHold(false);
            return false; // IMU读取失败
        }
        
        // 更新航向控制器
        Motor_UpdateHeadingController(current_yaw);
        
        // 计算角度误差
        float angle_error = normalize_angle(target_yaw - current_yaw);
        
        // 检查是否到达目标 (允许±3度误差)
        if(fabsf(angle_error) <= 3.0f)
        {
            Motor_StopAll();
            Motor_EnableHeadingHold(false);
            return true; // 旋转完成
        }
        
        // 根据剩余角度调整旋转速度
        float current_rotation_speed = base_rotation_speed;
        
        if(fabsf(angle_error) < 30.0f) // 剩余角度小于30度时开始减速
        {
            current_rotation_speed = base_rotation_speed * (fabsf(angle_error) / 30.0f);
            if(current_rotation_speed < 20.0f) current_rotation_speed = 20.0f; // 最小旋转速度
        }
        
        // 根据误差方向确定旋转方向
        if(angle_error > 0) // 需要逆时针旋转
        {
            current_rotation_speed = current_rotation_speed;
        }
        else // 需要顺时针旋转
        {
            current_rotation_speed = -current_rotation_speed;
        }
        
        // 设置麦克纳姆轮原地旋转轮速
        // 原地顺时针旋转: FL+, FR-, RL+, RR-
        // 原地逆时针旋转: FL-, FR+, RL-, RR+
        wheel_speeds[0] =  current_rotation_speed; // FL
        wheel_speeds[1] = -current_rotation_speed; // FR
        wheel_speeds[2] =  current_rotation_speed; // RL
        wheel_speeds[3] = -current_rotation_speed; // RR
        
        // 发送速度指令
        Motor_SetAllVelocities(wheel_speeds);
        
        HAL_Delay(50); // 50ms控制周期
    }
}
