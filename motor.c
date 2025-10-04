/**
  ******************************************************************************
  * @file    motor.c
  * @brief   麦克纳姆轮电机控制 (位置模式简化版)
  * @date    2025-10-04
  ******************************************************************************
  * 设计目标:
  *  - 删除 MPU6050 / 里程计依赖, 仅保留位置模式动作
  *  - 结合麦克纳姆运动学计算四轮脉冲
  *  - 电机 2、4 反向安装, 下发命令时需翻转方向
  ******************************************************************************
  */

#include "motor.h"
#include "Emm_V5.h"
#include "main.h"
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

/* ===================== 机械参数 ===================== */
#define PI_F                    3.14159265f
#define WHEEL_RADIUS_MM         48.0f      // 麦克纳姆轮半径 (mm)
#define WHEEL_BASE_MM           300.0f     // 左右轮中心距 (mm)
#define WHEEL_TRACK_MM          300.0f     // 前后轮中心距 (mm)

/* ===================== 电机地址 ===================== */
#define MOTOR_FL                1U         // Front Left  左前
#define MOTOR_FR                2U         // Front Right 右前 (倒装)
#define MOTOR_RL                3U         // Rear  Left  左后
#define MOTOR_RR                4U         // Rear  Right 右后 (倒装)

/* ===================== 控制参数 ===================== */
#define DEFAULT_SPEED_RPM       40U
#define DEFAULT_ACCEL           200U
#define MAX_SPEED_RPM           120U
#define EXTRA_DELAY_MS          100U       // 动作结束附加等待(使用反馈后可缩短)
#define POSITION_POLL_TIMEOUT   10000U     // 位置等待超时 10s
#define POSITION_POLL_INTERVAL  20U        // 位置查询间隔 20ms

// 电机2、4方向翻转 (输入: 1=正转, 0=反转)
#define FIX_DIR_MOTOR_2(dir)    ((dir) ? 0U : 1U)
#define FIX_DIR_MOTOR_4(dir)    ((dir) ? 0U : 1U)

/* ===================== 速度配置 ===================== */
/**
 * @brief 运动速度档位枚举
 * @details 根据不同操作场景选择合适的运动速度
 */
typedef enum {
	SPEED_PRECISE = 0,   /**< 精确模式: 30 RPM (取料/放料/对位) */
	SPEED_NORMAL  = 1,   /**< 正常模式: 50 RPM (载货移动) */
	SPEED_FAST    = 2,   /**< 快速模式: 80 RPM (空载移动/寻位) */
} SpeedProfile_t;

static const uint16_t SPEED_PROFILES[3] = {30, 50, 80}; /**< 各档位对应的RPM值 */
static SpeedProfile_t g_current_speed_profile = SPEED_NORMAL; /**< 当前速度档位 */

/* ===================== 静态工具函数 ===================== */
/**
 * @brief 内部工具函数声明
 * @note 这些函数不对外暴露,仅供内部使用
 */
static uint32_t distance_to_cmd_ticks(float distance_mm);     /**< 距离转Emm_V5命令参数 */
static uint32_t rotation_to_cmd_ticks(float angle_deg);       /**< 角度转Emm_V5命令参数 */
static uint32_t sanitize_speed(uint32_t rpm);                 /**< 速度值合法性检查 */
// static void     issue_position_commands(const uint8_t dirs[4], const uint32_t pulses[4], uint32_t speed_rpm, uint8_t accel); /**< 下发位置控制命令(未使用) */
static void     wait_estimated_motion(float distance_mm, uint32_t speed_rpm); /**< UART反馈等待 */
// static bool     wait_motion_complete_with_feedback(uint32_t timeout_ms); /**< 位置反馈等待(未使用) */
//static bool     check_motor_arrived(uint8_t addr);            /**< 检查单个电机到位 */

/* ===================== 对外接口 ===================== */

/**
 * @brief 电机模块初始化
 * @note 主要用于延时等待系统稳定,可以添加电机使能等操作
 */
void Motor_Init(void)
{
	HAL_Delay(100);
	// 如需: 在此加入驱动器细分设置/回零/使能等操作
}

/**
 * @brief 前进/后退运动 (位置模式)
 * @param distance_mm 移动距离(mm), 正值前进,负值后退
 * @param speed_rpm 速度(RPM), 0表示使用默认速度
 * @note 使用麦克纳姆轮运动学,四个轮子同向同速旋转
 * @attention 距离精度受轮子半径等机械参数影响
 */
void Motor_Move_Forward(float distance_mm, uint16_t speed_rpm)
{
	// 如果用户指定速度,使用指定值;否则使用当前速度配置档位
	uint32_t speed = sanitize_speed((speed_rpm == 0U) ? SPEED_PROFILES[g_current_speed_profile] : speed_rpm);
	uint32_t pulses = distance_to_cmd_ticks(distance_mm);
	uint8_t dir = (distance_mm >= 0.0f) ? 0U : 1U;  // 1=后退, 0=前进

	// 按照原来的方式: 依次发送4个电机命令
	Emm_V5_Pos_Control(MOTOR_FL, dir, speed, DEFAULT_ACCEL, pulses, 0U, 1U);
	HAL_Delay(10);
	
	Emm_V5_Pos_Control(MOTOR_FR, FIX_DIR_MOTOR_2(dir), speed, DEFAULT_ACCEL, pulses, 0U, 1U);
	HAL_Delay(10);
	
	Emm_V5_Pos_Control(MOTOR_RL, dir, speed, DEFAULT_ACCEL, pulses, 0U, 1U);
	HAL_Delay(10);
	
	Emm_V5_Pos_Control(MOTOR_RR, FIX_DIR_MOTOR_4(dir), speed, DEFAULT_ACCEL, pulses, 0U, 1U);
	HAL_Delay(10);
	
	// 发送同步运动命令
	Emm_V5_Synchronous_motion(0U);
	HAL_Delay(10);
	
	// 简单延时等待(按照原来的方式)
	wait_estimated_motion(fabsf(distance_mm), speed);
}

/**
 * @brief 左右平移运动 (位置模式)
 * @param distance_mm 移动距离(mm), 正值右移,负值左移
 * @param speed_rpm 速度(RPM), 0表示使用默认速度
 * @note 使用麦克纳姆轮运动学,左前/右后轮与右前/左后轮反向旋转
 * @attention 比赛场地边界检测很重要,防止越界
 */
void Motor_Move_Lateral(float distance_mm, uint16_t speed_rpm)
{
	// 如果用户指定速度,使用指定值;否则使用当前速度配置档位
	uint32_t speed = sanitize_speed((speed_rpm == 0U) ? SPEED_PROFILES[g_current_speed_profile] : speed_rpm);
	uint32_t pulses = distance_to_cmd_ticks(distance_mm);

	if (distance_mm >= 0.0f)
	{
		// 向右平移 (车体 +X): FL/RR 正转, FR/RL 反转
		Emm_V5_Pos_Control(MOTOR_FL, 1U, speed, DEFAULT_ACCEL, pulses, 0U, 1U);
		HAL_Delay(10);
		Emm_V5_Pos_Control(MOTOR_FR, FIX_DIR_MOTOR_2(0U), speed, DEFAULT_ACCEL, pulses, 0U, 1U);
		HAL_Delay(10);
		Emm_V5_Pos_Control(MOTOR_RL, 0U, speed, DEFAULT_ACCEL, pulses, 0U, 1U);
		HAL_Delay(10);
		Emm_V5_Pos_Control(MOTOR_RR, FIX_DIR_MOTOR_4(1U), speed, DEFAULT_ACCEL, pulses, 0U, 1U);
		HAL_Delay(10);
	}
	else
	{
		// 向左平移 (车体 -X)
		Emm_V5_Pos_Control(MOTOR_FL, 0U, speed, DEFAULT_ACCEL, pulses, 0U, 1U);
		HAL_Delay(10);
		Emm_V5_Pos_Control(MOTOR_FR, FIX_DIR_MOTOR_2(1U), speed, DEFAULT_ACCEL, pulses, 0U, 1U);
		HAL_Delay(10);
		Emm_V5_Pos_Control(MOTOR_RL, 1U, speed, DEFAULT_ACCEL, pulses, 0U, 1U);
		HAL_Delay(10);
		Emm_V5_Pos_Control(MOTOR_RR, FIX_DIR_MOTOR_4(0U), speed, DEFAULT_ACCEL, pulses, 0U, 1U);
		HAL_Delay(10);
	}

	Emm_V5_Synchronous_motion(0U);
	HAL_Delay(10);
	
	wait_estimated_motion(fabsf(distance_mm), speed);
}

/**
 * @brief 原地旋转运动 (位置模式)
 * @param angle_deg 旋转角度(度), 正值逆时针,负值顺时针
 * @param speed_rpm 速度(RPM), 0表示使用默认速度
 * @note 左边轮子与右边轮子反向旋转,旋转半径由轮距决定
 * @attention 角度精度受轮距测量精度影响
 */
void Motor_Move_Rotate(float angle_deg, uint16_t speed_rpm)
{
	// 如果用户指定速度,使用指定值;否则使用当前速度配置档位
	uint32_t speed = sanitize_speed((speed_rpm == 0U) ? SPEED_PROFILES[g_current_speed_profile] : speed_rpm);
	uint32_t pulses = rotation_to_cmd_ticks(angle_deg);

	if (angle_deg >= 0.0f)
	{
		// 逆时针 (车体 +Yaw): 左轮正转, 右轮反转
		Emm_V5_Pos_Control(MOTOR_FL, 1U, speed, DEFAULT_ACCEL, pulses, 0U, 1U);
		HAL_Delay(10);
		Emm_V5_Pos_Control(MOTOR_FR, FIX_DIR_MOTOR_2(0U), speed, DEFAULT_ACCEL, pulses, 0U, 1U);
		HAL_Delay(10);
		Emm_V5_Pos_Control(MOTOR_RL, 1U, speed, DEFAULT_ACCEL, pulses, 0U, 1U);
		HAL_Delay(10);
		Emm_V5_Pos_Control(MOTOR_RR, FIX_DIR_MOTOR_4(0U), speed, DEFAULT_ACCEL, pulses, 0U, 1U);
		HAL_Delay(10);
	}
	else
	{
		// 顺时针 (车体 -Yaw)
		Emm_V5_Pos_Control(MOTOR_FL, 0U, speed, DEFAULT_ACCEL, pulses, 0U, 1U);
		HAL_Delay(10);
		Emm_V5_Pos_Control(MOTOR_FR, FIX_DIR_MOTOR_2(1U), speed, DEFAULT_ACCEL, pulses, 0U, 1U);
		HAL_Delay(10);
		Emm_V5_Pos_Control(MOTOR_RL, 0U, speed, DEFAULT_ACCEL, pulses, 0U, 1U);
		HAL_Delay(10);
		Emm_V5_Pos_Control(MOTOR_RR, FIX_DIR_MOTOR_4(1U), speed, DEFAULT_ACCEL, pulses, 0U, 1U);
		HAL_Delay(10);
	}

	Emm_V5_Synchronous_motion(0U);
	HAL_Delay(10);
	
	float angle_rad = fabsf(angle_deg) * PI_F / 180.0f;
	float radius = sqrtf((WHEEL_BASE_MM * 0.5f) * (WHEEL_BASE_MM * 0.5f) +
						 (WHEEL_TRACK_MM * 0.5f) * (WHEEL_TRACK_MM * 0.5f));
	float arc = radius * angle_rad;
	wait_estimated_motion(arc, speed);
}

/**
 * @brief 立即停止所有电机
 * @note 发送广播停止命令,所有电机立即停止
 */
void Motor_Stop_All(void)
{
	Emm_V5_Stop_Now(0U, 1U); // 广播 + 立即停止
	HAL_Delay(50);
}

/**
 * @brief 紧急停止 (与Motor_Stop_All相同)
 */
void Motor_Emergency_Stop(void)
{
	Motor_Stop_All();
}

/**
 * @brief 设置运动速度档位
 * @param profile 速度档位: 0=精确, 1=正常, 2=快速
 * @note 影响后续所有运动函数的速度选择
 */
void Motor_Set_Speed_Profile(uint8_t profile)
{
	if (profile <= SPEED_FAST)
	{
		g_current_speed_profile = (SpeedProfile_t)profile;
	}
}

/* ===================== 工具函数实现 ===================== */

/**
 * @brief 将线性移动距离转换为Emm_V5位置命令参数
 * @param distance_mm 车体需要移动的距离(mm)
 * @return Emm_V5命令的脉冲参数值
 * 
 * @details 计算过程:
 * 1. 计算轮子需要转动的圈数: turns = distance_mm / (2π × radius)
 * 2. 转换为角度: angle_deg = turns × 360°
 * 3. 转换为Emm_V5格式: ticks = angle_deg × 10 (0.1°单位)
 * 
 * @note Emm_V5协议规定角度精度为0.1°,所以需要乘以10
 * @attention 轮子半径误差会直接影响距离精度
 */
static uint32_t distance_to_cmd_ticks(float distance_mm)
{
	// Emm_V5 位置命令的 "clk" 参数以 0.1° 为单位(10 tick = 1°)。
	// 这里根据车体行进距离折算轮子旋转角度, 最终返回需要写入命令的数据位。
	float circumference = 2.0f * PI_F * WHEEL_RADIUS_MM;
	float turns = (circumference > 0.0f) ? fabsf(distance_mm) / circumference : 0.0f;
	float degree = turns * 360.0f;
	return (uint32_t)lrintf(degree * 10.0f);
}

/**
 * @brief 将旋转角度转换为Emm_V5位置命令参数
 * @param angle_deg 车体需要旋转的角度(度)
 * @return Emm_V5命令的脉冲参数值
 * 
 * @details 计算过程:
 * 1. 计算旋转半径: radius = sqrt((wheel_base/2)² + (wheel_track/2)²)
 * 2. 计算轮子移动弧长: arc = radius × angle_rad
 * 3. 复用距离转换函数计算脉冲数
 * 
 * @note 原地旋转时轮子走的是圆弧路径,不是直线
 */
static uint32_t rotation_to_cmd_ticks(float angle_deg)
{
	float angle_rad = fabsf(angle_deg) * PI_F / 180.0f;
	float radius = sqrtf((WHEEL_BASE_MM * 0.5f) * (WHEEL_BASE_MM * 0.5f) +
					 (WHEEL_TRACK_MM * 0.5f) * (WHEEL_TRACK_MM * 0.5f));
	float arc = radius * angle_rad;
	return distance_to_cmd_ticks(arc);
}

/**
 * @brief 速度值合法性检查和限制
 * @param rpm 输入的RPM值
 * @return 经过检查的速度值
 * @note 防止速度过高导致电机丢步或过载
 */
static uint32_t sanitize_speed(uint32_t rpm)
{
	if (rpm == 0U) return DEFAULT_SPEED_RPM;
	if (rpm > MAX_SPEED_RPM) return MAX_SPEED_RPM;
	return rpm;
}

/**
 * @brief 下发位置控制命令到四个电机 (未使用,已注释)
 * @param dirs 四个电机的方向数组 [FL, FR, RL, RR]
 * @param pulses 四个电机的脉冲数数组 [FL, FR, RL, RR]
 * @param speed_rpm 运动速度(RPM)
 * @param accel 加速度
 * 
 * @details Emm_V5_Pos_Control函数参数说明:
 * - addr: 电机地址(1-4)
 * - dir: 方向(0=反转, 1=正转)
 * - vel: 速度(RPM)
 * - acc: 加速度
 * - clk: 位置参数(0.1°单位)
 * - raF: 相对/绝对标志(0=相对, 1=绝对)
 * - snF: 多机同步标志(0=立即执行, 1=等待同步命令)
 * 
 * @note 使用同步运动命令让四个电机同时启动
 */
// static void issue_position_commands(const uint8_t dirs[4], const uint32_t pulses[4], uint32_t speed_rpm, uint8_t accel)
// {
// 	// snF=1U: 启用多机同步,电机收到命令后不立即执行,等待同步信号
// 	Emm_V5_Pos_Control(MOTOR_FL, dirs[0], speed_rpm, accel, pulses[0], 0U, 1U);
// 	Emm_V5_Pos_Control(MOTOR_FR, dirs[1], speed_rpm, accel, pulses[1], 0U, 1U);
// 	Emm_V5_Pos_Control(MOTOR_RL, dirs[2], speed_rpm, accel, pulses[2], 0U, 1U);
// 	Emm_V5_Pos_Control(MOTOR_RR, dirs[3], speed_rpm, accel, pulses[3], 0U, 1U);
// 	
// 	// 发送同步运动命令,让所有电机同时启动
// 	Emm_V5_Synchronous_motion(0U);
// }

/**
 * @brief 等待电机运动完成(通过主动查询状态)
 * @param distance_mm 移动距离(mm) - 用于计算超时时间
 * @param speed_rpm 运动速度(RPM) - 用于计算超时时间
 * 
 * @details 主动查询电机状态标志位:
 * - 周期性发送查询命令 Emm_V5_Read_Sys_Params(addr, S_FLAG)
 * - 解析返回数据: rxCmd[4] bit0=到位标志
 * - 支持超时保护,防止无限等待
 * 
 * @note 电机驱动器不会主动发送完成信号,必须主动查询
 */
static void wait_estimated_motion(float distance_mm, uint32_t speed_rpm)
{
	// 计算超时时间: 基于距离和速度估算,留2倍余量
	// 运动时间(s) = 距离(mm) / (速度(RPM) × 周长(mm) / 60)
	float circumference = 2.0f * PI_F * WHEEL_RADIUS_MM;
	float estimated_time_s = fabsf(distance_mm) / (speed_rpm * circumference / 60.0f);
	uint32_t timeout_ms = (uint32_t)(estimated_time_s * 2000.0f) + 1000U;  // 2倍余量 + 1秒保底
	
	uint32_t start_tick = HAL_GetTick();
	uint32_t last_query_tick = 0;
	const uint32_t query_interval = 50;  // 查询间隔50ms
	
	// 轮询等待,直到任意一个电机到位或超时
	while ((HAL_GetTick() - start_tick) < timeout_ms)
	{
		// 限制查询频率,避免总线拥塞
		if ((HAL_GetTick() - last_query_tick) >= query_interval)
		{
			last_query_tick = HAL_GetTick();
			
			// 查询电机1(左前轮)的状态标志位
			// 因为是同步运动,所以只需查询一个电机即可
			Emm_V5_Read_Sys_Params(MOTOR_FL, S_FLAG);
			
			// 等待UART接收完成
			HAL_Delay(10);
			
			// 检查返回数据: rxCmd[0]=地址, rxCmd[1]=0xFD, rxCmd[2]=0x3A(读取命令), rxCmd[3]=参数类型, rxCmd[4]=状态标志
			// rxCmd[4] bit0=到位标志: 1=到位, 0=未到位
			if (rxFrameFlag && rxCmd[1] == 0xFD && rxCmd[2] == 0x3A)
			{
				if (rxCmd[4] & 0x01)  // bit0 置位表示到位
				{
					rxFrameFlag = false;
					return;  // 电机已到位,退出等待
				}
				rxFrameFlag = false;  // 清除标志,继续等待
			}
		}
		
		// 短暂休眠,降低CPU占用
		HAL_Delay(5);
	}
	
	// 超时仍未到位,清除标志并返回(可能堵转或参数错误)
	rxFrameFlag = false;
}

/**
 * @brief 检查单个电机是否到位
 * @param addr 电机地址
 * @return true=到位, false=未到位或读取失败
 * @note 方案三: 返回false强制使用延时等待,不读取反馈
 */
// static bool check_motor_arrived(uint8_t addr)
// {
// 	// 方案三: 直接返回false,强制wait函数使用几何计算的延时
// 	// 优点: 简单可靠,不依赖UART协议解析
// 	// 缺点: 无法检测堵转/失步,需要延时估算准确
// 	(void)addr; // 避免未使用参数警告
// 	return false; // 强制使用延时等待
// }

/**
 * @brief 等待所有电机到位 (未使用,已注释)
 * @param timeout_ms 超时时间(毫秒)
 * @return 始终返回false
 * @note 此函数已被 wait_estimated_motion 中的 UART 反馈等待替代
 */
// static bool wait_motion_complete_with_feedback(uint32_t timeout_ms)
// {
// 	(void)timeout_ms; // 避免未使用参数警告
// 	return false;
// }

/* ===================== 兼容封装 ===================== */
/**
 * @brief 兼容旧接口: 前进/后退运动 (忽略航向参数)
 * @param d_mm 移动距离(mm), 正值前进,负值后退
 * @param target_yaw_deg 目标航向角(度), 此版本忽略
 * @return 始终返回true
 * @note 为保持与旧代码兼容,忽略航向保持功能
 */
bool Move_Forward_Distance_Stable(float d_mm, float target_yaw_deg)
{
	(void)target_yaw_deg; // 忽略航向参数
	Motor_Move_Forward(d_mm, 0U);
	return true;
}

/**
 * @brief 兼容旧接口: 左右平移运动 (忽略航向参数)
 * @param d_mm 移动距离(mm), 正值右移,负值左移
 * @param target_yaw_deg 目标航向角(度), 此版本忽略
 * @return 始终返回true
 * @note 为保持与旧代码兼容,忽略航向保持功能
 */
bool Move_Lateral_Distance_Stable(float d_mm, float target_yaw_deg)
{
	(void)target_yaw_deg; // 忽略航向参数
	Motor_Move_Lateral(d_mm, 0U);
	return true;
}

/**
 * @brief 兼容旧接口: 原地旋转90度
 * @param clockwise true: 顺时针旋转90°, false: 逆时针旋转90°
 * @return 始终返回true
 * @note 旋转方向: clockwise=true表示顺时针(车体-yaw)
 */
bool Move_Rotate_90_Degrees(bool clockwise)
{
	Motor_Move_Rotate(clockwise ? -90.0f : 90.0f, 0U);
	return true;
}

