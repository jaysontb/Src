

#include "motor.h"
#include "Emm_V5.h"
#include "main.h"
#include "inv_mpu.h"    // 用于航向保持功能
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

/* ===================== 机械参数 ===================== */
#define PI_F                    3.14159265f
#define WHEEL_RADIUS_MM         48.0f      // 麦克纳姆轮半径 (mm)
#define WHEEL_BASE_MM           260.0f     // 左右轮中心距 (mm)
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

/* ===================== 卡尔曼滤波器 ===================== */
/**
 * @brief 一维卡尔曼滤波器结构体(用于航向角平滑)
 */
typedef struct {
	float x;          /**< 状态估计值(当前最优估计) */
	float p;          /**< 估计误差协方差(不确定性) */
	float q;          /**< 过程噪声协方差(系统模型不确定性,越大越信任测量值) */
	float r;          /**< 测量噪声协方差(传感器噪声,越大越信任预测值) */
	float k;          /**< 卡尔曼增益(自动计算) */
	bool initialized; /**< 初始化标志 */
} KalmanFilter1D_t;

static KalmanFilter1D_t g_yaw_filter = {0};  /**< 全局航向角滤波器 */

/**
 * @brief 初始化卡尔曼滤波器
 * @param kf 滤波器指针
 * @param process_noise 过程噪声(Q): 0.01~0.1,值越大越信任测量
 * @param measurement_noise 测量噪声(R): 0.1~1.0,值越大越平滑但响应慢
 */
static void Kalman_Init(KalmanFilter1D_t *kf, float process_noise, float measurement_noise)
{
	kf->q = process_noise;
	kf->r = measurement_noise;
	kf->p = 1.0f;  // 初始估计误差协方差
	kf->x = 0.0f;
	kf->initialized = false;
}

/**
 * @brief 航向角专用卡尔曼滤波(处理角度环绕问题)
 * @param kf 滤波器指针
 * @param measurement_deg 新的测量值(度)
 * @return 滤波后的航向角(度)
 */
static float Kalman_Update_Yaw(KalmanFilter1D_t *kf, float measurement_deg)
{
	if (!kf->initialized)
	{
		kf->x = measurement_deg;
		kf->initialized = true;
		return kf->x;
	}
	
	// 处理角度环绕: 如果测量值与估计值相差>180°,说明跨越了±180°边界
	float delta = measurement_deg - kf->x;
	if (delta > 180.0f)
	{
		measurement_deg -= 360.0f;
	}
	else if (delta < -180.0f)
	{
		measurement_deg += 360.0f;
	}
	
	// 正常卡尔曼更新
	kf->p = kf->p + kf->q;
	kf->k = kf->p / (kf->p + kf->r);
	kf->x = kf->x + kf->k * (measurement_deg - kf->x);
	kf->p = (1.0f - kf->k) * kf->p;
	
	// 归一化到[-180, 180]
	while (kf->x > 180.0f) kf->x -= 360.0f;
	while (kf->x < -180.0f) kf->x += 360.0f;
	
	return kf->x;
}

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

static float normalize_angle_deg(float angle_deg);

/* ===================== 对外接口 ===================== */

/**
 * @brief 电机模块初始化
 * @note 主要用于延时等待系统稳定,可以添加电机使能等操作
 */
void Motor_Init(void)
{
	HAL_Delay(100);
	
	// 初始化航向角卡尔曼滤波器
	// Q=0.05: 过程噪声,适中的信任度
	// R=0.5: 测量噪声,适度平滑但保持响应性
	Kalman_Init(&g_yaw_filter, 0.05f, 0.5f);
	
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
 * @brief 等待电机运动完成 (UART空闲中断优化版)
 * @param distance_mm 移动距离(mm) - 用于计算超时时间
 * @param speed_rpm 运动速度(RPM) - 用于计算超时时间
 * 
 * @details 使用中断标志+__WFI()低功耗等待,响应速度<1ms (原50ms):
 * - 周期性发送查询命令 Emm_V5_Read_Sys_Params(addr, S_FLAG)
 * - USART1空闲中断检测到位标志并设置motor_arrived_flag
 * - 主循环检查标志或__WFI()休眠等待中断唤醒
 * - 支持超时保护,防止无限等待
 * 
 * @note 电机驱动器不会主动发送完成信号,必须主动查询
 */
static void wait_estimated_motion(float distance_mm, uint32_t speed_rpm)
{
	extern volatile bool motor_arrived_flag;
	
	// 计算超时时间: 基于距离和速度估算,留2倍余量
	// 运动时间(s) = 距离(mm) / (速度(RPM) × 周长(mm) / 60)
	float circumference = 2.0f * PI_F * WHEEL_RADIUS_MM;
	float estimated_time_s = fabsf(distance_mm) / (speed_rpm * circumference / 60.0f);
	uint32_t timeout_ms = (uint32_t)(estimated_time_s * 2000.0f);  // 2倍余量 + 1秒保底
	
	uint32_t start_tick = HAL_GetTick();
	uint32_t last_query_tick = 0;
	const uint32_t query_interval = 20;  // 优化: 20ms查询间隔(原50ms)
	
	motor_arrived_flag = false;  // 清除到位标志
	rxFrameFlag = false;
	
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
			
			// 等待UART接收完成或到位标志置位
			uint32_t wait_start = HAL_GetTick();
			while ((HAL_GetTick() - wait_start) < 20) {  // 最多等20ms
				if (motor_arrived_flag) {  // 中断已检测到到位
					motor_arrived_flag = false;
					return;
				}
				__WFI();  // 进入低功耗模式,等待任意中断唤醒
			}
			
			// 兼容旧方式: 如果中断未检测到,手动检查rxCmd
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
		else
		{
			__WFI();  // 查询间隔内休眠,降低CPU占用
		}
	}
	
	// 超时清除标志
	motor_arrived_flag = false;
	rxFrameFlag = false;
}

//获取旋转角度
float Motor_Get_Rotation_From_Yaw(float yaw_start_deg, float yaw_now_deg)
{
	float delta = yaw_now_deg - yaw_start_deg;
	delta = normalize_angle_deg(delta);
	return -delta;
}
static float normalize_angle_deg(float angle_deg)
{
	while (angle_deg > 180.0f)
	{
		angle_deg -= 360.0f;
	}
	while (angle_deg < -180.0f)
	{
		angle_deg += 360.0f;
	}
	return angle_deg;
}


/**
 * @brief 带航向保持的前进运动
 * @details 分段执行移动,每100mm检查航向并自动修正偏差
 * @param distance_mm 目标距离(mm)
 * @param target_yaw_deg 目标航向角(度)
 * @param tolerance_deg 允许偏差(度)
 * @param speed_rpm 速度(RPM), 0=使用默认
 */
void Motor_Move_Forward_WithYawHold(float distance_mm, float target_yaw_deg,
                                     float tolerance_deg, uint16_t speed_rpm)
{
	const float segment = 400.0f;  // 每100mm检查一次航向
	float remaining = distance_mm;
	float pitch, roll, current_yaw;
	
	// 外部声明OLED函数
	extern void OLED_ShowString(uint8_t Line, uint8_t Column, const char *String);
	char debug_str[20];
	
	// 分段移动循环
	while (fabsf(remaining) > 1.0f)
	{
		// 1. 计算本次移动步长
		float step = (fabsf(remaining) > segment) ? 
		             (remaining > 0 ? segment : -segment) : remaining;
		
		// 2. 执行小段移动
		Motor_Move_Forward(step, speed_rpm);
		
		// 3. 读取当前航向(带重试机制)
		uint8_t read_result = 1;  // 默认失败
		for (uint8_t retry = 0; retry < 5; retry++)  // 最多重试5次
		{
			read_result = mpu_dmp_get_data(&pitch, &roll, &current_yaw);
			if (read_result == 0) break;  // 读取成功,退出重试
			HAL_Delay(10);  // 等待FIFO填充新数据
		}
		
		if (read_result == 0)
		{
			// 应用卡尔曼滤波平滑航向角
			float filtered_yaw = Kalman_Update_Yaw(&g_yaw_filter, current_yaw);
			
			// 4. 计算航向偏差(归一化到±180°)
			float yaw_error = 0.1f * (180 - normalize_angle_deg(filtered_yaw - target_yaw_deg));
			
			// 显示当前yaw(滤波后)和误差
			snprintf(debug_str, sizeof(debug_str), "Y:%.1f E:%.1f", filtered_yaw, yaw_error);
			OLED_ShowString(4, 1, debug_str);
			
			// 5. 如果超出容差,立即修正
			if (fabsf(yaw_error) > tolerance_deg)
			{
				snprintf(debug_str, sizeof(debug_str), "Fix Err:%.1f", yaw_error);
				OLED_ShowString(3, 1, debug_str);
				
				Motor_Move_Rotate(-yaw_error, speed_rpm);  // 反向修正偏航
				HAL_Delay(100);
			}
			else
			{
				OLED_ShowString(3, 1, "YawOK          ");
			}
		}
		else
		{
			// 读取失败,显示警告
			OLED_ShowString(3, 1, "MPU Read Fail!");
			OLED_ShowString(4, 1, "Check FIFO...  ");
		}
		
		remaining -= step;
	}
}

/**
 * @brief 带航向保持的平移运动
 * @details 分段执行平移,每100mm检查航向并自动修正偏差
 * @param distance_mm 目标距离(mm)
 * @param target_yaw_deg 目标航向角(度)
 * @param tolerance_deg 允许偏差(度)
 * @param speed_rpm 速度(RPM), 0=使用默认
 */
void Motor_Move_Lateral_WithYawHold(float distance_mm, float target_yaw_deg,
                                     float tolerance_deg, uint16_t speed_rpm)
{
	const float segment = 400.0f;  // 每400mm检查一次航向
	float remaining = distance_mm;
	float pitch, roll, current_yaw;
	uint8_t correction_count = 0;  // 记录修正次数
	uint8_t read_fail_count = 0;   // 记录读取失败次数
	
	// 外部声明OLED函数
	extern void OLED_ShowString(uint8_t Line, uint8_t Column, const char *String);
	char debug_str[20];
	
	// 分段移动循环
	while (fabsf(remaining) > 1.0f)
	{
		// 1. 计算本次移动步长
		float step = (fabsf(remaining) > segment) ? 
		             (remaining > 0 ? segment : -segment) : remaining;
		
		// 2. 执行小段平移
		Motor_Move_Lateral(step, speed_rpm);
		
		// 3. 读取当前航向(带重试机制)
		uint8_t read_result = 1;  // 默认失败
		for (uint8_t retry = 0; retry < 5; retry++)  // 最多重试5次
		{
			read_result = mpu_dmp_get_data(&pitch, &roll, &current_yaw);
			if (read_result == 0) break;  // 读取成功,退出重试
			HAL_Delay(10);  // 等待FIFO填充新数据
		}
		
		if (read_result == 0)
		{
			// 应用卡尔曼滤波平滑航向角
			float filtered_yaw = Kalman_Update_Yaw(&g_yaw_filter, current_yaw);

			// 4. 计算航向偏差(归一化到±180°)
			float yaw_error = 0.2f*(180 - normalize_angle_deg(filtered_yaw - target_yaw_deg));

			// 显示当前yaw(滤波后)和误差
			snprintf(debug_str, sizeof(debug_str), "Y:%.1f E:%.1f", filtered_yaw, yaw_error);
			OLED_ShowString(4, 1, debug_str);
			
			// 5. 如果超出容差,立即修正
			if (fabsf(yaw_error) > tolerance_deg)
			{
				correction_count++;
				snprintf(debug_str, sizeof(debug_str), "Fix#%d Err:%.1f", correction_count, yaw_error);
				OLED_ShowString(3, 1, debug_str);
				
				Motor_Move_Rotate(-yaw_error, speed_rpm);  // 反向修正偏航
				HAL_Delay(100);  // 短暂延时让你看清修正
			}
			else
			{
				// 航向正常,清空第3行
				OLED_ShowString(3, 1, "YawOK          ");
			}
		}
		else
		{
			// 读取失败,显示警告
			read_fail_count++;
			snprintf(debug_str, sizeof(debug_str), "MPU Fail #%d", read_fail_count);
			OLED_ShowString(3, 1, debug_str);
			OLED_ShowString(4, 1, "FIFO Empty?    ");
		}
		
		remaining -= step;
	}
}
