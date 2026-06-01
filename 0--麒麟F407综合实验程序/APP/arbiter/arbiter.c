#include "arbiter.h"
#include "can.h"
#include "usart.h"
#include "stdio.h"
#include "SysTick.h"

// 外部声明 OSTime（在 time.c 中定义）
extern volatile u32 OSTime;

static s16 Chassis_ParseS16BE(u8 hi, u8 lo);
static s16 Chassis_MotionAbs(s16 v);

// ============================================================================
// 全局变量
// ============================================================================

// 状态名称字符串
const char* ARBITER_MODE_NAMES[] = {
	"NORMAL",       // 正常模式
	"SPEED_LIMIT",  // 限速模式
	"DEGRADED",     // 降级模式
	"EMERGENCY",    // 紧急停车
	"RECOVERING"    // 恢复中
};

// 仲裁器状态实例
ArbiterState_t arb_state;

// ============================================================================
// 内部辅助函数声明
// ============================================================================

static void Arbiter_SwitchMode(ArbiterMode_t new_mode);
static void Arbiter_ProcessNormalMode(void);
static void Arbiter_ProcessSpeedLimitMode(void);
static void Arbiter_ProcessDegradedMode(void);
static void Arbiter_ProcessEmergencyMode(void);
static void Arbiter_ProcessRecoveringMode(void);
static void Arbiter_CheckHeartbeat(void);

// ============================================================================
// 函数实现
// ============================================================================

/*******************************************************************************
* 函 数 名         : Arbiter_Init
* 功能描述		   : 初始化仲裁器
*******************************************************************************/
void Arbiter_Init(void)
{
	// 清零状态
	arb_state.current_mode = ARBITER_MODE_NORMAL;
	arb_state.last_heartbeat_tick = OSTime;
	arb_state.heartbeat_lost = 0;
	arb_state.recover_start_tick = 0;
	
	// 初始化Jetson指令
	arb_state.jetson_cmd.v = 0;
	arb_state.jetson_cmd.omega = 0;
	arb_state.jetson_cmd.heartbeat = 0;
	arb_state.jetson_cmd.valid = 0;
	
	// 初始化输出
	arb_state.output.v = 0;
	arb_state.output.omega = 0;
	arb_state.output.mode = ARBITER_MODE_NORMAL;
	arb_state.output.emergency = 0;
	
	// 初始化传感器数据
	arb_state.nearest_dist = 0xFFFF;
	arb_state.obstacle_detected = 0;
	
	printf("[Arbiter] Initialized, mode: %s\r\n", ARBITER_MODE_NAMES[arb_state.current_mode]);
}

/*******************************************************************************
* 函 数 名         : Arbiter_UpdateHeartbeat
* 功能描述		   : 更新心跳（收到Jetson数据时调用）
*******************************************************************************/
void Arbiter_UpdateHeartbeat(void)
{
	arb_state.last_heartbeat_tick = OSTime;
	arb_state.heartbeat_lost = 0;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseJetsonCmd
* 功能描述		   : 接收并解析Jetson指令
* 输    入         : frame: 数据帧指针, len: 数据长度
* 输    出         : 0=成功, 1=失败
* 帧格式           : [0xFF][v高][v低][ω高][ω低][心跳][预留][校验和]
*******************************************************************************/
u8 Arbiter_ParseJetsonCmd(u8* frame, u8 len)
{
	u8 checksum = 0;
	u8 i;
	
	// 验证长度
	if(len != JETSON_FRAME_LEN)
		return 1;
	
	// 验证帧头
	if(frame[0] != JETSON_FRAME_HEADER)
		return 1;
	
	// 计算校验和
	for(i = 0; i < JETSON_FRAME_LEN - 1; i++)
	{
		checksum ^= frame[i];
	}
	
	// 验证校验和
	if(checksum != frame[7])
	{
		printf("[Arbiter] Checksum failed: expected 0x%02X, got 0x%02X\r\n", 
		       checksum, frame[7]);
		return 1;
	}
	
	// 解析速度指令（大端序）
	arb_state.jetson_cmd.v = (frame[1] << 8) | frame[2];
	arb_state.jetson_cmd.omega = (frame[3] << 8) | frame[4];
	arb_state.jetson_cmd.heartbeat = frame[5];
	
	// 验证心跳标识
	if(arb_state.jetson_cmd.heartbeat != JETSON_HEARTBEAT_VAL)
	{
		printf("[Arbiter] Invalid heartbeat: 0x%02X\r\n", arb_state.jetson_cmd.heartbeat);
		return 1;
	}
	
	// 标记数据有效
	arb_state.jetson_cmd.valid = 1;
	
	// 更新心跳
	Arbiter_UpdateHeartbeat();
	
	return 0;
}

/*******************************************************************************
* 函 数 名         : Arbiter_SetObstacleDistance
* 功能描述		   : 设置最近障碍物距离
*******************************************************************************/
void Arbiter_SetObstacleDistance(u16 dist_mm)
{
	/* 65533/0xFFFF 为无效读数，不参与避障 */
	if(dist_mm >= 60000)
	{
		arb_state.nearest_dist = 0xFFFF;
		arb_state.obstacle_detected = 0;
		return;
	}

	arb_state.nearest_dist = dist_mm;
	
	// 判断是否有障碍物
	if(dist_mm < ARBITER_OBSTACLE_NEAR_MM)
	{
		arb_state.obstacle_detected = 1;
	}
	else if(dist_mm < ARBITER_OBSTACLE_FAR_MM)
	{
		arb_state.obstacle_detected = 1;
	}
	else
	{
		arb_state.obstacle_detected = 0;
	}
}

/*******************************************************************************
* 函 数 名         : Arbiter_ClampSpeed
* 功能描述		   : 速度限幅（安全防线）
*******************************************************************************/
s16 Arbiter_ClampSpeed(s16 speed)
{
	if(speed > ARBITER_MAX_SPEED_MM_S)
		return ARBITER_MAX_SPEED_MM_S;
	if(speed < -ARBITER_MAX_SPEED_MM_S)
		return -ARBITER_MAX_SPEED_MM_S;
	return speed;
}

/*******************************************************************************
* 函 数 名         : Arbiter_SwitchMode
* 功能描述		   : 切换仲裁模式
*******************************************************************************/
static void Arbiter_SwitchMode(ArbiterMode_t new_mode)
{
	if(arb_state.current_mode != new_mode)
	{
		printf("[Arbiter] Mode switch: %s -> %s\r\n", 
		       ARBITER_MODE_NAMES[arb_state.current_mode],
		       ARBITER_MODE_NAMES[new_mode]);
		
		arb_state.current_mode = new_mode;
		
		// 记录恢复开始时间
		if(new_mode == ARBITER_MODE_RECOVERING)
		{
			arb_state.recover_start_tick = OSTime;
		}
	}
}

/*******************************************************************************
* 函 数 名         : Arbiter_CheckHeartbeat
* 功能描述		   : 检查心跳是否超时
*******************************************************************************/
static void Arbiter_CheckHeartbeat(void)
{
	u32 elapsed = OSTime - arb_state.last_heartbeat_tick;
	
	if(elapsed >= ARBITER_HEARTBEAT_TIMEOUT_MS)
	{
		if(!arb_state.heartbeat_lost)
		{
			printf("[Arbiter] Heartbeat lost! (%lu ms)\r\n", (unsigned long)elapsed);
			arb_state.heartbeat_lost = 1;
		}
	}
}

/*******************************************************************************
* 函 数 名         : Arbiter_ProcessNormalMode
* 功能描述		   : 状态1：正常模式 - 透传Jetson指令
*******************************************************************************/
static void Arbiter_ProcessNormalMode(void)
{
	// 检查紧急停车条件
	if(arb_state.nearest_dist < ARBITER_OBSTACLE_NEAR_MM)
	{
		Arbiter_SwitchMode(ARBITER_MODE_EMERGENCY);
		return;
	}
	
	// 检查心跳丢失
	if(arb_state.heartbeat_lost)
	{
		Arbiter_SwitchMode(ARBITER_MODE_DEGRADED);
		return;
	}
	
	// 检查是否有障碍物
	if(arb_state.obstacle_detected)
	{
		Arbiter_SwitchMode(ARBITER_MODE_SPEED_LIMIT);
		return;
	}
	
	// 正常模式：透传Jetson指令
	arb_state.output.v = Arbiter_ClampSpeed(arb_state.jetson_cmd.v);
	arb_state.output.omega = arb_state.jetson_cmd.omega;
	arb_state.output.mode = ARBITER_MODE_NORMAL;
	arb_state.output.emergency = 0;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ProcessSpeedLimitMode
* 功能描述		   : 状态2：限速/避障模式 - 按距离比例限速
*******************************************************************************/
static void Arbiter_ProcessSpeedLimitMode(void)
{
	float ratio;
	s16 limited_v;
	
	// 检查紧急停车条件（最高优先级）
	if(arb_state.nearest_dist < ARBITER_OBSTACLE_NEAR_MM)
	{
		Arbiter_SwitchMode(ARBITER_MODE_EMERGENCY);
		return;
	}
	
	// 检查心跳丢失
	if(arb_state.heartbeat_lost)
	{
		Arbiter_SwitchMode(ARBITER_MODE_DEGRADED);
		return;
	}
	
	// 检查障碍物是否消失
	if(!arb_state.obstacle_detected)
	{
		Arbiter_SwitchMode(ARBITER_MODE_NORMAL);
		return;
	}
	
	// 按距离比例限速
	// 距离越近，限速越严格
	ratio = 0.0f;
	if(arb_state.nearest_dist <= ARBITER_OBSTACLE_NEAR_MM)
	{
		ratio = 0.0f;  // 极近，停止
	}
	else if(arb_state.nearest_dist >= ARBITER_OBSTACLE_FAR_MM)
	{
		ratio = 1.0f;  // 较远，不限速
	}
	else
	{
		// 线性插值：30mm=0%, 150mm=100%
		ratio = (float)(arb_state.nearest_dist - ARBITER_OBSTACLE_NEAR_MM) / 
		        (ARBITER_OBSTACLE_FAR_MM - ARBITER_OBSTACLE_NEAR_MM);
	}
	
	// 应用限速比例
	limited_v = (s16)((float)arb_state.jetson_cmd.v * ratio);
	arb_state.output.v = Arbiter_ClampSpeed(limited_v);
	arb_state.output.omega = arb_state.jetson_cmd.omega;  // 角速度不限速（可根据需要调整）
	arb_state.output.mode = ARBITER_MODE_SPEED_LIMIT;
	arb_state.output.emergency = 0;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ProcessDegradedMode
* 功能描述		   : 状态3：降级模式 - 反应式避障（if-else规则）
*******************************************************************************/
static void Arbiter_ProcessDegradedMode(void)
{
	// 检查紧急停车条件
	if(arb_state.nearest_dist < ARBITER_OBSTACLE_NEAR_MM)
	{
		Arbiter_SwitchMode(ARBITER_MODE_EMERGENCY);
		return;
	}
	
	// 检查心跳是否恢复
	if(!arb_state.heartbeat_lost && arb_state.jetson_cmd.valid)
	{
		// 心跳恢复，但需要车速为0且持续1秒才能切回正常模式
		if(arb_state.output.v == 0 && 
		   (OSTime - arb_state.recover_start_tick) >= ARBITER_RECOVER_STABLE_MS)
		{
			Arbiter_SwitchMode(ARBITER_MODE_RECOVERING);
			return;
		}
	}
	
	// 反应式避障规则
	if(arb_state.nearest_dist < ARBITER_OBSTACLE_NEAR_MM)
	{
		// 极近：紧急停车
		arb_state.output.v = 0;
		arb_state.output.omega = 0;
		arb_state.output.emergency = 1;
	}
	else if(arb_state.nearest_dist < ARBITER_OBSTACLE_FAR_MM)
	{
		// 较近：减速并尝试转向
		arb_state.output.v = 200;  // 低速前进
		arb_state.output.omega = 500;  // 转向避障
		arb_state.output.emergency = 0;
	}
	else
	{
		// 无障碍：低速前进
		arb_state.output.v = 300;
		arb_state.output.omega = 0;
		arb_state.output.emergency = 0;
	}
	
	arb_state.output.mode = ARBITER_MODE_DEGRADED;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ProcessEmergencyMode
* 功能描述		   : 状态4：紧急停车模式 - 强制速度0
*******************************************************************************/
static void Arbiter_ProcessEmergencyMode(void)
{
	// 强制停车
	arb_state.output.v = 0;
	arb_state.output.omega = 0;
	arb_state.output.mode = ARBITER_MODE_EMERGENCY;
	arb_state.output.emergency = 1;
	
	// 检查障碍物是否清除
	if(arb_state.nearest_dist >= ARBITER_OBSTACLE_NEAR_MM)
	{
		// 障碍物清除，进入恢复状态
		Arbiter_SwitchMode(ARBITER_MODE_RECOVERING);
	}
}

/*******************************************************************************
* 函 数 名         : Arbiter_ProcessRecoveringMode
* 功能描述		   : 恢复中状态 - 等待稳定后切回正常模式
*******************************************************************************/
static void Arbiter_ProcessRecoveringMode(void)
{
	// 检查是否再次进入紧急情况
	if(arb_state.nearest_dist < ARBITER_OBSTACLE_NEAR_MM)
	{
		Arbiter_SwitchMode(ARBITER_MODE_EMERGENCY);
		return;
	}
	
	// 检查心跳是否再次丢失
	if(arb_state.heartbeat_lost)
	{
		Arbiter_SwitchMode(ARBITER_MODE_DEGRADED);
		return;
	}
	
	// 检查是否稳定足够时间
	if((OSTime - arb_state.recover_start_tick) >= ARBITER_RECOVER_STABLE_MS)
	{
		// 稳定完成，切回正常模式
		Arbiter_SwitchMode(ARBITER_MODE_NORMAL);
		return;
	}
	
	// 恢复期间保持停车
	arb_state.output.v = 0;
	arb_state.output.omega = 0;
	arb_state.output.mode = ARBITER_MODE_RECOVERING;
	arb_state.output.emergency = 0;
}

/*******************************************************************************
* 函 数 名         : Arbiter_Process
* 功能描述		   : 执行仲裁逻辑（主循环中周期调用）
*******************************************************************************/
void Arbiter_Process(void)
{
	// 检查心跳
	Arbiter_CheckHeartbeat();
	
	// 根据当前模式执行相应逻辑
	switch(arb_state.current_mode)
	{
		case ARBITER_MODE_NORMAL:
			Arbiter_ProcessNormalMode();
			break;
			
		case ARBITER_MODE_SPEED_LIMIT:
			Arbiter_ProcessSpeedLimitMode();
			break;
			
		case ARBITER_MODE_DEGRADED:
			Arbiter_ProcessDegradedMode();
			break;
			
		case ARBITER_MODE_EMERGENCY:
			Arbiter_ProcessEmergencyMode();
			break;
			
		case ARBITER_MODE_RECOVERING:
			Arbiter_ProcessRecoveringMode();
			break;
			
		default:
			Arbiter_SwitchMode(ARBITER_MODE_NORMAL);
			break;
	}
}

/*******************************************************************************
* 函 数 名         : Arbiter_GetOutput
* 功能描述		   : 获取仲裁后输出
*******************************************************************************/
ArbiterOutput_t* Arbiter_GetOutput(void)
{
	return &arb_state.output;
}

/*******************************************************************************
* 函 数 名         : Arbiter_GetMode
* 功能描述		   : 获取当前模式
*******************************************************************************/
ArbiterMode_t Arbiter_GetMode(void)
{
	return arb_state.current_mode;
}

/*******************************************************************************
* 函 数 名         : Arbiter_SendToSTM32A
* 功能描述		   : 发送仲裁后指令到STM32 A（通过CAN）
* 帧格式           : 符合RANGER MINI 3.0 CAN协议（ID:0x111）
*                   byte[0-1]: 线速度 (signed int16, 单位mm/s, MOTOROLA格式)
*                   byte[2-3]: 自旋速度 (signed int16, 单位0.001rad/s, MOTOROLA格式)
*                   byte[4-5]: 保留
*                   byte[6-7]: 转角 (signed int16, 单位0.001rad, MOTOROLA格式)
*******************************************************************************/
void Arbiter_SendToSTM32A(void)
{
	u8 txbuf[8] = {0};
	s16 speed_mm_s;
	s16 spin_speed;  // 自旋速度，单位0.001rad/s
	s16 angle_scaled; // 转角，单位0.001rad
	
	// 获取仲裁后的速度
	speed_mm_s = arb_state.output.v;
	
	// 将角速度转换为自旋速度（单位：0.001rad/s）
	// arb_state.output.omega 的单位已经是 0.01 rad/s
	spin_speed = (s16)(arb_state.output.omega * 100); // 0.01 rad/s -> 0.001rad/s
	
	// 转角暂时设为0（后续可根据需要计算）
	angle_scaled = 0;
	
	// MOTOROLA格式（大端序）：高字节在前
	// byte[0-1]: 线速度
	txbuf[0] = (speed_mm_s >> 8) & 0xFF;
	txbuf[1] = speed_mm_s & 0xFF;
	
	// byte[2-3]: 自旋速度
	txbuf[2] = (spin_speed >> 8) & 0xFF;
	txbuf[3] = spin_speed & 0xFF;
	
	// byte[4-5]: 保留
	txbuf[4] = 0x00;
	txbuf[5] = 0x00;
	
	// byte[6-7]: 转角
	txbuf[6] = (angle_scaled >> 8) & 0xFF;
	txbuf[7] = angle_scaled & 0xFF;
	
	// 发送CAN指令（ID: 0x111）
	CAN1_Send_Msg_WithID(CAN_ARBITER_CMD_ID, txbuf, 8);
	
	// 打印发送调试信息
	{
		static u32 last_print = 0;
		if((OSTime - last_print) >= 500)  // 每500ms打印一次
		{
			last_print = OSTime;
			printf("[CAN TX] 0x111: %02X %02X %02X %02X %02X %02X %02X %02X | v=%d w=%d\r\n",
				txbuf[0], txbuf[1], txbuf[2], txbuf[3],
				txbuf[4], txbuf[5], txbuf[6], txbuf[7],
				speed_mm_s, spin_speed);
		}
	}
}

/*******************************************************************************
* 函 数 名         : Arbiter_PrintStatus
* 功能描述		   : 打印调试信息
*******************************************************************************/
void Arbiter_PrintStatus(void)
{
	printf("[Arbiter] Mode=%s | Jetson(v=%d, ω=%d) | Output(v=%d, ω=%d) | "
	       "Dist=%d mm | HB=%s\r\n",
	       ARBITER_MODE_NAMES[arb_state.current_mode],
	       arb_state.jetson_cmd.v,
	       arb_state.jetson_cmd.omega,
	       arb_state.output.v,
	       arb_state.output.omega,
	       arb_state.nearest_dist,
	       arb_state.heartbeat_lost ? "LOST" : "OK");
}

/*******************************************************************************
* 函 数 名         : Arbiter_SetMode
* 功能描述		   : 发送模式设定指令（ID:0x421）
*******************************************************************************/
void Arbiter_SetMode(CanMode_t mode)
{
	u8 txbuf[1];
	txbuf[0] = (u8)mode;
	CAN1_Send_Msg_WithID(CAN_MODE_CMD_ID, txbuf, 1);
	printf("[Arbiter] Set mode: %s\r\n", mode == CAN_MODE_STANDBY ? "STANDBY" : "CAN_CTRL");
}

/*******************************************************************************
* 函 数 名         : Arbiter_SetDriveMode
* 功能描述		   : 发送驱动模式切换指令（ID:0x423）
*******************************************************************************/
void Arbiter_SetDriveMode(CanDriveMode_t mode)
{
	u8 txbuf[1];
	txbuf[0] = (u8)mode;
	CAN1_Send_Msg_WithID(CAN_DRIVE_CMD_ID, txbuf, 1);
	printf("[Arbiter] Set drive mode: %s\r\n", mode == CAN_DRIVE_CURRENT ? "CURRENT" : "VOLTAGE");
}

/*******************************************************************************
* 函 数 名         : Arbiter_ClearError
* 功能描述		   : 发送错误清除指令（ID:0x441）
*******************************************************************************/
void Arbiter_ClearError(CanErrorClear_t error_type)
{
	u8 txbuf[1];
	txbuf[0] = (u8)error_type;
	CAN1_Send_Msg_WithID(CAN_ERROR_CLEAR_ID, txbuf, 1);
	printf("[Arbiter] Clear error: 0x%02X\r\n", error_type);
}

/*******************************************************************************
* 函 数 名         : Arbiter_SetMotionMode
* 功能描述		   : 发送运动模型切换指令（ID:0x141）
*******************************************************************************/
void Arbiter_SetMotionMode(CanMotionMode_t mode)
{
	u8 txbuf[1];
	txbuf[0] = (u8)mode;
	CAN1_Send_Msg_WithID(CAN_MOTION_MODEL_ID, txbuf, 1);
	printf("[Arbiter] Set motion mode: %d\r\n", mode);
}

/*******************************************************************************
* 函 数 名         : Arbiter_SetLight
* 功能描述		   : 发送灯光控制指令（ID:0x121）
*******************************************************************************/
void Arbiter_SetLight(u8 enable, u8 mode)
{
	u8 txbuf[8] = {0};
	txbuf[0] = enable;
	txbuf[1] = mode;
	CAN1_Send_Msg_WithID(CAN_LIGHT_CTRL_ID, txbuf, 8);
	printf("[Arbiter] Set light: enable=%d, mode=%d\r\n", enable, mode);
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseSysStatus
* 功能描述		   : 解析系统状态回馈帧（ID:0x211）
*******************************************************************************/
void Arbiter_ParseSysStatus(u8* data, u8 len)
{
	if(len != 8) return;
	
	arb_state.sys_status.system_status = data[0];
	arb_state.sys_status.mode_control = data[1];
	arb_state.sys_status.battery_voltage = (data[2] << 8) | data[3];
	arb_state.sys_status.fault_info = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
	
	arb_state.feedback_updated = 1;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseMotionFeedback
* 功能描述		   : 解析运动控制回馈帧（ID:0x221）
*******************************************************************************/
void Arbiter_ParseMotionFeedback(u8* data, u8 len)
{
	if(len != 8) return;
	
	arb_state.motion_fb.linear_speed = Chassis_ParseS16BE(data[0], data[1]);
	arb_state.motion_fb.spin_speed = Chassis_ParseS16BE(data[2], data[3]);
	arb_state.motion_fb.steering_angle = Chassis_ParseS16BE(data[6], data[7]);
	
	arb_state.feedback_updated = 1;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseWheelAngle
* 功能描述		   : 解析四轮转角反馈帧（ID:0x271）
*******************************************************************************/
void Arbiter_ParseWheelAngle(u8* data, u8 len)
{
	if(len != 8) return;
	
	arb_state.wheel_angle.steer5 = (data[0] << 8) | data[1];
	arb_state.wheel_angle.steer6 = (data[2] << 8) | data[3];
	arb_state.wheel_angle.steer7 = (data[4] << 8) | data[5];
	arb_state.wheel_angle.steer8 = (data[6] << 8) | data[7];
	
	arb_state.feedback_updated = 1;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseWheelSpeed
* 功能描述		   : 解析四轮转速反馈帧（ID:0x281）
*******************************************************************************/
void Arbiter_ParseWheelSpeed(u8* data, u8 len)
{
	if(len != 8) return;
	
	arb_state.wheel_speed.wheel1 = Chassis_ParseS16BE(data[0], data[1]);
	arb_state.wheel_speed.wheel2 = Chassis_ParseS16BE(data[2], data[3]);
	arb_state.wheel_speed.wheel3 = Chassis_ParseS16BE(data[4], data[5]);
	arb_state.wheel_speed.wheel4 = Chassis_ParseS16BE(data[6], data[7]);
	
	arb_state.feedback_updated = 1;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseMotionModeFeedback
* 功能描述		   : 解析运动模式回馈帧（ID:0x291）
*******************************************************************************/
void Arbiter_ParseMotionModeFeedback(u8* data, u8 len)
{
	if(len != 3) return;
	
	arb_state.motion_mode_fb.motion_mode = data[0];
	arb_state.motion_mode_fb.switching = data[1];
	arb_state.motion_mode_fb.drive_mode = data[2];
	
	arb_state.feedback_updated = 1;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseMotorHighInfo
* 功能描述		   : 解析电机高速信息反馈帧（ID:0x251~0x258）
*******************************************************************************/
void Arbiter_ParseMotorHighInfo(u8 motor_id, u8* data, u8 len)
{
	u8 idx;
	
	if(len != 8 || motor_id < 1 || motor_id > 8) return;
	
	idx = motor_id - 1;
	arb_state.motor_high[idx].speed_rpm = (data[0] << 8) | data[1];
	arb_state.motor_high[idx].current = (data[2] << 8) | data[3];
	arb_state.motor_high[idx].position = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
	
	arb_state.feedback_updated = 1;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseMotorLowInfo
* 功能描述		   : 解析电机低速信息反馈帧（ID:0x261~0x268）
*******************************************************************************/
void Arbiter_ParseMotorLowInfo(u8 motor_id, u8* data, u8 len)
{
	u8 idx;
	
	if(len != 8 || motor_id < 1 || motor_id > 8) return;
	
	idx = motor_id - 1;
	arb_state.motor_low[idx].voltage = (data[0] << 8) | data[1];
	arb_state.motor_low[idx].driver_temp = (data[2] << 8) | data[3];
	arb_state.motor_low[idx].motor_temp = (s8)data[4];
	arb_state.motor_low[idx].status = data[5];
	
	arb_state.feedback_updated = 1;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseRemoteCtrl
* 功能描述		   : 解析遥控器信息反馈帧（ID:0x241）
*******************************************************************************/
void Arbiter_ParseRemoteCtrl(u8* data, u8 len)
{
	if(len != 8) return;
	
	arb_state.remote_ctrl.sw = data[0];
	arb_state.remote_ctrl.right_lr = (s8)data[1];
	arb_state.remote_ctrl.right_ud = (s8)data[2];
	arb_state.remote_ctrl.left_ud = (s8)data[3];
	arb_state.remote_ctrl.left_lr = (s8)data[4];
	arb_state.remote_ctrl.vra = (s8)data[5];
	
	arb_state.feedback_updated = 1;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseOdometerFront
* 功能描述		   : 解析前轮里程反馈帧（ID:0x311）
*******************************************************************************/
void Arbiter_ParseOdometerFront(u8* data, u8 len)
{
	if(len != 8) return;
	
	arb_state.odometer.front_left = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
	arb_state.odometer.front_right = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
	
	arb_state.feedback_updated = 1;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseOdometerRear
* 功能描述		   : 解析后轮里程反馈帧（ID:0x312）
*******************************************************************************/
void Arbiter_ParseOdometerRear(u8* data, u8 len)
{
	if(len != 8) return;
	
	arb_state.odometer.rear_left = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
	arb_state.odometer.rear_right = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
	
	arb_state.feedback_updated = 1;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseBMSData
* 功能描述		   : 解析BMS数据反馈帧（ID:0x361）
*******************************************************************************/
void Arbiter_ParseBMSData(u8* data, u8 len)
{
	if(len != 8) return;
	
	arb_state.bms_data.soc = data[0];
	arb_state.bms_data.soh = data[1];
	arb_state.bms_data.voltage = (data[2] << 8) | data[3];
	arb_state.bms_data.current = (data[4] << 8) | data[5];
	arb_state.bms_data.temperature = (data[6] << 8) | data[7];
	
	arb_state.feedback_updated = 1;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseBMSAlarm
* 功能描述		   : 解析BMS告警状态反馈帧（ID:0x362）
*******************************************************************************/
void Arbiter_ParseBMSAlarm(u8* data, u8 len)
{
	if(len != 4) return;
	
	arb_state.bms_alarm.alarm_status1 = data[0];
	arb_state.bms_alarm.alarm_status2 = data[1];
	arb_state.bms_alarm.warning_status1 = data[2];
	arb_state.bms_alarm.warning_status2 = data[3];
	
	arb_state.feedback_updated = 1;
}

#define CHASSIS_SPEED_DEAD      10
#define CHASSIS_SPIN_DEAD       50
#define CHASSIS_WDIFF_DEAD      50

static s16 Chassis_MotionAbs(s16 v)
{
	return (v < 0) ? (s16)(-v) : v;
}

static s16 Chassis_ParseS16BE(u8 hi, u8 lo)
{
	return (s16)((u16)((hi << 8) | lo));
}

/*******************************************************************************
* 函 数 名         : Arbiter_GetMotionInfo
* 功能描述		   : 根据 CAN 反馈判断运动方向与速度
* 说    明         : 不用 Steer 字段(常为固定偏置~637)；
*                   左右转由四轮速差判断，直行/进退由 V 判断
*******************************************************************************/
void Arbiter_GetMotionInfo(u8 *dir, s16 *speed)
{
	s16 v, spin;
	ChassisWheelSpeed_t ws;
	s16 left, right, wdiff, avg_ws;
	u8 motion_dir = CHASSIS_MOTION_STOP;
	s16 motion_speed = 0;

	v = arb_state.motion_fb.linear_speed;
	spin = arb_state.motion_fb.spin_speed;
	Arbiter_GetWheelSpeedPhysical(&ws);
	left = (ws.lf + ws.lr) / 2;
	right = (ws.rf + ws.rr) / 2;
	wdiff = left - right;
	avg_ws = (Chassis_MotionAbs(ws.lf) + Chassis_MotionAbs(ws.rf) +
	          Chassis_MotionAbs(ws.lr) + Chassis_MotionAbs(ws.rr)) / 4;

	if(arb_state.motion_mode_fb.motion_mode == CAN_MOTION_SPIN ||
	   (Chassis_MotionAbs(spin) > CHASSIS_SPIN_DEAD && Chassis_MotionAbs(v) < 50))
	{
		motion_dir = CHASSIS_MOTION_SPIN;
		motion_speed = Chassis_MotionAbs(spin);
	}
	else if(Chassis_MotionAbs(v) <= CHASSIS_SPEED_DEAD && avg_ws <= CHASSIS_SPEED_DEAD)
	{
		motion_dir = CHASSIS_MOTION_STOP;
		motion_speed = 0;
	}
	else if(wdiff > CHASSIS_WDIFF_DEAD && (Chassis_MotionAbs(v) > CHASSIS_SPEED_DEAD || avg_ws > CHASSIS_WDIFF_DEAD))
	{
		motion_dir = CHASSIS_MOTION_LEFT;
		motion_speed = Chassis_MotionAbs(v);
		if(motion_speed < CHASSIS_SPEED_DEAD)
			motion_speed = avg_ws;
	}
	else if(wdiff < -CHASSIS_WDIFF_DEAD && (Chassis_MotionAbs(v) > CHASSIS_SPEED_DEAD || avg_ws > CHASSIS_WDIFF_DEAD))
	{
		motion_dir = CHASSIS_MOTION_RIGHT;
		motion_speed = Chassis_MotionAbs(v);
		if(motion_speed < CHASSIS_SPEED_DEAD)
			motion_speed = avg_ws;
	}
	else if(v > CHASSIS_SPEED_DEAD)
	{
		motion_dir = CHASSIS_MOTION_FORWARD;
		motion_speed = v;
	}
	else if(v < -CHASSIS_SPEED_DEAD)
	{
		motion_dir = CHASSIS_MOTION_BACKWARD;
		motion_speed = (s16)(-v);
	}
	else
	{
		motion_dir = CHASSIS_MOTION_STOP;
		motion_speed = avg_ws;
	}

	if(dir)
		*dir = motion_dir;
	if(speed)
		*speed = motion_speed;
}

const char* Arbiter_MotionDirNameAscii(u8 dir)
{
	switch(dir)
	{
		case CHASSIS_MOTION_LEFT:     return "LEFT";
		case CHASSIS_MOTION_RIGHT:    return "RIGHT";
		case CHASSIS_MOTION_SPIN:     return "SPIN";
		case CHASSIS_MOTION_FORWARD:  return "FWD";
		case CHASSIS_MOTION_BACKWARD: return "BACK";
		default:                      return "STOP";
	}
}

/*******************************************************************************
* 函 数 名         : Arbiter_GetWheelSpeedPhysical
* 功能描述		   : CAN轮编号 -> 车体物理位置
* 映    射         : CAN W1=左后  W2=左前  W3=右前  W4=右后
*******************************************************************************/
void Arbiter_GetWheelSpeedPhysical(ChassisWheelSpeed_t *w)
{
	if(!w) return;

	w->lr = arb_state.wheel_speed.wheel1;
	w->lf = arb_state.wheel_speed.wheel2;
	w->rf = arb_state.wheel_speed.wheel3;
	w->rr = arb_state.wheel_speed.wheel4;
}

/*******************************************************************************
* 函 数 名         : Arbiter_PrintChassisFeedback
* 功能描述		   : 打印底盘 CAN 反馈到串口
*******************************************************************************/
void Arbiter_PrintChassisFeedback(void)
{
	u8 motion_dir;
	s16 motion_speed;
	ChassisWheelSpeed_t ws;

	Arbiter_GetMotionInfo(&motion_dir, &motion_speed);
	Arbiter_GetWheelSpeedPhysical(&ws);

	printf("[CAN FB] DIR=%s SPD=%d CMD=%d MODE=%s\r\n",
		Arbiter_MotionDirNameAscii(motion_dir),
		(int)motion_speed,
		(int)arb_state.output.v,
		ARBITER_MODE_NAMES[arb_state.current_mode]);
	printf("[CAN FB] V=%d LF=%d RF=%d LR=%d RR=%d BAT=%d.%dV\r\n",
		(int)arb_state.motion_fb.linear_speed,
		(int)ws.lf, (int)ws.rf, (int)ws.lr, (int)ws.rr,
		(int)(arb_state.sys_status.battery_voltage / 10),
		(int)(arb_state.sys_status.battery_voltage % 10));
}

/*******************************************************************************
* 函 数 名         : Arbiter_ProcessCANFeedback
* 功能描述		   : 处理 CAN 反馈数据（在主循环中调用）
* 说    明         : 接收 STM32 A 的反馈帧并分发到对应解析函数
*******************************************************************************/
void Arbiter_ProcessCANFeedback(void)
{
	u32 can_id;
	u8 buf[8];
	u8 len;
	
	// 尝试接收 CAN 消息
	len = CAN1_Receive_Msg_WithID(&can_id, buf);
	if(len == 0) return;
	
	// 根据 CAN ID 分发到对应解析函数
	switch(can_id)
	{
		case CAN_SYS_STATUS_ID:
			Arbiter_ParseSysStatus(buf, len);
			break;
			
		case CAN_MOTION_FB_ID:
			Arbiter_ParseMotionFeedback(buf, len);
			break;
			
		case CAN_LIGHT_FB_ID:
			// 灯光控制反馈帧（可根据需要扩展）
			break;
			
		case CAN_REMOTE_CTRL_ID:
			Arbiter_ParseRemoteCtrl(buf, len);
			break;
			
		case CAN_MOTOR_HIGH_ID:
		case CAN_MOTOR_HIGH_ID + 1:
		case CAN_MOTOR_HIGH_ID + 2:
		case CAN_MOTOR_HIGH_ID + 3:
		case CAN_MOTOR_HIGH_ID + 4:
		case CAN_MOTOR_HIGH_ID + 5:
		case CAN_MOTOR_HIGH_ID + 6:
		case CAN_MOTOR_HIGH_ID + 7:
			Arbiter_ParseMotorHighInfo(can_id - CAN_MOTOR_HIGH_ID + 1, buf, len);
			break;
			
		case CAN_MOTOR_LOW_ID:
		case CAN_MOTOR_LOW_ID + 1:
		case CAN_MOTOR_LOW_ID + 2:
		case CAN_MOTOR_LOW_ID + 3:
		case CAN_MOTOR_LOW_ID + 4:
		case CAN_MOTOR_LOW_ID + 5:
		case CAN_MOTOR_LOW_ID + 6:
		case CAN_MOTOR_LOW_ID + 7:
			Arbiter_ParseMotorLowInfo(can_id - CAN_MOTOR_LOW_ID + 1, buf, len);
			break;
			
		case CAN_WHEEL_ANGLE_ID:
			Arbiter_ParseWheelAngle(buf, len);
			break;
			
		case CAN_WHEEL_SPEED_ID:
			Arbiter_ParseWheelSpeed(buf, len);
			break;
			
		case CAN_MOTION_MODE_FB_ID:
			Arbiter_ParseMotionModeFeedback(buf, len);
			break;
			
		case CAN_ODOM_FRONT_ID:
			Arbiter_ParseOdometerFront(buf, len);
			break;
			
		case CAN_ODOM_REAR_ID:
			Arbiter_ParseOdometerRear(buf, len);
			break;
			
		case CAN_BMS_DATA_ID:
			Arbiter_ParseBMSData(buf, len);
			break;
			
		case CAN_BMS_ALARM_ID:
			Arbiter_ParseBMSAlarm(buf, len);
			break;
			
		default:
			// 未知 CAN ID，忽略
			break;
	}
}

/*******************************************************************************
* 函 数 名         : Arbiter_EnableCANMode
* 功能描述		   : 上电初始化时切换到 CAN 指令模式
*******************************************************************************/
void Arbiter_EnableCANMode(void)
{
	// 发送模式设定指令，切换到 CAN 指令模式
	Arbiter_SetMode(CAN_MODE_CAN_CTRL);
	
	// 默认使用电流驱动模式（静音）
	Arbiter_SetDriveMode(CAN_DRIVE_CURRENT);
	
	// 默认使用前后阿克曼模式
	Arbiter_SetMotionMode(CAN_MOTION_ACKERMANN);
	
	printf("[Arbiter] CAN mode enabled, ready to control chassis\r\n");
}

/*******************************************************************************
* 函 数 名         : Arbiter_SetLocalCmd
* 功能描述		   : 设置本地运动指令（main 循环调用，无 Jetson 时使用）
*******************************************************************************/
void Arbiter_SetLocalCmd(s16 v, s16 omega)
{
	arb_state.jetson_cmd.v = v;
	arb_state.jetson_cmd.omega = omega;
	arb_state.jetson_cmd.heartbeat = JETSON_HEARTBEAT_VAL;
	arb_state.jetson_cmd.valid = 1;
	Arbiter_UpdateHeartbeat();
}
