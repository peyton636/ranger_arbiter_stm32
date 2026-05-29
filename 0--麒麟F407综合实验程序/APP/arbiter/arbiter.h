#ifndef _arbiter_H
#define _arbiter_H

#include "system.h"

// ============================================================================
// 仲裁层状态机定义（STM32 B）
// ============================================================================

// 5种工作状态
typedef enum
{
	ARBITER_MODE_NORMAL = 0,      // 状态1：正常模式（透传Jetson指令）
	ARBITER_MODE_SPEED_LIMIT = 1, // 状态2：限速/避障模式（按比例限速）
	ARBITER_MODE_DEGRADED = 2,    // 状态3：降级模式（反应式避障）
	ARBITER_MODE_EMERGENCY = 3,   // 状态4：紧急停车模式（最高优先级）
	ARBITER_MODE_RECOVERING = 4   // 恢复中状态（等待稳定）
} ArbiterMode_t;

// 状态名称字符串（用于调试显示）
extern const char* ARBITER_MODE_NAMES[];

// ============================================================================
// 通信协议定义（RANGER MINI 3.0 CAN2.0B 协议，MOTOROLA格式，500K波特率）
// ============================================================================

// Jetson → STM32 B 数据帧格式（通过USART2接收）
// [0xFF][v高][v低][ω高][ω低][心跳标识][预留][校验和]
#define JETSON_FRAME_HEADER   0xFF
#define JETSON_FRAME_LEN      8
#define JETSON_HEARTBEAT_VAL  0xAA  // 心跳标识

// STM32 B → STM32 A 控制指令帧
#define CAN_ARBITER_CMD_ID    0x111  // 运动控制帧（周期20ms，超时500ms）
#define CAN_MODE_CMD_ID       0x421  // 模式设定帧
#define CAN_DRIVE_CMD_ID      0x423  // 驱动模式切换帧
#define CAN_ERROR_CLEAR_ID    0x441  // 错误清除帧
#define CAN_MOTION_MODEL_ID   0x141  // 运动模型切换帧
#define CAN_LIGHT_CTRL_ID     0x121  // 灯光控制帧（周期20ms，超时500ms）

// STM32 A → STM32 B 反馈帧
#define CAN_SYS_STATUS_ID     0x211  // 系统状态回馈帧（周期20ms）
#define CAN_MOTION_FB_ID      0x221  // 运动控制回馈帧（周期20ms）
#define CAN_LIGHT_FB_ID       0x231  // 灯光控制反馈帧（周期20ms）
#define CAN_REMOTE_CTRL_ID    0x241  // 遥控器信息反馈帧（周期20ms）
#define CAN_MOTOR_HIGH_ID     0x251  // 电机高速信息反馈帧（0x251~0x258，周期20ms）
#define CAN_MOTOR_LOW_ID      0x261  // 电机低速信息反馈帧（0x261~0x268，周期100ms）
#define CAN_WHEEL_ANGLE_ID    0x271  // 四轮转角反馈帧（周期20ms）
#define CAN_WHEEL_SPEED_ID    0x281  // 四轮转速反馈帧（周期20ms）
#define CAN_MOTION_MODE_FB_ID 0x291  // 运动模式回馈帧（周期20ms）
#define CAN_ODOM_FRONT_ID     0x311  // 前轮里程反馈帧（周期20ms）
#define CAN_ODOM_REAR_ID      0x312  // 后轮里程反馈帧（周期20ms）
#define CAN_BMS_DATA_ID       0x361  // BMS数据反馈帧（周期500ms）
#define CAN_BMS_ALARM_ID      0x362  // BMS告警状态反馈帧（周期500ms）

// 控制模式定义
typedef enum
{
	CAN_MODE_STANDBY = 0x00,     // 待机模式
	CAN_MODE_CAN_CTRL = 0x01     // CAN指令控制模式
} CanMode_t;

// 驱动模式定义
typedef enum
{
	CAN_DRIVE_CURRENT = 0x00,    // 电流驱动模式（默认，伺服软，静音）
	CAN_DRIVE_VOLTAGE = 0x01     // 电压驱动模式（伺服硬，噪音大，越障强）
} CanDriveMode_t;

// 运动模式定义
typedef enum
{
	CAN_MOTION_ACKERMANN = 0x00, // 前后阿克曼模式（默认）
	CAN_MOTION_SLIDE = 0x01,     // 斜移模式
	CAN_MOTION_SPIN = 0x02,      // 自旋模式
	CAN_MOTION_PARK = 0x03       // 驻车模式
} CanMotionMode_t;

// 错误清除指令定义
typedef enum
{
	CAN_ERR_CLEAR_ALL = 0x00,        // 清除全部非严重故障
	CAN_ERR_CLEAR_MOTOR1 = 0x01,     // 清除1号电机驱动器通讯故障
	CAN_ERR_CLEAR_MOTOR2 = 0x02,     // 清除2号电机驱动器通讯故障
	CAN_ERR_CLEAR_MOTOR3 = 0x03,     // 清除3号电机驱动器通讯故障
	CAN_ERR_CLEAR_MOTOR4 = 0x04,     // 清除4号电机驱动器通讯故障
	CAN_ERR_CLEAR_MOTOR5 = 0x05,     // 清除5号电机驱动器通讯故障
	CAN_ERR_CLEAR_MOTOR6 = 0x06,     // 清除6号电机驱动器通讯故障
	CAN_ERR_CLEAR_MOTOR7 = 0x07,     // 清除7号电机驱动器通讯故障
	CAN_ERR_CLEAR_MOTOR8 = 0x08,     // 清除8号电机驱动器通讯故障
	CAN_ERR_CLEAR_BATTERY = 0x09,    // 清除电池欠压故障
	CAN_ERR_CLEAR_REMOTE = 0x0A,     // 清除遥控信号丢失故障
	CAN_ERR_CLEAR_STEER5 = 0x0B,     // 清除5号电机转向校准故障
	CAN_ERR_CLEAR_STEER6 = 0x0C,     // 清除6号电机转向校准故障
	CAN_ERR_CLEAR_STEER7 = 0x0D,     // 清除7号电机转向校准故障
	CAN_ERR_CLEAR_STEER8 = 0x0E,     // 清除8号电机转向校准故障
	CAN_ERR_CLEAR_OVERCURRENT = 0x0F,// 清除过流故障
	CAN_ERR_CLEAR_OVERTEMP = 0x10    // 清除过温故障
} CanErrorClear_t;

// ============================================================================
// 仲裁参数配置
// ============================================================================

// 心跳超时阈值（ms）
#define ARBITER_HEARTBEAT_TIMEOUT_MS  300

// 恢复稳定时间（ms）
#define ARBITER_RECOVER_STABLE_MS     1000

// 超声波避障距离阈值（mm）
#define ARBITER_OBSTACLE_NEAR_MM      30   // 紧急停车距离
#define ARBITER_OBSTACLE_FAR_MM       80   // 限速距离阈值

// 速度限幅参数（mm/s）
#define ARBITER_MAX_SPEED_MM_S        1000  // 最大速度
#define ARBITER_MIN_SPEED_MM_S        0     // 最小速度

// ============================================================================
// 数据结构定义
// ============================================================================

// Jetson速度指令
typedef struct
{
	s16 v;       // 期望线速度（mm/s），正=前进，负=后退
	s16 omega;   // 期望角速度（0.01 rad/s），正=左转，负=右转
	u8  heartbeat; // 心跳标识
	u8  valid;     // 数据有效标志
} JetsonCmd_t;

// 仲裁后输出指令
typedef struct
{
	s16 v;         // 仲裁后线速度（mm/s）
	s16 omega;     // 仲裁后角速度（0.01 rad/s）
	ArbiterMode_t mode; // 当前仲裁模式
	u8  emergency;    // 紧急标志
} ArbiterOutput_t;

// 系统状态定义
typedef struct
{
	u8 system_status;      // byte[0]: 0x00=系统正常, 0x02=系统异常
	u8 mode_control;       // byte[1]: 0x00=待机, 0x01=CAN指令, 0x03=遥控
	u16 battery_voltage;   // byte[2-3]: 实际电压X10 (0.1V精度)
	u32 fault_info;        // byte[4-7]: 故障信息
} CanSysStatus_t;

// 运动反馈定义
typedef struct
{
	s16 linear_speed;      // byte[0-1]: 线速度 (mm/s, X1000)
	s16 spin_speed;        // byte[2-3]: 自旋速度 (0.001rad/s)
	s16 steering_angle;    // byte[6-7]: 内转角 (0.001rad)
} CanMotionFeedback_t;

// 四轮转角定义
typedef struct
{
	s16 steer5;            // byte[0-1]: 5号转向转角 (0.001rad)
	s16 steer6;            // byte[2-3]: 6号转向转角 (0.001rad)
	s16 steer7;            // byte[4-5]: 7号转向转角 (0.001rad)
	s16 steer8;            // byte[6-7]: 8号转向转角 (0.001rad)
} CanWheelAngle_t;

// 四轮转速定义
typedef struct
{
	s16 wheel1;            // byte[0-1]: 1号轮转速 (mm/s)
	s16 wheel2;            // byte[2-3]: 2号轮转速 (mm/s)
	s16 wheel3;            // byte[4-5]: 3号轮转速 (mm/s)
	s16 wheel4;            // byte[6-7]: 4号轮转速 (mm/s)
} CanWheelSpeed_t;

// 运动模式反馈定义
typedef struct
{
	u8 motion_mode;        // byte[0]: 0x00=阿克曼, 0x01=斜移, 0x02=自旋, 0x03=驻车
	u8 switching;          // byte[1]: 0x00=切换完成, 0x01=切换中
	u8 drive_mode;         // byte[2]: 0x00=电流驱动, 0x01=电压驱动
} CanMotionModeFeedback_t;

// 电机高速信息定义
typedef struct
{
	s16 speed_rpm;         // byte[0-1]: 电机转速 (RPM)
	s16 current;           // byte[2-3]: 电机电流 (0.1A)
	s32 position;          // byte[4-7]: 电机位置 (脉冲数)
} CanMotorHighInfo_t;

// 电机低速信息定义
typedef struct
{
	u16 voltage;           // byte[0-1]: 驱动器电压 (0.1V)
	s16 driver_temp;       // byte[2-3]: 驱动器温度 (℃)
	s8  motor_temp;        // byte[4]: 电机温度 (℃)
	u8  status;            // byte[5]: 驱动器状态位
} CanMotorLowInfo_t;

// 遥控器信息定义
typedef struct
{
	u8 sw;                 // byte[0]: SW开关状态
	s8 right_lr;           // byte[1]: 右边拨杆左右 [-100,100]
	s8 right_ud;           // byte[2]: 右边拨杆上下 [-100,100]
	s8 left_ud;            // byte[3]: 左边拨杆上下 [-100,100]
	s8 left_lr;            // byte[4]: 左边拨杆左右 [-100,100]
	s8 vra;                // byte[5]: 左边旋钮VRA [-100,100]
} CanRemoteCtrl_t;

// 里程计信息定义
typedef struct
{
	s32 front_left;        // byte[0-3]: 前轮左里程 (mm)
	s32 front_right;       // byte[4-7]: 前轮右里程 (mm)
	s32 rear_left;         // byte[0-3]: 后轮左里程 (mm)
	s32 rear_right;        // byte[4-7]: 后轮右里程 (mm)
} CanOdometer_t;

// BMS 电池数据定义
typedef struct
{
	u8 soc;                // byte[0]: 电池SOC (0~100%)
	u8 soh;                // byte[1]: 电池SOH (0~100%)
	u16 voltage;           // byte[2-3]: 电池电压 (0.1V)
	s16 current;           // byte[4-5]: 电池电流 (0.1A)
	s16 temperature;       // byte[6-7]: 电池温度 (0.1℃)
} CanBMSData_t;

// BMS 告警状态定义
typedef struct
{
	u8 alarm_status1;      // byte[0]: BIT1=过压 BIT2=欠压 BIT3=高温 BIT4=低温 BIT7=放电过流
	u8 alarm_status2;      // byte[1]: BIT0=充电过流
	u8 warning_status1;    // byte[2]: BIT1=过压 BIT2=欠压 BIT3=高温 BIT4=低温 BIT7=放电过流
	u8 warning_status2;    // byte[3]: BIT0=充电过流
} CanBMSAlarm_t;

// 仲裁器状态（扩展）
typedef struct
{
	ArbiterMode_t current_mode;  // 当前模式
	u32 last_heartbeat_tick;     // 上次心跳时间戳
	u8  heartbeat_lost;          // 心跳丢失标志
	u32 recover_start_tick;      // 恢复开始时间戳
	
	JetsonCmd_t jetson_cmd;      // Jetson原始指令
	ArbiterOutput_t output;      // 仲裁后输出
	
	u16 nearest_dist;            // 最近障碍物距离（mm）
	u8  obstacle_detected;       // 障碍物检测标志
	
	// 底盘反馈数据
	CanSysStatus_t sys_status;           // 系统状态
	CanMotionFeedback_t motion_fb;       // 运动反馈
	CanWheelAngle_t wheel_angle;         // 四轮转角
	CanWheelSpeed_t wheel_speed;         // 四轮转速
	CanMotionModeFeedback_t motion_mode_fb; // 运动模式反馈
	CanMotorHighInfo_t motor_high[8];    // 8个电机高速信息
	CanMotorLowInfo_t motor_low[8];      // 8个电机低速信息
	CanRemoteCtrl_t remote_ctrl;         // 遥控器信息
	CanOdometer_t odometer;              // 里程计信息
	CanBMSData_t bms_data;               // BMS电池数据
	CanBMSAlarm_t bms_alarm;             // BMS告警状态
	u8 feedback_updated;                 // 反馈数据更新标志
} ArbiterState_t;

// ============================================================================
// 函数接口声明
// ============================================================================

// 初始化仲裁器
void Arbiter_Init(void);

// 更新心跳（收到Jetson数据时调用）
void Arbiter_UpdateHeartbeat(void);

// 接收并解析Jetson指令（返回0=成功，1=失败）
u8 Arbiter_ParseJetsonCmd(u8* frame, u8 len);

// 设置最近障碍物距离（由传感器模块调用）
void Arbiter_SetObstacleDistance(u16 dist_mm);

// 执行仲裁逻辑（主循环中周期调用）
void Arbiter_Process(void);

// 获取仲裁后输出
ArbiterOutput_t* Arbiter_GetOutput(void);

// 获取当前模式
ArbiterMode_t Arbiter_GetMode(void);

// 发送仲裁后指令到STM32 A（通过CAN）
void Arbiter_SendToSTM32A(void);

// 打印调试信息（通过USART1）
void Arbiter_PrintStatus(void);

// 速度限幅（安全防线）
s16 Arbiter_ClampSpeed(s16 speed);

// 发送模式设定指令
void Arbiter_SetMode(CanMode_t mode);

// 发送驱动模式切换指令
void Arbiter_SetDriveMode(CanDriveMode_t mode);

// 发送错误清除指令
void Arbiter_ClearError(CanErrorClear_t error_type);

// 发送运动模型切换指令
void Arbiter_SetMotionMode(CanMotionMode_t mode);

// 发送灯光控制指令
void Arbiter_SetLight(u8 enable, u8 mode);

// 解析系统状态反馈帧
void Arbiter_ParseSysStatus(u8* data, u8 len);

// 解析运动控制反馈帧
void Arbiter_ParseMotionFeedback(u8* data, u8 len);

// 解析四轮转角反馈帧
void Arbiter_ParseWheelAngle(u8* data, u8 len);

// 解析四轮转速反馈帧
void Arbiter_ParseWheelSpeed(u8* data, u8 len);

// 解析运动模式反馈帧
void Arbiter_ParseMotionModeFeedback(u8* data, u8 len);

// 解析电机高速信息反馈帧
void Arbiter_ParseMotorHighInfo(u8 motor_id, u8* data, u8 len);

// 解析电机低速信息反馈帧
void Arbiter_ParseMotorLowInfo(u8 motor_id, u8* data, u8 len);

// 解析遥控器信息反馈帧
void Arbiter_ParseRemoteCtrl(u8* data, u8 len);

// 解析前轮里程反馈帧
void Arbiter_ParseOdometerFront(u8* data, u8 len);

// 解析后轮里程反馈帧
void Arbiter_ParseOdometerRear(u8* data, u8 len);

// 解析BMS数据反馈帧（ID:0x361）
void Arbiter_ParseBMSData(u8* data, u8 len);

// 解析BMS告警状态反馈帧（ID:0x362）
void Arbiter_ParseBMSAlarm(u8* data, u8 len);

// 处理 CAN 反馈数据（在主循环中调用）
void Arbiter_ProcessCANFeedback(void);

// 打印底盘 CAN 反馈到串口
void Arbiter_PrintChassisFeedback(void);

// 底盘运动方向（供 LCD/串口显示）
#define CHASSIS_MOTION_STOP     0
#define CHASSIS_MOTION_LEFT     1
#define CHASSIS_MOTION_RIGHT    2
#define CHASSIS_MOTION_SPIN     3
#define CHASSIS_MOTION_FORWARD  4
#define CHASSIS_MOTION_BACKWARD 5

void Arbiter_GetMotionInfo(u8 *dir, s16 *speed);

// 上电初始化时切换到 CAN 指令模式
void Arbiter_EnableCANMode(void);

// 外部声明仲裁器状态（供 LCD 显示使用）
extern ArbiterState_t arb_state;

#endif
