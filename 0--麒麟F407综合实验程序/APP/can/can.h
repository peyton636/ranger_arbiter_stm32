#ifndef _can_H
#define _can_H

#include "system.h"

// RANGER MINI 3.0 CAN通信配置
#define CAN_BAUDRATE 500000  // 500kbps

// CAN ID定义 - RANGER MINI 3.0
#define CAN_ID_CTRL_CMD     0x111  // STM32→底盘：运动控制指令
#define CAN_ID_MODE_SET     0x421  // STM32→底盘：控制模式设定
#define CAN_ID_ERR_CLEAR    0x441  // STM32→底盘：错误清除指令
#define CAN_ID_SYS_STATUS   0x211  // 底盘→STM32：系统状态反馈
#define CAN_ID_MOTOR_STATUS 0x221  // 底盘→STM32：运动控制反馈
#define CAN_ID_WHEEL_ANGLE  0x271  // 底盘→STM32：四轮转角反馈
#define CAN_ID_WHEEL_SPEED  0x281  // 底盘→STM32：四轮转速反馈

// CAN初始化函数
void CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);

// 发送指定ID的CAN消息
u8 CAN1_Send_Msg_WithID(u32 id, u8* msg, u8 len);

// 发送默认ID的CAN消息（保持兼容）
u8 CAN1_Send_Msg(u8* msg, u8 len);

// 接收CAN消息（带ID返回）
u8 CAN1_Receive_Msg_WithID(u32 *id, u8 *buf);

// 接收CAN消息（保持兼容）
u8 CAN1_Receive_Msg(u8 *buf);

// RANGER MINI 3.0 专用函数
void CAN1_Init_RangerMini(void);  // 初始化适配RANGER MINI的CAN配置
u8 CAN1_Send_ControlCmd(u16 speed_mm_s, float angle_rad);  // 发送运动控制指令
u8 CAN1_Send_ModeSet(u8 mode);  // 发送控制模式设定
u8 CAN1_Send_ErrorClear(void);  // 发送错误清除指令

#endif
