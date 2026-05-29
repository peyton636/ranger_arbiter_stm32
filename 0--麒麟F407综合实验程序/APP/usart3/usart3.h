#ifndef __USART3_H
#define __USART3_H

#include "system.h"

// USART2 配置（用于连接 Jetson，PA2-TX, PA3-RX）
#define USART3_BAUDRATE    115200

// 数据帧格式定义
#define USART3_FRAME_HEADER 0xFF
#define USART3_FRAME_LEN    12  // 帧头(1) + 传感器数(1) + 距离数据(8) + 蜂鸣器(1) + 校验和(1)

// Jetson 指令帧格式
#define JETSON_FRAME_LEN    8   // 帧头(1) + v(2) + ω(2) + 心跳(1) + 预留(1) + 校验和(1)

void USART3_Init(void);
void USART3_SendByte(u8 data);
void USART3_SendData(u8* data, u16 len);
void USART3_SendSensorData(u16* dist, u8 beep_duty);

// 接收相关函数
void USART3_ProcessRxByte(u8 byte);
u8 USART3_GetJetsonFrame(u8* frame);

#endif
