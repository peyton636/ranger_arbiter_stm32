#ifndef __USART3_H
#define __USART3_H

#include "system.h"
#include "arbiter.h"
#include "jetson_can.h"

// USART2 配置（用于连接 Jetson，PA2-TX, PA3-RX）
#define USART3_BAUDRATE    115200

// V3 统一帧长度
#define JETSON_V3_FRAME_LEN 24

#if !JETSON_LINK_CAN
void USART3_Init(void);
void USART3_SendByte(u8 data);
void USART3_SendData(u8* data, u16 len);
void USART3_ProcessRxByte(u8 byte);
u8 USART3_GetJetsonFrame(u8* frame);
#endif

// V3 上行帧发送
void USART3_SendV3StatusFrame(const ArbiterState_t *state, u16 sonar_front, u16 sonar_back,
	u16 sonar_left, u16 sonar_right);
void USART3_SendV3DetailFrame(const ArbiterState_t *state);

#endif
