#ifndef __RS232_H
#define __RS232_H

#include "system.h"
#include "arbiter.h"
#include "jetson_can.h"

/* 硬件 USART2：PA2-TX / PA3-RX，Jetson RS232 */
#define RS232_BAUDRATE         115200

#define JETSON_V3_FRAME_LEN    24

#if !JETSON_LINK_CAN
void RS232_Init(void);
void RS232_TxMutexInit(void);
void RS232_SendByte(u8 data);
void RS232_SendData(u8 *data, u16 len);
void RS232_SendServiceFrame(u32 can_id, const u8 *payload, u8 len);
void RS232_ProcessRxByte(u8 byte);
u8 RS232_GetJetsonFrame(u8 *frame);
u8 RS232_GetServiceRequest(u32 *can_id, u8 *payload);
void RS232_GetRxStats(u32 *rx_bytes, u32 *ore_cnt, u32 *ab_idle);
void RS232_GetRxMagicStats(u32 *raw_ab, u32 *raw_a5, u32 *raw_aa);
void RS232_PrintHwDiag(void);
#endif

void RS232_SendV3StatusFrame(const ArbiterState_t *state, u16 sonar_front, u16 sonar_back,
	u16 sonar_left, u16 sonar_right);
void RS232_SendV3DetailFrame(const ArbiterState_t *state);

#endif
