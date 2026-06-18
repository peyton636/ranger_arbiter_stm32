#ifndef __AGV_BLOB_WIRE_H
#define __AGV_BLOB_WIRE_H

#include "system.h"

/* 1=Jetson RS232 �� BLOB v2��0=���� V3(0xAA) Ӧ��֡ */
#ifndef JETSON_USE_BLOB_V2
#define JETSON_USE_BLOB_V2  1
#endif

/* 1=联调档位：只高频发 0x02/0x03，其余帧降频，减轻 115200 混流 */
#ifndef BLOB_UPLINK_MINIMAL
#define BLOB_UPLINK_MINIMAL  1
#endif

/* DMA TX：非阻塞发送（推荐开） */
#ifndef JETSON_USART2_DMA_TX
#define JETSON_USART2_DMA_TX  1
#endif
/* DMA+IDLE RX：联调中出现 ore 风暴且 dn=0 时暂关，改回字节中断 RX */
#ifndef JETSON_USART2_DMA_RX
#define JETSON_USART2_DMA_RX  0
#endif
#if !defined(JETSON_USART2_DMA)
#define JETSON_USART2_DMA     (JETSON_USART2_DMA_TX || JETSON_USART2_DMA_RX)
#endif

#define BLOB_HDR_MAGIC        0xABu
#define BLOB_HDR_VER          0x01u
#define BLOB_HDR_SIZE         9u
#define BLOB_MAX_PAYLOAD      48u
#define BLOB_MAX_WIRE         (BLOB_HDR_SIZE + BLOB_MAX_PAYLOAD)

#define BLOB_MSG_CONTROL      0x01u
#define BLOB_MSG_MOTION       0x02u
#define BLOB_MSG_MCU_STATUS   0x03u
#define BLOB_MSG_SENSOR       0x04u
#define BLOB_MSG_GPS          0x05u
#define BLOB_MSG_MOTOR04      0x06u
#define BLOB_MSG_MOTOR58      0x07u
#define BLOB_MSG_ENERGY       0x08u
#define BLOB_MSG_MOTOR_POS    0x0Bu
#define BLOB_MSG_SENSOR_CFG   0x10u

#define BLOB_PAYLOAD_CONTROL      14u
#define BLOB_PAYLOAD_MOTION       40u
#define BLOB_PAYLOAD_MCU_STATUS   42u
#define BLOB_PAYLOAD_SENSOR       28u
#define BLOB_PAYLOAD_GPS          32u
#define BLOB_PAYLOAD_MOTOR04      44u
#define BLOB_PAYLOAD_MOTOR58      44u
#define BLOB_PAYLOAD_ENERGY       41u
#define BLOB_PAYLOAD_MOTOR_POS    36u
#define BLOB_PAYLOAD_SENSOR_CFG   8u

u16 Blob_PayloadSize(u8 msg_id);
u8  Blob_ValidatePayloadLen(u8 msg_id, u16 len);

#endif
