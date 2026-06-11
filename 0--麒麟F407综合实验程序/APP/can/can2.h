#ifndef _can2_H
#define _can2_H

#include "system.h"

/* Jetson ??? CAN2??PB5=RX, PB6=TX?????????????? Jetson USB-CAN
 * ????? CAN1(PA11/PA12) ???????????????? 500 kbps */

#define JETSON_CAN_BAUDRATE  500000

/* Jetson V3 over CAN ?? ???? 11-bit ID */
#define JETSON_CAN_ID_DOWN     0x101u  /* Jetson??STM32??V3 type 0x01??3??8B */
#define JETSON_CAN_ID_STATUS   0x102u  /* STM32??Jetson??V3 type 0x02 */
#define JETSON_CAN_ID_DETAIL   0x103u  /* STM32??Jetson??V3 type 0x03 */
#define JETSON_CAN_ID_GPS      0x104u
#define JETSON_CAN_ID_GPS_B    0x105u
#define JETSON_CAN_ID_GPS_C    0x106u

#define JETSON_CAN_ID_TIME_REQ     0x107u
#define JETSON_CAN_ID_TIME_RSP     0x108u
#define JETSON_CAN_ID_FAULT        0x109u
#define JETSON_CAN_ID_STATUS_REQ   0x10Au
#define JETSON_CAN_ID_STATUS_RSP   0x10Bu

#define JETSON_CAN_CMD_TIME_SYNC   0x01u
#define JETSON_CAN_CMD_STATUS_QRY  0x01u

#define JETSON_CAN_GPS_MAGIC       0xA4u
#define JETSON_CAN_GPS_FRAG_GAP_MS 2u

#define JETSON_CAN_V3_FRAG_CNT 3u
#define JETSON_CAN_V3_FRAG_LEN 8u

void CAN2_Mode_Init(u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode);
void CAN2_Init_Jetson(void);

u8 CAN2_Send_Msg_WithID(u32 id, u8 *msg, u8 len);
u8 CAN2_Receive_Msg_WithID(u32 *id, u8 *buf);

#endif
