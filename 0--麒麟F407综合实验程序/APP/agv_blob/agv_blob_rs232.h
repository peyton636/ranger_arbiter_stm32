#ifndef __AGV_BLOB_RS232_H
#define __AGV_BLOB_RS232_H

#include "agv_blob_wire.h"

typedef struct
{
	u8  msg_id;
	u8  seq;
	u16 payload_len;
	u8  payload[BLOB_MAX_PAYLOAD];
} blob_rx_frame_t;

#if !JETSON_LINK_CAN

u8 BlobRs232_Send(u8 msg_id, u8 seq, const u8 *payload, u16 payload_len);
u8 BlobRs232_GetFrame(blob_rx_frame_t *out);
void BlobRs232_RxReset(void);
/* 1=??????????????????????? BLOB ?????? */
u8 BlobRs232_RxFeed(u8 byte);

u32 BlobRs232_GetHdrRejectCount(void);
void BlobRs232_GetTxStats(u32 *tx_bytes, u32 *tx_frames, u32 *tx_m02, u32 *tx_m03);

#endif

#endif
