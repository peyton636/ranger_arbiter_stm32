#include "agv_blob_rs232.h"
#include "usart3.h"

#if !JETSON_LINK_CAN

typedef enum
{
	BLOB_RX_COLLECT_HDR = 0,
	BLOB_RX_COLLECT_PAYLOAD
} blob_rx_phase_t;

static blob_rx_phase_t s_rx_phase = BLOB_RX_COLLECT_HDR;
static u8  s_rx_buf[BLOB_MAX_WIRE];
static u16 s_rx_idx = 0;
static u16 s_rx_payload_len = 0;
static u8  s_rx_ready = 0;
static blob_rx_frame_t s_rx_complete;
static volatile u32 s_hdr_reject = 0;
static volatile u32 s_tx_bytes = 0;
static volatile u32 s_tx_frames = 0;
static volatile u32 s_tx_m02 = 0;
static volatile u32 s_tx_m03 = 0;

static u16 BlobRs232_ReadU16BE(const u8 *p)
{
	return ((u16)p[0] << 8) | p[1];
}

static void BlobRs232_CommitFrame(void)
{
	u16 i;

	s_rx_complete.msg_id = s_rx_buf[2];
	s_rx_complete.seq = s_rx_buf[3];
	s_rx_complete.payload_len = s_rx_payload_len;
	for(i = 0; i < s_rx_payload_len; i++)
		s_rx_complete.payload[i] = s_rx_buf[BLOB_HDR_SIZE + i];
	s_rx_ready = 1;
}

static u8 BlobRs232_HeaderValid(void)
{
	if(s_rx_buf[0] != BLOB_HDR_MAGIC)
		return 0;
	if(s_rx_buf[1] != BLOB_HDR_VER)
		return 0;
	if(s_rx_buf[6] != 0)
		return 0;
	if(s_rx_buf[8] != 0)
		return 0;
	if(s_rx_payload_len == 0 || s_rx_payload_len > BLOB_MAX_PAYLOAD)
		return 0;
	if(!Blob_ValidatePayloadLen(s_rx_buf[2], s_rx_payload_len))
		return 0;
	return 1;
}

void BlobRs232_RxReset(void)
{
	s_rx_phase = BLOB_RX_COLLECT_HDR;
	s_rx_idx = 0;
	s_rx_payload_len = 0;
}

static u8 BlobRs232_RxResync(u8 byte)
{
	BlobRs232_RxReset();
	s_rx_buf[0] = byte;
	s_rx_idx = 1;
	return 0;
}

u8 BlobRs232_RxFeed(u8 byte)
{
	if(s_rx_phase == BLOB_RX_COLLECT_HDR)
	{
		if(byte == BLOB_HDR_MAGIC && s_rx_idx > 0)
			return BlobRs232_RxResync(byte);

		s_rx_buf[s_rx_idx++] = byte;
		if(s_rx_idx < BLOB_HDR_SIZE)
			return 0;

		s_rx_payload_len = BlobRs232_ReadU16BE(&s_rx_buf[4]);
		if(!BlobRs232_HeaderValid())
		{
			s_hdr_reject++;
			if(s_hdr_reject <= 5u || (s_hdr_reject % 50u) == 0u)
			{
				printf("[BLOB RX] hdr reject #%lu id=%02X len=%u fidx=%u fcnt=%u flg=%02X\r\n",
					(unsigned long)s_hdr_reject,
					(unsigned)s_rx_buf[2], (unsigned)s_rx_payload_len,
					(unsigned)s_rx_buf[6], (unsigned)s_rx_buf[7],
					(unsigned)s_rx_buf[8]);
			}
			BlobRs232_RxReset();
			return 1;
		}

		if(s_rx_payload_len == 0)
		{
			BlobRs232_CommitFrame();
			BlobRs232_RxReset();
			return 1;
		}

		s_rx_phase = BLOB_RX_COLLECT_PAYLOAD;
		return 0;
	}

	if(byte == BLOB_HDR_MAGIC)
		return BlobRs232_RxResync(byte);

	s_rx_buf[s_rx_idx++] = byte;
	if(s_rx_idx >= BLOB_MAX_WIRE)
	{
		s_hdr_reject++;
		BlobRs232_RxReset();
		return 1;
	}
	if(s_rx_idx >= (u16)(BLOB_HDR_SIZE + s_rx_payload_len))
	{
		BlobRs232_CommitFrame();
		BlobRs232_RxReset();
		return 1;
	}
	return 0;
}

u8 BlobRs232_Send(u8 msg_id, u8 seq, const u8 *payload, u16 payload_len)
{
	u8 wire[BLOB_MAX_WIRE];
	u16 wire_len;

	if(!payload || payload_len == 0)
		return 0;
	if(payload_len > BLOB_MAX_PAYLOAD)
		return 0;
	if(!Blob_ValidatePayloadLen(msg_id, payload_len))
		return 0;

	wire_len = (u16)(BLOB_HDR_SIZE + payload_len);
	wire[0] = BLOB_HDR_MAGIC;
	wire[1] = BLOB_HDR_VER;
	wire[2] = msg_id;
	wire[3] = seq;
	wire[4] = (u8)((payload_len >> 8) & 0xFF);
	wire[5] = (u8)(payload_len & 0xFF);
	wire[6] = 0;
	wire[7] = 1;
	wire[8] = 0;

	{
		u16 i;
		for(i = 0; i < payload_len; i++)
			wire[BLOB_HDR_SIZE + i] = payload[i];
	}

	USART3_SendData(wire, wire_len);
	s_tx_bytes += wire_len;
	s_tx_frames++;
	if(msg_id == BLOB_MSG_MOTION)
		s_tx_m02++;
	else if(msg_id == BLOB_MSG_MCU_STATUS)
		s_tx_m03++;
	return 1;
}

void BlobRs232_GetTxStats(u32 *tx_bytes, u32 *tx_frames, u32 *tx_m02, u32 *tx_m03)
{
	if(tx_bytes)  *tx_bytes  = s_tx_bytes;
	if(tx_frames) *tx_frames = s_tx_frames;
	if(tx_m02)    *tx_m02    = s_tx_m02;
	if(tx_m03)    *tx_m03    = s_tx_m03;
}

u8 BlobRs232_GetFrame(blob_rx_frame_t *out)
{
	u16 i;

	if(!out || !s_rx_ready)
		return 0;

	out->msg_id = s_rx_complete.msg_id;
	out->seq = s_rx_complete.seq;
	out->payload_len = s_rx_complete.payload_len;
	for(i = 0; i < s_rx_complete.payload_len; i++)
		out->payload[i] = s_rx_complete.payload[i];

	s_rx_ready = 0;
	return 1;
}

u32 BlobRs232_GetHdrRejectCount(void)
{
	return s_hdr_reject;
}

#endif
