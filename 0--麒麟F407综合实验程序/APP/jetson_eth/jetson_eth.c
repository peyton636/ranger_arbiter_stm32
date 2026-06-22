#include "jetson_eth.h"



#if ETH_LWIP_ENABLE



#include "lwip_comm.h"

#include "lwip/udp.h"

#include "lwip/pbuf.h"

#include "agv_blob_wire.h"

#include "can2.h"

#include "string.h"

#include "stdio.h"



#define JETSON_ETH_RX_DEPTH  8u



typedef struct

{

	u32 can_id;

	u8  payload[8];

} jetson_eth_svc_t;



static struct udp_pcb *s_udp_pcb = NULL;

static blob_rx_frame_t s_rx_q[JETSON_ETH_RX_DEPTH];

static jetson_eth_svc_t s_svc_q[JETSON_ETH_RX_DEPTH];

static volatile u8 s_rx_w = 0;

static volatile u8 s_rx_r = 0;

static volatile u8 s_rx_cnt = 0;

static volatile u8 s_svc_w = 0;

static volatile u8 s_svc_r = 0;

static volatile u8 s_svc_cnt = 0;

static volatile u32 s_rx_drop = 0;

static volatile u32 s_tx_bytes = 0;

static volatile u32 s_tx_frames = 0;

static volatile u32 s_tx_m02 = 0;

static volatile u32 s_tx_m03 = 0;

static volatile u32 s_svc_rx = 0;

static volatile u32 s_svc_107 = 0;

static volatile u32 s_svc_tx_108 = 0;

static volatile u32 s_svc_reject = 0;

static u8 s_log_first_107 = 0;

static u8 s_log_first_tx108 = 0;



static u16 JetsonEth_ReadU16BE(const u8 *p)

{

	return ((u16)p[0] << 8) | p[1];

}



static u8 JetsonEth_PbufCopy(struct pbuf *p, u8 *dst, u16 len)

{

	u16 n;



	if(!p || !dst)

		return 0;

	n = pbuf_copy_partial(p, dst, len, 0);

	return (n == len) ? 1 : 0;

}



static u8 JetsonEth_HeaderValid(const u8 *hdr, u16 payload_len)

{

	if(hdr[0] != BLOB_HDR_MAGIC)

		return 0;

	if(hdr[1] != BLOB_HDR_VER)

		return 0;

	if(hdr[6] != 0)

		return 0;

	if(hdr[8] != 0)

		return 0;

	if(payload_len == 0 || payload_len > BLOB_MAX_PAYLOAD)

		return 0;

	if(!Blob_ValidatePayloadLen(hdr[2], payload_len))

		return 0;

	return 1;

}



static void JetsonEth_RxPush(const blob_rx_frame_t *frame)

{

	if(s_rx_cnt >= JETSON_ETH_RX_DEPTH)

	{

		s_rx_drop++;

		return;

	}

	s_rx_q[s_rx_w] = *frame;

	s_rx_w = (u8)((s_rx_w + 1u) % JETSON_ETH_RX_DEPTH);

	s_rx_cnt++;

}



static void JetsonEth_SvcPush(u32 can_id, const u8 *payload)

{

	jetson_eth_svc_t item;

	u8 i;



	if(s_svc_cnt >= JETSON_ETH_RX_DEPTH)

	{

		s_rx_drop++;

		return;

	}



	item.can_id = can_id;

	for(i = 0; i < 8; i++)

		item.payload[i] = payload[i];

	s_svc_q[s_svc_w] = item;

	s_svc_w = (u8)((s_svc_w + 1u) % JETSON_ETH_RX_DEPTH);

	s_svc_cnt++;

}



static u8 JetsonEth_HandleBlobRx(const u8 *data, u16 len)

{

	u16 payload_len;

	u16 i;

	blob_rx_frame_t frame;



	if(len < BLOB_HDR_SIZE)

		return 0;



	payload_len = JetsonEth_ReadU16BE(&data[4]);

	if(len < (u16)(BLOB_HDR_SIZE + payload_len))

		return 0;

	if(!JetsonEth_HeaderValid(data, payload_len))

		return 0;



	frame.msg_id = data[2];

	frame.seq = data[3];

	frame.payload_len = payload_len;

	for(i = 0; i < payload_len; i++)

		frame.payload[i] = data[BLOB_HDR_SIZE + i];



	JetsonEth_RxPush(&frame);

	return 1;

}



static u8 JetsonEth_HandleSvcRx(const u8 *data, u16 len)

{

	u32 can_id;

	u8 i;



	if(len != JETSON_RS232_SVC_LEN)

		return 0;

	if(data[0] != JETSON_RS232_SVC_MAGIC)

		return 0;



	can_id = ((u32)data[1] << 8) | data[2];

	s_svc_rx++;

	if(can_id == JETSON_CAN_ID_TIME_REQ)

	{

		s_svc_107++;

		if(!s_log_first_107)

		{

			s_log_first_107 = 1;

			printf("[ETH SVC] first 0x107 cmd=0x%02X\r\n", (unsigned)data[3]);

		}

	}



	JetsonEth_SvcPush(can_id, &data[3]);

	return 1;

}



static void JetsonEth_OnUdpRx(void *arg, struct udp_pcb *pcb, struct pbuf *p,

	struct ip_addr *addr, u16_t port)

{

	u8 pkt[256];

	u16 len;



	(void)arg;

	(void)pcb;

	(void)addr;

	(void)port;



	if(!p)

		return;



	len = p->tot_len;

	if(len == 0 || len > sizeof(pkt))

	{

		s_svc_reject++;

		goto done;

	}

	if(!JetsonEth_PbufCopy(p, pkt, len))

	{

		s_svc_reject++;

		goto done;

	}



	if(pkt[0] == JETSON_RS232_SVC_MAGIC)

	{

		if(!JetsonEth_HandleSvcRx(pkt, len))

			s_svc_reject++;

	}

	else if(pkt[0] == BLOB_HDR_MAGIC)

	{

		if(!JetsonEth_HandleBlobRx(pkt, len))

			s_svc_reject++;

	}

	else

	{

		s_svc_reject++;

	}



done:

	pbuf_free(p);

}



void JetsonEth_Init(void)

{

	err_t err;



	if(s_udp_pcb)

		return;



	s_udp_pcb = udp_new();

	if(!s_udp_pcb)

	{

		printf("[ETH] JetsonEth udp_new failed\r\n");

		return;

	}



	err = udp_bind(s_udp_pcb, IP_ADDR_ANY, JETSON_ETH_PORT_DOWN);

	if(err != ERR_OK)

	{

		printf("[ETH] JetsonEth udp_bind %u failed err=%d\r\n",

			(unsigned)JETSON_ETH_PORT_DOWN, (int)err);

		udp_remove(s_udp_pcb);

		s_udp_pcb = NULL;

		return;

	}



	udp_recv(s_udp_pcb, JetsonEth_OnUdpRx, NULL);

	printf("[ETH] JetsonEth UDP bind :%u (0xAB BLOB + 0xA5 svc), uplink -> %u.%u.%u.%u:%u\r\n",

		(unsigned)JETSON_ETH_PORT_DOWN,

		(unsigned)lwipdev.remoteip[0], (unsigned)lwipdev.remoteip[1],

		(unsigned)lwipdev.remoteip[2], (unsigned)lwipdev.remoteip[3],

		(unsigned)JETSON_ETH_PORT_UP);

}



static u8 JetsonEth_UdpSendRaw(const u8 *wire, u16 wire_len)

{

	struct pbuf *p;

	struct ip_addr dst;

	err_t err;

	u32 primask;



	if(!s_udp_pcb || !wire || wire_len == 0)

		return 0;



	p = pbuf_alloc(PBUF_TRANSPORT, wire_len, PBUF_RAM);

	if(!p)

		return 0;

	memcpy(p->payload, wire, wire_len);



	IP4_ADDR(&dst,

		lwipdev.remoteip[0], lwipdev.remoteip[1],

		lwipdev.remoteip[2], lwipdev.remoteip[3]);



	primask = __get_PRIMASK();

	__disable_irq();

	err = udp_sendto(s_udp_pcb, p, &dst, JETSON_ETH_PORT_UP);

	if(!primask)

		__enable_irq();



	pbuf_free(p);

	return (err == ERR_OK) ? 1 : 0;

}



u8 JetsonEth_Send(u8 msg_id, u8 seq, const u8 *payload, u16 payload_len)

{

	u8 wire[BLOB_MAX_WIRE];

	u16 wire_len;

	u16 i;



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

	for(i = 0; i < payload_len; i++)

		wire[BLOB_HDR_SIZE + i] = payload[i];



	if(!JetsonEth_UdpSendRaw(wire, wire_len))

		return 0;



	s_tx_bytes += wire_len;

	s_tx_frames++;

	if(msg_id == BLOB_MSG_MOTION)

		s_tx_m02++;

	else if(msg_id == BLOB_MSG_MCU_STATUS)

		s_tx_m03++;

	return 1;

}



u8 JetsonEth_SendServiceFrame(u32 can_id, const u8 *payload, u8 len)

{

	u8 frame[JETSON_RS232_SVC_LEN];

	u8 i;

	u8 copy_len;



	if(!payload)

		return 0;

	if(len > JETSON_CAN_V3_FRAG_LEN)

		len = JETSON_CAN_V3_FRAG_LEN;



	frame[0] = JETSON_RS232_SVC_MAGIC;

	frame[1] = (u8)((can_id >> 8) & 0xFF);

	frame[2] = (u8)(can_id & 0xFF);

	copy_len = len;

	for(i = 0; i < JETSON_CAN_V3_FRAG_LEN; i++)

		frame[3 + i] = (i < copy_len) ? payload[i] : 0;



	if(!JetsonEth_UdpSendRaw(frame, JETSON_RS232_SVC_LEN))

		return 0;



	s_tx_bytes += JETSON_RS232_SVC_LEN;

	s_tx_frames++;

	if(can_id == JETSON_CAN_ID_TIME_RSP)

	{

		s_svc_tx_108++;

		if(!s_log_first_tx108)

		{

			s_log_first_tx108 = 1;

			printf("[ETH SVC] tx 0x108 cmd=0x%02X\r\n", (unsigned)payload[0]);

		}

	}

	return 1;

}



u8 JetsonEth_GetFrame(blob_rx_frame_t *out)

{

	u32 primask;

	u16 i;



	if(!out)

		return 0;



	primask = __get_PRIMASK();

	__disable_irq();

	if(s_rx_cnt == 0)

	{

		if(!primask)

			__enable_irq();

		return 0;

	}



	out->msg_id = s_rx_q[s_rx_r].msg_id;

	out->seq = s_rx_q[s_rx_r].seq;

	out->payload_len = s_rx_q[s_rx_r].payload_len;

	for(i = 0; i < out->payload_len; i++)

		out->payload[i] = s_rx_q[s_rx_r].payload[i];

	s_rx_r = (u8)((s_rx_r + 1u) % JETSON_ETH_RX_DEPTH);

	s_rx_cnt--;

	if(!primask)

		__enable_irq();

	return 1;

}



u8 JetsonEth_GetServiceRequest(u32 *can_id, u8 *payload)

{

	u32 primask;

	u8 i;



	if(!can_id || !payload)

		return 0;



	primask = __get_PRIMASK();

	__disable_irq();

	if(s_svc_cnt == 0)

	{

		if(!primask)

			__enable_irq();

		return 0;

	}



	*can_id = s_svc_q[s_svc_r].can_id;

	for(i = 0; i < 8; i++)

		payload[i] = s_svc_q[s_svc_r].payload[i];

	s_svc_r = (u8)((s_svc_r + 1u) % JETSON_ETH_RX_DEPTH);

	s_svc_cnt--;

	if(!primask)

		__enable_irq();

	return 1;

}



void JetsonEth_GetTxStats(u32 *tx_bytes, u32 *tx_frames, u32 *tx_m02, u32 *tx_m03)

{

	if(tx_bytes)

		*tx_bytes = s_tx_bytes;

	if(tx_frames)

		*tx_frames = s_tx_frames;

	if(tx_m02)

		*tx_m02 = s_tx_m02;

	if(tx_m03)

		*tx_m03 = s_tx_m03;

}



void JetsonEth_GetSvcStats(u32 *svc_rx, u32 *svc_107, u32 *svc_tx_108, u32 *svc_reject)

{

	if(svc_rx)

		*svc_rx = s_svc_rx;

	if(svc_107)

		*svc_107 = s_svc_107;

	if(svc_tx_108)

		*svc_tx_108 = s_svc_tx_108;

	if(svc_reject)

		*svc_reject = s_svc_reject;

}



u32 JetsonEth_GetRxDropCount(void)

{

	return s_rx_drop;

}



#endif

