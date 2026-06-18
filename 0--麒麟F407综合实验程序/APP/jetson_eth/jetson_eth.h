#ifndef __JETSON_ETH_H
#define __JETSON_ETH_H

#include "system.h"
#include "app_boot.h"
#include "agv_blob_rs232.h"

#if ETH_LWIP_ENABLE

#define JETSON_ETH_PORT_DOWN  50001u
#define JETSON_ETH_PORT_UP    50002u

#define JETSON_LINK_ETH       1

void JetsonEth_Init(void);
u8   JetsonEth_Send(u8 msg_id, u8 seq, const u8 *payload, u16 payload_len);
u8   JetsonEth_SendServiceFrame(u32 can_id, const u8 *payload, u8 len);
u8   JetsonEth_GetFrame(blob_rx_frame_t *out);
u8   JetsonEth_GetServiceRequest(u32 *can_id, u8 *payload);
void JetsonEth_GetTxStats(u32 *tx_bytes, u32 *tx_frames, u32 *tx_m02, u32 *tx_m03);
void JetsonEth_GetSvcStats(u32 *svc_rx, u32 *svc_107, u32 *svc_tx_108, u32 *svc_reject);
u32  JetsonEth_GetRxDropCount(void);

#else

#define JETSON_LINK_ETH       0

#endif

#endif
