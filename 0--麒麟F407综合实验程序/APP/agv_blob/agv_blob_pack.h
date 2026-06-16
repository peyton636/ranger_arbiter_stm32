#ifndef __AGV_BLOB_PACK_H
#define __AGV_BLOB_PACK_H

#include "agv_blob_wire.h"
#include "agv_blob_rs232.h"
#include "arbiter.h"
#include "gps.h"

#if JETSON_USE_BLOB_V2 && !JETSON_LINK_CAN

void BlobPack_HandleDownlink(const blob_rx_frame_t *frame);
void BlobPack_UplinkTick(const ArbiterState_t *state,
	u16 sonar_f, u16 sonar_b, u16 sonar_l, u16 sonar_r, u16 nearest_mm);
void BlobPack_SendGps(const GPS_Data_t *gps);

u32 BlobPack_GetRxCtrlCount(void);
u32 BlobPack_GetRxOtherCount(void);

#endif

#endif
