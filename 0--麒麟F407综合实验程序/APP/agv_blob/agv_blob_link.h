#ifndef __AGV_BLOB_LINK_H
#define __AGV_BLOB_LINK_H

#include "agv_blob_rs232.h"
#include "app_boot.h"
#if ETH_LWIP_ENABLE
#include "jetson_eth.h"
#endif

#if JETSON_LINK_ETH
#define BlobLink_Send       JetsonEth_Send
#define BlobLink_GetFrame   JetsonEth_GetFrame
#define BlobLink_GetTxStats JetsonEth_GetTxStats
#else
#define BlobLink_Send       BlobRs232_Send
#define BlobLink_GetFrame   BlobRs232_GetFrame
#define BlobLink_GetTxStats BlobRs232_GetTxStats
#endif

#endif
