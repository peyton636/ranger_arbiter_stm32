#include "app_boot.h"
#include "usart3.h"
#include "jetson_can.h"
#include "gps.h"
#include "distance_sensor.h"
#include "can.h"
#include "arbiter.h"
#include "motion_ui_shared.h"
#include "sensor_ui.h"
#include "tftlcd.h"
#include "time.h"
#include "SysTick.h"
#include "malloc.h"
#include "stdio.h"
#include "agv_blob_wire.h"
#if ETH_LWIP_ENABLE
#include "lwip_comm.h"
#include "lan8720.h"
#endif

#if ETH_LWIP_ENABLE
#ifndef ETH_RX_POLL_ONLY
#define ETH_RX_POLL_ONLY 0
#endif
static u8 s_eth_ready = 0;
static volatile u32 s_eth_rx_frames = 0;

void App_EthInit(void)
{
	u8 i;
	u8 ret;

	/* TIM3 10ms 节拍，供 lwip_localtime / ARP / TCP 定时器 */
	TIM3_Init(999, 839);

	printf("[ETH] Init lwIP (plug cable before power-on if possible)\r\n");
	printf("[ETH] mem1 used %u%% before lwIP\r\n", (unsigned)my_mem_perused(SRAMIN));
	for(i = 0; i < 5; i++)
	{
		ret = lwip_comm_init();
		if(ret == 0)
		{
			s_eth_ready = 1;
			printf("[ETH] lwIP OK  MCU IP %u.%u.%u.%u  mask 255.255.255.0  gw 192.168.10.200\r\n",
				(unsigned)lwipdev.ip[0], (unsigned)lwipdev.ip[1],
				(unsigned)lwipdev.ip[2], (unsigned)lwipdev.ip[3]);
			printf("[ETH] mem1=%u%% mem2=%u%% after lwIP\r\n",
				(unsigned)my_mem_perused(SRAMIN), (unsigned)my_mem_perused(SRAMEX));
			if(LAN8720_LinkUp())
			{
				u8 spd = LAN8720_Get_Speed();
				printf("[ETH] PHY link UP, speed code=%u (6/14=100M FD)\r\n", (unsigned)spd);
			}
			else
			{
				printf("[ETH] PHY link DOWN - no cable or wrong port! ping will fail\r\n");
			}
			printf("[ETH] MAC %02X:%02X:%02X:%02X:%02X:%02X\r\n",
				lwipdev.mac[0], lwipdev.mac[1], lwipdev.mac[2],
				lwipdev.mac[3], lwipdev.mac[4], lwipdev.mac[5]);
			printf("[ETH] Peer(Jetson eno1) %u.%u.%u.%u, test: ping -I %u.%u.%u.%u %u.%u.%u.%u\r\n",
				(unsigned)lwipdev.remoteip[0], (unsigned)lwipdev.remoteip[1],
				(unsigned)lwipdev.remoteip[2], (unsigned)lwipdev.remoteip[3],
				(unsigned)lwipdev.remoteip[0], (unsigned)lwipdev.remoteip[1],
				(unsigned)lwipdev.remoteip[2], (unsigned)lwipdev.remoteip[3],
				(unsigned)lwipdev.ip[0], (unsigned)lwipdev.ip[1],
				(unsigned)lwipdev.ip[2], (unsigned)lwipdev.ip[3]);
#if ETH_RX_POLL_ONLY
			printf("[ETH] RX poll mode: NetTask drains DMA (no ETH IRQ)\r\n");
#endif
			ETH_RecoverRxDma();
			ETH->DMARPDR = 0;
			lwip_comm_gratuitous_arp();
			printf("[ETH] gratuitous ARP sent, tx_ok=%lu\r\n",
				(unsigned long)ETH_TxOkCount());
			return;
		}
		printf("[ETH] lwIP init failed code=%u retry %u/5 (1=mem 2=PHY 3=netif)\r\n",
			(unsigned)ret, (unsigned)(i + 1));
		delay_ms(200);
	}
	printf("[ETH] lwIP disabled; CAN/RS232/motion unaffected\r\n");
}

void App_EthPoll(void)
{
	u32 primask;

	if(!s_eth_ready)
		return;

	primask = __get_PRIMASK();
	__disable_irq();
#if ETH_RX_POLL_ONLY
	while(ETH_GetRxPktSize(DMARxDescToGet) != 0)
	{
		lwip_pkt_handle();
		s_eth_rx_frames++;
	}
#endif
	ETH_RecoverRxDma();
	lwip_periodic_handle();
	if(!primask)
		__enable_irq();
}

u8 App_EthIsReady(void)
{
	return s_eth_ready;
}

void App_EthGetStats(u32 *rx_frames, u8 *link_up)
{
	if(rx_frames)
		*rx_frames = s_eth_rx_frames;
	if(link_up)
		*link_up = LAN8720_LinkUp();
}
#endif

void App_MotionHwInit(void)
{
	/* PRECHIN 在 Hardware_Check 里开了 TIM2(100ms) 拍照中断，ISR 内 KEY_Scan+delay_ms 与 KeyTask 冲突 */
	TIM_Cmd(TIM2, DISABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
#if JETSON_LINK_CAN
	JetsonCAN_Init();
#else
	USART3_Init();
	USART3_PrintHwDiag();
	JetsonCAN_Init();
#endif
	GPS_USART6_Init(9600);
	DistanceSensor_Init();
	CAN1_Init_RangerMini();
	Arbiter_Init();
	Arbiter_EnableCANMode();

#if JETSON_LINK_CAN
	printf("[JETSON] CAN2 PB5(RX)/PB6(TX), 500kbps, V3 24B as 3x8B frames\r\n");
	printf("[JETSON] CAN IDs: down=0x101 status=0x102 detail=0x103 gps=0x104~0x106\r\n");
	printf("[JETSON] Wire PB6->TJA1050 TXD, PB5<-RXD, CANH/L to Jetson USB-CAN\r\n");
#else
#if JETSON_USE_BLOB_V2
	printf("[JETSON] USART2 PA2/PA3, 115200, BLOB v2 (0xAB) down/up + 0xA5 svc\r\n");
#else
	printf("[JETSON] USART2 PA2/PA3, 115200, V3 24-byte (0xAA) frame\r\n");
#endif
#endif
	printf("[CAN1] Chassis PA11/12 init OK, MCR=0x%08X MSR=0x%08X\r\n",
		(unsigned int)CAN1->MCR, (unsigned int)CAN1->MSR);
#if ARBITER_IGNORE_DIST_SENSOR
	printf("[MOTION] Dist sensor IGNORED, Jetson passthrough test, beep=OFF\r\n");
#else
	printf("[MOTION] Dist ctrl ON, wait Jetson V3, beep=%s (KEY1 toggle)\r\n",
		g_beep_dist_enable ? "ON" : "OFF");
#endif
#if !JETSON_LINK_CAN
#if JETSON_USE_BLOB_V2
	printf("[JETSON] downlink: 0xAB MSG 0x01; uplink: 0x02/0x03/0x04...; log tag [JETSON BLOB CMD]\r\n");
#else
	printf("[JETSON] RS232 also sends GPS/0x108/0x109/0x10B via 0xA5 service frames\r\n");
#endif
#endif
	printf("[GPS] USART6 RX ready on PC7 (TX=PC6), auto cfg: GPS+BDS 5Hz RMC/GGA/GSA\r\n");
#if ETH_LWIP_ENABLE
	/* 必须放在 USART3_Init 之后：PA2 与 ETH_MDIO 复用，以太网 init 会最后占住 PA2 */
	App_EthInit();
#if !JETSON_LINK_CAN
	printf("[ETH] PA2=ETH_MDIO after init; Jetson RS232 TX(PA2) disabled while ETH on\r\n");
#endif
#endif
}

void App_ShowBootSplash(void)
{
	LCD_Clear(BLACK);
	FRONT_COLOR = WHITE;
	BACK_COLOR = BLACK;
	SensorUI_DrawStatic();
	SensorUI_UpdateDistances(DistanceSensor_GetData());
	SensorUI_UpdateBeepStatus();
	SensorUI_UpdateGps();
	printf("[LCD] Sensor UI ready; layout=pre-RTOS + GPS\r\n");
}
