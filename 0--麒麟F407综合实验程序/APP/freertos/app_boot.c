#include "app_boot.h"
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
#include "usart.h"
#include "agv_blob_wire.h"
#if ETH_LWIP_ENABLE
#include "lwip_comm.h"
#include "lan8720.h"
#include "jetson_eth.h"
#endif

#if ETH_LWIP_ENABLE
#ifndef ETH_RX_POLL_ONLY
#define ETH_RX_POLL_ONLY 0
#endif
static u8 s_eth_ready = 0;
static volatile u32 s_eth_rx_frames = 0;

static void Hardware_EthInit(void)
{
	u8 i;
	u8 ret;

	TIM3_Init(999, 839);
	for(i = 0; i < 5; i++)
	{
		ret = lwip_comm_init();
		if(ret == 0)
		{
			s_eth_ready = 1;
			ETH_RecoverRxDma();
			ETH->DMARPDR = 0;
			lwip_comm_gratuitous_arp();
			JetsonEth_Init();
			return;
		}
		delay_ms(200);
	}
	s_eth_ready = 0;
}

void App_EthPoll(void)
{
	u32 primask;

	if(!s_eth_ready)
		return;

#if ETH_RX_POLL_ONLY
	while(ETH_GetRxPktSize(DMARxDescToGet) != 0)
	{
		primask = __get_PRIMASK();
		__disable_irq();
		lwip_pkt_handle();
		if(!primask)
			__enable_irq();
		s_eth_rx_frames++;
	}
#endif
	primask = __get_PRIMASK();
	__disable_irq();
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

/*
 * 运动/通信外设初始化（在 Hardware_Check 基础外设 + TIM4 之后调用）
 *
 * 1. GPS（USART6 9600）
 * 2. 距离传感器（硬件 USART3）
 * 3. 底盘 CAN 收发（Arv_TranReceive_Init）
 * 4. 仲裁器 + 使能 CAN 发送
 * 5. Jetson 协议层状态复位（无 RS232/CAN2 物理链路）
 * 6. 以太网 + JetsonEth（固定 ETH，PA2=MDIO 不复用 RS232）
 */
void Hardware_Init(void)
{
	TIM_Cmd(TIM2, DISABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);

#if APP_SENSOR_TEST_ONLY
	DistanceSensor_Init();
	return;
#endif

	Usart_PrintMutexInit();

	GPS_USART6_Init(9600);
	DistanceSensor_Init();
	Arv_TranReceive_Init();
	Arbiter_Init();
	Arbiter_EnableCANMode();
	Protocol_Init();

#if ETH_LWIP_ENABLE
	Hardware_EthInit();
	USART1_Init(115200);
#endif
}

void App_MotionHwInit(void)
{
	USART1_Probe("MOTION");

#if APP_SENSOR_TEST_ONLY
	printf("[BOOT] SENSOR_TEST_ONLY: ETH/GPS/CAN/Jetson/Arbiter OFF\r\n");
	printf("[BOOT] TIM4 20ms + hardware USART3 PB10(trig)/PB11(RX) 9600\r\n");
	printf("[BOOT] Wait [DS TEST] every 2s: proc_tick should grow, uart_rx>0 if module OK\r\n");
	return;
#endif

#if ETH_LWIP_ENABLE
	printf("[JETSON] Ethernet UDP BLOB v2 down:%u up_peer:%u (0xAB)\r\n",
		(unsigned)JETSON_ETH_PORT_DOWN, (unsigned)JETSON_ETH_PORT_UP);
#if BLOB_UPLINK_MINIMAL
	printf("[JETSON] uplink profile=MINIMAL (0x02/0x03@50Hz, others reduced)\r\n");
#else
	printf("[JETSON] uplink profile=FULL\r\n");
#endif
	printf("[JETSON] downlink: 0xAB MSG 0x01; log tag [JETSON BLOB CMD]\r\n");
#endif

	printf("[CAN1] Chassis PA11/12 init OK, MCR=0x%08X MSR=0x%08X\r\n",
		(unsigned int)CAN1->MCR, (unsigned int)CAN1->MSR);
#if ARBITER_IGNORE_DIST_SENSOR
	printf("[MOTION] Dist sensor IGNORED, Jetson passthrough test, beep=OFF\r\n");
#else
	printf("[MOTION] Dist ctrl ON, wait Jetson V3, beep=%s (KEY1 toggle)\r\n",
		g_beep_dist_enable ? "ON" : "OFF");
#endif

	printf("[GPS] hardware USART6 PC6(TX)/PC7(RX), auto cfg GPS+BDS 5Hz RMC/GGA/GSA\r\n");
	printf("[DS] hardware USART3 PB10(trig)/PB11(RX) 9600, TIM4 20ms trigger\r\n");

#if ETH_LWIP_ENABLE
	if(App_EthIsReady())
	{
		printf("[ETH] lwIP OK  MCU IP %u.%u.%u.%u  peer %u.%u.%u.%u\r\n",
			(unsigned)lwipdev.ip[0], (unsigned)lwipdev.ip[1],
			(unsigned)lwipdev.ip[2], (unsigned)lwipdev.ip[3],
			(unsigned)lwipdev.remoteip[0], (unsigned)lwipdev.remoteip[1],
			(unsigned)lwipdev.remoteip[2], (unsigned)lwipdev.remoteip[3]);
		printf("[ETH] mem1=%u%% mem2=%u%%\r\n",
			(unsigned)my_mem_perused(SRAMIN), (unsigned)my_mem_perused(SRAMEX));
		if(LAN8720_LinkUp())
			printf("[ETH] PHY link UP, speed code=%u (6/14=100M FD)\r\n",
				(unsigned)LAN8720_Get_Speed());
		else
			printf("[ETH] PHY link DOWN - no cable or wrong port\r\n");
		printf("[ETH] MAC %02X:%02X:%02X:%02X:%02X:%02X\r\n",
			lwipdev.mac[0], lwipdev.mac[1], lwipdev.mac[2],
			lwipdev.mac[3], lwipdev.mac[4], lwipdev.mac[5]);
#if ETH_RX_POLL_ONLY
		printf("[ETH] RX poll mode: NetTask drains DMA (no ETH IRQ)\r\n");
#endif
		printf("[ETH] gratuitous ARP sent, tx_ok=%lu\r\n",
			(unsigned long)ETH_TxOkCount());
		USART1_Probe("ETHOK");
	}
	else
	{
		printf("[ETH] lwIP init failed; CAN/motion unaffected\r\n");
	}
#endif
}

void App_ShowBootSplash(void)
{
	LCD_Clear(BLACK);
	FRONT_COLOR = WHITE;
	BACK_COLOR = BLACK;
#if APP_LCD_MINIMAL_TEST
	LCD_ShowString(10, 120, tftlcd_data.width, tftlcd_data.height, 24, "test");
	USART1_Probe("LCD");
	printf("[LCD] MINIMAL test screen (SensorUI off)\r\n");
#else
	SensorUI_DrawStatic();
	SensorUI_UpdateDistances(DistanceSensor_GetData());
	SensorUI_UpdateBeepStatus();
	SensorUI_UpdateGps();
	printf("[LCD] Sensor UI ready; layout=pre-RTOS + GPS\r\n");
#endif
}
