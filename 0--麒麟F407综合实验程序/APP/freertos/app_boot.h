#ifndef _APP_BOOT_H

#define _APP_BOOT_H



#include "system.h"



/* 1=仅测距：关 ETH/GPS/CAN/Jetson/Arbiter，只跑 TIM4+USART3+LCD */

#define APP_SENSOR_TEST_ONLY  0



#if APP_SENSOR_TEST_ONLY

#undef ETH_LWIP_ENABLE

#define ETH_LWIP_ENABLE  0

#else

/* 1=仲裁模式启动 lwIP + Jetson 以太网；测距独占模式见 APP_SENSOR_TEST_ONLY */

#define ETH_LWIP_ENABLE  1

#endif

/* FreeRTOS 下在 Net 任务轮询收包，避免 ETH IRQ 与 lwip_periodic_handle 重入 */

#define ETH_RX_POLL_ONLY  1



/* 1=LCD 仅清屏显示 "test"，关闭 SensorUI/Chassis 刷新（串口联调） */

#define APP_LCD_MINIMAL_TEST  0



void Hardware_Init(void);

void App_MotionHwInit(void);

void App_ShowBootSplash(void);



#if ETH_LWIP_ENABLE

void App_EthPoll(void);

u8 App_EthIsReady(void);

void App_EthGetStats(u32 *rx_frames, u8 *link_up);

#endif



#endif


