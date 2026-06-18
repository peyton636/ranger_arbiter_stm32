#ifndef _APP_BOOT_H
#define _APP_BOOT_H

#include "system.h"

/* 1=仅测距：关 ETH/GPS/CAN/Jetson/Arbiter，只跑 TIM4+USART3+LCD */
#define APP_SENSOR_TEST_ONLY  0

#if APP_SENSOR_TEST_ONLY
#undef ETH_LWIP_ENABLE
#define ETH_LWIP_ENABLE  0
#else
/* 1=仲裁模式启动 lwIP（网线 ping 测试）；0=关闭，不影响 CAN/RS232 */
#define ETH_LWIP_ENABLE  1
#endif
/* FreeRTOS 下在 Net 任务轮询收包，避免 ETH IRQ 与 lwip_periodic_handle 重入 */
#define ETH_RX_POLL_ONLY  1

void App_MotionHwInit(void);
void App_ShowBootSplash(void);

#if ETH_LWIP_ENABLE
void App_EthInit(void);
void App_EthPoll(void);
u8 App_EthIsReady(void);
void App_EthGetStats(u32 *rx_frames, u8 *link_up);
#endif

#endif
