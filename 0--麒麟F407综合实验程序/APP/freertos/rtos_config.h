#ifndef __RTOS_CONFIG_H
#define __RTOS_CONFIG_H

#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"

#define RTOS_DEBUG_ENABLE    1

/* 串口调试 verbosity：Jetson BLOB L3/L4 联调时建议仅开 JETSON_LINK */
#ifndef RTOS_VERBOSE_CHASSIS_LOG
#define RTOS_VERBOSE_CHASSIS_LOG   0   /* [WHEEL][CMDOUT][MOTION][PWR][SAFE] */
#endif
#ifndef RTOS_VERBOSE_SENSOR_LOG
#define RTOS_VERBOSE_SENSOR_LOG    0   /* [DS] IF1~4 */
#endif
#ifndef RTOS_VERBOSE_GPS_LOG
#define RTOS_VERBOSE_GPS_LOG       0   /* [GPS] 周期状态 */
#endif
#ifndef RTOS_VERBOSE_STACK_LOG
#define RTOS_VERBOSE_STACK_LOG     0   /* [RTOS] stack free */
#endif
#ifndef RTOS_VERBOSE_JETSON_LINK
#define RTOS_VERBOSE_JETSON_LINK   1   /* [JETSON LINK] +5s 下行/上行统计 */
#endif

#if RTOS_DEBUG_ENABLE
#define RTOS_PRINT(fmt, ...)    printf(fmt, ##__VA_ARGS__)
#else
#define RTOS_PRINT(fmt, ...)
#endif

#define CREATE_TASK(fn, name, stack, prio, handle) \
	xTaskCreate(fn, name, stack, NULL, prio, handle)

#define RTOS_STACK_LOG_MS    5000u

#endif
