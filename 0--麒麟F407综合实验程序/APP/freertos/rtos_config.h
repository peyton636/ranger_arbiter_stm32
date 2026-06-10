#ifndef __RTOS_CONFIG_H
#define __RTOS_CONFIG_H

#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"

#define RTOS_DEBUG_ENABLE    1

#if RTOS_DEBUG_ENABLE
#define RTOS_PRINT(fmt, ...)    printf(fmt, ##__VA_ARGS__)
#else
#define RTOS_PRINT(fmt, ...)
#endif

#define CREATE_TASK(fn, name, stack, prio, handle) \
	xTaskCreate(fn, name, stack, NULL, prio, handle)

#define RTOS_STACK_LOG_MS    5000u

#endif
