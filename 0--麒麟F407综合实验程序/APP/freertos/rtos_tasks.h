#ifndef __RTOS_TASKS_H
#define __RTOS_TASKS_H

#include "app_boot.h"
#include "FreeRTOS.h"
#include "task.h"

#define MOTION_TASK_STACK_SIZE    512   /* 含 Jetson 延后 debug printf */
#define MOTION_TASK_PRIO          6
#define MOTION_TASK_CYCLE_MS      10

#define CAN_TASK_STACK_SIZE       384
#define CAN_TASK_PRIO             5
#define CAN_TASK_CYCLE_MS         10

#define SENSOR_TASK_STACK_SIZE    384
#define SENSOR_TASK_PRIO          4
#define SENSOR_TASK_CYCLE_MS      10

#define KEY_TASK_STACK_SIZE       256
#define KEY_TASK_PRIO             4
#define KEY_TASK_CYCLE_MS         20

#define JETSON_TASK_STACK_SIZE    1024
#define JETSON_TASK_PRIO          3
#define JETSON_TASK_CYCLE_MS      20

#define GPS_TASK_STACK_SIZE       768
#define GPS_TASK_PRIO             2
#define GPS_TASK_CYCLE_MS         100

#define NET_TASK_STACK_SIZE       512
#define NET_TASK_PRIO             2
#define NET_TASK_CYCLE_MS         5

#define UI_TASK_STACK_SIZE            768
#define UI_TASK_PRIO                  3
#define UI_TASK_NOTIFY_TIMEOUT_MS     20
#define UI_DS_PRINT_MS                200

extern TaskHandle_t xMotionTaskHandle;
extern TaskHandle_t xCanTaskHandle;
extern TaskHandle_t xSensorTaskHandle;
extern TaskHandle_t xKeyTaskHandle;
extern TaskHandle_t xJetsonTaskHandle;
extern TaskHandle_t xGpsTaskHandle;
extern TaskHandle_t xUiTaskHandle;
#if ETH_LWIP_ENABLE
extern TaskHandle_t xNetTaskHandle;
#endif

void vMotionTask(void *pvParameters);
void vCanTask(void *pvParameters);
void vSensorTask(void *pvParameters);
void vKeyTask(void *pvParameters);
void vJetsonTask(void *pvParameters);
void vGpsTask(void *pvParameters);
void vUiTask(void *pvParameters);
#if ETH_LWIP_ENABLE
void vNetTask(void *pvParameters);
#endif

#endif
