#include "freertos_app.h"
#include "rtos_config.h"
#include "rtos_tasks.h"
#include "motion_ui_shared.h"
#include "usart.h"
#include "usart3.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_usart.h"
#include "stdio.h"

static void RTOS_PanicPutc(char c)
{
	USART_SendData(USART1, (u8)c);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

static void RTOS_PanicPrint(const char *msg)
{
	while(msg && *msg)
		RTOS_PanicPutc(*msg++);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	(void)xTask;
	RTOS_PanicPrint("[RTOS] stack overflow: ");
	RTOS_PanicPrint(pcTaskName ? pcTaskName : "?");
	RTOS_PanicPrint("\r\n");
	while(1);
}

void App_SharedInit(void)
{
	Usart_PrintMutexInit();
#if !JETSON_LINK_CAN
	USART3_TxMutexInit();
#endif
	MotionUI_SharedInit();
}

void App_TasksCreate(void)
{
	RTOS_PRINT("[RTOS] Creating tasks\r\n");

	CREATE_TASK(vMotionTask, "Motion", MOTION_TASK_STACK_SIZE, MOTION_TASK_PRIO, &xMotionTaskHandle);
	CREATE_TASK(vCanTask, "Can", CAN_TASK_STACK_SIZE, CAN_TASK_PRIO, &xCanTaskHandle);
	CREATE_TASK(vSensorTask, "Sensor", SENSOR_TASK_STACK_SIZE, SENSOR_TASK_PRIO, &xSensorTaskHandle);
	CREATE_TASK(vKeyTask, "Key", KEY_TASK_STACK_SIZE, KEY_TASK_PRIO, &xKeyTaskHandle);
	CREATE_TASK(vJetsonTask, "Jetson", JETSON_TASK_STACK_SIZE, JETSON_TASK_PRIO, &xJetsonTaskHandle);
	CREATE_TASK(vGpsTask, "Gps", GPS_TASK_STACK_SIZE, GPS_TASK_PRIO, &xGpsTaskHandle);
#if ETH_LWIP_ENABLE
	CREATE_TASK(vNetTask, "Net", NET_TASK_STACK_SIZE, NET_TASK_PRIO, &xNetTaskHandle);
#endif
	CREATE_TASK(vUiTask, "Ui", UI_TASK_STACK_SIZE, UI_TASK_PRIO, &xUiTaskHandle);

	MotionUI_SetUiTaskHandle(xUiTaskHandle);
}

void RTOS_AppStart(void)
{
	App_SharedInit();
	App_TasksCreate();

	RTOS_PRINT("[RTOS] Starting scheduler\r\n");
	vTaskStartScheduler();

	RTOS_PRINT("[RTOS] Scheduler returned unexpectedly\r\n");
	while(1);
}
