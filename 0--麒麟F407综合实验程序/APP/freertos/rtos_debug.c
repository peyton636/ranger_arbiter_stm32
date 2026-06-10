#include "rtos_debug.h"
#include "rtos_config.h"
#include "rtos_tasks.h"
#include "task.h"

static void RTOS_LogStackFree(TaskHandle_t xTask)
{
	if(xTask)
	{
		RTOS_PRINT("[RTOS] %s stack free: %u w\r\n",
			pcTaskGetName(xTask), (unsigned)uxTaskGetStackHighWaterMark(xTask));
	}
}

void RTOS_PrintTaskStackWatermarks(void)
{
	RTOS_LogStackFree(xMotionTaskHandle);
	RTOS_LogStackFree(xCanTaskHandle);
	RTOS_LogStackFree(xSensorTaskHandle);
	RTOS_LogStackFree(xKeyTaskHandle);
	RTOS_LogStackFree(xJetsonTaskHandle);
	RTOS_LogStackFree(xGpsTaskHandle);
	RTOS_LogStackFree(xUiTaskHandle);
}
