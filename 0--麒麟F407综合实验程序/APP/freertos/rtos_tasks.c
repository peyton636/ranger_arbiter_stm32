#include "rtos_tasks.h"
#include "rtos_config.h"
#include "rtos_debug.h"
#include "motion_ui_shared.h"
#include "motion_control.h"
#include "distance_sensor.h"
#include "sensor_ui.h"
#include "arbiter.h"
#include "can.h"
#include "usart3.h"
#include "jetson_can.h"
#include "gps.h"
#include "FreeRTOS.h"
#include "task.h"

TaskHandle_t xMotionTaskHandle  = NULL;
TaskHandle_t xCanTaskHandle     = NULL;
TaskHandle_t xSensorTaskHandle  = NULL;
TaskHandle_t xKeyTaskHandle     = NULL;
TaskHandle_t xJetsonTaskHandle  = NULL;
TaskHandle_t xGpsTaskHandle     = NULL;
TaskHandle_t xUiTaskHandle      = NULL;

static u16 s_can_lcd_div = 0;
static u8 s_jetson_tx_toggle = 0;
static u32 s_ui_frame_count = 0;
static TickType_t s_ui_last_stack_log = 0;

static u16 s_pick_filtered_mm(u8 idx, u16 raw_if_ok)
{
	u16 filtered = DistanceSensor_GetFilteredMm(idx);

	if(filtered != DS_DIST_UNKNOWN && filtered != DS_DIST_FAILSAFE_MM)
		return filtered;
	if(raw_if_ok != DS_DIST_UNKNOWN)
		return raw_if_ok;
	return DS_DIST_UNKNOWN;
}

void vMotionTask(void *pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();

	(void)pvParameters;

	for(;;)
	{
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MOTION_TASK_CYCLE_MS));
		MotionControl_Run();
	}
}

void vCanTask(void *pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	u8 can_updated;

	(void)pvParameters;

	for(;;)
	{
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CAN_TASK_CYCLE_MS));

		can_updated = 0;
		while(CAN_MessagePending(CAN1, CAN_FIFO0))
		{
			App_ArbiterLock();
			Arbiter_ProcessCANFeedback();
			App_ArbiterUnlock();
			can_updated = 1;
		}

		if(can_updated)
		{
			s_can_lcd_div++;
			if(s_can_lcd_div >= UI_CAN_LCD_DIV)
			{
				s_can_lcd_div = 0;
				g_can_lcd_due = 1;
			}
		}
	}
}

void vSensorTask(void *pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	DistanceSensor_Data *ds;
	u16 f, b, l, r, n;

	(void)pvParameters;

	for(;;)
	{
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_TASK_CYCLE_MS));

		ds = DistanceSensor_GetData();
		if(DistanceSensor_NewData() && ds->valid)
		{
			u16 rf, rb, rl, rr;

			rf = (ds->error[0] == DS_ERR_NONE) ? ds->dist[0] : DS_DIST_UNKNOWN;
			rb = (ds->error[1] == DS_ERR_NONE) ? ds->dist[1] : DS_DIST_UNKNOWN;
			rl = (ds->error[2] == DS_ERR_NONE) ? ds->dist[2] : DS_DIST_UNKNOWN;
			rr = (ds->error[3] == DS_ERR_NONE) ? ds->dist[3] : DS_DIST_UNKNOWN;
			f = s_pick_filtered_mm(0, rf);
			b = s_pick_filtered_mm(1, rb);
			l = s_pick_filtered_mm(2, rl);
			r = s_pick_filtered_mm(3, rr);
			n = DistanceSensor_GetFilteredMinDistMm();
			if(n == DS_DIST_UNKNOWN)
				n = DistanceSensor_MinDistMm();
			DistSnapshot_Write(f, b, l, r, n);
			MotionControl_BeepUpdateByDistance(n);
			g_sensor_updated = 1;
			MotionUI_NotifySensorFrame();
		}
	}
}

void vKeyTask(void *pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();

	(void)pvParameters;

	for(;;)
	{
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(KEY_TASK_CYCLE_MS));
		MotionControl_KeyProcess();
	}
}

void vJetsonTask(void *pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	u8 jetson_frame[JETSON_FRAME_LEN];
	u16 f, b, l, r, n;

	(void)pvParameters;

	for(;;)
	{
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(JETSON_TASK_CYCLE_MS));

		DistSnapshot_Read(&f, &b, &l, &r, &n);

#if JETSON_LINK_CAN
		JetsonCAN_ProcessRx(&arb_state, n);
#else
		{
			u32 svc_id;
			u8 svc_buf[8];

			if(USART3_GetServiceRequest(&svc_id, svc_buf))
				JetsonCAN_HandleServiceRequest(svc_id, svc_buf, 8, &arb_state, n);
		}
#endif

		App_ArbiterLock();
		
#if JETSON_LINK_CAN
		if(JetsonCAN_GetFrame(jetson_frame))
#else
		if(USART3_GetJetsonFrame(jetson_frame))
#endif
		
		{
			if(Arbiter_ParseJetsonCmd(jetson_frame, JETSON_FRAME_LEN) != 0)
				RTOS_PRINT("[JETSON CMD] parse failed\r\n");
		}
		JetsonCAN_ServiceFault(&arb_state);
		if(s_jetson_tx_toggle == 0)
		{
			USART3_SendV3StatusFrame(&arb_state, f, b, l, r);
			s_jetson_tx_toggle = 1;
		}
		else
		{
			USART3_SendV3DetailFrame(&arb_state);
			s_jetson_tx_toggle = 0;
		}
		App_ArbiterUnlock();
	}
}

void vGpsTask(void *pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();

	(void)pvParameters;

	for(;;)
	{
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(GPS_TASK_CYCLE_MS));
		GPS_Process();
		JetsonCAN_SendGps(GPS_GetData());
		GPS_PrintStatus();
	}
}

void vUiTask(void *pvParameters)
{
	DistanceSensor_Data *ds;
	TickType_t now;

	(void)pvParameters;

	for(;;)
	{
		(void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(UI_TASK_NOTIFY_TIMEOUT_MS));

		ds = DistanceSensor_GetData();
		now = xTaskGetTickCount();

		if(!g_sensor_ui_inited)
		{
			SensorUI_DrawStatic();
			SensorUI_UpdateCount(0);
			SensorUI_UpdateDistances(ds);
			SensorUI_UpdateBeepStatus();
			SensorUI_UpdateGps();
			Chassis_UpdateOnLCD();
			s_ui_last_stack_log = now;
		}

		if(g_beep_ui_dirty)
		{
			SensorUI_UpdateBeepStatus();
			g_beep_ui_dirty = 0;
		}

		if(g_force_stop_ui_dirty)
		{
			SensorUI_UpdateForceStopBanner();
			g_force_stop_ui_dirty = 0;
		}

		/* ?? RTOS ? SensorData_ShowScreen ????????????????????????/???? */
		if(g_sensor_updated)
		{
			s_ui_frame_count++;
			DistanceSensor_Print();
			SensorUI_UpdateCount(s_ui_frame_count);
			if(g_sensor_ui_inited)
				SensorUI_UpdateDistances(ds);
			g_sensor_updated = 0;
		}

		if(g_sensor_ui_inited)
			SensorUI_UpdateGps();

		if(g_can_lcd_due)
		{
			Arbiter_PrintChassisFeedback();
			Chassis_UpdateOnLCD();
			g_can_lcd_due = 0;
		}

		DistanceSensor_DrainLog();

		if((now - s_ui_last_stack_log) >= pdMS_TO_TICKS(RTOS_STACK_LOG_MS))
		{
			RTOS_PrintTaskStackWatermarks();
			s_ui_last_stack_log = now;
		}
	}
}
