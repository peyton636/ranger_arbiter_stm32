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
#include "app_boot.h"
#include "stdio.h"
#include "usart.h"
#if ETH_LWIP_ENABLE
#include "lwip_comm.h"
#include "lan8720.h"
#endif
#include "agv_blob_wire.h"
#if JETSON_USE_BLOB_V2 && !JETSON_LINK_CAN
#include "agv_blob_pack.h"
#include "agv_blob_link.h"
#if JETSON_LINK_ETH
#include "jetson_eth.h"
#endif
#endif
#include "FreeRTOS.h"
#include "task.h"

TaskHandle_t xMotionTaskHandle  = NULL;
TaskHandle_t xCanTaskHandle     = NULL;
TaskHandle_t xSensorTaskHandle  = NULL;
TaskHandle_t xKeyTaskHandle     = NULL;
TaskHandle_t xJetsonTaskHandle  = NULL;
TaskHandle_t xGpsTaskHandle     = NULL;
TaskHandle_t xUiTaskHandle      = NULL;
#if ETH_LWIP_ENABLE
TaskHandle_t xNetTaskHandle     = NULL;
#endif

static u16 s_can_lcd_div = 0;
static u8 s_jetson_tx_toggle = 0;
static u32 s_ui_frame_count = 0;
static TickType_t s_ui_last_stack_log = 0;

#if !JETSON_LINK_CAN && JETSON_USE_BLOB_V2 && RTOS_VERBOSE_JETSON_LINK
static u16 s_gps_link_div = 0;
static u32 s_link_last_rx_b = 0;
static u32 s_link_last_ab = 0;
static u32 s_link_last_tx_b = 0;
static u32 s_link_last_m02 = 0;
static u32 s_link_last_m03 = 0;
#if JETSON_LINK_ETH
static u32 s_link_last_svc_rx = 0;
static u32 s_link_last_svc_107 = 0;
static u32 s_link_last_svc_tx108 = 0;
static u32 s_link_last_svc_rej = 0;
#endif

/* 在 vGpsTask 定时打印，不依赖 vJetsonTask 循环计数 */
static void JetsonLink_TickEvery5s(void)
{
	u32 rx_b = 0, ore = 0, ab = 0;
	u32 tx_b, tx_f, tx_m02, tx_m03;
	u32 ctrl, db = 0, dab = 0, dtx, dm02, dm03;
	u16 ds_f, ds_b, ds_l, ds_r, ds_n;
	u8 arb, hb_lost;
#if JETSON_LINK_ETH
	u32 svc_rx, svc_107, svc_tx108, svc_rej;
	u32 dsvc, d107, dtx108, drej;
#endif

	if(++s_gps_link_div < 50)
		return;
	s_gps_link_div = 0;

#if JETSON_LINK_ETH
	JetsonEth_GetSvcStats(&svc_rx, &svc_107, &svc_tx108, &svc_rej);
	dsvc = svc_rx - s_link_last_svc_rx;
	d107 = svc_107 - s_link_last_svc_107;
	dtx108 = svc_tx108 - s_link_last_svc_tx108;
	drej = svc_rej - s_link_last_svc_rej;
	s_link_last_svc_rx = svc_rx;
	s_link_last_svc_107 = svc_107;
	s_link_last_svc_tx108 = svc_tx108;
	s_link_last_svc_rej = svc_rej;
#else
	USART3_GetRxStats(&rx_b, &ore, &ab);
	db = rx_b - s_link_last_rx_b;
	dab = ab - s_link_last_ab;
	s_link_last_rx_b = rx_b;
	s_link_last_ab = ab;
#endif
	BlobLink_GetTxStats(&tx_b, &tx_f, &tx_m02, &tx_m03);
	ctrl = BlobPack_GetRxCtrlCount();
	dtx = tx_b - s_link_last_tx_b;
	dm02 = tx_m02 - s_link_last_m02;
	dm03 = tx_m03 - s_link_last_m03;
	s_link_last_tx_b = tx_b;
	s_link_last_m02 = tx_m02;
	s_link_last_m03 = tx_m03;

	DistSnapshot_Read(&ds_f, &ds_b, &ds_l, &ds_r, &ds_n);

	App_ArbiterLock();
	arb = (u8)arb_state.current_mode;
	hb_lost = arb_state.heartbeat_lost ? 1u : 0u;
	App_ArbiterUnlock();
	if(arb > ARBITER_MODE_RECOVERING)
		arb = ARBITER_MODE_DEGRADED;

#if JETSON_LINK_ETH
	printf("[JETSON LINK] +5s: ctrl=%lu rej=%lu | up=%luB 0x02=%lu 0x03=%lu | svc=%lu 107=%lu tx108=%lu svc_rej=%lu | DS F=%u B=%u L=%u R=%u near=%u ARB=%s HB=%s\r\n",
		(unsigned long)ctrl,
		(unsigned long)JetsonEth_GetRxDropCount(),
		(unsigned long)dtx, (unsigned long)dm02, (unsigned long)dm03,
		(unsigned long)dsvc, (unsigned long)d107, (unsigned long)dtx108, (unsigned long)drej,
		(unsigned)ds_f, (unsigned)ds_b, (unsigned)ds_l, (unsigned)ds_r, (unsigned)ds_n,
		ARBITER_MODE_NAMES[arb],
		hb_lost ? "LOST" : "OK");
#else
	printf("[JETSON LINK] +5s: dn=%luB ab=%lu ctrl=%lu rej=%lu ore=%lu | up=%luB 0x02=%lu 0x03=%lu | DS F=%u B=%u L=%u R=%u near=%u ARB=%s HB=%s\r\n",
		(unsigned long)db, (unsigned long)dab,
		(unsigned long)ctrl,
		(unsigned long)BlobRs232_GetHdrRejectCount(),
		(unsigned long)ore,
		(unsigned long)dtx, (unsigned long)dm02, (unsigned long)dm03,
		(unsigned)ds_f, (unsigned)ds_b, (unsigned)ds_l, (unsigned)ds_r, (unsigned)ds_n,
		ARBITER_MODE_NAMES[arb],
		hb_lost ? "LOST" : "OK");
#endif
}
#endif

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
#if ETH_LWIP_ENABLE || APP_SENSOR_TEST_ONLY
	u16 ds_diag_log_div = 0;
#endif

	(void)pvParameters;

	for(;;)
	{
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_TASK_CYCLE_MS));

		DistanceSensor_PollRx();

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

#if ETH_LWIP_ENABLE || APP_SENSOR_TEST_ONLY
		/* 每 2s 打印测距诊断（独占测试 / 以太网联调共用） */
		if(++ds_diag_log_div >= 200)
		{
			char ff[16], bb[16], ll[16], rr[16];
			u32 ds_rx, ds_ok, ds_proc, ds_trig, ds_poll;
			u32 ds_skip, ds_hdr, ds_chk;
			char rxhex[40];

			ds_diag_log_div = 0;
			DistSnapshot_Read(&f, &b, &l, &r, &n);
			DistanceSensor_GetDiag(&ds_rx, &ds_ok);
			ds_proc = DistanceSensor_GetProcessTick();
			ds_trig = DistanceSensor_GetTrigCount();
			ds_poll = DistanceSensor_GetPollRxBytes();
			DistanceSensor_GetRxParseDiag(&ds_skip, &ds_hdr, &ds_chk);
			DistanceSensor_FormatLastRxHex(rxhex, sizeof(rxhex));
			DistanceSensor_FormatFilteredLane(0, ff, sizeof(ff));
			DistanceSensor_FormatFilteredLane(1, bb, sizeof(bb));
			DistanceSensor_FormatFilteredLane(2, ll, sizeof(ll));
			DistanceSensor_FormatFilteredLane(3, rr, sizeof(rr));
#if APP_SENSOR_TEST_ONLY
			printf("[DS TEST] F=%s B=%s L=%s R=%s near=%u valid=%u proc=%lu trig=%lu rx=%lu ok=%lu skip=%lu FF=%lu chk=%lu raw=%s\r\n",
				ff, bb, ll, rr, (unsigned)n, (unsigned)(ds->valid ? 1u : 0u),
				(unsigned long)ds_proc, (unsigned long)ds_trig,
				(unsigned long)ds_rx, (unsigned long)ds_ok,
				(unsigned long)ds_skip, (unsigned long)ds_hdr, (unsigned long)ds_chk,
				rxhex);
#else
			printf("[DS ETH] F=%s B=%s L=%s R=%s near=%u valid=%u uart_rx=%lu frame_ok=%lu\r\n",
				ff, bb, ll, rr, (unsigned)n, (unsigned)(ds->valid ? 1u : 0u),
				(unsigned long)ds_rx, (unsigned long)ds_ok);
#endif
		}
#endif
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
#elif JETSON_LINK_ETH
		{
			u32 svc_id;
			u8 svc_buf[8];

			while(JetsonEth_GetServiceRequest(&svc_id, svc_buf))
				JetsonCAN_HandleServiceRequest(svc_id, svc_buf, 8, &arb_state, n);
		}
#else
		{
			u32 svc_id;
			u8 svc_buf[8];

			if(USART3_GetServiceRequest(&svc_id, svc_buf))
				JetsonCAN_HandleServiceRequest(svc_id, svc_buf, 8, &arb_state, n);
		}
#endif

		App_ArbiterLock();

#if !JETSON_LINK_CAN && JETSON_USE_BLOB_V2
		{
			blob_rx_frame_t blob_frame;

			while(BlobLink_GetFrame(&blob_frame))
				BlobPack_HandleDownlink(&blob_frame);
		}
#endif

#if !JETSON_LINK_CAN && !JETSON_USE_BLOB_V2
		if(USART3_GetJetsonFrame(jetson_frame))
		{
			if(Arbiter_ParseJetsonCmd(jetson_frame, JETSON_FRAME_LEN) != 0)
				RTOS_PRINT("[JETSON CMD] parse failed\r\n");
		}
#elif JETSON_LINK_CAN
		if(JetsonCAN_GetFrame(jetson_frame))
		{
			if(Arbiter_ParseJetsonCmd(jetson_frame, JETSON_FRAME_LEN) != 0)
				RTOS_PRINT("[JETSON CMD] parse failed\r\n");
		}
#endif
		JetsonCAN_ServiceFault(&arb_state);
#if JETSON_USE_BLOB_V2 && !JETSON_LINK_CAN
		BlobPack_UplinkTick(&arb_state, f, b, l, r, n);
#else
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
#endif
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
#if JETSON_USE_BLOB_V2 && !JETSON_LINK_CAN
		BlobPack_FlushDebugLog();
#if RTOS_VERBOSE_JETSON_LINK
		JetsonLink_TickEvery5s();
#endif
		BlobPack_SendGps(GPS_GetData());
#else
		JetsonCAN_SendGps(GPS_GetData());
#endif
#if RTOS_VERBOSE_GPS_LOG
		GPS_PrintStatus();
#endif
	}
}

#if ETH_LWIP_ENABLE
void vNetTask(void *pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	u16 stat_div = 0;

	(void)pvParameters;

	for(;;)
	{
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(NET_TASK_CYCLE_MS));
		App_EthPoll();
		if(++stat_div >= 1000)
		{
			u32 rx;
			u8 link;

			stat_div = 0;
			if(App_EthIsReady())
			{
				App_EthGetStats(&rx, &link);
#if APP_LCD_MINIMAL_TEST
				USART1_Probe("HB");
#endif
				printf("[ETH] link=%s rx=%lu tx_ok=%lu rbus=%u desc_cpu=%u (rx=0=no L2 frames)\r\n",
					link ? "UP" : "DOWN", (unsigned long)rx,
					(unsigned long)ETH_TxOkCount(),
					(unsigned)ETH_DmaSrHasRbus(),
					(unsigned)ETH_RxDescCpuCount());
			}
		}
	}
}
#endif

void vUiTask(void *pvParameters)
{
	DistanceSensor_Data *ds;
	TickType_t now;

	(void)pvParameters;

	for(;;)
	{
		(void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(UI_TASK_NOTIFY_TIMEOUT_MS));

#if APP_LCD_MINIMAL_TEST
		DistanceSensor_DrainLog();
		continue;
#endif

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

		/* 与 RTOS 前 SensorData_ShowScreen 不同：仲裁模式由 vUiTask 刷新测距/GPS */
		if(g_sensor_updated)
		{
			s_ui_frame_count++;
#if RTOS_VERBOSE_SENSOR_LOG
			DistanceSensor_Print();
#endif
			if(!g_sensor_ui_inited)
				SensorUI_DrawStatic();
			SensorUI_UpdateCount(s_ui_frame_count);
			SensorUI_UpdateDistances(ds);
			g_sensor_updated = 0;
		}

		if(g_sensor_ui_inited)
			SensorUI_UpdateGps();

		if(g_can_lcd_due)
		{
#if RTOS_VERBOSE_CHASSIS_LOG
			Arbiter_PrintChassisFeedback();
#endif
			Chassis_UpdateOnLCD();
			g_can_lcd_due = 0;
		}

		DistanceSensor_DrainLog();

#if RTOS_VERBOSE_STACK_LOG
		if((now - s_ui_last_stack_log) >= pdMS_TO_TICKS(RTOS_STACK_LOG_MS))
		{
			RTOS_PrintTaskStackWatermarks();
			s_ui_last_stack_log = now;
		}
#endif
	}
}
