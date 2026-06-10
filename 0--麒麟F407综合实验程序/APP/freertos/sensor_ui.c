#include "sensor_ui.h"
#include "motion_ui_shared.h"
#include "tftlcd.h"
#include "arbiter.h"
#include "distance_sensor.h"
#include "gps.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"

static void SensorUI_UpdateLine(u16 y, const char *text)
{
	LCD_Fill(UI_X, y, UI_X + UI_LINE_W, y + UI_FS - 1, BLACK);
	LCD_ShowString(UI_X, y, tftlcd_data.width, tftlcd_data.height, UI_FS, (u8*)text);
}

void SensorUI_UpdateForceStopBanner(void)
{
	LCD_Fill(UI_X, UI_Y_TITLE, tftlcd_data.width - 1, UI_Y_TITLE + UI_FS - 1, BLACK);
	if(g_force_stop_enable)
		LCD_ShowString(UI_X, UI_Y_TITLE, tftlcd_data.width, tftlcd_data.height, UI_FS, "*** FORCE STOP ON ***");
	else
		LCD_ShowString(UI_X, UI_Y_TITLE, tftlcd_data.width, tftlcd_data.height, UI_FS, "=== Sensor Data ===");
}

void SensorUI_DrawStatic(void)
{
	u16 cx;
	u16 top_y;
	u16 bot_y;
	u16 left_x;
	u16 right_x;
	u16 mid_y;

	LCD_Clear(BLACK);
	FRONT_COLOR = WHITE;
	BACK_COLOR = BLACK;
	cx = tftlcd_data.width / 2;
	top_y = UI_Y_IF1;
	bot_y = UI_Y_IF4;
	mid_y = (u16)((top_y + bot_y) / 2);
	left_x = 10;
	right_x = (u16)(cx + 20);

	SensorUI_UpdateForceStopBanner();
	LCD_ShowString(UI_X, UI_Y_COUNT, tftlcd_data.width, tftlcd_data.height, UI_FS, "Count:");
	LCD_ShowString(UI_X, UI_Y_DIST_HDR, tftlcd_data.width, tftlcd_data.height, UI_FS, "Distance Sensors:");
	/* IF1 上、IF2 下、IF3 左、IF4 右（与 RTOS 前 main.c 十字布局一致） */
	LCD_ShowString((u16)(cx - 56), top_y, tftlcd_data.width, tftlcd_data.height, UI_FS, "IF1:");
	LCD_ShowString((u16)(cx - 56), bot_y, tftlcd_data.width, tftlcd_data.height, UI_FS, "IF2:");
	LCD_ShowString(left_x, mid_y, tftlcd_data.width, tftlcd_data.height, UI_FS, "IF3:");
	LCD_ShowString(right_x, mid_y, tftlcd_data.width, tftlcd_data.height, UI_FS, "IF4:");
	LCD_ShowString(UI_X, UI_Y_CHASSIS, tftlcd_data.width, tftlcd_data.height, UI_FS, "--- Chassis CAN ---");
	LCD_ShowString(UI_X, UI_Y_GPS_HDR, tftlcd_data.width, tftlcd_data.height, UI_FS, "--- GPS ---");

	g_sensor_ui_inited = 1;
}

void SensorUI_UpdateBeepStatus(void)
{
	if(g_force_stop_enable)
		SensorUI_UpdateLine(UI_Y_BEEP, "STOP:ON KEY0=resume");
	else if(g_beep_dist_enable)
		SensorUI_UpdateLine(UI_Y_BEEP, "Beep:ON  KEY1=toggle");
	else
		SensorUI_UpdateLine(UI_Y_BEEP, "Beep:OFF KEY1=toggle");
}

void SensorUI_UpdateCount(u32 cnt)
{
	char buf[16];

	sprintf(buf, "%lu", (unsigned long)cnt);
	LCD_Fill(UI_X + 8 * (UI_FS / 2), UI_Y_COUNT, tftlcd_data.width - 1, UI_Y_COUNT + UI_FS - 1, BLACK);
	LCD_ShowString(UI_X + 8 * (UI_FS / 2), UI_Y_COUNT, tftlcd_data.width, tftlcd_data.height, UI_FS, (u8*)buf);
}

/* 与 RTOS 前 main.c 一致：原始值 + 括号内滤波值，如 "257 mm (257)" / "0 (---)" */
static void SensorUI_FormatDist(DistanceSensor_Data *ds, u8 idx, char *buf)
{
	if(!ds->valid)
	{
		sprintf(buf, "---");
		return;
	}
	if(ds->error[idx] == DS_ERR_NONE)
	{
		if(ds->dist[idx] == DS_DIST_FAILSAFE_MM)
			sprintf(buf, "0");
		else
			sprintf(buf, "%u mm", (unsigned int)ds->dist[idx]);
	}
	else if(ds->error[idx] == DS_ERR_CHKFAIL)
		sprintf(buf, "ERR");
	else
		sprintf(buf, "0");
}

static void SensorUI_FormatDistWithFilt(DistanceSensor_Data *ds, u8 idx, char *buf)
{
	char raw[16];
	u16 filt;

	SensorUI_FormatDist(ds, idx, raw);
	filt = DistanceSensor_GetFilteredMm(idx);
	if(filt == DS_DIST_UNKNOWN)
		sprintf(buf, "%s (---)", raw);
	else if(filt == DS_DIST_FAILSAFE_MM)
		sprintf(buf, "%s (0)", raw);
	else
		sprintf(buf, "%s (%u)", raw, (unsigned int)filt);
}

void SensorUI_UpdateDistances(DistanceSensor_Data *ds)
{
	u16 cx;
	u16 top_y;
	u16 bot_y;
	u16 right_x;
	u16 mid_y;
	char buf1[28], buf2[28], buf3[28], buf4[28];

	cx = tftlcd_data.width / 2;
	top_y = UI_Y_IF1;
	bot_y = UI_Y_IF4;
	mid_y = (u16)((top_y + bot_y) / 2);
	right_x = (u16)(cx + 20);

	SensorUI_FormatDistWithFilt(ds, 0, buf1);
	SensorUI_FormatDistWithFilt(ds, 1, buf2);
	SensorUI_FormatDistWithFilt(ds, 2, buf3);
	SensorUI_FormatDistWithFilt(ds, 3, buf4);

	LCD_Fill((u16)(cx - 18), top_y, (u16)(tftlcd_data.width - 1), (u16)(top_y + UI_FS - 1), BLACK);
	LCD_ShowString((u16)(cx - 18), top_y, tftlcd_data.width, tftlcd_data.height, UI_FS, (u8*)buf1);

	LCD_Fill((u16)(cx - 18), bot_y, (u16)(tftlcd_data.width - 1), (u16)(bot_y + UI_FS - 1), BLACK);
	LCD_ShowString((u16)(cx - 18), bot_y, tftlcd_data.width, tftlcd_data.height, UI_FS, (u8*)buf2);

	LCD_Fill(48, mid_y, (u16)(cx - 10), (u16)(mid_y + UI_FS - 1), BLACK);
	LCD_ShowString(48, mid_y, tftlcd_data.width, tftlcd_data.height, UI_FS, (u8*)buf3);

	LCD_Fill((u16)(right_x + 38), mid_y, (u16)(tftlcd_data.width - 1), (u16)(mid_y + UI_FS - 1), BLACK);
	LCD_ShowString((u16)(right_x + 38), mid_y, tftlcd_data.width, tftlcd_data.height, UI_FS, (u8*)buf4);
}

void SensorUI_UpdateGps(void)
{
	const GPS_Data_t *g = GPS_GetData();
	char buf[40];
	static TickType_t s_last_tick = 0;
	TickType_t now = xTaskGetTickCount();

	if((now - s_last_tick) < pdMS_TO_TICKS(1000))
		return;
	s_last_tick = now;

	if(g == NULL || g->parsed == 0)
	{
		SensorUI_UpdateLine(UI_Y_GPS1, "GPS: waiting...");
		SensorUI_UpdateLine(UI_Y_GPS2, "");
		return;
	}

	if(g->usefull)
	{
		sprintf(buf, "GPS:FIX sv=%u %.5f%s",
			(unsigned int)g->num_sv,
			g->latitude_deg, g->ns);
		SensorUI_UpdateLine(UI_Y_GPS1, buf);
		sprintf(buf, "LON:%.5f%s SPD:%.1fm/s",
			g->longitude_deg, g->ew, g->speed_mps);
		SensorUI_UpdateLine(UI_Y_GPS2, buf);
	}
	else
	{
		sprintf(buf, "GPS: no fix (%s)", g->last_nmea_type);
		SensorUI_UpdateLine(UI_Y_GPS1, buf);
		SensorUI_UpdateLine(UI_Y_GPS2, "");
	}
}

static void Chassis_UpdateMotionLine(void)
{
	u8 fb_dir, cmd_dir;
	s16 fb_speed, cmd_speed;
	char buf[56];

	App_ArbiterLock();
	Arbiter_GetMotionInfo(&fb_dir, &fb_speed);
	Arbiter_GetCmdMotionInfo(&cmd_dir, &cmd_speed);

	sprintf(buf, "Recv:%-5s %3d v%4d s%4d w%4d",
		Arbiter_MotionDirNameAscii(fb_dir),
		(int)fb_speed,
		(int)arb_state.motion_fb.linear_speed,
		(int)arb_state.motion_fb.steering_angle,
		(int)arb_state.motion_fb.spin_speed);
	SensorUI_UpdateLine(UI_Y_MOTION_FB, buf);

	sprintf(buf, "Send:%-5s %3d v%4d s%4d w%4d",
		Arbiter_MotionDirNameAscii(cmd_dir),
		(int)cmd_speed,
		(int)arb_state.output.v,
		(int)arb_state.output.steering,
		(int)arb_state.output.omega);
	SensorUI_UpdateLine(UI_Y_MOTION_CMD, buf);
	App_ArbiterUnlock();
}

static void Chassis_UpdateWheelLines(void)
{
	char buf1[32], buf2[32];
	ChassisWheelSpeed_t ws;

	App_ArbiterLock();
	Arbiter_GetWheelSpeedPhysical(&ws);
	sprintf(buf1, "LF:%5d   RF:%5d", (int)ws.lf, (int)ws.rf);
	sprintf(buf2, "LR:%5d   RR:%5d", (int)ws.lr, (int)ws.rr);
	App_ArbiterUnlock();
	SensorUI_UpdateLine(UI_Y_WHEELS1, buf1);
	SensorUI_UpdateLine(UI_Y_WHEELS2, buf2);
}

void Chassis_UpdateOnLCD(void)
{
	char buf[40];
	u16 bms_v;

	Chassis_UpdateMotionLine();
	Chassis_UpdateWheelLines();

	App_ArbiterLock();
	sprintf(buf, "Batt:%2d.%1dV Mode:0x%02X",
		(int)(arb_state.sys_status.battery_voltage / 10),
		(int)(arb_state.sys_status.battery_voltage % 10),
		(unsigned int)arb_state.sys_status.mode_control);
	SensorUI_UpdateLine(UI_Y_BATT, buf);

	if(arb_state.bms_data.soc > 0)
	{
		bms_v = arb_state.bms_data.voltage;
		if(bms_v > 1000)
			bms_v = bms_v / 10;
		sprintf(buf, "BMS SOC:%2d%% V:%2d.%1dV",
			(int)arb_state.bms_data.soc,
			(int)(bms_v / 10),
			(int)(bms_v % 10));
		SensorUI_UpdateLine(UI_Y_BEEP + UI_FS, buf);
	}
	App_ArbiterUnlock();
}
