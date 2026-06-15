#include "motion_control.h"
#include "motion_ui_shared.h"
#include "arbiter.h"
#include "beep.h"
#include "key.h"
#include "SysTick.h"
#include "stdio.h"

#define ARBITER_CAN_DIV  2

void MotionControl_KeyProcess(void)
{
	u8 key;

	key = KEY_Scan(0);
	if(key == KEY0_PRESS)
	{
		g_force_stop_enable = !g_force_stop_enable;
		g_beep_ui_dirty = 1;
		g_force_stop_ui_dirty = 1;
		App_ArbiterLock();
		if(g_force_stop_enable)
		{
			arb_state.output.v = 0;
			arb_state.output.omega = 0;
			arb_state.output.steering = 0;
			arb_state.output.mode = ARBITER_MODE_EMERGENCY;
			arb_state.output.emergency = 1;
			BEEP_SetVolume(0);
			printf("[CTRL] FORCE STOP ON (KEY0)\r\n");
		}
		else
		{
			Arbiter_Init();
			Arbiter_EnableCANMode();
			printf("[CTRL] FORCE STOP OFF -> Arbiter restart (KEY0)\r\n");
		}
		App_ArbiterUnlock();
		delay_ms(200);
		return;
	}

	if(key == KEY1_PRESS)
	{
		g_beep_dist_enable = !g_beep_dist_enable;
		if(!g_beep_dist_enable)
			BEEP_SetVolume(0);
		g_beep_ui_dirty = 1;
		printf("[BEEP] Distance alert %s (KEY1 toggle)\r\n",
			g_beep_dist_enable ? "ON" : "OFF");
		delay_ms(200);
	}
}

void MotionControl_BeepUpdateByDistance(u16 nearest_mm)
{
#if ARBITER_IGNORE_DIST_SENSOR
	(void)nearest_mm;
	BEEP_SetVolume(0);
#else
	u8 vol;

	if(!g_beep_dist_enable || nearest_mm == DS_DIST_UNKNOWN)
	{
		BEEP_SetVolume(0);
		return;
	}

	if(nearest_mm >= ARBITER_OBSTACLE_FAR_MM)
		vol = 0;
	else if(nearest_mm < ARBITER_OBSTACLE_WARN_MM)
		vol = 100;
	else
		vol = 20 + (u8)((ARBITER_OBSTACLE_FAR_MM - nearest_mm) * 60 /
			(ARBITER_OBSTACLE_FAR_MM - ARBITER_OBSTACLE_WARN_MM));

	BEEP_SetVolume(vol);
#endif
}

void MotionControl_Run(void)
{
	static u16 can_send_div = 0;
	u16 f, b, l, r, n;

	DistSnapshot_Read(&f, &b, &l, &r, &n);

	App_ArbiterLock();
	if(g_force_stop_enable)
	{
		arb_state.output.v = 0;
		arb_state.output.omega = 0;
		arb_state.output.steering = 0;
		arb_state.output.mode = ARBITER_MODE_EMERGENCY;
		arb_state.output.emergency = 1;
		can_send_div++;
		if(can_send_div >= ARBITER_CAN_DIV)
		{
			can_send_div = 0;
			Arbiter_SendToSTM32A();
		}
		App_ArbiterUnlock();
		return;
	}

	Arbiter_SetObstacleDistances(f, b, l, r);
	Arbiter_Process();

	can_send_div++;
	if(can_send_div >= ARBITER_CAN_DIV)
	{
		can_send_div = 0;
		Arbiter_SendToSTM32A();
	}
	App_ArbiterUnlock();
}
