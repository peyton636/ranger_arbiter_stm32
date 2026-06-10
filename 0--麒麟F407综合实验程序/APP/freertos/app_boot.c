#include "app_boot.h"
#include "usart3.h"
#include "gps.h"
#include "distance_sensor.h"
#include "can.h"
#include "arbiter.h"
#include "motion_ui_shared.h"
#include "sensor_ui.h"
#include "tftlcd.h"
#include "stdio.h"

void App_MotionHwInit(void)
{
	USART3_Init();
	GPS_USART6_Init(9600);
	DistanceSensor_Init();
	CAN1_Init_RangerMini();
	Arbiter_Init();
	Arbiter_EnableCANMode();

	printf("[JETSON] USART2 RX ready, expecting 24-byte V3 frame\r\n");
	printf("[CAN] Init done, MCR=0x%08X MSR=0x%08X\r\n",
		(unsigned int)CAN1->MCR, (unsigned int)CAN1->MSR);
	printf("[MOTION] Dist ctrl ON, wait Jetson V3 on USART2(PA2/PA3), beep=%s (KEY1 toggle)\r\n",
		g_beep_dist_enable ? "ON" : "OFF");
	printf("[GPS] USART6 RX ready on PC7 (TX=PC6), auto cfg: GPS+BDS 5Hz RMC/GGA/GSA\r\n");
}

void App_ShowBootSplash(void)
{
	LCD_Clear(BLACK);
	FRONT_COLOR = WHITE;
	BACK_COLOR = BLACK;
	SensorUI_DrawStatic();
	SensorUI_UpdateDistances(DistanceSensor_GetData());
	SensorUI_UpdateBeepStatus();
	SensorUI_UpdateGps();
	printf("[LCD] Sensor UI ready; layout=pre-RTOS + GPS\r\n");
}
