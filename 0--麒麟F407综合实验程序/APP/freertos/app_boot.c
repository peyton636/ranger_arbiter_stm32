#include "app_boot.h"
#include "usart3.h"
#include "jetson_can.h"
#include "gps.h"
#include "distance_sensor.h"
#include "can.h"
#include "arbiter.h"
#include "motion_ui_shared.h"
#include "sensor_ui.h"
#include "tftlcd.h"
#include "time.h"
#include "stdio.h"

void App_MotionHwInit(void)
{
	/* PRECHIN 在 Hardware_Check 里开了 TIM2(100ms) 拍照中断，ISR 内 KEY_Scan+delay_ms 与 KeyTask 冲突 */
	TIM_Cmd(TIM2, DISABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
#if JETSON_LINK_CAN
	JetsonCAN_Init();
#else
	USART3_Init();
#endif
	GPS_USART6_Init(9600);
	DistanceSensor_Init();
	CAN1_Init_RangerMini();
	Arbiter_Init();
	Arbiter_EnableCANMode();

#if JETSON_LINK_CAN
	printf("[JETSON] CAN2 PB5(RX)/PB6(TX), 500kbps, V3 24B as 3x8B frames\r\n");
	printf("[JETSON] CAN IDs: down=0x101 status=0x102 detail=0x103 gps=0x104~0x106\r\n");
	printf("[JETSON] Wire PB6->TJA1050 TXD, PB5<-RXD, CANH/L to Jetson USB-CAN\r\n");
#else
	printf("[JETSON] USART2 PA2/PA3, 115200, 24-byte V3 frame\r\n");
#endif
	printf("[CAN1] Chassis PA11/12 init OK, MCR=0x%08X MSR=0x%08X\r\n",
		(unsigned int)CAN1->MCR, (unsigned int)CAN1->MSR);
	printf("[MOTION] Dist ctrl ON, wait Jetson V3, beep=%s (KEY1 toggle)\r\n",
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
