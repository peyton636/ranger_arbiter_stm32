#include "motion_ui_shared.h"
#include "task.h"

u16 g_front_dist_mm = DS_DIST_UNKNOWN;
u16 g_back_dist_mm = DS_DIST_UNKNOWN;
u16 g_left_dist_mm = DS_DIST_UNKNOWN;
u16 g_right_dist_mm = DS_DIST_UNKNOWN;
u16 g_nearest_dist_mm = DS_DIST_UNKNOWN;

u8 g_beep_dist_enable = 0;
u8 g_force_stop_enable = 0;
volatile u8 g_beep_ui_dirty = 0;
volatile u8 g_force_stop_ui_dirty = 1;
volatile u8 g_sensor_updated = 0;
volatile u8 g_can_lcd_due = 0;
u8 g_sensor_ui_inited = 0;

SemaphoreHandle_t xArbMutex = NULL;

static TaskHandle_t s_ui_task_handle = NULL;

void MotionUI_SetUiTaskHandle(TaskHandle_t h)
{
	s_ui_task_handle = h;
}

void MotionUI_NotifySensorFrame(void)
{
	if(s_ui_task_handle)
		xTaskNotifyGive(s_ui_task_handle);
}

void MotionUI_SharedInit(void)
{
	xArbMutex = xSemaphoreCreateMutex();
}

void DistSnapshot_Write(u16 f, u16 b, u16 l, u16 r, u16 n)
{
	taskENTER_CRITICAL();
	g_front_dist_mm = f;
	g_back_dist_mm = b;
	g_left_dist_mm = l;
	g_right_dist_mm = r;
	g_nearest_dist_mm = n;
	taskEXIT_CRITICAL();
}

void DistSnapshot_Read(u16 *f, u16 *b, u16 *l, u16 *r, u16 *n)
{
	taskENTER_CRITICAL();
	if(f) *f = g_front_dist_mm;
	if(b) *b = g_back_dist_mm;
	if(l) *l = g_left_dist_mm;
	if(r) *r = g_right_dist_mm;
	if(n) *n = g_nearest_dist_mm;
	taskEXIT_CRITICAL();
}

void App_ArbiterLock(void)
{
	if(xArbMutex)
		xSemaphoreTake(xArbMutex, portMAX_DELAY);
}

void App_ArbiterUnlock(void)
{
	if(xArbMutex)
		xSemaphoreGive(xArbMutex);
}
