#ifndef _MOTION_UI_SHARED_H
#define _MOTION_UI_SHARED_H

#include "system.h"
#include "distance_sensor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define DS_IDX_FRONT   0
#define DS_IDX_BACK    1
#define DS_IDX_LEFT    2
#define DS_IDX_RIGHT   3

#define UI_CAN_LCD_DIV 30

extern u16 g_front_dist_mm;
extern u16 g_back_dist_mm;
extern u16 g_left_dist_mm;
extern u16 g_right_dist_mm;
extern u16 g_nearest_dist_mm;

extern u8 g_beep_dist_enable;
extern u8 g_force_stop_enable;
extern volatile u8 g_beep_ui_dirty;
extern volatile u8 g_force_stop_ui_dirty;
extern volatile u8 g_sensor_updated;
extern volatile u8 g_can_lcd_due;
extern u8 g_sensor_ui_inited;

extern SemaphoreHandle_t xArbMutex;

void MotionUI_SetUiTaskHandle(TaskHandle_t h);
void MotionUI_NotifySensorFrame(void);

void MotionUI_SharedInit(void);
void DistSnapshot_Write(u16 f, u16 b, u16 l, u16 r, u16 n);
void DistSnapshot_Read(u16 *f, u16 *b, u16 *l, u16 *r, u16 *n);

void App_ArbiterLock(void);
void App_ArbiterUnlock(void);

#endif
