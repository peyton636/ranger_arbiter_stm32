#ifndef _SENSOR_UI_H
#define _SENSOR_UI_H

#include "system.h"
#include "distance_sensor.h"

#define UI_X          5
#define UI_FS         16
#define UI_Y_TITLE    20
#define UI_Y_COUNT    42
#define UI_Y_DIST_HDR 64
#define UI_Y_IF1      80
#define UI_Y_IF2      96
#define UI_Y_IF3      112
#define UI_Y_IF4      128
#define UI_Y_CHASSIS  144
#define UI_Y_MOTION_FB  160
#define UI_Y_MOTION_CMD 176
#define UI_Y_WHEELS1  192
#define UI_Y_WHEELS2  208
#define UI_Y_BATT     224
#define UI_Y_BEEP     240
#define UI_Y_GPS_HDR  272   /* 在 Beep(240)/BMS(256) 之下 */
#define UI_Y_GPS1     288
#define UI_Y_GPS2     304
#define UI_X_VAL      45
#define UI_LINE_W     230

void SensorUI_DrawStatic(void);
void SensorUI_UpdateBeepStatus(void);
void SensorUI_UpdateForceStopBanner(void);
void SensorUI_UpdateCount(u32 cnt);
void SensorUI_UpdateDistances(DistanceSensor_Data *ds);
void SensorUI_UpdateGps(void);
void Chassis_UpdateOnLCD(void);

#endif
