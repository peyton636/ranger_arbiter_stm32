#ifndef __DISTANCE_SENSOR_H
#define __DISTANCE_SENSOR_H

#include "system.h"

#define DS_FRAME_LEN    10
#define DS_HEADER       0xFF

#define DS_ERR_NONE     0
#define DS_ERR_TIMEOUT  1
#define DS_ERR_CHKFAIL  2

typedef struct {
	u16 dist[4];
	u8  valid;
	u8  error[4];
} DistanceSensor_Data;

void DistanceSensor_Init(void);
void DistanceSensor_Process(void);
DistanceSensor_Data* DistanceSensor_GetData(void);
u8 DistanceSensor_NewData(void);
void DistanceSensor_Print(void);
void USART2_IRQHandler(void);  // UART2中断服务函数声明

#endif