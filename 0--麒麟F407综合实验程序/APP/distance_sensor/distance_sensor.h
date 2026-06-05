#ifndef __DISTANCE_SENSOR_H
#define __DISTANCE_SENSOR_H

#include "system.h"

#define DS_FRAME_LEN    10
#define DS_HEADER       0xFF

#define DS_ERR_NONE     0
#define DS_ERR_TIMEOUT  1
#define DS_ERR_CHKFAIL  2

/*
 * 四路距离语义 (mm)，与 E08 转接模组 UART 帧一致：
 *   0xFFFF (65535)  模组连续 3 次触发超时 → DS_ERR_TIMEOUT → FAILSAFE(0)
 *   0xEEEE (61166)  连续 3 次校验失败   → DS_ERR_CHKFAIL → FAILSAFE(0)
 *   DS_DIST_UNKNOWN 0xFFFF  本机尚未收到该路滤波输出
 *   有效测量          1 .. DS_DIST_MAX_MM（帧格式最大 65535，实际量程看探头）
 */
#define DS_DIST_UNKNOWN       0xFFFFu
#define DS_DIST_FAILSAFE_MM   0u
#define DS_DIST_MAX_MM        59999u

/* TIM4 每 20ms 调一次 DistanceSensor_Process
 * E08 轮询模式整轮约 170~250ms(49ms+4×探头)；触发周期须 ≥ 整轮且为 20ms 整数倍
 * 原 150/20=7 → 实际 140ms，短于模组周期，易打断 UART 收帧 → IF4 TIMEOUT */
#define DS_PROCESS_MS             20u
#define DS_TRIG_EVERY_N_TICKS     12u   /* 12×20ms = 240ms */
#define DS_TRIG_INTERVAL_MS       (DS_TRIG_EVERY_N_TICKS * DS_PROCESS_MS)

/* 运动场景：最近 5 帧、2s 内中位数；≥2 样点即输出，单帧相对中位数突跳>800 才丢 */
#define DS_FILTER_WINDOW_MS       2000u
#define DS_FILTER_WINDOW_TICKS    (DS_FILTER_WINDOW_MS / DS_PROCESS_MS)
#define DS_FILTER_MIN_SAMPLES     2u
#define DS_FILTER_MAX_SAMPLES     5u

#define DS_DIST_MIN_VALID_MM      30u
#define DS_SPIKE_ABS_MAX_MM       3000u
#define DS_FILTER_SPIKE_JUMP_MM   800u
#define DS_FILTER_MOTION_FLUSH_MM 150u   /* 相对 stable 变化超过此值则清窗，快速跟踪运动 */
#define DS_FILTER_MAX_STEP_MM     1200u  /* 单帧相对 stable 远离超过此值视为野值 */
#define DS_RX_TIMEOUT_MS          200u   /* 与模组 0x0215 默认 0x14×10ms 一致 */
#define DS_RX_TIMEOUT_TICKS         (DS_RX_TIMEOUT_MS / DS_PROCESS_MS)

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
void DistanceSensor_PrintStatus(void);
u16 DistanceSensor_GetNearestDistance(void);
u16 DistanceSensor_NormalizedMm(u8 idx);
u16 DistanceSensor_MinDistMm(void);
u16 DistanceSensor_GetFilteredMm(u8 idx);
u16 DistanceSensor_GetFilteredMinDistMm(void);
u16 DistanceSensor_GetArbiterMm(u8 idx);
void DistanceSensor_DrainLog(void);
void DistanceSensor_UpdateBuzzer(void);
void USART3_IRQHandler(void);

#endif
