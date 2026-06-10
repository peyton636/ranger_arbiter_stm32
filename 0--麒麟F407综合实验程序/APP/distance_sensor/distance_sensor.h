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
 *   0xFFFF / >=60000 (如65533)  模组超时哨兵 → DS_ERR_TIMEOUT，内部存 FAILSAFE(0)
 *   0xEEEE (61166)              校验失败     → DS_ERR_CHKFAIL，内部存 FAILSAFE(0)
 *   0                           归一化后的 FAILSAFE，与 TIMEOUT 显示不同（见 FormatLane）
 *   DS_DIST_UNKNOWN             本机尚无该路有效输出
 *   有效测量                    DS_DIST_MIN_VALID_MM .. DS_DIST_VALID_MAX_MM
 */
#define DS_DIST_UNKNOWN         0xFFFFu
#define DS_DIST_FAILSAFE_MM     0u
#define DS_DIST_MOD_ERR_MARKER  0xEEEEu
#define DS_DIST_MOD_TIMEOUT_RAW 0xFFFFu
#define DS_DIST_VALID_MAX_MM    59999u  /* 与 docs/测距传感器数据流.md 一致：有效 30~59999 */
#define DS_DIST_MOD_TIMEOUT_MIN 60000u  /* 0xFFFF/65533 等哨兵区，非真实量程 */
#define DS_DIST_MAX_MM          DS_DIST_VALID_MAX_MM

/* TIM4 每 20ms 调一次 DistanceSensor_Process
 * E08 轮询整轮约 170~250ms；触发周期须 ≥ 整轮且为 20ms 整数倍 */
#define DS_PROCESS_MS             20u
#define DS_TRIG_EVERY_N_TICKS     14u   /* 280ms，减少触发打断 UART 收 IF4 */
#define DS_TRIG_INTERVAL_MS       (DS_TRIG_EVERY_N_TICKS * DS_PROCESS_MS)
/* E08 四路轮询整轮约 170~250ms；快触发须 ≥200ms，避免打断 IF4 收包 */
#define DS_BOOT_FAST_TRIG_TICKS   10u   /* 200ms */
#define DS_BOOT_FAST_TRIG_COUNT   6u
#define DS_FILTER_TIMEOUT_CLEAR_N 3u    /* 连续 N 次本路超时才清 stable */

#define DS_FILTER_WINDOW_MS       2000u
#define DS_FILTER_WINDOW_TICKS    (DS_FILTER_WINDOW_MS / DS_PROCESS_MS)
#define DS_FILTER_MIN_SAMPLES     1u
#define DS_FILTER_MAX_SAMPLES     5u

#define DS_DIST_MIN_VALID_MM      30u
#define DS_FILTER_SPIKE_JUMP_MM   600u  /* 同一路相邻有效帧突跳>600mm 视为抖动/错帧，拒收本帧 */
#define DS_FILTER_MOTION_FLUSH_MM 150u
#define DS_FILTER_MAX_STEP_MM     2000u /* 真实运动允许较大单帧变化 */
#define DS_RX_TIMEOUT_MS          280u
#define DS_RX_TIMEOUT_TICKS       (DS_RX_TIMEOUT_MS / DS_PROCESS_MS)

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
/* 统一显示：---/TO/ERR/0/N mm，LCD 与串口共用 */
void DistanceSensor_FormatLane(u8 idx, char *buf, u16 buf_sz);
void DistanceSensor_FormatFilteredLane(u8 idx, char *buf, u16 buf_sz);
void USART3_IRQHandler(void);

#endif
