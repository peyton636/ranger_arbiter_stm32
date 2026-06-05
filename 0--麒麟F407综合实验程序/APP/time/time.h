#ifndef _time_H
#define _time_H

#include "system.h"

#define LOG_TICK_MS        20u   /* TIM4 周期，OSTime 每 tick +1 */
#define LOG_TIM4_PERIOD    201u  /* TIM4_Init(200,8399) 的 ARR+1，用于子 tick 毫秒 */
#define LOG_TS_WRAP_MS     1000000u  /* 时间戳 6 位：999.999s 后归零 */

extern volatile u32 OSTime;

void TIM2_Init(u16 per,u16 psc);
void TIM4_Init(u16 per,u16 psc);
void TIM3_Init(u16 per,u16 psc);
void TIM6_Init(u16 per,u16 psc);

u32 Log_GetUptimeMs(void);
void Log_TsPrefix(char *buf, u16 buf_len);  /* 写入 "[T123456] " = 123.456s */

#endif
