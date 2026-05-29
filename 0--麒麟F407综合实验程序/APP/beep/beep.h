#ifndef _beep_H
#define _beep_H

#include "system.h"

/* 蜂鸣器时钟端口、引脚定义 */
#define BEEP_PORT 			GPIOF   
#define BEEP_PIN 			GPIO_Pin_8
#define BEEP_PORT_RCC		RCC_AHB1Periph_GPIOF

#define BEEP PFout(8)

void BEEP_Init(void);
void BEEP_GPIO_Init(void);
void BEEP_PWM_Init(void);
void BEEP_SetDuty(u8 duty);
void BEEP_SetVolume(u8 volume);
u8 BEEP_GetDuty(void);
void BEEP_Beep(u16 duration_ms);
void BEEP_Process(void);  // 定时器中断调用的处理函数

#endif
