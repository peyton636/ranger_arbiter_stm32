#ifndef __usart_H
#define __usart_H

#include "system.h" 
#include "stdio.h"

#define USART1_REC_LEN		200  	//??????????????? 200

extern u8  USART1_RX_BUF[USART1_REC_LEN]; //???????,???USART_REC_LEN?????.????????��? 
extern u16 USART1_RX_STA;         		//?????????

void USART1_Init(u32 bound);

void Usart_PrintMutexInit(void);
void Usart_PrintLock(void);
void Usart_PrintUnlock(void);
/* 非阻塞写调试口：拿不到锁或 TX 满则丢弃，避免 printf 永久拖死任务 */
u8 Usart_TryWriteStr(const char *s);

#endif


