#ifndef _rs232_H
#define _rs232_H

#include "system.h" 

#define RS232_ENABLE
#define RS232_REC_LEN  			200  	//定义最大接收字节数 200
	  	
extern u8  RS232_RX_BUF[RS232_REC_LEN]; //接收缓冲,最大RS232_REC_LEN个字节.末字节为换行符 
extern u16 RS232_RX_STA;         		//接收状态标记	

void RS232_Init(u32 bound);
void RS232_SendString(u8 *buf);

#endif
