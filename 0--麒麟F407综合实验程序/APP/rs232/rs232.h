#ifndef _rs232_H
#define _rs232_H

#include "system.h" 

//#define RS232_ENABLE
#define RS232_REC_LEN  			200  	//�����������ֽ��� 200
	  	
extern u8  RS232_RX_BUF[RS232_REC_LEN]; //���ջ���,���RS232_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 RS232_RX_STA;         		//����״̬���	

void RS232_Init(u32 bound);
void RS232_SendString(u8 *buf);

#endif
