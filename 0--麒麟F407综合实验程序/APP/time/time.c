#include "time.h"
#include "distance_sensor.h"
#include "can.h"
#include "led.h"
#include "key.h"
#include "usart.h"
#include "sdio_sdcard.h"
#include "SysTick.h"
#include "camera_app.h"
#include "beep.h"
#include "piclib.h"

/*******************************************************************************
* �� �� ��         : TIM4_Init
* ��������		   : TIM4��ʼ������
* ��    ��         : per:��װ��ֵ
					 psc:��Ƶϵ��
* ��    ��         : ��
*******************************************************************************/
void TIM4_Init(u16 per,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//ʹ��TIM4ʱ��
	
	TIM_TimeBaseInitStructure.TIM_Period=per;   //�Զ�װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc; //��Ƶϵ��
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //�������ϼ���ģʽ
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //������ʱ���ж�
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;//��ʱ���ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	
	
	TIM_Cmd(TIM4,ENABLE); //ʹ�ܶ�ʱ��	
}

volatile  u32  OSTime;
/*******************************************************************************
* �� �� ��         : TIM4_IRQHandler
* ��������		   : TIM4�жϺ���
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void TIM4_IRQHandler(void)
{
	u32 can_id;
	u8 can_buf[8];
	u8 can_len;
	u8 ci;
	static u32 can_tick = 0;

	if(TIM_GetITStatus(TIM4,TIM_IT_Update))
	{
//		LED2=!LED2;		
		OSTime++;
		DistanceSensor_Process();

		can_len = CAN1_Receive_Msg_WithID(&can_id, can_buf);
		if(can_len)
		{
			printf("[CAN RX] 0x%03X:", (unsigned int)can_id);
			for(ci = 0; ci < can_len; ci++)
				printf(" %02X", can_buf[ci]);
			printf("\r\n");
		}

		can_tick++;
		if(can_tick % 500 == 0)
		{
			u8 test_data = (u8)(can_tick / 500);
			u8 send_result;
			
			send_result = CAN1_Send_Msg_WithID(0x7FF, &test_data, 1);
			
			printf("[CAN DIAG] MCR=%08X MSR=%08X RF0R=%08X RF1R=%08X ESR=%08X SEND=%d\r\n",
				(unsigned int)CAN1->MCR,
				(unsigned int)CAN1->MSR,
				(unsigned int)CAN1->RF0R,
				(unsigned int)CAN1->RF1R,
				(unsigned int)CAN1->ESR,
				send_result);
		}
	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);	
}



extern u32 lwip_localtime;	//lwip����ʱ�������,��λ:ms
//ͨ�ö�ʱ��3�жϳ�ʼ��
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ��3!
void TIM3_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///ʹ��TIM3ʱ��
	
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //������ʱ��3�����ж�
	TIM_Cmd(TIM3,ENABLE); //ʹ�ܶ�ʱ��3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

//��ʱ��3�жϷ�����
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //����ж�
	{
		lwip_localtime +=10; //��10
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //����жϱ�־λ
}


u8 sd_ok=1;				//0,sd��������;1,SD������
void TIM2_Init(u16 per,u16 psc)
{
    u8 res=0;
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʱ��ʹ��
	
	//��ʱ��TIM��ʼ��
	TIM_TimeBaseInitStructure.TIM_Period=per;   //�Զ�װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc; //��Ƶϵ��
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //�������ϼ���ģʽ
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig( TIM2,TIM_IT_Update|TIM_IT_Trigger,ENABLE);//ʹ�ܶ�ʱ��2���´����ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //��ռ���ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //�����ȼ�1��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���
	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIMx	

	res=f_mkdir("0:/PHOTO");		//����PHOTO�ļ���
	if(res!=FR_EXIST&&res!=FR_OK) 	//�����˴���
	{		    
		LED2=0;
		sd_ok=0; 
	}
	else
	{
		LED2=1;
		sd_ok=1;   
	}		
}

void camera_new_pathname(u8 *pname,u8 mode);
//��ʱ��2�жϷ������	 
void TIM2_IRQHandler(void)
{ 	
	u8 *pname;				//��·�����ļ���
	
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
	{
		
		pname=mymalloc(SRAMIN,30);	//Ϊ��·�����ļ�������30���ֽڵ��ڴ�		    
		while(pname==NULL)			//�ڴ�������
		{	    
			LED2=!LED2;
			delay_ms(100);				  
		}
		
		if(KEY_Scan(1)==KEY0_PRESS&&sd_ok)
		{
			LED2=0;	//����DS1,��ʾ��������
			camera_new_pathname(pname,0);//�õ��ļ���
			if(bmp_encode(pname,0,0,tftlcd_data.width,tftlcd_data.height,0))//��������
			{
				LED2=1;				 
			}
			else
			{
				//�������̽У���ʾ�������
				BEEP=1;
				delay_ms(300);
				BEEP=0;
				LED2=1;
			}
		}
	} 
	myfree(SRAMIN,pname);
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //���TIMx���жϴ�����λ:TIM �ж�Դ 			    		  			    	    
}


u16 frame;
vu8 frameup; 
//��ʱ��6�жϷ������	 
void TIM6_DAC_IRQHandler(void)
{ 		    		  			    
	if(TIM_GetITStatus(TIM6,TIM_IT_Update)==SET) //����ж�
	{ 
		frameup=1;
	}				   
	TIM_ClearITPendingBit(TIM6,TIM_IT_Update);  //����жϱ�־λ    
}
//ͨ�ö�ʱ��6�жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ42M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ��3!
void TIM6_Init(u16 arr,u16 psc)
{	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);  ///ʹ��TIM6ʱ��
	
	TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE); //������ʱ��6�����ж�
	TIM_Cmd(TIM6,ENABLE); //ʹ�ܶ�ʱ��6
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM6_DAC_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure); 									 
}

