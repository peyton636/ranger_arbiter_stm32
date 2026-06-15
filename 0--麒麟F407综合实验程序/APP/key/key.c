#include "key.h"
#include "SysTick.h"

/*******************************************************************************
* �� �� ��         : KEY_Init
* ��������		   : ������ʼ��
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void KEY_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; //����ṹ�����	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOA,ENABLE); //ʹ�ܶ˿�ʱ��
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN; //����ģʽ
	GPIO_InitStructure.GPIO_Pin=KEY0_PIN|KEY1_PIN|KEY2_PIN;//�ܽ�����
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//����
	GPIO_Init(KEY_PORT,&GPIO_InitStructure); //��ʼ���ṹ��
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN; //����ģʽ
	GPIO_InitStructure.GPIO_Pin=KEY_UP_PIN;//�ܽ�����
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;//����
	GPIO_Init(KEY_UP_PORT,&GPIO_InitStructure); //��ʼ���ṹ��
}

/*******************************************************************************
* �� �� ��         : KEY_Scan
* ��������		   : ����ɨ����
* ��    ��         : mode=0:���ΰ��°���
					 mode=1���������°���
* ��    ��         : 0��δ�а�������
					 KEY_UP_PRESS��KEY_UP������
					 KEY0_PRESS��KEY0������
					 KEY1_PRESS��KEY1������
					 KEY2_PRESS��KEY2������
*******************************************************************************/
u8 KEY_Scan(u8 mode)
{
	static u8 key=1;
	
	if(mode==1) //������������
		key=1;
	if(key==1&&(KEY_UP==1||KEY0==0||KEY1==0||KEY2==0)) //����һ����������
	{
		delay_ms(10);  //����
		key=0;
		if(KEY_UP==1)
			return KEY_UP_PRESS; 
		else if(KEY0==0)
			return KEY0_PRESS; 
		else if(KEY1==0)
			return KEY1_PRESS; 
		else if(KEY2==0)
			return KEY2_PRESS; 
	}
	else if(KEY0==1&&KEY1==1&&KEY2==1)    /* PE �������ͷż����ٴδ����������� KEY_UP�� */
		key=1;
	return 0;
}
