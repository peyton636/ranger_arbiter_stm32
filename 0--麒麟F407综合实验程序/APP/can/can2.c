#include "can2.h"

/* STM32F407��CAN2 ���� CAN1 ?�?��?��� bank14~27 ?�?��� CAN2 */

void CAN2_Mode_Init(u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = ENABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = ENABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = mode;
	CAN_InitStructure.CAN_SJW = tsjw;
	CAN_InitStructure.CAN_BS1 = tbs1;
	CAN_InitStructure.CAN_BS2 = tbs2;
	CAN_InitStructure.CAN_Prescaler = brp;
	CAN_Init(CAN2, &CAN_InitStructure);

	/* �?��� 14������ Jetson ���� 0x101����??�� */
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (0x7FFu << 5);
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;

	CAN_FilterInitStructure.CAN_FilterNumber = 14;
	CAN_FilterInitStructure.CAN_FilterIdHigh = (JETSON_CAN_ID_DOWN << 5);
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInit(&CAN_FilterInitStructure);

	CAN_FilterInitStructure.CAN_FilterNumber = 15;
	CAN_FilterInitStructure.CAN_FilterIdHigh = (JETSON_CAN_ID_TIME_REQ << 5);
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInit(&CAN_FilterInitStructure);

	CAN_FilterInitStructure.CAN_FilterNumber = 16;
	CAN_FilterInitStructure.CAN_FilterIdHigh = (JETSON_CAN_ID_STATUS_REQ << 5);
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInit(&CAN_FilterInitStructure);
}

void CAN2_Init_Jetson(void)
{
	/* �� CAN1 ��������??��APB1=42MHz �� 500 kbps */
	CAN2_Mode_Init(CAN_SJW_1tq, CAN_BS2_5tq, CAN_BS1_6tq, 7, CAN_Mode_Normal);
}

u8 CAN2_Send_Msg_WithID(u32 id, u8 *msg, u8 len)
{
	u8 mbox;
	u16 i = 0;
	CanTxMsg TxMessage;

	if(len > 8)
		return 1;

	TxMessage.StdId = id;
	TxMessage.ExtId = 0x00;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = len;

	for(i = 0; i < len; i++)
		TxMessage.Data[i] = msg[i];

	mbox = CAN_Transmit(CAN2, &TxMessage);
	i = 0;
	while((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0xFFF))
		i++;

	if(i >= 0xFFF)
		return 1;
	return 0;
}

u8 CAN2_Receive_Msg_WithID(u32 *id, u8 *buf)
{
	u32 i;
	CanRxMsg RxMessage;

	if(CAN_MessagePending(CAN2, CAN_FIFO0) == 0)
		return 0;

	CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);
	*id = RxMessage.StdId;

	for(i = 0; i < RxMessage.DLC; i++)
		buf[i] = RxMessage.Data[i];

	return RxMessage.DLC;
}
