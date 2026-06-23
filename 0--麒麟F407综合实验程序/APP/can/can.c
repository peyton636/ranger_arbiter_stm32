#include "can.h"
#include "usart.h"

//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:ʱ���2��ʱ�䵥Ԫ.   ��Χ:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:ʱ���1��ʱ�䵥Ԫ.   ��Χ:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :�����ʷ�Ƶ��.��Χ:1~1024; tq=(brp)*tpclk1
//������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
void CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	
	//ʹ�����ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
	
	//���Ÿ���ӳ������
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11����ΪCAN1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12����ΪCAN1
		
	//��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��PA11,PA12
	
	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=ENABLE;	//�Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ����������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=ENABLE;	//ʹ�ñ����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=tsjw;	//����ͬ����Ծ����(Tsjw)Ϊtsjw+1��ʱ�䵥λ
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1
	
	//���ù�����
 	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000; //32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000; //32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
}

//����ָ��ID��CAN��Ϣ
//id: CAN ID (11λ��׼֡)
//len:���ݳ���(���Ϊ8)
//msg:����ָ��
//����ֵ:0,�ɹ�;����,ʧ��;
u8 CAN1_Send_Msg_WithID(u32 id, u8* msg, u8 len)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	
	TxMessage.StdId = id;       // ��׼��ʶ��
	TxMessage.ExtId = 0x00;     // ��չ��ʶ��
	TxMessage.IDE = CAN_Id_Standard;  // ʹ�ñ�׼��ʶ��(11λ)
	TxMessage.RTR = CAN_RTR_Data;     // ����֡
	TxMessage.DLC = len;                // ���ݳ���
	
	for(i=0; i<len; i++)
		TxMessage.Data[i] = msg[i];
	
	mbox = CAN_Transmit(CAN1, &TxMessage);   
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
		i++;
	
	if(i >= 0XFFF) return 1;
	return 0;		
}

//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)
//���ּ��ݾɽӿ�
u8 CAN1_Send_Msg(u8* msg, u8 len)
{
	return CAN1_Send_Msg_WithID(0x12, msg, len);
}

//����CAN��Ϣ����ID���أ�
//id: ������������յ���CAN ID
//buf:���ݻ�����
//����ֵ:0,�����ݱ��յ�;����,���յ����ݳ���;
u8 CAN1_Receive_Msg_WithID(u32 *id, u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
	
    if(CAN_MessagePending(CAN1, CAN_FIFO0) == 0)
		return 0;		//û�н��յ�����,ֱ���˳� 
	
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
    
    *id = RxMessage.StdId;  // ����CAN ID
    
    for(i=0; i<RxMessage.DLC; i++)
        buf[i] = RxMessage.Data[i];  
    
	return RxMessage.DLC;	
}

//can�ڽ������ݲ�ѯ�����ּ��ݣ�
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if(CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//��ȡ����	
    for(i=0; i<RxMessage.DLC; i++)
        buf[i] = RxMessage.Data[i];  
	return RxMessage.DLC;	
}

// RANGER MINI 3.0 ר�ó�ʼ������
// ���ã�APB1=42MHz, 500kbps
// SJW=1, BS1=6, BS2=5, Prescaler=7
// ���㣺42MHz / (7 �� (6+5+1)) = 500kbps
void CAN1_Init_RangerMini(void)
{
	CAN1_Mode_Init(CAN_SJW_1tq, CAN_BS2_5tq, CAN_BS1_6tq, 7, CAN_Mode_Normal);
}

// �����˶�����ָ�� (0x111֡)
// speed_mm_s: �ٶȣ���λmm/s
// angle_rad: ת��Ƕȣ���λrad
// ����ֵ: 0=�ɹ�, 1=ʧ��
u8 CAN1_Send_ControlCmd(u16 speed_mm_s, float angle_rad)
{
	u8 txbuf[8] = {0};
	s16 angle_scaled;
	
	// �ٶȣ�byte[0-1]��Motorola��ʽ�������
	txbuf[0] = (speed_mm_s >> 8) & 0xFF;
	txbuf[1] = speed_mm_s & 0xFF;
	
	// ת�ǣ�byte[6-7]��Motorola��ʽ�������
	// �Ƕȵ�λ��rad �� 1000
	angle_scaled = (s16)(angle_rad * 1000.0f);
	txbuf[6] = (angle_scaled >> 8) & 0xFF;
	txbuf[7] = angle_scaled & 0xFF;
	
	// byte[2-5] ����Ϊ0
	
	return CAN1_Send_Msg_WithID(CAN_ID_CTRL_CMD, txbuf, 8);
}

// ���Ϳ���ģʽ�趨 (0x421֡)
// mode: 0=ң��ģʽ, 1=CANָ��ģʽ
// ����ֵ: 0=�ɹ�, 1=ʧ��
u8 CAN1_Send_ModeSet(u8 mode)
{
	u8 txbuf[1];
	txbuf[0] = mode;
	return CAN1_Send_Msg_WithID(CAN_ID_MODE_SET, txbuf, 1);
}

// ���ʹ������ָ�� (0x441֡)
// ����ֵ: 0=�ɹ�, 1=ʧ��
u8 CAN1_Send_ErrorClear(void)
{
	u8 txbuf[1] = {0};
	return CAN1_Send_Msg_WithID(CAN_ID_ERR_CLEAR, txbuf, 1);
}
