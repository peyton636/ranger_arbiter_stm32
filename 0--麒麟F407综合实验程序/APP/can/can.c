#include "can.h"
#include "usart.h"

//CAN初始化
//tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:时间段2的时间单元.   范围:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:时间段1的时间单元.   范围:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :波特率分频器.范围:1~1024; tq=(brp)*tpclk1
//波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
void CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	
	//使能相关时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PORTA时钟	                   											 
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	
	//引脚复用映射配置
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11复用为CAN1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12复用为CAN1
		
	//初始化GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA11,PA12
	
	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;	//使用报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1
	
	//配置过滤器
 	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000; //32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000; //32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
}

//发送指定ID的CAN消息
//id: CAN ID (11位标准帧)
//len:数据长度(最大为8)
//msg:数据指针
//返回值:0,成功;其他,失败;
u8 CAN1_Send_Msg_WithID(u32 id, u8* msg, u8 len)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	
	TxMessage.StdId = id;       // 标准标识符
	TxMessage.ExtId = 0x00;     // 扩展标识符
	TxMessage.IDE = CAN_Id_Standard;  // 使用标准标识符(11位)
	TxMessage.RTR = CAN_RTR_Data;     // 数据帧
	TxMessage.DLC = len;                // 数据长度
	
	for(i=0; i<len; i++)
		TxMessage.Data[i] = msg[i];
	
	mbox = CAN_Transmit(CAN1, &TxMessage);   
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
		i++;
	
	if(i >= 0XFFF) return 1;
	return 0;		
}

//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)
//保持兼容旧接口
u8 CAN1_Send_Msg(u8* msg, u8 len)
{
	return CAN1_Send_Msg_WithID(0x12, msg, len);
}

//接收CAN消息（带ID返回）
//id: 输出参数，接收到的CAN ID
//buf:数据缓存区
//返回值:0,无数据被收到;其他,接收的数据长度;
u8 CAN1_Receive_Msg_WithID(u32 *id, u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
	
    if(CAN_MessagePending(CAN1, CAN_FIFO0) == 0)
		return 0;		//没有接收到数据,直接退出 
	
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
    
    *id = RxMessage.StdId;  // 返回CAN ID
    
    for(i=0; i<RxMessage.DLC; i++)
        buf[i] = RxMessage.Data[i];  
    
	return RxMessage.DLC;	
}

//can口接收数据查询（保持兼容）
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if(CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
    for(i=0; i<RxMessage.DLC; i++)
        buf[i] = RxMessage.Data[i];  
	return RxMessage.DLC;	
}

// RANGER MINI 3.0 专用初始化函数
// 配置：APB1=42MHz, 500kbps
// SJW=1, BS1=6, BS2=5, Prescaler=7
// 计算：42MHz / (7 × (6+5+1)) = 500kbps
void CAN1_Init_RangerMini(void)
{
	CAN1_Mode_Init(CAN_SJW_1tq, CAN_BS2_5tq, CAN_BS1_6tq, 7, CAN_Mode_Normal);
}

// 发送运动控制指令 (0x111帧)
// speed_mm_s: 速度，单位mm/s
// angle_rad: 转向角度，单位rad
// 返回值: 0=成功, 1=失败
u8 CAN1_Send_ControlCmd(u16 speed_mm_s, float angle_rad)
{
	u8 txbuf[8] = {0};
	s16 angle_scaled;
	
	// 速度：byte[0-1]，Motorola格式（大端序）
	txbuf[0] = (speed_mm_s >> 8) & 0xFF;
	txbuf[1] = speed_mm_s & 0xFF;
	
	// 转角：byte[6-7]，Motorola格式（大端序）
	// 角度单位：rad × 1000
	angle_scaled = (s16)(angle_rad * 1000.0f);
	txbuf[6] = (angle_scaled >> 8) & 0xFF;
	txbuf[7] = angle_scaled & 0xFF;
	
	// byte[2-5] 保留为0
	
	return CAN1_Send_Msg_WithID(CAN_ID_CTRL_CMD, txbuf, 8);
}

// 发送控制模式设定 (0x421帧)
// mode: 0=遥控模式, 1=CAN指令模式
// 返回值: 0=成功, 1=失败
u8 CAN1_Send_ModeSet(u8 mode)
{
	u8 txbuf[1];
	txbuf[0] = mode;
	return CAN1_Send_Msg_WithID(CAN_ID_MODE_SET, txbuf, 1);
}

// 发送错误清除指令 (0x441帧)
// 返回值: 0=成功, 1=失败
u8 CAN1_Send_ErrorClear(void)
{
	u8 txbuf[1] = {0};
	return CAN1_Send_Msg_WithID(CAN_ID_ERR_CLEAR, txbuf, 1);
}
