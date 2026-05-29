#include "usart3.h"
#include "arbiter.h"
#include "stm32f4xx.h"

// 接收缓冲区
static u8 usart3_rx_buf[32];
static u8 usart3_rx_idx = 0;
static u8 usart3_frame_ready = 0;
static u8 usart3_complete_frame[JETSON_FRAME_LEN];

/*******************************************************************************
* 函 数 名         : USART3_Init
* 功能描述		   : USART2初始化（PA2-TX, PA3-RX）用于连接Jetson
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void USART3_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// 使能时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  // GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // USART2时钟

	// 配置PA2和PA3为USART2复用功能
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); // PA2 -> USART2_TX
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); // PA3 -> USART2_RX

	// 配置GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 配置USART2
	USART_InitStructure.USART_BaudRate = USART3_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	// 配置 USART2 中断
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// 启用 USART2 接收中断
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	// 启用USART2
	USART_Cmd(USART2, ENABLE);
}

/*******************************************************************************
* 函 数 名         : USART3_SendByte
* 功能描述		   : 发送单个字节
* 输    入         : data: 要发送的字节
* 输    出         : 无
*******************************************************************************/
void USART3_SendByte(u8 data)
{
	USART_SendData(USART2, data);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

/*******************************************************************************
* 函 数 名         : USART3_SendData
* 功能描述		   : 发送多个字节
* 输    入         : data: 数据指针, len: 数据长度
* 输    出         : 无
*******************************************************************************/
void USART3_SendData(u8* data, u16 len)
{
	while(len--)
	{
		USART3_SendByte(*data++);
	}
}

/*******************************************************************************
* 函 数 名         : USART3_SendSensorData
* 功能描述		   : 发送传感器数据到Jetson
* 输    入         : dist: 4个传感器距离数组(mm), beep_duty: 蜂鸣器占空比(0-100)
* 输    出         : 无
* 数据格式         : [0xFF][传感器数][距离1高][距离1低][距离2高][距离2低][距离3高][距离3低][距离4高][距离4低][蜂鸣器][校验和]
*******************************************************************************/
void USART3_SendSensorData(u16* dist, u8 beep_duty)
{
	u8 frame[USART3_FRAME_LEN];
	u8 checksum = 0;
	u8 i;

	// 帧头
	frame[0] = USART3_FRAME_HEADER;
	
	// 传感器数量
	frame[1] = 4;
	
	// 4个传感器距离数据（大端序）
	for(i = 0; i < 4; i++)
	{
		frame[2 + i*2] = (dist[i] >> 8) & 0xFF;
		frame[3 + i*2] = dist[i] & 0xFF;
	}
	
	// 蜂鸣器占空比
	frame[10] = beep_duty;
	
	// 计算校验和（除校验和字节外的所有字节异或）
	for(i = 0; i < USART3_FRAME_LEN - 1; i++)
	{
		checksum ^= frame[i];
	}
	frame[11] = checksum;
	
	// 发送数据
	USART3_SendData(frame, USART3_FRAME_LEN);
}

/*******************************************************************************
* 函 数 名         : USART3_ProcessRxByte
* 功能描述		   : 处理接收到的单个字节（在中断中调用）
* 输    入         : byte: 接收到的字节
* 输    出         : 无
*******************************************************************************/
void USART3_ProcessRxByte(u8 byte)
{
	// 查找帧头
	if(usart3_rx_idx == 0)
	{
		if(byte == JETSON_FRAME_HEADER)
		{
			usart3_rx_buf[0] = byte;
			usart3_rx_idx = 1;
		}
	}
	else
	{
		usart3_rx_buf[usart3_rx_idx] = byte;
		usart3_rx_idx++;
		
		// 接收完整帧
		if(usart3_rx_idx >= JETSON_FRAME_LEN)
		{
			// 复制完整帧
			u8 i;
			for(i = 0; i < JETSON_FRAME_LEN; i++)
			{
				usart3_complete_frame[i] = usart3_rx_buf[i];
			}
			usart3_frame_ready = 1;
			usart3_rx_idx = 0;
		}
	}
}

/*******************************************************************************
* 函 数 名         : USART3_GetJetsonFrame
* 功能描述		   : 获取Jetson指令帧
* 输    入         : frame: 输出缓冲区
* 输    出         : 0=无数据, 1=有完整帧
*******************************************************************************/
u8 USART3_GetJetsonFrame(u8* frame)
{
	if(usart3_frame_ready)
	{
		u8 i;
		for(i = 0; i < JETSON_FRAME_LEN; i++)
		{
			frame[i] = usart3_complete_frame[i];
		}
		usart3_frame_ready = 0;
		return 1;
	}
	return 0;
}

/*******************************************************************************
* 函 数 名         : USART2_IRQHandler
* 功能描述		   : USART2中断服务函数（接收Jetson数据）
*******************************************************************************/
void USART2_IRQHandler(void)
{
	u8 byte;
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		byte = USART_ReceiveData(USART2);
		USART3_ProcessRxByte(byte);
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
}