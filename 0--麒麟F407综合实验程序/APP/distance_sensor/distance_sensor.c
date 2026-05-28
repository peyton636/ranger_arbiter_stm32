#include "distance_sensor.h"
#include "usart.h"
#include "stdio.h"

static DistanceSensor_Data ds_data;
static u8 ds_rx_buf[DS_FRAME_LEN];
static u8 ds_rx_idx = 0;
static u8 ds_frame_ready = 0;

void DistanceSensor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	// 配置 UART2 中断
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// 启用 UART2 接收中断
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART2, ENABLE);

	ds_rx_idx = 0;
	ds_frame_ready = 0;
	ds_data.valid = 0;
}

void DistanceSensor_Process(void)
{
	u8 i;
	GPIO_InitTypeDef gpio;
	static u32 tick = 0;
	static u8 trig_pending = 0;
	static u8 rx_idle_cnt = 0;

	tick++;

	if(tick % 100 == 0)  // 每2秒触发一次
	{
		ds_rx_idx = 0;
		rx_idle_cnt = 0;
		// PA2配置为输出模式并拉低（下降沿触发）
		USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);  // 禁用中断
		gpio.GPIO_Pin = GPIO_Pin_2;
		gpio.GPIO_Mode = GPIO_Mode_OUT;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_50MHz;
		gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA, &gpio);
		GPIO_ResetBits(GPIOA, GPIO_Pin_2);
		trig_pending = 1;
	}
	else if(trig_pending)  // 恢复PA2为UART功能
	{
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
		gpio.GPIO_Pin = GPIO_Pin_2;
		gpio.GPIO_Mode = GPIO_Mode_AF;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_50MHz;
		gpio.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOA, &gpio);
		trig_pending = 0;
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);  // 重新启用中断
		printf("[DS] TRIG pulse done\r\n");
	}

	// 接收超时处理（200ms无数据则重置）
	if(ds_rx_idx > 0)
	{
		rx_idle_cnt++;
		if(rx_idle_cnt > 10)  // 10*20ms=200ms超时
		{
			printf("[DS] RX timeout, reset idx=%d\r\n", ds_rx_idx);
			ds_rx_idx = 0;
		}
	}
}

// UART2 中断服务函数 - 实时接收传感器数据
void USART2_IRQHandler(void)
{
	u8 byte;
	u16 sum;
	u8 i;

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		byte = USART_ReceiveData(USART2);

		if(ds_rx_idx == 0)
		{
			if(byte == DS_HEADER)
			{
				ds_rx_buf[0] = byte;
				ds_rx_idx = 1;
			}
		}
		else
		{
			ds_rx_buf[ds_rx_idx] = byte;
			ds_rx_idx++;

			if(ds_rx_idx >= DS_FRAME_LEN)
			{
				sum = 0;
				for(i = 0; i < DS_FRAME_LEN - 1; i++)
				{
					sum += ds_rx_buf[i];
				}

				if((sum & 0xFF) == ds_rx_buf[DS_FRAME_LEN - 1])
				{
					ds_data.valid = 1;
					ds_data.dist[0] = (ds_rx_buf[1] << 8) | ds_rx_buf[2];
					ds_data.dist[1] = (ds_rx_buf[3] << 8) | ds_rx_buf[4];
					ds_data.dist[2] = (ds_rx_buf[5] << 8) | ds_rx_buf[6];
					ds_data.dist[3] = (ds_rx_buf[7] << 8) | ds_rx_buf[8];

					for(i = 0; i < 4; i++)
					{
						if(ds_data.dist[i] == 0xFFFF)
							ds_data.error[i] = DS_ERR_TIMEOUT;
						else if(ds_data.dist[i] == 0xEEEE)
							ds_data.error[i] = DS_ERR_CHKFAIL;
						else
							ds_data.error[i] = DS_ERR_NONE;
					}

					ds_frame_ready = 1;
					DistanceSensor_Print();
				}
				ds_rx_idx = 0;
			}
		}
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
}

DistanceSensor_Data* DistanceSensor_GetData(void)
{
	return &ds_data;
}

u8 DistanceSensor_NewData(void)
{
	if(ds_frame_ready)
	{
		ds_frame_ready = 0;
		return 1;
	}
	return 0;
}

void DistanceSensor_Print(void)
{
	u8 i;

	if(!ds_data.valid)
	{
		printf("[DS] Waiting for data...\r\n");
		return;
	}

	for(i = 0; i < 4; i++)
	{
		if(ds_data.error[i] == DS_ERR_TIMEOUT)
		{
			printf("[DS] IF%d: TIMEOUT\r\n", i + 1);
		}
		else if(ds_data.error[i] == DS_ERR_CHKFAIL)
		{
			printf("[DS] IF%d: CHK ERR\r\n", i + 1);
		}
		else
		{
			printf("[DS] IF%d: %d mm\r\n", i + 1, ds_data.dist[i]);
		}
	}
	printf("---\r\n");
}
