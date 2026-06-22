#include "usart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
#pragma import(__use_no_semihosting)
struct __FILE
{
	int handle;
};
FILE __stdout;
FILE __stdin;
void _sys_exit(int x)
{
	(void)x;
	while(1);
}
#endif

static SemaphoreHandle_t s_print_mutex = NULL;

void Usart1_RawPutc(u8 ch)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		;
	USART_SendData(USART1, ch);
}

void Usart1_RawPuts(const char *s)
{
	if(!s)
		return;
	while(*s)
		Usart1_RawPutc((u8)*s++);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
		;
}

void USART1_Probe(const char *tag)
{
	Usart1_RawPuts("\r\n[U1 ");
	if(tag)
		Usart1_RawPuts(tag);
	Usart1_RawPuts("]\r\n");
}

static void Usart_PutByteTry(u8 ch)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		;
	USART_SendData(USART1, ch);
}

void Usart_PrintMutexInit(void)
{
	if(s_print_mutex == NULL)
		s_print_mutex = xSemaphoreCreateMutex();
}

void Usart_PrintLock(void)
{
	if(s_print_mutex != NULL &&
	   xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	{
		xSemaphoreTake(s_print_mutex, portMAX_DELAY);
	}
}

void Usart_PrintUnlock(void)
{
	if(s_print_mutex != NULL &&
	   xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	{
		xSemaphoreGive(s_print_mutex);
	}
}

u8 Usart_TryWriteStr(const char *s)
{
	u8 locked = 0;

	if(!s)
		return 0;

	if(s_print_mutex != NULL &&
	   xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	{
		if(xSemaphoreTake(s_print_mutex, pdMS_TO_TICKS(50)) != pdTRUE)
			return 0;
		locked = 1;
	}

	while(*s)
		Usart_PutByteTry((u8)*s++);

	if(locked)
		xSemaphoreGive(s_print_mutex);
	return 1;
}

int fputc(int ch, FILE *p)
{
	u8 locked = 0;

	(void)p;
	if(s_print_mutex != NULL &&
	   xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	{
		xSemaphoreTake(s_print_mutex, portMAX_DELAY);
		locked = 1;
	}

	Usart_PutByteTry((u8)ch);

	if(locked)
		xSemaphoreGive(s_print_mutex);
	return ch;
}

u8 USART1_RX_BUF[USART1_REC_LEN];
u16 USART1_RX_STA=0;

void USART1_Init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_Cmd(USART1, ENABLE);
	USART_ClearFlag(USART1, USART_FLAG_TC);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void USART1_IRQHandler(void)
{
	u8 r;

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		r = USART_ReceiveData(USART1);
		if((USART1_RX_STA & 0x8000) == 0)
		{
			if(USART1_RX_STA & 0x4000)
			{
				if(r != 0x0a)
					USART1_RX_STA = 0;
				else
					USART1_RX_STA |= 0x8000;
			}
			else
			{
				if(r == 0x0d)
					USART1_RX_STA |= 0x4000;
				else
				{
					USART1_RX_BUF[USART1_RX_STA & 0X3FFF] = r;
					USART1_RX_STA++;
					if(USART1_RX_STA > (USART1_REC_LEN - 1))
						USART1_RX_STA = 0;
				}
			}
		}
	}
}
