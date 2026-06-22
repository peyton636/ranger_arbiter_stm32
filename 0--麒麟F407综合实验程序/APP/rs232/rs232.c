/* Jetson ????????? USART2??PA2-TX / PA3-RX?????? rs232.h */
#include "rs232.h"
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
#include "stdio.h"
#include "beep.h"
#include "can2.h"
#if JETSON_LINK_CAN
#include "jetson_can.h"
#endif
#if !JETSON_LINK_CAN
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "agv_blob_wire.h"
#if JETSON_USE_BLOB_V2
#include "agv_blob_rs232.h"
#endif
#endif

#define JETSON_RX_DEBUG 0

// ?????????
#define RS232_RX_MODE_IDLE  0
#define RS232_RX_MODE_V3    1
#define RS232_RX_MODE_SVC   2
#define RS232_RX_MODE_BLOB  3

static u8 rs232_rx_mode = RS232_RX_MODE_IDLE;
static u8 rs232_rx_buf[JETSON_V3_FRAME_LEN];
static u8 rs232_rx_idx = 0;
static u8 rs232_frame_ready = 0;
static u8 rs232_complete_frame[JETSON_V3_FRAME_LEN];
static u8 rs232_svc_ready = 0;
static u32 rs232_svc_can_id = 0;
static u8 rs232_svc_payload[8];
static u8 jetson_tx_seq = 0;

#if !JETSON_LINK_CAN
static volatile u32 s_usart2_rx_bytes = 0;
static volatile u32 s_usart2_ore_cnt = 0;
static volatile u32 s_usart2_ab_idle = 0;
static volatile u32 s_usart2_raw_ab = 0;
static volatile u32 s_usart2_raw_a5 = 0;
static volatile u32 s_usart2_raw_aa = 0;
static SemaphoreHandle_t s_usart2_tx_mutex = NULL;

static void rs232_TxLock(void)
{
	if(s_usart2_tx_mutex != NULL &&
	   xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	{
		xSemaphoreTake(s_usart2_tx_mutex, portMAX_DELAY);
	}
}

static void rs232_TxUnlock(void)
{
	if(s_usart2_tx_mutex != NULL &&
	   xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	{
		xSemaphoreGive(s_usart2_tx_mutex);
	}
}

void RS232_TxMutexInit(void)
{
	if(s_usart2_tx_mutex == NULL)
		s_usart2_tx_mutex = xSemaphoreCreateMutex();
}

#if JETSON_USART2_DMA_TX && JETSON_USE_BLOB_V2
#define JETSON_TX_DMA_MAX   128u

static u8 s_tx_dma_buf[JETSON_TX_DMA_MAX];
static u8 s_tx_pend_buf[JETSON_TX_DMA_MAX];
static volatile u16 s_tx_pend_len = 0;
static volatile u8 s_tx_pend_valid = 0;
static volatile u8 s_tx_dma_busy = 0;

#if JETSON_USART2_DMA_RX
#define JETSON_RX_DMA_SIZE  256u
static u8 s_rx_dma_buf[JETSON_RX_DMA_SIZE];
void RS232_ProcessRxByte(u8 byte);
#endif

static void rs232_TxDmaKick(const u8 *data, u16 len)
{
	u16 i;

	for(i = 0; i < len; i++)
		s_tx_dma_buf[i] = data[i];
	DMA_Cmd(DMA1_Stream6, DISABLE);
	while(DMA1_Stream6->CR & DMA_SxCR_EN)
		;
	DMA1_Stream6->M0AR = (u32)s_tx_dma_buf;
	DMA_SetCurrDataCounter(DMA1_Stream6, len);
	DMA_Cmd(DMA1_Stream6, ENABLE);
	s_tx_dma_busy = 1;
}

static void rs232_DmaInit(void)
{
	DMA_InitTypeDef dma;
	NVIC_InitTypeDef nvic;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

#if JETSON_USART2_DMA_RX
	DMA_DeInit(DMA1_Stream5);
	dma.DMA_Channel = DMA_Channel_4;
	dma.DMA_PeripheralBaseAddr = (u32)&USART2->DR;
	dma.DMA_Memory0BaseAddr = (u32)s_rx_dma_buf;
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dma.DMA_BufferSize = JETSON_RX_DMA_SIZE;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_Mode = DMA_Mode_Normal;
	dma.DMA_Priority = DMA_Priority_VeryHigh;
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream5, &dma);
	DMA_Cmd(DMA1_Stream5, ENABLE);
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
#else
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
#endif

	DMA_DeInit(DMA1_Stream6);
	dma.DMA_Channel = DMA_Channel_4;
	dma.DMA_PeripheralBaseAddr = (u32)&USART2->DR;
	dma.DMA_Memory0BaseAddr = (u32)s_tx_dma_buf;
	dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	dma.DMA_BufferSize = 0;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_Mode = DMA_Mode_Normal;
	dma.DMA_Priority = DMA_Priority_High;
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream6, &dma);

	DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
	nvic.NVIC_IRQChannel = DMA1_Stream6_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 6;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
}
#endif

#if JETSON_USART2_DMA_RX && JETSON_USE_BLOB_V2
static void rs232_RxDmaRestart(void)
{
	DMA_Cmd(DMA1_Stream5, DISABLE);
	while(DMA1_Stream5->CR & DMA_SxCR_EN)
		;
	DMA_SetCurrDataCounter(DMA1_Stream5, JETSON_RX_DMA_SIZE);
	DMA_Cmd(DMA1_Stream5, ENABLE);
}

static void rs232_RxDmaFeed(u16 n)
{
	u16 i;

	for(i = 0; i < n; i++)
	{
		s_usart2_rx_bytes++;
		RS232_ProcessRxByte(s_rx_dma_buf[i]);
	}
}
#endif

void RS232_GetRxStats(u32 *rx_bytes, u32 *ore_cnt, u32 *ab_idle)
{
	if(rx_bytes) *rx_bytes = s_usart2_rx_bytes;
	if(ore_cnt)  *ore_cnt  = s_usart2_ore_cnt;
	if(ab_idle)  *ab_idle  = s_usart2_ab_idle;
}

void RS232_GetRxMagicStats(u32 *raw_ab, u32 *raw_a5, u32 *raw_aa)
{
	if(raw_ab) *raw_ab = s_usart2_raw_ab;
	if(raw_a5) *raw_a5 = s_usart2_raw_a5;
	if(raw_aa) *raw_aa = s_usart2_raw_aa;
}

void RS232_PrintHwDiag(void)
{
#if JETSON_USART2_DMA_TX && JETSON_USE_BLOB_V2
#if JETSON_USART2_DMA_RX
	printf("[JETSON HW] USART2 DMA+IDLE RX / DMA TX | CR1=0x%04X SR=0x%04X\r\n",
#else
	printf("[JETSON HW] USART2 byte-ISR RX / DMA TX | CR1=0x%04X SR=0x%04X\r\n",
#endif
		(unsigned int)USART2->CR1, (unsigned int)USART2->SR);
#else
	printf("[JETSON HW] USART2 CR1=0x%04X SR=0x%04X | Jetson TX->PA3(RX) MCU TX=PA2 (blocking TXE)\r\n",
		(unsigned int)USART2->CR1, (unsigned int)USART2->SR);
#endif
}
#endif

static void rs232_PrintRxFrame(const u8 *frame)
{
#if JETSON_RX_DEBUG
	u8 i;
	u8 checksum = 0;

	printf("[JETSON RX] ");
	for(i = 0; i < JETSON_V3_FRAME_LEN; i++)
	{
		printf("%02X ", frame[i]);
		if(i < JETSON_V3_FRAME_LEN - 1)
			checksum ^= frame[i];
	}
	printf("| chk=%02X calc=%02X\r\n", frame[JETSON_V3_FRAME_LEN - 1], checksum);
#else
	(void)frame;
#endif
}

static void rs232_PutS16BE(u8 *buf, u16 idx, s16 value)
{
	buf[idx] = (u8)((value >> 8) & 0xFF);
	buf[idx + 1] = (u8)(value & 0xFF);
}

static void rs232_PutU16BE(u8 *buf, u16 idx, u16 value)
{
	buf[idx] = (u8)((value >> 8) & 0xFF);
	buf[idx + 1] = (u8)(value & 0xFF);
}

static u8 rs232_BuildXor(const u8 *frame)
{
	u8 i;
	u8 checksum = 0;
	for(i = 0; i < JETSON_V3_FRAME_LEN - 1; i++)
	{
		checksum ^= frame[i];
	}
	return checksum;
}

static u8 rs232_MapSafetyState(ArbiterMode_t mode)
{
	switch(mode)
	{
		case ARBITER_MODE_NORMAL:      return 0x01;
		case ARBITER_MODE_SPEED_LIMIT: return 0x02;
		case ARBITER_MODE_DEGRADED:    return 0x03;
		case ARBITER_MODE_EMERGENCY:   return 0x04;
		case ARBITER_MODE_RECOVERING:  return 0x03;
		default:                       return 0x03;
	}
}

static u8 rs232_CalcLimitFactor(const ArbiterState_t *state)
{
	s32 num;
	s32 den;
	s32 factor;
	u16 dist;

	if(!state)
		return 100;
	if(state->current_mode != ARBITER_MODE_SPEED_LIMIT)
		return 100;

	dist = state->nearest_dist;
	if(dist == ARBITER_DIST_UNKNOWN || dist <= ARBITER_OBSTACLE_NEAR_MM)
		return 0;
	if(dist >= ARBITER_OBSTACLE_FAR_MM)
		return 100;

	num = (s32)dist - (s32)ARBITER_OBSTACLE_NEAR_MM;
	den = (s32)ARBITER_OBSTACLE_FAR_MM - (s32)ARBITER_OBSTACLE_NEAR_MM;
	factor = (num * 100) / den;
	if(factor < 0) factor = 0;
	if(factor > 100) factor = 100;
	return (u8)factor;
}

/*******************************************************************************
* RS232_Init ?? ??? USART2??PA2-TX, PA3-RX?????? Jetson
*******************************************************************************/
#if !JETSON_LINK_CAN
void RS232_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// ??????
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  // GPIOA???
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // USART2???

	// ????PA2??PA3?USART2????????
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); // PA2 -> USART2_TX
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); // PA3 -> USART2_RX

	// ????GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// ????USART2
	USART_InitStructure.USART_BaudRate = RS232_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	// ???? USART2 ????
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	/* ??????? >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5)??FromISR ???? */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART2, ENABLE);

#if JETSON_USART2_DMA_TX && JETSON_USE_BLOB_V2
	rs232_DmaInit();
#else
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
#endif
}
#endif

static void rs232_DeliverV3Frame(u32 can_id, const u8 *frame)
{
#if JETSON_LINK_CAN
	JetsonCAN_SendV3Frame(can_id, frame);
#else
	RS232_SendData((u8 *)frame, JETSON_V3_FRAME_LEN);
#endif
}

#if !JETSON_LINK_CAN
void RS232_SendByte(u8 data)
{
	USART_SendData(USART2, data);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

void RS232_SendData(u8* data, u16 len)
{
	if(!data || len == 0)
		return;

#if JETSON_USART2_DMA_TX && JETSON_USE_BLOB_V2
	if(len > JETSON_TX_DMA_MAX)
		len = JETSON_TX_DMA_MAX;

	rs232_TxLock();
	if(!s_tx_dma_busy)
		rs232_TxDmaKick(data, len);
	else
	{
		u16 i;
		for(i = 0; i < len; i++)
			s_tx_pend_buf[i] = data[i];
		s_tx_pend_len = len;
		s_tx_pend_valid = 1;
	}
	rs232_TxUnlock();
#else
	rs232_TxLock();
	while(len--)
		RS232_SendByte(*data++);
	rs232_TxUnlock();
#endif
}

void RS232_SendServiceFrame(u32 can_id, const u8 *payload, u8 len)
{
	u8 frame[JETSON_RS232_SVC_LEN];
	u8 i;
	u8 copy_len;

	if(!payload)
		return;
	if(len > JETSON_CAN_V3_FRAG_LEN)
		len = JETSON_CAN_V3_FRAG_LEN;

	frame[0] = JETSON_RS232_SVC_MAGIC;
	frame[1] = (u8)((can_id >> 8) & 0xFF);
	frame[2] = (u8)(can_id & 0xFF);
	copy_len = len;
	for(i = 0; i < JETSON_CAN_V3_FRAG_LEN; i++)
		frame[3 + i] = (i < copy_len) ? payload[i] : 0;
	RS232_SendData(frame, JETSON_RS232_SVC_LEN);
}
#endif

/*******************************************************************************
* ?? ?? ??         : RS232_SendV3StatusFrame
* ????????		   : ???? V3 ?????????frame_type=0x02??
*******************************************************************************/
void RS232_SendV3StatusFrame(const ArbiterState_t *state, u16 sonar_front, u16 sonar_back,
	u16 sonar_left, u16 sonar_right)
{
	u8 frame[JETSON_V3_FRAME_LEN] = {0};
	u8 link_state = 0;

	if(!state)
		return;

	frame[0] = JETSON_FRAME_HEADER;
	frame[1] = JETSON_FRAME_TYPE_UP_ST;
	frame[2] = jetson_tx_seq++;
	frame[3] = rs232_MapSafetyState(state->current_mode);

	if(state->heartbeat_lost)
		link_state |= 0x01;
	if(state->sys_status.system_status == 0x02)
		link_state |= 0x02;
	if(BEEP_GetDuty() > 0)
		link_state |= 0x04;
	frame[4] = link_state;
	frame[5] = rs232_CalcLimitFactor(state);

	rs232_PutS16BE(frame, 6, state->motion_fb.linear_speed);
	rs232_PutS16BE(frame, 8, state->motion_fb.spin_speed);
	rs232_PutS16BE(frame, 10, state->motion_fb.steering_angle);

	rs232_PutU16BE(frame, 12, sonar_front);
	rs232_PutU16BE(frame, 14, sonar_back);
	rs232_PutU16BE(frame, 16, sonar_left);
	rs232_PutU16BE(frame, 18, sonar_right);
	rs232_PutU16BE(frame, 20, state->sys_status.battery_voltage);
	frame[22] = state->bms_data.soc;

	frame[23] = rs232_BuildXor(frame);
	rs232_DeliverV3Frame(JETSON_CAN_ID_STATUS, frame);
}

/*******************************************************************************
* ?? ?? ??         : RS232_SendV3DetailFrame
* ????????		   : ???? V3 ??????????frame_type=0x03??
*******************************************************************************/
void RS232_SendV3DetailFrame(const ArbiterState_t *state)
{
	u8 frame[JETSON_V3_FRAME_LEN] = {0};
	u8 i;
	s8 temp_max;
	u8 driver_state;
	ChassisWheelSpeed_t ws;

	if(!state)
		return;

	frame[0] = JETSON_FRAME_HEADER;
	frame[1] = JETSON_FRAME_TYPE_UP_EX;
	frame[2] = jetson_tx_seq++;

	/* ????????????????RF/RR/LR/LF????? CAN ??????????????? */
	Arbiter_GetWheelSpeedPhysical(&ws);
	rs232_PutS16BE(frame, 3, ws.rf);   // RF
	rs232_PutS16BE(frame, 5, ws.rr);   // RR
	rs232_PutS16BE(frame, 7, ws.lr);   // LR
	rs232_PutS16BE(frame, 9, ws.lf);   // LF

	rs232_PutS16BE(frame, 11, state->wheel_angle.steer5);  // RF steer
	rs232_PutS16BE(frame, 13, state->wheel_angle.steer6);  // RR steer
	rs232_PutS16BE(frame, 15, state->wheel_angle.steer7);  // LR steer
	rs232_PutS16BE(frame, 17, state->wheel_angle.steer8);  // LF steer

	temp_max = state->motor_low[0].motor_temp;
	driver_state = 0;
	for(i = 0; i < 8; i++)
	{
		if(state->motor_low[i].motor_temp > temp_max)
			temp_max = state->motor_low[i].motor_temp;
		driver_state |= state->motor_low[i].status;
	}
	frame[19] = (u8)temp_max;
	frame[20] = driver_state;
	frame[21] = 0;
	frame[22] = 0;

	frame[23] = rs232_BuildXor(frame);
	rs232_DeliverV3Frame(JETSON_CAN_ID_DETAIL, frame);
}

#if !JETSON_LINK_CAN
/*******************************************************************************
* ?? ?? ??         : RS232_ProcessRxByte
* ????????		   : ???????????????????????????????
* ??    ??         : byte: ??????????
* ??    ??         : ??
*******************************************************************************/
void RS232_ProcessRxByte(u8 byte)
{
	u8 i;

	if(byte == 0xABu)
		s_usart2_raw_ab++;
	else if(byte == JETSON_RS232_SVC_MAGIC)
		s_usart2_raw_a5++;
	else if(byte == JETSON_FRAME_HEADER)
		s_usart2_raw_aa++;

	if(rs232_rx_mode == RS232_RX_MODE_IDLE)
	{
#if JETSON_USE_BLOB_V2
		if(byte == BLOB_HDR_MAGIC)
		{
			s_usart2_ab_idle++;
			BlobRs232_RxReset();
			BlobRs232_RxFeed(byte);
			rs232_rx_mode = RS232_RX_MODE_BLOB;
		}
		else if(byte == JETSON_RS232_SVC_MAGIC)
		{
			rs232_rx_buf[0] = byte;
			rs232_rx_idx = 1;
			rs232_rx_mode = RS232_RX_MODE_SVC;
		}
		/* BLOB ????IDLE ????? 0xAA?????????????? V3 ??????? 0xAB ?? */
#else
		if(byte == JETSON_FRAME_HEADER)
		{
			rs232_rx_buf[0] = byte;
			rs232_rx_idx = 1;
			rs232_rx_mode = RS232_RX_MODE_V3;
		}
		else if(byte == JETSON_RS232_SVC_MAGIC)
		{
			rs232_rx_buf[0] = byte;
			rs232_rx_idx = 1;
			rs232_rx_mode = RS232_RX_MODE_SVC;
		}
#endif
		return;
	}

#if JETSON_USE_BLOB_V2
	if(rs232_rx_mode == RS232_RX_MODE_BLOB)
	{
		if(BlobRs232_RxFeed(byte))
			rs232_rx_mode = RS232_RX_MODE_IDLE;
		return;
	}
#endif

	rs232_rx_buf[rs232_rx_idx] = byte;
	rs232_rx_idx++;

	if(rs232_rx_mode == RS232_RX_MODE_V3)
	{
		/* Jetson ?????? 0x01???????????????????????????? 0xAB */
		if(rs232_rx_idx == 2 && rs232_rx_buf[1] != 0x01)
		{
			rs232_rx_mode = RS232_RX_MODE_IDLE;
			rs232_rx_idx = 0;
			RS232_ProcessRxByte(byte);
			return;
		}
		if(rs232_rx_idx >= JETSON_V3_FRAME_LEN)
		{
			for(i = 0; i < JETSON_V3_FRAME_LEN; i++)
				rs232_complete_frame[i] = rs232_rx_buf[i];
			rs232_PrintRxFrame(rs232_complete_frame);
			rs232_frame_ready = 1;
			rs232_rx_mode = RS232_RX_MODE_IDLE;
			rs232_rx_idx = 0;
		}
		return;
	}

	if(rs232_rx_mode == RS232_RX_MODE_SVC && rs232_rx_idx >= JETSON_RS232_SVC_LEN)
	{
		u32 can_id;
		u8 payload[8];

		can_id = ((u32)rs232_rx_buf[1] << 8) | rs232_rx_buf[2];
		for(i = 0; i < 8; i++)
			payload[i] = rs232_rx_buf[3 + i];

		rs232_svc_can_id = can_id;
		for(i = 0; i < 8; i++)
			rs232_svc_payload[i] = payload[i];
		rs232_svc_ready = 1;
		rs232_rx_mode = RS232_RX_MODE_IDLE;
		rs232_rx_idx = 0;
	}
}

/*******************************************************************************
* ?? ?? ??         : RS232_GetJetsonFrame
* ????????		   : ???Jetson????
* ??    ??         : frame: ?????????
* ??    ??         : 0=??????, 1=???????
*******************************************************************************/
u8 RS232_GetJetsonFrame(u8* frame)
{
	if(rs232_frame_ready)
	{
		u8 i;
		for(i = 0; i < JETSON_V3_FRAME_LEN; i++)
		{
			frame[i] = rs232_complete_frame[i];
		}
		rs232_frame_ready = 0;
		return 1;
	}
	return 0;
}

u8 RS232_GetServiceRequest(u32 *can_id, u8 *payload)
{
	u8 i;

	if(!rs232_svc_ready || !can_id || !payload)
		return 0;

	*can_id = rs232_svc_can_id;
	for(i = 0; i < 8; i++)
		payload[i] = rs232_svc_payload[i];
	rs232_svc_ready = 0;
	return 1;
}

/*******************************************************************************
* ?? ?? ??         : USART2_IRQHandler
* ????????		   : USART2???????????????Jetson?????
*******************************************************************************/
void USART2_IRQHandler(void)
{
#if JETSON_USART2_DMA_RX && JETSON_USE_BLOB_V2
	u32 sr;
	u16 n;

	sr = USART2->SR;
	if(sr & USART_SR_ORE)
	{
		s_usart2_ore_cnt++;
		(void)USART2->DR;
	}

	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		(void)USART2->SR;
		(void)USART2->DR;

		DMA_Cmd(DMA1_Stream5, DISABLE);
		while(DMA1_Stream5->CR & DMA_SxCR_EN)
			;
		n = (u16)(JETSON_RX_DMA_SIZE - DMA_GetCurrDataCounter(DMA1_Stream5));
		if(n > 0)
			rs232_RxDmaFeed(n);
		rs232_RxDmaRestart();
	}
#else
	u8 byte;
	u32 sr;

	sr = USART2->SR;
	if(sr & USART_SR_ORE)
		s_usart2_ore_cnt++;

	if(sr & (USART_SR_RXNE | USART_SR_ORE))
	{
		byte = (u8)USART2->DR;
		s_usart2_rx_bytes++;
		RS232_ProcessRxByte(byte);
	}
#endif
}

#if JETSON_USART2_DMA_TX && JETSON_USE_BLOB_V2
void DMA1_Stream6_IRQHandler(void)
{
	u16 len;
	u8 kick_buf[JETSON_TX_DMA_MAX];
	u16 i;

	if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6) == RESET)
		return;

	DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
	DMA_Cmd(DMA1_Stream6, DISABLE);

	if(s_tx_pend_valid)
	{
		len = s_tx_pend_len;
		for(i = 0; i < len; i++)
			kick_buf[i] = s_tx_pend_buf[i];
		s_tx_pend_valid = 0;
		rs232_TxDmaKick(kick_buf, len);
	}
	else
		s_tx_dma_busy = 0;
}
#endif
#endif
