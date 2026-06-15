#include "usart3.h"
#include "stm32f4xx.h"
#include "stdio.h"
#include "beep.h"
#include "can2.h"
#if JETSON_LINK_CAN
#include "jetson_can.h"
#endif

#define JETSON_RX_DEBUG 0

// 接收缓冲区
#define USART3_RX_MODE_IDLE  0
#define USART3_RX_MODE_V3    1
#define USART3_RX_MODE_SVC   2

static u8 usart3_rx_mode = USART3_RX_MODE_IDLE;
static u8 usart3_rx_buf[JETSON_V3_FRAME_LEN];
static u8 usart3_rx_idx = 0;
static u8 usart3_frame_ready = 0;
static u8 usart3_complete_frame[JETSON_V3_FRAME_LEN];
static u8 usart3_svc_ready = 0;
static u32 usart3_svc_can_id = 0;
static u8 usart3_svc_payload[8];
static u8 jetson_tx_seq = 0;

static void USART3_PrintRxFrame(const u8 *frame)
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

static void USART3_PutS16BE(u8 *buf, u16 idx, s16 value)
{
	buf[idx] = (u8)((value >> 8) & 0xFF);
	buf[idx + 1] = (u8)(value & 0xFF);
}

static void USART3_PutU16BE(u8 *buf, u16 idx, u16 value)
{
	buf[idx] = (u8)((value >> 8) & 0xFF);
	buf[idx + 1] = (u8)(value & 0xFF);
}

static u8 USART3_BuildXor(const u8 *frame)
{
	u8 i;
	u8 checksum = 0;
	for(i = 0; i < JETSON_V3_FRAME_LEN - 1; i++)
	{
		checksum ^= frame[i];
	}
	return checksum;
}

static u8 USART3_MapSafetyState(ArbiterMode_t mode)
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

static u8 USART3_CalcLimitFactor(const ArbiterState_t *state)
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
* 函 数 名         : USART3_Init
* 功能描述		   : USART2初始化（PA2-TX, PA3-RX）用于连接Jetson
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
#if !JETSON_LINK_CAN
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
	/* 优先级须 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5)，FromISR 才合法 */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// 启用 USART2 接收中断
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	// 启用USART2
	USART_Cmd(USART2, ENABLE);
}
#endif

static void USART3_DeliverV3Frame(u32 can_id, const u8 *frame)
{
#if JETSON_LINK_CAN
	JetsonCAN_SendV3Frame(can_id, frame);
#else
	USART3_SendData((u8 *)frame, JETSON_V3_FRAME_LEN);
#endif
}

#if !JETSON_LINK_CAN
void USART3_SendByte(u8 data)
{
	USART_SendData(USART2, data);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

void USART3_SendData(u8* data, u16 len)
{
	while(len--)
		USART3_SendByte(*data++);
}

void USART3_SendServiceFrame(u32 can_id, const u8 *payload, u8 len)
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
	USART3_SendData(frame, JETSON_RS232_SVC_LEN);
}
#endif

/*******************************************************************************
* 函 数 名         : USART3_SendV3StatusFrame
* 功能描述		   : 发送 V3 上行状态帧（frame_type=0x02）
*******************************************************************************/
void USART3_SendV3StatusFrame(const ArbiterState_t *state, u16 sonar_front, u16 sonar_back,
	u16 sonar_left, u16 sonar_right)
{
	u8 frame[JETSON_V3_FRAME_LEN] = {0};
	u8 link_state = 0;

	if(!state)
		return;

	frame[0] = JETSON_FRAME_HEADER;
	frame[1] = JETSON_FRAME_TYPE_UP_ST;
	frame[2] = jetson_tx_seq++;
	frame[3] = USART3_MapSafetyState(state->current_mode);

	if(state->heartbeat_lost)
		link_state |= 0x01;
	if(state->sys_status.system_status == 0x02)
		link_state |= 0x02;
	if(BEEP_GetDuty() > 0)
		link_state |= 0x04;
	frame[4] = link_state;
	frame[5] = USART3_CalcLimitFactor(state);

	USART3_PutS16BE(frame, 6, state->motion_fb.linear_speed);
	USART3_PutS16BE(frame, 8, state->motion_fb.spin_speed);
	USART3_PutS16BE(frame, 10, state->motion_fb.steering_angle);

	USART3_PutU16BE(frame, 12, sonar_front);
	USART3_PutU16BE(frame, 14, sonar_back);
	USART3_PutU16BE(frame, 16, sonar_left);
	USART3_PutU16BE(frame, 18, sonar_right);
	USART3_PutU16BE(frame, 20, state->sys_status.battery_voltage);
	frame[22] = state->bms_data.soc;

	frame[23] = USART3_BuildXor(frame);
	USART3_DeliverV3Frame(JETSON_CAN_ID_STATUS, frame);
}

/*******************************************************************************
* 函 数 名         : USART3_SendV3DetailFrame
* 功能描述		   : 发送 V3 上行扩展帧（frame_type=0x03）
*******************************************************************************/
void USART3_SendV3DetailFrame(const ArbiterState_t *state)
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

	/* 按协议固定顺序输出：RF/RR/LR/LF（先将 CAN 编号映射到物理位置） */
	Arbiter_GetWheelSpeedPhysical(&ws);
	USART3_PutS16BE(frame, 3, ws.rf);   // RF
	USART3_PutS16BE(frame, 5, ws.rr);   // RR
	USART3_PutS16BE(frame, 7, ws.lr);   // LR
	USART3_PutS16BE(frame, 9, ws.lf);   // LF

	USART3_PutS16BE(frame, 11, state->wheel_angle.steer5);  // RF steer
	USART3_PutS16BE(frame, 13, state->wheel_angle.steer6);  // RR steer
	USART3_PutS16BE(frame, 15, state->wheel_angle.steer7);  // LR steer
	USART3_PutS16BE(frame, 17, state->wheel_angle.steer8);  // LF steer

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

	frame[23] = USART3_BuildXor(frame);
	USART3_DeliverV3Frame(JETSON_CAN_ID_DETAIL, frame);
}

#if !JETSON_LINK_CAN
/*******************************************************************************
* 函 数 名         : USART3_ProcessRxByte
* 功能描述		   : 处理接收到的单个字节（在中断中调用）
* 输    入         : byte: 接收到的字节
* 输    出         : 无
*******************************************************************************/
void USART3_ProcessRxByte(u8 byte)
{
	u8 i;

	if(usart3_rx_mode == USART3_RX_MODE_IDLE)
	{
		if(byte == JETSON_FRAME_HEADER)
		{
			usart3_rx_buf[0] = byte;
			usart3_rx_idx = 1;
			usart3_rx_mode = USART3_RX_MODE_V3;
		}
		else if(byte == JETSON_RS232_SVC_MAGIC)
		{
			usart3_rx_buf[0] = byte;
			usart3_rx_idx = 1;
			usart3_rx_mode = USART3_RX_MODE_SVC;
		}
		return;
	}

	usart3_rx_buf[usart3_rx_idx] = byte;
	usart3_rx_idx++;

	if(usart3_rx_mode == USART3_RX_MODE_V3)
	{
		if(usart3_rx_idx >= JETSON_V3_FRAME_LEN)
		{
			for(i = 0; i < JETSON_V3_FRAME_LEN; i++)
				usart3_complete_frame[i] = usart3_rx_buf[i];
			USART3_PrintRxFrame(usart3_complete_frame);
			usart3_frame_ready = 1;
			usart3_rx_mode = USART3_RX_MODE_IDLE;
			usart3_rx_idx = 0;
		}
		return;
	}

	if(usart3_rx_mode == USART3_RX_MODE_SVC && usart3_rx_idx >= JETSON_RS232_SVC_LEN)
	{
		u32 can_id;
		u8 payload[8];

		can_id = ((u32)usart3_rx_buf[1] << 8) | usart3_rx_buf[2];
		for(i = 0; i < 8; i++)
			payload[i] = usart3_rx_buf[3 + i];

		usart3_svc_can_id = can_id;
		for(i = 0; i < 8; i++)
			usart3_svc_payload[i] = payload[i];
		usart3_svc_ready = 1;
		usart3_rx_mode = USART3_RX_MODE_IDLE;
		usart3_rx_idx = 0;
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
		for(i = 0; i < JETSON_V3_FRAME_LEN; i++)
		{
			frame[i] = usart3_complete_frame[i];
		}
		usart3_frame_ready = 0;
		return 1;
	}
	return 0;
}

u8 USART3_GetServiceRequest(u32 *can_id, u8 *payload)
{
	u8 i;

	if(!usart3_svc_ready || !can_id || !payload)
		return 0;

	*can_id = usart3_svc_can_id;
	for(i = 0; i < 8; i++)
		payload[i] = usart3_svc_payload[i];
	usart3_svc_ready = 0;
	return 1;
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
#endif
