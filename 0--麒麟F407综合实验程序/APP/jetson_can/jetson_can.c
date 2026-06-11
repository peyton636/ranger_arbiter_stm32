#include "jetson_can.h"
#include "can2.h"
#include "distance_sensor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"

#define JETSON_CAN_RX_DEBUG       0
#define JETSON_CAN_ASM_TIMEOUT    50u
#define JETSON_CAN_FAULT_KEEP_MS  1000u

static u8 s_rx_asm[JETSON_FRAME_LEN];
static u8 s_rx_frag_cnt = 0;
static u8 s_rx_frame_ready = 0;
static u8 s_rx_complete[JETSON_FRAME_LEN];
static u32 s_rx_last_ms = 0;
static u32 s_last_fault_sig = 0;
static u32 s_last_fault_send_ms = 0;

static u32 JetsonCAN_NowMs(void)
{
	return (u32)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void JetsonCAN_ResetRxAsm(void)
{
	s_rx_frag_cnt = 0;
	s_rx_last_ms = 0;
}

static u8 JetsonCAN_ValidateDownFrame(const u8 *frame)
{
	u8 i;
	u8 checksum = 0;

	if(frame[0] != JETSON_FRAME_HEADER)
		return 0;
	if(frame[1] != JETSON_FRAME_TYPE_DOWN)
		return 0;

	for(i = 0; i < JETSON_FRAME_LEN - 1; i++)
		checksum ^= frame[i];

	return (checksum == frame[JETSON_FRAME_LEN - 1]);
}

static void JetsonCAN_OnFrag(const u8 *data, u8 dlc)
{
	u8 i;
	u32 now = JetsonCAN_NowMs();
	u16 base;

	if(dlc != JETSON_CAN_V3_FRAG_LEN)
		return;

	if(s_rx_frag_cnt > 0 && (now - s_rx_last_ms) > JETSON_CAN_ASM_TIMEOUT)
		JetsonCAN_ResetRxAsm();

	if(s_rx_frag_cnt >= JETSON_CAN_V3_FRAG_CNT)
		JetsonCAN_ResetRxAsm();

	base = (u16)s_rx_frag_cnt * JETSON_CAN_V3_FRAG_LEN;
	for(i = 0; i < JETSON_CAN_V3_FRAG_LEN; i++)
		s_rx_asm[base + i] = data[i];

	s_rx_frag_cnt++;
	s_rx_last_ms = now;

	if(s_rx_frag_cnt >= JETSON_CAN_V3_FRAG_CNT)
	{
		if(JetsonCAN_ValidateDownFrame(s_rx_asm))
		{
			for(i = 0; i < JETSON_FRAME_LEN; i++)
				s_rx_complete[i] = s_rx_asm[i];
			s_rx_frame_ready = 1;
#if JETSON_CAN_RX_DEBUG
			printf("[JETSON CAN RX] V3 down frame OK\r\n");
#endif
		}
#if JETSON_CAN_RX_DEBUG
		else
			printf("[JETSON CAN RX] V3 down frame checksum fail\r\n");
#endif
		JetsonCAN_ResetRxAsm();
	}
}

void JetsonCAN_Init(void)
{
	CAN2_Init_Jetson();
	JetsonCAN_ResetRxAsm();
	s_rx_frame_ready = 0;
	s_last_fault_sig = 0;
	s_last_fault_send_ms = 0;
}

static void JetsonCAN_PutU32BE(u8 *buf, u8 idx, u32 value)
{
	buf[idx] = (u8)((value >> 24) & 0xFF);
	buf[idx + 1] = (u8)((value >> 16) & 0xFF);
	buf[idx + 2] = (u8)((value >> 8) & 0xFF);
	buf[idx + 3] = (u8)(value & 0xFF);
}

static void JetsonCAN_PutU16BE(u8 *buf, u8 idx, u16 value)
{
	buf[idx] = (u8)((value >> 8) & 0xFF);
	buf[idx + 1] = (u8)(value & 0xFF);
}

static void JetsonCAN_PutS16BE(u8 *buf, u8 idx, s16 value)
{
	JetsonCAN_PutU16BE(buf, idx, (u16)value);
}

static void JetsonCAN_PutS32BE(u8 *buf, u8 idx, s32 value)
{
	buf[idx] = (u8)((value >> 24) & 0xFF);
	buf[idx + 1] = (u8)((value >> 16) & 0xFF);
	buf[idx + 2] = (u8)((value >> 8) & 0xFF);
	buf[idx + 3] = (u8)(value & 0xFF);
}

static u8 JetsonCAN_MapSafetyState(ArbiterMode_t mode)
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

static u8 JetsonCAN_CalcLimitFactor(const ArbiterState_t *state)
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

static u8 JetsonCAN_BuildLinkState(const ArbiterState_t *state)
{
	u8 link_state = 0;

	if(!state)
		return 0;
	if(state->heartbeat_lost)
		link_state |= 0x01;
	if(state->sys_status.system_status == 0x02)
		link_state |= 0x02;
	return link_state;
}

static void JetsonCAN_BuildFaultCodes(const ArbiterState_t *state, u8 *codes, u8 *count)
{
	u8 n = 0;

	if(!state || !codes || !count)
		return;

	*count = 0;
	if(state->heartbeat_lost && n < 4)
		codes[n++] = JETSON_ERR_JETSON_HB_LOST;
	if(state->sys_status.system_status == 0x02 && n < 4)
		codes[n++] = JETSON_ERR_CHASSIS_SYS_ERR;
	if(state->sys_status.fault_info != 0 && n < 4)
		codes[n++] = JETSON_ERR_CHASSIS_FAULT_INFO;
	if(state->current_mode == ARBITER_MODE_EMERGENCY && n < 4)
		codes[n++] = JETSON_ERR_OBSTACLE_EMERGENCY;
	if(state->nearest_dist == ARBITER_DIST_UNKNOWN && n < 4)
		codes[n++] = JETSON_ERR_DIST_ALL_UNKNOWN;
	if(state->obstacle_valid_mask == 0 && n < 4)
		codes[n++] = JETSON_ERR_DIST_SENSOR_FAULT;
	if(!GPS_HasFix() && n < 4)
		codes[n++] = JETSON_ERR_GPS_NO_FIX;

	*count = n;
}

static u32 JetsonCAN_FaultSignature(const ArbiterState_t *state)
{
	u8 codes[4];
	u8 count = 0;
	u32 sig = 0;
	u8 i;

	if(!state)
		return 0;

	JetsonCAN_BuildFaultCodes(state, codes, &count);
	for(i = 0; i < count; i++)
		sig |= ((u32)codes[i]) << (i * 8);
	sig |= ((u32)state->sys_status.fault_info & 0xFFu) << 16;
	return sig;
}

static void JetsonCAN_SendTimeSync(void)
{
	u8 frame[8];
	u32 tick_ms = JetsonCAN_NowMs();
	u32 utc_sec = GPS_GetUtcUnixSec();

	JetsonCAN_PutU32BE(frame, 0, tick_ms);
	JetsonCAN_PutU32BE(frame, 4, utc_sec);
	CAN2_Send_Msg_WithID(JETSON_CAN_ID_TIME_RSP, frame, 8);
}

static void JetsonCAN_SendStatusQueryRsp(const ArbiterState_t *state, u16 nearest_mm)
{
	u8 frame[8];
	u8 flags = 0;

	if(!state)
		return;

	frame[0] = JetsonCAN_MapSafetyState(state->current_mode);
	frame[1] = JetsonCAN_BuildLinkState(state);
	JetsonCAN_PutS16BE(frame, 2, state->motion_fb.linear_speed);
	JetsonCAN_PutU16BE(frame, 4, nearest_mm);
	frame[6] = JetsonCAN_CalcLimitFactor(state);

	if(GPS_HasFix())
		flags |= 0x01;
	if(state->sys_status.mode_control == 0x01)
		flags |= 0x02;
	if(state->obstacle_valid_mask != 0)
		flags |= 0x04;
	frame[7] = flags;

	CAN2_Send_Msg_WithID(JETSON_CAN_ID_STATUS_RSP, frame, 8);
}

static void JetsonCAN_SendFaultReport(const ArbiterState_t *state)
{
	u8 frame[8];
	u8 codes[4];
	u8 count = 0;
	u32 tick_ms = JetsonCAN_NowMs();

	if(!state)
		return;

	JetsonCAN_BuildFaultCodes(state, codes, &count);
	frame[0] = (count > 0) ? codes[0] : JETSON_ERR_OK;
	frame[1] = (count > 1) ? codes[1] : 0;
	frame[2] = (count > 2) ? codes[2] : 0;
	frame[3] = (count > 3) ? codes[3] : 0;
	JetsonCAN_PutU16BE(frame, 4, (u16)(tick_ms & 0xFFFFu));
	frame[6] = state->sys_status.system_status;
	frame[7] = (u8)(state->sys_status.fault_info & 0xFFu);
	CAN2_Send_Msg_WithID(JETSON_CAN_ID_FAULT, frame, 8);
}

void JetsonCAN_ServiceFault(const ArbiterState_t *state)
{
	u32 sig;
	u32 now;

	if(!state)
		return;

	sig = JetsonCAN_FaultSignature(state);
	now = JetsonCAN_NowMs();

	if(sig == 0 && s_last_fault_sig == 0)
		return;

	if(sig != s_last_fault_sig ||
	   (sig != 0 && (now - s_last_fault_send_ms) >= JETSON_CAN_FAULT_KEEP_MS))
	{
		JetsonCAN_SendFaultReport(state);
		s_last_fault_sig = sig;
		s_last_fault_send_ms = now;
	}
}

static void JetsonCAN_HandleServiceRx(u32 can_id, const u8 *buf, u8 dlc,
	const ArbiterState_t *state, u16 nearest_mm)
{
	if(!buf || dlc == 0)
		return;

	if(can_id == JETSON_CAN_ID_TIME_REQ)
	{
		if(buf[0] == JETSON_CAN_CMD_TIME_SYNC)
			JetsonCAN_SendTimeSync();
		return;
	}

	if(can_id == JETSON_CAN_ID_STATUS_REQ)
	{
		if(buf[0] == JETSON_CAN_CMD_STATUS_QRY)
			JetsonCAN_SendStatusQueryRsp(state, nearest_mm);
		return;
	}
}

void JetsonCAN_ProcessRx(const ArbiterState_t *state, u16 nearest_mm)
{
	u32 can_id;
	u8 buf[JETSON_CAN_V3_FRAG_LEN];
	u8 dlc;
	u32 now = JetsonCAN_NowMs();

	if(s_rx_frag_cnt > 0 && s_rx_last_ms > 0 &&
	   (now - s_rx_last_ms) > JETSON_CAN_ASM_TIMEOUT)
	{
		JetsonCAN_ResetRxAsm();
	}

	while((dlc = CAN2_Receive_Msg_WithID(&can_id, buf)) != 0)
	{
		if(can_id == JETSON_CAN_ID_DOWN)
			JetsonCAN_OnFrag(buf, dlc);
		else if(can_id == JETSON_CAN_ID_TIME_REQ || can_id == JETSON_CAN_ID_STATUS_REQ)
			JetsonCAN_HandleServiceRx(can_id, buf, dlc, state, nearest_mm);
	}
}

u8 JetsonCAN_GetFrame(u8 *frame)
{
	u8 i;

	if(!s_rx_frame_ready)
		return 0;

	for(i = 0; i < JETSON_FRAME_LEN; i++)
		frame[i] = s_rx_complete[i];

	s_rx_frame_ready = 0;
	return 1;
}

void JetsonCAN_SendV3Frame(u32 can_id, const u8 *frame)
{
	if(!frame)
		return;

	CAN2_Send_Msg_WithID(can_id, (u8 *)&frame[0], JETSON_CAN_V3_FRAG_LEN);
	CAN2_Send_Msg_WithID(can_id, (u8 *)&frame[8], JETSON_CAN_V3_FRAG_LEN);
	CAN2_Send_Msg_WithID(can_id, (u8 *)&frame[16], JETSON_CAN_V3_FRAG_LEN);
}

void JetsonCAN_SendGps(const GPS_Data_t *gps)
{
	u8 frame_a[8];
	u8 frame_b[8];
	u8 frame_c[8];
	u8 flags = 0;
	u16 hdop_x100;
	u16 speed_cms;
	s32 lat_e7 = 0;
	s32 lon_e7 = 0;
	s16 heading_x100 = 0;
	s16 alt_dm = (s16)0x7FFF;
	u8 heading_valid;

	if(!gps)
		return;

	if(gps->pos_valid)
		flags |= 0x01;
	if(gps->vel_valid)
		flags |= 0x02;

	heading_valid = (gps->vel_valid ||
	                 (gps->usefull && gps->speed_mps > 0.1f)) ? 1 : 0;
	if(heading_valid)
		flags |= 0x04;
	if(gps->num_sv >= 4 && gps->pos_valid)
		flags |= 0x08;
	if(gps->usefull)
		flags |= 0x10;

	if(gps->pos_valid)
	{
		lat_e7 = (s32)(gps->latitude_deg * 10000000.0);
		lon_e7 = (s32)(gps->longitude_deg * 10000000.0);
		if(gps->altitude_m > -500.0f && gps->altitude_m < 10000.0f)
			alt_dm = (s16)(gps->altitude_m * 10.0f);
	}

	if(gps->usefull && gps->hdop > 0.0f && gps->hdop < 100.0f)
		hdop_x100 = (u16)(gps->hdop * 100.0f);
	else
		hdop_x100 = 0xFFFF;

	if(gps->vel_valid || (gps->usefull && gps->speed_mps >= 0.0f))
		speed_cms = (u16)(gps->speed_mps * 100.0f);
	else
		speed_cms = 0;

	if(heading_valid)
		heading_x100 = (s16)(gps->heading_deg * 100.0f);

	frame_a[0] = JETSON_CAN_GPS_MAGIC;
	frame_a[1] = 0;
	frame_a[2] = flags;
	frame_a[3] = gps->num_sv;
	JetsonCAN_PutU16BE(frame_a, 4, hdop_x100);
	JetsonCAN_PutU16BE(frame_a, 6, speed_cms);
	CAN2_Send_Msg_WithID(JETSON_CAN_ID_GPS, frame_a, 8);
	vTaskDelay(pdMS_TO_TICKS(JETSON_CAN_GPS_FRAG_GAP_MS));

	frame_b[0] = JETSON_CAN_GPS_MAGIC;
	frame_b[1] = 1;
	JetsonCAN_PutS32BE(frame_b, 2, lat_e7);
	JetsonCAN_PutS16BE(frame_b, 6, heading_x100);
	CAN2_Send_Msg_WithID(JETSON_CAN_ID_GPS_B, frame_b, 8);
	vTaskDelay(pdMS_TO_TICKS(JETSON_CAN_GPS_FRAG_GAP_MS));

	frame_c[0] = JETSON_CAN_GPS_MAGIC;
	frame_c[1] = 2;
	JetsonCAN_PutS32BE(frame_c, 2, lon_e7);
	JetsonCAN_PutS16BE(frame_c, 6, alt_dm);
	CAN2_Send_Msg_WithID(JETSON_CAN_ID_GPS_C, frame_c, 8);
}
