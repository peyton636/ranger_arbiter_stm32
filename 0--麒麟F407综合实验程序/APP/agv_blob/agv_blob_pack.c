#include "agv_blob_pack.h"

#if JETSON_USE_BLOB_V2 && !JETSON_LINK_CAN

#include "agv_blob_link.h"
#include "beep.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"

static u8 s_blob_tx_seq = 0;
static u8 s_motor_toggle = 0;
static u32 s_uplink_div = 0;
static u16 s_blob_cmd_log_div = 0;
static volatile u32 s_blob_rx_ctrl = 0;
static volatile u32 s_blob_rx_other = 0;

/* 调试 log 延后到 vGpsTask 打印，避免 vJetsonTask 内 printf 阻塞/栈溢出 */
static volatile u8 s_log_first_rx = 0;
static u8 s_log_first_rx_seq = 0;
static u16 s_log_first_rx_len = 0;
static volatile u8 s_log_cmd = 0;
static u8 s_log_cmd_seq = 0;
static s16 s_log_cmd_v = 0;
static u8 s_log_cmd_mode = 0;
static u8 s_log_cmd_arb = 0;
static volatile u8 s_log_bad_len = 0;
static u16 s_log_bad_len_val = 0;

static u32 BlobPack_NowMs(void)
{
	return (u32)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void BlobPack_PutU16BE(u8 *buf, u16 idx, u16 v)
{
	buf[idx] = (u8)((v >> 8) & 0xFF);
	buf[idx + 1] = (u8)(v & 0xFF);
}

static void BlobPack_PutS16BE(u8 *buf, u16 idx, s16 v)
{
	BlobPack_PutU16BE(buf, idx, (u16)v);
}

static void BlobPack_PutU32BE(u8 *buf, u16 idx, u32 v)
{
	buf[idx] = (u8)((v >> 24) & 0xFF);
	buf[idx + 1] = (u8)((v >> 16) & 0xFF);
	buf[idx + 2] = (u8)((v >> 8) & 0xFF);
	buf[idx + 3] = (u8)(v & 0xFF);
}

static void BlobPack_PutS32BE(u8 *buf, u16 idx, s32 v)
{
	BlobPack_PutU32BE(buf, idx, (u32)v);
}

static u16 BlobPack_GetU16BE(const u8 *buf, u16 idx)
{
	return ((u16)buf[idx] << 8) | buf[idx + 1];
}

static s16 BlobPack_GetS16BE(const u8 *buf, u16 idx)
{
	return (s16)BlobPack_GetU16BE(buf, idx);
}

static u32 BlobPack_GetU32BE(const u8 *buf, u16 idx)
{
	return ((u32)buf[idx] << 24) | ((u32)buf[idx + 1] << 16) |
	       ((u32)buf[idx + 2] << 8) | buf[idx + 3];
}

static u8 BlobPack_MapSafetyState(ArbiterMode_t mode)
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

static u8 BlobPack_CalcLimitFactor(const ArbiterState_t *state)
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

static u8 BlobPack_BuildLinkFlags(const ArbiterState_t *state)
{
	u8 link = 0;

	if(!state)
		return 0;
	if(state->heartbeat_lost)
		link |= 0x01;
	if(state->sys_status.system_status == 0x02)
		link |= 0x02;
	if(BEEP_GetDuty() > 0)
		link |= 0x04;
	return link;
}

static void BlobPack_ApplyControlPayload(const u8 *p, u8 hdr_seq)
{
	static u8 last_mode_req = 0xFF;
	static u8 last_motion_model = 0xFF;
	static u8 last_light_en = 0xFF;
	static u8 last_light_mode = 0xFF;
	u8 control_mode;
	u8 motion_drive_info;
	u8 light_info;
	u8 clear_fault;

	control_mode = p[10];
	motion_drive_info = p[11];
	clear_fault = p[12];
	light_info = p[13];

	arb_state.jetson_cmd.seq = hdr_seq;
	arb_state.jetson_cmd.mode_req = control_mode;
	arb_state.jetson_cmd.v = BlobPack_GetS16BE(p, 4);
	arb_state.jetson_cmd.omega = BlobPack_GetS16BE(p, 6);
	arb_state.jetson_cmd.steer = BlobPack_GetS16BE(p, 8);
	arb_state.jetson_cmd.motion_model = motion_drive_info & 0x0F;
	arb_state.jetson_cmd.light_en = light_info & 0x01;
	arb_state.jetson_cmd.light_mode = (light_info >> 1) & 0x01;
	arb_state.jetson_cmd.clear_error = clear_fault;
	arb_state.jetson_cmd.valid = 1;

	if(last_mode_req != arb_state.jetson_cmd.mode_req)
	{
		last_mode_req = arb_state.jetson_cmd.mode_req;
		if(arb_state.jetson_cmd.mode_req == 1)
			Arbiter_SetMode(CAN_MODE_CAN_CTRL);
		else
			Arbiter_SetMode(CAN_MODE_STANDBY);
	}
	if(last_motion_model != arb_state.jetson_cmd.motion_model)
	{
		last_motion_model = arb_state.jetson_cmd.motion_model;
		Arbiter_SetMotionMode((CanMotionMode_t)(arb_state.jetson_cmd.motion_model & 0x03));
	}
	if(last_light_en != arb_state.jetson_cmd.light_en ||
	   last_light_mode != arb_state.jetson_cmd.light_mode)
	{
		last_light_en = arb_state.jetson_cmd.light_en;
		last_light_mode = arb_state.jetson_cmd.light_mode;
		Arbiter_SetLight(arb_state.jetson_cmd.light_en ? 1 : 0,
			arb_state.jetson_cmd.light_mode ? 1 : 0);
	}
	if(arb_state.jetson_cmd.clear_error != 0)
		Arbiter_ClearError((CanErrorClear_t)arb_state.jetson_cmd.clear_error);

	if(!arb_state.jetson_seq_inited || arb_state.last_jetson_seq != hdr_seq)
	{
		arb_state.last_jetson_seq = hdr_seq;
		arb_state.jetson_seq_inited = 1;
		Arbiter_UpdateHeartbeat();
	}

	s_blob_cmd_log_div++;
	if(s_blob_cmd_log_div >= 50)
	{
		s_blob_cmd_log_div = 0;
		s_log_cmd_seq = hdr_seq;
		s_log_cmd_v = arb_state.jetson_cmd.v;
		s_log_cmd_mode = arb_state.jetson_cmd.mode_req;
		s_log_cmd_arb = (u8)arb_state.current_mode;
		s_log_cmd = 1;
	}
}

static void BlobPack_ApplySensorCfg(const u8 *p)
{
	(void)BlobPack_GetU16BE(p, 4);
	(void)p[6];
}

static u8 BlobPack_SendMotion(const ArbiterState_t *state)
{
	u8 p[BLOB_PAYLOAD_MOTION];
	u32 ts = BlobPack_NowMs();

	if(!state)
		return 0;

	BlobPack_PutU32BE(p, 0, ts);
	p[4] = state->sys_status.system_status;
	p[5] = state->motion_mode_fb.motion_mode;
	p[6] = (u8)((state->motion_mode_fb.motion_mode & 0x0F) |
	            ((state->motion_mode_fb.drive_mode & 0x0F) << 4));
	BlobPack_PutU32BE(p, 7, state->sys_status.fault_info);
	BlobPack_PutU16BE(p, 11, state->sys_status.battery_voltage);
	BlobPack_PutS16BE(p, 13, state->motion_fb.linear_speed);
	BlobPack_PutS16BE(p, 15, state->motion_fb.spin_speed);
	BlobPack_PutS16BE(p, 17, state->motion_fb.steering_angle);
	BlobPack_PutS16BE(p, 19, state->wheel_angle.steer5);
	BlobPack_PutS16BE(p, 21, state->wheel_angle.steer6);
	BlobPack_PutS16BE(p, 23, state->wheel_angle.steer7);
	BlobPack_PutS16BE(p, 25, state->wheel_angle.steer8);
	BlobPack_PutS16BE(p, 27, state->wheel_speed.wheel1);
	BlobPack_PutS16BE(p, 29, state->wheel_speed.wheel2);
	BlobPack_PutS16BE(p, 31, state->wheel_speed.wheel3);
	BlobPack_PutS16BE(p, 33, state->wheel_speed.wheel4);
	p[35] = 0;
	p[36] = 0;
	p[37] = 0;
	p[38] = 0;
	p[39] = 0;

	return BlobLink_Send(BLOB_MSG_MOTION, s_blob_tx_seq++, p, BLOB_PAYLOAD_MOTION);
}

static u8 BlobPack_SendSensor(u16 f, u16 b, u16 l, u16 r)
{
	u8 p[BLOB_PAYLOAD_SENSOR];
	u32 ts = BlobPack_NowMs();

	BlobPack_PutU32BE(p, 0, ts);
	BlobPack_PutU16BE(p, 4, f);
	BlobPack_PutU16BE(p, 6, b);
	BlobPack_PutU16BE(p, 8, l);
	BlobPack_PutU16BE(p, 10, r);
	BlobPack_PutU32BE(p, 12, ts);
	BlobPack_PutU32BE(p, 16, ts);
	BlobPack_PutU32BE(p, 20, ts);
	BlobPack_PutU32BE(p, 24, ts);

	return BlobLink_Send(BLOB_MSG_SENSOR, s_blob_tx_seq++, p, BLOB_PAYLOAD_SENSOR);
}

static u8 BlobPack_SendMcuStatus(const ArbiterState_t *state,
	u16 f, u16 b, u16 l, u16 r, u16 nearest_mm)
{
	u8 p[BLOB_PAYLOAD_MCU_STATUS];
	u32 ts = BlobPack_NowMs();

	if(!state)
		return 0;

	BlobPack_PutU32BE(p, 0, ts);
	p[4] = s_blob_tx_seq;
	p[5] = BlobPack_MapSafetyState(state->current_mode);
	p[6] = BlobPack_BuildLinkFlags(state);
	p[7] = BlobPack_CalcLimitFactor(state);
	BlobPack_PutS16BE(p, 8, state->output.v);
	BlobPack_PutS16BE(p, 10, state->output.omega);
	BlobPack_PutS16BE(p, 12, state->output.steering);
	BlobPack_PutU16BE(p, 14, f);
	BlobPack_PutU16BE(p, 16, b);
	BlobPack_PutU16BE(p, 18, l);
	BlobPack_PutU16BE(p, 20, r);
	BlobPack_PutU32BE(p, 22, ts);
	BlobPack_PutU32BE(p, 26, ts);
	BlobPack_PutU32BE(p, 30, ts);
	BlobPack_PutU32BE(p, 34, ts);
	BlobPack_PutU16BE(p, 38, nearest_mm);
	p[40] = state->last_jetson_seq;
	p[41] = 0;

	return BlobLink_Send(BLOB_MSG_MCU_STATUS, s_blob_tx_seq++, p, BLOB_PAYLOAD_MCU_STATUS);
}

static void BlobPack_PackMotorCompact(u8 *dst, const ArbiterState_t *state, u8 motor_idx)
{
	BlobPack_PutS16BE(dst, 0, state->motor_high[motor_idx].speed_rpm);
	BlobPack_PutS16BE(dst, 2, state->motor_high[motor_idx].current);
	BlobPack_PutU16BE(dst, 4, state->motor_low[motor_idx].voltage);
	dst[6] = (u8)state->motor_low[motor_idx].motor_temp;
	dst[7] = state->motor_low[motor_idx].status;
	BlobPack_PutU16BE(dst, 8, (u16)(state->motor_high[motor_idx].position & 0xFFFF));
}

static u8 BlobPack_SendMotorBlock(u8 msg_id, u8 base_motor)
{
	u8 p[BLOB_PAYLOAD_MOTOR04];
	u32 ts = BlobPack_NowMs();
	u8 i;

	BlobPack_PutU32BE(p, 0, ts);
	for(i = 0; i < 4; i++)
		BlobPack_PackMotorCompact(&p[4 + i * 10], &arb_state, (u8)(base_motor + i));

	return BlobLink_Send(msg_id, s_blob_tx_seq++, p, BLOB_PAYLOAD_MOTOR04);
}

static u8 BlobPack_SendEnergy(const ArbiterState_t *state)
{
	u8 p[BLOB_PAYLOAD_ENERGY];
	u32 ts = BlobPack_NowMs();

	if(!state)
		return 0;

	BlobPack_PutU32BE(p, 0, ts);
	BlobPack_PutS32BE(p, 4, state->odometer.front_left);
	BlobPack_PutS32BE(p, 8, state->odometer.front_right);
	BlobPack_PutS32BE(p, 12, state->odometer.rear_left);
	BlobPack_PutS32BE(p, 16, state->odometer.rear_right);
	p[20] = state->bms_data.soc;
	p[21] = state->bms_data.soh;
	BlobPack_PutU16BE(p, 22, state->bms_data.voltage);
	BlobPack_PutS16BE(p, 24, state->bms_data.current);
	BlobPack_PutS16BE(p, 26, state->bms_data.temperature);
	p[28] = state->bms_alarm.alarm_status1;
	p[29] = state->bms_alarm.alarm_status2;
	p[30] = state->bms_alarm.warning_status1;
	p[31] = state->bms_alarm.warning_status2;
	p[32] = state->remote_ctrl.sw;
	p[33] = (u8)state->remote_ctrl.right_lr;
	p[34] = (u8)state->remote_ctrl.right_ud;
	p[35] = (u8)state->remote_ctrl.left_ud;
	p[36] = (u8)state->remote_ctrl.left_lr;
	p[37] = (u8)state->remote_ctrl.vra;
	p[38] = 0;
	p[39] = 0;
	p[40] = 0;

	return BlobLink_Send(BLOB_MSG_ENERGY, s_blob_tx_seq++, p, BLOB_PAYLOAD_ENERGY);
}

static u8 BlobPack_SendMotorPos(const ArbiterState_t *state)
{
	u8 p[BLOB_PAYLOAD_MOTOR_POS];
	u32 ts = BlobPack_NowMs();
	u8 i;

	if(!state)
		return 0;

	BlobPack_PutU32BE(p, 0, ts);
	for(i = 0; i < 8; i++)
		BlobPack_PutS32BE(p, (u16)(4 + i * 4), state->motor_high[i].position);

	return BlobLink_Send(BLOB_MSG_MOTOR_POS, s_blob_tx_seq++, p, BLOB_PAYLOAD_MOTOR_POS);
}

void BlobPack_HandleDownlink(const blob_rx_frame_t *frame)
{
	if(!frame)
		return;

	switch(frame->msg_id)
	{
		case BLOB_MSG_CONTROL:
			s_blob_rx_ctrl++;
			if(s_blob_rx_ctrl == 1u)
			{
				s_log_first_rx_seq = frame->seq;
				s_log_first_rx_len = frame->payload_len;
				s_log_first_rx = 1;
			}
			if(frame->payload_len == BLOB_PAYLOAD_CONTROL)
				BlobPack_ApplyControlPayload(frame->payload, frame->seq);
			else
			{
				s_log_bad_len_val = frame->payload_len;
				s_log_bad_len = 1;
			}
			break;
		case BLOB_MSG_SENSOR_CFG:
			s_blob_rx_other++;
			if(frame->payload_len == BLOB_PAYLOAD_SENSOR_CFG)
				BlobPack_ApplySensorCfg(frame->payload);
			break;
		default:
			s_blob_rx_other++;
			break;
	}
}

u32 BlobPack_GetRxCtrlCount(void)
{
	return s_blob_rx_ctrl;
}

u32 BlobPack_GetRxOtherCount(void)
{
	return s_blob_rx_other;
}

void BlobPack_FlushDebugLog(void)
{
	u8 first;
	u8 cmd;
	u8 bad;

	taskENTER_CRITICAL();
	first = s_log_first_rx;
	s_log_first_rx = 0;
	cmd = s_log_cmd;
	s_log_cmd = 0;
	bad = s_log_bad_len;
	s_log_bad_len = 0;
	taskEXIT_CRITICAL();

	if(first)
	{
		static char line[64];
		int n;

		n = snprintf(line, sizeof(line),
			"[JETSON BLOB RX] first 0x01 seq=%u len=%u\r\n",
			(unsigned)s_log_first_rx_seq, (unsigned)s_log_first_rx_len);
		if(n > 0)
			printf("%s", line);
	}
	if(cmd)
	{
		static char line[72];
		int n;
		u8 arb = s_log_cmd_arb;

		if(arb > ARBITER_MODE_RECOVERING)
			arb = ARBITER_MODE_DEGRADED;
		n = snprintf(line, sizeof(line),
			"[JETSON BLOB CMD] seq=%u v=%d mode=%u ARB=%s\r\n",
			(unsigned)s_log_cmd_seq,
			(int)s_log_cmd_v,
			(unsigned)s_log_cmd_mode,
			ARBITER_MODE_NAMES[arb]);
		if(n > 0)
			printf("%s", line);
	}
	if(bad)
	{
		static char line[56];
		int n;

		n = snprintf(line, sizeof(line),
			"[JETSON BLOB RX] 0x01 bad len=%u expect=%u\r\n",
			(unsigned)s_log_bad_len_val, (unsigned)BLOB_PAYLOAD_CONTROL);
		if(n > 0)
			printf("%s", line);
	}
}

void BlobPack_UplinkTick(const ArbiterState_t *state,
	u16 sonar_f, u16 sonar_b, u16 sonar_l, u16 sonar_r, u16 nearest_mm)
{
	s_uplink_div++;

	/* 心跳丢失时也保持 0x02/0x03/0x04，便于 gateway 与本地调试 */
	if(state->heartbeat_lost)
	{
		BlobPack_SendMotion(state);
		if((s_uplink_div % 2u) == 0u)
			BlobPack_SendMcuStatus(state, sonar_f, sonar_b, sonar_l, sonar_r, nearest_mm);
		if((s_uplink_div % 4u) == 0u)
			BlobPack_SendSensor(sonar_f, sonar_b, sonar_l, sonar_r);
		return;
	}

#if BLOB_UPLINK_MINIMAL
	/* 联调档位：必保 0x02+0x03@50Hz；0x04 每 40ms（以太网）或 80ms */
	BlobPack_SendMotion(state);
	BlobPack_SendMcuStatus(state, sonar_f, sonar_b, sonar_l, sonar_r, nearest_mm);

#if JETSON_LINK_ETH
	if((s_uplink_div % 2u) == 0u)
#else
	if((s_uplink_div % 4u) == 0u)
#endif
		BlobPack_SendSensor(sonar_f, sonar_b, sonar_l, sonar_r);

	if((s_uplink_div % 8u) == 0u)
	{
		if(s_motor_toggle == 0)
		{
			BlobPack_SendMotorBlock(BLOB_MSG_MOTOR04, 0);
			s_motor_toggle = 1;
		}
		else
		{
			BlobPack_SendMotorBlock(BLOB_MSG_MOTOR58, 4);
			s_motor_toggle = 0;
		}
	}

	if((s_uplink_div % 20u) == 0u)
		BlobPack_SendEnergy(state);

	if((s_uplink_div % 50u) == 0u)
		BlobPack_SendMotorPos(state);
#else
	/* 全量档位：每 tick 0x02+0x03，其余错峰 */
	BlobPack_SendMotion(state);
	BlobPack_SendMcuStatus(state, sonar_f, sonar_b, sonar_l, sonar_r, nearest_mm);

	if((s_uplink_div % 2u) == 0u)
		BlobPack_SendSensor(sonar_f, sonar_b, sonar_l, sonar_r);

	if((s_uplink_div % 4u) == 0u)
	{
		if(s_motor_toggle == 0)
		{
			BlobPack_SendMotorBlock(BLOB_MSG_MOTOR04, 0);
			s_motor_toggle = 1;
		}
		else
		{
			BlobPack_SendMotorBlock(BLOB_MSG_MOTOR58, 4);
			s_motor_toggle = 0;
		}
	}

	if((s_uplink_div % 10u) == 0u)
		BlobPack_SendEnergy(state);

	if((s_uplink_div % 50u) == 0u)
		BlobPack_SendMotorPos(state);
#endif
}

void BlobPack_SendGps(const GPS_Data_t *gps)
{
	u8 p[BLOB_PAYLOAD_GPS];
	u8 flags = 0;
	u16 hdop_x100;
	u16 speed_cms;
	s32 lat_e7 = 0;
	s32 lon_e7 = 0;
	s16 heading_x100 = 0;
	s16 alt_dm = (s16)0x7FFF;
	u8 heading_valid;
	u32 ts = BlobPack_NowMs();

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

	BlobPack_PutU32BE(p, 0, ts);
	p[4] = flags;
	p[5] = gps->num_sv;
	BlobPack_PutU16BE(p, 6, hdop_x100);
	BlobPack_PutU16BE(p, 8, speed_cms);
	BlobPack_PutS32BE(p, 10, lat_e7);
	BlobPack_PutS32BE(p, 14, lon_e7);
	BlobPack_PutS16BE(p, 18, heading_x100);
	BlobPack_PutS16BE(p, 20, alt_dm);
	BlobPack_PutU32BE(p, 22, GPS_GetUtcUnixSec());
	p[26] = 0;
	p[27] = 0;
	p[28] = 0;
	p[29] = 0;
	p[30] = 0;
	p[31] = 0;

	BlobLink_Send(BLOB_MSG_GPS, s_blob_tx_seq++, p, BLOB_PAYLOAD_GPS);
}

#endif
