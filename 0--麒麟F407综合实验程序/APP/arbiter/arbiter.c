#include "arbiter.h"
#include "can.h"
#include "usart.h"
#include "stdio.h"
#include "SysTick.h"
#include "time.h"

// 外部声明 OSTime（在 time.c 中定义）
extern volatile u32 OSTime;

static s16 Chassis_ParseS16BE(u8 hi, u8 lo);
static s16 Chassis_MotionAbs(s16 v);

// ============================================================================
// 全局变量
// ============================================================================

// 状态名称字符串
const char* ARBITER_MODE_NAMES[] = {
	"NORMAL",       // 正常模式
	"SPEED_LIMIT",  // 限速模式
	"DEGRADED",     // 降级模式
	"EMERGENCY",    // 紧急停车
	"RECOVERING"    // 恢复中
};

// 仲裁器状态实例
ArbiterState_t arb_state;

/* ProcessDirectionalPolicy 最后命中的规则号，供串口调试 */
static u8 arb_last_policy_rule = 0;

#define ARB_CMD_FORWARD_MM_S       100
#define ARB_CMD_SLOW_FORWARD_MM_S   65
#define ARB_CMD_BACKWARD_MM_S      (-75)
#define ARB_CMD_TURN_MOVE_MM_S      55   /* 阿克曼转向线速度（必须非0） */
#define ARB_CMD_ESCAPE_BACK_MM_S   (-60) /* 前方受阻时带转向后退 */
#define ARB_CMD_TURN_OMEGA          320  /* 0.001rad/s，自旋 */
#define ARB_CMD_TURN_STEER          280  /* 0.001rad≈16°，阿克曼小角转弯 */
#define ARB_STEER_CMD_LEFT          ARB_CMD_TURN_STEER   /* 0x111：左转=正值 */
#define ARB_STEER_CMD_RIGHT         (-ARB_CMD_TURN_STEER) /* 0x111：右转=负值 */
#define ARB_SLIDE_ANGLE_90          1570  /* 斜移模式 0.001rad≈90°（上限±1571） */
#define ARB_SLIDE_CMD_LEFT          ARB_SLIDE_ANGLE_90
#define ARB_SLIDE_CMD_RIGHT         (-ARB_SLIDE_ANGLE_90)
#define ARB_SLIDE_SPEED_MM_S         70   /* 斜移/90°转向，低于手册700限制 */
#define ARB_INTENT_STEER_DEADZONE   80    /* 0.001rad，低于此视为无转向意图 */
#define ARB_INTENT_OMEGA_DEADZONE   80    /* 0.001rad/s */
#define ARB_INTENT_MIN_MOVE_MM_S    20    /* Jetson v 低于此用策略默认兜底 */
#define ARB_TICK_MS                 20    // OSTime 在 TIM4 20ms 中断里自增
#define ARB_MS_TO_TICKS(ms)        (((ms) + ARB_TICK_MS - 1) / ARB_TICK_MS)

// ============================================================================
// 内部辅助函数声明
// ============================================================================

static void Arbiter_SwitchMode(ArbiterMode_t new_mode);
static void Arbiter_ProcessNormalMode(void);
static void Arbiter_ProcessSpeedLimitMode(void);
static void Arbiter_ProcessDegradedMode(void);
static void Arbiter_ProcessEmergencyMode(void);
static void Arbiter_ProcessRecoveringMode(void);
static void Arbiter_CheckHeartbeat(void);
static u8 Arbiter_IsDangerDist(u16 dist_mm);
static u8 Arbiter_IsObstacleDist(u16 dist_mm);
static u8 Arbiter_IsValidDist(u16 dist_mm);
static u16 Arbiter_SanitizeDist(u16 dist_mm);
static u16 Arbiter_MinDistance4(u16 a, u16 b, u16 c, u16 d);
static s16 Arbiter_SteerToSaferSide(u16 left_mm, u8 left_valid, u16 right_mm, u8 right_valid);
static void Arbiter_ProcessJetsonIntentPolicy(void);
static u8 Arbiter_IsStopIntent(void);
static void Arbiter_ApplyStopOutput(void);

static void Arbiter_SendZeroMotion111(void)
{
	u8 txbuf[8] = {0};

	CAN1_Send_Msg_WithID(CAN_ARBITER_CMD_ID, txbuf, 8);
}

/* 策略斜移模式状态 */
static CanMotionMode_t arb_policy_chassis_mode = CAN_MOTION_ACKERMANN;

static void Arbiter_PolicyEnsureMotionMode(CanMotionMode_t mode)
{
	if(arb_policy_chassis_mode != mode)
	{
		Arbiter_SetMotionMode(mode);
		arb_policy_chassis_mode = mode;
	}
}

static void Arbiter_PolicySetAckermannStraight(s16 v)
{
	Arbiter_PolicyEnsureMotionMode(CAN_MOTION_ACKERMANN);
	arb_state.output.v = v;
	arb_state.output.steering = 0;
	arb_state.output.omega = 0;
}

/* 斜移模式 90°：用于所有需要转向的避障 */
static void Arbiter_PolicyMoveSlide90Ex(s16 slide_angle, s16 speed_mm_s)
{
	s16 spd = speed_mm_s;

	if(spd < ARB_INTENT_MIN_MOVE_MM_S)
		spd = ARB_SLIDE_SPEED_MM_S;
	Arbiter_PolicyEnsureMotionMode(CAN_MOTION_SLIDE);
	arb_state.output.v = spd;
	arb_state.output.steering = slide_angle;
	arb_state.output.omega = 0;
}

static void Arbiter_PolicyMoveSlide90(s16 slide_angle)
{
	Arbiter_PolicyMoveSlide90Ex(slide_angle, ARB_SLIDE_SPEED_MM_S);
}

// ============================================================================
// 函数实现
// ============================================================================

/*
 * 距离判定（输入均为 SetObstacleDistances 写入的值）：
 *   UNKNOWN 0xFFFF  无效/未更新，不参与最近距，策略侧当“未知有障”
 *   FAILSAFE 0      TIMEOUT 归一化值，<=NEAR 触发危险/急停
 *   1..MAX          正常超声波读数
 */

static u8 Arbiter_IsDangerDist(u16 dist_mm)
{
	return (dist_mm != ARBITER_DIST_UNKNOWN && dist_mm <= ARBITER_OBSTACLE_NEAR_MM);
}

static u8 Arbiter_IsObstacleDist(u16 dist_mm)
{
	return (dist_mm != ARBITER_DIST_UNKNOWN && dist_mm <= ARBITER_OBSTACLE_FAR_MM);
}

static u8 Arbiter_IsValidDist(u16 dist_mm)
{
	return (dist_mm != ARBITER_DIST_UNKNOWN && dist_mm <= ARBITER_DIST_MAX_MM);
}

static u16 Arbiter_SanitizeDist(u16 dist_mm)
{
	if(dist_mm == ARBITER_DIST_UNKNOWN)
		return ARBITER_DIST_UNKNOWN;
	if(dist_mm <= ARBITER_DIST_MAX_MM)
		return dist_mm;
	return ARBITER_DIST_UNKNOWN;
}

static void Arbiter_FormatDistMm(u16 mm, char *buf, u8 buf_len)
{
	if(mm == ARBITER_DIST_UNKNOWN)
		sprintf(buf, "---");
	else
		sprintf(buf, "%u", (unsigned int)mm);
}

static void Arbiter_PolicyEmergencyStop(u8 rule)
{
	Arbiter_PolicyEnsureMotionMode(CAN_MOTION_ACKERMANN);
	arb_state.output.v = 0;
	arb_state.output.steering = 0;
	arb_state.output.omega = 0;
	arb_state.output.emergency = 1;
	arb_last_policy_rule = rule;
}

/* 四向均进入危险区(<=NEAR)才真急停；单向前近但侧/后有空间走 R2/R6 等 */
static u8 Arbiter_AllDirectionsDanger(void)
{
	u16 f = arb_state.obstacle_dist.front;
	u16 b = arb_state.obstacle_dist.back;
	u16 l = arb_state.obstacle_dist.left;
	u16 r = arb_state.obstacle_dist.right;
	u8 vf = (arb_state.obstacle_valid_mask & 0x01) ? 1 : 0;
	u8 vb = (arb_state.obstacle_valid_mask & 0x02) ? 1 : 0;
	u8 vl = (arb_state.obstacle_valid_mask & 0x04) ? 1 : 0;
	u8 vr = (arb_state.obstacle_valid_mask & 0x08) ? 1 : 0;
	u8 df = vf ? Arbiter_IsDangerDist(f) : 1;
	u8 db = vb ? Arbiter_IsDangerDist(b) : 1;
	u8 dl = vl ? Arbiter_IsDangerDist(l) : 1;
	u8 dr = vr ? Arbiter_IsDangerDist(r) : 1;

	return (df && db && dl && dr) ? 1 : 0;
}

static u16 Arbiter_MinDistance4(u16 a, u16 b, u16 c, u16 d)
{
	u16 minv = ARBITER_DIST_UNKNOWN;

	if(Arbiter_IsValidDist(a) && a < minv) minv = a;
	if(Arbiter_IsValidDist(b) && b < minv) minv = b;
	if(Arbiter_IsValidDist(c) && c < minv) minv = c;
	if(Arbiter_IsValidDist(d) && d < minv) minv = d;
	return minv;
}

static s16 Arbiter_SteerToSaferSide(u16 left_mm, u8 left_valid, u16 right_mm, u8 right_valid)
{
	/* 只朝“有有效数据且更空”的一侧转 */
	if(left_valid && !right_valid)
		return ARB_STEER_CMD_LEFT;
	if(!left_valid && right_valid)
		return ARB_STEER_CMD_RIGHT;
	if(!left_valid && !right_valid)
		return ARB_STEER_CMD_RIGHT;
	if(right_mm >= left_mm)
		return ARB_STEER_CMD_RIGHT;
	return ARB_STEER_CMD_LEFT;
}

/* 策略比“哪边更空”：UNKNOWN/无效→0(最糟)；FAILSAFE(0)也是 0 */
static u16 Arbiter_PolicyDist(u16 dist_mm, u8 valid)
{
	if(!valid || dist_mm == ARBITER_DIST_UNKNOWN)
		return ARBITER_DIST_FAILSAFE_MM;
	if(dist_mm > ARBITER_DIST_MAX_MM)
		return ARBITER_DIST_FAILSAFE_MM;
	return dist_mm;
}

/* 在 back/left/right 中选距离最大：0=后 1=左 2=右 */
static u8 Arbiter_PickFreestBackLeftRight(u16 b, u8 vb, u16 l, u8 vl, u16 r, u8 vr)
{
	u16 db = Arbiter_PolicyDist(b, vb);
	u16 dl = Arbiter_PolicyDist(l, vl);
	u16 dr = Arbiter_PolicyDist(r, vr);

	if(db >= dl && db >= dr)
		return 0;
	if(dl >= dr)
		return 1;
	return 2;
}

/* 在 front/left/right 中选距离最大：0=前 1=左 2=右 */
static u8 Arbiter_PickFreestFrontLeftRight(u16 f, u8 vf, u16 l, u8 vl, u16 r, u8 vr)
{
	u16 df = Arbiter_PolicyDist(f, vf);
	u16 dl = Arbiter_PolicyDist(l, vl);
	u16 dr = Arbiter_PolicyDist(r, vr);

	if(df >= dl && df >= dr)
		return 0;
	if(dl >= dr)
		return 1;
	return 2;
}

/* 四向中选距离最大：0=前 1=后 2=左 3=右（R2 等多向有障时用） */
static u8 Arbiter_PickFreestAll4(u16 f, u8 vf, u16 b, u8 vb, u16 l, u8 vl, u16 r, u8 vr)
{
	u16 df = Arbiter_PolicyDist(f, vf);
	u16 db = Arbiter_PolicyDist(b, vb);
	u16 dl = Arbiter_PolicyDist(l, vl);
	u16 dr = Arbiter_PolicyDist(r, vr);
	u8 best = 0;
	u16 best_d = df;

	if(db > best_d) { best_d = db; best = 1; }
	if(dl > best_d) { best_d = dl; best = 2; }
	if(dr > best_d) { best = 3; }
	return best;
}

/* Jetson 运动意图（SPEED_LIMIT / DEGRADED 均以之为准，不再 autonomous 乱走） */
typedef enum
{
	ARB_INTENT_STOP = 0,
	ARB_INTENT_FORWARD,
	ARB_INTENT_BACKWARD,
	ARB_INTENT_SLIDE_LEFT,
	ARB_INTENT_SLIDE_RIGHT,
	ARB_INTENT_SPIN,
	ARB_INTENT_ACKERMAN
} ArbMotionIntent_t;

static u8 Arbiter_IsStopIntent(void)
{
	JetsonCmd_t *c = &arb_state.jetson_cmd;

	if(c->motion_model == CAN_MOTION_PARK)
		return 1;
	if(c->v != 0)
		return 0;
	if(c->omega > ARB_INTENT_OMEGA_DEADZONE || c->omega < -ARB_INTENT_OMEGA_DEADZONE)
		return 0;
	if(c->steer > ARB_INTENT_STEER_DEADZONE || c->steer < -ARB_INTENT_STEER_DEADZONE)
		return 0;
	if(c->motion_model == CAN_MOTION_SPIN)
		return 0;
	return 1;
}

static void Arbiter_ApplyStopOutput(void)
{
	Arbiter_PolicyEnsureMotionMode(CAN_MOTION_ACKERMANN);
	arb_state.output.v = 0;
	arb_state.output.omega = 0;
	arb_state.output.steering = 0;
	arb_state.output.emergency = 0;
	arb_last_policy_rule = 0;
}

static ArbMotionIntent_t Arbiter_GetMotionIntent(void)
{
	JetsonCmd_t *c = &arb_state.jetson_cmd;

	if(Arbiter_IsStopIntent())
		return ARB_INTENT_STOP;
	if(c->motion_model == CAN_MOTION_SPIN)
		return ARB_INTENT_SPIN;
	if(c->motion_model == CAN_MOTION_SLIDE)
	{
		if(c->steer >= ARB_INTENT_STEER_DEADZONE)
			return ARB_INTENT_SLIDE_LEFT;
		if(c->steer <= -ARB_INTENT_STEER_DEADZONE)
			return ARB_INTENT_SLIDE_RIGHT;
		if(c->v > 0)
			return ARB_INTENT_FORWARD;
		if(c->v < 0)
			return ARB_INTENT_BACKWARD;
		return ARB_INTENT_STOP;
	}
	if(c->v > 0)
		return ARB_INTENT_FORWARD;
	if(c->v < 0)
		return ARB_INTENT_BACKWARD;
	if(c->omega > ARB_INTENT_OMEGA_DEADZONE || c->omega < -ARB_INTENT_OMEGA_DEADZONE)
		return ARB_INTENT_SPIN;
	if(c->steer > ARB_INTENT_STEER_DEADZONE || c->steer < -ARB_INTENT_STEER_DEADZONE)
		return ARB_INTENT_ACKERMAN;
	return ARB_INTENT_STOP;
}

static s32 Arbiter_CalcDistFactorPct(u16 dist_mm, u8 valid)
{
	if(!valid || dist_mm == ARBITER_DIST_UNKNOWN)
		return 0;
	if(dist_mm <= ARBITER_OBSTACLE_NEAR_MM)
		return 0;
	if(dist_mm >= ARBITER_OBSTACLE_FAR_MM)
		return 100;
	return ((s32)dist_mm - (s32)ARBITER_OBSTACLE_NEAR_MM) * 100 /
	       ((s32)ARBITER_OBSTACLE_FAR_MM - (s32)ARBITER_OBSTACLE_NEAR_MM);
}

static s16 Arbiter_ScaleJetsonVByDist(s16 v_cmd, u16 dist_mm, u8 valid)
{
	s32 pct = Arbiter_CalcDistFactorPct(dist_mm, valid);
	s32 scaled = ((s32)v_cmd * pct) / 100;

	return Arbiter_ClampSpeed((s16)scaled);
}

static s16 Arbiter_JetsonForwardSpeed(u16 dist_mm, u8 valid)
{
	s16 v = arb_state.jetson_cmd.v;
	s16 mag;

	if(v <= 0)
		v = ARB_CMD_FORWARD_MM_S;
	mag = Chassis_MotionAbs(v);
	if(mag < ARB_INTENT_MIN_MOVE_MM_S)
		mag = ARB_CMD_FORWARD_MM_S;
	return Arbiter_ScaleJetsonVByDist(mag, dist_mm, valid);
}

static s16 Arbiter_JetsonBackwardSpeed(u16 dist_mm, u8 valid)
{
	s16 v = arb_state.jetson_cmd.v;
	s16 mag;

	if(v >= 0)
		mag = Chassis_MotionAbs(ARB_CMD_BACKWARD_MM_S);
	else
		mag = Chassis_MotionAbs(v);
	if(mag < ARB_INTENT_MIN_MOVE_MM_S)
		mag = Chassis_MotionAbs(ARB_CMD_BACKWARD_MM_S);
	return -Arbiter_ScaleJetsonVByDist(mag, dist_mm, valid);
}

static s16 Arbiter_JetsonSlideSpeed(void)
{
	s16 mag = Chassis_MotionAbs(arb_state.jetson_cmd.v);

	if(mag < ARB_INTENT_MIN_MOVE_MM_S)
		mag = ARB_SLIDE_SPEED_MM_S;
	return Arbiter_ClampSpeed(mag);
}

static void Arbiter_ApplyForwardIntent(u16 f, u8 vf, u16 b, u8 vb,
	u16 l, u8 vl, u16 r, u8 vr, u8 obs_f, u8 danger_f)
{
	JetsonCmd_t *cmd = &arb_state.jetson_cmd;

	if(!obs_f && !danger_f)
	{
		arb_last_policy_rule = 12;
		Arbiter_PolicySetAckermannStraight(Arbiter_JetsonForwardSpeed(f, vf));
		arb_state.output.steering = cmd->steer;
		return;
	}
	if(vf && !danger_f && f > ARBITER_OBSTACLE_NEAR_MM && f <= ARBITER_OBSTACLE_FAR_MM)
	{
		arb_last_policy_rule = 11;
		Arbiter_PolicySetAckermannStraight(Arbiter_JetsonForwardSpeed(f, vf));
		arb_state.output.steering = cmd->steer;
		return;
	}
	/* 前向受阻：绕行，前方空出后下一周期自动回到 rule 11/12 */
	arb_last_policy_rule = 6;
	switch(Arbiter_PickFreestBackLeftRight(b, vb, l, vl, r, vr))
	{
		case 0:
			Arbiter_PolicySetAckermannStraight(Arbiter_JetsonBackwardSpeed(
				arb_state.obstacle_dist.back, vb));
			break;
		case 1:
			Arbiter_PolicyMoveSlide90Ex(ARB_SLIDE_CMD_LEFT, Arbiter_JetsonSlideSpeed());
			break;
		default:
			Arbiter_PolicyMoveSlide90Ex(ARB_SLIDE_CMD_RIGHT, Arbiter_JetsonSlideSpeed());
			break;
	}
}

static void Arbiter_ApplyBackwardIntent(u16 f, u8 vf, u16 b, u8 vb,
	u16 l, u8 vl, u16 r, u8 vr, u8 obs_b, u8 danger_b)
{
	JetsonCmd_t *cmd = &arb_state.jetson_cmd;

	if(!obs_b && !danger_b)
	{
		arb_last_policy_rule = 7;
		Arbiter_PolicySetAckermannStraight(Arbiter_JetsonBackwardSpeed(b, vb));
		arb_state.output.steering = cmd->steer;
		return;
	}
	if(vb && !danger_b && b > ARBITER_OBSTACLE_NEAR_MM && b <= ARBITER_OBSTACLE_FAR_MM)
	{
		arb_last_policy_rule = 7;
		Arbiter_PolicySetAckermannStraight(Arbiter_JetsonBackwardSpeed(b, vb));
		arb_state.output.steering = cmd->steer;
		return;
	}
	arb_last_policy_rule = 7;
	switch(Arbiter_PickFreestFrontLeftRight(f, vf, l, vl, r, vr))
	{
		case 0:
			Arbiter_PolicySetAckermannStraight(Arbiter_JetsonForwardSpeed(f, vf));
			break;
		case 1:
			Arbiter_PolicyMoveSlide90Ex(ARB_SLIDE_CMD_LEFT, Arbiter_JetsonSlideSpeed());
			break;
		default:
			Arbiter_PolicyMoveSlide90Ex(ARB_SLIDE_CMD_RIGHT, Arbiter_JetsonSlideSpeed());
			break;
	}
}

static void Arbiter_ApplySlideLeftIntent(u16 f, u8 vf, u16 b, u8 vb,
	u8 obs_l, u8 danger_l, u8 obs_r, u8 danger_r)
{
	s16 spd = Arbiter_JetsonSlideSpeed();

	if(!obs_l && !danger_l)
	{
		arb_last_policy_rule = 8;
		Arbiter_PolicyMoveSlide90Ex(ARB_SLIDE_CMD_LEFT, spd);
		return;
	}
	if(!obs_r && !danger_r)
	{
		arb_last_policy_rule = 9;
		Arbiter_PolicyMoveSlide90Ex(ARB_SLIDE_CMD_RIGHT, spd);
		return;
	}
	if(Arbiter_PolicyDist(f, vf) >= Arbiter_PolicyDist(b, vb))
	{
		arb_last_policy_rule = 10;
		Arbiter_PolicySetAckermannStraight(Arbiter_JetsonForwardSpeed(f, vf));
	}
	else
	{
		arb_last_policy_rule = 10;
		Arbiter_PolicySetAckermannStraight(Arbiter_JetsonBackwardSpeed(b, vb));
	}
}

static void Arbiter_ApplySlideRightIntent(u16 f, u8 vf, u16 b, u8 vb,
	u8 obs_l, u8 danger_l, u8 obs_r, u8 danger_r)
{
	s16 spd = Arbiter_JetsonSlideSpeed();

	if(!obs_r && !danger_r)
	{
		arb_last_policy_rule = 9;
		Arbiter_PolicyMoveSlide90Ex(ARB_SLIDE_CMD_RIGHT, spd);
		return;
	}
	if(!obs_l && !danger_l)
	{
		arb_last_policy_rule = 8;
		Arbiter_PolicyMoveSlide90Ex(ARB_SLIDE_CMD_LEFT, spd);
		return;
	}
	if(Arbiter_PolicyDist(f, vf) >= Arbiter_PolicyDist(b, vb))
	{
		arb_last_policy_rule = 10;
		Arbiter_PolicySetAckermannStraight(Arbiter_JetsonForwardSpeed(f, vf));
	}
	else
	{
		arb_last_policy_rule = 10;
		Arbiter_PolicySetAckermannStraight(Arbiter_JetsonBackwardSpeed(b, vb));
	}
}

static void Arbiter_ApplySpinIntent(void)
{
	JetsonCmd_t *cmd = &arb_state.jetson_cmd;

	Arbiter_PolicyEnsureMotionMode(CAN_MOTION_SPIN);
	arb_state.output.v = 0;
	arb_state.output.steering = 0;
	arb_last_policy_rule = 13;
	if(Arbiter_AllDirectionsDanger())
		arb_state.output.omega = 0;
	else
		arb_state.output.omega = cmd->omega;
}

static void Arbiter_ApplyAckermannIntent(u16 f, u8 vf, u16 b, u8 vb,
	u16 l, u8 vl, u16 r, u8 vr, u8 obs_f, u8 danger_f, u8 obs_b, u8 danger_b,
	u8 obs_l, u8 danger_l, u8 obs_r, u8 danger_r)
{
	JetsonCmd_t *cmd = &arb_state.jetson_cmd;

	if(cmd->steer > 0)
	{
		if(!obs_l && !danger_l)
		{
			arb_last_policy_rule = 8;
			Arbiter_PolicyMoveSlide90Ex(ARB_SLIDE_CMD_LEFT, Arbiter_JetsonSlideSpeed());
			return;
		}
	}
	else if(cmd->steer < 0)
	{
		if(!obs_r && !danger_r)
		{
			arb_last_policy_rule = 9;
			Arbiter_PolicyMoveSlide90Ex(ARB_SLIDE_CMD_RIGHT, Arbiter_JetsonSlideSpeed());
			return;
		}
	}
	if(cmd->v > 0 || cmd->steer != 0)
		Arbiter_ApplyForwardIntent(f, vf, b, vb, l, vl, r, vr, obs_f, danger_f);
	else if(cmd->v < 0)
		Arbiter_ApplyBackwardIntent(f, vf, b, vb, l, vl, r, vr, obs_b, danger_b);
	else
	{
		arb_last_policy_rule = 13;
		Arbiter_PolicyEnsureMotionMode(CAN_MOTION_ACKERMANN);
		arb_state.output.v = ARB_CMD_TURN_MOVE_MM_S;
		arb_state.output.steering = cmd->steer;
		arb_state.output.omega = 0;
	}
}

/* 以 Jetson 运动意图为主：先尝试指令方向+限速，受阻再绕行，路径空出后回到指令方向 */
static void Arbiter_ProcessJetsonIntentPolicy(void)
{
	u16 f = arb_state.obstacle_dist.front;
	u16 b = arb_state.obstacle_dist.back;
	u16 l = arb_state.obstacle_dist.left;
	u16 r = arb_state.obstacle_dist.right;
	u8 valid_f = (arb_state.obstacle_valid_mask & 0x01) ? 1 : 0;
	u8 valid_b = (arb_state.obstacle_valid_mask & 0x02) ? 1 : 0;
	u8 valid_l = (arb_state.obstacle_valid_mask & 0x04) ? 1 : 0;
	u8 valid_r = (arb_state.obstacle_valid_mask & 0x08) ? 1 : 0;
	u8 obs_f, obs_b, obs_l, obs_r;
	u8 danger_f, danger_b, danger_l, danger_r;
	ArbMotionIntent_t intent;

	arb_state.output.emergency = 0;
	arb_state.output.omega = 0;

	obs_f = valid_f ? Arbiter_IsObstacleDist(f) : 1;
	obs_b = valid_b ? Arbiter_IsObstacleDist(b) : 1;
	obs_l = valid_l ? Arbiter_IsObstacleDist(l) : 1;
	obs_r = valid_r ? Arbiter_IsObstacleDist(r) : 1;
	danger_f = valid_f ? Arbiter_IsDangerDist(f) : 1;
	danger_b = valid_b ? Arbiter_IsDangerDist(b) : 1;
	danger_l = valid_l ? Arbiter_IsDangerDist(l) : 1;
	danger_r = valid_r ? Arbiter_IsDangerDist(r) : 1;

	if((arb_state.obstacle_valid_mask & 0x0F) == 0)
	{
		Arbiter_PolicyEmergencyStop(1);
		return;
	}
	if(Arbiter_AllDirectionsDanger())
	{
		Arbiter_PolicyEmergencyStop(1);
		return;
	}
	if(Arbiter_IsStopIntent())
	{
		Arbiter_ApplyStopOutput();
		return;
	}

	intent = Arbiter_GetMotionIntent();
	switch(intent)
	{
		case ARB_INTENT_FORWARD:
			Arbiter_ApplyForwardIntent(f, valid_f, b, valid_b, l, valid_l, r, valid_r, obs_f, danger_f);
			break;
		case ARB_INTENT_BACKWARD:
			Arbiter_ApplyBackwardIntent(f, valid_f, b, valid_b, l, valid_l, r, valid_r, obs_b, danger_b);
			break;
		case ARB_INTENT_SLIDE_LEFT:
			Arbiter_ApplySlideLeftIntent(f, valid_f, b, valid_b, obs_l, danger_l, obs_r, danger_r);
			break;
		case ARB_INTENT_SLIDE_RIGHT:
			Arbiter_ApplySlideRightIntent(f, valid_f, b, valid_b, obs_l, danger_l, obs_r, danger_r);
			break;
		case ARB_INTENT_SPIN:
			Arbiter_ApplySpinIntent();
			break;
		case ARB_INTENT_ACKERMAN:
			Arbiter_ApplyAckermannIntent(f, valid_f, b, valid_b, l, valid_l, r, valid_r,
				obs_f, danger_f, obs_b, danger_b, obs_l, danger_l, obs_r, danger_r);
			break;
		default:
			Arbiter_ApplyStopOutput();
			break;
	}
}

/*******************************************************************************
* 函 数 名         : Arbiter_Init
* 功能描述		   : 初始化仲裁器
*******************************************************************************/
void Arbiter_Init(void)
{
	// 清零状态
	arb_state.current_mode = ARBITER_MODE_NORMAL;
	arb_state.last_heartbeat_tick = OSTime;
	arb_state.heartbeat_lost = 0;
	arb_state.last_jetson_seq = 0;
	arb_state.jetson_seq_inited = 0;
	arb_state.recover_start_tick = 0;
	
	// 初始化Jetson指令
	arb_state.jetson_cmd.mode_req = 1;
	arb_state.jetson_cmd.v = 0;
	arb_state.jetson_cmd.omega = 0;
	arb_state.jetson_cmd.steer = 0;
	arb_state.jetson_cmd.motion_model = CAN_MOTION_ACKERMANN;
	arb_state.jetson_cmd.light_en = 0;
	arb_state.jetson_cmd.light_mode = 0;
	arb_state.jetson_cmd.clear_error = 0;
	arb_state.jetson_cmd.seq = 0;
	arb_state.jetson_cmd.valid = 0;
	
	// 初始化输出
	arb_state.output.v = 0;
	arb_state.output.omega = 0;
	arb_state.output.steering = 0;
	arb_state.output.mode = ARBITER_MODE_NORMAL;
	arb_state.output.emergency = 0;
	
	// 初始化传感器数据
	arb_state.nearest_dist = ARBITER_DIST_UNKNOWN;
	arb_state.obstacle_detected = 0;
	arb_state.obstacle_dist.front = ARBITER_DIST_UNKNOWN;
	arb_state.obstacle_dist.back = ARBITER_DIST_UNKNOWN;
	arb_state.obstacle_dist.left = ARBITER_DIST_UNKNOWN;
	arb_state.obstacle_dist.right = ARBITER_DIST_UNKNOWN;
	arb_state.obstacle_valid_mask = 0;

	arb_policy_chassis_mode = CAN_MOTION_ACKERMANN;
	
	printf("[Arbiter] Initialized, mode: %s\r\n", ARBITER_MODE_NAMES[arb_state.current_mode]);
}

/*******************************************************************************
* 函 数 名         : Arbiter_UpdateHeartbeat
* 功能描述		   : 更新心跳（收到Jetson数据时调用）
*******************************************************************************/
void Arbiter_UpdateHeartbeat(void)
{
	arb_state.last_heartbeat_tick = OSTime;
	arb_state.heartbeat_lost = 0;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseJetsonCmd
* 功能描述		   : 接收并解析Jetson指令
* 输    入         : frame: 数据帧指针, len: 数据长度
* 输    出         : 0=成功, 1=失败
* 帧格式           : V3.0 24字节 [0xAA][type][seq][payload20][xor]
*******************************************************************************/
u8 Arbiter_ParseJetsonCmd(u8* frame, u8 len)
{
	static u8 last_mode_req = 0xFF;
	static u8 last_motion_model = 0xFF;
	static u8 last_light_en = 0xFF;
	static u8 last_light_mode = 0xFF;
	static u8 jetson_cmd_log_div = 0;
	u8 checksum = 0;
	u8 i;
	
	// 验证长度
	if(len != JETSON_FRAME_LEN)
		return 1;
	
	// 验证帧头
	if(frame[0] != JETSON_FRAME_HEADER)
		return 1;
	
	// 验证帧类型
	if(frame[1] != JETSON_FRAME_TYPE_DOWN)
		return 1;

	// 计算校验和
	for(i = 0; i < JETSON_FRAME_LEN - 1; i++)
	{
		checksum ^= frame[i];
	}
	
	// 验证校验和
	if(checksum != frame[JETSON_FRAME_LEN - 1])
	{
		printf("[Arbiter] Checksum failed: expected 0x%02X, got 0x%02X\r\n", 
		       checksum, frame[JETSON_FRAME_LEN - 1]);
		return 1;
	}

	arb_state.jetson_cmd.seq = frame[2];
	arb_state.jetson_cmd.mode_req = frame[3];
	arb_state.jetson_cmd.v = Chassis_ParseS16BE(frame[4], frame[5]);
	arb_state.jetson_cmd.omega = Chassis_ParseS16BE(frame[6], frame[7]);
	arb_state.jetson_cmd.steer = Chassis_ParseS16BE(frame[8], frame[9]);
	arb_state.jetson_cmd.motion_model = frame[10];
	arb_state.jetson_cmd.light_en = frame[11];
	arb_state.jetson_cmd.light_mode = frame[12];
	arb_state.jetson_cmd.clear_error = frame[13];
	arb_state.jetson_cmd.valid = 1;

	/* 将 V3 下行控制字段映射到底盘 CAN 配置帧 */
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
	{
		Arbiter_ClearError((CanErrorClear_t)arb_state.jetson_cmd.clear_error);
	}

	/* seq 必须变化才刷新心跳，满足 V3 心跳语义 */
	if(!arb_state.jetson_seq_inited || arb_state.last_jetson_seq != arb_state.jetson_cmd.seq)
	{
		arb_state.last_jetson_seq = arb_state.jetson_cmd.seq;
		arb_state.jetson_seq_inited = 1;
		Arbiter_UpdateHeartbeat();
	}

	jetson_cmd_log_div++;
	if(jetson_cmd_log_div >= 25)
	{
		char ts[20];

		jetson_cmd_log_div = 0;
		Log_TsPrefix(ts, sizeof(ts));
		printf("%s[JETSON CMD] seq=%u v=%d steer=%d omega=%d motion=%u\r\n",
			ts,
			(unsigned int)arb_state.jetson_cmd.seq,
			(int)arb_state.jetson_cmd.v,
			(int)arb_state.jetson_cmd.steer,
			(int)arb_state.jetson_cmd.omega,
			(unsigned int)arb_state.jetson_cmd.motion_model);
	}

	return 0;
}

/*******************************************************************************
* 函 数 名         : Arbiter_SetObstacleDistance
* 功能描述		   : 设置最近障碍物距离
*******************************************************************************/
void Arbiter_SetObstacleDistance(u16 dist_mm)
{
	/* 兼容老接口：四方向都使用同一距离 */
	Arbiter_SetObstacleDistances(dist_mm, dist_mm, dist_mm, dist_mm);
}

void Arbiter_SetObstacleDistances(u16 front_mm, u16 back_mm, u16 left_mm, u16 right_mm)
{
	u8 valid_f = Arbiter_IsValidDist(front_mm);
	u8 valid_b = Arbiter_IsValidDist(back_mm);
	u8 valid_l = Arbiter_IsValidDist(left_mm);
	u8 valid_r = Arbiter_IsValidDist(right_mm);

	arb_state.obstacle_dist.front = Arbiter_SanitizeDist(front_mm);
	arb_state.obstacle_dist.back = Arbiter_SanitizeDist(back_mm);
	arb_state.obstacle_dist.left = Arbiter_SanitizeDist(left_mm);
	arb_state.obstacle_dist.right = Arbiter_SanitizeDist(right_mm);
	arb_state.obstacle_valid_mask =
		(valid_f ? 0x01 : 0) |
		(valid_b ? 0x02 : 0) |
		(valid_l ? 0x04 : 0) |
		(valid_r ? 0x08 : 0);

	arb_state.nearest_dist = Arbiter_MinDistance4(
		arb_state.obstacle_dist.front,
		arb_state.obstacle_dist.back,
		arb_state.obstacle_dist.left,
		arb_state.obstacle_dist.right);

	/* 仅在有已知最近距且进入限速区时视为有障碍 */
	arb_state.obstacle_detected = (arb_state.nearest_dist != ARBITER_DIST_UNKNOWN &&
		arb_state.nearest_dist <= ARBITER_OBSTACLE_FAR_MM) ? 1 : 0;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ClampSpeed
* 功能描述		   : 速度限幅（安全防线）
*******************************************************************************/
s16 Arbiter_ClampSpeed(s16 speed)
{
	if(speed > ARBITER_MAX_SPEED_MM_S)
		return ARBITER_MAX_SPEED_MM_S;
	if(speed < -ARBITER_MAX_SPEED_MM_S)
		return -ARBITER_MAX_SPEED_MM_S;
	return speed;
}

/*******************************************************************************
* 函 数 名         : Arbiter_SwitchMode
* 功能描述		   : 切换仲裁模式
*******************************************************************************/
static void Arbiter_SwitchMode(ArbiterMode_t new_mode)
{
	if(arb_state.current_mode != new_mode)
	{
		printf("[Arbiter] Mode switch: %s -> %s\r\n", 
		       ARBITER_MODE_NAMES[arb_state.current_mode],
		       ARBITER_MODE_NAMES[new_mode]);
		
		arb_state.current_mode = new_mode;
		
		// 记录恢复开始时间
		if(new_mode == ARBITER_MODE_RECOVERING)
		{
			arb_state.recover_start_tick = OSTime;
		}
		if(new_mode == ARBITER_MODE_RECOVERING || new_mode == ARBITER_MODE_NORMAL)
		{
			arb_state.output.emergency = 0;
		}
		if(new_mode == ARBITER_MODE_DEGRADED)
		{
			/* 进入降级：清输出与 Jetson 残留转角/角速度，避免底盘继续执行旧指令 */
			arb_state.output.v = 0;
			arb_state.output.omega = 0;
			arb_state.output.steering = 0;
			arb_state.jetson_cmd.steer = 0;
			arb_state.jetson_cmd.omega = 0;
		}
	}
}

/*******************************************************************************
* 函 数 名         : Arbiter_CheckHeartbeat
* 功能描述		   : 检查心跳是否超时
*******************************************************************************/
static void Arbiter_CheckHeartbeat(void)
{
	u32 elapsed = OSTime - arb_state.last_heartbeat_tick;
	
	if(elapsed >= ARB_MS_TO_TICKS(ARBITER_HEARTBEAT_TIMEOUT_MS))
	{
		if(!arb_state.heartbeat_lost)
		{
			printf("[Arbiter] Heartbeat lost! (%lu ms)\r\n",
				(unsigned long)(elapsed * ARB_TICK_MS));
			arb_state.heartbeat_lost = 1;
			/* 链路断开：清残留指令，避免 DEGRADED 仍按旧 v 前进 */
			arb_state.jetson_cmd.v = 0;
			arb_state.jetson_cmd.omega = 0;
			arb_state.jetson_cmd.steer = 0;
		}
	}
	else
	{
		arb_state.heartbeat_lost = 0;
	}
}

/*******************************************************************************
* 函 数 名         : Arbiter_ProcessNormalMode
* 功能描述		   : 状态1：正常模式 - 透传Jetson指令
*******************************************************************************/
static void Arbiter_ProcessNormalMode(void)
{
	// 检查心跳丢失
	if(arb_state.heartbeat_lost)
	{
		Arbiter_SwitchMode(ARBITER_MODE_DEGRADED);
		return;
	}
	
	// 检查是否有障碍物或前向传感器无效
	if(arb_state.obstacle_detected || !(arb_state.obstacle_valid_mask & 0x01))
	{
		Arbiter_SwitchMode(ARBITER_MODE_SPEED_LIMIT);
		return;
	}
	
	// 正常模式：透传 Jetson 指令（v=0 即停车）
	arb_state.output.v = Arbiter_ClampSpeed(arb_state.jetson_cmd.v);
	arb_state.output.omega = arb_state.jetson_cmd.omega;
	arb_state.output.steering = arb_state.jetson_cmd.steer;
	arb_state.output.mode = ARBITER_MODE_NORMAL;
	arb_state.output.emergency = 0;
	arb_last_policy_rule = 0;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ProcessSpeedLimitMode
* 功能描述		   : 状态2：限速/避障模式 - 按距离比例限速
*******************************************************************************/
static void Arbiter_ProcessSpeedLimitMode(void)
{
	arb_state.output.emergency = 0;

	// 检查心跳丢失
	if(arb_state.heartbeat_lost)
	{
		Arbiter_SwitchMode(ARBITER_MODE_DEGRADED);
		return;
	}

	/* 前向有效且无真实障碍时才能回到 NORMAL */
	if((arb_state.obstacle_valid_mask & 0x01) && !arb_state.obstacle_detected)
	{
		Arbiter_SwitchMode(ARBITER_MODE_NORMAL);
		return;
	}

	/* Jetson v=0：无条件停车，不跑本地策略 */
	if(Arbiter_IsStopIntent())
	{
		Arbiter_ApplyStopOutput();
		arb_state.output.mode = ARBITER_MODE_SPEED_LIMIT;
		return;
	}

	/* 以 Jetson 意图为主：先尝试指令方向+按距离限速，受阻再绕行 */
	Arbiter_ProcessJetsonIntentPolicy();
	arb_state.output.v = Arbiter_ClampSpeed(arb_state.output.v);
	arb_state.output.mode = ARBITER_MODE_SPEED_LIMIT;
	if(arb_state.output.emergency)
	{
		Arbiter_SwitchMode(ARBITER_MODE_EMERGENCY);
	}
}

/*******************************************************************************
* 函 数 名         : Arbiter_ProcessDegradedMode
* 功能描述		   : 状态3：降级模式 - 反应式避障（if-else规则）
*******************************************************************************/
static void Arbiter_ProcessDegradedMode(void)
{
	u32 elapsed;

	/* 心跳恢复：以 last_heartbeat_tick 为准，不依赖可能卡住的 heartbeat_lost 标志 */
	if(arb_state.jetson_cmd.valid)
	{
		elapsed = OSTime - arb_state.last_heartbeat_tick;
		if(elapsed < ARB_MS_TO_TICKS(ARBITER_HEARTBEAT_TIMEOUT_MS))
		{
			Arbiter_SwitchMode(ARBITER_MODE_RECOVERING);
			return;
		}
	}

	/* Jetson v=0 或未收到有效帧：停车，不 autonomous 避障 */
	if(Arbiter_IsStopIntent() || !arb_state.jetson_cmd.valid)
	{
		Arbiter_ApplyStopOutput();
		arb_state.output.mode = ARBITER_MODE_DEGRADED;
		return;
	}

	Arbiter_ProcessJetsonIntentPolicy();
	arb_state.output.v = Arbiter_ClampSpeed(arb_state.output.v);
	arb_state.output.mode = ARBITER_MODE_DEGRADED;
	if(arb_state.output.emergency)
	{
		Arbiter_SwitchMode(ARBITER_MODE_EMERGENCY);
	}
}

/*******************************************************************************
* 函 数 名         : Arbiter_ProcessEmergencyMode
* 功能描述		   : 状态4：紧急停车模式 - 强制速度0
*******************************************************************************/
static void Arbiter_ProcessEmergencyMode(void)
{
	// 强制停车
	arb_state.output.v = 0;
	arb_state.output.omega = 0;
	arb_state.output.steering = 0;
	arb_state.output.mode = ARBITER_MODE_EMERGENCY;
	arb_state.output.emergency = 1;

	/* 非四向全危险：退出 EMERGENCY，改由 DEGRADED 策略选方向 */
	if(!Arbiter_AllDirectionsDanger())
	{
		if(arb_state.heartbeat_lost)
			Arbiter_SwitchMode(ARBITER_MODE_DEGRADED);
		else
			Arbiter_SwitchMode(ARBITER_MODE_RECOVERING);
	}
}

/*******************************************************************************
* 函 数 名         : Arbiter_ProcessRecoveringMode
* 功能描述		   : 恢复中状态 - 等待稳定后切回正常模式
*******************************************************************************/
static void Arbiter_ProcessRecoveringMode(void)
{
	// 四方向全危险，重新进入紧急模式
	if(Arbiter_AllDirectionsDanger())
	{
		Arbiter_SwitchMode(ARBITER_MODE_EMERGENCY);
		return;
	}
	
	// 检查心跳是否再次丢失
	if(arb_state.heartbeat_lost)
	{
		Arbiter_SwitchMode(ARBITER_MODE_DEGRADED);
		return;
	}
	
	// 检查是否稳定足够时间
	if((OSTime - arb_state.recover_start_tick) >= ARB_MS_TO_TICKS(ARBITER_RECOVER_STABLE_MS))
	{
		// 稳定完成，切回正常模式
		Arbiter_SwitchMode(ARBITER_MODE_NORMAL);
		return;
	}
	
	// 恢复期间保持停车
	arb_state.output.v = 0;
	arb_state.output.omega = 0;
	arb_state.output.steering = 0;
	arb_state.output.mode = ARBITER_MODE_RECOVERING;
	arb_state.output.emergency = 0;
}

/*******************************************************************************
* 函 数 名         : Arbiter_Process
* 功能描述		   : 执行仲裁逻辑（主循环中周期调用）
*******************************************************************************/
void Arbiter_Process(void)
{
	// 检查心跳
	Arbiter_CheckHeartbeat();
	
	// 根据当前模式执行相应逻辑
	switch(arb_state.current_mode)
	{
		case ARBITER_MODE_NORMAL:
			Arbiter_ProcessNormalMode();
			break;
			
		case ARBITER_MODE_SPEED_LIMIT:
			Arbiter_ProcessSpeedLimitMode();
			break;
			
		case ARBITER_MODE_DEGRADED:
			Arbiter_ProcessDegradedMode();
			break;
			
		case ARBITER_MODE_EMERGENCY:
			Arbiter_ProcessEmergencyMode();
			break;
			
		case ARBITER_MODE_RECOVERING:
			Arbiter_ProcessRecoveringMode();
			break;
			
		default:
			Arbiter_SwitchMode(ARBITER_MODE_NORMAL);
			break;
	}
}

/*******************************************************************************
* 函 数 名         : Arbiter_GetOutput
* 功能描述		   : 获取仲裁后输出
*******************************************************************************/
ArbiterOutput_t* Arbiter_GetOutput(void)
{
	return &arb_state.output;
}

/*******************************************************************************
* 函 数 名         : Arbiter_GetMode
* 功能描述		   : 获取当前模式
*******************************************************************************/
ArbiterMode_t Arbiter_GetMode(void)
{
	return arb_state.current_mode;
}

/*******************************************************************************
* 函 数 名         : Arbiter_SendToSTM32A
* 功能描述		   : 发送仲裁后指令到STM32 A（通过CAN）
* 帧格式           : 符合RANGER MINI 3.0 CAN协议（ID:0x111）
*                   byte[0-1]: 线速度 (signed int16, 单位mm/s, MOTOROLA格式)
*                   byte[2-3]: 自旋速度 (signed int16, 单位0.001rad/s, MOTOROLA格式)
*                   byte[4-5]: 保留
*                   byte[6-7]: 转角 (signed int16, 单位0.001rad, MOTOROLA格式)
*******************************************************************************/
void Arbiter_SendToSTM32A(void)
{
	static u8 logged_remote_block = 0;
	static u8 logged_switch_block = 0;
	u8 txbuf[8] = {0};
	s16 speed_mm_s;
	s16 spin_speed;  // 自旋速度，单位0.001rad/s
	s16 angle_scaled; // 转角，单位0.001rad
	s32 spin_speed_i32;
	s32 angle_i32;

	if(arb_state.output.emergency)
	{
		Arbiter_SendZeroMotion111();
		return;
	}

	if(arb_state.sys_status.mode_control == CAN_SYS_MODE_REMOTE)
	{
		if(!logged_remote_block)
		{
			logged_remote_block = 1;
			printf("[Arbiter] Remote mode (0x211), pause 0x111\r\n");
		}
		logged_switch_block = 0;
		Arbiter_SendZeroMotion111();
		return;
	}
	logged_remote_block = 0;

	if(arb_state.motion_mode_fb.switching == CAN_MOTION_SWITCHING)
	{
		if(!logged_switch_block)
		{
			logged_switch_block = 1;
			printf("[Arbiter] Motion model switching (0x291), pause 0x111\r\n");
		}
		Arbiter_SendZeroMotion111();
		return;
	}
	logged_switch_block = 0;

	speed_mm_s = arb_state.output.v;

	/* 阿克曼转角优先；无转角时透传 omega（含 DEGRADED 原地自旋） */
	if(arb_state.output.steering != 0)
	{
		spin_speed = 0;
		angle_i32 = arb_state.output.steering;
		if(speed_mm_s == 0)
			speed_mm_s = ARB_CMD_TURN_MOVE_MM_S;
	}
	else if(arb_state.output.omega != 0)
	{
		spin_speed_i32 = (s32)arb_state.output.omega;
		if(spin_speed_i32 > 32767) spin_speed_i32 = 32767;
		if(spin_speed_i32 < -32768) spin_speed_i32 = -32768;
		spin_speed = (s16)spin_speed_i32;
		angle_i32 = 0;
	}
	else
	{
		spin_speed = 0;
		angle_i32 = 0;
	}
	if(angle_i32 > 32767) angle_i32 = 32767;
	if(angle_i32 < -32768) angle_i32 = -32768;
	angle_scaled = (s16)angle_i32;
	
	// MOTOROLA格式（大端序）：高字节在前
	// byte[0-1]: 线速度
	txbuf[0] = (speed_mm_s >> 8) & 0xFF;
	txbuf[1] = speed_mm_s & 0xFF;
	
	// byte[2-3]: 自旋速度
	txbuf[2] = (spin_speed >> 8) & 0xFF;
	txbuf[3] = spin_speed & 0xFF;
	
	// byte[4-5]: 保留
	txbuf[4] = 0x00;
	txbuf[5] = 0x00;
	
	// byte[6-7]: 转角
	txbuf[6] = (angle_scaled >> 8) & 0xFF;
	txbuf[7] = angle_scaled & 0xFF;
	
	// 发送CAN指令（ID: 0x111）
	CAN1_Send_Msg_WithID(CAN_ARBITER_CMD_ID, txbuf, 8);
	
}

/*******************************************************************************
* 函 数 名         : Arbiter_PrintStatus
* 功能描述		   : 打印调试信息
*******************************************************************************/
void Arbiter_PrintStatus(void)
{
	printf("[Arbiter] Mode=%s | Jetson(v=%d, ω=%d) | Output(v=%d, ω=%d) | "
	       "Dist=%d mm | HB=%s\r\n",
	       ARBITER_MODE_NAMES[arb_state.current_mode],
	       arb_state.jetson_cmd.v,
	       arb_state.jetson_cmd.omega,
	       arb_state.output.v,
	       arb_state.output.omega,
	       arb_state.nearest_dist,
	       arb_state.heartbeat_lost ? "LOST" : "OK");
}

/*******************************************************************************
* 函 数 名         : Arbiter_SetMode
* 功能描述		   : 发送模式设定指令（ID:0x421）
*******************************************************************************/
void Arbiter_SetMode(CanMode_t mode)
{
	u8 txbuf[1];
	txbuf[0] = (u8)mode;
	CAN1_Send_Msg_WithID(CAN_MODE_CMD_ID, txbuf, 1);
	printf("[Arbiter] Set mode: %s\r\n", mode == CAN_MODE_STANDBY ? "STANDBY" : "CAN_CTRL");
}

/*******************************************************************************
* 函 数 名         : Arbiter_SetDriveMode
* 功能描述		   : 发送驱动模式切换指令（ID:0x423）
*******************************************************************************/
void Arbiter_SetDriveMode(CanDriveMode_t mode)
{
	u8 txbuf[1];
	txbuf[0] = (u8)mode;
	CAN1_Send_Msg_WithID(CAN_DRIVE_CMD_ID, txbuf, 1);
	printf("[Arbiter] Set drive mode: %s\r\n", mode == CAN_DRIVE_CURRENT ? "CURRENT" : "VOLTAGE");
}

/*******************************************************************************
* 函 数 名         : Arbiter_ClearError
* 功能描述		   : 发送错误清除指令（ID:0x441）
*******************************************************************************/
void Arbiter_ClearError(CanErrorClear_t error_type)
{
	u8 txbuf[1];
	txbuf[0] = (u8)error_type;
	CAN1_Send_Msg_WithID(CAN_ERROR_CLEAR_ID, txbuf, 1);
	printf("[Arbiter] Clear error: 0x%02X\r\n", error_type);
}

/*******************************************************************************
* 函 数 名         : Arbiter_SetMotionMode
* 功能描述		   : 发送运动模型切换指令（ID:0x141）
*******************************************************************************/
void Arbiter_SetMotionMode(CanMotionMode_t mode)
{
	u8 txbuf[1];
	txbuf[0] = (u8)mode;
	CAN1_Send_Msg_WithID(CAN_MOTION_MODEL_ID, txbuf, 1);
	printf("[Arbiter] Set motion mode: %d\r\n", mode);
}

/*******************************************************************************
* 函 数 名         : Arbiter_SetLight
* 功能描述		   : 发送灯光控制指令（ID:0x121）
*******************************************************************************/
void Arbiter_SetLight(u8 enable, u8 mode)
{
	u8 txbuf[8] = {0};
	txbuf[0] = enable;
	txbuf[1] = mode;
	CAN1_Send_Msg_WithID(CAN_LIGHT_CTRL_ID, txbuf, 8);
	printf("[Arbiter] Set light: enable=%d, mode=%d\r\n", enable, mode);
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseSysStatus
* 功能描述		   : 解析系统状态回馈帧（ID:0x211）
*******************************************************************************/
void Arbiter_ParseSysStatus(u8* data, u8 len)
{
	if(len != 8) return;
	
	arb_state.sys_status.system_status = data[0];
	arb_state.sys_status.mode_control = data[1];
	arb_state.sys_status.battery_voltage = (data[2] << 8) | data[3];
	arb_state.sys_status.fault_info = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
	
	arb_state.feedback_updated = 1;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseMotionFeedback
* 功能描述		   : 解析运动控制回馈帧（ID:0x221）
*******************************************************************************/
void Arbiter_ParseMotionFeedback(u8* data, u8 len)
{
	if(len != 8) return;
	
	arb_state.motion_fb.linear_speed = Chassis_ParseS16BE(data[0], data[1]);
	arb_state.motion_fb.spin_speed = Chassis_ParseS16BE(data[2], data[3]);
	arb_state.motion_fb.steering_angle = Chassis_ParseS16BE(data[6], data[7]);
	
	arb_state.feedback_updated = 1;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseWheelAngle
* 功能描述		   : 解析四轮转角反馈帧（ID:0x271）
*******************************************************************************/
void Arbiter_ParseWheelAngle(u8* data, u8 len)
{
	if(len != 8) return;
	
	arb_state.wheel_angle.steer5 = (data[0] << 8) | data[1];
	arb_state.wheel_angle.steer6 = (data[2] << 8) | data[3];
	arb_state.wheel_angle.steer7 = (data[4] << 8) | data[5];
	arb_state.wheel_angle.steer8 = (data[6] << 8) | data[7];
	
	arb_state.feedback_updated = 1;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseWheelSpeed
* 功能描述		   : 解析四轮转速反馈帧（ID:0x281）
*******************************************************************************/
void Arbiter_ParseWheelSpeed(u8* data, u8 len)
{
	if(len != 8) return;
	
	arb_state.wheel_speed.wheel1 = Chassis_ParseS16BE(data[0], data[1]);
	arb_state.wheel_speed.wheel2 = Chassis_ParseS16BE(data[2], data[3]);
	arb_state.wheel_speed.wheel3 = Chassis_ParseS16BE(data[4], data[5]);
	arb_state.wheel_speed.wheel4 = Chassis_ParseS16BE(data[6], data[7]);
	
	arb_state.feedback_updated = 1;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseMotionModeFeedback
* 功能描述		   : 解析运动模式回馈帧（ID:0x291）
*******************************************************************************/
void Arbiter_ParseMotionModeFeedback(u8* data, u8 len)
{
	if(len != 3) return;
	
	arb_state.motion_mode_fb.motion_mode = data[0];
	arb_state.motion_mode_fb.switching = data[1];
	arb_state.motion_mode_fb.drive_mode = data[2];
	
	arb_state.feedback_updated = 1;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseMotorHighInfo
* 功能描述		   : 解析电机高速信息反馈帧（ID:0x251~0x258）
*******************************************************************************/
void Arbiter_ParseMotorHighInfo(u8 motor_id, u8* data, u8 len)
{
	u8 idx;
	
	if(len != 8 || motor_id < 1 || motor_id > 8) return;
	
	idx = motor_id - 1;
	arb_state.motor_high[idx].speed_rpm = (data[0] << 8) | data[1];
	arb_state.motor_high[idx].current = (data[2] << 8) | data[3];
	arb_state.motor_high[idx].position = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
	
	arb_state.feedback_updated = 1;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseMotorLowInfo
* 功能描述		   : 解析电机低速信息反馈帧（ID:0x261~0x268）
*******************************************************************************/
void Arbiter_ParseMotorLowInfo(u8 motor_id, u8* data, u8 len)
{
	u8 idx;
	
	if(len != 8 || motor_id < 1 || motor_id > 8) return;
	
	idx = motor_id - 1;
	arb_state.motor_low[idx].voltage = (data[0] << 8) | data[1];
	arb_state.motor_low[idx].driver_temp = (data[2] << 8) | data[3];
	arb_state.motor_low[idx].motor_temp = (s8)data[4];
	arb_state.motor_low[idx].status = data[5];
	
	arb_state.feedback_updated = 1;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseRemoteCtrl
* 功能描述		   : 解析遥控器信息反馈帧（ID:0x241）
*******************************************************************************/
void Arbiter_ParseRemoteCtrl(u8* data, u8 len)
{
	if(len != 8) return;
	
	arb_state.remote_ctrl.sw = data[0];
	arb_state.remote_ctrl.right_lr = (s8)data[1];
	arb_state.remote_ctrl.right_ud = (s8)data[2];
	arb_state.remote_ctrl.left_ud = (s8)data[3];
	arb_state.remote_ctrl.left_lr = (s8)data[4];
	arb_state.remote_ctrl.vra = (s8)data[5];
	
	arb_state.feedback_updated = 1;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseOdometerFront
* 功能描述		   : 解析前轮里程反馈帧（ID:0x311）
*******************************************************************************/
void Arbiter_ParseOdometerFront(u8* data, u8 len)
{
	if(len != 8) return;
	
	arb_state.odometer.front_left = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
	arb_state.odometer.front_right = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
	
	arb_state.feedback_updated = 1;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseOdometerRear
* 功能描述		   : 解析后轮里程反馈帧（ID:0x312）
*******************************************************************************/
void Arbiter_ParseOdometerRear(u8* data, u8 len)
{
	if(len != 8) return;
	
	arb_state.odometer.rear_left = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
	arb_state.odometer.rear_right = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
	
	arb_state.feedback_updated = 1;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseBMSData
* 功能描述		   : 解析BMS数据反馈帧（ID:0x361）
*******************************************************************************/
void Arbiter_ParseBMSData(u8* data, u8 len)
{
	if(len != 8) return;
	
	arb_state.bms_data.soc = data[0];
	arb_state.bms_data.soh = data[1];
	arb_state.bms_data.voltage = (data[2] << 8) | data[3];
	arb_state.bms_data.current = (data[4] << 8) | data[5];
	arb_state.bms_data.temperature = (data[6] << 8) | data[7];
	
	arb_state.feedback_updated = 1;
}

/*******************************************************************************
* 函 数 名         : Arbiter_ParseBMSAlarm
* 功能描述		   : 解析BMS告警状态反馈帧（ID:0x362）
*******************************************************************************/
void Arbiter_ParseBMSAlarm(u8* data, u8 len)
{
	if(len != 4) return;
	
	arb_state.bms_alarm.alarm_status1 = data[0];
	arb_state.bms_alarm.alarm_status2 = data[1];
	arb_state.bms_alarm.warning_status1 = data[2];
	arb_state.bms_alarm.warning_status2 = data[3];
	
	arb_state.feedback_updated = 1;
}

/* 运动状态显示阈值（0x221/0x291 反馈，单位见 CAN 协议） */
#define CHASSIS_MOTION_V_DEAD    50    /* 线速度 mm/s，低于此视为静止 */
#define CHASSIS_STEER_STRAIGHT   30    /* 转角 0.001rad，约 1.7°，阿克曼直行 */
#define CHASSIS_SLIDE_SMALL     200    /* 0.2 rad，斜移模式小角 */
#define CHASSIS_SLIDE_CRAB     1200    /* 1.2 rad，斜移横移 */
#define CHASSIS_SPIN_DEAD        50    /* 自旋 0.001rad/s */

static s16 Chassis_MotionAbs(s16 v)
{
	return (v < 0) ? (s16)(-v) : v;
}

static s16 Chassis_ParseS16BE(u8 hi, u8 lo)
{
	return (s16)((u16)((hi << 8) | lo));
}

/* 0x221 阿克曼反馈：负转角=左转，正转角=右转（与 0x111 命令符号相反） */
static u8 Chassis_MotionAckermannFb(s16 v, s16 steer, s16 *out_speed)
{
	s16 av = Chassis_MotionAbs(v);

	if(v > CHASSIS_MOTION_V_DEAD)
	{
		if(steer <= -CHASSIS_STEER_STRAIGHT)
		{
			*out_speed = av;
			return CHASSIS_MOTION_LEFT;
		}
		if(steer >= CHASSIS_STEER_STRAIGHT)
		{
			*out_speed = av;
			return CHASSIS_MOTION_RIGHT;
		}
		*out_speed = v;
		return CHASSIS_MOTION_FORWARD;
	}
	if(v < -CHASSIS_MOTION_V_DEAD)
	{
		if(steer <= -CHASSIS_STEER_STRAIGHT)
		{
			*out_speed = av;
			return CHASSIS_MOTION_LEFT;
		}
		if(steer >= CHASSIS_STEER_STRAIGHT)
		{
			*out_speed = av;
			return CHASSIS_MOTION_RIGHT;
		}
		*out_speed = av;
		return CHASSIS_MOTION_BACKWARD;
	}
	*out_speed = 0;
	return CHASSIS_MOTION_STOP;
}

/* 0x111 命令：正转角=左转，负转角=右转 */
static u8 Chassis_MotionAckermannCmd(s16 v, s16 steer, s16 *out_speed)
{
	s16 av = Chassis_MotionAbs(v);

	if(v > CHASSIS_MOTION_V_DEAD)
	{
		if(steer >= CHASSIS_STEER_STRAIGHT)
		{
			*out_speed = av;
			return CHASSIS_MOTION_LEFT;
		}
		if(steer <= -CHASSIS_STEER_STRAIGHT)
		{
			*out_speed = av;
			return CHASSIS_MOTION_RIGHT;
		}
		*out_speed = v;
		return CHASSIS_MOTION_FORWARD;
	}
	if(v < -CHASSIS_MOTION_V_DEAD)
	{
		if(steer >= CHASSIS_STEER_STRAIGHT)
		{
			*out_speed = av;
			return CHASSIS_MOTION_LEFT;
		}
		if(steer <= -CHASSIS_STEER_STRAIGHT)
		{
			*out_speed = av;
			return CHASSIS_MOTION_RIGHT;
		}
		*out_speed = av;
		return CHASSIS_MOTION_BACKWARD;
	}
	*out_speed = 0;
	return CHASSIS_MOTION_STOP;
}

/* 斜移模式：小角≈直走，大角斜移，≥1.2rad 横移（正=左，负=右） */
static u8 Chassis_MotionSlide(s16 v, s16 steer, s16 *out_speed)
{
	s16 av = Chassis_MotionAbs(v);
	s16 ast = Chassis_MotionAbs(steer);

	if(v > CHASSIS_MOTION_V_DEAD)
	{
		*out_speed = av;
		if(ast <= CHASSIS_SLIDE_SMALL)
			return CHASSIS_MOTION_FORWARD;
		if(steer >= CHASSIS_SLIDE_CRAB)
			return CHASSIS_MOTION_LEFT;
		if(steer <= -CHASSIS_SLIDE_CRAB)
			return CHASSIS_MOTION_RIGHT;
		if(steer > CHASSIS_SLIDE_SMALL)
			return CHASSIS_MOTION_LEFT;
		return CHASSIS_MOTION_RIGHT;
	}
	if(v < -CHASSIS_MOTION_V_DEAD)
	{
		*out_speed = av;
		if(ast <= CHASSIS_SLIDE_SMALL)
			return CHASSIS_MOTION_BACKWARD;
		if(steer >= CHASSIS_SLIDE_CRAB)
			return CHASSIS_MOTION_LEFT;
		if(steer <= -CHASSIS_SLIDE_CRAB)
			return CHASSIS_MOTION_RIGHT;
		if(steer > CHASSIS_SLIDE_SMALL)
			return CHASSIS_MOTION_LEFT;
		return CHASSIS_MOTION_RIGHT;
	}
	*out_speed = 0;
	return CHASSIS_MOTION_STOP;
}

/*******************************************************************************
* 函 数 名         : Arbiter_GetMotionInfo
* 功能描述		   : 根据底盘 CAN 反馈判断运动方向与速度（供 LCD/串口）
* 说    明         : 主判据 0x291 运动模式 + 0x221 线速度/自旋/转角；
*                   不再用 0x281 轮速 side_diff（避免死轮/打滑误判）
*******************************************************************************/
void Arbiter_GetMotionInfo(u8 *dir, s16 *speed)
{
	s16 v, spin, steer;
	s16 av, aspin;
	u8 mode;
	u8 motion_dir = CHASSIS_MOTION_STOP;
	s16 motion_speed = 0;

	v = arb_state.motion_fb.linear_speed;
	spin = arb_state.motion_fb.spin_speed;
	steer = arb_state.motion_fb.steering_angle;
	mode = arb_state.motion_mode_fb.motion_mode;
	av = Chassis_MotionAbs(v);
	aspin = Chassis_MotionAbs(spin);

	/* 1. 驻车（优先于静止） */
	if(mode == CAN_MOTION_PARK)
	{
		motion_dir = CHASSIS_MOTION_STOP;
		motion_speed = 0;
		goto motion_done;
	}

	/* 2. 仲裁层紧急停车 */
	if(arb_state.output.emergency)
	{
		motion_dir = CHASSIS_MOTION_STOP;
		motion_speed = 0;
		goto motion_done;
	}

	/* 3. 整车静止 */
	if(av < CHASSIS_MOTION_V_DEAD && aspin < CHASSIS_SPIN_DEAD)
	{
		motion_dir = CHASSIS_MOTION_STOP;
		motion_speed = 0;
		goto motion_done;
	}

	switch(mode)
	{
		case CAN_MOTION_SPIN:
			motion_dir = CHASSIS_MOTION_SPIN;
			if(aspin >= CHASSIS_SPIN_DEAD)
				motion_speed = aspin;
			else if(av >= CHASSIS_MOTION_V_DEAD)
				motion_speed = av;   /* 自旋模式下线速度非零 */
			else
				motion_speed = aspin;
			break;

		case CAN_MOTION_SLIDE:
			motion_dir = Chassis_MotionSlide(v, steer, &motion_speed);
			break;

		case CAN_MOTION_ACKERMANN:
		default:
			motion_dir = Chassis_MotionAckermannFb(v, steer, &motion_speed);
			/* 阿克曼模式下原地自旋（线速度近 0 但 spin 有效） */
			if(motion_dir == CHASSIS_MOTION_STOP && aspin >= CHASSIS_SPIN_DEAD)
			{
				motion_dir = CHASSIS_MOTION_SPIN;
				motion_speed = aspin;
			}
			break;
	}

motion_done:
	if(dir)
		*dir = motion_dir;
	if(speed)
		*speed = motion_speed;
}

/*******************************************************************************
* 函 数 名         : Arbiter_GetCmdMotionInfo
* 功能描述		   : 根据仲裁输出 output（即将/已下发 0x111）推断运动方向与速度
* 说    明         : 与 SendToSTM32A 一致：有 steer 时 v=0 会补 ARB_CMD_TURN_MOVE_MM_S
*******************************************************************************/
void Arbiter_GetCmdMotionInfo(u8 *dir, s16 *speed)
{
	s16 v, steer, omega, eff_v;
	u8 motion_dir = CHASSIS_MOTION_STOP;
	s16 motion_speed = 0;

	v = arb_state.output.v;
	steer = arb_state.output.steering;
	omega = arb_state.output.omega;

	if(arb_state.output.emergency)
	{
		motion_dir = CHASSIS_MOTION_STOP;
		motion_speed = 0;
		goto cmd_done;
	}

	if(steer != 0)
	{
		eff_v = v;
		if(eff_v == 0)
			eff_v = ARB_CMD_TURN_MOVE_MM_S;
		motion_dir = Chassis_MotionAckermannCmd(eff_v, steer, &motion_speed);
	}
	else if(arb_state.output.omega != 0)
	{
		motion_dir = CHASSIS_MOTION_SPIN;
		motion_speed = Chassis_MotionAbs(omega);
	}
	else
	{
		motion_dir = Chassis_MotionAckermannCmd(v, 0, &motion_speed);
	}

cmd_done:
	if(dir)
		*dir = motion_dir;
	if(speed)
		*speed = motion_speed;
}

const char* Arbiter_MotionDirNameAscii(u8 dir)
{
	switch(dir)
	{
		case CHASSIS_MOTION_LEFT:     return "LEFT";
		case CHASSIS_MOTION_RIGHT:    return "RIGHT";
		case CHASSIS_MOTION_SPIN:     return "SPIN";
		case CHASSIS_MOTION_FORWARD:  return "FWD";
		case CHASSIS_MOTION_BACKWARD: return "BACK";
		default:                      return "STOP";
	}
}

/*******************************************************************************
* 函 数 名         : Arbiter_GetWheelSpeedPhysical
* 功能描述		   : CAN轮编号 -> 车体物理位置
* 映    射         : CAN W1=左后  W2=左前  W3=右前  W4=右后
*******************************************************************************/
void Arbiter_GetWheelSpeedPhysical(ChassisWheelSpeed_t *w)
{
	if(!w) return;

	w->lr = arb_state.wheel_speed.wheel1;
	w->lf = arb_state.wheel_speed.wheel2;
	w->rf = arb_state.wheel_speed.wheel3;
	w->rr = arb_state.wheel_speed.wheel4;
}

static void Arbiter_PrintPowerDriverStatus(void)
{
	u8 i;
	u8 vlow = 0;
	u16 sys_v;
	char ts[20];

	sys_v = arb_state.sys_status.battery_voltage;
	for(i = 0; i < 8; i++)
	{
		if(arb_state.motor_low[i].status & CAN_DRV_ST_VLOW)
			vlow = 1;
	}

	Log_TsPrefix(ts, sizeof(ts));
	printf("%s[PWR] SysBatt=%u.%uV Vlow=%s\r\n",
		ts,
		(unsigned int)(sys_v / 10),
		(unsigned int)(sys_v % 10),
		vlow ? "Low" : "Normal");
}

/*******************************************************************************
* 函 数 名         : Arbiter_PrintChassisFeedback
* 功能描述		   : 打印底盘 CAN 反馈到串口
*******************************************************************************/
void Arbiter_PrintChassisFeedback(void)
{
	u8 fb_dir, cmd_dir;
	s16 fb_speed, cmd_speed;
	ChassisWheelSpeed_t ws;
	s16 left_avg, right_avg, side_diff, signed_avg, avg_ws;
	char ts[20];
	char sf[8], sb[8], sl[8], sr[8];

	Arbiter_GetWheelSpeedPhysical(&ws);
	left_avg = (ws.lf + ws.lr) / 2;
	right_avg = (ws.rf + ws.rr) / 2;
	side_diff = left_avg - right_avg;
	signed_avg = (s16)((ws.lf + ws.rf + ws.lr + ws.rr) / 4);
	avg_ws = (Chassis_MotionAbs(ws.lf) + Chassis_MotionAbs(ws.rf) +
	          Chassis_MotionAbs(ws.lr) + Chassis_MotionAbs(ws.rr)) / 4;

	Log_TsPrefix(ts, sizeof(ts));
	Arbiter_FormatDistMm(arb_state.obstacle_dist.front, sf, sizeof(sf));
	Arbiter_FormatDistMm(arb_state.obstacle_dist.back, sb, sizeof(sb));
	Arbiter_FormatDistMm(arb_state.obstacle_dist.left, sl, sizeof(sl));
	Arbiter_FormatDistMm(arb_state.obstacle_dist.right, sr, sizeof(sr));
	printf("%s[WHEEL] LF=%d RF=%d LR=%d RR=%d | RAW w1=%d w2=%d w3=%d w4=%d | v221=%d steer=%d mm=%u sd=%d sa=%d aw=%d\r\n",
		ts,
		(int)ws.lf, (int)ws.rf, (int)ws.lr, (int)ws.rr,
		(int)arb_state.wheel_speed.wheel1,
		(int)arb_state.wheel_speed.wheel2,
		(int)arb_state.wheel_speed.wheel3,
		(int)arb_state.wheel_speed.wheel4,
		(int)arb_state.motion_fb.linear_speed,
		(int)arb_state.motion_fb.steering_angle,
		(unsigned int)arb_state.motion_mode_fb.motion_mode,
		(int)side_diff, (int)signed_avg, (int)avg_ws);

	printf("%s[CMDOUT] v=%d omega=%d steer=%d mode=%s | F=%s B=%s L=%s R=%s pol=R%u\r\n",
		ts,
		(int)arb_state.output.v,
		(int)arb_state.output.omega,
		(int)arb_state.output.steering,
		ARBITER_MODE_NAMES[arb_state.current_mode],
		sf, sb, sl, sr,
		(unsigned int)arb_last_policy_rule);

	Arbiter_GetMotionInfo(&fb_dir, &fb_speed);
	Arbiter_GetCmdMotionInfo(&cmd_dir, &cmd_speed);
	printf("%s[MOTION] FB=%s %d | CMD=%s %d | ARB=%s\r\n",
		ts,
		Arbiter_MotionDirNameAscii(fb_dir),
		(int)fb_speed,
		Arbiter_MotionDirNameAscii(cmd_dir),
		(int)cmd_speed,
		ARBITER_MODE_NAMES[arb_state.current_mode]);

	Arbiter_PrintPowerDriverStatus();
}

/*******************************************************************************
* 函 数 名         : Arbiter_ProcessCANFeedback
* 功能描述		   : 处理 CAN 反馈数据（在主循环中调用）
* 说    明         : 接收 STM32 A 的反馈帧并分发到对应解析函数
*******************************************************************************/
void Arbiter_ProcessCANFeedback(void)
{
	u32 can_id;
	u8 buf[8];
	u8 len;
	
	// 尝试接收 CAN 消息
	len = CAN1_Receive_Msg_WithID(&can_id, buf);
	if(len == 0) return;
	
	// 根据 CAN ID 分发到对应解析函数
	switch(can_id)
	{
		case CAN_SYS_STATUS_ID:
			Arbiter_ParseSysStatus(buf, len);
			break;
			
		case CAN_MOTION_FB_ID:
			Arbiter_ParseMotionFeedback(buf, len);
			break;
			
		case CAN_LIGHT_FB_ID:
			// 灯光控制反馈帧（可根据需要扩展）
			break;
			
		case CAN_REMOTE_CTRL_ID:
			Arbiter_ParseRemoteCtrl(buf, len);
			break;
			
		case CAN_MOTOR_HIGH_ID:
		case CAN_MOTOR_HIGH_ID + 1:
		case CAN_MOTOR_HIGH_ID + 2:
		case CAN_MOTOR_HIGH_ID + 3:
		case CAN_MOTOR_HIGH_ID + 4:
		case CAN_MOTOR_HIGH_ID + 5:
		case CAN_MOTOR_HIGH_ID + 6:
		case CAN_MOTOR_HIGH_ID + 7:
			Arbiter_ParseMotorHighInfo(can_id - CAN_MOTOR_HIGH_ID + 1, buf, len);
			break;
			
		case CAN_MOTOR_LOW_ID:
		case CAN_MOTOR_LOW_ID + 1:
		case CAN_MOTOR_LOW_ID + 2:
		case CAN_MOTOR_LOW_ID + 3:
		case CAN_MOTOR_LOW_ID + 4:
		case CAN_MOTOR_LOW_ID + 5:
		case CAN_MOTOR_LOW_ID + 6:
		case CAN_MOTOR_LOW_ID + 7:
			Arbiter_ParseMotorLowInfo(can_id - CAN_MOTOR_LOW_ID + 1, buf, len);
			break;
			
		case CAN_WHEEL_ANGLE_ID:
			Arbiter_ParseWheelAngle(buf, len);
			break;
			
		case CAN_WHEEL_SPEED_ID:
			Arbiter_ParseWheelSpeed(buf, len);
			break;
			
		case CAN_MOTION_MODE_FB_ID:
			Arbiter_ParseMotionModeFeedback(buf, len);
			break;
			
		case CAN_ODOM_FRONT_ID:
			Arbiter_ParseOdometerFront(buf, len);
			break;
			
		case CAN_ODOM_REAR_ID:
			Arbiter_ParseOdometerRear(buf, len);
			break;
			
		case CAN_BMS_DATA_ID:
			Arbiter_ParseBMSData(buf, len);
			break;
			
		case CAN_BMS_ALARM_ID:
			Arbiter_ParseBMSAlarm(buf, len);
			break;
			
		default:
			// 未知 CAN ID，忽略
			break;
	}
}

/*******************************************************************************
* 函 数 名         : Arbiter_EnableCANMode
* 功能描述		   : 上电初始化时切换到 CAN 指令模式
*******************************************************************************/
void Arbiter_EnableCANMode(void)
{
	// 发送模式设定指令，切换到 CAN 指令模式
	Arbiter_SetMode(CAN_MODE_CAN_CTRL);
	
	// 默认使用电流驱动模式（静音）
	Arbiter_SetDriveMode(CAN_DRIVE_CURRENT);
	
	// 默认使用前后阿克曼模式
	Arbiter_SetMotionMode(CAN_MOTION_ACKERMANN);
	
	printf("[Arbiter] CAN mode enabled, ready to control chassis\r\n");
}

/*******************************************************************************
* 函 数 名         : Arbiter_SetLocalCmd
* 功能描述		   : 设置本地运动指令（main 循环调用，无 Jetson 时使用）
*******************************************************************************/
void Arbiter_SetLocalCmd(s16 v, s16 omega)
{
	arb_state.jetson_cmd.mode_req = 1;
	arb_state.jetson_cmd.v = v;
	arb_state.jetson_cmd.omega = omega;
	arb_state.jetson_cmd.steer = 0;
	arb_state.jetson_cmd.seq++;
	arb_state.jetson_cmd.valid = 1;
	Arbiter_UpdateHeartbeat();
}
