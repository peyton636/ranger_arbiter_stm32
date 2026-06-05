#include "chassis_can_test.h"
#include "arbiter.h"
#include "can.h"
#include "SysTick.h"
#include "stdio.h"

#define CHASSIS_TEST_SLIDE_V_MM_S     70
#define CHASSIS_TEST_SLIDE_ANGLE_90   1570
#define CHASSIS_TEST_DURATION_MS      2500
#define CHASSIS_TEST_PERIOD_MS        20
#define CHASSIS_TEST_WAIT_CTRL_MS     3000
#define CHASSIS_TEST_WAIT_MODE_MS     2000

static u32 g_can_test_rx_total;
static u32 g_can_test_rx_211;
static u32 g_can_test_rx_291;
static u32 g_can_test_rx_221;

static u16 ChassisCanTest_DrainCan(void)
{
	u16 n = 0;

	while(CAN_MessagePending(CAN1, CAN_FIFO0))
	{
		u32 id;
		u8 buf[8];
		u8 len;

		len = CAN1_Receive_Msg_WithID(&id, buf);
		if(len == 0)
			break;

		g_can_test_rx_total++;
		if(id == CAN_SYS_STATUS_ID)
		{
			g_can_test_rx_211++;
			Arbiter_ParseSysStatus(buf, len);
		}
		else if(id == CAN_MOTION_MODE_FB_ID)
		{
			g_can_test_rx_291++;
			Arbiter_ParseMotionModeFeedback(buf, len);
		}
		else if(id == CAN_MOTION_FB_ID)
		{
			g_can_test_rx_221++;
			Arbiter_ParseMotionFeedback(buf, len);
		}
		n++;
	}
	return n;
}

static void ChassisCanTest_PrintBusDiag(const char *tag)
{
	u32 esr = CAN1->ESR;
	u8 tec = (u8)((esr >> 16) & 0xFF);
	u8 rec = (u8)((esr >> 24) & 0xFF);

	printf("[CAN TEST] %s bus: MSR=0x%08lX ESR=0x%08lX TEC=%u REC=%u rx=%lu (211=%lu 291=%lu 221=%lu)\r\n",
		tag,
		(unsigned long)CAN1->MSR,
		(unsigned long)esr,
		(unsigned int)tec,
		(unsigned int)rec,
		(unsigned long)g_can_test_rx_total,
		(unsigned long)g_can_test_rx_211,
		(unsigned long)g_can_test_rx_291,
		(unsigned long)g_can_test_rx_221);
}

static void ChassisCanTest_PrintStatus(void)
{
	printf("[CAN TEST] status: 0x211 sys=0x%02X ctrl=0x%02X bat=%u fault=0x%08lX\r\n",
		arb_state.sys_status.system_status,
		arb_state.sys_status.mode_control,
		(unsigned int)arb_state.sys_status.battery_voltage,
		(unsigned long)arb_state.sys_status.fault_info);
	printf("[CAN TEST] status: 0x291 mode=%u switch=%u, 0x221 fb_v=%d mm/s\r\n",
		arb_state.motion_mode_fb.motion_mode,
		arb_state.motion_mode_fb.switching,
		(int)arb_state.motion_fb.linear_speed);
}

static u8 ChassisCanTest_TxOk(u8 rc, const char *what)
{
	if(rc != 0)
	{
		printf("[CAN TEST] TX FAIL: %s\r\n", what);
		return 0;
	}
	return 1;
}

static void ChassisCanTest_Send421CanCtrl(void)
{
	u8 txbuf[1];

	txbuf[0] = CAN_MODE_CAN_CTRL;
	if(ChassisCanTest_TxOk(CAN1_Send_Msg_WithID(CAN_MODE_CMD_ID, txbuf, 1), "0x421 CAN_CTRL"))
		printf("[Arbiter] Set mode: CAN_CTRL\r\n");
}

static void ChassisCanTest_Send441Clear(void)
{
	u8 txbuf[1];

	txbuf[0] = CAN_ERR_CLEAR_ALL;
	if(ChassisCanTest_TxOk(CAN1_Send_Msg_WithID(CAN_ERROR_CLEAR_ID, txbuf, 1), "0x441 clear"))
		printf("[Arbiter] Clear error: 0x00\r\n");
}

static void ChassisCanTest_Send141Motion(u8 mode)
{
	u8 txbuf[1];

	txbuf[0] = mode;
	if(ChassisCanTest_TxOk(CAN1_Send_Msg_WithID(CAN_MOTION_MODEL_ID, txbuf, 1), "0x141 motion"))
		printf("[Arbiter] Set motion mode: %u\r\n", (unsigned int)mode);
}

static u8 ChassisCanTest_WaitCtrlMode(u16 timeout_ms)
{
	u16 elapsed = 0;

	while(elapsed < timeout_ms)
	{
		ChassisCanTest_DrainCan();
		if(arb_state.sys_status.mode_control == CAN_SYS_MODE_CAN_CTRL)
			return 1;
		if((elapsed % 500) == 0)
			ChassisCanTest_Send421CanCtrl();
		delay_ms(20);
		elapsed += 20;
	}
	return 0;
}

static u8 ChassisCanTest_WaitMotionMode(u8 target_mode, u16 timeout_ms)
{
	u16 elapsed = 0;

	while(elapsed < timeout_ms)
	{
		ChassisCanTest_DrainCan();
		if(arb_state.motion_mode_fb.motion_mode == target_mode &&
		   arb_state.motion_mode_fb.switching == CAN_MOTION_SWITCH_DONE)
			return 1;
		delay_ms(20);
		elapsed += 20;
	}
	return 0;
}

static void ChassisCanTest_Send111(s16 v_mm_s, s16 omega_mrad_s, s16 steer_mrad)
{
	u8 txbuf[8] = {0};

	txbuf[0] = (u8)((v_mm_s >> 8) & 0xFF);
	txbuf[1] = (u8)(v_mm_s & 0xFF);
	txbuf[2] = (u8)((omega_mrad_s >> 8) & 0xFF);
	txbuf[3] = (u8)(omega_mrad_s & 0xFF);
	txbuf[6] = (u8)((steer_mrad >> 8) & 0xFF);
	txbuf[7] = (u8)(steer_mrad & 0xFF);
	CAN1_Send_Msg_WithID(CAN_ARBITER_CMD_ID, txbuf, 8);
}

void ChassisCanTest_RunOnce(void)
{
	u16 step = 0;
	u16 tick;
	s16 slide_steer;
	u8 link_ok = 0;

#if CHASSIS_TEST_TURN_LEFT
	slide_steer = CHASSIS_TEST_SLIDE_ANGLE_90;
#else
	slide_steer = -CHASSIS_TEST_SLIDE_ANGLE_90;
#endif

	g_can_test_rx_total = 0;
	g_can_test_rx_211 = 0;
	g_can_test_rx_291 = 0;
	g_can_test_rx_221 = 0;

	printf("\r\n[CAN TEST] ===== start (unlock + %s slide turn) =====\r\n",
#if CHASSIS_TEST_TURN_LEFT
		"LEFT"
#else
		"RIGHT"
#endif
	);
	printf("[CAN TEST] check: chassis ON, CAN PA11/PA12, e-stop OFF, remote not PARK\r\n");
	ChassisCanTest_PrintBusDiag("before");

	for(tick = 0; tick < 25; tick++)
	{
		ChassisCanTest_DrainCan();
		delay_ms(20);
	}
	if(g_can_test_rx_total == 0)
	{
		printf("[CAN TEST] ERROR: no CAN RX in 500ms ?? wiring/power/terminator?\r\n");
		printf("[CAN TEST] ctrl=0x00 below may be default, NOT real chassis state\r\n");
	}
	else
	{
		link_ok = 1;
		printf("[CAN TEST] CAN RX OK, got %lu frames\r\n", (unsigned long)g_can_test_rx_total);
	}

	printf("[CAN TEST] step %u: 0x421 CAN_CTRL\r\n", ++step);
	ChassisCanTest_Send421CanCtrl();
	delay_ms(100);
	ChassisCanTest_DrainCan();

	printf("[CAN TEST] step %u: 0x441 clear all errors\r\n", ++step);
	ChassisCanTest_Send441Clear();
	delay_ms(100);
	ChassisCanTest_DrainCan();

	if(ChassisCanTest_WaitCtrlMode(CHASSIS_TEST_WAIT_CTRL_MS))
	{
		printf("[CAN TEST] OK: chassis ctrl=CAN(0x01)\r\n");
		link_ok = 1;
	}
	else
	{
		printf("[CAN TEST] FAIL: ctrl still 0x%02X (need 0x01 CAN_CTRL)\r\n",
			arb_state.sys_status.mode_control);
		if(arb_state.sys_status.mode_control == CAN_SYS_MODE_REMOTE)
			printf("[CAN TEST] chassis in REMOTE(0x03): switch remote SWC or release takeover\r\n");
	}

	printf("[CAN TEST] step %u: 0x141 SLIDE (for 90 turn)\r\n", ++step);
	ChassisCanTest_Send141Motion(CAN_MOTION_SLIDE);

	if(ChassisCanTest_WaitMotionMode(CAN_MOTION_SLIDE, CHASSIS_TEST_WAIT_MODE_MS))
		printf("[CAN TEST] OK: motion mode SLIDE ready\r\n");
	else
		printf("[CAN TEST] WARN: 0x291 mode=%u switch=%u (target SLIDE=1, done=0)\r\n",
			arb_state.motion_mode_fb.motion_mode,
			arb_state.motion_mode_fb.switching);

	ChassisCanTest_PrintStatus();
	ChassisCanTest_PrintBusDiag("pre-motion");

	if(!link_ok && g_can_test_rx_total == 0)
	{
		printf("[CAN TEST] ABORT motion ?? fix CAN link first, then re-test\r\n");
		printf("[CAN TEST] ===== done =====\r\n\r\n");
		return;
	}

	printf("[CAN TEST] step %u: 0x111 slide v=%d steer=%d x %d ms\r\n",
		++step, CHASSIS_TEST_SLIDE_V_MM_S, (int)slide_steer, CHASSIS_TEST_DURATION_MS);

	for(tick = 0; tick < (CHASSIS_TEST_DURATION_MS / CHASSIS_TEST_PERIOD_MS); tick++)
	{
		ChassisCanTest_Send111(CHASSIS_TEST_SLIDE_V_MM_S, 0, slide_steer);
		ChassisCanTest_DrainCan();
		delay_ms(CHASSIS_TEST_PERIOD_MS);
	}

	printf("[CAN TEST] step %u: stop + back to ACKERMANN\r\n", ++step);
	for(tick = 0; tick < 25; tick++)
	{
		ChassisCanTest_Send111(0, 0, 0);
		ChassisCanTest_DrainCan();
		delay_ms(20);
	}
	ChassisCanTest_Send141Motion(CAN_MOTION_ACKERMANN);
	for(tick = 0; tick < 25; tick++)
	{
		ChassisCanTest_DrainCan();
		delay_ms(20);
	}

	ChassisCanTest_PrintStatus();
	ChassisCanTest_PrintBusDiag("after");
	printf("[CAN TEST] ===== done - did chassis turn? tell me =====\r\n\r\n");
}
