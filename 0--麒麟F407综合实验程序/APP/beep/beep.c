#include "beep.h"

// 蜂鸣器状态
static u8 beep_enabled = 0;
static u8 current_volume = 0;  // 0-100 音量级别
static u16 beep_toggle = 0;

/*******************************************************************************
* 函 数 名         : BEEP_Init
* 功能描述		   : 蜂鸣器初始化（有源蜂鸣器，GPIO控制）
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void BEEP_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(BEEP_PORT_RCC, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = BEEP_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(BEEP_PORT, &GPIO_InitStructure);

	GPIO_ResetBits(BEEP_PORT, BEEP_PIN);  // 初始关闭

	beep_enabled = 1;
	current_volume = 0;
	beep_toggle = 0;
	
	// 蜂鸣器初始化测试 - 响100ms确认硬件正常
	GPIO_SetBits(BEEP_PORT, BEEP_PIN);
	delay_ms(100);
	GPIO_ResetBits(BEEP_PORT, BEEP_PIN);
}

/*******************************************************************************
* 函 数 名         : BEEP_SetVolume
* 功能描述		   : 设置蜂鸣器音量（通过改变鸣叫频率模拟音量变化）
* 输    入         : volume: 音量级别(0-100)
* 输    出         : 无
* 说明             : 有源蜂鸣器无法真正调节音量，通过以下方式模拟：
*                   - 0: 不响
*                   - 1-20: 低频鸣叫（约1Hz）- 很慢的滴答声
*                   - 21-40: 中低频鸣叫（约3Hz）- 较慢的滴答声
*                   - 41-60: 中频鸣叫（约6Hz）- 中等速度滴答声
*                   - 61-80: 中高频鸣叫（约10Hz）- 快速滴答声
*                   - 81-100: 高频鸣叫（约20Hz）- 几乎连续响
*******************************************************************************/
void BEEP_SetVolume(u8 volume)
{
	if(!beep_enabled) return;

	if(volume > 100) volume = 100;

	current_volume = volume;

	if(volume == 0)
	{
		GPIO_ResetBits(BEEP_PORT, BEEP_PIN);
	}
}

/*******************************************************************************
* 函 数 名         : BEEP_SetDuty
* 功能描述		   : 设置蜂鸣器占空比（兼容之前的接口）
* 输    入         : duty: 占空比(0-100)
* 输    出         : 无
*******************************************************************************/
void BEEP_SetDuty(u8 duty)
{
	BEEP_SetVolume(duty);
}

/*******************************************************************************
* 函 数 名         : BEEP_GetDuty
* 功能描述		   : 获取当前蜂鸣器占空比
* 输    入         : 无
* 输    出         : 当前占空比(0-100)
*******************************************************************************/
u8 BEEP_GetDuty(void)
{
	return current_volume;
}

/*******************************************************************************
* 函 数 名         : BEEP_Process
* 功能描述		   : 蜂鸣器处理函数（由定时器中断调用）
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void BEEP_Process(void)
{
	if(!beep_enabled || current_volume == 0)
	{
		GPIO_ResetBits(BEEP_PORT, BEEP_PIN);
		return;
	}

	beep_toggle++;

	if(current_volume <= 20)
	{
		// 低频鸣叫 - 约1Hz（每20个周期响1个周期）
		if(beep_toggle % 20 < 1)
		{
			GPIO_SetBits(BEEP_PORT, BEEP_PIN);
		}
		else
		{
			GPIO_ResetBits(BEEP_PORT, BEEP_PIN);
		}
	}
	else if(current_volume <= 40)
	{
		// 中低频鸣叫 - 约3Hz（每10个周期响3个周期）
		if(beep_toggle % 10 < 3)
		{
			GPIO_SetBits(BEEP_PORT, BEEP_PIN);
		}
		else
		{
			GPIO_ResetBits(BEEP_PORT, BEEP_PIN);
		}
	}
	else if(current_volume <= 60)
	{
		// 中频鸣叫 - 约6Hz（每5个周期响3个周期）
		if(beep_toggle % 5 < 3)
		{
			GPIO_SetBits(BEEP_PORT, BEEP_PIN);
		}
		else
		{
			GPIO_ResetBits(BEEP_PORT, BEEP_PIN);
		}
	}
	else if(current_volume <= 80)
	{
		// 中高频鸣叫 - 约10Hz（每5个周期响4个周期）
		if(beep_toggle % 5 < 4)
		{
			GPIO_SetBits(BEEP_PORT, BEEP_PIN);
		}
		else
		{
			GPIO_ResetBits(BEEP_PORT, BEEP_PIN);
		}
	}
	else
	{
		// 高频鸣叫 - 约20Hz（几乎一直响）
		if(beep_toggle % 10 < 9)
		{
			GPIO_SetBits(BEEP_PORT, BEEP_PIN);
		}
		else
		{
			GPIO_ResetBits(BEEP_PORT, BEEP_PIN);
		}
	}
}

/*******************************************************************************
* 函 数 名         : BEEP_Beep
* 功能描述		   : 简单蜂鸣（短响一次）
* 输    入         : duration_ms: 鸣叫时长(ms)
* 输    出         : 无
*******************************************************************************/
void BEEP_Beep(u16 duration_ms)
{
	if(!beep_enabled) return;

	GPIO_SetBits(BEEP_PORT, BEEP_PIN);
	delay_ms(duration_ms);
	GPIO_ResetBits(BEEP_PORT, BEEP_PIN);
}

void BEEP_GPIO_Init(void)
{
	BEEP_Init();
}

void BEEP_PWM_Init(void)
{
	BEEP_Init();
}
