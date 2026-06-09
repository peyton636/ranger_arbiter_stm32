#include "distance_sensor.h"
#include "usart.h"
#include "stdio.h"
#include "time.h"

static DistanceSensor_Data ds_data;
static u8 ds_rx_buf[DS_FRAME_LEN];
static u8 ds_rx_idx = 0;
static u8 ds_frame_ready = 0;
static u8 ds_rx_idle_cnt = 0;
static u32 ds_process_tick = 0;

typedef struct
{
	u16 val;
	u32 tick;
} DsFilterSample_t;

typedef struct
{
	DsFilterSample_t buf[DS_FILTER_MAX_SAMPLES];
	u8 count;
	u16 stable_mm;
	u8 stable_valid;
} DsChannelFilter_t;

static DsChannelFilter_t ds_filter[4];
static volatile u8 ds_log_rx_timeout = 0;

static u16 DistanceSensor_AbsDiffU16(u16 a, u16 b)
{
	return (a >= b) ? (u16)(a - b) : (u16)(b - a);
}

static void DistanceSensor_FilterReset(void)
{
	u8 i;

	for(i = 0; i < 4; i++)
	{
		ds_filter[i].count = 0;
		ds_filter[i].stable_mm = DS_DIST_UNKNOWN;
		ds_filter[i].stable_valid = 0;
	}
}

static u8 DistanceSensor_FilterSampleKind(u16 mm)
{
	if(mm == DS_DIST_UNKNOWN)
		return 2;
	if(mm == DS_DIST_FAILSAFE_MM)
		return 1;
	return 0;
}

/* 中位数：比去极值平均更抗单点野值 */
static u16 DistanceSensor_FilterMedian(u16 *arr, u8 n)
{
	u16 tmp[DS_FILTER_MAX_SAMPLES];
	u8 i;
	u8 j;

	if(n == 0)
		return DS_DIST_UNKNOWN;
	for(i = 0; i < n; i++)
		tmp[i] = arr[i];
	for(i = 1; i < n; i++)
	{
		u16 key = tmp[i];
		j = i;
		while(j > 0 && tmp[j - 1] > key)
		{
			tmp[j] = tmp[j - 1];
			j--;
		}
		tmp[j] = key;
	}
	return tmp[n / 2];
}

static u8 DistanceSensor_FilterIsPlausible(u16 mm)
{
	if(DistanceSensor_FilterSampleKind(mm) != 0)
		return 0;
	if(mm < DS_DIST_MIN_VALID_MM || mm > DS_SPIKE_ABS_MAX_MM)
		return 0;
	return 1;
}

static u8 DistanceSensor_FilterIsSpikeJump(u16 mm, u16 valid_buf[], u8 n_valid, u16 stable_mm)
{
	u16 med;

	if(n_valid < 2)
		return 0;
	/* 相对 stable 在远离：合法运动，不做突跳拒收 */
	if(stable_mm != DS_DIST_UNKNOWN && mm > stable_mm)
		return 0;
	med = DistanceSensor_FilterMedian(valid_buf, n_valid);
	if(DistanceSensor_AbsDiffU16(mm, med) > DS_FILTER_SPIKE_JUMP_MM)
		return 1;
	return 0;
}

static void DistanceSensor_UpdateArbEma(u8 ch, u16 stable)
{
	(void)ch;
	(void)stable;
}

static void DistanceSensor_FilterPushChannel(u8 ch, u16 mm)
{
	DsChannelFilter_t *f = &ds_filter[ch];
	u8 i;
	u8 n_valid = 0;
	u8 n_fail = 0;
	u16 valid_buf[DS_FILTER_MAX_SAMPLES];
	u16 new_stable;
	u8 kind;

	if(ch >= 4)
		return;

	kind = DistanceSensor_FilterSampleKind(mm);
	if(kind == 0)
	{
		if(!DistanceSensor_FilterIsPlausible(mm))
			return;

		if(f->stable_valid && f->stable_mm != DS_DIST_UNKNOWN &&
		   mm > f->stable_mm &&
		   (mm - f->stable_mm) > DS_FILTER_MAX_STEP_MM)
			return;

		/* 相对 stable 大幅变化：清掉旧窗口，避免 1m 外读数被旧中位数拖住 */
		if(f->stable_valid && f->stable_mm != DS_DIST_UNKNOWN)
		{
			if(mm > f->stable_mm &&
			   (mm - f->stable_mm) >= DS_FILTER_MOTION_FLUSH_MM)
				f->count = 0;
			else if(mm < f->stable_mm &&
			        (f->stable_mm - mm) >= DS_FILTER_MOTION_FLUSH_MM)
				f->count = 0;
		}

		for(i = 0; i < f->count; i++)
		{
			if(DistanceSensor_FilterIsPlausible(f->buf[i].val))
				valid_buf[n_valid++] = f->buf[i].val;
		}
		if(DistanceSensor_FilterIsSpikeJump(mm, valid_buf, n_valid,
		                                    f->stable_valid ? f->stable_mm : DS_DIST_UNKNOWN))
			return;
	}

	if(f->count < DS_FILTER_MAX_SAMPLES)
		f->count++;
	else
	{
		for(i = 0; i < DS_FILTER_MAX_SAMPLES - 1; i++)
			f->buf[i] = f->buf[i + 1];
	}
	f->buf[f->count - 1].val = mm;
	f->buf[f->count - 1].tick = ds_process_tick;

	while(f->count > 0 &&
	      (ds_process_tick - f->buf[0].tick) > DS_FILTER_WINDOW_TICKS)
	{
		for(i = 0; i < f->count - 1; i++)
			f->buf[i] = f->buf[i + 1];
		f->count--;
	}

	n_valid = 0;
	n_fail = 0;
	for(i = 0; i < f->count; i++)
	{
		if(DistanceSensor_FilterIsPlausible(f->buf[i].val))
			valid_buf[n_valid++] = f->buf[i].val;
		else if(DistanceSensor_FilterSampleKind(f->buf[i].val) == 1)
			n_fail++;
	}

	if(n_valid >= DS_FILTER_MIN_SAMPLES)
	{
		new_stable = DistanceSensor_FilterMedian(valid_buf, n_valid);
		f->stable_mm = new_stable;
		f->stable_valid = 1;
		DistanceSensor_UpdateArbEma(ch, new_stable);
		return;
	}
	if(n_valid == 1)
	{
		f->stable_mm = valid_buf[0];
		f->stable_valid = 1;
		DistanceSensor_UpdateArbEma(ch, valid_buf[0]);
		return;
	}
	if(n_fail >= DS_FILTER_MIN_SAMPLES && n_valid == 0)
	{
		f->stable_mm = DS_DIST_FAILSAFE_MM;
		f->stable_valid = 1;
		DistanceSensor_UpdateArbEma(ch, DS_DIST_FAILSAFE_MM);
		return;
	}
	if(f->count == 0)
	{
		f->stable_valid = 0;
		f->stable_mm = DS_DIST_UNKNOWN;
	}
}

static void DistanceSensor_FilterOnNewFrame(void)
{
	u8 i;

	for(i = 0; i < 4; i++)
		DistanceSensor_FilterPushChannel(i, DistanceSensor_NormalizedMm(i));
}

void DistanceSensor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	// 配置 UART3 中断
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// 启用 UART3 接收中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART3, ENABLE);

	ds_rx_idx = 0;
	ds_frame_ready = 0;
	ds_data.valid = 0;
	ds_process_tick = 0;
	DistanceSensor_FilterReset();
}

void DistanceSensor_Process(void)
{
	GPIO_InitTypeDef gpio;
	static u32 tick = 0;
	static u8 trig_pending = 0;
	static u8 trig_armed = 0;

	ds_process_tick++;
	tick++;

	/* 到周期则武装；若仍在收帧则推迟，避免 ds_rx_idx 被清导致 IF4 TIMEOUT */
	if(tick >= DS_TRIG_EVERY_N_TICKS)
	{
		tick = 0;
		trig_armed = 1;
	}

	if(trig_armed && !trig_pending && ds_rx_idx == 0)
	{
		trig_armed = 0;
		ds_rx_idle_cnt = 0;
		USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
		gpio.GPIO_Pin = GPIO_Pin_10;
		gpio.GPIO_Mode = GPIO_Mode_OUT;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_50MHz;
		gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &gpio);
		GPIO_ResetBits(GPIOB, GPIO_Pin_10);
		trig_pending = 1;
	}
	else if(trig_pending)
	{
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
		gpio.GPIO_Pin = GPIO_Pin_10;
		gpio.GPIO_Mode = GPIO_Mode_AF;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_50MHz;
		gpio.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOB, &gpio);
		trig_pending = 0;
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	}

	// 接收超时（与模组默认 200ms 一致）
	if(ds_rx_idx > 0)
	{
		ds_rx_idle_cnt++;
		if(ds_rx_idle_cnt > DS_RX_TIMEOUT_TICKS)
		{
			ds_log_rx_timeout = 1;
			ds_rx_idx = 0;
		}
	}
}

// UART3 中断服务函数 - 实时接收传感器数据
void USART3_IRQHandler(void)
{
	u8 byte;
	u16 sum;
	u8 i;

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		byte = USART_ReceiveData(USART3);
		ds_rx_idle_cnt = 0;

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
						/* 原始异常 → error + 归一化为 0(FAILSAFE) */
						if(ds_data.dist[i] == 0xEEEE)
						{
							ds_data.error[i] = DS_ERR_CHKFAIL;
							ds_data.dist[i] = DS_DIST_FAILSAFE_MM;
						}
						else if(ds_data.dist[i] == 0xFFFF || ds_data.dist[i] == 0 ||
								ds_data.dist[i] > DS_DIST_MAX_MM)
						{
							ds_data.error[i] = DS_ERR_TIMEOUT;
							ds_data.dist[i] = DS_DIST_FAILSAFE_MM;
						}
						else
						{
							ds_data.error[i] = DS_ERR_NONE;
						}
					}

					ds_frame_ready = 1;
				}
				ds_rx_idx = 0;
			}
		}
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
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
		DistanceSensor_FilterOnNewFrame();
		return 1;
	}
	return 0;
}

void DistanceSensor_DrainLog(void)
{
	if(!ds_log_rx_timeout)
		return;
	ds_log_rx_timeout = 0;
	{
		char ts[20];
		Log_TsPrefix(ts, sizeof(ts));
		printf("%s[DS] RX timeout, reset\r\n", ts);
	}
}

void DistanceSensor_Print(void)
{
	u8 i;
	char ts[20];

	Log_TsPrefix(ts, sizeof(ts));

	if(!ds_data.valid)
	{
		printf("%s[DS] Waiting for data...\r\n", ts);
		return;
	}

	for(i = 0; i < 4; i++)
	{
		if(ds_data.error[i] == DS_ERR_TIMEOUT)
		{
			printf("%s[DS] IF%d: TIMEOUT\r\n", ts, i + 1);
		}
		else if(ds_data.error[i] == DS_ERR_CHKFAIL)
		{
			printf("%s[DS] IF%d: CHK ERR\r\n", ts, i + 1);
		}
		else
		{
			printf("%s[DS] IF%d: %d mm\r\n", ts, i + 1, ds_data.dist[i]);
		}
	}
	{
		char ff[8], bb[8], ll[8], rr[8];
		u16 d;

		d = DistanceSensor_GetFilteredMm(0);
		if(d == DS_DIST_UNKNOWN) sprintf(ff, "---"); else sprintf(ff, "%u", (unsigned int)d);
		d = DistanceSensor_GetFilteredMm(1);
		if(d == DS_DIST_UNKNOWN) sprintf(bb, "---"); else sprintf(bb, "%u", (unsigned int)d);
		d = DistanceSensor_GetFilteredMm(2);
		if(d == DS_DIST_UNKNOWN) sprintf(ll, "---"); else sprintf(ll, "%u", (unsigned int)d);
		d = DistanceSensor_GetFilteredMm(3);
		if(d == DS_DIST_UNKNOWN) sprintf(rr, "---"); else sprintf(rr, "%u", (unsigned int)d);
		printf("%s[DS] Filt F=%s B=%s L=%s R=%s\r\n", ts, ff, bb, ll, rr);
	}
	printf("---\r\n");
}

void DistanceSensor_PrintStatus(void)
{
	printf("[DS] Waiting...\r\n");
}

u16 DistanceSensor_GetNearestDistance(void)
{
	return DistanceSensor_MinDistMm();
}

/* 供 main/仲裁使用：无整帧数据→UNKNOWN；有 error→FAILSAFE(0)；否则 1..MAX */
u16 DistanceSensor_NormalizedMm(u8 idx)
{
	if(idx >= 4)
		return DS_DIST_UNKNOWN;
	if(!ds_data.valid)
		return DS_DIST_UNKNOWN;
	if(ds_data.error[idx] != DS_ERR_NONE)
		return DS_DIST_FAILSAFE_MM;
	if(ds_data.dist[idx] > DS_DIST_MAX_MM)
		return DS_DIST_FAILSAFE_MM;
	return ds_data.dist[idx];
}

u16 DistanceSensor_MinDistMm(void)
{
	u8 i;
	u16 min_dist = DS_DIST_UNKNOWN;

	if(!ds_data.valid)
		return DS_DIST_UNKNOWN;

	for(i = 0; i < 4; i++)
	{
		u16 d = DistanceSensor_NormalizedMm(i);
		if(d != DS_DIST_UNKNOWN && d < min_dist)
			min_dist = d;
	}
	return min_dist;
}

u16 DistanceSensor_GetFilteredMm(u8 idx)
{
	if(idx >= 4)
		return DS_DIST_UNKNOWN;
	if(!ds_filter[idx].stable_valid)
		return DS_DIST_UNKNOWN;
	return ds_filter[idx].stable_mm;
}

u16 DistanceSensor_GetFilteredMinDistMm(void)
{
	u8 i;
	u16 min_dist = DS_DIST_UNKNOWN;

	for(i = 0; i < 4; i++)
	{
		u16 d = DistanceSensor_GetFilteredMm(i);
		if(d != DS_DIST_UNKNOWN && d < min_dist)
			min_dist = d;
	}
	return min_dist;
}

/* 仲裁与 Filt 一致，不再叠 EMA（避免 CMDOUT 与 Filt 长期脱节） */
u16 DistanceSensor_GetArbiterMm(u8 idx)
{
	return DistanceSensor_GetFilteredMm(idx);
}

void DistanceSensor_UpdateBuzzer(void)
{
}
