#include "distance_sensor.h"
#include "usart.h"
#include "stdio.h"
#include "time.h"
#include "FreeRTOS.h"
#include "task.h"

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
	u32 stable_stamp_ms;
	u8 stable_valid;
} DsChannelFilter_t;

static DsChannelFilter_t ds_filter[4];
static volatile u8 ds_log_rx_timeout = 0;
static volatile u32 s_ds_rx_bytes = 0;
static volatile u32 s_ds_frame_ok = 0;
static volatile u32 s_ds_trig_count = 0;
static volatile u32 s_ds_poll_rx = 0;
static volatile u32 s_ds_uart_err = 0;
static volatile u32 s_ds_sync_skip = 0;
static volatile u32 s_ds_hdr_ok = 0;
static volatile u32 s_ds_chk_fail = 0;
static u8 s_last_rx[12];
static u8 s_last_rx_n = 0;
static u8 ds_boot_fast_remain = DS_BOOT_FAST_TRIG_COUNT;
static u8 ds_first_trig_done = 0;
static u8 ds_timeout_streak[4];

static void DistanceSensor_ClassifyRaw(u16 raw, u8 *err, u16 *store_mm);

static void DistanceSensor_RawPush(u8 byte)
{
	if(s_last_rx_n < sizeof(s_last_rx))
		s_last_rx[s_last_rx_n++] = byte;
	else
	{
		u8 i;
		for(i = 0; i < sizeof(s_last_rx) - 1u; i++)
			s_last_rx[i] = s_last_rx[i + 1u];
		s_last_rx[sizeof(s_last_rx) - 1u] = byte;
	}
}

static void DistanceSensor_OnRxByte(u8 byte)
{
	u16 sum;
	u8 i;

	s_ds_rx_bytes++;
	ds_rx_idle_cnt = 0;
	DistanceSensor_RawPush(byte);

	if(ds_rx_idx == 0)
	{
		if(byte == DS_HEADER)
		{
			s_ds_hdr_ok++;
			ds_rx_buf[0] = byte;
			ds_rx_idx = 1;
		}
		else
		{
			s_ds_sync_skip++;
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
				sum += ds_rx_buf[i];

			if((sum & 0xFF) == ds_rx_buf[DS_FRAME_LEN - 1])
			{
				ds_data.valid = 1;
				s_ds_frame_ok++;
				for(i = 0; i < 4; i++)
				{
					u16 raw = (ds_rx_buf[1 + i * 2] << 8) | ds_rx_buf[2 + i * 2];
					DistanceSensor_ClassifyRaw(raw, &ds_data.error[i], &ds_data.dist[i]);
				}
				ds_frame_ready = 1;
			}
			else
			{
				s_ds_chk_fail++;
			}
			ds_rx_idx = 0;
		}
	}
}

static void DistanceSensor_UartClearErrors(void)
{
	u32 sr;

	sr = USART3->SR;
	if(sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE))
	{
		s_ds_uart_err++;
		(void)USART3->DR;
	}
}

void DistanceSensor_PollRx(void)
{
	u32 primask;

	primask = __get_PRIMASK();
	__disable_irq();
	while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) != RESET)
	{
		s_ds_poll_rx++;
		DistanceSensor_OnRxByte((u8)USART_ReceiveData(USART3));
	}
	DistanceSensor_UartClearErrors();
	if(!primask)
		__enable_irq();
}

void DistanceSensor_PrintHwDiag(void)
{
	u32 moder, afrl;

	moder = GPIOB->MODER;
	afrl = GPIOB->AFR[1]; /* PB8~PB15 */
	printf("[DS] HW USART3 UE=%u RXNE_IE=%u SR=0x%04lX\r\n",
		(unsigned)((USART3->CR1 & USART_CR1_UE) ? 1u : 0u),
		(unsigned)((USART3->CR1 & USART_CR1_RXNEIE) ? 1u : 0u),
		(unsigned long)USART3->SR);
	printf("[DS] HW PB10 MOD=%lu AF10=%lu | PB11 MOD=%lu AF11=%lu\r\n",
		(unsigned long)((moder >> 20) & 3u),
		(unsigned long)((afrl >> 8) & 0xFu),
		(unsigned long)((moder >> 22) & 3u),
		(unsigned long)((afrl >> 12) & 0xFu));
}

/* 原始 16bit 距离 → error 标记 + 归一化存储值 */
static void DistanceSensor_ClassifyRaw(u16 raw, u8 *err, u16 *store_mm)
{
	if(raw == DS_DIST_MOD_ERR_MARKER)
	{
		*err = DS_ERR_CHKFAIL;
		*store_mm = DS_DIST_FAILSAFE_MM;
	}
	else if(raw == 0 || raw == DS_DIST_MOD_TIMEOUT_RAW || raw >= DS_DIST_MOD_TIMEOUT_MIN)
	{
		*err = DS_ERR_TIMEOUT;
		*store_mm = DS_DIST_FAILSAFE_MM;
	}
	else if(raw > DS_DIST_VALID_MAX_MM)
	{
		*err = DS_ERR_TIMEOUT;
		*store_mm = DS_DIST_FAILSAFE_MM;
	}
	else
	{
		*err = DS_ERR_NONE;
		*store_mm = raw;
	}
}

static u16 DistanceSensor_AbsDiffU16(u16 a, u16 b)
{
	return (a >= b) ? (u16)(a - b) : (u16)(b - a);
}

static u32 DistanceSensor_NowMs(void)
{
	return (u32)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void DistanceSensor_UpdateArbEma(u8 ch, u16 stable);

static void DistanceSensor_FilterReset(void)
{
	u8 i;

	for(i = 0; i < 4; i++)
	{
		ds_filter[i].count = 0;
		ds_filter[i].stable_mm = DS_DIST_UNKNOWN;
		ds_filter[i].stable_stamp_ms = 0;
		ds_filter[i].stable_valid = 0;
	}
}

static void DistanceSensor_SetStable(u8 ch, u16 new_stable)
{
	DsChannelFilter_t *f = &ds_filter[ch];

	if(!f->stable_valid || f->stable_mm != new_stable)
		f->stable_stamp_ms = DistanceSensor_NowMs();
	f->stable_mm = new_stable;
	f->stable_valid = 1;
	DistanceSensor_UpdateArbEma(ch, new_stable);
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
	if(mm < DS_DIST_MIN_VALID_MM || mm > DS_DIST_MAX_MM)
		return 0;
	return 1;
}

static u8 DistanceSensor_FilterIsSpikeJump(u16 mm, u16 valid_buf[], u8 n_valid, u16 stable_mm)
{
	u16 med;
	u16 ref;

	if(n_valid < 1)
		return 0;
	med = DistanceSensor_FilterMedian(valid_buf, n_valid);
	ref = med;
	if(stable_mm != DS_DIST_UNKNOWN)
	{
		if(DistanceSensor_AbsDiffU16(mm, stable_mm) < DistanceSensor_AbsDiffU16(mm, med))
			ref = stable_mm;
	}
	if(DistanceSensor_AbsDiffU16(mm, ref) > DS_FILTER_SPIKE_JUMP_MM)
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
		DistanceSensor_SetStable(ch, new_stable);
		return;
	}
	if(n_valid == 1)
	{
		DistanceSensor_SetStable(ch, valid_buf[0]);
		return;
	}
	if(n_fail >= DS_FILTER_MIN_SAMPLES && n_valid == 0)
	{
		DistanceSensor_SetStable(ch, DS_DIST_FAILSAFE_MM);
		return;
	}
	if(f->count == 0)
	{
		f->stable_valid = 0;
		f->stable_mm = DS_DIST_UNKNOWN;
		f->stable_stamp_ms = 0;
	}
}

static void DistanceSensor_FilterOnNewFrame(void)
{
	u8 i;

	for(i = 0; i < 4; i++)
	{
		if(ds_data.error[i] == DS_ERR_NONE)
		{
			ds_timeout_streak[i] = 0;
			DistanceSensor_FilterPushChannel(i, ds_data.dist[i]);
		}
		else
		{
			if(ds_timeout_streak[i] < 255)
				ds_timeout_streak[i]++;
			if(ds_timeout_streak[i] >= DS_FILTER_TIMEOUT_CLEAR_N)
			{
				ds_filter[i].count = 0;
				ds_filter[i].stable_valid = 0;
				ds_filter[i].stable_mm = DS_DIST_UNKNOWN;
				ds_filter[i].stable_stamp_ms = 0;
			}
		}
	}
}

/* 硬件 USART3：PB10 触发 / PB11 RX，9600，E08 四路测距 */
void DistanceSensor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	u8 i;

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
	ds_boot_fast_remain = DS_BOOT_FAST_TRIG_COUNT;
	ds_first_trig_done = 0;
	for(i = 0; i < 4; i++)
		ds_timeout_streak[i] = 0;
	DistanceSensor_FilterReset();

	printf("[DS] Init OK, trig: boot %ux%dms then %ums, hold=%ums, valid 30~%u TO>=%u\r\n",
		(unsigned)DS_BOOT_FAST_TRIG_COUNT,
		(unsigned)(DS_BOOT_FAST_TRIG_TICKS * DS_PROCESS_MS),
		(unsigned)DS_TRIG_INTERVAL_MS,
		(unsigned)(DS_TRIG_HOLD_TICKS * DS_PROCESS_MS),
		(unsigned)DS_DIST_VALID_MAX_MM,
		(unsigned)DS_DIST_MOD_TIMEOUT_MIN);
	DistanceSensor_PrintHwDiag();
}

void DistanceSensor_Process(void)
{
	GPIO_InitTypeDef gpio;
	static u32 tick = 0;
	static u8 trig_pending = 0;
	static u8 trig_armed = 0;
	static u8 trig_low_ticks = 0;

	ds_process_tick++;
	tick++;

	if(!ds_first_trig_done)
	{
		ds_first_trig_done = 1;
		trig_armed = 1;
	}
	else if(ds_boot_fast_remain > 0)
	{
		if(tick >= DS_BOOT_FAST_TRIG_TICKS)
		{
			tick = 0;
			ds_boot_fast_remain--;
			trig_armed = 1;
		}
	}
	else if(tick >= DS_TRIG_EVERY_N_TICKS)
	{
		tick = 0;
		trig_armed = 1;
	}

	if(trig_low_ticks > 0)
	{
		trig_low_ticks--;
		if(trig_low_ticks == 0)
		{
			GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
			gpio.GPIO_Pin = GPIO_Pin_10;
			gpio.GPIO_Mode = GPIO_Mode_AF;
			gpio.GPIO_OType = GPIO_OType_PP;
			gpio.GPIO_Speed = GPIO_Speed_50MHz;
			gpio.GPIO_PuPd = GPIO_PuPd_UP;
			GPIO_Init(GPIOB, &gpio);
			trig_pending = 0;
			DistanceSensor_UartClearErrors();
			USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
		}
	}
	else if(trig_armed && !trig_pending && ds_rx_idx == 0)
	{
		trig_armed = 0;
		ds_rx_idle_cnt = 0;
		s_ds_trig_count++;
		USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
		gpio.GPIO_Pin = GPIO_Pin_10;
		gpio.GPIO_Mode = GPIO_Mode_OUT;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_50MHz;
		gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &gpio);
		GPIO_ResetBits(GPIOB, GPIO_Pin_10);
		trig_pending = 1;
		trig_low_ticks = DS_TRIG_HOLD_TICKS;
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
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		DistanceSensor_OnRxByte((u8)USART_ReceiveData(USART3));
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
		printf("%s[DS] RX timeout, reset (rx_bytes=%lu valid=%u)\r\n",
			ts, (unsigned long)s_ds_rx_bytes, (unsigned)(ds_data.valid ? 1u : 0u));
	}
}

void DistanceSensor_GetDiag(u32 *rx_bytes, u32 *frame_ok)
{
	if(rx_bytes)
		*rx_bytes = s_ds_rx_bytes;
	if(frame_ok)
		*frame_ok = s_ds_frame_ok;
}

u32 DistanceSensor_GetProcessTick(void)
{
	return ds_process_tick;
}

u32 DistanceSensor_GetTrigCount(void)
{
	return s_ds_trig_count;
}

u32 DistanceSensor_GetPollRxBytes(void)
{
	return s_ds_poll_rx;
}

void DistanceSensor_GetRxParseDiag(u32 *sync_skip, u32 *hdr_ok, u32 *chk_fail)
{
	if(sync_skip)
		*sync_skip = s_ds_sync_skip;
	if(hdr_ok)
		*hdr_ok = s_ds_hdr_ok;
	if(chk_fail)
		*chk_fail = s_ds_chk_fail;
}

void DistanceSensor_FormatLastRxHex(char *buf, u16 buf_sz)
{
	u8 i;
	u16 pos = 0;

	if(!buf || buf_sz < 4)
		return;
	for(i = 0; i < s_last_rx_n && pos + 3 < buf_sz; i++)
		pos += (u16)snprintf(buf + pos, buf_sz - pos, "%02X ", (unsigned)s_last_rx[i]);
	if(pos == 0)
		snprintf(buf, buf_sz, "(none)");
}

void DistanceSensor_FormatLane(u8 idx, char *buf, u16 buf_sz)
{
	if(buf == NULL || buf_sz == 0)
		return;
	if(idx >= 4)
	{
		buf[0] = '\0';
		return;
	}
	if(!ds_data.valid)
	{
		snprintf(buf, buf_sz, "---");
		return;
	}
	if(ds_data.error[idx] == DS_ERR_TIMEOUT)
		snprintf(buf, buf_sz, "TO");
	else if(ds_data.error[idx] == DS_ERR_CHKFAIL)
		snprintf(buf, buf_sz, "ERR");
	else if(ds_data.dist[idx] == DS_DIST_FAILSAFE_MM)
		snprintf(buf, buf_sz, "0");
	else
		snprintf(buf, buf_sz, "%u mm", (unsigned int)ds_data.dist[idx]);
}

void DistanceSensor_FormatFilteredLane(u8 idx, char *buf, u16 buf_sz)
{
	u16 d;

	if(buf == NULL || buf_sz == 0)
		return;
	if(idx >= 4)
	{
		buf[0] = '\0';
		return;
	}
	/* 本帧该路已失效：Filt 也不沿用旧值 */
	if(ds_data.valid &&
	   (ds_data.error[idx] == DS_ERR_TIMEOUT || ds_data.error[idx] == DS_ERR_CHKFAIL))
	{
		DistanceSensor_FormatLane(idx, buf, buf_sz);
		return;
	}
	d = DistanceSensor_GetFilteredMm(idx);
	if(d == DS_DIST_UNKNOWN)
		snprintf(buf, buf_sz, "---");
	else if(d == DS_DIST_FAILSAFE_MM)
		snprintf(buf, buf_sz, "0");
	else
		snprintf(buf, buf_sz, "%u mm", (unsigned int)d);
}

void DistanceSensor_Print(void)
{
	u8 i;
	char ts[20];
	char lane[16];
	char ff[16], bb[16], ll[16], rr[16];

	Log_TsPrefix(ts, sizeof(ts));

	if(!ds_data.valid)
	{
		printf("%s[DS] Waiting for data...\r\n", ts);
		return;
	}

	for(i = 0; i < 4; i++)
	{
		DistanceSensor_FormatLane(i, lane, sizeof(lane));
		printf("%s[DS] IF%d: %s\r\n", ts, i + 1, lane);
	}
	DistanceSensor_FormatFilteredLane(0, ff, sizeof(ff));
	DistanceSensor_FormatFilteredLane(1, bb, sizeof(bb));
	DistanceSensor_FormatFilteredLane(2, ll, sizeof(ll));
	DistanceSensor_FormatFilteredLane(3, rr, sizeof(rr));
	printf("%s[DS] Filt F=%s B=%s L=%s R=%s\r\n", ts, ff, bb, ll, rr);
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

u32 DistanceSensor_GetStableStampMs(u8 idx)
{
	if(idx >= 4)
		return 0;
	if(!ds_filter[idx].stable_valid)
		return 0;
	return ds_filter[idx].stable_stamp_ms;
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
