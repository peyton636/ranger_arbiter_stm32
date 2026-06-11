#include "gps.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"

#define GPS_PRINT_INTERVAL_MS    1000u
#define GPS_CFG_BOOT_DELAY_MS    1000u
#define GPS_CFG_STEP_DELAY_MS    250u
#define GPS_CFG_BAUD_SWITCH_MS   300u
#define GPS_DEFAULT_BOOT_BAUD    9600u
#define GPS_TARGET_BAUD          115200u

typedef struct
{
	u16 len;
	u8 cls;
	u8 id;
	u8 payload[GPS_CSIP_MAX_PAYLOAD];
	u32 checksum;
} GPS_CsipFrame_t;

typedef struct
{
	u8 state;
	u16 len;
	u8 cls;
	u8 id;
	u16 payload_idx;
	u8 overflow;
	u8 payload[GPS_CSIP_MAX_PAYLOAD];
	u8 cksum_bytes[4];
	u8 cksum_idx;
} GPS_CsipRxState_t;

enum
{
	CSIP_WAIT_BA = 0,
	CSIP_WAIT_CE,
	CSIP_WAIT_LEN_L,
	CSIP_WAIT_LEN_H,
	CSIP_WAIT_CLASS,
	CSIP_WAIT_ID,
	CSIP_WAIT_PAYLOAD,
	CSIP_WAIT_CK0,
	CSIP_WAIT_CK1,
	CSIP_WAIT_CK2,
	CSIP_WAIT_CK3
};

static char gps_nmea_line[GPS_NMEA_MAX_LEN];
static u16 gps_nmea_idx = 0;
static volatile u8 gps_nmea_ready = 0;
static char gps_nmea_sentence[GPS_NMEA_MAX_LEN];

static GPS_CsipRxState_t gps_csip_rx;
static volatile u8 gps_csip_ready = 0;
static GPS_CsipFrame_t gps_csip_frame;

static GPS_Data_t g_gps_data;
static u32 g_gps_last_print_ms = 0;
static u32 g_gps_uart_baud = GPS_DEFAULT_BOOT_BAUD;
static u8 g_gps_cfg_done = 0;
static u8 g_gps_cfg_state = 0;
static u32 g_gps_cfg_next_ms = 0;
extern volatile u32 OSTime;

enum
{
	GPS_CFG_WAIT_BOOT = 0,
	GPS_CFG_SEND_PCAS01,
	GPS_CFG_SWITCH_BAUD,
	GPS_CFG_SEND_PCAS04,
	GPS_CFG_SEND_PCAS02,
	GPS_CFG_SEND_PCAS03,
	GPS_CFG_DONE
};

static void GPS_ResetCsipRx(void)
{
	memset(&gps_csip_rx, 0, sizeof(gps_csip_rx));
	gps_csip_rx.state = CSIP_WAIT_BA;
}

static u16 GPS_ReadLeU16(const u8 *p)
{
	return (u16)((u16)p[0] | ((u16)p[1] << 8));
}

static u32 GPS_ReadLeU32(const u8 *p)
{
	return (u32)((u32)p[0] |
				 ((u32)p[1] << 8) |
				 ((u32)p[2] << 16) |
				 ((u32)p[3] << 24));
}

static float GPS_ReadLeR4(const u8 *p)
{
	float v;
	memcpy(&v, p, sizeof(v));
	return v;
}

static double GPS_ReadLeR8(const u8 *p)
{
	double v;
	memcpy(&v, p, sizeof(v));
	return v;
}

static u32 GPS_CalcCsipChecksum(u8 cls, u8 id, u16 len, const u8 *payload)
{
	u32 ck;
	u16 i;

	ck = (((u32)id) << 24) | (((u32)cls) << 16) | (u32)len;
	for(i = 0; i < len; i += 4)
	{
		ck += GPS_ReadLeU32(payload + i);
	}
	return ck;
}

static double GPS_ConvertNmeaToDegrees(const char *data)
{
	double raw;
	s32 degree;
	double minute;

	if((data == 0) || (data[0] == '\0'))
		return 0.0;

	raw = atof(data);
	degree = (s32)(raw / 100.0);
	minute = raw - ((double)degree * 100.0);
	return ((double)degree) + (minute / 60.0);
}

static u8 GPS_NmeaChecksumOk(const char *sentence)
{
	const char *star;
	u8 sum;
	const char *p;
	u8 rx;

	if((sentence == 0) || (sentence[0] != '$'))
		return 0;

	star = strchr(sentence, '*');
	if((star == 0) || ((star - sentence) < 1))
		return 0;
	if((star[1] == '\0') || (star[2] == '\0'))
		return 0;

	sum = 0;
	for(p = sentence + 1; p < star; ++p)
	{
		sum ^= (u8)(*p);
	}

	rx = (u8)strtoul(star + 1, 0, 16);
	return (sum == rx) ? 1 : 0;
}

static void GPS_NmeaTrimTail(char *token)
{
	char *star;
	char *cr;
	char *lf;

	if(token == 0)
		return;

	star = strchr(token, '*');
	if(star != 0)
		*star = '\0';
	cr = strchr(token, '\r');
	if(cr != 0)
		*cr = '\0';
	lf = strchr(token, '\n');
	if(lf != 0)
		*lf = '\0';
}

static void GPS_ParseRmc(char *buf)
{
	char *token;
	u8 field;
	char status;
	char mode;
	float speed_kn;
	double lat;
	double lon;

	field = 0;
	status = 'V';
	mode = 'A';
	speed_kn = 0.0f;
	lat = 0.0;
	lon = 0.0;

	token = strtok(buf, ",");
	while(token != 0)
	{
		GPS_NmeaTrimTail(token);
		switch(field)
		{
			case 1:
				strncpy(g_gps_data.utc_time, token, sizeof(g_gps_data.utc_time) - 1);
				break;
			case 2:
				status = token[0];
				break;
			case 3:
				strncpy(g_gps_data.latitude_raw, token, sizeof(g_gps_data.latitude_raw) - 1);
				break;
			case 4:
				g_gps_data.ns[0] = token[0];
				g_gps_data.ns[1] = '\0';
				break;
			case 5:
				strncpy(g_gps_data.longitude_raw, token, sizeof(g_gps_data.longitude_raw) - 1);
				break;
			case 6:
				g_gps_data.ew[0] = token[0];
				g_gps_data.ew[1] = '\0';
				break;
			case 7:
				speed_kn = (float)atof(token);
				break;
			case 8:
				g_gps_data.heading_deg = (float)atof(token);
				break;
			case 9:
				strncpy(g_gps_data.date, token, sizeof(g_gps_data.date) - 1);
				break;
			case 12:
				mode = token[0];
				break;
			default:
				break;
		}
		field++;
		token = strtok(0, ",");
	}

	gps_nmea_sentence[0] = '\0';
	g_gps_data.has_data = 1;
	g_gps_data.source = 1;
	g_gps_data.parsed = 1;
	g_gps_data.usefull = ((status == 'A') && (mode != 'N')) ? 1 : 0;
	g_gps_data.speed_mps = speed_kn * 0.514444f;

	if(g_gps_data.usefull)
	{
		lat = GPS_ConvertNmeaToDegrees(g_gps_data.latitude_raw);
		lon = GPS_ConvertNmeaToDegrees(g_gps_data.longitude_raw);
		if(g_gps_data.ns[0] == 'S')
			lat = -lat;
		if(g_gps_data.ew[0] == 'W')
			lon = -lon;
		g_gps_data.latitude_deg = lat;
		g_gps_data.longitude_deg = lon;
	}
}

static void GPS_ParseGga(char *buf)
{
	char *token;
	u8 field;
	u8 fix;

	field = 0;
	fix = 0;
	token = strtok(buf, ",");
	while(token != 0)
	{
		GPS_NmeaTrimTail(token);
		switch(field)
		{
			case 6:
				fix = (u8)atoi(token);
				g_gps_data.pos_valid = fix;
				break;
			case 7:
				g_gps_data.num_sv = (u8)atoi(token);
				break;
			case 8:
				g_gps_data.hdop = (float)atof(token);
				break;
			case 9:
				g_gps_data.altitude_m = (float)atof(token);
				break;
			default:
				break;
		}
		field++;
		token = strtok(0, ",");
	}

	if(fix > 0)
	{
		g_gps_data.usefull = 1;
	}
}

static void GPS_ParseVtg(char *buf)
{
	char *token;
	u8 field;
	float kph;

	field = 0;
	kph = 0.0f;
	token = strtok(buf, ",");
	while(token != 0)
	{
		GPS_NmeaTrimTail(token);
		if(field == 7)
		{
			kph = (float)atof(token);
		}
		field++;
		token = strtok(0, ",");
	}
	g_gps_data.speed_mps = kph / 3.6f;
}

static void GPS_ParseZda(char *buf)
{
	char *token;
	u8 field;
	char day[3];
	char mon[3];
	char year[5];

	field = 0;
	memset(day, 0, sizeof(day));
	memset(mon, 0, sizeof(mon));
	memset(year, 0, sizeof(year));

	token = strtok(buf, ",");
	while(token != 0)
	{
		GPS_NmeaTrimTail(token);
		switch(field)
		{
			case 1:
				strncpy(g_gps_data.utc_time, token, sizeof(g_gps_data.utc_time) - 1);
				break;
			case 2:
				strncpy(day, token, sizeof(day) - 1);
				break;
			case 3:
				strncpy(mon, token, sizeof(mon) - 1);
				break;
			case 4:
				strncpy(year, token, sizeof(year) - 1);
				break;
			default:
				break;
		}
		field++;
		token = strtok(0, ",");
	}

	if((day[0] != '\0') && (mon[0] != '\0') && (year[0] != '\0'))
	{
		snprintf(g_gps_data.date, sizeof(g_gps_data.date), "%s%s%s", day, mon, year);
	}
}

static void GPS_ParsePcas60(char *buf)
{
	char *token;
	u8 field;
	u8 time_valid;
	char ddmmyyyy[16];

	field = 0;
	time_valid = 0;
	memset(ddmmyyyy, 0, sizeof(ddmmyyyy));

	token = strtok(buf, ",");
	while(token != 0)
	{
		GPS_NmeaTrimTail(token);
		switch(field)
		{
			case 1:
				strncpy(g_gps_data.utc_time, token, sizeof(g_gps_data.utc_time) - 1);
				break;
			case 2:
				strncpy(ddmmyyyy, token, sizeof(ddmmyyyy) - 1);
				break;
			case 5:
				time_valid = (u8)atoi(token);
				break;
			default:
				break;
		}
		field++;
		token = strtok(0, ",");
	}

	if(ddmmyyyy[0] != '\0')
	{
		strncpy(g_gps_data.date, ddmmyyyy, sizeof(g_gps_data.date) - 1);
	}
	if(time_valid != 0)
	{
		g_gps_data.parsed = 1;
	}
}

static void GPS_ParseNmeaSentence(char *sentence)
{
	char parse_buf[GPS_NMEA_MAX_LEN];
	char type[6];

	if((sentence == 0) || (sentence[0] != '$'))
		return;

	if(!GPS_NmeaChecksumOk(sentence))
	{
		g_gps_data.nmea_crc_err_count++;
		return;
	}

	g_gps_data.nmea_ok_count++;
	memset(parse_buf, 0, sizeof(parse_buf));
	strncpy(parse_buf, sentence, sizeof(parse_buf) - 1);

	if(strncmp(parse_buf, "$PCAS60", 7) == 0)
	{
		memset(g_gps_data.last_nmea_type, 0, sizeof(g_gps_data.last_nmea_type));
		strncpy((char *)g_gps_data.last_nmea_type, "CAS60", sizeof(g_gps_data.last_nmea_type) - 1);
		GPS_ParsePcas60(parse_buf);
		return;
	}

	if(strlen(parse_buf) < 6)
		return;

	memset(type, 0, sizeof(type));
	type[0] = parse_buf[3];
	type[1] = parse_buf[4];
	type[2] = parse_buf[5];
	type[3] = '\0';
	strncpy((char *)g_gps_data.last_nmea_type, type, sizeof(g_gps_data.last_nmea_type) - 1);

	if(strcmp(type, "RMC") == 0)
	{
		GPS_ParseRmc(parse_buf);
	}
	else if(strcmp(type, "GGA") == 0)
	{
		GPS_ParseGga(parse_buf);
	}
	else if(strcmp(type, "VTG") == 0)
	{
		GPS_ParseVtg(parse_buf);
	}
	else if(strcmp(type, "ZDA") == 0)
	{
		GPS_ParseZda(parse_buf);
	}
}

static void GPS_ParseCasicNavPv(const u8 *payload, u16 len)
{
	double lon;
	double lat;
	float height;
	float speed2d;
	float heading;
	u8 pos_valid;
	u8 vel_valid;

	if(len < 80)
		return;

	pos_valid = payload[4];
	vel_valid = payload[5];
	lon = GPS_ReadLeR8(payload + 16);
	lat = GPS_ReadLeR8(payload + 24);
	height = GPS_ReadLeR4(payload + 32);
	speed2d = GPS_ReadLeR4(payload + 64);
	heading = GPS_ReadLeR4(payload + 68);

	g_gps_data.source = 2;
	g_gps_data.has_data = 1;
	g_gps_data.parsed = 1;
	g_gps_data.pos_valid = pos_valid;
	g_gps_data.vel_valid = vel_valid;
	g_gps_data.nav_system = payload[6];
	g_gps_data.num_sv = payload[7];
	g_gps_data.hdop = GPS_ReadLeR4(payload + 12);
	g_gps_data.altitude_m = height;
	g_gps_data.speed_mps = speed2d;
	g_gps_data.heading_deg = heading;
	g_gps_data.latitude_deg = lat;
	g_gps_data.longitude_deg = lon;

	if(lat >= 0.0)
		g_gps_data.ns[0] = 'N';
	else
		g_gps_data.ns[0] = 'S';
	g_gps_data.ns[1] = '\0';

	if(lon >= 0.0)
		g_gps_data.ew[0] = 'E';
	else
		g_gps_data.ew[0] = 'W';
	g_gps_data.ew[1] = '\0';

	g_gps_data.usefull = (pos_valid >= 4) ? 1 : 0;
}

static void GPS_ParseCasicTimeUtc(const u8 *payload, u16 len)
{
	u16 year;
	u16 ms;
	u8 month;
	u8 day;
	u8 hour;
	u8 minute;
	u8 sec;

	if(len < 24)
		return;

	ms = GPS_ReadLeU16(payload + 12);
	year = GPS_ReadLeU16(payload + 14);
	month = payload[16];
	day = payload[17];
	hour = payload[18];
	minute = payload[19];
	sec = payload[20];

	snprintf(g_gps_data.utc_time, sizeof(g_gps_data.utc_time),
			 "%02u%02u%02u.%03u",
			 (unsigned int)hour,
			 (unsigned int)minute,
			 (unsigned int)sec,
			 (unsigned int)ms);

	snprintf(g_gps_data.date, sizeof(g_gps_data.date),
			 "%02u%02u%04u",
			 (unsigned int)day,
			 (unsigned int)month,
			 (unsigned int)year);
}

static void GPS_ParseCsipFrame(const GPS_CsipFrame_t *frm)
{
	if(frm == 0)
		return;

	g_gps_data.last_casic_cls = frm->cls;
	g_gps_data.last_casic_id = frm->id;
	g_gps_data.csip_ok_count++;

	if((frm->cls == 0x01u) && (frm->id == 0x03u))
	{
		GPS_ParseCasicNavPv(frm->payload, frm->len);
	}
	else if((frm->cls == 0x01u) && (frm->id == 0x10u))
	{
		GPS_ParseCasicTimeUtc(frm->payload, frm->len);
	}
}

static void GPS_HandleCsipByte(u8 ch)
{
	u32 ck_calc;
	u32 ck_rx;

	switch(gps_csip_rx.state)
	{
		case CSIP_WAIT_BA:
			if(ch == 0xBAu)
				gps_csip_rx.state = CSIP_WAIT_CE;
			break;

		case CSIP_WAIT_CE:
			if(ch == 0xCEu)
			{
				gps_csip_rx.state = CSIP_WAIT_LEN_L;
			}
			else if(ch != 0xBAu)
			{
				gps_csip_rx.state = CSIP_WAIT_BA;
			}
			break;

		case CSIP_WAIT_LEN_L:
			gps_csip_rx.len = ch;
			gps_csip_rx.state = CSIP_WAIT_LEN_H;
			break;

		case CSIP_WAIT_LEN_H:
			gps_csip_rx.len |= ((u16)ch << 8);
			gps_csip_rx.overflow = ((gps_csip_rx.len > GPS_CSIP_MAX_PAYLOAD) ||
									((gps_csip_rx.len & 0x03u) != 0u)) ? 1u : 0u;
			gps_csip_rx.state = CSIP_WAIT_CLASS;
			break;

		case CSIP_WAIT_CLASS:
			gps_csip_rx.cls = ch;
			gps_csip_rx.state = CSIP_WAIT_ID;
			break;

		case CSIP_WAIT_ID:
			gps_csip_rx.id = ch;
			gps_csip_rx.payload_idx = 0;
			if(gps_csip_rx.len == 0u)
				gps_csip_rx.state = CSIP_WAIT_CK0;
			else
				gps_csip_rx.state = CSIP_WAIT_PAYLOAD;
			break;

		case CSIP_WAIT_PAYLOAD:
			if((gps_csip_rx.overflow == 0u) && (gps_csip_rx.payload_idx < GPS_CSIP_MAX_PAYLOAD))
			{
				gps_csip_rx.payload[gps_csip_rx.payload_idx] = ch;
			}
			gps_csip_rx.payload_idx++;
			if(gps_csip_rx.payload_idx >= gps_csip_rx.len)
				gps_csip_rx.state = CSIP_WAIT_CK0;
			break;

		case CSIP_WAIT_CK0:
			gps_csip_rx.cksum_bytes[0] = ch;
			gps_csip_rx.state = CSIP_WAIT_CK1;
			break;
		case CSIP_WAIT_CK1:
			gps_csip_rx.cksum_bytes[1] = ch;
			gps_csip_rx.state = CSIP_WAIT_CK2;
			break;
		case CSIP_WAIT_CK2:
			gps_csip_rx.cksum_bytes[2] = ch;
			gps_csip_rx.state = CSIP_WAIT_CK3;
			break;
		case CSIP_WAIT_CK3:
			gps_csip_rx.cksum_bytes[3] = ch;

			if(gps_csip_rx.overflow != 0u)
			{
				g_gps_data.csip_drop_count++;
				GPS_ResetCsipRx();
				break;
			}

			ck_calc = GPS_CalcCsipChecksum(gps_csip_rx.cls, gps_csip_rx.id,
										   gps_csip_rx.len, gps_csip_rx.payload);
			ck_rx = GPS_ReadLeU32(gps_csip_rx.cksum_bytes);
			if(ck_calc == ck_rx)
			{
				gps_csip_frame.len = gps_csip_rx.len;
				gps_csip_frame.cls = gps_csip_rx.cls;
				gps_csip_frame.id = gps_csip_rx.id;
				memcpy(gps_csip_frame.payload, gps_csip_rx.payload, gps_csip_rx.len);
				gps_csip_frame.checksum = ck_rx;
				gps_csip_ready = 1;
			}
			else
			{
				g_gps_data.csip_crc_err_count++;
			}
			GPS_ResetCsipRx();
			break;

		default:
			GPS_ResetCsipRx();
			break;
	}
}

static void GPS_HandleNmeaByte(u8 ch)
{
	if(ch == '$')
	{
		gps_nmea_idx = 0;
		memset(gps_nmea_line, 0, sizeof(gps_nmea_line));
	}

	if(gps_nmea_idx < (GPS_NMEA_MAX_LEN - 1))
	{
		gps_nmea_line[gps_nmea_idx++] = (char)ch;
	}

	if(ch == '\n')
	{
		if(gps_nmea_line[0] == '$')
		{
			memcpy(gps_nmea_sentence, gps_nmea_line, gps_nmea_idx);
			gps_nmea_sentence[gps_nmea_idx] = '\0';
			gps_nmea_ready = 1;
		}
		gps_nmea_idx = 0;
		memset(gps_nmea_line, 0, sizeof(gps_nmea_line));
	}
}

static void GPS_Usart6SendBytes(const u8 *buf, u16 len)
{
	u16 i;
	for(i = 0; i < len; i++)
	{
		USART_SendData(USART6, buf[i]);
		while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET)
		{
		}
	}
}

static void GPS_USART6_SetBaud(u32 bound)
{
	USART_InitTypeDef usart_init;

	USART_Cmd(USART6, DISABLE);
	usart_init.USART_BaudRate = bound;
	usart_init.USART_WordLength = USART_WordLength_8b;
	usart_init.USART_StopBits = USART_StopBits_1;
	usart_init.USART_Parity = USART_Parity_No;
	usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART6, &usart_init);
	USART_Cmd(USART6, ENABLE);
}

static void GPS_AutoConfigTick(void)
{
	u32 now;
	u8 ok;

	if(g_gps_cfg_done != 0u)
		return;

	now = OSTime * 20u;
	if(now < g_gps_cfg_next_ms)
		return;

	ok = 0;
	switch(g_gps_cfg_state)
	{
		case GPS_CFG_WAIT_BOOT:
			if(g_gps_uart_baud != GPS_TARGET_BAUD)
			{
				g_gps_cfg_state = GPS_CFG_SEND_PCAS01;
			}
			else
			{
				g_gps_cfg_state = GPS_CFG_SEND_PCAS04;
			}
			g_gps_cfg_next_ms = now + GPS_CFG_STEP_DELAY_MS;
			break;

		case GPS_CFG_SEND_PCAS01:
			ok = GPS_SendNmeaCommand("PCAS01,5");
			if(ok != 0u)
			{
				printf("[GPS][CFG] sent PCAS01,5, switching UART to 115200\r\n");
				g_gps_cfg_state = GPS_CFG_SWITCH_BAUD;
				g_gps_cfg_next_ms = now + GPS_CFG_BAUD_SWITCH_MS;
			}
			else
			{
				g_gps_cfg_next_ms = now + GPS_CFG_STEP_DELAY_MS;
			}
			break;

		case GPS_CFG_SWITCH_BAUD:
			GPS_USART6_SetBaud(GPS_TARGET_BAUD);
			g_gps_uart_baud = GPS_TARGET_BAUD;
			g_gps_cfg_state = GPS_CFG_SEND_PCAS04;
			g_gps_cfg_next_ms = now + GPS_CFG_BAUD_SWITCH_MS;
			break;

		case GPS_CFG_SEND_PCAS04:
			ok = GPS_SendNmeaCommand("PCAS04,3");
			if(ok != 0u)
			{
				printf("[GPS][CFG] sent PCAS04,3 (GPS+BDS)\r\n");
				g_gps_cfg_state = GPS_CFG_SEND_PCAS02;
				g_gps_cfg_next_ms = now + GPS_CFG_STEP_DELAY_MS;
			}
			else
			{
				g_gps_cfg_next_ms = now + GPS_CFG_STEP_DELAY_MS;
			}
			break;

		case GPS_CFG_SEND_PCAS02:
			ok = GPS_SendNmeaCommand("PCAS02,200");
			if(ok != 0u)
			{
				printf("[GPS][CFG] sent PCAS02,200 (5Hz)\r\n");
				g_gps_cfg_state = GPS_CFG_SEND_PCAS03;
				g_gps_cfg_next_ms = now + GPS_CFG_STEP_DELAY_MS;
			}
			else
			{
				g_gps_cfg_next_ms = now + GPS_CFG_STEP_DELAY_MS;
			}
			break;

		case GPS_CFG_SEND_PCAS03:
			ok = GPS_SendNmeaCommand("PCAS03,1,0,1,0,1,0,0,0,0,0,,,0,0,,,,0");
			if(ok != 0u)
			{
				printf("[GPS][CFG] sent PCAS03 (RMC+GGA+GSA only)\r\n");
				g_gps_cfg_state = GPS_CFG_DONE;
				g_gps_cfg_next_ms = now + GPS_CFG_STEP_DELAY_MS;
			}
			else
			{
				g_gps_cfg_next_ms = now + GPS_CFG_STEP_DELAY_MS;
			}
			break;

		case GPS_CFG_DONE:
			g_gps_cfg_done = 1u;
			printf("[GPS][CFG] done: baud=115200 mode=GPS+BDS rate=5Hz out=RMC/GGA/GSA\r\n");
			break;

		default:
			g_gps_cfg_state = GPS_CFG_WAIT_BOOT;
			g_gps_cfg_next_ms = now + GPS_CFG_STEP_DELAY_MS;
			break;
	}
}

void GPS_USART6_Init(u32 bound)
{
	GPIO_InitTypeDef gpio_init;
	USART_InitTypeDef usart_init;
	NVIC_InitTypeDef nvic_init;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

	gpio_init.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	gpio_init.GPIO_Mode = GPIO_Mode_AF;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_init.GPIO_OType = GPIO_OType_PP;
	gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &gpio_init);

	usart_init.USART_BaudRate = bound;
	usart_init.USART_WordLength = USART_WordLength_8b;
	usart_init.USART_StopBits = USART_StopBits_1;
	usart_init.USART_Parity = USART_Parity_No;
	usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART6, &usart_init);

	nvic_init.NVIC_IRQChannel = USART6_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 2;
	nvic_init.NVIC_IRQChannelSubPriority = 1;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);

	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART6, ENABLE);

	memset(&g_gps_data, 0, sizeof(g_gps_data));
	memset(gps_nmea_line, 0, sizeof(gps_nmea_line));
	memset(gps_nmea_sentence, 0, sizeof(gps_nmea_sentence));
	memset(&gps_csip_frame, 0, sizeof(gps_csip_frame));
	gps_nmea_idx = 0;
	gps_nmea_ready = 0;
	gps_csip_ready = 0;
	g_gps_last_print_ms = 0;
	g_gps_uart_baud = bound;
	g_gps_cfg_done = 0;
	g_gps_cfg_state = GPS_CFG_WAIT_BOOT;
	g_gps_cfg_next_ms = GPS_CFG_BOOT_DELAY_MS;
	GPS_ResetCsipRx();
}

void USART6_IRQHandler(void)
{
	u8 ch;

	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
	{
		ch = (u8)USART_ReceiveData(USART6);
		GPS_HandleNmeaByte(ch);
		GPS_HandleCsipByte(ch);
		USART_ClearITPendingBit(USART6, USART_IT_RXNE);
	}
}

void GPS_Process(void)
{
	char nmea_copy[GPS_NMEA_MAX_LEN];
	GPS_CsipFrame_t csip_copy;

	GPS_AutoConfigTick();

	while(1)
	{
		__disable_irq();
		if(gps_nmea_ready == 0)
		{
			__enable_irq();
			break;
		}
		gps_nmea_ready = 0;
		strncpy(nmea_copy, gps_nmea_sentence, sizeof(nmea_copy) - 1);
		nmea_copy[sizeof(nmea_copy) - 1] = '\0';
		__enable_irq();

		GPS_ParseNmeaSentence(nmea_copy);
	}

	while(1)
	{
		__disable_irq();
		if(gps_csip_ready == 0)
		{
			__enable_irq();
			break;
		}
		gps_csip_ready = 0;
		memcpy(&csip_copy, &gps_csip_frame, sizeof(csip_copy));
		__enable_irq();

		GPS_ParseCsipFrame(&csip_copy);
	}
}

void GPS_PrintStatus(void)
{
	u32 now;

	now = OSTime * 20u;
	if((now - g_gps_last_print_ms) < GPS_PRINT_INTERVAL_MS)
		return;
	g_gps_last_print_ms = now;

	if(g_gps_data.parsed == 0)
	{
		printf("[GPS] waiting data...\r\n");
		return;
	}

	if(g_gps_data.source == 2u)
	{
		printf("[GPS][CASIC] cls=0x%02X id=0x%02X fix=%u sv=%u lat=%.6f lon=%.6f alt=%.2fm v=%.2fm/s hdg=%.2f\r\n",
			   g_gps_data.last_casic_cls,
			   g_gps_data.last_casic_id,
			   g_gps_data.usefull,
			   g_gps_data.num_sv,
			   g_gps_data.latitude_deg,
			   g_gps_data.longitude_deg,
			   g_gps_data.altitude_m,
			   g_gps_data.speed_mps,
			   g_gps_data.heading_deg);
	}
	else
	{
		if(g_gps_data.usefull)
		{
			printf("[GPS][NMEA] %s UTC=%s LAT=%.6f%s LON=%.6f%s SPD=%.2fm/s HDG=%.2f\r\n",
				   g_gps_data.last_nmea_type,
				   g_gps_data.utc_time,
				   g_gps_data.latitude_deg, g_gps_data.ns,
				   g_gps_data.longitude_deg, g_gps_data.ew,
				   g_gps_data.speed_mps,
				   g_gps_data.heading_deg);
		}
		else
		{
			printf("[GPS][NMEA] parsed but no fix (%s)\r\n", g_gps_data.last_nmea_type);
		}
	}
}

u8 GPS_HasFix(void)
{
	return g_gps_data.usefull;
}

const GPS_Data_t *GPS_GetData(void)
{
	return &g_gps_data;
}

static u8 GPS_Dec2(const char *s)
{
	if(!s || s[0] < '0' || s[0] > '9' || s[1] < '0' || s[1] > '9')
		return 0;
	return (u8)((s[0] - '0') * 10 + (s[1] - '0'));
}

static u16 GPS_Dec4(const char *s)
{
	u16 v = 0;
	u8 i;

	if(!s)
		return 0;
	for(i = 0; i < 4; i++)
	{
		if(s[i] < '0' || s[i] > '9')
			return 0;
		v = (u16)(v * 10 + (u16)(s[i] - '0'));
	}
	return v;
}

static u8 GPS_IsLeapYear(u16 year)
{
	return (u8)(((year % 4u) == 0u && (year % 100u) != 0u) || ((year % 400u) == 0u));
}

u32 GPS_GetUtcUnixSec(void)
{
	u16 year;
	u8 month;
	u8 day;
	u8 hour;
	u8 minute;
	u8 second;
	u32 days;
	u16 y;
	const u16 month_days[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
	u16 dlen;

	if(!g_gps_data.usefull && !g_gps_data.pos_valid)
		return 0;
	if(g_gps_data.utc_time[0] == '\0' || g_gps_data.date[0] == '\0')
		return 0;

	hour = GPS_Dec2(g_gps_data.utc_time);
	minute = GPS_Dec2(g_gps_data.utc_time + 2);
	second = GPS_Dec2(g_gps_data.utc_time + 4);

	dlen = (u16)strlen(g_gps_data.date);
	if(dlen >= 8u)
	{
		day = GPS_Dec2(g_gps_data.date);
		month = GPS_Dec2(g_gps_data.date + 2);
		year = GPS_Dec4(g_gps_data.date + 4);
	}
	else if(dlen >= 6u)
	{
		day = GPS_Dec2(g_gps_data.date);
		month = GPS_Dec2(g_gps_data.date + 2);
		year = (u16)(2000u + GPS_Dec2(g_gps_data.date + 4));
	}
	else
		return 0;

	if(year < 1970u || month < 1u || month > 12u || day < 1u || day > 31u)
		return 0;

	days = 0;
	for(y = 1970u; y < year; y++)
		days += GPS_IsLeapYear(y) ? 366u : 365u;
	days += (u32)month_days[month - 1u];
	if(month > 2u && GPS_IsLeapYear(year))
		days++;
	days += (u32)(day - 1u);

	return days * 86400u + (u32)hour * 3600u + (u32)minute * 60u + (u32)second;
}

u8 GPS_SendNmeaCommand(const char *body)
{
	u8 cs;
	u16 i;
	char out[192];
	u16 pos;

	if((body == 0) || (body[0] == '\0'))
		return 0;

	cs = 0;
	for(i = 0; body[i] != '\0'; i++)
	{
		cs ^= (u8)body[i];
	}

	pos = (u16)snprintf(out, sizeof(out), "$%s*%02X\r\n", body, cs);
	if((pos == 0u) || (pos >= sizeof(out)))
		return 0;

	GPS_Usart6SendBytes((const u8 *)out, pos);
	return 1;
}

u8 GPS_SendCsipFrame(u8 cls, u8 id, const u8 *payload, u16 len)
{
	u8 head[6];
	u8 ckb[4];
	u32 ck;

	if((len & 0x03u) != 0u)
		return 0;
	if((len > 0u) && (payload == 0))
		return 0;

	head[0] = 0xBA;
	head[1] = 0xCE;
	head[2] = (u8)(len & 0xFFu);
	head[3] = (u8)((len >> 8) & 0xFFu);
	head[4] = cls;
	head[5] = id;

	ck = GPS_CalcCsipChecksum(cls, id, len, payload);
	ckb[0] = (u8)(ck & 0xFFu);
	ckb[1] = (u8)((ck >> 8) & 0xFFu);
	ckb[2] = (u8)((ck >> 16) & 0xFFu);
	ckb[3] = (u8)((ck >> 24) & 0xFFu);

	GPS_Usart6SendBytes(head, sizeof(head));
	if(len > 0u)
		GPS_Usart6SendBytes(payload, len);
	GPS_Usart6SendBytes(ckb, sizeof(ckb));
	return 1;
}
