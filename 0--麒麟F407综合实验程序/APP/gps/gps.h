#ifndef __GPS_H
#define __GPS_H

#include "system.h"

#define GPS_NMEA_MAX_LEN        160
#define GPS_CSIP_MAX_PAYLOAD    256

typedef struct
{
	u8 has_data;
	u8 parsed;
	u8 usefull;
	u8 source; /* 0:none 1:NMEA 2:CASIC */

	/* NMEA�����ֶΣ�����ԭʼ���̽ӿ����壩 */
	char utc_time[16];
	char latitude_raw[16];
	char ns[2];
	char longitude_raw[16];
	char ew[2];
	double latitude_deg;
	double longitude_deg;

	/* ��չ״̬ */
	char date[16];
	float altitude_m;
	float speed_mps;
	float heading_deg;
	float hdop;
	u8 num_sv;
	u8 pos_valid;
	u8 vel_valid;
	u8 nav_system;

	u8 last_casic_cls;
	u8 last_casic_id;
	u8 last_nmea_type[6];

	u32 nmea_ok_count;
	u32 nmea_crc_err_count;
	u32 csip_ok_count;
	u32 csip_crc_err_count;
	u32 csip_drop_count;
} GPS_Data_t;

void GPS_USART6_Init(u32 bound);
void GPS_Process(void);
void GPS_PrintStatus(void);
u8 GPS_HasFix(void);
const GPS_Data_t *GPS_GetData(void);

/* ����NMEA�����壨����$��*CS�������磺PCAS03,1,1,1,1,1,1,1,1,0,0,,,1,1,,,,1 */
u8 GPS_SendNmeaCommand(const char *body);

/* ����CASIC������CSIP֡ */
u8 GPS_SendCsipFrame(u8 cls, u8 id, const u8 *payload, u16 len);

/* �ж���ڣ���������ֱ�ӵ��� */
void USART6_IRQHandler(void);

#endif
