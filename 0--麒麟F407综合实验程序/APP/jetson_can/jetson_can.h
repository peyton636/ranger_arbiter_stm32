#ifndef __JETSON_CAN_H

#define __JETSON_CAN_H



#include "system.h"

#include "arbiter.h"

#include "gps.h"



/* Jetson 固定以太网 UDP；CAN2/RS232 链路已弃用 */
#define JETSON_LINK_CAN  0



/* Jetson CAN 协议统一错误码（0x109） */

#define JETSON_ERR_OK                 0x00u

#define JETSON_ERR_GPS_NO_FIX         0x02u

#define JETSON_ERR_DIST_SENSOR_FAULT  0x03u

#define JETSON_ERR_CHASSIS_CAN_FAULT  0x04u

#define JETSON_ERR_JETSON_HB_LOST     0x10u

#define JETSON_ERR_CHASSIS_SYS_ERR    0x20u

#define JETSON_ERR_CHASSIS_FAULT_INFO 0x21u

#define JETSON_ERR_OBSTACLE_EMERGENCY 0x30u

#define JETSON_ERR_DIST_ALL_UNKNOWN   0x40u



void Protocol_Init(void);

void JetsonCAN_ProcessRx(const ArbiterState_t *state, u16 nearest_mm);

u8 JetsonCAN_GetFrame(u8 *frame);

void JetsonCAN_HandleServiceRequest(u32 can_id, const u8 *buf, u8 dlc,
	const ArbiterState_t *state, u16 nearest_mm);

void JetsonCAN_ServiceFault(const ArbiterState_t *state);

void JetsonCAN_SendV3Frame(u32 can_id, const u8 *frame);

void JetsonCAN_SendGps(const GPS_Data_t *gps);



#endif


