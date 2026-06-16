#include "agv_blob_wire.h"

u16 Blob_PayloadSize(u8 msg_id)
{
	switch(msg_id)
	{
		case BLOB_MSG_CONTROL:     return BLOB_PAYLOAD_CONTROL;
		case BLOB_MSG_MOTION:      return BLOB_PAYLOAD_MOTION;
		case BLOB_MSG_MCU_STATUS:  return BLOB_PAYLOAD_MCU_STATUS;
		case BLOB_MSG_SENSOR:      return BLOB_PAYLOAD_SENSOR;
		case BLOB_MSG_GPS:         return BLOB_PAYLOAD_GPS;
		case BLOB_MSG_MOTOR04:     return BLOB_PAYLOAD_MOTOR04;
		case BLOB_MSG_MOTOR58:     return BLOB_PAYLOAD_MOTOR58;
		case BLOB_MSG_ENERGY:      return BLOB_PAYLOAD_ENERGY;
		case BLOB_MSG_MOTOR_POS:   return BLOB_PAYLOAD_MOTOR_POS;
		case BLOB_MSG_SENSOR_CFG:  return BLOB_PAYLOAD_SENSOR_CFG;
		default:                   return 0;
	}
}

u8 Blob_ValidatePayloadLen(u8 msg_id, u16 len)
{
	u16 expect = Blob_PayloadSize(msg_id);

	if(expect == 0)
		return 0;
	return (len == expect) ? 1 : 0;
}
