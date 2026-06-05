#ifndef CHASSIS_CAN_TEST_H
#define CHASSIS_CAN_TEST_H

/* 1=左转(斜移90°)，0=右转(斜移90°)；开关在 main.c 的 CHASSIS_CAN_MOTION_TEST */
#define CHASSIS_TEST_TURN_LEFT   1

void ChassisCanTest_RunOnce(void);

#endif
