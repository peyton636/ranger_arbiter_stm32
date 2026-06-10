#ifndef _MOTION_CONTROL_H
#define _MOTION_CONTROL_H

#include "system.h"

void MotionControl_KeyProcess(void);
void MotionControl_BeepUpdateByDistance(u16 nearest_mm);
void MotionControl_Run(void);

#endif
