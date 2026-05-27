#ifndef _picture_app_H
#define _picture_app_H

#include "gui.h"

//图片浏览模式
//0：顺序播放，1：随机播放
extern u8 picmode;

void Picture_APP_Test(void);
u8 pic_tp_scan(void);
#endif
