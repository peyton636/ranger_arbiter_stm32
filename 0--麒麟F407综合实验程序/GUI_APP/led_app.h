#ifndef _led_app_H
#define _led_app_H

#include "gui.h"


typedef struct 
{
	u16 x;	//x,y,r LED位置半径
	u16 y;
	u8 r;
	u16 fcolor;//LED显示颜色
	u16	bcolor;//LED背景颜色
	u8 *caption;//LED名称
	u8 xspace;	//LED间距X
	u8 yspace;	//LED间距Y
}_led_ctrl;

extern _led_ctrl led_ctrl;


void LED_APP_Test(void);


#endif
