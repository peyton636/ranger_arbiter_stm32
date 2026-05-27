#ifndef _calculator_app_H
#define _calculator_app_H

#include "gui.h"


__packed typedef struct 
{							  
	u16 xoff;	//x方向偏移:2,10,10
	u16 yoff;	//y方向偏移:10,20,50
	u16 width;	//宽度
	u16 height;	//高度
	u8 xdis;	//数据区域x方向间隔大小:2,4,2
	u8 ydis;	//数据区域y方向间隔大小:5,11,19
	u8 fsize;	//主结果字体大小:28,36,60 
}_calcdis_obj;
extern _calcdis_obj *cdis;



u8 calc_show_res(_calcdis_obj*calcdis,double res);
void calc_input_fresh(_calcdis_obj*calcdis,u8* calc_sta,u8 *inbuf,u8 len);
void calc_ctype_show(_calcdis_obj*calcdis,u8 ctype);
void calc_show_flag(_calcdis_obj*calcdis,u8 fg);
void calc_show_exp(_calcdis_obj*calcdis,short exp);
u8 calc_exe(_calcdis_obj*calcdis,double *x1,double *x2,u8 *buf,u8 ctype,u8 *calc_sta);
void calc_show_inbuf(_calcdis_obj*calcdis,u8 *buf);
void calc_load_ui(_calcdis_obj*calcdis);


void Calculator_APP_Test(void);

#endif
