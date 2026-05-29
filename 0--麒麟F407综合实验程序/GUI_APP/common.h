#ifndef __COMMON_H
#define __COMMON_H 

#include "system.h"
#include "tftlcd.h"
#include "touch.h"
#include "guix.h"


//根据TFTLCD触摸屏型号选择像素大小，选择对应的宏定义值
#if	defined(TFTLCD_HX8357D)||defined(TFTLCD_HX8352C)||defined(TFTLCD_ILI9341)|| \
	defined(TFTLCD_R61509V)||defined(TFTLCD_R61509VN)||defined(TFTLCD_R61509V3)|| \
	defined(TFTLCD_ST7793)||defined(TFTLCD_ILI9325)||defined(TFTLCD_R61509VE)|| \
	defined(TFTLCD_SSD1963N)
#define TFTLCD_PIXEL_240X320	//TFTLCD触摸屏像素：240*320、240*400
#endif

#if	defined(TFTLCD_ILI9486)||defined(TFTLCD_ILI9327)|| \
	defined(TFTLCD_HX8357DN)||defined(TFTLCD_ILI9481)|| \
	defined(TFTLCD_ILI9488)
#define TFTLCD_PIXEL_320X480	//TFTLCD触摸屏像素：320*480
#endif

#if	defined(TFTLCD_NT35510)||defined(TFTLCD_SSD1963)|| \
	defined(TFTLCD_ILI9806) 
#define TFTLCD_PIXEL_480X800	//TFTLCD触摸屏像素：480*800
#endif


//开机LOGO图标尺寸
#define PIC_KJ_LOGO_WIDTH		175
#define PIC_KJ_LOGO_HEIGHT		83

//UI界面背景、前景色
#define UI_BACKCOLOR			WHITE	//白色背景
#define UI_FRONTCOLOR			BLACK	//黑色文字



//#ifdef TFTLCD_PIXEL_240X320		//TFTLCD触摸屏像素：240*320、240*400
////ICON应用图标尺寸
//#define PIC_ICON_APP_WIDTH		46
//#define PIC_ICON_APP_HEIGHT		46

//#define PIC_ICON_APP_XSTART		10
//#define PIC_ICON_APP_YSTART		20

//#define PIC_ICON_APP_XSPACE		10
//#define PIC_ICON_APP_YSPACE		20

////APP应用图标名称显示尺寸
//#define PIC_ICON_APP_NAME_WIDTH		PIC_ICON_APP_WIDTH
//#define PIC_ICON_APP_NAME_HEIGHT	16
//#define PIC_ICON_APP_NAME_FONT_SIZE	12

////顶部状态显示（产品型号、时间）
//#define TOP_STATUS_NAME_HEIGHT		16
//#define TOP_STATUS_NAME_FONT_SIZE	12
//#endif

#ifdef TFTLCD_PIXEL_240X320		//TFTLCD触摸屏像素：240*320、240*400
//ICON应用图标尺寸
#define PIC_ICON_APP_WIDTH		46
#define PIC_ICON_APP_HEIGHT		46

#define PIC_ICON_APP_XSTART		15
#define PIC_ICON_APP_YSTART		20

#define PIC_ICON_APP_XSPACE		20
#define PIC_ICON_APP_YSPACE		20

//APP应用图标名称显示尺寸
#define PIC_ICON_APP_NAME_WIDTH		PIC_ICON_APP_WIDTH
#define PIC_ICON_APP_NAME_HEIGHT	16
#define PIC_ICON_APP_NAME_FONT_SIZE	12

//顶部状态显示（产品型号、时间）
#define TOP_STATUS_NAME_HEIGHT		16
#define TOP_STATUS_NAME_FONT_SIZE	12
#endif

#ifdef TFTLCD_PIXEL_320X480		//TFTLCD触摸屏像素：320*480
#define PIC_ICON_APP_WIDTH		66
#define PIC_ICON_APP_HEIGHT		66

#define PIC_ICON_APP_XSTART		13
#define PIC_ICON_APP_YSTART		25

#define PIC_ICON_APP_XSPACE		10
#define PIC_ICON_APP_YSPACE		30

//APP应用图标名称显示尺寸
#define PIC_ICON_APP_NAME_WIDTH		PIC_ICON_APP_WIDTH
#define PIC_ICON_APP_NAME_HEIGHT	20
#define PIC_ICON_APP_NAME_FONT_SIZE	12

//顶部状态显示（产品型号、时间）
#define TOP_STATUS_NAME_HEIGHT		20
#define TOP_STATUS_NAME_FONT_SIZE	16
#endif

#ifdef TFTLCD_PIXEL_480X800		//TFTLCD触摸屏像素：480*800
#define PIC_ICON_APP_WIDTH		86
#define PIC_ICON_APP_HEIGHT		86

#define PIC_ICON_APP_XSTART		25
#define PIC_ICON_APP_YSTART		25

#define PIC_ICON_APP_XSPACE		30
#define PIC_ICON_APP_YSPACE		30

//APP应用图标名称显示尺寸
#define PIC_ICON_APP_NAME_WIDTH		PIC_ICON_APP_WIDTH
#define PIC_ICON_APP_NAME_HEIGHT	20
#define PIC_ICON_APP_NAME_FONT_SIZE	16

//顶部状态显示（产品型号、时间）
#define TOP_STATUS_NAME_HEIGHT		20
#define TOP_STATUS_NAME_FONT_SIZE	16
#endif



//平滑线的起止颜色定义
#define WIN_SMOOTH_LINE_SEC	0XB1FFC4	//起止颜色
#define WIN_SMOOTH_LINE_MC	0X1600B1	//中间颜色

//弹出窗口选择条目的设置信息
#define APP_ITEM_BTN1_WIDTH		60	  		//有2个按键时的宽度
#define APP_ITEM_BTN2_WIDTH		100			//只有1个按键时的宽度
#define APP_ITEM_BTN_HEIGHT		30			//按键高度
#define APP_ITEM_ICO_SIZE		32			//ICO图标的尺寸

#define APP_ITEM_SEL_BKCOLOR	0X0EC3		//选择时的背景色
#define APP_WIN_BACK_COLOR	 	0XC618		//窗体背景色


extern u8 rtc_showflag;
extern vu8 system_task_return;		//任务强制返回标志
extern u8*const APP_REMIND_CAPTION_TBL[GUI_LANGUAGE_NUM];
extern u8*const APP_MODESEL_CAPTION_TBL[GUI_LANGUAGE_NUM];
extern u8*const APP_SAVE_CAPTION_TBL[GUI_LANGUAGE_NUM];
extern u8*const APP_CREAT_ERR_MSG_TBL[GUI_LANGUAGE_NUM];

extern u8*const APP_OK_PIC;			//确认图标
extern u8*const APP_CANCEL_PIC;		//取消图标
extern u8*const APP_UNSELECT_PIC;	//未选中图标
extern u8*const APP_SELECT_PIC;		//选中图标
extern u8*const APP_VOL_PIC;		//音量图片路径


void LOGO_Display(void);
void ICON_UI_Init(void);
u8 get_icon_app_table(void);

void app_filebrower(u8 *topname,u8 mode);
void app_gui_tcbar(u16 x,u16 y,u16 width,u16 height,u8 mode);
u8 * app_get_icopath(u8 mode,u8 *selpath,u8 *unselpath,u8 selx,u8 index);
void app_draw_smooth_line(u16 x,u16 y,u16 width,u16 height,u32 sergb,u32 mrgb);
u8 app_tp_is_in_area(_m_tp_dev *tp,u16 x,u16 y,u16 width,u16 height);
void app_show_items(u16 x,u16 y,u16 itemwidth,u16 itemheight,u8*name,u8*icopath,u16 color,u16 bkcolor);
u8 app_items_sel(u16 x,u16 y,u16 width,u16 height,u8 *items[],u8 itemsize,u8 *selx,u8 mode,u8*caption);
void app_srand(u32 seed);
u32 app_get_rand(u32 max);
void app_read_bkcolor(u16 x,u16 y,u16 width,u16 height,u16 *ctbl);
void app_recover_bkcolor(u16 x,u16 y,u16 width,u16 height,u16 *ctbl);
#endif
