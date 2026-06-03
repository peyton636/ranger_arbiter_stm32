#include "system.h"
#include "SysTick.h"
#include "led.h"
#include "usart.h"
#include "tftlcd.h"
#include "key.h"
#include "beep.h"
#include "24cxx.h"
#include "rtc.h"
#include "adc.h"
#include "adc_temp.h"
#include "touch_key.h"
#include "lsens.h"
#include "ds18b20.h"
#include "time.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "touch.h"
#include "flash.h"
#include "sdio_sdcard.h"
#include "hwjs.h"
#include "malloc.h" 
#include "sram.h"
#include "rs232.h"
#include "fatfs_app.h"
#include "ff.h"
#include "font_show.h"
#include "piclib.h"
#include "string.h"		
#include "math.h"
#include "wm8978.h"
#include "lwip/netif.h"
#include "lwip_comm.h"
#include "lwipopts.h"


#include "common.h" 
#include "distance_sensor.h" 
#include "can.h" 
#include "arbiter.h"
#include "stdio.h"


// 版本号定义
u8 VERSION[]="HARDWARE:V1.0   SOFTWARE:V1.2";

// 外部内存测试（最多支持 1MB 字节）
// x,y: 显示坐标
// fsize: 字体大小
// 返回值: 0 成功; 1 失败
u8 system_exsram_test(u16 x,u16 y,u8 fsize)
{  
	u32 i=0;  	  
	u16 temp=0;	   
	u16 sval=0;	// 地址 0 处保存的初始值
  	LCD_ShowString(x,y,tftlcd_data.width,y+fsize,fsize,"Ex Memory Test:   0KB"); 
	// 每 1KB 写入一个数据，共写 1024 次，覆盖 1MB 空间
	for(i=0;i<1024*1024;i+=1024)
	{
		FSMC_SRAM_WriteBuffer((u8*)&temp,i,2);
		temp++;
	}
	// 读取之前写入的数据并进行校验
 	for(i=0;i<1024*1024;i+=1024) 
	{
  		FSMC_SRAM_ReadBuffer((u8*)&temp,i,2);
		if(i==0)sval=temp;
 		else if(temp<=sval)break;// 后一次读出的值必须大于前一次，否则判定异常
		LCD_ShowxNum(x+15*(fsize/2),y,(u16)(temp-sval+1),4,fsize,0);// 显示内存大小
 	}
	if(i>=1024*1024)
	{
		LCD_ShowxNum(x+15*(fsize/2),y,i/1024,4,fsize,0);// 显示内存容量
		return 0;// 内存正常，成功
	}
	return 1;// 失败
}

// 显示错误信息
// x,y: 坐标
// fsize: 字体大小
// err: 错误字符串
void system_error_show(u16 x,u16 y,u8*err,u8 fsize)
{
	FRONT_COLOR=RED;
 	while(1)
	{
		LCD_ShowString(x,y,tftlcd_data.width,tftlcd_data.height,fsize,err);
		delay_ms(200);
		LCD_Fill(x,y,tftlcd_data.width,y+fsize,BLACK);
		delay_ms(200);
	} 
}

// 擦除 SPI FLASH（系统资源文件），用于恢复系统
// x,y: 坐标
// fsize: 字体大小
// 返回值: 0 未擦除; 1 已擦除
u8 system_files_erase(u16 x,u16 y,u8 fsize)
{
	u8 key;
	u8 t=0;
	FRONT_COLOR=RED;
	LCD_ShowString(x,y,tftlcd_data.width,tftlcd_data.height,fsize,"Erase all system files?");
	while(1)
	{
		t++;
		if(t==20)LCD_ShowString(x,y+fsize,tftlcd_data.width,tftlcd_data.height,fsize,"KEY0:NO / KEY1:YES");
		if(t==40)
		{
			gui_fill_rectangle(x,y+fsize,tftlcd_data.width,fsize,BLACK);// 清除提示显示
			t=0;
			LED1=!LED1;
		}
		key=KEY_Scan(0);
		if(key==KEY0_PRESS)// 取消操作，不擦除
		{ 
			gui_fill_rectangle(x,y,tftlcd_data.width,fsize*2,BLACK);// 清除提示显示
			FRONT_COLOR=WHITE;
			LED1=1;
			return 0;
		}
		if(key==KEY1_PRESS)// 确认擦除，执行操作
		{
			LED1=1;
			LCD_ShowString(x,y+fsize,tftlcd_data.width,tftlcd_data.height,fsize,"Erasing SPI FLASH...");
			EN25QXX_Erase_Chip();
			LCD_ShowString(x,y+fsize,tftlcd_data.width,tftlcd_data.height,fsize,"Erasing SPI FLASH OK");
			delay_ms(600);
			return 1;
		}
		delay_ms(10);
	}
}

// 字库更新确认提示
// x,y: 坐标
// fsize: 字体大小
// 返回值: 0 不更新; 1 确认更新
u8 system_font_update_confirm(u16 x,u16 y,u8 fsize)
{
	u8 key;
	u8 t=0;
	u8 res=0;
	FRONT_COLOR=RED;
	LCD_ShowString(x,y,tftlcd_data.width,tftlcd_data.height,fsize,"Update font?");
	while(1)
	{
		t++;
		if(t==20)LCD_ShowString(x,y+fsize,tftlcd_data.width,tftlcd_data.height,fsize,"KEY1:NO / KEY0:YES");
		if(t==40)
		{
			LCD_Fill(x,y+fsize,tftlcd_data.width,y+fsize+fsize,BLACK);// 清除提示显示
			t=0;
		}
		key=KEY_Scan(0);
		if(key==KEY1_PRESS)break;// 不更新
		if(key==KEY0_PRESS){res=1;break;}// 确认更新
		delay_ms(10);
	}
	LCD_Fill(x,y,tftlcd_data.width,y+2*fsize,BLACK);// 清除提示显示
	FRONT_COLOR=WHITE;
	return res;
}


// 硬件检查
void Hardware_Check(void)
{
	u16 okoffset=162;
 	u16 ypos=0;
	u16 j=0;
	u16 temp=0;
	u8 res;
	u32 dtsize,dfsize;
	u8 *stastr=0;
	u8 fsize;
	
REINIT:// 重新初始化
	SysTick_Init(168);				// 延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  // 中断优先级分组 2
	USART1_Init(115200);		// 初始化串口1，波特率 115200
 	LED_Init();					// 初始化 LED
 	TFTLCD_Init();					// LCD 初始化
 	BEEP_Init();				// 蜂鸣器初始化
 	KEY_Init();					// 按键初始化
	FSMC_SRAM_Init();			// 初始化外部 SRAM
 	AT24CXX_Init();    		// EEPROM 初始化
	EN25QXX_Init();				// 初始化 W25Q128
 	Lsens_Init();				// 初始化光敏传感器
	ADC_Temp_Init();			// 初始化内部温度传感器
	my_mem_init(SRAMIN);		// 初始化内部内存池
	my_mem_init(SRAMCCM);		// 初始化 CCM 内存池
	TP_Init();
	piclib_init();				// piclib 初始化
	gui_init();	  				// GUI 初始化
	FATFS_Init();				// 为 fatfs 申请相关内存
	
//	LOGO_Display();// 显示 LOGO
	LCD_Clear(BLACK);			// 清屏
	FRONT_COLOR=WHITE;
	BACK_COLOR=BLACK;
	j=0;
	// 显示版权信息
	ypos=2;
	if(tftlcd_data.width<=272)
	{
		fsize=16;
		okoffset=190;
	}else if(tftlcd_data.width==320)
	{
		fsize=16;
		okoffset=250;		
	}else if(tftlcd_data.width==480)
	{
		fsize=24;
		okoffset=370;		
	}
	LCD_ShowString(5,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize,"---PRECHIN---");
	LCD_ShowString(5,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize,VERSION);
	
	// 开始硬件检测初始化
	WM8978_Init();// 禁止左右声道输出
	WM8978_HPvol_Set(0,0);// 关闭耳机输出
	WM8978_SPKvol_Set(0);// 关闭喇叭输出
	LED1=0;LED2=0;				// 同时点亮两个 LED
	LCD_ShowString(5,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "CPU:STM32F407ZGT6 168Mhz");
	LCD_ShowString(5,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "FLASH:1024KB  SRAM:192KB");	
	if(system_exsram_test(5,ypos+fsize*j,fsize))
		system_error_show(5,ypos+fsize*j++,"EX Memory Error!",fsize);
	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize,"OK");			 
	my_mem_init(SRAMEX);		// 初始化外部内存池（内存测试后）
	// 外部 Flash 检查
	if((EN25QXX_ReadID()==0x00)||(EN25QXX_ReadID()==0xFF))// 检测不到 EN25QXX
	{	 
		system_error_show(5,ypos+fsize*j++,"Ex Flash Error!!",fsize); 
	}else temp=16*1024;	// 16MB 大小
	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "Ex Flash:     KB");			   
	LCD_ShowxNum(5+9*(fsize/2),ypos+fsize*j,temp,5,fsize,0);// 显示 flash 容量
	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");
	// 检测是否需要擦除 SPI FLASH
	res=KEY_Scan(1);//
	if(res==KEY_UP_PRESS)
	{
		res=system_files_erase(5,ypos+fsize*j,fsize);
		if(res)goto REINIT; 
	}
    // RTC 检测
  	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "RTC Check...");			   
 	if(RTC_Config())system_error_show(5,ypos+fsize*(j+1),"RTC Error!",fsize);// RTC 错误
	else 
	{
		RTC_Set_WakeUp(RTC_WakeUpClock_CK_SPRE_16bits,0);// 配置 WAKE UP 中断，1 秒中断一次
		RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);// 获取当前时间
		RTC_GetDate(RTC_Format_BIN,&RTC_DateStruct);// 获取当前日期
		LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");			   
	}
	// 挂载 SPI FLASH 文件系统
	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "FATFS Check...");//FATFS   			   
  	f_mount(fs[0],"0:",1); 		// 挂载 SD 卡
  	f_mount(fs[1],"1:",1); 		// 挂载板载 FLASH
 	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");			   
	// SD 卡检测
	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "SD Card:     MB");//FATFS   
	temp=0;	
 	do
	{
		temp++;
 		res=fatfs_getfree("0:",&dtsize,&dfsize);// 获取 SD 卡剩余容量信息
		delay_ms(200);		   
	}while(res&&temp<5);// 最多尝试 5 次
 	if(res==0)// 获取成功
	{ 
		gui_phy.memdevflag|=1<<0;	// 设置 SD 卡标志位
		temp=dtsize>>10;// 单位转换为 MB
		stastr="OK";
 	}else 
	{
 		temp=0;// 获取失败，容量置 0
		stastr="ERROR";
	}
 	LCD_ShowxNum(5+8*(fsize/2),ypos+fsize*j,temp,5,fsize,0);					// 显示 SD 卡容量大小
	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize,stastr);	// SD 状态
	// W25Q128 检测：若无文件系统则自动创建
	temp=0;	
 	do
	{
		temp++;
 		res=fatfs_getfree("1:",&dtsize,&dfsize);// 获取 FLASH 剩余容量信息
		delay_ms(200);		   
	}while(res&&temp<20);// 最多尝试 20 次
	if(res==0X0D)// 文件系统错误
	{
		LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "Flash Disk Formatting...");	// 格式化 FLASH
		res=f_mkfs("1:",1,4096);// 格式化 FLASH，1=盘符，4096=簇大小
		if(res==0)
		{
			f_setlabel((const TCHAR *)"1:PRECHIN");	// 设置 Flash 磁盘卷标为 PRECHIN
			LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");// 标记格式化成功
 			res=fatfs_getfree("1:",&dtsize,&dfsize);// 重新获取容量
		}
	}   
	if(res==0)// 获取 FLASH 剩余容量成功
	{
		gui_phy.memdevflag|=1<<1;	// 设置 SPI FLASH 标志位
		LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "Flash Disk:     KB");//FATFS   			   
		temp=dtsize; 	   
 	}else system_error_show(5,ypos+fsize*(j+1),"Flash Fat Error!",fsize);	// flash 文件系统错误
 	LCD_ShowxNum(5+11*(fsize/2),ypos+fsize*j,temp,5,fsize,0);						// 显示 FLASH 容量大小
	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize,"OK");			// FLASH 状态
	// TPAD 检测
 	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "TPAD Check...");			   
// 	if(Touch_Key_Init(4))system_error_show(5,ypos+fsize*(j+1),"TPAD Error!",fsize);// 触摸按键检测
//	else LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK"); 
	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK"); 
	// 24C02 检测
   	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "24C02 Check...");			   
 	if(AT24CXX_Check())system_error_show(5,ypos+fsize*(j+1),"24C02 Error!",fsize);// 24C02 异常
	else LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");  
	// MPU6050 检测
   	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "MPU6050 Check...");			   
 	if(MPU6050_Init())system_error_show(5,ypos+fsize*j++,"MPU6050 Error!",fsize);
	else LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");
	// WM8978 检测
 	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "WM8978 Check...");			   
 	if(WM8978_Init())system_error_show(5,ypos+fsize*(j+1),"WM8978 Error!",fsize);// WM8978 异常
	else 
	{
		LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");	
		WM8978_HPvol_Set(0,0);// 关闭耳机输出
		WM8978_SPKvol_Set(0);// 关闭喇叭输出
  	}
//	// LAN8720 检测
//	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "LAN8720 Check...");			   
// 	if(LAN8720_Init())system_error_show(5,ypos+fsize*(j+1),"LAN8720 Error!",fsize);// LAN8720 异常
//	else LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK"); 
//	LAN8720_RST=0;		// 复位 LAN8720
	// 字库检查
   	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "Font Check...");
	res=KEY_Scan(1);// 检测按键
	if(res==KEY1_PRESS)// 按下确认键
	{
		res=system_font_update_confirm(5,ypos+fsize*(j+1),fsize);
	}else res=0;
	if(font_init()||(res==1))// 字库异常或强制更新时，更新字库
	{
		res=0;// 按键无效
 		if(update_font(5,ypos+fsize*j,fsize,"0:")!=0)// 从 SD 卡更新
		{
 			system_error_show(5,ypos+fsize*(j+1),"Font Error!",fsize);	// 字库错误
		}
		else 
		{
			ypos=0;
			goto REINIT;
		}
		LCD_Fill(5,ypos+fsize*j,tftlcd_data.width,ypos+fsize*(j+1),BLACK);// 黑底清除
    	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "Font Check...");			   
 	} 
	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");// 字库检测 OK
	// 触摸检测
 	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "Touch Check...");			   
	res=KEY_Scan(1);// 检测按键
	if(TP_Init()||(res==KEY0_PRESS&&(tp_dev.touchtype&0X80)==0))// 有故障/按下 KEY0 且非电容屏时，执行校准
	{
		if(res==KEY0_PRESS)TP_Adjust();
		res=0;// 按键无效
		goto REINIT;				// 重新开始初始化
	}
	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");// 触摸检测 OK
   	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "SYSTEM Starting...");  
	// 熄灭 LED
	LED1=1;LED2=1;
	// 蜂鸣器短响，提示启动完成
	BEEP=1;
	delay_ms(100);
	BEEP=0; 
	delay_ms(500);
	TIM4_Init(200,8399);		// 定时 20ms
//	TIM3_Init(500,8399);		// 定时 50ms
	TIM2_Init(1000,8399);		// 10KHz 计数频率，100ms 中断
//	delay_ms(1000);
//	delay_ms(1000);
}




extern void LED_APP_Test(void);
extern void RTC_APP_Test(void);
extern void Calculator_APP_Test(void);
extern void Gyroscope_APP_Test(void);
extern void Picture_APP_Test(void);
extern void Paint_APP_Test(void);
extern void Ebook_APP_Test(void);
extern void Notepad_APP_Test(void);
extern void USB_APP_Test(void);
extern void Ethernet_APP_Test(void);
extern void Music_APP_Test(void);
extern void Camera_APP_Test(void);
extern void COM_APP_Test(void);
extern void Wireless_APP_Test(void);
extern void Phone_APP_Test(void);
extern void Qrcode_APP_Test(void);

// 传感器数据刷屏显示函数
void SensorData_ShowScreen(void);

// LCD 固定布局（16号字体，行高16px）
#define UI_X          5
#define UI_FS         16
#define UI_Y_TITLE    20
#define UI_Y_COUNT    42
#define UI_Y_DIST_HDR 64
#define UI_Y_IF1      80
#define UI_Y_IF2      96
#define UI_Y_IF3      112
#define UI_Y_IF4      128
#define UI_Y_CHASSIS  150
#define UI_Y_MOTION   166
#define UI_Y_WFL      182
#define UI_Y_WRL      198
#define UI_Y_BATT     214
#define UI_Y_BEEP     230
#define UI_X_VAL      45
#define UI_CAN_LCD_DIV 30   // 约300ms刷新底盘区，减轻串口/LCD负担
#define UI_LINE_W     230   // 固定行宽，避免数字长短不一跳动
#define CRUISE_SPEED_MM_S  300   // 无Jetson时默认前进速度 mm/s
#define ARBITER_CAN_DIV    2     // 主循环10ms×2=20ms发一次0x111
// IF1~IF4 到车体方向映射（可按实际安装调整）
#define DS_IDX_FRONT   0
#define DS_IDX_BACK    1
#define DS_IDX_LEFT    2
#define DS_IDX_RIGHT   3
// 界面分流开关：
// 0 = 旧版图标界面（legacy_gui）
// 1 = 传感器/仲裁界面（arbiter_gui）
// 2 = 自定义界面占位（my_gui）
#define UI_TEST_MODE       1

static u8 sensor_ui_inited = 0;
static u8 g_beep_dist_enable = 1;   // 距离蜂鸣开关：1=开 0=关，KEY1切换
static u16 g_nearest_dist_mm = 0xFFFF;
static u16 g_front_dist_mm = 0xFFFF;
static u16 g_back_dist_mm = 0xFFFF;
static u16 g_left_dist_mm = 0xFFFF;
static u16 g_right_dist_mm = 0xFFFF;
static u8 g_beep_ui_dirty = 0;

// KEY1 切换距离蜂鸣开关
static void Beep_ToggleSwitch(void)
{
	u8 key;

	key = KEY_Scan(0);
	if(key == KEY1_PRESS)
	{
		g_beep_dist_enable = !g_beep_dist_enable;
		if(!g_beep_dist_enable)
			BEEP_SetVolume(0);
		g_beep_ui_dirty = 1;
		printf("[BEEP] Distance alert %s (KEY1 toggle)\r\n",
			g_beep_dist_enable ? "ON" : "OFF");
		delay_ms(200);
	}
}

// 根据最近距离更新蜂鸣器音量（main 内实现，受 g_beep_dist_enable 控制）
static void Beep_UpdateByDistance(u16 nearest_mm)
{
	u8 vol;

	if(!g_beep_dist_enable || nearest_mm == 0xFFFF)
	{
		BEEP_SetVolume(0);
		return;
	}

	if(nearest_mm >= ARBITER_OBSTACLE_FAR_MM)
	{
		vol = 0;
	}
	else if(nearest_mm < ARBITER_OBSTACLE_WARN_MM)
	{
		vol = 100;
	}
	else
	{
		vol = 20 + (u8)((ARBITER_OBSTACLE_FAR_MM - nearest_mm) * 60 /
			(ARBITER_OBSTACLE_FAR_MM - ARBITER_OBSTACLE_WARN_MM));
	}

	BEEP_SetVolume(vol);
}

// 距离避障运动控制：更新障碍物距离 + 仲裁 + CAN下发
static void Motion_ControlByDistance(void)
{
	static u16 can_send_div = 0;

	Arbiter_SetLocalCmd(CRUISE_SPEED_MM_S, 0);
	Arbiter_SetObstacleDistances(
		g_front_dist_mm,
		g_back_dist_mm,
		g_left_dist_mm,
		g_right_dist_mm);
	Arbiter_Process();

	can_send_div++;
	if(can_send_div >= ARBITER_CAN_DIV)
	{
		can_send_div = 0;
		Arbiter_SendToSTM32A();
	}
}

// 局部擦除固定宽度区域后重绘（中文）
static void SensorUI_UpdateLineGBK(u16 y, const char *text)
{
	LCD_Fill(UI_X, y, UI_X + UI_LINE_W, y + UI_FS - 1, BLACK);
	LCD_ShowFontString(UI_X, y, tftlcd_data.width, tftlcd_data.height, (u8*)text, UI_FS, 0);
}

// 局部擦除固定宽度区域后重绘（ASCII）
static void SensorUI_UpdateLine(u16 y, const char *text)
{
	LCD_Fill(UI_X, y, UI_X + UI_LINE_W, y + UI_FS - 1, BLACK);
	LCD_ShowString(UI_X, y, tftlcd_data.width, tftlcd_data.height, UI_FS, (u8*)text);
}

// 局部擦除数值区后重绘
static void SensorUI_UpdateValue(u16 y, const char *text)
{
	LCD_Fill(UI_X_VAL, y, tftlcd_data.width - 1, y + UI_FS - 1, BLACK);
	LCD_ShowString(UI_X_VAL, y, tftlcd_data.width, tftlcd_data.height, UI_FS, (u8*)text);
}

// 首次绘制：静态标签只画一次
static void SensorUI_DrawStatic(void)
{
	u16 cx;
	u16 top_y;
	u16 bot_y;
	u16 left_x;
	u16 right_x;
	u16 mid_y;

	LCD_Clear(BLACK);
	FRONT_COLOR = WHITE;
	BACK_COLOR = BLACK;
	cx = tftlcd_data.width / 2;
	top_y = UI_Y_IF1;
	bot_y = UI_Y_IF4;
	mid_y = (u16)((top_y + bot_y) / 2);
	left_x = 10;
	right_x = (u16)(cx + 20);

	LCD_ShowString(UI_X, UI_Y_TITLE, tftlcd_data.width, tftlcd_data.height, UI_FS, "=== Sensor Data ===");
	LCD_ShowString(UI_X, UI_Y_COUNT, tftlcd_data.width, tftlcd_data.height, UI_FS, "Count:");
	LCD_ShowString(UI_X, UI_Y_DIST_HDR, tftlcd_data.width, tftlcd_data.height, UI_FS, "Distance Sensors:");
	/* IF1 上、IF2 下、IF3 左、IF4 右 */
	LCD_ShowString((u16)(cx - 56), top_y, tftlcd_data.width, tftlcd_data.height, UI_FS, "IF1:");
	LCD_ShowString((u16)(cx - 56), bot_y, tftlcd_data.width, tftlcd_data.height, UI_FS, "IF2:");
	LCD_ShowString(left_x, mid_y, tftlcd_data.width, tftlcd_data.height, UI_FS, "IF3:");
	LCD_ShowString(right_x, mid_y, tftlcd_data.width, tftlcd_data.height, UI_FS, "IF4:");
	LCD_ShowString(UI_X, UI_Y_CHASSIS, tftlcd_data.width, tftlcd_data.height, UI_FS, "--- Chassis CAN ---");

	sensor_ui_inited = 1;
}

static void SensorUI_UpdateBeepStatus(void)
{
	if(g_beep_dist_enable)
		SensorUI_UpdateLine(UI_Y_BEEP, "Beep:ON  KEY1=OFF");
	else
		SensorUI_UpdateLine(UI_Y_BEEP, "Beep:OFF KEY1=ON ");
}

// 局部更新计数
static void SensorUI_UpdateCount(u32 cnt)
{
	char buf[16];

	sprintf(buf, "%lu", (unsigned long)cnt);
	LCD_Fill(UI_X + 8 * (UI_FS / 2), UI_Y_COUNT, tftlcd_data.width - 1, UI_Y_COUNT + UI_FS - 1, BLACK);
	LCD_ShowString(UI_X + 8 * (UI_FS / 2), UI_Y_COUNT, tftlcd_data.width, tftlcd_data.height, UI_FS, (u8*)buf);
}

// 局部更新四路距离
static void SensorUI_UpdateDistances(DistanceSensor_Data *ds)
{
	u16 cx;
	u16 top_y;
	u16 bot_y;
	u16 left_x;
	u16 right_x;
	u16 mid_y;
	char buf1[16], buf2[16], buf3[16], buf4[16];

	cx = tftlcd_data.width / 2;
	top_y = UI_Y_IF1;
	bot_y = UI_Y_IF4;
	mid_y = (u16)((top_y + bot_y) / 2);
	left_x = 10;
	right_x = (u16)(cx + 20);

	if(ds->valid && ds->error[0] == DS_ERR_NONE) sprintf(buf1, "%u mm", (unsigned int)ds->dist[0]);
	else sprintf(buf1, "---");
	if(ds->valid && ds->error[1] == DS_ERR_NONE) sprintf(buf2, "%u mm", (unsigned int)ds->dist[1]);
	else sprintf(buf2, "---");
	if(ds->valid && ds->error[2] == DS_ERR_NONE) sprintf(buf3, "%u mm", (unsigned int)ds->dist[2]);
	else sprintf(buf3, "---");
	if(ds->valid && ds->error[3] == DS_ERR_NONE) sprintf(buf4, "%u mm", (unsigned int)ds->dist[3]);
	else sprintf(buf4, "---");

	/* 清除并重绘4个方位值 */
	LCD_Fill((u16)(cx - 18), top_y, (u16)(cx + 80), (u16)(top_y + UI_FS - 1), BLACK);
	LCD_ShowString((u16)(cx - 18), top_y, tftlcd_data.width, tftlcd_data.height, UI_FS, (u8*)buf1);

	LCD_Fill((u16)(cx - 18), bot_y, (u16)(cx + 80), (u16)(bot_y + UI_FS - 1), BLACK);
	LCD_ShowString((u16)(cx - 18), bot_y, tftlcd_data.width, tftlcd_data.height, UI_FS, (u8*)buf2);

	LCD_Fill(48, mid_y, (u16)(cx - 20), (u16)(mid_y + UI_FS - 1), BLACK);
	LCD_ShowString(48, mid_y, tftlcd_data.width, tftlcd_data.height, UI_FS, (u8*)buf3);

	LCD_Fill((u16)(right_x + 38), mid_y, (u16)(tftlcd_data.width - 1), (u16)(mid_y + UI_FS - 1), BLACK);
	LCD_ShowString((u16)(right_x + 38), mid_y, tftlcd_data.width, tftlcd_data.height, UI_FS, (u8*)buf4);
}

// 局部更新运动方向与速度（中文，固定宽度）
static void Chassis_UpdateMotionLine(void)
{
	u8 motion_dir;
	s16 motion_speed;
	char buf[48];

	Arbiter_GetMotionInfo(&motion_dir, &motion_speed);

	switch(motion_dir)
	{
		case CHASSIS_MOTION_LEFT:
			sprintf(buf, "\xD4\xCB\xB6\xAF:\xCF\xF2\xD7\xF3 \xCB\xD9\xB6\xC8:%4dmm/s", (int)motion_speed);
			break;
		case CHASSIS_MOTION_RIGHT:
			sprintf(buf, "\xD4\xCB\xB6\xAF:\xCF\xF2\xD3\xD2 \xCB\xD9\xB6\xC8:%4dmm/s", (int)motion_speed);
			break;
		case CHASSIS_MOTION_SPIN:
			sprintf(buf, "\xD4\xCB\xB6\xAF:\xD7\xD4\xD7\xAA \xCB\xD9\xB6\xC8:%4d    ", (int)motion_speed);
			break;
		case CHASSIS_MOTION_FORWARD:
			sprintf(buf, "\xD4\xCB\xB6\xAF:\xC7\xB0\xBD\xF8 \xCB\xD9\xB6\xC8:%4dmm/s", (int)motion_speed);
			break;
		case CHASSIS_MOTION_BACKWARD:
			sprintf(buf, "\xD4\xCB\xB6\xAF:\xBA\xF3\xCD\xCB \xCB\xD9\xB6\xC8:%4dmm/s", (int)motion_speed);
			break;
		default:
			sprintf(buf, "\xD4\xCB\xB6\xAF:\xCD\xA3\xD6\xB9 \xCB\xD9\xB6\xC8:%4d    ", 0);
			break;
	}

	SensorUI_UpdateLineGBK(UI_Y_MOTION, buf);
}

// 局部更新四轮速（固定宽度，已映射到真实车体位置）
static void Chassis_UpdateWheelLines(void)
{
	char buf[48];
	ChassisWheelSpeed_t ws;

	Arbiter_GetWheelSpeedPhysical(&ws);

	sprintf(buf, "\xD7\xF3\xC7\xB0:%4d \xD3\xD2\xC7\xB0:%4d",
		(int)ws.lf, (int)ws.rf);
	SensorUI_UpdateLineGBK(UI_Y_WFL, buf);

	sprintf(buf, "\xD7\xF3\xBA\xF3:%4d \xD3\xD2\xBA\xF3:%4d",
		(int)ws.lr, (int)ws.rr);
	SensorUI_UpdateLineGBK(UI_Y_WRL, buf);
}

// 局部更新底盘 CAN 区
static void Chassis_UpdateOnLCD(void)
{
	char buf[40];
	u16 bms_v;

	Chassis_UpdateMotionLine();
	Chassis_UpdateWheelLines();

	sprintf(buf, "Batt:%2d.%1dV Mode:0x%02X",
		(int)(arb_state.sys_status.battery_voltage / 10),
		(int)(arb_state.sys_status.battery_voltage % 10),
		(unsigned int)arb_state.sys_status.mode_control);
	SensorUI_UpdateLine(UI_Y_BATT, buf);

	if(arb_state.bms_data.soc > 0)
	{
		bms_v = arb_state.bms_data.voltage;
		if(bms_v > 1000)
			bms_v = bms_v / 10;
		sprintf(buf, "BMS SOC:%2d%% V:%2d.%1dV",
			(int)arb_state.bms_data.soc,
			(int)(bms_v / 10),
			(int)(bms_v % 10));
		SensorUI_UpdateLine(UI_Y_BEEP + UI_FS, buf);
	}
}

// 旧版图标界面入口
static void Legacy_UI_Loop(void)
{
	u8 index;

	while(1)
	{
		ICON_UI_Init();
		index = 0xFF;

		while(index == 0xFF)
		{
			index = get_icon_app_table();
			delay_ms(10);
		}

		switch(index)
		{
			case 0:  LED_APP_Test(); break;
			case 1:  RTC_APP_Test(); break;
			case 2:  Calculator_APP_Test(); break;
			case 3:  Gyroscope_APP_Test(); break;
			case 4:  Picture_APP_Test(); break;
			case 5:  Paint_APP_Test(); break;
			case 6:  Ebook_APP_Test(); break;
			case 7:  Notepad_APP_Test(); break;
			case 8:  USB_APP_Test(); break;
			case 9:  Ethernet_APP_Test(); break;
			case 10: Music_APP_Test(); break;
			case 11: Camera_APP_Test(); break;
			case 12: COM_APP_Test(); break;
			case 13: Wireless_APP_Test(); break;
			case 14: Phone_APP_Test(); break;
			case 15: Qrcode_APP_Test(); break;
			default: break;
		}
	}
}

// 自定义界面入口（my_gui 占位）
static void My_UI_Loop(void)
{
	u8 key;

	LCD_Clear(BLACK);
	FRONT_COLOR = WHITE;
	BACK_COLOR = BLACK;
	LCD_ShowString(5, 20, tftlcd_data.width, tftlcd_data.height, 16, "=== MY GUI MODE ===");
	LCD_ShowString(5, 44, tftlcd_data.width, tftlcd_data.height, 16, "Put images in MY_UI/Pictures");
	LCD_ShowString(5, 68, tftlcd_data.width, tftlcd_data.height, 16, "KEY0: Legacy  KEY1: Arbiter");
	printf("[BOOT] UI_MODE=2 (my_gui)\r\n");

	while(1)
	{
		key = KEY_Scan(0);
		if(key == KEY0_PRESS)
		{
			printf("[MY_GUI] KEY0 -> Legacy\r\n");
			Legacy_UI_Loop();
		}
		else if(key == KEY1_PRESS)
		{
			printf("[MY_GUI] KEY1 -> Arbiter\r\n");
			break;
		}
		delay_ms(10);
	}
}

int main()
{	
	Hardware_Check();

#if (UI_TEST_MODE == 0)
	printf("[BOOT] UI_MODE=0 (legacy_gui)\r\n");
	Legacy_UI_Loop();
#elif (UI_TEST_MODE == 1)
	printf("[BOOT] UI_MODE=1 (arbiter_gui)\r\n");
	DistanceSensor_Init();
	CAN1_Init_RangerMini();
	Arbiter_Init();
	Arbiter_EnableCANMode();
	printf("[CAN] Init done, MCR=0x%08X MSR=0x%08X\r\n",
		(unsigned int)CAN1->MCR, (unsigned int)CAN1->MSR);
	printf("[MOTION] Dist ctrl ON, cruise=%d mm/s, beep=%s (KEY1 toggle)\r\n",
		CRUISE_SPEED_MM_S, g_beep_dist_enable ? "ON" : "OFF");
	
	// 确保 LCD 初始化完成
	delay_ms(100);
	
	// 跳过 APP 图标界面，直接显示传感器数据
	LCD_Clear(BLACK);
	FRONT_COLOR = WHITE;
	BACK_COLOR = BLACK;
	
	// 先显示初始内容
	LCD_ShowString(5, 20, tftlcd_data.width, tftlcd_data.height, 16, "=== System Ready ===");
	printf("[LCD] Ready to display sensor data\r\n");
	
	while(1)
	{
		SensorData_ShowScreen();
	}
#elif (UI_TEST_MODE == 2)
	My_UI_Loop();
	DistanceSensor_Init();
	CAN1_Init_RangerMini();
	Arbiter_Init();
	Arbiter_EnableCANMode();
	printf("[CAN] Init done, MCR=0x%08X MSR=0x%08X\r\n",
		(unsigned int)CAN1->MCR, (unsigned int)CAN1->MSR);
	printf("[MOTION] Dist ctrl ON, cruise=%d mm/s, beep=%s (KEY1 toggle)\r\n",
		CRUISE_SPEED_MM_S, g_beep_dist_enable ? "ON" : "OFF");
	LCD_Clear(BLACK);
	FRONT_COLOR = WHITE;
	BACK_COLOR = BLACK;
	LCD_ShowString(5, 20, tftlcd_data.width, tftlcd_data.height, 16, "=== System Ready ===");
	printf("[LCD] Ready to display sensor data\r\n");
	while(1)
	{
		SensorData_ShowScreen();
	}
#else
#error "UI_TEST_MODE must be 0, 1, or 2"
#endif
}

// 传感器数据刷屏显示函数
void SensorData_ShowScreen(void)
{
	DistanceSensor_Data *ds = DistanceSensor_GetData();
	static u32 cnt = 0;
	static u16 can_lcd_div = 0;
	u8 sensor_updated = 0;
	u8 can_updated = 0;
	u8 can_lcd_due = 0;

	Beep_ToggleSwitch();

	while(CAN_MessagePending(CAN1, CAN_FIFO0))
	{
		Arbiter_ProcessCANFeedback();
		can_updated = 1;
	}

	sensor_updated = DistanceSensor_NewData();
	if(sensor_updated && ds->valid)
	{
		if(ds->error[DS_IDX_FRONT] == DS_ERR_NONE) g_front_dist_mm = ds->dist[DS_IDX_FRONT];
		else g_front_dist_mm = 0xFFFF;
		if(ds->error[DS_IDX_BACK] == DS_ERR_NONE) g_back_dist_mm = ds->dist[DS_IDX_BACK];
		else g_back_dist_mm = 0xFFFF;
		if(ds->error[DS_IDX_LEFT] == DS_ERR_NONE) g_left_dist_mm = ds->dist[DS_IDX_LEFT];
		else g_left_dist_mm = 0xFFFF;
		if(ds->error[DS_IDX_RIGHT] == DS_ERR_NONE) g_right_dist_mm = ds->dist[DS_IDX_RIGHT];
		else g_right_dist_mm = 0xFFFF;

		g_nearest_dist_mm = DistanceSensor_GetNearestDistance();
		Beep_UpdateByDistance(g_nearest_dist_mm);
	}

	Motion_ControlByDistance();
	Beep_UpdateByDistance(g_nearest_dist_mm);

	if(can_updated)
	{
		can_lcd_div++;
		if(can_lcd_div >= UI_CAN_LCD_DIV)
		{
			can_lcd_div = 0;
			can_lcd_due = 1;
		}
	}

	if(!sensor_ui_inited)
	{
		SensorUI_DrawStatic();
		SensorUI_UpdateCount(0);
		SensorUI_UpdateDistances(ds);
		SensorUI_UpdateBeepStatus();
		Chassis_UpdateOnLCD();
	}

	if(g_beep_ui_dirty)
	{
		SensorUI_UpdateBeepStatus();
		g_beep_ui_dirty = 0;
	}

	if(sensor_updated)
	{
		cnt++;
		DistanceSensor_Print();
		SensorUI_UpdateCount(cnt);
		SensorUI_UpdateDistances(ds);
	}

	if(can_lcd_due)
	{
		Arbiter_PrintChassisFeedback();
		Chassis_UpdateOnLCD();
	}

	delay_ms(10);
}
