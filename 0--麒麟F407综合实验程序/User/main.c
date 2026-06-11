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
#include "chassis_can_test.h"
#include "usart3.h"
#include "gps.h"
#include "stdio.h"
#include "freertos_app.h"
#include "app_boot.h"

/* 界面分流：0=legacy  1=arbiter(运动控制)  2=my_gui */
#define UI_TEST_MODE             1
#define CHASSIS_CAN_MOTION_TEST  0

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
		res=fatfs_getfree("1:",&dtsize,&dfsize);
		delay_ms(200);
	}while(res&&temp<20);
	if(res != 0)
	{
		printf("[FAT] 1: getfree err=%u, try mkfs...\r\n", (unsigned)res);
		LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "Flash Disk Formatting...");
		/* mkfs 需要 FatFs 工作区；不可 f_mount(NULL) 否则 err=12 FR_NOT_ENABLED */
		f_mount(fs[1],"1:",1);
		res=f_mkfs("1:",1,4096);
		if(res==0)
		{
			f_setlabel((const TCHAR *)"1:PRECHIN");
			f_mount(fs[1],"1:",1);
			res=fatfs_getfree("1:",&dtsize,&dfsize);
			if(res==0)
				LCD_ShowString(5+okoffset,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "OK");
		}
		else
		{
			printf("[FAT] mkfs 1: failed, err=%u\r\n", (unsigned)res);
		}
	}
	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "Flash Disk:     KB");
	if(res==0)
	{
		gui_phy.memdevflag|=1<<1;
		temp=dtsize;
		LCD_ShowxNum(5+11*(fsize/2),ypos+fsize*j,temp,5,fsize,0);
		LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize,"OK");
	}
	else
	{
		temp=0;
		LCD_ShowxNum(5+11*(fsize/2),ypos+fsize*j,temp,5,fsize,0);
#if (UI_TEST_MODE == 1)
		printf("[FAT] 1: unavailable err=%u (arbiter skip, KEY_UP@boot=erase flash)\r\n", (unsigned)res);
		LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize,"SKIP");
#else
		system_error_show(5,ypos+fsize*(j+1),"Flash Fat Error!",fsize);
#endif
	}
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

// 界面分流开关见文件顶部 UI_TEST_MODE / CHASSIS_CAN_MOTION_TEST

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
#if (UI_TEST_MODE == 2)
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
#endif

int main()
{	
	Hardware_Check();

#if (UI_TEST_MODE == 1) || (UI_TEST_MODE == 2)
	printf("[MEM] pool IN=%u%% EX=%u%% CCM=%u%% (mem1=%uKB mem2=%uKB heap=32KB)\r\n",
		my_mem_perused(SRAMIN), my_mem_perused(SRAMEX), my_mem_perused(SRAMCCM),
		(unsigned)(MEM1_MAX_SIZE / 1024u), (unsigned)(MEM2_MAX_SIZE / 1024u));
#endif

#if (UI_TEST_MODE == 0)
	printf("[BOOT] UI_MODE=0 (legacy_gui)\r\n");
	Legacy_UI_Loop();
#elif (UI_TEST_MODE == 1)
	printf("[BOOT] UI_MODE=1 (arbiter_gui)\r\n");
	App_MotionHwInit();
#if CHASSIS_CAN_MOTION_TEST
	ChassisCanTest_RunOnce();
#endif
	App_ShowBootSplash();
	RTOS_AppStart();
#elif (UI_TEST_MODE == 2)
	printf("[BOOT] UI_MODE=2 (my_gui)\r\n");
	My_UI_Loop();
	App_MotionHwInit();
	App_ShowBootSplash();
	RTOS_AppStart();
#else
#error "UI_TEST_MODE must be 0, 1, or 2"
#endif
}
