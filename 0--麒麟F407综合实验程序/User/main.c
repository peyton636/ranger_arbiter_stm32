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


//版本号定义
u8 VERSION[]="HARDWARE:V1.0   SOFTWARE:V1.2";

//外部内存测试(最大支持1M字节内存测试)
//x,y:坐标
//fsize:字体大小
//返回值:0,成功;1,失败.
u8 system_exsram_test(u16 x,u16 y,u8 fsize)
{  
	u32 i=0;  	  
	u16 temp=0;	   
	u16 sval=0;	//在地址0读到的数据	  				   
  	LCD_ShowString(x,y,tftlcd_data.width,y+fsize,fsize,"Ex Memory Test:   0KB"); 
	//每隔1K字节,写入一个数据,总共写入1024个数据,刚好是1M字节
	for(i=0;i<1024*1024;i+=1024)
	{
		FSMC_SRAM_WriteBuffer((u8*)&temp,i,2);
		temp++;
	}
	//依次读出之前写入的数据,进行校验		  
 	for(i=0;i<1024*1024;i+=1024) 
	{
  		FSMC_SRAM_ReadBuffer((u8*)&temp,i,2);
		if(i==0)sval=temp;
 		else if(temp<=sval)break;//后面读出的数据一定要比第一次读到的数据大.	   		   
		LCD_ShowxNum(x+15*(fsize/2),y,(u16)(temp-sval+1),4,fsize,0);//显示内存容量  
 	}
	if(i>=1024*1024)
	{
		LCD_ShowxNum(x+15*(fsize/2),y,i/1024,4,fsize,0);//显示内存值  		
		return 0;//内存正常,成功
	}
	return 1;//失败
}

//显示错误信息
//x,y:坐标
//fsize:字体大小
//x,y:坐标.err:错误信息
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

//擦除整个SPI FLASH(即所有资源都删除),以快速更新系统.
//x,y:坐标
//fsize:字体大小
//x,y:坐标.err:错误信息
//返回值:0,没有擦除;1,擦除了
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
			gui_fill_rectangle(x,y+fsize,tftlcd_data.width,fsize,BLACK);//清除显示
			t=0;
			LED1=!LED1;
		}
		key=KEY_Scan(0);
		if(key==KEY0_PRESS)//不擦除,用户取消了
		{ 
			gui_fill_rectangle(x,y,tftlcd_data.width,fsize*2,BLACK);//清除显示
			FRONT_COLOR=WHITE;
			LED1=1;
			return 0;
		}
		if(key==KEY1_PRESS)//要擦除,要重新来过
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

//字库更新确认提示.
//x,y:坐标
//fsize:字体大小 
//返回值:0,不需要更新;1,确认要更新
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
			LCD_Fill(x,y+fsize,tftlcd_data.width,y+fsize+fsize,BLACK);//清除显示
			t=0;
		}
		key=KEY_Scan(0);
		if(key==KEY1_PRESS)break;//不更新 
		if(key==KEY0_PRESS){res=1;break;}//要更新 
		delay_ms(10);
	}
	LCD_Fill(x,y,tftlcd_data.width,y+2*fsize,BLACK);//清除显示
	FRONT_COLOR=WHITE;
	return res;
}


//硬件检测
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
	
REINIT://重新初始化
	SysTick_Init(168);				//延时初始化 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //中断优先级分组 分2组
	USART1_Init(115200);		//初始化串口波特率为115200 
 	LED_Init();					//初始化LED 
 	TFTLCD_Init();					//LCD初始化    
 	BEEP_Init();				//蜂鸣器初始化
 	KEY_Init();					//按键初始化 
	FSMC_SRAM_Init();			//初始化外部SRAM
 	AT24CXX_Init();    			//EEPROM初始化
	EN25QXX_Init();				//初始化W25Q128
 	Lsens_Init();				//初始化光敏传感器
	ADC_Temp_Init();			//初始化内部温度传感器 
	my_mem_init(SRAMIN);		//初始化内部内存池
	my_mem_init(SRAMCCM);		//初始化CCM内存池 
	TP_Init();
	piclib_init();				//piclib初始化
	gui_init();	  				//gui初始化
	FATFS_Init();				//为fatfs相关变量申请内存
	
//	LOGO_Display();//开机LOGO
	LCD_Clear(BLACK);			//黑屏
	FRONT_COLOR=WHITE;
	BACK_COLOR=BLACK;
	j=0;
	//显示版权信息
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
	
	//开始硬件检测初始化
	WM8978_Init();//防止喇叭乱叫
	WM8978_HPvol_Set(0,0);//关闭耳机输出
	WM8978_SPKvol_Set(0);//关闭喇叭输出
	LED1=0;LED2=0;				//同时点亮两个LED
	LCD_ShowString(5,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "CPU:STM32F407ZGT6 168Mhz");
	LCD_ShowString(5,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "FLASH:1024KB  SRAM:192KB");	
	if(system_exsram_test(5,ypos+fsize*j,fsize))
		system_error_show(5,ypos+fsize*j++,"EX Memory Error!",fsize);
	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize,"OK");			 
	my_mem_init(SRAMEX);		//初始化外部内存池,必须放在内存检测之后
	//外部FLASH检测
	if((EN25QXX_ReadID()==0x00)||(EN25QXX_ReadID()==0xFF))//检测不到EN25QXX
	{	 
		system_error_show(5,ypos+fsize*j++,"Ex Flash Error!!",fsize); 
	}else temp=16*1024;	//16M字节大小
	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "Ex Flash:     KB");			   
	LCD_ShowxNum(5+9*(fsize/2),ypos+fsize*j,temp,5,fsize,0);//显示flash大小  
	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");
	//检测是否需要擦除SPI FLASH?
	res=KEY_Scan(1);//
	if(res==KEY_UP_PRESS)
	{
		res=system_files_erase(5,ypos+fsize*j,fsize);
		if(res)goto REINIT; 
	}
    //RTC检测
  	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "RTC Check...");			   
 	if(RTC_Config())system_error_show(5,ypos+fsize*(j+1),"RTC Error!",fsize);//RTC检测
	else 
	{
		RTC_Set_WakeUp(RTC_WakeUpClock_CK_SPRE_16bits,0);//配置WAKE UP中断,1秒钟中断一次
		RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);//得到当前时间
		RTC_GetDate(RTC_Format_BIN,&RTC_DateStruct);//得到当前日期
		LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");			   
	}
	//检查SPI FLASH的文件系统
	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "FATFS Check...");//FATFS检测			   
  	f_mount(fs[0],"0:",1); 		//挂载SD卡  
  	f_mount(fs[1],"1:",1); 		//挂载挂载FLASH. 
 	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");			   
	//SD卡检测
	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "SD Card:     MB");//FATFS检测
	temp=0;	
 	do
	{
		temp++;
 		res=fatfs_getfree("0:",&dtsize,&dfsize);//得到SD卡剩余容量和总容量
		delay_ms(200);		   
	}while(res&&temp<5);//连续检测5次
 	if(res==0)//得到容量正常
	{ 
		gui_phy.memdevflag|=1<<0;	//设置SD卡在位.
		temp=dtsize>>10;//单位转换为MB
		stastr="OK";
 	}else 
	{
 		temp=0;//出错了,单位为0
		stastr="ERROR";
	}
 	LCD_ShowxNum(5+8*(fsize/2),ypos+fsize*j,temp,5,fsize,0);					//显示SD卡容量大小
	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize,stastr);	//SD卡状态			   
	//W25Q128检测,如果不存在文件系统,则先创建.
	temp=0;	
 	do
	{
		temp++;
 		res=fatfs_getfree("1:",&dtsize,&dfsize);//得到FLASH剩余容量和总容量
		delay_ms(200);		   
	}while(res&&temp<20);//连续检测20次		  
	if(res==0X0D)//文件系统不存在
	{
		LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "Flash Disk Formatting...");	//格式化FLASH
		res=f_mkfs("1:",1,4096);//格式化FLASH,1,盘符;1,不需要引导区,8个扇区为1个簇
		if(res==0)
		{
			f_setlabel((const TCHAR *)"1:PRECHIN");	//设置Flash磁盘的名字为：PRECHIN
			LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");//标志格式化成功
 			res=fatfs_getfree("1:",&dtsize,&dfsize);//重新获取容量
		}
	}   
	if(res==0)//得到FLASH卡剩余容量和总容量
	{
		gui_phy.memdevflag|=1<<1;	//设置SPI FLASH在位.
		LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "Flash Disk:     KB");//FATFS检测			   
		temp=dtsize; 	   
 	}else system_error_show(5,ypos+fsize*(j+1),"Flash Fat Error!",fsize);	//flash 文件系统错误 
 	LCD_ShowxNum(5+11*(fsize/2),ypos+fsize*j,temp,5,fsize,0);						//显示FLASH容量大小
	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize,"OK");			//FLASH卡状态	
	//TPAD检测		 
 	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "TPAD Check...");			   
// 	if(Touch_Key_Init(4))system_error_show(5,ypos+fsize*(j+1),"TPAD Error!",fsize);//触摸按键检测
//	else LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK"); 
	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK"); 
	//24C02检测
   	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "24C02 Check...");			   
 	if(AT24CXX_Check())system_error_show(5,ypos+fsize*(j+1),"24C02 Error!",fsize);//24C02检测
	else LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");  
	//MPU6050检测 
   	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "MPU6050 Check...");			   
 	if(MPU6050_Init())system_error_show(5,ypos+fsize*j++,"MPU6050 Error!",fsize);
	else LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");
	//WM8978检测			   
 	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "WM8978 Check...");			   
 	if(WM8978_Init())system_error_show(5,ypos+fsize*(j+1),"WM8978 Error!",fsize);//WM8978检测
	else 
	{
		LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");	
		WM8978_HPvol_Set(0,0);//关闭耳机输出
		WM8978_SPKvol_Set(0);//关闭喇叭输出	
  	}
//	//LAN8720检测   
//	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "LAN8720 Check...");			   
// 	if(LAN8720_Init())system_error_show(5,ypos+fsize*(j+1),"LAN8720 Error!",fsize);//LAN8720检测
//	else LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK"); 
//	LAN8720_RST=0;		//复位LAN8720	
	//字库检测									    
   	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "Font Check...");
	res=KEY_Scan(1);//检测按键
	if(res==KEY1_PRESS)//更新？确认
	{
		res=system_font_update_confirm(5,ypos+fsize*(j+1),fsize);
	}else res=0;
	if(font_init()||(res==1))//检测字体,如果字体不存在/强制更新,则更新字库	
	{
		res=0;//按键无效
 		if(update_font(5,ypos+fsize*j,fsize,"0:")!=0)//从SD卡更新
		{
 			system_error_show(5,ypos+fsize*(j+1),"Font Error!",fsize);	//字体错误
		}
		else 
		{
			ypos=0;
			goto REINIT;
		}
		LCD_Fill(5,ypos+fsize*j,tftlcd_data.width,ypos+fsize*(j+1),BLACK);//填充底色
    	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "Font Check...");			   
 	} 
	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");//字库检测OK
	//触摸屏检测 
	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "Touch Check...");			   
	res=KEY_Scan(1);//检测按键			   
	if(TP_Init()||(res==KEY0_PRESS&&(tp_dev.touchtype&0X80)==0))//有更新/按下了KEY0且不是电容屏,执行校准 	
	{
		if(res==KEY0_PRESS)TP_Adjust();
		res=0;//按键无效
		goto REINIT;				//重新开始初始化
	}
	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");//触摸屏检测OK
   	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "SYSTEM Starting...");  
	//关闭LED
	LED1=1;LED2=1;
	//蜂鸣器短叫,提示正常启动
	BEEP=1;
	delay_ms(100);
	BEEP=0; 
	delay_ms(500);
	TIM4_Init(200,8399);		//定时20ms	
//	TIM3_Init(500,8399);		//定时50ms
	TIM2_Init(1000,8399);		//10Khz计数频率,100ms中断
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

int main()
{	
	u8 index=0;
	
	Hardware_Check();
	ICON_UI_Init();
	while(1)
	{
		index=get_icon_app_table();
		switch(index)
		{
			case 0: rtc_showflag=0;LED_APP_Test();rtc_showflag=1;break;
			case 1: rtc_showflag=0;RTC_APP_Test();rtc_showflag=1;break;
			case 2: rtc_showflag=0;Calculator_APP_Test();rtc_showflag=1;break;
			case 3: rtc_showflag=0;Gyroscope_APP_Test();rtc_showflag=1;break;
			case 4: rtc_showflag=0;Picture_APP_Test();rtc_showflag=1;break;
			case 5: rtc_showflag=0;Paint_APP_Test();rtc_showflag=1;break;
			case 6: rtc_showflag=0;Ebook_APP_Test();rtc_showflag=1;break;
			case 7: rtc_showflag=0;Notepad_APP_Test();rtc_showflag=1;break;
			case 8: rtc_showflag=0;USB_APP_Test();rtc_showflag=1;break;
			case 9: rtc_showflag=0;Ethernet_APP_Test();rtc_showflag=1;break;
			case 10: rtc_showflag=0;Music_APP_Test();rtc_showflag=1;break;
			case 11: rtc_showflag=0;Camera_APP_Test();rtc_showflag=1;break;
			case 12: rtc_showflag=0;COM_APP_Test();rtc_showflag=1;break;
			case 13: rtc_showflag=0;Wireless_APP_Test();rtc_showflag=1;break;
			case 14: rtc_showflag=0;Phone_APP_Test();rtc_showflag=1;break;
			case 15: rtc_showflag=0;Qrcode_APP_Test();rtc_showflag=1;break;
		}
	}
}


