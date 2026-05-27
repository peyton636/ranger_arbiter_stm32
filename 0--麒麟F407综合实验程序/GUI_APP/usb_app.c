#include "usb_app.h"
#include "button.h"
#include "touch.h"
#include "common.h"
#include "sdio_sdcard.h"
#include "usbd_msc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usb_conf.h"

//如果移植USB库时报“bool或者TRUE、FLASE”未定义，可以在stm32f10x.h头文件中添加这个类型的定义
//在stm32f10x.h头文件519行可以看到我们添加的以下代码
/**
#ifndef __cplusplus			 //定义布尔类型
typedef enum {FALSE = 0, TRUE = !FALSE} bool;
#endif
*/

USB_OTG_CORE_HANDLE USB_OTG_dev;
extern vu8 USB_STATUS_REG;		//USB状态
extern vu8 bDeviceState;		//USB连接 情况

void USB_APP_Test(void)
{
	_btn_obj* rbtn=0;				//返回按钮控件
	u8 rval=0;
	u8 key; 
	u8 i=0;
	u8 offline_cnt=0;
 	u8 USB_STA=0;
	u8 Divece_STA=0;
	
	FRONT_COLOR=WHITE;  
  	BACK_COLOR=GRAY;  
	LCD_Clear(BACK_COLOR);		//清屏
	app_filebrower("USB读卡器应用",0X05);//显示标题
	app_gui_tcbar(0,tftlcd_data.height-gui_phy.tbheight,tftlcd_data.width,gui_phy.tbheight,0x01);	//上分界线
	rbtn=btn_creat(tftlcd_data.width-2*gui_phy.tbfsize-8-1,tftlcd_data.height-gui_phy.tbheight,2*gui_phy.tbfsize+8,gui_phy.tbheight-1,0,0x03);//创建文字按钮
	if(!rbtn)rval=1;			//没有足够内存够分配
	else
	{																				
		rbtn->caption=(u8*)GUI_BACK_CAPTION_TBL[gui_phy.language];//返回 
		rbtn->font=gui_phy.tbfsize;//设置新的字体大小	 	 
		rbtn->bcfdcolor=WHITE;	//按下时的颜色
		rbtn->bcfucolor=WHITE;	//松开时的颜色
		btn_draw(rbtn);			//重画按钮
	}
	
	FRONT_COLOR=RED;
	if(SD_Init()==0)		//SD卡存在
	{  		
		LCD_ShowString(10,50,tftlcd_data.width,tftlcd_data.height,16,"SD Card Size:     MB"); 
 		LCD_ShowNum(10+13*8,50,SDCardInfo.CardCapacity>>20,5,16);	//显示SD卡容量
	}	   
	
	//USB配置	   
 	LCD_ShowString(10,90,tftlcd_data.width,tftlcd_data.height,16,"USB Connecting...");//提示正在建立连接 	    
	printf("USB Connecting...\r\n");
	USBD_Init(&USB_OTG_dev,USB_OTG_FS_CORE_ID,&USR_desc,&USBD_MSC_cb,&USR_cb);
	delay_ms(1800);
	
	while(rval==0)//主循环
	{
		tp_dev.scan(0);    
		in_obj.get_key(&tp_dev,IN_TYPE_TOUCH);	//得到按键键值   
		delay_ms(5);
		
		if(USB_STA!=USB_STATUS_REG)//状态改变了 
		{	 						   			  	   
			if(USB_STATUS_REG&0x01)//正在写		  
			{
				LCD_ShowString(10,120,tftlcd_data.width,tftlcd_data.height,16,"USB Writing...");//提示USB正在写入数据	 
				printf("USB Writing...\r\n"); 
			}
			if(USB_STATUS_REG&0x02)//正在读
			{
				LCD_ShowString(10,120,tftlcd_data.width,tftlcd_data.height,16,"USB Reading...");//提示USB正在读出数据  		 
				printf("USB Reading...\r\n"); 		 
			}	 										  
			if(USB_STATUS_REG&0x04)
				LCD_ShowString(10,140,tftlcd_data.width,tftlcd_data.height,16,"USB Write Err ");//提示写入错误
			else
				gui_fill_rectangle(10,140,tftlcd_data.width,140+16,BACK_COLOR);//清除显示	    	  
			if(USB_STATUS_REG&0x08)
				LCD_ShowString(10,140,tftlcd_data.width,tftlcd_data.height,16,"USB Read  Err ");//提示读出错误
			else 
				gui_fill_rectangle(10,140,tftlcd_data.width,140+16,BACK_COLOR);//清除显示      
			USB_STA=USB_STATUS_REG;//记录最后的状态
		}
		if(Divece_STA!=bDeviceState) 
		{
			if(bDeviceState==1)
			{
				LCD_ShowString(10,90,tftlcd_data.width,tftlcd_data.height,16,"USB Connected    ");//提示USB连接已经建立	
				printf("USB Connected\r\n");
			}
			else 
			{
				LCD_ShowString(10,90,tftlcd_data.width,tftlcd_data.height,16,"USB DisConnected ");//提示USB被拔出了
				printf("USB DisConnected\r\n");
			}
			Divece_STA=bDeviceState;
		}
		i++;
		if(i==200)
		{
			i=0;
			if(USB_STATUS_REG&0x10)
			{
				offline_cnt=0;//USB连接了,则清除offline计数器
				bDeviceState=1;
			}
			else//没有得到轮询 
			{
				offline_cnt++;  
				if(offline_cnt>10)bDeviceState=0;//2s内没收到在线标记,代表USB被拔出了
			}
			USB_STATUS_REG=0;
		}
		
		//检测返回键是否按下
		key=btn_check(rbtn,&in_obj);
		if(key&&((rbtn->sta&(1<<7))==0)&&(rbtn->sta&(1<<6)))//有按键按下且松开,并且TP松开了
		{
			if(USB_STATUS_REG&0x03)//USB正在读写
			{
				FRONT_COLOR=RED;  					    
				LCD_ShowString(10,110,tftlcd_data.width,tftlcd_data.height,16,"USB BUSY!!!");   
			}
			else 
			{
				gui_fill_rectangle(10,110,tftlcd_data.width,110+16,BACK_COLOR);//清除显示
				break;//USB空闲,则退出USB
			}
		}
	}
	DCD_DevDisconnect(&USB_OTG_dev);
	USB_OTG_StopDevice(&USB_OTG_dev);
	RCC->AHB2RSTR|=1<<7;	//USB OTG FS 复位
	delay_ms(5);
	RCC->AHB2RSTR&=~(1<<7);	//复位结束
	btn_delete(rbtn);
	ICON_UI_Init();
}
