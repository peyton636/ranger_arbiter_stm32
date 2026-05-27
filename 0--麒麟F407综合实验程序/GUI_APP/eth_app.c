#include "eth_app.h"
#include "button.h"
#include "touch.h"
#include "common.h"
#include "led.h"
#include "key.h"
#include "adc.h"
#include "time.h"
#include "lan8720.h"
#include "lwip/netif.h"
#include "lwip_comm.h"
#include "lwipopts.h"			   
#include "httpd.h"


//加载UI
//mode:
//bit0:0,不加载;1,加载前半部分UI
//bit1:0,不加载;1,加载后半部分UI
void lwip_test_ui(u8 mode)
{
	u8 speed;
	u8 buf[30]; 
	FRONT_COLOR=RED;
	BACK_COLOR=GRAY;  
	if(mode&1<<0)
	{
		LCD_Fill(30,30,tftlcd_data.width,110,BACK_COLOR);	//清除显示
		LCD_ShowString(30,50,200,16,16,"Ethernet lwIP Test");
		LCD_ShowString(30,70,200,16,16,"www.prechin.net"); 	
	}
	if(mode&1<<1)
	{
		LCD_Fill(30,110,tftlcd_data.width,tftlcd_data.height-gui_phy.tbheight,BACK_COLOR);	//清除显示
		LCD_ShowString(30,110,200,16,16,"lwIP Init Successed");
		if(lwipdev.dhcpstatus==2)sprintf((char*)buf,"DHCP IP:%d.%d.%d.%d",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);//打印动态IP地址
		else sprintf((char*)buf,"Static IP:%d.%d.%d.%d",lwipdev.ip[0],lwipdev.ip[1],lwipdev.ip[2],lwipdev.ip[3]);//打印静态IP地址
		LCD_ShowString(30,130,210,16,16,buf); 
		speed=LAN8720_Get_Speed();//得到网速
		if(speed&1<<1)LCD_ShowString(30,150,200,16,16,"Ethernet Speed:100M");
		else LCD_ShowString(30,150,200,16,16,"Ethernet Speed:10M");
	}
}

void Ethernet_APP_Test(void)
{
	_btn_obj* rbtn=0;				//返回按钮控件
	u8 rval=0;
	u8 key; 
	
	ADCx_Init();  			//ADC初始化 
	TIM3_Init(999,839); 	//100khz的频率,计数1000为10ms
	
	FRONT_COLOR=WHITE;  
  	BACK_COLOR=GRAY;  
	LCD_Clear(BACK_COLOR);		//清屏
	app_filebrower("以太网应用",0X05);//显示标题
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
	
	FRONT_COLOR = RED; 		//红色字体
	lwip_test_ui(1);		//加载前半部分UI
	
	//先初始化lwIP(包括LAN8720初始化),此时必须插上网线,否则初始化会失败!! 
	LCD_ShowString(30,110,200,16,16,"lwIP Initing...");
	while(lwip_comm_init()!=0)
	{
		LCD_ShowString(30,110,200,16,16,"lwIP Init failed!");
		LCD_ShowString(30,130,200,16,16,"KEY1: Quit!");
		if(KEY_Scan(1)==KEY1_PRESS)
		{
			TIM_Cmd(TIM3,DISABLE);
			lwip_comm_destroy();
			LAN8720_RST=0;//保持LAN8720复位状态,减少功耗.
			btn_delete(rbtn);
			ICON_UI_Init();
			return;
		}  
	}
	LCD_ShowString(30,110,200,16,16,"lwIP Init Successed");
	//等待DHCP获取 
 	LCD_ShowString(30,130,200,16,16,"DHCP IP configing...");
#if LWIP_DHCP   //使用DHCP
	while((lwipdev.dhcpstatus!=2)&&(lwipdev.dhcpstatus!=0XFF))//等待DHCP获取成功/超时溢出
	{
		lwip_periodic_handle();
	}
#endif
	lwip_test_ui(2);//加载后半部分UI 
	httpd_init();	//HTTP初始化(默认开启websever)
	
	while(rval==0)//主循环
	{
		lwip_periodic_handle();
		
		tp_dev.scan(0);    
		in_obj.get_key(&tp_dev,IN_TYPE_TOUCH);	//得到按键键值   
		delay_ms(5);
		
		//检测返回键是否按下
		key=btn_check(rbtn,&in_obj);
		if(key&&((rbtn->sta&(1<<7))==0)&&(rbtn->sta&(1<<6)))//有按键按下且松开,并且TP松开了
		{
			TIM_Cmd(TIM3,DISABLE);
			lwip_comm_destroy();
			LAN8720_RST=0;//保持LAN8720复位状态,减少功耗.
			btn_delete(rbtn);
			ICON_UI_Init();
			return;
		}
	}
}
