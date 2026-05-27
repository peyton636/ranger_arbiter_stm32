#include "wireless_app.h"
#include "button.h"
#include "touch.h"
#include "common.h"
#include "spi.h"
#include "nrf24l01.h"
#include "paint_app.h"


#define WIRELESS_WIN_MSG_BOX_WIDTH	150
#define WIRELESS_WIN_MSG_BOX_HIGHT	65
#define WIRELESS_WIN_MSG_BOX_XSTART	tftlcd_data.width/2-WIRELESS_WIN_MSG_BOX_WIDTH/2
#define WIRELESS_WIN_MSG_BOX_YSTART	tftlcd_data.height/2-WIRELESS_WIN_MSG_BOX_HIGHT/2

#define WIRELESS_MSG_BTN_XSTART		WIRELESS_WIN_MSG_BOX_XSTART+20
#define WIRELESS_MSG_BTN_YSTART		WIRELESS_WIN_MSG_BOX_YSTART+30
#define WIRELESS_MSG_BTN_XSPACE		30
#define WIRELESS_MSG_BTN_WIDTH		40
#define WIRELESS_MSG_BTN_HIGTH		30

u8 *WIRELESS_MSG_BTN_CAPTION_TBL[]={"发送","接收"};

//消息提示窗口
//返回值：0：发送模式，1：接收模式
u8 Wireless_Window_Message_Box(void)
{
	u8 i=0;
	u8 res=0;
	u8 key=0;
	
	_btn_obj* ctbtn[2];
	
	gui_draw_arcrectangle(WIRELESS_WIN_MSG_BOX_XSTART,WIRELESS_WIN_MSG_BOX_YSTART,
							WIRELESS_WIN_MSG_BOX_WIDTH,WIRELESS_WIN_MSG_BOX_HIGHT,
							5,1,LIGHTGRAY,LIGHTGRAY);
	gui_show_strmid(WIRELESS_WIN_MSG_BOX_XSTART,WIRELESS_WIN_MSG_BOX_YSTART,
					WIRELESS_WIN_MSG_BOX_WIDTH,20,
					BLUE,16,"请选择模式？");
	gui_draw_bline1(WIRELESS_WIN_MSG_BOX_XSTART,WIRELESS_WIN_MSG_BOX_YSTART+20,
					WIRELESS_WIN_MSG_BOX_XSTART+WIRELESS_WIN_MSG_BOX_WIDTH-1,
					WIRELESS_WIN_MSG_BOX_YSTART+20,0,GREEN);
	
	for(i=0;i<2;i++)
	{
		ctbtn[i]=btn_creat(WIRELESS_MSG_BTN_XSTART+(WIRELESS_MSG_BTN_WIDTH+WIRELESS_MSG_BTN_XSPACE)*i,WIRELESS_MSG_BTN_YSTART,WIRELESS_MSG_BTN_WIDTH,WIRELESS_MSG_BTN_HIGTH,0,0x02);//创建按钮
		if(ctbtn[i]==NULL)res=1;	//没有足够内存够分配
		else
		{
			ctbtn[i]->caption=WIRELESS_MSG_BTN_CAPTION_TBL[i];//返回 
			ctbtn[i]->font=16;//设置新的字体大小	 	 
			ctbtn[i]->bcfdcolor=WHITE;	//按下时的颜色
			ctbtn[i]->bcfucolor=WHITE;	//松开时的颜色
			btn_draw(ctbtn[i]);			//重画按钮
		}
	}
	while(1)
	{
		tp_dev.scan(0);    
		in_obj.get_key(&tp_dev,IN_TYPE_TOUCH);	//得到按键键值   
		delay_ms(5);
		
		for(i=0;i<2;i++)
		{
			key=btn_check(ctbtn[i],&in_obj);
			if(key&&((ctbtn[i]->sta&(1<<7))==0)&&(ctbtn[i]->sta&(1<<6)))//有按键按下且松开,并且TP松开了
			{
				switch(i)
				{
					case 0: return 0;//发送模式
					case 1: return 1;//接收模式
				}
			}
		}
	}
}

//模式选择
u8*const wireless_mode_tbl[2]={"发送模式","接收模式"};

void Wireless_APP_Test(void)
{
	_btn_obj* rbtn=0;				//返回按钮控件
	_btn_obj* clearbtn=0;//清除按钮
	u8 rval=0;
	u8 key;
	u8 i=0;
	u8 res=0;
	u8 mode=0;				//0,发送模式;1,接收模式
	u8 tmp_buf[6];			//buf[0~3]:坐标值;buf[4]:0,正常画图;1,清屏;2,退出. 
	u16 x=0,y=0;
	u8 *caption;			//标题
	
	FRONT_COLOR=WHITE;  
  	BACK_COLOR=GRAY;  
	LCD_Clear(BACK_COLOR);		//清屏
	app_filebrower("2.4G无线传输应用",0X05);//显示标题
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
	NRF24L01_Init();    		//初始化NRF24L01
	while(NRF24L01_Check())		//检测不到24L01
	{
		i++;
		if(i%20==0)
			LCD_ShowString(10,10+gui_phy.tbheight,tftlcd_data.width,tftlcd_data.height,16,"NRF24L01 Error!");
		if(i%40==0)
			LCD_ShowString(10,10+gui_phy.tbheight,tftlcd_data.width,tftlcd_data.height,16,"               ");	
	
		tp_dev.scan(0);    
		in_obj.get_key(&tp_dev,IN_TYPE_TOUCH);	//得到按键键值   
		delay_ms(5);
		
		//检测返回键是否按下
		key=btn_check(rbtn,&in_obj);
		if(key&&((rbtn->sta&(1<<7))==0)&&(rbtn->sta&(1<<6)))//有按键按下且松开,并且TP松开了
		{
			btn_delete(rbtn);
			ICON_UI_Init();
			SPI1_Init();    	//初始化SPI	 
			SPI1_SetSpeed(SPI_BaudRatePrescaler_2);//设置到高速模式
			return;
		}
	}
	
	//获取模式
	mode=Wireless_Window_Message_Box();
	gui_fill_rectangle(0,gui_phy.tbheight,tftlcd_data.width,tftlcd_data.height-2*gui_phy.tbheight,LGRAY);//清除屏幕						

	if(mode==0)NRF24L01_TX_Mode();		//设置模式
	else NRF24L01_RX_Mode();  
	caption=(u8*)wireless_mode_tbl[mode];	//标题
	app_filebrower(caption,0X05);	 		//显示标题
	clearbtn=btn_creat(5,tftlcd_data.height-gui_phy.tbheight,2*gui_phy.tbfsize+8,gui_phy.tbheight-1,0,0x03);//创建文字按钮
	if(!clearbtn)rval=1;			//没有足够内存够分配
	else
	{																				
		clearbtn->caption="清除";//返回 
		clearbtn->font=gui_phy.tbfsize;//设置新的字体大小	 	 
		clearbtn->bcfdcolor=WHITE;	//按下时的颜色
		clearbtn->bcfucolor=WHITE;	//松开时的颜色
		btn_draw(clearbtn);			//重画按钮
	}
	
	while(rval==0)//主循环
	{
		tp_dev.scan(0);    
		in_obj.get_key(&tp_dev,IN_TYPE_TOUCH);	//得到按键键值   
		
		tmp_buf[4]=0X00;//清除原来的设置
		//检测返回键是否按下
		key=btn_check(rbtn,&in_obj);
		if(key&&((rbtn->sta&(1<<7))==0)&&(rbtn->sta&(1<<6)))//有按键按下且松开,并且TP松开了
		{
			tmp_buf[4]|=0X03;	//功能3,退出
			if(mode==0)			//发送模式,需要发送退出指令
			{
				NRF24L01_TxPacket(tmp_buf);	//发送模式,则发送退出指令	
			}	
			btn_delete(rbtn);
			btn_delete(clearbtn);
			ICON_UI_Init();
			SPI1_Init();    	//初始化SPI	 
			SPI1_SetSpeed(SPI_BaudRatePrescaler_4);//设置到高速模式
			return;
		}
		
		if(mode==0)//发送模式
		{
			res=btn_check(clearbtn,&in_obj);//检查重画按钮
			if(res)//重画按钮有有效操作
			{
				if(((clearbtn->sta&0X80)==0))//按钮状态改变了
				{ 
					tmp_buf[4]|=0X02;			//功能2,清屏
					NRF24L01_TxPacket(tmp_buf);	//发送清除指令
				}	 
			}
			
			if(tp_dev.sta&TP_PRES_DOWN)			//触摸屏被按下
			{	
				if(tp_dev.y[0]<(tftlcd_data.height-gui_phy.tbheight)&&tp_dev.y[0]>(gui_phy.tbheight+1))	//在画图区域内
				{		
					x=tp_dev.x[0];
					y=tp_dev.y[0];
					tmp_buf[0]=tp_dev.x[0]>>8;
					tmp_buf[1]=tp_dev.x[0]&0xFF;
					tmp_buf[2]=tp_dev.y[0]>>8;	 
					tmp_buf[3]=tp_dev.y[0]&0xFF;  
					tmp_buf[4]|=0X01;			//功能为1,正常画图													        			   
					NRF24L01_TxPacket(tmp_buf);	//发送数据
//					printf("tp_dev.x[0]=%d   tp_dev.y[0]=%d\r\n",tp_dev.x[0],tp_dev.y[0]);
				}
			}
		}else	//接收模式
		{
			if(NRF24L01_RxPacket(tmp_buf)==0)//一旦接收到信息,则显示出来.
			{
				x=tmp_buf[0];
				x=(x<<8)+tmp_buf[1];
				y=tmp_buf[2];
				y=(y<<8)+tmp_buf[3];   
			}	  
		}
		if(tmp_buf[4]&0X7F)	//需要处理
		{
//			printf("tmp_buf[4]=%x\r\n",tmp_buf[4]);
			switch(tmp_buf[4]&0X7F)
			{
				case 0x01://正常画点
					paint_draw_point(x,y,RED,2);			//画图,半径为2 
					break;
				case 0x02://清除
					gui_fill_rectangle(0,gui_phy.tbheight,tftlcd_data.width,tftlcd_data.height-2*gui_phy.tbheight,LGRAY);//清除屏幕		
					break;
				case 0x03://退出						  
					rval=1;//标志退出
					break; 
			} 
		}
	}
	SPI1_Init();    	//初始化SPI	 
	SPI1_SetSpeed(SPI_BaudRatePrescaler_4);//设置到高速模式
	btn_delete(rbtn);
	btn_delete(clearbtn);
	ICON_UI_Init();
}
