#include "paint_app.h"
#include "button.h"
#include "touch.h"
#include "common.h"



//x,y:坐标
//color:颜色
//mode:
//[7:4]:保留
//[3:0]:尺寸.(圆形,即半径)
void paint_draw_point(u16 x,u16 y,u16 color,u8 mode)
{
	u8 size=mode&0X0F;//得到尺寸大小	    
	if(size==0)gui_phy.draw_point(x,y,color);
	else gui_fill_circle(x,y,size,color);		   
}

#define COLOR_CIRCLE_R		12
#define COLOR_CIRCLE_D		2*COLOR_CIRCLE_R
#define COLOR_CIRCLE_XSTART	15
#define COLOR_CIRCLE_YSTART	tftlcd_data.height-gui_phy.tbheight-15
#define COLOR_XSTART		COLOR_CIRCLE_XSTART-COLOR_CIRCLE_R
#define COLOR_YSTART		COLOR_CIRCLE_YSTART-COLOR_CIRCLE_R
#define COLOR_XSPACE		8	


void Paint_APP_Test(void)
{
	_btn_obj* rbtn=0;				//返回按钮控件
	u8 rval=0;
	u8 key; 
	
	_btn_obj* addbtn=0;
	_btn_obj* minbtn=0;
	u8 i=0;
	u16 pencolor=RED;
	u8 mode=3;					//画图模式				 
								//[7:4]:保留
								//[3:0]:点半径
	u8 addkey;
	u8 minkey;
	u16 color_buf[]={RED,GREEN,BLUE,YELLOW,BLACK,MAGENTA,BRRED};
	
	_btn_obj* clearbtn=0;
	
start:	
	FRONT_COLOR=WHITE;  
  	BACK_COLOR=GRAY;  
	LCD_Clear(BACK_COLOR);		//清屏
	app_filebrower("小画家应用",0X05);//显示标题
	app_gui_tcbar(0,tftlcd_data.height-gui_phy.tbheight,tftlcd_data.width,gui_phy.tbheight,0x01);	//上分界线
	rbtn=btn_creat(tftlcd_data.width-2*gui_phy.tbfsize-8-1,tftlcd_data.height-gui_phy.tbheight,2*gui_phy.tbfsize+8,gui_phy.tbheight-1,0,0x03);//创建文字按钮
	if(rbtn==NULL)rval=1;	//没有足够内存够分配
	else
	{																				
		rbtn->caption=(u8*)GUI_BACK_CAPTION_TBL[gui_phy.language];//返回 
		rbtn->font=gui_phy.tbfsize;//设置新的字体大小	 	 
		rbtn->bcfdcolor=WHITE;	//按下时的颜色
		rbtn->bcfucolor=WHITE;	//松开时的颜色
		btn_draw(rbtn);			//重画按钮
	}
	//增加清除键
	clearbtn=btn_creat(5,tftlcd_data.height-gui_phy.tbheight,2*gui_phy.tbfsize+8,gui_phy.tbheight-1,0,0x03);//创建文字按钮
	if(clearbtn==NULL)rval=1;	//没有足够内存够分配
	else
	{																				
		clearbtn->caption="清除";//返回 
		clearbtn->font=gui_phy.tbfsize;//设置新的字体大小	 	 
		clearbtn->bcfdcolor=WHITE;	//按下时的颜色
		clearbtn->bcfucolor=WHITE;	//松开时的颜色
		btn_draw(clearbtn);			//重画按钮
	}
	
	//创建画笔大小设置按键“加”
	addbtn=btn_creat(tftlcd_data.width-2*12-1,tftlcd_data.height/2-45,2*12,2*12,0,0x02);//创建文字按钮
	if(addbtn==NULL)rval=1;	//没有足够内存够分配
	else
	{																				
		addbtn->caption="+";//返回 
		addbtn->font=12;//设置新的字体大小	 	 
		addbtn->bcfdcolor=WHITE;	//按下时的颜色
		addbtn->bcfucolor=WHITE;	//松开时的颜色
		btn_draw(addbtn);			//重画按钮
	}
	//创建画笔大小设置按键“减”
	minbtn=btn_creat(tftlcd_data.width-2*12-1,tftlcd_data.height/2+21,2*12,2*12,0,0x02);//创建文字按钮
	if(minbtn==NULL)rval=1;	//没有足够内存够分配
	else
	{																				
		minbtn->caption="-";//返回 
		minbtn->font=12;//设置新的字体大小	 	 
		minbtn->bcfdcolor=WHITE;	//按下时的颜色
		minbtn->bcfucolor=WHITE;	//松开时的颜色
		btn_draw(minbtn);			//重画按钮
	}
	for(i=0;i<7;i++)
	{
		paint_draw_point(COLOR_CIRCLE_XSTART+i*(COLOR_CIRCLE_D+COLOR_XSPACE),COLOR_CIRCLE_YSTART,color_buf[i],COLOR_CIRCLE_R);
	}
	paint_draw_point(tftlcd_data.width-3*6-1,tftlcd_data.height/2,pencolor,mode);
	
	while(rval==0)//主循环
	{
		tp_dev.scan(0);    
		in_obj.get_key(&tp_dev,IN_TYPE_TOUCH);	//得到按键键值   
//		delay_ms(5);
		
		if(tp_dev.sta&TP_PRES_DOWN)	
		{
			for(i=0;i<7;i++)
			{
				if(tp_dev.x[0]>(COLOR_XSTART+(COLOR_CIRCLE_D+COLOR_XSPACE)*i) && 
					tp_dev.x[0]<(COLOR_XSTART+COLOR_CIRCLE_D+(COLOR_CIRCLE_D+COLOR_XSPACE)*i) &&
					tp_dev.y[0]>(COLOR_YSTART) && tp_dev.y[0]<(COLOR_YSTART+COLOR_CIRCLE_D))
				{
					pencolor=color_buf[i];
					paint_draw_point(tftlcd_data.width-3*6-1,tftlcd_data.height/2,pencolor,mode);
				}
			}
		}
		
		//检测清除键是否按下
		key=btn_check(clearbtn,&in_obj);
		if(key&&((clearbtn->sta&(1<<7))==0)&&(clearbtn->sta&(1<<6)))//有按键按下且松开,并且TP松开了
		{
			btn_delete(rbtn);
			btn_delete(clearbtn);
			btn_delete(addbtn);
			btn_delete(minbtn);
			goto start;
		}
		
		//检测返回键是否按下
		key=btn_check(rbtn,&in_obj);
		if(key&&((rbtn->sta&(1<<7))==0)&&(rbtn->sta&(1<<6)))//有按键按下且松开,并且TP松开了
		{
			btn_delete(rbtn);
			btn_delete(clearbtn);
			btn_delete(addbtn);
			btn_delete(minbtn);
			
			ICON_UI_Init();
			return;
		}
		
		//检测画笔-是否按下
		minkey=btn_check(minbtn,&in_obj);
		if(minkey&&((minbtn->sta&(1<<7))==0)&&(minbtn->sta&(1<<6)))//有按键按下且松开,并且TP松开了
		{
			paint_draw_point(tftlcd_data.width-3*6-1,tftlcd_data.height/2,LGRAY,mode);
			if(mode>0)
				mode--;
			paint_draw_point(tftlcd_data.width-3*6-1,tftlcd_data.height/2,pencolor,mode);
		}
		//检测画笔+是否按下
		addkey=btn_check(addbtn,&in_obj);
		if(addkey&&((addbtn->sta&(1<<7))==0)&&(addbtn->sta&(1<<6)))//有按键按下且松开,并且TP松开了
		{
			paint_draw_point(tftlcd_data.width-3*6-1,tftlcd_data.height/2,LGRAY,mode);
			if(mode<16)
				mode++;
			paint_draw_point(tftlcd_data.width-3*6-1,tftlcd_data.height/2,pencolor,mode);
		}

		
		if(tp_dev.sta&TP_PRES_DOWN)	
		{
			if(tp_dev.x[0]>=0 && tp_dev.x[0]<(tftlcd_data.width-3*12-1) &&
				tp_dev.y[0]>=(gui_phy.tbheight+mode) && tp_dev.y[0]<tftlcd_data.height-gui_phy.tbheight-mode-COLOR_CIRCLE_D)
			{
				paint_draw_point(tp_dev.x[0],tp_dev.y[0],pencolor,mode);
			}			
		}		
	}
}


