#include "common.h"
#include "tftlcd.h"
#include "SysTick.h"
#include "gui.h"
#include "touch.h"
#include "led.h"

//#include "pic_kj_logo.h"
#include "led_icon.h"
#include "clock_icon.h"
#include "calc_icon.h"
#include "paint_icon.h"
#include "picture_icon.h"
#include "3d_icon.h"
#include "ebook_icon.h"
#include "notepad_icon.h"
#include "earthnet_icon.h"
#include "usb_icon.h"
#include "music_icon.h"
#include "camera_icon.h"
#include "com_icon.h"
#include "qrcode_icon.h"
#include "phone_icon.h"
#include "wireless_icon.h"


vu8 system_task_return;		//任务强制返回标志

u8* asc2_s6030=0;	//数码管字体60*30大字体点阵集
u8* asc2_5427=0;	//普通字体54*27大字体点阵集
u8* asc2_3618=0;	//普通字体36*18大字体点阵集
u8* asc2_2814=0;	//普通字体28*14大字体点阵集


u8*const APP_OK_PIC="0:/SYSTEM/APP/COMMON/ok.bmp";				//确认图标
u8*const APP_CANCEL_PIC="0:/SYSTEM/APP/COMMON/cancel.bmp";		//取消图标
u8*const APP_UNSELECT_PIC="0:/SYSTEM/APP/COMMON/unselect.bmp";	//未选中图标
u8*const APP_SELECT_PIC="0:/SYSTEM/APP/COMMON/select.bmp";		//选中图标
u8*const APP_VOL_PIC="0:/SYSTEM/APP/COMMON/VOL.bmp";			//音量图片路径


//模式选择列表的窗体名字
u8*const APP_MODESEL_CAPTION_TBL[GUI_LANGUAGE_NUM]=
{
"模式选择","模式選擇","Mode Select",
}; 
//提示信息的窗体名字
u8*const APP_REMIND_CAPTION_TBL[GUI_LANGUAGE_NUM]=
{
"提示信息","提示信息","Remind",	 
};
//提醒保存的窗体名字
u8*const APP_SAVE_CAPTION_TBL[GUI_LANGUAGE_NUM]=
{
"是否保存编辑后的文件?","是否保存編輯后的文件?","Do you want to save?",
};
//创建文件失败,提示是否存在SD卡? ,200的宽度
u8*const APP_CREAT_ERR_MSG_TBL[GUI_LANGUAGE_NUM]=
{							 
"创建文件失败,请检查!","創建文件失敗,請檢查!","Creat File Failed,Please Check!",
};


//ICON应用APP图标列表索引
const unsigned char* icon_ui_app_tbl[][2]=
{
	{gImage_led_icon,"LED"},
	{gImage_clock_icon,"时钟"},
	{gImage_calc_icon,"计算器"},
	{gImage_3d_icon,"3D"},
	{gImage_picture_icon,"相册"},
	{gImage_paint_icon,"小画家"},
	{gImage_ebook_icon,"电子书"},
	{gImage_notepad_icon,"记事本"},
	{gImage_usb_icon,"读卡器"},
	{gImage_earthnet_icon,"以太网"},
	{gImage_music_icon,"音乐"},
	{gImage_camera_icon,"照相机"},
	{gImage_com_icon,"通信"},
	{gImage_wireless_icon,"飞书"},
	{gImage_phone_icon,"电话"},
	{gImage_qrcode_icon,"视频"}
};
	

//开机LOGO界面初始化
void LOGO_Display(void)
{
#if defined(TFTLCD_PIXEL_240X320)||defined(TFTLCD_PIXEL_320X480)		
//	LCD_Clear(WHITE);
//	LCD_ShowPicture(tftlcd_data.width/2-PIC_KJ_LOGO_WIDTH/2,tftlcd_data.height/2-PIC_KJ_LOGO_HEIGHT/2,PIC_KJ_LOGO_WIDTH,PIC_KJ_LOGO_HEIGHT,(u8 *)gImage_pic_kj_logo);
//	delay_ms(1000);
#endif	
}

u8 rtc_showflag=0;
//人机界面初始化
void ICON_UI_Init(void)
{
	u8 i=0,j=0;
	
	LCD_Clear(UI_BACKCOLOR);//LIGHTGREEN	
	FRONT_COLOR=UI_FRONTCOLOR;
	BACK_COLOR=UI_BACKCOLOR;
	
	//顶部产品型号、公司名称、时间显示
	gui_show_string("STM32F407",10,2,100,TOP_STATUS_NAME_HEIGHT,TOP_STATUS_NAME_FONT_SIZE,FRONT_COLOR);
	gui_show_strmid(0,0,tftlcd_data.width,TOP_STATUS_NAME_HEIGHT,FRONT_COLOR,TOP_STATUS_NAME_FONT_SIZE,"PRECHIN");
	rtc_showflag=1;
	
	//APP应用图标、名称显示
	for(i=0;i<4;i++)
	{
		for(j=0;j<4;j++)
		{
			LCD_ShowPicture(PIC_ICON_APP_XSTART+(PIC_ICON_APP_XSPACE+PIC_ICON_APP_WIDTH)*j,
							PIC_ICON_APP_YSTART+(PIC_ICON_APP_YSPACE+PIC_ICON_APP_HEIGHT)*i,
							PIC_ICON_APP_WIDTH,PIC_ICON_APP_HEIGHT,
							(u8 *)icon_ui_app_tbl[j+4*i][0]);
			gui_show_strmid(PIC_ICON_APP_XSTART+(PIC_ICON_APP_XSPACE+PIC_ICON_APP_WIDTH)*j,
							PIC_ICON_APP_YSTART+PIC_ICON_APP_HEIGHT+(PIC_ICON_APP_YSPACE+PIC_ICON_APP_HEIGHT)*i,
							PIC_ICON_APP_NAME_WIDTH,PIC_ICON_APP_NAME_HEIGHT,
							FRONT_COLOR,PIC_ICON_APP_NAME_FONT_SIZE,
							(u8 *)icon_ui_app_tbl[j+4*i][1]);
		}
	}
}

//判断哪个应用APP图标被按下，并返回对应索引
//返回值:0~15,被双击的图标编号
//		0xff,没有任何图标被双击或者按下
u8 get_icon_app_table(void)
{
	u8 index=0xff;
	static u16 curxpos=0;//当前tp按下的x坐标
	static u16 curypos=0;//当前tp按下的y坐标
	static u8 curtpsta=0;//触摸按下标志
	u8 i=0,j=0;
	
	tp_dev.scan(0);	//扫描
	if(tp_dev.sta&TP_PRES_DOWN)//有按键被按下
	{
		curxpos=tp_dev.x[0];	//记录当前坐标
		curypos=tp_dev.y[0];	//记录当前坐标
		curtpsta=1;
	}
	else	//按键松开了
	{
		if(curtpsta)//之前有按下
		{
			for(i=0;i<4;i++)
			{
				for(j=0;j<4;j++)
				{
					if(curxpos>(PIC_ICON_APP_XSTART+(PIC_ICON_APP_XSPACE+PIC_ICON_APP_WIDTH)*j)
						&& curxpos<(PIC_ICON_APP_XSTART+(PIC_ICON_APP_XSPACE+PIC_ICON_APP_WIDTH)*j+PIC_ICON_APP_WIDTH)
						&& curypos>(PIC_ICON_APP_YSTART+(PIC_ICON_APP_YSPACE+PIC_ICON_APP_HEIGHT)*i)
						&& curypos<(PIC_ICON_APP_YSTART+(PIC_ICON_APP_YSPACE+PIC_ICON_APP_HEIGHT)*i+PIC_ICON_APP_HEIGHT))
					{
						index=j+4*i;//得到选中的编号
						break;
					}
				}
			}
//			printf("index=%d\r\n",index);
//			printf("curxpos=%d  curypos=%d\r\n",curxpos,curypos);
		}
		curtpsta=0;//清空标志
	}
	return index;
}


//文件浏览横条显示
//topname:浏览的时候要显示的名字	 
//mode:
//[0]:0,不显示上方色条;1,显示上方色条
//[1]:0,不显示下方色条;1,显示下方色条
//[2]:0,不显示名字;1,显示名字
//[3~7]:保留
//返回值:无	 						  
void app_filebrower(u8 *topname,u8 mode)
{		
  	if(mode&0X01)app_gui_tcbar(0,0,tftlcd_data.width,gui_phy.tbheight,0x02);								//下分界线
	if(mode&0X02)app_gui_tcbar(0,tftlcd_data.height-gui_phy.tbheight,tftlcd_data.width,gui_phy.tbheight,0x01);//上分界线
	if(mode&0X04)gui_show_strmid(0,0,tftlcd_data.width,gui_phy.tbheight,WHITE,gui_phy.tbfsize,topname);	  
}

//2色条
//x,y,width,height:坐标及尺寸.
//mode:	设置分界线
//	    [3]:右边分界线
//		[2]:左边分界线
//		[1]:下边分界线
//		[0]:上边分界线
void app_gui_tcbar(u16 x,u16 y,u16 width,u16 height,u8 mode)
{
 	u16 halfheight=height/2;
 	gui_fill_rectangle(x,y,width,halfheight,LIGHTBLUE);  			//填充底部颜色(浅蓝色)	
	gui_fill_rectangle(x,y+halfheight,width,halfheight,GRAYBLUE); 	//填充底部颜色(灰蓝色)
	if(mode&0x01)gui_draw_hline(x,y,width,DARKBLUE);
	if(mode&0x02)gui_draw_hline(x,y+height-1,width,DARKBLUE);
	if(mode&0x04)gui_draw_vline(x,y,height,DARKBLUE);
	if(mode&0x08)gui_draw_vline(x+width-1,y,width,DARKBLUE);
} 


//获得当前条目的图标路径
//mode:0,单选模式;1,多选模式
//selpath,unselpath:选中和非选中的图标路径
//selx:选中图标编号(单选模式)/有效图标掩码(多选模式)
//index:当前条目编号
u8 * app_get_icopath(u8 mode,u8 *selpath,u8 *unselpath,u8 selx,u8 index)
{
	u8 *icopath=0;
	if(mode)//多选模式
	{
		if(selx&(1<<index))icopath=selpath;	//是选中的条目
		else icopath=unselpath;			  	//是非选中的条目
	}else
	{
		if(selx==index)icopath=selpath;		//是选中的条目
		else icopath=unselpath;			  	//是非选中的条目
	}
	return icopath;
}

//显示条目
//x,y,itemwidth,itemheight:条目坐标及尺寸
//name:条目名字
//icopath:图标路径
void app_show_items(u16 x,u16 y,u16 itemwidth,u16 itemheight,u8*name,u8*icopath,u16 color,u16 bkcolor)
{
  	gui_fill_rectangle(x,y,itemwidth,itemheight,bkcolor);	//填充背景色
	gui_show_ptstr(x+5,y+(itemheight-16)/2,x+itemwidth-10-APP_ITEM_ICO_SIZE-5,y+itemheight,0,color,16,name,1);	//显示条目名字
	if(icopath)minibmp_decode(icopath,x+itemwidth-10-APP_ITEM_ICO_SIZE,y+(itemheight-APP_ITEM_ICO_SIZE)/2,APP_ITEM_ICO_SIZE,APP_ITEM_ICO_SIZE,0,0);			//解码APP_ITEM_ICO_SIZE*APP_ITEM_ICO_SIZE的bmp图片
}	

//画一条平滑过渡的彩色线(或矩形)
//以中间为间隔,两边展开
//x,y,width,height:线条的坐标尺寸
//sergb,mrgb:起止颜色和中间颜色
void app_draw_smooth_line(u16 x,u16 y,u16 width,u16 height,u32 sergb,u32 mrgb)
{	  
	gui_draw_smooth_rectangle(x,y,width/2,height,sergb,mrgb);	   		//前半段渐变
	gui_draw_smooth_rectangle(x+width/2,y,width/2,height,mrgb,sergb);   //后半段渐变
}

//判断触摸屏当前值是不是在某个区域内
//tp:触摸屏
//x,y,width,height:坐标和尺寸 
//返回值:0,不在区域内;1,在区域内.
u8 app_tp_is_in_area(_m_tp_dev *tp,u16 x,u16 y,u16 width,u16 height)
{						 	 
	if(tp->x[0]<=(x+width)&&tp->x[0]>=x&&tp->y[0]<=(y+height)&&tp->y[0]>=y)return 1;
	else return 0;							 	
}

//条目选择
//x,y,width,height:坐标尺寸(width最小为150,height最小为72)
//items[]:条目名字集
//itemsize:总条目数(最大不超过8个)
//selx:结果.多选模式时,对应各项的选择情况.单选模式时,对应选择的条目.
//mode:
//[7]:0,无OK按钮;1,有OK按钮
//[6]:0,不读取背景色;1,读取背景色
//[5]:0,单选模式;1,多选模式
//[4]:0,不加载图表;1,加载图标
//[3:0]:保留
//caption:窗口名字	  
//返回值:0,ok;其他,取消或者错误.
u8 app_items_sel(u16 x,u16 y,u16 width,u16 height,u8 *items[],u8 itemsize,u8 *selx,u8 mode,u8*caption) 
{
	u8 rval=0,res;
	u8 selsta=0;	//选中状态为0,
					//[7]:标记是否已经记录第一次按下的条目;
					//[6:4]:保留
	                //[3:0]:第一次按下的条目
	u16 i;

	u8 temp;
	u16 itemheight=0;		//每个条目的高度
	u16 itemwidth=0;		//每个条目的宽度
	u8* unselpath=0;		//未选中的图标的路径
	u8* selpath=0;			//选中图标的路径
	u8* icopath=0;

 	_window_obj* twin=0;	//窗体
 	_btn_obj * okbtn=0;		//确定按钮
 	_btn_obj * cancelbtn=0; //取消按钮

	if(itemsize>8||itemsize<1)return 0xff;	//条目数错误
	if(width<150||height<72)return 0xff; 	//尺寸错误
	
	itemheight=(height-72)/itemsize-1;	//得到每个条目的高度
	itemwidth=width-10;					//每个条目的宽度

 	twin=window_creat(x,y,width,height,0,1|(1<<5)|((1<<6)&mode),16);//创建窗口
	if(twin==NULL)
	{
		return 0XFE;
		//spb_delete();//释放SPB占用的内存
		//twin=window_creat(x,y,width,height,0,1|(1<<5)|((1<<6)&mode),16);//重新创建窗口
 	}
  	if(mode&(1<<7))
	{
   		temp=(width-APP_ITEM_BTN1_WIDTH*2)/3;
		okbtn=btn_creat(x+temp,y+height-APP_ITEM_BTN_HEIGHT-5,APP_ITEM_BTN1_WIDTH,APP_ITEM_BTN_HEIGHT,0,0x02);							//创建OK按钮
		cancelbtn=btn_creat(x+APP_ITEM_BTN1_WIDTH+temp*2,y+height-APP_ITEM_BTN_HEIGHT-5,APP_ITEM_BTN1_WIDTH,APP_ITEM_BTN_HEIGHT,0,0x02);//创建CANCEL按钮
		if(twin==NULL||okbtn==NULL||cancelbtn==NULL)rval=1;
		else
		{
	 		okbtn->caption=(u8*)GUI_OK_CAPTION_TBL[gui_phy.language];//确认
			okbtn->bkctbl[0]=0X8452;//边框颜色
			okbtn->bkctbl[1]=0XAD97;//第一行的颜色				
			okbtn->bkctbl[2]=0XAD97;//上半部分颜色
			okbtn->bkctbl[3]=0X8452;//下半部分颜色
		}
	}else 
	{
   		temp=(width-APP_ITEM_BTN2_WIDTH)/2;
		cancelbtn=btn_creat(x+temp,y+height-APP_ITEM_BTN_HEIGHT-5,APP_ITEM_BTN2_WIDTH,APP_ITEM_BTN_HEIGHT,0,0x02);	//创建CANCEL按钮
		if(twin==NULL||cancelbtn==NULL)rval=1;
	}
 	if(rval==0)//之前的操作正常
	{
 		twin->caption=caption;
		twin->windowbkc=APP_WIN_BACK_COLOR;	     
 		cancelbtn->caption=(u8*)GUI_CANCEL_CAPTION_TBL[gui_phy.language];//取消
 		cancelbtn->bkctbl[0]=0X8452;//边框颜色
		cancelbtn->bkctbl[1]=0XAD97;//第一行的颜色				
		cancelbtn->bkctbl[2]=0XAD97;//上半部分颜色
		cancelbtn->bkctbl[3]=0X8452;//下半部分颜色

		if(mode&(1<<4))//需要加载图标
		{
  			if(mode&(1<<5))//多选模式
			{
				unselpath=(u8*)APP_CANCEL_PIC;		//未选中的图标的路径
				selpath=(u8*)APP_OK_PIC;			//选中图标的路径
			}else		   //单选模式
			{
				unselpath=(u8*)APP_UNSELECT_PIC;	//未选中的图标的路径
				selpath=(u8*)APP_SELECT_PIC;		//选中图标的路径
			}
		}
		window_draw(twin);				//画出窗体
		btn_draw(cancelbtn);			//画按钮
	    if(mode&(1<<7))btn_draw(okbtn);	//画按钮
		for(i=0;i<itemsize;i++)
		{
			icopath=app_get_icopath(mode&(1<<5),selpath,unselpath,*selx,i); //得到图标路径
			app_show_items(x+5,y+32+i*(itemheight+1),itemwidth,itemheight,items[i],icopath,BLACK,twin->windowbkc);//显示所有的条目
			if((i+1)!=itemsize)app_draw_smooth_line(x+5,y+32+(i+1)*(itemheight+1)-1,itemwidth,1,0Xb1ffc4,0X1600b1);//画彩线
 		}
		while(rval==0)
		{
			tp_dev.scan(0);    
			in_obj.get_key(&tp_dev,IN_TYPE_TOUCH);	//得到按键键值   
			delay_ms(5);		//延时一个时钟节拍
			if(system_task_return){rval=1;break;};	//TPAD返回	
			if(mode&(1<<7))
			{
				res=btn_check(okbtn,&in_obj);		//确认按钮检测
				if(res)
				{
					if((okbtn->sta&0X80)==0)//有有效操作
					{
						rval=0XFF;
						break;//确认按钮
					}
				}
			}   
			res=btn_check(cancelbtn,&in_obj);		//取消按钮检测
			if(res)
			{
				if((cancelbtn->sta&0X80)==0)//有有效操作
				{
					rval=1;
					break;//取消按钮	 
				}
			}
			temp=0XFF;//标记量,如果为0XFF,在松开的时候,说明是不在有效区域内的.如果非0XFF,则表示TP松开的时候,是在有效区域内.
			for(i=0;i<itemsize;i++)
			{
				if(tp_dev.sta&TP_PRES_DOWN)//触摸屏被按下
				{
				 	if(app_tp_is_in_area(&tp_dev,x+5,y+32+i*(itemheight+1),itemwidth,itemheight))//判断某个时刻,触摸屏的值是不是在某个区域内
					{ 
						if((selsta&0X80)==0)//还没有按下过
						{
							icopath=app_get_icopath(mode&(1<<5),selpath,unselpath,*selx,i); //得到图标路径
							app_show_items(x+5,y+32+i*(itemheight+1),itemwidth,itemheight,items[i],icopath,BLACK,APP_ITEM_SEL_BKCOLOR);//反选条目
							selsta=i;		//记录第一次按下的条目
							selsta|=0X80;	//标记已经按下过了
						}
						break;		
					}
				}else //触摸屏被松开了
				{
				 	if(app_tp_is_in_area(&tp_dev,x+5,y+32+i*(itemheight+1),itemwidth,itemheight))//判断某个时刻,触摸屏的值是不是在某个区域内
					{ 
						temp=i;	   
						break;
					}
				}
			}
			if((selsta&0X80)&&(tp_dev.sta&TP_PRES_DOWN)==0)//有按下过,且按键松开了
			{
				if((selsta&0X0F)==temp)//松开之前的坐标也是在按下时的区域内.
				{
					if(mode&(1<<5))//多选模式,执行取反操作
					{
						if((*selx)&(1<<temp))*selx&=~(1<<temp);
						else *selx|=1<<temp;
					}else//单选模式
					{																					  
						app_show_items(x+5,y+32+(*selx)*(itemheight+1),itemwidth,itemheight,items[*selx],unselpath,BLACK,twin->windowbkc);//取消之前选择的条目
						*selx=temp;
					}
				}else temp=selsta&0X0F;//得到当时按下的条目号
 				icopath=app_get_icopath(mode&(1<<5),selpath,unselpath,*selx,temp); //得到图标路径
				app_show_items(x+5,y+32+temp*(itemheight+1),itemwidth,itemheight,items[temp],icopath,BLACK,twin->windowbkc);//反选条目
				selsta=0;//取消
			}
 		}
 	}
	window_delete(twin);
	btn_delete(okbtn);
	btn_delete(cancelbtn);
	system_task_return=0;
	if(rval==0XFF)return 0;
	return rval;
} 

////////////////////////////////伪随机数产生办法////////////////////////////////
u32 random_seed=1;
void app_srand(u32 seed)
{
	random_seed=seed;
}
//获取伪随机数
//可以产生0~RANDOM_MAX-1的随机数
//seed:种子
//max:最大值	  		  
//返回值:0~(max-1)中的一个值 		
u32 app_get_rand(u32 max)
{			    	    
	random_seed=random_seed*22695477+1;
	return (random_seed)%max; 
}

//读取背景色
//x,y,width,height:背景色读取范围
//ctbl:背景色存放指针
void app_read_bkcolor(u16 x,u16 y,u16 width,u16 height,u16 *ctbl)
{
	u32	x0,y0,ccnt;
	ccnt=0;
	for(y0=y;y0<y+height;y0++)
	{
		for(x0=x;x0<x+width;x0++)
		{
			ctbl[ccnt]=gui_phy.read_point(x0,y0);//读取颜色
			ccnt++;
		}
	}
}  
//恢复背景色
//x,y,width,height:背景色还原范围
//ctbl:背景色存放指针
void app_recover_bkcolor(u16 x,u16 y,u16 width,u16 height,u16 *ctbl)
{
	u32 x0,y0,ccnt;
	ccnt=0;
	for(y0=y;y0<y+height;y0++)
	{
		for(x0=x;x0<x+width;x0++)
		{
			gui_phy.draw_point(x0,y0,ctbl[ccnt]);//读取颜色
			ccnt++;
		}
	}
}

