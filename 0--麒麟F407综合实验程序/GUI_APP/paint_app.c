#include "paint_app.h"
#include "button.h"
#include "touch.h"
#include "common.h"



//x,y:����
//color:��ɫ
//mode:
//[7:4]:����
//[3:0]:�ߴ�.(Բ��,���뾶)
void paint_draw_point(u16 x,u16 y,u16 color,u8 mode)
{
	u8 size=mode&0X0F;//�õ��ߴ��С	    
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
	_btn_obj* rbtn=0;				//���ذ�ť�ؼ�
	u8 rval=0;
	u8 key; 
	
	_btn_obj* addbtn=0;
	_btn_obj* minbtn=0;
	u8 i=0;
	u16 pencolor=RED;
	u8 mode=3;					//��ͼģʽ				 
								//[7:4]:����
								//[3:0]:��뾶
	u8 addkey;
	u8 minkey;
	u16 color_buf[]={RED,GREEN,BLUE,YELLOW,BLACK,MAGENTA,BRRED};
	
	_btn_obj* clearbtn=0;
	
start:	
	FRONT_COLOR=WHITE;  
  	BACK_COLOR=GRAY;  
	LCD_Clear(BACK_COLOR);		//����
	app_filebrower("С����Ӧ��",0X05);//��ʾ����
	app_gui_tcbar(0,tftlcd_data.height-gui_phy.tbheight,tftlcd_data.width,gui_phy.tbheight,0x01);	//�Ϸֽ���
	rbtn=btn_creat(tftlcd_data.width-2*gui_phy.tbfsize-8-1,tftlcd_data.height-gui_phy.tbheight,2*gui_phy.tbfsize+8,gui_phy.tbheight-1,0,0x03);//�������ְ�ť
	if(rbtn==NULL)rval=1;	//û���㹻�ڴ湻����
	else
	{																				
		rbtn->caption=(u8*)GUI_BACK_CAPTION_TBL[gui_phy.language];//���� 
		rbtn->font=gui_phy.tbfsize;//�����µ������С	 	 
		rbtn->bcfdcolor=WHITE;	//����ʱ����ɫ
		rbtn->bcfucolor=WHITE;	//�ɿ�ʱ����ɫ
		btn_draw(rbtn);			//�ػ���ť
	}
	//���������
	clearbtn=btn_creat(5,tftlcd_data.height-gui_phy.tbheight,2*gui_phy.tbfsize+8,gui_phy.tbheight-1,0,0x03);//�������ְ�ť
	if(clearbtn==NULL)rval=1;	//û���㹻�ڴ湻����
	else
	{																				
		clearbtn->caption="Clear";
		clearbtn->font=gui_phy.tbfsize;//�����µ������С	 	 
		clearbtn->bcfdcolor=WHITE;	//����ʱ����ɫ
		clearbtn->bcfucolor=WHITE;	//�ɿ�ʱ����ɫ
		btn_draw(clearbtn);			//�ػ���ť
	}
	
	//�������ʴ�С���ð������ӡ�
	addbtn=btn_creat(tftlcd_data.width-2*12-1,tftlcd_data.height/2-45,2*12,2*12,0,0x02);//�������ְ�ť
	if(addbtn==NULL)rval=1;	//û���㹻�ڴ湻����
	else
	{																				
		addbtn->caption="+";//���� 
		addbtn->font=12;//�����µ������С	 	 
		addbtn->bcfdcolor=WHITE;	//����ʱ����ɫ
		addbtn->bcfucolor=WHITE;	//�ɿ�ʱ����ɫ
		btn_draw(addbtn);			//�ػ���ť
	}
	//�������ʴ�С���ð���������
	minbtn=btn_creat(tftlcd_data.width-2*12-1,tftlcd_data.height/2+21,2*12,2*12,0,0x02);//�������ְ�ť
	if(minbtn==NULL)rval=1;	//û���㹻�ڴ湻����
	else
	{																				
		minbtn->caption="-";//���� 
		minbtn->font=12;//�����µ������С	 	 
		minbtn->bcfdcolor=WHITE;	//����ʱ����ɫ
		minbtn->bcfucolor=WHITE;	//�ɿ�ʱ����ɫ
		btn_draw(minbtn);			//�ػ���ť
	}
	for(i=0;i<7;i++)
	{
		paint_draw_point(COLOR_CIRCLE_XSTART+i*(COLOR_CIRCLE_D+COLOR_XSPACE),COLOR_CIRCLE_YSTART,color_buf[i],COLOR_CIRCLE_R);
	}
	paint_draw_point(tftlcd_data.width-3*6-1,tftlcd_data.height/2,pencolor,mode);
	
	while(rval==0)//��ѭ��
	{
		tp_dev.scan(0);    
		in_obj.get_key(&tp_dev,IN_TYPE_TOUCH);	//�õ�������ֵ   
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
		
		//���������Ƿ���
		key=btn_check(clearbtn,&in_obj);
		if(key&&((clearbtn->sta&(1<<7))==0)&&(clearbtn->sta&(1<<6)))//�а����������ɿ�,����TP�ɿ���
		{
			btn_delete(rbtn);
			btn_delete(clearbtn);
			btn_delete(addbtn);
			btn_delete(minbtn);
			goto start;
		}
		
		//��ⷵ�ؼ��Ƿ���
		key=btn_check(rbtn,&in_obj);
		if(key&&((rbtn->sta&(1<<7))==0)&&(rbtn->sta&(1<<6)))//�а����������ɿ�,����TP�ɿ���
		{
			btn_delete(rbtn);
			btn_delete(clearbtn);
			btn_delete(addbtn);
			btn_delete(minbtn);
			
			ICON_UI_Init();
			return;
		}
		
		//��⻭��-�Ƿ���
		minkey=btn_check(minbtn,&in_obj);
		if(minkey&&((minbtn->sta&(1<<7))==0)&&(minbtn->sta&(1<<6)))//�а����������ɿ�,����TP�ɿ���
		{
			paint_draw_point(tftlcd_data.width-3*6-1,tftlcd_data.height/2,LGRAY,mode);
			if(mode>0)
				mode--;
			paint_draw_point(tftlcd_data.width-3*6-1,tftlcd_data.height/2,pencolor,mode);
		}
		//��⻭��+�Ƿ���
		addkey=btn_check(addbtn,&in_obj);
		if(addkey&&((addbtn->sta&(1<<7))==0)&&(addbtn->sta&(1<<6)))//�а����������ɿ�,����TP�ɿ���
		{
			paint_draw_point(tftlcd_data.width-3*6-1,tftlcd_data.height/2,LGRAY,mode);
			if(mode<16)
				mode++;
			paint_draw_point(tftlcd_data.width-3*6-1,tftlcd_data.height/2,pencolor,mode);
		}

		
		if(tp_dev.sta&TP_PRES_DOWN)	
		{
			if(tp_dev.x[0]<(tftlcd_data.width-3*12-1) &&
				tp_dev.y[0]>=(gui_phy.tbheight+mode) && tp_dev.y[0]<tftlcd_data.height-gui_phy.tbheight-mode-COLOR_CIRCLE_D)
			{
				paint_draw_point(tp_dev.x[0],tp_dev.y[0],pencolor,mode);
			}			
		}		
	}
}


