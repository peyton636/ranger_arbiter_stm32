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

u8 *WIRELESS_MSG_BTN_CAPTION_TBL[]={"����","����"};

//��Ϣ��ʾ����
//����ֵ��0������ģʽ��1������ģʽ
u8 Wireless_Window_Message_Box(void)
{
	u8 i=0;
	u8 key=0;
	
	_btn_obj* ctbtn[2];
	
	gui_draw_arcrectangle(WIRELESS_WIN_MSG_BOX_XSTART,WIRELESS_WIN_MSG_BOX_YSTART,
							WIRELESS_WIN_MSG_BOX_WIDTH,WIRELESS_WIN_MSG_BOX_HIGHT,
							5,1,LIGHTGRAY,LIGHTGRAY);
	gui_show_strmid(WIRELESS_WIN_MSG_BOX_XSTART,WIRELESS_WIN_MSG_BOX_YSTART,
					WIRELESS_WIN_MSG_BOX_WIDTH,20,
					BLUE,16,"��ѡ��ģʽ��");
	gui_draw_bline1(WIRELESS_WIN_MSG_BOX_XSTART,WIRELESS_WIN_MSG_BOX_YSTART+20,
					WIRELESS_WIN_MSG_BOX_XSTART+WIRELESS_WIN_MSG_BOX_WIDTH-1,
					WIRELESS_WIN_MSG_BOX_YSTART+20,0,GREEN);
	
	for(i=0;i<2;i++)
	{
		ctbtn[i]=btn_creat(WIRELESS_MSG_BTN_XSTART+(WIRELESS_MSG_BTN_WIDTH+WIRELESS_MSG_BTN_XSPACE)*i,WIRELESS_MSG_BTN_YSTART,WIRELESS_MSG_BTN_WIDTH,WIRELESS_MSG_BTN_HIGTH,0,0x02);//������ť
		if(ctbtn[i]==NULL){}	//û���㹻�ڴ湻����
		else
		{
			ctbtn[i]->caption=WIRELESS_MSG_BTN_CAPTION_TBL[i];//���� 
			ctbtn[i]->font=16;//�����µ������С	 	 
			ctbtn[i]->bcfdcolor=WHITE;	//����ʱ����ɫ
			ctbtn[i]->bcfucolor=WHITE;	//�ɿ�ʱ����ɫ
			btn_draw(ctbtn[i]);			//�ػ���ť
		}
	}
	while(1)
	{
		tp_dev.scan(0);    
		in_obj.get_key(&tp_dev,IN_TYPE_TOUCH);	//�õ�������ֵ   
		delay_ms(5);
		
		for(i=0;i<2;i++)
		{
			key=btn_check(ctbtn[i],&in_obj);
			if(key&&((ctbtn[i]->sta&(1<<7))==0)&&(ctbtn[i]->sta&(1<<6)))//�а����������ɿ�,����TP�ɿ���
			{
				switch(i)
				{
					case 0: return 0;//����ģʽ
					case 1: return 1;//����ģʽ
				}
			}
		}
	}
}

//ģʽѡ��
u8*const wireless_mode_tbl[2]={"����ģʽ","����ģʽ"};

void Wireless_APP_Test(void)
{
	_btn_obj* rbtn=0;				//���ذ�ť�ؼ�
	_btn_obj* clearbtn=0;//�����ť
	u8 rval=0;
	u8 key;
	u8 i=0;
	u8 res=0;
	u8 mode=0;				//0,����ģʽ;1,����ģʽ
	u8 tmp_buf[6];			//buf[0~3]:����ֵ;buf[4]:0,������ͼ;1,����;2,�˳�. 
	u16 x=0,y=0;
	u8 *caption;			//����
	
	FRONT_COLOR=WHITE;  
  	BACK_COLOR=GRAY;  
	LCD_Clear(BACK_COLOR);		//����
	app_filebrower("2.4G���ߴ���Ӧ��",0X05);//��ʾ����
	app_gui_tcbar(0,tftlcd_data.height-gui_phy.tbheight,tftlcd_data.width,gui_phy.tbheight,0x01);	//�Ϸֽ���
	rbtn=btn_creat(tftlcd_data.width-2*gui_phy.tbfsize-8-1,tftlcd_data.height-gui_phy.tbheight,2*gui_phy.tbfsize+8,gui_phy.tbheight-1,0,0x03);//�������ְ�ť
	if(!rbtn)rval=1;			//û���㹻�ڴ湻����
	else
	{																				
		rbtn->caption=(u8*)GUI_BACK_CAPTION_TBL[gui_phy.language];//���� 
		rbtn->font=gui_phy.tbfsize;//�����µ������С	 	 
		rbtn->bcfdcolor=WHITE;	//����ʱ����ɫ
		rbtn->bcfucolor=WHITE;	//�ɿ�ʱ����ɫ
		btn_draw(rbtn);			//�ػ���ť
	}
	
	FRONT_COLOR=RED;
	NRF24L01_Init();    		//��ʼ��NRF24L01
	while(NRF24L01_Check())		//��ⲻ��24L01
	{
		i++;
		if(i%20==0)
			LCD_ShowString(10,10+gui_phy.tbheight,tftlcd_data.width,tftlcd_data.height,16,"NRF24L01 Error!");
		if(i%40==0)
			LCD_ShowString(10,10+gui_phy.tbheight,tftlcd_data.width,tftlcd_data.height,16,"               ");	
	
		tp_dev.scan(0);    
		in_obj.get_key(&tp_dev,IN_TYPE_TOUCH);	//�õ�������ֵ   
		delay_ms(5);
		
		//��ⷵ�ؼ��Ƿ���
		key=btn_check(rbtn,&in_obj);
		if(key&&((rbtn->sta&(1<<7))==0)&&(rbtn->sta&(1<<6)))//�а����������ɿ�,����TP�ɿ���
		{
			btn_delete(rbtn);
			ICON_UI_Init();
			SPI1_Init();    	//��ʼ��SPI	 
			SPI1_SetSpeed(SPI_BaudRatePrescaler_2);//���õ�����ģʽ
			return;
		}
	}
	
	//��ȡģʽ
	mode=Wireless_Window_Message_Box();
	gui_fill_rectangle(0,gui_phy.tbheight,tftlcd_data.width,tftlcd_data.height-2*gui_phy.tbheight,LGRAY);//�����Ļ						

	if(mode==0)NRF24L01_TX_Mode();		//����ģʽ
	else NRF24L01_RX_Mode();  
	caption=(u8*)wireless_mode_tbl[mode];	//����
	app_filebrower(caption,0X05);	 		//��ʾ����
	clearbtn=btn_creat(5,tftlcd_data.height-gui_phy.tbheight,2*gui_phy.tbfsize+8,gui_phy.tbheight-1,0,0x03);//�������ְ�ť
	if(!clearbtn)rval=1;			//û���㹻�ڴ湻����
	else
	{																				
		clearbtn->caption="���";//���� 
		clearbtn->font=gui_phy.tbfsize;//�����µ������С	 	 
		clearbtn->bcfdcolor=WHITE;	//����ʱ����ɫ
		clearbtn->bcfucolor=WHITE;	//�ɿ�ʱ����ɫ
		btn_draw(clearbtn);			//�ػ���ť
	}
	
	while(rval==0)//��ѭ��
	{
		tp_dev.scan(0);    
		in_obj.get_key(&tp_dev,IN_TYPE_TOUCH);	//�õ�������ֵ   
		
		tmp_buf[4]=0X00;//���ԭ��������
		//��ⷵ�ؼ��Ƿ���
		key=btn_check(rbtn,&in_obj);
		if(key&&((rbtn->sta&(1<<7))==0)&&(rbtn->sta&(1<<6)))//�а����������ɿ�,����TP�ɿ���
		{
			tmp_buf[4]|=0X03;	//����3,�˳�
			if(mode==0)			//����ģʽ,��Ҫ�����˳�ָ��
			{
				NRF24L01_TxPacket(tmp_buf);	//����ģʽ,�����˳�ָ��	
			}	
			btn_delete(rbtn);
			btn_delete(clearbtn);
			ICON_UI_Init();
			SPI1_Init();    	//��ʼ��SPI	 
			SPI1_SetSpeed(SPI_BaudRatePrescaler_4);//���õ�����ģʽ
			return;
		}
		
		if(mode==0)//����ģʽ
		{
			res=btn_check(clearbtn,&in_obj);//����ػ���ť
			if(res)//�ػ���ť����Ч����
			{
				if(((clearbtn->sta&0X80)==0))//��ť״̬�ı���
				{ 
					tmp_buf[4]|=0X02;			//����2,����
					NRF24L01_TxPacket(tmp_buf);	//�������ָ��
				}	 
			}
			
			if(tp_dev.sta&TP_PRES_DOWN)			//������������
			{	
				if(tp_dev.y[0]<(tftlcd_data.height-gui_phy.tbheight)&&tp_dev.y[0]>(gui_phy.tbheight+1))	//�ڻ�ͼ������
				{		
					x=tp_dev.x[0];
					y=tp_dev.y[0];
					tmp_buf[0]=tp_dev.x[0]>>8;
					tmp_buf[1]=tp_dev.x[0]&0xFF;
					tmp_buf[2]=tp_dev.y[0]>>8;	 
					tmp_buf[3]=tp_dev.y[0]&0xFF;  
					tmp_buf[4]|=0X01;			//����Ϊ1,������ͼ													        			   
					NRF24L01_TxPacket(tmp_buf);	//��������
//					printf("tp_dev.x[0]=%d   tp_dev.y[0]=%d\r\n",tp_dev.x[0],tp_dev.y[0]);
				}
			}
		}else	//����ģʽ
		{
			if(NRF24L01_RxPacket(tmp_buf)==0)//һ�����յ���Ϣ,����ʾ����.
			{
				x=tmp_buf[0];
				x=(x<<8)+tmp_buf[1];
				y=tmp_buf[2];
				y=(y<<8)+tmp_buf[3];   
			}	  
		}
		if(tmp_buf[4]&0X7F)	//��Ҫ����
		{
//			printf("tmp_buf[4]=%x\r\n",tmp_buf[4]);
			switch(tmp_buf[4]&0X7F)
			{
				case 0x01://��������
					paint_draw_point(x,y,RED,2);			//��ͼ,�뾶Ϊ2 
					break;
				case 0x02://���
					gui_fill_rectangle(0,gui_phy.tbheight,tftlcd_data.width,tftlcd_data.height-2*gui_phy.tbheight,LGRAY);//�����Ļ		
					break;
				case 0x03://�˳�						  
					rval=1;//��־�˳�
					break; 
			} 
		}
	}
	SPI1_Init();    	//��ʼ��SPI	 
	SPI1_SetSpeed(SPI_BaudRatePrescaler_4);//���õ�����ģʽ
	btn_delete(rbtn);
	btn_delete(clearbtn);
	ICON_UI_Init();
}
