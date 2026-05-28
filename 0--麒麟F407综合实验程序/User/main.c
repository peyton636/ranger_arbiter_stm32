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


//�汾�Ŷ���
u8 VERSION[]="HARDWARE:V1.0   SOFTWARE:V1.2";

//�ⲿ�ڴ����(���֧��1M�ֽڴ����)
//x,y:����
//fsize:�����С
//����ֵ:0,�ɹ�;1,ʧ��.
u8 system_exsram_test(u16 x,u16 y,u8 fsize)
{  
	u32 i=0;  	  
	u16 temp=0;	   
	u16 sval=0;	//�ڵ�ַ0����������	  				   
  	LCD_ShowString(x,y,tftlcd_data.width,y+fsize,fsize,"Ex Memory Test:   0KB"); 
	//ÿ��1K�ֽ�,д��һ������,�ܹ�д��1024������,�պ���1M�ֽ�
	for(i=0;i<1024*1024;i+=1024)
	{
		FSMC_SRAM_WriteBuffer((u8*)&temp,i,2);
		temp++;
	}
	//���ζ���֮ǰд������,����У��		  
 	for(i=0;i<1024*1024;i+=1024) 
	{
  		FSMC_SRAM_ReadBuffer((u8*)&temp,i,2);
		if(i==0)sval=temp;
 		else if(temp<=sval)break;//������������һ��Ҫȡ����һ�ζ��������ݴ�.	   		   
		LCD_ShowxNum(x+15*(fsize/2),y,(u16)(temp-sval+1),4,fsize,0);//��ʾ�ڴ�����  
 	}
	if(i>=1024*1024)
	{
		LCD_ShowxNum(x+15*(fsize/2),y,i/1024,4,fsize,0);//��ʾ�ڴ�ֵ  		
		return 0;//�ڴ�����,�ɹ�
	}
	return 1;//ʧ��
}

//��ʾ������Ϣ
//x,y:����
//fsize:�����С
//x,y:����.err:������Ϣ
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

//��������SPI FLASH(��������Դ��ɾ��),�Կ��ٸ���ϵͳ.
//x,y:����
//fsize:�����С
//x,y:����.err:������Ϣ
//����ֵ:0,û�в���;1,������
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
			gui_fill_rectangle(x,y+fsize,tftlcd_data.width,fsize,BLACK);//�����ʾ
			t=0;
			LED1=!LED1;
		}
		key=KEY_Scan(0);
		if(key==KEY0_PRESS)//������,�û�ȡ����
		{ 
			gui_fill_rectangle(x,y,tftlcd_data.width,fsize*2,BLACK);//�����ʾ
			FRONT_COLOR=WHITE;
			LED1=1;
			return 0;
		}
		if(key==KEY1_PRESS)//Ҫ����,Ҫ��������
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

//�ֿ����ȷ����ʾ.
//x,y:����
//fsize:�����С 
//����ֵ:0,����Ҫ����;1,ȷ��Ҫ����
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
			LCD_Fill(x,y+fsize,tftlcd_data.width,y+fsize+fsize,BLACK);//�����ʾ
			t=0;
		}
		key=KEY_Scan(0);
		if(key==KEY1_PRESS)break;//������ 
		if(key==KEY0_PRESS){res=1;break;}//Ҫ���� 
		delay_ms(10);
	}
	LCD_Fill(x,y,tftlcd_data.width,y+2*fsize,BLACK);//�����ʾ
	FRONT_COLOR=WHITE;
	return res;
}


//Ӳ�����
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
	
REINIT://���³�ʼ��
	SysTick_Init(168);				//��ʱ��ʼ�� 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //�ж����ȼ����� ��2��
	USART1_Init(115200);		//��ʼ�����ڲ�����Ϊ115200 
 	LED_Init();					//��ʼ��LED 
 	TFTLCD_Init();					//LCD��ʼ��    
 	BEEP_Init();				//��������ʼ��
 	KEY_Init();					//������ʼ�� 
	FSMC_SRAM_Init();			//��ʼ���ⲿSRAM
 	AT24CXX_Init();    		//EEPROM��ʼ��
	EN25QXX_Init();				//��ʼ��W25Q128
 	Lsens_Init();				//��ʼ������������
	ADC_Temp_Init();			//��ʼ���ڲ��¶ȴ����� 
	my_mem_init(SRAMIN);		//��ʼ���ڲ��ڴ��
	my_mem_init(SRAMCCM);		//��ʼ��CCM�ڴ�� 
	TP_Init();
	piclib_init();				//piclib��ʼ��
	gui_init();	  				//gui��ʼ��
	FATFS_Init();				//Ϊfatfs�ر��������ڴ�
	
//	LOGO_Display();//����LOGO
	LCD_Clear(BLACK);			//����
	FRONT_COLOR=WHITE;
	BACK_COLOR=BLACK;
	j=0;
	//��ʾ��Ȩ��Ϣ
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
	
	//��ʼӲ������ʼ��
	WM8978_Init();//��ֹ�����ҽ�
	WM8978_HPvol_Set(0,0);//�رն������
	WM8978_SPKvol_Set(0);//�ر��������
	LED1=0;LED2=0;				//ͬʱ��������LED
	LCD_ShowString(5,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "CPU:STM32F407ZGT6 168Mhz");
	LCD_ShowString(5,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "FLASH:1024KB  SRAM:192KB");	
	if(system_exsram_test(5,ypos+fsize*j,fsize))
		system_error_show(5,ypos+fsize*j++,"EX Memory Error!",fsize);
	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize,"OK");			 
	my_mem_init(SRAMEX);		//��ʼ���ⲿ�ڴ��,��������ڴ���֮��
	//�ⲿFLASH���
	if((EN25QXX_ReadID()==0x00)||(EN25QXX_ReadID()==0xFF))//��ⲻ��EN25QXX
	{	 
		system_error_show(5,ypos+fsize*j++,"Ex Flash Error!!",fsize); 
	}else temp=16*1024;	//16M�ֽڴ�С
	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "Ex Flash:     KB");			   
	LCD_ShowxNum(5+9*(fsize/2),ypos+fsize*j,temp,5,fsize,0);//��ʾflash��С  
	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");
	//����Ƿ���Ҫ����SPI FLASH?
	res=KEY_Scan(1);//
	if(res==KEY_UP_PRESS)
	{
		res=system_files_erase(5,ypos+fsize*j,fsize);
		if(res)goto REINIT; 
	}
    //RTC���
  	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "RTC Check...");			   
 	if(RTC_Config())system_error_show(5,ypos+fsize*(j+1),"RTC Error!",fsize);//RTC���
	else 
	{
		RTC_Set_WakeUp(RTC_WakeUpClock_CK_SPRE_16bits,0);//����WAKE UP�ж�,1�����ж�һ��
		RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);//�õ���ǰʱ��
		RTC_GetDate(RTC_Format_BIN,&RTC_DateStruct);//�õ���ǰ����
		LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");			   
	}
	//����SPI FLASH���ļ�ϵͳ
	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "FATFS Check...");//FATFS���			   
  	f_mount(fs[0],"0:",1); 		//����SD��  
  	f_mount(fs[1],"1:",1); 		//���ع���FLASH. 
 	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");			   
	//SD�����
	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "SD Card:     MB");//FATFS���
	temp=0;	
 	do
	{
		temp++;
 		res=fatfs_getfree("0:",&dtsize,&dfsize);//�õ�SD��ʣ��������������
		delay_ms(200);		   
	}while(res&&temp<5);//�������5��
 	if(res==0)//�õ���������
	{ 
		gui_phy.memdevflag|=1<<0;	//����SD����λ.
		temp=dtsize>>10;//��λת��ΪMB
		stastr="OK";
 	}else 
	{
 		temp=0;//������,��λΪ0
		stastr="ERROR";
	}
 	LCD_ShowxNum(5+8*(fsize/2),ypos+fsize*j,temp,5,fsize,0);					//��ʾSD��������С
	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize,stastr);	//SD��״̬			   
	//W25Q128���,����������ļ�ϵͳ,���ȴ���.
	temp=0;	
 	do
	{
		temp++;
 		res=fatfs_getfree("1:",&dtsize,&dfsize);//�õ�FLASHʣ��������������
		delay_ms(200);		   
	}while(res&&temp<20);//�������20��		  
	if(res==0X0D)//�ļ�ϵͳ������
	{
		LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "Flash Disk Formatting...");	//��ʽ��FLASH
		res=f_mkfs("1:",1,4096);//��ʽ��FLASH,1,�̷�;1,����Ҫ����,8����Ϊ1����
		if(res==0)
		{
			f_setlabel((const TCHAR *)"1:PRECHIN");	//����Flash���̵�����Ϊ��PRECHIN
			LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");//��־��ʽ���ɹ�
 			res=fatfs_getfree("1:",&dtsize,&dfsize);//���»�ȡ����
		}
	}   
	if(res==0)//�õ�FLASH��ʣ��������������
	{
		gui_phy.memdevflag|=1<<1;	//����SPI FLASH��λ.
		LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "Flash Disk:     KB");//FATFS���			   
		temp=dtsize; 	   
 	}else system_error_show(5,ypos+fsize*(j+1),"Flash Fat Error!",fsize);	//flash �ļ�ϵͳ���� 
 	LCD_ShowxNum(5+11*(fsize/2),ypos+fsize*j,temp,5,fsize,0);						//��ʾFLASH������С
	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize,"OK");			//FLASH��״̬	
	//TPAD���		 
 	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "TPAD Check...");			   
// 	if(Touch_Key_Init(4))system_error_show(5,ypos+fsize*(j+1),"TPAD Error!",fsize);//�����������
//	else LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK"); 
	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK"); 
	//24C02���
   	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "24C02 Check...");			   
 	if(AT24CXX_Check())system_error_show(5,ypos+fsize*(j+1),"24C02 Error!",fsize);//24C02���
	else LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");  
	//MPU6050��� 
   	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "MPU6050 Check...");			   
 	if(MPU6050_Init())system_error_show(5,ypos+fsize*j++,"MPU6050 Error!",fsize);
	else LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");
	//WM8978���			   
 	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "WM8978 Check...");			   
 	if(WM8978_Init())system_error_show(5,ypos+fsize*(j+1),"WM8978 Error!",fsize);//WM8978���
	else 
	{
		LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");	
		WM8978_HPvol_Set(0,0);//�رն������
		WM8978_SPKvol_Set(0);//�ر��������
  	}
//	//LAN8720���   
//	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "LAN8720 Check...");			   
// 	if(LAN8720_Init())system_error_show(5,ypos+fsize*(j+1),"LAN8720 Error!",fsize);//LAN8720���
//	else LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK"); 
//	LAN8720_RST=0;		//��λLAN8720	
	//�ֿ���								    
   	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "Font Check...");
	res=KEY_Scan(1);//��ⰴ��
	if(res==KEY1_PRESS)//���£�ȷ��
	{
		res=system_font_update_confirm(5,ypos+fsize*(j+1),fsize);
	}else res=0;
	if(font_init()||(res==1))//�������,������岻����/ǿ�Ƹ���,������ֿ�	
	{
		res=0;//������Ч
 		if(update_font(5,ypos+fsize*j,fsize,"0:")!=0)//��SD������
		{
 			system_error_show(5,ypos+fsize*(j+1),"Font Error!",fsize);	//�������
		}
		else 
		{
			ypos=0;
			goto REINIT;
		}
		LCD_Fill(5,ypos+fsize*j,tftlcd_data.width,ypos+fsize*(j+1),BLACK);//����ɫ
    	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "Font Check...");			   
 	} 
	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");//�ֿ���OK
	//��������� 
 	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "Touch Check...");			   
	res=KEY_Scan(1);//��ⰴ��			   
	if(TP_Init()||(res==KEY0_PRESS&&(tp_dev.touchtype&0X80)==0))//�и���/������KEY0�Ҳ��ǵ�����,ִ��У׼ 	
	{
		if(res==KEY0_PRESS)TP_Adjust();
		res=0;//������Ч
		goto REINIT;				//���¿�ʼ��ʼ��
	}
	LCD_ShowString(5+okoffset,ypos+fsize*j++,tftlcd_data.width,tftlcd_data.height,fsize, "OK");//���������OK
   	LCD_ShowString(5,ypos+fsize*j,tftlcd_data.width,tftlcd_data.height,fsize, "SYSTEM Starting...");  
	//�ر�LED
	LED1=1;LED2=1;
	//�������̽�,��ʾ��������
	BEEP=1;
	delay_ms(100);
	BEEP=0; 
	delay_ms(500);
	TIM4_Init(200,8399);		//��ʱ20ms	
//	TIM3_Init(500,8399);		//��ʱ50ms
	TIM2_Init(1000,8399);		//10Khz����Ƶ��,100ms�ж�
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

int main()
{	
	u8 index=0;
	
	Hardware_Check();
	DistanceSensor_Init();
	CAN1_Init_RangerMini();
	printf("[CAN] Init done, MCR=0x%08X MSR=0x%08X\r\n",
		(unsigned int)CAN1->MCR, (unsigned int)CAN1->MSR);
	
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
}

// 传感器数据刷屏显示函数
void SensorData_ShowScreen(void)
{
	DistanceSensor_Data *ds = DistanceSensor_GetData();
	u8 i;
	u16 y = 20;
	u8 fsize = 16;
	u8 buf[8];
	static u32 cnt = 0;
	
	cnt++;
	
	// 清屏
	LCD_Clear(BLACK);
	
	// 显示标题
	LCD_ShowString(5, y, tftlcd_data.width, tftlcd_data.height, fsize, "=== Sensor Data ===");
	y += fsize + 10;
	
	// 显示计数器（验证刷新）
	LCD_ShowString(5, y, tftlcd_data.width, tftlcd_data.height, fsize, "Count: ");
	LCD_ShowxNum(5+8*(fsize/2), y, cnt, 5, fsize, 0);
	y += fsize + 10;
	
	// 显示 CAN 状态
	LCD_ShowString(5, y, tftlcd_data.width, tftlcd_data.height, fsize, "[CAN] ESR=0x");
	LCD_ShowxNum(5+12*(fsize/2), y, (u32)CAN1->ESR, 8, fsize, 0);
	y += fsize + 10;
	
	// 显示传感器数据
	LCD_ShowString(5, y, tftlcd_data.width, tftlcd_data.height, fsize, "Distance Sensors:");
	y += fsize;
	
	for(i = 0; i < 4; i++)
	{
		LCD_ShowString(5, y, tftlcd_data.width, tftlcd_data.height, fsize, "IF");
		LCD_ShowxNum(5+3*(fsize/2), y, i+1, 1, fsize, 0);
		LCD_ShowString(5+4*(fsize/2), y, tftlcd_data.width, tftlcd_data.height, fsize, ": ");
		
		if(ds->valid && ds->error[i] == DS_ERR_NONE)
		{
			LCD_ShowxNum(5+6*(fsize/2), y, ds->dist[i], 5, fsize, 0);
			LCD_ShowString(5+12*(fsize/2), y, tftlcd_data.width, tftlcd_data.height, fsize, " mm");
			
			// 准备 CAN 发送数据（大端序）
			buf[i*2] = (ds->dist[i] >> 8) & 0xFF;
			buf[i*2+1] = ds->dist[i] & 0xFF;
		}
		else
		{
			LCD_ShowString(5+6*(fsize/2), y, tftlcd_data.width, tftlcd_data.height, fsize, "---");
		}
		y += fsize;
	}
	
	// 如果有新数据，通过 CAN 发送
	if(DistanceSensor_NewData() && ds->valid)
	{
		CAN1_Send_Msg_WithID(0x111, buf, 8);
		LCD_ShowString(5, y, tftlcd_data.width, tftlcd_data.height, fsize, "[CAN] Data sent!");
	}
	
	delay_ms(100);
}
