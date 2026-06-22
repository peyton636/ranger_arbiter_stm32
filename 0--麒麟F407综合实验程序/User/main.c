/* 界面分流：0=legacy  1=arbiter(运动控制)  2=my_gui */
#define UI_TEST_MODE             1
#define CHASSIS_CAN_MOTION_TEST  0

#include "system.h"
#include "SysTick.h"
#include "led.h"
#include "usart.h"
#include "tftlcd.h"
#include "key.h"
#include "beep.h"
#include "time.h"
#include "malloc.h"
#include "sram.h"
#include "stdio.h"
#include "freertos_app.h"
#include "app_boot.h"
#include "chassis_can_test.h"
#if (UI_TEST_MODE == 0)
#include "24cxx.h"
#include "rtc.h"
#include "adc_temp.h"
#include "touch_key.h"
#include "lsens.h"
#include "mpu6050.h"
#include "touch.h"
#include "flash.h"
#include "fatfs_app.h"
#include "ff.h"
#include "font_show.h"
#include "piclib.h"
#include "wm8978.h"
#include "common.h"
#endif

u8 VERSION[] = "HARDWARE:V1.0   SOFTWARE:V1.2";



#if (UI_TEST_MODE == 0)



u8 system_exsram_test(u16 x, u16 y, u8 fsize)

{

	u32 i = 0;

	u16 temp = 0;

	u16 sval = 0;



	LCD_ShowString(x, y, tftlcd_data.width, y + fsize, fsize, "Ex Memory Test:   0KB");

	for(i = 0; i < 1024 * 1024; i += 1024)

	{

		FSMC_SRAM_WriteBuffer((u8*)&temp, i, 2);

		temp++;

	}

	for(i = 0; i < 1024 * 1024; i += 1024)

	{

		FSMC_SRAM_ReadBuffer((u8*)&temp, i, 2);

		if(i == 0)

			sval = temp;

		else if(temp <= sval)

			break;

		LCD_ShowxNum(x + 15 * (fsize / 2), y, (u16)(temp - sval + 1), 4, fsize, 0);

	}

	if(i >= 1024 * 1024)

	{

		LCD_ShowxNum(x + 15 * (fsize / 2), y, i / 1024, 4, fsize, 0);

		return 0;

	}

	return 1;

}



void system_error_show(u16 x, u16 y, u8 *err, u8 fsize)

{

	FRONT_COLOR = RED;

	while(1)

	{

		LCD_ShowString(x, y, tftlcd_data.width, tftlcd_data.height, fsize, err);

		delay_ms(200);

		LCD_Fill(x, y, tftlcd_data.width, y + fsize, BLACK);

		delay_ms(200);

	}

}



u8 system_files_erase(u16 x, u16 y, u8 fsize)

{

	u8 key;

	u8 t = 0;



	FRONT_COLOR = RED;

	LCD_ShowString(x, y, tftlcd_data.width, tftlcd_data.height, fsize, "Erase all system files?");

	while(1)

	{

		t++;

		if(t == 20)

			LCD_ShowString(x, y + fsize, tftlcd_data.width, tftlcd_data.height, fsize, "KEY0:NO / KEY1:YES");

		if(t == 40)

		{

			gui_fill_rectangle(x, y + fsize, tftlcd_data.width, fsize, BLACK);

			t = 0;

			LED1 = !LED1;

		}

		key = KEY_Scan(0);

		if(key == KEY0_PRESS)

		{

			gui_fill_rectangle(x, y, tftlcd_data.width, fsize * 2, BLACK);

			FRONT_COLOR = WHITE;

			LED1 = 1;

			return 0;

		}

		if(key == KEY1_PRESS)

		{

			LED1 = 1;

			LCD_ShowString(x, y + fsize, tftlcd_data.width, tftlcd_data.height, fsize, "Erasing SPI FLASH...");

			EN25QXX_Erase_Chip();

			LCD_ShowString(x, y + fsize, tftlcd_data.width, tftlcd_data.height, fsize, "Erasing SPI FLASH OK");

			delay_ms(600);

			return 1;

		}

		delay_ms(10);

	}

}



u8 system_font_update_confirm(u16 x, u16 y, u8 fsize)

{

	u8 key;

	u8 t = 0;

	u8 res = 0;



	FRONT_COLOR = RED;

	LCD_ShowString(x, y, tftlcd_data.width, tftlcd_data.height, fsize, "Update font?");

	while(1)

	{

		t++;

		if(t == 20)

			LCD_ShowString(x, y + fsize, tftlcd_data.width, tftlcd_data.height, fsize, "KEY1:NO / KEY0:YES");

		if(t == 40)

		{

			LCD_Fill(x, y + fsize, tftlcd_data.width, y + fsize + fsize, BLACK);

			t = 0;

		}

		key = KEY_Scan(0);

		if(key == KEY1_PRESS)

			break;

		if(key == KEY0_PRESS)

		{

			res = 1;

			break;

		}

		delay_ms(10);

	}

	LCD_Fill(x, y, tftlcd_data.width, y + 2 * fsize, BLACK);

	FRONT_COLOR = WHITE;

	return res;

}



#endif /* UI_TEST_MODE == 0 */



static void Hardware_BootPeripherals(void)

{

	SysTick_Init(168);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	USART1_Init(115200);

	USART1_Probe("BOOT");

	LED_Init();

	TFTLCD_Init();

	BEEP_Init();

	KEY_Init();

	FSMC_SRAM_Init();

	my_mem_init(SRAMIN);

	my_mem_init(SRAMCCM);

	my_mem_init(SRAMEX);

}



#if (UI_TEST_MODE == 0)

void Hardware_Check(void)

{

	u16 okoffset = 162;

	u16 ypos = 0;

	u16 j = 0;

	u16 temp = 0;

	u8 res;

	u32 dtsize, dfsize;

	u8 *stastr = 0;

	u8 fsize;



REINIT:

	Hardware_BootPeripherals();

	AT24CXX_Init();

	EN25QXX_Init();

	Lsens_Init();

	ADC_Temp_Init();

	TP_Init();

	piclib_init();

	gui_init();

	FATFS_Init();



	LCD_Clear(BLACK);

	FRONT_COLOR = WHITE;

	BACK_COLOR = BLACK;

	j = 0;

	ypos = 2;

	if(tftlcd_data.width <= 272)

	{

		fsize = 16;

		okoffset = 190;

	}

	else if(tftlcd_data.width == 320)

	{

		fsize = 16;

		okoffset = 250;

	}

	else if(tftlcd_data.width == 480)

	{

		fsize = 24;

		okoffset = 370;

	}

	LCD_ShowString(5, ypos + fsize * j++, tftlcd_data.width, tftlcd_data.height, fsize, "---PRECHIN---");

	LCD_ShowString(5, ypos + fsize * j++, tftlcd_data.width, tftlcd_data.height, fsize, VERSION);



	WM8978_Init();

	WM8978_HPvol_Set(0, 0);

	WM8978_SPKvol_Set(0);

	LED1 = 0;

	LED2 = 0;

	LCD_ShowString(5, ypos + fsize * j++, tftlcd_data.width, tftlcd_data.height, fsize, "CPU:STM32F407ZGT6 168Mhz");

	LCD_ShowString(5, ypos + fsize * j++, tftlcd_data.width, tftlcd_data.height, fsize, "FLASH:1024KB  SRAM:192KB");

	if(system_exsram_test(5, ypos + fsize * j, fsize))

		system_error_show(5, ypos + fsize * j++, "EX Memory Error!", fsize);

	LCD_ShowString(5 + okoffset, ypos + fsize * j++, tftlcd_data.width, tftlcd_data.height, fsize, "OK");

	if((EN25QXX_ReadID() == 0x00) || (EN25QXX_ReadID() == 0xFF))

		system_error_show(5, ypos + fsize * j++, "Ex Flash Error!!", fsize);

	else

		temp = 16 * 1024;

	LCD_ShowString(5, ypos + fsize * j, tftlcd_data.width, tftlcd_data.height, fsize, "Ex Flash:     KB");

	LCD_ShowxNum(5 + 9 * (fsize / 2), ypos + fsize * j, temp, 5, fsize, 0);

	LCD_ShowString(5 + okoffset, ypos + fsize * j++, tftlcd_data.width, tftlcd_data.height, fsize, "OK");

	res = KEY_Scan(1);

	if(res == KEY_UP_PRESS)

	{

		res = system_files_erase(5, ypos + fsize * j, fsize);

		if(res)

			goto REINIT;

	}

	LCD_ShowString(5, ypos + fsize * j, tftlcd_data.width, tftlcd_data.height, fsize, "RTC Check...");

	if(RTC_Config())

		system_error_show(5, ypos + fsize * (j + 1), "RTC Error!", fsize);

	else

	{

		RTC_Set_WakeUp(RTC_WakeUpClock_CK_SPRE_16bits, 0);

		RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);

		RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);

		LCD_ShowString(5 + okoffset, ypos + fsize * j++, tftlcd_data.width, tftlcd_data.height, fsize, "OK");

	}

	LCD_ShowString(5, ypos + fsize * j, tftlcd_data.width, tftlcd_data.height, fsize, "FATFS Check...");

	f_mount(fs[0], "0:", 1);

	f_mount(fs[1], "1:", 1);

	LCD_ShowString(5 + okoffset, ypos + fsize * j++, tftlcd_data.width, tftlcd_data.height, fsize, "OK");

	LCD_ShowString(5, ypos + fsize * j, tftlcd_data.width, tftlcd_data.height, fsize, "SD Card:     MB");

	temp = 0;

	do

	{

		temp++;

		res = fatfs_getfree("0:", &dtsize, &dfsize);

		delay_ms(200);

	} while(res && temp < 5);

	if(res == 0)

	{

		gui_phy.memdevflag |= 1 << 0;

		temp = dtsize >> 10;

		stastr = "OK";

	}

	else

	{

		temp = 0;

		stastr = "ERROR";

	}

	LCD_ShowxNum(5 + 8 * (fsize / 2), ypos + fsize * j, temp, 5, fsize, 0);

	LCD_ShowString(5 + okoffset, ypos + fsize * j++, tftlcd_data.width, tftlcd_data.height, fsize, stastr);

	temp = 0;

	do

	{

		temp++;

		res = fatfs_getfree("1:", &dtsize, &dfsize);

		delay_ms(200);

	} while(res && temp < 20);

	if(res != 0)

	{

		printf("[FAT] 1: getfree err=%u, try mkfs...\r\n", (unsigned)res);

		LCD_ShowString(5, ypos + fsize * j, tftlcd_data.width, tftlcd_data.height, fsize, "Flash Disk Formatting...");

		f_mount(fs[1], "1:", 1);

		res = f_mkfs("1:", 1, 4096);

		if(res == 0)

		{

			f_setlabel((const TCHAR *)"1:PRECHIN");

			f_mount(fs[1], "1:", 1);

			res = fatfs_getfree("1:", &dtsize, &dfsize);

			if(res == 0)

				LCD_ShowString(5 + okoffset, ypos + fsize * j, tftlcd_data.width, tftlcd_data.height, fsize, "OK");

		}

		else

		{

			printf("[FAT] mkfs 1: failed, err=%u\r\n", (unsigned)res);

		}

	}

	LCD_ShowString(5, ypos + fsize * j, tftlcd_data.width, tftlcd_data.height, fsize, "Flash Disk:     KB");

	if(res == 0)

	{

		gui_phy.memdevflag |= 1 << 1;

		temp = dtsize;

		LCD_ShowxNum(5 + 11 * (fsize / 2), ypos + fsize * j, temp, 5, fsize, 0);

		LCD_ShowString(5 + okoffset, ypos + fsize * j++, tftlcd_data.width, tftlcd_data.height, fsize, "OK");

	}

	else

	{

		temp = 0;

		LCD_ShowxNum(5 + 11 * (fsize / 2), ypos + fsize * j, temp, 5, fsize, 0);

		system_error_show(5, ypos + fsize * (j + 1), "Flash Fat Error!", fsize);

	}

	LCD_ShowString(5, ypos + fsize * j, tftlcd_data.width, tftlcd_data.height, fsize, "TPAD Check...");

	LCD_ShowString(5 + okoffset, ypos + fsize * j++, tftlcd_data.width, tftlcd_data.height, fsize, "OK");

	LCD_ShowString(5, ypos + fsize * j, tftlcd_data.width, tftlcd_data.height, fsize, "24C02 Check...");

	if(AT24CXX_Check())

		system_error_show(5, ypos + fsize * (j + 1), "24C02 Error!", fsize);

	else

		LCD_ShowString(5 + okoffset, ypos + fsize * j++, tftlcd_data.width, tftlcd_data.height, fsize, "OK");

	LCD_ShowString(5, ypos + fsize * j, tftlcd_data.width, tftlcd_data.height, fsize, "MPU6050 Check...");

	if(MPU6050_Init())

		system_error_show(5, ypos + fsize * j++, "MPU6050 Error!", fsize);

	else

		LCD_ShowString(5 + okoffset, ypos + fsize * j++, tftlcd_data.width, tftlcd_data.height, fsize, "OK");

	LCD_ShowString(5, ypos + fsize * j, tftlcd_data.width, tftlcd_data.height, fsize, "WM8978 Check...");

	if(WM8978_Init())

		system_error_show(5, ypos + fsize * (j + 1), "WM8978 Error!", fsize);

	else

	{

		LCD_ShowString(5 + okoffset, ypos + fsize * j++, tftlcd_data.width, tftlcd_data.height, fsize, "OK");

		WM8978_HPvol_Set(0, 0);

		WM8978_SPKvol_Set(0);

	}

	LCD_ShowString(5, ypos + fsize * j, tftlcd_data.width, tftlcd_data.height, fsize, "Font Check...");

	res = KEY_Scan(1);

	if(res == KEY1_PRESS)

		res = system_font_update_confirm(5, ypos + fsize * (j + 1), fsize);

	else

		res = 0;

	if(font_init() || (res == 1))

	{

		res = 0;

		if(update_font(5, ypos + fsize * j, fsize, "0:") != 0)

			system_error_show(5, ypos + fsize * (j + 1), "Font Error!", fsize);

		else

		{

			ypos = 0;

			goto REINIT;

		}

		LCD_Fill(5, ypos + fsize * j, tftlcd_data.width, ypos + fsize * (j + 1), BLACK);

		LCD_ShowString(5, ypos + fsize * j, tftlcd_data.width, tftlcd_data.height, fsize, "Font Check...");

	}

	LCD_ShowString(5 + okoffset, ypos + fsize * j++, tftlcd_data.width, tftlcd_data.height, fsize, "OK");

	LCD_ShowString(5, ypos + fsize * j, tftlcd_data.width, tftlcd_data.height, fsize, "Touch Check...");

	res = KEY_Scan(1);

	if(TP_Init() || (res == KEY0_PRESS && (tp_dev.touchtype & 0X80) == 0))

	{

		if(res == KEY0_PRESS)

			TP_Adjust();

		res = 0;

		goto REINIT;

	}

	LCD_ShowString(5 + okoffset, ypos + fsize * j++, tftlcd_data.width, tftlcd_data.height, fsize, "OK");

	LCD_ShowString(5, ypos + fsize * j, tftlcd_data.width, tftlcd_data.height, fsize, "SYSTEM Starting...");

	LED1 = 1;

	LED2 = 1;

	BEEP = 1;

	delay_ms(100);

	BEEP = 0;

	delay_ms(500);

	TIM4_Init(200, 8399);

	TIM2_Init(1000, 8399);

}

#else

void Hardware_Check(void)

{

	Hardware_BootPeripherals();



	LCD_Clear(BLACK);

	FRONT_COLOR = WHITE;

	BACK_COLOR = BLACK;

	LCD_ShowString(5, 20, tftlcd_data.width, tftlcd_data.height, 16, "AGV Boot...");

	printf("[BOOT] %s\r\n", VERSION);



	LED1 = 1;
	LED2 = 1;
	/* 20ms：OSTime + 测距触发；不开 TIM2（拍照 ISR 含 delay_ms，与 KeyTask 冲突） */
	TIM4_Init(200, 8399);
	Hardware_Init();
}

#endif



#if (UI_TEST_MODE == 0)

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

#endif



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
			printf("[MY_GUI] KEY0: legacy disabled, set UI_TEST_MODE=0\r\n");

		else if(key == KEY1_PRESS)

		{

			printf("[MY_GUI] KEY1 -> Arbiter\r\n");

			break;

		}

		delay_ms(10);

	}

}

#endif



int main(void)

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


