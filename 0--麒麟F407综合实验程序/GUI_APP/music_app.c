#include "music_app.h"
#include "button.h"
#include "touch.h"
#include "common.h"	
#include "key.h"
#include "wm8978.h"	
#include "audioplay.h"






//音乐列表
u8*const MUSIC_LIST[GUI_LANGUAGE_NUM]=
{
	"音乐列表","音樂列表","MUSIC LIST",
};

//audio文件浏览,带文件存储功能
//audiodevx:audio结构体
//返回值:0,正常返回/按了退出按钮.
//		 1,内存分配失败	
//		2,退出
u8 audio_filelist(__audiodev *audiodevx)
{	 	   
	u8 res;
	u8 rval=0;			//返回值	  
  	u16 i;	    						   
 	_btn_obj* rbtn;		//返回按钮控件
 	_btn_obj* qbtn;		//退出按钮控件

   	_filelistbox_obj * flistbox;
	_filelistbox_list * filelistx; 	//文件
 	app_filebrower((u8*)MUSIC_LIST[gui_phy.language],0X07);	//选择目标文件,并得到目标数量
 
  	flistbox=filelistbox_creat(0,gui_phy.tbheight,tftlcd_data.width,tftlcd_data.height-gui_phy.tbheight*2,1,gui_phy.listfsize);//创建一个filelistbox
 	if(flistbox==NULL)rval=1;							//申请内存失败.
	else if(audiodevx->path==NULL)  
	{
		flistbox->fliter=FLBOX_FLT_MUSIC;	//查找音乐文件
 		filelistbox_add_disk(flistbox);		//添加磁盘路径
		filelistbox_draw_listbox(flistbox);
	}else
	{
		flistbox->fliter=FLBOX_FLT_MUSIC;		//查找音乐文件	 
		flistbox->path=(u8*)gui_memin_malloc(strlen((const char*)audiodevx->path)+1);//为路径申请内存
		strcpy((char *)flistbox->path,(char *)audiodevx->path);//复制路径	    
		filelistbox_scan_filelist(flistbox);	//重新扫描列表 
		flistbox->selindex=flistbox->foldercnt+audiodevx->curindex;//选中条目为当前正在播放的条目
		if(flistbox->scbv->totalitems>flistbox->scbv->itemsperpage)flistbox->scbv->topitem=flistbox->selindex;  
		filelistbox_draw_listbox(flistbox);		//重画 		 
	} 	 		 
	rbtn=btn_creat(tftlcd_data.width-2*gui_phy.tbfsize-8-1,tftlcd_data.height-gui_phy.tbheight,2*gui_phy.tbfsize+8,gui_phy.tbheight-1,0,0x03);//创建文字按钮
  	qbtn=btn_creat(0,tftlcd_data.height-gui_phy.tbheight,2*gui_phy.tbfsize+8,gui_phy.tbheight,0,0x03);//创建退出文字按钮
	if(rbtn==NULL||qbtn==NULL)rval=1;	//没有足够内存够分配
	else
	{
	 	rbtn->caption=(u8*)GUI_BACK_CAPTION_TBL[gui_phy.language];	//返回
	 	rbtn->font=gui_phy.tbfsize;//设置新的字体大小	 	 
		rbtn->bcfdcolor=WHITE;	//按下时的颜色
		rbtn->bcfucolor=WHITE;	//松开时的颜色
		btn_draw(rbtn);//画按钮 
		
	 	qbtn->caption=(u8*)GUI_QUIT_CAPTION_TBL[gui_phy.language];	//名字
	 	qbtn->font=gui_phy.tbfsize;//设置新的字体大小	 
		qbtn->bcfdcolor=WHITE;	//按下时的颜色
		qbtn->bcfucolor=WHITE;	//松开时的颜色
		btn_draw(qbtn);//画按钮
	}	   
   	while(rval==0)
	{
		tp_dev.scan(0);    
		in_obj.get_key(&tp_dev,IN_TYPE_TOUCH);	//得到按键键值   
		delay_ms(10);		//延时一个时钟节拍
		if(KEY_Scan(1)==KEY1_PRESS)break;		//KEY1返回
		filelistbox_check(flistbox,&in_obj);	//扫描文件
		res=btn_check(rbtn,&in_obj);
		if(res)
		{
			if(((rbtn->sta&0X80)==0))//按钮状态改变了
			{
				if(flistbox->dbclick!=0X81)
				{
 					filelistx=filelist_search(flistbox->list,flistbox->selindex);//得到此时选中的list的信息
					if(filelistx->type==FICO_DISK)//已经不能再往上了
					{				 
						rval=2;
						break;//退出
					}else filelistbox_back(flistbox);//退回上一层目录	 
				} 
 			}	 
		}
		res=btn_check(qbtn,&in_obj);
		if(res)
		{
			if(((qbtn->sta&0X80)==0))//按钮状态改变了
			{ 
				rval=2;
				break;//退出
 			}	 
		}   
		if(flistbox->dbclick==0X81)//双击文件了
		{											 
			gui_memin_free(audiodevx->path);		//释放内存
			gui_memin_free(audiodevx->mfindextbl);	//释放内存
			audiodevx->path=(u8*)gui_memin_malloc(strlen((const char*)flistbox->path)+1);//为新的路径申请内存
			if(audiodevx->path==NULL){rval=1;break;}
			audiodevx->path[0]='\0';//在最开始加入结束符.
 			strcpy((char *)audiodevx->path,(char *)flistbox->path);
			audiodevx->mfindextbl=(u16*)gui_memin_malloc(flistbox->filecnt*2);//为新的tbl申请内存
			if(audiodevx->mfindextbl==NULL){rval=1;break;}
		    for(i=0;i<flistbox->filecnt;i++)audiodevx->mfindextbl[i]=flistbox->findextbl[i];//复制
			audiodevx->mfilenum=flistbox->filecnt;		//记录文件个数	
			audiodev.curindex=flistbox->selindex-flistbox->foldercnt;
			flistbox->dbclick=0;
			break;	 							   			   
		}
	}	
	filelistbox_delete(flistbox);	//删除filelist
	btn_delete(qbtn);				//删除按钮	  	 
	btn_delete(rbtn);				//删除按钮	   	
	if(rval)
	{
		gui_memin_free(audiodevx->path);		//释放内存
		gui_memin_free(audiodevx->mfindextbl); 	//释放内存
		gui_memin_free(audiodevx);
 	}	 
 	return rval; 
} 

extern u8 audio_mvol;
void Music_APP_Test(void)
{
	u8 rval=0;
	u8 key; 
	DIR mp3dir;	 		//目录
	FILINFO* mp3fileinfo;//文件信息
	u8 *pname;			//带路径的文件名
	u8 res;
	
	WM8978_HPvol_Set(audio_mvol,audio_mvol);	//耳机音量设置
	WM8978_SPKvol_Set(audio_mvol);		//喇叭音量设置
	WM8978_ADDA_Cfg(1,0);	//开启DAC
	WM8978_Input_Cfg(0,0,0);//关闭输入通道
	WM8978_Output_Cfg(1,0);	//开启DAC输出
	
	memset(&audiodev,0,sizeof(__audiodev));//audiodev所有数据清零
	res=audio_filelist(&audiodev);//选择音频文件进行播放
	if(res)	//失败返回主界面
	{
		gui_memin_free(audiodev.path);		//释放内存
		gui_memin_free(audiodev.mfindextbl);//释放内存 
		WM8978_ADDA_Cfg(0,0);				//关闭DAC&ADC
		WM8978_Input_Cfg(0,0,0);			//关闭输入通道
		WM8978_Output_Cfg(0,0);				//关闭DAC输出
		WM8978_HPvol_Set(0,0);	//耳机音量设置
		WM8978_SPKvol_Set(0);	//喇叭音量设置
		ICON_UI_Init();
		return;
	}
	FRONT_COLOR=WHITE;  
  	BACK_COLOR=GRAY;  
	LCD_Clear(BACK_COLOR);		//清屏
	app_filebrower("音乐播放器应用",0X05);//显示标题
	app_gui_tcbar(0,tftlcd_data.height-gui_phy.tbheight,tftlcd_data.width,gui_phy.tbheight,0x01);	//上分界线
	
	audio_mvol=30;		//默认设置音量为30.
	mp3fileinfo=(FILINFO*)gui_memin_malloc(sizeof(FILINFO));	//申请FILENFO内存
	rval=f_opendir(&mp3dir,(const TCHAR*)audiodev.path);	//打开选中的目录
	while(rval==0&&mp3fileinfo)
	{
MUSIC_START:
		dir_sdi(&mp3dir,audiodev.mfindextbl[audiodev.curindex]);
		rval=f_readdir(&mp3dir,mp3fileinfo);//读取文件信息
		if(rval)break;//打开失败
		audiodev.name=(u8*)(mp3fileinfo->fname); 
		pname=gui_memin_malloc(strlen((const char*)audiodev.name)+strlen((const char*)audiodev.path)+2);//申请内存
		if(pname==NULL)break;//申请失败    
		pname=gui_path_name(pname,audiodev.path,audiodev.name);	//文件名加入路径 
		printf("play:%s\r\n",pname); 
		gui_fill_rectangle(0,gui_phy.tbheight,tftlcd_data.width,tftlcd_data.height-gui_phy.tbheight,BACK_COLOR);//填充内部					
		gui_show_string("KEY1:下一首",10,gui_phy.tbheight+80,tftlcd_data.width,16,16,RED);		 	 
		gui_show_string("KEY2:音量+",10,gui_phy.tbheight+100,tftlcd_data.width,16,16,RED);		 	 
		gui_show_string("KEY_UP:返回音乐列表",10,gui_phy.tbheight+120,tftlcd_data.width,16,16,RED);		 	 
		gui_show_string(audiodev.name,10,gui_phy.tbheight+10,tftlcd_data.width,16,16,RED);//显示歌曲名字
		audio_vol_show(10+7*8+30,gui_phy.tbheight+50,audio_mvol);
		audio_index_show(10,gui_phy.tbheight+50,audiodev.curindex+1,audiodev.mfilenum);
		if((f_typetell(pname)&0X40)==0X40)//是音频文件
		{ 
			key=audio_play_song(pname);	//播放音乐文件 
			if(key==2)		//返回主界面
			{
				gui_memin_free(pname);
				res=audio_filelist(&audiodev);//选择音频文件进行播放
				if(res==1||res==2)break;
				goto MUSIC_START;
			}else if(key<=1)//下一曲
			{
				audiodev.curindex++;		   	
				if(audiodev.curindex>=audiodev.mfilenum)audiodev.curindex=0;//到末尾的时候,自动从头开始
			}else break;	//产生了错误
		}
		gui_memin_free(pname);
	}
	WM8978_ADDA_Cfg(0,0);				//关闭DAC&ADC
	WM8978_Input_Cfg(0,0,0);			//关闭输入通道
	WM8978_Output_Cfg(0,0);				//关闭DAC输出
	WM8978_HPvol_Set(0,0);	//耳机音量设置
	WM8978_SPKvol_Set(0);	//喇叭音量设置 
	gui_memin_free(pname);
	gui_memin_free(mp3fileinfo);	//释放内存
	gui_memin_free(audiodev.path);		//释放内存
	gui_memin_free(audiodev.mfindextbl);//释放内存 

	ICON_UI_Init();
}

