#ifndef _qrcode_app_H
#define _qrcode_app_H


#include "gui.h"


//视频播放控制器
typedef __packed struct
{    
	u8 *path;			//当前文件夹路径 
	u8 *name;			//当前视频名字
	vu8 status;			//bit0:0,暂停播放;1,继续播放
						//bit1:0,快进/快退中;1,继续播放
						//其他,保留
	
	u16 curindex;		//当前播放的视频文件索引
	u16 mfilenum;		//视频文件数目	    
	u16 *mfindextbl;	//视频文件索引表
	
	FIL *file;			//视频文件指针 	
	vu8 i2splaybuf;		//即将播放的音频帧缓冲编号
	u8* i2sbuf[4]; 		//音频缓冲帧,共4帧,4*AVI_AUDIO_BUF_SIZE
}__videodev; 
extern __videodev videodev;//视频播放控制器

void Qrcode_APP_Test(void);


#endif
