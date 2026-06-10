#ifndef _malloc_H
#define _malloc_H


#include "system.h"


#ifndef NULL
#define NULL 0
#endif

//���������ڴ��
#define SRAMIN	 0		//�ڲ��ڴ��
#define SRAMEX   1		//�ⲿ�ڴ��
#define SRAMCCM  2		//CCM�ڴ��(�˲���SRAM����CPU���Է���!!!)


#define SRAMBANK 	3	//����֧�ֵ�SRAM����.	


//mem1内存池设定：片内主SRAM动态堆，mymalloc(SRAMIN,…) 从此分配
// sizing: FatFS常驻~3KB + WiFi AT~6KB + 临时峰值~8KB → 32KB 足够（不含 lwIP/以太网）
#define MEM1_BLOCK_SIZE			32
#define MEM1_MAX_SIZE			(32*1024)
#define MEM1_ALLOC_TABLE_SIZE	MEM1_MAX_SIZE/MEM1_BLOCK_SIZE

//mem2内存池设定：外扩SRAM @ 0x68000000，WiFi/日志/大缓冲请用 mymalloc(SRAMEX,…)
#define MEM2_BLOCK_SIZE			32
#define MEM2_MAX_SIZE			(960*1024)
#define MEM2_ALLOC_TABLE_SIZE	MEM2_MAX_SIZE/MEM2_BLOCK_SIZE
		 
//mem3内存池设定：CCM @ 0x10000000（仅CPU，无DMA）；arbiter 不用 MJPEG，缩小以给 ucHeap 留余量
#define MEM3_BLOCK_SIZE			32
#define MEM3_MAX_SIZE			(16*1024)
#define MEM3_ALLOC_TABLE_SIZE	MEM3_MAX_SIZE/MEM3_BLOCK_SIZE		 


//�ڴ����������
struct _m_mallco_dev
{
	void (*init)(u8);					//��ʼ��
	u8 (*perused)(u8);		  	    	//�ڴ�ʹ����
	u8 	*membase[SRAMBANK];				//�ڴ�� ����SRAMBANK��������ڴ�
	u16 *memmap[SRAMBANK]; 				//�ڴ����״̬��
	u8  memrdy[SRAMBANK]; 				//�ڴ�����Ƿ����
};
extern struct _m_mallco_dev mallco_dev;	 //��mallco.c���涨��

void my_mem_set(void *s,u8 c,u32 count);	//�����ڴ�
void my_mem_cpy(void *des,void *src,u32 n);//�����ڴ�     
void my_mem_init(u8 memx);				//�ڴ������ʼ������(��/�ڲ�����)
u32 my_mem_malloc(u8 memx,u32 size);	//�ڴ����(�ڲ�����)
u8 my_mem_free(u8 memx,u32 offset);		//�ڴ��ͷ�(�ڲ�����)
u8 my_mem_perused(u8 memx);				//����ڴ�ʹ����(��/�ڲ�����) 
////////////////////////////////////////////////////////////////////////////////
//�û����ú���
void myfree(u8 memx,void *ptr);  			//�ڴ��ͷ�(�ⲿ����)
void *mymalloc(u8 memx,u32 size);			//�ڴ����(�ⲿ����)
void *myrealloc(u8 memx,void *ptr,u32 size);//���·����ڴ�(�ⲿ����)
#endif
