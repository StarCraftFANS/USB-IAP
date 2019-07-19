#ifndef __FIFO_H__
#define __FIFO_H__
#include "boot1.h"

//WIFI-DMX.COM

#define FifoType    volatile unsigned short
#if defined (USE_STDPERIPH_DRIVER)
#define FifoSize    (1024*9)
#else
	#if IAP1_Enable
	#define FifoSize    (1024*5/2)
	#else
	#define FifoSize    (1024*10/3)
	#endif
#endif
#define FifoFull    1
#define FifoEmpty   2
#define FifoNormal  3

typedef struct 	
{ 
	volatile unsigned short front;  //队列头
	volatile unsigned short rear;   //队列尾
	volatile unsigned short count;  //队列数
	FifoType Buf[FifoSize];		  	
}FIFO_ST;

extern FIFO_ST MyFifo;

void FifoInit(FIFO_ST *Queue);
unsigned char FifoIn(FIFO_ST *Queue,FifoType Idat);
unsigned char FifoOut(FIFO_ST *Queue,FifoType *Odat);

#endif
