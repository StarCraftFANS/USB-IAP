#ifndef __BOOT1_H__
#define __BOOT1_H__

//WIFI-DMX.COM

#include "bootconfig.h"

#if BPS_115200
#define USART_BPS  115200
#else
#define USART_BPS  250000
#endif

//0x0138 //115200
//USART2->BRR=0X0090; //250000

typedef enum
{
 CMD_IAP  =((unsigned short)0x00A0),
 CMD_S	  =((unsigned short)0x0101),
 CMD_S1	  =((unsigned short)0x0111),
 CMD_S2	  =((unsigned short)0x0121),
 CMD_S3	  =((unsigned short)0x0131),
 CMD_S4	  =((unsigned short)0x0141),
}CMD_Typedef;

//#define S103_S1_Enable      1
//#define S103_S1_S2_Enable   1
#define S107_S1_S2_Enable   1

#if STM32_Enable
#define MENU_ESC_Pin		GPIO_Pin_12
#define MENU_ESC_Port		GPIOA
#endif

#define IAP_Version	    "XY-026 V1.00"
#define IAP_Version_S	"XY-026-V1.00-S  "
#define IAP_Version_S1	"XY-026-V1.00-S1 "
#define IAP_Version_S2	"XY-026-V1.00-S2 "

#if S107_S1_S2_Enable  
	#define BANK1_IAP_START_ADDR  ((unsigned int)(0x08000000+START_ADDR))
	#define BANK1_IAP_END_ADDR    ((unsigned int)(0x08010000))
	#define BANK2_IAP_START_ADDR  ((unsigned int)(0x08010000+START_ADDR))
	#define BANK2_IAP_END_ADDR    ((unsigned int)(0x08020000))
	#define BANK3_IAP_START_ADDR  ((unsigned int)(0x08020000+START_ADDR))
	#define BANK3_IAP_END_ADDR    ((unsigned int)(0x08039000))
#endif

#if S103_S1_S2_Enable 
	#define BANK1_IAP_START_ADDR  ((unsigned int)(0x08000000+START_ADDR))
	#define BANK1_IAP_END_ADDR    ((unsigned int)(0x08010000))
	#define BANK2_IAP_START_ADDR  ((unsigned int)(0x08010000+START_ADDR))
	#define BANK2_IAP_END_ADDR    ((unsigned int)(0x08020000))
	#define BANK3_IAP_START_ADDR  ((unsigned int)(0x08020000+START_ADDR)) 
	#define BANK3_IAP_END_ADDR    ((unsigned int)(0x08030000))
#endif

#if S103_S1_Enable 
	#define BANK2_WRITE_START_ADDR  ((unsigned int)(0x08000000+START_ADDR))
	#define BANK2_WRITE_END_ADDR    ((unsigned int)(0x08010000))
	#define BANK3_WRITE_START_ADDR  ((unsigned int)(0x08010000+START_ADDR))
	#define BANK3_WRITE_END_ADDR    ((unsigned int)(0x08020000))
#endif

void SYS_Init(void);
void FLASH_ProgramUnLock(void);
void FLASH_ProgramLock(void);
void Jump_To_App(void);
void FLASH_EraseAll(void);
void FLASH_ProgramOneWord(unsigned int Address, unsigned int Data);
void D485_SendData(unsigned short UDR);
void DMX_SendData(unsigned short UDR);
void IAP_Wait(void);
void IAP_MASTER(void);

#endif
