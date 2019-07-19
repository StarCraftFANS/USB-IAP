#ifndef __MAIN_H__
#define __MAIN_H__
#include <stdio.h>
#include "M0518.h"
#include "oled.h"
#include "Font.h"
#include "zimoc.h" 
#include "boot1.h"
#include "ledpwm.h"
#include "boot1.h"
#include "fifo.h"

#define SUPPORT_HL      1
//#define SUPPORT_CHINESE 1
//#define AUTO_ROTA       1
//#define LED_ADJUST      1

//#define SUPPORT_RDM	 1
//#define USART_RDM	 USART3  

#define DEVICE_STM32     (0)
#define DEVICE_M051      (1)
#define PWM_ENABLE       (0)
#define PWM_M0_DISP      (0)
#define PWM_M0_SW300P    (1)


#if DEVICE_STM32
	#define DMX512_UART   USART3
#else
	#define DMX512_UART   UART0
#endif

#define ABS(x,y) ((x>y)?(x-y):(y-x))

#define DMX_Value_20Y   55
#define DMX_Value_45Y   100
#define DMX_Value_90Y   128
#define DMX_Value_135Y  154
#define DMX_Value_150Y  200
#define DMX_Value_255Y  255

//MY-127-A V1.0
//#define MENU_ESC_Pin	PA9
//#define MENU_ENTER_Pin	PB5
//#define MENU_DOWM_Pin	PB4
//#define MENU_UP_Pin		PA8	
//FX-400P-A V1.0
#define MENU_ESC_Pin	PA9
#define MENU_ENTER_Pin	PD7
#define MENU_DOWM_Pin	PD6
#define MENU_UP_Pin		PA8	

#define LED_Pin      BIT5
#define LED_PORT     PA
#define LED_DMX      PA5

#define FAN_PORT	 PA15
#define FAN_OFF		 FAN_PORT=0
#define FAN_ON		 FAN_PORT=1

#define DEVICE_PWMTABLE  (0)
#define PWM_33333		 (1)
#define DEVICE_MASTER    (1)
#define DEVICE_SLAVE1    (1)
#define DEVICE_SLAVE2    (0)
#define DEVICE_SLAVE3    (0)

#define ADC_Mas 0xc0 //6.7
#define ADC_Chan 2
#define ADC_Temp 7
#define ADC_Mic  6
#define ADC_INT  2
#define ADC_BAT1 1
#define ADC_BAT2 3

#define DMX_DIMMER_CH1     1
#define DMX_STROBE_CH1     2
#define DMX_COLOR_CH1      3
#define DMX_GOBOZ_CH1      4
#define DMX_PRIMZ_CH1      5
#define DMX_FOCUS_CH1      6
						  
#define DMX_DIMMER_CH2     1
#define DMX_STROBE_CH2     2
#define DMX_COLOR_CH2      3
#define DMX_GOBOZ_CH2      4
#define DMX_PRIMZ_CH2      5
#define DMX_FOCUS_CH2      6
#define DMX_FUNCTION_CH2   7

//oled 
#define SCLK	PC7				// Serial Clock Input
#define SDIN	PC6				// Serial Data Input

#define RES	PF8				// Reset
#define CS	PB12			// Chip Select
#define DC	PB8				// Data/Command Control

//#define E	PA13			// Read/Write Enable
//#define RW	PA12			// Read/Write Select
//
//#define RD	PA13			// Read Signal
//#define WR	PA12			// Write Signal

#define OLED_DISP_Rotate180	 (0)

#define Wireless_Pin	
#define Wireless_Port	
#define Wireless_H      
#define Wireless_L    

#define LED_TOTAL  (1*1)  

#if PWM_12000
#define    XPWM_CNR_VAL_SET    12000
#endif
#if PWM_16666
#define    XPWM_CNR_VAL_SET    16666
#endif
#if PWM_18000
#define    XPWM_CNR_VAL_SET    18000
#endif
#if PWM_24000
#define    XPWM_CNR_VAL_SET    24000
#endif
#if PWM_33333
#define    XPWM_CNR_VAL_SET    33333
#endif
#define    XPWM_INIT_SET       XPWM_CNR_VAL_SET

#define    SWITCH_SET       1


#define    MOTOR_CNR_VAL_SET    3600
#define    MOTOR_INIT_SET       (MOTOR_CNR_VAL_SET/2)
#define    MOTOR_SWITCH_SET       1

#define  MARK_NUM       25

typedef enum
{	        
	LED12	        =((uint8_t)0),
	LED11	        ,
	LED22	        ,

	LED9	        ,
	LED20	        ,
	LED10	        ,

	LED18	        ,
	LED19	        ,
	LED17	        ,

	LED15	        ,
	LED16	        ,
	LED14	        ,

	LED23	        ,//
	LED21	        ,
	LED24	        ,

	LED7	        ,
	LED8	        ,
	LED6	        ,

	LED1	        ,
	LED5	        ,
	LED2	        ,

	LED13	        ,
	LED4	        ,
	LED3	        ,
}NUM_Typedef;

typedef enum
{
 SYS_Dmx	  =((uint8_t)0),
 SYS_Auto	  =((uint8_t)1),
 SYS_Soud	  =((uint8_t)2),
 SYS_Colo	  =((uint8_t)3),
 SYS_Slave	  =((uint8_t)4),
}SYS_Typedef;
extern SYS_Typedef SYS_State;
extern SYS_Typedef SYS_StateBuf;

typedef enum
{
 RUN_Reset	  =((uint8_t)0),
 RUN_Prepare  =((uint8_t)1),
 RUN_Normal	  =((uint8_t)2),
 RUN_AutoTest =((uint8_t)3),
}RUN_Typedef;
extern RUN_Typedef RUN_State;

typedef enum
{
	_Normal	      =((uint8_t)0),
	_Rotate180	  =((uint8_t)1),
	_Auto_On  	  =((uint8_t)2),
	_Auto_Off  	  =((uint8_t)3),
}Rota_Type;

typedef struct 	  //要更改的数据结构
{ 		
Rota_Type
	  Memory_Rota;
}Memory_TypeDef;

typedef struct {
	unsigned short LED_PWM[LED_TOTAL];
	unsigned short LED_PWM_[LED_TOTAL]; 
}LED_Type;
extern LED_Type Led;

extern uint8_t SysBuf,LCDDisplight,BackLightMark,TempErrorMark;
extern __IO uint8_t MotoResetTime,COLO_SIGNAL_MARK,Reset_Mark,Reset_Time,TempDispMark;
extern uint16_t KeyBuf1,KeyBuf2;

extern __IO uint32_t BackLightNum,Task_Sys_i,Task_XY_State,Task_COLOR_State;
extern __IO int32_t Temperature,TemperatureBuf[],TempCount;

extern __IO uint8_t Wireless,Reset_all,Reset_xy,Reset_gobo;

extern __IO uint8_t X_Reset,Y_Reset,Gobo_Reset,Color_Reset,Temp_Reset;

extern __IO uint32_t LocalTime;

extern Memory_TypeDef Memory_Struct;

extern const uint8_t RxMark[];
extern uint8_t RxBuf[];


void Task_TempUpdate(void);
void Task_LedAutoStob(void);
void Task_LedStob(void);
void Task_LedHcl(void);
void Task_LedFade(void);
void Task_LedFading(void);
void Task_LedDMXFade(void);
void Task_Menu(void);
void Task_DMXUpdate(void);
void Task_MotoXUpdate(void);
void Task_MotoYUpdate(void);
void Task_MotoZUpdate(void);

void Task_MotoY(void);
void Task_MotoZ(void);
void Task_Sys(void);
void Task_Colo(void);
void Task_Clear(void);

void WdtClear(void);


#endif



