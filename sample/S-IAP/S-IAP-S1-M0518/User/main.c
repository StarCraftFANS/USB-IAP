/******************************************************************************
 * @file     APROM_main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/12/23 10:06a $
 * @brief    FMC APROM IAP sample for M0518 series MCU
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "main.h"

#define PLLCON_SETTING      CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK           50000000

#define MENU_TMR_INTERVAL      20 
#define ADC_TMR_INTERVAL       50 
#define SECOND_TMR_INTERVAL    1000 

#define ABS(x,y) ((x>y)?(x-y):(y-x))

SYS_Typedef SYS_State=SYS_Dmx;
SYS_Typedef SYS_StateBuf=SYS_Dmx;
RUN_Typedef RUN_State=RUN_Reset;
uint8_t SysBuf=0,LCDDisplight=1,BackLightMark=1,TempErrorMark=0;
uint16_t KeyBuf1=0xff,KeyBuf2=0xff;

__IO uint8_t Dimmer_State=0,MotoResetTime=0,COLO_SIGNAL_MARK=1,Reset_Mark=0,Reset_Time=0,TempDispMark=0;
__IO int32_t Temperature=31,TemperatureBuf[3]={30,30,30},TempCount=0; 
__IO uint8_t X_Reset=0,Y_Reset=0,Gobo_Reset=0,Color_Reset=0,Temp_Reset=0;

__IO uint32_t BackLightNum=1,Task_Sys_i=0,Task_XY_State=0,Task_COLOR_State=0;

__IO uint8_t Wireless=1,Reset_all=0,Reset_xy=1,Reset_gobo=1;

__IO uint32_t LocalTime = 0,MENUTimer=0,ADCTimer=0,SECONDTimer=0;

Memory_TypeDef Memory_Struct;
LED_Type Led;

const uint8_t RxMark[MARK_NUM]="WIFI-DMX.COM IAP-START S1";
uint8_t RxBuf[255];

int main()
{
	IAP_Wait();
}
/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
