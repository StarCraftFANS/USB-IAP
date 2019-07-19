/******************** (C) COPYRIGHT 2012 ********************
 * 文件名  ：SysTick.c
 * 描述    ：SysTick 系统滴答时钟10us中断函数库,中断时间可自由配置，
 *           常用的有 1us 10us 1ms 中断。         
**********************************************************************************/
//WIFI-DMX.COM
#include "SysTick.h"
/* ---------------------- SysTick registers bit mask -------------------- */
__IO uint32_t Sys_Time=0;//时间
/*
 * 函数名：SysTick_Init
 * 描述  ：启动系统滴答定时器 SysTick
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用 
 */
void SysTick_Init(__IO u32 DelayTime)
{
	/* SystemFrequency / 1000    1ms中断一次
	 * SystemFrequency / 100000	 10us中断一次
	           72000000 / 100000
			   8000000 /Motor_Acc_Table[0]
	 * SystemFrequency / 1000000 1us中断一次
	 Motor_Speed_Up_Table[Speed_Up_Num]
	 调整加速时间快慢
	 72000 1ms
	 7200  100us
	 720   10us
	 72    1us 
	 (72000 ~ 720)
	 */			  
if (SysTick_Config(DelayTime))
    { 
	  while (1);
	}
   SysTick->CTRL &= CTRL_TICKINT_Reset;//失能
   //SysTick->CTRL |= CTRL_TICKINT_Set;	//使能
}
/******************* (C) COPYRIGHT 2012 *****END OF FILE****/
