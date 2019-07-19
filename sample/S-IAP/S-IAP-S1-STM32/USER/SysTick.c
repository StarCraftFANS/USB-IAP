/******************** (C) COPYRIGHT 2012 ********************
 * �ļ���  ��SysTick.c
 * ����    ��SysTick ϵͳ�δ�ʱ��10us�жϺ�����,�ж�ʱ����������ã�
 *           ���õ��� 1us 10us 1ms �жϡ�         
**********************************************************************************/
//WIFI-DMX.COM
#include "SysTick.h"
/* ---------------------- SysTick registers bit mask -------------------- */
__IO uint32_t Sys_Time=0;//ʱ��
/*
 * ��������SysTick_Init
 * ����  ������ϵͳ�δ�ʱ�� SysTick
 * ����  ����
 * ���  ����
 * ����  ���ⲿ���� 
 */
void SysTick_Init(__IO u32 DelayTime)
{
	/* SystemFrequency / 1000    1ms�ж�һ��
	 * SystemFrequency / 100000	 10us�ж�һ��
	           72000000 / 100000
			   8000000 /Motor_Acc_Table[0]
	 * SystemFrequency / 1000000 1us�ж�һ��
	 Motor_Speed_Up_Table[Speed_Up_Num]
	 ��������ʱ�����
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
   SysTick->CTRL &= CTRL_TICKINT_Reset;//ʧ��
   //SysTick->CTRL |= CTRL_TICKINT_Set;	//ʹ��
}
/******************* (C) COPYRIGHT 2012 *****END OF FILE****/
