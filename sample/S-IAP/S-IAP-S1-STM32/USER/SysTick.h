#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f10x.h"
/* CTRL TICKINT Mask */
#define CTRL_TICKINT_Set      ((u32)0x00000002)
#define CTRL_TICKINT_Reset    ((u32)0xFFFFFFFD)
extern __IO uint32_t Sys_Time;//Ê±¼ä
void SysTick_Init(__IO u32 DelayTime);

#endif /* __SYSTICK_H */
