#ifndef __MAIN_H__
#define __MAIN_H__
#include "stm32f10x.h"
#include "12864i_012.h"
#include "menu_fun.h"
#include "Font.h"
#include "boot1.h"
#include "SysTick.h"
#include "Pwm_T4.h"
#include "Pwm_T3.h"
#include "Pwm_T2.h"
#include "Pwm_T1.h"


#if S2_Enable

#define TIME1_CC_ENABLE    0
#define TIME1_CC1    TIM_IT_CC1
#define TIME1_CC2    0
#define TIME1_CC3    0
#define TIME1_CC4    0
#define TIME1_PWM_PERIOD       3600
#define TIME1_PWM_INIT         1800
#define TIME1_Polarity_ENABLE  0
#define TIME1_TIM_Prescaler	   0
#define TIME1_PinRemap		   0
#define TIME1_PWMCH1           0x01
#define TIME1_PWMCH2           0x02
#define TIME1_PWMCH3           0x04
#define TIME1_PWMCH4           0x08

#define TIME2_CC_ENABLE    0
#define TIME2_CC1    TIM_IT_CC1
#define TIME2_CC2    TIM_IT_CC2
#define TIME2_CC3    0
#define TIME2_CC4    0
#define TIME2_PWM_PERIOD       3600
#define TIME2_PWM_INIT         1800
#define TIME2_Polarity_ENABLE  0
#define TIME2_TIM_Prescaler	   0
#define TIME2_PinRemap		   0
#define TIME2_PWMCH1           0x01
#define TIME2_PWMCH2           0x02
#define TIME2_PWMCH3           0x04
#define TIME2_PWMCH4           0x08

#define TIME3_CC_ENABLE    0
#define TIME3_CC1    TIM_IT_CC1
#define TIME3_CC2    TIM_IT_CC2
#define TIME3_CC3    0
#define TIME3_CC4    0
#define TIME3_PWM_PERIOD       3600
#define TIME3_PWM_INIT         1800
#define TIME3_Polarity_ENABLE  1
#define TIME3_TIM_Prescaler	   0
#define TIME3_PinRemap		   0
#define TIME3_PWMCH1           0x01
#define TIME3_PWMCH2           0x02
#define TIME3_PWMCH3           0x04
#define TIME3_PWMCH4           0x08

#define TIME4_CC_ENABLE    0
#define TIME4_CC1    TIM_IT_CC1
#define TIME4_CC2    TIM_IT_CC2
#define TIME4_CC3    0
#define TIME4_CC4    0
#define TIME4_PWM_PERIOD       3600
#define TIME4_PWM_INIT         1800
#define TIME4_Polarity_ENABLE  0
#define TIME4_TIM_Prescaler	   0
#define TIME4_PinRemap		   0
#define TIME4_PWMCH1           0x01
#define TIME4_PWMCH2           0x02
#define TIME4_PWMCH3           0x04
#define TIME4_PWMCH4           0x08

#else

#define TIME1_CC_ENABLE    0
#define TIME1_CC1    TIM_IT_CC1
#define TIME1_CC2    0
#define TIME1_CC3    0
#define TIME1_CC4    0
#define TIME1_PWM_PERIOD       3600
#define TIME1_PWM_INIT         1800
#define TIME1_Polarity_ENABLE  0
#define TIME1_TIM_Prescaler	   0
#define TIME1_PinRemap		   0
#define TIME1_PWMCH1           0x01
#define TIME1_PWMCH2           0x02
#define TIME1_PWMCH3           0x04
#define TIME1_PWMCH4           0x08

#define TIME2_CC_ENABLE    0
#define TIME2_CC1    TIM_IT_CC1
#define TIME2_CC2    TIM_IT_CC2
#define TIME2_CC3    0
#define TIME2_CC4    0
#define TIME2_PWM_PERIOD       3600
#define TIME2_PWM_INIT         1800
#define TIME2_Polarity_ENABLE  0
#define TIME2_TIM_Prescaler	   0
#define TIME2_PinRemap		   0
#define TIME2_PWMCH1           0x01
#define TIME2_PWMCH2           0x02
#define TIME2_PWMCH3           0x04
#define TIME2_PWMCH4           0x08

#define TIME3_CC_ENABLE    0
#define TIME3_CC1    TIM_IT_CC1
#define TIME3_CC2    TIM_IT_CC2
#define TIME3_CC3    0
#define TIME3_CC4    0
#define TIME3_PWM_PERIOD       3600
#define TIME3_PWM_INIT         1800
#define TIME3_Polarity_ENABLE  0
#define TIME3_TIM_Prescaler	   0
#define TIME3_PinRemap		   1
#define TIME3_PWMCH1           0x01
#define TIME3_PWMCH2           0x02
#define TIME3_PWMCH3           0x04
#define TIME3_PWMCH4           0x08

#define TIME4_CC_ENABLE    0
#define TIME4_CC1    TIM_IT_CC1
#define TIME4_CC2    TIM_IT_CC2
#define TIME4_CC3    0
#define TIME4_CC4    0
#define TIME4_PWM_PERIOD       18000
#define TIME4_PWM_INIT         0
#define TIME4_Polarity_ENABLE  0
#define TIME4_TIM_Prescaler	   0
#define TIME4_PinRemap		   0
#define TIME4_PWMCH1           0x01
#define TIME4_PWMCH2           0x02
#define TIME4_PWMCH3           0x04
#define TIME4_PWMCH4           0x08

#endif

#define LCM_RS_Pin		GPIO_Pin_0
#define LCM_RS_Port		GPIOA
#define LCM_RW_Pin		GPIO_Pin_1
#define LCM_RW_Port		GPIOA
#define LCM_EN_Pin		GPIO_Pin_5
#define LCM_EN_Port		GPIOA
#define LCM_CS1_Pin		GPIO_Pin_6
#define LCM_CS1_Port	GPIOA
#define LCM_CS2_Pin		GPIO_Pin_7
#define LCM_CS2_Port	GPIOA
#define LCM_LIGHT_Pin	GPIO_Pin_1
#define LCM_LIGHT_Port	GPIOB 
#define LCM_REST_Pin	GPIO_Pin_0
#define LCM_REST_Port	GPIOB
#define LCM_D0_Pin		GPIO_Pin_8
#define LCM_D1_Pin		GPIO_Pin_9
#define LCM_D2_Pin		GPIO_Pin_10
#define LCM_D3_Pin		GPIO_Pin_11
#define LCM_D4_Pin		GPIO_Pin_12
#define LCM_D5_Pin		GPIO_Pin_13
#define LCM_D6_Pin		GPIO_Pin_14
#define LCM_D7_Pin		GPIO_Pin_15
#define LCM_Port	    GPIOA


#endif

#define  MARK_NUM       25

extern const uint8_t RxMark[];
extern uint8_t RxBuf[];



