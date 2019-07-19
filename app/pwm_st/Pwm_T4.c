/******************** (C) COPYRIGHT 2012 ********************
TIM_OCMode_Timing    TIM输出比较时间模式
TIM_OCMode_Active    TIM输出比较主动模式
TIM_OCMode_Inactive  TIM输出比较非主动模式
TIM_OCMode_Toggle    TIM输出比较触发模式
TIM_OCMode_PWM1      TIM脉冲宽度调制模式1
TIM_OCMode_PWM2      TIM脉冲宽度调制模式2

 * 硬件连接：---------------------
 *          |  PB.06: (TIM4_CH1)  |
 *          |  PB.07: (TIM4_CH2)  |
 *      	  |  PB.08: (TIM4_CH3)  | 
 *    		  |  PB.09: (TIM4_CH4)  |
 *           ---------------------   
 * 描述    ：定时器TIM4产生四路PWM波输出。
 *           - PB.06: (TIM4_CH1)
 *           - PB.07: (TIM4_CH2)
 *           - PB.08: (TIM4_CH3)
 *           - PB.09: (TIM4_CH4) 
 *           TIM4 Channel1 duty cycle = (TIM4_CCR1/ TIM4_ARR)* 100 = 50%
 *           TIM4 Channel2 duty cycle = (TIM4_CCR2/ TIM4_ARR)* 100 = 37.5%
 *           TIM4 Channel3 duty cycle = (TIM4_CCR3/ TIM4_ARR)* 100 = 25%
 *           TIM4 Channel4 duty cycle = (TIM4_CCR4/ TIM4_ARR)* 100 = 12.5% 			
**********************************************************************************/
#include "main.h"
//WIFI-DMX.COM
__IO uint16_t T4_CCR1_Val = 30000;
__IO uint16_t T4_CCR2_Val = 30000;
__IO uint16_t T4_CCR3_Val = 30000;
__IO uint16_t T4_CCR4_Val = 30000;

void TIM4_GPIO_Config(uint16_t TIM_CH) 
{
	uint16_t GPIO=0;
	GPIO_InitTypeDef GPIO_InitStructure;
	if(TIM_CH&0x01)GPIO|=GPIO_Pin_6;
	if(TIM_CH&0x02)GPIO|=GPIO_Pin_7;
	if(TIM_CH&0x04)GPIO|=GPIO_Pin_8;
	if(TIM_CH&0x08)GPIO|=GPIO_Pin_9;
	
	GPIO_InitStructure.GPIO_Pin   = GPIO;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
} 

void TIME4_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable the TIM4 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 10;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM4_PWM_Init(void)
{
#if TIME4_CC_ENABLE	
	TIM4_NVIC_Configuration();
	TIMx_TimeBase(TIM4,TIME4_CC1|TIME4_CC2|TIME4_CC3|TIME4_CC4);	
#else 
	TIM4_GPIO_Config(TIME4_PWMCH1|TIME4_PWMCH2|TIME4_PWMCH3|TIME4_PWMCH4);
	TIMx_PWMconfig(TIM4,TIME4_PWMCH1|TIME4_PWMCH2|TIME4_PWMCH3|TIME4_PWMCH4,TIME4_Polarity_ENABLE,TIME4_TIM_Prescaler);
#endif
}

/******************* (C) COPYRIGHT 2012 *****END OF FILE*****/
