/******************** (C) COPYRIGHT 2012********************

**********************************************************************************/
#include "main.h"
//WIFI-DMX.COM
__IO uint16_t T2_CCR1_Val = 30000;
__IO uint16_t T2_CCR2_Val = 30000;
__IO uint16_t T2_CCR3_Val = 30000;
__IO uint16_t T2_CCR4_Val = 30000;

void TIM2_GPIO_Config(uint16_t TIM_CH) 
{
	uint16_t GPIO=0;
	GPIO_InitTypeDef GPIO_InitStructure;
	if(TIM_CH&0x01)GPIO|=GPIO_Pin_0;
	if(TIM_CH&0x02)GPIO|=GPIO_Pin_1;
	if(TIM_CH&0x04)GPIO|=GPIO_Pin_2;
	if(TIM_CH&0x08)GPIO|=GPIO_Pin_3;
	
	GPIO_InitStructure.GPIO_Pin   = GPIO;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
} 

void TIM2_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable the TIM2 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 12;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM2_PWM_Init(void)
{
#if TIME2_CC_ENABLE	
	TIM2_NVIC_Configuration();
	TIMx_TimeBase(TIM2,TIME2_CC1|TIME2_CC2|TIME2_CC3|TIME2_CC4);	
#else 
	TIM2_GPIO_Config(TIME2_PWMCH1|TIME2_PWMCH2|TIME2_PWMCH3|TIME2_PWMCH4);
	TIMx_PWMconfig(TIM2,TIME2_PWMCH1|TIME2_PWMCH2|TIME2_PWMCH3|TIME2_PWMCH4,TIME2_Polarity_ENABLE,TIME2_TIM_Prescaler);
#endif
}

/******************* (C) COPYRIGHT 2012*****END OF FILE*****/
