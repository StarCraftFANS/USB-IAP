/******************** (C) COPYRIGHT 2012 ********************
**********************************************************************************/
#include "main.h"
//WIFI-DMX.COM
__IO uint16_t T3_CCR1_Val = 30000;
__IO uint16_t T3_CCR2_Val = 30000;
__IO uint16_t T3_CCR3_Val = 30000;
__IO uint16_t T3_CCR4_Val = 30000;

void TIM3_GPIO_Config(uint16_t TIM_CH) 
{
	uint16_t GPIO=0;
	GPIO_InitTypeDef GPIO_InitStructure;

#if TIME3_PinRemap
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
	if(TIM_CH&0x01)GPIO|=GPIO_Pin_6;
	if(TIM_CH&0x02)GPIO|=GPIO_Pin_7;
	if(TIM_CH&0x04)GPIO|=GPIO_Pin_8;
	if(TIM_CH&0x08)GPIO|=GPIO_Pin_9;

	GPIO_InitStructure.GPIO_Pin   = GPIO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
#else
    if(TIM_CH&0x01)GPIO|=GPIO_Pin_6;
	if(TIM_CH&0x02)GPIO|=GPIO_Pin_7;

	GPIO_InitStructure.GPIO_Pin   = GPIO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	if(TIM_CH&0x04)GPIO|=GPIO_Pin_0;
	if(TIM_CH&0x08)GPIO|=GPIO_Pin_1;

	GPIO_InitStructure.GPIO_Pin   = GPIO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif	
} 

void TIM3_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable the TIM2 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 12;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM3_PWM_Init(void)
{
#if TIME3_CC_ENABLE	
	TIM3_NVIC_Configuration();
	TIMx_TimeBase(TIM3,TIME3_CC1|TIME3_CC2|TIME3_CC3|TIME3_CC4);	
#else 
	TIM3_GPIO_Config(TIME3_PWMCH1|TIME3_PWMCH2|TIME3_PWMCH3|TIME3_PWMCH4);
	TIMx_PWMconfig(TIM3,TIME3_PWMCH1|TIME3_PWMCH2|TIME3_PWMCH3|TIME3_PWMCH4,TIME3_Polarity_ENABLE,TIME3_TIM_Prescaler);
#endif
}

/******************* (C) COPYRIGHT 2012 *****END OF FILE*****/
