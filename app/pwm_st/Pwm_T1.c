/******************** (C) COPYRIGHT 2012 ********************
 *          |  PA.08: (TIM1_CH1)   R|
 *          |  PA.09: (TIM1_CH2)   G|
 *      	|  PA.10: (TIM1_CH3)  B| 
 *    		|  PA.11: (TIM1_CH4)  W|
 *           ---------------------   		
**********************************************************************************/
#include "main.h"
//WIFI-DMX.COM
__IO uint16_t T1_CCR1_Val = 30000;
__IO uint16_t T1_CCR2_Val = 61000;
__IO uint16_t T1_CCR3_Val = 30000;
__IO uint16_t T1_CCR4_Val = 30000;

void TIM1_GPIO_Config(uint16_t TIM_CH) 
{
	uint16_t GPIO=0;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*GPIOA Configuration: TIM1 channel 1 and 2 channel 3 and 4as alternate function push-pull */
	if(TIM_CH&0x01)GPIO|=GPIO_Pin_8;
	if(TIM_CH&0x02)GPIO|=GPIO_Pin_9;
	if(TIM_CH&0x04)GPIO|=GPIO_Pin_10;
	if(TIM_CH&0x08)GPIO|=GPIO_Pin_11;

	GPIO_InitStructure.GPIO_Pin   = GPIO;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;		    // 复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void TIMx_PWMconfig(TIM_TypeDef* TIMx,uint16_t TIM_CH ,uint16_t TIM_Polarity,uint16_t TIM_Prescaler)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	u16 CCR1_Val = TIME1_PWM_INIT;        
	u16 CCR2_Val = TIME1_PWM_INIT;
	u16 CCR3_Val = TIME1_PWM_INIT;
	u16 CCR4_Val = TIME1_PWM_INIT;
	u16 PERIOD_Val = TIME1_PWM_PERIOD;

	if(TIM1==TIMx){
	CCR1_Val = TIME1_PWM_INIT;        
	CCR2_Val = TIME1_PWM_INIT;
	CCR3_Val = TIME1_PWM_INIT;
	CCR4_Val = TIME1_PWM_INIT;
	PERIOD_Val = TIME1_PWM_PERIOD;
	}
	else if(TIM2==TIMx){
	CCR1_Val = TIME2_PWM_INIT;        
	CCR2_Val = TIME2_PWM_INIT;
	CCR3_Val = TIME2_PWM_INIT;
	CCR4_Val = TIME2_PWM_INIT;
	PERIOD_Val = TIME2_PWM_PERIOD;	
	}
	else if(TIM3==TIMx){
	CCR1_Val = TIME3_PWM_INIT;        
	CCR2_Val = TIME3_PWM_INIT;
	CCR3_Val = TIME3_PWM_INIT;
	CCR4_Val = TIME3_PWM_INIT;
	PERIOD_Val = TIME3_PWM_PERIOD;	
	}
	else {
	CCR1_Val = TIME4_PWM_INIT;        
	CCR2_Val = TIME4_PWM_INIT;
	CCR3_Val = TIME4_PWM_INIT;
	CCR4_Val = TIME4_PWM_INIT;	
	PERIOD_Val = TIME4_PWM_PERIOD;
	}
	/* Time base configuration */		 
	TIM_TimeBaseStructure.TIM_Prescaler = TIM_Prescaler;//TIMx时钟频率的预分频值 72MHZ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseStructure.TIM_Period = PERIOD_Val-1;//自动重装载寄存器周期值 6KHZ的PWM
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;//时钟分割值
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
	TIM_TimeBaseInit(TIMx,&TIM_TimeBaseStructure);//初始化TIMx的时间计数数据
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	if(TIM_CH&0x01)
	TIM_OC1Init(TIMx,&TIM_OCInitStructure); //设置通道1 
	TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);
	

	if(TIM_Polarity==1){
		TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
		if(TIM_CH&0x04)
		TIM_OC3Init(TIMx,&TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);

		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;  //当定时器计数值小于CCR1_Val时为高电平
	
		TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
		if(TIM_CH&0x02)
		TIM_OC2Init(TIMx,&TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);
	}
	else if(TIM_Polarity==2){
		TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
		if(TIM_CH&0x02)
		TIM_OC2Init(TIMx,&TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);

		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;  //当定时器计数值小于CCR1_Val时为高电平
		TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
		if(TIM_CH&0x04)
		TIM_OC3Init(TIMx,&TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);

	}
	else{
		TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
		if(TIM_CH&0x02)
		TIM_OC2Init(TIMx,&TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);

		TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
		if(TIM_CH&0x04)
		TIM_OC3Init(TIMx,&TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);
	
	}
	TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
	if(TIM_CH&0x08)
	TIM_OC4Init(TIMx,&TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIMx, ENABLE);  // 使能TIMx重载寄存器ARR//ARR预装载缓冲器
	
	/* TIM3 enable counter */
	TIM_Cmd(TIMx, ENABLE);                   //使能定时器1
	
	if(TIMx==TIM1)TIM_CtrlPWMOutputs(TIMx, ENABLE);	
}

void TIM1_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  /* Enable the TIM3 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 10;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void TIMx_TimeBase(TIM_TypeDef* TIMx,uint16_t TIM_IT)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  TIM_TimeBaseStructure.TIM_Period = 65536-1;
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;	   
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;	
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;

  TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

/* Prescaler configuration */
  TIM_PrescalerConfig(TIMx,0, TIM_PSCReloadMode_Immediate);

  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = T1_CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIMx, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Disable);

  /* Output Compare Timing Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = T1_CCR2_Val;

  TIM_OC2Init(TIMx, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Disable);

  /* Output Compare Timing Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = T1_CCR3_Val;

  TIM_OC3Init(TIMx, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Disable);

  /* Output Compare Timing Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = T1_CCR4_Val;

  TIM_OC4Init(TIMx, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Disable);

  TIM_ITConfig(TIMx, TIM_IT, ENABLE);

  TIM_Cmd(TIMx, ENABLE);
}

uint8_t TIMx_CC_IRQ(TIM_TypeDef* TIMx,__IO uint16_t *CR,uint16_t TIM_IT,uint16_t (*p)(void))
{
	if(TIM_GetITStatus(TIMx, TIM_IT) != RESET){
		TIM_ClearITPendingBit(TIMx, TIM_IT);
		*CR+=p();
		return 1;
	}
	return 0;
}
/*
void TIM1_CC_IRQHandler(void){
	if(TIMx_CC_IRQ(TIM1,&TIM1->CCR1,TIM_IT_CC1,MA_Run));
	else if(TIMx_CC_IRQ(TIM1,&TIM1->CCR2,TIM_IT_CC2,MB_Run));
	else if(TIMx_CC_IRQ(TIM1,&TIM1->CCR3,TIM_IT_CC3,MC_Run));
	else if(TIMx_CC_IRQ(TIM1,&TIM1->CCR4,TIM_IT_CC4,MD_Run));
}   
*/

void TIM1_PWM_Init(void)
{	
#if TIME1_CC_ENABLE	
	TIM1_NVIC_Configuration();
	TIMx_TimeBase(TIM1,TIME1_CC1|TIME1_CC2|TIME1_CC3|TIME1_CC4);	
#else 
	TIM1_GPIO_Config(TIME1_PWMCH1|TIME1_PWMCH2|TIME1_PWMCH3|TIME1_PWMCH4);
	TIMx_PWMconfig(TIM1,TIME1_PWMCH1|TIME1_PWMCH2|TIME1_PWMCH3|TIME1_PWMCH4,TIME1_Polarity_ENABLE,TIME1_TIM_Prescaler);
#endif
}


/******************* (C) COPYRIGHT 2012 *****END OF FILE*****/
