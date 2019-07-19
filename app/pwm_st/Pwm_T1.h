#ifndef __PWM_T1_H
#define	__PWM_T1_H

#include "stm32f10x.h"

extern __IO uint16_t T1_CCR1_Val ;
extern __IO uint16_t T1_CCR2_Val ;
extern __IO uint16_t T1_CCR3_Val ;
extern __IO uint16_t T1_CCR4_Val ;

void TIMx_PWMconfig(TIM_TypeDef* TIMx,uint16_t TIM_CH ,uint16_t TIM_Polarity,uint16_t TIM_Prescaler);
void TIMx_TimeBase(TIM_TypeDef* TIMx,uint16_t TIM_IT);
uint8_t TIMx_CC_IRQ(TIM_TypeDef* TIMx,__IO uint16_t *CR,uint16_t TIM_IT,uint16_t (*p)(void));

void TIM1_PWM_Init(void);

#endif /* __PWM_OUTPUT_H */

