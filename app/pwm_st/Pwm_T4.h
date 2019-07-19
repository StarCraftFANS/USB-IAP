#ifndef __PWM_T4_H
#define	__PWM_T4_H

#include "stm32f10x.h"

extern __IO uint16_t T4_CCR1_Val ;
extern __IO uint16_t T4_CCR2_Val ;
extern __IO uint16_t T4_CCR3_Val ;
extern __IO uint16_t T4_CCR4_Val ;

void TIM4_PWM_Init(void);
#endif /* __PWM_OUTPUT_H */

