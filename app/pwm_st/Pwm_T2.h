#ifndef __PWM_T2_H
#define	__PWM_T2_H

#include "stm32f10x.h"

extern __IO uint16_t T2_CCR1_Val ;
extern __IO uint16_t T2_CCR2_Val ;
extern __IO uint16_t T2_CCR3_Val ;
extern __IO uint16_t T2_CCR4_Val ;

void TIM2_PWM_Init(void);

#endif /* __PWM_OUTPUT_H */

