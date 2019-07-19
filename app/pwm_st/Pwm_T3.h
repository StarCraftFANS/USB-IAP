#ifndef __PWM_T3_H
#define	__PWM_T3_H
#include "stm32f10x.h"

extern __IO uint16_t T3_CCR1_Val ;
extern __IO uint16_t T3_CCR2_Val ;
extern __IO uint16_t T3_CCR3_Val ;
extern __IO uint16_t T3_CCR4_Val ;

void TIM3_PWM_Init(void);

#endif /* __PWM_OUTPUT_H */

