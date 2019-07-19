#include "main.h"

T Timer_m0={Task_Null,Task_Null,Task_Null,Task_Null,0,0,0,0};

void TIMER_Init(void)
{
#if TIMER0_ENABLE
	CLK_EnableModuleClock(TMR0_MODULE);
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HCLK, 0);
	/* Configure Timer0 settings and for event counter application */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);
    TIMER_SET_CMP_VALUE(TIMER0, TIMER0_CMP);//1ms
    TIMER_EnableInt(TIMER0);
	NVIC_EnableIRQ(TMR0_IRQn);
	TIMER_Start(TIMER0);
#endif
#if TIMER1_ENABLE
	CLK_EnableModuleClock(TMR1_MODULE);
	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1_S_HCLK, 0);
	/* Configure Timer1 settings and for event counter application */
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1);
    TIMER_SET_PRESCALE_VALUE(TIMER1, 0);
    TIMER_SET_CMP_VALUE(TIMER1, TIMER1_CMP);
    TIMER_EnableInt(TIMER1);
	NVIC_EnableIRQ(TMR1_IRQn);
	TIMER_Start(TIMER1);
#endif
#if TIMER2_ENABLE
	CLK_EnableModuleClock(TMR2_MODULE);
	CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2_S_HCLK, 0);
	/* Configure Timer2 settings and for event counter application */
    TIMER_Open(TIMER2, TIMER_PERIODIC_MODE, 1);
    TIMER_SET_PRESCALE_VALUE(TIMER2, 0);
    TIMER_SET_CMP_VALUE(TIMER2, TIMER2_CMP);
    TIMER_EnableInt(TIMER2);
	NVIC_EnableIRQ(TMR2_IRQn);
	TIMER_Start(TIMER2);
#endif
#if TIMER3_ENABLE
	CLK_EnableModuleClock(TMR3_MODULE);
	CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3_S_HCLK, 0);
	/* Configure Timer3 settings and for event counter application */
    TIMER_Open(TIMER3, TIMER_PERIODIC_MODE, 1);
    TIMER_SET_PRESCALE_VALUE(TIMER3, 0);
    TIMER_SET_CMP_VALUE(TIMER3, TIMER3_CMP);
    TIMER_EnableInt(TIMER3);
	NVIC_EnableIRQ(TMR3_IRQn);
	TIMER_Start(TIMER3);
#endif
}


void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);
		(Timer_m0.fpT0)();
        Timer_m0.g_au32TMRINTCount[0]++;
    }
}

void TMR1_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        /* Clear Timer1 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER1);
		(Timer_m0.fpT1)();
        Timer_m0.g_au32TMRINTCount[1]++;
    }
}

void TMR2_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER2) == 1)
    {
        /* Clear Timer2 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER2);
		(Timer_m0.fpT2)();
        Timer_m0.g_au32TMRINTCount[2]++;
    }
}

void TMR3_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER3) == 1)
    {
        /* Clear Timer3 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER3);
		(Timer_m0.fpT3)();
        Timer_m0.g_au32TMRINTCount[3]++;
    }
}		  
