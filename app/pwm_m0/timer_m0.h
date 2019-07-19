#ifndef _TIMER_M0_H
#define _TIMER_M0_H

typedef struct 
{
	void (*fpT0)();
	void (*fpT1)();
	void (*fpT2)();
	void (*fpT3)();
	volatile unsigned int 
	g_au32TMRINTCount[4];
}T; 
extern T Timer_m0;

void TIMER_Init(void);


#endif
