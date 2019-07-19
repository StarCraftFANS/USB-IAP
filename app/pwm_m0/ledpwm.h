#ifndef _LEDPWM_H
#define _LEDPWM_H


//WIFI-DMX.COM

void PWM_SetOutputChannel(PWM_T *pwm,uint32_t u32ChannelNum, uint16_t Timerperiod,uint16_t PWMduty, uint8_t u8Switch,uint32_t Prescaler);
void BPWM_SetOutputChannel(BPWM_T *bpwm,uint32_t u32ChannelNum, uint16_t Timerperiod,uint16_t PWMduty, uint8_t u8Switch,uint32_t Prescaler);

void LedPwmInit(void);
void LedPwmOutput(uint16_t*p);

#endif



