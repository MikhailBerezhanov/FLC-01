
#ifndef _TIMER_H
#define _TIMER_H

#include <stdint.h>
#include "stm32f1xx_hal.h"



// Indication LED pwm generation
void TIM1_PWM_Init(void);
void TIM1_PWM_ConfigAndStart(uint32_t percent);
void TIM1_PWM_Start(void);
void TIM1_PWM_Stop(void);

// Timer for smooth power-up \ power-down

// Virtual timers' names (bit masks)
typedef enum
{
	vtim1 = 1<<1,
	vtim2 = 1<<2,
	vtim3 = 1<<3,
}v_tim_name;

// General purpose timer - handles virtual timers
void TIM2_Init(uint16_t period);
void TIM2_set_period(uint16_t period);
void TIM2_set_virt_period(v_tim_name name, uint32_t period_ms);
void TIM2_ISR(void);
void TIM2_Start(void);
void TIM2_Stop(void);
_Bool TIM2_Expired(void);
int TIM2_virt_Expired(void);

// PWM channels generation for output LED load
void TIM3_PWM_Init(void);
// channel: TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3
// percent: 0..1000 [0.1%]
void TIM3_PWM_ConfigAndStart(uint32_t channel, uint32_t percent);
void TIM3_PWM_Config(uint32_t channel, uint32_t percent);
void TIM3_PWM_Start(uint32_t channel);
void TIM3_PWM_Stop(uint32_t channel);

void TIM4_Init(void);
void TIM4_ISR(void);
_Bool TIM4_Expired(void);

#endif	//_TIMER_H
