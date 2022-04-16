
#ifndef _RTC_H
#define _RTC_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

typedef struct
{
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t date;
	uint8_t month;
	uint16_t year;
}rt_t;


void RTC_Init(int poll_period);
void RTC_Enable_SecIRQ(void);
void RTC_ISR(void);
_Bool RTC_TimeToPoll(void);

void RTC_Get_DateTime(rt_t *rt);
void RTC_Get_Time(RTC_TimeTypeDef *sTime);
void RTC_Get_Date(RTC_DateTypeDef *sDate);

void RTC_Set_DateTime(rt_t *rt);
void RTC_Set_Time(uint8_t hour, uint8_t minute, uint8_t second);
void RTC_Set_Date(uint8_t date, uint8_t weekday, uint8_t month, uint16_t year);

#endif	//_RTC_H
