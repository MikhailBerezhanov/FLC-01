
#include "rtc.h"
#include "config.h"

#define DEBUG_MSG 0
#include "dbgmsg.h"

static RTC_HandleTypeDef hrtc;
static int rtc_poll_period = 0;
static volatile int sec_cnt = 0;
static volatile _Bool rtc_ready = 0;

/**
* @brief RTC MSP Initialization
* This function configures the hardware resources used in this example
* @param hrtc: RTC handle pointer
* @retval None
*/
void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
{
  if(hrtc->Instance==RTC)
  {
    HAL_PWR_EnableBkUpAccess();
    /* Enable BKP CLK enable for backup registers */
    __HAL_RCC_BKP_CLK_ENABLE();
    /* Peripheral clock enable */
    __HAL_RCC_RTC_ENABLE();
		
		HAL_NVIC_SetPriority(RTC_IRQn, RTC_PreP, 0);
    HAL_NVIC_EnableIRQ(RTC_IRQn);
  }
}

/**
* @brief RTC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hrtc: RTC handle pointer
* @retval None
*/
void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc)
{
  if(hrtc->Instance==RTC)
  {
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
		
		 HAL_NVIC_DisableIRQ(RTC_IRQn);
  }
}

void RTC_ISR(void)
{
	HAL_RTCEx_RTCIRQHandler(&hrtc);
}

void HAL_RTCEx_RTCEventErrorCallback(RTC_HandleTypeDef *hrtc)
{
	errmsg("RTC Error occured\r\n");
	hrtc->State = HAL_RTC_STATE_READY;
}

// Seconds interrupt callback
void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc)
{
	if(sec_cnt >= (rtc_poll_period-1)){ 
		rtc_ready = 1;
		sec_cnt = 0;
	}
	else sec_cnt++;
}

_Bool RTC_TimeToPoll(void)
{
	if(rtc_ready){
		rtc_ready = 0;
		return 1;
	}
	return 0;
}

void RTC_Enable_SecIRQ(void)
{
	HAL_RTCEx_SetSecond_IT(&hrtc);
	HAL_NVIC_SetPriority(RTC_IRQn, RTC_PreP, 0);
  HAL_NVIC_EnableIRQ(RTC_IRQn);
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
void RTC_Init(int poll_period)
{
	/* RTC CLK source init*/
	/*Initialize RTC and LSE/LSI (if not initialized yet) */
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  _Bool LSE_CLK = 1;  

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;	
	
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)			// LSE init failed
	{
		errmsg("LSE Init failed, Trying to init LSI\r\n");
    LSE_CLK = 0;
		RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI;
		RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
		RCC_OscInitStruct.LSIState = RCC_LSI_ON;
		PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;

		if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)		// LSI init failed
		{
			errmsg("Failed to init RTC CLK source\r\n");
			return;
		}
	}	

  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)		
  {
    errmsg("Failed to Init RTC PeriphClk\r\n");
		return;
	}

	dbgmsg("RTC CLK source:\t%s\r\n", LSE_CLK ? "LSE" : "LSI");
	
  /** Initialize RTC */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK){
    errmsg("HAL_RTC_Init() failed\r\n"); 
		return;
	}	
	
	rtc_poll_period = poll_period;
	RTC_Enable_SecIRQ();
}


void RTC_Get_Time(RTC_TimeTypeDef *sTime)
{
	uint8_t hal_status = HAL_RTC_GetTime(&hrtc, sTime, RTC_FORMAT_BIN);	
	if(hal_status != HAL_OK)
		errmsg("HAL_RTC_GetTime() failed (%u)\r\n", hal_status);			
}

void RTC_Get_Date(RTC_DateTypeDef *sDate)
{
	uint8_t hal_status = HAL_RTC_GetDate(&hrtc, sDate, RTC_FORMAT_BIN);
	if(hal_status != HAL_OK)
		errmsg("HAL_RTC_GetDate() failed (%u)\r\n", hal_status);	
}

void RTC_Get_DateTime(rt_t *rt)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	
	uint8_t hal_status = HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	if(hal_status != HAL_OK){
	  errmsg("HAL_RTC_GetTime() failed (%u)\r\n", hal_status);
		return;
	}		
	hal_status = HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	if(hal_status != HAL_OK)
		errmsg("HAL_RTC_GetDate() failed (%u)\r\n", hal_status);
		
	rt->sec = sTime.Seconds;
	rt->min = sTime.Minutes; 
	rt->hour = sTime.Hours;
	rt->date = sDate.Date;
	rt->month = sDate.Month;
	rt->year = sDate.Year + 1980;
}

void RTC_Set_DateTime(rt_t *rt)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	
	sTime.Hours = rt->hour;
	sTime.Minutes = rt->min;
	sTime.Seconds = rt->sec;
	sDate.Date = rt->date;
	sDate.Month = rt->month;
	sDate.Year = (rt->year - 1980);
	
	uint8_t hal_status = HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);	
	if(hal_status != HAL_OK)
		errmsg("HAL_RTC_SetTime() failed (%u)\r\n", hal_status);
	
	hal_status = HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	if(hal_status != HAL_OK)
		errmsg("HAL_RTC_SetDate() failed (%u)\r\n", hal_status);
}

void RTC_Set_Time(uint8_t hour, uint8_t minute, uint8_t second)
{
	RTC_TimeTypeDef sTime;
	sTime.Hours = hour;
	sTime.Minutes = minute;
	sTime.Seconds = second;
	
	uint8_t hal_status = HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);	
	if(hal_status != HAL_OK)
		errmsg("HAL_RTC_SetTime() failed (%u)\r\n", hal_status);			
}

void RTC_Set_Date(uint8_t date, uint8_t weekday, uint8_t month, uint16_t year)
{
	RTC_DateTypeDef sDate;
	sDate.Date = date;
	sDate.WeekDay = weekday;
	sDate.Month = month;
	sDate.Year = (year - 1980);		//calculate the year since 1980
	
	uint8_t hal_status = HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	if(hal_status != HAL_OK)
		errmsg("HAL_RTC_SetDate() failed (%u)\r\n", hal_status);
}


















