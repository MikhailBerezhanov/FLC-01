
#include "timer.h"
#include "config.h"

#define DEBUG_MSG 0
#include "dbgmsg.h"

static TIM_HandleTypeDef htim1;
static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim3;
static TIM_HandleTypeDef htim4;

static volatile _Bool tim2_expired = 0;		// Timer2 for smooth PWM start and stop
static volatile _Bool tim4_expired = 0;		// Timer4 for polling RTC every 5 sec

// Virtual timers
typedef struct 
{
	v_tim_name name;					// Timer's handle (name)
	volatile uint32_t cnt;		// Timer's counter
	volatile _Bool expired;		// Sets truw when timer's counter reaches 0
}virt_tim_t;

static volatile virt_tim_t virt_timers[] = {
	{vtim1, 0, 0},
	{vtim2, 0, 0},
	{vtim3, 0, 0},
};

static const size_t virt_timers_num = sizeof(virt_timers) / sizeof(virt_tim_t);

/**
* @brief TIM_Base MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
	if(htim_base->Instance==TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();
  }
  else if(htim_base->Instance==TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, TIM2_PreP, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  }
  else if(htim_base->Instance==TIM3)
  {
    __HAL_RCC_TIM3_CLK_ENABLE();
  }
  else if(htim_base->Instance==TIM4)
  {
    __HAL_RCC_TIM4_CLK_ENABLE();
    /* TIM4 interrupt Init */
    HAL_NVIC_SetPriority(TIM4_IRQn, TIM4_PreP, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
	if(htim->Instance==TIM1)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM1 GPIO Configuration    
    PB14     ------> TIM1_CH2N 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  if(htim->Instance==TIM3)
  { 
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM3 GPIO Configuration    
    PA6     ------> TIM3_CH1	-	OnBoard CH2
    PA7     ------> TIM3_CH2	-	OnBoard CH3
    PB0     ------> TIM3_CH3  -	OnBoard CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

/**
* @brief TIM_Base MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
	if(htim_base->Instance==TIM1)
  {
    __HAL_RCC_TIM1_CLK_DISABLE();
  }
  else if(htim_base->Instance==TIM2)
  {
    __HAL_RCC_TIM2_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  }
  else if(htim_base->Instance==TIM3)
  {
    __HAL_RCC_TIM3_CLK_DISABLE();
  }
  else if(htim_base->Instance==TIM4)
  {
    __HAL_RCC_TIM4_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
  }
}


static uint16_t CalculatePrescaler(uint32_t base_clk, uint32_t tick_us)
{
	uint32_t tick_freq = 1000000 / tick_us; // us -> Hz
	uint16_t prescaler = (base_clk / tick_freq) - 1; // less by 1 for timers register
	//dbgmsg("Base_clk: %u, tick_us: %u, Prescaler: %u\r\n", base_clk, tick_us, prescaler);
	return prescaler;
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
void TIM3_PWM_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

	// PCLK1 from APB1x2
	// Ttick = 20 us, Tpwm = 20us*100 = 2 ms
	// Fpwm = 1/2ms = 500 Hz
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = CalculatePrescaler((HAL_RCC_GetPCLK1Freq() * 2), 2/*20*/); //159 -  base 8 Mhz; 1439 - base 72 MHz
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;//99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    errmsg("HAL_TIM_Base_Init() failed\r\n");
		return;
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    errmsg("HAL_TIM_ConfigClockSource() failed\r\n");
		return;
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    errmsg("HAL_TIM_TIM3_PWM_Init() failed\r\n");
		return;
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    errmsg("HAL_TIMEx_MasterConfigSynchronization() failed\r\n");
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    errmsg("HAL_TIM_PWM_ConfigChannel() 1 failed\r\n");
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    errmsg("HAL_TIM_PWM_ConfigChannel() 2 failed\r\n");
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    errmsg("HAL_TIM_PWM_ConfigChannel() 3 failed\r\n");
  }
	
  HAL_TIM_MspPostInit(&htim3);
}

// channel: TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3
// percent: 0..1000 [0.1%]
void TIM3_PWM_Start(uint32_t channel)
{	
	HAL_TIM_PWM_Start(&htim3, channel);
}

void TIM3_PWM_Config(uint32_t channel, uint32_t percent)
{
	TIM_OC_InitTypeDef sConfigOC = {0};
	
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = percent;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	
	//HAL_TIM_PWM_Stop(&htim3, channel);
	
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, channel) != HAL_OK)
  {
		char str[32] = {0};
		switch(channel){
			case TIM_CHANNEL_1: strcpy(str, "1"); break;
			case TIM_CHANNEL_2: strcpy(str, "2"); break;
			case TIM_CHANNEL_3: strcpy(str, "3"); break;
			default: strcpy(str, "Unknown");
		}
    errmsg("HAL_TIM_PWM_ConfigChannel(%s) failed\r\n", str);
  }
}

void TIM3_PWM_ConfigAndStart(uint32_t channel, uint32_t percent)
{
	TIM_OC_InitTypeDef sConfigOC = {0};
	
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = percent;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	
	HAL_TIM_PWM_Stop(&htim3, channel);
	
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, channel) != HAL_OK)
  {
#if DEBUG_MSG
		char str[32] = {0};
		switch(channel){
			case TIM_CHANNEL_1: strcpy(str, "1"); break;
			case TIM_CHANNEL_2: strcpy(str, "2"); break;
			case TIM_CHANNEL_3: strcpy(str, "3"); break;
			default: strcpy(str, "Unknown");
		}
    errmsg("HAL_TIM_PWM_ConfigChannel(%s) failed\r\n", str);
#endif
  }
	
	HAL_TIM_PWM_Start(&htim3, channel);
}

void TIM3_PWM_Stop(uint32_t channel)
{
	HAL_TIM_PWM_Stop(&htim3, channel);
}

/**
  * @brief TIM1 Initialization Function
	*	@note	Uses complementary output CH2N (PB14)
  * @param None
  * @retval None
  */
void TIM1_PWM_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	// PCLK2 from APB2 (72MHz)
	// Ttick = 20 us, Tpwm = 20us*100 = 2 ms
	// Fpwm = 1/2ms = 500 Hz
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = CalculatePrescaler((HAL_RCC_GetPCLK1Freq() * 2), 20); //1439 - base 72 Mhz
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    errmsg("HAL_TIM_Base_Init() failed\r\n");
		return;
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    errmsg("HAL_TIM_Base_Init() failed\r\n");
		return;
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    errmsg("HAL_TIM_TIM_PWM_Init() failed\r\n");
		return;
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    errmsg("HAL_TIMEx_MasterConfigSynchronization() failed\r\n");
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 90;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    errmsg("HAL_TIM_PWM_ConfigChannel() failed\r\n");
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    errmsg("HAL_TIMEx_ConfigBreakDeadTime() failed\r\n");
  }

  HAL_TIM_MspPostInit(&htim1);	 
}

void TIM1_PWM_ConfigAndStart(uint32_t percent)
{
	TIM_OC_InitTypeDef sConfigOC = {0};
	
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	if (percent > 100) percent = 100;
  sConfigOC.Pulse = 100 - percent;	// LED connection is inverted, so we also invert pulse 
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
	
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    errmsg("HAL_TIM_PWM_ConfigChannel() failed\r\n");
  }
	
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
}

void TIM1_PWM_Start(void)
{
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
}

void TIM1_PWM_Stop(void)
{
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
}


/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
void TIM2_Init(uint16_t period)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

	// PCLK1 from APB1x2 
	// Ttick = 100us = 0.1 ms
	// Wanted: Ttim2 = Ttick*10 = 1 ms
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = CalculatePrescaler((HAL_RCC_GetPCLK1Freq() * 2), 100); //799 - base 8 MHz; 7199 - base 72 Mhz
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = period - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    errmsg("HAL_TIM_Base_Init() failed\r\n");
		return;
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    errmsg("HAL_TIM_ConfigClockSource() failed\r\n");
		return;
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    errmsg("HAL_TIMEx_MasterConfigSynchronization() failed\r\n");
  }

	//HAL_TIM_Base_Start_IT(&htim2);
}

// Changes period of hardware timer
void TIM2_set_period(uint16_t period)
{
	htim2.Init.Period = period - 1;
	
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    errmsg("HAL_TIM_Base_Init() failed\r\n");
}

// Changes period of software timer
void TIM2_set_virt_period(v_tim_name name, uint32_t period_ms)
{
	for(int i = 0; i < virt_timers_num; ++i){
		if(virt_timers[i].name == name){
			virt_timers[i].cnt = period_ms;
		}
	}
}

void TIM2_ISR(void)
{
	if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET)
  {
    if(__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) != RESET)
    {
			tim2_expired = 1;
			
			for(int i = 0; i < virt_timers_num; ++i){
				if(virt_timers[i].cnt){
					--virt_timers[i].cnt;
					//printf("tim '%d' cnt: %u\r\n", i, virt_timers[i].cnt);
					if(!virt_timers[i].cnt) virt_timers[i].expired = 1;
				}
			}
			
      __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
    }
  }
}

void TIM2_Start(void)
{
	HAL_TIM_Base_Start_IT(&htim2);
}

void TIM2_Stop(void)
{
	HAL_TIM_Base_Stop_IT(&htim2);
}

_Bool TIM2_Expired(void)
{
	if(tim2_expired){
		tim2_expired = 0;
		return 1;
	}
	
	return 0;
}

//
int TIM2_virt_Expired(void)
{
	int res = 0;
	
	for(int i = 0; i < virt_timers_num; ++i){
		if(virt_timers[i].expired){
			virt_timers[i].expired = 0;
			res |= virt_timers[i].name;
		}
	}
	
	return res;
}


/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
void TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

	// PCLK1 from APB1x2 (72MHz)
	// Ttick = 0.1 ms
	// Wanted: Ttim4 = 5 s
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = CalculatePrescaler((HAL_RCC_GetPCLK1Freq() * 2), 100); //799 - base 8 MHz; 7199 - base 72 Mhz
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 49999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    errmsg("HAL_TIM_Base_Init() failed\r\n");
		return;
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    errmsg("HAL_TIM_ConfigClockSource() failed\r\n");
		return;
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    errmsg("HAL_TIMEx_MasterConfigSynchronization() failed\r\n");
  }
	
	HAL_TIM_Base_Start_IT(&htim4);
}

void TIM4_ISR(void)
{
	if(__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) != RESET)
  {
    if(__HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_UPDATE) != RESET)
    {
			tim4_expired = 1;
      __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
    }
  }
}

_Bool TIM4_Expired(void)
{
	if(tim4_expired){
		tim4_expired = 0;
		return 1;
	}
	
	return 0;
}

