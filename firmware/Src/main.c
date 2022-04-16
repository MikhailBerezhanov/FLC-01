
#include <stdio.h>
#include <string.h>

#include "stm32f1xx_hal.h"
#include "uart.h"
#include "timer.h"
#include "app.h"
#include "rtc.h"
#include "hc05.h"

#define DEBUG_MSG 1
#include "dbgmsg.h"

#define CONFIG_BT 	0
#define BT_ENABLED	0
#define WDT_ENABLED	0

#if WDT_ENABLED
	#define WDT_Init() 			IWDG_Init()
	#define WDT_Refresh()		HAL_IWDG_Refresh(&hiwdg)
#else	
	#define WDT_Init()
	#define WDT_Refresh()
#endif


/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

/* Private function prototypes -----------------------------------------------*/
//static int SystemClock_Config_PLL72(void);
static int SystemClock_Config_HSE8(void);
static void GPIO_Init(void);
static void IWDG_Init(void);
static void Sleep(void);

int main(void)
{	
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
	GPIO_Init();
  /* Configure the system clock */
  int res = SystemClock_Config_HSE8();
	rt_t rt;

  /* Initialize all configured peripherals */
	UART1_Init(115200);		// debug port	
	if(res < 0) errmsg("SystemClock_Config() failed (%d)\r\n", res);
 
#if BT_ENABLED
	BT_init();
#endif
  TIM3_PWM_Init();
  WDT_Init();
  RTC_Init(RTC_POLL_PERIOD);
	TIM1_PWM_Init();
	TIM1_PWM_ConfigAndStart(20);	// Indication LED

	dbgmsg("~ Fitolamp v.%s (%s %s) ~\r\n", APP_VERSION, __DATE__, __TIME__);

	app_init();
	
	// Virtual timers' numbers storage
	int tim_num = 0;

  while (1)
  {
		WDT_Refresh();

#if CONFIG_BT
		config_BT();
#else
		serve_input();
#endif
				
		if(RTC_TimeToPoll()){
			RTC_Get_DateTime(&rt);
			check_work_periods(&rt);
			adjust_led_power(&rt);
		}
		
		if((tim_num = TIM2_virt_Expired()) > 0) {
			//dbgmsg("Timer %d expired\r\n", tim_num);
			smooth_power_up_handler(tim_num);
		}
		
		Sleep();
  }
}

// Sleep until any interrupt occures
void Sleep(void)
{
	RTC_Enable_SecIRQ();
	// Reset WatchDog Timer before sleep
	WDT_Refresh();
	// Disable SysTick interrupt
	HAL_SuspendTick();
	// Sleep until any interrupt will rise
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	
	// After interrupt rises will be here
	
	// Enable SysTick interrupt
	HAL_ResumeTick();
}

void HAL_MspInit(void)
{
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled */
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
}

#if 0
/**
  * @brief System Clock Configuration with PLL from HSE to 72 MHz
  * @retval None
  */
static int SystemClock_Config_PLL72(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) 
		return -1;

  /** Initializes the CPU, AHB and APB busses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
		return -2;
	
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
		return -3;
	
	return 0;
}
#endif

/**
  * @brief System Clock Configuration with HSE to 8 MHz
  * @retval None
  */
static int SystemClock_Config_HSE8(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  //RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  //RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) return -1;
  /** Initializes the CPU, AHB and APB busses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) return -2;
 /*
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) return -3;
  */
	return 0;
}

/**
  * @brief IWDG Initialization Function
	* @note	 IWDG base clk is 40 kHz. Ftick = base_clk \ Prescaler
	*				 Treload = Ttick * Reload. Reload MAX is 4095
  */
static void IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;	// * Ttick = 0.0008 s
  hiwdg.Init.Reload = 3749;									// * Treload = Ttick * Reload = 3 s 
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    errmsg("HAL_IWDG_Init() failed\r\n");
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  //__HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	
	// PB14 - Indication LED
	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	// PA6, PA7, PB0 - PWM ouput
	GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); */
}

