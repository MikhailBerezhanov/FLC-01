
#include "uart.h"
#include "fifo.h"
#include "config.h"

#define DEBUG_MSG 0
#include "dbgmsg.h"

static UART_HandleTypeDef huart1;
static UART_HandleTypeDef huart3;

static FIFO_CREATE(uart1_fifo, char, UART_BUF_SIZE);
static FIFO_CREATE(uart3_fifo, char, UART_BUF_SIZE);

static volatile _Bool uart1_data_ready = 0;
static volatile _Bool uart3_data_ready = 0;
static volatile char uart1_prev_char = '\0';
static volatile char uart3_prev_char = '\0';

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART1)
  {
    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, USART1_PreP, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
  else if(huart->Instance==USART3)
  {
    /* Peripheral clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;//GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, USART3_PreP, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  }
}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  }
  else if(huart->Instance==USART3)
  {
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();
  
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  }
}



/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
int UART1_Init(uint32_t baudrate)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = baudrate;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    return -1;
  }
	
	FIFO_FLUSH(uart1_fifo);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	
	return 0;
}

void UART1_ISR(void)
{
	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET)
	{
		if (__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_RXNE) != RESET)
		{
			char tmp = (uint16_t)(huart1.Instance->DR & (uint16_t)0x01FF);

			if ((tmp != '\r') && (tmp != '\n')) { FIFO_PUSH(uart1_fifo, tmp); }
			
			if ((uart1_prev_char == '\r') && (tmp == '\n'))	uart1_data_ready = 1;
			
			uart1_prev_char = tmp;
			__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
		}
	}			
}

int UART1_Get(uint8_t *buf, uint32_t *len)
{
	if(uart1_data_ready){
		uint32_t cnt = FIFO_AVAIL_COUNT(uart1_fifo);
		dbgmsg("UART1_Get available '%u' bytes\r\n", cnt);
		
		uint8_t *ptr = buf;
		for (int i =0; i < cnt; i++){
			FIFO_POP(uart1_fifo, *ptr++);
		}
		*ptr = '\0';
		
		dbgmsg("UART1_Get '%u' bytes: '%s'\r\n", cnt, buf);
		
		if(len) *len = cnt;
		uart1_data_ready = 0;
		
		return 1;
	}
	
	return 0;
}

void UART1_Send(uint8_t *buf, uint16_t len)	
{
	HAL_UART_Transmit(&huart1, buf, len, HAL_MAX_DELAY);
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
int UART3_Init(uint32_t baudrate)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = baudrate;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    return -1;
  }
	
	FIFO_FLUSH(uart3_fifo);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
	
	return 0;
}

void UART3_ISR(void)
{
	if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET)
	{
		if (__HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_RXNE) != RESET)
		{
			
			char tmp = (uint16_t)(huart3.Instance->DR & (uint16_t)0x01FF);
		
			if ((tmp != '\r') && (tmp != '\n')) { FIFO_PUSH(uart3_fifo, tmp); }
			
			if ((uart3_prev_char == '\r') && (tmp == '\n'))	uart3_data_ready = 1;
			
			uart3_prev_char = tmp;
			__HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_RXNE);
		}
	}			
}

int UART3_Get(uint8_t *buf, uint32_t *len)
{
	if(uart3_data_ready){
		uint32_t cnt = FIFO_AVAIL_COUNT(uart3_fifo);
		
		uint8_t *ptr = buf;
		for (int i = 0; i < cnt; i++){
			FIFO_POP(uart3_fifo, *ptr++);
		}
		*ptr = '\0';
		
		dbgmsg("UART3_Get '%u' bytes:\r\n%s\r\n", cnt, buf);
		
		if(len) *len = cnt;
		uart3_data_ready = 0;
		
		return 1;
	}
	
	return 0;
}

void UART3_Send(uint8_t *buf, uint16_t len)	
{
	HAL_UART_Transmit(&huart3, buf, len, HAL_MAX_DELAY);
}



