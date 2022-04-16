

#include "hc05.h"
#include "uart.h"
#include "config.h"

#define DEBUG_MSG 1
#include "dbgmsg.h"

#define HC05_RESET()		do{ \
													HAL_GPIO_WritePin(BL_nRST_GPIO_Port, BL_nRST_Pin, GPIO_PIN_RESET); \
													HAL_Delay(10); \
													HAL_GPIO_WritePin(BL_nRST_GPIO_Port, BL_nRST_Pin, GPIO_PIN_SET); \
												}while(0)

#define HC05_ENTER_AT()	do{ \
													HAL_GPIO_WritePin(BL_KEY_GPIO_Port, BL_KEY_Pin, GPIO_PIN_SET); \
	                        HC05_RESET(); \
												}while(0)
	
#define HC05_EXIT_AT()	do{ \
													HAL_GPIO_WritePin(BL_KEY_GPIO_Port, BL_KEY_Pin, GPIO_PIN_RESET); \
													HC05_RESET(); \
												}while(0)

int BT_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	/* Not Configuration mode for HC-05 module */
  HAL_GPIO_WritePin(BL_KEY_GPIO_Port, BL_KEY_Pin, GPIO_PIN_RESET);

  /* Not Reset state for HC-05 module */
  //HAL_GPIO_WritePin(BL_nRST_GPIO_Port, BL_nRST_Pin, GPIO_PIN_SET);
	
	/* Configure output GPIO pin : BL_KEY_Pin */
  GPIO_InitStruct.Pin = BL_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BL_KEY_GPIO_Port, &GPIO_InitStruct);

  /* Configure input GPIO pin : BL_CONN_Pin (Connection State) */
  GPIO_InitStruct.Pin = BL_CONN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BL_CONN_GPIO_Port, &GPIO_InitStruct);

  /* Configure output GPIO pin : BL_nRST_Pin 
  GPIO_InitStruct.Pin = BL_nRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BL_nRST_GPIO_Port, &GPIO_InitStruct);
	*/
	if (UART3_Init(9600) < 0){		// 38400
		errmsg("UART3_Init() failed\r\n");
		return -1;
	}
	
	return 0;
}


_Bool BT_connected(void)
{
	if ( GPIO_PIN_SET == HAL_GPIO_ReadPin(BL_CONN_GPIO_Port, BL_CONN_Pin) )
		return 1;
	
	return 0;
}


int BT_tryAT(void)
{
	uint32_t tickstart = 0U;
	char buf[64] = {0};
	uint32_t rec_len = 0;
	strcpy(buf, "AT\r\n");
	
	if (UART3_Init(38400) < 0){
		errmsg("UART3_Init(38400) failed\r\n");
		return -1;
	}
	
	dbgmsg("HC05: going to AT mode\r\n");
	HC05_ENTER_AT();
	dbgmsg("HC05: AT mode set\r\n");
	
	UART3_Send((uint8_t*)buf, strlen(buf));
	memset(buf, 0, sizeof(buf));
	
	// Wait for response
	tickstart = HAL_GetTick();	
	while(!UART3_Get((uint8_t*)buf, &rec_len))
	{
		if ((HAL_GetTick() - tickstart) > GET_AT_RESPONSE_TMOUT){
			errmsg("Timeout waiting response for 'AT'\r\n");
			HC05_EXIT_AT();
			return -2;           
		}
	}
	
	dbgmsg("HC05: %s\r\n", buf);
	
	HC05_EXIT_AT();
	return 0;
}



