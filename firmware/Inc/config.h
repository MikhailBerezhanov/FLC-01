#ifndef	_CONFIG_H
#define	_CONFIG_H

// NVIC Priorities  0 – MAX, 15 – MIN
// We use PriorityGroup_4 in HAL_Init, so 0..15 Preemption priorities
/*
 ===============================================================================
    NVIC_PriorityGroup 	|  
  ============================================================================== 
   NVIC_PriorityGroup_0 | 0 		| 0 bits for pre-emption priority
                        | 0-15	|	4 bits for subpriority
  -------------------------------------------------- --------------------------- 
   NVIC_PriorityGroup_1 | 0-1 	| 1 bits for pre-emption priority
                        |	0-7 	|	3 bits for subpriority
  -------------------------------------------------- ---------------------------     
   NVIC_PriorityGroup_2 | 0-3 	| 2 bits for pre-emption priority
                        |	0-3  	| 2 bits for subpriority
  -------------------------------------------------- ---------------------------     
   NVIC_PriorityGroup_3 | 0-7   | 3 bits for pre-emption priority
                        |	0-1 	| 1 bits for subpriority
  -------------------------------------------------- ---------------------------    
   NVIC_PriorityGroup_4 | 0-15  | 4 bits for pre-emption priority
                        |	0  		|	0 bits for subpriority                       
  ==============================================================================
*/
#define BL_KEY_Pin 					GPIO_PIN_3
#define BL_KEY_GPIO_Port 		GPIOA
#define BL_CONN_Pin 				GPIO_PIN_4
#define BL_CONN_GPIO_Port 	GPIOA
#define BL_nRST_Pin 				GPIO_PIN_1
#define BL_nRST_GPIO_Port 	GPIOB

#define USART1_PreP		5

#define USART3_PreP		6

#define TIM2_PreP			13			// General purpose timer

#define TIM4_PreP			7				// RTC poll timer

#define RTC_PreP			7

#endif	//_CONFIG_H
