/*==============================================================================
>File name:  	FIFO.h
>Brief:         
                
>Author:      Berezhanov.M
>Date:        09.09.2018
>Version:     1.0
==============================================================================*/

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#ifndef __FIFO_H_
#define __FIFO_H_
/* Private define ------------------------------------------------------------*/			

/* Functional macroses -------------------------------------------------------*/
/**
  * @brief 	Creates static data queue FIFO 
  * @param	hfifo 	- handle of queue
						data_t	- elements' data type (char, int, etc.)
						size 		-	max number of elements to store in queue  
  * @retval	None  
 */
#define FIFO_CREATE( hfifo, data_t, size )\
  struct {\
    data_t buf[size];\
    uint32_t tail;\
    uint32_t head;\
  }hfifo

// clear fifo
#define FIFO_FLUSH( hfifo ) \
  {\
    hfifo.tail=0;\
    hfifo.head=0;\
  } 
	
// data element size in bytes
#define FIFO_ELEM_SIZE( hfifo )   ( sizeof(hfifo.buf[0]) )	
	
// max number of elements in queue
#define FIFO_SIZE( hfifo )      	( sizeof(hfifo.buf)/sizeof(hfifo.buf[0]) )

// number of available elements in queue
#define FIFO_AVAIL_COUNT( hfifo )	( (hfifo.head >= hfifo.tail) ? (hfifo.head - hfifo.tail) : \
																	  (FIFO_SIZE( hfifo ) - hfifo.tail + hfifo.head) )

// is fifo full?
#define FIFO_IS_FULL( hfifo )   	( FIFO_AVAIL_COUNT(hfifo) == FIFO_SIZE(hfifo) )

// is fifo overflow detected?
#define FIFO_IS_OVRFLOW( hfifo )	( )

// is fifo empty?
#define FIFO_IS_EMPTY( hfifo )  	( hfifo.tail == hfifo.head )
 
// size of free space in fifo
#define FIFO_SPACE( hfifo )     	( FIFO_SIZE(hfifo) - FIFO_AVAIL_COUNT(hfifo) )
 
/**
  * @brief 	Put element into queue 
  * @param	hfifo 	- handle of queue
						data		- element to put into queue 
  * @retval	None  
 */
#define FIFO_PUSH( hfifo, data ) \
  {\
		if (hfifo.head == FIFO_SIZE(hfifo)) hfifo.head = 0; \
    /*hfifo.buf[hfifo.head] = data; \*/memcpy(&hfifo.buf[hfifo.head], &(data), FIFO_ELEM_SIZE(hfifo)); \
    hfifo.head++; \
  } 
	
/**
* @brief 	Get element from queue and decreese counter of elements 
* @param	hfifo 	- handle of queue
					store		- buffer to store element from queue 
* @retval	None  
*/
#define FIFO_POP( hfifo, store ) \
  {\
		if (hfifo.tail == FIFO_SIZE(hfifo)) hfifo.tail = 0;\
		/*store = hfifo.buf[hfifo.tail];\*/memcpy(&(store), &hfifo.buf[hfifo.tail], FIFO_ELEM_SIZE(hfifo)); \
		hfifo.tail++; \
  }
	
// get first element from queue
#define FIFO_FRONT( hfifo ) 			( (hfifo.tail == FIFO_SIZE(hfifo)) ? hfifo.buf[0] : hfifo.buf[hfifo.tail] )
 
#endif /* FIFO_H_ */

