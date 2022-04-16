
#ifndef _HC05_H
#define _HC05_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

#define GET_AT_RESPONSE_TMOUT		100		// [ms]

int BT_init(void);
_Bool BT_connected(void);
int BT_tryAT(void);

#endif
