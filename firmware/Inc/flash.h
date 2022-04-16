
#ifndef _FLASH_H
#define _FLASH_H

#include <stdint.h>
#include "stm32f1xx_hal.h"


#define ADDR_FLASH_PAGE_127	((uint32_t)0x0801FC00)	/* Last Page 127, 1 Kbyte */


void flash_write_word(uint32_t addr, uint32_t value);
void flash_write_page(uint32_t addr, uint32_t *buf, size_t len);

uint8_t flash_read_byte(uint32_t addr);
uint32_t flash_read_word(uint32_t addr);
void flash_read_page(uint32_t addr, uint32_t *buf, size_t len);

#endif
