
#include "flash.h"

#define DEBUG_MSG 1
#include "dbgmsg.h"

void flash_write_word(uint32_t addr, uint32_t value)
{
	uint32_t page_error = 0;
	HAL_StatusTypeDef res = HAL_OK;
	FLASH_EraseInitTypeDef EraseInitStruct;
	
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);
	
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = ADDR_FLASH_PAGE_127;
	EraseInitStruct.NbPages = 1;
	
	res = HAL_FLASHEx_Erase(&EraseInitStruct, &page_error);
	
	if((res != HAL_OK) || (page_error != 0xFFFFFFFF)){
		errmsg("HAL_FLASHEx_Erase() failed (%d) page_error: 0x%08X\r\n", res, page_error);
		return;
	}
	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADDR_FLASH_PAGE_127, value);
	
	HAL_FLASH_Lock();
}

void flash_write_page(uint32_t addr, uint32_t *buf, size_t len)
{
	uint32_t page_error = 0;
	HAL_StatusTypeDef res = HAL_OK;
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t *ptr = buf;
	uint32_t size = (len > 1000) ? 1000 : len;
	uint32_t dest_addr = addr;
	
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);
	
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = ADDR_FLASH_PAGE_127;
	EraseInitStruct.NbPages = 1;
	
	res = HAL_FLASHEx_Erase(&EraseInitStruct, &page_error);
	
	if((res != HAL_OK) || (page_error != 0xFFFFFFFF)){
		errmsg("HAL_FLASHEx_Erase() failed (%d) page_error: 0x%08X\r\n", res, page_error);
		return;
	}
	
	for(int i = 0; i < size; i++){
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, dest_addr, *ptr);
		ptr++;
    dest_addr += 0x00000004;
	}
	
	HAL_FLASH_Lock();
}

uint8_t flash_read_byte(uint32_t addr)
{
	return (*(volatile uint8_t *)addr);
}

uint32_t flash_read_word(uint32_t addr)
{
	return (*(volatile uint32_t *)addr);
}

void flash_read_page(uint32_t addr, uint32_t *buf, size_t len)
{
	uint32_t size = (len > 1000) ? 1000 : len;
	uint32_t src_addr = addr;
	uint32_t *ptr = buf;
	
	for(int i = 0; i < size; i++){
		*ptr = *(volatile uint32_t *)src_addr;
		ptr++;
		src_addr += 0x00000004;
	}
}

