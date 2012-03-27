#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>
#include <stm32f10x_flash.h>
#include "nvram.h"

#ifdef STM32F10X_HD
  #define FLASH_PAGE_SIZE  ((uint16_t)0x800)
#elif defined (STM32F10X_MD)
  #define FLASH_PAGE_SIZE  ((uint16_t)0x400)
#endif
const uint8_t flash_reserved_area[FLASH_PAGE_SIZE*2] __attribute__ ((aligned(FLASH_PAGE_SIZE))) = {0,};
#define FLASH_WRITE_ADDR ((uint32_t)&flash_reserved_area[0])
//#define FLASH_WRITE_ADDR                (0x08000000 + (uint32_t)FLASH_PAGE_SIZE * 63)    // use the last KB for storage

static char *_address;
static uint8_t _mode = NVRAM_MODE_CLOSED;
static uint32_t _flash_word;
static uint8_t _flash_word_idx;

void nvram_open(uint8_t mode) {
  if (_mode != NVRAM_MODE_CLOSED)
    nvram_close();
  _address = (char *)FLASH_WRITE_ADDR;
  _mode = mode;
  if (_mode == NVRAM_MODE_WRITE) {
    FLASH_Unlock();
    FLASH_ErasePage(FLASH_WRITE_ADDR);
    _flash_word = 0;
    _flash_word_idx = 0;
  }
}

void nvram_close() {
 if (_mode == NVRAM_MODE_WRITE) {
    if (_flash_word_idx != 0)
      FLASH_ProgramWord(FLASH_WRITE_ADDR + (uint32_t)_address, _flash_word);
    FLASH_Lock();
 }
 _mode = NVRAM_MODE_CLOSED;
}

void flash_send_byte(uint8_t b) {
  _flash_word = (_flash_word << 8) | b;
  _flash_word_idx++;
  if (_flash_word_idx == 4) {
    _flash_word_idx = 0;
    FLASH_ProgramWord(FLASH_WRITE_ADDR + (uint32_t)_address, _flash_word);
    _address += 4;
  }
}

void nvram_read(void *buff, size_t len) {
  memcpy(buff, _address, len);
  _address += len;
}

void nvram_write(const void *buff, size_t len) {
  char *_src_ptr = (char *)buff;
  while (len--) {
    flash_send_byte(*_src_ptr);
    _src_ptr++;
  }
}
