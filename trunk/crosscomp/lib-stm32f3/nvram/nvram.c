#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>
#include <stm32f30x_flash.h>
#include "nvram.h"

#define FLASH_PAGE_SIZE  ((uint16_t)0x800)

const uint8_t flash_reserved_area[FLASH_PAGE_SIZE*2] __attribute__ ((section(".nvram")));
#define FLASH_WRITE_ADDR ((uint32_t)&flash_reserved_area[0])

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
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    FLASH_ErasePage(FLASH_WRITE_ADDR);
    _flash_word = 0;
    _flash_word_idx = 0;
  }
}

void nvram_close() {
 if (_mode == NVRAM_MODE_WRITE) {
    if (_flash_word_idx != 0)
      FLASH_ProgramWord((uint32_t)_address, _flash_word);
    FLASH_Lock();
 }
 _mode = NVRAM_MODE_CLOSED;
}

void flash_send_byte(uint8_t b) {
  _flash_word = (_flash_word >> 8) | (b << 24);
  _flash_word_idx++;
  if (_flash_word_idx == 4) {
    FLASH_ProgramWord((uint32_t)_address, _flash_word);
    _flash_word_idx = 0;
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
