#include <stdint.h>
#include <stddef.h>
#include "nvram.h"

static uint16_t _address;

void nvram_open(uint8_t mode) {
  _address = 0;
}

void nvram_close() {
}

void nvram_read(void *buff, size_t len) {
  _address += len;
}

void nvram_write(const void *buff, size_t len) {
  _address += len;
}
