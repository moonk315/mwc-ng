/**
 * MultiWii NG 0.1 - 2012
 * Permanent storage for onboard parameters
 *
 * This program is free software: you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, or 
 * (at your option) any later version. 
 *
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
 * GNU General Public License for more details. 
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>. 
 */

#include <avr/eeprom.h>

static uint8_t ver_magic = 10;

typedef struct eep_entry eep_entry_t;
struct eep_entry{
  void *var;
  size_t size;
};

eep_entry_t eep_entry[] = {
  {&ver_magic, sizeof(ver_magic)},
  {&pid.setup, sizeof(pid.setup)},
  {&input.setup, sizeof(input.setup)}, 
  {&mavlink_system.sysid, sizeof(mavlink_system.sysid)}, 
  {&imu.acc_offset, sizeof(imu.acc_offset)}, 
  {&imu.mag_offset, sizeof(imu.mag_offset)}, 
  {&batt_voltage_scaler, sizeof(batt_voltage_scaler)}, 
};  
const uint8_t eep_entry_cnt = sizeof(eep_entry) / sizeof(eep_entry_t);

void eeprom_update_block (const void *__src, void *__dst, size_t __n) {
  char *_dst_ptr = (char *)__dst;
  char *_src_ptr = (char *)__src;
  while (__n--) {
    uint8_t b = eeprom_read_byte((const uint8_t *)_dst_ptr);
    if (b != *_src_ptr) eeprom_write_byte((uint8_t *)_dst_ptr, *_src_ptr);
    _dst_ptr++;
    _src_ptr++;
  }
}  

void read_storage() {
  uint16_t _address = eep_entry[0].size;
  for(uint8_t i = 1; i < eep_entry_cnt; i++) { 
    eeprom_read_block(eep_entry[i].var, (void*)(_address), eep_entry[i].size); _address += eep_entry[i].size;
  }  
  build_expo_table();
}

void write_storage() {
  uint16_t _address = 0;
  for(uint8_t i=0; i<eep_entry_cnt; i++) {
    eeprom_update_block(eep_entry[i].var, (void*)(_address), eep_entry[i].size); _address += eep_entry[i].size;
  } 
}

void Storage_Init() {
  uint8_t test_val; eeprom_read_block((void*)&test_val, (void*)(0), sizeof(test_val));
  if (test_val != ver_magic) 
    write_storage();
  else
    read_storage();  
}

