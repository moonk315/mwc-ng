/**
 * MultiWii NG 0.1 - 2012
 * RC Receiver support. ->(RX)
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
uint16_t raw_ppm_data[RX_NUMBER_OF_CHANNELS];
uint16_t ppm_edge_time[RX_NUMBER_OF_CHANNELS];

#define PPM_HYST 1

inline uint16_t get_raw_ppm_data_no_block(uint8_t ch) {
  uint16_t res = raw_ppm_data[ch];
  while (res != raw_ppm_data[ch]) res = raw_ppm_data[ch];
  return res >> 1;
}  

void filter_ppm_data() {
  for (uint8_t i = 0; i < RX_NUMBER_OF_CHANNELS; i++) { 
    uint16_t tmp = get_raw_ppm_data_no_block(i);
    if ((tmp > 900) && (tmp < 2200)) {
    #if (PPM_HYST > 0)
      if (tmp > rx_data.raw[i] + (PPM_HYST)) rx_data.raw[i] = tmp - (PPM_HYST - 1); 
      if (tmp < rx_data.raw[i] - (PPM_HYST)) rx_data.raw[i] = tmp + (PPM_HYST - 1);
    #else
      rx_data.raw[i] = tmp;
    #endif
    }  
  }  
}

inline void init_ppm() {
  for (uint8_t i = 0; i < RX_NUMBER_OF_CHANNELS; i++)
    raw_ppm_data[i] = 1500*2;
}    

#if (RX == _PPM_)
void PPMCallback(uint8_t ch, uint16_t time, uint8_t state) {
  if (state) {
    ppm_edge_time[ch] = time;
  } else {
    uint16_t d_time = __interval(ppm_edge_time[ch], time);
    raw_ppm_data[ch] = d_time;
  }  
}  

inline void RX_Init() {
  AttachPPM();
  init_ppm();
}  

inline void RX_loop_50hz() {
  filter_ppm_data();
}  

inline void RX_loop_200hz() {
}  
#endif


#if (RX == _PPM_SERIAL_)
inline void rx_ppm_serial_callback(uint16_t time) {
  static uint16_t last;
  static uint8_t ch;  
  uint16_t d_time = time - last;
  last = time;
  if(d_time > 3000*2) {
    ch = 0;
  } else {
    if(ch < RX_NUMBER_OF_CHANNELS)
      raw_ppm_data[ch++] = d_time;
  }
}

inline void RX_Init() {
  AttachPPMSerial();
  init_ppm();
}  

inline void RX_loop_50hz() {
  filter_ppm_data();
}  

inline void RX_loop_200hz() {
}  
#endif



