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
volatile uint16_t raw_ppm_data[RX_NUMBER_OF_CHANNELS];
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
__attribute__ ((always_inline))
void PPMCallback(uint8_t ch, uint16_t time, uint8_t state) {
  if (state) {
    ppm_edge_time[ch] = time;
  } else {
    uint16_t d_time = time - ppm_edge_time[ch];
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
__attribute__ ((always_inline))
void rx_ppm_serial_callback(uint16_t time) {
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

#if (RX == _DSM_) || (RX == _DSM2_)

#define SPEK_STATE_START     0x00
#define SPEK_STATE_PAYLOAD00 0x10
#define SPEK_STATE_PAYLOAD01 0x30

#if (RX == _DSM_)
  #define SPEK_CHAN_SHIFT  2       // Assumes 10 bit frames, that is 1024 mode.
  #define SPEK_CHAN_MASK   0x03    // Assumes 10 bit frames, that is 1024 mode.
#endif
#if (RX == _DSM2_)
  #define SPEK_CHAN_SHIFT  3       // Assumes 11 bit frames, that is 2048 mode.
  #define SPEK_CHAN_MASK   0x07    // Assumes 11 bit frames, that is 2048 mode.
#endif

inline void RX_Init() {
  SPK_serial_open(115200);
}

inline void RX_loop_50hz() {
}

inline void RX_loop_200hz() {
  static uint8_t state = SPEK_STATE_START;
  static uint8_t val0;
  uint8_t avail = SPK_serial_available();
  if (avail) {
    do {
      uint8_t val1 = SPK_serial_read();
      switch (state & 0xF0) {
        case SPEK_STATE_START:
          if (val1 == 0x01) {
            state = SPEK_STATE_PAYLOAD00;
            // ToDo: Read status/RSI
            #if defined(FAILSAFE)
            #endif
          } else
            val0  = val1;
          break;
        case SPEK_STATE_PAYLOAD00:
          val0   = val1;
          state |= SPEK_STATE_PAYLOAD01;
          break;
        case SPEK_STATE_PAYLOAD01:
          uint8_t C = 0x0F & (val0 >> SPEK_CHAN_SHIFT);
          if (C < 10) {
          #if (RX == _DSM_)
            rx_data.raw[C] = 988  +  (((uint16_t(val0 & SPEK_CHAN_MASK)  << 8)) | val1);
          #endif
          #if (RX == _DSM2_)
            rx_data.raw[C] = 988  + (((uint16_t(val0 & SPEK_CHAN_MASK)  << 8)) | val1) >> 1);
          #endif
          }
          if (((state++) & 0x0F) == 0x06)
            state = SPEK_STATE_START;
          else
            state = (state & 0x0F) | SPEK_STATE_PAYLOAD00;
          break;
      }
    } while ((avail = SPK_serial_available()));
  } else state = SPEK_STATE_START;
}
#endif


#if (RX == _NONE_)

inline void RX_Init() {};
inline void RX_loop_50hz() {};
inline void RX_loop_200hz() {};

#endif

