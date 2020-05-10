#include <stdlib.h>

#include "decoder.hpp"
#include "plugin.hpp"

/*----------------------------------------------------------------------------*/

#define TicPulse_RawSignalLength 260 // minimum TicPulse packet length
#define TicPulse_TLength 21          // pulse length in 10µs
#define CLOCK_PULSE TicPulse_TLength / 2

/*----------------------------------------------------------------------------*/

bool plugin255Rx(const Plugin *ps_plugin, RawSignal *ps_rawSignal,
                 char *pc_cmd) {
  (void) pc_cmd;

  uint32_t u32_bitStream;
  uint32_t u32_serial;
  uint32_t u32_counter;
  uint16_t u16_power;
  uint16_t *pu16_pulses;
  uint16_t u16_pulses;
  uint8_t pu8_rawSignal[19];
  uint8_t u8_size;
  int i_pulses;
  int i_timing;
  int i_bits;
  int i_bit;

  i_pulses = ps_rawSignal->u16_pulses;
  if (i_pulses < TicPulse_RawSignalLength) {
    return false;
  }

  pu16_pulses = ps_rawSignal->pu16_pulses;
  i_timing = *pu16_pulses++;
  u32_bitStream = 0;
  i_bits = 0;
  i_bit = 1; // start signal high

  for (;;) {
    while (CLOCK_PULSE < i_timing) {
      i_timing -= TicPulse_TLength;
      u32_bitStream = (u32_bitStream << 1) | i_bit;
      if (++i_bits == 24) {
        goto endRnzEncoding;
      }
    }

    i_timing = *pu16_pulses++;
    i_bit = !i_bit;
  }

endRnzEncoding:
  if (u32_bitStream != 0xaaaa9c) {
    return false;
  }

  u8_size = sizeof(pu8_rawSignal);
  u16_pulses = i_pulses - i_bits;
  i_bits =
    decodeManchester(pu16_pulses, &u16_pulses, TicPulse_TLength, pu8_rawSignal,
                     &u8_size, true, i_bit, CLOCK_PULSE >= i_timing);
  if (i_bits < 0) {
    return false;
  }

  u32_serial = pu8_rawSignal[6];
  u32_serial = (u32_serial << 8) + pu8_rawSignal[5];
  u32_serial = (u32_serial << 8) + pu8_rawSignal[4];
  u32_serial = (u32_serial << 8) + pu8_rawSignal[3];
  u32_counter = pu8_rawSignal[11];
  u32_counter = (u32_counter << 8) + pu8_rawSignal[10];
  u32_counter = (u32_counter << 8) + pu8_rawSignal[9];
  u32_counter = (u32_counter << 8) + pu8_rawSignal[8];
  u16_power = (pu8_rawSignal[17] << 8) + pu8_rawSignal[16];

  output(ps_plugin->pc_name, "ID=%02x%08x;COUNT=%x;POWER=%04x;",
         pu8_rawSignal[2], u32_serial, u32_counter, u16_power);

  return true;
}

/*----------------------------------------------------------------------------*/
