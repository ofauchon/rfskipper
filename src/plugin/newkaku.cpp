#include <stdlib.h>

#include "plugin.hpp"

/*----------------------------------------------------------------------------*/

#define NewKAKU_RawSignalLength 130    // regular KAKU packet length
#define NewKAKUdim_RawSignalLength 146 // KAKU packet length including DIM bits
#define NewKAKU_TLength 28             // pulse length in 10µs
#define NewKAKU_mT (5 * NewKAKU_TLength) / 2 // us, approx. in between 1T and 4T

const char *const ppc_KAKUCmd[] = { "OFF", "ON", "ALLOFF", "ALLON" };

/*----------------------------------------------------------------------------*/

bool plugin004Rx(const Plugin *ps_plugin, RawSignal *ps_rawSignal,
                 char *pc_cmd) {
  uint32_t u32_bitStream;
  uint16_t *pu16_pulse;
  // bool b_dimPresent;
  int i_pulses;
  uint16_t p0;
  uint16_t p1;
  uint16_t p2;
  uint16_t p3;
  int8_t i_dim;
  int i_bit;
  int i;

  i_pulses = ps_rawSignal->u16_pulses - 1; // remove stop bit
  if (i_pulses != NewKAKU_RawSignalLength &&
      i_pulses != NewKAKUdim_RawSignalLength) {
    return false;
  }

  i = 2; // skip start bit
  // b_dimPresent = false;
  u32_bitStream = i_dim = 0;
  pu16_pulse = &ps_rawSignal->pu16_pulses[2];

  do {
    p0 = *pu16_pulse++;
    p1 = *pu16_pulse++;
    p2 = *pu16_pulse++;
    p3 = *pu16_pulse++;
    i += 4;

    if (p0 < NewKAKU_mT && p1 < NewKAKU_mT && p2 < NewKAKU_mT &&
        p3 > NewKAKU_mT) {
      i_bit = 0; // T,T,T,4T
    } else if (p0 < NewKAKU_mT && p1 > NewKAKU_mT && p2 < NewKAKU_mT &&
               p3 < NewKAKU_mT) {
      i_bit = 1; // T,4T,T,T
    } else if (p0 < NewKAKU_mT && p1 < NewKAKU_mT && p2 < NewKAKU_mT &&
               p3 < NewKAKU_mT) { // T,T,T,T should be on i=111 (bit 28)
      // b_dimPresent = true;
      if (i_pulses != NewKAKUdim_RawSignalLength) { // dim set but no dim bits
                                                    // present => invalid signal
        return false;
      }
      i_bit = 0;
    } else {
      return false; // Other pulse patterns are invalid within the AC KAKU
                    // signal.
    }
    if (i <= 130) { // all bits that belong to the 32-bit pulse sequence (32bits
                    // * 4 positions per bit + pulse/space for startbit)
      u32_bitStream = (u32_bitStream << 1) | i_bit;
    } else { // remaining 4 bits that set the dim level
      i_dim = (i_dim << 1) | i_bit;
    }
  } while (i < i_pulses);

  //===========================================================================
  // Prevent repeating signals from showing up
  //===========================================================================
  if (ps_rawSignal->u32_prevCRC == u32_bitStream &&
      ps_rawSignal->u32_startTime - ps_rawSignal->u32_prevTime < 1500) {
    return true; // already seen the RF packet recently
  }
  ps_rawSignal->u32_prevCRC = u32_bitStream;
  ps_rawSignal->u32_prevTime = ps_rawSignal->u32_startTime;

  i = (u32_bitStream >> 4) & 0x03;
  o_usart.printf("20;%02X;%s;ID=%08x;SWITCH=%x;CMD=%s;\n", u8_sequenceNumber++,
                 ps_plugin->pc_name, ((u32_bitStream) >> 6),
                 ((u32_bitStream) &0x0f) + 1, ppc_KAKUCmd[i]);

  return true;
}

/*----------------------------------------------------------------------------*/

bool plugin004Tx(const Plugin *ps_plugin, RawSignal *ps_rawSignal,
                 char *pc_cmd) {
  register uint32_t u32_bitStream;
  register uint16_t *pu16_pulse;
  uint32_t u32_mask;
  int i_address;
  int i_unit;
  int i_cmd;

  if (!getParamAsHex(&pc_cmd, &i_address) || !getParamAsHex(&pc_cmd, &i_unit) ||
      !getParamAsEnum(&pc_cmd, &i_cmd, ppc_KAKUCmd, 4)) {
    return false;
  }

  i_cmd &= 0x03;
  u32_bitStream = (i_address << 6) | ((i_unit - 1) & 0x000f) | (i_cmd << 4);

  pu16_pulse = ps_rawSignal->pu16_pulses;
  // start bit
  *pu16_pulse++ = NewKAKU_TLength;
  *pu16_pulse++ = NewKAKU_TLength * 10;

  for (u32_mask = 0x80000000; u32_mask; u32_mask >>= 1) {
    if (u32_bitStream & u32_mask) {
      *pu16_pulse++ = NewKAKU_TLength;
      *pu16_pulse++ = NewKAKU_TLength * 4;
      *pu16_pulse++ = NewKAKU_TLength;
      *pu16_pulse++ = NewKAKU_TLength;
    } else {
      *pu16_pulse++ = NewKAKU_TLength;
      *pu16_pulse++ = NewKAKU_TLength;
      *pu16_pulse++ = NewKAKU_TLength;
      *pu16_pulse++ = NewKAKU_TLength * 4;
    }
  }
  // stop bit
  *pu16_pulse++ = NewKAKU_TLength;

  // delay of 10ms
  ps_rawSignal->u16_interFrameGap = 1000;
  ps_rawSignal->u16_pulses = pu16_pulse - ps_rawSignal->pu16_pulses;
  ps_rawSignal->u8_repeats = 5; // Number of code retransmissions

  return true;
}

/*----------------------------------------------------------------------------*/
