#include <stdlib.h>

#include "plugin.hpp"

/*----------------------------------------------------------------------------*/

#define EV1527_RawSignalLength 49 // regular packet length
#define EV1527_PULSEMIN 420 / RAWSIGNAL_SAMPLE_RATE
#define EV1527_PULSEMAX 1250 / RAWSIGNAL_SAMPLE_RATE
#define EV1527_PULSEMID ((EV1527_PULSEMIN + EV1527_PULSEMAX) / 2)

/*----------------------------------------------------------------------------*/

bool plugin061Rx(const Plugin *ps_plugin, RawSignal *ps_rawSignal,
                 char *pc_cmd) {
  (void) pc_cmd;

  uint32_t u32_bitStream;
  uint16_t *pu16_pulse;
  int i_pulses;
  uint16_t p0;
  uint16_t p1;

  i_pulses = ps_rawSignal->u16_pulses;
  if (i_pulses != EV1527_RawSignalLength) {
    return false;
  }

  u32_bitStream = 0;
  pu16_pulse = ps_rawSignal->pu16_pulses;

  while (i_pulses >= 2) {
    p0 = *pu16_pulse++;
    p1 = *pu16_pulse++;
    i_pulses -= 2;

    u32_bitStream <<= 1;
    if (p1 > EV1527_PULSEMID) {
      if (p1 > EV1527_PULSEMAX + 10)
        return false; // pulse too long
      if (p0 > EV1527_PULSEMID)
        return false; // invalid pulse sequence 10/01
    } else {
      if (p1 < EV1527_PULSEMIN - 10)
        return false; // pulse too short
      if (p0 < EV1527_PULSEMID)
        return false; // invalid pulse sequence 10/01
      u32_bitStream |= 0x1;
    }
  }

  //===========================================================================
  // Prevent repeating signals from showing up
  //===========================================================================
  if (ps_rawSignal->u32_prevCRC == u32_bitStream &&
      ps_rawSignal->u32_startTime - ps_rawSignal->u32_prevTime < 2000) {
    return true; // already seen the RF packet recently
  }
  ps_rawSignal->u32_prevCRC = u32_bitStream;
  ps_rawSignal->u32_prevTime = ps_rawSignal->u32_startTime;

  fPfxOutput(ps_plugin->pc_name, "ID=%06x;SWITCH=01;CMD=%s;",
             (u32_bitStream >> 4) & 0xffffff,
             (u32_bitStream & 0x0f) == 0x9 ? "OFF" : "ON");

  return true;
}

/*----------------------------------------------------------------------------*/
