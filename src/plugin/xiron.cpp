#include <stdlib.h>

#include "decoder.hpp"
#include "plugin.hpp"

/*----------------------------------------------------------------------------*/
/*
 * Technical Information:
 * Decodes signals from a Auriol and Xiron Weatherstation outdoor unit, (36
 * bits, 433 MHz).
 *
 * Auriol Message Format:
 * 10100100 10 00 000011100110 1111 00000000  Temp = 23.60
 * AAAAAAAA BB CC DDDDDDDDDDDD EEEE FFFFFFFF
 * A = Rolling Code, no change during normal operation. (Device 'Session' ID)
 * (Might also be 4 bits RC and 4 bits for channel number) B = Battery status
 * indicator on highest bit, 1=OK 0=LOW C = Always 00 (Most likely channel
 * number) D = Temperature (12 bit, 21.5 degrees is shown as decimal value 215,
 * minus values have the high bit set and need to be subtracted from a base
 * value of 4096) E = Always 1111 ? F = Always 0000 0000 ?
 *
 * Xiron Message Format:
 * 01101110 10 00 000011101101 1111 00110011
 * AAAAAAAA BB CC DDDDDDDDDDDD EEEE FFFFFFFF
 * ID       ?? Ch Temperature  ?    Humidity
 *
 * A = ID (Rolling Code, changes after battery replacement)
 * B = Battery status indicator on highest bit, 1=OK 0=LOW
 * C = Channel (1,2,3)
 * D = Temperature (12 bit, 21.5 degrees is shown as decimal value 215, minus
 * values have the high bit set and need to be subtracted from a base value of
 * 4096) E = Always 1111 ? F = Humidity
 */

#define Xiron_RawSignalLength 74
#define Xiron_TLength 50 // pulse length in 10µs

#define EQUALS(x, p) (((x) > p * .9) && ((x) < p * 1.1))

/*----------------------------------------------------------------------------*/

bool plugin046Rx(const Plugin *ps_plugin, RawSignal *ps_rawSignal,
                 char *pc_cmd) {
  (void) pc_cmd;

  uint16_t *pu16_pulses;
  uint16_t u16_pulse;
  uint8_t pu8_rawSignal[5];
  uint8_t *pu8;
  uint8_t u8_byte;
  uint8_t u8_humidity;
  uint8_t u8_channel;
  int i_address;
  int i_temperature;
  int i_battery;
  int i_bits;

  if (ps_rawSignal->u16_pulses < Xiron_RawSignalLength) {
    return false;
  }

  pu16_pulses = ps_rawSignal->pu16_pulses;
  u16_pulse = pu16_pulses[Xiron_RawSignalLength - 1];
  if (!EQUALS(u16_pulse, 8 * Xiron_TLength)) {
    return false;
  }

  pu8 = pu8_rawSignal;
  u8_byte = 0;
  i_bits = 0;

  for (int i = 2; i < Xiron_RawSignalLength; i += 2) {
    u16_pulse = *pu16_pulses++;
    if (!EQUALS(u16_pulse, Xiron_TLength)) {
      return false;
    }
    u16_pulse = *pu16_pulses++;
    u8_byte <<= 1;
    if (u16_pulse > 3 * Xiron_TLength) {
      if (u16_pulse >= 5 * Xiron_TLength) {
        return false;
      }
      u8_byte |= 1;
    } else {
      if (u16_pulse <= Xiron_TLength) {
        return false;
      }
    }
    if ((++i_bits & 0b0111) == 0) {
      *pu8++ = u8_byte;
      u8_byte = 0;
    }
  }
  if (i_bits & 0b0111) {
    *pu8 = u8_byte << (8 - (i_bits & 0b0111));
  }

  if ((pu8_rawSignal[3] & 0xf0) != 0xf0) {
    return false;
  }

  i_address = pu8_rawSignal[0];
  u8_channel = ((pu8_rawSignal[1] >> 4) & 0x03) + 1;
  u8_humidity = (pu8_rawSignal[3] << 4) | (pu8_rawSignal[4] >> 4);
  i_battery = (pu8_rawSignal[1] >> 7) & 0x1;
  i_temperature = ((pu8_rawSignal[1] & 0x0f) << 8) | pu8_rawSignal[2];
  if (i_temperature > 3000) {
    i_temperature -= 4096; // for minus temperature
  }

  fPfxOutput(ps_plugin->pc_name, "ID=%02X%02X;TEMP=%04x;HUM=%02d;BAT=%s;",
             i_address, u8_channel, i_temperature, u8_humidity,
             i_battery ? "OK" : "LOW");

  return true;
}

/*----------------------------------------------------------------------------*/
