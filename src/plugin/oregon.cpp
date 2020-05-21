#include <stdio.h>
#include <stdlib.h>

#include "plugin.hpp"
#include "decoder.hpp"

/*----------------------------------------------------------------------------*/

#define OSV3_PULSECOUNT_MIN 50 // 126
#define OSV3_PULSECOUNT_MAX                                                    \
  290 // make sure to check the max length in plugin 1 as well..!

#define OREGON_PulseLength 41
#define OREGON_PULSEMIN 490 / RAWSIGNAL_SAMPLE_RATE
#define OREGON_PULSEMAX 980 / RAWSIGNAL_SAMPLE_RATE
#define OREGON_PULSEMID ((OREGON_PULSEMIN + OREGON_PULSEMAX) / 2)

const char *const ppc_OregonCmd[] = { "OFF", "ON",     "BRIGHT",
                                      "DIM", "ALLOFF", "ALLON" };

/*----------------------------------------------------------------------------*/

uint32_t readNibble(uint8_t *pu8_data, int i_offset, int i_nibbles) {
  uint32_t u32_return;
  int i;

  u32_return = 0;
  for (i = 0; i < i_nibbles; i++) {
    u32_return <<= 4;
    if (i_offset & 1) {
      u32_return |= pu8_data[i_offset / 2] & 0xf;
    } else {
      u32_return |= pu8_data[i_offset / 2] >> 4;
    }
    i_offset++;
  }

  return u32_return;
}

/*----------------------------------------------------------------------------*/

bool plugin048Rx(const Plugin *ps_plugin, RawSignal *ps_rawSignal,
                 char *pc_cmd) {
  (void) ps_plugin;
  (void) pc_cmd;

  uint16_t *pu16_pulses;
  uint16_t u16_pulses;
  uint8_t pu8_rawSignal[16];
  uint8_t u8_sum;
  uint8_t *pu8;
  uint8_t u8;
  bool b_clockAligned;
  int i_address;
  int i_temperature;
  int i_humidity;
  int i_flag;
  int i_bits;
  int i;

  // check number of pulses and start pulse which must be short
  u16_pulses = ps_rawSignal->u16_pulses;
  if (u16_pulses < OSV3_PULSECOUNT_MIN) {
    return false;
  }

  pu16_pulses = ps_rawSignal->pu16_pulses;
  //   if (i_pulses != POWERFIX_PulseLength || *pu16_pulses++ >
  //   POWERFIX_PULSEMID) {
  //      return false;
  //   }

  // Oregon v3: preambule is 48 pulses length (16 bits '1')
  for (i = 0; i < 47; i++) {
    if (*pu16_pulses++ > OREGON_PULSEMID) {
      return false; // illegal pulse during preambule
    }
  }
  b_clockAligned = *pu16_pulses++ < OREGON_PULSEMID;
  u16_pulses -= 48;
  /*
   for (i = 0; i < u16_pulses; i += 2) {
   if (pu16_pulses[i] > OREGON_PULSEMID) {
   return -1;  // illegal pulse during preambule
   }
   if (pu16_pulses[i + 1] > OREGON_PULSEMID) {
   break;
   }
   }

   pu16_pulses += i + 2;
   u16_pulses -= i + 2;

   // Oregon Scientific v3 sync
   for (i = 0; i < 3; i++) {
   if (*pu16_pulses++ < OREGON_PULSEMID) {
   return -1;  // illegal pulse during sync
   }
   }

   b_clockAligned = *pu16_pulses++ < OREGON_PULSEMID;
   u16_pulses -= 4;
   */
  //   b_clockAligned = true;
  // ==========================================================================
  //   i_code = decodeManchesterLSB(pu16_pulses, u16_pulses, pu8_rawSignal,
  //         sizeof(pu8_rawSignal), 0, b_clockAligned);
  u8 = sizeof(pu8_rawSignal);
  i_bits = decodeManchester(pu16_pulses, &u16_pulses, OREGON_PULSEMIN,
                            pu8_rawSignal, &u8, false, false, b_clockAligned);
  if (i_bits < 0) {
    return false;
  }

  // check sync nibble which should be 0xA
  if ((pu8_rawSignal[0] & 0x0F) != 0x0A) {
    return false;
  }

  // compute checksum and verify it
  pu8 = pu8_rawSignal;
  u8_sum = *pu8++ >> 4;
  for (i = 1; i <= 7; i++) {
    u8 = *pu8++;
    u8_sum += (u8 >> 4) + (u8 & 0x0F);
  }

  if (*pu8 != u8_sum) {
    return false;
  }

  // swap the nibbles
  pu8 = pu8_rawSignal;
  for (i = i_bits; i > 0; i -= 8) {
    u8 = *pu8;
    *pu8++ = (u8 >> 4) | (u8 << 4);
  }

  // A F824 1 4F 8 3120 140     44 8D
  //   AAAA B CC D EEEE FFFFFFF GG HH

  // add the channelId to the deviceId
  i_address = readNibble(pu8_rawSignal, 1, 5);
  i_temperature = ((pu8_rawSignal[5] >> 4) * 10) +
                  ((pu8_rawSignal[5] & 0x0F) * 100) +
                  ((pu8_rawSignal[4] & 0x0F));
  if (pu8_rawSignal[6] & 0x80) {
    i_temperature = -i_temperature;
  }
  i_humidity = (pu8_rawSignal[7] & 0xF0) | (pu8_rawSignal[6] & 0x0F);
  i_flag = pu8_rawSignal[4] >> 4;

#ifdef TRACE
  pu8 = pu8_rawSignal;
  for (i = i_bits; i > 0; i -= 8) {
    u8 = *pu8++;
    o_usb.printf("%X%X ", u8 >> 4, u8 & 0x0f);
  }
  o_usb.printf("\n");
#endif

  //===========================================================================
  fPfxOutput("Oregon TempHygro", "ID=%05X;TEMP=%04x;HUM=%02x;BAT=%s;",
             i_address, i_temperature, i_humidity, i_flag & 0x4 ? "LOW" : "OK");

  return true;
}

/*----------------------------------------------------------------------------*/
