#include <stdio.h>
#include <stdlib.h>

#include "plugin.hpp"

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
/**
 * Decode a Manchester encoded signal.
 *
 * @param pu16_pulses Array of pulses lengths
 * @param pu16_pulse Pointer on number of pulses in the array
 * @param u16_pulseLength Short pulse length (half-clock)
 * @param pu8_data Array to store the decoded signal
 * @param pu8_size Pointer to size of the output storage
 * @param b_msb true if first bit is MSB otherwise false LSB
 * @param b_pulse true if first pulse is high (on) otherwise false (off)
 * @param b_clockAligned true if signal starts at clock otherwise false (half
 * clock)
 * @retval - the number bits decoded
 *         - -1 if an error is detected.
 */
int decodeManchester(uint16_t *pu16_pulses, uint16_t *pu16_pulse,
                     uint16_t u16_pulseLength, uint8_t *pu8_data,
                     uint8_t *pu8_size, bool b_msb, bool b_pulse,
                     bool b_clockAligned) {
  uint16_t u16_pulse;
  uint8_t u8_mask;
  uint8_t u8_size;
  uint8_t u8;
  int i_pulses;
  int i_count;
  int i_bits;
  int i_bit;

  // Manchester pulse constants:
  // shortPulseMin is minimum length of a short pulse
  // pulseSeparator is pulse length which splits short from long pulses
  // longPulseMax is maximum length of a long pulse
  const uint16_t u16_shortPulseMin = u16_pulseLength / 2;
  const uint16_t u16_pulseSeparator = u16_pulseLength + u16_shortPulseMin;
  const uint16_t u16_longPulseMax = u16_pulseSeparator + u16_pulseLength;

  // If first pulse is aligned to clock, it should be a short pulse
  if (b_clockAligned) {
    u16_pulse = *pu16_pulses++;
    if (u16_pulse > u16_pulseSeparator || u16_pulse < u16_shortPulseMin) {
        return -1;
      }
    }

  i_pulses = *pu16_pulse;
  u8_size = *pu8_size;
  u8_mask = b_msb ? 0x01 : 0x80;
  i_bit = b_pulse ? u8_mask : 0x00;
  i_count = i_bits = 0;
  u8 = 0;

  do {
    u8 = (b_msb ? (u8 << 1) : (u8 >> 1)) | i_bit;
    if (++i_bits == 8) {
      if (++i_count == u8_size) {
        break;
      }
      *pu8_data++ = u8;
      u8 = i_bits = 0;
    }

    u16_pulse = *pu16_pulses++;
    if (u16_pulse < u16_pulseSeparator) {
      if (u16_pulse < u16_shortPulseMin) {
        break;
      }
      u16_pulse = *pu16_pulses++;
      if (u16_pulse > u16_pulseSeparator || u16_pulse < u16_shortPulseMin) {
        break;
      }
      i_pulses -= 2;
    } else {
      if (u16_pulse > u16_longPulseMax) {
        break;
      }
      i_bit ^= u8_mask;
      i_pulses--;
    }
  } while (i_pulses > 0);

  if (i_bits != 0) {
    *pu8_data = (uint8_t) u8;
  }

  *pu16_pulse -= i_pulses;
  *pu8_size -= i_count;

  return (i_pulses <= 0 || i_count == u8_size) ? i_count * 8 + i_bits : -1;
}

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
      return -1; // illegal pulse during preambule
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
    o_usart.printf("%X%X ", u8 >> 4, u8 & 0x0f);
  }
  o_usart.printf("\n");
#endif

  //===========================================================================
  o_usart.printf("20;%02X;%s;ID=%05X;TEMP=%04x;HUM=%02x;BAT=%s;\n",
                 u8_sequenceNumber++, "Oregon TempHygro", i_address,
                 i_temperature, i_humidity, i_flag & 0x4 ? "LOW" : "OK");

  return true;
}

/*----------------------------------------------------------------------------*/
