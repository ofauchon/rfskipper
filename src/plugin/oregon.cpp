
#include <stdlib.h>

#include "plugin.hpp"

/*----------------------------------------------------------------------------*/

#define OSV3_PULSECOUNT_MIN 50  // 126
#define OSV3_PULSECOUNT_MAX 290 // make sure to check the max length in plugin 1 as well..!

#define OREGON_PulseLength    41
#define OREGON_PULSEMIN  490 / RAWSIGNAL_SAMPLE_RATE
#define OREGON_PULSEMAX  980 / RAWSIGNAL_SAMPLE_RATE
#define OREGON_PULSEMID  ((OREGON_PULSEMIN + OREGON_PULSEMAX) / 2)

const char *const ppc_OregonCmd[] =
   { "OFF", "ON", "BRIGHT", "DIM", "ALLOFF", "ALLON" };

/*----------------------------------------------------------------------------*/
/**
 * Decode a manchester encoded signal.
 *
 * @param pu16_pulses Array of pulses lengths
 * @param u16_pulses Number of pulses in the array
 * @param pu8_data Array to store the decoded signal
 * @param u8_size Size if the output storage
 * @param i_pulse 1 if first pulse is high (on) otherwise 0
 * @param b_clockAligned true if signal starts at clock otherwise false (half clock)
 */
int decodeManchester(uint16_t *pu16_pulses, uint16_t u16_pulses,
                     uint8_t *pu8_data, uint8_t u8_size, int i_pulse,
                     bool b_clockAligned) {
   uint16_t u16;
   int i_count;
   int i_bits;
   int i_bit;
   int i;

   // If first pulse is aligned to clock, it should be a short pulse
   if (b_clockAligned && *pu16_pulses++ > OREGON_PULSEMID) {
      return -1;
   }

   u16 = 0;
   i_bit = i_pulse;
   i = i_count = i_bits = 0;
   do {
      u16 = (u16 << 1) | i_bit;
      if (++i_bits == 8) {
         *pu8_data++ = (uint8_t) u16;
         if (++i_count == u8_size) {
            return -1;
         }
         u16 = i_bits = 0;
      }
      if (*pu16_pulses++ < OREGON_PULSEMID) {
         if (*pu16_pulses++ > OREGON_PULSEMID) {
            return -1;
         }
         i += 2;
      } else {
         i_bit ^= 1;
         i++;
      }
   } while (i < u16_pulses);

   if (i_bits != 0) {
      *pu8_data = (uint8_t) u16;
   }

   return i_count;
}

/*----------------------------------------------------------------------------*/
/**
 * Decode a manchester encoded signal.
 *
 * @param pu16_pulses Array of pulses lengths
 * @param u16_pulses Number of pulses in the array
 * @param pu8_data Array to store the decoded signal
 * @param u8_size Size if the output storage
 * @param i_pulse 1 if first pulse is high (on) otherwise 0
 * @param b_clockAligned true if signal starts at clock otherwise false (half clock)
 */
int decodeManchesterLSB(uint16_t *pu16_pulses, uint16_t u16_pulses,
                        uint8_t *pu8_data, uint8_t u8_size, int i_pulse,
                        bool b_clockAligned) {
   uint16_t u16;
   int i_count;
   int i_bits;
   int i_bit;
   int i;

   // If first pulse is aligned to clock, it should be a short pulse
   if (b_clockAligned && *pu16_pulses++ > OREGON_PULSEMID) {
      return -1;
   }

   u16 = 0;
   i_bit = i_pulse ? 0x80 : 0x00;
   i = i_count = i_bits = 0;
   do {
      u16 = (u16 >> 1) | i_bit;
      if (++i_bits == 8) {
         *pu8_data++ = (uint8_t) u16;
         if (++i_count == u8_size) {
            return i_count;
         }
         u16 = i_bits = 0;
      }
      if (*pu16_pulses++ < OREGON_PULSEMID) {
         if (*pu16_pulses++ > OREGON_PULSEMID) {
            return -1;
         }
         i += 2;
      } else {
         i_bit ^= 0x80;
         i++;
      }
   } while (i < u16_pulses);

   if (i_bits != 0) {
      *pu8_data = (uint8_t) u16;
   }

   return i_count;
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
   int i_code;
   int i;

   // check number of pulses and start pulse which must be short
   u16_pulses = ps_rawSignal->u16_pulses;
   if (u16_pulses < OSV3_PULSECOUNT_MIN) {
      return false;
   }


   pu16_pulses = ps_rawSignal->pu16_pulses;
//   if (i_pulses != POWERFIX_PulseLength || *pu16_pulses++ > POWERFIX_PULSEMID) {
//      return false;
//   }

// Oregon v3: preambule is 48 pulses length (16 bits '1')
   for (i = 0; i < 47; i++) {
      if (*pu16_pulses++ > OREGON_PULSEMID) {
         return -1;  // illegal pulse during preambule
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
   i_code = decodeManchesterLSB(pu16_pulses, u16_pulses, pu8_rawSignal,
         sizeof(pu8_rawSignal), 0, b_clockAligned);

   // check sync nibble
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
   for (i = 0; i < i_code; i++) {
      u8 = *pu8;
      *pu8++ = (u8 >> 4) | (u8 << 4);
   }

   // A F824 1 4F 8 3120 140     44 8D
   //   AAAA B CC D EEEE FFFFFFF GG HH

   // add the channelId to the deviceId
   i_address = readNibble(pu8_rawSignal, 1, 5);
   i_temperature = ((pu8_rawSignal[5] >> 4) * 10)
         + ((pu8_rawSignal[5] & 0x0F) * 100) + ((pu8_rawSignal[4] >> 4));
   if (pu8_rawSignal[6] & 0x80) {
      i_temperature = -i_temperature;
   }
   i_humidity = (pu8_rawSignal[7] & 0xF0) | (pu8_rawSignal[6] & 0x0F);
   i_flag = pu8_rawSignal[4] >> 4;

#ifdef TRACE
   pu8 = pu8_rawSignal;
   for (i = 0; i < i_code; i++) {
      u8 = *pu8++;
      o_usart.printf("%X%X ", u8 >> 4, u8 & 0x0f);
   }
   o_usart.printf("\n");
#endif

   //===========================================================================
   o_usart.printf("20;%02X;%s;ID=%05X;TEMP=%04x;HUM=%02x;BAT=%s;\n",
         i_sequenceNumber++, "Oregon TempHygro", i_address, i_temperature,
         i_humidity, i_flag & 0x4 ? "LOW" : "OK");

   return true;
}

/*----------------------------------------------------------------------------*/
