#include <stdlib.h>

#include "plugin.hpp"

/*----------------------------------------------------------------------------*/

#define OREGON_PulseLength    41
#define OREGON_PULSEMIN  490 / RAWSIGNAL_SAMPLE_RATE
#define OREGON_PULSEMAX  980 / RAWSIGNAL_SAMPLE_RATE
#define OREGON_PULSEMID  ((OREGON_PULSEMIN + OREGON_PULSEMAX) / 2)

const char *const ppc_OregonCmd[] =
   { "OFF", "ON", "BRIGHT", "DIM", "ALLOFF", "ALLON" };

/*----------------------------------------------------------------------------*/
/**
 * Decode a manchester encoded signal. This first pulse of input stream
 * is supposed to be high (off -> on).
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
   i_bit = !i_pulse;
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
 * Decode a manchester encoded signal. This first pulse of input stream
 * is supposed to be high (off -> on).
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

bool plugin048Rx(const Plugin *ps_plugin, RawSignal *ps_rawSignal,
                 char *pc_cmd) {
   uint16_t *pu16_pulses;
   uint16_t u16_pulses;
   uint8_t pu8_rawSignal[64];
   bool b_clockAligned;
   int i_code;
   int i;

   // check number of pulses and start pulse which must be short
   u16_pulses = ps_rawSignal->u16_pulses;
   pu16_pulses = ps_rawSignal->pu16_pulses;
//   if (i_pulses != POWERFIX_PulseLength || *pu16_pulses++ > POWERFIX_PULSEMID) {
//      return false;
//   }

// Oregon v2: preambule is 32 pulses length (16 bits '1')

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

   // Oregon Scientific v3 preambule
   for (i = 0; i < 3; i++) {
      if (*pu16_pulses++ < OREGON_PULSEMID) {
         return -1;  // illegal pulse during preambule
      }
   }

   b_clockAligned = *pu16_pulses++ < OREGON_PULSEMID;
   u16_pulses -= 4;
   // ==========================================================================

   i_code = decodeManchesterLSB(pu16_pulses, u16_pulses, pu8_rawSignal,
         sizeof(pu8_rawSignal), 1, b_clockAligned);

   //===========================================================================
//   o_usart.printf("20;%02X;%s;ID=%04x;SWITCH=%02x;CMD=%s;\n", i_sequenceNumber++,
//         ps_plugin->pc_name, i_address, i_unitCode, ppc_OregonCmd[i_command]);

   return true;
}

/*----------------------------------------------------------------------------*/
