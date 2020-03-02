#include <stdlib.h>

#include "plugin.hpp"

/*----------------------------------------------------------------------------*/

#define POWERFIX_PulseLength    41
#define POWERFIX_PULSEMID  900 / RAWSIGNAL_SAMPLE_RATE
#define POWERFIX_PULSEMIN  450 / RAWSIGNAL_SAMPLE_RATE
#define POWERFIX_PULSEMAX  1400 / RAWSIGNAL_SAMPLE_RATE

const char * const ppc_PowerFixCmd[] =
   { "OFF", "ON", "BRIGHT", "DIM", "ALLOFF", "ALLON" };

/*----------------------------------------------------------------------------*/

bool plugin013Rx(const Plugin *ps_plugin, RawSignal *ps_rawSignal,
                 char *pc_cmd) {
   uint16_t *pu16_pulses;
   uint16_t u16_pulse;
   uint32_t u32_bitStream;
   int i_pulses;
   int i_unitCode;
   int i_command;
   int i_address;
   int i_parity;
   int i_bits;
   int i;

   // check number of pulses and start pulse which must be short
   i_pulses = ps_rawSignal->u16_pulses;
   pu16_pulses = ps_rawSignal->pu16_pulses;
   if (i_pulses != POWERFIX_PulseLength || *pu16_pulses++ > POWERFIX_PULSEMID) {
      return false;
   }

   // ==========================================================================
   i_bits = i_parity = 0;
   u32_bitStream = 0;
   for (i = 1; i < i_pulses; i += 2) {
      u16_pulse = *pu16_pulses++;
      u32_bitStream <<= 1;
      if (u16_pulse > POWERFIX_PULSEMID) {
         if (u16_pulse > POWERFIX_PULSEMAX) {
            return false; // Long pulse too long
         }
         if (*pu16_pulses++ > POWERFIX_PULSEMID) {
            return false;
         }
         u32_bitStream |= 1;
         if (i_bits > 11) {
            i_parity ^= 1;
         }
      } else {
         if (u16_pulse < POWERFIX_PULSEMIN) {
            return false; // Short pulse too short
         }
         if (*pu16_pulses++ < POWERFIX_PULSEMID) {
            return false;
         }
      }
      i_bits++;
   }

   if (i_parity) {
      return false; // Parity error
   }
   if (u32_bitStream & 0x04) {
      return false; // Tested bit should always be zero
   }

   //===========================================================================
   // Prevent repeating signals from showing up
   //===========================================================================
   if (ps_rawSignal->u32_prevCRC == u32_bitStream
         && ps_rawSignal->u32_startTime - ps_rawSignal->u32_prevTime < 1500) {
      return true; // already seen the RF packet recently
   }
   ps_rawSignal->u32_prevCRC = u32_bitStream;
   ps_rawSignal->u32_prevTime = ps_rawSignal->u32_startTime;

   //===========================================================================
   // Sort data
   i_address = u32_bitStream >> 8;      // 12 bits address
   i_unitCode = (u32_bitStream >> 6) & 0x03;
   if (i_unitCode == 1 || i_unitCode == 2) {
      i_unitCode ^= 0x03;   // bits are simply reversed
   }

   i_bits = u32_bitStream & 0x3f;                    // re-use parity variable
   i_command = (i_bits >> 4) & 0x01;                   // On/Off command
   if (i_bits & 0x08) {                // dim command
      i_command += 2;
   } else if (i_bits & 0x20) {              // group command
      i_command += 4;
   }

   //===========================================================================
   o_usart.printf("20;%02X;%s;ID=%04x;SWITCH=%02x;CMD=%s;\n", i_sequenceNumber++,
         ps_plugin->pc_name, i_address, i_unitCode, ppc_PowerFixCmd[i_command]);

   return true;
}

/*----------------------------------------------------------------------------*/
