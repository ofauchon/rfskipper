/*
 * Somfy.C
 *
 *  Created on: 31 Jan 2020
 *      Author: Pierre
 */

#include <plugin.H>
#include <stdlib.h>

#include "rfm69.H"

/*----------------------------------------------------------------------------*/

#define Somfy_TLength               63   // pulse length in 10�s
#define Somfy_lT     (1*Somfy_TLength)/2 // approx. in between 0T and 1T
#define Somfy_mT     (3*Somfy_TLength)/2 // approx. in between 1T and 2T
#define Somfy_hT     (5*Somfy_TLength)/2 // approx. in between 2T and 3T

const char * const ppc_SomfyCmd[] =
   { "NONE", "STOP", "UP", "MYUP", "DOWN", "MYDOWN", "UPDOWN", "PROG",
         "SUNFLAG", "FLAG" };

typedef struct __attribute__((__packed__)) _SomfyFrame
{
   uint8_t u8_key;
   uint8_t u8_cmdChksum;
   uint8_t u8_rollingCodeMSB;
   uint8_t u8_rollingCodeLSB;
   uint8_t pu8_address[3];
} SomfyFrame;

/*----------------------------------------------------------------------------*/

bool plugin099Rx(const Plugin *ps_plugin, RawSignal *ps_rawSignal, char *pc_cmd) {
   (void)(pc_cmd);
   SomfyFrame s_frame;
   uint32_t u32_address;
   uint16_t *pu16_pulse;
   uint16_t u16_pulse;
   uint16_t u16;
   uint8_t u8_cksum;
   uint8_t *pu8;
   uint8_t u8;
   int i_polarity;
   int i_pulses;
   int i_index;
   int i;

   pu16_pulse = ps_rawSignal->pu16_pulses;
   i_pulses = ps_rawSignal->u16_pulses;
   for (i = 0; i < i_pulses; i++) {
      u16_pulse = *pu16_pulse++;
      if (u16_pulse < 220 || u16_pulse > 270) {
         if (i < 4 || (i & 1) || u16_pulse < 440 || u16_pulse > 540) {
            return false;
         }
         break;
      }
   }

   i_polarity = 0;
   i_index = *pu16_pulse++ > Somfy_mT;

   pu8 = (uint8_t *) &s_frame;
   u16 = 1;
   i += 2;     // skip hardware and soft sync
   do {
      i_polarity ^= 1;
      if (i_index & 1) {
         u16 = (u16 << 1) | i_polarity;
         if (u16 & 0x100) {
            *pu8++ = (uint8_t) u16;
            u16 = 1;
         }
      }

      u16_pulse = *pu16_pulse++;
      if (u16_pulse < Somfy_lT || u16_pulse > Somfy_hT) {
         return false;
      }
      if (u16_pulse < Somfy_mT) {
         i_index++;
      }
   } while (++i < i_pulses);

   if (u16 != 1) {
      *pu8 = (uint8_t) u16 << 1;
   }

   // Deobfuscate the frame content
   pu8 = (uint8_t *) &s_frame;
   for (i = sizeof(SomfyFrame) - 1; i > 0; i--) {
      pu8[i] ^= pu8[i - 1];
   }

   // Verify checksum
   u8_cksum = 0;
   for (i = 0; i < (int) sizeof(SomfyFrame); i++) {
      u8 = *pu8++;
      u8_cksum = u8_cksum ^ u8 ^ (u8 >> 4);
   }

   if (u8_cksum & 0x0f) {
      return false;
   }

   //===========================================================================
   // Prevent repeating signals from showing up
   //===========================================================================
//   if (s_rawSignal.u32_prevCRC == u32_bitStream
//         && s_rawSignal.u32_startTime - s_rawSignal.u32_prevTime < 1500) {
//      return true; // already seen the RF packet recently
//   }
//   s_rawSignal.u32_prevCRC = u32_bitStream;
//   s_rawSignal.u32_prevTime = s_rawSignal.u32_startTime;

   u32_address = s_frame.pu8_address[2];
   u32_address = (u32_address << 8) | s_frame.pu8_address[1];
   u32_address = (u32_address << 8) | s_frame.pu8_address[0];

   o_usart.printf("20;%02X;%s;ID=%08d;SWITCH=%x;CMD=%s;\n", i_sequenceNumber++,
         ps_plugin->pc_name, u32_address,
         (s_frame.u8_rollingCodeMSB << 8) + s_frame.u8_rollingCodeLSB,
         ppc_SomfyCmd[s_frame.u8_cmdChksum >> 4]);

   return true;
}

/*----------------------------------------------------------------------------*/

bool plugin099Tx(const Plugin *ps_plugin, RawSignal *ps_rawSignal, char *pc_cmd) {
   (void)(ps_plugin);
   SomfyFrame s_frame;
   uint16_t *pu16_pulse;
   uint8_t *pu8_src;
   uint8_t u8_cksum;
   uint8_t u8_mask;
   uint8_t u8;
   int i_polarity;
   int i_address;
   int i_unit;
   int i_cmd;
   uint i;

   if (!getParamAsHex(&pc_cmd, &i_address) || !getParamAsHex(&pc_cmd, &i_unit)
         || !getParamAsEnum(&pc_cmd, &i_cmd, ppc_SomfyCmd, 10)) {
      return false;
   }

   pu16_pulse = ps_rawSignal->pu16_pulses;
   // wakeup pulse
   *pu16_pulse++ = 1000;
   *pu16_pulse++ = 9750;
   // insert hardware sync pulses
   for (i = 0; i < 7; i++) {
      *pu16_pulse++ = 250;
      *pu16_pulse++ = 250;
   }
   *pu16_pulse++ = 475; // software sync

   s_frame.u8_key = 0xa0 | (i_unit & 0x000F);
   s_frame.u8_cmdChksum = i_cmd << 4;
   s_frame.u8_rollingCodeMSB = (i_unit & 0xFF00) >> 8;
   s_frame.u8_rollingCodeLSB = (i_unit & 0x00FF);
   s_frame.pu8_address[0] = (i_address & 0x0000FF);
   s_frame.pu8_address[1] = (i_address & 0x00FF00) >> 8;
   s_frame.pu8_address[2] = (i_address & 0xFF0000) >> 16;

   // Compute frame checksum.
   u8_cksum = 0;
   pu8_src = (uint8_t *) &s_frame;
   for (i = 0; i < sizeof(SomfyFrame); i++) {
      u8 = *pu8_src++;
      u8_cksum = u8_cksum ^ u8 ^ (u8 >> 4);
   }
   s_frame.u8_cmdChksum |= u8_cksum & 0x0F;

   // Obfuscate the frame content
   pu8_src = (uint8_t *) &s_frame;
   u8 = 0;
   for (i = 0; i < sizeof(SomfyFrame); i++) {
      u8 ^= *pu8_src;
      *pu8_src++ = u8;
   }

   // Generate pulses
   pu8_src = (uint8_t *) &s_frame;
   i_polarity = 0;

   for (i = 0; i < sizeof(SomfyFrame); i++) {
      u8 = *pu8_src++;
      u8_mask = 0x80;
      while (u8_mask) {
         if (u8 & u8_mask) {
            if (i_polarity == 0) {
               *pu16_pulse++ = Somfy_TLength * 2;
               i_polarity ^= 1;
            } else {
               *pu16_pulse++ = Somfy_TLength;
               *pu16_pulse++ = Somfy_TLength;
            }
         } else {
            if (i_polarity == 0) {
               *pu16_pulse++ = Somfy_TLength;
               *pu16_pulse++ = Somfy_TLength;
            } else {
               *pu16_pulse++ = Somfy_TLength * 2;
               i_polarity ^= 1;
            }
         }
         u8_mask >>= 1;
      }
   }

   if (!i_polarity) {
      *pu16_pulse++ = Somfy_TLength + 3000;
   } else {
      // delay of 30ms
      *pu16_pulse++ = 3000;
   }

   ps_rawSignal->u32_frequency = RFM69_FREQ_TO_REG(433420000); // 433.42MHz
   ps_rawSignal->u16_pulses = pu16_pulse - ps_rawSignal->pu16_pulses;
   ps_rawSignal->u8_repeats = 2; // Number of code retransmissions

   return true;
}

/*----------------------------------------------------------------------------*/
