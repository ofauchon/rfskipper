
#include "plugin.hpp"

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
