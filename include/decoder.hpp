#ifndef __DECODER_H
#define __DECODER_H

/* Includes ------------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

extern int decodeManchester(uint16_t *pu16_pulses, uint16_t *pu16_pulse,
                            uint16_t u16_pulseLength, uint8_t *pu8_data,
                            uint8_t *pu8_size, bool b_msb, bool b_pulse,
                            bool b_clockAligned);

/*----------------------------------------------------------------------------*/

#endif /* __DECODER_H */
