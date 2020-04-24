/*
 * standard.h
 *
 *  Created on: 09 Jun 2018
 *      Author: Pierre
 */

#ifndef STANDARD_H_
#define STANDARD_H_

#include <stdint.h>

#include <libopencm3/cm3/cortex.h>

/*----------------------------------------------------------------------------*/

enum _Polarity { Low = 0, High = 1 };
typedef enum _Polarity Polarity;

typedef unsigned char uchar;
typedef unsigned short ushort;
typedef unsigned int uint;
typedef unsigned long ulong;

/*----------------------------------------------------------------------------*/

#define TRUE 1
#define FALSE 0

#define ENTER_CRITICAL() cm_disable_interrupts()
#define EXIT_CRITICAL() cm_enable_interrupts()

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint32_t u32_systemMillis;

inline uint32_t millis() __attribute__((always_inline));
inline uint32_t millis() {
  return u32_systemMillis;
}

extern void msleep(uint32_t delay);

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------*/

#endif /* STANDARD_H_ */
