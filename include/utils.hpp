#ifndef __UTILS_H
#define __UTILS_H

/* Includes ------------------------------------------------------------------*/

#include <stdarg.h>
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

extern uint8_t u8_sequenceNumber;

extern int sprintf(char *pc_message, const char *pc_format, ...);
extern int vsprintf(char *pc_message, const char *pc_format, va_list s_args);

extern void output(const char *pc_message, int i_length);
extern void fPfxOutput(const char *pc_origin, const char *pc_format, ...);
extern void fOutput(const char *pc_format, ...);

/*----------------------------------------------------------------------------*/

#endif
