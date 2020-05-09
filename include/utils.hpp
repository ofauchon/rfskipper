#ifndef __UTILS_H
#define __UTILS_H

/* Includes ------------------------------------------------------------------*/

#include <stdarg.h>

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

extern int sprintf(char *pc_message, const char *pc_format, ...);
extern int vsprintf(char *pc_message, const char *pc_format, va_list s_args);

/*----------------------------------------------------------------------------*/

#endif
