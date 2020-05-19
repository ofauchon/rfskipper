#ifndef USART_HPP
#define USART_HPP

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/

class USART {
private:
  char _pc_lastLine[1024];
  int _i_offset;

public:
  USART();
  void puts(const char *pc_string);
  int printf(const char *format, ...);
  int output(const char *format, ...);
  char *getLastLine();
};

/*----------------------------------------------------------------------------*/

#endif
