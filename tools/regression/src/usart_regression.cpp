/* Includes ------------------------------------------------------------------*/

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "usart_regression.hpp"
#include "utils.hpp"

/*----------------------------------------------------------------------------*/

USART::USART() {
  _i_offset = 0;
}

/*----------------------------------------------------------------------------*/

int USART::printf(const char *pc_format, ...) {
  char pc_message[128];
  va_list s_args;
  int i_length;

  va_start(s_args, pc_format);
  i_length = vsprintf(pc_message, pc_format, s_args);
  va_end(s_args);

  puts(pc_message);

  return i_length;
}

/*----------------------------------------------------------------------------*/

void USART::puts(const char *pc_string) {
  int i_size;

  ::puts(pc_string);

  i_size = strlen(pc_string);
  strcpy(&_pc_lastLine[_i_offset], pc_string);

  if (pc_string[i_size - 1] == '\n') {
    _i_offset = 0;
  } else {
    _i_offset += i_size;
  }
}

/*----------------------------------------------------------------------------*/

char *USART::getLastLine() {
  return _pc_lastLine;
}

/*----------------------------------------------------------------------------*/
