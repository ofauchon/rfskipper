

/*----------------------------------------------------------------------------*/

#include <stddef.h>

#include "utils.hpp"
#ifdef USB_ENABLE
#include "usb.h"
#endif
#ifdef USART_ENABLE
#include "usart.hpp"
#include "radio.hpp"
#endif

/*----------------------------------------------------------------------------*/

uint8_t u8_sequenceNumber = 0;

/*----------------------------------------------------------------------------*/

#define PAD_RIGHT 1
#define PAD_ZERO 2

static void prints(char **out, const char *string, int width, int pad) {
  register int padchar = ' ';

  if (width > 0) {
    register int len = 0;
    register const char *ptr;
    for (ptr = string; *ptr; ++ptr)
      ++len;
    if (len >= width)
      width = 0;
    else
      width -= len;
    if (pad & PAD_ZERO)
      padchar = '0';
  }
  if (!(pad & PAD_RIGHT)) {
    for (; width > 0; --width) {
      *(*out)++ = padchar;
    }
  }
  for (; *string; ++string) {
    *(*out)++ = *string;
  }
  for (; width > 0; --width) {
    *(*out)++ = padchar;
  }
}

/*----------------------------------------------------------------------------*/
// the following should be enough for 32 bits integer
#define PRINT_BUF_LEN 12

static void printi(char **out, int i, int b, int sg, int width, int pad,
                   int letbase) {
  char print_buf[PRINT_BUF_LEN];
  register char *s;
  register int t, neg = 0;
  register unsigned int u = i;

  if (i == 0) {
    print_buf[0] = '0';
    print_buf[1] = 0;
    prints(out, print_buf, width, pad);
    return;
  }

  if (sg && b == 10 && i < 0) {
    neg = 1;
    u = -i;
  }

  s = print_buf + PRINT_BUF_LEN - 1;
  *s = 0;

  while (u) {
    t = u % b;
    if (t >= 10) {
      t += letbase - '0' - 10;
    }
    *--s = t + '0';
    u /= b;
  }

  if (neg) {
    if (width && (pad & PAD_ZERO)) {
      *(*out)++ = '-';
      --width;
    } else {
      *--s = '-';
    }
  }

  prints(out, s, width, pad);
}

/*----------------------------------------------------------------------------*/

int vsprintf(char *pc_buffer, const char *pc_format, va_list s_args) {
  char *pc_dst;
  char scr[2];
  int width;
  int pad;

  pc_dst = pc_buffer;
  for (; *pc_format != 0; ++pc_format) {
    if (*pc_format == '%') {
      ++pc_format;
      width = pad = 0;
      if (*pc_format == 0) {
        break;
      }
      if (*pc_format == '%') {
        goto copyChar;
      }
      if (*pc_format == '-') {
        ++pc_format;
        pad = PAD_RIGHT;
      }
      while (*pc_format == '0') {
        ++pc_format;
        pad |= PAD_ZERO;
      }
      for (; *pc_format >= '0' && *pc_format <= '9'; ++pc_format) {
        width *= 10;
        width += *pc_format - '0';
      }
      if (*pc_format == 's') {
        register char *s = (char *) va_arg(s_args, char *);
        prints(&pc_dst, s ? s : "(null)", width, pad);
        continue;
      }
      if (*pc_format == 'd') {
        printi(&pc_dst, va_arg(s_args, int), 10, 1, width, pad, 'a');
        continue;
      }
      if (*pc_format == 'x') {
        printi(&pc_dst, va_arg(s_args, int), 16, 0, width, pad, 'a');
        continue;
      }
      if (*pc_format == 'X') {
        printi(&pc_dst, va_arg(s_args, int), 16, 0, width, pad, 'A');
        continue;
      }
      if (*pc_format == 'u') {
        printi(&pc_dst, va_arg(s_args, int), 10, 0, width, pad, 'a');
        continue;
      }
      if (*pc_format == 'b') {
        printi(&pc_dst, va_arg(s_args, int), 2, 0, width, pad, 'a');
        continue;
      }
      if (*pc_format == 'c') {
        /* char are converted to int then pushed on the stack */
        scr[0] = (char) va_arg(s_args, int);
        scr[1] = 0;
        prints(&pc_dst, scr, width, pad);
        continue;
      }
    } else {
    copyChar:
      *pc_dst++ = *pc_format;
    }
  }

  *pc_dst = 0;

  return pc_dst - pc_buffer;
}

/*----------------------------------------------------------------------------*/

int sprintf(char *pc_message, const char *pc_format, ...) {
  va_list s_args;
  int i_length;

  va_start(s_args, pc_format);
  i_length = vsprintf(pc_message, pc_format, s_args);
  va_end(s_args);

  return i_length;
}

/*----------------------------------------------------------------------------*/

void output(const char *pc_message, int i_length) {
#ifdef USB_ENABLE
  usbOutput(pc_message, i_length);
#endif
#ifdef USART_ENABLE
  o_usart.output(pc_message, i_length);
#endif
}

/*----------------------------------------------------------------------------*/

void fPfxOutput(const char *pc_origin, const char *pc_format, ...) {
  char pc_message[128];
  va_list s_args;
  int i_length;

  if (pc_origin != NULL) {
    i_length =
      sprintf(pc_message, "20;%02X;%s;", u8_sequenceNumber++, pc_origin);
  } else {
    i_length = sprintf(pc_message, "20;%02X;", u8_sequenceNumber++);
  }

  if (pc_format != NULL) {
    va_start(s_args, pc_format);
    i_length += vsprintf(&pc_message[i_length], pc_format, s_args);
    va_end(s_args);
  }

  pc_message[i_length++] = '\r';
  pc_message[i_length++] = '\n';

  output(pc_message, i_length);
}

/*----------------------------------------------------------------------------*/

void fOutput(const char *pc_format, ...) {
  char pc_message[128];
  va_list s_args;
  int i_length;

  va_start(s_args, pc_format);
  i_length = vsprintf(pc_message, pc_format, s_args);
  va_end(s_args);

  output(pc_message, i_length);
}

/*----------------------------------------------------------------------------*/
