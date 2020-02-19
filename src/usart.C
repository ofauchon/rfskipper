/* Includes ------------------------------------------------------------------*/

#include <string.h>
#include <stdarg.h>

#include <libopencm3/stm32/rcc.h>

#include "usart.H"
#include "cdcacm.h"

/*----------------------------------------------------------------------------*/

inline void printchar(char **str, int c) {
   if (str) {
      *(*str)++ = c;
   }
}

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
         printchar(out, padchar);
      }
   }
   for (; *string; ++string) {
      printchar(out, *string);
   }
   for (; width > 0; --width) {
      printchar(out, padchar);
   }
}

/*----------------------------------------------------------------------------*/
/* the following should be enough for 32 bit int */
#define PRINT_BUF_LEN 12

static void printi(char **out, int i, int b, int sg, int width, int pad,
                   int letbase) {
   char print_buf[PRINT_BUF_LEN];
   register char *s;
   register int t, neg = 0;
   register unsigned int u = i;

   if (i == 0) {
      print_buf[0] = '0';
      print_buf[1] = '\0';
      prints(out, print_buf, width, pad);
      return;
   }

   if (sg && b == 10 && i < 0) {
      neg = 1;
      u = -i;
   }

   s = print_buf + PRINT_BUF_LEN - 1;
   *s = '\0';

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
         printchar(out, '-');
         --width;
      } else {
         *--s = '-';
      }
   }

   prints(out, s, width, pad);
}

/*----------------------------------------------------------------------------*/

int print(char *pc_buffer, const char *format, va_list args) {
   register int width, pad;
   register char **out;
   char *pc_end;
   char scr[2];

   pc_end = pc_buffer;
   out = &pc_end;
   for (; *format != 0; ++format) {
      if (*format == '%') {
         ++format;
         width = pad = 0;
         if (*format == '\0')
            break;
         if (*format == '%')
            goto out;
         if (*format == '-') {
            ++format;
            pad = PAD_RIGHT;
         }
         while (*format == '0') {
            ++format;
            pad |= PAD_ZERO;
         }
         for (; *format >= '0' && *format <= '9'; ++format) {
            width *= 10;
            width += *format - '0';
         }
         if (*format == 's') {
            register char *s = (char *) va_arg(args, char *);
            prints(out, s ? s : "(null)", width, pad);
            continue;
         }
         if (*format == 'd') {
            printi(out, va_arg(args, int), 10, 1, width, pad, 'a');
            continue;
         }
         if (*format == 'x') {
            printi(out, va_arg(args, int), 16, 0, width, pad, 'a');
            continue;
         }
         if (*format == 'X') {
            printi(out, va_arg(args, int), 16, 0, width, pad, 'A');
            continue;
         }
         if (*format == 'u') {
            printi(out, va_arg(args, int), 10, 0, width, pad, 'a');
            continue;
         }
         if (*format == 'b') {
            printi(out, va_arg(args, int), 2, 0, width, pad, 'a');
            continue;
         }
         if (*format == 'c') {
            /* char are converted to int then pushed on the stack */
            scr[0] = (char) va_arg(args, int);
            scr[1] = 0;
            prints(out, scr, width, pad);
            continue;
         }
      } else {
         out: printchar(out, *format);
      }
   }

   if (out) {
      **out = 0;
   }

   va_end(args);

   return pc_end - pc_buffer;
}

/*----------------------------------------------------------------------------*/
/*
 * Default parameter values are:
 * uint32_t u32_baudRate = 57600,
 * uint32_t u32_dataBits = 8,
 * uint32_t u32_parity = USART_PARITY_NONE,
 * uint32_t u32_stopBits = USART_STOPBITS_1,
 * uint32_t u32_mode = USART_MODE_TX_RX,
 * uint32_t u32_flowControl = USART_FLOWCONTROL_NONE
 */
void USART::init(USART_BASE o_usart, uint32_t u32_baudRate,
                 uint32_t u32_dataBits, uint32_t u32_parity,
                 uint32_t u32_stopBits, uint32_t u32_mode,
                 uint32_t u32_flowControl) {
   _o_usart = o_usart;

   setBaudRate(u32_baudRate);
   setDataBits(u32_dataBits);
   setParity(u32_parity);
   setStopBits(u32_stopBits);
   setMode(u32_mode);
   setFlowControl(u32_flowControl);
}

/*----------------------------------------------------------------------------*/

int USART::printf(const char *pc_format, ...) {
   char pc_message[128];
   va_list s_args;
   int i_length;

   va_start(s_args, pc_format);
   i_length = print(pc_message, pc_format, s_args);

   puts(pc_message);

   return i_length;
}

/*----------------------------------------------------------------------------*/

void USART::puts(const char *pc_string) {
   uint8_t u8_byte;

   //usbuart_write(1, pc_string, strlen(pc_string));

   while ((u8_byte = (uint8_t) *pc_string++) != 0) {
      if (u8_byte == '\n') {
         send('\r');
      }
      send(u8_byte);
   }
}

/*----------------------------------------------------------------------------*/
