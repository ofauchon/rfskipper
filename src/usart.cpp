/* Includes ------------------------------------------------------------------*/

#include <string.h>
#include <stdarg.h>

#include <libopencm3/stm32/rcc.h>

#include "usart.hpp"

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

void USART::output(const char *pc_string, int i_length) {
  uint8_t u8_byte;

  while (i_length-- > 0) {
    u8_byte = (uint8_t) *pc_string++;
    if (u8_byte == '\n') {
      send('\r');
    }
    send(u8_byte);
  }
}

/*----------------------------------------------------------------------------*/
