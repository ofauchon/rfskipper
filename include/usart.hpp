#ifndef USART_HPP
#define USART_HPP

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

/*----------------------------------------------------------------------------*/

typedef uint32_t USART_BASE;

/*----------------------------------------------------------------------------*/

class USART {
private:
  USART_BASE _o_usart;

public:
  void init(USART_BASE o_usart, uint32_t u32_baudRate = 57600,
            uint32_t u32_dataBits = 8, uint32_t u32_parity = USART_PARITY_NONE,
            uint32_t u32_stopBits = USART_STOPBITS_1,
            uint32_t u32_mode = USART_MODE_TX_RX,
            uint32_t u32_flowControl = USART_FLOWCONTROL_NONE);

  inline void enable() { usart_enable(_o_usart); }
  inline void disable() { usart_disable(_o_usart); }
  inline void setBaudRate(uint32_t u32_baudRate) {
    usart_set_baudrate(_o_usart, u32_baudRate);
  }
  inline void setDataBits(uint32_t u32_dataBits) {
    usart_set_databits(_o_usart, u32_dataBits);
  }
  inline void setMode(uint32_t u32_mode) { usart_set_mode(_o_usart, u32_mode); }
  inline void setFlowControl(uint32_t u32_flowControl) {
    usart_set_flow_control(_o_usart, u32_flowControl);
  }
  inline void setParity(uint32_t u32_parity) {
    usart_set_parity(_o_usart, u32_parity);
  }
  inline void setStopBits(uint32_t u32_stopBits) {
    usart_set_stopbits(_o_usart, u32_stopBits);
  }
  inline void send(uint16_t u16_data) {
    usart_send_blocking(_o_usart, u16_data);
  }
  inline uint16_t receive() { return usart_recv(_o_usart); }
  inline void enableRxInterrupt() { usart_enable_rx_interrupt(_o_usart); }
  inline void enableTxInterrupt() { usart_enable_tx_interrupt(_o_usart); }
  inline void enableErrorInterrupt() { usart_enable_error_interrupt(_o_usart); }
  inline void disableRxInterrupt() { usart_disable_rx_interrupt(_o_usart); }
  inline void disableTxInterrupt() { usart_disable_tx_interrupt(_o_usart); }
  inline void disableErrorInterrupt() {
    usart_disable_error_interrupt(_o_usart);
  }

  void puts(const char *pc_string);
  int printf(const char *format, ...);
};

/*----------------------------------------------------------------------------*/

#endif
