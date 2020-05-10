#ifndef __USB_H
#define __USB_H

/* Includes ------------------------------------------------------------------*/

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/nvic.h>

#include "version.h"

/* Exported constants --------------------------------------------------------*/

#define BOARD_IDENT "RFSkipper Gateway, (Firmware " FIRMWARE_VERSION ")"

#define USB_DRIVER st_usbfs_v1_usb_driver
#define USB_IRQ NVIC_USB_LP_CAN_RX0_IRQ
#define USB_ISR usb_lp_can_rx0_isr

#define CDCACM_DEBUG_ENDPOINT 1
#define CDCACM_CMD_ENDPOINT 3

#define CDCACM_PACKET_SIZE 32

/* Interrupt priorities.  Low numbers are high priority.
 * For now USART2 preempts USB which may spin while buffer is drained.
 */
#define IRQ_PRI_USB (2 << 4)
#define IRQ_PRI_USBUSART (1 << 4)
#define IRQ_PRI_USBUSART_TIM (3 << 4)
#define IRQ_PRI_USB_VBUS (14 << 4)

/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

extern void usbuart_usb_in_cb(usbd_device *ps_dev, uint8_t u8_ep);
extern void usbuart_usb_out_cb(usbd_device *ps_dev, uint8_t u8_ep);

/*----------------------------------------------------------------------------*/

class USB {
private:
  uint8_t _pu8_usbdControlBuffer[256];
  usbd_device *_ps_usbDev;
  static int _i_configured;

public:
  static char _pc_serialNumber[9];

private:
  static void cdcacm_set_config(usbd_device *ps_dev, uint16_t u16_value);
  static enum usbd_request_return_codes cdcacm_control_request(
    usbd_device *ps_dev, struct usb_setup_data *ps_req, uint8_t **ppu8_buf,
    uint16_t *pu16_len,
    void (**ppf_complete)(usbd_device *, struct usb_setup_data *));
  static void cdcacm_set_modem_state(usbd_device *ps_dev, int i_itf, bool dsr,
                                     bool dcd);
  void readSerialNumber();

public:
  void init(void);
  inline void poll(void) { usbd_poll(_ps_usbDev); }
  inline int cdcacm_get_config(void) { return _i_configured; }
  void usbuart_usb_out(uint8_t u8_endPoint, uint8_t *pu8_buffer, int i_size);
  int printf(const char *pc_format, ...);
  int puts(const char *pc_string);
};

#endif /* __USB_H */
