#ifndef __USB_H
#define __USB_H

/* Includes ------------------------------------------------------------------*/

#include <stdarg.h>

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/nvic.h>

#include "version.h"

#ifdef __cplusplus
extern "C" {
#endif

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

extern void usbOutput(const char *pc_string, int i_length);
extern void usbuart_usb_in_cb(usbd_device *ps_dev, uint8_t u8_ep);
extern void usbuart_usb_out_cb(usbd_device *ps_dev, uint8_t u8_ep);
extern void usbPoll(void);
extern void usbInit(void);

/*----------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __USB_H */
