/*
 * usb.C
 *
 *  Created on: 26 Apr 2020
 *      Author: Pierre
 */
/*----------------------------------------------------------------------------*/

#include <stddef.h>
#include <string.h>

#include "usb.h"

/*----------------------------------------------------------------------------*/

static uint8_t pu8_usbdControlBuffer[256];
static char pc_serialNumber[9];
static int i_configured;
static usbd_device *ps_usbDev;

static const char *ppc_usbStrings[] = { "RFSkipper", BOARD_IDENT,
                                        pc_serialNumber, "RFSkipper Debug Port",
                                        "RFSkipper Proto Port" };

static const struct usb_endpoint_descriptor uart_comm_endp1[] = { {
  USB_DT_ENDPOINT_SIZE,        // bLength
  USB_DT_ENDPOINT,             // bDescriptorType
  0x82,                        // bEndpointAddress
  USB_ENDPOINT_ATTR_INTERRUPT, // bmAttributes
  16,                          // wMaxPacketSize
  255,                         // bInterval
  NULL,                        // extra
  0                            // extralen
} };

static const struct usb_endpoint_descriptor uart_comm_endp2[] = { {
  USB_DT_ENDPOINT_SIZE,        // bLength
  USB_DT_ENDPOINT,             // bDescriptorType
  0x84,                        // bEndpointAddress
  USB_ENDPOINT_ATTR_INTERRUPT, // bmAttributes
  16,                          // wMaxPacketSize
  255,                         // bInterval
  NULL,                        // extra
  0                            // extralen
} };

static const struct usb_endpoint_descriptor uart_data_endp1[] = {
  {
    USB_DT_ENDPOINT_SIZE,   // bLength
    USB_DT_ENDPOINT,        // bDescriptorType
    0x01,                   // bEndpointAddress
    USB_ENDPOINT_ATTR_BULK, // bmAttributes
    CDCACM_PACKET_SIZE,     // wMaxPacketSize
    1,                      // bInterval
    NULL,                   // extra
    0                       // extralen
  },
  {
    USB_DT_ENDPOINT_SIZE,   // bLength
    USB_DT_ENDPOINT,        // bDescriptorType
    0x81,                   // bEndpointAddress
    USB_ENDPOINT_ATTR_BULK, // bmAttributes
    CDCACM_PACKET_SIZE,     // wMaxPacketSize
    1,                      // bInterval
    NULL,                   // extra
    0                       // extralen
  }
};

static const struct usb_endpoint_descriptor uart_data_endp2[] = {
  {
    USB_DT_ENDPOINT_SIZE,   // bLength
    USB_DT_ENDPOINT,        // bDescriptorType
    0x03,                   // bEndpointAddress
    USB_ENDPOINT_ATTR_BULK, // bmAttributes
    CDCACM_PACKET_SIZE,     // wMaxPacketSize
    1,                      // bInterval
    NULL,                   // extra
    0                       // extralen
  },
  {
    USB_DT_ENDPOINT_SIZE,   // bLength
    USB_DT_ENDPOINT,        // bDescriptorType
    0x83,                   // bEndpointAddress
    USB_ENDPOINT_ATTR_BULK, // bmAttributes
    CDCACM_PACKET_SIZE,     // wMaxPacketSize
    1,                      // bInterval
    NULL,                   // extra
    0                       // extralen
  }
};

static const struct usb_interface_descriptor uart_data_iface1[] = { {
  USB_DT_INTERFACE_SIZE, // bLength
  USB_DT_INTERFACE,      // bDescriptorType
  1,                     // bInterfaceNumber
  0,                     // bAlternateSetting
  2,                     // bNumEndpoints
  USB_CLASS_DATA,        // bInterfaceClass
  0,                     // bInterfaceSubClass
  0,                     // bInterfaceProtocol
  0,                     // iInterface
  uart_data_endp1,       // endpoint
  NULL,                  // extra
  0                      // extralen
} };

static const struct usb_interface_descriptor uart_data_iface2[] = { {
  USB_DT_INTERFACE_SIZE, // bLength
  USB_DT_INTERFACE,      // bDescriptorType
  3,                     // bInterfaceNumber
  0,                     // bAlternateSetting
  2,                     // bNumEndpoints
  USB_CLASS_DATA,        // bInterfaceClass
  0,                     // bInterfaceSubClass
  0,                     // bInterfaceProtocol
  0,                     // iInterface
  uart_data_endp2,       // endpoint
  NULL,                  // extra
  0                      // extralen
} };

typedef struct {
  struct usb_cdc_header_descriptor header;
  struct usb_cdc_call_management_descriptor call_mgmt;
  struct usb_cdc_acm_descriptor acm;
  struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) uart_cdcacm_functional_descriptor;

static const uart_cdcacm_functional_descriptor
  uart_cdcacm_functional_descriptors1 = {
    {
      sizeof(struct usb_cdc_header_descriptor), // bFunctionLength
      CS_INTERFACE,                             // bDescriptorType
      USB_CDC_TYPE_HEADER,                      // bDescriptorSubtype
      0x0110                                    // bcdCDC
    },                                          // header
    {
      sizeof(struct usb_cdc_call_management_descriptor), // bFunctionLength
      CS_INTERFACE,                                      // bDescriptorType
      USB_CDC_TYPE_CALL_MANAGEMENT,                      // bDescriptorSubtype
      0,                                                 // bmCapabilities
      1                                                  // bDataInterface
    },                                                   // call_mgmt
    {
      sizeof(struct usb_cdc_acm_descriptor), // bFunctionLength
      CS_INTERFACE,                          // bDescriptorType
      USB_CDC_TYPE_ACM,                      // bDescriptorSubtype
      2                                      // bmCapabilities
    },                                       // acm
    {
      sizeof(struct usb_cdc_union_descriptor), // bFunctionLength
      CS_INTERFACE,                            // bDescriptorType
      USB_CDC_TYPE_UNION,                      // bDescriptorSubtype
      0,                                       // bControlInterface
      1                                        // bSubordinateInterface0
    }                                          // cdc_union
  };

static const uart_cdcacm_functional_descriptor
  uart_cdcacm_functional_descriptors2 = {
    {
      sizeof(struct usb_cdc_header_descriptor), // bFunctionLength
      CS_INTERFACE,                             // bDescriptorType
      USB_CDC_TYPE_HEADER,                      // bDescriptorSubtype
      0x0110                                    // bcdCDC
    },                                          // header
    {
      sizeof(struct usb_cdc_call_management_descriptor), // bFunctionLength
      CS_INTERFACE,                                      // bDescriptorType
      USB_CDC_TYPE_CALL_MANAGEMENT,                      // bDescriptorSubtype
      0,                                                 // bmCapabilities
      3                                                  // bDataInterface
    },                                                   // call_mgmt
    {
      sizeof(struct usb_cdc_acm_descriptor), // bFunctionLength
      CS_INTERFACE,                          // bDescriptorType
      USB_CDC_TYPE_ACM,                      // bDescriptorSubtype
      2                                      // bmCapabilities
    },                                       // acm
    {
      sizeof(struct usb_cdc_union_descriptor), // bFunctionLength
      CS_INTERFACE,                            // bDescriptorType
      USB_CDC_TYPE_UNION,                      // bDescriptorSubtype
      2,                                       // bControlInterface
      3                                        // bSubordinateInterface0
    }                                          // cdc_union
  };

static const struct usb_interface_descriptor uart_comm_iface1[] = { {
  USB_DT_INTERFACE_SIZE,                      // bLength
  USB_DT_INTERFACE,                           // bDescriptorType
  0,                                          // bInterfaceNumber
  0,                                          // bAlternateSetting
  1,                                          // bNumEndpoints
  USB_CLASS_CDC,                              // bInterfaceClass
  USB_CDC_SUBCLASS_ACM,                       // bInterfaceSubClass
  USB_CDC_PROTOCOL_AT,                        // bInterfaceProtocol
  5,                                          // iInterface
  uart_comm_endp1,                            // endpoint
  &uart_cdcacm_functional_descriptors1,       // extra
  sizeof(uart_cdcacm_functional_descriptors1) // extralen
} };

static const struct usb_interface_descriptor uart_comm_iface2[] = { {
  USB_DT_INTERFACE_SIZE,                      // bLength
  USB_DT_INTERFACE,                           // bDescriptorType
  2,                                          // bInterfaceNumber
  0,                                          // bAlternateSetting
  1,                                          // bNumEndpoints
  USB_CLASS_CDC,                              // bInterfaceClass
  USB_CDC_SUBCLASS_ACM,                       // bInterfaceSubClass
  USB_CDC_PROTOCOL_AT,                        // bInterfaceProtocol
  5,                                          // iInterface
  uart_comm_endp2,                            // endpoint
  &uart_cdcacm_functional_descriptors2,       // extra
  sizeof(uart_cdcacm_functional_descriptors2) // extralen
} };

static const struct usb_iface_assoc_descriptor uart_assoc1 = {
  USB_DT_INTERFACE_ASSOCIATION_SIZE, // bLength
  USB_DT_INTERFACE_ASSOCIATION,      // bDescriptorType
  0,                                 // bFirstInterface
  2,                                 // bInterfaceCount
  USB_CLASS_CDC,                     // bFunctionClass
  USB_CDC_SUBCLASS_ACM,              // bFunctionSubClass
  USB_CDC_PROTOCOL_AT,               // bFunctionProtocol
  0                                  // iFunction
};

static const struct usb_iface_assoc_descriptor uart_assoc2 = {
  USB_DT_INTERFACE_ASSOCIATION_SIZE, // bLength
  USB_DT_INTERFACE_ASSOCIATION,      // bDescriptorType
  2,                                 // bFirstInterface
  2,                                 // bInterfaceCount
  USB_CLASS_CDC,                     // bFunctionClass
  USB_CDC_SUBCLASS_ACM,              // bFunctionSubClass
  USB_CDC_PROTOCOL_AT,               // bFunctionProtocol
  0                                  // iFunction
};

static const struct usb_interface ps_itfs[] = {
  {
    NULL,             // cur_altsetting
    1,                // num_altsetting
    &uart_assoc1,     // iface_assoc
    uart_comm_iface1, // altsetting
  },
  {
    NULL,             // cur_altsetting
    1,                // num_altsetting
    NULL,             // iface_assoc
    uart_data_iface1, // altsetting
  },
  {
    NULL,             // cur_altsetting
    1,                // num_altsetting
    &uart_assoc2,     // iface_assoc
    uart_comm_iface2, // altsetting
  },
  {
    NULL,             // cur_altsetting
    1,                // num_altsetting
    NULL,             // iface_assoc
    uart_data_iface2, // altsetting
  }
};

static const struct usb_device_descriptor s_devDesc = {
  USB_DT_DEVICE_SIZE, // bLength
  USB_DT_DEVICE,      // bDescriptorType
  0x0200,             // bcdUSB
  0xEF,               // bDeviceClass : Miscellaneous Device
  2,                  // bDeviceSubClass: Common Class
  1,                  // bDeviceProtocol: Interface Association
  64,                 // bMaxPacketSize0
  0,                  // idVendor
  0x6018,             // idProduct
  0x0100,             // bcdDevice
  1,                  // iManufacturer
  2,                  // iProduct
  3,                  // iSerialNumber
  1                   // bNumConfigurations
};

static const struct usb_config_descriptor s_configDesc = {
  USB_DT_CONFIGURATION_SIZE,            // bLength
  USB_DT_CONFIGURATION,                 // bDescriptorType
  0,                                    // wTotalLength
  sizeof(ps_itfs) / sizeof(ps_itfs[0]), // bNumInterfaces
  1,                                    // bConfigurationValue
  0,                                    // iConfiguration
  0x80,                                 // bmAttributes
  0x32,                                 // bMaxPower
  ps_itfs                               // interface
};

/*----------------------------------------------------------------------------*/
//
// int printf(const char *pc_format, ...) {
//  char pc_message[128];
//  va_list s_args;
//  int i_length;
//
//  va_start(s_args, pc_format);
//  i_length = vsprintf(pc_message, pc_format, s_args);
//  va_end(s_args);
//
//  usbuart_usb_out(CDCACM_CMD_ENDPOINT, (uint8_t *) pc_message, i_length);
//
//  return i_length;
//}
//
/*----------------------------------------------------------------------------*/

inline static int cdcacm_get_config(void) {
  return i_configured;
}

/*----------------------------------------------------------------------------*/
void cdcacm_set_modem_state(usbd_device *ps_dev, int i_itf, bool dsr,
                            bool dcd) {
  const int i_notifSize = sizeof(struct usb_cdc_notification) + 2;
  uint8_t pu8_buffer[i_notifSize];
  struct usb_cdc_notification *ps_notif;

  ps_notif = (struct usb_cdc_notification *) pu8_buffer;

  /* We echo signals back to host as notification */
  ps_notif->bmRequestType = 0xA1;
  ps_notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
  ps_notif->wValue = 0;
  ps_notif->wIndex = i_itf;
  ps_notif->wLength = 2;
  pu8_buffer[8] = (dsr ? 2 : 0) | (dcd ? 1 : 0);
  pu8_buffer[9] = 0;

  while (usbd_ep_write_packet(ps_dev, 0x82 + i_itf, pu8_buffer, i_notifSize) <=
         0) {
  }
}

/*----------------------------------------------------------------------------*/

enum usbd_request_return_codes cdcacm_control_request(
  usbd_device *ps_dev, struct usb_setup_data *ps_req, uint8_t **ppu8_buf,
  uint16_t *pu16_len,
  void (**ppf_complete)(usbd_device *, struct usb_setup_data *)) {
  (void) ppf_complete;
  (void) ppu8_buf;
  (void) pu16_len;

  switch (ps_req->bRequest) {
  case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
    cdcacm_set_modem_state(ps_dev, ps_req->wIndex, true, true);
    //    cdcacm_gdb_dtr = ps_req->wValue & 1; // wValue = number of readers ??
    return USBD_REQ_HANDLED;
  case USB_CDC_REQ_SET_LINE_CODING:
    return USBD_REQ_HANDLED;
  }
  return USBD_REQ_NOTSUPP;
}

/*----------------------------------------------------------------------------*/

void cdcacm_set_config(usbd_device *ps_dev, uint16_t u16_value) {
  i_configured = u16_value;

  // USB EP address: 0x01 to read and 0x81 to write
  usbd_ep_setup(ps_dev, 0x01, USB_ENDPOINT_ATTR_BULK, CDCACM_PACKET_SIZE,
                usbuart_usb_in_cb);
  usbd_ep_setup(ps_dev, 0x81, USB_ENDPOINT_ATTR_BULK, CDCACM_PACKET_SIZE, NULL);
  usbd_ep_setup(ps_dev, 0x82, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

  usbd_ep_setup(ps_dev, 0x03, USB_ENDPOINT_ATTR_BULK, CDCACM_PACKET_SIZE,
                usbuart_usb_in_cb);
  usbd_ep_setup(ps_dev, 0x83, USB_ENDPOINT_ATTR_BULK, CDCACM_PACKET_SIZE, NULL);
  usbd_ep_setup(ps_dev, 0x84, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

  usbd_register_control_callback(
    ps_dev, USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
    USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT, cdcacm_control_request);

  /* Notify the host that DCD is asserted.
   * Allows the use of /dev/tty* devices on *BSD/MacOS
   */
  cdcacm_set_modem_state(ps_dev, 0, true, true);
  cdcacm_set_modem_state(ps_dev, 2, true, true);
}

/*----------------------------------------------------------------------------*/

void readSerialNumber(void) {
  uint32_t *pu32_cpuId = (uint32_t *) 0x1FFFF7E8;
  uint32_t u32_uniqueId;
  char *pc;
  uint8_t u8;
  int i;

  u32_uniqueId = pu32_cpuId[0] + pu32_cpuId[1] + pu32_cpuId[2];

  pc = pc_serialNumber;
  for (i = 0; i < sizeof(u32_uniqueId); i++) {
    u8 = u32_uniqueId & 0xf;
    if (u8 >= 10) {
      *pc++ = u8 - 10 + 'A';
    } else {
      *pc++ = u8 + '0';
    }
    u32_uniqueId >>= 4;
  }
  *pc = 0;
}

/*----------------------------------------------------------------------------*/

void usbPoll(void) {
  usbd_poll(ps_usbDev);
}

/*----------------------------------------------------------------------------*/

void usbInit(void) {
  readSerialNumber();

  ps_usbDev = usbd_init(&USB_DRIVER, &s_devDesc, &s_configDesc, ppc_usbStrings,
                        sizeof(ppc_usbStrings) / sizeof(char **),
                        pu8_usbdControlBuffer, sizeof(pu8_usbdControlBuffer));

  usbd_register_set_config_callback(ps_usbDev, cdcacm_set_config);

  nvic_set_priority(USB_IRQ, IRQ_PRI_USB);
  nvic_enable_irq(USB_IRQ);
}

/*----------------------------------------------------------------------------*/
// When usbuart writes data to host computer
void usbuart_usb_out(uint8_t u8_endPoint, uint8_t *pu8_buffer, int i_size) {
  int i_length;

  if (cdcacm_get_config() != 1) {
    return;
  }

  i_length = 0;
  while (i_size > 0) {
    i_length = i_size > CDCACM_PACKET_SIZE ? CDCACM_PACKET_SIZE : i_size;
    while (usbd_ep_write_packet(ps_usbDev, u8_endPoint, pu8_buffer, i_length) <=
           0) {
    }
    pu8_buffer += i_length;
    i_size -= i_length;
  }

  /*
   * We need to send an empty packet for some hosts to accept this as a
   * complete transfer. libopencm3 needs a change for us to confirm when that
   * transfer is complete, so we just send a packet containing a null byte for
   * now.
   */
  if (i_length == CDCACM_PACKET_SIZE) {
    while (usbd_ep_write_packet(ps_usbDev, u8_endPoint, "", 1) <= 0) {
    }
  }
}

/*----------------------------------------------------------------------------*/

void usbOutput(const char *pc_string, int i_length) {
  usbuart_usb_out(CDCACM_CMD_ENDPOINT, (uint8_t *) pc_string, i_length);
}

/*----------------------------------------------------------------------------*/
