/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2015  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements a the USB Communications Device Class - Abstract
 * Control Model (CDC-ACM) as defined in CDC PSTN subclass 1.2.
 *
 * The device's unique id is used as the USB serial number string.
 */
#ifndef __CDCACM_H
#define __CDCACM_H

#include <libopencm3/usb/usbd.h>
#include "eventqueue.H"


#define CDCACM_PACKET_SIZE 	32

#define BOARD_IDENT       "RFSkipper RF Gateway, (Firmware " FIRMWARE_VERSION ")"

#define USB_DRIVER      st_usbfs_v1_usb_driver
#define USB_IRQ	        NVIC_USB_LP_CAN_RX0_IRQ
#define USB_ISR	        usb_lp_can_rx0_isr
/* Interrupt priorities.  Low numbers are high priority.
 * For now USART2 preempts USB which may spin while buffer is drained.
 */
#define IRQ_PRI_USB		(2 << 4)
#define IRQ_PRI_USBUSART	(1 << 4)
#define IRQ_PRI_USBUSART_TIM	(3 << 4)
#define IRQ_PRI_USB_VBUS	(14 << 4)


extern usbd_device *usbdev;

void cdcacm_init(void);
/* Returns current usb configuration, or 0 if not configured. */
int cdcacm_get_config(void);
int cdcacm_get_dtr(void);

void usbuart_write(uint8_t ep, const char *buf, uint8_t len);
void usbuart_enable(void);
void usbuart_disable(void);

extern EventQueue o_queue;



#endif
