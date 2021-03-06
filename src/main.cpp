/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>,
 * Copyright (C) 2011 Piotr Esden-Tempski <piotr@esden.net>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <errno.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

#include "standard.hpp"
#include "radio.hpp"
#include "ring.hpp"

/*----------------------------------------------------------------------------*/

volatile uint32_t u32_systemMillis = 0;

/*----------------------------------------------------------------------------*/

static void clock_setup() {
  rcc_clock_setup_in_hse_8mhz_out_72mhz();

  /* Enable clocks for GPIO port A/C for LED. */
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_AFIO);
#ifdef USART_ENABLE
  rcc_periph_clock_enable(RCC_USART1);
#endif
#ifdef USB_ENABLE
  rcc_periph_clock_enable(RCC_USB);
  rcc_periph_reset_pulse(RST_USB);
#endif
}

/*----------------------------------------------------------------------------*/

static void systick_setup() {
  /* 72MHz / 8 => 9000000 counts per second. */
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);

  /* 9000000/9000 = 1000 overflows per second - every 1ms one interrupt */
  /* SysTick interrupt every N clock pulses: set reload to N-1 */
  systick_set_reload(8999);

  systick_interrupt_enable();

  /* Start counting. */
  systick_counter_enable();
}

/*----------------------------------------------------------------------------*/

/* sleep for delay milliseconds */
extern "C" void msleep(uint32_t u32_delay) {
  uint32_t u32_wake = u32_systemMillis + u32_delay;
  while (u32_wake > u32_systemMillis) {
    asm("wfi");
  }
}

/*----------------------------------------------------------------------------*/
// the following creates an Arduino-like environment with setup() and loop()
extern void setup();
extern void loop();

int main() {
  clock_setup();
  systick_setup();

  setup();

  while (true) {
    loop();
  }
  // never returns
}
