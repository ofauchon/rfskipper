/* Includes ------------------------------------------------------------------*/

#include <libopencm3/stm32/rcc.h>

#include "spi.H"

/*----------------------------------------------------------------------------*/
/*
 * When a master is communicating with SPI slaves which need to be de-selected between
 * transmissions, the NSS pin must be configured as GPIO or another GPIO must be used and
 * toggled by software.
 */
void SPI::enableSlaveDevice() {
   if (_pf_chipSelect == NULL) {
      // Set slave NSS pin low
      gpio_clear(GPIO_BANK_SPI1_NSS, GPIO_SPI1_NSS);
   } else {
      _pf_chipSelect(SPI_LOW);
   }
}

/*----------------------------------------------------------------------------*/

void SPI::disableSlaveDevice() {
   // Wait the SPI to be idle
   while (SPI_SR(_o_spi) & SPI_SR_BSY) {
   }

   if (_pf_chipSelect == NULL) {
      // Set slave NSS pin high
      gpio_set(GPIO_BANK_SPI1_NSS, GPIO_SPI1_NSS);
   } else {
      _pf_chipSelect(SPI_HIGH);
   }
}

/*----------------------------------------------------------------------------*/

void SPI::sendRecvBuffer(uint8_t *pu8_send, uint8_t *pu8_recv, int i_size) {
   while (i_size-- > 0) {
      *pu8_recv++ = spi_xfer(_o_spi, *pu8_send++);
   }
}

/*----------------------------------------------------------------------------*/

void SPI::sendRecvBuffer(uint16_t *pu16_send, uint16_t *pu16_recv, int i_size) {
   while (i_size-- > 0) {
      *pu16_recv++ = spi_xfer(_o_spi, *pu16_send++);
   }
}

/*----------------------------------------------------------------------------*/

void SPI::sendBuffer(const uint8_t *pu8_send, int i_size) {
   while (i_size-- > 0) {
      spi_xfer(_o_spi, *pu8_send++);
   }
}

/*----------------------------------------------------------------------------*/

void SPI::sendBuffer(const uint16_t *pu16_send, int i_size) {
   while (i_size-- > 0) {
      spi_xfer(_o_spi, *pu16_send++);
   }
}

/*----------------------------------------------------------------------------*/

void SPI::recvBuffer(uint8_t *pu8_buffer, uint8_t u8_nopMask, int i_size) {
   while (i_size-- > 0) {
      *pu8_buffer++ = spi_xfer(_o_spi, u8_nopMask);
   }
}

/*----------------------------------------------------------------------------*/

void SPI::recvBuffer(uint16_t *pu16_buffer, uint16_t u16_nopMask, int i_size) {
   while (i_size-- > 0) {
      *pu16_buffer++ = spi_xfer(_o_spi, u16_nopMask);
   }
}

/*----------------------------------------------------------------------------*/
/*
 * Initialize GPIO SPI pins according to the STM32 datasheets.
 * Initialize SPI interface in master mode with default parameters:
 * synchronous full-duplex protocol corresponding to CPOL = 0 and CPHA = 0
 * in Motorola/Freescale nomenclature.
 * The data byte (8bits) is transmitted MSB first.
 * Transmission baud rate is set to CLK_DIV_4.
 * Enable CRC.
 */
void SPI::init(SPI_BASE o_spi, SPI_ChipSelect pf_chipSelect) {

   _o_spi = o_spi;
   _pf_chipSelect = pf_chipSelect;

   switch (o_spi) {
#if defined(SPI1_BASE)
   case SPI1_BASE:
      rcc_periph_clock_enable(RCC_SPI1);
      gpio_set_mode(GPIO_BANK_SPI1_NSS, GPIO_MODE_OUTPUT_10_MHZ,
      GPIO_CNF_OUTPUT_PUSHPULL, GPIO_SPI1_NSS);
      gpio_set(GPIO_BANK_SPI1_NSS, GPIO_SPI1_NSS);

      gpio_set_mode(GPIO_BANK_SPI1_SCK, GPIO_MODE_OUTPUT_10_MHZ,
      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI1_SCK | GPIO_SPI1_MOSI);

      gpio_set_mode(GPIO_BANK_SPI1_SCK, GPIO_MODE_INPUT,
      GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_SPI1_MISO);
      break;
#endif
#if defined(SPI2_BASE)
   case SPI2_BASE:
      rcc_periph_clock_enable(RCC_SPI2);
      gpio_set_mode(GPIO_BANK_SPI2_NSS, GPIO_MODE_OUTPUT_10_MHZ,
      GPIO_CNF_OUTPUT_PUSHPULL, GPIO_SPI2_NSS);
      gpio_set(GPIO_BANK_SPI2_NSS, GPIO_SPI2_NSS);

      gpio_set_mode(GPIO_BANK_SPI2_SCK, GPIO_MODE_OUTPUT_10_MHZ,
      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI2_SCK | GPIO_SPI2_MOSI);

      gpio_set_mode(GPIO_BANK_SPI2_SCK, GPIO_MODE_INPUT,
      GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_SPI2_MISO);
      break;
#endif
   }

   spi_init_master(o_spi, SPI_CR1_BAUDRATE_FPCLK_DIV_4,
   SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
   SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
   spi_enable_crc(o_spi);
}

/*----------------------------------------------------------------------------*/

void SPI::reconfigure(uint32_t u32_cr1Config) {
//    spi_irq_disable(ps_spi, SPI_INTERRUPTS_ALL);
   disable();
   SPI_CR1(_o_spi) = u32_cr1Config;
   enable();
}

/*----------------------------------------------------------------------------*/
