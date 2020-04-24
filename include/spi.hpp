#ifndef SPI_HPP
#define SPI_HPP

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#define SPI_HIGH 1
#define SPI_LOW 0

typedef void (*SPI_ChipSelect)(int);

/*-----------------------------------------------------------*/
/**
 * @brief  Check SPI busy status
 */
//#define SPI_IS_BUSY(SPIx) (((SPIx)->SR & (SPI_SR_TXE | SPI_SR_RXNE)) == 0 ||
//((SPIx)->SR & SPI_SR_BSY))

/**
 * @brief  SPI wait till end
 */
//#define SPI_WAIT(SPIx)            while (SPI_IS_BUSY(SPIx))

/**
 * @brief  Checks if SPI is enabled
 */
//#define SPI_CHECK_ENABLED(SPIx)   if (!((SPIx)->CR1 & SPI_CR1_SPE)) {return;}

/**
 * @brief  Checks if SPI is enabled and returns value from function if not
 */
//#define SPI_CHECK_ENABLED_RESP(SPIx, val)   if (!((SPIx)->CR1 & SPI_CR1_SPE))
//{return (val);}

//#define SPI_DUMMY                               0

/*----------------------------------------------------------------------------*/

typedef uint32_t SPI_BASE;
typedef uint32_t GPIO_PORT;
typedef uint16_t GPIO_PIN;

class SPI {
private:
  SPI_BASE _o_spi;
  SPI_ChipSelect _pf_chipSelect;

public:
  inline void enable() { spi_enable(_o_spi); }

  inline void disable() { spi_disable(_o_spi); }

  inline void cleanDisable() { spi_clean_disable(_o_spi); }

  inline void setMasterMode() { spi_set_master_mode(_o_spi); }

  inline void setSlaveMode() { spi_set_slave_mode(_o_spi); }

  inline void setFullDuplexMode() { spi_set_full_duplex_mode(_o_spi); }

  inline void setReceiveOnlyMode() { spi_set_receive_only_mode(_o_spi); }

  inline void setUnidirectionalMode() { spi_set_unidirectional_mode(_o_spi); }

  inline void setBidirectionalMode() { spi_set_bidirectional_mode(_o_spi); }

  inline void setClockPolarity0() { spi_set_clock_polarity_0(_o_spi); }

  inline void setClockPolarity1() { spi_set_clock_polarity_1(_o_spi); }

  inline void setClockPhase0() { spi_set_clock_phase_0(_o_spi); }

  inline void setClockPhase1() { spi_set_clock_phase_1(_o_spi); }

  inline void setBaudratePrescaler(uint8_t u8_baudrate) {
    spi_set_baudrate_prescaler(_o_spi, u8_baudrate);
  }

  inline void setSendLsbFirst() { spi_send_lsb_first(_o_spi); }

  inline void setSendMsbFirst() { spi_send_msb_first(_o_spi); }

  inline void enableCrc() { spi_enable_crc(_o_spi); }

  inline void disableCrc() { spi_disable_crc(_o_spi); }

  inline void setDff8bit() { spi_set_dff_8bit(_o_spi); }

  inline void setDff16bit() { spi_set_dff_16bit(_o_spi); }

  inline void enableTxBufferEmptyInterrupt() {
    spi_enable_tx_buffer_empty_interrupt(_o_spi);
  }

  inline void disableTxBufferEmptyInterrupt() {
    spi_disable_tx_buffer_empty_interrupt(_o_spi);
  }

  inline void enableRxBufferNotEmptyInterrupt() {
    spi_enable_rx_buffer_not_empty_interrupt(_o_spi);
  }

  inline void disableRxBufferNotEmptyInterrupt() {
    spi_disable_rx_buffer_not_empty_interrupt(_o_spi);
  }

  inline void enableErrorInterrupt() { spi_enable_error_interrupt(_o_spi); }

  inline void disableErrorInterrupt() { spi_disable_error_interrupt(_o_spi); }

  void enableSlaveDevice();
  void disableSlaveDevice();

  inline uint16_t sendRecvData(uint16_t u16_data) {
    return spi_xfer(_o_spi, u16_data);
  }

  inline void sendData(uint16_t u16_data) { spi_send(_o_spi, u16_data); }

  void sendRecvBuffer(uint8_t *pu8_send, uint8_t *pu8_recv, int i_size);
  void sendRecvBuffer(uint16_t *pu16_send, uint16_t *pu16_recv, int i_size);

  void sendBuffer(const uint8_t *pu8_send, int i_size);
  void sendBuffer(const uint16_t *pu16_send, int i_size);

  inline uint16_t recvData() { return spi_read(_o_spi); }

  void recvBuffer(uint8_t *pu8_buffer, uint8_t u8_nopMask, int i_size);
  void recvBuffer(uint16_t *pu16_buffer, uint16_t u16_nopMask, int i_size);

  void init(SPI_BASE o_spi, SPI_ChipSelect pf_chipSelect);

  void reconfigure(uint32_t u32_cr1Config);
};

/*----------------------------------------------------------------------------*/

#endif /* SPI_HPP */
