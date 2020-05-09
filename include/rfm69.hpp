#ifndef RFM69_H
#define RFM69_H 100
/**
 * Includes
 */

#include "standard.hpp"
#include "spi.hpp"
#include "usb.hpp"
#include "rfm69_registers.hpp"

/*----------------------------------------------------------------------------*/

#define RFM69_WRITE_REGISTER_MASK(reg) ((reg & 0x7f) | 0x80)
#define RFM69_READ_REGISTER_MASK(reg) (reg & 0x7f)

#define RFM69_NOP_MASK 0xff

#define RFM69_HIGH 1
#define RFM69_LOW 0

#define RFM69_MAX_PAYLOAD 64 ///< Maximum bytes payload

// Clock constants. DO NOT CHANGE THESE!
#define RFM69_XO 32000000       ///< Internal clock frequency [Hz]
#define RFM69_FSTEP 61.03515625 ///< Step width of synthesizer [Hz]
// Frequency steps are in units of (32,000,000 >> 19) = 61.03515625 Hz

#define RFM69_FREQ_TO_REG(x) ((x) / RFM69_FSTEP)
#define RFM69_BITRATE_TO_REG(x) (RFM69_XO / (x))

/*----------------------------------------------------------------------------*/

typedef void (*RFM69_hardReset)(void);

/**
 * Valid RFM69 operation modes.
 */
enum _RFM69Mode {
  RFM69_MODE_SLEEP = 0, //!< Sleep mode (lowest power consumption)
  RFM69_MODE_STANDBY,   //!< Standby mode
  RFM69_MODE_FS,        //!< Frequency synthesizer enabled
  RFM69_MODE_TX,        //!< TX mode (carrier active)
  RFM69_MODE_RX,        //!< RX mode
  RFM69_MODE_ILLEGAL
};
typedef enum _RFM69Mode RFM69Mode;

/**
 * Valid RFM69 data modes.
 */
typedef enum {
  //!< Packet engine active
  RFM69_DATA_MODE_PACKET = 0,
  //!< Continuous mode with clock recovery
  RFM69_DATA_MODE_CONTINUOUS_WITH_SYNC = 2,
  //!< Continuous mode without clock recovery
  RFM69_DATA_MODE_CONTINUOUS_WITHOUT_SYNC = 3
} RFM69DataMode;

/*----------------------------------------------------------------------------*/

class RFM69 {
private:
  SPI _o_spi;
  RFM69_hardReset _pf_hardReset;
  RFM69DataMode _e_dataMode;
  RFM69Mode _e_mode;

  GPIO_PORT _h_gpioPortData;
  GPIO_PIN _h_gpioPinData;

  uint32_t _u32_frequency;
  uint32_t _u32_networkId;
  uint8_t _u8_nodeId;

  uint8_t _pu8_rxBuffer[RFM69_MAX_PAYLOAD];
  uint8_t _u8_rxBufferLength;

  bool _i_highPowerDevice;
  bool _i_highPowerSettings;
  bool _i_ookEnabled;
  bool _i_csmaEnabled;
  bool _i_autoReadRSSI;

public:
  uint8_t readRegister(uint8_t u8_register);
  void readRegisterMulti(uint8_t u8_register, uint8_t *pu8_data, int i_count);
  void writeRegister(uint8_t u8_register, uint8_t u8_value);
  void writeRegisterMulti(uint8_t u8_register, const uint8_t *pu8_data,
                          int i_count);

private:
  int internalReceive(uint8_t *pu8_data, int i_dataLength);
  void clearFIFO();

public:
  uint8_t init(SPI_BASE o_spi_base, SPI_ChipSelect pf_chipSelect,
               RFM69_hardReset pf_hardReset, int i_highPowerDevice);
  void setCustomConfig(const uint8_t config[][2], int i_length);
  int setPowerDBm(int8_t i8_dBm);
  void setOOKMode(bool b_enable);
  void setHighPowerSettings(bool b_enable);
  void setPASettings(uint8_t u8_forcePA);
  void setNetworkId(uint32_t u32_networkId);
  void setNodeId(uint8_t u8_nodeId);
  void setFrequency(uint32_t u32_frequency);
  void setFrequencyDeviation(uint32_t u32_frequency);
  void setBitRate(uint32_t u32_bitrate);
  void setDataMode(RFM69DataMode e_dataMode);
  void setDataPin(GPIO_PORT h_gpioPortData, GPIO_PIN h_gpioPinData);
  inline void setRssiThreshold(uint8_t u8_rssiThreshold) {
    writeRegister(REG_RSSITHRESH, (-u8_rssiThreshold << 1));
  }
  inline void setFixedThreshold(uint8_t u8_threshold) {
    writeRegister(REG_OOKFIX, (-u8_threshold << 1));
  }
  inline void setOOKPeak(uint8_t u8_value) {
    writeRegister(REG_OOKPEAK, u8_value);
  }
  inline void restartReception() {
    writeRegister(REG_PACKETCONFIG2,
                  readRegister(REG_PACKETCONFIG2) | RF_PACKET2_RESTART_RX);
  }
  void testReset();

  RFM69Mode setMode(RFM69Mode e_mode);
  void waitForModeReady();
  void waitForPacketSent();
  void continuousBit(bool b_signal);
  int receive(uint8_t *pu8_data, int i_dataLength);
  int send(const uint8_t *pu8_data, int i_dataLength);
  int readRSSI(bool b_forceTrigger);
  uint8_t readTemperature(uint8_t u8_calFactor); // returns centigrade
  int channelFree();
  void sleep();
  void reset();
  void rcCalibration();
  bool setAESEncryption(const uint8_t *pu8_aesKey, int i_keyLength);
  void dumpRegisters(USB &o_usb);
  inline uint8_t getVersion() { return readRegister(REG_VERSION); }

public:
  inline uint8_t getNodeId() { return _u8_nodeId; }
};

/*-----------------------------------------------------------*/

#endif
