/*
 * rfm69.c
 *
 *  Created on: 22 december 2017
 *      Author: Pierre
 */

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "rfm69.hpp"
#include "utils.hpp"

/*----------------------------------------------------------------------------*/
/* Private functions */

//< Maximum amount of time until mode switch [ms]
#define TIMEOUT_MODE_READY 100
//< Maximum amount of time until packet must be sent [ms]
#define TIMEOUT_PACKET_SENT 100
//< Maximum CSMA wait time for channel free detection [ms]
#define TIMEOUT_CSMA_READY 500
//< If RSSI value is smaller than this, consider channel as free [dBm]
#define CSMA_RSSI_THRESHOLD -85
// puts the temperature reading in the ballpark, user can fine tune the returned
// value
#define COURSE_TEMP_COEF -90

/** RFM69 base configuration after init().
 *
 * Change these to your needs or call setCustomConfig() after module init.
 */
// clang-format off
static const uint8_t g_ppu8_rfm69BaseConfig[][2] = {
//      { /*0x01*/REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY }, // RegOpMode: Standby Mode
//      { /*0x02*/REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // RegDataModul: Packet mode, FSK, no shaping
//      { /*0x03*/REG_BITRATEMSB, RF_BITRATEMSB_55555 }, // RegBitrateMsb: 4.8 kbps
//      { /*0x04*/REG_BITRATELSB, RF_BITRATELSB_55555 }, // RegBitrateLsb
//      { /*0x05*/REG_FDEVMSB, RF_FDEVMSB_50000 }, // RegFdevMsb: default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
//      { /*0x06*/REG_FDEVLSB, RF_FDEVLSB_50000 }, // RegFdevLsb
//      { /*0x07*/REG_FRFMSB, RF_FRFMSB_433 }, // RegFrfMsb: 433MHz
//      { /*0x08*/REG_FRFMID, RF_FRFMID_433 }, // RegFrfMid
//      { /*0x09*/REG_FRFLSB, RF_FRFLSB_433 }, // RegFrfLsb
//      { /*0x18*/REG_LNA, RF_LNA_ZIN_200 }, // RegLNA: 200 Ohm impedance, gain set by AGC loop
//      { /*0x19*/REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
////      { /*0x25*/REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
//      { /*0x26*/REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
//      { /*0x2C*/REG_PREAMBLEMSB, 0x00 }, // RegPreambleMsb: 3 bytes preamble
//      { /*0x2D*/REG_PREAMBLELSB, 0x03 }, // RegPreambleLsb
//      { /*0x2E*/REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 }, // RegSyncConfig: Enable sync word, 2 bytes sync word
//      { /*0x2F*/REG_SYNCVALUE1, 0x2D }, // RegSyncValue1: 0x4148 (=networkId)
//      { /*0x30*/REG_SYNCVALUE2, 0x64 }, // RegSyncValue2
//      { /*0x37*/REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF }, // RegPacketConfig1: Variable length, CRC on, whitening
//      { /*0x38*/REG_PAYLOADLENGTH, RF_PAYLOADLENGTH_VALUE }, // RegPayloadLength: 64 bytes max payload
//      { /*0x3C*/REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // RegFifoThresh: TxStart on FifoNotEmpty, 15 bytes FifoLevel
//      { /*0x3D*/REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
////      { 0x58, 0x1B }, // RegTestLna: Normal sensitivity mode
//      { /*0x6F*/REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // RegTestDagc: Improved margin, use if AfcLowBetaOn=0 (default)
//};
//   /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_OFF | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
//   /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_CONTINUOUSNOBSYNC | RF_DATAMODUL_MODULATIONTYPE_OOK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
//   /* 0x03 */ { REG_BITRATEMSB, 0x03}, // bitrate: 32768 Hz
//   /* 0x04 */ { REG_BITRATELSB, 0xD1},
//   /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_4}, // BW: 10.4 kHz
//   /* 0x1B */ { REG_OOKPEAK, RF_OOKPEAK_THRESHTYPE_PEAK | RF_OOKPEAK_PEAKTHRESHSTEP_000 | RF_OOKPEAK_PEAKTHRESHDEC_000 },
//   /* 0x1D */ { REG_OOKFIX, 6 }, // Fixed threshold value (in dB) in the OOK demodulator
//   /* 0x29 */ { REG_RSSITHRESH, 140 }, // RSSI threshold in dBm = -(REG_RSSITHRESH / 2)
//   /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 } // run DAGC continuously in RX mode, recommended default for AfcLowBetaOn=0
// };
   /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
   /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_CONTINUOUSNOBSYNC | RF_DATAMODUL_MODULATIONTYPE_OOK | RF_DATAMODUL_MODULATIONSHAPING_00 }, //no shaping
   /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_4800 }, //default:4.8 KBPS
   /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_4800 }, //default:4.8 KBPS
   /* 0x05 */ //{ REG_FDEVMSB, RF_FDEVMSB_50000}, //default:5khz, (FDEV + BitRate/2 <= 500Khz)
   /* 0x06 */ //{ REG_FDEVLSB, RF_FDEVLSB_50000},

   /* 0x07 */ { REG_FRFMSB, RF_FRFMSB_433 },
   /* 0x08 */ { REG_FRFMID, RF_FRFMID_433 },
   /* 0x09 */ { REG_FRFLSB, RF_FRFLSB_433 },
//   /* 0x0D */ { REG_LISTEN1, 0x92 },
//   /* 0x0E */ { REG_LISTEN2, 0x49 },
//   /* 0x0F */ { REG_LISTEN3, 0xAA },

   // looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
   // +17dBm and +20dBm are possible on RFM69HW
   // +13dBm formula: Pout=-18+OutputPower (with PA0 or PA1**)
   // +17dBm formula: Pout=-14+OutputPower (with PA1 and PA2)**
   // +20dBm formula: Pout=-11+OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
   //* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
//   /* 0x12 */ { REG_PARAMP, RF_PARAMP_20 },
   //* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, //over current protection (default is 95mA)

   // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4khz)
   /* 0x18 */ { REG_LNA, RF_LNA_ZIN_200 | RF_LNA_GAINSELECT_AUTO }, //LNA input impedance 200 Ohm, auto gain select
   /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_001 | RF_RXBW_MANT_20 | RF_RXBW_EXP_1 }, // 20 en 5 berekend
   /* 0x1B */ { REG_OOKPEAK, RF_OOKPEAK_THRESHTYPE_FIXED | RF_OOKPEAK_PEAKTHRESHSTEP_000 | RF_OOKPEAK_PEAKTHRESHDEC_000 }, //74
   //* 0x1C */ { REG_OOKAVG, 0x80 },//80   //Geen idee waarvoor
//   /* 0x1D */ { REG_OOKFIX, 6 }, //0   //zie pagina 30 (default is 6)
   /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 | RF_DIOMAPPING1_DIO2_00 | RF_DIOMAPPING1_DIO3_01 }, //DIO0 is the only IRQ we're using
   /* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
   /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // Writing to this bit ensures the FIFO & status flags are reset
   /* 0x29 */ { REG_RSSITHRESH, 160 }, //220 }, //must be set to dBm = (-Sensitivity / 2) - default is 0xE4=228 so -114dBm
   /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 } // run DAGC continuously in RX mode, recommended default for AfcLowBetaOn=0
};
/*
{ REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // RegDataModul: Packet mode, FSK, no shaping
{ REG_BITRATEMSB, RF_BITRATEMSB_55555 }, // RegBitrateMsb: 4.8 kbps
{ REG_BITRATELSB, RF_BITRATELSB_55555 }, // RegBitrateLsb
{ REG_FDEVMSB, RF_FDEVMSB_50000 }, // RegFdevMsb: default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
{ REG_FDEVLSB, RF_FDEVLSB_50000 }, // RegFdevLsb
{ REG_FRFMSB, RF_FRFMSB_433 }, // RegFrfMsb: 433MHz
{ REG_FRFMID, RF_FRFMID_433 }, // RegFrfMid
{ REG_FRFLSB, RF_FRFLSB_433 }, // RegFrfLsb
{ REG_LNA, RF_LNA_ZIN_200 }, // RegLNA: 200 Ohm impedance, gain set by AGC loop
{ REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
{ REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
{ REG_PREAMBLEMSB, 0x00 }, // RegPreambleMsb: 3 bytes preamble
{ REG_PREAMBLELSB, 0x03 }, // RegPreambleLsb
{ REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 }, // RegSyncConfig: Enable sync word, 2 bytes sync word
{ REG_SYNCVALUE1, 0x2D }, // RegSyncValue1: 0x4148 (=networkId)
{ REG_SYNCVALUE2, 0x64 }, // RegSyncValue2
{ REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF }, // RegPacketConfig1: Variable length, CRC on, whitening
{ REG_PAYLOADLENGTH, RF_PAYLOADLENGTH_VALUE }, // RegPayloadLength: 64 bytes max payload
{ REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // RegFifoThresh: TxStart on FifoNotEmpty, 15 bytes FifoLevel
{ REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
{ REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // RegTestDagc: Improved margin, use if AfcLowBetaOn=0 (default)
*/
// clang-format on

/*----------------------------------------------------------------------------*/

uint8_t RFM69::readRegister(uint8_t u8_register) {
  register uint8_t u8_value;

  _o_spi.enableSlaveDevice();
  _o_spi.sendRecvData(RFM69_READ_REGISTER_MASK(u8_register));
  u8_value = _o_spi.sendRecvData(RFM69_NOP_MASK);
  _o_spi.disableSlaveDevice();

  return u8_value;
}

/*----------------------------------------------------------------------------*/

void RFM69::readRegisterMulti(uint8_t u8_register, uint8_t *pu8_data,
                              int i_count) {
  _o_spi.enableSlaveDevice();
  _o_spi.sendRecvData(RFM69_READ_REGISTER_MASK(u8_register));
  _o_spi.recvBuffer(pu8_data, RFM69_NOP_MASK, i_count);
  _o_spi.disableSlaveDevice();
}

/*----------------------------------------------------------------------------*/

void RFM69::writeRegister(uint8_t u8_register, uint8_t u8_value) {
  /*
   * SINGLE access: an address byte followed by a data byte is sent for a
   * write access whereas an address byte is sent and a read byte is received
   * for the read access. The NSS pin goes low at the begin of the frame and
   * goes high after the data byte.
   */
  _o_spi.enableSlaveDevice();
  _o_spi.sendRecvData(RFM69_WRITE_REGISTER_MASK(u8_register));
  _o_spi.sendRecvData(u8_value);
  _o_spi.disableSlaveDevice();
}

/*----------------------------------------------------------------------------*/

void RFM69::writeRegisterMulti(uint8_t u8_register, const uint8_t *pu8_data,
                               int i_count) {
  /*
   * BURST access: the address byte is followed by several data bytes.
   * The address is automatically incremented internally between each data
   * byte. This mode is available for both read and write accesses. The NSS
   * pin goes low at the beginning of the frame and stay low between each byte.
   * It goes high only after the last byte transfer.
   */
  _o_spi.enableSlaveDevice();
  _o_spi.sendRecvData(RFM69_WRITE_REGISTER_MASK(u8_register));
  _o_spi.sendBuffer(pu8_data, i_count);
  _o_spi.disableSlaveDevice();
}

/*----------------------------------------------------------------------------*/
/**
 * Reset the RFM69 module using the external reset line.
 *
 * @note RFM69_hardReset function should have been given when initializing the
 * device if calling this function.
 */
void RFM69::reset() {
  if (NULL != _pf_hardReset) {
    _pf_hardReset();
  }

  _e_mode = RFM69_MODE_STANDBY;
}

/*----------------------------------------------------------------------------*/
/**
 * Clear FIFO and flags of RFM69 module.
 */
void RFM69::clearFIFO() {
  // clear flags and FIFO
  // writing to this bit ensures that the FIFO & status flags are reset
  writeRegister(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);
}

/*----------------------------------------------------------------------------*/
/**
 * Wait until the requested mode is available or timeout.
 */
void RFM69::waitForModeReady() {
  int i_loop;

  i_loop = 0;
  while (((readRegister(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0) &&
         i_loop++ < TIMEOUT_MODE_READY) {
    //      vTaskDelay(pdMS_TO_TICKS(1));
    msleep(1);
  }
}

/*----------------------------------------------------------------------------*/
/**
 * Reconfigure the RFM69 module by writing multiple registers at once.
 *
 * @param config Array of register/value tuples
 * @param length Number of elements in config array
 */
void RFM69::setCustomConfig(const uint8_t ppu8_config[][2], int i_length) {
  for (int i = 0; i < i_length; i++) {
    writeRegister(ppu8_config[i][0], ppu8_config[i][1]);
  }
}

/*----------------------------------------------------------------------------*/
/**
 * RFM69 default constructor. Use init() to start working with the RFM69 module.
 *
 * @param spi Pointer to a SPI device
 * @param highPowerDevice Set to true, if this is a RFM69Hxx device (default:
 * false)
 */
uint8_t RFM69::init(SPI_BASE o_spiBase, SPI_ChipSelect pf_chipSelect,
                    RFM69_hardReset pf_hardReset, int i_highPowerDevice) {
  /*
   * The SPI interface gives access to the configuration register via a
   * synchronous full-duplex protocol corresponding to CPOL = 0 and CPHA = 0
   * in Motorola/Freescale nomenclature. Only the slave side is implemented.
   * The data byte is transmitted MSB first.
   */
  _o_spi.init(o_spiBase, pf_chipSelect);
  _o_spi.setFullDuplexMode();
  _o_spi.setMasterMode();
  _o_spi.setDff8bit();
  _o_spi.setClockPolarity0();
  _o_spi.setClockPhase1();
  _o_spi.setBaudratePrescaler(SPI_CR1_BR_FPCLK_DIV_16);
  _o_spi.setSendMsbFirst();
  _o_spi.enable();

  _e_dataMode = RFM69_DATA_MODE_PACKET;
  _e_mode = RFM69_MODE_STANDBY;
  _h_gpioPortData = 0;
  _h_gpioPinData = 0;
  _i_highPowerDevice = i_highPowerDevice;
  _i_highPowerSettings = false;
  _i_ookEnabled = false;
  _i_csmaEnabled = false;
  _i_autoReadRSSI = false;
  _pf_hardReset = pf_hardReset;
  _u8_nodeId = 2;
  _u32_networkId = 0x2D64;
  _u32_frequency = 0;

  /*
   * The user should wait for 10 ms from of the end of the POR cycle before
   * commencing communications over the SPI bus.
   */
  reset();
  // set base configuration
  setCustomConfig(g_ppu8_rfm69BaseConfig, sizeof(g_ppu8_rfm69BaseConfig) / 2);

  /* Return OK */
  return 1;
}

/*----------------------------------------------------------------------------*/
/**
 * Enable the +20 dBm high power settings of RFM69Hxx modules.
 *
 * @note Enabling only works with high power devices.
 *
 * @param enable true or false
 */
void RFM69::setHighPowerSettings(bool b_enable) {
  // enabling only works if this is a high power device
  if (b_enable && !_i_highPowerDevice) {
    b_enable = false;
  }

  writeRegister(0x5A, b_enable ? 0x5D : 0x55);
  writeRegister(0x5C, b_enable ? 0x7C : 0x70);

  _i_highPowerSettings = b_enable;
}

/*----------------------------------------------------------------------------*/
/**
 * Set the output power level in dBm.
 *
 * This function takes care of the different PA settings of the modules.
 * Depending on the requested power output setting and the available module,
 * PA0, PA1 or PA1+PA2 is enabled.
 *
 * @param dBm Output power in dBm
 * @return 0 if dBm valid; else -1.
 */
int RFM69::setPowerDBm(int8_t i8_dBm) {
  /* Output power of module is from -18 dBm to +13 dBm
   * in "low" power devices, -2 dBm to +20 dBm in high power devices */
  if (i8_dBm < -18 || i8_dBm > 20) {
    return -1;
  }

  if (!_i_highPowerDevice && i8_dBm > 13) {
    return -1;
  }

  if (_i_highPowerDevice && i8_dBm < -2) {
    return -1;
  }

  uint8_t u8_powerLevel = 0;

  if (!_i_highPowerDevice) {
    // only PA0 can be used
    u8_powerLevel = i8_dBm + 18;

    // enable PA0 only
    writeRegister(REG_PALEVEL, 0x80 | u8_powerLevel);
  } else {
    if (i8_dBm >= -2 && i8_dBm <= 13) {
      // use PA1 on pin PA_BOOST
      u8_powerLevel = i8_dBm + 18;

      // enable PA1 only
      writeRegister(REG_PALEVEL, 0x40 | u8_powerLevel);

      // disable high power settings
      setHighPowerSettings(false);
    } else if (i8_dBm > 13 && i8_dBm <= 17) {
      // use PA1 and PA2 combined on pin PA_BOOST
      u8_powerLevel = i8_dBm + 14;

      // enable PA1+PA2
      writeRegister(REG_PALEVEL, 0x60 | u8_powerLevel);

      // disable high power settings
      setHighPowerSettings(false);
    } else {
      // output power from 18 dBm to 20 dBm, use PA1+PA2 with high power
      // settings
      u8_powerLevel = i8_dBm + 11;

      // enable PA1+PA2
      writeRegister(REG_PALEVEL, 0x60 | u8_powerLevel);

      // enable high power settings
      setHighPowerSettings(true);
    }
  }

  return 0;
}

/*----------------------------------------------------------------------------*/
/**
 * Enable/disable the power amplifier(s) of the RFM69 module.
 *
 * PA0 for regular devices is enabled and PA1 is used for high power devices
 * (default).
 *
 * @note Use this function if you want to manually override the PA settings.
 * @note PA0 can only be used with regular devices (not the high power ones!)
 * @note PA1 and PA2 can only be used with high power devices (not the regular
 * ones!)
 *
 * @param forcePA If this is 0, default values are used. Otherwise, PA settings
 * are forced. 0x01 for PA0, 0x02 for PA1, 0x04 for PA2, 0x08 for +20 dBm high
 * power settings.
 */
void RFM69::setPASettings(uint8_t u8_forcePA) {
  uint8_t u8_value;
  uint8_t u8_pa;

  // disable OCP for high power devices, enable otherwise
  writeRegister(REG_OCP, 0x0A | (_i_highPowerDevice ? 0x00 : 0x10));

  u8_value = readRegister(REG_PALEVEL) & 0x1F;

  if (0 == u8_forcePA) {
    if (_i_highPowerDevice) {
      // enable PA1 only
      u8_pa = 0x40;
    } else {
      // enable PA0 only
      u8_pa = 0x80;
    }
  } else {
    // PA settings forced
    u8_pa = 0;

    if (u8_forcePA & 0x01) {
      u8_pa |= 0x80;
    }

    if (u8_forcePA & 0x02) {
      u8_pa |= 0x40;
    }

    if (u8_forcePA & 0x04) {
      u8_pa |= 0x20;
    }

    // check if high power settings are forced
    setHighPowerSettings((u8_forcePA & 0x08) ? TRUE : FALSE);
  }

  writeRegister(REG_PALEVEL, u8_value | u8_pa);
}

/*----------------------------------------------------------------------------*/
/**
 * Switch the mode of the RFM69 module.
 * Using this function you can manually select the RFM69 mode (sleep for
 * example).
 *
 * This function also takes care of the special registers that need to be set
 * when the RFM69 module is a high power device (RFM69Hxx).
 *
 * This function is usually not needed because the library handles mode changes
 * automatically.
 *
 * @param mode RFM69_MODE_SLEEP, RFM69_MODE_STANDBY, RFM69_MODE_FS,
 * RFM69_MODE_TX, RFM69_MODE_RX
 * @return The new mode
 */
RFM69Mode RFM69::setMode(RFM69Mode e_mode) {
  if ((e_mode == _e_mode) || (e_mode >= RFM69_MODE_ILLEGAL)) {
    return _e_mode;
  }

  // set new mode
  writeRegister(REG_OPMODE, (readRegister(REG_OPMODE) & 0xE3) | e_mode << 2);

  // set special registers if this is a high power device (RFM69HW)
  if (_i_highPowerDevice) {
    switch (e_mode) {
    case RFM69_MODE_RX:
      // normal RX mode
      if (_i_highPowerSettings) {
        setHighPowerSettings(false);
      }
      break;
    case RFM69_MODE_TX:
      // +20dBm operation on PA_BOOST
      if (_i_highPowerSettings) {
        setHighPowerSettings(true);
      }
      break;
    default:
      break;
    }
  }

  _e_mode = e_mode;

  return e_mode;
}

/*----------------------------------------------------------------------------*/
/**
 * Put the RFM69 module to sleep (lowest power consumption).
 */
void RFM69::sleep() {
  setMode(RFM69_MODE_SLEEP);
}

/*----------------------------------------------------------------------------*/
/**
 * Set this node's network ID.
 */
void RFM69::setNetworkId(uint32_t u32_networkId) {
  uint8_t u8_byte;
  register int i;

  _u32_networkId = u32_networkId;

  for (i = 4; i > 0; i--) {
    u8_byte = (u32_networkId & 0xFF000000) >> 24;
    if (u8_byte != 0) {
      break;
    }
    u32_networkId <<= 8;
  }

  writeRegister(REG_SYNCCONFIG,
                RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_TOL_0 | (i << 3));
  for (; i > 0; i--) {
    writeRegister(REG_SYNCVALUE1, u8_byte);
    u32_networkId <<= 8;
    u8_byte = (u32_networkId & 0xFF000000) >> 24;
  }
}

/*----------------------------------------------------------------------------*/
/**
 * Set this node's address.
 */
void RFM69::setNodeId(uint8_t u8_nodeId) {
  _u8_nodeId = u8_nodeId;
  writeRegister(REG_NODEADRS, u8_nodeId);
}

/*----------------------------------------------------------------------------*/
/**
 * Enable and set or disable AES hardware encryption/decryption.
 *
 * The AES encryption module will be disabled if an invalid key or key length
 * is passed to this function (aesKey = 0 or keyLength != 16).
 * Otherwise encryption will be enabled.
 *
 * The key is stored as MSB first in the RF module.
 *
 * @param aesKey Pointer to a buffer with the 16 byte AES key
 * @param keyLength Number of bytes in buffer aesKey; must be 16 bytes
 * @return State of encryption module (false = disabled; true = enabled)
 */
bool RFM69::setAESEncryption(const uint8_t *pu8_aesKey, int i_keyLength) {
  bool b_enable;
  uint8_t u8_value;

  // check if encryption shall be enabled or disabled
  b_enable = (NULL != pu8_aesKey) && (16 == i_keyLength);

  // switch to standby
  setMode(RFM69_MODE_STANDBY);

  if (b_enable) {
    // transfer AES key to AES key register
    writeRegisterMulti(REG_AESKEY1, pu8_aesKey, i_keyLength);
  }

  // set/reset AesOn Bit in packet config
  u8_value = readRegister(REG_PACKETCONFIG2) & 0xFE;
  writeRegister(REG_PACKETCONFIG2, u8_value | (b_enable ? 1 : 0));

  return b_enable;
}

/*----------------------------------------------------------------------------*/
/**
 * Enable/disable OOK modulation (On-Off-Keying).
 *
 * Default modulation is FSK.
 * The module is switched to standby mode if RX or TX was active.
 *
 * @param enable true or false
 */
void RFM69::setOOKMode(bool b_enable) {
  // switch to standby if TX/RX was active
  if (RFM69_MODE_RX == _e_mode || RFM69_MODE_TX == _e_mode) {
    setMode(RFM69_MODE_STANDBY);
  }

  uint8_t u8_value = readRegister(REG_DATAMODUL) & 0xE7;
  if (b_enable) {
    u8_value |= 0x08;
  }
  writeRegister(REG_DATAMODUL, u8_value);

  _i_ookEnabled = b_enable;
}

/*----------------------------------------------------------------------------*/
/**
 * Transmit a high or low bit in continuous mode using the external data line.
 *
 * @note Use setDataPin() before calling this function.
 * @note Call setDataMode() before to enable continuous mode.
 *
 * @param bit true: high bit; false: low bit
 */
void RFM69::continuousBit(bool b_signal) {
  // only allow this in continuous mode and if data pin was specified
  if ((RFM69_DATA_MODE_PACKET == _e_dataMode) || (0 == _h_gpioPinData)) {
    return;
  }

  // send low or high bit
  if (b_signal) {
    gpio_set(_h_gpioPortData, _h_gpioPinData);
  } else {
    gpio_clear(_h_gpioPortData, _h_gpioPinData);
  }
}

/*----------------------------------------------------------------------------*/
/**
 * Wait until packet has been sent over the air or timeout.
 */
void RFM69::waitForPacketSent() {
  int i_loop;

  i_loop = 0;
  while (((readRegister(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT) == 0) &&
         i_loop++ < TIMEOUT_PACKET_SENT) {
    msleep(1);
  }
}

/*----------------------------------------------------------------------------*/
/**
 * Read the last RSSI (Received Signal Strength Indicator) value.
 *
 * @note Only if the last RSSI value was above the RSSI threshold, a sample can
 * be read. Otherwise, you always get -127 dBm. Be also careful if you just
 * switched to RX mode. You may have to wait until a RSSI sample is available.
 *
 * @return RSSI value in dBm.
 */
int RFM69::readRSSI(bool b_forceTrigger) {
  if (b_forceTrigger) {
    // RSSI trigger not needed if DAGC is in continuous mode
    writeRegister(REG_RSSICONFIG, RF_RSSI_START);
    while ((readRegister(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00) {
      // Wait for RSSI_Ready
    }
  }
  return -readRegister(REG_RSSIVALUE) / 2;
}

/*----------------------------------------------------------------------------*/
/**
 * Check if the channel is free using RSSI measurements.
 *
 * This function is part of the CSMA/CA algorithm.
 *
 * @return true = channel free; otherwise false.
 */
int RFM69::channelFree() {
  return readRSSI(false) < CSMA_RSSI_THRESHOLD;
}

/*----------------------------------------------------------------------------*/
/**
 * Put the RFM69 module in RX mode and try to receive a packet.
 *
 * @note This is an internal function.
 * @note The module resides in RX mode.
 *
 * @param data Pointer to a receiving buffer
 * @param dataLength Maximum size of buffer
 * @return Number of received bytes; 0 if no payload is available.
 */
int RFM69::internalReceive(uint8_t *pu8_data, int i_dataLength) {
  // go to RX mode if not already in this mode
  if (RFM69_MODE_RX != _e_mode) {
    setMode(RFM69_MODE_RX);
    waitForModeReady();
  }

  // check for flag PayloadReady
  if (readRegister(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY) {
    // go to standby before reading data
    setMode(RFM69_MODE_STANDBY);

    // get FIFO content
    int i_bytesRead = 0;

    // read until FIFO is empty or buffer length exceeded
    while ((readRegister(REG_IRQFLAGS2) & RF_IRQFLAGS2_FIFONOTEMPTY) &&
           (i_bytesRead < i_dataLength)) {
      // read next byte
      pu8_data[i_bytesRead] = readRegister(0x00);
      i_bytesRead++;
    }

    // automatically read RSSI if requested
    if (_i_autoReadRSSI) {
      readRSSI(false);
    }

    // go back to RX mode
    setMode(RFM69_MODE_RX);
    // todo: wait needed?
    //      waitForModeReady();

    return i_bytesRead;
  }

  return 0;
}

/*----------------------------------------------------------------------------*/
/**
 * Put the RFM69 module in RX mode and try to receive a packet.
 *
 * @note The module resides in RX mode.
 *
 * @param data Pointer to a receiving buffer
 * @param dataLength Maximum size of buffer
 * @return Number of received bytes; 0 if no payload is available.
 */
int RFM69::receive(uint8_t *pu8_data, int i_dataLength) {
  // check if there is a packet in the internal buffer and copy it
  if (_u8_rxBufferLength > 0) {
    // copy only until dataLength, even if packet in local buffer is actually
    // larger
    memcpy(pu8_data, _pu8_rxBuffer, (size_t) i_dataLength);

    int i_bytesRead = _u8_rxBufferLength;

    // empty local buffer
    _u8_rxBufferLength = 0;

    return i_bytesRead;
  } else {
    // regular receive
    return internalReceive(pu8_data, i_dataLength);
  }
}

/*----------------------------------------------------------------------------*/
/**
 * Send a packet over the air.
 *
 * After sending the packet, the module goes to standby mode.
 * CSMA/CA is used before sending if enabled by function setCSMA() (default:
 * off).
 *
 * @note A maximum amount of RFM69_MAX_PAYLOAD bytes can be sent.
 * @note This function blocks until packet has been sent.
 *
 * @param data Pointer to buffer with data
 * @param dataLength Size of buffer
 *
 * @return Number of bytes that have been sent
 */
int RFM69::send(const uint8_t *pu8_data, int i_dataLength) {
  uint8_t u8_value;
  int i_loop;

  // switch to standby and wait for mode ready, if not in sleep mode
  if (RFM69_MODE_SLEEP != _e_mode) {
    setMode(RFM69_MODE_STANDBY);
    waitForModeReady();
  }

  // clear FIFO to remove old data and clear flags
  clearFIFO();

  // limit max payload
  if (i_dataLength > RFM69_MAX_PAYLOAD) {
    i_dataLength = RFM69_MAX_PAYLOAD;
  }

  // payload must be available
  if (i_dataLength <= 0) {
    return 0;
  }

  /* Wait for a free channel, if CSMA/CA algorithm is enabled.
   * This takes around 1,4 ms to finish if channel is free */
  if (_i_csmaEnabled) {
    // Restart RX
    u8_value = readRegister(REG_PACKETCONFIG2) & 0xFB;
    writeRegister(REG_PACKETCONFIG2, u8_value | 0x20);

    // switch to RX mode
    setMode(RFM69_MODE_RX);

    // wait until RSSI sampling is done; otherwise, 0xFF (-127 dBm) is read
    // RSSI sampling phase takes ~960 µs after switch from standby to RX
    i_loop = 0;
    while (((readRegister(REG_RSSICONFIG) & RF_RSSI_DONE) == 0) &&
           i_loop++ < 10) {
      //         vTaskDelay(pdMS_TO_TICKS(1));
      asm("nop");
    }

    uint32_t u32_timeout = millis() + TIMEOUT_CSMA_READY;
    //      TickType_t i_timeout =
    //            xTaskGetTickCount() + pdMS_TO_TICKS(TIMEOUT_CSMA_READY);
    while (!channelFree() && millis() < u32_timeout) {
      // wait for a random time before checking again
      uint32_t u32_waitTime = millis() + rand() % 10;
      if (u32_waitTime > u32_timeout) {
        u32_waitTime = u32_timeout;
      }
      while (millis() < u32_waitTime) {
        asm("nop");
      }
      //         vTaskDelay(pdMS_TO_TICKS(rand() % 10));

      /* try to receive packets while waiting for a free channel
       * and put them into a temporary buffer */
      int i_bytesRead;

      i_bytesRead = internalReceive(_pu8_rxBuffer, RFM69_MAX_PAYLOAD);
      if (i_bytesRead > 0) {
        _u8_rxBufferLength = i_bytesRead;

        // module is in RX mode again

        // Restart RX and wait until RSSI sampling is done
        u8_value = readRegister(REG_PACKETCONFIG2) & 0xFB;
        writeRegister(REG_PACKETCONFIG2, u8_value | 0x20);

        i_loop = 0;
        while (((readRegister(REG_RSSICONFIG) & RF_RSSI_DONE) == 0) &&
               i_loop++ < 10) {
          //               vTaskDelay(pdMS_TO_TICKS(1));
          asm("nop");
        }
      }
    }

    setMode(RFM69_MODE_STANDBY);
  }

  u8_value = readRegister(REG_PACKETCONFIG1);
  writeRegister(REG_PACKETCONFIG1, (u8_value & 0x0F) |
                                     RF_PACKET1_FORMAT_VARIABLE |
                                     RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON);

  // transfer packet to FIFO
  _o_spi.enableSlaveDevice();
  // address FIFO
  _o_spi.sendRecvData(RFM69_WRITE_REGISTER_MASK(REG_FIFO));
  // send length byte
  _o_spi.sendRecvData((uint8_t) i_dataLength);
  // send length byte
  for (i_loop = 0; i_loop < i_dataLength; i_loop++) {
    _o_spi.sendRecvData(pu8_data[i_loop]);
  }
  _o_spi.disableSlaveDevice();

  // start radio transmission
  setMode(RFM69_MODE_TX);

  // wait for packet sent
  waitForPacketSent();

  // go to standby
  setMode(RFM69_MODE_STANDBY);

  return i_dataLength;
}

/*----------------------------------------------------------------------------*/
/**
 * Configure the data mode of the RFM69 module.
 *
 * Default data mode is 'packet'. You can choose between 'packet',
 * 'continuous with clock recovery', 'continuous without clock recovery'.
 * For continuous mode, see the DIO mapping 0x04 in
 * "Table 21 DIO Mapping, Continuous Mode" page 48
 *
 * The module is switched to standby mode if RX or TX was active.
 *
 * @param dataMode RFM69_DATA_MODE_PACKET, RFM69_DATA_MODE_CONTINUOUS_WITH_SYNC,
 * RFM69_DATA_MODE_CONTINUOUS_WITHOUT_SYNC
 */
void RFM69::setDataMode(RFM69DataMode e_dataMode) {
  uint8_t u8_value;

  // switch to standby if TX/RX was active
  if (RFM69_MODE_RX == _e_mode || RFM69_MODE_TX == _e_mode) {
    setMode(RFM69_MODE_STANDBY);
  }

  u8_value = readRegister(REG_DATAMODUL) & 0x1F;
  switch (e_dataMode) {
  case RFM69_DATA_MODE_PACKET:
    writeRegister(REG_DATAMODUL, u8_value);
    writeRegister(
      REG_DIOMAPPING1,
      RF_DIOMAPPING1_DIO0_00); // Dio0Mapping = 00 (DIO0 is "Packet Sent")
    break;

  case RFM69_DATA_MODE_CONTINUOUS_WITH_SYNC:
    writeRegister(REG_DATAMODUL, u8_value | 0x40);
    writeRegister(REG_DIOMAPPING1,
                  RF_DIOMAPPING1_DIO2_01); // Dio2Mapping = 01 (Data)
                                           //      continuousBit(false);
    break;

  case RFM69_DATA_MODE_CONTINUOUS_WITHOUT_SYNC:
    writeRegister(REG_DATAMODUL, u8_value | 0x60);
    writeRegister(REG_DIOMAPPING1,
                  RF_DIOMAPPING1_DIO2_01); // Dio2Mapping = 01 (Data)
                                           //      continuousBit(false);
    break;

  default:
    return;
  }

  _e_dataMode = e_dataMode;
}

/*----------------------------------------------------------------------------*/

void RFM69::setDataPin(GPIO_PORT h_gpioPortData, GPIO_PIN h_gpioPinData) {
  _h_gpioPortData = h_gpioPortData;
  _h_gpioPinData = h_gpioPinData;
}

/*----------------------------------------------------------------------------*/
/**
 * Set the carrier frequency in Hz. Should use macro RFM69_FREQ_TO_REG()
 * After calling this function, the module is in standby mode.
 *
 * @param frequency Carrier frequency in Hz
 */
void RFM69::setFrequency(uint32_t u32_frequency) {
  if (u32_frequency == _u32_frequency) {
    return;
  }

  // switch to standby if TX/RX was active
  if (RFM69_MODE_RX == _e_mode || RFM69_MODE_TX == _e_mode) {
    setMode(RFM69_MODE_STANDBY);
  }

  // calculate register value
  //   u32_frequency /= RFM69_FSTEP;

  // set new frequency
  writeRegister(REG_FRFMSB, u32_frequency >> 16);
  writeRegister(REG_FRFMID, u32_frequency >> 8);
  writeRegister(REG_FRFLSB, u32_frequency);

  _u32_frequency = u32_frequency;
}

/*----------------------------------------------------------------------------*/
/**
 * Set the FSK frequency deviation in Hz.
 * After calling this function, the module is in standby mode.
 *
 * @param frequency Frequency deviation in Hz
 */
void RFM69::setFrequencyDeviation(uint32_t u32_frequency) {
  // switch to standby if TX/RX was active
  if (RFM69_MODE_RX == _e_mode || RFM69_MODE_TX == _e_mode) {
    setMode(RFM69_MODE_STANDBY);
  }

  // calculate register value
  u32_frequency /= RFM69_FSTEP;

  // set new frequency
  writeRegister(REG_FDEVMSB, u32_frequency >> 8);
  writeRegister(REG_FDEVLSB, u32_frequency);
}

/*----------------------------------------------------------------------------*/
/**
 * Set the bitrate in bits per second. Should use macro RFM69_BITRATE_TO_REG()
 * After calling this function, the module is in standby mode.
 *
 * @param bitrate Bitrate in bits per second
 */
void RFM69::setBitRate(uint32_t u32_bitrate) {
  // switch to standby if TX/RX was active
  if (RFM69_MODE_RX == _e_mode || RFM69_MODE_TX == _e_mode) {
    setMode(RFM69_MODE_STANDBY);
  }

  // calculate register value
  //   u32_bitrate = RFM69_XO / u32_bitrate;

  // set new bitrate
  writeRegister(REG_BITRATEMSB, u32_bitrate >> 8);
  writeRegister(REG_BITRATELSB, u32_bitrate);
}

/*----------------------------------------------------------------------------*/
/**
 * Debug function to dump all RFM69 registers.
 *
 * Symbol 'DEBUG' has to be defined.
 */
void RFM69::dumpRegisters() {
  uint8_t u8_value;

  for (uint i = 1; i <= 0x71; i++) {
    u8_value = readRegister(i);
    fOutput("[0x%02X]: 0x%02X 0b%08b\r\n", i, u8_value, u8_value);
  }
}

/*----------------------------------------------------------------------------*/

uint8_t RFM69::readTemperature(uint8_t u8_calFactor) // returns centigrade
{
  if (RFM69_MODE_RX == _e_mode || RFM69_MODE_TX == _e_mode) {
    setMode(RFM69_MODE_STANDBY);
  }

  writeRegister(REG_TEMP1, RF_TEMP1_MEAS_START);
  while ((readRegister(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING)) {
  }

  // 'complement' corrects the slope, rising temp = rising val
  // COURSE_TEMP_COEF puts reading in the ballpark, user can add additional
  // correction
  return ~readRegister(REG_TEMP2) + COURSE_TEMP_COEF + u8_calFactor;
}

/*----------------------------------------------------------------------------*/

void RFM69::rcCalibration() {
  writeRegister(REG_OSC1, RF_OSC1_RCCAL_START);
  while ((readRegister(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00) {
  }
}

/*----------------------------------------------------------------------------*/
