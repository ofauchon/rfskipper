// Simple RF69 demo application.

#include <stdio.h>
#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/cortex.h>

#include "radio.hpp"
#include "standard.hpp"
#include "rfm69.hpp"
#ifdef USART_DEBUG
#include "usart.hpp"
#endif
#include "usb.hpp"
#include "ring.hpp"
#include "eventqueue.hpp"
#include "plugin.hpp"

/*----------------------------------------------------------------------------*/

// Timeout, after this time in mSec. the RF signal will be considered to have
// stopped.
#define SIGNAL_TIMEOUT 7
// =8 bits. Minimal number of bits*2 that need to have been received before we
// spend CPU time on decoding the signal.
#define MIN_RAW_PULSES 20

#define EVENT_RF_STOP_IN 30
#define EVENT_RF_STOP_OUT 25
#define EVENT_RF_START_OUT 20
#define EVENT_USB_OUT 16
#define EVENT_USART_OUT 15
#define EVENT_USB_IN 11
#define EVENT_USART_IN 10
#define EVENT_RF_START_IN 5

#define COMMAND_LINE_SIZE 60

#define LED_IDLE 0
#define LED_START_BLINK 1
#define LED_ON 2
#define LED_OFF 3

/*----------------------------------------------------------------------------*/

typedef enum _Rfm69State {
  IDLE,
  LISTENING,
  RECEIVING,
  PROCESSING,
  SENDING
} Rfm69State;

// clang-format off
const Command ps_commands[] = {
   { "RTSSHOW", plugin099Show },
   { "RTSCLEAN", plugin099Clean }
};
// clang-format on

const uint8_t u8_commands = sizeof(ps_commands) / sizeof(Command);

// local variables
namespace {
RFM69 o_rfm69;
RING<256> o_cmdIn;
#ifdef USART_DEBUG
RING<128> o_cmdOut;
#endif
EventQueue o_queue;
bool b_rfDebug = false;
volatile uint32_t u32_txTimeout = 0;
volatile Rfm69State e_state = IDLE;
RawSignal s_rawSignalIn;
RawSignal s_rawSignalOut;
uint8_t u8_ledState;
} // namespace

// global variables accessed from the plugins
USB o_usb;
#ifdef USART_DEBUG
USART o_usart;
#endif
uint8_t u8_sequenceNumber = 0;

/*----------------------------------------------------------------------------*/
/**
 * @brief  LDR Exclusive (8 bit)
 *
 * @param  *addr  address pointer
 * @return        value of (*address)
 *
 * Exclusive LDR command for 8 bit value
 */
uint8_t __LDREXB(uint8_t *addr) {
  uint8_t result = 0;

  __asm__ volatile("ldrexb %0, [%1]" : "=&r"(result) : "r"(addr));
  return (result);
}

/*----------------------------------------------------------------------------*/
/**
 * @brief  STR Exclusive (8 bit)
 *
 * @param  value  value to store
 * @param  *addr  address pointer
 * @return        successful / failed
 *
 * Exclusive STR command for 8 bit values
 */
uint32_t __STREXB(uint8_t value, uint8_t *addr) {
  uint32_t result = 0;

  __asm__ volatile("strexb %0, %2, [%1]"
                   : "=&r"(result)
                   : "r"(addr), "r"(value));
  return (result);
}

/*----------------------------------------------------------------------------*/
/*
 * Test and set a state. If the current state is equal to the "e_currentState",
 * the state is changed to "e_nextState" and a value of 0 is returned.
 * If the current state value is different, a value of 2 is returned.
 * Finally, if we can't lock the state, a value of 1 is returned.
 */
int __attribute__((noinline))
testAndSetState(volatile Rfm69State &e_state, Rfm69State e_currentState,
                Rfm69State e_nextState) {
  int i_result;

  __asm__ __volatile__("   LDREXB  r1, %[state]\n"
                       "   CMP     %[current], r1\n"
                       "   BNE     stateChanged\n"
                       "   STREXB  r1, %[next], %[state]\n"
                       "   DMB\n"
                       "   B       saveResult\n"
                       "stateChanged:\n"
                       "   MOV     r1, #2\n"
                       "saveResult:\n"
                       "   MOV     %[result], r1\n"
                       : [ result ] "=r"(i_result), [ state ] "=m"(e_state)
                       : [ current ] "r"(e_currentState),
                         [ next ] "r"(e_nextState)
                       : "r1");

  return i_result;
}

/*----------------------------------------------------------------------------*/
#ifdef USART_DEBUG
extern "C" void usart1_isr(void) {
  /* Check if we were called because of RXNE. */
  if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
      ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {
    int16_t u16_data = usart_recv(USART1);

    /* Retrieve the data from the peripheral. */
    o_cmdIn.writeByte(u16_data);
    if (u16_data == '\n') {
      o_queue.put(EVENT_USART_IN, 0);
    }

    /* Enable transmit interrupt so it sends back the data. */
    //      USART_CR1(USART1) |= USART_CR1_TXEIE;
  }

  /* Check if we were called because of TXE. */
  if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
      ((USART_SR(USART1) & USART_SR_TXE) != 0)) {

    int i_data = o_cmdOut.readByte();
    if (i_data == -1) {
      /* Disable the TXE interrupt, it's no longer needed. */
      usart_disable_tx_interrupt(USART1);
    } else {
      /* Put data into the transmit register. */
      usart_send(USART1, i_data);
    }
  }
}
#endif

/*----------------------------------------------------------------------------*/

extern "C" void sys_tick_handler() {
  ++u32_systemMillis;

  // test for reception timeout
  if (u32_txTimeout != 0 && u32_systemMillis >= u32_txTimeout &&
      e_state == RECEIVING) {
    u32_txTimeout = 0;

    timer_ic_disable(TIM3, TIM_IC3);
    timer_clear_flag(TIM3, TIM_SR_CC3IF | TIM_SR_CC3OF);

    if (testAndSetState(e_state, RECEIVING, PROCESSING) == 0) {
      o_queue.put(EVENT_RF_STOP_IN, 0);
    }
  }
}

/*----------------------------------------------------------------------------*/

void rfm69Reset(void) {
  /*
   * A manual reset of the SX1231 is possible even for applications in which VDD
   * cannot be physically disconnected. Pin 6 should be pulled high for a
   * hundred microseconds, and then released. The user should then wait for 5 ms
   * before using the chip.
   */
  gpio_set(GPIOA, GPIO0);
  msleep(1);
  gpio_clear(GPIOA, GPIO0);
  msleep(5);
}

/*----------------------------------------------------------------------------*/

void timer3Init() {
  /* Enable TIM3 clock. */
  rcc_periph_clock_enable(RCC_TIM3);

  /* Reset TIM3 peripheral to defaults. */
  rcc_periph_reset_pulse(RST_TIM3);

  /* Enable TIM3 interrupt. */
  nvic_enable_irq(NVIC_TIM3_IRQ);
}

/*----------------------------------------------------------------------------*/

void timer3SetupInputCapture(uint16_t u16_autoReload, uint16_t u16_prescaler) {
  gpio_set_mode(GPIO_BANK_TIM3_CH3, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN,
                GPIO_TIM3_CH3);

  /* Timer global mode:
   * - Internal clock with no divider (72Mhz)
   * - Alignment edge
   * - Direction up
   * (These are actually default values after reset above, so this call
   * is strictly unnecessary, but demos the api for alternative settings)
   */
  timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

  // The timer clock is prescaled by the 16 bit scale value plus 1
  timer_set_prescaler(TIM3, u16_prescaler);
  // Specify the timer period in the auto-reload register
  timer_set_period(TIM3, u16_autoReload);
  //   timer_set_clock_division(TIM3, TIM_CR1_CKD_CK_INT);
  //   timer_direction_up(TIM3);

  /*
   *  This causes the counter to be loaded immediately with a new count value
   *  when the auto-reload register is written, so that the new value becomes
   *  effective for the current count cycle rather than for the cycle following
   *  an update event.
   */
  timer_disable_preload(TIM3);
  // Enable the Timer to run continuously
  timer_continuous_mode(TIM3);

  /*
   * TIM3 configuration: Input Capture mode
   * The external signal is connected to TIM3 CH3 pin (PB.00)
   * The Rising edge is used as active edge
   */
  timer_ic_set_input(TIM3, TIM_IC3, TIM_IC_IN_TI3);
  timer_ic_set_polarity(TIM3, TIM_IC3, TIM_IC_RISING);
  timer_ic_set_prescaler(TIM3, TIM_IC3, TIM_IC_PSC_OFF);
  timer_ic_set_filter(TIM3, TIM_IC3, TIM_IC_OFF);

  timer_clear_flag(TIM3, TIM_SR_CC3IF | TIM_SR_CC3OF);
  timer_ic_enable(TIM3, TIM_IC3);

  // Enable Timer 3 Channel 3 compare interrupt
  timer_enable_irq(TIM3, TIM_DIER_CC3IE);

  // Enable Timer 3 counter
  timer_enable_counter(TIM3);
}

/*----------------------------------------------------------------------------*/

void timer3SetupOutputCompare(uint16_t u16_autoReload, uint16_t u16_prescaler) {
  /* Reset TIM3 peripheral to defaults. */
  rcc_periph_reset_pulse(RST_TIM3);

  gpio_set_mode(GPIO_BANK_TIM3_CH3, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_TIM3_CH3);

  timer_set_prescaler(TIM3, u16_prescaler);
  timer_set_period(TIM3, u16_autoReload);

  /*
   *  This causes the counter to be loaded immediately with a new count value
   *  when the auto-reload register is written, so that the new value becomes
   *  effective for the current count cycle rather than for the cycle following
   *  an update event.Disable preload.
   */
  timer_disable_preload(TIM3);
  // Enable the Timer to run continuously
  timer_continuous_mode(TIM3);

  //   timer_disable_oc_output(TIM3, TIM_OC1 | TIM_OC2 | TIM_OC4);
  timer_clear_flag(TIM3, TIM_SR_CC3IF | TIM_SR_CC3OF);
  timer_enable_oc_output(TIM3, TIM_OC3);

  timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_TOGGLE);

  // Enable Timer 3 Channel 3 compare interrupt
  timer_enable_irq(TIM3, TIM_DIER_CC3IE);

  /* Counter enable. */
  timer_enable_counter(TIM3);
}

/*----------------------------------------------------------------------------*/

extern "C" void tim3_isr(void) {
  static uint16_t u16_prevTimer;
  uint16_t u16_timer;
  uint16_t u16_pulse;
  bool b_startNewFrame;
  bool b_lastPulse;

  if (timer_get_flag(TIM3, TIM_SR_CC3IF) != 0) {
    if (e_state == LISTENING) {
      e_state = RECEIVING;
    }
    if (e_state == RECEIVING) {
      // read the value of the counter on the active transition AND
      // clear the CC3IF flag
      u16_timer = timer_get_counter(TIM3);
      b_startNewFrame = s_rawSignalIn.u32_startTime == 0;

      if (gpio_get(GPIO_BANK_TIM3_CH3, GPIO_TIM3_CH3) != 0) {
        timer_ic_set_polarity(TIM3, TIM_IC3, TIM_IC_FALLING);
        if (b_startNewFrame) {
          s_rawSignalIn.u32_startTime = millis();
        }
      } else {
        timer_ic_set_polarity(TIM3, TIM_IC3, TIM_IC_RISING);
        u32_txTimeout = millis() + SIGNAL_TIMEOUT;
      }

      if (!b_startNewFrame) {
        if (s_rawSignalIn.u16_pulses >= RAW_BUFFER_SIZE) {
          timer_ic_disable(TIM3, TIM_IC3);
          timer_clear_flag(TIM3, TIM_SR_CC3OF);
        } else {
          if (u16_prevTimer > u16_timer) {
            u16_pulse = (0xffff - u16_prevTimer) + u16_timer;
          } else {
            u16_pulse = u16_timer - u16_prevTimer;
          }
          *s_rawSignalIn.pu16_pulse++ = u16_pulse;
          s_rawSignalIn.u16_pulses++;
        }
      }
      u16_prevTimer = u16_timer;
    } else if (e_state == SENDING) {
      if (s_rawSignalOut.u8_repeats == 0) {
        timer_disable_oc_output(TIM3, TIM_OC3);
        timer_clear_flag(TIM3, TIM_SR_CC3IF | TIM_SR_CC3OF);

        o_queue.put(EVENT_RF_STOP_OUT, 0);
        e_state = IDLE;
        return;
      }

      u16_pulse = s_rawSignalOut.pu16_pulse - s_rawSignalOut.pu16_pulses;
      b_lastPulse = u16_pulse == s_rawSignalOut.u16_pulses;
      if (b_lastPulse) {
        u16_timer = 0;
      } else {
        u16_timer = *s_rawSignalOut.pu16_pulse++;
        if ((u16_pulse & 1) && u16_pulse == s_rawSignalOut.u16_pulses - 1) {
          // signal is low -> merge pulse with inter frame gap
          b_lastPulse = true;
        }
      }

      if (b_lastPulse) {
        u16_timer += s_rawSignalOut.u16_interFrameGap;
        s_rawSignalOut.pu16_pulse = s_rawSignalOut.pu16_pulses;

        if (s_rawSignalOut.u8_repeats-- == 1) {
          timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_FORCE_LOW);
          u16_timer += 15000;
        }
      }

      u16_timer += timer_get_counter(TIM3);
      timer_set_oc_value(TIM3, TIM_OC3, u16_timer);
    }
    timer_clear_flag(TIM3, TIM_SR_CC3IF);
  }
}

/*----------------------------------------------------------------------------*/

extern "C" void usb_lp_can_rx0_isr(void) {
  o_usb.poll();
}

/*----------------------------------------------------------------------------*/
// When host computer writes data to usbuart
void usbuart_usb_in_cb(usbd_device *ps_dev, uint8_t u8_ep) {
  // Read usb packet data
  uint8_t pu8_buffer[CDCACM_PACKET_SIZE];
  uint8_t u8_data;
  int i_length;

  i_length = usbd_ep_read_packet(ps_dev, u8_ep, pu8_buffer, CDCACM_PACKET_SIZE);
  for (int i = 0; i < i_length; i++) {
    u8_data = pu8_buffer[i];
    o_cmdIn.writeByte(u8_data);
    if (u8_data == '\n') {
      o_queue.put(EVENT_USART_IN, 0);
    }
  }
}

/*----------------------------------------------------------------------------*/

void recvUsart() {
  char pc_commandLine[COMMAND_LINE_SIZE];
  const Command *ps_command;
  const Plugin *ps_plugin;
  const char *pc_type;
  char *pc_dst;
  char *pc_src;
  uint i;
  int c;

  pc_dst = pc_commandLine;
  while ((c = o_cmdIn.readByte()) != -1) {
    if (c == '\n') {
      break;
    }
    if (c > ' ') {
      *pc_dst++ = c;
    }
  }

  // test if command line is too small
  if (pc_dst - pc_commandLine < 8) {
    return;
  }

  *pc_dst = 0;

  if (memcmp(pc_commandLine, "10;", 3) == 0) {
    pc_src = &pc_commandLine[3];
    if (memcmp(pc_src, "PING;", 5) == 0) {
      o_usb.printf("20;%02X;PONG;\r\n", u8_sequenceNumber++);
    } else if (memcmp(pc_src, "VERSION;", 8) == 0) {
      o_usb.printf("20;%02X;VER=0.1;REV=%02x;BUILD=%02x;\r\n",
                   u8_sequenceNumber++, 0x07, 0x33);
    } else if (memcmp(pc_src, "RFDEBUG=O", 9) == 0) {
      b_rfDebug = pc_src[9] == 'N';
      o_usb.printf("20;%02X;RFDEBUG=%s;\r\n", u8_sequenceNumber++,
                   b_rfDebug ? "ON" : "OFF");
    } else if (memcmp(pc_src, "RFDUMP", 6) == 0) {
      o_rfm69.dumpRegisters(o_usb);
    } else {
      getParamAsString(&pc_src, &pc_type);

      ps_command = ps_commands;
      for (i = 0; i < u8_commands; i++) {
        if (strcasecmp(pc_type, ps_command->pc_name) == 0) {
          ps_command->pf_cmdCore(ps_command, pc_src);
          return;
        }
        ps_command++;
      }

      s_rawSignalOut.u16_pulses = 0;
      s_rawSignalOut.pu16_pulse = s_rawSignalOut.pu16_pulses;
      s_rawSignalOut.u32_startTime = 0;
      s_rawSignalOut.u32_frequency = RFM69_FREQ_TO_REG(433920000); // 433.92MHz

      ps_plugin = ps_plugins;
      for (i = 0; i < u8_plugins; i++) {
        if (strcasecmp(pc_type, ps_plugin->pc_name) == 0) {
          if (ps_plugin->pf_txCore != NULL &&
              ps_plugin->pf_txCore(ps_plugin, &s_rawSignalOut, pc_src)) {
            o_queue.put(EVENT_RF_START_OUT, 0);
          }
          break;
        }
        ps_plugin++;
      }
    }
  }
}

/*----------------------------------------------------------------------------*/

void recvRfStart() {
  s_rawSignalIn.u16_pulses = 0;
  s_rawSignalIn.pu16_pulse = s_rawSignalIn.pu16_pulses;
  s_rawSignalIn.u32_startTime = 0;
  e_state = LISTENING;

  o_rfm69.setFrequency(RFM69_FREQ_TO_REG(433920000)); // 433.92MHz DIO
  o_rfm69.setMode(RFM69_MODE_RX);
  o_rfm69.waitForModeReady();

  timer3SetupInputCapture(0xffff, 720);
}

/*----------------------------------------------------------------------------*/

void recvRfStop() {
  register const Plugin *ps_plugin;
  bool b_identified;
  int i_pulses;
  int i;

  e_state = IDLE;

  i_pulses = s_rawSignalIn.u16_pulses;
  if (i_pulses > MIN_RAW_PULSES) {
    if (b_rfDebug) {
      o_usb.printf("20;%02X;DEBUG;Pulses=%d;Pulses(uSec)=", u8_sequenceNumber++,
                   i_pulses);

      for (i = 0; i < i_pulses; i++) {
        o_usb.printf(i == i_pulses - 1 ? "%d;\n" : "%d,",
                     s_rawSignalIn.pu16_pulses[i] * 10);
      }
    }

    b_identified = false;
    ps_plugin = ps_plugins;
    do {
      if (ps_plugin->pf_rxCore != NULL &&
          ps_plugin->pf_rxCore(ps_plugin, &s_rawSignalIn, NULL)) {
        b_identified = true;
        break;
      }
    } while ((uint)(++ps_plugin - ps_plugins) < u8_plugins);

    if (b_identified) {
      u8_ledState = LED_START_BLINK;
    }
  }

  o_queue.put(EVENT_RF_START_IN, 0);
}

/*----------------------------------------------------------------------------*/

void sendRfStart() {
  // If reception is ongoing, wait until state is IDLE or LISTENING
  while (e_state != IDLE && e_state != LISTENING && e_state != PROCESSING) {
    asm("wfi");
  }

  s_rawSignalOut.pu16_pulse = s_rawSignalOut.pu16_pulses;
  e_state = SENDING;

  o_rfm69.setFrequency(s_rawSignalOut.u32_frequency);
  o_rfm69.setPowerDBm(13);
  o_rfm69.setMode(RFM69_MODE_TX);
  o_rfm69.waitForModeReady();

  timer3SetupOutputCompare(0xffff, 720);
}

/*----------------------------------------------------------------------------*/

void sendRfStop() {
  o_queue.put(EVENT_RF_START_IN, 0);
}

/*----------------------------------------------------------------------------*/

void fatalError(uint8_t u8_code) {
  uint8_t u8_startMask;
  uint8_t u8_mask;

  u8_startMask = 0x80;
  while ((u8_code & u8_startMask) == 0 && u8_startMask > 0x04) {
    u8_startMask >>= 1;
  }

  for (;;) {
    for (u8_mask = u8_startMask; u8_mask != 0; u8_mask >>= 1) {
      gpio_clear(GPIOC, GPIO13);
      msleep((u8_code & u8_mask) ? 1500 : 500);
      gpio_set(GPIOC, GPIO13);
      msleep(500);
    }
    msleep(3000);
  }
}

/*----------------------------------------------------------------------------*/
/*
 * Setup the RFM69 based on http://members.home.nl/hilcoklaassen/
 * Also see:
 * https://acassis.wordpress.com/2016/04/19/getting-started-with-rfm69/
 */
void setup() {
  uint8_t u8_version;

  // LED on BluePill F103 is PC13
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO13);

  // Setup GPIO pin A0 to reset the RFM69
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);
  gpio_clear(GPIOA, GPIO0);

  timer3Init();

#ifdef USART_DEBUG
  // Init USART device on A9-TX and A10-RX
  nvic_enable_irq(NVIC_USART1_IRQ);
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_USART1_TX);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
#endif

  o_queue.init();
  o_usb.init();

#ifdef USART_DEBUG
  o_usart.init(USART1_BASE);
  o_usart.enableRxInterrupt();
  o_usart.enable();
#endif

  //   o_usart.puts("USART initialized\n");
  o_usb.printf("20;%02X;Nodo RadioFrequencyLink - RFSkipper V0.1 - R%02d;\r\n",
               u8_sequenceNumber++, 13);

  pluginsInitialization();

  // Init RFM69 device
  o_rfm69.init(SPI1, NULL, rfm69Reset, true);
  o_rfm69.setFrequency(RFM69_FREQ_TO_REG(433920000)); // 433.92MHz DIO
  o_rfm69.writeRegister(REG_TESTLNA, 0x2D);           // High sensitivity mode
  o_rfm69.setBitRate(RFM69_BITRATE_TO_REG(32000));
  o_rfm69.setOOKPeak(RF_OOKPEAK_THRESHTYPE_FIXED);
  o_rfm69.setMode(RFM69_MODE_RX);
  o_rfm69.setRssiThreshold(-127);

  u8_version = o_rfm69.getVersion();
  if (u8_version != 0x24) {
    o_usb.puts("Couldn't detect RFM69 device\r\n");
    fatalError(1);
  }

  int i_rssiAverage = 0;
  for (int i_loop = 0; i_loop < 1024; i_loop++) {
    if ((i_loop & 0x3F) == 0) {
      gpio_toggle(GPIOC, GPIO13);
    }
    i_rssiAverage += o_rfm69.readRSSI(false);
    msleep(10);
  }
  gpio_set(GPIOC, GPIO13);

  i_rssiAverage /= 1024;
  o_usb.printf("20;%02X;DEBUG;RFM69 calibrated RSSI=%d\r\n",
               u8_sequenceNumber++, i_rssiAverage);

  // should be average RSSI when no signal + 20dB (~ -80dB)
  o_rfm69.setFixedThreshold(i_rssiAverage + 20);

  o_queue.put(EVENT_RF_START_IN, 0);
}

/*----------------------------------------------------------------------------*/

void loop() {
  uint8_t u8_event;

  if (o_queue.isEmpty()) {
    __asm("wfi");
  }

  if (!o_queue.isEmpty()) {
    u8_event = o_queue.get();
    switch (u8_event) {
    case EVENT_USART_IN:
      recvUsart();
      break;
    case EVENT_RF_START_IN:
      recvRfStart();
      break;
    case EVENT_RF_STOP_IN:
      recvRfStop();
      break;
    case EVENT_USART_OUT:
      break;
    case EVENT_RF_START_OUT:
      sendRfStart();
      // TODO: search for a better synchronization mechanism
      while (e_state == SENDING) {
        __asm volatile("wfi");
      }
      break;
    case EVENT_RF_STOP_OUT:
      sendRfStop();
      break;
    }
  }

  if ((millis() & 511) == 0) {
    //      int i_rssi = o_rfm69.readRSSI(false);
    //      o_usart.printf("RSSI=%d\n", i_rssi);
    switch (u8_ledState) {
    case LED_IDLE:
      break;
    case LED_START_BLINK:
      gpio_clear(GPIOC, GPIO13);
      u8_ledState = LED_ON;
      break;
    case LED_ON:
      gpio_set(GPIOC, GPIO13);
      u8_ledState = LED_OFF;
      break;
    case LED_OFF:
      u8_ledState = LED_IDLE;
      break;
    }
  }
}

/*----------------------------------------------------------------------------*/
