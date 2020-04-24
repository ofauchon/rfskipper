#ifndef PLUGIN_H_
#define PLUGIN_H_

/*----------------------------------------------------------------------------*/

#include <stdint.h>

#ifndef _WIN32
#include "radio.hpp"
#else
#include "regression.H"
#endif

/*----------------------------------------------------------------------------*/

// unit sample is 10µs
#define RAWSIGNAL_SAMPLE_RATE 10
// Maximum number of pulses that is received in one go.
#define RAW_BUFFER_SIZE 512

/*----------------------------------------------------------------------------*/

typedef struct _RawSignal {
  uint8_t u8_repeats;  // Number of re-transmits on transmit actions.
  uint16_t u16_pulses; // Number of pulses, times two as every pulse has a mark
                       // and a space.
  uint16_t u16_interFrameGap;
  uint16_t pu16_pulses[RAW_BUFFER_SIZE];
  // Timestamp indicating when the signal was received (millis())
  uint32_t u32_startTime;
  // Timestamp indicating when the previous signal was received (millis())
  uint32_t u32_prevTime;
  uint32_t u32_prevCRC;
  uint32_t u32_frequency;
  uint16_t *pu16_pulse;
} RawSignal;

typedef struct _Plugin Plugin;
typedef bool(PluginFunction)(const Plugin *ps_plugin, RawSignal *ps_rawSignal,
                             char *pc_string);
typedef PluginFunction *PluginFunctionPtr;

struct _Plugin {
  uint8_t u8_pluginIndex;
  const char *pc_name;
  PluginFunctionPtr pf_rxCore;
  PluginFunctionPtr pf_txCore;
};

/*----------------------------------------------------------------------------*/

extern const Plugin ps_plugins[];
extern const uint8_t u8_plugins;

/*----------------------------------------------------------------------------*/

extern void pluginsInitialization();

extern bool getParamAsString(char **ppc_start, const char **ppc_value);
extern bool getParamAsDec(char **ppc_start, int *pi_value);
extern bool getParamAsHex(char **ppc_start, int *pi_value);
extern bool getParamAsEnum(char **ppc_start, int *pi_value,
                           const char *const *ppc_values, int i_count);

extern PluginFunction plugin004Rx;
extern PluginFunction plugin004Tx;
extern PluginFunction plugin013Rx;
extern PluginFunction plugin048Rx;
extern PluginFunction plugin099Rx;
extern PluginFunction plugin099Tx;
extern CommandFunction plugin099Show;
extern CommandFunction plugin099Clean;
extern void plugin099Init();

/*----------------------------------------------------------------------------*/

#endif /* PLUGIN_H_ */
