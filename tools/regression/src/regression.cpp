//============================================================================
// Name        : rf_reg.cpp
// Author      : TSD
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fstream>
#include <iostream>

#include "getopt.h"

#include "plugin.hpp"
//#include "eeprom_regression.hpp"

/*----------------------------------------------------------------------------*/

#define BUFFER_SIZE 8192

#define ERROR_OPEN_TRACE_FILE 0x10001
#define ERROR_ILLEGAL_TRACE_FILE 0x10002

/*----------------------------------------------------------------------------*/

// global variables accessed from the plugins
USART o_usb;
//uint8_t u8_sequenceNumber = 0;

RawSignal s_rawSignal;

/*----------------------------------------------------------------------------*/

char *getNextTraceLine(FILE *ps_file, int &i_line) {
  static char pc_line[BUFFER_SIZE];
  char *pc;
  int c;

  while ((pc = fgets(pc_line, BUFFER_SIZE, ps_file)) != NULL) {
    i_line++;

    while ((c = *pc) == ' ' || c == '\t') {
      pc++;
    }

    if (c == 0 || c == '\n' || c == '#') {
      continue;
    }

    if (strncmp("20;", pc, 3) == 0) {
      break;
    }

    printf("Trace should start with \"20;\" at line %d\n", i_line);
  }

  return pc;
}

/*----------------------------------------------------------------------------*/

int parseTraceFile(const char *pc_filename) {
  const Plugin *ps_plugin;
  uint16_t *pu16_pulses;
  uint16_t u16_pulses;
  bool b_identified;
  FILE *ps_file;
  char *pc_expected;
  char *pc_line;
  char *pc;
  int i_fatalError;
  int i_regression;
  int i_tests;
  int i_value;
  int i_line;
  int c;

  ps_file = fopen(pc_filename, "rt");
  if (ps_file == NULL) {
    printf("Couldn't open input file %s, errno=%d\n", pc_filename, errno);
    return ERROR_OPEN_TRACE_FILE;
  }

  i_fatalError = 0;
  s_rawSignal.u32_prevCRC = 0;
  s_rawSignal.u32_startTime = 0;

  i_line = i_regression = i_tests = 0;
  for (;;) {
    pc_line = getNextTraceLine(ps_file, i_line);
    if (pc_line == NULL) {
      break;
    }

    pc = pc_line + 3;
    while ((c = *pc++) != ';') {
      if (c == 0) {
        printf("Unexpected end of line at line %d, column %d\n", i_line,
               (int) (pc - pc_line));
        return ERROR_ILLEGAL_TRACE_FILE;
      }
      pc++;
    }

    if (strncmp("DEBUG;", pc, 6) != 0) {
      printf("Trace should be of type 'DEBUG' at line %d\n", i_line);
      return ERROR_ILLEGAL_TRACE_FILE;
    }
    pc += 6;

    if (strncmp("Pulses=", pc, 7) != 0) {
      printf("Trace should contain the 'Pulses' parameter at line %d\n",
             i_line);
      return ERROR_ILLEGAL_TRACE_FILE;
    }
    pc += 7;

    i_value = 0;
    while ((c = *pc++) != ';' && c != 0) {
      if (!isdigit(c)) {
        printf("'Pulses' should be numeric at line %d\n", i_line);
        return ERROR_ILLEGAL_TRACE_FILE;
      }
      i_value = i_value * 10 + c - '0';
    }
    s_rawSignal.u16_pulses = i_value;

    if (c != ';') {
      printf("Unexpected end of line at line %d, column %d\n", i_line,
             (int) (pc - pc_line));
      return ERROR_ILLEGAL_TRACE_FILE;
    }

    if (strncmp("Pulses(uSec)=", pc, 13) != 0) {
      printf("Trace should contain the pulses values at line %d\n", i_line);
      return ERROR_ILLEGAL_TRACE_FILE;
    }
    pc += 13;

    u16_pulses = 0;
    pu16_pulses = s_rawSignal.pu16_pulses;

    while (u16_pulses < RAW_BUFFER_SIZE) {
      i_value = 0;
      while ((c = *pc++) != ',' && c != ';' && c != 0) {
        if (isdigit(c)) {
          i_value = i_value * 10 + c - '0';
        }
      }
      *pu16_pulses++ = i_value / RAWSIGNAL_SAMPLE_RATE;
      u16_pulses++;

      if (c == ';') {
        if (u16_pulses != s_rawSignal.u16_pulses) {
          printf("The trace size %d doesn't match the number of pulses %d at "
                 "line %d\n",
                 s_rawSignal.u16_pulses, u16_pulses, i_line);
          return ERROR_ILLEGAL_TRACE_FILE;
        }
        break;
      }

      if (c == 0) {
        printf("Unexpected end of line at line %d, column %d\n", i_line,
               (int) (pc - pc_line));
        return ERROR_ILLEGAL_TRACE_FILE;
      }
    }
    i_tests++;

    s_rawSignal.pu16_pulse = NULL;
    s_rawSignal.u8_repeats = 0;

    printf("Trace at line %d has %d pulses\n", i_line, u16_pulses);

    b_identified = false;
    ps_plugin = ps_plugins;
    do {
      if (ps_plugin->pf_rxCore(ps_plugin, &s_rawSignal, NULL)) {
        b_identified = true;
        break;
      }
    } while ((unsigned int) (++ps_plugin - ps_plugins) < u8_plugins);

    if (!b_identified) {
      printf("Trace at line %d wasn't identified\n", i_line);
    }

    // get the expected value string

    pc_line = getNextTraceLine(ps_file, i_line);
    if (pc_line == NULL) {
      printf("Couldn't find expected value for current trace\n");
      i_fatalError = ERROR_ILLEGAL_TRACE_FILE;
      goto exitMainLoop;
    }

    pc = pc_line + 3;
    while ((c = *pc++) != ';') {
      if (c == 0) {
        printf("Unexpected end of line at line %d, column %d\n", i_line,
               (int) (pc - pc_line));
        i_fatalError = ERROR_ILLEGAL_TRACE_FILE;
        goto exitMainLoop;
      }
    }

    pc_expected = o_usb.getLastLine();
    if (pc_expected == NULL) {
      printf("No expected result at line %d\n", i_line);
      i_fatalError = ERROR_ILLEGAL_TRACE_FILE;
      goto exitMainLoop;
    }

    if (strncmp("20;", pc_expected, 3) != 0) {
      printf("Expected result should start with '20;' at line %d\n", i_line);
      i_fatalError = ERROR_ILLEGAL_TRACE_FILE;
      goto exitMainLoop;
    }

    pc_expected += 3;
    while ((c = *pc_expected++) != ';') {
      if (c == 0) {
        printf("Unexpected end of line at line %d, column %d\n", i_line,
               (int) (pc - pc_line));
        i_fatalError = ERROR_ILLEGAL_TRACE_FILE;
        goto exitMainLoop;
      }
    }

    pc_line = pc_expected;
    while ((c = *pc_line) != 0 && c != '\r' && c != '\n') {
      pc_line++;
    }
    *pc_line = 0;

    pc_line = pc;
    while ((c = *pc_line) != 0 && c != '\r' && c != '\n') {
      pc_line++;
    }
    *pc_line = 0;

    if (strcasecmp(pc_expected, pc) != 0) {
      printf("Regression detected at line %d\n", i_line);
      printf("  Received -> %s", pc);
      printf("  Expected -> %s", pc_expected);
      i_regression++;
    }
  }

exitMainLoop:

  fclose(ps_file);

  if (i_fatalError == 0) {
    if (i_regression > 0) {
      printf("%d/%d regressions detected\n", i_regression, i_tests);
    } else {
      printf("All %d tests passed successfully\n", i_tests);
    }
    return EXIT_SUCCESS;
  } else {
    printf("A fatal error occurred while processing line %d\n", i_line);
    printf("Execution stopped!\n");
    return i_fatalError;
  }
}

/*----------------------------------------------------------------------------*/

int main(int i_argc, char *ppc_argv[]) {
  const char *pc_inputFile;
  int i_code;
  int c;

  while ((c = getopt(i_argc, (const char **) ppc_argv, "i:?")) != EOF) {
    switch (c) {
    case 'i':
      pc_inputFile = optarg;
      break;
    }
  }

  //  i_code = eepromTest();

  printf("Using input file: %s\n", pc_inputFile);

  i_code = parseTraceFile(pc_inputFile);
  if (i_code != 0) {
    printf("Couldn't read trace file %s, error=0x%06x\n", pc_inputFile, i_code);
    return EXIT_FAILURE;
  }

  return 0;
}
