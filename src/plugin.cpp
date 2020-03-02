/*
 * plugin.c
 *
 *  Created on: 31 Jan 2020
 *      Author: Pierre
 */

#include <stdlib.h>
#include <string.h>

#include "plugin.hpp"


/*----------------------------------------------------------------------------*/

// @formatter:off
const Plugin ps_plugins[] = {
   { 4, "NewKaku", plugin004Rx, plugin004Tx },
   { 13, "Powerfix", plugin013Rx, NULL },
   { 48, "Oregon", plugin048Rx, NULL },
   { 99, "RTS", plugin099Rx, plugin099Tx }
};
// @formatter:on

const uint8_t u8_plugins = sizeof(ps_plugins) / sizeof(Plugin);

/*----------------------------------------------------------------------------*/

bool getParamAsString(char **ppc_start, const char **ppc_value) {
   char *pc_src;

   pc_src = *ppc_start;
   if (*pc_src == 0) {
      return false;
   }

   *ppc_value = pc_src;
   while (*pc_src != ';') {
      pc_src++;
   }
   *pc_src++ = 0;

   *ppc_start = pc_src;

   return true;
}

/*----------------------------------------------------------------------------*/

bool getParamAsHex(char **ppc_start, int *pi_value) {
   char *pc_src;
   int i_value;
   char c;

   pc_src = *ppc_start;
   if (*pc_src == 0) {
      return false;
   }

   i_value = 0;
   while ((c = *pc_src++) != ';') {
      i_value <<= 4;
      if (c >= '0' && c <= '9') {
         i_value += c - '0';
      } else if (c >= 'A' && c <= 'F') {
         i_value += 10 + c - 'A';
      } else if (c >= 'a' && c <= 'f') {
         i_value += 10 + c - 'a';
      } else {
         return false;
      }
   }

   *pi_value = i_value;
   *ppc_start = pc_src;

   return true;
}

/*----------------------------------------------------------------------------*/

bool getParamAsEnum(char **ppc_start, int *pi_value,
                    const char * const *ppc_values, int i_count) {
   const char *pc_value;

   if (getParamAsString(ppc_start, &pc_value)) {
      for (int i = 0; i < i_count; i++) {
         if (strcasecmp(pc_value, ppc_values[i]) == 0) {
            *pi_value = i;
            return true;
         }
      }
   }

   return false;
}

/*----------------------------------------------------------------------------*/

