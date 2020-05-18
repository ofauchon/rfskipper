#ifndef RADIO_H_
#define RADIO_H_

/*----------------------------------------------------------------------------*/

#include "usb.h"
#ifdef USART_ENABLE
#include "usart.hpp"
#endif

/*----------------------------------------------------------------------------*/

#ifdef USART_ENABLE
extern USART o_usart;
#endif

/*----------------------------------------------------------------------------*/

typedef struct _Command Command;
typedef void(CommandFunction)(const Command *ps_cmd, const char *pc_option);
typedef CommandFunction *CommandFunctionPtr;

struct _Command {
  const char *pc_name;
  CommandFunctionPtr pf_cmdCore;
};

/*----------------------------------------------------------------------------*/

extern CommandFunction setParameter;

/*----------------------------------------------------------------------------*/

#endif /* RADIO_H_ */
