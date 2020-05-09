#ifndef RADIO_H_
#define RADIO_H_

/*----------------------------------------------------------------------------*/

#include "usb.hpp"
#include "usart.hpp"

/*----------------------------------------------------------------------------*/

extern USB o_usb;
extern USART o_usart;
extern uint8_t u8_sequenceNumber;

/*----------------------------------------------------------------------------*/

typedef struct _Command Command;
typedef void(CommandFunction)(const Command *ps_cmd, const char *pc_option);
typedef CommandFunction *CommandFunctionPtr;

struct _Command {
  const char *pc_name;
  CommandFunctionPtr pf_cmdCore;
};

/*----------------------------------------------------------------------------*/

#endif /* RADIO_H_ */
