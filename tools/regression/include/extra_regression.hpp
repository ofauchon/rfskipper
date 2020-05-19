#ifndef EXTRA_REGRESSION_HPP
#define EXTRA_REGRESSION_HPP

typedef struct _Command Command;
typedef void(CommandFunction)(const Command *ps_cmd, const char *pc_option);
typedef CommandFunction *CommandFunctionPtr;

struct _Command {
  const char *pc_name;
  CommandFunctionPtr pf_cmdCore;
};


#endif
