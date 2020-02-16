ifneq ($(V), 1)
MAKEFLAGS += --no-print-dir
Q := @
endif

SRC =			\
	spi.C		\
	radio.C		\
	rfm69.C		\
	usart.C		\
	ring.C		\
	cdcacm.C	\
	plugin.C	\
	eventqueue.C	\
	plugin/newKaku.C \
	plugin/powerFix.C \
	plugin/somfy.C \
	main.C		\


PREFIX           = arm-none-eabi
CC		:= $(PREFIX)-gcc
CXX		:= $(PREFIX)-g++
LD		:= $(PREFIX)-gcc
AR		:= $(PREFIX)-ar
AS		:= $(PREFIX)-as
SZ		:= $(PREFIX)-size
OBJCOPY		:= $(PREFIX)-objcopy
OBJDUMP		:= $(PREFIX)-objdump
GDB		:= $(PREFIX)-gdb


CFLAGS		+= -O0 -g
CFLAGS		+= -Wextra -Wshadow -Wimplicit-function-declaration
CFLAGS		+= -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes
CFLAGS		+= -fno-common -ffunction-sections -fdata-sections

CXXFLAGS	+= -Os -g
CXXFLAGS	+= -Wextra -Wshadow -Wredundant-decls  -Weffc++
CXXFLAGS	+= -fno-common -ffunction-sections -fdata-sections

CPPFLAGS	+= -MD
CPPFLAGS	+= -Wall -Wundef
CPPFLAGS	+= -I . -I ../inc -I../libopencm3/include
CPPFLAGS    += -mthumb -mcpu=cortex-m3 -msoft-float -mfix-cortex-m3-ldrd
CPPFLAGS    += -DSTM32F1 -DDISCOVERY_STLINK

LDFLAGS      = -specs=nano.specs -specs=nosys.specs
LDFLAGS     += -L../libopencm3/lib -lopencm3_stm32f1 -lnosys
LDFLAGS     += -nostartfiles -lc  
LDFLAGS     += -Wl,-Map=mapfile -Wl,-gc-sections -Wl,-T,rfskipper.ld
LDFLAGS     += -Wl,--defsym,_stack=0x20005000 
LDFLAGS     += -mthumb -mcpu=cortex-m3  
	

all:	rfskipper.bin

host_clean:
	-$(Q)$(RM) rfskipper.bin

OBJ = $(SRC:.C=.o)

rfskipper.elf: version.h $(OBJ)
	@echo "  LD      $@"
	$(Q)$(CC) -o $@ $(OBJ) $(LDFLAGS)

%.o:	%.c
	@echo "  CC      $<"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

%.bin:	%.elf
	@echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O binary $^ $@

%.hex:	%
	@echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O ihex $^ $@

.PHONY:	clean host_clean FORCE

clean:	host_clean
	$(Q)echo "  CLEAN"
	-$(Q)$(RM) -f plugin/*.o plugin/*.d 
	-$(Q)$(RM) -f *.o *.d *~ *.elf *.bin $(HOSTFILES)
	-$(Q)$(RM) -f mapfile version.h

version.h: FORCE
	$(Q)echo "  GIT     version.h"
	$(Q)echo "#define FIRMWARE_VERSION \"`git describe --always --dirty`\"" > $@

-include *.d
