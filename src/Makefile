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

CROSS_COMPILE = arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

OPT_FLAGS ?= -O2

CXXFLAGS += -Wall -Wextra -Werror -Wno-char-subscripts\
	$(OPT_FLAGS) -g3 -MD \
	-I . -I ../inc -I../libopencm3/include \
	-DSTM32F1 -DDISCOVERY_STLINK  \
	-mthumb  -march=armv7-m
LDFLAGS += $(OPT_FLAGS)


OPT_FLAGS = -Os
CFLAGS += -mcpu=cortex-m3 -mthumb \
	-DSTM32F1 -DDISCOVERY_STLINK -I../libopencm3/include \
	-I .

LDFLAGS := $(LDFLAGS) --specs=nano.specs \
	-lopencm3_stm32f1 -Wl,--defsym,_stack=0x20005000 \
	-Wl,-T,rfskipper.ld -nostartfiles -lc \
	-Wl,-Map=mapfile -mthumb -mcpu=cortex-m3 -Wl,-gc-sections \
	-L../libopencm3/lib

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
