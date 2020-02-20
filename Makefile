ifneq ($(V), 1)
MFLAGS += --no-print-dir
Q := @
endif

MCU := stm32/f1

all:
	$(Q)if [ ! -f libopencm3/Makefile ]; then \
		echo "Initialising git submodules..." ;\
		git submodule init ;\
		git submodule update ;\
	fi
	$(Q)$(MAKE) $(MFLAGS) TARGETS=$(MCU) -C libopencm3
	$(Q)$(MAKE) $(MFLAGS) -C src

clean:
	$(Q)$(MAKE) $(MFLAGS) -C src $@

realclean:
	$(Q)$(MAKE) $(MFLAGS) TARGETS=$(MCU) -C libopencm3 $@
	$(Q)$(MAKE) $(MFLAGS) -C src $@

flash:
	 openocd -f interface/stlink-v2.cfg  -f target/stm32f1x.cfg -c "init" -c "reset init" -c "halt" -c "flash write_image erase src/rfskipper.bin 0x08000000" -c "reset" -c "shutdown"

