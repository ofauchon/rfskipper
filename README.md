# rfskipper
Radio protocol decoder for sensor devices

#Flashing: 

```openocd -f interface/stlink-v2.cfg  -f target/stm32f1x_clone.cfg -c "init" -c "reset init" -c "halt" -c "flash write_image erase rfskipper.bin 0x08000000" -c "shutdown"
```
