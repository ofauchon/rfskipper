# Requirements:
 
  - Bluepill board
  - Stlink v2 flasher
  - make
  - arm toolchain
  - openocd
  
# Build and flash

```
git clone git@github.com:ofauchon/rfskipper.git
make 
make flash
```

# Connect to serial

```
$ picocom --imap lfcrlf --omap crcrlf -b 57600 -c /dev/ttyUSB0

USART initialized
20;00;Nodo RadioFrequencyLink - RFLink Gateway V1.1 - R45;
RADIO::timer3Init
RFM69::init
SPI::init
RFM69::reset with GPIO 
RFM69::init End
RFM69 version 2-4
Trying to calibrate RFM69 RSSI
Calibration result: Average RSSI=-101

20;00;NewKaku;ID=01069802;SWITCH=b;CMD=ALLOFF;
20;01;NewKaku;ID=03367cb1;SWITCH=9;CMD=OFF;
20;02;NewKaku;ID=0227305a;SWITCH=1;CMD=ON;
20;03;NewKaku;ID=02364b5a;SWITCH=7;CMD=OFF;
20;04;NewKaku;ID=0034a05a;SWITCH=8;CMD=ON;


```

# References 

https://jeelabs.org/article/1649e/

