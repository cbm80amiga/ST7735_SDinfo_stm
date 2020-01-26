# ST7735_SDinfo_stm
Portable  SD card tester and CID analyzer with STM32 and ST7735 LCD

YouTube videos:

https://youtu.be/s2bYx58kJ_U

https://youtu.be/ZAe61GZ-52Y

More ST7735 and STM32 projects:

https://www.youtube.com/watch?v=o3AqITHf0mo&list=PLxb1losWErZ6y6GombzvtwRZ2l7brPv1s


## Connections (header at the top):

|LCD pin|LCD pin name|STM32|
|--|--|--|
 |#01| LED| 3.3V|
 |#02| SCK |PA5/SCK|
 |#03| SCA |PA7/MOSI|
 |#04| A0/DC|PA1 or any digital
 |#05| RESET|PA0 or any digital|
 |#06| CS|PA2 or any digital|
 |#07| GND | GND|
 |#08| VCC | 3.3V|

|SD pin|SD pin name|STM32|
|--|--|--|
|#01| SD_SCK| PA5|
|#02| SD_MISO |PA6|
|#03| SD_MOSI |PA7|
|#04| SD_CS |PA4|

## Features:
- uses cheap STM32 board and ST7735 LCD with built-in SD socket
- CID, OCR, CSD and status registers decoding
- partition table and optional FAT filesystem parameters
- CID unlock check based on Richard Burton's code
- requires STM32 port of SdFat library
- fast ST7735 library and RREFont
- provided compiled binary ready to flash
- SD cards specs: https://www.sdcard.org/downloads/pls/

If you find it useful and you want to buy me a coffee or a beer:

https://www.paypal.me/cbm80amiga
