# Tag v0.1
Basic version that supports:<br />
1. Command line interface available via USB connection. Based on [microcrl](https://github.com/Helius/microrl) library.<br />
After connecting to USB Host (PC) the STM32 board is recognized as a USB-to-serial "tty" interface.<br />
Simple Linux-style terminal is available on this interface. Several example commands implemented<br />
to switch the LED on/off or make it blink.<br />
2. Event queue.<br />
3. Timers.<br />
## Hardware
1. This code is configured to be running on [STM32 Smart V2.0](https://wiki.stm32duino.com/index.php?title=STM32_Smart_V2.0) board built around STM32F103C8T6 chip.<br />But it can be ported easily to any other STM32 device with USB support like Discovery board, etc.<br />
## Install and compile
1. Install [SW4](https://www.st.com/en/development-tools/sw4stm32.html) Eclipse based IDE and build the project.<br />
2. Build and install [stlink](https://github.com/texane/stlink) tools to flash the board.<br />
3. Install **minicom** serial communication program.

## Flash the hex
1. Run the command while holding the S1 "Reset" button:<br />
```sh
$ st-info --probe
```
2. Release S1 Reset button.<br />
3. Issue the command:<br />
```sh
$ st-flash --format ihex write emat_stm32_smart.hex
```
## Send console commands
1. Connect the board to PC using mini-USB cable.<br />
2. Connect PA12 pin to the GND and release immediately.<br />
3. Check the output of "dmesg" command, find the message from the "USB-Serial" driver,<br />
it should be something like this:<br />
```sh
$ dmesg
...
[11925.184116] cdc_acm 1-1.4:1.0: ttyACM0: USB ACM device
```

4. Start **minicom**:<br />
```sh
$ minicom -D /dev/ttyACM0
```
5. Send the command **hv \<on \| off\>** to switch the green LED on/off:<br />
```sh
EMAT > hv on
EMAT > hv off
```
6. Send the command **width \<time_ms\>** to make the green LED blink .<br />
```sh
EMAT > width 1000
```
