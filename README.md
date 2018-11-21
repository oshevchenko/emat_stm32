#Tag v0.1
Basic version that supports <br />
1.USB console<br />
2.Timers<br />

To flash the hex:<br />
1. Run the command while holding the S1 "Reset" button:<br />
```sh
$ st-info --probe
```
2. Release S1 Reset button.<br />
3. Issue the command:<br />
```sh
$ st-flash --format ihex write emat_stm32_smart.hex
```

1. Connect the board to PC using mini-USB cable.<br />
2. Connect PA12 pin to the GND and release immediately.<br />
3. Chech the output of:<br />
```sh
$ dmesg
```
there should be something like:<br />
```sh
...
[11925.184116] cdc_acm 1-1.4:1.0: ttyACM0: USB ACM device
```

4. Start minicom:<br />
```sh
minicom -D /dev/ttyACM0
```
5. Send commands to switch the green LED on/off:<br />
```sh
EMAT > hv on
EMAT > hv off
```
6. Send commands to make the green LED blink:<br />
```sh
EMAT > width 1000
```