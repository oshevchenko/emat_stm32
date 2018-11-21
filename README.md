To flash the hex:<br />
Press and hold S1 reset button.<br />
Run the command:<br />
```sh
$ st-info --probe
```
Release S1 Reset button.<br />
Issue the command:<br />
```sh
$ st-flash --format ihex write /home/oleksandr/workspace/stm32/emat_stm32/emat_stm32_smart/Debug/emat_stm32_smart.hex
```