#!/bin/bash

mbed compile -t gcc_arm -m LPC11U35_401
lpc_checksum ./BUILD/LPC11U35_401/gcc_arm/blink-test.bin
dd if=./BUILD/LPC11U35_401/gcc_arm/spc2019-parachute-mbed.bin of=/media/iwait/CRP\ DISABLD/firmware.bin conv=notrunc
