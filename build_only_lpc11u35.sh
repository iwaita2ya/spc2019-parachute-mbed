#!/bin/bash

TARGET_BOARD=LPC11U35_401
BIN_FILE=spc2019-parachute-mbed.bin

BIN_PATH=./BUILD/${TARGET_BOARD}/gcc_arm/${BIN_FILE}

# add compile
echo "step 1: compile"
mbed compile -t gcc_arm -m ${TARGET_BOARD}

# add checksum
echo "step 2: add checksum"
lpc_checksum ${BIN_PATH}
