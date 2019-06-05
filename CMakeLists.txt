cmake_minimum_required(VERSION 3.2)
project(spc2019-parachute-mbed)

SET(CMAKE_ASM_COMPILER    "/home/iwait/opt/gcc-arm-none-eabi/bin/arm-none-eabi-gcc")
SET(CMAKE_C_COMPILER      "/home/iwait/opt/gcc-arm-none-eabi/bin/arm-none-eabi-gcc")
SET(CMAKE_CXX_COMPILER    "/home/iwait/opt/gcc-arm-none-eabi/bin/arm-none-eabi-g++")
SET(ELF2BIN               "/home/iwait/opt/gcc-arm-none-eabi/bin/arm-none-eabi-objcopy")

#SET(CMAKE_C_COMPILER "$ENV{HOME}/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-gcc")
#SET(CMAKE_CXX_COMPILER "$ENV{HOME}/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-g++")
#SET(CMAKE_CXX_FLAGS_DISTRIBUTION "-std=gnu++98 -fno-rtti -Wvla -c -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fmessage-length=0 -fno-exceptions -fno-builtin -ffunction-sections -fdata-sections -funsigned-char -MMD -fno-delete-null-pointer-checks -fomit-frame-pointer -Os -g1 -DMBED_RTOS_SINGLE_THREAD -mcpu=cortex-m0 -mthumb -DMBED_ROM_START=0x0 -DMBED_ROM_SIZE=0x8000 -DMBED_RAM_START=0x10000000 -DMBED_RAM_SIZE=0x1000 -include mbed_config.h")
#SET(CMAKE_C_FLAGS_DISTRIBUTION "-c -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fmessage-length=0 -fno-exceptions -fno-builtin -ffunction-sections -fdata-sections -funsigned-char -MMD -fno-delete-null-pointer-checks -fomit-frame-pointer -Os -g1 -DMBED_RTOS_SINGLE_THREAD -mcpu=cortex-m0 -mthumb -DMBED_ROM_START=0x0 -DMBED_ROM_SIZE=0x8000 -DMBED_RAM_START=0x10000000 -DMBED_RAM_SIZE=0x1000 -include mbed_config.h")
set(CMAKE_CXX_STANDARD 11)

include_directories("${CMAKE_CURRENT_LIST_DIR}/src")
include_directories("${CMAKE_CURRENT_LIST_DIR}/lib/SerialSRAM")
include_directories("${CMAKE_CURRENT_LIST_DIR}/lib/MS5607")
include_directories("${CMAKE_CURRENT_LIST_DIR}/include")

FILE(GLOB_RECURSE SRC_LIST "${CMAKE_CURRENT_LIST_DIR}/src/*.*" "${CMAKE_CURRENT_LIST_DIR}/lib/*.*")
