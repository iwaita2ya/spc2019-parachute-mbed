cmake_minimum_required(VERSION 3.9)
SET(CMAKE_SYSTEM_NAME Generic)
#SET(CMAKE_SYSTEM_PROCESSOR arm)
SET(CMAKE_CROSSCOMPILING TRUE)

# force compiler settings
SET(CMAKE_C_COMPILER_WORKS TRUE)
SET(CMAKE_CXX_COMPILER_WORKS TRUE)

SET(CMAKE_ASM_COMPILER    "/home/iwait/opt/gcc-arm-none-eabi/bin/arm-none-eabi-gcc")
SET(CMAKE_C_COMPILER      "/home/iwait/opt/gcc-arm-none-eabi/bin/arm-none-eabi-gcc")
SET(CMAKE_CXX_COMPILER    "/home/iwait/opt/gcc-arm-none-eabi/bin/arm-none-eabi-g++")
SET(ELF2BIN               "/home/iwait/opt/gcc-arm-none-eabi/bin/arm-none-eabi-objcopy")

# if the environment does not specify build type, set to Debug
IF(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug"
            CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
            FORCE)
ENDIF()

# here starts the project
PROJECT(spc2019-parachute-mbed C CXX ASM)

# uncomment below to have a verbose build process
#SET(CMAKE_VERBOSE_MAKEFILE ON)

SET(LD_SYS_LIBS "-Wl,--start-group -lstdc++ -lsupc++ -lm -lc -lgcc -lnosys -lmbed -Wl,--end-group")

SET(CMAKE_C_FLAGS "-std=gnu99 -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fmessage-length=0 -fno-exceptions -fno-builtin -ffunction-sections -fdata-sections -funsigned-char -MMD -fno-delete-null-pointer-checks -fomit-frame-pointer -Os -g1 -DMBED_TRAP_ERRORS_ENABLED=1 -DMBED_RTOS_SINGLE_THREAD -mcpu=cortex-m0 -mthumb -include mbed_config.h")
#SET(CMAKE_CXX_FLAGS "-std=gnu++98 -fno-rtti -Wvla -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fmessage-length=0 -fno-exceptions -fno-builtin -ffunction-sections -fdata-sections -funsigned-char -MMD -fno-delete-null-pointer-checks -fomit-frame-pointer -Os -g1 -DMBED_TRAP_ERRORS_ENABLED=1 -DMBED_RTOS_SINGLE_THREAD -mcpu=cortex-m0 -mthumb  -include mbed_config.h")
SET(CMAKE_ASM_FLAGS "-x assembler-with-cpp -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fmessage-length=0 -fno-exceptions -fno-builtin -ffunction-sections -fdata-sections -funsigned-char -MMD -fno-delete-null-pointer-checks -fomit-frame-pointer -Os -g1 -DMBED_TRAP_ERRORS_ENABLED=1 -DMBED_RTOS_SINGLE_THREAD -mcpu=cortex-m0 -mthumb  -include mbed_config.h")
SET(CMAKE_CXX_LINK_FLAGS "-Wl,--gc-sections -Wl,--wrap,main -Wl,--wrap,__malloc_r -Wl,--wrap,__free_r -Wl,--wrap,__realloc_r -Wl,--wrap,__memalign_r -Wl,--wrap,__calloc_r -Wl,--wrap,exit -Wl,--wrap,atexit -Wl,-n --specs=nano.specs -mcpu=cortex-m0 -mthumb  -L${CMAKE_CURRENT_SOURCE_DIR}/mbed/TARGET_LPC1114/TOOLCHAIN_GCC_ARM ")
SET(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} ${LD_SYS_LIBS} -T ${CMAKE_BINARY_DIR}/mbed_blinky_pp.link_script.ld")

ADD_DEFINITIONS(
        -DARM_MATH_CM0
        -DCMSIS_VECTAB_VIRTUAL
        -DCMSIS_VECTAB_VIRTUAL_HEADER_FILE="cmsis_nvic.h"
        -DDEVICE_ANALOGIN=1
        -DDEVICE_ERROR_PATTERN=1
        -DDEVICE_I2C=1
        -DDEVICE_I2CSLAVE=1
        -DDEVICE_INTERRUPTIN=1
        -DDEVICE_PORTIN=1
        -DDEVICE_PORTINOUT=1
        -DDEVICE_PORTOUT=1
        -DDEVICE_PWMOUT=1
        -DDEVICE_SERIAL=1
        -DDEVICE_SLEEP=1
        -DDEVICE_SPI=1
        -DDEVICE_SPISLAVE=1
        -DDEVICE_STDIO_MESSAGES=1
        -DMBED_BUILD_TIMESTAMP=1559715806.72
        -DTARGET_CORTEX
        -DTARGET_CORTEX_M
        -DTARGET_LIKE_CORTEX_M0
        -DTARGET_LIKE_MBED
        -DTARGET_LPC1114
        -DTARGET_LPC11XX
        -DTARGET_LPC11XX_11CXX
        -DTARGET_LPCTarget
        -DTARGET_M0
        -DTARGET_NAME=LPC1114
        -DTARGET_NXP
        -DTARGET_RELEASE
        -DTOOLCHAIN_GCC
        -DTOOLCHAIN_GCC_ARM
        -D__CMSIS_RTOS
        -D__CORTEX_M0
        -D__MBED_CMSIS_RTOS_CM
        -D__MBED__=1
)

INCLUDE_DIRECTORIES(
        mbed/e95d10626187/platform
        mbed/e95d10626187/hal
        mbed/e95d10626187/drivers
        mbed/e95d10626187/TARGET_LPC1114/TARGET_NXP/TARGET_LPC11XX_11CXX/device
        mbed/e95d10626187/TARGET_LPC1114/TARGET_NXP/TARGET_LPC11XX_11CXX/TARGET_LPC11XX
        mbed/e95d10626187/TARGET_LPC1114/TARGET_NXP/TARGET_LPC11XX_11CXX
        mbed/e95d10626187/TARGET_LPC1114/TARGET_NXP
        mbed/e95d10626187/TARGET_LPC1114
        mbede95d10626187
        /usr/src/mbed-sdk
)

include_directories("${CMAKE_CURRENT_LIST_DIR}/src")
include_directories("${CMAKE_CURRENT_LIST_DIR}/lib/SerialSRAM")
include_directories("${CMAKE_CURRENT_LIST_DIR}/lib/MS5607")
include_directories("${CMAKE_CURRENT_LIST_DIR}/include")
include_directories("${CMAKE_CURRENT_LIST_DIR}/mbed/e95d10626187")

FILE(GLOB_RECURSE SRC_LIST "${CMAKE_CURRENT_LIST_DIR}/src/*.*" "${CMAKE_CURRENT_LIST_DIR}/lib/*.*")

add_executable(spc2019-parachute-mbed ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp mbed_config.h)


SET_TARGET_PROPERTIES(spc2019-parachute-mbed PROPERTIES ENABLE_EXPORTS 1)
# add syslibs dependencies to create the correct linker order
TARGET_LINK_LIBRARIES(spc2019-parachute-mbed -lstdc++ -lsupc++ -lm -lc -lgcc -lnosys)

add_custom_command(TARGET spc2019-parachute-mbed PRE_LINK
        COMMAND "arm-none-eabi-cpp" -E -P -Wl,--gc-sections -Wl,--wrap,main -Wl,--wrap,_malloc_r -Wl,--wrap,_free_r -Wl,--wrap,_realloc_r -Wl,--wrap,_memalign_r -Wl,--wrap,_calloc_r -Wl,--wrap,exit -Wl,--wrap,atexit -Wl,-n --specs=nano.specs -mcpu=cortex-m0 -mthumb mbed/TARGET_LPC1114/TOOLCHAIN_GCC_ARM/LPC1114.ld -o ${CMAKE_CURRENT_BINARY_DIR}/mbed_blinky_pp.link_script.ld
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        BYPRODUCTS "${CMAKE_CURRENT_BINARY_DIR}/mbed_blinky_pp.link_script.ld"
        )

add_custom_command(TARGET spc2019-parachute-mbed POST_BUILD
        COMMAND ${ELF2BIN} -O ihex $<TARGET_FILE:spc2019-parachute-mbed> $<TARGET_FILE:spc2019-parachute-mbed>.hex
        COMMAND ${CMAKE_COMMAND} -E echo "-- built: $<TARGET_FILE:spc2019-parachute-mbed>.hex"
        )

##########################################################################
# mbed-cli specific targets
##########################################################################

# detect the build type and select the corresponding cli profile
SET(MBED_BUILD_PROFILE "")
STRING(TOLOWER ${CMAKE_BUILD_TYPE} LOWERCASE_CMAKE_BUILD_TYPE)
IF(LOWERCASE_CMAKE_BUILD_TYPE MATCHES debug)
    SET(MBED_BUILD_PROFILE "mbed-os/tools/profiles/debug.json")
ELSEIF(LOWERCASE_CMAKE_BUILD_TYPE MATCHES relwithdebinfo)
    SET(MBED_BUILD_PROFILE "mbed-os/tools/profiles/develop.json")
ELSEIF(LOWERCASE_CMAKE_BUILD_TYPE MATCHES release)
    SET(MBED_BUILD_PROFILE "mbed-os/tools/profiles/release.json")
ELSEIF(LOWERCASE_CMAKE_BUILD_TYPE MATCHES minsizerel)
    SET(MBED_BUILD_PROFILE "mbed-os/tools/profiles/release.json")
ELSE()
    MESSAGE(WARNING "Build type '${CMAKE_BUILD_TYPE}' is unknown, using debug profile")
    SET(MBED_BUILD_PROFILE "mbed-os/tools/profiles/debug.json")
ENDIF()

# optional custom target to build via mbed-cli
ADD_CUSTOM_TARGET(mbed-cli-build
        COMMAND ${CMAKE_COMMAND} -E echo "mbed compile --build BUILD/${CMAKE_BUILD_TYPE} --profile ${MBED_BUILD_PROFILE}"
        COMMAND mbed compile --build BUILD/${CMAKE_BUILD_TYPE} --profile ${MBED_BUILD_PROFILE}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        SOURCES ${SOURCE_FILES} ${SYS_SOURCE_FILES})

IF(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/project.cmake)
    INCLUDE(${CMAKE_CURRENT_SOURCE_DIR}/project.cmake)
ELSE()
    MESSAGE(STATUS "Add a local project.cmake file to add your own targets.")
ENDIF()

# this will take care of binary directories generated by cmake/clion not to confuse the cli build
FILE(GENERATE OUTPUT "${CMAKE_BINARY_DIR}/.mbedignore" CONTENT "*")