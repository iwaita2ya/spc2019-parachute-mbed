/**
 * 最新の気圧情報（岩見沢）
 * https://www.jma.go.jp/jp/amedas_h/today-15356.html?areaCode=000&groupCode=13
 */
#include <mbed.h>
//#include "lib/MS5607/MS5607I2C.h"
#include "lib/SerialSRAM/SerialSRAM.h"

/**
 * flags
 */
uint8_t isActive;

/**
 * Serial Port
 */
RawSerial *serial;

/**
 * LED
 */
DigitalOut *led; // LED

using namespace greysound;

/**
 * SRAM
 */
SerialSRAM *sram;

// Main  ----------------------------------------------------------------------
int main() {
    /**
     * Init
     */
    // set active flag
    isActive = 1;
    led = new DigitalOut(dp14);

    // set serial baud rate
    serial = new RawSerial(dp16, dp15); // tx, rx
    serial->baud(115200); // default:9600bps

    // init SRAM
    sram = new SerialSRAM(dp5, dp27, dp26); // sda, scl, hs, A2=0, A1=0

    //-----------
    // Main Loop
    //-----------
    while(isActive == 1) {
        led->write(!(led->read())); // reverse value
        wait(0.5);
    }

    return 0;

}
