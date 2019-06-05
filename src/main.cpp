/**
 * 最新の気圧情報（岩見沢）
 * https://www.jma.go.jp/jp/amedas_h/today-15356.html?areaCode=000&groupCode=13
 */
#include <mbed.h>
#include "MS5607I2C.h"
#include "SerialSRAM.h"

#define BUFFER_SIZE 8
#define CONFIG_MEMORY_AREA 25

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
 *
 * 0x0000-0x0003 float    海抜0mの気圧 (hPa)
 * 0x0004-0x0007 float    地上の気圧 (hPa)
 * 0x0008        uint8_t  上空判定 (0-255m)
 * 0x0009        uint8_t  開放判定 (0-255m)
 * 0x000a-0x000d float    サーボ開 duty比 (0-1.0f)
 * 0x000e-0x0011 float    サーボ閉 duty比 (0-1.0f)
 * 0x0012        uint8_t  ロギング設定 (0x00:無効 0x01:有効)
 * 0x0013-0x0016 uint32_t ロギング開始時刻(RTCから取得する）
 * 0x0017-0x0018 uint16_t 最終ログ格納アドレス (0x0020-0x0800)
 * 0x0019        uint8_t  ステータスフラグ格納領域
 * 0x001a-0x001f          (予備)
 * 0x0020-0x0800          ログデータ格納領域
 */
SerialSRAM *sram;

struct SystemArea {
    float pressureAt0m;
    float pressureAtGround;
    uint8_t aboveTheSky;
    uint8_t openParachuteAt;
    float openServoDuty;
    float closeServoDuty;
    uint8_t enableLogging;
    uint32_t loggingAt;
    uint16_t lastLogStoredAt;
    uint8_t statusFlags;
};
SystemArea config; // 設定情報を格納する

// Function Prototypes --------------------------------------------------------
void dumpSRAM(); // dump all data in SRAM
void loadConfig(); // load config data from SRAM

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
    loadConfig();

    //-----------
    // TEST
    //-----------
    dumpSRAM();

    //-----------
    // Main Loop
    //-----------
    while(isActive == 1) {
        led->write(!(led->read())); // reverse value
        wait(0.5);
    }

    // release objects
    delete serial;
    delete led;

    return 0;
}

// Functions  -----------------------------------------------------------------
void dumpSRAM() {
    char *buffer = new char[BUFFER_SIZE];
    uint16_t address;

    for(address=0x0000; address<0x0800; address+=BUFFER_SIZE) {
        // Seq. Read
        sram->read(address, buffer, BUFFER_SIZE);
        serial->printf("%04x %02x %02x %02x %02x %02x %02x %02x %02x"
                , address, buffer[0], buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7]);

        /*
        serial->printf("%04x", address);
        for(uint8_t i=0; i<BUFFER_SIZE; i++) {
            serial->printf("%02x ", buffer[i]);
        }
        serial->printf("\n");
        */
    }

    delete[] buffer;
}

void loadConfig() {
    char *buffer = new char[CONFIG_MEMORY_AREA];
    sram->read(0x0000, buffer, CONFIG_MEMORY_AREA); // read 0x0000-0x0019

    /**
     * 0x0000-0x0003 float    海抜0mの気圧 (hPa)
     * 0x0004-0x0007 float    地上の気圧 (hPa)
     * 0x0008        uint8_t  上空判定 (0-255m)
     * 0x0009        uint8_t  開放判定 (0-255m)
     * 0x000a-0x000d float    サーボ開 duty比 (0-1.0f)
     * 0x000e-0x0011 float    サーボ閉 duty比 (0-1.0f)
     * 0x0012        uint8_t  ロギング設定 (0x00:無効 0x01:有効)
     * 0x0013-0x0016 uint32_t ロギング開始時刻(RTCから取得する）
     * 0x0017-0x0018 uint16_t 最終ログ格納アドレス (0x0020-0x0800)
     * 0x0019        uint8_t  ステータスフラグ格納領域
     * 0x001a-0x001f          (予備)
     * 0x0020-0x0800          ログデータ格納領域
     */

    /**
     * float   pressureAt0m;
     * float   pressureAtGround;
     * uint8_t aboveTheSky;
     * uint8_t openParachuteAt;
     * float   openServoDuty;
     * float   closeServoDuty;
     * uint8_t enableLogging;
     * uint32_t loggingAt;
     * uint16_t lastLogStoredAt;
     * uint8_t statusFlags;
     */

    config.pressureAt0m     = (float) (buffer[3] >> 24 || buffer[2] >> 16 || buffer[1] >> 8 || buffer[0]);
    config.pressureAtGround = (float) (buffer[7] >> 24 || buffer[6] >> 16 || buffer[5] >> 8 || buffer[4]);
    config.aboveTheSky      = (uint8_t) buffer[8];
    config.openParachuteAt  = (uint8_t) buffer[9];
    config.openServoDuty    = (float) (buffer[13] >> 24 || buffer[12] >> 16 || buffer[11] >> 8 || buffer[10]);
    config.closeServoDuty   = (float) (buffer[17] >> 24 || buffer[16] >> 16 || buffer[15] >> 8 || buffer[14]);
    config.enableLogging    = (uint8_t) buffer[18];
    config.loggingAt        = (uint32_t) (buffer[22] >> 24 || buffer[21] >> 16 || buffer[20] >> 8 || buffer[19]);
    config.lastLogStoredAt  = (uint16_t) (buffer[24] >> 8 || buffer[23]);
    config.statusFlags      = (uint8_t) buffer[25];
}