/**
 * 最新の気圧情報（岩見沢）
 * https://www.jma.go.jp/jp/amedas_h/today-15356.html?areaCode=000&groupCode=13
 */
#include <mbed.h>
#include "MS5607I2C.h"
#include "SerialSRAM.h"
#include "ServoManager.h"
#include "SensorManager.h"

#define BUFFER_SIZE 16
#define CONFIG_MEMORY_AREA_SIZE 30
#define SENSOR_UPDATE_FREQ 0.1

using namespace greysound;

/**
 * flags
 */
uint8_t shouldLoop;
uint8_t currentState;

// 状態管理
enum MCUState {
    STAND_BY,   // 準備完了
    GET_START,  // 開始指示受信
    BUSY,       // 開始中
    GET_PAUSE,  // 一時停止
    ERROR       // エラー発生
};

enum RESULTS {
    RESULT_OK = 0,
    RESULT_NG = 1
};


/**
 * Serial Port
 */
RawSerial *serial;

/**
 * Servo Manager
 */
ServoManager *servoManager;

/**
 * Sensor Manager
 */
SensorManager *sensorManager;
Ticker *sensorTicker;   // センサ更新タイマ

/**
 * Buttons
 */
InterruptIn *setAltitudePin;    // 地表高度セットトリガ
InterruptIn *servoControlPin;   // パラシュートロック／アンロックトリガ

/**
 * LED
 */
DigitalOut *led; // LED


/**
 * SRAM
 *
 * 0x0000-0x0003 float    海抜0mの気圧 (hPa)
 * 0x0004-0x0007 float    地上の気圧 (hPa)
 * 0x0008        uint8_t  上空判定 (0-255m)
 * 0x0009        uint8_t  開放判定 (0-255m)
 * 0x000a-0x000d float    サーボ周期 Futaba:20ms(0.020f)
 * 0x000e-0x0011 float    サーボ開 duty比 (0-1.0f)
 * 0x0012-0x0015 float    サーボ閉 duty比 (0-1.0f)
 * 0x0016        uint8_t  ロギング設定 (0x00:無効 0x01:有効)
 * 0x0017-0x001a time_t   ロギング開始時刻(RTCから取得する）
 * 0x001b-0x001c uint16_t 最終ログ格納アドレス (0x0020-0x0800)
 * 0x001d        uint8_t  ステータスフラグ格納領域
 * 0x001e-0x001f          (予備)
 * 0x0020-0x0800          ログデータ格納領域
 */
SerialSRAM *sram;

struct SystemArea {
    float pressureAt0m;
    float pressureAtGround;
    uint8_t aboveTheSkyAt;
    uint8_t openParachuteAt;
    float servoPeriod;
    float openServoDuty;
    float closeServoDuty;
    uint8_t enableLogging;
    time_t logStartTime;
    uint16_t lastLogAddress;
    uint8_t statusFlags;
};
SystemArea *config; // 設定情報を格納する

// Function Prototypes --------------------------------------------------------
// ----- SRAM -----
void dumpAll();                         // dump all data in SRAM
void dumpConfig();                      // dump config memory area in SRAM
void clearLog(uint16_t startAddress);   // clear logged data, not config
void resetConfig();                     // reset config with default value;
void loadConfig();                      // load config data from SRAM

// ----- SERVO -----
static void changeServoState();

// ----- ALTIMETER -----
static void setAltitude();

// ----- Sensor -----
static uint8_t startSensor();
static void updateSensor();
static uint8_t stopSensor();
void dumpSensorValues();

// ----- Utils -----
int32_t x10(float);

// Main  ----------------------------------------------------------------------
int main() {
    /**
     * Init Objects/Vars
     */
    // set active flag
    shouldLoop = 1;

    // getStandBy serial baud rate
    serial = new RawSerial(dp16, dp15); // tx, rx
    serial->baud(115200); // default:9600bps

    // getStandBy SRAM
    sram = new SerialSRAM(dp5, dp27, dp26); // sda, scl, hs, A2=0, A1=0
    config = new SystemArea();
//    loadConfig();

    //-----------
    // SRAM TEST //MEMO: まだ途中
    //-----------
//    // reset config with default value
//    resetConfig();
//    // dump config
//    dumpConfig();

    // getStandBy ServoManager
    servoManager = new ServoManager(dp18);
    servoManager->setRange(0.03f, 0.037f); // minValue, maxValue
    servoManager->init();
    servoManager->moveRight(); // open

    // getStandBy SensorManager (and Ticker)
    sensorTicker  = new Ticker();
    sensorManager = new SensorManager(dp5, dp27, 0xD6, 0x3C); // sda, scl, agAddr, mAddr
    sensorManager->getStandBy(); //reset altitude, reset counter, then set as STAND_BY

    //-----------
    // SENSOR TEST
    //-----------
    dumpSensorValues();

    // getStandBy buttons #1
    servoControlPin = new InterruptIn(dp17);// サーボ操作ボタン
    servoControlPin->mode(PullUp);
    servoControlPin->fall(&changeServoState);

    // getStandBy buttons #2
    setAltitudePin = new InterruptIn(dp28); // 地表高度設定ボタン
    setAltitudePin->mode(PullUp);
    setAltitudePin->fall(&setAltitude);

    // getStandBy LED
    led = new DigitalOut(dp14);

    // スタンバイ状態に遷移する
    currentState = STAND_BY;

    /**
     * Main Loop
     */
    while(shouldLoop == 1) {
        //TODO: ステータスに応じて処理を分岐させる
        uint8_t result = 0;
        switch (currentState) {

            case GET_START:
                result = startSensor();
                // 開始成功なら BUSY 状態に遷移
                if(result == RESULT_OK) {
                    currentState = BUSY;
                }
                break;

            case GET_PAUSE:
                result = stopSensor();
                // 停止成功なら STAND_BY 状態に遷移
                if(result == RESULT_OK) {
                    currentState = STAND_BY;
                }
                break;

            default:
                // blink LED
                led->write(!(led->read())); // reverse value
                wait(0.5);
                break;
        }
    }

    /**
     * Exit
     */
    sensorTicker->detach();

    // release objects
    delete led;
    delete setAltitudePin;
    delete servoControlPin;
    delete sensorManager;
    delete sensorTicker;
    delete servoManager;
    delete sram;
    delete serial;

    return RESULT_OK;
}

// Functions  -----------------------------------------------------------------
/**
 * SRAM
 */

void dumpAll() {

    static const uint8_t bufferLength = 16;
    char *buffer = new char[bufferLength];

    serial->printf("\r\nADDR 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f\r\n");
    serial->printf("----------------------------------------------------\r\n");

    for(uint16_t address=0x0000; address<0x0800; address+=bufferLength) {
        // Seq. Read
        sram->read(address, buffer, bufferLength);

        serial->printf("%04x ", address);
        for(uint8_t i=0; i<bufferLength; i++) {
            serial->printf("%02x ", buffer[i]);
        }
        serial->printf("\r\n");
    }

    delete[] buffer;
}

void dumpConfig() {

    static const uint8_t bufferLength = 16;
    char *buffer = new char[bufferLength];

    serial->printf("\r\nADDR 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f\r\n");
    serial->printf("----------------------------------------------------\r\n");

    for(uint16_t address=0x0000; address<0x0020; address+=bufferLength) {
        // Seq. Read
        sram->read(address, buffer, bufferLength);

        serial->printf("%04x ", address);
        for(uint8_t i=0; i<bufferLength; i++) {
            serial->printf("%02x ", buffer[i]);
        }
        serial->printf("\r\n");
    }

    delete[] buffer;
}

void resetConfig() {

    // getStandBy config with default value
    config->pressureAt0m     = 0.0f;
    config->pressureAtGround = 0.0f;
    config->aboveTheSkyAt    = 30;
    config->openParachuteAt  = 20;
    config->servoPeriod      = 0.020f; // 20 ms
    config->openServoDuty    = 0.120f; // 0-1.0f
    config->closeServoDuty   = 0.025f; // 0-1.0f
    config->enableLogging    = 0x01;   // 0x01:true 0x00:false
    config->logStartTime     = 0x01020304;
    config->lastLogAddress   = 0x0020; // 0x0020-0x0800
    config->statusFlags      = 0x01;

    // save onto SRAM
    sram->write(0x0000, (char*)config, CONFIG_MEMORY_AREA_SIZE);
}

void loadConfig() {

    char *buffer = new char[CONFIG_MEMORY_AREA_SIZE];
    sram->read(0x0000, buffer, CONFIG_MEMORY_AREA_SIZE); // update 0x0000-0x0019

    config->pressureAt0m     = (float) (buffer[3] >> 24 || buffer[2] >> 16 || buffer[1] >> 8 || buffer[0]);
    config->pressureAtGround = (float) (buffer[7] >> 24 || buffer[6] >> 16 || buffer[5] >> 8 || buffer[4]);
    config->aboveTheSkyAt    = (uint8_t) buffer[8];
    config->openParachuteAt  = (uint8_t) buffer[9];
    config->servoPeriod      = (float) (buffer[13] >> 24 || buffer[12] >> 16 || buffer[11] >> 8 || buffer[10]);
    config->openServoDuty    = (float) (buffer[17] >> 24 || buffer[16] >> 16 || buffer[15] >> 8 || buffer[14]);
    config->closeServoDuty   = (float) (buffer[21] >> 24 || buffer[20] >> 16 || buffer[19] >> 8 || buffer[18]);
    config->enableLogging    = (uint8_t) buffer[22];
    config->logStartTime     = (uint32_t) (buffer[26] >> 24 || buffer[25] >> 16 || buffer[24] >> 8 || buffer[23]);
    config->lastLogAddress   = (uint16_t) (buffer[28] >> 8 || buffer[27]);
    config->statusFlags      = (uint8_t) buffer[29];
}

/**
 * SENSOR
 */

// センサ開始
static uint8_t startSensor()
{
    // センサ開始
    if(sensorManager && sensorManager->getCurrentState() == SensorManager::STAND_BY)
    {
        uint8_t result = sensorManager->begin();

        // error
        if(result != 0) {
            return RESULT_NG;
        }
    } else {
        // couldn't start sensor
        return RESULT_NG;
    }

    // センサ値更新処理開始
    if(sensorTicker) {
        sensorTicker->attach(&updateSensor, SENSOR_UPDATE_FREQ); //MEMO: only :void can be attached
    }

    return RESULT_OK;
}

// センサ情報を更新する
static void updateSensor()
{
    if(sensorManager) {
        sensorManager->update();
    }
}

static uint8_t stopSensor()
{
    // センサ値更新処理停止
    if(sensorTicker) {
        sensorTicker->detach();
    }

    // センサ停止
    if(sensorManager && sensorManager->getCurrentState() != SensorManager::STAND_BY) {
        sensorManager->end();
    }

    return RESULT_OK;
}

//
void dumpSensorValues() {
    if(sensorManager != NULL) {
        sensorManager->updateForced();
        serial->printf("\r\n----------------------");
        serial->printf("\r\n dumpSensorValues()");
        serial->printf("\r\n----------------------");
        serial->printf("\r\nPressure: %ld", (uint32_t)(sensorManager->currentPressure));
        serial->printf("\r\nALT(x10): %ld", (uint32_t)x10(sensorManager->currentAltitude));
        serial->printf("\r\nTMP(x10): %ld", (uint32_t)x10(sensorManager->currentTemperature));
    }
}

/**
 * BUTTONS
 */

// ボタン押下に応じてサーボの開閉を行う
static void changeServoState()
{
    static uint8_t currentState = 0;

    if(servoManager != NULL)
    {
        if (currentState == 1) {
            servoManager->moveRight();  // lock
        } else {
            servoManager->moveLeft();   // release
        }
        currentState = static_cast<uint8_t>(1 - currentState);
    }
}

// 地上の高度をセットする
static void setAltitude()
{
    if(sensorManager != NULL)
    {
        // 地表高度設定
        sensorManager->updateAltitudesFromCurrentAltitude();
    }
}

/**
 * Utils
 */

int32_t x10(float val)
{
    int32_t int32 = (int16_t)(val * 100);
    if( int32 % 10 >= 5 )
    {
        int32 = (int16_t)((int32 / 10) + 1);
    }
    else if( int32 % 10 <= -5 )
    {
        int32 = (int16_t)((int32 / 10) - 1);
    }
    else
    {
        int32 /= 10 ;
    }

    return int32 ;
}
