#define DEBUG

/**
 * 最新の気圧情報
 * --- 岩見沢 ---
 * https://www.jma.go.jp/jp/amedas_h/today-15356.html?areaCode=000&groupCode=13
 * --- 札幌 ---
 * https://www.jma.go.jp/jp/amedas_h/today-14163.html?areaCode=000&groupCode=12
 */

#include <mbed.h>
#include "SystemParameters.h"
#include "MS5607I2C.h"
#include "SerialSRAM.h"
#include "ServoManager.h"
#include "SensorManager.h"

#define CONFIG_AREA_SIZE 0x20   // 0x00-0x20
#define CONFIG_LOG_SIZE 0x02    // Status + Altitude = 2byte
#define SRAM_LOG_START_AT 0x0020    // Status + Altitude = 2byte
#define SRAM_MAX_SIZE 0x0800    // 0x0000-0x0800
#define SENSOR_UPDATE_FREQ 0.1  // センサ更新頻度

using namespace greysound;

// 状態管理
enum MCUState {
    INIT       = 0x01,
    STAND_BY   = 0x02,
    FLYING     = 0x04,
    FALLING    = 0x08,
    OPEN_PARA  = 0x10,
    TOUCH_DOWN = 0x20,
    FINISH     = 0x40,
    ERROR      = 0x80
};

enum RESULTS {
    RESULT_OK = 0,
    RESULT_NG = 1
};

enum SERIAL_OUT {
    ENABLE_SERIAL = 0,
    DISABLE_SERIAL = 1
};

/**
 * flags
 */
uint8_t shouldLoop;

/**
 * Serial Port
 */
RawSerial *serial;


// Circular buffers for serial TX and RX data - used by interrupt routines
const int serialBufferSize = 64;

// might need to increase buffer size for high baud rates
char txBuffer[serialBufferSize+1];
char rxBuffer[serialBufferSize+1];

// Circular buffer pointers
// volatile makes read-modify-write atomic
volatile int txInPointer=0;
volatile int txOutPointer=0;
volatile int rxInPointer=0;
volatile int rxOutPointer=0;

// Line buffers for sprintf and sscanf
char txLineBuffer[32];
char rxLineBuffer[32];

#ifdef DEBUG
#define DEBUG_PRINT(fmt) serial->printf(fmt)
#define DEBUG_PRINTF(fmt, ...) serial->printf(fmt, __VA_ARGS__)
#define DEBUG_PUTC(x) serial->putc(x)
#else
#define DEBUG_PRINT(fmt)
#define DEBUG_PRINTF(fmt, ...)
#define DEBUG_PUTC(x)
#endif

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
InterruptIn *initProbePin;      // 開始信号

/**
 * Digital Out
 */
DigitalOut *led; // LED
DigitalOut *falling; // set low if falling

/**
 * Jumper
 */
DigitalIn *enableSerialPin;

/**
 * SRAM
 * 0x0000        uint8_t  ステータスフラグ格納領域
 * 0x0001-0x0004 float    海抜0mの気圧 (hPa)
 * 0x0005-0x0006 uint16_t 地表の高度 (0-65536m)
 * 0x0007-0x0008 uint16_t 現在の高度 (0-65536m)
 * 0x0009        uint8_t  パラシュート開放高度(地表高度に加算) (0-256m)
 * 0x000A        uint8_t  状態カウンタしきい値 (0-255回)
 * 0x000B        uint8_t  状態遷移に必要な高度しきい値 (0-255回)
 * 0x000C-0x000F float    サーボ開 duty比 (0-1.0f)
 * 0x0010-0x0013 float    サーボ閉 duty比 (0-1.0f)
 * 0x0014        uint8_t  ロギング設定 (0x00:無効 0x01:有効)
 * 0x0015-0x0018 time_t   ロギング開始時刻(RTCから取得する）
 * 0x0019-0x001A uint16_t 最終ログ格納アドレス (0x0020-0x0800)
 * 0x001B-0x001F (予備)
 * 0x0020-0x0800          ログデータ格納領域
 */
SerialSRAM *sram;

SystemParameters *config; // 設定情報を格納する

// Function Prototypes --------------------------------------------------------

// ----- Serial -----
void interruptTx();
void interruptRx();
void sendLine();
void readLine();

// ----- SRAM -----
void printStatusSRAM();                 // get status from SRAM (hex)
void printStatusVarsReadable();         // get config data from SRAM (ascii)
void updateStatus(uint8_t newStatus);   // update status
void printConfigSRAM();                 // get config data from SRAM (hex)
void printConfigSRAMReadable();         // get config data from SRAM (ascii)
void printConfigVars();                 // get config data from SRAM (hex)
void printConfigVarsReadable();         // get config data from SRAM (ascii)
void loadConfig();                      // load config data from SRAM to Vars
void saveConfig();                      // Save config data onto SRAM
/**
 * config->statusFlags         = 0x00;      // ステータスフラグ
 * config->pressureAtSeaLevel  = 101440.0f; // 海抜0mの大気圧(低すぎると高度が0になるので注意)
 * config->groundAltitude      = 0;        // 地表高度
 * config->currentAltitude     = 0;        // 現在高度
 * config->deployParachuteAt   = 20;        // パラシュート開放高度(地表高度に加算)
 * config->counterThreshold    = 3;         // 状態カウンタのしきい値（この回数に達したら、その状態が発生したと判断する）
 * config->altitudeThreshold   = 20;         // 現在高度が地表高度＋この値を上回っていたら飛行中とみなす
 * config->openServoPeriod     = 0.039f;    // 0-1.0f // close
 * config->closeServoPeriod    = 0.030f;     // 0-1.0f
 * config->enableLogging       = 0x00;      // 0x01:true 0x00:false
 * config->logStartTime        = 0x01020304;
 * config->logPointer          = SRAM_LOG_START_AT;    // 0x0021-0x0800
 */
void updateConfig();
void resetConfig();                     // Init config with default value (and save onto SRAM)
void dumpMemory();                      // dump all data in SRAM (hex)
void dumpMemoryReadable();              // dump all data in SRAM (ascii)
void clearLog(uint16_t startAddress=CONFIG_AREA_SIZE, uint16_t endAddress=SRAM_MAX_SIZE); // clear logged data
void writeCurrentData(uint8_t currentStatus, uint8_t currentAltitude);

// ----- SERVO -----
static void changeServoState();         // open/close servo

// ----- ALTIMETER -----
static void setGroundAltitude();

// ----- Start Device -----
static void startDevice();

// ----- Reset Device -----
static void resetDevice();

// ----- Sensor -----
static uint8_t startSensor();
static void updateSensor();
static uint8_t stopSensor();
void printSensorValues();
void printSensorValuesReadable();

// ----- Utils -----
int32_t x10(float);

// Main  ----------------------------------------------------------------------
int main() {

    /**
     * Init Objects/Vars
     */
    // set active flag
    shouldLoop = 1;

    // set Serial Enable/Disable
    enableSerialPin = new DigitalIn(P1_15);
    enableSerialPin->mode(PullUp);

    // get StandBy serial baud rate
    serial = new RawSerial(P0_19, P0_18); // tx, rx
    serial->baud(115200); // default:9600bps
    // set interrupts for Tx/Rx
    serial->attach(&interruptRx, Serial::RxIrq);
    //serial->attach(&interruptTx, Serial::TxIrq); // not used

    // get StandBy SRAM
    sram = new SerialSRAM(P0_5, P0_4, P0_21); // sda, scl, hs, A2=0, A1=0
    sram->setAutoStore(1); // enable auto-store

    config = new SystemParameters();
//    resetConfig(); //MEMO: 設定初期化 (DEBUG ONLY)
    loadConfig(); // 設定をSRAMから変数に読込

    // getStandBy ServoManager
    servoManager = new ServoManager(P0_22);
    servoManager->setRange(config->closeServoPeriod, config->openServoPeriod); // minValue, maxValue //FIXME: 設定値変更時に再度呼ぶ必要がある

    // getStandBy SensorManager (and Ticker)`
    sensorTicker  = new Ticker();
    sensorManager = new SensorManager(P0_5, P0_4, 0xD6, 0x3C, config); // sda, scl, agAddr, mAddr

    // Set Altitude Button
    setAltitudePin = new InterruptIn(P1_19); // 地表高度設定ボタン
    setAltitudePin->mode(PullUp);
    setAltitudePin->fall(&setGroundAltitude);

    // Servo Open/Close Buttons
    servoControlPin = new InterruptIn(P0_20);// サーボ操作ボタン
    servoControlPin->mode(PullUp);
    servoControlPin->fall(&changeServoState);

    // Init Probe Buttons
    initProbePin = new InterruptIn(P0_23);     // デバイス開始ボタン
    initProbePin->mode(PullUp);
    initProbePin->fall(&startDevice);

    // set falling as high
    falling = new DigitalOut(P0_17);
    falling->write(1);

    // LED 点滅開始
    led = new DigitalOut(P0_7);
    led->write(0); // set led off

    /**
     * Main Loop
     */
    while(shouldLoop == 1) {

        /**
        * プローブステータスに応じて処理を分岐
        * ステータスフラグはINITから順番に立てられる（クリアされない）ので
        * FINISHから順に判定する
        */
        if(config->statusFlags & FINISH) { // 終了
            // DO Nothing
        }
        else if(config->statusFlags & TOUCH_DOWN) { // 着地
            // システム停止
            if(stopSensor() == RESULT_OK) {
                // ロギング停止
                config->enableLogging = false;

                // save data onto EEPROM
                sram->callHardwareStore();

                // センサ停止したら FINISH に遷移
                if(stopSensor() == RESULT_OK) {
                    //DEBUG_PRINT("TOUCH_DOWN->FINISH\r\n");
                    updateStatus(config->statusFlags | FINISH);
                }
            }
        }
        else if(config->statusFlags & OPEN_PARA) { // パラシュート開放
            // パラシュート開放
            servoManager->moveRight(); // open

            // 現在高度が地上高度と等しくなったら TOUCH_DOWN に遷移
            if(sensorManager->isTouchDown()) {
                //DEBUG_PRINT("OPEN_PARA->TOUCH_DOWN\r\n");
                updateStatus(config->statusFlags | TOUCH_DOWN);
                falling->write(1);
            }
        }
        else if(config->statusFlags & FALLING) { // 落下中

            // 開放高度に達したら OPEN_PARA に遷移
            if(sensorManager->isOkToDeployParachute()) {
                //DEBUG_PRINT("FALLING->OPEN_PARA\r\n");
                updateStatus(config->statusFlags | OPEN_PARA);
            }
        }
        else if(config->statusFlags & FLYING) { // 飛行中
            //高度が減少に転じたら FALLING に遷移
            if(sensorManager->isFalling()) {
                //DEBUG_PRINT("FLYING->FALLING\r\n");
                updateStatus(config->statusFlags | FALLING);
                falling->write(0);
            }
        }
        else if(config->statusFlags & STAND_BY) { //
            // 規定高度に達したら FLYING に遷移
            if(sensorManager->isFlying()) {
                //DEBUG_PRINT("STAND_BY->FLYING\r\n");
                updateStatus(config->statusFlags | FLYING);
                // ロギング開始
                config->enableLogging = true;
            }
        }
        else if(config->statusFlags & INIT) {
            // センサ開始したら STAND_BY に遷移
            if(startSensor() == RESULT_OK) {
                //DEBUG_PRINT("INIT->STAND_BY\r\n");
                updateStatus(config->statusFlags | STAND_BY);
            }
        }

        // ロギングが有効ならログに記録
        if(config->enableLogging) {
            writeCurrentData(config->statusFlags, config->currentAltitude);
        }

        /**
         * シリアル通信の受送信処理
         * 受信データの最初の1バイトがコマンドバイト(CB)なのでその値をチェックして処理を分岐
         * 更新系のコマンドについては、CBの後に更新値が1−20バイト続く
         */
        if(enableSerialPin->read() == ENABLE_SERIAL || config->statusFlags & FINISH) {

            // data received and not read yet?
            if (rxInPointer != rxOutPointer) {
                //DEBUG_PRINTF("Diff: rxIN:%d, rxOut:%d\r\n", rxInPointer, rxOutPointer);

                readLine();
                char commandByte = rxLineBuffer[0];

                switch (commandByte) {
//                    case 0x00: // ステータス表示 (hex)
//                        printStatusSRAM();
//                        break;
//                    case 0x10: // ステータス表示 (ascii)
//                        printStatusVarsReadable();
//                        break;
//                    case 0x20: // ステータス更新
//                        updateStatus(rxLineBuffer[1]);
//                        break;
//                    case 0x30: // ステータス初期化
//                        updateStatus(0x00);
//                        break;
                    case 0x40: // SRAM 設定表示 (hex)
                        printConfigSRAM();
                        break;
                    case 0x41: // SRAM 設定表示 (ascii)
                        printConfigSRAMReadable();
                        break;
                    case 0x50: // 変数設定表示 (hex)
                        printConfigVars();
                        break;
                    case 0x51: // 変数設定表示 (ascii)
                        printConfigVarsReadable();
                        break;
                    case 0x60: //TODO: 設定更新
                        break;
                    case 0x70: // 設定初期化
                        resetConfig();
                        break;
                    case 0x71: // 設定をSRAMから読み込む
                        loadConfig();
                        break;
                    case 0x72: // 設定をSRAMに書き込む
                        saveConfig();
                        break;
                    case 0x80: // ログ書込
                        writeCurrentData(config->statusFlags, config->currentAltitude);
                        break;
                    case 0x90: // ログデータ消去
                        clearLog();
                        break;
                    case 0xA0: // メモリダンプ　(hex)
                        dumpMemory();
                        break;
                    case 0xB0: // メモリダンプ　(ascii)
                        dumpMemoryReadable();
                        break;
                    case 0xC0: // センサ値取得 (hex)
                        printSensorValues();
                        break;
//                    case 0xD0: // センサ値取得 (ascii)
//                        printSensorValuesReadable();
//                        break;
                    case 0xE0: // サーボ開閉
                        servoManager->flipState();
                        break;
                    case 0xF0: // 地表高度設定
                        setGroundAltitude();
                        break;
                    case 0xF1: // デバイス開始
                        startDevice();
                        break;
//                    case 0xF2: // デバイス初期化
//                        resetDevice();
//                        break;
                    default:
                        break;
                }
            }
        }

        // blink LED
        led->write(!(led->read()));
        wait(0.1);
    }

    /**
     * Exit
     */
    // stop timers
    sensorTicker->detach();

    // release objects
    delete(sensorTicker);
    delete(led);
    delete(initProbePin);
    delete(setAltitudePin);
    delete(servoControlPin);
    delete(sensorManager);
    delete(servoManager);
    delete(sram);
    delete(serial);

    return RESULT_OK;
}

// Functions  -----------------------------------------------------------------
/**
 * Serial
 */

// Copy tx line buffer to large tx buffer for tx interrupt routine
void sendLine() {

    int i;
    char temp_char;
    bool empty;
    i = 0;

    // Start Critical Section - don't interrupt while changing global buffer variables
    NVIC_DisableIRQ(UART_IRQn);
    empty = (txInPointer == txOutPointer);

    while ((i==0) || (txLineBuffer[i-1] != '\n')) {
        // Wait if buffer full
        if (((txInPointer + 1) % serialBufferSize) == txOutPointer) {

            // End Critical Section - need to let interrupt routine empty buffer by sending
            NVIC_EnableIRQ(UART_IRQn);
            while (((txInPointer + 1) % serialBufferSize) == txOutPointer) {
            }

            // Start Critical Section - don't interrupt while changing global buffer variables
            NVIC_DisableIRQ(UART_IRQn);
        }

        txBuffer[txInPointer] = txLineBuffer[i];
        i++;
        txInPointer = (txInPointer + 1) % serialBufferSize;
    }

    if (serial->writeable() && (empty)) {
        temp_char = txBuffer[txOutPointer];
        txOutPointer = (txOutPointer + 1) % serialBufferSize;
        // Send first character to start tx interrupts, if stopped
        serial->putc(temp_char);
    }
    // End Critical Section
    NVIC_EnableIRQ(UART_IRQn);
}


// Read a line from the large rx buffer from rx interrupt routine
void readLine() {

    // Start Critical Section - don't interrupt while changing global buffer variables
    NVIC_DisableIRQ(UART_IRQn);

    // Loop reading rx buffer characters until end of line character
    int i = 0;
    while (rxInPointer != rxOutPointer) {
        rxLineBuffer[i++] = rxBuffer[rxOutPointer];
        rxOutPointer = (rxOutPointer + 1) % serialBufferSize;
        //DEBUG_PRINTF("rxInPointer:%d rxOutPointer:%d\r\n", rxInPointer, rxOutPointer);
    }

    // End Critical Section
    NVIC_EnableIRQ(UART_IRQn);
}


// Interrupt Routine to read in data from serial port
void interruptRx() {

    // Loop just in case more than one character is in UART's receive FIFO buffer
    // Stop if buffer full
    while ((serial->readable()) && (((rxInPointer + 1) % serialBufferSize) != rxOutPointer)) {
        rxBuffer[rxInPointer] = serial->getc();

        // Uncomment to Echo to USB serial to watch data flow
        //DEBUG_PUTC(rxBuffer[rxInPointer]); // echo back
        rxInPointer = (rxInPointer + 1) % serialBufferSize;
    }
}


// Interrupt Routine to write out data to serial port
void interruptTx() {

    // Loop to fill more than one character in UART's transmit FIFO buffer
    // Stop if buffer empty
    while ((serial->writeable()) && (txInPointer != txOutPointer)) {
        serial->putc(txBuffer[txOutPointer]);
        txOutPointer = (txOutPointer + 1) % serialBufferSize;
    }
}

/**
 * SRAM
 */
// dump all data stored in SRAM (hex)
void dumpMemory() {

    static const uint8_t bufferLength = 16;
    char *buffer = new char[bufferLength];

    for(uint16_t address=SRAM_LOG_START_AT; address<SRAM_MAX_SIZE; address+=bufferLength) {

        // Seq. Read
        sram->read(address, buffer, bufferLength);

        for(uint8_t i=0; i<bufferLength; i++) {
            serial->putc(buffer[i]);
            wait_ms(10); // 表示崩れ対策
        }
    }

    delete[] buffer;
}

// dump all data stored in SRAM (ascii)
void dumpMemoryReadable() {

    static const uint8_t bufferLength = 16;
    char *buffer = new char[bufferLength];

    serial->printf("ADDR 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f\r\n");
    serial->printf("----------------------------------------------------\r\n");

    for(uint16_t address=0x0000; address<SRAM_MAX_SIZE; address+=bufferLength) {
        // Seq. Read
        sram->read(address, buffer, bufferLength);

        serial->printf("%04x ", address);
        for(uint8_t i=0; i<bufferLength; i++) {
            serial->printf("%02x ", buffer[i]);
        }
        serial->printf("\r\n");
        wait_ms(100); // 表示崩れ対策
    }

    delete[] buffer;
}

void printStatusSRAM() { //MEMO: 廃止予定

    char statusByte = 0x00;

    // Read single byte at 0x0000
    sram->read(0x0000, &statusByte);

    serial->putc(statusByte);
}

void printStatusVarsReadable() { //MEMO: 廃止予定

    // read data from config
    char statusByte = config->statusFlags;

//    // read data from sram
//    sram->read(0x0000, &statusByte);

    serial->printf("IN ST FL FA OP TD FN ER\r\n");
    serial->printf("-----------------------\r\n");
    serial->printf(" %d  %d  %d  %d  %d  %d  %d  %d\r\n"
            , ((statusByte & 0x01) ? 1 : 0)
            , ((statusByte & 0x02) ? 1 : 0)
            , ((statusByte & 0x04) ? 1 : 0)
            , ((statusByte & 0x08) ? 1 : 0)
            , ((statusByte & 0x10) ? 1 : 0)
            , ((statusByte & 0x20) ? 1 : 0)
            , ((statusByte & 0x40) ? 1 : 0)
            , ((statusByte & 0x80) ? 1 : 0)
            );
}

void updateStatus(uint8_t newStatus) {

    // update status var
    config->statusFlags = newStatus;

    // update sram
    sram->write(0x0000, newStatus);
}

/*
 * SRAMに格納されている設定値を返す(hex)
 */
void printConfigSRAM() {

    char *buffer = new char[CONFIG_AREA_SIZE];

    // Seq. Read
    sram->read(0x0000, buffer, CONFIG_AREA_SIZE);

    for(uint8_t i=0; i<CONFIG_AREA_SIZE; i++) {
        serial->putc(buffer[i]);
    }

    delete[] buffer;
}

/*
 * SRAMに格納されている設定値を返す(ascii)
 */
void printConfigSRAMReadable() {

    static const uint8_t bufferLength = 16;
    char *buffer = new char[bufferLength];

    // format in hex
    serial->printf("ADDR 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f\r\n");
    serial->printf("----------------------------------------------------\r\n");

    for(uint16_t address=0x0000; address<0x0020; address+=bufferLength) {
        // Seq. Read
        sram->read(address, buffer, bufferLength);

        serial->printf("%04X ", address);
        for(uint8_t i=0; i<bufferLength; i++) {
            serial->printf("%02X ", buffer[i]);
        }
        serial->printf("\r\n");
    }

    delete[] buffer;
}

/*
 * 変数に格納された設定値を返す(hex)
 */
void printConfigVars() {

    unsigned char charValue[sizeof(time_t)];
    uint8_t i;

    serial->putc(config->statusFlags); // ステータスフラグ
    memcpy(charValue, &config->pressureAtSeaLevel, sizeof(float));  // 海抜0mの気圧
    for(i=0;i<sizeof(float);i++) {
        serial->putc(charValue[i]);
    }

    memcpy(charValue, &sensorManager->currentPressure, sizeof(float)); // 現在の気圧
    for(i=0;i<sizeof(float);i++) {
        serial->putc(charValue[i]);
    }

    memcpy(charValue, &config->groundAltitude, sizeof(uint16_t));   // 地上高度
    for(i=0;i<sizeof(uint16_t);i++) {
        serial->putc(charValue[i]);
    }
    memcpy(charValue, &config->currentAltitude, sizeof(uint16_t));  // 現在高度
    for(i=0;i<sizeof(uint16_t);i++) {
        serial->putc(charValue[i]);
    }
    serial->putc(config->deployParachuteAt);                        // パラシュート開放高度
    serial->putc(config->counterThreshold);                         // counter threshold
    serial->putc(config->altitudeThreshold);                        // altitude threshold
    memcpy(charValue, &config->openServoPeriod, sizeof(float));     // open servo period
    for(i=0;i<sizeof(float);i++) {
        serial->putc(charValue[i]);
    }
    memcpy(charValue, &config->closeServoPeriod, sizeof(float));    // close servo period
    for(i=0;i<sizeof(float);i++) {
        serial->putc(charValue[i]);
    }
    serial->putc(config->enableLogging);                            // logging enable/disable
    memcpy(charValue, &config->logStartTime, sizeof(time_t));       // logging started time
    for(i=0;i<sizeof(time_t);i++) {
        serial->putc(charValue[i]);
    }
    memcpy(charValue, &config->logPointer, sizeof(uint16_t));       // latest log pointer
    for(i=0;i<sizeof(uint16_t);i++) {
        serial->putc(charValue[i]);
    }
}

/**
 * 変数に格納された設定値を返す(ascii)
 */
void printConfigVarsReadable() {

    // status flag
    char statusByte = config->statusFlags;
    serial->printf("ER FN TD OP FA FL ST IN\r\n");
    serial->printf("-----------------------\r\n");
    serial->printf(" %d  %d  %d  %d  %d  %d  %d  %d\r\n"
            , ((statusByte & 0x80) ? 1 : 0)
            , ((statusByte & 0x40) ? 1 : 0)
            , ((statusByte & 0x20) ? 1 : 0)
            , ((statusByte & 0x10) ? 1 : 0)
            , ((statusByte & 0x08) ? 1 : 0)
            , ((statusByte & 0x04) ? 1 : 0)
            , ((statusByte & 0x02) ? 1 : 0)
            , ((statusByte & 0x01) ? 1 : 0)
    );

    serial->printf("Pressure At Sea Lv : %d Pa\r\n", (uint32_t) config->pressureAtSeaLevel);
    serial->printf("Ground Altitude    : %d m\r\n", config->groundAltitude);
    serial->printf("Current Altitude   : %d m\r\n", config->currentAltitude);
    serial->printf("Deploy Parachute At: +%d m\r\n", config->deployParachuteAt);
    serial->printf("Counter Threshold  : %d times\r\n", config->counterThreshold);
    serial->printf("Altitude Threshold : +%d m\r\n", config->altitudeThreshold);
    serial->printf("Open Servo         : %d ms\r\n", (uint32_t) (config->openServoPeriod * 1000));
    serial->printf("Close Servo        : %d ms\r\n", (uint32_t) (config->closeServoPeriod * 1000));
    serial->printf("Enable Logging     : %d\r\n", config->enableLogging);
    serial->printf("Last Logged at     : %d\r\n", (uint32_t) config->logStartTime);
    serial->printf("Last Logged Address: 0x%04X\r\n", config->logPointer);
}

/**
 * 設定値をSRAMから変数にロードする
 */
void loadConfig() {

    uint32_t uint32Value;
    char *buffer = new char[CONFIG_AREA_SIZE];

    sram->read(0x0000, (char*)buffer, CONFIG_AREA_SIZE); // read 0x0000-0x0020

    /**
     * 0x0000        uint8_t  ステータスフラグ格納領域
     * 0x0001-0x0004 float    海抜0mの気圧 (hPa)
     * 0x0005-0x0006 uint16_t 地表の高度 (0-65536m)
     * 0x0007-0x0008 uint16_t 現在の高度 (0-65536m)
     * 0x0009        uint8_t  パラシュート開放高度(地表高度に加算) (0-256m)
     * 0x000A        uint8_t  状態カウンタしきい値 (0-255回)
     * 0x000B        uint8_t  状態遷移に必要な高度しきい値 (0-255回)
     * 0x000C-0x000F float    サーボ開 duty比 (0-1.0f)
     * 0x0010-0x0013 float    サーボ閉 duty比 (0-1.0f)
     * 0x0014        uint8_t  ロギング設定 (0x00:無効 0x01:有効)
     * 0x0015-0x0018 time_t   ロギング開始時刻(RTCから取得する）
     * 0x0019-0x001A uint16_t 最終ログ格納アドレス (0x0020-0x0800)
     * 0x001B-0x001F (予備)
     * 0x0020-0x0800          ログデータ格納領域
     */

    config->statusFlags        = (uint8_t) buffer[0];
    uint32Value = (buffer[4] << 24 | buffer[3] << 16 | buffer[2] << 8 | buffer[1]);
    config->pressureAtSeaLevel = *(float*)&uint32Value;
    config->groundAltitude     = (uint16_t) (buffer[6] << 8 | buffer[5]);
    config->currentAltitude    = (uint16_t) (buffer[8] << 8 | buffer[7]);
    config->deployParachuteAt  = (uint8_t) buffer[9];
    config->counterThreshold   = (uint8_t) buffer[10];
    config->altitudeThreshold  = (uint8_t) buffer[11];
    uint32Value = (buffer[15] << 24 | buffer[14] << 15 | buffer[13] << 8 | buffer[12]);
    config->openServoPeriod    = *(float*)&uint32Value;
    uint32Value = (buffer[19] << 24 | buffer[18] << 16 | buffer[17] << 8 | buffer[16]);
    config->closeServoPeriod   = *(float*)&uint32Value;;
    config->enableLogging      = (uint8_t) buffer[20];
    config->logStartTime       = (time_t) (buffer[24] << 24 | buffer[23] << 16 | buffer[22] << 8 | buffer[21]);
    config->logPointer         = (uint16_t) (buffer[26] << 8 | buffer[25]);

    delete[] buffer;
}

/**
 * 設定値を変数からSRAMに書き出す
 */
void saveConfig() {

    char charValue[sizeof(float)];

    /**
     * 0x0000        uint8_t  ステータスフラグ格納領域
     * 0x0001-0x0004 float    海抜0mの気圧 (hPa)
     * 0x0005-0x0006 uint16_t 地表の高度 (0-65536m)
     * 0x0007-0x0008 uint16_t 現在の高度 (0-65536m)
     * 0x0009        uint8_t  パラシュート開放高度(地表高度に加算) (0-256m)
     * 0x000A        uint8_t  状態カウンタしきい値 (0-255回)
     * 0x000B        uint8_t  状態遷移に必要な高度しきい値 (0-255回)
     * 0x000C-0x000F float    サーボ開 duty比 (0-1.0f)
     * 0x0010-0x0013 float    サーボ閉 duty比 (0-1.0f)
     * 0x0014        uint8_t  ロギング設定 (0x00:無効 0x01:有効)
     * 0x0015-0x0018 time_t   ロギング開始時刻(RTCから取得する）
     * 0x0019-0x001A uint16_t 最終ログ格納アドレス (0x0020-0x0800)
     * 0x001B-0x001F (予備)
     * 0x0020-0x0800          ログデータ格納領域
     */

    sram->write(0x0000, config->statusFlags);
    memcpy(charValue, &config->pressureAtSeaLevel, sizeof(float));
    sram->write(0x0001, charValue, sizeof(float));
    memcpy(charValue, &config->groundAltitude, sizeof(uint16_t));
    sram->write(0x0005, charValue, sizeof(uint16_t));
    memcpy(charValue, &config->currentAltitude, sizeof(uint16_t));
    sram->write(0x0007, charValue, sizeof(uint16_t));
    sram->write(0x0009, config->deployParachuteAt);
    sram->write(0x000A, config->counterThreshold);
    sram->write(0x000B, config->altitudeThreshold);
    memcpy(charValue, &config->openServoPeriod, sizeof(float));
    sram->write(0x000C, charValue, sizeof(float));
    memcpy(charValue, &config->closeServoPeriod, sizeof(float));
    sram->write(0x0010, charValue, sizeof(float));
    sram->write(0x0014, config->enableLogging);
    memcpy(charValue, &config->logStartTime, sizeof(time_t));
    sram->write(0x0015, charValue, sizeof(time_t));
    memcpy(charValue, &config->logPointer, sizeof(uint16_t));
    sram->write(0x0019, charValue, sizeof(uint16_t));

    // save data onto EEPROM
    sram->callHardwareStore();
}

/**
 * MEMO: 初期値設定
 * 設定値を初期化して変数とSRAMにそれぞれ保存する
 */
void resetConfig() {

    // getStandBy config with default value
    config->statusFlags         = 0x00;      // ステータスフラグ
    config->pressureAtSeaLevel  = 101448.0f; // 海抜0mの大気圧(低すぎると高度が0になるので注意)
    config->groundAltitude      = 0;        // 地表高度
    config->currentAltitude     = 0;        // 現在高度
    config->deployParachuteAt   = 20;        // パラシュート開放高度(地表高度に加算)
    config->counterThreshold    = 3;         // 状態カウンタのしきい値（この回数に達したら、その状態が発生したと判断する）
    config->altitudeThreshold   = 30;         // 現在高度が地表高度＋この値を上回っていたら飛行中とみなす
    config->openServoPeriod     = 0.041f;    // 0-1.0f // close
    config->closeServoPeriod    = 0.030f;    // 0-1.0f
    config->enableLogging       = 0x00;      // 0x01:true 0x00:false
    config->logStartTime        = 0x01020304;
    config->logPointer          = SRAM_LOG_START_AT;    // 0x0021-0x0800

    // 地表高度を設定
    setGroundAltitude();

    // SRAM に保存
//    saveConfig(); //MEMO: setGroundAltitude() で saveConfig() が呼ばれるのでコメントアウト

    // サーボの開閉値更新
    servoManager->setRange(config->closeServoPeriod, config->openServoPeriod);

}

/**
 * LOG
 */
void clearLog(uint16_t startAddress, uint16_t endAddress)
{
    uint8_t retryCount = 0;
    for (uint16_t i=startAddress; i<endAddress; i++) {
        retryCount = 0;
        while(sram->write(i, 0x00) != 0 && retryCount++ < 5) {
            sram->write(i, 0x00);
        }
    }

    // reset log pointer
    config->logPointer = SRAM_LOG_START_AT;
    sram->write(0x0019, config->logPointer);

    // save data onto EEPROM
    sram->callHardwareStore();
}

/**
 * save current data onto SRAM
 */
void writeCurrentData(uint8_t currentStatus, uint8_t currentAltitude) {
    // if overflow, reset pointer
    if(config->logPointer + CONFIG_LOG_SIZE > SRAM_MAX_SIZE) {
        config->logPointer = SRAM_LOG_START_AT;
    }
    // write current status
    sram->write(config->logPointer++, currentStatus);

    // write current altitude
    sram->write(config->logPointer++, currentAltitude);

    // save current logging pointer
    char charValue[sizeof(uint16_t)];
    memcpy(charValue, &config->logPointer, sizeof(uint16_t));
    sram->write(0x0019, charValue, sizeof(uint16_t));
}

/**
 * SENSOR
 */

// センサ開始
static uint8_t startSensor()
{
    // null チェック
    if(sensorManager == NULL) {
        return RESULT_NG;
    }

    // センサ開始
    if(sensorManager->begin() != RESULT_OK) {
        return RESULT_NG;
    }

    // センサ値更新処理開始
    if(sensorTicker) {
        sensorTicker->attach(&updateSensor, SENSOR_UPDATE_FREQ); // only :void can be attached
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


/**
 * センサの現在値を返す(hex)
 */
void printSensorValues() {
    if(sensorManager != NULL) {

        uint8_t i;

        // センサ値更新
        sensorManager->updateForced();

        // バイナリデータとして書き出す
        char charValue[sizeof(float)];

        memcpy(charValue, &config->pressureAtSeaLevel, sizeof(float)); // Pressure at Sea Level (0m)
        for(i=0;i<sizeof(float);i++) {
            serial->putc(charValue[i]);
        }

        memcpy(charValue, &sensorManager->currentPressure, sizeof(float)); // Current Pressure
        for(i=0;i<sizeof(float);i++) {
            serial->putc(charValue[i]);
        }

        memcpy(charValue, &config->currentAltitude, sizeof(uint16_t)); // Current Altitude
        for(i=0;i<sizeof(uint16_t);i++) {
            serial->putc(charValue[i]);
        }

        memcpy(charValue, &sensorManager->currentTemperature, sizeof(float)); // Current Temperature
        for(i=0;i<sizeof(float);i++) {
            serial->putc(charValue[i]);
        }
    }
}

/**
 * センサの現在値を返す(ascii)
 */
void printSensorValuesReadable() {
    if(sensorManager != NULL) {

        sensorManager->updateForced();

        serial->printf("SeaLev P: %d\r\n", (uint32_t)(config->pressureAtSeaLevel));     // Pa: 100Pa=1hPa
        serial->printf("CurrentP: %d\r\n", (uint32_t)(sensorManager->currentPressure)); // Pa: 100Pa=1hPa
        serial->printf("Gnd Alt : %u\r\n", config->groundAltitude);
        serial->printf("Cur Alt : %u\r\n", config->currentAltitude);
        serial->printf("TMP(x10): %d\r\n", (uint16_t)x10(sensorManager->currentTemperature));
    }
}

/**
 * BUTTONS
 */

// ボタン押下に応じてサーボの開閉を行う
static void changeServoState()
{
//    DEBUG_PRINT("changeServoState()\r\n");
    servoManager->flipState();
}

// 地上の高度をセットする
static void setGroundAltitude()
{
//    DEBUG_PRINT("setGroundAltitude()\r\n");
    // 地表高度設定
    sensorManager->calculateGroundAltitude();

    // SRAM に保存
    saveConfig();
}

// デバイスを開始する
static void startDevice()
{
//    DEBUG_PRINT("startDevice()\r\n");
    updateStatus(config->statusFlags | INIT);
}

// プローブを開始する
static void resetDevice()
{
    resetConfig();              // 設定初期化
    servoManager->moveRight();  // サーボ開放
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
