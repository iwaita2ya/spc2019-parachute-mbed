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

#define CONFIG_MEMORY_AREA_SIZE 30
#define SENSOR_UPDATE_FREQ 0.1

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
const int serialBufferSize = 255;

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
char txLineBuffer[80];
char rxLineBuffer[80];

#ifdef DEBUG
#define DEBUG_PRINT(x)  serial->printf(x)
#else
#define DEBUG_PRINT(x)
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

/**
 * LED
 */
DigitalOut *led; // LED

/**
 * Jumper
 */
DigitalIn *enableSerial;

/**
 * SRAM
 * 0x0000        uint8_t  ステータスフラグ格納領域
 * 0x0001-0x0004 float    海抜0mの気圧 (hPa)
 * 0x0005-0x0008 float    地上の気圧 (hPa)
 * 0x0009        uint8_t  上空判定 (0-255m)
 * 0x000A        uint8_t  開放判定 (0-255m)
 * 0x000B-0x000E float    サーボ周期 Futaba:20ms(0.020f)
 * 0x000F-0x0012 float    サーボ開 duty比 (0-1.0f)
 * 0x0013-0x0016 float    サーボ閉 duty比 (0-1.0f)
 * 0x0017        uint8_t  ロギング設定 (0x00:無効 0x01:有効)
 * 0x0018-0x001B time_t   ロギング開始時刻(RTCから取得する）
 * 0x001C-0x001D uint16_t 最終ログ格納アドレス (0x0020-0x0800)
 * 0x001E-0x001F          (予備)
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
void getStatus();           // get config data from SRAM
void getStatusReadable();   // get config data from SRAM (in readable format)
void updateStatus(uint8_t newStatus);
void getConfig();           // get config data from SRAM
void getConfigReadable();   // get config data from SRAM (in readable format)
void resetConfig();         // reset config with default value;
void loadConfig();          // load config data from SRAM
void dumpMemory();          // dump all data in SRAM
void dumpMemoryReadable();  // dump all data in SRAM (in readable format)
void clearLog(uint16_t startAddress);   // clear logged data, not config

// ----- SERVO -----
static void changeServoState();

// ----- ALTIMETER -----
static void setAltitude();

// ----- Sensor -----
static uint8_t startSensor();
static void updateSensor();
static uint8_t stopSensor();
void getSensorValues();
void getSensorValuesReadable();

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
    enableSerial = new DigitalIn(P1_15);
    enableSerial->mode(PullUp);

    // getStandBy serial baud rate
    serial = new RawSerial(P0_19, P0_18); // tx, rx
    serial->baud(115200); // default:9600bps
    // set interrupts for Tx/Rx
    serial->attach(&interruptRx, Serial::RxIrq);
    serial->attach(&interruptTx, Serial::TxIrq);

    // getStandBy SRAM
    sram = new SerialSRAM(P0_5, P0_4, P0_21); // sda, scl, hs, A2=0, A1=0
    config = new SystemParameters();
    loadConfig(); //TODO: 値の妥当性を検証する

    // getStandBy ServoManager
    servoManager = new ServoManager(P0_22);
    servoManager->setPeriod(config->servoPeriod);
    servoManager->setRange(config->closeServoDuty, config->openServoDuty); // minValue, maxValue
    servoManager->init();
    servoManager->moveRight(); // open

    // getStandBy SensorManager (and Ticker)
    sensorTicker  = new Ticker();
    sensorManager = new SensorManager(P0_5, P0_4, 0xD6, 0x3C, config); // sda, scl, agAddr, mAddr

    // Set Altitude Button
    setAltitudePin = new InterruptIn(P0_20); // 地表高度設定ボタン
    setAltitudePin->mode(PullUp);
    setAltitudePin->fall(&setAltitude);

    // Servo Open/Close Buttons
    servoControlPin = new InterruptIn(P1_19);// サーボ操作ボタン
    servoControlPin->mode(PullUp);
    servoControlPin->fall(&changeServoState);

    // Status LED
    led = new DigitalOut(P0_7);

    /**
     * Main Loop
     */
    while(shouldLoop == 1) {

        /**
         * ステータスに応じて処理を分岐
         * ステータスフラグはINITから順番に立てられる（クリアされない）ので、
         * FINISHから順に判定する
         * INIT       = 0x01,
         * STAND_BY   = 0x02,
         * FLYING     = 0x04,
         * FALLING    = 0x08,
         * OPEN_PARA  = 0x10,
         * TOUCH_DOWN = 0x20,
         * FINISH     = 0x40,
         * ERROR      = 0xFF  // エラー発生
         */
        if(config->statusFlags & TOUCH_DOWN) { // 着地
            // システム停止
            if(stopSensor() == RESULT_OK) {
                updateStatus(config->statusFlags | FINISH);
            }
        }
        else if(config->statusFlags & OPEN_PARA) { // パラシュート開放
            // パラシュート開放
            servoManager->moveLeft();

            // 現在高度が地上高度と等しくなったら TOUCH_DOWN に遷移
            if(sensorManager->isTouchDown()) {
                updateStatus(config->statusFlags | TOUCH_DOWN);
            }
        }
        else if(config->statusFlags & FALLING) { // 落下中

            // 開放高度に達したら OPEN_PARA に遷移
            if(sensorManager->isOkToDeployParachute()) {
                updateStatus(config->statusFlags | OPEN_PARA);
            }
        }
        else if(config->statusFlags & FLYING) { // 飛行中
            //TODO: ロギング開始

            //高度が減少に転じたら FALLING に遷移
            if(sensorManager->isFalling()) {
                updateStatus(config->statusFlags | FALLING);
            }
        }
        else if(config->statusFlags & STAND_BY) { //
            // 規定高度に達したら FLYING に遷移
            if(sensorManager->isFlying()) {
                updateStatus(config->statusFlags | FLYING);
            }
        }
        else if(config->statusFlags & INIT) {
            // センサ開始したら STAND_BY に遷移
            if(startSensor() == RESULT_OK) {
                updateStatus(config->statusFlags | STAND_BY);
            }
        }

        /**
         * 受信データの最初の1バイトがコマンドバイト(CB)
         * 更新系のコマンドについては、CBの後に更新値が1−20バイト続く
         * 0x00 ステータス取得
         * 0x10 ステータス取得 (human readable)
         * 0x20 ステータス更新
         * 0x30 ステータスクリア
         * 0x40 設定取得
         * 0x50 設定取得 (human readable)
         * 0x60 設定更新
         * 0x70 設定初期化
         * 0x80 ログ取得
         * 0x90 ログクリア
         * 0xA0 メモリダンプ
         * 0xB0 メモリダンプ (human readable)
         * 0xC0 センサ値取得
         * 0xD0 センサ値取得 (human readable)
         * 0xE0 サーボ開閉
         */
        if(enableSerial->read() == ENABLE_SERIAL || config->statusFlags & FINISH) {

            // data received and not read yet?
            if (rxInPointer != rxOutPointer) {

                readLine();
                char commandByte = rxLineBuffer[0];

                switch (commandByte) {
                    case 0x00: // ステータス取得
                        getStatus();
                        break;
                    case 0x10: // ステータス取得（フォーマット済）
                        getStatusReadable();
                        break;
                    case 0x20: // ステータス更新
                        updateStatus(rxLineBuffer[1]);
                        break;
                    case 0x30: // ステータス初期化
                        updateStatus(0x00);
                        break;
                    case 0x40: // 設定取得
                        getConfig();
                        break;
                    case 0x50: // 設定取得（フォーマット済）
                        getConfigReadable();
                        break;
                    case 0x60: //TODO: 設定更新
                        break;
                    case 0x70: // 設定初期化
                        resetConfig();
                        break;
                    case 0x80: //TODO: ログ取得
                        break;
                    case 0x90: //TODO: ログ初期化
                        break;
                    case 0xA0: // メモリダンプ
                        dumpMemory();
                        break;
                    case 0xB0: // メモリダンプ（フォーマット済）
                        dumpMemoryReadable();
                        break;
                    case 0xC0: //TODO: センサ値取得
                        getSensorValues();
                        break;
                    case 0xD0: // センサ値取得（フォーマット済）
                        getSensorValuesReadable();
                        break;
                    case 0xE0: // サーボ開閉
                        changeServoState();
                        break;
                    case 0xF0: // 地表高度設定
                        sensorManager->calculateGroundAltitude();
                        break;
                    default:
                        break;
                }
            }
        }

        // blink LED
        for (uint8_t loop=0; loop <= config->statusFlags; loop++) {
            led->write(!(led->read())); // reverse value
            wait(0.1);
        }
        wait(0.5);
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
 * Serial
 */

// Copy tx line buffer to large tx buffer for tx interrupt routine
void sendLine() {

    // null check
    if(serial == NULL) {
        return;
    }

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

    // null check
    if(serial == NULL) {
        return;
    }

    int i;
    i = 0;
    // Start Critical Section - don't interrupt while changing global buffer variables
    NVIC_DisableIRQ(UART_IRQn);

    // Loop reading rx buffer characters until end of line character
    while ((i==0) || (rxLineBuffer[i-1] != '\r')) { // '\r' = 0x0d
        rxLineBuffer[i] = rxBuffer[rxOutPointer];
        i++;
        rxOutPointer = (rxOutPointer + 1) % serialBufferSize;
    }

    // End Critical Section
    NVIC_EnableIRQ(UART_IRQn);
    rxLineBuffer[i-1] = 0;
}


// Interrupt Routine to read in data from serial port
void interruptRx() {

    // null check
    if(serial == NULL) {
        return;
    }

    // Loop just in case more than one character is in UART's receive FIFO buffer
    // Stop if buffer full
    while ((serial->readable()) && (((rxInPointer + 1) % serialBufferSize) != rxOutPointer)) {
        rxBuffer[rxInPointer] = serial->getc();

        // Uncomment to Echo to USB serial to watch data flow
        //serial->putc(rxBuffer[rxInPointer]); // echo back
        rxInPointer = (rxInPointer + 1) % serialBufferSize;
    }
}


// Interrupt Routine to write out data to serial port
void interruptTx() {

    // null check
    if(serial == NULL) {
        return;
    }
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

void dumpMemory() {

    static const uint8_t bufferLength = 16;
    char *buffer = new char[bufferLength];

    for(uint16_t address=0x0000; address<0x0800; address+=bufferLength) {

        // Seq. Read
        sram->read(address, buffer, bufferLength);

        for(uint8_t i=0; i<bufferLength; i++) {
            serial->putc(buffer[i]);
        }
    }

    delete[] buffer;
}

/**
 * dump all data stored in 47L16 (2048byte)
 */
void dumpMemoryReadable() {

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

void getStatus() {

    char statusByte = 0x00;

    // Read single byte at 0x0000
    sram->read(0x0000, &statusByte);

    serial->putc(statusByte);
}

void getStatusReadable() {

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
 * 設定値を返す
 */
void getConfig() {

    char *buffer = new char[0x20];

    // Seq. Read
    sram->read(0x0000, buffer, 0x20);

    for(uint8_t i=0; i<0x20; i++) {
        serial->putc(buffer[i]);
    }

    delete[] buffer;
}

/**
 * 設定値をフォーマットして返す
 */
void getConfigReadable() {

    static const uint8_t bufferLength = 16;
    char *buffer = new char[bufferLength];

    // format in hex
    serial->printf("ADDR 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f\r\n");
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

    // print each value

    // status flag
    getStatusReadable();

    // 海抜0mの大気圧
    sprintf(txLineBuffer, "Pressure: %u Pa\r\n", (uint32_t) config->pressureAtSeaLevel); //FIXME: 変な値が表示される
    sendLine();

    sprintf(txLineBuffer, "Ground Altitude: %d m\r\n", config->groundAltitude);
    sendLine();

    sprintf(txLineBuffer, "Counter Threshold: %d\r\n", config->counterThreshold);
    sendLine();

    sprintf(txLineBuffer, "Altitude Threshold: %d m\r\n", config->altitudeThreshold);
    sendLine();

    sprintf(txLineBuffer, "Deploy Parachute At: %d m\r\n", config->deployParachuteAt);
    sendLine();

    sprintf(txLineBuffer, "Servo Period: %d m\r\n", (uint32_t) config->servoPeriod * 1000);
    sendLine();

    sprintf(txLineBuffer, "Open Servo Duty: %d m\r\n", (uint32_t) config->servoPeriod * 1000);
    sendLine();

    sprintf(txLineBuffer, "Close Servo Duty: %d m\r\n", (uint32_t) config->closeServoDuty * 1000);
    sendLine();

    sprintf(txLineBuffer, "Enable Logging: %d\r\n", config->enableLogging);
    sendLine();

    sprintf(txLineBuffer, "Start Logging at: %ld\r\n", config->logStartTime);
    sendLine();

    sprintf(txLineBuffer, "LastLog Address: 0x%04X\r\n", config->lastLogAddress);
    sendLine();

    delete[] buffer;
}

void resetConfig() {

    // getStandBy config with default value
    config->statusFlags         = 0x00;      // ステータスフラグ
    config->pressureAtSeaLevel  = 100000.0f; // 海抜0mの大気圧
    config->groundAltitude      = 34;        // 地表高度
    config->counterThreshold    = 5;         // 状態カウンタのしきい値（この値に達したら、その状態が発生したと判断する）
    config->altitudeThreshold   = 2;         // 状態遷移に必要な高度しきい値
    config->deployParachuteAt   = 30;        // パラシュート開放高度(m)(地表高度に加算する)
    config->servoPeriod         = 0.020f;    // 20 ms
    config->openServoDuty       = 0.037f;    // 0-1.0f
    config->closeServoDuty      = 0.03f;     // 0-1.0f
    config->enableLogging       = 0x01;      // 0x01:true 0x00:false
    config->logStartTime        = 0x01020304;
    config->lastLogAddress      = 0x0020;    // 0x0020-0x0800

    // save onto SRAM
    char charValue[sizeof(float)];
    sram->write(0x0000, config->statusFlags);
    memcpy(charValue, &config->pressureAtSeaLevel, sizeof(float));
    sram->write(0x0001, charValue, sizeof(float));
    sram->write(0x0005, config->groundAltitude);
    sram->write(0x0006, config->counterThreshold);
    sram->write(0x0007, config->altitudeThreshold);
    sram->write(0x0008, config->deployParachuteAt);
    memcpy(charValue, &config->servoPeriod, sizeof(float));
    sram->write(0x0009, charValue, sizeof(float));
    memcpy(charValue, &config->openServoDuty, sizeof(float));
    sram->write(0x000D, charValue, sizeof(float));
    memcpy(charValue, &config->closeServoDuty, sizeof(float));
    sram->write(0x0011, charValue, sizeof(float));
    sram->write(0x0015, config->enableLogging);
    memcpy(charValue, &config->logStartTime, sizeof(time_t));
    sram->write(0x0016, charValue, sizeof(time_t));
    memcpy(charValue, &config->enableLogging, sizeof(uint16_t));
    sram->write(0x001B, charValue, sizeof(uint16_t));
}

void loadConfig() {

    char *buffer = new char[0x20];
    sram->read(0x0000, buffer, 0x20); // update 0x0000-0x0019

    config->statusFlags         = (uint8_t) buffer[0];
    config->pressureAtSeaLevel  = (float) (buffer[1] << 24 | buffer[2] << 16 | buffer[3] << 8 | buffer[4]);
    config->groundAltitude      = (uint8_t) buffer[5];
    config->counterThreshold    = (uint8_t) buffer[6];
    config->altitudeThreshold   = (uint8_t) buffer[7];
    config->deployParachuteAt   = (uint8_t) buffer[8];
    config->servoPeriod         = (float) (buffer[9] << 24 | buffer[10] << 16 | buffer[11] << 8 | buffer[12]);
    config->openServoDuty       = (float) (buffer[13] << 24 | buffer[14] << 16 | buffer[15] << 8 | buffer[16]);
    config->closeServoDuty      = (float) (buffer[17] << 24 | buffer[18] << 16 | buffer[19] << 8 | buffer[20]);
    config->enableLogging       = (uint8_t) buffer[21];
    config->logStartTime        = (time_t) (buffer[22] << 24 | buffer[23] << 16 | buffer[24] << 8 | buffer[25]);
    config->lastLogAddress      = (uint16_t) (buffer[26] << 8 | buffer[27]);
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

void getSensorValues() {
    if(sensorManager != NULL) {

        sensorManager->updateForced();

        //TODO: 書く
    }
}

void getSensorValuesReadable() {
    if(sensorManager != NULL) {

        sensorManager->updateForced();

        serial->printf("Pressure: %d\r\n", (uint32_t)(sensorManager->currentPressure)); // Pa: 100Pa=1hPa
        serial->printf("ALT(x10): %d\r\n", (uint16_t)x10(sensorManager->currentAltitude));
        serial->printf("TMP(x10): %d\r\n", (uint16_t)x10(sensorManager->currentTemperature));
    }
}

/**
 * BUTTONS
 */

// ボタン押下に応じてサーボの開閉を行う
static void changeServoState()
{
    DEBUG_PRINT("\nchangeServoState()\r");

    if(servoManager != NULL)
    {
        if (config->statusFlags & 0x01) {

            // release (1->0)
            servoManager->moveLeft();

            // clear flag
            updateStatus(config->statusFlags & 0xFE);
        } else {

            // lock (0->1)
            servoManager->moveRight();

            // set flag
            updateStatus(config->statusFlags | 0x01);
        }
    }
}

// 地上の高度をセットする
static void setAltitude()
{
    DEBUG_PRINT("\nsetAltitude()\r");

    if(sensorManager != NULL)
    {
        // 地表高度設定
        sensorManager->calculateGroundAltitude();
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
