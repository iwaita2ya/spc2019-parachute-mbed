//
// Created by iwait on 4/8/19.
//
#include <stdint.h>
#include "SerialSRAM.h"

namespace greysound {

/**
 * Constructor
 * @param sda I2C SDA
 * @param scl I2C SCL
 * @param hs Hardware Store (Manually Initiate Store/Recall)
 * @param A2
 * @param A1
 */
SerialSRAM::SerialSRAM(PinName sda, PinName scl, PinName hs, const uint8_t A2, const uint8_t A1) : i2c (sda, scl) {
    //build mask
    uint8_t mask = (A2 << 3) | (A1 << 2);

    //save registers addresses
    this->SRAM_REGISTER_WRITE = (uint8_t) 0xA0 | mask;
    this->SRAM_REGISTER_READ  = (uint8_t) 0xA1 | mask;
    this->CONTROL_REGISTER_WRITE = (uint8_t) 0x30 | mask;
    this->CONTROL_REGISTER_READ  = (uint8_t) 0x31 | mask;

    // init hs pin
    this->hardwareStore = new DigitalOut(hs);
    this->hardwareStore->write(0); // hs=0

    //TODO: Set I2C bus speed as 1Mhz (default: 100KHz)
//    i2c.frequency(1000000);
}

/**
 * Read at current address
 * @param buffer
 * @return
 */
    uint8_t SerialSRAM::read(char *buffer) {

        static uint8_t result;

        // read data
        result = i2c.read(this->SRAM_REGISTER_READ, buffer, 1);

        return result;
    }

/**
 * Random Read
 * @param address
 * @param buffer
 * @return
 */
    uint8_t SerialSRAM::read(const uint16_t address, char *buffer) {

        static char array[2];
        static uint8_t result;
        array[0] = address >> 8;
        array[1] = address & 0xFF;

        // move address pointer
        result = i2c.write(this->SRAM_REGISTER_WRITE, array, sizeof(array), true);

        //TODO: write に失敗したときの処理を記述する

        // read data
        result = i2c.read(this->SRAM_REGISTER_READ, buffer, 1);

        return result;
    }

/**
 * Seq. Read w/ address
 * @param address
 * @param buffer
 * @param size
 * @return
 */
    uint8_t SerialSRAM::read(const uint16_t address, char *buffer, const uint16_t size) {

        static char array[2];
        static uint8_t result;
        array[0] = address >> 8;
        array[1] = address & 0xFF;

        // move address pointer
        result = i2c.write(this->SRAM_REGISTER_WRITE, array, sizeof(array), true);

        //TODO: write に失敗したときの処理を記述する

        // read data
        result = i2c.read(this->SRAM_REGISTER_READ, buffer, size);

        return result;
    }

/**
 * Single Byte Write
 * @param address
 * @param data
 * @return
 */
    uint8_t SerialSRAM::write(const uint16_t address, const uint8_t data) {

        static uint8_t result = 0x00;
        static char array[3];
        array[0] = address >> 8;
        array[1] = address & 0xFF;
        array[2] = data;

        result = i2c.write(this->SRAM_REGISTER_WRITE, array, sizeof(array));

        return result;
    }

/**
 * Seq. Bytes Write w/ address
 * @param address
 * @param data
 * @param size
 * @return
 */
    uint8_t SerialSRAM::write(const uint16_t address, const char *data, const uint16_t size) {

//    char *buffer = new char[(size+2)];
        char *buffer = new char[size+2];

        // set address
        buffer[0] = address >> 8;
        buffer[1] = address & 0xFF;
        memcpy(&buffer[2], data, size);

        // write
        uint8_t result = i2c.write(this->SRAM_REGISTER_WRITE, buffer, size+2);

        delete[] buffer;

        return result;
    }

/**
 * Read Control Register
 * @return
 */
    uint8_t SerialSRAM::readControlRegister(char *buffer) {
        return i2c.read(this->CONTROL_REGISTER_READ, buffer, 1);
    }

/**
 * Write Control Register (either STATUS or COMMAND)
 * @param address
 * @param value
 * @return
 */
    uint8_t SerialSRAM::writeControlRegister(const uint8_t address, const uint8_t value) {

        static uint8_t result = 0x00;
        static char array[2];
        array[0] = address;
        array[1] = value;

        result = i2c.write(this->CONTROL_REGISTER_WRITE, array, sizeof(array));

        // STATUSレジスタ書き込みサイクル時間 Twc(=1ms) 待つ
        wait_ms(1);

        return result;
    }

    uint8_t SerialSRAM::getAutoStore() {
        static char buffer = 0x00;
        this->readControlRegister(&buffer);

        return (buffer & 0x02) >> 1;
    }

/**
 * Set AutoStore Value (ASE)
 * @param value true:enable, false:disable
 */
    void SerialSRAM::setAutoStore(const uint8_t value) {
        static char buffer = 0x00;

        // 現在の値を読み出す
        this->readControlRegister(&buffer);

        // ASE の値を修正
        if(value != 0) {
            // enable ASE (Bit1=1)
            buffer |= 0x02;
        } else {
            // disable ASE (Bit1=0)
            buffer &= 0xFD;
        }

        // レジスタ更新
        this->writeControlRegister(0x00, buffer);
    }
    
}
