//
// Created by Tatsuya Iwai (GreySound) on 4/8/19.
//

#ifndef GS_SERIAL_SRAM_H
#define GS_SERIAL_SRAM_H

#include <mbed.h>

#define SRAM_BUFFER_SIZE 8

namespace greysound {
    
class SerialSRAM {

private:
    uint8_t SRAM_REGISTER_READ;
    uint8_t SRAM_REGISTER_WRITE;
    uint8_t CONTROL_REGISTER_READ;
    uint8_t CONTROL_REGISTER_WRITE;
    I2C i2c;
    DigitalOut *hardwareStore;

public:

    // Constructor
    SerialSRAM(PinName sda, PinName scl, PinName hs, const uint8_t A2=0, const uint8_t A1=0);
    virtual ~SerialSRAM() {
        delete hardwareStore;
    }

    /**
     * SRAM Read Operations
     */
    uint8_t read(char *buffer); // Read from current address (1 Byte)
    uint8_t read(const uint16_t address, char *buffer); // Random update (1 Byte)
    uint8_t read(const uint16_t address, char *buffer, const uint16_t size); // Seq. update w/ address (Multiple Bytes)

    /**
     * SRAM Write Operations
     */
    uint8_t write(const uint16_t address, const uint8_t data); // Byte write (1 Byte)
    uint8_t write(const uint16_t address, const char *data, const uint16_t size); // Seq. write
    //TODO: メモリクリア用のメソッドも用意する

    /**
     * Control Register Operations
     */
    // Write (Control Register)
    uint8_t readControlRegister(char *buffer);
    uint8_t writeControlRegister(const uint8_t address, const uint8_t value);
    uint8_t getAutoStore();
    void setAutoStore(const uint8_t value); //MEMO: 他のメソッドに合わせて uint8_t を返却するべき？

    /**
     * Hardware Store
     */
    uint8_t callHardwareStore();

};

}
#endif //GS_SERIAL_SRAM_H
