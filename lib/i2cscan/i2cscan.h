#ifndef _I2CSCAN_H_
#define _I2CSCAN_H_ 1

#include <set>
#include <Arduino.h>
#include <Wire.h>

namespace I2CSCAN {
    struct PortInfo {
        uint8_t addr;
        uint8_t sda;
        uint8_t scl;
    };

    void scani2cports();
    std::vector<PortInfo> scani2cports(std::set<uint8_t> addresses);
    bool checkI2C(uint8_t i, uint8_t j);
    bool checkI2C(uint8_t sdaPin, uint8_t sclPin, uint8_t address);
    bool isI2CExist(uint8_t addr);
    uint8_t pickDevice(uint8_t addr1, uint8_t addr2, bool scanIfNotFound);
    int clearBus(uint8_t SDA, uint8_t SCL);
}

#endif // _I2CSCAN_H_