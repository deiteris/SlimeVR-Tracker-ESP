/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2022 TheDevMinerTV

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "SensorManager.h"
#include <i2cscan.h>
#include "network/network.h"
#include "bno055sensor.h"
#include "bno080sensor.h"
#include "mpu9250sensor.h"
#include "mpu6050sensor.h"
#include "bmi160sensor.h"
#include "icm20948sensor.h"
#include "ErroneousSensor.h"

namespace SlimeVR
{
    namespace Sensors
    {
        void SensorManager::setup()
        {
#if IMUS & IMU_BNO080 || IMUS & IMU_BNO085 || IMUS & IMU_BNO086
            scanAddresses.insert(0x4A);
            scanAddresses.insert(0x4B);
#endif
#if IMUS & IMU_BNO055
            scanAddresses.insert(0x28);
            scanAddresses.insert(0x29);
#endif
#if IMUS & IMU_MPU9250 || IMUS & IMU_BMI160 || IMUS & IMU_MPU6500 || IMUS & IMU_MPU6050 || IMUS & IMU_ICM20948
            scanAddresses.insert(0x68);
            scanAddresses.insert(0x69);
#endif
            std::vector<I2CSCAN::PortInfo> ports = I2CSCAN::scani2cports(scanAddresses);
            
            for (size_t i = 0; i < ports.size(); i++) {
                m_Logger.trace("IMU found at bus %d:%d, address 0x%02X", ports[i].sda, ports[i].scl, ports[i].addr);

                float rotation = 0;
                uint8_t intPin = 255;
                for (auto &imuInfo : imuInfos) {
                    if (ports[i].scl == imuInfo.sclPin && ports[i].addr == imuInfo.address) {
                        rotation = imuInfo.rotation;
                        intPin = imuInfo.intPin;
                        break;
                    }
                }

#if IMUS & IMU_BNO080 || IMUS & IMU_BNO085 || IMUS & IMU_BNO086
                sensors.emplace_back(std::make_unique<BNO080Sensor>(i, IMU_BNO080, ports[i].addr, rotation, intPin));
#endif
#if IMUS & IMU_BNO055
                sensors.emplace_back(std::make_unique<BNO055Sensor>(i, IMU_BNO055, ports[i].addr, rotation));
#endif
#if IMUS & IMU_MPU9250
                sensors.emplace_back(std::make_unique<MPU9250Sensor>(i, IMU_MPU9250, ports[i].addr, rotation));
#endif
#if IMUS & IMU_BMI160
                sensors.emplace_back(std::make_unique<BMI160Sensor>(i, IMU_BMI160, ports[i].addr, rotation));
#endif
#if IMUS & IMU_MPU6500 || IMUS & IMU_MPU6050
                sensors.emplace_back(std::make_unique<MPU6050Sensor>(i, IMU_MPU6050, ports[i].addr, rotation));
#endif
#if IMUS & IMU_ICM20948
                sensors.emplace_back(std::make_unique<ICM20948Sensor>(i, IMU_ICM20948, ports[i].addr, rotation));
#endif
            }
            for (auto &sensor : sensors)
                sensor->motionSetup();
        }

        void SensorManager::postSetup()
        {
            for (auto &sensor : sensors) {
                sensor->postSetup();
            }
        }

        void SensorManager::update()
        {
            // Gather IMU data
            for (auto &sensor : sensors) {
                sensor->motionLoop();
            }

            if (!ServerConnection::isConnected())
            {
                return;
            }

            // Send updates
            for (auto &sensor : sensors) {
                sensor->sendData();
            }
        }
    }
}
