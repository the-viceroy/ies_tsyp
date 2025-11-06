#pragma once  // Prevent multiple inclusions of this header file

#include <mbed.h>      // Mbed OS framework for embedded systems (I2C, GPIO, etc.)
#include <cstdint>     // Standard integer types (uint8_t, uint16_t, int16_t, etc.)
#include <vector>      // std::vector for Sensor::read payload
#include "../sensor.h" // Project Sensor base class interface

// I2C 8-bit address for TI gauge on mbed (7-bit 0x55 << 1 = 0xAA)
#define GAUGE_ADDRESS 0xAA

// BQ34Z100: Monitoring battery 
class BQ34Z100 : public Sensor {
public:
    // Constructor: initialize with I2C bus and clock
    BQ34Z100(I2C &i2c, int hz = 400000);

    // Basic monitoring getters
    uint8_t getSOC();        // % State of Charge
    uint16_t getVoltage();   // mV
    int16_t getCurrent();    // mA (signed, charge/discharge)
    double getTemperature(); // °C

protected:

    std::vector<uint8_t> read() override;

private:
    // Minimal register map used for monitoring
    enum class Command : uint8_t {
        StateOfCharge = 0x02,
        Voltage = 0x08,
        Temperature = 0x0C,
        Current = 0x10,
    };

    I2C &_i2c; // I2C bus reference
    
    uint32_t read(Command command, const uint8_t length);
};
