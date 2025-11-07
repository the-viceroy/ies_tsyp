#pragma once  // Prevent multiple inclusions of this header file

#include <cstdint>     // Standard integer types (uint8_t, uint16_t, int16_t, etc.)
#include <vector>      // std::vector for Sensor::read payload
#include "../sensor.h" // Project Sensor base class interface

// I2C 7-bit address for TI gauge
#define GAUGE_ADDRESS 0x55

// BQ34Z100: Monitoring battery 
class BQ34Z100 : public Sensor {
public:
    // Constructor: initialize with I2C device path
    BQ34Z100(const char* i2c_device = "/dev/i2c-1");
    
    // Destructor: close I2C device
    ~BQ34Z100();

    //  getters
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

    int _i2c_fd; // I2C file descriptor
    uint8_t _status; // Status flag for error tracking

    uint32_t read(Command command, const uint8_t length);
};
