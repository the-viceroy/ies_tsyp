#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include "../sensor.h"

// Default I2C address for INA219 sensor (7-bit address)
#define INA219_ADDRESS 0x40

/**
 * INA219 High-Side Current/Voltage/Power Monitor Sensor Driver
 * 
 * This class provides an interface to the Texas Instruments INA219 sensor
 * over I2C communication on Raspberry Pi or similar Linux systems.
 * 
 * Features:
 * - Measures bus voltage (0-26V range, configurable)
 * - Measures current through a shunt resistor
 * - Calculates power consumption
 * - Integrates with the project's Sensor base class
 * 
 * Typical Use Case: Solar panel or battery monitoring in power systems
 */
class INA219 : public Sensor {
public:
    /**
     * Constructor - Initialize INA219 sensor object
     * 
     * @param i2cDev  Path to I2C device (e.g., "/dev/i2c-1" for Raspberry Pi)
     * @param address I2C slave address of the INA219 chip (default: 0x40)
     * 
     * Note: Does not open I2C connection yet - call begin() to initialize
     */
    INA219(const char* i2cDev = "/dev/i2c-1", uint8_t address = INA219_ADDRESS);

    /**
     * Destructor - Clean up resources
     * Automatically closes I2C file descriptor if still open
     */
    ~INA219();

    /**
     * Initialize the sensor - must be called before reading data
     * 
     * Opens I2C device, sets slave address, and configures the INA219 chip
     * with calibration values for voltage/current measurement
     * 
     * @return true if initialization successful, false on error
     */
    bool begin();

    /**
     * Read the bus voltage (voltage at V+ pin)
     * 
     * @return Voltage in volts (V), or -1.0 on error
     */
    float getBusVoltage_V();

    /**
     * Read the current flowing through the shunt resistor
     * 
     * @return Current in amps (A), or -1000.0 on error
     * Note: Can be negative if current flows in reverse direction
     */
    float getCurrent_A();

    /**
     * Calculate power consumption (V × I)
     * 
     * @return Power in watts (W), or -1.0 on error
     */
    float getPower_W();

protected:
    /**
     * Override of Sensor base class method
     * Reads all sensor data and packs it into a byte array for telemetry
     * 
     * Data format (7 bytes total, little-endian):
     *   Bytes 0-1: Voltage in millivolts (uint16_t)
     *   Bytes 2-3: Current in milliamps (int16_t, signed)
     *   Bytes 4-5: Power in milliwatts (uint16_t)
     *   Byte 6:    Status/error code (uint8_t)
     * 
     * @return Vector of bytes containing packed sensor data
     */
    std::vector<uint8_t> read() override;

private:
    /**
     * INA219 internal register addresses
     * These match the datasheet register map
     */
    enum class Register : uint8_t {
        Config = 0x00,        // Configuration register (voltage range, ADC settings)
        ShuntVoltage = 0x01,  // Shunt voltage measurement register
        BusVoltage = 0x02,    // Bus voltage measurement register
        Power = 0x03,         // Power calculation register
        Current = 0x04,       // Current measurement register
        Calibration = 0x05    // Calibration register (for current/power scaling)
    };

    // Hardware interface variables
    int _i2c_fd;              // Linux file descriptor for I2C device
    std::string _devPath;     // Path to I2C device file (e.g., "/dev/i2c-1")
    uint8_t _addr;            // I2C slave address of this INA219 chip
    uint8_t _status;          // Status code: 0=OK, other values indicate errors

    /**
     * Write a 16-bit value to an INA219 register
     * 
     * @param reg   Register address to write to
     * @param value 16-bit value to write (sent as big-endian over I2C)
     * @return true if write successful, false on error
     */
    bool i2cWriteReg16(uint8_t reg, uint16_t value);

    /**
     * Read a 16-bit value from an INA219 register
     * 
     * @param reg   Register address to read from
     * @param value Reference to store the read value (big-endian from I2C)
     * @return true if read successful, false on error
     */
    bool i2cReadReg16(uint8_t reg, uint16_t &value);

    /**
     * Configure the INA219 calibration and settings
     * Sets up voltage range, ADC resolution, and calibration for current sensing
     * 
     * Note: Calibration values depend on your shunt resistor value
     * Default setup: 0.1Ω shunt, ~2A max current, 16V bus range
     * 
     * @return true if calibration successful, false on error
     */
    bool setCalibration();
};
