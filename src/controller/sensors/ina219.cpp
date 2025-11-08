#include "ina219.h"
#include <fcntl.h>        // For open() function
#include <sys/ioctl.h>    // For ioctl() to configure I2C
#include <linux/i2c-dev.h> // Linux I2C device definitions
#include <unistd.h>       // For close(), read(), write()
#include <cerrno>         // For errno error codes
#include <cstring>        // For strerror() to get error messages
#include <cmath>          // For std::lround() rounding function
#include <iostream>       // For std::cerr error logging

//=============================================================================
// Constructor - Set up initial values but don't open I2C yet
//=============================================================================
INA219::INA219(const char* i2cDev, uint8_t address)
    : _i2c_fd(-1),           // -1 means I2C not open yet
      _devPath(i2cDev),      // Store the device path (e.g., "/dev/i2c-1")
      _addr(address),        // Store the I2C slave address
      _status(0) {}          // Status 0 = no errors

//=============================================================================
// Destructor - Clean up when object is destroyed
//=============================================================================
INA219::~INA219() {
    // Only close if we actually opened the I2C device
    if (_i2c_fd >= 0) {
        close(_i2c_fd);
    }
}

//=============================================================================
// Initialize I2C connection and configure the INA219 sensor
// Call this once before reading any data
//=============================================================================
bool INA219::begin() {
    // Step 1: Open the I2C device file (e.g., /dev/i2c-1)
    // O_RDWR means open for both reading and writing
    _i2c_fd = open(_devPath.c_str(), O_RDWR);
    if (_i2c_fd < 0) {
        // Failed to open - might be permission issue or device doesn't exist
        std::cerr << "INA219: Failed to open I2C device: " << _devPath 
                  << " - " << strerror(errno) << std::endl;
        _status = 255; // Error code: can't open device
        return false;
    }

    // Step 2: Tell Linux which I2C slave device we want to talk to
    // I2C_SLAVE sets the slave address for all following operations
    if (ioctl(_i2c_fd, I2C_SLAVE, _addr) < 0) {
        std::cerr << "INA219: Failed to set I2C slave address - " 
                  << strerror(errno) << std::endl;
        close(_i2c_fd);  // Clean up the file descriptor
        _i2c_fd = -1;
        _status = 254;   // Error code: can't set slave address
        return false;
    }

    // Step 3: Configure the INA219 chip with our calibration settings
    if (!setCalibration()) {
        _status = 253;   // Error code: calibration failed
        return false;
    }

    // All good!
    return true;
}

//=============================================================================
// Write a 16-bit value to an INA219 register
// INA219 expects: [register_address][high_byte][low_byte]
//=============================================================================
bool INA219::i2cWriteReg16(uint8_t reg, uint16_t value) {
    // Make sure I2C is open first
    if (_i2c_fd < 0) {
        std::cerr << "INA219: I2C device not open" << std::endl;
        return false;
    }
    
    // Build a 3-byte buffer to send over I2C
    uint8_t buf[3];
    buf[0] = reg;                    // Byte 0: register address
    buf[1] = (value >> 8) & 0xFF;    // Byte 1: high byte of value
    buf[2] = value & 0xFF;           // Byte 2: low byte of value
    
    // Send all 3 bytes in one I2C transaction
    if (write(_i2c_fd, buf, 3) != 3) {
        std::cerr << "INA219: I2C write error" << std::endl;
        _status = 252; // Error code: write failed
        return false;
    }
    return true;
}

//=============================================================================
// Read a 16-bit value from an INA219 register
// I2C protocol: First write register address, then read 2 bytes back
//=============================================================================
bool INA219::i2cReadReg16(uint8_t reg, uint16_t &value) {
    // Make sure I2C is open first
    if (_i2c_fd < 0) {
        std::cerr << "INA219: I2C device not open" << std::endl;
        return false;
    }
    
    // Step 1: Tell the INA219 which register we want to read
    // Write just the register address (1 byte)
    if (write(_i2c_fd, &reg, 1) != 1) {
        std::cerr << "INA219: I2C write error when setting register" << std::endl;
        _status = 251; // Error code: can't set register pointer
        return false;
    }
    
    // Step 2: Read 2 bytes from that register
    uint8_t buf[2];
    if (::read(_i2c_fd, buf, 2) != 2) {
        std::cerr << "INA219: I2C read error" << std::endl;
        _status = 250; // Error code: read failed
        return false;
    }
    
    // Step 3: Combine the 2 bytes into a 16-bit value
    // INA219 sends high byte first (big-endian)
    value = (uint16_t(buf[0]) << 8) | uint16_t(buf[1]);
    return true;
}

//=============================================================================
// Configure the INA219 chip for our specific hardware setup
// IMPORTANT: Adjust these values based on your shunt resistor!
//=============================================================================
bool INA219::setCalibration() {
    // Config Register (0x00) = 0x399F
    // This configures:
    //   - Bus voltage range: 16V (bit 13 = 0)
    //   - Shunt voltage range: ±40mV (bits 11-12 = 01)
    //   - Bus ADC resolution: 12-bit (bits 7-10 = 0011)
    //   - Shunt ADC resolution: 12-bit (bits 3-6 = 0011)
    //   - Operating mode: Continuous, both shunt and bus (bits 0-2 = 111)
    if (!i2cWriteReg16(static_cast<uint8_t>(Register::Config), 0x399F)) {
        std::cerr << "INA219: Failed to write config register" << std::endl;
        return false;
    }
    
    // Calibration Register (0x05) = 4096
    // This value determines the scaling for current and power measurements
    // Formula: Cal = 0.04096 / (Current_LSB × R_shunt)
    // Example setup assumes:
    //   - Shunt resistor: 0.1Ω
    //   - Max expected current: ~2A
    // Adjust this value for your specific shunt resistor!
    if (!i2cWriteReg16(static_cast<uint8_t>(Register::Calibration), 4096)) {
        std::cerr << "INA219: Failed to write calibration register" << std::endl;
        return false;
    }
    
    return true;
}

//=============================================================================
// Read the bus voltage (voltage at V+ pin relative to ground)
// Returns voltage in volts (V)
//=============================================================================
float INA219::getBusVoltage_V() {
    uint16_t raw;
    // Read the Bus Voltage Register (0x02)
    if (!i2cReadReg16(static_cast<uint8_t>(Register::BusVoltage), raw)) {
        return -1.0f; // Return negative value to indicate error
    }

    // Check bit 0 (OVF - Overflow Flag)
    // If set, the internal calculations have exceeded their range
    if (raw & 0x01) {
        std::cerr << "INA219: Math overflow detected" << std::endl;
    }

    // Extract the voltage value from bits 15-3 (bit 0 is OVF, bits 1-2 are reserved)
    // Shift right by 3 bits to align the voltage data
    int16_t vraw = (raw >> 3);
    
    // Convert to actual voltage
    // Each LSB = 4mV according to INA219 datasheet
    float voltage = vraw * 0.004f;  // 4 mV per bit = 0.004 V per bit
    return voltage;
}

//=============================================================================
// Read the current flowing through the shunt resistor
// Returns current in amps (A) - can be negative for reverse current
//=============================================================================
float INA219::getCurrent_A() {
    uint16_t raw;
    // Read the Current Register (0x04)
    if (!i2cReadReg16(static_cast<uint8_t>(Register::Current), raw)) {
        return -1000.0f; // Return large negative value to indicate error
    }

    // Current register is a signed 16-bit value
    // Convert unsigned read value to signed
    int16_t sraw = static_cast<int16_t>(raw);
    
    // Convert to actual current based on calibration
    // LSB value depends on calibration register setting
    // Example: 1 mA per bit (adjust based on your calibration!)
    float current = sraw * 0.001f; // 1 mA per bit = 0.001 A per bit
    return current;
}

//=============================================================================
// Calculate power consumption (P = V × I)
// Returns power in watts (W)
//=============================================================================
float INA219::getPower_W() {
    // Get voltage and current measurements
    float voltage = getBusVoltage_V();
    float current = getCurrent_A();
    
    // Check if either measurement failed
    if (voltage < 0.0f || current < -999.0f) {
        return -1.0f; // Error indicator
    }
    
    // Calculate power = voltage × current
    return voltage * current;
}

//=============================================================================
// Implementation of Sensor base class read() method
// Reads all sensor data and packs it into a byte array for transmission
// This follows the same pattern as other sensors (e.g., BQ34Z100)
//=============================================================================
std::vector<uint8_t> INA219::read() {
    // Read all measurements from the sensor
    const float voltage = getBusVoltage_V();      // Voltage in volts
    const float current = getCurrent_A();         // Current in amps
    const float power = getPower_W();             // Power in watts
    
    // Convert floating-point values to fixed-point integers for compact transmission
    // This reduces data size and makes parsing easier on the receiving end
    
    // Voltage: convert V to mV (millivolts), unsigned 16-bit
    const uint16_t voltage_mV = static_cast<uint16_t>(std::lround(voltage * 1000.0f));
    
    // Current: convert A to mA (milliamps), signed 16-bit (can be negative)
    const int16_t current_mA = static_cast<int16_t>(std::lround(current * 1000.0f));
    
    // Power: convert W to mW (milliwatts), unsigned 16-bit
    const uint16_t power_mW = static_cast<uint16_t>(std::lround(power * 1000.0f));
    
    // Build the payload byte array
    std::vector<uint8_t> payload;
    
    // Add voltage (2 bytes, little-endian)
    payload.push_back(static_cast<uint8_t>(voltage_mV & 0xFF));         // Low byte
    payload.push_back(static_cast<uint8_t>((voltage_mV >> 8) & 0xFF));  // High byte
    
    // Add current (2 bytes, little-endian, signed)
    payload.push_back(static_cast<uint8_t>(current_mA & 0xFF));         // Low byte
    payload.push_back(static_cast<uint8_t>((current_mA >> 8) & 0xFF));  // High byte
    
    // Add power (2 bytes, little-endian)
    payload.push_back(static_cast<uint8_t>(power_mW & 0xFF));           // Low byte
    payload.push_back(static_cast<uint8_t>((power_mW >> 8) & 0xFF));    // High byte
    
    // Add status byte (for error tracking)
    // 0 = OK, other values indicate specific errors
    payload.push_back(_status);
    
    // Total payload size: 7 bytes
    // Format: [V_low][V_high][I_low][I_high][P_low][P_high][status]
    return payload;
}
