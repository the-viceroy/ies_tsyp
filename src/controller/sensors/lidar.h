
#pragma once  // Prevent multiple inclusions of this header file

#include "../sensor.h" // Base Sensor class for inheritance
#include <vector>    // STL vector container for dynamic arrays

// TF-Luna LiDAR: Benewake ToF (Time of Flight) LiDAR sensor
// Communicates via I2C protocol to measure distance
// Inherits from Sensor base class
class Lidar : public Sensor {
  public:
    // Constructor: Initialize the TF-Luna I2C LiDAR sensor
    Lidar();
    

    // Get complete sensor data (distance, signal strength, and temperature)
    // dist: Reference to store distance measurement in centimeters
    // flux: Reference to store signal strength/quality (flux intensity)
    // temp: Reference to store chip temperature in Celsius * 100
    // addr: I2C device address (default 0x10)
    // Returns: true if data read successfully, false on error
    bool getData( int16_t &dist, int16_t &flux, int16_t &temp, uint8_t addr);
    
    // Get distance data only (simplified version)
    // dist: Reference to store distance measurement in centimeters
    // addr: I2C device address (default 0x10)
    // Returns: true if data read successfully, false on error
    bool getData( int16_t &dist, uint8_t addr);

    // Read a single byte from an I2C register
    // nmbr: Register number/address to read from
    // addr: I2C device address
    // Returns: true if read successful, false on error
    bool readReg( uint8_t nmbr, uint8_t addr);
    
    // Write a single byte to an I2C register
    // nmbr: Register number/address to write to
    // addr: I2C device address
    // data: Byte value to write to the register
    // Returns: true if write successful, false on error
    bool writeReg( uint8_t nmbr, uint8_t addr, uint8_t data);

    // Get firmware version from the device
    // ver: Array to store 3-byte version (major, minor, revision)
    // adr: I2C device address
    // Returns: true if successful, false on error
    bool Get_Firmware_Version( uint8_t ver[], uint8_t adr);
    
    // Get current frame rate (measurement frequency)
    // frm: Reference to store frame rate in Hz
    // adr: I2C device address
    // Returns: true if successful, false on error
    bool Get_Frame_Rate( uint16_t &frm, uint8_t adr);
    
    // Get product code/identifier
    // cod: Array to store 14-byte product code string
    // adr: I2C device address
    // Returns: true if successful, false on error
    bool Get_Prod_Code( uint8_t cod[], uint8_t adr);
    
    // Get device running time (time since power-on)
    // tim: Reference to store time in milliseconds
    // adr: I2C device address
    // Returns: true if successful, false on error
    bool Get_Time( uint16_t &tim, uint8_t adr);

    // Set frame rate (measurement frequency)
    // frm: Desired frame rate in Hz (reference, may be updated with actual value)
    // adr: I2C device address
    // Returns: true if successful, false on error
    bool Set_Frame_Rate( uint16_t &frm, uint8_t adr);
    
    // Change the I2C address of the device
    // adrNew: New I2C address to assign (must be unique on bus)
    // adr: Current I2C device address
    // Returns: true if successful, false on error
    bool Set_I2C_Addr( uint8_t adrNew, uint8_t adr);
    
    // Enable the LiDAR sensor (start measurements)
    // adr: I2C device address
    // Returns: true if successful, false on error
    bool Set_Enable( uint8_t adr);
    
    // Disable the LiDAR sensor (stop measurements, low power mode)
    // adr: I2C device address
    // Returns: true if successful, false on error
    bool Set_Disable( uint8_t adr);
    
    // Soft reset: Restart the device without losing settings
    // adr: I2C device address
    // Returns: true if successful, false on error
    bool Soft_Reset( uint8_t adr);
    
    // Hard reset: Restore all factory default settings
    // adr: I2C device address
    // Returns: true if successful, false on error
    bool Hard_Reset( uint8_t adr);
    
    // Save current settings to non-volatile memory (persist across power cycles)
    // adr: I2C device address
    // Returns: true if successful, false on error
    bool Save_Settings( uint8_t adr);
    
    // Set trigger mode (single measurement on command)
    // adr: I2C device address
    // Returns: true if successful, false on error
    bool Set_Trig_Mode( uint8_t adr);
    
    // Set continuous mode (automatic repeated measurements)
    // adr: I2C device address
    // Returns: true if successful, false on error
    bool Set_Cont_Mode( uint8_t adr);
    
    // Trigger a single measurement (only in trigger mode)
    // adr: I2C device address
    // Returns: true if successful, false on error
    bool Set_Trigger( uint8_t adr);

    // Print the raw data array to console (for debugging)
    // Displays the 6-byte data buffer contents
    void printDataArray();
    
    // Print the current status register value (for debugging)
    // Displays error codes and device state
    void printStatus();
    
    // Override the read() method from Sensor base class
    std::vector<uint8_t> read() override;

  private:
    // Device status byte (0 = READY, non-zero = error code)
    // Common errors: 1 = weak signal, 2 = strong signal, 3 = out of range
    uint8_t tfStatus;
    
    // Raw data buffer: stores 6 bytes from sensor
    // Bytes 0-1: Distance (LSB, MSB)
    // Bytes 2-3: Signal strength/flux (LSB, MSB)
    // Bytes 4-5: Temperature (LSB, MSB)
    uint8_t dataArray[6];
    
    // Register reply byte (stores response from register read operations)
    uint8_t regReply;
};