#include "lidar.h"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>

// I2C file descriptor (global or could be a member variable)
static int i2c_fd = -1;

// TF-Luna I2C Register Addresses
#define TFL_DIST_LO      0x00  // Distance low byte
#define TFL_DIST_HI      0x01  // Distance high byte
#define TFL_FLUX_LO      0x02  // Signal strength low byte
#define TFL_FLUX_HI      0x03  // Signal strength high byte
#define TFL_TEMP_LO      0x04  // Temperature low byte
#define TFL_TEMP_HI      0x05  // Temperature high byte
#define TFL_TICK_LO      0x06  // Time tick low byte
#define TFL_TICK_HI      0x07  // Time tick high byte
#define TFL_ERR_LO       0x08  // Error code low byte
#define TFL_ERR_HI       0x09  // Error code high byte
#define TFL_VER_REV      0x0A  // Version revision
#define TFL_VER_MIN      0x0B  // Version minor
#define TFL_VER_MAJ      0x0C  // Version major

#define TFL_SAVE_SETTINGS 0x20 // Save settings command
#define TFL_SOFT_RESET    0x02 // Soft reset command
#define TFL_HARD_RESET    0x10 // Hard reset command
#define TFL_SET_I2C_ADDR  0x22 // Set I2C address command
#define TFL_SET_TRIG_MODE 0x00 // Trigger mode
#define TFL_SET_CONT_MODE 0x01 // Continuous mode
#define TFL_TRIGGER_ONCE  0x01 // Single trigger

// Constructor: Initialize the TF-Luna I2C LiDAR sensor
Lidar::Lidar() {
    tfStatus = 0;
    memset(dataArray, 0, sizeof(dataArray));
    regReply = 0;
    
    // Open I2C device (typically /dev/i2c-1 on Raspberry Pi)
    // You may need to adjust the device path
    i2c_fd = open("/dev/i2c-1", O_RDWR);
    if (i2c_fd < 0) {
        std::cerr << "Failed to open I2C device" << std::endl;
        tfStatus = 255; // Error code
    }
}

// Get complete sensor data (distance, signal strength, and temperature)
bool Lidar::getData(int16_t &dist, int16_t &flux, int16_t &temp, uint8_t addr) {
    if (i2c_fd < 0) {
        tfStatus = 255;
        return false;
    }
    
    // Set I2C slave address
    if (ioctl(i2c_fd, I2C_SLAVE, addr) < 0) {
        tfStatus = 254;
        return false;
    }
    
    // Read 6 bytes starting from register 0x00
    // The TF-Luna stores data in registers 0x00-0x05
    if (::read(i2c_fd, dataArray, 6) != 6) {
        tfStatus = 253;
        return false;
    }
    
    // Parse the data from the 6-byte array
    // Distance is in dataArray[0] (LSB) and dataArray[1] (MSB)
    dist = (dataArray[1] << 8) | dataArray[0];
    
    // Signal strength/flux is in dataArray[2] (LSB) and dataArray[3] (MSB)
    flux = (dataArray[3] << 8) | dataArray[2];
    
    // Temperature is in dataArray[4] (LSB) and dataArray[5] (MSB)
    temp = (dataArray[5] << 8) | dataArray[4];
    
    tfStatus = 0; // Success
    return true;
}

// Get distance data only (simplified version)
bool Lidar::getData(int16_t &dist, uint8_t addr) {
    int16_t flux, temp;
    return getData(dist, flux, temp, addr);
}

// Read a single byte from an I2C register
bool Lidar::readReg(uint8_t nmbr, uint8_t addr) {
    if (i2c_fd < 0) {
        tfStatus = 255;
        return false;
    }
    
    // Set I2C slave address
    if (ioctl(i2c_fd, I2C_SLAVE, addr) < 0) {
        tfStatus = 254;
        return false;
    }
    
    // Write register address to read from
    if (write(i2c_fd, &nmbr, 1) != 1) {
        tfStatus = 253;
        return false;
    }
    
    // Read one byte from the register
    if (::read(i2c_fd, &regReply, 1) != 1) {
        tfStatus = 252;
        return false;
    }
    
    tfStatus = 0;
    return true;
}

// Write a single byte to an I2C register
bool Lidar::writeReg(uint8_t nmbr, uint8_t addr, uint8_t data) {
    if (i2c_fd < 0) {
        tfStatus = 255;
        return false;
    }
    
    // Set I2C slave address
    if (ioctl(i2c_fd, I2C_SLAVE, addr) < 0) {
        tfStatus = 254;
        return false;
    }
    
    uint8_t buffer[2] = {nmbr, data};
    
    // Write register address and data
    if (write(i2c_fd, buffer, 2) != 2) {
        tfStatus = 253;
        return false;
    }
    
    tfStatus = 0;
    return true;
}

// Get firmware version from the device
bool Lidar::Get_Firmware_Version(uint8_t ver[], uint8_t adr) {
    if (!readReg(TFL_VER_MAJ, adr)) return false;
    ver[0] = regReply;
    
    if (!readReg(TFL_VER_MIN, adr)) return false;
    ver[1] = regReply;
    
    if (!readReg(TFL_VER_REV, adr)) return false;
    ver[2] = regReply;
    
    return true;
}

// Get current frame rate (measurement frequency)
bool Lidar::Get_Frame_Rate(uint16_t &frm, uint8_t adr) {
    uint8_t frameLo, frameHi;
    
    if (!readReg(0x12, adr)) return false; // Frame rate low byte register
    frameLo = regReply;
    
    if (!readReg(0x13, adr)) return false; // Frame rate high byte register
    frameHi = regReply;
    
    frm = (frameHi << 8) | frameLo;
    return true;
}

// Get product code/identifier
bool Lidar::Get_Prod_Code(uint8_t cod[], uint8_t adr) {
    // Product code typically starts at register 0x10 (14 bytes)
    for (int i = 0; i < 14; i++) {
        if (!readReg(0x10 + i, adr)) return false;
        cod[i] = regReply;
    }
    return true;
}

// Get device running time (time since power-on)
bool Lidar::Get_Time(uint16_t &tim, uint8_t adr) {
    uint8_t timeLo, timeHi;
    
    if (!readReg(TFL_TICK_LO, adr)) return false;
    timeLo = regReply;
    
    if (!readReg(TFL_TICK_HI, adr)) return false;
    timeHi = regReply;
    
    tim = (timeHi << 8) | timeLo;
    return true;
}

// Set frame rate (measurement frequency)
bool Lidar::Set_Frame_Rate(uint16_t &frm, uint8_t adr) {
    uint8_t frameLo = frm & 0xFF;
    uint8_t frameHi = (frm >> 8) & 0xFF;
    
    if (!writeReg(0x12, adr, frameLo)) return false;
    if (!writeReg(0x13, adr, frameHi)) return false;
    
    return true;
}

// Change the I2C address of the device
bool Lidar::Set_I2C_Addr(uint8_t adrNew, uint8_t adr) {
    return writeReg(TFL_SET_I2C_ADDR, adr, adrNew);
}

// Enable the LiDAR sensor (start measurements)
bool Lidar::Set_Enable(uint8_t adr) {
    return writeReg(0x25, adr, 0x01); // Enable register
}

// Disable the LiDAR sensor (stop measurements, low power mode)
bool Lidar::Set_Disable(uint8_t adr) {
    return writeReg(0x25, adr, 0x00); // Disable register
}

// Soft reset: Restart the device without losing settings
bool Lidar::Soft_Reset(uint8_t adr) {
    return writeReg(0x21, adr, TFL_SOFT_RESET);
}

// Hard reset: Restore all factory default settings
bool Lidar::Hard_Reset(uint8_t adr) {
    return writeReg(0x21, adr, TFL_HARD_RESET);
}

// Save current settings to non-volatile memory
bool Lidar::Save_Settings(uint8_t adr) {
    return writeReg(0x20, adr, TFL_SAVE_SETTINGS);
}

// Set trigger mode (single measurement on command)
bool Lidar::Set_Trig_Mode(uint8_t adr) {
    return writeReg(0x23, adr, TFL_SET_TRIG_MODE);
}

// Set continuous mode (automatic repeated measurements)
bool Lidar::Set_Cont_Mode(uint8_t adr) {
    return writeReg(0x23, adr, TFL_SET_CONT_MODE);
}

// Trigger a single measurement (only in trigger mode)
bool Lidar::Set_Trigger(uint8_t adr) {
    return writeReg(0x24, adr, TFL_TRIGGER_ONCE);
}

// Print the raw data array to console (for debugging)
void Lidar::printDataArray() {
    std::cout << "Data Array: ";
    for (int i = 0; i < 6; i++) {
        std::cout << "0x" << std::hex << static_cast<int>(dataArray[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}

// Print the current status register value (for debugging)
void Lidar::printStatus() {
    std::cout << "Status: ";
    switch (tfStatus) {
        case 0:
            std::cout << "READY" << std::endl;
            break;
        case 1:
            std::cout << "WEAK SIGNAL" << std::endl;
            break;
        case 2:
            std::cout << "STRONG SIGNAL" << std::endl;
            break;
        case 3:
            std::cout << "OUT OF RANGE" << std::endl;
            break;
        case 252:
            std::cout << "I2C READ ERROR" << std::endl;
            break;
        case 253:
            std::cout << "I2C WRITE ERROR" << std::endl;
            break;
        case 254:
            std::cout << "I2C ADDRESS ERROR" << std::endl;
            break;
        case 255:
            std::cout << "I2C DEVICE NOT OPEN" << std::endl;
            break;
        default:
            std::cout << "UNKNOWN ERROR (" << static_cast<int>(tfStatus) << ")" << std::endl;
            break;
    }
}

// Implement the virtual read() method from Sensor base class
std::vector<uint8_t> Lidar::read() {
    std::vector<uint8_t> data;
    
    // Read the 6 bytes of sensor data
    if (i2c_fd >= 0) {
        for (int i = 0; i < 6; i++) {
            data.push_back(dataArray[i]);
        }
    }
    
    return data;
}
