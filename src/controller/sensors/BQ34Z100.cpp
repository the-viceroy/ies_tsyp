#include "BQ34Z100.h"

#include <cinttypes>
#include <vector>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <iostream>
#include <cstring>

// Constructor: open I2C device
BQ34Z100::BQ34Z100(const char* i2c_device)
  : _i2c_fd(-1), _status(0) {
  _i2c_fd = open(i2c_device, O_RDWR);
  if (_i2c_fd < 0) {
    std::cerr << "Failed to open I2C device: " << i2c_device << std::endl;
    _status = 255;
  } else {
    // Set I2C slave address
    if (ioctl(_i2c_fd, I2C_SLAVE, GAUGE_ADDRESS) < 0) {
      std::cerr << "Failed to set I2C slave address" << std::endl;
      _status = 254;
    }
  }
}

// Destructor: close I2C device
BQ34Z100::~BQ34Z100() {
  if (_i2c_fd >= 0) {
    close(_i2c_fd);
  }
}




// Low-level multi-byte register read helper
uint32_t BQ34Z100::read(Command command, const uint8_t length) {
  if (_i2c_fd < 0) {
    std::cerr << "I2C device not open" << std::endl;
    return 0;
  }

  uint32_t val = 0;
  for (int i = 0; i < length; i++) {
    uint8_t cmdByte = static_cast<uint8_t>(command) + i;
    
    // Write register address
    if (write(_i2c_fd, &cmdByte, 1) != 1) {
      std::cerr << "I2C write error when setting register address" << std::endl;
      _status = 253;
      return 0;
    }

    // Read one byte from the register
    uint8_t readByte = 0;
    if (::read(_i2c_fd, &readByte, 1) != 1) {
      std::cerr << "I2C read error when reading data" << std::endl;
      _status = 252;
      return 0;
    }

    val |= (static_cast<uint32_t>(readByte) << (8 * i));
  }
  return val;
}

// getters
uint8_t BQ34Z100::getSOC() { return read(Command::StateOfCharge, 1); }

uint16_t BQ34Z100::getVoltage() { return read(Command::Voltage, 2); }

int16_t BQ34Z100::getCurrent() { return static_cast<int16_t>(read(Command::Current, 2)); }

double BQ34Z100::getTemperature() {
  // Temperature returned in 0.1 K; convert to Celsius
  return (read(Command::Temperature, 2) / 10.0) - 273.15;
}

// Sensor::read override: pack a compact telemetry frame
std::vector<uint8_t> BQ34Z100::read() {
  const uint16_t voltage = getVoltage();        // mV
  const int16_t current = getCurrent();         // mA (signed)
  const uint8_t soc = getSOC();                 // %
  const int16_t tempCenti = static_cast<int16_t>(std::lround(getTemperature() * 100.0)); // °C * 100

  std::vector<uint8_t> payload;
  // voltage (LE)
  payload.push_back(static_cast<uint8_t>(voltage & 0xFF));
  payload.push_back(static_cast<uint8_t>((voltage >> 8) & 0xFF));
  // current (LE)
  payload.push_back(static_cast<uint8_t>(current & 0xFF));
  payload.push_back(static_cast<uint8_t>((current >> 8) & 0xFF));
  // state of charge
  payload.push_back(soc);
  // temperature *100 (LE)
  payload.push_back(static_cast<uint8_t>(tempCenti & 0xFF));
  payload.push_back(static_cast<uint8_t>((tempCenti >> 8) & 0xFF));
  return payload;
}

