#include "sensor.h"

// Default implementation of virtual read() method
std::vector<uint8_t> Sensor::read() {
  return std::vector<uint8_t>();
}
