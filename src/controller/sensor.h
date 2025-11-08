#pragma once

#include <cstdint>
#include <vector>
#include "device.h"

class Sensor: public Device {
public:
  virtual std::vector<uint8_t> read();
};
