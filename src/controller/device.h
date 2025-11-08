#pragma once

#include <cstdint>

class Device {
public:
  
  bool getState();
  void setState(bool newState);

private:
  bool state;
  uint8_t lastOperationPriority = 255;
};
