#include "device.h"

bool Device::getState() {
  return state;
}

void Device::setState(bool newState) {
  state = newState;
}
