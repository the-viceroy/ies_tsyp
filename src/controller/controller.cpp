#include "controller.h"

// Default constructor implementation
Controller::Controller() : actuators(), sensors() {
}

Sensor &Controller::getSensor(uint32_t id) {
  return const_cast<Sensor&>(sensors.at(id));
}
Actuator &Controller::getActuator(uint32_t id) {
  return const_cast<Actuator&>(actuators.at(id));
}
