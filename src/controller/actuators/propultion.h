#pragma once  // Prevent multiple inclusions of this header file
#include "actuator.h"  // Base Actuator class for inheritance
#include <vector>      // STL vector container for dynamic arrays

class Propultion : public Actuator {
public:
    Propultion();

    void setThrust(std::vector<uint8_t> thrust);
    void engage();

private:
    std::vector<uint8_t> current_thrust;
};