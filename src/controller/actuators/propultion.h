#pragma once  

#include "../controller/actuator.h"
#include <vector>      

class Propulsion : public Actuator {
public:
    Propulsion();

    void setThrust(std::vector<uint8_t> thrust);
    void engage();

private:
    std::vector<uint8_t> current_thrust;
};