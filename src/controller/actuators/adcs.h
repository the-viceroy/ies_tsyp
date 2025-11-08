#pragma once  

#include "../controller/actuator.h"
#include <vector>

class ADCS : public Actuator {
public:


    virtual bool write(std::vector<uint8_t> bytes);
    
    void rotate(std::vector<uint8_t> axis, std::vector<float> angle);
    
private:

   std::vector<uint8_t> current_axis;
   std::vector<float> current_angle;
};