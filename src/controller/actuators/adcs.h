#pragma once  // Prevent multiple inclusions of this header file

class ADCS : public actuator {
public:
    ADCS();
    
    void rotate(std::vector<uint8_t> axis, std::vector<float> angle);
    
private:
    // Private member variables for actuator state
   std::vector<uint8_t> current_axis;
   std::vector<float> current_angle;
};