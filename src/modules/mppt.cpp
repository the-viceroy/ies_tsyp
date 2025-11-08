#include "mppt.h"
#include <cmath>       // For fabs() - absolute value of floating-point numbers
#include <iostream>    // For std::cerr - error logging

//=============================================================================
// Constructor - Initialize the MPPT module with configuration parameters
//=============================================================================
MPPTModule::MPPTModule(uint32_t sensorId, uint32_t actuatorId, 
                       double step, double dmin, double dmax)
    : _sensorId(sensorId),     // Store which sensor to read from
      _actuatorId(actuatorId), // Store which actuator to control
      prevV(0.0),              // No previous voltage yet
      prevI(0.0),              // No previous current yet
      duty(0.5),               // Start at 50% duty cycle (safe middle point)
      stepSize(step),          // How much to adjust duty each iteration
      dutyMin(dmin),           // Lower safety limit
      dutyMax(dmax),           // Upper safety limit
      lastPower(0.0) {}        // No power calculated yet

//=============================================================================
// Module condition check - determines if MPPT should run
// Currently always returns true, meaning MPPT runs continuously
//=============================================================================
bool MPPTModule::condition(Controller &controller) {
    // Could add conditions here, such as:
    // - Only run if sun is up (check time or light sensor)
    // - Only run if battery isn't full
    // - Only run if input voltage is above minimum threshold
    return true;
}

//=============================================================================
// Main MPPT polling function - called periodically by the Manager
// This is where the MPPT algorithm runs each iteration
//=============================================================================
void MPPTModule::poll(Controller &controller) {
    try {
        // Step 1: Get the voltage/current sensor from the controller
        Sensor &sensor = controller.getSensor(_sensorId);
        
        // Step 2: Read sensor data (returns byte array)
        std::vector<uint8_t> data = sensor.read();
        
        // Step 3: Validate that we got enough data
        // Expected format for INA219: [V_low][V_high][I_low][I_high][P_low][P_high][status]
        // We need at least 4 bytes for voltage and current
        if (data.size() < 4) {
            std::cerr << "MPPT: Insufficient sensor data (got " << data.size() 
                      << " bytes, need at least 4)" << std::endl;
            return;
        }
        
        // Step 4: Parse the sensor data (little-endian format)
        // Bytes 0-1: Voltage in millivolts (unsigned 16-bit)
        uint16_t voltage_mV = data[0] | (data[1] << 8);
        
        // Bytes 2-3: Current in milliamps (signed 16-bit)
        int16_t current_mA = data[2] | (data[3] << 8);
        
        // Step 5: Convert from fixed-point integers to floating-point
        double voltage = voltage_mV / 1000.0;  // mV to V
        double current = current_mA / 1000.0;  // mA to A
        
        // Step 6: Run the MPPT algorithm to calculate new duty cycle
        updateMPPT(voltage, current);
        
        // Step 7: Apply the new duty cycle to the actuator
        // NOTE: This is currently commented out because the Actuator interface
        // would need to be extended with a setDuty() method
        // Future implementation:
        // Actuator &actuator = controller.getActuator(_actuatorId);
        // actuator.setDuty(duty);
        
    } catch (const std::exception &e) {
        // Catch any errors (e.g., sensor not found, invalid ID)
        std::cerr << "MPPT poll error: " << e.what() << std::endl;
    }
}

//=============================================================================
// Incremental Conductance MPPT Algorithm
// This is the heart of the MPPT controller - it finds the maximum power point
//=============================================================================
void MPPTModule::updateMPPT(double voltage, double current) {
    // Safety check: if voltage is near zero, set safe duty cycle and exit
    if (voltage <= 0.0001) {
        duty = dutyMin;       // Use minimum duty cycle
        prevV = voltage;      // Save current values
        prevI = current;
        lastPower = 0.0;
        return;
    }

    // Calculate the changes in voltage and current since last iteration
    double dV = voltage - prevV;  // Change in voltage (delta V)
    double dI = current - prevI;  // Change in current (delta I)

    // Case 1: Voltage hasn't changed (dV ≈ 0)
    if (fabs(dV) < 1e-8) {  // Use small threshold due to floating-point precision
        if (fabs(dI) < 1e-8) {
            // No change in voltage OR current - we're stable, keep current duty
            // This happens when we're at or very near the MPP
        } else if (dI > 0) {
            // Current increased while voltage stayed same
            // We're on the left side of the power curve, decrease duty
            duty -= stepSize;
        } else {
            // Current decreased while voltage stayed same
            // We're on the right side of the power curve, increase duty
            duty += stepSize;
        }
    } 
    // Case 2: Voltage has changed - use incremental conductance
    else {
        // Calculate incremental conductance: dI/dV
        double dIdV = dI / dV;
        
        // Calculate instantaneous conductance: -I/V
        double negIV = -current / voltage;
        
        // Tolerance for "close enough" to MPP
        const double tol = 1e-3;

        // At MPP: dI/dV = -I/V
        if (fabs(dIdV - negIV) <= tol) {
            // We're at the maximum power point - don't change duty
            // The tolerance accounts for sensor noise and discretization
        } 
        // Left of MPP: dI/dV > -I/V
        else if (dIdV > negIV) {
            // Operating point is to the left of MPP (lower voltage side)
            // Decrease duty cycle to increase voltage and move toward MPP
            duty -= stepSize;
        } 
        // Right of MPP: dI/dV < -I/V
        else {
            // Operating point is to the right of MPP (higher voltage side)
            // Increase duty cycle to decrease voltage and move toward MPP
            duty += stepSize;
        }
    }

    // Apply safety limits to prevent duty cycle going out of bounds
    if (duty > dutyMax) duty = dutyMax;  // Cap at maximum (e.g., 95%)
    if (duty < dutyMin) duty = dutyMin;  // Floor at minimum (e.g., 5%)

    // Save current measurements for next iteration
    prevV = voltage;
    prevI = current;
    
    // Calculate and store power for monitoring/debugging
    lastPower = voltage * current;
}

//=============================================================================
// Get the current PWM duty cycle
// Returns a value from 0.0 (0% - off) to 1.0 (100% - full on)
//=============================================================================
double MPPTModule::getDuty() const { 
    return duty; 
}

//=============================================================================
// Get the last calculated power value
// Returns power in watts (W) calculated as voltage × current
// Useful for monitoring system performance and verifying MPPT is working
//=============================================================================
double MPPTModule::getPower() const { 
    return lastPower; 
}
